"""Six-axis IMU integration in an arbitrary, startup-aligned inertial frame.

No absolute attitude, gravity vector, or position is required. Specific force
predicts chaser motion relative to a nearby unforced, co-falling target. The
unmodelled gravity gradient and target acceleration remain filter process noise.
"""
from collections import deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class InertialSample:
    stamp: float
    rotation: Rotation
    gyro: np.ndarray
    force: np.ndarray
    velocity_integral: np.ndarray
    position_integral: np.ndarray


class ImuIntegrator:
    def __init__(self, max_gap=.25, history_sec=3.):
        self.max_gap = max_gap
        self.history_sec = history_sec
        self.samples = deque()

    def reset(self):
        self.samples.clear()

    def push(self, stamp, gyro, specific_force):
        gyro, force = np.asarray(gyro, float), np.asarray(specific_force, float)
        if not np.isfinite(stamp) or not np.isfinite(gyro).all() or not np.isfinite(force).all():
            return False
        if self.samples:
            previous = self.samples[-1]
            dt = stamp - previous.stamp
            if dt <= 0:
                return False
            if dt > self.max_gap:
                # A lost gyro interval cannot be reconstructed. Never silently
                # continue with an old attitude after reconnect or time warp.
                raise ValueError(f'IMU sample gap {dt:.3f} s exceeds {self.max_gap:.3f} s; restart local navigation')
            rotation = previous.rotation * Rotation.from_rotvec((previous.gyro + gyro) * (.5 * dt))
            local_force = rotation.apply(force)
            mean_force = .5 * (previous.force + local_force)
            velocity = previous.velocity_integral + mean_force * dt
            position = previous.position_integral + previous.velocity_integral * dt + .5 * mean_force * dt**2
        else:
            rotation = Rotation.identity()
            local_force = force.copy()
            velocity = np.zeros(3)
            position = np.zeros(3)
        self.samples.append(InertialSample(stamp, rotation, gyro, local_force, velocity, position))
        while len(self.samples) > 2 and stamp - self.samples[1].stamp > self.history_sec:
            self.samples.popleft()
        return True

    def at(self, stamp):
        if not self.samples or stamp < self.samples[0].stamp - 1e-6 or stamp > self.samples[-1].stamp + 1e-6:
            return None
        for left, right in zip(self.samples, list(self.samples)[1:]):
            if left.stamp <= stamp <= right.stamp:
                dt = stamp - left.stamp
                fraction = dt / (right.stamp - left.stamp)
                relative_rotation = left.rotation.inv() * right.rotation
                rotation = left.rotation * Rotation.from_rotvec(relative_rotation.as_rotvec() * fraction)
                force = (left.force + right.force) * .5
                return InertialSample(stamp, rotation,
                    left.gyro + fraction*(right.gyro-left.gyro), force,
                    left.velocity_integral + force*dt,
                    left.position_integral + left.velocity_integral*dt + .5*force*dt**2)
        return self.samples[-1]

    def relative_motion(self, start, end):
        a, b = self.at(start), self.at(end)
        if a is None or b is None:
            return None
        # Target minus chaser: chaser specific force has the opposite sign.
        return (-(b.position_integral-a.position_integral-a.velocity_integral*(end-start)),
                -(b.velocity_integral-a.velocity_integral))
