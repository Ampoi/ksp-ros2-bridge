"""Sensor-only rover geometry, motion and registration. No ROS or world truth."""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from position_estimator.core import transform_points, voxel_downsample


@dataclass(frozen=True)
class Wheel:
    id: str
    x: float
    y: float
    z: float
    radius: float
    rolling_sign: float
    steering_sign: float
    steering: bool
    max_angle: float


class Geometry:
    def __init__(self, wheels, body_min, body_max, margin=0.5):
        self.wheels = tuple(sorted(wheels, key=lambda w: w.id))
        if len(wheels) not in (4, 6) or len({w.id for w in wheels}) != len(wheels):
            raise ValueError('need all 4 or 6 wheels')
        for w in wheels:
            if not np.isfinite([w.x,w.y,w.z,w.radius,w.rolling_sign,w.steering_sign,w.max_angle]).all():
                raise ValueError('non-finite wheel geometry')
            if w.radius <= 0 or abs(w.rolling_sign) != 1 or abs(w.steering_sign) != 1:
                raise ValueError('missing geometry or wheel axes not aligned with base_link')
            partners = [v for v in wheels if v.id != w.id and abs(v.x-w.x)<0.25 and abs(v.y+w.y)<0.25]
            if len(partners) != 1 or partners[0].steering != w.steering or abs(w.y)<0.1:
                raise ValueError('wheels must form symmetric left/right pairs')
        front = [w for w in wheels if w.steering]
        fixed = [w for w in wheels if not w.steering]
        if len(front)!=2 or not fixed or min(w.x for w in front) <= max(w.x for w in fixed)+0.1:
            raise ValueError('enable steering only on the front pair')
        if min(w.max_angle for w in front) < 0.05:
            raise ValueError('front steering angle unavailable')
        self.rear_x = float(np.mean([w.x for w in fixed]))
        self.wheelbase = float(np.mean([w.x for w in front])) - self.rear_x
        self.min_radius = max((w.x-self.rear_x)/math.tan(min(w.max_angle, 1.2))+abs(w.y) for w in front)
        lo, hi = np.asarray(body_min, float).copy(), np.asarray(body_max, float).copy()
        if not np.isfinite([lo,hi]).all() or np.any(hi<=lo):
            raise ValueError('body collision bounds unavailable')
        lo[0] -= self.rear_x; hi[0] -= self.rear_x
        self.footprint = [[lo[0]-margin,lo[1]-margin], [hi[0]+margin,lo[1]-margin],
                          [hi[0]+margin,hi[1]+margin], [lo[0]-margin,hi[1]+margin]]
        self.height = -float(np.mean([w.z-w.radius for w in wheels]))

    def commands(self, v, omega):
        if not np.isfinite([v,omega]).all() or v < 0:
            raise ValueError('only finite forward commands supported')
        v = min(v, 0.5)
        omega = float(np.clip(omega, -v/self.min_radius, v/self.min_radius))
        result = {}
        for w in self.wheels:
            vx, vy = v-omega*w.y, omega*(w.x-self.rear_x)
            angle = math.atan2(vy,vx) if w.steering and v>1e-5 else 0.
            speed = math.hypot(vx,vy) if w.steering else vx
            result[w.id] = (speed/w.radius*w.rolling_sign, angle*w.steering_sign)
        return result

    def speed(self, states, yaw_rate):
        values = [states[w.id][0]*w.radius*w.rolling_sign+yaw_rate*w.y
                  for w in self.wheels if not w.steering]
        return float(np.median(values)), float(np.std(values))


class Estimator:
    def __init__(self):
        self.pose = np.eye(4)
        self.initialized = False
        self.gyro_bias = np.zeros(3)
        self.sigma = 0.05
        self.submap = np.empty((0,3))
        self.last_keyframe = None
        self.quality = 'waiting_for_stationary_imu'
        self.rmse = 0.
        self.observed_rank = 0

    def initialize(self, accelerations, gyros):
        a, g = np.asarray(accelerations), np.asarray(gyros)
        if len(a)<30 or not np.isfinite([a,g]).all():
            return False
        up = a.mean(axis=0)
        gravity = np.linalg.norm(up)
        if gravity < 0.5 or gravity > 3 or np.max(np.std(a,axis=0))>0.08 or np.max(np.std(g,axis=0))>0.005:
            return False
        up /= gravity
        forward = np.array([1.,0.,0.]) - up*up[0]
        if np.linalg.norm(forward)<0.5: return False
        forward /= np.linalg.norm(forward)
        left = np.cross(up,forward)
        self.pose[:3,:3] = np.stack([forward,left,up])
        self.gyro_bias = g.mean(axis=0)  # Includes static Mun spin, no ephemeris.
        self.initialized = True
        self.quality = 'wheel_imu'
        return True

    def predict(self, gyro, speed, rear_x, dt, slip=0.):
        if not self.initialized or not 0<dt<=0.2: return False
        gyro = np.asarray(gyro)-self.gyro_bias
        if not np.isfinite([*gyro,speed,slip]).all(): return False
        delta = Rotation.from_rotvec(gyro*dt)
        velocity = np.array([speed, -rear_x*gyro[2], 0.])
        self.pose[:3,3] += self.pose[:3,:3] @ Rotation.from_rotvec(gyro*dt/2).apply(velocity)*dt
        self.pose[:3,:3] = self.pose[:3,:3] @ delta.as_matrix()
        self.sigma += (0.00005 + abs(speed)*0.02 + min(abs(slip),2.)*0.3)*dt
        return True

    def correct(self, body_points):
        """Point-to-plane ICP: discard null directions instead of inventing motion
        from tangential nearest neighbours on a featureless plane.
        """
        points = voxel_downsample(body_points, 0.18, 1400)
        if len(points)<50:
            self.quality = 'insufficient_lidar'; return False
        prior = self.pose.copy()
        if len(self.submap)<60:
            self._insert(points); self.quality='wheel_imu'; return True
        target = self.submap[np.linalg.norm(self.submap-prior[:3,3],axis=1)<45]
        if len(target)<60:
            self.quality='no_submap_overlap'; return False
        tree = cKDTree(target)
        _, neighbours = tree.query(target,k=min(12,len(target)))
        patches = target[neighbours]
        centered = patches-patches.mean(axis=1)[:,None,:]
        eig, axes = np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered))
        normals = axes[:,:,0]
        planar = eig[:,0] < np.maximum(eig[:,1],1e-8)*0.2
        estimate = prior.copy()
        rank = 0
        for _ in range(8):
            world = transform_points(points,estimate)
            distance, indices = tree.query(world)
            valid = (distance<0.8)&planar[indices]
            if np.count_nonzero(valid)<40:
                self.quality='poor_lidar_overlap'; return False
            p,n,q = world[valid],normals[indices[valid]],target[indices[valid]]
            residual = np.einsum('ij,ij->i',n,p-q)
            keep = np.abs(residual)<=max(0.08,float(np.quantile(np.abs(residual),0.8)))+1e-9
            p,n,residual = p[keep],n[keep],residual[keep]
            # Rotation increment about current body origin, not map origin.
            A = np.column_stack([n,np.cross(p-estimate[:3,3],n)])
            u,s,vt = np.linalg.svd(A,full_matrices=False)
            observable = s>max(0.8,float(s[0])*0.05)
            rank = int(observable.sum())
            inv = np.divide(1.,s,out=np.zeros_like(s),where=observable)
            update = -vt.T @ (inv*(u.T@residual))
            if np.linalg.norm(update[:3])>0.4 or np.linalg.norm(update[3:])>0.15:
                self.quality='lidar_jump_rejected'; return False
            estimate[:3,3] += update[:3]
            estimate[:3,:3] = Rotation.from_rotvec(update[3:]).as_matrix() @ estimate[:3,:3]
            self.rmse = float(np.sqrt(np.mean(residual**2)))
            if np.linalg.norm(update)<1e-4: break
        if self.rmse>0.18 or np.linalg.norm(estimate[:3,3]-prior[:3,3])>0.5:
            self.quality='lidar_residual_rejected'; return False
        self.pose=estimate
        self.observed_rank=rank
        if rank==6: self.sigma=max(0.05,self.sigma*0.8)
        self.quality='lidar_fused' if rank==6 else 'degenerate_lidar_wheel_imu'
        self._insert(points)
        return True

    def _insert(self,points):
        if self.last_keyframe is not None:
            distance=np.linalg.norm(self.pose[:3,3]-self.last_keyframe[:3,3])
            angle=Rotation.from_matrix(self.pose[:3,:3]@self.last_keyframe[:3,:3].T).magnitude()
            if distance<0.25 and angle<0.08: return
        world=transform_points(points,self.pose)
        combined=np.vstack([self.submap,world])
        combined=combined[np.linalg.norm(combined-self.pose[:3,3],axis=1)<45]
        self.submap=voxel_downsample(combined,0.2,12000)
        self.last_keyframe=self.pose.copy()


def limited_speed(previous, requested, dt):
    return float(np.clip(requested,0.,min(0.5,previous+0.2*max(dt,0.))))
