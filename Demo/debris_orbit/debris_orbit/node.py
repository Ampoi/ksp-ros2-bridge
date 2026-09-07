"""Source-independent relative-target orbit guidance and image capture."""

from __future__ import annotations

import json
import math
import os
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped
from ksp_ros2_interfaces.msg import ControlSetpoint, RelativeTarget, VesselLifecycle
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .core import (
    Vector3,
    add,
    clamp_norm,
    body_orientation_for_sensor_direction,
    detumble_required,
    dot,
    cross,
    sanitize_ros_component,
    make_orbit_basis,
    norm,
    normalize,
    png_bytes,
    quaternion_error_vector,
    rotate_vector,
    safe_filename_component,
    search_direction,
    scale,
    subtract,
    unwrap_angle,
    vessel_topics,
)


class DebrisOrbitNode(Node):
    """Orbit a LiDAR/IMU relative target through the common controller."""

    def __init__(self, **kwargs) -> None:
        super().__init__("debris_orbit", **kwargs)
        self._declare_parameters()

        self.demo_instance_id = self._string("demo_instance_id")
        prefix = self._string("vessel_topic_prefix").rstrip("/")
        lidar_id = self._string("lidar_sensor_id").strip("/")
        camera_id = self._string("camera_sensor_id").strip("/")
        default_topics = vessel_topics(
            prefix, lidar_id, camera_id, self.demo_instance_id
        )
        target_topic = self._string("target_topic") or default_topics.demo_status.rsplit("/", 1)[0] + "/target"
        self.lidar_frame = self._string("lidar_frame") or (
            "ros2_ksp_" + sanitize_ros_component(lidar_id, "lidar_3d") + "_lidar_frame"
        )
        self.target_source = self._string("target_source")
        camera_topic = self._string("camera_topic") or default_topics.camera_image
        navigation = default_topics.demo_status.rsplit('/', 1)[0] + '/navigation'
        pose_topic = self._string("pose_topic") or navigation + '/pose'
        twist_topic = self._string("twist_topic") or navigation + '/twist'
        setpoint_topic = self._string("setpoint_topic") or default_topics.control_setpoint
        status_topic = self._string("status_topic") or default_topics.demo_status

        self.world_frame = self._string("world_frame")
        self.body_frame = self._string("body_frame")
        self.enabled = bool(self.get_parameter("enabled").value)
        self.orbit_radius = self._positive("orbit_radius")
        self.max_position_step = self._positive("max_position_step")
        self.angular_speed = math.radians(self._positive("angular_speed_deg_s"))
        self.orbit_direction = 1 if int(self.get_parameter("orbit_direction").value) >= 0 else -1
        plane_normal = tuple(self.get_parameter("orbit_plane_normal").value)
        self.plane_normal = normalize(plane_normal)  # type: ignore[arg-type]
        self.detumble_enter_rate = math.radians(
            self._positive("detumble_enter_rate_deg_s")
        )
        self.detumble_exit_rate = math.radians(
            self._positive("detumble_exit_rate_deg_s")
        )
        if self.detumble_exit_rate >= self.detumble_enter_rate:
            raise ValueError(
                "detumble_exit_rate_deg_s must be less than "
                "detumble_enter_rate_deg_s"
            )
        self.search_start_delay = self._nonnegative("search_start_delay_sec")
        self.search_yaw_amplitude = math.radians(self._positive("search_yaw_amplitude_deg"))
        self.search_pitch_amplitude = math.radians(self._positive("search_pitch_amplitude_deg"))
        self.search_period = self._positive("search_period_sec")
        self.search_memory_timeout = Duration(
            seconds=self._positive("search_memory_timeout_sec")
        )
        self.arrival_position_tolerance = self._positive(
            "arrival_position_tolerance"
        )
        self.arrival_speed_tolerance = self._positive("arrival_speed_tolerance")
        self.arrival_attitude_tolerance = math.radians(
            self._positive("arrival_attitude_tolerance_deg")
        )
        self.target_acquisition_samples = max(
            1, int(self.get_parameter("target_acquisition_samples").value)
        )
        self.state_timeout = Duration(seconds=self._positive("state_timeout_sec"))
        self.target_timeout = Duration(seconds=self._positive("target_timeout_sec"))
        self.pointing_tolerance = math.radians(self._positive("translation_pointing_tolerance_deg"))
        self.capture_step = math.radians(self._positive("capture_step_deg"))
        self.capture_count = max(1, int(self.get_parameter("capture_count").value))
        self.capture_tolerance = math.radians(self._positive("capture_tolerance_deg"))
        self.stop_after_capture = bool(self.get_parameter("stop_after_capture").value)
        output_directory = os.path.expandvars(
            os.path.expanduser(self._string("output_directory"))
        )
        self.output_directory = Path(output_directory).resolve() / safe_filename_component(
            self.demo_instance_id, 'vehicle') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        self.angle_history = deque(maxlen=200)
        self.capture_images = deque(maxlen=3)
        self.capture_sequence = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose: Optional[PoseStamped] = None
        self.twist: Optional[TwistStamped] = None
        self.pose_received: Optional[Time] = None
        self.twist_received: Optional[Time] = None
        self.target: Optional[Vector3] = None
        self.target_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.target_received: Optional[Time] = None
        self.target_stamp: Optional[Time] = None
        self.target_observations = 0
        self.last_target: Optional[Vector3] = None
        self.last_target_velocity: Vector3 = (0.0, 0.0, 0.0)
        self.last_target_stamp: Optional[Time] = None
        self.last_target_received: Optional[Time] = None
        self.lidar_mount_rotation = (0.0, 0.0, 0.0, 1.0)
        self.lidar_mount_known = False
        self.lidar_mount_translation = (0.0, 0.0, 0.0)
        self.lifecycle_key = None
        self.target_id = ""
        self.search_started: Optional[Time] = None
        self.search_reference_direction: Optional[Vector3] = None
        self.detumbling = False
        self.orbit_started: Optional[Time] = None
        self.basis_u: Optional[Vector3] = None
        self.basis_v: Optional[Vector3] = None
        self.actual_angle: Optional[float] = None
        self.next_capture_angle = 0.0
        self.pending_capture: Optional[Tuple[int, float]] = None
        self.saved_captures = 0
        self.complete = False
        self.last_status = ""
        self.next_progress_status: Optional[Time] = None

        self.setpoint_publisher = self.create_publisher(ControlSetpoint, setpoint_topic, 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(String, status_topic, status_qos)
        self.create_subscription(RelativeTarget, target_topic, self.receive_target, 10)
        self.create_subscription(VesselLifecycle, f"{prefix}/lifecycle", self.receive_lifecycle, status_qos)
        self.create_subscription(Image, camera_topic, self.receive_image, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, pose_topic, self.receive_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, twist_topic, self.receive_twist, qos_profile_sensor_data
        )
        rate = self._positive("control_rate_hz")
        self.create_timer(1.0 / rate, self.control)
        self.get_logger().info(
            f"demo_instance={self.demo_instance_id} target={target_topic} source={self.target_source} "
            f"camera={camera_topic} setpoint={setpoint_topic} "
            f"status={status_topic} enabled={self.enabled}"
        )
        self._publish_status("waiting_for_sensor_data")

    def _declare_parameters(self) -> None:
        parameters = {
            "enabled": False,
            "demo_instance_id": "demo_vehicle",
            "lidar_sensor_id": "front_lidar",
            "camera_sensor_id": "orbit_camera",
            "vessel_topic_prefix": "/ksp_vessel",
            "target_topic": "",
            "target_source": "lidar_imu",
            "lidar_frame": "",
            "translation_pointing_tolerance_deg": 8.0,
            "camera_topic": "",
            "pose_topic": "",
            "twist_topic": "",
            "setpoint_topic": "",
            "status_topic": "",
            "world_frame": "debris_inertial",
            "body_frame": "base_link",
            "orbit_radius": 15.0,
            "max_position_step": 2.0,
            "angular_speed_deg_s": 1.0,
            "orbit_direction": 1,
            "orbit_plane_normal": [0.0, 0.0, 1.0],
            "detumble_enter_rate_deg_s": 6.0,
            "detumble_exit_rate_deg_s": 2.0,
            "search_start_delay_sec": 0.3,
            "search_yaw_amplitude_deg": 180.0,
            "search_pitch_amplitude_deg": 20.0,
            "search_period_sec": 300.0,
            "search_memory_timeout_sec": 30.0,
            "arrival_position_tolerance": 1.5,
            "arrival_speed_tolerance": 0.25,
            "arrival_attitude_tolerance_deg": 5.0,
            "target_acquisition_samples": 3,
            "state_timeout_sec": 1.0,
            "target_timeout_sec": 3.0,
            "control_rate_hz": 20.0,
            "capture_step_deg": 36.0,
            "capture_count": 10,
            "capture_tolerance_deg": 2.0,
            "stop_after_capture": False,
            "output_directory": "debris_orbit_captures",
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _positive(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"parameter {name} must be finite and greater than zero")
        return value

    def _nonnegative(self, name: str) -> float:
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"parameter {name} must be finite and nonnegative")
        return value

    def receive_pose(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring pose in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        self.pose = message
        self.pose_received = self.get_clock().now()

    def receive_twist(self, message: TwistStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.world_frame:
            self.get_logger().warning(
                f"Ignoring twist in {message.header.frame_id}; expected {self.world_frame}",
                throttle_duration_sec=5.0,
            )
            return
        self.twist = message
        self.twist_received = self.get_clock().now()

    def receive_lifecycle(self, message: VesselLifecycle) -> None:
        key = (message.vessel_id, message.origin_sequence)
        active = message.state in (VesselLifecycle.STATE_ACTIVE, VesselLifecycle.STATE_CHANGED)
        if not active or key != self.lifecycle_key:
            self._reset_target_tracking()
            self.last_target = None
            self.pose = self.twist = None
            self.lidar_mount_known = False
            self.search_reference_direction = None
            self.complete = False
            self.saved_captures = 0
        self.lifecycle_key = key if active else None

    def receive_target(self, message: RelativeTarget) -> None:
        if (message.header.frame_id != self.world_frame
                or message.source != self.target_source
                or self.lifecycle_key != (message.observer_vessel_id, message.origin_sequence)):
            return
        stamp = Time.from_msg(message.header.stamp)
        if self.target_stamp is not None and stamp <= self.target_stamp:
            return
        position = message.relative_position
        velocity = message.relative_velocity
        relative = (position.x, position.y, position.z)
        relative_velocity = (velocity.x, velocity.y, velocity.z)
        if not all(math.isfinite(x) for x in relative + relative_velocity) or norm(relative) < 0.1:
            return
        if self.target_id and message.target_id != self.target_id:
            self._reset_target_tracking()
        self.target_id = message.target_id
        self.target = relative
        self.target_velocity = relative_velocity
        self.target_stamp = stamp
        self.target_received = self.get_clock().now()
        self.target_observations = message.observations
        self.last_target = relative
        self.last_target_velocity = relative_velocity
        self.last_target_stamp = stamp
        self.last_target_received = self.target_received
        self.search_started = None
        self.search_reference_direction = None

    def _refresh_lidar_mount(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(self.body_frame, self.lidar_frame, Time())
        except TransformException:
            self.lidar_mount_known = False
            return
        q = transform.transform.rotation
        t = transform.transform.translation
        self.lidar_mount_rotation = (q.x, q.y, q.z, q.w)
        self.lidar_mount_translation = (t.x, t.y, t.z)
        self.lidar_mount_known = True

    def _sensor_direction(self, target: Vector3) -> Vector3:
        sensor_position = add(self._pose_position(), rotate_vector(
            self._pose_quaternion(), self.lidar_mount_translation))
        return subtract(target, sensor_position)

    def receive_image(self, message: Image) -> None:
        if self.pending_capture is None or self.complete:
            return
        self.capture_images.append(message)
        self._try_capture_image()

    def _try_capture_image(self) -> None:
        if self.pending_capture is None or not self.angle_history:
            return
        capture_index, angle = self.pending_capture
        selected = None
        measured_angle = None
        for message in self.capture_images:
            stamp = Time.from_msg(message.header.stamp).nanoseconds
            for left, right in zip(self.angle_history, list(self.angle_history)[1:]):
                if left[0] <= stamp <= right[0] and right[0] > left[0]:
                    measured_angle = left[1] + (right[1]-left[1])*(stamp-left[0])/(right[0]-left[0])
                    if 0 <= measured_angle-angle <= self.capture_tolerance:
                        selected = message
                    break
            if selected is not None:
                break
        if selected is None:
            return
        message = selected
        try:
            image_data = png_bytes(
                message.width, message.height, message.encoding, message.step, bytes(message.data)
            )
            self.output_directory.mkdir(parents=True, exist_ok=True)
            platform_name = safe_filename_component(self.demo_instance_id, "vehicle")
            filename = (
                f"{platform_name}_detected_debris_"
                f"{int(round(math.degrees(angle))) % 360:03d}deg_{capture_index:05d}.png"
            )
            path = self.output_directory / filename
            path.write_bytes(image_data)
            path.with_suffix('.json').write_text(json.dumps({
                'source': self.target_source,
                'image_stamp_sec': message.header.stamp.sec + message.header.stamp.nanosec*1e-9,
                'image_frame': message.header.frame_id,
                'capture_sequence': capture_index,
                'requested_angle_deg': math.degrees(angle),
                'measured_angle_deg': math.degrees(measured_angle),
                'orbit': math.floor((angle + 1e-10) / (2*math.pi)),
            }, indent=2) + '\n')
        except (OSError, ValueError) as error:
            self.get_logger().error(f"Could not save capture: {error}")
            return
        self.pending_capture = None
        self.capture_images.clear()
        self.saved_captures += 1
        self.get_logger().info(f"Saved capture {self.saved_captures} at {math.degrees(angle):.0f} deg: {path}")
        if self.saved_captures == self.capture_count:
            self._publish_status("captures_complete", image_path=str(path))

    def _initialize_orbit(self, target: Vector3) -> bool:
        if self.pose is None:
            return False
        position = self._pose_position()
        radial = subtract(position, target)
        self.basis_u, self.basis_v, self.plane_normal = make_orbit_basis(
            radial, self.plane_normal, self.orbit_direction
        )
        self._publish_status("approaching_orbit")
        return True

    def control(self) -> None:
        now = self.get_clock().now()
        if not self.enabled:
            self._stop_detumble()
            self._publish_idle_setpoint(now)
            self._publish_status("disabled")
            return
        if self.complete and self.stop_after_capture:
            self._publish_idle_setpoint(now)
            return
        if not self._state_is_fresh(now):
            self._stop_detumble()
            self._publish_idle_setpoint(now)
            self._publish_status("waiting_for_fresh_pose_and_twist")
            return
        if self._update_detumble_mode(now):
            return
        self._refresh_lidar_mount()
        if not self.lidar_mount_known:
            self._publish_idle_setpoint(now)
            self._publish_status("waiting_for_lidar_transform")
            return
        if (
            self.target is None
            or self.target_received is None
            or now - self.target_received > self.target_timeout
            or self.target_stamp is None
            or abs((self._state_stamp() - self.target_stamp).nanoseconds) > self.target_timeout.nanoseconds
        ):
            self._reset_target_tracking()
            self._publish_search_setpoint(now)
            return
        if self.target_observations < self.target_acquisition_samples:
            target = self._predicted_target(self._state_stamp())
            if target is not None:
                self._publish_target_attitude_setpoint(now, target)
            self._publish_status(
                "acquiring_target",
                observations=self.target_observations,
                required_observations=self.target_acquisition_samples,
            )
            return
        target = self._predicted_target(self._state_stamp())
        if target is None:
            self._publish_search_setpoint(now)
            return
        if self.basis_u is None and not self._initialize_orbit(target):
            self._publish_idle_setpoint(now)
            return
        assert self.pose is not None and self.twist is not None and self.target is not None
        assert self.basis_u is not None and self.basis_v is not None

        if self.orbit_started is None:
            desired_angle = 0.0
        else:
            # Advance along the circle at the measured azimuth. A wall-clock
            # trajectory would run away while the craft aligns or is thrust-limited.
            radial = subtract(self._pose_position(), target)
            desired_angle = math.atan2(dot(radial, self.basis_v), dot(radial, self.basis_u))
        cosine = math.cos(desired_angle)
        sine = math.sin(desired_angle)
        radial_unit = add(scale(self.basis_u, cosine), scale(self.basis_v, sine))
        tangent_unit = add(scale(self.basis_u, -sine), scale(self.basis_v, cosine))
        desired_position = add(target, scale(radial_unit, self.orbit_radius))
        target_world_velocity = add(self._twist_linear(), self.target_velocity)
        desired_velocity = (
            target_world_velocity
            if self.orbit_started is None
            else add(
                target_world_velocity,
                scale(tangent_unit, self.orbit_radius * self.angular_speed),
            )
        )

        position = self._pose_position()
        velocity = self._twist_linear()
        position_error = subtract(desired_position, position)
        # Limit approach speed through the position/velocity loop. A distant
        # position step otherwise holds RCS at maximum until too late to brake.
        desired_position = add(position, clamp_norm(position_error, self.max_position_step))
        current_q = self._pose_quaternion()
        desired_q = body_orientation_for_sensor_direction(
            current_q, self.lidar_mount_rotation, self._sensor_direction(target)
        )
        attitude_error = quaternion_error_vector(desired_q, current_q)
        pointing_error = norm(attitude_error)
        if pointing_error > self.pointing_tolerance:
            # Brake relative translation until the real sensor axis is aligned.
            desired_position = position
            desired_velocity = target_world_velocity
        direction = self._sensor_direction(target)
        mount_velocity = cross(self._twist_angular(), rotate_vector(current_q, self.lidar_mount_translation))
        line_of_sight_rate = scale(cross(direction, subtract(self.target_velocity, mount_velocity)),
                                   1.0 / max(dot(direction, direction), 0.01))
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_SIX_DOF,
            desired_position,
            desired_q,
            desired_velocity,
            line_of_sight_rate,
        )
        if self.orbit_started is None:
            self._publish_progress_status(
                now,
                "approaching_orbit",
                target_range_m=round(norm(subtract(target, position)), 2),
                position_error_m=round(norm(position_error), 2),
                relative_speed_mps=round(
                    norm(self.target_velocity), 2
                ),
                attitude_error_deg=round(math.degrees(norm(attitude_error)), 1),
            )
            if (
                norm(position_error) <= self.arrival_position_tolerance
                and norm(self.target_velocity)
                <= self.arrival_speed_tolerance
                and norm(attitude_error) <= self.arrival_attitude_tolerance
            ):
                self.orbit_started = now
                self.actual_angle = 0.0
                self.next_capture_angle = self.capture_step
                self.pending_capture = (self.capture_sequence, 0.0)
                self.capture_sequence += 1
                self.angle_history.append((self._state_stamp().nanoseconds, 0.0))
                self._publish_status("orbiting")
            return
        self._publish_progress_status(now, "orbiting" if pointing_error <= self.pointing_tolerance else "aligning_lidar",
            target_range_m=round(norm(subtract(target, position)), 3),
            relative_speed_mps=round(norm(self.target_velocity), 3),
            orbit_progress_deg=round(math.degrees(self.actual_angle or 0.0), 2),
            attitude_error_deg=round(math.degrees(pointing_error), 2))
        self._update_capture_progress(position, target,
            allow_capture=pointing_error <= self.arrival_attitude_tolerance)

    def _publish_target_attitude_setpoint(self, now: Time, target: Vector3) -> None:
        direction = self._sensor_direction(target)
        desired_q = body_orientation_for_sensor_direction(
            self._pose_quaternion(), self.lidar_mount_rotation, direction
        )
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_ATTITUDE_HOLD,
            self._pose_position(),
            desired_q,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    def _publish_search_setpoint(self, now: Time) -> None:
        if not self.lidar_mount_known:
            self._publish_setpoint(
                now,
                ControlSetpoint.MODE_ATTITUDE_HOLD,
                self._pose_position(),
                self._pose_quaternion(),
                (0.0, 0.0, 0.0),
                (0.0, 0.0, 0.0),
            )
            self._publish_status("waiting_for_lidar_transform")
            return
        if self.search_started is None:
            self.search_started = now
        elapsed = max(0.0, (now - self.search_started).nanoseconds * 1.0e-9)
        if self.search_reference_direction is None:
            remembered = self._remembered_target(now)
            if remembered is not None:
                self.search_reference_direction = subtract(remembered, self._pose_position())
            else:
                sensor_world = rotate_vector(
                    self._pose_quaternion(),
                    rotate_vector(self.lidar_mount_rotation, (1.0, 0.0, 0.0)),
                )
                self.search_reference_direction = sensor_world
        scan_elapsed = max(0.0, elapsed - self.search_start_delay)
        direction = search_direction(
            self.search_reference_direction,
            self.plane_normal,
            scan_elapsed,
            self.search_period,
            0.0 if elapsed < self.search_start_delay else self.search_yaw_amplitude,
            0.0 if elapsed < self.search_start_delay else self.search_pitch_amplitude,
        )
        desired_q = body_orientation_for_sensor_direction(
            self._pose_quaternion(), self.lidar_mount_rotation, direction
        )
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_ATTITUDE_HOLD,
            self._pose_position(),
            desired_q,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        self._publish_status(
            "searching",
            elapsed_sec=round(elapsed, 1),
            remembered_target=self._remembered_target(now) is not None,
        )

    def _update_capture_progress(self, position: Vector3, target: Vector3, allow_capture=True) -> None:
        assert self.basis_u is not None and self.basis_v is not None
        radial = subtract(position, target)
        wrapped = math.atan2(dot(radial, self.basis_v), dot(radial, self.basis_u))
        if self.actual_angle is None:
            self.actual_angle = wrapped
        else:
            self.actual_angle = unwrap_angle(self.actual_angle, wrapped)
        self.angle_history.append((self._state_stamp().nanoseconds, self.actual_angle))
        self._try_capture_image()
        if self.stop_after_capture and self.actual_angle >= 2.0 * math.pi and self.saved_captures >= self.capture_count:
            self.complete = True
            self._publish_status("complete")
        if self.pending_capture is not None and self.actual_angle > self.pending_capture[1] + self.capture_tolerance + self.angular_speed:
            self._publish_status('capture_missed', angle_deg=math.degrees(self.pending_capture[1]))
            self.pending_capture = None
            self.capture_images.clear()
        if self.pending_capture is not None or self.complete:
            return
        if self.actual_angle >= self.next_capture_angle and allow_capture:
            self.pending_capture = (self.capture_sequence, self.next_capture_angle)
            self.capture_sequence += 1
            self.next_capture_angle += self.capture_step

    def _state_is_fresh(self, now: Time) -> bool:
        return bool(
            self.lifecycle_key is not None
            and self.pose is not None
            and self.twist is not None
            and self.pose_received is not None
            and self.twist_received is not None
            and now - self.pose_received <= self.state_timeout
            and now - self.twist_received <= self.state_timeout
        )

    def _update_detumble_mode(self, now: Time) -> bool:
        angular_velocity = self._twist_angular()
        angular_rate = norm(angular_velocity)
        required = detumble_required(
            self.detumbling,
            angular_rate,
            self.detumble_enter_rate,
            self.detumble_exit_rate,
        )
        if not required:
            self._stop_detumble()
            return False

        self.detumbling = True
        self._reset_target_tracking()
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_DETUMBLE,
            self._pose_position(),
            self._pose_quaternion(),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )
        self._publish_status(
            "detumbling",
            angular_rate_deg_s=round(math.degrees(angular_rate), 1),
        )
        return True

    def _stop_detumble(self) -> None:
        self.detumbling = False

    def _reset_target_tracking(self) -> None:
        self.target = None
        self.target_velocity = (0.0, 0.0, 0.0)
        self.target_received = None
        self.target_stamp = None
        self.target_observations = 0
        self.basis_u = None
        self.basis_v = None
        self.orbit_started = None
        self.actual_angle = None
        self.pending_capture = None
        self.angle_history.clear()
        self.capture_images.clear()
        self.next_progress_status = None

    def _publish_progress_status(self, now: Time, state: str, **extra: object) -> None:
        """Publish bounded-rate guidance diagnostics for live integration tests."""
        if self.next_progress_status is not None and now < self.next_progress_status:
            return
        self.next_progress_status = now + Duration(seconds=1.0)
        self._publish_status(state, **extra)

    def _remembered_target(self, now: Time) -> Optional[Vector3]:
        if (self.last_target is None or self.last_target_stamp is None
                or self.last_target_received is None
                or now - self.last_target_received > self.search_memory_timeout):
            return None
        dt = (self._state_stamp() - self.last_target_stamp).nanoseconds * 1.0e-9
        return add(self._pose_position(), add(self.last_target, scale(self.last_target_velocity, dt)))

    def _predicted_target(self, reference_stamp: Time) -> Optional[Vector3]:
        if self.target is None or self.target_stamp is None:
            return None
        # Extrapolate only relative motion: a 2200 m/s common orbital velocity
        # must never turn a 50 ms scheduling delay into 110 m of range error.
        dt = (reference_stamp - self.target_stamp).nanoseconds * 1.0e-9
        return add(self._pose_position(), add(self.target, scale(self.target_velocity, dt)))

    def _state_stamp(self) -> Time:
        assert self.pose is not None
        return Time.from_msg(self.pose.header.stamp)

    def _pose_position(self) -> Vector3:
        assert self.pose is not None
        p = self.pose.pose.position
        return (p.x, p.y, p.z)

    def _pose_quaternion(self) -> Tuple[float, float, float, float]:
        assert self.pose is not None
        q = self.pose.pose.orientation
        return (q.x, q.y, q.z, q.w)

    def _twist_linear(self) -> Vector3:
        assert self.twist is not None
        v = self.twist.twist.linear
        return (v.x, v.y, v.z)

    def _twist_angular(self) -> Vector3:
        assert self.twist is not None
        v = self.twist.twist.angular
        return (v.x, v.y, v.z)

    def _publish_setpoint(
        self,
        now: Time,
        mode: int,
        position: Vector3,
        orientation: Tuple[float, float, float, float],
        linear_velocity: Vector3,
        angular_velocity: Vector3,
    ) -> None:
        message = ControlSetpoint()
        message.header.stamp = (self._state_stamp() if self.pose is not None else now).to_msg()
        message.header.frame_id = self.world_frame
        message.mode = mode
        message.position.x, message.position.y, message.position.z = position
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = orientation
        (
            message.linear_velocity.x,
            message.linear_velocity.y,
            message.linear_velocity.z,
        ) = linear_velocity
        (
            message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z,
        ) = angular_velocity
        self.setpoint_publisher.publish(message)

    def _publish_idle_setpoint(self, now: Time) -> None:
        self._publish_setpoint(
            now,
            ControlSetpoint.MODE_IDLE,
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0),
        )

    def _publish_status(self, state: str, **extra: object) -> None:
        payload = {
            "state": state,
            "target_source": self.target_source,
            "target_id": self.target_id,
            "demo_instance_id": self.demo_instance_id,
            "captures": self.saved_captures,
            "capture_count": self.capture_count,
        }
        payload.update(extra)
        serialized = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        if serialized == self.last_status:
            return
        self.last_status = serialized
        message = String()
        message.data = serialized
        self.status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DebrisOrbitNode()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except RuntimeError:
        # Jazzy can surface a pybind take_message conversion error while the
        # launch service is invalidating subscriptions during Ctrl-C.
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
