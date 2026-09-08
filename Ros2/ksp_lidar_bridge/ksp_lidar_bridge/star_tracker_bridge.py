"""ROS publication and loss-of-transport handling for optical attitude sensors."""
from dataclasses import replace
import time

from geometry_msgs.msg import QuaternionStamped
from ksp_ros2_interfaces.msg import StarTrackerState

from .star_tracker_packets import star_tracker_from_packet


class StarTrackerBridgeMixin:
    def publish_star_tracker(self, packet):
        try:
            data = star_tracker_from_packet(packet)
        except ValueError as exc:
            self.get_logger().warning(f'Dropped invalid star tracker packet: {exc}')
            return
        if not hasattr(self, 'star_trackers'):
            self.star_trackers = {}
        key = data.sensor_id
        previous = self.star_trackers.get(key)
        # Reject UDP duplication/reordering, including replay of a valid sample
        # after a later loss-of-lock message. Sequence continues over quickload.
        if previous is not None:
            old = previous[0]
            if old.session_id == data.session_id and data.sequence <= old.sequence:
                return
        active = getattr(self, 'active_vessel_id', '')
        if active and active != data.vessel_id:
            return
        if previous is None and len(self.star_trackers) >= 128:
            self.get_logger().warning('Star tracker publisher limit reached')
            return
        now = time.monotonic()
        self.star_trackers[key] = (data, now, False)
        stamp = self.stamp_for_packet(packet)
        self.emit_star_tracker(data, stamp)

    def emit_star_tracker(self, data, stamp):
        prefix = f'{self.topic_prefix}/star_tracker/{data.sensor_id}'
        state = StarTrackerState()
        state.header.stamp = stamp
        state.header.frame_id = 'kerbol_inertial'
        state.vessel_id = data.vessel_id
        state.sensor_id = data.sensor_id
        state.session_id = data.session_id
        state.sequence = data.sequence
        state.universal_time = data.universal_time
        state.measured_frame_id = 'base_link'
        state.valid = data.valid
        state.reason = data.reason
        state.angular_rate_deg_sec = data.angular_rate_deg_sec
        # Explicitly zero even if the generated message defaults ever change.
        state.orientation.x = state.orientation.y = state.orientation.z = state.orientation.w = 0.0
        state.orientation_covariance[0] = -1.0
        if data.valid:
            (state.orientation.x, state.orientation.y,
             state.orientation.z, state.orientation.w) = data.orientation
            variance = data.noise_std_rad**2
            state.orientation_covariance = [variance, 0., 0., 0., variance, 0., 0., 0., variance]
        publisher = self.publisher_for(prefix + '/state', 'StarTrackerState', StarTrackerState)
        if publisher is not None:
            publisher.publish(state)
        if data.valid:
            attitude = QuaternionStamped()
            attitude.header = state.header
            attitude.quaternion = state.orientation
            publisher = self.publisher_for(prefix + '/attitude', 'QuaternionStamped', QuaternionStamped)
            if publisher is not None:
                publisher.publish(attitude)

    def expire_star_trackers(self):
        now = time.monotonic()
        for key, (data, seen, stale) in list(getattr(self, 'star_trackers', {}).items()):
            active = getattr(self, 'active_vessel_id', '')
            switched = bool(active and active != data.vessel_id)
            if switched or now - seen > self.args.topic_timeout_sec:
                invalid = replace(data, valid=False, reason='inactive' if switched else 'stale', orientation=None)
                # Continue the invalid heartbeat while transport is missing, so a
                # late subscriber also learns the loss rather than seeing silence.
                self.emit_star_tracker(invalid, self.get_clock().now().to_msg())
                self.star_trackers[key] = (data, seen, True)
                if now - seen > max(30., self.args.topic_timeout_sec * 3):
                    prefix = f'{self.topic_prefix}/star_tracker/{key}'
                    for suffix in ('/state', '/attitude'):
                        self.remove_publisher(prefix + suffix, 'star tracker expired')
                    del self.star_trackers[key]
