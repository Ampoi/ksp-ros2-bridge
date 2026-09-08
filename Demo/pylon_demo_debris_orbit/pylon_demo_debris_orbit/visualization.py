"""RViz view centred on the estimated target, separate from the truth TF tree."""
import math
from collections import deque

from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from .core import add, make_orbit_basis, scale, sanitize_ros_component


class TargetView:
    def __init__(self, node):
        self.node = node
        self.frame = 'pylon_debris_view_' + sanitize_ros_component(node.value('demo_instance_id'), 'demo_vehicle')
        self.markers = node.create_publisher(MarkerArray, node.namespace + '/markers', 10)
        self.path = node.create_publisher(Path, node.namespace + '/path', 10)
        self.tf = TransformBroadcaster(node)
        # Keep up to 30 minutes at 2 Hz; a 1 deg/s orbit takes six minutes.
        # Publishing every scan would both truncate the circle and repeatedly
        # serialize thousands of poses unnecessarily.
        self.history = deque(maxlen=3600)
        self.last_history_time = None
        self.last_time = None

    def reset(self):
        self.history.clear()
        self.last_history_time = None
        self.last_time = None

    def publish(self, stamp, relative, orientation=(0., 0., 0., 1.)):
        now = stamp.sec + stamp.nanosec * 1e-9
        if self.last_time is not None and now - self.last_time < .1:
            return
        self.last_time = now
        header = Header(stamp=stamp, frame_id=self.frame)
        observer = [-float(x) for x in relative]
        pose = PoseStamped(header=header)
        pose.pose.position = Point(x=observer[0], y=observer[1], z=observer[2])
        q = pose.pose.orientation; q.x, q.y, q.z, q.w = orientation
        if self.last_history_time is None or now - self.last_history_time >= .5:
            self.last_history_time = now
            self.history.append(pose)
            self.path.publish(Path(header=header, poses=list(self.history)))
        tf = TransformStamped(header=header, child_frame_id=self.frame + '_chaser')
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = observer
        tf.transform.rotation = q
        self.tf.sendTransform(tf)
        markers = []
        for index, (position, color, size) in enumerate(((Point(), (.15, .9, 1.), .6),
                                                       (pose.pose.position, (1., .7, .15), .7))):
            marker = Marker(header=header, ns='bodies', id=index, type=Marker.SPHERE, action=Marker.ADD)
            marker.pose.position = position; marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = size
            marker.color.r, marker.color.g, marker.color.b = color; marker.color.a = 1.
            marker.lifetime.sec = 1
            markers.append(marker)
        line = Marker(header=header, ns='line_of_sight', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        line.pose.orientation.w = 1.; line.scale.x = .05
        line.color.g = .8; line.color.b = 1.; line.color.a = .8
        line.points = [pose.pose.position, Point()]; line.lifetime.sec = 1
        markers.append(line)
        ring = Marker(header=header, ns='orbit', id=0, type=Marker.LINE_STRIP, action=Marker.ADD)
        ring.pose.orientation.w = 1.; ring.scale.x = .06
        ring.color.r = .7; ring.color.g = .7; ring.color.b = .7; ring.color.a = .7
        u, v, _ = make_orbit_basis((1., 0., 0.), tuple(self.node.value('orbit_plane_normal')), 1)
        for i in range(129):
            angle = i * 2 * math.pi / 128
            point = scale(add(scale(u, math.cos(angle)), scale(v, math.sin(angle))), self.node.value('orbit_radius'))
            ring.points.append(Point(x=point[0], y=point[1], z=point[2]))
        ring.lifetime.sec = 1; markers.append(ring)
        self.markers.publish(MarkerArray(markers=markers))
