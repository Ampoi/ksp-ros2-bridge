"""Republish replaceable sensor mounts as one transient-local TF snapshot."""

from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage


class StaticTransformSnapshot:
    def __init__(self, node):
        self.publisher = node.create_publisher(
            TFMessage, '/tf_static', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.transforms = {}

    def clear(self):
        self.transforms.clear()
        self.publisher.publish(TFMessage())

    def sendTransform(self, transforms):
        if not isinstance(transforms, list):
            transforms = [transforms]
        for transform in transforms:
            self.transforms[transform.child_frame_id] = transform
        self.publisher.publish(TFMessage(transforms=list(self.transforms.values())))
