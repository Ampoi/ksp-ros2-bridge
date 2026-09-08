"""Independent ground-truth observer. Never publishes navigation inputs."""
import json
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from collections import deque


def matrix(pose):
    p,q=pose.position,pose.orientation
    T=np.eye(4); T[:3,3]=[p.x,p.y,p.z]
    T[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
    return T


def stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9


class Evaluation(Node):
    def __init__(self):
        super().__init__('mun_rover_evaluation')
        self.truth=deque(maxlen=150); self.alignment=None; self.last_stamp=-1.
        self.pub=self.create_publisher(String,'/mun_rover/evaluation',10)
        self.create_subscription(PoseStamped,'/ksp_vessel/ground_truth/pose',lambda m:self.truth.append(m),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/mun_rover/odom_3d',self.estimate,qos_profile_sensor_data)
        from ksp_ros2_interfaces.msg import VesselLifecycle
        from rclpy.qos import QoSProfile,DurabilityPolicy
        self.epoch=None; self.rear_x=None; self.previous=None; self.truth_speed=0.
        self.create_subscription(String,'/mun_rover/geometry',self.geometry,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(VesselLifecycle,'/ksp_vessel/lifecycle',self.lifecycle,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def geometry(self,msg):
        self.rear_x=float(json.loads(msg.data)['rear_x'])
    def lifecycle(self,msg):
        epoch=(msg.vessel_id,msg.generation)
        if self.epoch!=epoch: self.truth.clear(); self.alignment=None; self.previous=None
        self.epoch=epoch
    def estimate(self,msg):
        if not self.truth:return
        truth=min(self.truth,key=lambda m:abs(stamp(m)-stamp(msg)))
        if abs(stamp(truth)-stamp(msg))>0.05:return
        est=matrix(msg.pose.pose); actual=matrix(truth.pose)
        if self.alignment is None: self.alignment=est@np.linalg.inv(actual)
        aligned=self.alignment@actual
        error=np.linalg.norm(est[:3,3]-aligned[:3,3])
        angle=Rotation.from_matrix(est[:3,:3]@aligned[:3,:3].T).magnitude()*180/np.pi
        report=dict(stamp=stamp(msg),position_error=float(error),attitude_error_deg=float(angle),
            truth_position=aligned[:3,3].tolist(),estimated_position=est[:3,3].tolist())
        if self.rear_x is not None:
            rear=aligned[:3,3]+aligned[:3,:3]@np.array([self.rear_x,0.,0.])
            report['truth_rear_pose']=[float(rear[0]),float(rear[1]),float(np.arctan2(aligned[1,0],aligned[0,0]))]
        if self.previous is not None and stamp(truth)>self.previous[0]:
            self.truth_speed=float(np.linalg.norm(actual[:3,3]-self.previous[1])/(stamp(truth)-self.previous[0]))
        if self.previous is None or stamp(truth)>self.previous[0]: self.previous=(stamp(truth),actual[:3,3].copy())
        report['truth_speed']=self.truth_speed
        self.pub.publish(String(data=json.dumps(report)))


def main(args=None):
    rclpy.init(args=args);node=Evaluation()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
