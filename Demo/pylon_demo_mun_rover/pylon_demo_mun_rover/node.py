"""Sensor-only rover runtime and guarded public NavigateToPose action."""
from collections import deque
import json
import math
import time
import threading
import copy
from functools import wraps
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist, TransformStamped, Polygon, Point32
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, StaticTransformBroadcaster, TransformException
from pylon_interfaces.msg import WheelState, WheelCommand, ControlAuthorityState, ControlAuthorityCommand, VesselLifecycle
from pylon_vehicle_control.application.lease import LeaseCoordinator
from pylon_perception.points import transform_points
from .core import Geometry, Wheel, Estimator, limited_speed
from .terrain import Terrain


def seconds(stamp): return stamp.sec+stamp.nanosec*1e-9

def xyz(v): return [v.x,v.y,v.z]


def locked(method):
    @wraps(method)
    def call(self,*args,**kwargs):
        with self.lock: return method(self,*args,**kwargs)
    return call


class Rover(Node):
    def __init__(self):
        super().__init__('pylon_mun_rover')
        self.lock=threading.RLock(); self.reset_serial=0
        self.cloud_group=MutuallyExclusiveCallbackGroup()
        self.declare_parameter('lidar_sensor_id','front_lidar')
        self.declare_parameter('lidar_topic','')
        self.declare_parameter('position_sigma_limit',0.6)
        self.declare_parameter('footprint_margin',0.5)
        self.declare_parameter('command_timeout',0.5)
        self.qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.group=ReentrantCallbackGroup()
        self.buffer=Buffer(); self.listener=TransformListener(self.buffer,self)
        self.tf=TransformBroadcaster(self); self.static_tf=StaticTransformBroadcaster(self)
        identity=TransformStamped(); identity.header.frame_id='map'; identity.child_frame_id='pylon_rover_odom'; identity.transform.rotation.w=1.
        self.static_tf.sendTransform(identity)
        self.odom_pub=self.create_publisher(Odometry,'/pylon/mun_rover/odom',10)
        self.pose_pub=self.create_publisher(Odometry,'/pylon/mun_rover/odom_3d',10)
        self.map_pub=self.create_publisher(OccupancyGrid,'/pylon/mun_rover/map',self.qos)
        self.cloud_pub=self.create_publisher(PointCloud2,'/pylon/mun_rover/points',qos_profile_sensor_data)
        self.ground_pub=self.create_publisher(PointCloud2,'/pylon/mun_rover/ground',qos_profile_sensor_data)
        self.obstacle_pub=self.create_publisher(PointCloud2,'/pylon/mun_rover/obstacles',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/pylon/mun_rover/status',self.qos)
        self.marker_pub=self.create_publisher(Marker,'/pylon/mun_rover/status_marker',10)
        self.trail_pub=self.create_publisher(Path,'/pylon/mun_rover/trajectory',10)
        self.geometry_pub=self.create_publisher(String,'/pylon/mun_rover/geometry',self.qos)
        self.wheel_pub=self.create_publisher(WheelCommand,'/ksp_vessel/actuators/wheel/command',10)
        self.authority_pub=self.create_publisher(ControlAuthorityCommand,'/ksp_vessel/control/authority/command',10)
        self.footprint_pubs=[self.create_publisher(Polygon,f'/pylon/mun_rover/{name}_costmap/footprint',10) for name in ('local','global')]
        self.nav_client=ActionClient(self,NavigateToPose,'/pylon/mun_rover/navigate_to_pose',callback_group=self.group)
        self.server=ActionServer(self,NavigateToPose,'/navigate_to_pose',self.execute,
            goal_callback=self.accept_goal,cancel_callback=self.cancel_goal,callback_group=self.group)
        self.create_service(Trigger,'/pylon/mun_rover/reset',self.reset_service)
        self.create_subscription(VesselLifecycle,'/ksp_vessel/lifecycle',self.lifecycle,self.qos)
        self.create_subscription(ControlAuthorityState,'/ksp_vessel/control/authority/state',self.authority,self.qos)
        self.create_subscription(WheelState,'/ksp_vessel/actuators/wheel/state',self.wheel,qos_profile_sensor_data)
        self.create_subscription(Imu,'/ksp_vessel/imu/data_raw',self.imu,qos_profile_sensor_data)
        topic=self.get_parameter('lidar_topic').value or '/ksp_vessel/lidar_3d/'+self.get_parameter('lidar_sensor_id').value+'/points'
        self.create_subscription(PointCloud2,topic,self.cloud,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT),callback_group=self.cloud_group)
        self.create_subscription(Twist,'/pylon/mun_rover/cmd_vel',self.command,10)
        self.param_client=AsyncParameterClient(self,'/pylon/mun_rover/planner_server',callback_group=self.group)
        self.sequence=0; self.epoch=None; self.vessel_id=''; self.vessel_active=False
        self.lease=LeaseCoordinator('pylon_mun_rover',0.4)
        self.active_goal=None; self.goal_pending=False; self.child_goal=None; self.fault=''; self.command_time=0.
        self.started_at=0.; self.target=(0.,0.); self.sent_speed=0.; self.last_tick=time.monotonic()
        self.reset_state()
        self.create_timer(0.05,self.tick)
        self.create_timer(1.,self.report)
        self.get_logger().info('Waiting for all wheel geometry, stationary Mun IMU and 3D terrain. Public goal: /navigate_to_pose')

    def reset_state(self):
        self.reset_serial+=1
        self.est=Estimator(); self.terrain=Terrain(); self.geometry=None
        self.wheels={}; self.wheel_times={}; self.wheel_stamps={}; self.imu_samples=deque(maxlen=45)
        self.imu_time=0.; self.last_imu_stamp=None; self.cloud_time=0.; self.cloud_stamp=-1.
        self.gyro=np.zeros(3); self.speed=0.; self.slip=0.; self.poses=deque(maxlen=150)
        self.configured=False; self.config_future=None; self.configured_at=0.
        self.trail=Path(); self.trail.header.frame_id='map'; self.last_trail=0.
        self.last_report='waiting_for_sensors'; self.last_stamp=None; self.map_time=0.
        self.cloud_duration=0.; self.mount_age=0.; self.cloud_interval=0.
        self.publish_map(self.get_clock().now().to_msg())

    @locked
    def lifecycle(self,msg):
        epoch=(msg.vessel_id,msg.generation)
        if epoch!=self.epoch:
            if self.epoch is not None: self.stop('lifecycle_changed')
            self.buffer.clear()
            self.reset_state(); self.epoch=epoch
            self.lease.observe_vessel('',False)
        self.vessel_id=msg.vessel_id
        self.vessel_active=msg.state in (VesselLifecycle.STATE_ACTIVE,VesselLifecycle.STATE_CHANGED)
        if not self.vessel_active: self.stop('vessel_unavailable')
        self.lease.observe_vessel(self.vessel_id,self.vessel_active,msg.generation)

    @locked
    def authority(self,msg):
        was=self.lease.owned
        self.lease.observe_authority(msg.vessel_id,msg.controller_id,msg.lease_id,msg.state==ControlAuthorityState.STATE_OWNED)
        if self.active_goal and was and not self.lease.owned: self.stop('authority_lost')
        if msg.state==ControlAuthorityState.STATE_EMERGENCY_STOP: self.stop('emergency_stop')

    @locked
    def wheel(self,msg):
        if msg.vessel_id!=self.vessel_id or msg.radius<=0: return
        stamp=seconds(msg.header.stamp)
        if stamp<=self.wheel_stamps.get(msg.id,-1.): return
        self.wheel_stamps[msg.id]=stamp; self.wheel_times[msg.id]=time.monotonic(); self.wheels[msg.id]=msg
        if self.geometry is None and len(self.wheels)==msg.wheel_count:
            try:
                wheels=[Wheel(m.id,*xyz(m.position),m.radius,m.rolling_sign,m.steering_sign,m.steering_enabled,m.max_steering_angle) for m in self.wheels.values()]
                self.geometry=Geometry(wheels,xyz(msg.body_min),xyz(msg.body_max),self.get_parameter('footprint_margin').value)
                self.geometry_pub.publish(String(data=json.dumps(dict(minimum_turning_radius=self.geometry.min_radius,
                    rear_x=self.geometry.rear_x,footprint=self.geometry.footprint,wheels=[w.__dict__ for w in self.geometry.wheels]))))
                self.get_logger().info(f'Geometry ready: {len(wheels)} wheels, minimum radius {self.geometry.min_radius:.2f} m')
            except ValueError as e: self.last_report=str(e)

    @locked
    def imu(self,msg):
        stamp=seconds(msg.header.stamp)
        if self.last_imu_stamp is not None and stamp<=self.last_imu_stamp: return
        now=time.monotonic(); gyro=np.array(xyz(msg.angular_velocity)); accel=np.array(xyz(msg.linear_acceleration))
        if not np.isfinite([gyro,accel]).all(): self.stop('nonfinite_imu'); return
        dt=0. if self.last_imu_stamp is None else stamp-self.last_imu_stamp
        self.last_imu_stamp=stamp; self.imu_time=now; self.gyro=gyro; self.last_stamp=msg.header.stamp
        if self.geometry is None or not self.wheels_fresh(now): return
        state={k:(v.angular_velocity,v.slip) for k,v in self.wheels.items()}
        self.speed,self.slip=self.geometry.speed(state,float((gyro-self.est.gyro_bias)[2]))
        self.slip=max(self.slip,float(np.median([abs(m.slip) for m in self.wheels.values()])))
        if not self.est.initialized:
            if abs(self.speed)<0.02 and all(m.grounded for m in self.wheels.values()): self.imu_samples.append((accel,gyro))
            else: self.imu_samples.clear()
            if len(self.imu_samples)>=30:
                self.est.initialize([x[0] for x in self.imu_samples],[x[1] for x in self.imu_samples])
                if self.est.initialized: self.est.pose[2,3]=self.geometry.height
        elif 0<dt<=0.2:
            self.est.predict(gyro,self.speed,self.geometry.rear_x,dt,self.slip)
        elif dt>0.2:
            self.est.sigma=1.0; self.stop('imu_time_gap_reset_required')
        if self.est.initialized:
            self.poses.append((stamp,self.est.pose.copy()))
            self.publish_pose(msg.header.stamp)

    def wheels_fresh(self,now):
        return self.geometry is not None and all(now-self.wheel_times.get(w.id,0)<0.5 for w in self.geometry.wheels)

    def cloud(self,msg):
        begun=time.monotonic()
        stamp=seconds(msg.header.stamp)
        with self.lock:
            if not self.est.initialized or not self.poses or stamp<=self.cloud_stamp: return
            self.cloud_stamp=stamp
            sample=min(self.poses,key=lambda item:abs(item[0]-stamp))
            if abs(sample[0]-stamp)>0.15: self.last_report='lidar_imu_time_mismatch'; return
            serial=self.reset_serial; work=copy.deepcopy(self.est)
            work.pose=sample[1].copy(); geometry=self.geometry
            update_map=time.monotonic()-self.map_time>=0.5
            terrain=copy.deepcopy(self.terrain) if update_map else None
            stationary=abs(self.speed)<0.02 and all(m.grounded for m in self.wheels.values())
        try:
            # Fixed sensor mounts may arrive a tick after the scan. Use only a
            # recent relative mount transform, never a world/ground-truth pose.
            try:
                tf=self.buffer.lookup_transform('base_link',msg.header.frame_id,Time.from_msg(msg.header.stamp))
            except TransformException:
                tf=self.buffer.lookup_transform('base_link',msg.header.frame_id,Time())
            self.mount_age=abs(seconds(tf.header.stamp)-stamp)
            if seconds(tf.header.stamp)!=0 and self.mount_age>0.5:
                self.last_report='stale_sensor_mount_transform'; return
            t=tf.transform
            extrinsic=np.eye(4); extrinsic[:3,:3]=Rotation.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
            extrinsic[:3,3]=xyz(t.translation)
            data=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
            points=np.asarray(data,float).reshape(-1,3)
            points=points[(np.linalg.norm(points,axis=1)>0.3)&(np.linalg.norm(points,axis=1)<45)]
            body=transform_points(points,extrinsic)
            accepted=work.correct(body); at_scan=work.pose.copy()
            if not accepted:
                self.last_report=work.quality; return
            correction=at_scan@np.linalg.inv(sample[1])
            world=transform_points(body,at_scan)
            if terrain is not None:
                if not terrain.bootstrapped and stationary: terrain.seed_contact_patch(at_scan,geometry)
                terrain.update(world)
            with self.lock:
                if serial!=self.reset_serial: return
                self.est.pose=correction@self.est.pose
                self.poses=deque([(t,correction@p) for t,p in self.poses],maxlen=150)
                self.est.submap=work.submap; self.est.last_keyframe=work.last_keyframe
                self.est.quality=work.quality; self.est.rmse=work.rmse; self.est.observed_rank=work.observed_rank
                if work.observed_rank==6:self.est.sigma=max(0.05,self.est.sigma*0.8)
                self.cloud_interval=time.monotonic()-self.cloud_time
                self.cloud_time=time.monotonic(); self.cloud_duration=self.cloud_time-begun
                self.last_report=work.quality
                if terrain is not None:self.terrain=terrain;self.map_time=time.monotonic()
            h=Header(stamp=msg.header.stamp,frame_id='map')
            self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(h,world.astype(np.float32)))
            if terrain is not None:
                self.ground_pub.publish(point_cloud2.create_cloud_xyz32(h,terrain.ground.astype(np.float32)))
                self.obstacle_pub.publish(point_cloud2.create_cloud_xyz32(h,terrain.obstacles.astype(np.float32)))
                self.publish_map(msg.header.stamp)
        except (TransformException,ValueError,TypeError) as e:
            self.last_report='pointcloud: '+str(e)

    def publish_pose(self,stamp):
        R=self.est.pose[:3,:3]; yaw=math.atan2(R[1,0],R[0,0])
        position=self.est.pose[:3,3]; rear=position+R@np.array([self.geometry.rear_x,0.,0.])
        qyaw=Rotation.from_euler('z',yaw).as_quat()
        # footprint lies in the start tangent plane. The child carries true
        # height, roll/pitch and the rear-axle to CoM offset.
        tf=TransformStamped(); tf.header=Header(stamp=stamp,frame_id='pylon_rover_odom'); tf.child_frame_id='pylon_rover_base_footprint'
        tf.transform.translation.x=float(rear[0]); tf.transform.translation.y=float(rear[1])
        tf.transform.rotation.x,tf.transform.rotation.y,tf.transform.rotation.z,tf.transform.rotation.w=map(float,qyaw)
        child=TransformStamped(); child.header=Header(stamp=stamp,frame_id='pylon_rover_base_footprint'); child.child_frame_id='pylon_rover_base_link'
        flat=Rotation.from_euler('z',yaw)
        local=flat.inv().apply(position-np.array([rear[0],rear[1],0.]))
        child.transform.translation.x,child.transform.translation.y,child.transform.translation.z=map(float,local)
        quat=(flat.inv()*Rotation.from_matrix(R)).as_quat()
        child.transform.rotation.x,child.transform.rotation.y,child.transform.rotation.z,child.transform.rotation.w=map(float,quat)
        self.tf.sendTransform([tf,child])
        msg=Odometry(); msg.header=Header(stamp=stamp,frame_id='pylon_rover_odom'); msg.child_frame_id='pylon_rover_base_footprint'
        msg.pose.pose.position.x=float(rear[0]); msg.pose.pose.position.y=float(rear[1]); msg.pose.pose.orientation=tf.transform.rotation
        msg.twist.twist.linear.x=float(self.speed); msg.twist.twist.angular.z=float((self.gyro-self.est.gyro_bias)[2])
        for i in (0,7,14): msg.pose.covariance[i]=self.est.sigma**2
        msg.pose.covariance[35]=math.radians(3)**2
        self.odom_pub.publish(msg)
        full=Odometry(); full.header=msg.header; full.child_frame_id='pylon_rover_base_link'
        full.pose.pose.position.x,full.pose.pose.position.y,full.pose.pose.position.z=map(float,position)
        quat=Rotation.from_matrix(R).as_quat()
        full.pose.pose.orientation.x,full.pose.pose.orientation.y,full.pose.pose.orientation.z,full.pose.pose.orientation.w=map(float,quat)
        full.pose.covariance=msg.pose.covariance; self.pose_pub.publish(full)
        if time.monotonic()-self.last_trail>0.5:
            from geometry_msgs.msg import PoseStamped
            self.trail.poses.append(PoseStamped(header=msg.header,pose=msg.pose.pose))
            self.trail.poses=self.trail.poses[-2000:]; self.trail.header.stamp=stamp
            self.trail_pub.publish(self.trail); self.last_trail=time.monotonic()

    def publish_map(self,stamp):
        msg=OccupancyGrid(); msg.header=Header(stamp=stamp,frame_id='map')
        msg.info.resolution=self.terrain.resolution; msg.info.width=self.terrain.n; msg.info.height=self.terrain.n
        msg.info.origin.position.x=-self.terrain.size/2; msg.info.origin.position.y=-self.terrain.size/2
        msg.info.origin.orientation.w=1.; msg.data=self.terrain.grid.ravel().tolist()
        self.map_pub.publish(msg)

    def ready_reason(self):
        now=time.monotonic()
        if not self.vessel_active: return 'waiting_for_vessel'
        if self.geometry is None: return self.last_report
        if not self.wheels_fresh(now): return 'wheel_timeout'
        if not all(m.enabled and m.grounded for m in self.wheels.values()): return 'wheel_disabled_or_airborne'
        if now-self.imu_time>0.5: return 'imu_timeout'
        if not self.est.initialized: return 'waiting_for_stationary_imu_on_Mun'
        if now-self.cloud_time>0.5: return 'lidar_timeout: '+self.last_report
        if self.est.sigma>self.get_parameter('position_sigma_limit').value: return 'position_uncertain'
        up=self.est.pose[:3,2]
        if up[2]<math.cos(math.radians(20)): return 'excessive_tilt'
        if not self.configured or now-self.configured_at<2: return 'configuring_nav2_geometry'
        return ''

    def configure(self):
        if self.geometry is None or self.configured: return
        if self.config_future is not None:
            if not self.config_future.done(): return
            try:
                results=self.config_future.result().results
                if results and all(r.successful for r in results):
                    self.configured=True; self.configured_at=time.monotonic()
            except Exception as e: self.last_report='nav2 configuration: '+str(e)
            self.config_future=None
        elif self.param_client.services_are_ready():
            self.config_future=self.param_client.set_parameters([Parameter('GridBased.minimum_turning_radius',value=float(self.geometry.min_radius))])

    @locked
    def command(self,msg):
        if self.active_goal is None or self.fault: return
        if not np.isfinite([msg.linear.x,msg.linear.y,msg.angular.z]).all() or msg.linear.x < -0.001 or abs(msg.linear.y)>0.001:
            self.stop('unsupported_velocity'); return
        self.target=(max(0.,msg.linear.x),msg.angular.z); self.command_time=time.monotonic()

    @locked
    def send_authority(self,action):
        if action is None: return
        self.sequence+=1
        msg=ControlAuthorityCommand(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.action=getattr(ControlAuthorityCommand,'ACTION_'+action.action.upper()); msg.vessel_id=action.vessel_id; msg.controller_id=action.controller_id
        msg.lease_id=action.lease_id; msg.sequence=self.sequence; msg.priority=80
        msg.lease_duration_sec=2.; msg.suppress_sas=True; self.authority_pub.publish(msg)

    @locked
    def send_wheels(self,speed,omega,brake):
        if self.geometry is None or not self.lease.owned: return
        commands=self.geometry.commands(speed,omega)
        for w in self.geometry.wheels:
            self.sequence+=1
            msg=WheelCommand(); msg.header.stamp=self.get_clock().now().to_msg(); msg.header.frame_id='base_link'
            msg.vessel_id=self.lease.vessel_id; msg.controller_id=self.lease.controller_id; msg.lease_id=self.lease.lease_id
            msg.sequence=self.sequence; msg.id=w.id; msg.enabled=True
            msg.target_angular_velocity,msg.steering_angle=commands[w.id]
            msg.brake=float(brake); msg.timeout_sec=0.25
            self.wheel_pub.publish(msg)

    @locked
    def tick(self):
        now=time.monotonic(); dt=now-self.last_tick; self.last_tick=now
        self.configure()
        if self.geometry:
            p=Polygon(points=[Point32(x=float(x),y=float(y),z=0.) for x,y in self.geometry.footprint])
            for publisher in self.footprint_pubs: publisher.publish(p)
        if self.active_goal is None:
            if self.lease.owned:
                self.send_wheels(0.,0.,1.); self.send_authority(self.lease.release_action())
            self.sent_speed=0.; return
        reason=self.ready_reason()
        if reason: self.stop(reason)
        if self.command_time and now-self.command_time>0.5: self.stop('cmd_vel_timeout')
        if not self.command_time and now-self.started_at>10: self.stop('no_nav2_command')
        if self.fault:
            self.send_wheels(0.,0.,1.)
            self.send_authority(self.lease.release_action()); return
        if not self.command_time: return
        self.send_authority(self.lease.due_action(now))
        v,w=self.target
        v=limited_speed(self.sent_speed,v,dt)
        w=float(np.clip(w,-v/self.geometry.min_radius,v/self.geometry.min_radius))
        if v>0.005:
            R=self.est.pose[:3,:3]; rear=self.est.pose[:3,3]+R@np.array([self.geometry.rear_x,0.,0.])
            yaw=math.atan2(R[1,0],R[0,0])
            if not self.terrain.corridor_safe(rear[:2],yaw,self.geometry.footprint,v,w/v,distance=max(0.5,v*0.5+v*v/0.4)):
                self.stop('unobserved_or_blocked_stopping_corridor'); self.send_wheels(0.,0.,1.); return
        self.send_wheels(v,w,1. if v<0.005 else 0.)
        self.sent_speed=v

    @locked
    def stop(self,reason):
        if self.active_goal is not None and not self.fault:
            self.fault=reason; self.target=(0.,0.); self.sent_speed=0.
            self.get_logger().warning('Navigation stopped: '+reason)
            if self.child_goal is not None: self.child_goal.cancel_goal_async()

    @locked
    def accept_goal(self,request):
        reason=self.ready_reason()
        if reason or self.goal_pending or self.active_goal is not None or not self.nav_client.server_is_ready():
            self.get_logger().warning('Goal rejected: '+(reason or 'navigation busy/unavailable')); return GoalResponse.REJECT
        p=request.pose.pose.position; q=request.pose.pose.orientation
        if request.pose.header.frame_id!='map' or not np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all() or abs(np.linalg.norm([q.x,q.y,q.z,q.w])-1)>0.05:
            return GoalResponse.REJECT
        ij=self.terrain.index([p.x,p.y])
        if np.any(ij<0) or np.any(ij>=self.terrain.n) or self.terrain.grid[ij[1],ij[0]]!=0:
            self.get_logger().warning('Goal must be on observed traversable ground'); return GoalResponse.REJECT
        self.goal_pending=True
        return GoalResponse.ACCEPT

    def cancel_goal(self,handle):
        self.stop('goal_cancelled'); return CancelResponse.ACCEPT

    async def await_nav2(self,future,timeout):
        # rclpy futures (not asyncio): guarantee a crashed child cannot leave the
        # public action busy forever after the wheel watchdog has stopped it.
        completion=rclpy.task.Future(); deadline=time.monotonic()+timeout
        def check():
            nonlocal deadline
            if completion.done(): return
            if future.done():
                try: completion.set_result(future.result())
                except Exception as error: completion.set_exception(error)
            else:
                if self.fault: deadline=min(deadline,time.monotonic()+2.)
                if time.monotonic()>=deadline: completion.set_exception(TimeoutError('Nav2 response timed out'))
        timer=self.create_timer(.1,check,callback_group=self.group)
        try: return await completion
        finally: self.destroy_timer(timer)

    async def execute(self,handle):
        self.active_goal=handle; self.child_goal=None; self.fault=''; self.command_time=0.; self.started_at=time.monotonic()
        self.target=(0.,0.); result=NavigateToPose.Result()
        try:
            child=await self.await_nav2(self.nav_client.send_goal_async(handle.request,feedback_callback=lambda m:handle.publish_feedback(m.feedback) if handle.is_active else None),5.)
            self.child_goal=child
            if not child.accepted: self.fault='nav2_rejected_goal'
            else:
                if self.fault or handle.is_cancel_requested: child.cancel_goal_async()
                response=await self.await_nav2(child.get_result_async(),180.); result=response.result
                if response.status!=GoalStatus.STATUS_SUCCEEDED and not self.fault: self.fault='nav2_failed'
            self.send_wheels(0.,0.,1.)
            if handle.is_cancel_requested: handle.canceled()
            elif self.fault:
                handle.abort()
                if hasattr(result,'error_msg'): result.error_msg=self.fault
                if hasattr(result,'error_code'): result.error_code=999
            else: handle.succeed()
        except Exception as error:
            self.stop('nav2_response_failure: '+str(error))
            if handle.is_active:
                if handle.is_cancel_requested: handle.canceled()
                else: handle.abort()
            result.error_code=999
            if hasattr(result,'error_msg'): result.error_msg=self.fault
        finally:
            self.send_wheels(0.,0.,1.); self.send_authority(self.lease.release_action())
            self.active_goal=None; self.goal_pending=False; self.child_goal=None; self.command_time=0.; self.sent_speed=0.
        return result

    @locked
    def reset_service(self,request,response):
        if self.active_goal is not None or abs(self.speed)>0.02:
            response.success=False; response.message='Cancel navigation and stop before resetting'; return response
        self.reset_state(); self.fault=''
        response.success=True; response.message='Map and estimator reset; hold still for initialization'; return response

    def report(self):
        reason=self.ready_reason()
        state=dict(ready=not bool(reason),reason=reason,active=self.active_goal is not None,fault=self.fault,
                   estimator=self.est.quality,position_sigma=self.est.sigma,lidar_rmse=self.est.rmse,
                   lidar_rank=self.est.observed_rank,cloud_duration=self.cloud_duration,cloud_interval=self.cloud_interval,mount_age=self.mount_age,speed=self.speed,observed_free=int((self.terrain.grid==0).sum()))
        self.status_pub.publish(String(data=json.dumps(state)))
        marker=Marker(); marker.header=Header(stamp=self.get_clock().now().to_msg(),frame_id='map')
        marker.ns='status'; marker.id=0; marker.type=Marker.TEXT_VIEW_FACING; marker.action=Marker.ADD
        marker.pose.position.x=float(self.est.pose[0,3]); marker.pose.position.y=float(self.est.pose[1,3]); marker.pose.position.z=5.
        marker.pose.orientation.w=1.; marker.scale.z=0.5; marker.color.a=1.; marker.color.g=1. if not reason else 0.3; marker.color.r=0.3 if not reason else 1.
        marker.text=self.fault or reason or ('Navigating' if self.active_goal else 'Ready: set Nav2 Goal')
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args); node=Rover(); executor=MultiThreadedExecutor(num_threads=3); executor.add_node(node)
    try: executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        if rclpy.ok():
            node.send_wheels(0.,0.,1.); node.send_authority(node.lease.release_action())
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
