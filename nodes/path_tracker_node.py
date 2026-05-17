#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, os, sys
PKG_ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
if PKG_ROOT not in sys.path: sys.path.insert(0,PKG_ROOT)
import rospy
import tf.transformations
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker, MarkerArray
from alpha_control.msg import RefTrajectory, TrackingCommand
from core.common.geometry import normalize_angle
from core.common.types import RefTrajectoryPoint
from core.control.pure_pursuit import PurePursuitController
from core.control.pid_speed import PIDSpeedController
from core.control.reference_tracker import ReferenceTracker

def yaw_from_pose(msg):
    q=msg.pose.orientation
    return normalize_angle(tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])[2])

def q_from_yaw(yaw): return tf.transformations.quaternion_from_euler(0,0,yaw)

class PathTrackerNode:
    def __init__(self):
        rospy.init_node('alpha_control_path_tracker', anonymous=False)
        self.ref=[]; self.pose=None; self.yaw=0.0; self.speed=0.0; self.last_pose_time=None; self.last_speed_time=None; self.last_update=rospy.Time.now()
        pp=PurePursuitController(rospy.get_param('~wheel_base',2.70),rospy.get_param('~max_steer_deg',35.0),rospy.get_param('~min_lookahead',0.75),rospy.get_param('~max_lookahead',2.20),rospy.get_param('~lookahead_gain',0.80))
        pid=PIDSpeedController(rospy.get_param('~speed_kp',0.9),rospy.get_param('~speed_ki',0.04),rospy.get_param('~speed_kd',0.02))
        self.tracker=ReferenceTracker(pp,pid,rospy.get_param('~goal_dist_tolerance',0.30),rospy.get_param('~goal_speed_tolerance',0.08))
        self.rate_hz=rospy.get_param('~control_rate',20.0); self.pose_timeout=rospy.get_param('~pose_timeout',1.0)
        rospy.Subscriber('/alpha_control/ref_trajectory',RefTrajectory,self.ref_cb,queue_size=1)
        rospy.Subscriber('/alpha_control/current_pose',PoseStamped,self.pose_cb,queue_size=1)
        rospy.Subscriber('/alpha_control/current_speed',Float64,self.speed_cb,queue_size=1)
        self.cmd_pub=rospy.Publisher('/alpha_control/tracking_cmd',TrackingCommand,queue_size=1)
        self.target_pub=rospy.Publisher('/alpha_control/tracking_target_pose',PoseStamped,queue_size=1)
        self.marker_pub=rospy.Publisher('/alpha_control/tracking_markers',MarkerArray,queue_size=1)
        rospy.loginfo('Path tracker initialized. Current pose and reference pose are rear-axle-center poses.')
    def ref_cb(self,msg):
        self.ref=[RefTrajectoryPoint(p.t,p.x,p.y,p.yaw,p.s,p.v,p.a,p.curvature,p.steer,p.gear) for p in msg.points]
        self.tracker.reset(); rospy.loginfo('Reference trajectory received: %d points',len(self.ref))
    def pose_cb(self,msg):
        now=msg.header.stamp if msg.header.stamp!=rospy.Time(0) else rospy.Time.now()
        yaw=yaw_from_pose(msg)
        if self.pose is not None and (self.last_speed_time is None or (rospy.Time.now()-self.last_speed_time).to_sec()>0.5):
            dt=(now-self.last_pose_time).to_sec() if self.last_pose_time is not None else 0.0
            if dt>1e-3:
                dx=msg.pose.position.x-self.pose.pose.position.x; dy=msg.pose.position.y-self.pose.pose.position.y
                mag=math.hypot(dx,dy)/dt; signed=dx*math.cos(yaw)+dy*math.sin(yaw)
                self.speed=mag if signed>=0 else -mag
        self.pose=msg; self.yaw=yaw; self.last_pose_time=now
    def speed_cb(self,msg):
        self.speed=msg.data; self.last_speed_time=rospy.Time.now()
    def publish_target(self,res):
        ps=PoseStamped(); ps.header.stamp=rospy.Time.now(); ps.header.frame_id='map'; ps.pose.position.x=res.target_x; ps.pose.position.y=res.target_y
        q=q_from_yaw(res.target_yaw); ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
        self.target_pub.publish(ps)
    def publish_markers(self,res):
        ma=MarkerArray(); delete=Marker(); delete.action=Marker.DELETEALL; ma.markers.append(delete)
        m=Marker(); m.header.stamp=rospy.Time.now(); m.header.frame_id='map'; m.ns='tracking_target'; m.id=1; m.type=Marker.SPHERE; m.action=Marker.ADD; m.pose.position.x=res.target_x; m.pose.position.y=res.target_y; m.pose.position.z=0.25; m.pose.orientation.w=1; m.scale.x=m.scale.y=m.scale.z=0.28; m.color.r=m.color.g=1; m.color.b=0; m.color.a=1; ma.markers.append(m)
        if self.pose is not None:
            line=Marker(); line.header.stamp=rospy.Time.now(); line.header.frame_id='map'; line.ns='lookahead_line'; line.id=2; line.type=Marker.LINE_STRIP; line.action=Marker.ADD; line.pose.orientation.w=1; line.scale.x=0.045; line.color.r=line.color.g=1; line.color.a=0.9
            p0=Point(); p0.x=self.pose.pose.position.x; p0.y=self.pose.pose.position.y; p0.z=0.2
            p1=Point(); p1.x=res.target_x; p1.y=res.target_y; p1.z=0.2; line.points=[p0,p1]; ma.markers.append(line)
        self.marker_pub.publish(ma)
    def spin(self):
        rate=rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now=rospy.Time.now(); dt=(now-self.last_update).to_sec(); self.last_update=now
            if not self.ref or self.pose is None:
                rate.sleep(); continue
            if self.last_pose_time and (now-self.last_pose_time).to_sec()>self.pose_timeout:
                rospy.logwarn_throttle(1.0,'No recent /alpha_control/current_pose'); rate.sleep(); continue
            x=self.pose.pose.position.x; y=self.pose.pose.position.y
            res=self.tracker.update(self.ref,x,y,self.yaw,self.speed,dt)
            if res is None:
                rate.sleep(); continue
            msg=TrackingCommand(); msg.header.stamp=now; msg.header.frame_id='map'; msg.target_speed=res.target_speed; msg.current_speed=res.current_speed; msg.accel_cmd=res.accel_cmd; msg.brake_cmd=res.brake_cmd; msg.steering_rad=res.steering_rad; msg.gear=res.gear; msg.nearest_index=res.nearest_index; msg.target_index=res.target_index; msg.lookahead_distance=res.lookahead_distance; msg.lateral_error=res.lateral_error; msg.heading_error=res.heading_error; msg.goal_reached=res.goal_reached
            self.cmd_pub.publish(msg); self.publish_target(res); self.publish_markers(res)
            rate.sleep()
if __name__=='__main__':
    try: PathTrackerNode().spin()
    except rospy.ROSInterruptException: pass
