#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math, os, sys, traceback
PKG_ROOT=os.path.abspath(os.path.join(os.path.dirname(__file__),'..'))
if PKG_ROOT not in sys.path: sys.path.insert(0,PKG_ROOT)
from core.common.types import PlannerConfig
from core.planning.drivable_area import DrivableArea
from core.planning.hybrid_astar import HybridAStarPlanner
from core.scenario.rear_parking import make_rear_parking_scenario
from core.trajectory.reference_trajectory import ReferenceTrajectoryGenerator
from core.trajectory.csv_export import write_ref_csv
from core.visualization.rviz_visualizer import RvizVisualizer

def cfg_from_ros(rospy):
    return PlannerConfig(
        vehicle_length=rospy.get_param('~vehicle_length',4.430),
        vehicle_width=rospy.get_param('~vehicle_width',1.825),
        wheel_base=rospy.get_param('~wheel_base',2.700),
        rear_overhang=rospy.get_param('~rear_overhang',0.850),
        parking_slot_width=rospy.get_param('~parking_slot_width',2.200),
        parking_slot_length=rospy.get_param('~parking_slot_length',4.400),
        max_steer_deg=rospy.get_param('~max_steer_deg',35.0),
        xy_resolution=rospy.get_param('~xy_resolution',0.30),
        yaw_resolution_deg=rospy.get_param('~yaw_resolution_deg',8.0),
        primitive_length=rospy.get_param('~primitive_length',0.55),
        integration_step=rospy.get_param('~integration_step',0.08),
        steer_sample_count=rospy.get_param('~steer_sample_count',7),
        max_iterations=rospy.get_param('~max_iterations',120000),
        goal_xy_tolerance=rospy.get_param('~goal_xy_tolerance',0.35),
        goal_yaw_tolerance_deg=rospy.get_param('~goal_yaw_tolerance_deg',10.0),
        ref_ds=rospy.get_param('~ref_ds',0.12),
        max_ref_speed=rospy.get_param('~max_ref_speed',0.70),
    )

def main():
    import rospy
    from alpha_control.msg import RefTrajectory, RefTrajectoryPoint
    rospy.init_node('alpha_control_parking_planner', anonymous=False)
    pub=rospy.Publisher('/alpha_control/ref_trajectory', RefTrajectory, queue_size=1, latch=True)
    cfg=cfg_from_ros(rospy)
    obstacles, default_area, start, goal = make_rear_parking_scenario(cfg)
    yaml_path=rospy.get_param('~drivable_area_yaml','')
    area=DrivableArea.from_yaml(yaml_path) if yaml_path else default_area
    if rospy.get_param('~use_external_start_goal',False):
        sp=rospy.get_param('~start',[start[0],start[1],math.degrees(start[2])])
        gp=rospy.get_param('~goal',[goal[0],goal[1],math.degrees(goal[2])])
        start=(float(sp[0]),float(sp[1]),math.radians(float(sp[2])))
        goal=(float(gp[0]),float(gp[1]),math.radians(float(gp[2])))
    planner=HybridAStarPlanner(obstacles,area,cfg)
    viz=RvizVisualizer(planner,start,goal)
    rospy.sleep(0.8); viz.publish_static()
    every=int(rospy.get_param('~visualize_every_n',8)); dt=float(rospy.get_param('~animation_dt',0.02))
    def cb(*args):
        viz.publish_search_step(*args)
        if dt>0: rospy.sleep(dt)
    try:
        rospy.loginfo('vehicle origin: rear axle center; length=%.3fm width=%.3fm slot=%.3fm x %.3fm wheelbase=%.3fm rear_overhang=%.3fm', cfg.vehicle_length,cfg.vehicle_width,cfg.parking_slot_width,cfg.parking_slot_length,cfg.wheel_base,cfg.rear_overhang)
        rospy.loginfo('rear-axle start=%s goal=%s', start, goal)
        chain=planner.plan(start,goal,step_callback=cb,callback_every_n=every)
        if chain is None:
            rospy.logerr('Planning failed: %s', planner.last_stats); return
        raw=planner.chain_to_dense_path(chain)
        ref=ReferenceTrajectoryGenerator(cfg).generate(raw)
        csv_path=rospy.get_param('~ref_csv_path',os.path.expanduser('~/.ros/alpha_control_ref_traj.csv'))
        write_ref_csv(csv_path,ref)
        msg=RefTrajectory(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='map'
        for p in ref:
            q=RefTrajectoryPoint(); q.t=p.t; q.x=p.x; q.y=p.y; q.yaw=p.yaw; q.s=p.s; q.v=p.v; q.a=p.a; q.curvature=p.curvature; q.steer=p.steer; q.gear=p.gear; msg.points.append(q)
        pub.publish(msg)
        viz.publish_raw_and_ref(raw,ref,csv_path)
        rospy.loginfo('Planning succeeded. raw=%d ref=%d stats=%s', len(raw), len(ref), planner.last_stats)
        rospy.spin()
    except Exception as e:
        rospy.logerr('Exception: %s', e); rospy.logerr(traceback.format_exc())
if __name__=='__main__': main()
