# -*- coding: utf-8 -*-
import math
from core.common.types import DensePathPoint, RectObstacle, RefTrajectoryPoint
from core.scenario.rear_parking import DEFAULT_SLOT_CENTERS, TARGET_SLOT_CENTER, TARGET_YAW, rear_axle_to_body_center

class RvizVisualizer:
    def __init__(self, planner, start, goal):
        import rospy
        from nav_msgs.msg import Path
        from visualization_msgs.msg import MarkerArray
        self.rospy = rospy
        self.planner = planner
        self.cfg = planner.cfg
        self.start = start
        self.goal = goal
        self.static_pub = rospy.Publisher('/hybrid_astar/static_markers', MarkerArray, queue_size=1, latch=True)
        self.search_pub = rospy.Publisher('/hybrid_astar/search_markers', MarkerArray, queue_size=1)
        self.partial_pub = rospy.Publisher('/hybrid_astar/partial_path', Path, queue_size=1)
        self.raw_pub = rospy.Publisher('/hybrid_astar/path_raw', Path, queue_size=1, latch=True)
        self.ref_pub = rospy.Publisher('/hybrid_astar/path_ref', Path, queue_size=1, latch=True)
        self.final_pub = rospy.Publisher('/hybrid_astar/final_markers', MarkerArray, queue_size=1, latch=True)
        self.ref_marker_pub = rospy.Publisher('/hybrid_astar/ref_markers', MarkerArray, queue_size=1, latch=True)

    def q_from_yaw(self, yaw):
        import tf.transformations
        return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

    def header(self):
        from std_msgs.msg import Header
        h = Header(); h.stamp = self.rospy.Time.now(); h.frame_id = 'map'; return h

    def point(self, x, y, z=0.0):
        from geometry_msgs.msg import Point
        p = Point(); p.x=x; p.y=y; p.z=z; return p

    def color(self, r,g,b,a):
        from std_msgs.msg import ColorRGBA
        c = ColorRGBA(); c.r=r; c.g=g; c.b=b; c.a=a; return c

    def marker(self, mid, ns, t):
        from visualization_msgs.msg import Marker
        m = Marker(); m.header=self.header(); m.ns=ns; m.id=mid; m.type=t; m.action=Marker.ADD; m.pose.orientation.w=1.0; return m

    def delete_all(self):
        from visualization_msgs.msg import Marker
        m = Marker(); m.action = Marker.DELETEALL; return m

    def line_strip(self, mid, ns, pts, rgba, width=0.04, z=0.04):
        from visualization_msgs.msg import Marker
        m = self.marker(mid, ns, Marker.LINE_STRIP); m.scale.x=width; m.color=self.color(*rgba)
        for x,y in pts: m.points.append(self.point(x,y,z))
        return m

    def line_list(self, mid, ns, segs, rgba, width=0.03, z=0.1):
        from visualization_msgs.msg import Marker
        m = self.marker(mid, ns, Marker.LINE_LIST); m.scale.x=width; m.color=self.color(*rgba)
        for x1,y1,x2,y2 in segs:
            m.points.append(self.point(x1,y1,z)); m.points.append(self.point(x2,y2,z))
        return m

    def points_marker(self, mid, ns, pts, rgba, scale=0.06, z=0.05):
        from visualization_msgs.msg import Marker
        m = self.marker(mid, ns, Marker.POINTS); m.scale.x=scale; m.scale.y=scale; m.color=self.color(*rgba)
        for x,y in pts: m.points.append(self.point(x,y,z))
        return m

    def cube_center(self, mid, ns, cx, cy, yaw, length, width, height, rgba):
        from visualization_msgs.msg import Marker
        m = self.marker(mid, ns, Marker.CUBE)
        m.pose.position.x=cx; m.pose.position.y=cy; m.pose.position.z=height*0.5
        q=self.q_from_yaw(yaw); m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q[0],q[1],q[2],q[3]
        m.scale.x=length; m.scale.y=width; m.scale.z=height; m.color=self.color(*rgba)
        return m

    def cube_rear_axle(self, mid, ns, rx, ry, yaw, rgba):
        cx, cy = self.planner.vehicle_body_center_from_rear_axle(rx, ry, yaw)
        return self.cube_center(mid, ns, cx, cy, yaw, self.cfg.vehicle_length, self.cfg.vehicle_width, 0.18, rgba)

    def arrow(self, mid, ns, x, y, yaw, rgba, length=0.7):
        from visualization_msgs.msg import Marker
        m = self.marker(mid, ns, Marker.ARROW)
        m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=0.18
        q=self.q_from_yaw(yaw); m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
        m.scale.x=length; m.scale.y=0.12; m.scale.z=0.12; m.color=self.color(*rgba)
        return m

    def text(self, mid, ns, text, x, y, z, scale, rgba):
        from visualization_msgs.msg import Marker
        m=self.marker(mid, ns, Marker.TEXT_VIEW_FACING); m.pose.position.x=x; m.pose.position.y=y; m.pose.position.z=z; m.scale.z=scale; m.color=self.color(*rgba); m.text=text; return m

    def path_msg_dense(self, dense):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        msg=Path(); msg.header=self.header()
        for p in dense:
            ps=PoseStamped(); ps.header=msg.header; ps.pose.position.x=p.x; ps.pose.position.y=p.y
            q=self.q_from_yaw(p.yaw); ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
            msg.poses.append(ps)
        return msg

    def path_msg_ref(self, ref):
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        msg=Path(); msg.header=self.header()
        for p in ref:
            ps=PoseStamped(); ps.header=msg.header; ps.pose.position.x=p.x; ps.pose.position.y=p.y
            q=self.q_from_yaw(p.yaw); ps.pose.orientation.x,ps.pose.orientation.y,ps.pose.orientation.z,ps.pose.orientation.w=q
            msg.poses.append(ps)
        return msg

    def publish_static(self):
        from visualization_msgs.msg import MarkerArray
        ma=MarkerArray(); ma.markers.append(self.delete_all()); mid=1
        boundary=[(self.cfg.min_x,self.cfg.min_y),(self.cfg.max_x,self.cfg.min_y),(self.cfg.max_x,self.cfg.max_y),(self.cfg.min_x,self.cfg.max_y),(self.cfg.min_x,self.cfg.min_y)]
        ma.markers.append(self.line_strip(mid,'map_boundary',boundary,(0.8,0.8,0.8,1),0.04,0.02)); mid+=1
        outer=self.planner.drivable_area.outer_polygon+[self.planner.drivable_area.outer_polygon[0]]
        ma.markers.append(self.line_strip(mid,'manual_drivable_area',outer,(0.1,1.0,0.25,1),0.08,0.08)); mid+=1
        for name,poly in self.planner.drivable_area.forbidden_polygons:
            ma.markers.append(self.line_strip(mid,'forbidden_area',poly+[poly[0]],(1,0.15,0.05,0.55),0.035,0.08)); mid+=1
        for sx in DEFAULT_SLOT_CENTERS:
            obs=RectObstacle(sx,TARGET_SLOT_CENTER[1],TARGET_YAW,self.cfg.parking_slot_length,self.cfg.parking_slot_width,'slot')
            poly=obs.polygon(0.0)+[obs.polygon(0.0)[0]]
            color=(1,0.85,0.05,1) if abs(sx-TARGET_SLOT_CENTER[0])<0.01 else (0.75,0.75,0.75,0.35)
            ma.markers.append(self.line_strip(mid,'parking_slots',poly,color,0.035,0.05)); mid+=1
        for obs in self.planner.obstacles:
            is_wall='wall' in obs.name or 'boundary' in obs.name
            col=(0.55,0.55,0.55,0.85) if is_wall else (0.9,0.05,0.05,0.85)
            ma.markers.append(self.cube_center(mid,'obstacles',obs.x,obs.y,obs.yaw,obs.length,obs.width,0.22,col)); mid+=1
        ma.markers.append(self.cube_rear_axle(mid,'start_vehicle',*self.start,(0.05,1,0.2,0.40))); mid+=1
        ma.markers.append(self.arrow(mid,'start_rear_axle_arrow',*self.start,(0.05,1,0.2,1),0.85)); mid+=1
        ma.markers.append(self.cube_rear_axle(mid,'goal_vehicle',*self.goal,(1,0.85,0.05,0.40))); mid+=1
        ma.markers.append(self.arrow(mid,'goal_rear_axle_arrow',*self.goal,(1,0.85,0.05,1),0.85)); mid+=1
        ma.markers.append(self.text(mid,'legend','rear-axle origin model\nvehicle: 4.430m x 1.825m\nslot: 2.2m x 4.4m\n/path_raw: Hybrid A* rear-axle path\n/path_ref: reference trajectory',-0.5,5.1,0.7,0.25,(1,1,1,1))); mid+=1
        self.static_pub.publish(ma)

    def publish_search_step(self,current,current_id,open_heap,nodes,closed,visual_edges,latest_edges,rejected,step_count):
        from visualization_msgs.msg import MarkerArray
        ma=MarkerArray(); ma.markers.append(self.points_marker(1,'closed_nodes',[(n.x,n.y) for n in list(closed.values())[-2000:]],(0.55,0.55,0.55,0.55),0.045,0.05))
        ma.markers.append(self.points_marker(2,'open_nodes',[(nodes[i].x,nodes[i].y) for _,_,i in open_heap[-1500:]],(0.1,0.75,1,0.75),0.05,0.06))
        ma.markers.append(self.line_list(3,'expansion_edges',visual_edges[-3500:],(1,0.5,0,0.25),0.015,0.04))
        ma.markers.append(self.line_list(4,'latest_edges',latest_edges,(1,0.85,0,0.9),0.035,0.07))
        ma.markers.append(self.points_marker(5,'rejected',rejected,(1,0,0,0.8),0.08,0.08))
        ma.markers.append(self.arrow(6,'current_rear_axle',current.x,current.y,current.yaw,(1,1,0,1),0.7))
        ma.markers.append(self.cube_rear_axle(7,'current_vehicle',current.x,current.y,current.yaw,(1,1,0,0.25)))
        self.search_pub.publish(ma)
        try:
            chain=self.planner.reconstruct_chain(nodes,current_id); dense=self.planner.chain_to_dense_path(chain); self.partial_pub.publish(self.path_msg_dense(dense))
        except Exception:
            pass

    def publish_raw_and_ref(self, raw, ref, csv_path):
        from visualization_msgs.msg import MarkerArray
        self.raw_pub.publish(self.path_msg_dense(raw)); self.ref_pub.publish(self.path_msg_ref(ref))
        ma=MarkerArray(); ma.markers.append(self.delete_all()); mid=1
        fwd=[]; rev=[]
        for a,b in zip(raw[:-1], raw[1:]):
            (fwd if b.direction>=0 else rev).append((a.x,a.y,b.x,b.y))
        ma.markers.append(self.line_list(mid,'raw_forward',fwd,(0.05,0.35,1,1),0.08,0.18)); mid+=1
        ma.markers.append(self.line_list(mid,'raw_reverse',rev,(1,0,0.85,1),0.08,0.20)); mid+=1
        ma.markers.append(self.line_list(mid,'ref_traj',[(a.x,a.y,b.x,b.y) for a,b in zip(ref[:-1],ref[1:])],(1,1,0.05,1),0.04,0.30)); mid+=1
        for i in range(0,len(ref),max(1,int(len(ref)/16))):
            p=ref[i]; ma.markers.append(self.arrow(mid,'ref_arrows',p.x,p.y,p.yaw,(1,1,0.05,1) if p.gear>=0 else (1,0.55,0.05,1),0.45)); mid+=1
        if raw:
            p=raw[-1]; ma.markers.append(self.cube_rear_axle(mid,'final_vehicle',p.x,p.y,p.yaw,(0.1,1,0.3,0.45))); mid+=1
        ma.markers.append(self.text(mid,'result','planning succeeded\nrear axle path points: %d\nref points: %d\nCSV: %s'%(len(raw),len(ref),csv_path),8.5,5.1,0.7,0.22,(0.2,1,0.2,1))); mid+=1
        self.final_pub.publish(ma)
        self.ref_marker_pub.publish(ma)
