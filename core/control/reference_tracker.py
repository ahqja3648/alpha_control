# -*- coding: utf-8 -*-
from dataclasses import dataclass
from typing import List, Optional
from core.common.types import RefTrajectoryPoint
from .pid_speed import PIDSpeedController
from .pure_pursuit import PurePursuitController

@dataclass
class TrackingResult:
    target_speed: float; current_speed: float; accel_cmd: float; brake_cmd: float; steering_rad: float; gear: int; nearest_index: int; target_index: int; lookahead_distance: float; lateral_error: float; heading_error: float; target_x: float; target_y: float; target_yaw: float; goal_reached: bool

class ReferenceTracker:
    def __init__(self, pure_pursuit: PurePursuitController, speed_pid: PIDSpeedController, goal_dist_tolerance=0.30, goal_speed_tolerance=0.08):
        self.pp=pure_pursuit; self.pid=speed_pid; self.goal_dist_tolerance=float(goal_dist_tolerance); self.goal_speed_tolerance=float(goal_speed_tolerance); self.nearest_index=0
    def reset(self): self.nearest_index=0; self.pid.reset()
    def update(self, ref: List[RefTrajectoryPoint], x, y, yaw, current_speed, dt) -> Optional[TrackingResult]:
        if not ref: return None
        pp=self.pp.compute(ref,x,y,yaw,current_speed,self.nearest_index)
        if pp is None: return None
        self.nearest_index=pp.nearest_index
        target_speed=ref[pp.nearest_index].v
        pid=self.pid.step(target_speed,current_speed,dt)
        last=ref[-1]
        goal_reached=((last.x-x)**2+(last.y-y)**2)**0.5 <= self.goal_dist_tolerance and abs(current_speed)<=self.goal_speed_tolerance
        if goal_reached:
            target_speed=0.0; pid.accel_cmd=0.0; pid.brake_cmd=1.0
        return TrackingResult(target_speed,current_speed,pid.accel_cmd,pid.brake_cmd,pp.steering_rad,pp.gear,pp.nearest_index,pp.target_index,pp.lookahead_distance,pp.lateral_error,pp.heading_error,pp.target_x,pp.target_y,pp.target_yaw,goal_reached)
