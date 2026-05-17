# -*- coding: utf-8 -*-
import math
from dataclasses import dataclass
from typing import List, Optional
from core.common.geometry import angle_diff, clamp
from core.common.types import RefTrajectoryPoint

@dataclass
class PurePursuitResult:
    steering_rad: float
    nearest_index: int
    target_index: int
    lookahead_distance: float
    lateral_error: float
    heading_error: float
    target_x: float
    target_y: float
    target_yaw: float
    gear: int

class PurePursuitController:
    """Pure Pursuit using rear-axle-center current pose and rear-axle reference path."""
    def __init__(self, wheel_base=2.70, max_steer_deg=35.0, min_lookahead=0.75, max_lookahead=2.20, lookahead_gain=0.80):
        self.wheel_base=float(wheel_base); self.max_steer=math.radians(float(max_steer_deg)); self.min_lookahead=float(min_lookahead); self.max_lookahead=float(max_lookahead); self.lookahead_gain=float(lookahead_gain)

    def compute_lookahead(self, speed_abs):
        return clamp(self.min_lookahead + self.lookahead_gain * abs(float(speed_abs)), self.min_lookahead, self.max_lookahead)

    @staticmethod
    def nearest_index(ref, x, y, start_index=0):
        if not ref: return 0
        begin=max(0,min(int(start_index),len(ref)-1)-10); best=begin; best_d2=float('inf')
        for i in range(begin,len(ref)):
            dx=ref[i].x-x; dy=ref[i].y-y; d2=dx*dx+dy*dy
            if d2<best_d2: best_d2=d2; best=i
            if i>best+80: break
        return best

    @staticmethod
    def target_index_from_lookahead(ref, nearest, lookahead):
        base_s=ref[nearest].s
        for i in range(nearest,len(ref)):
            if abs(ref[i].s-base_s)>=lookahead: return i
        return len(ref)-1

    def compute(self, ref: List[RefTrajectoryPoint], x, y, yaw, speed, previous_nearest_index=0) -> Optional[PurePursuitResult]:
        if not ref: return None
        nearest=self.nearest_index(ref,x,y,previous_nearest_index)
        gear=1 if ref[nearest].gear>=0 else -1
        ld=self.compute_lookahead(abs(speed))
        target=self.target_index_from_lookahead(ref,nearest,ld)
        tp=ref[target]
        dx=tp.x-x; dy=tp.y-y
        c=math.cos(yaw); s=math.sin(yaw)
        local_x=c*dx+s*dy; local_y=-s*dx+c*dy
        if gear<0:
            local_x=-local_x; local_y=-local_y
        dist=max(math.hypot(local_x,local_y),1e-6)
        alpha=math.atan2(local_y,local_x)
        steer=math.atan2(2.0*self.wheel_base*math.sin(alpha),dist)
        steer=clamp(steer,-self.max_steer,self.max_steer)
        np=ref[nearest]
        ndx=x-np.x; ndy=y-np.y
        lateral=-math.sin(np.yaw)*ndx+math.cos(np.yaw)*ndy
        heading=angle_diff(yaw,np.yaw)
        return PurePursuitResult(steer,nearest,target,ld,lateral,heading,tp.x,tp.y,tp.yaw,gear)
