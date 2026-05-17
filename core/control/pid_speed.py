# -*- coding: utf-8 -*-
from dataclasses import dataclass
from core.common.geometry import clamp

@dataclass
class PIDResult:
    accel_cmd: float
    brake_cmd: float
    raw_output: float
    error: float

class PIDSpeedController:
    def __init__(self,kp=0.9,ki=0.04,kd=0.02,max_accel_cmd=1.0,max_brake_cmd=1.0,integral_limit=2.0,deadband=0.02):
        self.kp=float(kp); self.ki=float(ki); self.kd=float(kd); self.max_accel_cmd=float(max_accel_cmd); self.max_brake_cmd=float(max_brake_cmd); self.integral_limit=float(integral_limit); self.deadband=float(deadband); self.integral=0.0; self.prev_error=0.0; self.has_prev=False
    def reset(self): self.integral=0.0; self.prev_error=0.0; self.has_prev=False
    def step(self,target_speed,current_speed,dt):
        dt=max(float(dt),1e-3); err=float(target_speed)-float(current_speed)
        if abs(err)<self.deadband: err=0.0
        self.integral=clamp(self.integral+err*dt,-self.integral_limit,self.integral_limit)
        der=(err-self.prev_error)/dt if self.has_prev else 0.0
        self.has_prev=True; self.prev_error=err
        raw=self.kp*err+self.ki*self.integral+self.kd*der
        return PIDResult(clamp(raw,0.0,self.max_accel_cmd) if raw>=0 else 0.0, 0.0 if raw>=0 else clamp(-raw,0.0,self.max_brake_cmd), raw, err)
