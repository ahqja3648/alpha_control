# -*- coding: utf-8 -*-
import csv, math

def write_ref_csv(path, ref):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t','x_rear_axle','y_rear_axle','yaw_rad','yaw_deg','s','v','a','curvature','steer_rad','steer_deg','gear'])
        for p in ref:
            w.writerow(['%.4f'%p.t,'%.4f'%p.x,'%.4f'%p.y,'%.6f'%p.yaw,'%.3f'%math.degrees(p.yaw),'%.4f'%p.s,'%.4f'%p.v,'%.4f'%p.a,'%.6f'%p.curvature,'%.6f'%p.steer,'%.3f'%math.degrees(p.steer),'D' if p.gear>=0 else 'R'])
