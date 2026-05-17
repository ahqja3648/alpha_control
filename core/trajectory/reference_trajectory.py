# -*- coding: utf-8 -*-
import math
from typing import List, Tuple
from core.common.geometry import angle_diff, clamp, cumulative_distance_xy, normalize_angle
from core.common.types import DensePathPoint, PlannerConfig, RefTrajectoryPoint

class ReferenceTrajectoryGenerator:
    def __init__(self, cfg: PlannerConfig):
        self.cfg = cfg

    def split_by_gear(self, raw: List[DensePathPoint]):
        if len(raw) < 2:
            return []
        segments = []
        cur = [raw[0]]
        prev = raw[1].direction
        for p in raw[1:]:
            if p.direction != prev and len(cur) > 1:
                segments.append(cur)
                cur = [cur[-1]]
            cur.append(p)
            prev = p.direction
        if len(cur) > 1:
            segments.append(cur)
        return segments

    def resample_segment(self, segment: List[DensePathPoint], ds: float) -> List[Tuple[float, float, int, float]]:
        if len(segment) < 2:
            return []
        gear = segment[1].direction
        xy = [(p.x, p.y) for p in segment]
        ss = cumulative_distance_xy(xy)
        total = ss[-1]
        if total < 1e-6:
            return []
        out = []
        n = max(2, int(math.ceil(total / ds)) + 1)
        j = 0
        for i in range(n):
            target_s = min(total, float(i) * total / float(n - 1))
            while j + 1 < len(ss) and ss[j + 1] < target_s:
                j += 1
            if j + 1 >= len(ss):
                x, y = xy[-1]; steer = segment[-1].steer
            else:
                s0, s1 = ss[j], ss[j + 1]
                r = 0.0 if abs(s1 - s0) < 1e-9 else (target_s - s0) / (s1 - s0)
                x0, y0 = xy[j]; x1, y1 = xy[j + 1]
                x = x0 + (x1 - x0) * r; y = y0 + (y1 - y0) * r
                steer = segment[j + 1].steer
            out.append((x, y, gear, steer))
        return out

    def generate(self, raw: List[DensePathPoint]):
        ref = []
        global_t = 0.0; global_s = 0.0
        for seg in self.split_by_gear(raw):
            samples = self.resample_segment(seg, self.cfg.ref_ds)
            if len(samples) < 2:
                continue
            xy = [(x, y) for x, y, _, _ in samples]
            sl = cumulative_distance_xy(xy)
            S = max(sl[-1], 1e-6)
            T = max(self.cfg.min_segment_time, 1.65 * S / max(self.cfg.max_ref_speed, 1e-3))
            yaws = []
            for i in range(len(samples)):
                if i < len(samples) - 1:
                    dx = samples[i + 1][0] - samples[i][0]; dy = samples[i + 1][1] - samples[i][1]
                else:
                    dx = samples[i][0] - samples[i - 1][0]; dy = samples[i][1] - samples[i - 1][1]
                path_heading = math.atan2(dy, dx)
                gear = samples[i][2]
                yaw = path_heading if gear >= 0 else normalize_angle(path_heading + math.pi)
                yaws.append(normalize_angle(yaw))
            kappas = []; steers = []
            for i in range(len(samples)):
                if i == 0:
                    dyaw = angle_diff(yaws[1], yaws[0]); ds = max(sl[1] - sl[0], 1e-6)
                elif i == len(samples) - 1:
                    dyaw = angle_diff(yaws[-1], yaws[-2]); ds = max(sl[-1] - sl[-2], 1e-6)
                else:
                    dyaw = angle_diff(yaws[i + 1], yaws[i - 1]); ds = max(sl[i + 1] - sl[i - 1], 1e-6)
                kappa = dyaw / ds
                steer = clamp(math.atan(self.cfg.wheel_base * kappa), -self.cfg.max_steer, self.cfg.max_steer)
                kappas.append(kappa); steers.append(steer)
            for i, (x, y, gear, _) in enumerate(samples):
                r = clamp(sl[i] / S, 0.0, 1.0)
                local_t = r * T
                vmag = (S / T) * (6.0 * r - 6.0 * r * r)
                amag = (S / (T * T)) * (6.0 - 12.0 * r)
                if ref and i == 0:
                    continue
                ref.append(RefTrajectoryPoint(global_t + local_t, x, y, yaws[i], global_s + sl[i], float(gear) * vmag, float(gear) * amag, kappas[i], steers[i], gear))
            global_t += T; global_s += S
        return ref
