# -*- coding: utf-8 -*-
import math
from typing import List, Tuple

def normalize_angle(angle: float) -> float:
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def angle_diff(a: float, b: float) -> float:
    return normalize_angle(a - b)

def rotate_point(px: float, py: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw); s = math.sin(yaw)
    return c * px - s * py, s * px + c * py

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def polygon_axes(poly: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    axes = []
    for i in range(len(poly)):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % len(poly)]
        ex = x2 - x1; ey = y2 - y1
        ax = -ey; ay = ex
        n = math.hypot(ax, ay)
        if n > 1e-9:
            axes.append((ax / n, ay / n))
    return axes

def project_polygon(poly, axis):
    ax, ay = axis
    vals = [x * ax + y * ay for x, y in poly]
    return min(vals), max(vals)

def convex_polygons_intersect(poly_a, poly_b) -> bool:
    if len(poly_a) < 3 or len(poly_b) < 3:
        return False
    for axis in polygon_axes(poly_a) + polygon_axes(poly_b):
        a_min, a_max = project_polygon(poly_a, axis)
        b_min, b_max = project_polygon(poly_b, axis)
        if a_max < b_min or b_max < a_min:
            return False
    return True

def point_in_polygon(x: float, y: float, polygon) -> bool:
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        xi, yi = polygon[i]; xj, yj = polygon[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside

def sample_polygon_boundary(poly, samples_per_edge: int = 4):
    out = []
    for i in range(len(poly)):
        x1, y1 = poly[i]; x2, y2 = poly[(i + 1) % len(poly)]
        for k in range(samples_per_edge + 1):
            t = float(k) / float(samples_per_edge + 1)
            out.append((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t))
    return out

def cumulative_distance_xy(points):
    s = [0.0]
    for a, b in zip(points[:-1], points[1:]):
        s.append(s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    return s
