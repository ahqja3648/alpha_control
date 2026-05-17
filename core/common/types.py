# -*- coding: utf-8 -*-
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple
from .geometry import rotate_point

@dataclass(frozen=True)
class RectObstacle:
    # Obstacles are center-origin boxes, as common ObjectStatus/bounding-box outputs are center-based.
    x: float
    y: float
    yaw: float
    length: float
    width: float
    name: str = "obstacle"
    def polygon(self, padding: float = 0.0) -> List[Tuple[float, float]]:
        hl = 0.5 * self.length + padding
        hw = 0.5 * self.width + padding
        pts = []
        for lx, ly in [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]:
            rx, ry = rotate_point(lx, ly, self.yaw)
            pts.append((self.x + rx, self.y + ry))
        return pts

@dataclass
class SearchNode:
    # x,y,yaw are rear-axle-center pose.
    x: float
    y: float
    yaw: float
    g: float = 0.0
    h: float = 0.0
    parent: Optional[int] = None
    direction: int = 1
    steer: float = 0.0
    index: Optional[Tuple[int, int, int]] = None
    traj: Optional[List[Tuple[float, float, float]]] = None
    @property
    def f(self) -> float:
        return self.g + self.h

@dataclass
class DensePathPoint:
    # x,y,yaw are rear-axle-center pose.
    x: float
    y: float
    yaw: float
    direction: int
    steer: float

@dataclass
class RefTrajectoryPoint:
    # x,y,yaw are rear-axle-center reference pose.
    t: float
    x: float
    y: float
    yaw: float
    s: float
    v: float
    a: float
    curvature: float
    steer: float
    gear: int

@dataclass
class PlannerConfig:
    min_x: float = -3.0
    max_x: float = 13.0
    min_y: float = -5.7
    max_y: float = 5.8

    # Given vehicle dimensions.
    vehicle_length: float = 4.430
    vehicle_width: float = 1.825

    # Rear-axle-origin geometry.
    # x,y,yaw state is the center of the rear wheel axis.
    wheel_base: float = 2.700
    rear_overhang: float = 0.850
    max_steer_deg: float = 35.0
    collision_padding: float = 0.03
    drivable_padding: float = 0.02

    # Parking slot dimensions.
    parking_slot_width: float = 2.200
    parking_slot_length: float = 4.400

    # Hybrid A*.
    xy_resolution: float = 0.30
    yaw_resolution_deg: float = 8.0
    primitive_length: float = 0.55
    integration_step: float = 0.08
    steer_sample_count: int = 7
    max_iterations: int = 120000
    goal_xy_tolerance: float = 0.35
    goal_yaw_tolerance_deg: float = 10.0

    # Costs.
    reverse_penalty: float = 1.02
    gear_switch_penalty: float = 1.45
    steer_penalty: float = 0.16
    steer_change_penalty: float = 0.30
    distance_heuristic_weight: float = 1.10
    yaw_heuristic_weight: float = 0.60

    # Reference trajectory.
    ref_ds: float = 0.12
    max_ref_speed: float = 0.70
    min_segment_time: float = 2.0

    @property
    def front_overhang(self) -> float:
        return max(0.0, self.vehicle_length - self.wheel_base - self.rear_overhang)

    @property
    def rear_axle_to_front(self) -> float:
        return self.wheel_base + self.front_overhang

    @property
    def rear_axle_to_rear(self) -> float:
        return self.rear_overhang

    @property
    def body_center_from_rear_axle(self) -> float:
        return 0.5 * (self.rear_axle_to_front - self.rear_axle_to_rear)

    @property
    def max_steer(self) -> float:
        return math.radians(self.max_steer_deg)

    @property
    def yaw_resolution(self) -> float:
        return math.radians(self.yaw_resolution_deg)

    @property
    def goal_yaw_tolerance(self) -> float:
        return math.radians(self.goal_yaw_tolerance_deg)
