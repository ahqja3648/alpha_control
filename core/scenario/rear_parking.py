# -*- coding: utf-8 -*-
import math
from typing import List, Tuple
from core.common.types import PlannerConfig, RectObstacle
from core.common.geometry import rotate_point
from core.planning.drivable_area import DrivableArea

DEFAULT_SLOT_CENTERS = [-0.50, 1.70, 3.90, 6.10, 8.30, 10.50]
TARGET_SLOT_CENTER = (6.10, 2.825)
TARGET_YAW = math.radians(-90.0)

def body_center_to_rear_axle(cx: float, cy: float, yaw: float, cfg: PlannerConfig):
    # body_center = rear_axle + body_center_from_rear_axle * heading
    ox, oy = rotate_point(cfg.body_center_from_rear_axle, 0.0, yaw)
    return cx - ox, cy - oy, yaw

def rear_axle_to_body_center(rx: float, ry: float, yaw: float, cfg: PlannerConfig):
    ox, oy = rotate_point(cfg.body_center_from_rear_axle, 0.0, yaw)
    return rx + ox, ry + oy

def make_rear_parking_scenario(cfg: PlannerConfig = None) -> Tuple[List[RectObstacle], DrivableArea, Tuple[float, float, float], Tuple[float, float, float]]:
    cfg = cfg or PlannerConfig()
    obstacles: List[RectObstacle] = []
    slot_y = TARGET_SLOT_CENTER[1]
    parked_yaw = TARGET_YAW
    target_x = TARGET_SLOT_CENTER[0]

    for i, sx in enumerate(DEFAULT_SLOT_CENTERS):
        if abs(sx - target_x) < 1e-6:
            continue
        obstacles.append(RectObstacle(sx, slot_y, parked_yaw, cfg.vehicle_length, cfg.vehicle_width, 'parked_car_%02d' % (i + 1)))

    # Rear curb and side boundaries. Obstacles remain center-origin boxes.
    obstacles.append(RectObstacle(6.10, 5.24, 0.0, 15.2, 0.12, 'rear_curb_wall'))
    obstacles.append(RectObstacle(-2.92, 0.0, 0.0, 0.12, 11.4, 'left_boundary'))
    obstacles.append(RectObstacle(12.92, 0.0, 0.0, 0.12, 11.4, 'right_boundary'))

    drivable_area = DrivableArea.default_rear_parking_area()

    # State origin is rear axle. Start is therefore rear-axle pose, not body center.
    start = (2.10, -2.10, math.radians(0.0))

    # Goal is computed from the parking slot body-center pose to a rear-axle pose.
    goal = body_center_to_rear_axle(TARGET_SLOT_CENTER[0], TARGET_SLOT_CENTER[1], TARGET_YAW, cfg)
    return obstacles, drivable_area, start, goal
