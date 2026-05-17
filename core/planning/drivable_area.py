# -*- coding: utf-8 -*-
import os
from typing import List, Optional, Tuple
from core.common.geometry import convex_polygons_intersect, point_in_polygon, sample_polygon_boundary

class DrivableArea:
    def __init__(self, outer_polygon: List[Tuple[float, float]], forbidden_polygons: Optional[List[Tuple[str, List[Tuple[float, float]]]]] = None, name: str = "manual_drivable_area"):
        self.name = name
        self.outer_polygon = [(float(x), float(y)) for x, y in outer_polygon]
        self.forbidden_polygons = []
        for name_i, poly in (forbidden_polygons or []):
            self.forbidden_polygons.append((name_i, [(float(x), float(y)) for x, y in poly]))

    @staticmethod
    def default_rear_parking_area():
        # T-shaped drivable area: aisle + only the target parking slot.
        # Because the vehicle length is 4.430 m and the slot length is about 4.400 m,
        # the front line is opened by a small tolerance so a 3 cm protrusion does not make the demo impossible.
        outer = [
            (-2.70, -4.20),
            (12.80, -4.20),
            (12.80,  0.70),
            ( 7.20,  0.70),
            ( 7.20,  5.14),
            ( 5.00,  5.14),
            ( 5.00,  0.70),
            (-2.70,  0.70),
        ]
        forbidden = [
            ("opposite_side_no_drive_zone", [(-2.70, -5.70), (12.80, -5.70), (12.80, -4.20), (-2.70, -4.20)]),
            ("left_parking_slots_no_drive_zone", [(-2.70, 0.70), (5.00, 0.70), (5.00, 5.14), (-2.70, 5.14)]),
            ("right_parking_slots_no_drive_zone", [(7.20, 0.70), (12.80, 0.70), (12.80, 5.14), (7.20, 5.14)]),
        ]
        return DrivableArea(outer, forbidden)

    @staticmethod
    def from_yaml(path: str):
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError("PyYAML is required for loading a YAML drivable area file") from exc
        with open(path, "r") as f:
            cfg = yaml.safe_load(f)
        da = cfg.get("drivable_area", cfg)
        outer = da["outer_polygon"]
        forbidden = []
        for item in da.get("forbidden_polygons", []):
            forbidden.append((item.get("name", "forbidden"), item["points"]))
        return DrivableArea(outer, forbidden, name=os.path.basename(path))

    def footprint_inside_outer(self, footprint):
        for x, y in sample_polygon_boundary(footprint, samples_per_edge=5):
            if not point_in_polygon(x, y, self.outer_polygon):
                return False
        return True

    def footprint_hits_forbidden(self, footprint):
        for _, poly in self.forbidden_polygons:
            if convex_polygons_intersect(footprint, poly):
                return True
            for x, y in sample_polygon_boundary(footprint, samples_per_edge=2):
                if point_in_polygon(x, y, poly):
                    return True
        return False

    def is_footprint_valid(self, footprint):
        return self.footprint_inside_outer(footprint) and not self.footprint_hits_forbidden(footprint)
