# -*- coding: utf-8 -*-
import heapq
import math
import time
from typing import Callable, Dict, List, Optional, Tuple
from core.common.geometry import angle_diff, convex_polygons_intersect, normalize_angle, rotate_point
from core.common.types import DensePathPoint, PlannerConfig, RectObstacle, SearchNode
from core.planning.drivable_area import DrivableArea

class HybridAStarPlanner:
    """Hybrid-State A* planner.

    Important coordinate convention:
      - SearchNode.x, SearchNode.y, SearchNode.yaw are rear-axle-center pose.
      - Bicycle model integration is therefore directly based on the rear axle.
      - Ego footprint is computed by projecting the body forward/backward from the rear axle.
    """
    def __init__(self, obstacles: List[RectObstacle], drivable_area: DrivableArea, cfg: Optional[PlannerConfig] = None):
        self.cfg = cfg or PlannerConfig()
        self.obstacles = obstacles
        self.drivable_area = drivable_area
        self.last_stats: Dict[str, float] = {}
        self.last_closed_count = 0

    def vehicle_polygon_from_rear_axle(self, x: float, y: float, yaw: float, padding: float = 0.0):
        front = self.cfg.rear_axle_to_front + padding
        rear = self.cfg.rear_axle_to_rear + padding
        half_w = 0.5 * self.cfg.vehicle_width + padding
        # local x=0 is rear axle. Front bumper is +front, rear bumper is -rear.
        local = [(front, half_w), (front, -half_w), (-rear, -half_w), (-rear, half_w)]
        pts = []
        for lx, ly in local:
            rx, ry = rotate_point(lx, ly, yaw)
            pts.append((x + rx, y + ry))
        return pts

    def vehicle_body_center_from_rear_axle(self, x: float, y: float, yaw: float):
        offset = self.cfg.body_center_from_rear_axle
        rx, ry = rotate_point(offset, 0.0, yaw)
        return x + rx, y + ry

    def is_inside_map_boundary(self, footprint) -> bool:
        for x, y in footprint:
            if x < self.cfg.min_x or x > self.cfg.max_x or y < self.cfg.min_y or y > self.cfg.max_y:
                return False
        return True

    def is_pose_valid(self, x: float, y: float, yaw: float) -> bool:
        da_poly = self.vehicle_polygon_from_rear_axle(x, y, yaw, self.cfg.drivable_padding)
        if not self.is_inside_map_boundary(da_poly):
            return False
        if not self.drivable_area.is_footprint_valid(da_poly):
            return False
        col_poly = self.vehicle_polygon_from_rear_axle(x, y, yaw, self.cfg.collision_padding)
        for obs in self.obstacles:
            if convex_polygons_intersect(col_poly, obs.polygon(self.cfg.collision_padding)):
                return False
        return True

    def is_trajectory_valid(self, traj):
        return all(self.is_pose_valid(x, y, yaw) for x, y, yaw in traj)

    def calc_index(self, x: float, y: float, yaw: float):
        ix = int(round((x - self.cfg.min_x) / self.cfg.xy_resolution))
        iy = int(round((y - self.cfg.min_y) / self.cfg.xy_resolution))
        yaw_n = normalize_angle(yaw)
        if yaw_n < 0.0:
            yaw_n += 2.0 * math.pi
        bins = max(1, int(round(2.0 * math.pi / self.cfg.yaw_resolution)))
        iyaw = int(round(yaw_n / self.cfg.yaw_resolution)) % bins
        return ix, iy, iyaw

    def heuristic(self, node: SearchNode, goal):
        dist = math.hypot(goal[0] - node.x, goal[1] - node.y)
        yaw_cost = abs(angle_diff(node.yaw, goal[2]))
        return self.cfg.distance_heuristic_weight * dist + self.cfg.yaw_heuristic_weight * yaw_cost

    def reached_goal(self, node: SearchNode, goal):
        d = math.hypot(node.x - goal[0], node.y - goal[1])
        yaw_err = abs(angle_diff(node.yaw, goal[2]))
        return d <= self.cfg.goal_xy_tolerance and yaw_err <= self.cfg.goal_yaw_tolerance

    def steer_samples(self):
        n = max(1, int(self.cfg.steer_sample_count))
        if n == 1:
            return [0.0]
        return [-self.cfg.max_steer + 2.0 * self.cfg.max_steer * float(i) / float(n - 1) for i in range(n)]

    def simulate_motion(self, node: SearchNode, steer: float, direction: int) -> SearchNode:
        x, y, yaw = node.x, node.y, node.yaw
        steps = max(1, int(math.ceil(self.cfg.primitive_length / self.cfg.integration_step)))
        ds = self.cfg.primitive_length / float(steps) * float(direction)
        traj = []
        for _ in range(steps):
            x += math.cos(yaw) * ds
            y += math.sin(yaw) * ds
            yaw = normalize_angle(yaw + ds / self.cfg.wheel_base * math.tan(steer))
            traj.append((x, y, yaw))
        child = SearchNode(x=x, y=y, yaw=yaw, parent=None, direction=direction, steer=steer, traj=traj)
        child.index = self.calc_index(child.x, child.y, child.yaw)
        return child

    def transition_cost(self, parent, child):
        cost = self.cfg.primitive_length
        if child.direction < 0:
            cost *= self.cfg.reverse_penalty
        if parent.parent is not None and child.direction != parent.direction:
            cost += self.cfg.gear_switch_penalty
        cost += self.cfg.steer_penalty * abs(child.steer) / max(self.cfg.max_steer, 1e-9)
        cost += self.cfg.steer_change_penalty * abs(child.steer - parent.steer) / max(self.cfg.max_steer, 1e-9)
        return cost

    def reconstruct_chain(self, nodes, node_id: int):
        chain = []
        cur = node_id
        while cur is not None:
            n = nodes[cur]
            chain.append(n)
            cur = n.parent
        chain.reverse()
        return chain

    def chain_to_dense_path(self, chain):
        if not chain:
            return []
        dense = [DensePathPoint(chain[0].x, chain[0].y, chain[0].yaw, 0, chain[0].steer)]
        for n in chain[1:]:
            if n.traj:
                for x, y, yaw in n.traj:
                    dense.append(DensePathPoint(x, y, yaw, n.direction, n.steer))
            else:
                dense.append(DensePathPoint(n.x, n.y, n.yaw, n.direction, n.steer))
        return dense

    @staticmethod
    def count_gear_changes(chain):
        if not chain:
            return 0
        changes = 0; prev = chain[0].direction
        for n in chain[1:]:
            if n.direction != prev:
                changes += 1; prev = n.direction
        return changes

    @staticmethod
    def chain_length(chain):
        return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(chain[:-1], chain[1:]))

    def plan(self, start, goal, step_callback: Optional[Callable] = None, callback_every_n: int = 10):
        t0 = time.time()
        if not self.is_pose_valid(*start):
            raise ValueError("Start pose is invalid: rear-axle pose makes the vehicle outside drivable area or in collision")
        if not self.is_pose_valid(*goal):
            raise ValueError("Goal pose is invalid: rear-axle pose makes the vehicle outside drivable area or in collision")

        nodes: Dict[int, SearchNode] = {}
        best_g_by_index: Dict[Tuple[int, int, int], float] = {}
        closed: Dict[Tuple[int, int, int], SearchNode] = {}
        open_heap = []
        visual_edges = []

        start_node = SearchNode(x=start[0], y=start[1], yaw=normalize_angle(start[2]), direction=1, steer=0.0)
        start_node.index = self.calc_index(start_node.x, start_node.y, start_node.yaw)
        start_node.h = self.heuristic(start_node, goal)
        nodes[0] = start_node
        best_g_by_index[start_node.index] = 0.0
        heapq.heappush(open_heap, (start_node.f, 0, 0))
        counter = 0; step_count = 0; best_id = 0; best_h = start_node.h
        steers = self.steer_samples()

        while open_heap:
            _, _, cur_id = heapq.heappop(open_heap)
            current = nodes[cur_id]
            if current.index in closed:
                continue
            closed[current.index] = current
            step_count += 1
            if current.h < best_h:
                best_h = current.h; best_id = cur_id
            if self.reached_goal(current, goal):
                chain = self.reconstruct_chain(nodes, cur_id)
                self.last_stats = {"success": 1.0, "iterations": float(step_count), "nodes": float(len(nodes)), "closed": float(len(closed)), "time_sec": time.time() - t0, "path_length": self.chain_length(chain), "gear_changes": float(self.count_gear_changes(chain))}
                if step_callback:
                    step_callback(current, cur_id, open_heap, nodes, closed, visual_edges, [], [], step_count)
                return chain
            if step_count >= self.cfg.max_iterations:
                break
            latest_edges = []; rejected_points = []
            for direction in (1, -1):
                for steer in steers:
                    child = self.simulate_motion(current, steer, direction)
                    child.parent = cur_id
                    px, py = current.x, current.y
                    for tx, ty, _ in (child.traj or []):
                        latest_edges.append((px, py, tx, ty)); px, py = tx, ty
                    if not self.is_trajectory_valid(child.traj or []):
                        rejected_points.append((child.x, child.y)); continue
                    if child.index in closed:
                        rejected_points.append((child.x, child.y)); continue
                    ng = current.g + self.transition_cost(current, child)
                    if ng >= best_g_by_index.get(child.index, float("inf")):
                        continue
                    child.g = ng; child.h = self.heuristic(child, goal); child.index = self.calc_index(child.x, child.y, child.yaw)
                    best_g_by_index[child.index] = ng
                    new_id = len(nodes); nodes[new_id] = child; counter += 1
                    heapq.heappush(open_heap, (child.f, counter, new_id))
                    visual_edges.append((current.x, current.y, child.x, child.y))
            if len(visual_edges) > 4500:
                visual_edges = visual_edges[-4500:]
            if step_callback and step_count % max(1, callback_every_n) == 0:
                step_callback(current, cur_id, open_heap, nodes, closed, visual_edges, latest_edges, rejected_points, step_count)

        partial = self.reconstruct_chain(nodes, best_id)
        self.last_stats = {"success": 0.0, "iterations": float(step_count), "nodes": float(len(nodes)), "closed": float(len(closed)), "time_sec": time.time() - t0, "path_length": self.chain_length(partial), "gear_changes": float(self.count_gear_changes(partial))}
        return None
