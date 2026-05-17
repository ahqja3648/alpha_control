#!/usr/bin/env python3
import os, sys, math
PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)
from core.common.types import PlannerConfig
from core.planning.hybrid_astar import HybridAStarPlanner
from core.scenario.rear_parking import make_rear_parking_scenario
from core.trajectory.reference_trajectory import ReferenceTrajectoryGenerator

def main():
    cfg = PlannerConfig()
    obs, area, start, goal = make_rear_parking_scenario(cfg)
    p = HybridAStarPlanner(obs, area, cfg)
    print('front_overhang', cfg.front_overhang, 'rear_to_front', cfg.rear_axle_to_front, 'body_center_offset', cfg.body_center_from_rear_axle)
    print('start', start, 'valid', p.is_pose_valid(*start))
    print('goal', goal, 'valid', p.is_pose_valid(*goal), 'yawdeg', math.degrees(goal[2]))
    chain = p.plan(start, goal)
    print('stats', p.last_stats)
    if not chain:
        raise SystemExit(1)
    raw = p.chain_to_dense_path(chain)
    ref = ReferenceTrajectoryGenerator(cfg).generate(raw)
    print('chain', len(chain), 'raw', len(raw), 'ref', len(ref))
    print('last', raw[-1])
if __name__ == '__main__':
    main()
