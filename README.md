# alpha_control

Rear-axle-origin control package for autonomous parking.

## Coordinate convention

All ego states and reference trajectory points use the center of the rear wheel axis as `(x, y, yaw)`.
The vehicle body footprint is projected from the rear axle using:

- vehicle length: 4.430 m
- vehicle width: 1.825 m
- wheelbase: 2.700 m (configurable)
- rear overhang: 0.850 m (configurable)
- front overhang: `vehicle_length - wheel_base - rear_overhang`

## Parking slot

- width: 2.200 m
- length: 4.400 m

Because the vehicle length is 4.430 m and the slot length is about 4.400 m, the demo allows a small front-line tolerance in the manual drivable area.

## Run

```bash
roslaunch alpha_control parking_with_tracker.launch
```

## Topics

Planner/RViz:
- `/hybrid_astar/path_raw`
- `/hybrid_astar/path_ref`
- `/alpha_control/ref_trajectory`

Tracker:
- subscribes `/alpha_control/ref_trajectory`
- subscribes `/alpha_control/current_pose` as rear-axle pose
- publishes `/alpha_control/tracking_cmd`
