# alpha_control

후륜축 중심 좌표계(rear axle origin)를 기준으로 자율주행 주차 경로를 생성하고, 생성된 참조 궤적을 Pure Pursuit + PID 제어기로 추종하기 위한 ROS 패키지입니다.

---

## 1. 패키지 개요

`alpha_control` 패키지는 자동주차 제어부를 위한 패키지로, 다음 기능을 포함합니다.

- Hybrid A* 기반 주차 경로 생성
- 후륜축 중심 차량 모델 적용
- 주차 구역 및 장애물 기반 충돌 검사
- Raw path를 reference trajectory로 변환
- Pure Pursuit 기반 조향 제어
- PID 기반 속도 제어
- RViz를 통한 경로 및 주차 환경 시각화

전체 흐름은 다음과 같습니다.

```text
주차 시나리오 생성
        ↓
Hybrid A* 경로 생성
        ↓
Reference Trajectory 생성
        ↓
Pure Pursuit + PID 경로 추종
        ↓
Tracking Command 출력
```

---

## 2. 폴더 구조

```text
alpha_control/
├── CMakeLists.txt                                  # ROS 패키지 빌드, 메시지 생성, Python 실행 파일 설치 설정
├── package.xml                                     # 패키지 이름, 버전, 의존성, 설명을 정의하는 ROS 메타정보 파일
├── README.md                                       # 패키지 전체 설명과 사용법을 정리한 문서
├── __init__.py                                     # alpha_control 폴더를 Python 패키지로 인식시키는 파일
│
├── launch/                                         # ROS 노드 실행 조합을 정의하는 launch 파일 폴더
│   ├── parking_only.launch                         # Hybrid A* 주차 경로 생성 노드만 실행하는 launch 파일
│   ├── controller_only.launch                      # 생성된 경로를 추종하는 path tracker 노드만 실행하는 launch 파일
│   ├── parking_with_tracker.launch                 # 주차 경로 생성 노드와 추종 제어 노드를 함께 실행하는 launch 파일
│   ├── full_pipeline.launch                        # 전체 파이프라인 실행용 launch 파일
│   └── debug/                                      # 디버깅용 launch 파일 폴더
│       └── hybrid_astar_debug.launch               # Hybrid A* 경로 생성 결과를 RViz와 함께 확인하는 디버그 launch 파일
│
├── nodes/                                          # ROS topic publish/subscribe를 담당하는 실제 실행 노드 폴더
│   ├── parking_planner_node.py                     # 주차 시나리오 생성, Hybrid A* 경로 생성, reference trajectory 발행 노드
│   └── path_tracker_node.py                        # reference trajectory를 받아 Pure Pursuit + PID로 추종 명령을 계산하는 노드
│
├── scripts/                                        # 실행 편의를 위한 보조 스크립트 폴더
│   └── hybrid_astar_rviz_demo.py                   # Hybrid A* RViz 데모를 실행하는 스크립트
│
├── core/                                           # ROS와 분리된 핵심 알고리즘 코드 폴더
│   ├── __init__.py                                 # core 폴더를 Python 패키지로 인식시키는 파일
│   │
│   ├── common/                                     # 여러 모듈에서 공통으로 사용하는 자료형과 기하 함수 폴더
│   │   ├── __init__.py                             # common 폴더를 Python 패키지로 인식시키는 파일
│   │   ├── geometry.py                             # 각도 정규화, 좌표 회전, 다각형 충돌 검사 등 기하 계산 함수 모음
│   │   └── types.py                                # 장애물, 탐색 노드, 경로점, 플래너 설정 등 공통 데이터 클래스 정의
│   │
│   ├── planning/                                   # Hybrid A* 경로 계획 관련 코드 폴더
│   │   ├── __init__.py                             # planning 폴더를 Python 패키지로 인식시키는 파일
│   │   ├── drivable_area.py                        # 차량이 움직일 수 있는 주행 가능 영역과 금지 영역을 검사하는 코드
│   │   └── hybrid_astar.py                         # 후륜축 기준 차량 모델을 이용해 Hybrid A* 주차 경로를 생성하는 핵심 코드
│   │
│   ├── trajectory/                                 # 생성된 경로를 제어용 reference trajectory로 변환하는 코드 폴더
│   │   ├── __init__.py                             # trajectory 폴더를 Python 패키지로 인식시키는 파일
│   │   ├── reference_trajectory.py                 # raw path를 보간하고 속도, 가속도, 조향각이 포함된 참조 궤적으로 변환
│   │   └── csv_export.py                           # 생성된 reference trajectory를 CSV 파일로 저장하는 코드
│   │
│   ├── scenario/                                   # 주차 상황과 장애물 배치를 생성하는 코드 폴더
│   │   ├── __init__.py                             # scenario 폴더를 Python 패키지로 인식시키는 파일
│   │   └── rear_parking.py                         # 후진 주차 시나리오의 시작 위치, 목표 위치, 주차 차량 장애물을 생성
│   │
│   ├── control/                                    # 경로 추종 제어 알고리즘 폴더
│   │   ├── __init__.py                             # control 폴더를 Python 패키지로 인식시키는 파일
│   │   ├── pure_pursuit.py                         # lookahead target을 기준으로 조향각을 계산하는 Pure Pursuit 제어기
│   │   ├── pid_speed.py                            # 목표 속도와 현재 속도 차이를 이용해 accel/brake를 계산하는 PID 속도 제어기
│   │   └── reference_tracker.py                    # Pure Pursuit와 PID를 결합해 최종 추종 제어 결과를 만드는 코드
│   │
│   └── visualization/                              # RViz 시각화 관련 코드 폴더
│       ├── __init__.py                             # visualization 폴더를 Python 패키지로 인식시키는 파일
│       └── rviz_visualizer.py                      # 장애물, 주차칸, raw path, reference path, 탐색 결과를 RViz Marker로 표시
│
├── config/                                         # 차량 제원, 플래너, 제어기 파라미터 설정 파일 폴더
│   ├── vehicle_params.yaml                         # 차량 전장, 전폭, wheelbase, rear overhang, 주차칸 크기 설정 파일
│   ├── hybrid_astar_params.yaml                    # Hybrid A* 해상도, 조향 후보, 목표 허용 오차, reference 생성 파라미터 설정 파일
│   ├── controller_params.yaml                      # Pure Pursuit lookahead 거리와 PID gain, 제어 주기 설정 파일
│   └── manual_drivable_area_rear_parking.yaml      # 수동으로 정의한 주행 가능 영역과 금지 영역 지도 설정 파일
│
├── msgs/                                           # alpha_control에서 사용하는 커스텀 ROS 메시지 정의 폴더
│   ├── RefTrajectoryPoint.msg                      # reference trajectory의 한 점에 대한 위치, 속도, 가속도, 조향각, 기어 정보 메시지
│   ├── RefTrajectory.msg                           # 여러 개의 RefTrajectoryPoint를 묶어 전체 참조 궤적을 표현하는 메시지
│   └── TrackingCommand.msg                         # path tracker가 계산한 목표 속도, 조향각, accel/brake, 오차 정보 메시지
│
├── rviz/                                           # RViz 설정 파일 폴더
│   └── parking_debug.rviz                          # 주차 환경, 장애물, Hybrid A* 경로, reference trajectory를 보기 위한 RViz 설정 파일
│
├── tests/                                          # ROS 없이 알고리즘을 단독으로 확인하는 테스트 코드 폴더
│   └── test_planner_cli.py                         # 터미널에서 Hybrid A*와 reference trajectory 생성을 테스트하는 CLI 코드
│
└── data/                                           # 생성된 데이터나 결과물을 저장하기 위한 폴더
    └── reference/                                  # 생성된 reference trajectory 관련 데이터를 저장하기 위한 폴더
```

---

## 3. 주요 모듈 설명

### 3.1 `launch/`

| 파일 | 설명 |
|---|---|
| `parking_only.launch` | Hybrid A* 기반 주차 경로 생성 노드만 실행합니다. |
| `controller_only.launch` | 생성된 reference trajectory를 추종하는 제어 노드만 실행합니다. |
| `parking_with_tracker.launch` | 주차 경로 생성 노드와 경로 추종 노드를 함께 실행합니다. |
| `full_pipeline.launch` | 전체 자동주차 제어 파이프라인을 실행하기 위한 launch 파일입니다. |
| `debug/hybrid_astar_debug.launch` | Hybrid A* 경로 생성 결과를 RViz에서 디버깅하기 위한 launch 파일입니다. |

---

### 3.2 `nodes/`

| 파일 | 설명 |
|---|---|
| `parking_planner_node.py` | 주차 시나리오를 생성하고 Hybrid A*로 경로를 만든 뒤 reference trajectory를 발행합니다. |
| `path_tracker_node.py` | reference trajectory와 현재 차량 상태를 받아 Pure Pursuit + PID 기반 추종 명령을 계산합니다. |

---

### 3.3 `core/common/`

| 파일 | 설명 |
|---|---|
| `geometry.py` | 각도 정규화, 좌표 회전, 다각형 충돌 검사 등 공통 기하 연산을 제공합니다. |
| `types.py` | 장애물, 경로점, 탐색 노드, 플래너 설정 등 공통 데이터 구조를 정의합니다. |

---

### 3.4 `core/planning/`

| 파일 | 설명 |
|---|---|
| `drivable_area.py` | 차량이 주행 가능한 영역과 금지 영역을 정의하고 충돌 여부를 검사합니다. |
| `hybrid_astar.py` | 차량 기구학 제약을 고려하여 Hybrid A* 기반 주차 경로를 생성합니다. |

---

### 3.5 `core/trajectory/`

| 파일 | 설명 |
|---|---|
| `reference_trajectory.py` | Hybrid A*로 생성된 raw path를 제어기가 추종하기 쉬운 참조 궤적으로 변환합니다. |
| `csv_export.py` | 생성된 reference trajectory를 CSV 파일로 저장합니다. |

---

### 3.6 `core/scenario/`

| 파일 | 설명 |
|---|---|
| `rear_parking.py` | 후진 주차 상황의 시작 위치, 목표 위치, 주변 장애물 차량을 생성합니다. |

---

### 3.7 `core/control/`

| 파일 | 설명 |
|---|---|
| `pure_pursuit.py` | lookahead target을 기준으로 차량의 조향각을 계산합니다. |
| `pid_speed.py` | 목표 속도와 현재 속도 차이를 이용해 가속 및 브레이크 명령을 계산합니다. |
| `reference_tracker.py` | Pure Pursuit와 PID를 결합하여 최종 경로 추종 명령을 생성합니다. |

---

### 3.8 `core/visualization/`

| 파일 | 설명 |
|---|---|
| `rviz_visualizer.py` | 주차 환경, 장애물, 탐색 경로, 참조 궤적을 RViz Marker로 시각화합니다. |

---

### 3.9 `config/`

| 파일 | 설명 |
|---|---|
| `vehicle_params.yaml` | 차량 전장, 전폭, wheelbase, rear overhang, 최대 조향각, 주차칸 크기를 설정합니다. |
| `hybrid_astar_params.yaml` | Hybrid A* 탐색 해상도, 조향 후보, 목표 허용 오차, reference 생성 파라미터를 설정합니다. |
| `controller_params.yaml` | Pure Pursuit lookahead 거리, PID gain, 제어 주기 등을 설정합니다. |
| `manual_drivable_area_rear_parking.yaml` | 수동으로 정의한 주행 가능 영역과 금지 영역을 설정합니다. |

---

### 3.10 `msgs/`

| 파일 | 설명 |
|---|---|
| `RefTrajectoryPoint.msg` | 참조 궤적의 한 점에 대한 위치, 속도, 가속도, 조향각, 기어 정보를 담습니다. |
| `RefTrajectory.msg` | 여러 개의 `RefTrajectoryPoint`를 묶어 전체 reference trajectory를 표현합니다. |
| `TrackingCommand.msg` | 제어기가 계산한 목표 속도, 조향각, 가속, 브레이크, 추종 오차 정보를 담습니다. |

---

## 4. 차량 좌표계 기준

본 패키지의 모든 차량 pose는 **후륜축 중심(rear axle center)** 을 기준으로 합니다.

```text
pose = (x_rear_axle, y_rear_axle, yaw)
```

따라서 Hybrid A* 경로 생성, reference trajectory 생성, Pure Pursuit 제어 모두 같은 좌표 기준을 사용합니다.

이 기준을 사용하는 이유는 다음과 같습니다.

- 자전거 모델 기반 차량 운동 방정식 적용이 쉬움
- Pure Pursuit 제어에서 기준점이 명확함
- 차량 중심 기준보다 주차 시 전후방 충돌 검사가 정확함
- MORAI 제어부와 연결할 때 차량 기구학 모델을 일관되게 유지할 수 있음

---

## 5. 차량 및 주차 구역 기본 제원

```yaml
vehicle_length: 4.430        # 차량 전장 [m]
vehicle_width: 1.825         # 차량 전폭 [m]
wheel_base: 2.700            # 축거 [m]
rear_overhang: 0.850         # 후륜축에서 차량 후면까지 거리 [m]
max_steer_deg: 35.0          # 최대 조향각 [deg]

parking_slot_width: 2.200    # 일반 주차 구역 폭 [m]
parking_slot_length: 4.400   # 일반 주차 구역 길이 [m]
```

---

## 6. 실행 방법

### 6.1 패키지 빌드

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 6.2 주차 경로 생성만 실행

```bash
roslaunch alpha_control parking_only.launch
```

---

### 6.3 경로 추종 제어기만 실행

```bash
roslaunch alpha_control controller_only.launch
```

---

### 6.4 경로 생성 + 추종 제어 함께 실행

```bash
roslaunch alpha_control parking_with_tracker.launch
```

---

### 6.5 Hybrid A* 디버그 실행

```bash
roslaunch alpha_control debug/hybrid_astar_debug.launch
```

---

## 7. 주요 ROS Topic

### 7.1 발행 Topic

| Topic | 메시지 타입 | 설명 |
|---|---|---|
| `/alpha_control/ref_trajectory` | `alpha_control/RefTrajectory` | 생성된 reference trajectory |
| `/alpha_control/tracking_cmd` | `alpha_control/TrackingCommand` | 경로 추종 제어 결과 |
| `/alpha_control/tracking_target_pose` | `geometry_msgs/PoseStamped` | 현재 추종 중인 lookahead target pose |
| `/hybrid_astar/path_raw` | `nav_msgs/Path` | Hybrid A* raw path |
| `/hybrid_astar/path_ref` | `nav_msgs/Path` | 보간된 reference path |
| `/hybrid_astar/static_markers` | `visualization_msgs/MarkerArray` | 주차장, 장애물, 시작/목표 차량 시각화 |
| `/hybrid_astar/search_markers` | `visualization_msgs/MarkerArray` | Hybrid A* 탐색 과정 시각화 |
| `/hybrid_astar/final_markers` | `visualization_msgs/MarkerArray` | 최종 경로 및 차량 footprint 시각화 |

---

### 7.2 구독 Topic

| Topic | 메시지 타입 | 설명 |
|---|---|---|
| `/alpha_control/current_pose` | `geometry_msgs/PoseStamped` | 현재 차량의 후륜축 기준 pose |
| `/alpha_control/current_speed` | `std_msgs/Float32` | 현재 차량 속도 |
| `/alpha_control/ref_trajectory` | `alpha_control/RefTrajectory` | 추종할 reference trajectory |

---

## 8. 제어 출력 구조

`path_tracker_node.py`는 최종적으로 `/alpha_control/tracking_cmd`를 발행합니다.

```text
target_speed       # 목표 속도
current_speed      # 현재 속도
accel_cmd          # 가속 명령
brake_cmd          # 브레이크 명령
steering_rad       # 계산된 조향각 [rad]
gear               # 전진 또는 후진
nearest_index      # 현재 차량과 가장 가까운 reference index
target_index       # lookahead target index
lookahead_distance # lookahead 거리
lateral_error      # 횡방향 오차
heading_error      # heading 오차
goal_reached       # 목표 도달 여부
```

현재 메시지는 MORAI의 `morai_msgs/CtrlCmd`가 아니므로, MORAI 차량을 직접 제어하기 위해서는 별도의 변환 노드가 필요합니다.

```text
/alpha_control/tracking_cmd
        ↓
MORAI command bridge
        ↓
/ctrl_cmd 또는 /ctrl_cmd_0
```

---

## 9. MORAI 연동 시 주의사항

최신 MORAI `CtrlCmd` 메시지에서는 조향 입력이 `steering`이 아니라 `front_steer`, `rear_steer`로 분리되어 있을 수 있습니다.

또한 `front_steer`가 rad 단위 조향각이 아니라 최대 조향각 대비 비율값인 경우가 있으므로 다음과 같은 변환이 필요할 수 있습니다.

```python
front_steer = -steering_rad / max_steering_rad
```

예시:

```python
import math

def clip(value, lower=-1.0, upper=1.0):
    return max(lower, min(upper, value))

def steering_rad_to_front_steer(steering_rad, max_steering_deg):
    max_steering_rad = math.radians(max_steering_deg)
    front_steer = -steering_rad / max_steering_rad
    return clip(front_steer)
```

---

## 10. 전체 파이프라인 요약

```text
parking_planner_node.py
        ↓
rear_parking.py
        ↓
hybrid_astar.py
        ↓
reference_trajectory.py
        ↓
/alpha_control/ref_trajectory
        ↓
path_tracker_node.py
        ↓
pure_pursuit.py + pid_speed.py
        ↓
/alpha_control/tracking_cmd
```

---

## 11. 현재 구현 범위

현재 패키지에서 구현된 기능은 다음과 같습니다.

- 후륜축 중심 차량 모델 적용
- 후진 주차 시나리오 생성
- Hybrid A* 기반 경로 생성
- 차량 footprint 기반 충돌 검사
- 주행 가능 영역 검사
- reference trajectory 생성
- Pure Pursuit 조향 제어
- PID 속도 제어
- RViz 시각화
- CSV 저장 기능

---

## 12. 향후 추가할 수 있는 기능

- MORAI `CtrlCmd` 변환 노드 추가
- `/Ego_topic` 기반 현재 차량 pose 변환 노드 추가
- 실제 인지부에서 들어오는 parking space, occupancy grid 연동
- 장애물 실시간 업데이트
- NMPC 기반 trajectory optimization 추가
- 전면 주차, 측면 주차 시나리오 확장
