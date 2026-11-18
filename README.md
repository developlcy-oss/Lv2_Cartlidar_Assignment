# Lv2_Cartlidar_Assignment

퍽 운반 로봇 (4륜 · 매니퓰레이터 · LiDAR · D435 × 2 · Arducam) — Perception · Controller · Firmware. ROS 2 Lyrical · C++17 component.

- 노드 설계 기준: [workspace/docs/robot_nodes.html](workspace/docs/robot_nodes.html) (4륜, XM430 ID 1~4)
- 통신 정의: [workspace/docs/interfaces.md](workspace/docs/interfaces.md) (OpenCR 절은 v5 제안 — [변경 요청](workspace/docs/opencr_v5_change_request.md), 승인 대기)
- 설계 대비 검토 · 개선: [workspace/docs/review_robot_nodes.md](workspace/docs/review_robot_nodes.md)

## 패키지 (`workspace/ros2_ws/src`)

| 팀 | 패키지 | 내용 | 실행 위치 |
|---|---|---|---|
| 공동 | `robot_interfaces` | 메시지 · 서비스 · 액션, `names.hpp` (토픽 · 액션 이름과 QoS 를 한곳에) | — |
| Perception | `perception` | `perception_node` (D435 × 2 · Arducam 영상 토픽 · 검출 · 대상 추적 · AprilTag · 신호등 · 바코드 · `/arm_range`), `scan_merger`, `rear_obstacle`, `tfmini_node` · `obstacle_stop` (1층 TFmini 박스 거리 · 비상 정지) | ⑦ perception_container / ⑧ (TFmini · obstacle_stop) |
| Perception | `perception_survey` | `puck_survey` (B0 퍽 12칸 측량 → `/perception/puck_survey`, start · reset · finish · revisit) | ⑦ 옆 단독 프로세스 |
| Controller | `mission` | `mission_node` (미션표 순서 실행 · 재시도 · fallback · PAUSED 재개) | ⑧ control_container |
| Controller | `control` | `drive_node` (nav2 · 정렬 · skill), `manipulator_node` (IK · 집기 · skill), 공용 `fsm.hpp` | ⑧ control_container |
| Controller | `mission_planner` | `mission_planner_node` (plan 모드: 신호 · 바코드 → 할당 · 적재 · 경로 계획 → `/mission_planner/plan`), 경기장 지도 `map_nav` · `map_amcl`| 단독 (`planner.launch.py`) |
| Controller | `robot_policy` | 정책 추론 라이브러리 (ONNX Runtime 선택) | ⑧ (drive · manipulator 안) |
| Firmware | `opencr_hardware` | ros2_control SystemInterface (잠금식 E-stop · /diagnostics · 펌웨어 버전 확인), URDF · 컨트롤러 설정 | ① ros2_control_node |
| Firmware | `opencr_firmware` | OpenCR Arduino 스케치 (colcon 제외) | OpenCR |
| 공동 | `bringup` | 전체 실행 launch (`bringup.launch.py`, 센서 드라이버 `sensors.launch.py`), ekf 설정 | — |

## 빌드

```bash
cd workspace/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

선택 의존성은 있으면 켜지고, 없으면 경고만 내고 빌드된다: librealsense2 (없으면 D435 검색 모드 거부), TensorRT (없으면 HSV 색 검출), ONNX Runtime (없으면 `skill_enabled:=false`).

펌웨어는 OpenCR 2대 (주행 · 팔) 에 역할별로 따로 빌드해 올린다: `opencr_firmware/tools/build_firmware.sh wheel|arm` (설명: [opencr_firmware/README.md](workspace/ros2_ws/src/opencr_firmware/README.md)). `opencr_protocol.h`는 `opencr_hardware/include/.../opencr_protocol.hpp`와 같아야 한다.

## 실행

```bash
source workspace/ros2_ws/install/setup.bash
ros2 launch bringup bringup.launch.py        # 경기: 위치 추정 · Nav2 · 계획기 · 미션 (use_localization · use_nav2 기본 true, 2026-11-14)
```

벤치 시험 (주행 없이) 은 `use_localization:=false use_nav2:=false`, 측량 없이 미리 만든 등록표로 계획하려면 `use_survey:=false pucks_file:=<등록표.yaml>` (형식 `mission_planner/config/problem_q1_example.yaml`).

미션은 기본이 plan 모드 (`mission_source:=plan`: `mission_plan.yaml` 덧씌움 + 계획기 `mission_planner_node`). 옛 태그 미션표는 `mission_source:=tag` 일 때만 불러온다.
주요 인자: `use_hardware` · `use_teleop` · `use_perception` · `use_ekf` · `use_tfmini` (기본 true — TFmini 박스 거리 · obstacle_stop · twist_mux obstacle lock, 시뮬레이션은 false), `ports` (기본 `/dev/ttyACM0,/dev/ttyACM1`, 역할은 프로토콜 응답으로 판별), `skill_enabled` (기본 false, ONNX 빌드에서만 true),
`drive` (기본 `belt` = 증속벨트 1:2.0 (v7 44T → 22T), `direct` = 직결 — URDF 바퀴 위치 · 한계, 기어비, diff_drive 바퀴 간격 · 속도 한계가 같이 바뀜).
nav2 · slam_toolbox 는 기존 launch 를 쓰고, nav2 체인의 마지막 출력만 `/cmd_vel_nav` 로 둔다.
퍽 박스(205 × 205 × 70 mm)는 LiDAR 평면(바닥 위 약 80 mm)보다 낮아 `/scan` 에 잡히지 않는다 → 주행 지도 `mission_planner/maps/map_nav` 에 박스(70 mm)가 들어 있다 (AMCL 용 `map_amcl` 은 스캔면 위만). 마지막 접근은 drive 정렬(COARSE → FINE, TFmini 박스 거리)이 맡는다
Gazebo 모의에서 TFmini 를 쓰려면 `ros2 run perception scan_to_range` (`/tfmini/scan` LaserScan → `/tfmini/range`).

운용 순서 (경기 직전 체크리스트):
0. **빌드 확인** — 실물 벨트와 `opencr_hardware/urdf/robot.urdf.xacro` `wheel_gear` 가 같은지 (v7 1:2.0 을 달기 전이면 1.5 로 빌드. 다르면 오도메트리가 33 % 어긋남).
1. 켜기 전에 팔을 접은 대기 자세 (STOW) 에 둔다 — 범위 밖이면 opencr_hardware 가 시작을 거부한다 (관절별 메시지, 범위 = `opencr_hardware/urdf/opencr.ros2_control.xacro` `arm_standby_*`, #166).
2. 로봇을 출발 자세 (0.694, 0.950, yaw π) 에 놓는다 — AMCL 초기 자세 = `localization.launch.py` start_x · start_y · start_yaw.
3. `ros2 launch bringup bringup.launch.py`
4. **준비 확인** — mission_node 가 바뀔 때마다 터미널에 표를 찍는다 (`[ OK ]` · `[실패]` 답이 나쁨 · `[대기]` 아직 답 없음):
   `1 안전 (E-stop · 진단)` → `2 컨트롤러` (joint_state · diff_drive · arm · imu 가 active) → `3 노드 서버` (drive · manipulator · perception · 계획기 · 팔 정리 `/manipulator/startup_done`)
   → `4 위치 추정` (/localization/status OK) → `5 Nav2` (lifecycle_manager_navigation 모든 노드 active).
   한 번에 보기: `ros2 topic echo /mission/ready --once` = `READY` (아니면 `NOT_READY: <빠진 것>`).
   E-stop 은 켤 때 잠긴 상태라 5 단계에서 리셋 전까지 1 안전만 `[실패]` 로 남는다.
5. 조이스틱을 켠다 (0.5 s 끊기면 E-stop 잠김) → 리셋 버튼 → 팔은 STOW 에 머문다 (호스트만 다시 켜 DRIVE 에 남았으면 STOW 로 접음, 2026-11-18).
   터미널에 `startup: arm at 'STOW'` 가 나오고 표가 `READY` 가 되는지 확인.
6. 시작 버튼 (`/mission/start`) — 준비가 안 됐으면 거절하고 빠진 항목을 알려 준다 (`not ready: …`).
   팔을 STOW → DRIVE 로 폄 (`unfold DRIVE`, FOLD_VIA 경유) → 신호 · 바코드 읽기 (observe) → 퍽 조사 (survey) → 계획 (`/mission_planner/plan`) → 계획 단계 실행 (`mission_planner/README.md`).
7. 진행 상황: `ros2 topic echo /mission/status`. 집기를 재시도까지 실패한 퍽은 건너뛰고 (`skipped: pick <칸>`),
   주행 · 놓기 등 그 밖의 재시도 소진은 HOME 으로 돌아온 뒤 FAILED (`mission.yaml` `on_fail`).
8. 끝: 정지 버튼 (`/mission/stop`). 팔을 STOW 로 접을 때 팔레트 첫 줄 왼쪽 칸에 닿는 문제 주의 (#277).

E-stop · 진단 오류가 나면 미션이 PAUSED 가 된다. 원인 해결 → 리셋 → 시작 버튼이면 현재 단계부터 다시 한다. 끝내려면 정지 버튼 (`/mission/stop`).

설정: 미션 `mission/config/mission.yaml` · plan 모드 `mission/config/mission_plan.yaml` (옛 태그 미션표 `mission_table.yaml` 은 tag 모드만), 인식 `perception/config/perception.yaml` (HSV 범위 · 크기 검증 · D435 화이트밸런스), 정책 `robot_policy/config/policies.yaml`.

## 시험

```bash
cd workspace/ros2_ws
colcon test && colcon test-result --verbose
```

프로토콜 역할 분리 · 듀얼 보드 루프백 · 버전 불일치 거부 · 시리얼 독점, 미션표 문법, 정렬 · 안전, IK, 추적 · 검출, 정책 입출력.

## 실물에서 맞출 값 (★)

| 대상 | 맞출 값 |
|---|---|
| 접근 · 차체 | `approach.ready_reach`, `base.front_offset`, `target.exclude_min/max` |
| 인식 | HSV 범위 (현장 조명), `d435.white_balance` · `exposure`, `target.size_min_m/max_m` |
| TF · 카메라 | tfmini_link (URDF base_link (0.130, 0, 0.0243) = 설계값, 바닥 위 31 mm) · arm_camera_link TF, Arducam 보정, 태그 크기 |
| TFmini (박스 거리) | 실제 박스 재질 앞 0.10~0.40 m 줄자 실측 → `tfmini.yaml` `calib_raw_m` · `calib_true_m` (사양 ±5 cm), `min_strength`, mm 출력 확인. 미션표 `approach <거리>` = 렌즈 → 박스 앞면 (팔이 닿는 거리, ≥ 0.13). 빔이 박스 앞면(높이 70 mm)에 맞는지 (렌즈 바닥 위 약 31 mm) |
| /arm_range | `perception.yaml` `arm.range.*` (tag_gate · seed_gate · hold_s · plane_gate), D435 보유 점 · 평면 교차 오차 (정밀 거리에서 D435 시야 밖) |
| 팔 | STOW · READY · PLACE · TAG_VIEW 관절각, 링크 길이 · 관절 한계 (manipulator.yaml · URDF · config.h 세 곳 같게) |
| 펌웨어 | 바퀴 ID · 방향 · 속도 한계, `WHEEL_MODEL` (바퀴 XM430-W210 = 1030 확정) |
| 구동 (`drive:=belt`) | 바퀴 간격 실측 (URDF `wheel_y` 0.1585 · `drive_belt.yaml` `wheel_separation` 0.317 = 설계값), 실행 후 `ros2 param get /diff_drive_controller wheel_separation` = 0.317 확인 (`drive_belt.yaml` 이 기본 설정을 덮어쓰는지), `wheel_separation_multiplier` (회전 시험), 벨트 장력 · 이빨 건너뜀 |
| 조이스틱 · 스위치 | 버튼 번호 (`teleop.yaml`, URDF `estop_button` · `reset_button`), 물리 스위치 핀 |
| 그리퍼 | 개폐 액추에이터 (미정) |
