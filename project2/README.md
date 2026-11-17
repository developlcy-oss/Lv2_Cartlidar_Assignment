# Project 2 최종 제출 — 퍽 운반 로봇 (주사위 기억 로봇)

팀 저장소 `Lv2_Cartlidar_Assignment`, 기준 `dev` 2026-11-17 (`0bbc537`). 루트 [README.md](../README.md) 를 제출용으로 정리한 문서다.
확인하지 못한 값은 **미확인** 으로 적었다.

**공식 경기 (2026-11-16) 때 젯슨 코드 = `fd03441`** (dev, PR #319 병합 — 젯슨 git reflog 10:06 · 10:14 KST pull). #316 (팔 펌웨어 v6 · STOW 2.200) 코드는 들어 있으나 팔 OpenCR 에 v6 를 올렸는지는 미확인.

## 1. 프로젝트 목표

경기장 신호등의 R / Y / G 점멸 12 회와 바코드 (`P2-01` ~ `P2-04`) 를 관측해 장소 L1 ~ L3 의 그룹 · 횟수를 정하고, 시작 박스 6 개의 퍽 12 개를 할당 규칙대로 집어 팔레트에 실은 뒤 각 장소의 주사위 눈 자리에 놓고, 새로 관측해 재검증한 다음 출발 구역으로 돌아와 3 초 정지한다 (규칙: [problem/rules.md](problem/rules.md), 채점 근거: [submission/scoring_map.md](../submission/scoring_map.md)).

로봇: 4 륜 스키드 (XM430-W210 ×4, 증속 벨트 1 : 2.0) + 6 자유도 팔 (XM430-W350 ×6, 관절 1~5 + 그리퍼), LiDAR LDS-03 ×2, RealSense D435 ×2 (상단 앞 · 뒤), 팔 끝 Arducam, 1 층 TFmini Plus, OpenCR 2 대 (주행 · 팔), Jetson Orin Nano Super.
구조 (노드 · 토픽 · TF · 단계 · 안전): [docs/architecture.md](docs/architecture.md).

## 2. 사용 장비 · 소프트웨어 버전

| 구분 | 항목 | 값 | 근거 |
|---|---|---|---|
| 로봇 컴퓨터 | Jetson Orin Nano Super | Ubuntu 24.04.5, JetPack (L4T) R39.2.1, 커널 6.8.12-tegra | 팀 실측 (제출 요청서) |
| | ROS 2 | **Lyrical** (ros-base 0.13.0), Python 3.12.3 | 같음, 루트 README |
| | 사용자 · 저장소 | `cart11`, `~/Lv2_Cartlidar_Assignment` | 경기 로그 경로 (`results/real_20261116/mission_log`) |
| 펌웨어 | OpenCR ×2 | 주행 보드 프로토콜 v5 (`opencr_wheel_v5.bin`), 팔 보드 v6 (`opencr_arm_v6.bin`, 관절 1 π 보정 #316). arduino-cli, OpenCR 보드 패키지 1.5.3, Dynamixel2Arduino 0.8.2 | `opencr_firmware/README.md`, `opencr_protocol.hpp` |
| 센서 | D435 | 앞 시리얼 `261822072247` · 뒤 `261822079801` | `bringup.launch.py` `front_serial` · `rear_serial` |
| | LiDAR · TFmini · Arducam | LDS-03 (`coin_d4_driver`), TFmini Plus (CP2102N USB-UART), Arducam B0200 (UVC) | `requirements.txt`, `perception/udev/99-sensors.rules` |
| 조작기 | RC-100 리모컨 (경기 · 시험), USB 게임패드 (선택) | `rc100:=true` 이면 주행 OpenCR BT-410 경유 | `opencr_hardware/launch/base_bringup.launch.py` |
| 개발 · 학습 PC | 노트북 GPU | RTX 5060 Laptop 8 GB | `workspace/tools/arm_bench/rl/README.md`, `workspace/urdf_and_usd/isaac/README.md` |
| | 강화학습 | Isaac Lab 3.0 (Isaac Sim 6.1) — `workspace/tools/drive_rl`, `workspace/tools/arm_bench/rl` | 각 README |
| | 시뮬 브리지 | Isaac Sim 6.2 단독 (`workspace/tools/isaac_sim/run_bridge.sh`), Gazebo (`navigation/launch/gazebo_sim.launch.py`) | `workspace/docs/frames_tf.md` |
| | 노트북 OS · ROS | Ubuntu 24.04 · ROS 2 Lyrical (정확한 점 버전 **미확인**) | `requirements.txt` 머리말 |

## 3. 설치

### 3-1. apt 패키지 (PC · Jetson 공통)

```bash
grep -v '^#' requirements.txt | grep -v '^$' | xargs sudo apt install -y   # 저장소 루트에서
```

`ros-lyrical-ros2-control` · `ros2-controllers` · `navigation2` · `robot-localization` · `twist-mux` · `joy` · `realsense2-camera` · `coin-d4-driver` · `libzbar-dev` · OpenCV 등 ([requirements.txt](../requirements.txt)).

### 3-2. 선택 의존성 (있으면 켜지고, 없으면 경고만 내고 빌드됨)

| 의존성 | 없으면 | 비고 |
|---|---|---|
| librealsense2 | D435 검색 모드 거부 | `realsense2_camera` 와 같이 설치 |
| TensorRT (+ CUDA) | HSV 색 검출만 | Jetson 은 JetPack 에 포함 |
| ONNX Runtime | `skill_enabled:=false` (정책 skill 거절 → 미션 fallback) | 기본 bringup 은 `skill_enabled:=false`. Jetson 은 `~/onnxruntime/lib` 에 설치 · ldconfig 등록 (2026-11-08) |

### 3-3. udev 규칙 · 권한

```bash
sudo cp workspace/ros2_ws/src/opencr_hardware/udev/99-opencr.rules workspace/ros2_ws/src/perception/udev/99-sensors.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER     # 다시 로그인
```

- `99-opencr.rules`: OpenCR (0483:5740) 를 `dialout` · ModemManager 무시. 두 보드는 VID/PID · 시리얼이 같아 **이름을 만들지 않고** 프로토콜 `role` 로 구별한다.
- `99-sensors.rules`: `/dev/arm_cam` (Arducam) · `/dev/tfmini` (CP2102N 고유 시리얼) · `/dev/lidar_front` (USB 허브 **2 번 칸**) · `/dev/lidar_rear` (허브 **1 번 칸**). 라이다 두 대는 시리얼이 같아 허브 칸으로 잡는다 — 칸을 바꾸지 말 것 (#309).

### 3-4. 펌웨어 (OpenCR 2 대, 역할별로 따로)

```bash
cd workspace/ros2_ws/src/opencr_firmware
tools/build_firmware.sh wheel                          # 주행 보드 (OPENCR_BOARD_ROLE=1) → ~/opencr_builds/opencr_wheel_v5.bin
tools/build_firmware.sh arm                            # 팔 보드 (OPENCR_BOARD_ROLE=2)  → ~/opencr_builds/opencr_arm_v6.bin
tools/build_firmware.sh arm --upload /dev/ttyACM1      # 빌드 후 업로드 (역할 yes 확인)
python3 ../opencr_hardware/scripts/opencr_arm_tool.py monitor --port /dev/ttyACM0 --seconds 2   # 올라간 역할 · 버전 확인 (쓰기 없음)
```

- 역할을 지정하지 않으면 빌드가 실패한다 (기본값 없음). Arduino IDE 업로드 버튼은 쓰지 않는다.
- 업로드가 멈추면 Recovery Mode (SW2 누른 채 Reset). `opencr_protocol.h` 는 `opencr_hardware/include/opencr_hardware/opencr_protocol.hpp` 와 같아야 한다.

## 4. 빌드

```bash
cd workspace/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test && colcon test-result --verbose     # 선택: 단위 시험
```

`opencr_firmware` 는 colcon 대상이 아니다. Nav2 는 한 프로세스 (`nav2_composed`, #276) 라 `navigation` 패키지도 빌드해야 한다.

## 5. 실행 순서

### 5-1. 경기 명령 (전체 미션)

```bash
source /opt/ros/lyrical/setup.bash && source workspace/ros2_ws/install/setup.bash
ros2 launch bringup bringup.launch.py        # 하드웨어 · 센서 · 인식 · 위치 추정 · Nav2 · 계획기 · 미션 (plan 모드)
```

| 인자 | 기본 | 뜻 |
|---|---|---|
| `use_hardware` · `use_teleop` · `use_perception` · `use_tfmini` · `use_ekf` · `use_sensors` | true | 하드웨어 · 조이스틱 · 인식 · TFmini 잠금 · EKF · 드라이버 |
| `use_localization` · `use_nav2` | true (2026-11-14 부터) | AMCL · loc_monitor · Nav2. 벤치 시험은 둘 다 false |
| `mission_source` | `plan` | `plan` = 관측 → 계획기, `tag` = 옛 태그 미션표 |
| `use_survey` · `pucks_file` | true · 빈 값 | false 면 측량 없이 `pucks_file` 등록표로 계획 (형식 `mission_planner/config/problem_q1_example.yaml`) |
| `planner_stop_after` | `none` | `observe` · `first_pick` · `first_place` = 부분 시험 후 복귀 |
| `planner_mode` | `hybrid` | `per_location` · `single_load` |
| `drive` | `belt` | 증속 벨트 1 : 2.0 (`direct` = 직결) |
| `ports` | `/dev/ttyACM0,/dev/ttyACM1` | OpenCR 후보 포트 (역할은 응답으로) |
| `skill_enabled` | false | ONNX Runtime 빌드에서만 true |
| `log_dir` | `~/Lv2_Cartlidar_Assignment/mission_log` | 미션마다 `<log_dir>/<YYMMDD_HHMM>/` |
| `sim` · `use_sim_time` | false | Isaac Sim 브리지 (`sim:=true use_sim_time:=true use_teleop:=false use_tfmini:=false`) |

전체 목록: `workspace/ros2_ws/src/bringup/launch/bringup.launch.py` 머리말 · `DeclareLaunchArgument`.

### 5-2. 경기 직전 체크리스트 (루트 README "운용 순서")

0. **빌드 확인** — 실물 벨트와 `opencr_hardware/urdf/robot.urdf.xacro` `wheel_gear` (belt 2.0) 가 같은지. 다르면 오도메트리가 어긋난다.
1. 전원: **배터리 먼저, USB 나중**. USB 가 먼저 켜졌으면 배터리를 켠 채 두 OpenCR RESET (`MOTOR_CONFIG|FAULT` 방지, drive_demo README 0 장).
2. 팔을 **STOW** 에 둔다 — 범위 밖이면 opencr_hardware 가 시작을 거부 (관절별 메시지). 확인: `workspace/tools/drive_demo/jetson_check.sh opencr`.
3. 로봇을 **출발 자세 (0.694, 0.950, yaw π)** 에 2 ~ 3 cm 안으로 놓는다 (앞이 신호등 쪽).
4. `ros2 launch bringup bringup.launch.py`
5. **준비 확인** — 터미널 `[준비 확인]` 표 (1 안전 · 2 컨트롤러 · 3 노드 서버 · 4 위치 추정 · 5 Nav2), 또는 `ros2 topic echo /mission/ready --once` = `READY`. E-stop 은 켤 때 잠겨 있어 리셋 전까지 1 안전만 `[실패]`.
6. 조이스틱을 켜고 리셋 (RC-100 = 2 + 3) → 팔이 STOW → DRIVE 로 펴지는지 끝까지 본다 (`startup: arm at DRIVE`). **펴지기 전에 시작하면 observe 가 거절된다.** `startup: arm is not at STOW (… rad)` 가 보이면 팔이 STOW 와 어긋난 것이라 정리가 시작조차 안 되므로, 12 V 를 끄고 손으로 STOW 에 맞춘 뒤 다시 켠다 (2026-11-16 경기 실패 원인 — 0.23 rad 어긋남, [docs/architecture.md](docs/architecture.md) 머리 주의).
7. 시작: 조이스틱 START (`teleop.yaml` 버튼 9) 또는 `ros2 service call /mission/start std_srvs/srv/Trigger`. 준비가 안 됐으면 `not ready: …` 로 거절.
8. 진행: `ros2 topic echo /mission/status`. 기록: `<log_dir>/<YYMMDD_HHMM>/mission_log.jsonl` · `images/` · `planner/`.

### 5-3. 경기에서 실제로 쓴 축소 구성 (drive_demo)

2026-11-16 경기는 전체 미션이 observe 에서 FAILED 여서 팔 없이 Nav2 주행 시연으로 진행했다. 터미널 4 개 (J1 `base_bringup` · J2 `scan_rotate.py` · J3 위치 추정 · J4 `./run_demo.sh real slow step init`) — 명령 · 순서는 [workspace/tools/drive_demo/README.md](../workspace/tools/drive_demo/README.md) 2 ~ 3 장. A (시연) 와 B (전체 미션) 를 같이 띄우지 않는다.

## 6. 경기 초기화

| 항목 | 값 | 근거 |
|---|---|---|
| 로봇 출발 자세 | (0.694, 0.950) m, yaw π (−X = 신호등 쪽). 정확히는 `start_x 0.69365` = 출발 구역 가운데 0.734 − 0.04035 | `localization/launch/localization.launch.py` (AMCL 초기 자세) |
| 출발 구역 | 가운데 (0.734, 0.95), 0.6 m 사각 | `mission/config/mission_plan.yaml` `return_check` |
| 팔 | **STOW** `[2.200, -2.088, 1.549, -1.868, 0.0, 0.923]` rad (관절 1~5, 그리퍼 닫힘). 손으로 놓을 때 허용 tick: J1 3425 ~ 3539 · J2 653 ~ 721 · J3 3025 ~ 3091 · J4 797 ~ 864 · J5 1849 ~ 2246 · 그리퍼 1531 ~ 2669 (가운데 J1 3482 · J5 2048) | `control/config/manipulator.yaml` `poses.STOW`, `opencr_hardware/urdf/opencr.ros2_control.xacro` `arm_standby_*_tick` |
| 퍽 배치 | [problem/placement.csv](problem/placement.csv) · [problem/problem.md](problem/problem.md) 대로 (박스 중심 X − 40 mm = side −1, X + 40 mm = side +1, z 70 mm, 발제 좌표 mm) | `project2/problem/` |
| 퍽 등록표 (측량 끌 때) | `use_survey:=false pucks_file:=<등록표.yaml>` — 실물 단계 시험용 `workspace/docs/stage_test/pucks_stage.yaml` | `workspace/docs/mission_stage_test.md` |
| 지도 | 발제 도면 SDF 지도 `mission_planner/maps/map_amcl` (AMCL) · `map_nav` (주행, 박스 70 mm 포함). 실측 지도는 쓰지 않음 (11-15 결정) | `bringup.launch.py`, drive_demo README 6 장 |

> `problem/` 은 팀 실물 퍽 등록표 (`workspace/docs/stage_test/pucks_stage.yaml`, 빨강 · 초록 원기둥 · 직육면체 각 3) 와 발제 공개 연습 Q1 신호 `RGRRGRGGYGYG` · 바코드 `P2-01` 로 `mission_submit` 이 만든 것이다. 운영자가 다른 신호 · 바코드를 내면 [problem/README.md](problem/README.md) 의 명령으로 다시 만든다.

## 7. 정지 방법

| 방법 | 효과 | 풀기 |
|---|---|---|
| **RC-100 버튼 1** (게임패드는 `estop_button` 6 ★) | 호스트 E-stop 잠금 → `/e_stop` true, twist_mux 전부 막음, 펌웨어 ESTOP (바퀴 0 · 팔 HOLD), 미션 PAUSED | 원인 해제 + 리셋 (RC-100 **2 + 3**, 게임패드 `reset_button` 7 ★) → REARM 으로 팔 HOLD 해제 → `/mission/start` 로 현재 단계부터 |
| 조이스틱 끊김 · 전원 꺼짐 | heartbeat 0.5 s 끊기면 자동 E-stop 잠금 | 위와 같음. ⚠ **RC-100 전원이 꺼져 있으면 1 을 눌러도 E-stop 이 걸리지 않는다** (drive_demo README 0 장) |
| `/mission/stop` (조이스틱 BACK 8 또는 `ros2 service call /mission/stop std_srvs/srv/Trigger`) | 진행 중 goal 모두 취소 → IDLE (소프트웨어 정지) | 새 `/mission/start` |
| **Ctrl+C** (launch 터미널) | 노드 종료 → 펌웨어 명령 200 ms 무수신: 바퀴 0, 팔은 그 자리 HOLD (토크 유지) | 다시 launch → 리셋 |
| 자동 정지 | TFmini 0.25 m (obstacle lock) · 위치 잃음 (Nav2 만 lock) · 진단 ERROR (PAUSED) | 원인 해제 |

- **물리 비상정지 버튼은 달려 있지 않다** (프로토콜 비트 0x04 는 있음). 모든 정지는 소프트웨어 · 무선이므로 리모컨 담당이 1 번 위에 엄지를 두고 로봇만 본다.
- 끝낼 때: `/mission/stop` → Ctrl+C → 전원을 끄기 전 팔을 STOW 로 접는다 (다음 켤 때 범위 검사). STOW 로 접을 때 팔레트 첫 줄 왼쪽 칸에 닿는 문제 주의 (#277).

## 8. 주요 파일 경로

| 무엇 | 경로 (`workspace/ros2_ws/src/` 아래) |
|---|---|
| 전체 launch · 센서 | `bringup/launch/bringup.launch.py` · `sensors.launch.py`, EKF `bringup/config/ekf.yaml` |
| 미션 | `mission/src/mission_node.cpp`, `mission/include/mission/*.hpp`, 설정 `mission/config/mission.yaml` · `mission_plan.yaml` |
| 계획기 | `mission_planner/` (`allocation.py` · `mission_lines.py` · `score.py` · `submission.py`), 자세표 `config/approach_poses.yaml`, 지도 `maps/` |
| 주행 · 팔 | `control/src/drive_node.cpp` · `manipulator_node.cpp`, 상태기 `control/include/control/fsm.hpp`, 설정 `control/config/manipulator.yaml` · `robot_geometry.yaml` |
| 인식 | `perception/` (설정 `config/perception.yaml` · `lidar.yaml` · `tfmini.yaml`), `perception_survey/` |
| 위치 추정 · 주행 | `localization/launch/localization.launch.py` · `config/amcl.yaml`, `navigation/launch/navigation.launch.py` |
| 하드웨어 | `opencr_hardware/` (URDF `urdf/robot.urdf.xacro` · `opencr.ros2_control.xacro`, E-stop `include/opencr_hardware/estop_latch.hpp`, `config/twist_mux.yaml` · `teleop.yaml` · `drive_belt.yaml`) |
| 펌웨어 | `opencr_firmware/` (`config.h`, `tools/build_firmware.sh`) |
| 인터페이스 | `robot_interfaces/{msg,srv,action}`, `include/robot_interfaces/names.hpp` |
| 문서 · 도구 | `workspace/docs/interfaces.md` · `frames_tf.md` · `mission_stage_test.md`, `workspace/tools/drive_demo/` |

## 9. 재현 시 주의사항

| 항목 | 내용 | 근거 |
|---|---|---|
| 벨트 기어비 | 실물 v7 벨트 = 모터 44T → 바퀴 22T, `wheel_gear` 2.0 (`drive:=belt`). v6 (1.5) 로 빌드하면 오도메트리 33 % 어긋남. v7 은 토크가 줄어 실물 회전이 명령의 66 ~ 80 % | `robot.urdf.xacro` 36 행, drive_demo README 0 장 |
| 팔 펌웨어 v6 · 관절 1 π 보정 | 관절 1 모터가 모델과 180° 돌아 달려 있어 팔 펌웨어 v6 (`ARM_J1_FLIP 1`) 이 π 돌려 보고 · 명령 (#316). 팔 보드가 v5 면 호스트가 활성화를 거부. 주행 보드는 v5 그대로 | `opencr_firmware/config.h`, `opencr_protocol.hpp` |
| STOW 값 | 관절 1 2.200 rad (모델 각도, 실물 1420 tick 자리). 11-15 ~ 16 사이 1.8715 · −0.9633 등으로 여러 번 바뀌었으니 **설치된 빌드의 범위가 3425 ~ 3539 인지** 확인 (`grep arm_standby install/opencr_hardware/share/.../opencr.ros2_control.xacro`) | `manipulator.yaml` STOW 주석, drive_demo README 단계 1 |
| 포트 | OpenCR 두 대는 이름이 고정되지 않음 (`ports` 후보 + role). 라이다 udev 는 허브 칸 기준 — 칸을 바꾸면 이름이 사라짐 (그때는 `lidar_front_port:=/dev/serial/by-path/…`). D435 는 시리얼로 구분 | `99-sensors.rules`, drive_demo README |
| ROS_DOMAIN_ID | Jetson 경기 실행의 값은 기록에 없음 (**미확인**, 지정하지 않으면 0). 같은 망의 노트북 RViz 감시 · 다른 시뮬과 겹치지 않게 경기 전 고정할 것. 시뮬 도구는 따로 씀 (drive_demo `SIM_DOMAIN` 기본 73, `scan_view.sh` 74, 인식 모의 시험 87) | `drive_demo/jetson_check.sh` · `run_demo.sh` · `scan_view.sh`, `requirements.txt` |
| 시작 부하 | Jetson 에서 lifecycle 응답이 사라져 Nav2 노드가 inactive 로 남는 일 (#276). `stage_start` · `nav2_delay` (10 s) 로 늦춰 띄움. 남으면 `ros2 lifecycle set /<노드> activate` | `bringup.launch.py`, drive_demo README |
| 위치 감시 | loc_monitor 가 실물에서 예민해 거짓 LOST → Nav2 잠김 가능. 시연에서는 끔 | drive_demo README |
| 팔 정리 대기 | 리셋 뒤 STOW → DRIVE 정리가 끝나기 전 (또는 팔이 STOW 와 어긋나 정리가 거부된 상태, 11-16 은 0.23 rad) 에 `/mission/start` 를 누르면 manipulator 가 goal 을 거절해 observe FAILED. 준비 표에는 이 항목이 없음 | `manipulator_node.cpp` 1558 ~ 1620 행, 경기 로그 |
| 배터리 | 보드마다 LiPo 3S. 11.1 V 아래 경고 (부저), 10.5 V 위험 → 미션 정지 | `interfaces.md` `/diagnostics` |

## 10. 제출물 지도

상태는 최종 제출 (2026-11-17, 태그 `project2-submit`) 기준이다.

| 경로 | 내용 | 상태 |
|---|---|---|
| [problem/](problem/) | 문제 패키지: [problem.md](problem/problem.md) · [signals.json](problem/signals.json) · [placement.csv](problem/placement.csv) · [allocation_answers.md](problem/allocation_answers.md) / [.json](problem/allocation_answers.json) · [verification.md](problem/verification.md) · [dice_targets.csv](problem/dice_targets.csv) (+ [rules.md](problem/rules.md) · [review.md](problem/review.md)) | 있음 — 팀 실물 퍽 등록표 + 공개 연습 Q1 신호 · 바코드. `review.md` 다른 팀 검토 기록 비어 있음 |
| [docs/architecture.md](docs/architecture.md) | ROS 2 노드 · 토픽 · 메시지, TF, 단계 · 완료 조건, 안전 · 오류 상태도 | 있음 |
| [docs/issues.md](docs/issues.md) | GitHub 이슈 목록 (발견자 · 수정 담당 · 검토자 · 처리 현황) | 있음 |
| [docs/contributions.md](docs/contributions.md) | 팀원별 기여 · 근거 (0 절 저장소 기록 요약, 1 ~ 4 절 사람별 구현 · 검증 · 리뷰 · 설명) + [contributions_log.md](docs/contributions_log.md) | 있음 |
| [results/test-results.csv](results/test-results.csv) | 시험 결과 표 (실물 · 모의 · 단위 구분, 성공 · 실패 · 미확인 · 미수행) | 있음 |
| [results/signal.json](results/signal.json) · [results/allocation.json](results/allocation.json) | 실물 경기 신호 · 할당 기록 | 있음 — 실물은 observe FAILED 로 **미관측 · 미계산** 이라고 적혀 있음. 시뮬 Q1 계획기 출력은 [results/sim_plan_q1/](results/sim_plan_q1/) |
| [results/real_20261116/](results/real_20261116/) | 경기일 실물 기록: `mission_log/` (전체 미션 2 회, observe FAILED) · `launch_logs/` · `drive_runs/` (drive_demo 3 회, 최고 4/10 구간) | 있음 |
| [results/official_scoring.md](results/official_scoring.md) | 공식 1 · 2 회차 100 점 판정표 · 근거 | 있음 |
| [results/reproduction.md](results/reproduction.md) | 재현 시험 기록 (실행자 · 날짜 · 장비 · 커밋 · 결과) + [reproduction_20261115/](results/reproduction_20261115/) | 있음 |
| [challenge/B/](challenge/B/) | 도전 B: 퍽 할당 알고리즘 비교 (순차 vs 탐색, `report.md` · `cases.json` · `expected.json` · `results.csv`) | 있음 |

## 10. 경기 영상 · 발표 자료

| 항목 | 위치 |
|---|---|
| 공식 경기 영상 (2026-11-16, 공식 1 · 2 회차) | [Google Drive](https://drive.google.com/file/d/1GLdKyPdjwA_KjS184HGRhl4CGkyOpnHt/view?usp=drive_link) — 회차 대응 · 점수는 [results/official_scoring.md](results/official_scoring.md) |
| 발표 자료 | [docs/presentation.pdf](docs/presentation.pdf) (10 쪽) |
| 모의 시험 영상 (온창범, 20 개) | [results/sim_videos/](results/sim_videos/) — 주행 방식 V1 ~ V9 · MuJoCo M 시험 · 무게추 비교 |
| 개인 보고서 (온창범) | [docs/contributions/온창범_보고서.md](docs/contributions/온창범_보고서.md) |

### 제출 태그

| 태그 | 가리키는 것 |
|---|---|
| `project2-submit` | 최종 제출 (`main`) |
| `project2-problem-submit` | 문제 패키지 (`problem/`) 고정 버전 (`main`) |
