# waffle_4wd_description_261028

팀 4륜 차체 + RM-X52 팔 로봇 URDF (2026-10-28 확정본) 와 Gazebo Sim 센서 · 구동 설정.
ROS 2 Lyrical · Gazebo Sim 10.5 (ros-lyrical-ros-gz) 에서 시험했다.

- 받은 `waffle_4wd_description_261027_1711` 에서 요청 1~7 을 반영했다 (아래 표, 자세한 내용 · 가정은 `REVISION_261028.md`).
- 센서 사양 (실물 값 · 출처) 과 Gazebo 값은 **`config/sensors.yaml` 한 파일**에 있다.
- `urdf/` · `meshes/` · `config/gz_bridge.yaml` · `config/camera_info_fix.yaml` 은 `tools/` 스크립트가 만든 결과다. 직접 고치지 말 것.

## 빠른 시작

```bash
# 작업 공간 src 에 이 폴더를 넣고
cd ~/<작업공간> && colcon build --symlink-install --packages-select waffle_4wd_description_261028
source install/setup.bash

ros2 launch waffle_4wd_description_261028 display.launch.py      # RViz + 관절 슬라이더 (Gazebo 없이)
ros2 launch waffle_4wd_description_261028 gazebo.launch.py       # Gazebo 창 + 주사위 경기장 출발 자리
ros2 launch waffle_4wd_description_261028 gazebo.launch.py rviz:=true          # + RViz (스캔 · 앞 · 뒤 D435 · TFmini)
ros2 launch waffle_4wd_description_261028 gazebo.launch.py gui:=false          # 창 없이 (센서는 렌더함)
```

Gazebo launch 인자: `world` (기본 `worlds/dice_mission.sdf`), `gui`, `rviz`, `collision` (`light` 기본 / `full`), `x` `y` `yaw` (기본 0.734, 0.950, π = 신호등을 봄).

### 키보드로 움직이기

Gazebo 를 띄운 뒤 **다른 터미널**에서 (키 입력을 받으므로 터미널에서 직접 실행):

```bash
source ~/<작업공간>/install/setup.bash
ros2 run waffle_4wd_description_261028 keyboard_teleop.py
```

| 키 | 동작 |
|---|---|
| `w` / `x` | 앞 · 뒤 속도 +/− 0.02 m/s (최대 0.36) |
| `a` / `d` | 왼쪽 · 오른쪽 회전 +/− 0.2 rad/s (최대 2.0) |
| `s` · 스페이스 | 멈춤 |
| `1`~`5` | 팔 관절 고르기 (joint1~5) |
| `i` / `k` | 고른 관절 +/− 0.1 rad (URDF 한계 안) |
| `o` / `l` | 그리퍼 열기 · 닫기 (손가락 +/− 5 mm, −11 ~ +20 mm) |
| `h` | 팔 전부 0 (영 자세) |
| `q` · Ctrl-C | 멈춤 명령을 보내고 끝 |

영 자세에서 팔이 앞으로 0.41 m 나와 있어 출발 자리에서 0.2 m 넘게 앞으로 가면 팔이 신호등 · 벽에 닿는다 (닿으면 관절이 처지고 손가락이 안 움직임).
팔을 들거나 (`2` 고르고 `k`, `3` 고르고 `i` 등) 뒤로 가서 시험할 것.

토픽으로 직접:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}"   # 앞으로 0.1 m/s
ros2 topic pub --once /arm/joint2/cmd_pos std_msgs/msg/Float64 "{data: 0.5}"                  # 팔 관절 (rad) · 손가락 (m)
```

필요한 패키지: `ros-lyrical-ros-gz` (gz-sim 10.5, ros_gz_bridge, ros_gz_sim), `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`.

## 받은 패키지 (261027_1711) 에서 바뀐 것

| # | 바뀐 것 |
|---|---|
| 1 | 팔 = `~/GIT/arm_bench/urdf/arm_bench.urdf` 의 J2 아래 (J1 서보는 차체 것). 이름은 팀 기준: `joint1~5`, `gripper_roll` (그리퍼 개폐 모터), `gripper_left/right_joint`, `gripper_roll_link`, `arm_camera_link`, `tool_link` |
| 2 | J1 브라켓 · 서보를 앞으로 9 mm: 브라켓 구멍 4 개가 상판 (TB3 와플판) 구멍과 맞음. J1 축 = (0.096, 0, 0.131) m |
| 3 | 옛 LiDAR 2 개 + 브라켓 → LDS-03 2 개를 상판 밑면에 뒤집어 앞 · 뒤 가운데 (`lds_front/rear`, 스캔 frame `laser_front/rear`) |
| 4 | TFmini Plus + 브라켓 v2 를 하판 윗면 앞 가운데 (`tfmini_bracket`, 측정 frame `tfmini_link`) |
| 5 | Jetson Orin Nano 개발자 키트 (NVIDIA STEP 모델, `jetson_link`), 단자는 뒤 |
| 6 | 하판 배치: Jetson (뒤) · OpenCR 2 (`opencr_left/right_link`) · 배터리 2 (`battery_left/right_link`) 를 판 위에 바로, 사이 1 mm |
| 7 | 상판 · 하판 사이에 떠 있던 기판과 조각 지움 |
| + | Gazebo: 센서 14 개 · 구동 (DiffDrive) · 팔 위치 제어 · 실제 위치 (OdometryPublisher), IMU frame 2 개 (`imu_left/right_link`) |
| + | 팀 `dev` 이름 · 구조 (2026-10-28): 바퀴 `wheel_{left,right}_{front,rear}_joint` · `_link`, 그리퍼 모터 `gripper_roll`, **루트 = `base_link`** (`base_footprint` 는 그 아래), `imu_link` (두 IMU 가운데) |
| 10 | **퍽 보관통 3 개 + 레일 2 개 → 퍽 플레이트** (2026-10-30): `puck_plate_link`, 원점 = 팔 joint1 축 · 상판 윗면 (0.096, 0, 0.111) m. 컵 12 개 (팔 축 반지름 90 · 142 · 191 mm), 퍽 앉는 턱 상판 위 2 · 12 · 24 mm |

전체 질량 3.8613 kg (받은 것 4.2207 kg). 링크 43 · 관절 42 (움직이는 관절 12).

팀 `Lv2_Cartlidar_Assignment` `origin/dev` 의 `controllers_4wd.yaml` · `opencr.ros2_control.xacro` 가 쓰는 관절 이름 (바퀴 4, joint1~5, gripper_roll) 과
팀 코드가 쓰는 frame (`base_link`, `base_footprint`, `imu_link`, `laser_front/rear`, `tfmini_link`, `arm_base_link`, `tool_link`, `arm_camera_link`,
`top_camera_{front,rear}_color_optical_frame`) 이 모두 있다 (2026-10-28 대조). robot_state_publisher 가 루트 관성 KDL 경고를 내지만 동작에는 영향 없음.

## 센서 (실물 사양 · Gazebo 값)

| 센서 | 실물 사양 (출처는 `config/sensors.yaml`) | Gazebo | 토픽 · frame |
|---|---|---|---|
| LDS-03 x 2 (ROBOTIS, = COIN-D4) | 0.05~12 m, 360° · 0.9° (400 점), 10 Hz, ±10 mm (~1 m) · ±20 mm, UART 115200 bps 8N1 (USB2LDS), 5 V 240~320 mA, 42 g | gpu_lidar 400 점 10 Hz, 잡음 σ 10 mm | `/scan_front` · `/scan_rear` (LaserScan), `laser_front` · `laser_rear` (뒤집힘 roll π), 스캔면 바닥 위 82.2 mm |
| TFmini Plus (Benewake) | 0.1~12 m (흰 판), ±5 cm (~6 m) · ±1 %, 시야각 3.6°, 100 Hz (1~1000), UART 115200 bps 8N1, 5 V ≤110 mA, 12 g | 광선 1 개 gpu_lidar 100 Hz, 잡음 σ 25 mm → Range 변환 노드 | `/tfmini/range` (Range), `tfmini_link` (렌즈 가운데, +x 앞), 바닥 위 41.5 mm |
| D435 x 2 (Intel RealSense) | 깊이 화각 87° × 58°, 컬러 69° × 42°, 기준선 50 mm, 깊이 최소 0.28 m (최대 해상도) ~ 10 m | 640 × 480 30 Hz: 컬러 (fx 616.8) + 컬러에 맞춘 깊이 + IR 좌우 (fx 386.6) | `/top_camera_{front,rear}/` + `color/image_raw` · `color/camera_info` · `aligned_depth_to_color/image_raw` · `aligned_depth_to_color/camera_info` · `depth/color/points` · `infra1/image_rect_raw` · `infra1/camera_info` · `infra2/…` |
| 팔 카메라 (Arducam UC-684 B0201) | 대각 화각 120° (판매 정보) | 1280 × 720 30 Hz, 가로 113° 바늘구멍 (가정) | `/arm_camera/image_raw` · `/arm_camera/camera_info`, `arm_camera_optical_frame` |
| IMU x 2 (OpenCR 1.0, MPU9250) | 자이로 · 가속도 · 지자기 3 축 | 100 Hz, 잡음 σ 0.0017 rad/s · 0.03 m/s² (가정) | `/imu_left/data` · `/imu_right/data`, `imu_left_link` · `imu_right_link`. 평균은 팀 코드에서 → `/imu/data` frame `imu_link` (두 IMU 가운데, 센서 없음) |

구동 · 기타 토픽 (Gazebo):

| 토픽 | 타입 | 주기 | 비고 |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/TwistStamped | — | DiffDrive (바퀴 4, 좌우 간격 0.317 m = 바퀴 폭 가운데 사이, 반지름 0.033 m, 최대 0.363 m/s) |
| `/odom` · `/tf` (odom → base_link) | nav_msgs/Odometry | 50 Hz | 바퀴 기준 (미끄럼 모름). 팀 EKF 를 같이 띄우면 odom → base_link TF 가 겹치니 둘 중 하나만 |
| `/ground_truth/odom` | nav_msgs/Odometry | 50 Hz | 시뮬레이터가 아는 실제 위치 (world 기준), SLAM 채점용 |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | 바퀴 4 + 팔 8 |
| `/arm/<관절>/cmd_pos` | std_msgs/Float64 | — | joint1~5, gripper_roll, gripper_left/right_joint (속도 명령 방식, 최대 1 rad/s · 0.05 m/s) |
| `/clock` | rosgraph_msgs/Clock | | 모든 노드 `use_sim_time: true` |

카메라 토픽은 **구독할 때만 Gazebo 가 렌더**한다 (lazy). 안 쓰는 카메라는 시뮬 속도를 깎지 않는다.

## 시험 결과 (2026-10-28, RTX 5060 Laptop 8 GB, 창 없이)

- 주기 (sim 시간): LiDAR 10.0 · IMU 100 · TFmini 100 · odom · joint_states 50 Hz.
- LiDAR 출발 자리 정면 0.506~0.514 m (신호등 몸체까지 계산 0.515), 후면 0.242~0.251 m (칸막이 벽까지 0.256). TFmini 0.46 m.
- 전진 0.1 m/s × 3 s → 실제 0.300 m, odom 0.300 m.
- 제자리 회전 0.5 rad/s × 3 s (86°) → **실제 70°, odom 86°** (바퀴 4 개 미끄럼). 회전은 IMU 를 같이 써야 함 (팀 EKF).
- 팔 joint1 0.8 · joint2 0.5 rad 명령 → 2 s 뒤 그 값, 그동안 차체 이동 0.3 mm. 손가락 −11 ↔ +20 mm (URDF 한계값) 왕복 5 회 정상.
- 키보드 조작 (가상 터미널로 키 입력): `w` x5 → 0.10 m/s 전진, `a` x3 → 회전, `2` + `i` x5 → joint2 0.5 rad, `o` x4 → 손가락 20 mm, `q` 정상 종료.
- 뒤 D435: 컬러 · 깊이 (100 % 유효, 중앙값 0.73 m) · IR · camera_info 모두 나옴.
- 실시간 비율: 카메라 구독 없음 1.0 (collision:=light) / 0.76 (full), 카메라 9 개 모두 구독 약 0.5.

## 알아 둘 점 (미확정 포함)

- **D435 컬러 · 깊이 Gazebo 센서는 광축으로 8 mm 앞**: URDF optical frame 이 D435 몸체 메시 앞면보다 0.2 ~ 0.6 mm 안쪽이라 (실물 렌즈가 유리 뒤에 있는 것과 같음, frame 은 맞음)
  Gazebo 깊이 센서가 막혀서 (전부 −inf). `scripts/camera_info_fix.py` 가 이를 고쳐 내므로 **받는 쪽은 더할 것이 없다** (2026-10-29):
  - 깊이 영상 `<D435>/aligned_depth_to_color/image_raw`: Gazebo 원본 (`/gz/...`) + 0.008 m = optical frame 기준 (같은 장 비교 평균 +8.000 mm 확인).
  - 점구름 `<D435>/depth/color/points`: Gazebo 원본은 센서 축 (x 앞 · y 왼 · z 위) 인데 frame_id 는 optical frame 이라 RViz · TF 에서 90° 돌아가 보였음 (10-28 공유본 버그)
    -> optical 축 (x 오른 · y 아래 · z 앞) 으로 바꾸고 z + 0.008 m. TF 로 base_link 에 옮기면 바닥 점이 바닥 높이에 옴.
  - 주기 (창 없이, reliable 로 받을 때): 원본 30 Hz -> 고친 출력 약 25 Hz. 수 MB 메시지를 best-effort 로 받으면 조각 손실로 원본 · 고친 출력 모두 크게 떨어지니 reliable 로 받을 것.
  - 실물 데이터에는 해당 없음 (실물 드라이버는 frame 기준으로 낸다).
- **camera_info**: Gazebo 가 같은 링크의 카메라마다 엉뚱한 기준선을 P 에 넣어서 `scripts/camera_info_fix.py` 가 실물 드라이버처럼 고친다 (infra2 만 Tx = −fx × 0.050).
- D435 640 × 480 내부 값 (fx 615 · 383 대표값), Arducam 해상도 · 화각, IMU 칩 자리 · 축 방향, 잡음 값은 가정 → 실물 camera_info · 실측으로 바꿀 것.
- LDS-03 앞뒤 자리는 임시 (귀 구멍이 판 구멍과 안 맞아 테이프 고정 검토 중), 테이프 두께 0 mm 로 둠. LDS-03 0° 방향 = 모델 +x 가정.
- 스캔면 (82.2 mm) 이 70 mm 박스보다 높아 LiDAR 로 박스는 안 보임 (사용자 결정).
- `collision:=light` 는 차체 볼록 메시 128 개를 상자 4 개로, 퍽 플레이트 볼록 조각 154 개를 상자 15 개 (컵도 꽉 찬 상자) 로 바꾼다. 퍽 · 플레이트 시험은 `collision:=full`.
- Gazebo 는 팔 · 손가락 관절 물리 한계를 1 mm · 0.001 rad 넓혀 띄운다 (xacro `limit_margin:=0.001`): 한계 끝에 닿으면 Gazebo (DART) 에서
  속도 명령으로 빠져나오지 못하고 걸려서 (2026-10-28 시험). URDF 한계값을 그대로 명령해도 된다. `.urdf` 의 한계값은 그대로.
- 질량: 옮기거나 뺀 부품 (J1 묶음, 옛 브라켓, Jetson · OpenCR · 배터리 자리 상자, 뜬 기판) 이 받은 base_link 질량에 들어 있었다고 봄.
- 팀 저장소 값과 다른 것 (팀 저장소는 고치지 않음): `robot.urdf.xacro` 의 `arm_base_link` (0.087, 0, 0.111) → (0.096, 0, 0.131), `manipulator.yaml` `ik.base_height` → 0.0595,
  `tfmini_link` 는 집게 쪽 → 차체 앞, `laser_front/rear` (±0.15, 0, 0.08) → (±0.1089, 0, 0.0755) roll π, `perception/config/tfmini.yaml` 주석 "URDF 에 없음" → 이제 있음.
- **팀과 아직 맞춰야 할 값 (실물 기준 미정, 지금은 받은 2안 CAD 값)**:
  - 바퀴 좌우 간격 (2026-10-29 확인, 팀과 같음): 좌우 바퀴 폭 가운데 사이 0.317 m (바퀴 y ±0.1585) = 팀 `controllers_4wd.yaml` 0.317 m.
    URDF 바퀴 관절 원점 (y ±0.1546, 간격 0.3092) 은 바퀴 폭 가운데보다 3.9 mm 안쪽이라 diff_drive 값으로 쓰지 않는다. 팀 `robot.urdf.xacro` 는 아직 1안 값 (0.2744).
  - 바퀴 구동: 실물은 벨트 1:1.5 증속 (팀원 확인 2026-10-29, 이 URDF 관절 한계 10.99 rad/s · 1.8 N·m 와 같음) ↔ 팀 펌웨어 `config.h` `WHEEL_GEAR 1.0` (직결) 은 고쳐야 함.
  - D435 각도: URDF 35.2° 아래 ↔ 팀 문서 "35° → 10° (브라켓 v2)". 10° 로 바꾸면 frame 과 차체 메시 안 카메라 모양을 같이 돌려야 함.
  - 팔 관절 한계: 이 URDF = 펌웨어 config.h (A안, 팀 `feature/arm-from-dev` 와 같음) ↔ `origin/dev` 의 ros2_control 은 예전 값 (joint1 ±2.8 등).

## 파일

| 경로 | 내용 |
|---|---|
| `urdf/waffle_4wd.urdf` | 확정 URDF (Gazebo 태그 포함, RViz · robot_state_publisher 는 무시) |
| `urdf/waffle_4wd.urdf.xacro` | 같은 내용 + 인자 `prefix`, `collision` (full / light) |
| `config/sensors.yaml` | 센서 사양 · 출처 · Gazebo 값 (여기를 고친 뒤 tools 로 다시 만듦) |
| `config/gz_bridge.yaml` · `config/camera_info_fix.yaml` | Gazebo ↔ ROS 브리지 목록, camera_info · D435 깊이 · 점구름 고침 목록 (생성) |
| `launch/display.launch.py` · `launch/gazebo.launch.py` | RViz 보기 · Gazebo |
| `rviz/display.rviz` · `rviz/gazebo.rviz` | RViz 설정 |
| `worlds/dice_mission.sdf` | 주사위 기억 로봇 경기장 (발제 v5) |
| `scripts/` | `keyboard_teleop.py` (키보드 조작), `tfmini_scan_to_range.py`, `camera_info_fix.py` (Gazebo 보조 노드: camera_info 기준선, D435 깊이 +8 mm, 점구름 축) |
| `meshes/` | `main` (차체 · 바퀴 · OpenCR), `arm` (arm_bench), `parts` (LDS-03 · TFmini · 브라켓 · Jetson · 퍽 플레이트 + 충돌 조각 `puck_plate_col/`), `collision`, `puck_pallet.stl` (퍽 플레이트 원본, mm, 사용자 제공) |
| `tools/` | 만드는 스크립트 (아래) |
| `REVISION_261028.md` | 바꾼 것 · 가정 · 확인 기록 |

## 다시 만들기

```bash
freecadcmd -c "exec(open('tools/make_part_meshes.py').read())"   # 센서 · 부품 메시 (약 1 분, 바꿀 때만)
~/.venvs/puckplate/bin/python tools/make_puck_plate.py          # 퍽 플레이트 메시 · 충돌 조각 (플레이트 STL 을 바꿀 때만, REVISION 10 절)
python3 tools/make_revision.py                                   # URDF · 메시 · Gazebo 설정 (팔 원본이 다른 곳이면 --arm 경로)
```

읽는 원본 (이 PC 경로): 받은 패키지 `~/Downloads/urdf_261027_1711/waffle_4wd_description_261027_1711`, 팔 `~/GIT/arm_bench/urdf/arm_bench.urdf`,
TFmini 브라켓 `~/Desktop/CartRider/tfmini_plus_bracket_v2_print.stl`, Jetson STEP `~/Downloads/Jetson_Orin_Nano_DevKit_3D_STEP_Model/`. 없으면 다시 만들 수 없지만 결과 파일은 그대로 쓸 수 있다.
