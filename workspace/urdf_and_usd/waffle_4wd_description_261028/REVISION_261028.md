# 261028 리비전 변경 기록 (2026-10-28)

원본: `~/Downloads/urdf_261027_1711/waffle_4wd_description_261027_1711/` (바꾸지 않음) + `~/GIT/arm_bench/urdf/arm_bench.urdf` (바꾸지 않음).
만드는 도구: `tools/make_revision.py` — `urdf/` · `meshes/` · `config/joint_state.yaml` · `launch/` · `rviz/` 를 다시 만든다. 직접 고치지 말고 스크립트를 고칠 것.

```bash
freecadcmd -c "exec(open('tools/make_part_meshes.py').read())"   # 센서 · 부품 메시 (LDS-03 · TFmini · 브라켓 · Jetson, 약 1 분, 바꿀 때만)
python3 tools/make_revision.py                    # 원본 두 곳 + 부품 메시에서 다시 생성
cd ~/workspaces/urdf_view_ws && colcon build --symlink-install --packages-select waffle_4wd_description_261028
ros2 launch waffle_4wd_description_261028 display.launch.py
```

## 바꾼 것 (사용자 2026-10-28 요청 1번 + 2번의 J1 자리)

### 1. 팔 → arm_bench (J1 서보 제외, J2 아래)

- 뺀 것: `arm_link_1~5`, `gripper_left/right_link`, `camera_link`, `camera_optical_frame` 과 관절 `RMX52_*` · 손가락 · 카메라.
- 붙인 것: arm_bench 의 `link2` 아래 전부 (겉모양 STL 22 개 = `meshes/arm/`, 충돌 상자, 질량 · 관성, 관절 한계 = 펌웨어 `config.h`).
- J1 서보는 차체 메시 것을 그대로 쓴다 (arm_bench `link1` 과 크기 46.4 × 28.4 × 40.4 mm, J2 축 높이 = 서보 바닥 + 76.5 mm 같음).
- 이름 = 팀 Lv2 기준:

| arm_bench | 이 패키지 |
|---|---|
| joint1~4 | joint1~4 |
| wrist_roll_joint | joint5 |
| gripper_motor_joint | gripper_roll (그리퍼 모터 각, 빈 링크 gripper_motor_link) — 처음엔 gripper_joint 로 했다가 9 절에서 고침 |
| gripper_left/right_joint | 그대로 (따로 움직임, mimic 없음) |
| gripper_link | gripper_roll_link |
| gripper_camera_link · _optical_frame | arm_camera_link · arm_camera_optical_frame |
| end_effector_link | tool_link |
| arm_base_link (link1 + x 12 mm) | arm_base_link = base_link 아래 고정, J1 축 위 (96, 0, 131) mm. joint1 원점 = 여기 |

- 영 자세 위치 (base_link, mm): joint2 (96, 0, 190.5) · joint3 (120, 0, 318.5) · joint4 (244, 0, 318.5) · joint5 (272, 0, 318.5) · tool_link (411, 0, 318.5).
  받은 URDF 와 비교하면 J2~J4 는 +9 mm (2번) 만 다르고, 손목 롤은 J4 에서 68.4 → 28 mm (arm_bench = 실물 구조).
- 전체 질량 4.2207 → 4.2403 kg (팔 739.0 → 758.6 g).

### 2. J1 브라켓 · 서보 · 나사 x +9 mm

- 차체 메시 (`meshes/main/base_link.dae`) 의 이어진 덩어리 124 개 중 J1 묶음 19 개 (서보 1, 브라켓 2, 나사 16) 의 꼭짓점만 옮겼다. 다른 꼭짓점 · 삼각형은 원본과 같다 (스크립트가 다시 읽어 확인).
- 충돌 상자 3 개 (`static_431` · `432` 브라켓, `static_435` 서보) 도 +9 mm.
- 까닭: 브라켓 밑 구멍 4 개 (Ø2.6, x 64 · 88, y ±29.85) 가 상판 (TB3 와플판) 구멍과 3.0 · 5.85 mm 어긋남. +9 mm 에서 판 구멍 (73 · 97, ±30) 과 0.15 mm.
- J1 축 x 87 → 96 mm.

### 3. 옛 LiDAR → LDS-03 2 개 (요청 3번, 사용자 2026-10-28 결정: 판 밑면에 바로 · 팀 Lv2 이름 · 귀 구멍은 실측 뒤)

- 뺀 것: `lidar_front` · `lidar_rear` (LiDAR + 브라켓 한 몸, 각 143.9 g) 와 `lidar_*_scan` frame,
  차체 메시에서 상판 밑에 붙은 옛 브라켓 판 2 장 (53 × 79 × 3 mm, x ±92~145, 삼각형 1608 개), 충돌 상자 `static_443` · `444`.
  판을 지운 뒤 상판 밑면에 빈 곳이 생기지 않음을 아래에서 본 그림으로 확인 (판 밑면은 원래 따로 있었음).
- 더한 것: `lds_front` · `lds_rear` (몸체, 42 g, 겉모양 `meshes/parts/lds03.stl`, 충돌 = 몸체 상자 + 머리 원통) 와
  스캔 frame `laser_front` · `laser_rear` (= 광학 중심, 몸체와 같은 방향).
- 모양: `tools/make_part_meshes.py` (FreeCAD). 치수는 사용자 Fusion 모델 `COIN_D4_OPTICAL_ORIGIN_REFERENCE.f3d` 안의 형상 값
  (몸체 38.2 × 38.2 × 21.1, 머리 Ø35.3 × 13.3, 귀 3 개 구멍 Ø3.2 = (−21.8, 0) · (9, ±22), 원점 = 광학 중심 = 밑면에서 26.5 위).
- 자리: 상판 밑면 (z 102 mm) 에 장착면을 붙이고 뒤집음 (roll π). 광학 중심 = (±108.9, 0, 75.5) mm, 바닥 위 82.2 mm. 뒤는 yaw π.
  x 는 **임시**: 몸체 바깥 면 = 상판 곧은 끝 (x ±128). 귀 구멍 3 개가 판 구멍과 동시에 맞는 가운데 자리가 f3d 치수로는 없어서 실물 실측 뒤 다시 정함.
- 스캔면 (바닥 위 82.2 mm) 에서 0.4 m 안 가림 (차체 메시 + 다른 LDS 단면): 앞 LDS 는 로봇 앞쪽 약 −98° ~ +98° 가 트임 (옆 · 뒤 123° 가림),
  뒤 LDS 는 뒤쪽 약 116° ~ 244° 가 트임 (151° 가림). 옆쪽 (98~116°, 244~262°) 은 둘 다 옆벽 · 바퀴 브라켓에 가림.
  70 mm 박스는 스캔면보다 낮아 안 보임 (사용자 결정: 그대로).

### 4. TFmini Plus + 브라켓 v2 (요청 4번, 사용자 2026-10-28 결정: 하판 윗면 앞 가운데, 테이프 · 케이블 타이 고정, 벽이 앞 · OpenCR 과 안 겹치게)

- 브라켓: 사용자 `~/Desktop/CartRider/tfmini_plus_bracket_v2_print.stl` (43 × 44 × 32 mm, 바닥판 3.5 + 벽 5 + 받침 턱) 을 그대로 씀 → `meshes/parts/tfmini_plus_bracket_v2.stl` (평행 이동 · m 단위만 바꿈).
- 방향: 벽이 앞, TFmini 는 벽 바깥 면에 뒷면을 대고 앞을 봄, 바닥판 · 받침 턱은 벽 뒤.
  매뉴얼 Figure 4 의 뒷면 M2 대각 (뒤에서 보면 왼쪽 위 · 오른쪽 아래) 이 벽 구멍 (STL y 8 z 17 · y 36 z 28.5) 과 맞는 건 이 방향뿐 (벽 안쪽에 대면 대각이 엇갈림).
  처음 넣은 방향 (벽 뒤, TFmini 벽 안쪽) 은 사용자 지시로 바꿈.
- TFmini 모양: 매뉴얼 Figure 4 (35 × 18.5 × 21, R2, 뒷면 M2 대각 28 × 11.5, 케이블 위), 12 g.
- 링크: `tfmini_bracket` (base_link 에 고정, 원점 = 벽 바깥 면 가운데 밑면) → `tfmini_link` (TFmini 몸체, 원점 = 렌즈 가운데, +x 측정 방향).
- 자리 (요청 6 B안 뒤): 바닥판 뒤 끝 = OpenCR 앞 끝 (x 75.3) + 1 mm → 바닥판 x 76.3~119.3 (y ±22, z 12~15.5), 벽 x 114.3~119.3, TFmini x 119.3~140.3.
  → 렌즈 가운데 (140.3, 0, 34.75) mm, 바닥 위 41.5 mm. TFmini 앞면은 판 끝 (128) 보다 12.3 mm 앞 = 그 높이에서 로봇 맨 앞.
  (요청 6 전에는 벽 뒷면 = 받은 OpenCR 앞 끝 + 1 mm 로 바닥판이 OpenCR 밑 1.5 mm 틈으로 들어갔음 - B안에서 OpenCR 을 판 위로 내려 바꿈.)
- 바닥판 일부는 하판 앞 가운데 반원 홈 (판 이음매, 가장 안쪽 x 107.5) 위에 걸침.
- 브라켓 바닥 구멍 (TFmini 가운데에서 y ±6 · ±18) 은 하판 구멍과 안 맞음 → 테이프 · 케이블 타이 (사용자 결정).
- 빔 (반각 1.8°, 이론 확산각 6° 면 3°) 은 1 m 안에서 차체에 안 막힘. 바닥에 닿는 거리 1.32 m (1.8°) / 0.79 m (3°) — 그보다 먼 값은 바닥이 섞일 수 있음.

### 5. Jetson Orin Nano 개발자 키트 (요청 5번, 사용자 2026-10-28 결정: 지금 자리 · 단자는 뒤, 자리는 요청 6 에서 다시)

- 뺀 것: 차체 메시의 Jetson 자리 상자 (103 × 90.5 × 34.8 mm, 삼각형 12 개) 와 같은 크기 충돌 상자 (`static_440`).
- 더한 것: `jetson_link` (base_link 에 고정, 원점 = 받침 다리 바닥 가운데, 단자 변 = −x).
  모양 = NVIDIA STEP `~/Downloads/Jetson_Orin_Nano_DevKit_3D_STEP_Model/P3766-P3768SKU4-P3767ENVELOPE.stp` (부품 1393 개, 외곽 103.4 × 91.1 × 34.9).
  겉모양 `meshes/parts/jetson_orin_nano.stl` 은 부피 20 mm³ 이상 부품 56 개를 0.5 mm 편차로 (부피 99 %, 삼각형 24.6 만).
  충돌 = 상자 2 개 (보드 · 단자 / 방열판 · 팬). 질량 176 g, 무게중심 · 관성은 부품 1393 개에 고르게 퍼졌다고 보고 계산.
- 자리 (요청 6 B안): 뒷면 = 판 뒤 끝 (x −128) + 2 mm, y 0, 받침 다리 바닥 = 하판 윗면 (z 12). 단자가 뒤라 103 mm 변이 좌우 → base_link x −126.0~−34.9, y ±51.7, z 12~46.9.
  단자 쪽 일부는 하판 뒤 가운데 반원 홈 위. 뒤 LDS-03 (아래 끝 z 67.6) 과는 20.7 mm 띄어 있음.

### 6. 하판 배치 B안 (요청 6번, 사용자 2026-10-28 결정, 추가 부품 없음)

- OpenCR 2 장 (보드 + 작은 부품) · 배터리 2 개 (LB-012 상자) 를 차체 메시에서 떼어 링크로: `opencr_left_link` · `opencr_right_link` (겉모양 `meshes/main/opencr_*.stl` = 떼어 낸 메시),
  `battery_left_link` · `battery_right_link` (상자). 원점 = 밑면 가운데. 충돌 = 받은 충돌 상자와 같은 크기 (OpenCR 109.2 × 75 × 19, 배터리 88 × 35 × 26), 질량 OpenCR 60 g · 배터리 106 g.
  받은 충돌 상자 4 개 (`static_438` · `439` · `441` · `442`) 는 base_link 에서 뺌.
- 모두 하판 위에 바로 (z 12, 테이프 · 케이블 타이, 받침 없음). 자리 규칙 (`tools/make_revision.py` `layout()`, 부품 사이 1 mm):

| 부품 | base_link x | y | z | 규칙 |
|---|---|---|---|---|
| Jetson (단자 뒤) | −126.0 ~ −34.9 | ±51.7 | 12 ~ 46.9 | 뒷면 = 판 뒤 끝 + 2 mm |
| OpenCR 왼쪽 · 오른쪽 | −33.9 ~ 75.3 | 3.5 ~ 78.5 / −78.5 ~ −3.5 | 12 ~ 31 | Jetson 앞 + 1 mm, y 는 받은 자리 그대로 |
| 배터리 왼쪽 · 오른쪽 | −122.9 ~ −34.9 | 52.7 ~ 87.7 / −87.7 ~ −52.7 | 12 ~ 38 | Jetson 옆 + 1 mm, 앞 끝 = Jetson 앞 끝 (옆벽 91 까지 3.3 mm) |
| TFmini 브라켓 + TFmini | 76.3 ~ 140.3 | ±22 | 12 ~ 47 | 바닥판 뒤 끝 = OpenCR 앞 + 1 mm |

- OpenCR 은 받은 자리보다 37.3 mm 뒤 · 5 mm 아래 (받은 모델은 판 위 5 mm 에 떠 있었음), 배터리는 18.9 mm 뒤 · y 3.2 mm 바깥 · 1 mm 아래.
- 충돌 형상 겹침 검사 (영 자세, 외곽 상자): 하판 부품끼리 · 옆벽 · LDS-03 · TFmini 와 겹침 없음.
  남은 겹침은 J1 서보 충돌 상자 (혼 포함) 와 팔 link2 브라켓 상자 4.4 mm (혼 위에 브라켓이 얹히는 자리, 요청 1 · 2 때부터).

### 7. 뜬 부품 지우기 (요청 7번, 2026-10-28)

- 상판 · 하판 사이에 아무것에도 닿지 않고 떠 있던 기판 (USB-C + 8 핀 커넥터, 30.4 × 18 × 7.7 mm, x 35~65 · z 76.8~84.5, 상판 밑 14.5 mm) 을
  차체 메시 (삼각형 47222) 와 충돌 상자 (`static_430`) 에서 지움. 요청 6 에서 하판에 더할 부품 없음 (사용자) 이라 다시 넣지 않음.
- 삼각형 2 개 이하 조각 35 개 (보관함 레일 근처 x −130.5~−116, z 187~303, 부피 없음) 지움.
- 받은 모델에서 OpenCR 이 하판 위 5 mm 에 떠 있던 것은 요청 6 에서 판 위로 내림.
- 지운 뒤 차체 메시 덩어리 81 개, 다른 덩어리와 외곽 상자로 0.5 mm 넘게 떨어진 덩어리 0, 조각 0.

### 8. Gazebo Sim (2026-10-28, 사용자 결정: D435 640 × 480 30 Hz + IR 스테레오, DiffDrive, IMU 2 개 평균은 팀 코드)

- `config/sensors.yaml` (센서 실물 사양 · 출처 · Gazebo 값) → `tools/gazebo_ext.py` 가 URDF 에 `<gazebo>` 태그를 붙이고 `config/gz_bridge.yaml` · `config/camera_info_fix.yaml` 을 만든다.
- 센서 14 개: LDS-03 2 (gpu_lidar), TFmini (광선 1 개), D435 마다 컬러 camera · depth_camera · IR camera 2, 팔 카메라, IMU 2 (`imu_left/right_link` 새 frame, OpenCR 링크 위 5 mm).
- 플러그인: DiffDrive (바퀴 4), JointStatePublisher, OdometryPublisher (`/ground_truth/odom`), 팔 관절 8 개 JointPositionController (속도 명령 방식).
- `gripper_motor_link` 에 1 g 질량 (질량 없는 움직이는 링크는 sdformat 이 관절째 버림).
- 시험 중 고친 것:
  - 팔 위치 제어를 힘 (PID) → 속도 명령으로: 손목 · 그리퍼처럼 관성이 작은 관절이 1 ms 스텝에서 떨려 (속도 2~6 rad/s) 차체가 명령 없이 밀리고 시뮬이 0.02 배까지 느려짐.
  - 팔 최대 속도 1 rad/s · 0.05 m/s (URDF 4.8 rad/s 로는 반작용으로 차체 27 mm 밀림 → 0.3 mm).
  - D435 rgbd_camera 대신 camera + depth_camera: rgbd_camera 는 이 로봇에서 장면을 못 봄 (따로 둔 시험 월드에서는 정상).
  - D435 컬러 · 깊이 센서를 광축으로 8 mm 앞: frame 이 몸체 메시 안이라 깊이가 전부 −inf.
  - camera_info Tx: Gazebo 가 같은 링크 카메라의 y 거리를 기준선으로 넣음 (앞 IR2 65 mm, 뒤 컬러 80 mm) → `scripts/camera_info_fix.py` 가 실물 드라이버 값으로.
  - 카메라 브리지는 lazy, camera_info 고침 노드도 출력 구독이 있을 때만 Gazebo 쪽을 구독 → 안 쓰는 카메라는 렌더 안 함.
  - xacro 인자 `collision:=light`: base_link 볼록 메시 423 개 → 묶음별 상자 12 개 (Gazebo 기본).
  - xacro 인자 `limit_margin` (gazebo.launch.py 는 0.001): 관절 물리 한계를 넓힘. 손가락이 한계 끝 (−11 mm) 에 닿으면 속도 명령으로 빠져나오지 못하고 걸림
    (한계 안쪽 0.5 mm 왕복은 5/5 정상, 한계값 그대로는 걸림 → limit_margin 뒤 한계값 그대로 5/5 정상).
- 키보드 조작 `scripts/keyboard_teleop.py` (주행 + 팔 + 그리퍼, 설치 없이). teleop_twist_keyboard 는 설치돼 있지 않음 (apt 필요).
- 시험 결과는 README.md "시험 결과".

### 9. 팀 dev 이름 · 구조 맞춤 (2026-10-28, 사용자 "이름 맞춤 4 개 다 고쳐줘")

공유 전 점검에서 팀 `Lv2_Cartlidar_Assignment` `origin/dev` (83aa48f) 와 대조해 고침:
- 바퀴: `Wheel_Revolute_{Front,Rear}_{Left,Right}` → `wheel_{left,right}_{front,rear}_joint`, 링크 `wheel_*_link` (팀 controllers_4wd.yaml · ros2_control).
- 그리퍼 개폐 모터: `gripper_joint` → `gripper_roll` (팀 arm_controller 6 번째 관절 · ros2_control 칸 5). 요청 1 때 이름 표를 팀 컨트롤러 설정까지 확인하지 않고 정해 생긴 것.
  팀 `gripper_joint` 는 따로 (평행 그리퍼, 액추에이터 미정) 라 이 URDF 에는 없음.
- 루트: `base_footprint` → `base_link` (`base_footprint_joint`: base_link → base_footprint, z −6.75 mm). 팀 EKF · diff_drive 가 odom → base_link.
  Gazebo DiffDrive · OdometryPublisher 기준 frame 도 base_link, 생성 높이 z 0.012. robot_state_publisher 가 루트 관성 KDL 경고 (동작 영향 없음).
- `imu_link`: 두 IMU frame 가운데 (base_link 기준 (20.7, 0, 17) mm), 센서 없음 (팀 imu_sensor_broadcaster frame_id).
- 확인: 팀 관절 10 · frame 11 모두 있음, check_urdf · xacro · gz sdf 통과, Gazebo (odom → base_link → base_footprint TF, base_link 높이 6.75 mm, 전진 0.299 m, gripper_roll 0.5 rad).

### 10. 퍽 보관통 → 퍽 플레이트 (2026-10-30, 사용자 "기존 퍽 스토리지 3 개를 puck plate 로 대체")

- 지움: 차체 메시의 퍽 보관통 3 개 (통마다 위 · 아래 2 덩어리, z 0.18 ~ 0.405 m) + 보관함 레일 2 개 (z 0.175 ~ 0.205 · 0.285 ~ 0.315 m),
  덩어리 8 개 · 삼각형 121 440 개, 그 충돌 볼록 메시 묶음 8 개 (295 개: part022 · 025 · 026 · 029 · 140 · 141 · rail_hi_new · rail_lo_new).
  걸쳐 있는 다른 부품 없음 (덩어리 외곽 상자가 범위 안에 다 들어오는 것만, 2026-10-30 확인). base_link 충돌 444 → 149 개.
- 더함: 링크 `puck_plate_link` + 고정 관절 `puck_plate_joint` (부모 base_link, xyz (0.096, 0, 0.111) = 팔 joint1 축 x, y · 상판 윗면, 회전 없음).
  원본 `meshes/puck_pallet.stl` (사용자 제공, mm) 의 좌표 원점이 팔 축이라 그대로 둠: 앞 끝 반원 홈 = 원점 중심 반지름 58 mm (link2 반지름 23.5 mm 가 돎),
  컵 12 개 = 원점 중심 반지름 90 · 142 · 191 mm 원 위에서 원점 쪽을 봄 (팔 도달 0.263 m 안).
  플레이트 외곽 base_link x −129.3 ~ 76.5, y ±105.8, z 111 ~ 150 mm. 뒤 기둥 앞면 (x ≈ −135) 까지 5.7 mm.
- 퍽 자리: 퍽 앉는 턱 = 플레이트 밑면 위 24 (바깥 줄) · 12 (가운데) · 2 mm (안쪽), 벽 윗면 39 · 27 · 17 mm, 사각 구멍 31.9 ~ 32.5 mm (퍽 지름 30 · 30 x 30 상자 퍽 모두 들어감).
  퍽 윗면 base_link z 195 · 183 · 173 mm (보관통 입구 z ≈ 400 보다 205 mm 이상 낮음).
- 만드는 도구: `tools/make_puck_plate.py` (trimesh · shapely · manifold3d · mapbox_earcut, 아래) → `meshes/parts/puck_plate.{stl,json}` + `puck_plate_col/*.stl`.
  make_revision.py 는 JSON 의 원본 sha1 이 `meshes/puck_pallet.stl` 과 다르면 멈춘다 (도구를 다시 돌릴 것).
- 충돌: `collision:=full` (와 `.urdf`) = 볼록 조각 154 개: 바닥판 z 0 ~ 3 mm 74 개 (격자 구멍은 메움, 안쪽 줄 컵 자리만 z 2 ~ 3 비움) + 컵 80 개
  (컵마다 네 면 x 턱 아래 · 위, 이어진 덩어리별 볼록 껍질). 조각마다 꼭짓점 60 개 이하 (PhysX 한계 64 에서 줄이며 부풀리지 않게, 원래 꼭짓점의 부분 집합).
  컵 자리 퍽 (원기둥 · 상자, 0.3 mm 줄임) 과 겹침 0. 바깥 모서리 몇 곳은 리브까지 감싸 실물보다 조금 큼 (컵 안 · 위는 실물과 같음).
  `collision:=light` = 바닥판 상자 3 + 컵마다 꽉 찬 상자 12 (보관통 때처럼 퍽 시험은 full).
- 질량: 플레이트 103.4 g (부피 151.6 cm³ × PLA 1.24 × 채움 0.55, 가정). 지운 보관통 · 레일 283.0 g (부피 × 같은 가정) 을 base_link 에서 뺌
  → base_link 2554.8 → 2271.8 g, 전체 4.0409 → 3.8613 kg. 링크 43 · 관절 42.

```bash
python3 -m venv ~/.venvs/puckplate && ~/.venvs/puckplate/bin/pip install trimesh shapely manifold3d mapbox_earcut rtree scipy
~/.venvs/puckplate/bin/python tools/make_puck_plate.py           # 플레이트 STL 이 바뀌었을 때만 (약 30 초)
python3 tools/make_revision.py --arm <arm_bench.urdf>            # 2026-10-30: ~/git/PJ/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench/urdf/arm_bench.urdf
```

확인 (2026-10-30):
- 생성기 고치기 전 결과가 지금 패키지와 바이트까지 같음 (재현 확인) → 고친 뒤 바뀐 것은 base_link (충돌 · 질량) · puck_plate_link · puck_plate_joint 뿐, Gazebo 태그 · gz_bridge.yaml 같음.
- xacro full · light, `check_urdf` 통과 (ROS 2 Lyrical 컨테이너), `gz sdf -p` 통과.
- 팔과의 거리 (퍽 12 개 다 찼을 때): 주행 자세 24 mm, 전원 켤 때 자세 (stow) 13 mm, 겹침 없음. 플레이트 위 z 111 ~ 400 mm 에 차체 부품 없음.
- Gazebo Sim 10.5 (ROS 없이, 팔 관절 고정 · 영 자세): 퍽 12 개를 컵 2 mm 위에서 놓음 → 12/12 턱에 앉음 (높이 · 수평 오차 0.00 mm, 기울기 0°), 차체 높이 6.7 mm.
- Isaac (USD full): PhysX 광선으로 컵 12 개의 턱 높이 24 · 12 · 2 mm, 벽까지 15.9 mm (퍽 15 + 틈 0.9) 확인.
  `isaac/puck_plate_check.py`: 로봇이 자리 잡은 뒤 턱 위 2 mm 에서 놓음 -> 원기둥 12/12 · 상자 12/12 앉음 (높이 0.00, 수평 0.6 mm 안, 기울기 0°).
  `run.sh check` 가벼운 판 (주행 · stow) · 정밀 판 모두 값 정상, 질량 3.8613 kg, 이전과 다른 것은 질량과 1 mm 안 차이뿐.
  처음 시험에서 퍽이 튕겨 나간 원인 2 개: 볼록 조각 꼭짓점 > 64 (PhysX 가 줄이며 부풀림 -> 60 개 이하로 고침), 시험 스크립트의 쿼터니언 순서 (Isaac Lab 3.0 = x, y, z, w).
- 플레이트 실물 나사 구멍 4 개 (원본 STL: d5.0 (−92.8, ±65.5), d3.4 (−219.3, ±65.5) mm) 가 상판 구멍과 2.8 ~ 3.8 mm 어긋남 (y ±65.5 vs 판 ±68) - 실물 조립 때 다시 뚫거나 플레이트 구멍을 옮길 것.

## 가정

- base_link 무게중심 · 관성: 옮긴 J1 덩어리를 질량 110.6 g 으로 보고 다시 계산. 서보 94.0 g (arm_bench link1), 브라켓 = PLA 1.24 g/cm³ × 채움 0.55 (원본 REVISION_261027_1711 의 긴 브라켓 값), 나사 = 강철 7.85 g/cm³.
  받은 base_link 질량에 J1 서보가 들어 있다고 봤다 (미확정).
- 옛 LiDAR 브라켓 판 2 장은 base_link 질량에 들어 있었다고 보고 각 7.4 g (윗면 넓이 × 3 mm × PLA × 0.55) 을 뺐다 (미확정).
- Jetson 자리 상자 176 g, OpenCR 60 g × 2, 배터리 106 g × 2 도 base_link 질량에 들어 있었다고 보고 받은 자리에서 뺐다 (미확정).
  base_link 3079.9 → 2554.8 g. OpenCR · 배터리 관성은 외곽 상자에 고르게 퍼졌다고 봄.
- 뜬 기판 질량 2.3 g = 메시 부피 1.25 cm³ × FR-4 1.85 g/cm³ 를 base_link 에서 뺌 (가정). 전체 질량 4.0422 → 4.0399 kg.
- LDS-03 의 0° (스캔 frame +x) = f3d 의 +x 로 가정 (실물 스캔으로 확인 필요). 질량 42 g (ROBOTIS), 관성은 형상에 고르게 퍼졌다고 봄.
- TFmini 브라켓 질량 8.5 g = 부피 12.4 cm³ × PLA 1.24 × 채움 0.55 (가정). TFmini 12 g (데이터시트, 1안 기록). 측정 기준점 = 렌즈 앞면 가운데로 둠. 케이블 출구 크기 · 자리는 그림 비율.

## 확인 (2026-10-28)

- `check_urdf` 성공 (링크 41 · 관절 40, 움직이는 관절 12). xacro (collision:=full) 펼친 결과와 `.urdf` 같음. 전체 질량 4.0409 kg (gripper_motor_link 1 g 포함).
- `gz sdf -p` 변환 오류 없음 (경고는 `gz_frame_id` 가 SDF 사양에 없다는 것뿐, gz-sensors 가 읽음). Gazebo 시험은 README.md.
- 새 메시에서 브라켓 밑 구멍 4 개 = (73, ±29.85) · (97, ±29.85), 가장 가까운 상판 구멍까지 0.15 mm.
- LDS-03 외곽 (앞 x 82.8~128, y ±26.3, z 67.6~102) 이 상판 말고 다른 차체 부품과 겹치지 않음. 뒤도 같음.
- RViz + joint_state_publisher_gui 실행: 로그 오류 · 경고 없음, `/joint_states` 관절 12 개, TF base_link → laser_front = (0.109, 0, 0.075) roll π, → tfmini_link = (0.140, 0, 0.035), → jetson_link = (−0.080, 0, 0.012), → opencr_left_link = (0.021, 0.041, 0.012), → battery_right_link = (−0.079, −0.070, 0.012).

## 하지 않은 것

- 퍽 플레이트 (2026-10-30): Isaac RL 과제 (`isaac/rl/` 옮기기 · 넣기) 의 목표는 아직 보관통 3 개 (store_starts.json 의 C · P · N 통) 기준 - 컵 12 개로 바꿔야 함.

- 퍽을 집는 물리 시험 (collision:=full, 마찰 · 그리퍼 힘), 경기장 전체 주행 (SLAM · nav2), 실물 센서 값으로 맞추기 (camera_info · 잡음).
- LDS-03 고정 방법 (귀 구멍 실측 또는 양면 테이프) 결정 뒤 앞뒤 자리 확정.
- 테이프 두께 (LiDAR · TFmini 모두 0 mm 로 둠, 테이프 정해지면 반영).
- 원본의 `config/frame_overrides*`, `docs/transforms.md`, `simulation_metadata.json`, `robot_data.yaml` 은 옛 팔 기준이라 복사하지 않음.
- 팀 저장소 반영 (고치지 않음): `opencr_hardware/urdf/robot.urdf.xacro` 의 `arm_base_link` (0.087, 0, 0.111) → 이 패키지는 (0.096, 0, 0.131),
  `control/config/manipulator.yaml` `ik.base_height` (TODO 0.08) → arm_base_link 에서 joint2 까지 0.0595,
  `robot.urdf.xacro` 의 `tfmini_link` 는 집게 (`gripper_roll_link`) 아래 → 이 패키지는 차체 앞 (요청 4), `laser_front/rear` 자리 (±0.15, 0, 0.08) → (±0.1089, 0, 0.0755) roll π.
