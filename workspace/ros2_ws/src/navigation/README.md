# navigation

주행(Nav2) 패키지다. 지도 위 목표 자세까지 경로를 계획하고(Smac 2D) 따라간다(RotationShim + MPPI). 최종 속도 명령은 `/cmd_vel_nav`(twist_mux `nav` 입력) 하나다.
위치 추정(AMCL)은 `localization` launch 를 `use_amcl_node:=true` 로 불러 띄운다. AMCL 설정은 `localization/config/amcl.yaml`.

## 실행

```bash
ros2 launch navigation navigation.launch.py                                     # 단독: AMCL 포함, 지도 · loc_monitor 는 localization 기본값 (Isaac SLAM 지도). 실물은 아래 "실물 첫 실행"
ros2 launch navigation navigation.launch.py use_localization:=false            # map → odom · /map 을 다른 곳에서 낼 때
ros2 launch navigation navigation.launch.py use_collision_monitor:=false       # smoother 가 바로 cmd_vel_out 으로
ros2 launch navigation navigation.launch.py max_wz:=0.8 rotate_wz:=0.6         # 속도 한계만 바꿔서 (아래 "속도 한계 인자")
```

속도 한계 인자 (비우면 `config/nav2_params.yaml` 값 그대로):

| 인자 | 바꾸는 곳 | 기본 (YAML) |
|---|---|---|
| `max_vx` [m/s] | MPPI `vx_max` · smoother `max_velocity[0]` | 0.2 |
| `max_wz` [rad/s] | MPPI `wz_max` · behavior `max_rotational_vel` · smoother `max · min_velocity[2]` | 1.0 |
| `rotate_wz` [rad/s] | RotationShim `rotate_to_heading_angular_vel` (≤ `max_wz`, `max_wz` 만 주면 그보다 크지 않게) | 0.8 |

- 같은 한계가 4 노드 6 곳에 있어 YAML 을 고치면 함께 맞춰야 한다. 인자로 주면 launch 가 한 번에 덮어쓴다 (시작 때 로그 한 줄).
- `bringup.launch.py` · `gazebo_sim.launch.py` 에 줘도 된다 (명령 줄 인자는 포함된 launch 까지 간다).
- 실물 상한: diff_drive vx 0.34 (`opencr_hardware/config/drive_belt.yaml`) · wz **1.0** (`controllers_4wd.yaml`, 2026-11-12 2.0 → 1.0). 넘게 주면 알림만 내고, 실물에서는 바퀴 앞에서 잘린다.

- 먼저 떠 있어야 하는 것: `bringup`(robot_state_publisher, EKF `odom → base_link` · `/odometry/filtered`, twist_mux), 라이다 `/scan_front` · `/scan_rear`.
- localization 인자(`start_x/y/yaw`, `map`, `use_loc_monitor` 등)는 이 launch 에 그대로 준다. 출발 자세 기본값 = base_link (0.69365, 0.95, π).
- 목표 보내기: `drive_node` 가 `navigate_to_pose` · `spin` action 을 부른다. 손으로는
  `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.734, y: 0.2915}, orientation: {w: 1.0}}}}"`
### 실물 첫 실행 (★ 실물에서 아직 안 돌려 봄, 2026-11-09 · 11-12 갱신)

물리 비상정지 버튼은 없다. 조이스틱 E-stop 은 소프트웨어 정지라 첫 주행은 전원 스위치에 손이 닿는 곳에서 한다.
**주행 시험 (리모컨) 과 Nav2 자동 주행은 따로 띄운다** (2026-11-13 결정). 리모컨은 로봇이 제대로 움직이는지 보는 시험용이다.

- 리모컨 주행 시험 (Nav2 없음, RC-100 — 수신기는 주행 OpenCR 의 BT-410, `opencr_firmware/README.md`):
  `ros2 launch opencr_hardware base_bringup.launch.py rc100:=true obstacle_lock:=false`
  → 켠 뒤 **2 + 3 을 한 번** 눌러 E-stop 을 푼다 (켤 때는 늘 잠김). 방향 버튼 = 앞뒤 0.1 m/s · 회전 0.5 rad/s (누르는 동안만), 1 = E-stop.
  로그 `rc100_joy` 의 "누른 동안 가장 긴 패킷 간격" 이 누른 시간과 비슷하면 리모컨이 패킷을 되풀이하지 않는 것 → `rc100_hold_timeout:=0`.
- Nav2 자동 주행: 리모컨 · 조이스틱 없이 **`use_teleop:=false`** (아래 명령). 안 주면 조이스틱 heartbeat 가 없어 E-stop 이 계속 잠겨 바퀴가 안 돈다.
  이때는 조이스틱 E-stop 이 없으므로 멈출 때는 launch 를 Ctrl+C 하거나 전원을 내린다.

준비

- **켜는 순서: 두 OpenCR 의 배터리 먼저 → USB (Jetson).** USB 가 먼저 켜지면 부팅 점검에서 모터 ping 이 실패해 `MOTOR_CONFIG | FAULT` 가 걸리고,
  보드를 다시 켜야만 풀린다 (그때는 두 보드 RESET).
- **팔이 대기 자세여야 켜진다.** 호스트는 켤 때 팔 6 축 tick 이 대기 범위 (`opencr_hardware/urdf/opencr.ros2_control.xacro` `arm_standby_min/max_tick`) 밖이면
  하드웨어 활성화를 거부한다 (#166). 켜지면 첫 명령이 팔 토크를 켜고 그 자리를 유지한다.
  그리퍼는 #243 (2026-11-12) 부터 펌웨어 범위 전체를 허용한다 (예전: 손으로 다 닫아도 2568 ~ 2589 < 최소 2590 이라 막힘).
  리모컨 시험 (base_bringup 만) 에서는 팔이 대기 자세 그대로 토크로 버틴다. bringup 전체를 켜면 manipulator 가 주행 자세 (DRIVE) 로 편다 (#144).
- **회전 속도 (증속 벨트).** 모터 = 로봇 회전 × 3.20. 2026-11-11 바닥 시험에서 제자리 회전 모터 3.0 (로봇 0.94 rad/s) 에 뒤 바퀴 속도가 출렁였고 7.0 에서 벨트가 빠졌다.
  diff_drive 상한은 1.0 rad/s (모터 3.2). 첫 주행은 `max_wz:=0.8 rotate_wz:=0.6` 부터 ★ (벨트 장력을 손본 뒤 다시 정함).
- 로봇을 출발 구역 가운데에 서쪽 벽(−X)을 보게 놓는다. AMCL 은 출발 자세 (0.69365, 0.95, yaw π) 에서 시작한다. 다른 곳이면 `start_x:= start_y:= start_yaw:=`.
- 켠 뒤 2 s 는 움직이지 않는다. `scan_body_mask` 가 0.5 ~ 1.9 s 정지 스캔으로 차체 가림을 배운다.
- `/dev/lidar_front` · `/dev/lidar_rear` · `/dev/tfmini` 는 udev 규칙(`perception/udev/99-sensors.rules`)의 시리얼이 아직 자리표시라 안 생긴다.
  라이다는 `ls -l /dev/serial/by-path/` 의 이름을 `lidar_front_port:=` · `lidar_rear_port:=` 로 준다 (꽂은 USB 구멍 기준. ttyUSB 번호는 켤 때마다 바뀔 수 있음).
  bringup 은 이 인자를 `sensors.launch.py` 에 따로 넘기지 않지만, 명령 줄 인자는 포함된 launch 까지 가므로 그대로 쓰인다 (2026-11-12 시험 launch 로 확인).
  TFmini 가 없으면 obstacle_stop 이 "입력 없음" 정지를 계속 내서 Nav2 가 막히므로 `use_tfmini:=false` (TFmini 정지 없음, collision_monitor 만 남음).

```bash
ros2 launch bringup bringup.launch.py use_localization:=true use_nav2:=true use_teleop:=false \
  use_loc_monitor:=false use_loc_lock:=false use_loc_gate:=false \
  use_perception:=false use_d435:=false use_arm_camera:=false use_tfmini:=false \
  lidar_front_port:=/dev/serial/by-path/<앞> lidar_rear_port:=/dev/serial/by-path/<뒤> \
  max_wz:=0.8 rotate_wz:=0.6
```

- 지도는 bringup 기본 = 도면 SDF 지도(`mission_planner/maps`, #163).
- `use_loc_monitor:=false`: 잔차 문턱 0.034 m 는 모의에서 정한 값이라 Gazebo 에서도 거짓 LOST 가 났다(위).
  끄면 `/localization/status` · `/localization/lock` 이 없으므로 lock(끊겨도 잠김 → Nav2 계속 막힘) · gate(60 s 기다리다 실패, #121)도 같이 끈다.
- 카메라 3 대는 Nav2 가 안 쓰므로 끈다 (costmap 은 `/scan_merged` 만).

확인 (다른 터미널, 차례로)

```bash
ros2 topic hz /scan_front ; ros2 topic hz /scan_rear ; ros2 topic hz /scan_merged   # 라이다 두 대 · 합친 것
ros2 run tf2_ros tf2_echo map base_link        # 출발 자세 근처 (0.69, 0.95, yaw 180°)
```

1. `lifecycle_manager_navigation ... Managed nodes are active` 를 기다린다.
2. (리모컨 주행 시험을 먼저 끝낸 뒤) 짧은 목표로 0.3 m 앞 → 90° 회전을 시켜 보고 `tf2_echo map base_link` 가 같은 쪽으로 따라오는지 본다
   (x 가 줄고, yaw 180° → −90°). 반대로 가면 멈추고 바퀴 방향 · IMU 부호부터 본다.
3. 짧은 목표 (남쪽 0.5 m, 남쪽 봄) → 출발 자세로 돌아오기:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.734, y: 0.45}, orientation: {z: -0.7071, w: 0.7071}}}}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.69365, y: 0.95}, orientation: {z: 1.0, w: 0.0}}}}"
```

4. 된 뒤 `use_loc_monitor:=true` 만 켜고 (lock · gate 는 false 그대로) 같은 목표를 돌며 `ros2 topic echo /localization/status` 를 본다.
   거짓 LOST 가 없으면 lock · gate 를 기본(켬)으로 둔다.


### Gazebo 시험 (waffle_4wd_description)

`waffle_4wd_description` 은 `ros2_ws/src` 밖에 있으므로 한 번 같이 빌드한다.

```bash
cd workspace/ros2_ws
colcon build --symlink-install --base-paths src ../urdf_and_usd/waffle_4wd_description --packages-select waffle_4wd_description robot_interfaces mission_planner localization navigation control bringup   # control · bringup = EKF 설정 (use_ekf)
source install/setup.bash
ros2 launch navigation gazebo_sim.launch.py               # 출발 자세로 Gazebo → 팔 주행 자세 → 5 s 뒤 Nav2 + AMCL
ros2 launch navigation gazebo_sim.launch.py gui:=false    # 창 없이. nav:=false 면 Gazebo · 팔 자세만, x:= y:= yaw:= 로 다른 시작 자세
```

- `lifecycle_manager_navigation ... Managed nodes are active` 가 나오면 목표를 보낸다.
- 지도 기본값은 Gazebo 경기장과 같은 SDF 에서 만든 `mission_planner/maps` (`/map` = `map_amcl`, `/map_objects` = `map_nav`, 0.025 m).
  localization 기본 Isaac SLAM 지도(`slam_walls_001`)는 동쪽으로 갈수록 SDF 와 어긋나(x 3 ~ 4.5 m 에서 8 ~ 9 cm, 북동 구석 15 cm 이상)
  AMCL 이 한쪽으로 치우치다 미로 칸을 착각해 1 m 넘어갔다. `map:=` · `objects_map:=` 로 바꿀 수 있다.
- Gazebo 팔 관절은 0 (영 자세) 으로 시작한다. 팀 기본 시작 자세는 주행 자세(`isaac/robot_params.json` `drive`)라서 launch 가 `/arm/<관절>/cmd_pos` 로 보낸다. 영 자세로 달리면 팔이 걸린다.
- Gazebo 는 twist_mux · ros2_control 을 거치지 않는다(diff_drive 플러그인이 `/cmd_vel` 을 받음). 그래서 `cmd_vel_out:=/cmd_vel`.
- odom (`use_ekf`, 기본 true, #151): 실물 bringup ⑥ 처럼 `odom_icr` + EKF (`bringup/config/ekf.yaml`, 바퀴 vx · vy + IMU yaw rate) 가
  `/odometry/filtered` 와 `odom → base_link` 를 낸다 (Gazebo diff_drive TF 는 `odom_tf:=false` 로 끔). Nav2 `odom_topic:=/odometry/filtered`.
  Gazebo 바퀴 odom 은 제자리 회전을 실제보다 21 ~ 24 % 크게 센다 (스키드 미끄러짐, 2026-11-13 참값 비교, 직진은 오차 없음) → 그대로 쓰면
  AMCL yaw 가 회전마다 13 ~ 19° 어긋나 LOST 가 잦았다. EKF 는 참값과 1.000. `use_ekf:=false` 면 예전처럼 diff_drive `/odom` · TF.
- `use_loc_monitor` 기본 true (2026-11-13, #151). `loc_map` 도 같은 SDF 지도(`map_amcl`)가 기본이다 — Isaac `slam_walls` 를 보면 잔차가 커져 FAILED(#133).
  예전(바퀴 odom)에는 회전 중 LOST:residual 이 잦아 끄고 있었다 (2026-11-09 LOST 15 → FAILED, 11-12 LOC=1 LOST 17 · DONE 522 s).
  `use_ekf` 뒤 Q1 `LOC=1` 미션: LOST 0, 잔차 p95 중앙 0.024 m (문턱 0.034), DONE 507 s. 끄려면 `use_loc_monitor:=false`.
- Gazebo 는 카메라 3 대 렌더링으로 실시간의 약 0.5 ~ 0.66 배로 돈다. 모든 노드 `use_sim_time:=true`.

## 속도 명령 · TF

```
controller_server (RotationShim → MPPI) ─┐
behavior_server (spin · backup · wait) ──┴→ /nav2/cmd_vel_raw → velocity_smoother → /nav2/cmd_vel_smoothed
                                            → collision_monitor → cmd_vel_out (기본 /cmd_vel_nav → twist_mux)
```

- Nav2 기본 launch(nav2_bringup)는 중간 출력을 `cmd_vel_nav` 로 쓴다. 그대로 쓰면 twist_mux 가 smoother · collision_monitor 를 건너뛴 명령을 받으므로 이 launch 를 쓴다.
- 속도 명령은 TwistStamped(Nav2 1.5 기본 `enable_stamped_cmd_vel: true`, twist_mux `use_stamped: true`).

| TF | 내는 노드 |
|---|---|
| `map → odom` | `amcl` (localization launch, `use_amcl_node:=true`) |
| `odom → base_link` | 실물 = 팀 EKF, Gazebo = diff_drive 플러그인 |

## 노드 · 설정 (`config/nav2_params.yaml`, ★ = 임시값)

| 노드 | 설정 | 비고 |
|---|---|---|
| `planner_server` | Smac 2D (0.01 지도를 0.02 로 줄여 계산), `allow_unknown: false` | 시작 때 `Inflation layer ... not set sufficiently` ERROR 는 Smac 2D 가 늘 내는 알림 |
| `controller_server` | RotationShim (경로 방향과 45° 넘게 다르면 먼저 제자리 회전, 도착 뒤 목표 방향 회전) → MPPI(DiffDrive) | Nav2 1.4+ 는 MPPI 값을 `FollowPath.primary_controller` 아래에 둔다 |
| | vx −0.1 ~ 0.2 ★ · wz ±1.0 ★ (launch 인자 `max_vx` · `max_wz` · `rotate_wz` 로 덮어씀) · 가속 1.5, 도착 xy 0.05 ★ · yaw 0.1 ★ | drive.yaml `navigate.xy_goal_tolerance` 도 0.05 (#114) |
| `local_costmap` · `global_costmap` | 정적 층 2 장(`/map` 벽, `/map_objects` 라이다에 안 보이는 물체) + `/scan_merged` + inflation 0.30 ★ | footprint = B안 외곽(앞 0.1328 · 뒤 −0.2135 · 반폭 0.1676) ★ + 주행 자세 팔 (앞 0.25) |
| `behavior_server` | spin · backup · wait (기본 BT 와 drive_node 가 쓰는 것) | 최소 회전 0.3 rad/s ★ |
| `velocity_smoother` · `collision_monitor` | MPPI 와 같은 한계 / 차체가 1.2 s 안에 닿을 속도면 줄임 | |
| `bt_navigator` | 기본 BT 복사본 `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml` (플래너 복구 조건에 목표 · 시작 칸 막힘 205 · 206 추가, #135) | 플러그인 이름(GridBased · FollowPath · general_goal_checker · progress_checker · PathHandler)을 바꾸지 말 것 |

Gazebo 시험(2026-11-04)에서 기본값에서 바꾼 것과 이유:

| 설정 | 값 | 기본값으로 생긴 일 |
|---|---|---|
| costmap `use_maximum` | true (costmap 단위) | Nav2 1.5 는 정적 층마다 받지 않는다. 덮어쓰기라 objects 지도의 "모름"이 벽 · 빈칸을 덮어 목표가 막힘으로 계산됨 |
| local `track_unknown_space` | false | 움직이는 창에서는 큰 값으로 합칠 때 "모름"(255)이 이김 |
| MPPI `CostCritic.near_collision_cost` | 254 | 미로 통로(약 0.53 m)는 차체(폭 0.355) 가장자리가 inscribed(장애물 0.14 m 안)에 닿을 수밖에 없어 멈춤 |
| MPPI Path · GoalAngle critic `threshold_to_consider` | 0.15 | 목표 0.5 m 안에서 경로 critic 이 꺼지고 방향 벌점이 켜져, 옆에 있는 목표 앞에서 멈춤 |
| goal checker `xy_goal_tolerance` | 0.05 | 마지막 옆 2 ~ 3 cm 를 차동 구동으로 못 메워 실패 |
| RotationShim | 추가 | 도착 근처에서 MPPI 가 돌다 밀려나(스키드 회전 중심 ≠ base_link) 다시 멈춤 |
| 계획기 `downsample_costmap` | false | 0.025 지도를 0.05 로 줄여 거칠게 계획 |
| MPPI `PathAlignCritic` 무게 · `trajectory_point_step` | 28 · 2 (2026-11-12, 전 14 · 4) | 모서리를 Smac 경로보다 일찍 꺾어 기둥에 3 cm 안으로 붙고 궤적 검사기가 모든 궤적을 버려 멈춤 (북쪽 통로 3/5, B2:S → B6:W 3/3, 미션 Q1 FAILED) |

결과 (Gazebo, SDF 지도, `mission_planner/config/approach_poses.yaml` 의 stage 12 곳 차례로): **12/12 성공**, 실패 · 복구 0 회, sim 204 s.
AMCL 오차 주행 중 평균 1.9 cm · 95 % 3.4 cm · 최대 6.1 cm. 끝 위치 실제 오차 평균 7.5 cm · 최대 11.9 cm (도착 기준 5 cm + 도착 뒤 회전 밀림), 방향 평균 1.2°.
Isaac SLAM 지도로는 3 번째 목표에서 AMCL 이 위치를 잃어 2/12.
(위 12/12 는 2026-11-04, 그 뒤 stage 4 곳 (#135) · BT (#142) 가 바뀜. 같은 dev 에서 다른 도구 `run_goals.py` 로는 8 ~ 9/12 (#133, 원인 미확인).
 지금 기준 확인은 미션 전체 `mission_planner/test/gazebo_mission.sh`: 105 단계 DONE · 재시도 0 (2026-11-08, loc_monitor 끔).)

## 확인할 것 ★

- 차체 외곽(footprint): B안 값. control `front_offset` 0.263 · perception `self_box` 와 다름 → 실측으로 하나로.
- 회전 중심: 지금은 base_link. #49(−0.079) · #92(−0.0658) 실측 뒤 회전 중심 프레임을 쓸지 정함.
- 속도(vx 0.2 · wz 1.0) · inflation · 도착 기준은 실물에서 다시 맞춤.
- 지도: 실물 bringup 기본은 도면 SDF 지도(`mission_planner/maps`, #163). localization 단독 기본은 아직 Isaac SLAM 지도(동쪽 8 ~ 15 cm 어긋남, 위). 실물에서 다시 SLAM 하면 루프 닫힘 · 벽 위치를 도면과 비교한다.

