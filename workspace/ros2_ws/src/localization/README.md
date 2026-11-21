# localization

저장 지도 위 위치 추정의 입력과 감시를 맡는 패키지다. 위치를 잃었는지 감시해 다시 찾고(`loc_monitor`), 지도(`/map` · `/map_objects`)와 AMCL 입력 스캔(`/scan_merged`)을 낸다.
기본 위치 추정은 AMCL 이고, AMCL 노드(`map → odom`)는 주행(Nav2) 쪽 launch 가 띄운다. Cartographer 순수 위치 추정도 들어 있지만 기본으로는 꺼져 있다.

## 실행

```bash
ros2 launch localization localization.launch.py                          # 실물: AMCL 입력(/scan_merged · /map) + loc_monitor. AMCL 노드는 주행 쪽 launch
ros2 launch localization localization.launch.py use_sim_time:=true       # 모의
ros2 launch localization localization.launch.py use_amcl_node:=true      # AMCL 노드도 여기서 띄움(config/amcl.yaml, 단독 시험용)
ros2 launch localization localization.launch.py localization:=carto      # Cartographer 순수 위치 추정(기본으로는 안 뜸)
ros2 launch localization localization.launch.py use_amcl_node:=true loc_guard:=active   # loc_guard 가 map → odom (기본 shadow = 기록만)
```

- 먼저 떠 있어야 하는 것: `bringup`(robot_state_publisher, EKF `odom → base_link` · `/odometry/filtered`), 라이다 `/scan_front` · `/scan_rear`.

## 좌표계 · TF

| TF | 내는 노드 |
|---|---|
| `map → odom` | amcl: `amcl`(주행 쪽 launch, 또는 `use_amcl_node:=true`). `loc_guard:=active` 면 `loc_guard`(AMCL `tf_broadcast` 끔) / carto: `map_odom_relay` |
| `map → carto_map`(정적, carto 만) | `carto_map_tf` — 지도 맞춤 값 `align_x/y/yaw` |
| `odom → base_link` | 팀 EKF(`bringup/config/ekf.yaml`) — 이 패키지는 내지 않음 |

- `map` = 발제 좌표(`mission_planner/maps` 문서 좌표 지도와 같은 원점 · 방향).
- Cartographer 는 자기 지도 좌표를 `carto_map` 으로 쓴다. 저장 지도는 SLAM 때 좌표라 발제 좌표와 다르기 때문이다.
  TF 는 내지 않고 `/tracked_pose` 만 낸다. 이 값과 같은 스캔 시각의 EKF `odom → base_link` 로 `map_odom_relay` 가 `map → odom` 을 계산한다.

## 노드

| 노드 | 하는 일 | 주요 입출력 |
|---|---|---|
| `scan_body_mask` | 차체 · 기둥에 가린 라이다 칸을 지움(출발 전 0.5~1.9 s 정지 스캔에서 배우고, 매 스캔 차체 상자 검사) | `/scan_front` · `/scan_rear` → `/scan_*_filtered` |
| `cartographer_node` | 저장 지도(`maps/carto_map.pbstream`, 고정)에서 순수 위치 추정 | `/scan_*_filtered`, `/odometry/filtered` → `/tracked_pose` |
| `carto_start` | 출발 자세(`start_x/y/yaw`, 발제 좌표)에서 새 궤적을 시작시키고 끝남 | `/start_trajectory` |
| `map_odom_relay` | `map → odom` 계산 · 발행 | `/tracked_pose` → `/tf` |
| `scan_merge`(amcl) | 앞 · 뒤 스캔을 base_link 기준 한 스캔으로(움직임 보정) — AMCL 입력 | `/scan_*_filtered` → `/scan_merged` |
| `amcl`(`use_amcl_node:=true` 때만) | `config/amcl.yaml`. 기본은 주행 쪽 launch 가 띄움 | `/scan_merged`, `/map` → `/tf`, `/amcl_pose` |
| `loc_monitor` | 위치 잃음 감지 · 다시 찾기(아래) | → `/localization/status` · `/localization/lock` |
| `loc_guard`(amcl, `loc_guard:=shadow` 기본 · `active`) | AMCL 튐 감시 · 보정(아래). respawn | `/amcl_pose`, `/odometry/filtered`, 스캔 → `/loc_guard/pose`(, active 면 `/tf`) |
| `map_server` · `map_objects_server` | 주행 costmap 정적 층 | `/map`(SLAM 벽, 0.01 m) · `/map_objects`(라이다에 안 보이는 고정 물체, 0.01 m) |

## loc_monitor

- 상태 `/localization/status`(std_msgs/String, Reliable · TransientLocal, 10 Hz): `OK · SUSPECT · LOST · RECOVERING · CONFIRM · FAILED`. `perception_survey` 가 받는다.
- `/localization/lock`(std_msgs/Bool, 10 Hz): `OK` · `SUSPECT` 가 아니면 `true`. twist_mux lock 입력으로 쓸 수 있다(지금 twist_mux 설정에는 연결되어 있지 않음).
- 감지: 추정 자세에서 앞 · 뒤 라이다 끝점의 지도 거리(75 백분위, 문턱 `loc_res_thr` 0.034 m)가 0.5 s 이어지면 SUSPECT, 2 s 이어지면 LOST.
  위치 추정 자세가 EKF 이동과 0.10 m · 5° 넘게 어긋나거나 0.5 s 넘게 끊겨도 LOST.
- 다시 찾기: 마지막 정상 자세 + 그 뒤 odom 이동 둘레를 먼저 찾고, 못 찾으면 지도 전체를 찾는다. 찾으면 carto 는 궤적을 다시 시작하고, amcl 은 `/initialpose` 를 낸다.
  확인(CONFIRM) 1 s 뒤 OK. 60 s 안에 못 찾으면 FAILED.
- 명령 게이트(선택): `gate_in:=<토픽> gate_out:=<토픽>` 을 주면 OK · SUSPECT 가 아닐 때 그 명령을 0 으로 바꿔 낸다. 기본은 끔.

## loc_guard (AMCL 튐 감시 · 보정, 11-21)

실물 시연 기록에서 AMCL 자세가 순간적으로 튀었다(11-16 멈추기 직전 37 cm, 멈춘 동안 12 cm 흘러감, 11-15 18 cm 스파이크).
먼저 `config/amcl.yaml` 을 바꿨고(아래), loc_guard 는 그래도 남는 튐을 잡는 보조 장치다.

- 동작: `/amcl_pose` 증분을 EKF odom 증분과 비교해 튐 후보를 고르고, 후보와 예측 자세(직전 자세 ∘ odom 이동)의 스캔–지도 잔차(`loc_core`)를 비교한 뒤
  라이다 국소 정합으로 보정해 속도 제한을 두고 섞는다. 파라미터는 `config/loc_guard.yaml`(`loc_guard_params:=` 로 다른 파일).
- 모드(launch 인자 `loc_guard`, bringup 도 같은 이름으로 넘김):

| 값 | map → odom | loc_monitor 입력 | 쓰임 |
|---|---|---|---|
| `off` | AMCL | `/amcl_pose` | 예전과 같음(노드 안 뜸) |
| `shadow`(기본) | AMCL | `/amcl_pose` | 판정 · 기록 · `/loc_guard/pose` 만. 실물 기록으로 판정이 맞는지 먼저 본다 |
| `active` | loc_guard | `/loc_guard/pose` | AMCL `tf_broadcast` false 로 덮어씀. `use_amcl_node:=true` 일 때만(아니면 launch 가 오류로 멈춤 — 다른 launch 의 AMCL 은 TF 를 끌 수 없어 map → odom 이 둘이 됨) |

- 안전장치: 노드는 `respawn`(1 s 뒤 다시 뜸) — active 에서 죽으면 map → odom 이 끊겨 Nav2 · costmap · demo_follow 가 멈추기 때문.
  loc_monitor 가 켜져 있으면 `/initialpose` 는 loc_monitor 만 낸다(loc_guard `reinit_enabled` false, launch 가 넣음). 연속 거부 상한을 넘으면 거부를 멈춘다(`loc_guard.yaml`).
- **실물에서 되돌리기**: bringup 을 끄고 `loc_guard:=off`(또는 `shadow`)로 다시 띄운다 — 예: `ros2 launch bringup bringup.launch.py mode:=drive loc_guard:=off`.
  active 로 도는 중에 loc_guard 노드만 죽이면 respawn 으로 다시 뜨고, AMCL 은 TF 를 내지 않으므로 그것으로는 되돌릴 수 없다(반드시 다시 띄울 것).
- active 로 가기 전 shadow 기록에서 볼 것: 튐 판정 횟수 · 위치, 거부가 이어진 최대 시간, 보정량, CPU · 지연.

### amcl.yaml 11-21 변경

| 값 | 전 | 후 | 이유 |
|---|---|---|---|
| `update_min_d` · `update_min_a` | 0 · 0 | 0.02 m · 0.035 rad | 멈춘 동안 매 스캔 갱신 · 리샘플링 → 입자 다양성 붕괴가 튐의 유력 원인(추정). Gazebo 정확도 차이 없음(#129) |
| `resample_interval` | 1 | 2 | 같은 이유 |
| `sigma_hit` | 0.03 | 0.05 | 0.03 은 모의 잡음 기준이라 실물 LDS-03 · 시공 오차(서쪽 판 2.9 cm)에 너무 뾰족함 |

멈춘 동안에는 `/amcl_pose` 가 나오지 않는다. loc_monitor 는 map → odom TF 시각으로 살아 있음을 판단한다(#129). 실물 rosbag 으로 다시 맞출 것.

## 지도 · 값의 출처와 바꿀 곳

- `maps/carto_map.pbstream` · `slam_walls*.yaml` · 지도 맞춤 값 `align_*` 는 **Isaac 모의 미로**에서 만든 것이다.
  실물에서는 실물 미로로 SLAM 을 다시 해 지도와 맞춤 값을 바꿔 넣는다(launch 인자 `carto_state` · `map` · `loc_map` · `align_*` · `start_*`).
- `start_x/y/yaw` 는 **base_link** 의 출발 자세다. 기본값 (0.69365, 0.95, π) = 차체 외곽 가운데를 출발 구역 가운데 (0.734, 0.95) 에 놓을 때의 base_link
  (B안 외곽 x −0.2135 ~ +0.1328 → base_link 가 외곽 가운데보다 0.04035 m 앞, −X 를 봄). mission_planner HOME · 팀 Isaac 장면(make_maze_scene.py)과 같은 자리. 차체 외곽이 바뀌면 다시 계산한다.
- `/map`(`map`) 과 loc_monitor 지도(`loc_map`) 는 같은 세계의 지도로 함께 바꾼다. 기본 `slam_walls*` 는 Isaac `maze.usda` 세계 전용이고, 발제 도면 SDF(Gazebo · `scenes/dice_mission.usda`)와는 동 · 북 벽이 7~10 cm(최대 0.14 m) 다르다.
  `map:=` 만 바꾸면 AMCL 과 loc_monitor 가 다른 지도를 봐서 거짓 LOST 가 나고 FAILED 로 끝날 수 있다(Gazebo 모의: 잔차 중앙 0.021 → 0.077 m). SDF 세계는 `map:=` · `loc_map:=` 모두 `mission_planner/maps/map_amcl.yaml`, `objects_map:=` 는 `map_nav.yaml`.
- `objects.yaml` 은 발제 배치(모의 seed 1)의 고정 물체다.
- `loc_res_thr` 0.034 m 는 모의 정상 주행 잔차 최대(0.0337 m)에 맞춘 값이라 실물에서 SUSPECT 가 잦을 수 있다. 실물 정상 주행 기록으로 다시 정한다.

## 시험

- 단위 시험(ROS 없음): `python3 -m pytest -q test/`
- 모의 확인(Isaac, 이 패키지를 팀 EKF 설정과 함께 띄움) 결과는 PR 본문에 적는다.
