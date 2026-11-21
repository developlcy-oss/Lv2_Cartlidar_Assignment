# drive_avoid — 장애물 멈춤 · 다시 계획 주행 (2026-11-17)

경기 (11-16) 는 미리 계산한 길만 따라가서 장애물을 보지 못했다 (충돌 7 번). 이 폴더는 같은 길 (규칙 v2 길 모음) 을 따라가되
장애물이 나오면 **멈추고 → 자기 위치를 다시 찾고 → 3 초 동안 치워졌는지 보고 → 원래 길로 가거나 다시 계획**한다.
경기 도구 `../drive_demo` (조각 따라가기 · 규칙 v2 필터 · Nav2 설정) 를 불러 쓰며 그 폴더는 고치지 않는다.

## 흐름

```
라이다 2대 (원래 스캔) ─┬→ [obstacle_guard] ── /avoid/blocked ─→ [stop_gate] ──→ 바퀴 (/cmd_vel · 실물 cmd_vel_nav)
정면 TFmini ──────────┘
                                  ↑ 알려진 물체 지도 (벽 · 상자)          ↑ /avoid/hold          ↑
                                  ↑ /avoid/path_ahead (지금 조각)        │                     │
길 모음 (legs.json) → [avoid_runner] ─ FollowPath · Spin → controller_server (RPP) · behavior_server → v2_filter ┘
                          상태: 주행 → 정지 → 위치확인 → 판단 (3 s) → 복귀 | 우회계획 → 우회 | 건너뜀
```

| 단계 | 하는 일 | 값 |
|---|---|---|
| 감지 | 지금 따라가는 조각의 앞 1.0 m · 반폭 0.22 m 띠 안에, 알려진 물체에서 8 cm 넘게 떨어진 라이다 점이 5 개 이상 → 로봇 가운데 0.75 m 안이면 그 스캔에서 바로 (a), 그 밖은 같은 라이다 2 스캔 연속. TFmini 는 0.17 ~ 0.45 m · 알려진 물체에서 15 cm 밖 · 띠 안 · 정지점 0.30 m 앞까지만, 5 번 연속이면 (c) | `obstacle_guard` 인자 (`tfmini_*`) |
| 감지 범위 (b) | 라이다 원래 스캔을 직접 받음 (팀 `scan_body_mask` 는 앞 x ≤ 0.285 m 를 지워 차체 앞 15 cm 가 안 보였음). 차체 상자 x −0.235 ~ 0.16 · \|y\| ≤ 0.19 + 출발 전 정지 1.4 s 에 배운 '차체에 가린 칸'. 로봇 자세 = 최신 map → odom ∘ 스캔 시각 odom → base_link (11-17: 한 번에 찾으면 AMCL 마지막 갱신 시각 자세가 나와 거짓 장애물) | ★ 감시는 로봇이 서 있을 때 시작 |
| 1. 멈춤 | 감시가 막힘을 내면 정지 관문이 그 스캔에서 바로 바퀴 명령을 0 으로, 주행 관리자는 조각을 취소하고 멈춤 요청 | 감시 소식이 0.6 s 끊겨도 0 |
| 2. 위치 확인 | AMCL 제자리 갱신 5 번 (`demo_follow.relocalize`) | |
| 3. 판단 | 3 s 동안 같은 띠를 봄. 0.5 s 이어서 비면 '치워짐' | `--wait` |
| 4a. 치워짐 | 남은 조각을 이어서 | |
| 4b. 그대로 | 본 점 + 뒤 0.15 m 그림자 점을 넣고 규칙 v2 계획기로 다시: 원래 길의 장애물 뒤 0.45 · 0.80 m 로 합류 → 안 되면 정지점까지 | 계획 펼침 상한 100 000 |
| 건너뛰기 | 우회 길도 없으면 그 정지점을 건너뛰고 다음 정지점 먼저 → 그 다음 건너뛴 정지점 → 계속 (정지점마다 최대 2 번) | 사용자 결정 11-17 |
| 정지점 | 서기 2 s ("확인") | `--dwell` |

## 파일

| 파일 | 내용 |
|---|---|
| `obstacle_guard.py` | 장애물 감시 노드 |
| `stop_gate.py` | 정지 관문 노드 (규칙 v2 필터 출력 → 바퀴) |
| `avoid_runner.py` | 주행 관리자 (길 모음 · 장애물 처리 · 건너뛰기 · 기록) |
| `replan.py` | 규칙 v2 계획기 감싸기 (장애물 점 추가). 계획기는 노트북 `~/Desktop/CartRider/2026-11-14_규칙v2계획` (`PLAN_DIR`) ★ Jetson 에 없음 |
| `sim/run_sim.sh` | Gazebo 시험 한 번 (아래) |
| `sim/scenario.py` | 시험용 장애물 놓기 · 치우기 · Gazebo 실제 자세로 충돌 여유 측정 |
| `sim/make_known_map.py` | 시뮬 알려진 물체 지도 (상자 · L 을 길 모음 실측 자리로) |
| `sim/scenarios/*.json` | A 치움 · B 그대로 · C 길 막힘 (판) · P 사람 · S_* 갑자기 나타남 (차체 앞면에서 gap_m, L2 → L1 직선에서) |
| `sim/guard_v1_args.txt` | 예전 감시 (합친 스캔 · 2 스캔 확인 · TFmini 없음) 를 흉내 내는 인자 — `GUARD_ARGS="$(cat sim/guard_v1_args.txt)"` |
| `sim/diag_floor.py` | 진단: REC=1 녹화에서 감시 막힘 순간의 기울기 · 점 거리 · 출처 |
| `sim/sudden_table.py` | 갑자기 나타남 시험 모음 → `runs/sudden_table.json` (놓을 때 속도 · 멈출 때까지 거리 · 시간 · 6 각형 / 차체 여유) |
| `runs/` | 시험 기록 (아래 결과 표). 이름에 '무시' 가 든 디버그 · 고치기 전 실행과 녹화 (bag) 는 저장소에 안 올림 |
| `real_map/` | 실물 미로 지도: 팀원 녹화 bag → slam_toolbox → 발제 좌표로 맞춤 (아래) |
| `turn_speed/` | 회전 속도 상한 시나리오 비교 (아래) |

## Gazebo 시험 (노트북)

빌드 (저장소 밖 `~/lv2_sim`, 한 번): `perception` 은 이 노트북에 libzbar 가 없어 못 빌드 → 예전 저장소 빌드를 바탕으로 깔고 `control` · `bringup` 만 새로.

```bash
cd ~/lv2_sim && R=~/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment/workspace && source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --base-paths $R/ros2_ws/src $R/urdf_and_usd/waffle_4wd_description \
  --packages-select waffle_4wd_description robot_interfaces mission_planner localization navigation --cmake-args -DBUILD_TESTING=OFF
source $R/ros2_ws/install/setup.bash && source install/setup.bash
colcon build --symlink-install --base-paths $R/ros2_ws/src --packages-select control bringup --cmake-args -DBUILD_TESTING=OFF
```

```bash
cd workspace/tools/drive_avoid
ROUTE="box3 box5" ./sim/run_sim.sh sim/scenarios/A_치움.json      # 장애물 놓음 → 멈춤 → 1 s 뒤 치움 → 원래 길
ROUTE="box3 box5" ./sim/run_sim.sh sim/scenarios/B_그대로.json    # 그대로 → 우회
ROUTE="box1 box2 box6" ./sim/run_sim.sh sim/scenarios/C_막힘.json # 통로 막힘 → box2 건너뛰고 box6 먼저 → box2
./sim/run_sim.sh none 장애물없음                                   # 한 바퀴 (어제 순서)
ROUTE="L2 L1" SPEED=max ./sim/run_sim.sh sim/scenarios/S_상자_03.json 갑자기_상자_0.3m   # 차체 앞 0.3 m 에 갑자기
GUARD_ARGS="$(cat sim/guard_v1_args.txt)" ROUTE="L2 L1" SPEED=max ./sim/run_sim.sh sim/scenarios/S_상자_03.json 갑자기_상자_0.3m_예전감시
python3 sim/sudden_table.py                                        # 갑자기 나타남 표
GUI=1 ...  # Gazebo 창 · ./sim/run_sim.sh stop = 남은 프로세스 정리
```

결과 `runs/<월일_시각>_시뮬_<이름>/`: `summary.txt` (구간 · 장애물 멈춤) · `truth_summary.txt` (Gazebo 실제 자세로 잰 벽 · 상자 · 장애물 최소 여유) · `run.log` · `events.json` · `truth.csv`.

## 결과 (Gazebo, 2026-11-17)

조건: `gazebo_sim` + 진짜 Nav2 노드 · AMCL (EKF), 시뮬 라이다 · TFmini, 상자 · L 은 길 모음 실측 자리. 여유는 Gazebo 실제 자세로 잼 (`truth_summary.txt`).

**장애물 처리** (`runs/1117_2031~2037_*`, 전진 0.30 m/s)

| 시험 | 장애물 | 결과 | 멈춤 → 처리 | 장애물과 최소 여유 |
|---|---|---|---|---|
| A 치움 | 상자, box3 → box5 길 위 (로봇 앞 1.0 m), 멈춘 뒤 1 s 에 치움 | 구간 2/2 · 71.9 s | 멈춤 3.3 s → 원래 길 | 62.3 cm |
| B 그대로 | 같은 자리, 치우지 않음 | 구간 2/2 · 86.2 s | 멈춤 5.7 s → 다시 계획 (+0.45 m 에서 원래 길로 합류) | 6.8 cm |
| C 막힘 | 판 (0.10 × 0.55 m) 으로 box1 → box2 통로 막음 | 구간 2/4 (box2 건너뜀) · 95.0 s | 2D 로도 길 없음 → box6 먼저 → box2 다시 시도 → 또 막혀 포기 | 58.4 cm |

**갑자기 나타남** (`runs/sudden_table.json`, 최고 속도 0.44 ~ 0.46 m/s, L2 → L1 직선, 차체 앞면에서 gap 거리에 놓음)

| 감시 | 장애물 · gap | 멈출 때까지 | 6 각형 (팔 끝) 여유 | 차체 여유 |
|---|---|---|---|---|
| 새 | 상자 0.2 m | 18.6 cm · 0.5 s | −9.9 cm | 1.8 cm |
| 새 | 상자 0.3 m | 20.5 cm · 0.5 s | −2.3 cm | 9.4 cm |
| 새 | 상자 0.4 m | 15.3 cm · 0.4 s | 13.2 cm | 24.9 cm |
| 새 | 상자 0.5 m | 17.4 cm · 0.4 s | 21.0 cm | 32.7 cm |
| 새 | 사람 (두 발) 0.3 m | 20.0 cm · 0.5 s | −1.0 cm | 10.7 cm |
| 예전 (`guard_v1_args.txt`) | 상자 0.3 m | 31.2 cm · 1.4 s | −10.0 cm | **−0.2 cm (닿음)** |
| 예전 | 상자 0.5 m | 38.2 cm · 0.9 s | 0.5 cm | 12.2 cm |

- 예전 감시 = 합친 스캔 (`scan_body_mask` 가 차체 앞 15 cm 를 지움) · 2 스캔 확인 · TFmini 없음. 새 감시는 원래 스캔 · 가까우면 한 스캔 · TFmini 를 써서 멈출 때까지 거리가 31 ~ 38 → 15 ~ 21 cm.
- 6 각형 여유가 음수인 것은 팔 끝 (주행 자세, 차체 앞 0.25 m) 이 상자 위로 넘어간 것. 팔 끝 높이가 상자 (시험 10 cm) 보다 높은지는 실물 확인 ★.
- **TFmini 거짓 멈춤 고침**: 처음 최고 속도 한 바퀴 (`1117_210816`) 에서 L 정지점 근처 거짓 멈춤 2 번 (TFmini 0.13 · 0.18 m). Gazebo 주행 자세에서 집게 손가락이 TFmini 빛 안에 들어가 0.10 ~ 0.16 m 를 읽음 → 0.17 m 이하 무시 · 5 번 연속 · 정지점 0.30 m 앞부터 끔. 고친 뒤 (`1117_2134_*`) 장애물 없음 42.8 s 멈춤 0 번, 상자 0.5 m 는 26.1 cm 에 멈춤 · 차체 여유 23.5 cm.
- 모든 실행의 '지도 물체 최소 여유 −1.2 cm' 는 정지점 (상자 · L 앞) 에 섰을 때의 값. 정지점을 상자 쪽으로 7 cm 당겨 둔 것 (경기 때와 같은 길 모음) 이라 6 각형 앞 끝 (팔 끝) 기준으로 겹침.

## 실물 미로 지도 (`real_map/`, 2026-11-17)

경기장에서 녹화한 bag (11-16 아침, 팀원 녹화 · 328 MB 라 저장소 밖) 으로 slam_toolbox 지도를 만들고 발제 지도 좌표에 맞춤.

```bash
ROTATE_MERGED=1 python3 real_map/filter_bag.py <원본 bag> maze_slam_in   # SLAM 토픽만. 그 녹화는 /scan_merged 가 180° 돌아가 있어 돌림
./real_map/run_slam.sh 2.0 maze_real_slam                               # slam_toolbox (sync) → pgm/yaml
python3 real_map/align_map.py maze_real_slam <mission_planner/maps/map_amcl.yaml> <amcl map→odom csv> maze_real_aligned
python3 real_map/objects_from_slam.py <mission_planner/maps>              # 벽 어긋남 · 상자 덩어리 ↔ 지도 물체
python3 real_map/walls_only.py                                          # 상자 · 받침대를 지운 벽만 지도
```

- 결과: `maze_real_aligned` (상자 포함) · `maze_real_walls` (벽만), 비교 그림 `maze_real_aligned_비교.png`.
- 벽은 발제 지도와 중앙 2.5 cm, 북벽 5 ~ 10 cm 어긋남. 실물 라이다는 7 cm 상자 · L 을 봄 (그날 아침 상자 자리와 최대 14 cm 차이).
- 그 녹화의 odom 은 실제보다 약 8 % 크게 잼 (휠 지름 · 벨트비 확인 ★).

## 회전 속도 상한 비교 (`turn_speed/`, 2026-11-17)

```bash
python3 turn_speed/calc.py   # → data.json · 회전속도_비교.html (페이지, 저장소에는 안 올림)
```

경기 순서 한 바퀴 (`../drive_demo/legs.json`, 정지점 서기 제외) 에 반경별 속도 상한을 입혀 계산 (가감속 1.0 m/s², 실물 v7 벨트 1:2.0):

| 시나리오 | 상한 | 한 바퀴 | 모터 3.0 넘는 꺾기 | 꺾는 중 최고 모터 |
|---|---|---|---|---|
| 지금 (규칙 v2 · 11-16 최고 속도판) | 직진 0.45 m/s, 반경 1 m 이하 모터 3.0 | 153.7 s | 1.8 s | 4.51 |
| B | 직진 모터 7 → 반경 1 m 에서 3.0 | 152.7 s | 1.9 s | 4.32 |
| C | 직진 모터 7 → 반경 0.5 m 에서 3.0 | 149.0 s | 8.3 s | 5.51 |
| A | 직진 모터 7 → 반경 0.25 m (가장 급한 계획 반경) 에서 3.0 | 139.8 s | 23.7 s | 6.34 |

전진 28.5 m 중 9.2 m 가 반경 0.5 m 미만이라 직진 상한을 올려도 한 바퀴는 0.7 ~ 9.0 % 만 줄어듦. 대신 '꺾는 중 모터 ≤ 3.0' (규칙 v2 잠정) 을 넘는 시간이 C 8.3 s · A 23.7 s 로 늘어남. 어느 안으로 갈지는 정하지 않음.

## 시뮬과 실물이 다른 점 · 할 일 ★

- 시뮬 라이다 스캔면 (바닥 위 7.5 cm) 은 7 cm 퍽 상자를 못 봐서 시험 상자는 10 cm 높이. (2026-11-20 부터 앞 라이다만 1.5 cm 낮춰 5.98 cm — 앞 라이다는 시뮬에서도 7 cm 상자를 봄, 뒤는 7.48 cm 그대로) 실물 라이다는 7 cm 상자 · L 을 봤음 (위 실물 SLAM).
- Gazebo 고정 모델은 set_pose 로 안 옮겨짐 → `scenario.py --move-boxes` 는 지우고 다시 만듦.
- 최고 속도 (max) 는 노트북 바탕화면 `2026-11-14_규칙v2계획/demo_real` 의 params_demo_max.yaml · v2_filter.py 를 씀 (`SPEED=max`). ★ 저장소 `../drive_demo` 와 같은 내용인지 옮기기 전에 맞출 것.
- 알려진 물체 지도: 시뮬은 길 모음 실측 자리. 실물은 상자 자리가 날마다 다름 → 실물 SLAM 지도 (`real_map/maze_real_aligned`) 또는 그날 실측으로 만들어야 하고,
  지도와 8 cm 넘게 다른 상자는 장애물로 보여 정지점 앞에서 멈출 수 있음.
- 계획기 (`plan_v2.py` · `make_legs.py`) 는 노트북에만 있음 → Jetson 으로 옮겨야 실물에서 다시 계획 가능.
- 실물 시험은 아직 없음 (11-16 경기 뒤 시뮬만). 실물 주행 자세에서 팔이 TFmini 빛을 가리는지 · 팔 끝 높이 확인 ★.
- 두 바퀴 (1 바퀴 상자 6 + L 3, 2 바퀴 L 만 → 출발) 경로는 아직 안 만듦.
