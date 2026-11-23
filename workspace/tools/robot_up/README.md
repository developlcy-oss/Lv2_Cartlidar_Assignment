# robot_up — 젯슨 bringup 한 번에 띄우기 (11-23, ★ 실물 미시험)

주행 시연 · 미션을 **스크립트 하나로 순서대로** 띄웁니다. 단계마다 준비 (노드 · 토픽 · lifecycle · 서비스) 를 확인하고, 안 되면 그 단계 로그 끝을 보여 주고 멈춥니다.
예전에 터미널 4~5 개로 하던 `drive_demo/README.md` 단계 4~9 (주행) · 10~12 (미션) 와 같은 명령을 같은 순서로 띄웁니다.

## 쓰는 법 (젯슨 화면 또는 tmux 에서)

```bash
cd ~/robot_up
./up.sh drive max auto init      # 주행 시연: 뒤 인자는 run_demo.sh real 에 그대로 (예: slow step init 0-0)
./up.sh drive                    # 인자 없으면 run_demo 사용법만 보여 주고 끝
./up.sh mission                  # 미션: READY · REARM 뒤 Enter → /mission/start (⚠ 기본 max_wz:=0.5)
./up.sh mission max_wz:=0.4      # bringup 인자를 뒤에 붙이면 덮어씀
./up.sh down                     # 남은 ROS 프로세스 목록 → "kill" 을 입력해야 끔
UP_DRY=1 ./up.sh drive max auto  # 아무것도 띄우지 않고 명령 순서만 찍음
```
- **물리 비상정지는 없습니다** — 리모컨 1 이 정지 수단. 끝 요약에도 찍힙니다.
- ssh 로만 띄우면 끊길 때 정리가 제대로 안 될 수 있습니다 → 젯슨 화면 · `tmux` 권장 (HUP 을 받으면 정리는 시도함).
- Ctrl+C = 띄운 것을 역순으로 끔 (정리 중 다시 눌러도 정리는 끝까지). 미션은 먼저 `/mission/stop` 후 멈춤 확인 (≤ 10 s).

## 단계

| 모드 | 단계 | 확인 |
|---|---|---|
| 공통 | 사전 점검 | 남은 ROS 프로세스 (있으면 목록 · `kill` 입력 시만 끔, 다른 계정 것은 안 끔) · `/dev/shm` (프로세스 0 일 때만) · OpenCR 두 대 (`jetson_check.sh opencr`, FAULT 면 "12 V 켠 채 RESET" 안내 후 재점검) · `/dev/lidar_*` · `/dev/ttyACM*` 2 개 → ports · robot_console 데몬이 떠 있으면 거부 |
| drive | ① base_bringup `rc100:=true obstacle_lock:=false loc_lock:=false` | 컨트롤러 active · twist_mux |
| | ② `scan_rotate.py` | `/scan_front` · `/scan_rear` |
| | ③ bringup `mode:=drive` (⚠ 팔이 주행 자세로 펴짐 — Enter 확인) | map_server · amcl active · `/odometry/filtered` · loc_monitor 끔 |
| | ④ `arm_look.launch.py` (`ARM_LOOK=0` 이면 건너뜀) | `/perception/place_box/measure` |
| | ⑤ `check_jetson.sh` | '점검 통과' 아니면 `yes` 입력해야 계속 |
| | ⑥ REARM | `/e_stop` false (리모컨 2+3) — 메시지가 없으면 실패 |
| | ⑦ 출발 자리 (0.694, 0.950 · 180°) Enter → `run_demo.sh real <인자>` | |
| mission | ① bringup `rc100:=true ports:=… max_wz:=0.5` (Enter 확인) | controller_manager · mission_node · Nav2 lifecycle (25 s 넘게 멈출 때만 activate 개입) · collision_monitor |
| | ② `/mission/ready` = READY (최대 300 s) → REARM → 출발 자리 확인 → Enter → `/mission/start` | `/mission/status` DONE · FAILED 까지 표시 |

기록: `~/robot_logs/<YYMMDD_HHMM>_<모드>/` (단계별 로그 · pids · 끝 요약). run_demo 기록은 예전처럼 `~/demo_v2/runs/`.

## 환경 변수

| 변수 | 기본 | 뜻 |
|---|---|---|
| `REPO` | `~/Lv2_Cartlidar_Assignment` | source 할 저장소 (`~/git/…` 은 11-18 판이라 기준 아님). run_demo 가 source 할 install 과 다르면 멈춤 |
| `DEMO` | `~/demo_v2` | drive_demo 폴더 |
| `UP_ALLOW_MISMATCH=1` | 0 | install ≠ REPO 여도 경고만 |
| `ARM_LOOK` | on | run_demo 와 같은 변수. 0 이면 ④ 건너뜀 |
| `UP_ARM_CAMERA=0` · `UP_SKIP_ARM_WARN=1` | | ④ 에서 Arducam 안 띄움 · ③ 팔 경고 Enter 생략 |
| `UP_LC_STUCK` | 25 | lifecycle 이 이 초 동안 그대로일 때만 activate 개입 |
| `UP_AMCL_STRICT=1` | 0 | AMCL σ 가 크면 경고 대신 멈춤 |
| `UP_KILL_LEFTOVER=1` | 0 | 정리 때 그룹 밖으로 나간 **우리** 자손도 끔 |
| `UP_ROS2_TMO` | 8 | ros2 CLI 한 번의 시간 한도 [s] |

## 시험 · 확인할 것

- 노트북: `bash test_up_lib.sh` (가짜 ros2, 60 개) · `UP_DRY=1 REPO=<저장소> DEMO=<저장소>/workspace/tools/drive_demo ./up.sh drive max auto`
- **첫 실물에서 볼 것**: ③ 까지 띄우고 q 로 끝내 → 정리 (팔 DRIVE 로 접기 → 하드웨어 끔) 뒤 팔이 처지는지 · `use_loc_monitor:=false` 가 실제로 먹는지 (로그 "loc_monitor 없음") · REARM 대기 · 단계별 소요 시간 (끝 요약)
- 팔이 펴지는 도중 Ctrl+C 는 따로 다루지 않음 (리모컨 1).

만든 과정: 11-23 팀 검토 (분석 · 구현 · 반대 의견 3 차) — 근거 문서는 그 세션 scratchpad (휘발).
