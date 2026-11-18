# 로봇 콘솔 (Jetson)

스크립트 하나로 로봇을 띄우고, 다른 터미널에서 로그를 보고, TUI 로 비상정지 · 수동 조작 · 모드 바꾸기를 한다.
모드마다 필요한 bringup 과 할 일 (시연 · 미션 · 녹화) 을 콘솔이 띄우고 끈다. 여러 터미널에 launch 를 나눠 띄우던 것을 대신한다.

## 쓰는 법 (Jetson, 터미널 세 개)

| 터미널 | 명령 | 하는 일 |
|---|---|---|
| 1 | `~/Lv2_Cartlidar_Assignment/workspace/tools/robot_console/start.sh` | 콘솔 데몬. 모드 0 (대기 · 수동 조작) 으로 시작. Ctrl+C = 모두 끔 |
| 2 | `…/robot_console/log.sh` | 로그 보기 (콘솔 사건 · 비상정지 · 위치 추정 · 미션 · 신호등 · 바코드 · 퍽) |
| 3 | `…/robot_console/tui.sh` | 비상정지 · 수동 조작 · 모드 바꾸기 |

- 시작 전 (사람): 배터리 먼저 · USB 나중, 두 OpenCR 이 보이는지 (`ls /dev/ttyACM*` 두 개). 팔은 **어느 자세여도 됨** — 켜진 뒤 비상정지를 풀면 (`r`) 정해진 순서로 STOW 로 접힘.
- 다른 사람이 bringup · 하드웨어를 띄워 둔 상태면 콘솔이 모드를 띄우지 않고 알리기만 한다 (남의 프로세스는 끄지 않음).
- 로그 파일: `~/robot_logs/<날짜_시각>_mode<번호>/` (`launch.log` · `action.log` · `commands.txt`), **rosbag `bag/`** (모드가 준비되면 자동 녹화, 모드를 바꾸거나 끄면 멈춤. 라이다 · odom · IMU · TF · AMCL · 명령 · 인지 결과 · 미션 상태 — 영상은 빼서 분당 수십 MB 수준, 토픽은 `modes.yaml` `record_topics`, 모드마다 `record: false` 로 끔).

## TUI 키

| 키 | 하는 일 |
|---|---|
| Space | 비상정지 (하드웨어가 기억 — TUI · 콘솔이 꺼져도 `r` 전까지 잠김) |
| r | 비상정지 풀기 · 리셋. 팔 HOLD 도 풀려 **팔 정리가 움직임** (STOW 로 접기 → 모드에 따라 DRIVE 로 펴기) |
| w / s | 앞 / 뒤 (누르고 있는 동안) |
| a / d | 왼쪽 / 오른쪽 제자리 회전 |
| x | 바로 멈춤 |
| + / - | 속도 단계 (0.05 · 0.10 · 0.15 m/s, 0.25 · 0.40 · 0.60 rad/s) |
| 0 ~ 5 | 모드 고르기 → `y` 로 확정 |
| . | 모드 모두 끔 |
| Enter · Y · N · Q · S | 지금 할 일 (시연 등) 의 질문에 Enter · y · n · q · s |
| Esc | TUI 끝 (로봇 · 모드는 그대로) |

수동 조작은 `/cmd_vel_teleop` (twist_mux 우선순위 100) 이라 시연 · 미션 중에도 사람이 이긴다. 비상정지는 모든 움직임을 막는다.
물리 비상정지 버튼은 없다 — 소프트웨어 정지뿐이니, 시험 중에는 한 사람이 Space 를 누를 준비를 한다.

## 모드 (`modes.yaml` 에서 고침)

| 번호 | 모드 | 띄우는 것 | 할 일 |
|---|---|---|---|
| 0 | 대기 · 수동 조작 | 하드웨어 · 라이다 · 위치 추정, 팔 STOW | 없음 (TUI 로 몲) |
| 1 | 주행 시연 | 위 + 팔 STOW, TFmini · 위치 잠금 끔 | `drive_demo/run_demo.sh real slow init` (10 구간, 질문은 TUI Y · Enter) |
| 2 | 팔 시연 | 하드웨어 · 팔 (DRIVE 로 폄) | 이름 자세 DRIVE → ALIGN → TAG_VIEW → BOX_VIEW → DRIVE, 자세마다 Enter |
| 3 | 인지 시연 | 위 + 카메라 · 인지, 팔 STOW | 퍽 찾기 (앞 · 뒤 D435) · 장소 태그 · 신호등 점멸 세기 켬 → 로그 창에서 확인 |
| 4 | 통합 시연 | bringup 전체 (미션 · 계획기 · Nav2) | `/mission/start` 를 받아들여질 때까지 (준비 확인 14 개) |
| 5 | SLAM 녹화 주행 | 주행 시연과 같음 | 주행 시연 경로를 따라가며 bag 녹화 (`~/real_drive/bags/slam_*`) → PC 에서 Cartographer |

- 모드를 바꾸면 앞 모드를 모두 끄고 새로 띄운다 (하드웨어도 다시 켜져 수십 초 걸림).
- 5 번은 지도를 Jetson 에서 만들지 않는다 (Jetson 에 Cartographer 없음). 녹화한 bag 을 PC 로 가져와 `data/261114_1330_maze_map` 의 순서로 만든다.

## 시험 (PC, 로봇 없이)

`CONSOLE_MODES=<가짜 modes.yaml> CONSOLE_PORTS=/dev/fakeA,/dev/fakeB` 로 데몬을 띄우면 하드웨어 없이 모드 바꾸기 · 비상정지 · 입력 넘기기를 볼 수 있다
(2026-11-18 PC 확인: 모드 0 → 1 → 2 (안 뜨는 노드로 실패 알림) → 끔, 비상정지 버튼 유지 · 리셋 펄스, 할 일에 y · Enter 전달, TUI w → 0.05 m/s 뒤 0). 실물 확인은 아직.
