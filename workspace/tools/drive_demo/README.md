# 주행 · 미션 시험 안내 (팀원용 · 11-15 밤)

로봇 조립이 끝났다고 보고 씁니다. 제어 담당 (WindForce08) 이 없어도 **주행 시연** 과 **팀 전체 미션** 을 시험할 수 있게, 순서 · 명령 · 명령이 하는 일 · 생길 수 있는 문제를 모두 적었습니다.
11-15 낮 ~ 밤 실물 시험에서 직접 겪은 문제와 고친 방법이 들어 있습니다. 명령은 그대로 복사해 붙여 넣습니다.

| 시험 | 무엇 | 언제 쓰나 |
|---|---|---|
| **A. 주행 시연** | 미리 계산한 경로 10 구간 (출발 → 상자 6 개 → L3 → L2 → L1 → 출발 구역) 을 Nav2 로 따라감. 팔은 주행 자세, 정지점마다 사람이 Enter | 주행 · 위치 추정만 확인, 팔 담당이 정지점마다 팔 동작 시험 |
| **B. 전체 미션** | 팀 bringup 하나로 하드웨어 · 카메라 · 인식 · 위치 추정 · Nav2 · 계획기 · 미션을 모두 띄우고 `/mission/start` | 경기와 같은 흐름 확인 |
| **C. 노트북 시뮬** | 로봇 없이 같은 것을 노트북에서 (주행 시연 = loopback, 전체 미션 = Gazebo) | 코드 · 설정을 바꾼 뒤 먼저 확인 |

## 이 폴더 (`workspace/tools/drive_demo`)

| 파일 | 하는 일 |
|---|---|
| `run_demo.sh` | 시연 한 회차: 시연 Nav2 노드 · 규칙 v2 필터를 띄우고 active 확인 → 계획을 따라감 → 기록 → 끝나면 끔 (`real` · `sim`, `RUN_TAG=` 이름표, `NO_RECOVER=1` 되찾기 끔, `SIM_TURN=` 실물형 시뮬, `BOX_APPROACH=1\|0\|dry` 상자 앞 다가가기, `DEMO_BAG=0` rosbag 끔) |
| `demo_follow.py` | 계획의 조각 (전진 · 후진 · 제자리) 을 Nav2 에 차례로 보냄 · 출발 확인 · 경로 이탈 멈춤 (10 cm) · 방향 맞추기 거부 시 건너뛰기 · **되찾기** (조각 실패 → 위치 다시 찾기 → 왔던 길 ≤ 0.30 m → 다시, 최대 2 번) · 기록 (`track_live.csv`) |
| `box_quiet.py` · `test_box_quiet.py` | 11-20 상자 정지점 0.40 m 안 판정 (demo_follow 가 import — **`demo_follow.py` 와 같은 폴더에 함께 복사**) · 시험 `python3 -m pytest -q test_box_quiet.py` |
| `box_face.py` · `test_box_face.py` | 11-21 `/scan_front` 로 상자 앞면 재기 (선분 맞춤 · 길이 205 ± 25 mm, demo_follow 가 import — **같은 폴더에 함께 복사**) · 시험 `python3 -m pytest -q test_box_face.py` |
| `v2_filter.py` | Nav2 속도를 규칙 v2 로 자름 (제자리 ≤ 90° 실제 각도 · 이어진 후진 ≤ 0.40 m (실물 0.50) · 반경 ≥ 0.20 · 꺾는 중 모터 상한) |
| `sim_turn_scale.py` | 시뮬만: 실물처럼 덜 도는 로봇 흉내 (회전 명령을 꺾기 · 제자리 비율로 줄임, `SIM_TURN=0.79,0.70`) |
| `scan_rotate.py` | LDS-03 두 대 드라이버 (URDF 에 라이다 보정이 없을 때만 180° 돌림, 자동) |
| `check_jetson.sh` · `jetson_check.sh` | 시연 전 점검 · Jetson / OpenCR (펌웨어 버전 · 팔 대기 범위) / 센서 장치 점검 (읽기만) |
| `params_demo*.yaml` | 시연 Nav2 설정 (A = 기본, B · C = 벨트 v7 빠른 꺾기) |
| `demo_plan.json` · `legs.json` · `route_*.json` | 오늘 계획 `7cm_실측L_어제순서` (10 구간, L1 ~ L3 실측) · 미리 계산한 길 90 개 · 들르기 방식 |
| `demo_plan_wide.json` · `widen_plan.py` | 11-19 반경 넓힌 계획 (정지점 같음, 제자리 회전 · 방향 맞추기 적게) · 만드는 도구 (아래 "반경 넓힌 계획") |
| `scan_view/` | 노트북 RViz 감시 (ssh 로 Jetson 라이다 · AMCL 자세만 받음) |
| `rc100_state_watch.py` · `lv2_paths.py` | 리모컨 버튼 값 읽기 · 저장소 위치 찾기 |

- Jetson 에서는 `~/demo_v2` 로 복사해 씁니다 (아래 명령은 모두 `~/demo_v2` 기준, 2 장 단계 1).
- 계획을 새로 만드는 도구 (방문 순서 바꾸기 · 상자가 옮겨졌을 때 길 다시 계산) 는 제어 담당 노트북에만 있습니다.
- 같은 내용의 페이지: https://claude.ai/artifact/Ukqy8zXuXLv747KctzifRh

## 0. 먼저 알아 둘 것 (11-15 실물에서 확인한 것)

### 안전

- **물리 비상정지 버튼은 없습니다.** 멈추는 방법은 모두 소프트웨어 정지입니다: **리모컨 1** (E-stop), 시연 터미널의 **Ctrl+C**, 미션은 `/mission/stop`.
- **리모컨 전원이 꺼져 있으면 1 을 눌러도 E-stop 이 걸리지 않습니다** (로봇은 그대로 달림). 시작 전에 1 → 멈춤, 2 + 3 → 풀림을 꼭 해 봅니다.
- 리모컨 담당 한 명은 엄지를 1 번 위에 두고 로봇만 봅니다. 벽 · 상자에 닿을 것 같거나 벨트 소리가 나면 바로 1 을 누릅니다.
- 시연 · 미션 중에는 리모컨 **방향 버튼을 누르지 않습니다** (리모컨이 Nav2 보다 우선이라 경로가 어긋남).

### 오늘 실물에서 알게 된 것

| 무엇 | 내용 | 할 일 |
|---|---|---|
| 전원 순서 | USB (Jetson) 가 배터리보다 먼저 켜지면 OpenCR 부팅 점검이 모터를 못 찾아 `MOTOR_CONFIG\|FAULT` 가 걸림 (재부팅으로만 풀림). **Jetson 을 재부팅해도 같음** | 배터리를 켠 채 **두 OpenCR 모두 RESET** |
| 팔 대기 자세 | 하드웨어를 켤 때 팔 여섯 칸이 대기 범위 안이어야 함 (아니면 토크를 안 켜고 거부). 범위 (dev 11-16): J1 **3425 ~ 3539** · J2 653 ~ 721 · J3 3025 ~ 3091 · J4 797 ~ 864 · J5 **1849 ~ 2246** · 집게 **1531 ~ 2669** (tick, 팔 펌웨어 v6 이 보고하는 모델 각도). 11-15 에 J1 · J5 로 거부된 적 있음 | 손으로 가운데 쪽 (J1 3482 · J5 2048) 에 맞추고, 집게는 손으로 닫기 (단계 3) |
| 팔 펌웨어 | dev 11-16 (#316) 부터 호스트가 팔 보드 **v6** (관절 1 π 보정) 을 요구. 주행 보드는 v5 그대로. 다르면 하드웨어가 안 켜짐 | `./jetson_check.sh opencr` 가 버전이 다르면 ❌ — 팔 담당이 `opencr_arm_v6.bin` 업로드 |
| 라이다 USB | 11-15 에는 허브가 2.1.x 로 옮겨져 udev 이름이 안 생겼음. dev 11-16 (#309) 규칙은 허브 칸 기준이라 Jetson 에 설치되면 `/dev/lidar_front` · `rear` 가 생김. ⚠ Jetson `/etc/udev/rules.d/99-sensors.rules` 는 11-18 에 **1 번 칸 = 앞 · 2 번 칸 = 뒤** 로 바뀜 (예전 2 = 앞 · 1 = 뒤, 백업 `…before_codex_1118`) — 앞 · 뒤는 손으로 가려서 확인 | `ls -l /dev/lidar_*` 가 있으면 인자 없이, 없으면 by-path 로 직접 (11-15: 앞 2.1.2 · 뒤 2.1.1) |
| 라이다 방향 | dev 1b9266c (PR #304) 부터 URDF 가 실물 라이다 180° 를 고침 | 따로 돌리지 않음 (시연 도구가 자동 판단) |
| lifecycle 응답 지연 | Jetson 에서 노드 켜기 응답이 시간 초과로 사라져 `map_server` · `controller_server` · `behavior_server` · `bt_navigator` 가 `inactive` 로 남는 일이 잦음 (#276 같은 종류) | `ros2 lifecycle set /<노드> activate` (시연 도구는 자동으로 함) |
| 출발 자리 | 11 cm 벗어나 놓으면 출발 확인에서 멈춤 · 계획 경로와 어긋남 | 출발 자리 **(0.694, 0.950) · 앞이 신호등 쪽 (180°)**, 2 ~ 3 cm 안 |
| 실물이 덜 돎 | 벨트 v7 (1 : 2.0) 로 바퀴 토크가 줄어 꺾기는 명령의 **79 %**, 제자리 회전은 **66 ~ 80 %** 만 돎 | 시연 필터를 실제 각도로 세게 고침 · 실물만 이어진 후진 0.50 m |
| L1 ~ L3 자리 | 칠판 실측 L 상자가 지도와 다름 (L3 는 그대로 가면 상자 면까지 **3.4 cm**) | 시연 계획을 실측 L 로 다시 계산 (`7cm_실측L_어제순서`) — 6 장 |
| 지도 | 칠판 실측과 벽 끝 몇 곳이 3 ~ 14 cm 다르지만, 실물 라이다는 발제 지도와 잘 맞음 (RViz · 출발 자리에서 점 98 % 가 벽 4 cm 안) | **팀 기준 지도 = 발제 지도 그대로** (11-15 결정, 6 장) |
| 팔 배터리 | 11.06 V (알람 11.1 V 아래) 까지 내려감 | 시험 전 충전된 배터리로 |

## 1. 사람 · 준비물 · 터미널

| 역할 | 하는 일 |
|---|---|
| 리모컨 담당 | RC-100 을 쥐고 로봇 옆을 따라다님. 이상하면 1 |
| Jetson 조작 | 터미널을 띄우고 Enter · y · q, 미션 시작 · 멈춤 |
| 노트북 감시 | RViz 로 위치 추정 확인, 로그 보기 |

준비물: 충전된 LiPo 3S 2 개 (주행 OpenCR · 팔 OpenCR 에 하나씩) · RC-100 (건전지) · Jetson 화면 · 키보드 (권장) · 노트북 (같은 Wi-Fi).

| 어디 | 이름 | A. 주행 시연 | B. 전체 미션 |
|---|---|---|---|
| Jetson | J1 | 바퀴 · 팔 하드웨어 · 리모컨 (`base_bringup`) | **bringup 전체 (하나로 모두)** |
| Jetson | J2 | 라이다 두 대 (`scan_rotate.py`) | 미션 상태 보기 · 시작 · 멈춤 |
| Jetson | J3 | 위치 추정 · 팔 주행 자세 (`bringup` 일부) | (필요할 때) lifecycle 확인 |
| Jetson | J4 | 점검 · 시연 실행 (`run_demo.sh`) | – |
| 노트북 | P1 | 기록 가져오기 · 상태 | 같음 |
| 노트북 | P2 | RViz 감시 (`scan_view.sh amcl`) | 같음 |

- Jetson 화면 · 키보드로 터미널을 여는 것이 가장 안전합니다. 노트북 ssh (`ssh cart11@cart11.local`) 로 열면 Wi-Fi 가 끊길 때 그 터미널의 프로그램도 꺼집니다.
- **A 와 B 는 같이 띄우지 않습니다** (같은 하드웨어 · 라이다 · Nav2 이름을 씀). 바꿀 때는 모든 터미널을 Ctrl+C 로 끕니다.

Jetson 터미널을 열면 **매번 먼저**:
```bash
source /opt/ros/lyrical/setup.bash
source ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash
```
- ROS 2 와 우리 저장소 빌드 결과를 이 터미널에 불러옵니다. 빠뜨리면 `Package '…' not found`.

## 2. 공통 준비 (A · B 둘 다)

### 단계 1 — Jetson 판 확인 (읽기만)

```bash
cd ~/Lv2_Cartlidar_Assignment && git log -1 --oneline && git status --short | head
grep -o 'arm_standby_m.._tick">[0-9,]*' ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/opencr_hardware/share/opencr_hardware/urdf/opencr.ros2_control.xacro
python3 -c "import json;print(json.load(open('/home/cart11/demo_v2/demo_plan.json'))['plan'])"
```
- 첫 줄: 저장소 판. 11-16 아침 기준 **dev 1a381b2** 이상 (#316 팔 v6 · #309 라이다 udev · #306 이 폴더 포함) 이어야 합니다.
- 둘째 줄: 빌드된 팔 대기 범위. 첫 숫자 **3425** · J5 끝 (다섯째) **2246** 이면 11-16 판입니다. 다르면 빌드 담당에게 (`opencr_hardware` · `control` 다시 빌드).
- 셋째 줄: 시연 계획 이름. **`7cm_실측L_어제순서`** 이면 L 실측이 들어간 판입니다.
  `길모음7cm_어제순서` (L 실측 전) 가 나오거나 `~/demo_v2` 가 없으면 저장소의 이 폴더를 복사합니다 (아래). 복사 전 판으로는 **S6 ~ S9 (L 구간) 을 하지 않습니다** (L3 상자 면까지 3.4 cm).

```bash
# Jetson — 저장소 workspace/tools/drive_demo 를 ~/demo_v2 로 (runs/ 기록은 그대로 남음). 먼저 git pull
mkdir -p ~/demo_v2 && cp -r ~/Lv2_Cartlidar_Assignment/workspace/tools/drive_demo/. ~/demo_v2/ && chmod +x ~/demo_v2/*.sh ~/demo_v2/scan_view/*.sh
```

### 단계 2 — 전원

1. **배터리 먼저, USB 나중.** 두 OpenCR 에 배터리를 연결한 뒤 Jetson · USB 를 켭니다.
2. **Jetson 을 껐다 켰거나, USB 가 먼저 켜졌으면: 배터리를 켠 채 두 OpenCR 의 RESET 버튼을 한 번씩.** RESET 은 토크를 켜지 않고 움직이지 않습니다.
3. 상자 6 개와 L 상자 3 개가 제자리인지 봅니다 (어제 · 오늘과 같은 자리).

### 단계 3 — 팔 대기 자세 · 장치 점검 (읽기만, 아무것도 안 움직임)

```bash
cd ~/demo_v2 && ./jetson_check.sh opencr
ls -l /dev/serial/by-path/
```
- `jetson_check.sh opencr`: OpenCR 두 대의 상태 프레임만 1 초 읽음 → 보드 · 펌웨어 · **배터리 전압** · **오류** · **팔 tick 이 대기 범위 안인지**.
- 나와야 하는 것: 두 보드 `err -` (또는 `CMD_TIMEOUT` 만) · 팔 여섯 칸 범위 안 (❗ 없음) · 배터리 11.1 V 넘음.
- `ls -l /dev/serial/by-path/`: USB 자리. 라이다 `…usb-0:2.1.1:1.0-port0` · `…2.1.2…` 두 줄, OpenCR `2.1.3` · `2.1.4`, TFmini `2.1.6` (11-15 밤 기준).

팔이 범위 밖이면 (예: `J1 3138❗(3425~3539)`), tick 을 보면서 손으로 맞춥니다 (읽기만, 토크 꺼짐):
```bash
python3 ~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py monitor --port /dev/serial/by-path/platform-3610000.usb-usb-0:2.1.4:1.0 --seconds 120
```
- `2.1.4` 가 팔 보드 USB 자리입니다 (주행 보드로 나오면 `2.1.3`). 가운데 (J1 3482 · J5 2048) 쪽으로, 범위 끝은 피합니다. 끝나면 Ctrl+C.
- `jetson_check.sh` 는 **소스** 범위를 읽고 하드웨어는 **빌드된** 범위를 씁니다. 단계 1 의 grep 과 다르면 빌드 담당에게.

| 문제 | 원인 · 해결 |
|---|---|
| `MOTOR_CONFIG` · `FAULT` | 배터리보다 USB 가 먼저 → 배터리 켠 채 그 보드 RESET |
| 보드가 한 대만 | USB 케이블 · 허브 → Jetson 본체 포트 |
| 배터리 11.1 V 아래 | 바꿈. 부저: 11.1 V 아래 30 초마다 1 번, 10.5 V 아래 2 초마다 3 번 |
| `팔 보드 펌웨어 v5 ≠ 저장소 기대 v6` | 팔 보드에 11-16 펌웨어 (관절 1 π 보정) 가 안 올라감 → 팔 담당에게 (`tools/build_firmware.sh arm --upload`) |
| 라이다 by-path 가 2.1.1 · 2.1.2 가 아님 | USB 자리가 또 바뀜 → 아래 시험의 라이다 장치를 새 이름으로 (앞 · 뒤는 손으로 가려서 확인) |

## 3. A. 주행 시연 시험

### 단계 4 — J1: 바퀴 · 팔 하드웨어 · 리모컨

```bash
ros2 launch opencr_hardware base_bringup.launch.py rc100:=true obstacle_lock:=false loc_lock:=false 2>&1 | tee ~/demo_base.log
```
- ros2_control 이 OpenCR 두 대와 이야기함 (바퀴 `diff_drive_controller` · 팔 `arm_controller`) + `twist_mux` (속도 명령 우선순위) + 리모컨.
- `obstacle_lock:=false`: TFmini 정면 잠금 끔 (문턱 0.25 m 가 상자 앞 정지 0.227 m 보다 길어 켜면 상자마다 막힘). `loc_lock:=false`: 위치 잃음 잠금 끔.
- ⚠ 이때부터 팔 토크가 켜져 자세를 붙잡습니다.
- 나와야 하는 것: `팔 대기 자세 확인 — 여섯 칸 모두 범위 안` → `Configured and activated diff_drive_controller` → **리모컨 2 + 3** (E-stop 풀기).

| 문제 | 원인 · 해결 |
|---|---|
| `팔이 대기 자세가 아님 — … 거부합니다: jointN: … tick` | 단계 3 의 monitor 로 그 관절을 맞추고 다시 |
| `OpenCR 부팅 점검 실패 (모터 응답 …)` | 보드 FAULT → Ctrl+C · 두 보드 RESET · 다시 |
| `CMD 송신 실패` 뒤 모두 꺼짐 | OpenCR USB 끊김 (`journalctl -k -f`) → 본체 포트 |
| 리모컨이 안 먹음 | 리모컨 전원 · 블루투스 LED → `ros2 topic echo /rc100/raw` |

### 단계 5 — J2: 라이다

```bash
python3 ~/demo_v2/scan_rotate.py
```
- 인자 없이 = udev 이름 `/dev/lidar_front` · `/dev/lidar_rear` (#309 규칙). `ls -l /dev/lidar_*` 로 있는지 확인.
- udev 이름이 없을 때만 앞 · 뒤 장치를 인자로: `ls -l /dev/serial/by-path/` 에서 `…-port0` 두 개 (USB 허브 자리에 따라 `2.1.x` · `2.2.x` 로 바뀜, 11-20 실물 = 앞 `2.2.1` · 뒤 `2.2.2`).
- 나와야 하는 것: `라이다 그대로 (돌리지 않음) 내보내는 중 … [URDF 에 라이다 yaw 보정 있음 (PR #304) → 돌리지 않음]`.
- 팀원의 `sensors.launch.py` 가 켜져 있으면 라이다를 서로 잡습니다 → 그것을 끔.

### 단계 6 — J3: 위치 추정 · 팔 주행 자세

```bash
ros2 launch bringup bringup.launch.py mode:=drive use_hardware:=false use_sensors:=false use_perception:=false use_tfmini:=false \
  use_loc_monitor:=false 2>&1 | tee ~/demo_loc.log
```
- 지도 · 라이다 몸체 가림 · 합치기 (`/scan_merged`) · AMCL · EKF 를 띄움. 하드웨어 (J1) · 라이다 드라이버 (J2) · 인식 · 계획기 · 팀 Nav2 · mission_node 는 안 띄움.
- `mode:=drive` 가 팀 Nav2 · 계획기를 끄고 AMCL 을 켭니다 (예전의 `use_nav2:=false use_planner:=false use_amcl_node:=true` 를 대신함). 아래 "주행 모드 / 미션 모드" 참고.
- ⚠ **켜질 때 팔이 대기 → 주행 자세로 펴집니다** (J3 → J2 → J4 → J1 → J5). 손을 빼고, 리모컨 1 준비. 계산상 틈이 거의 없는 곳이 있어 지켜봅니다.
- 10 ~ 30 초 걸립니다.
- 위치 보호 `loc_guard` 는 이 bringup 인자 `loc_guard:=off|shadow|active` (기본 `shadow`) 로 켜고 끕니다. 자세한 것은 `workspace/ros2_ws/src/localization/README.md` 를 봅니다.

### 단계 7 — J4: 위치 감시 끄기 · 점검

```bash
pkill -INT -f "lib/(localization|perception)/loc_monitor"     # 단계 6 에서 use_loc_monitor:=false 면 이미 없음
cd ~/demo_v2 && ./check_jetson.sh
```
- `loc_monitor` (팀 위치 감시) 는 실물에서 너무 예민해 시연 때는 끕니다. dev 11-16 (#310) 부터 C++ 판 (`lib/perception/loc_monitor`) 이 기본이라, 예전 `lib/localization/…` 만 찾는 명령으로는 안 꺼집니다.
- `check_jetson.sh` (읽기만): Nav2 패키지 · 노드 · 토픽 · **라이다 방향** · 위치 · 배터리. 마지막 줄 **`== 점검 통과`**.

| ✗ 항목 | 해결 |
|---|---|
| `노드 … 안 보임` | 해당 터미널 (twist_mux · controller_manager = J1, lidar_node = J2, amcl · map_server · ekf = J3) |
| `토픽 /map 값이 안 옴` | `ros2 lifecycle get /map_server` 가 `inactive` 면 `ros2 lifecycle set /map_server activate` |
| `라이다 두 번 돌림` / `안 돌림` | J2 를 `SCAN_ROT=0` / `SCAN_ROT=1` 을 앞에 붙여 다시 |
| `/battery/…: .nan — 하드웨어가 안 켜짐` | J1 이 실패한 것 → J1 로그 확인 |
| `E-stop: data: true` | 리모컨 2 + 3 |

### 단계 8 — 노트북: RViz 감시

```bash
<저장소>/workspace/tools/drive_demo/scan_view/scan_view.sh amcl     # 노트북
```
- ssh 로 Jetson 의 합친 라이다 · AMCL 자세를 받아 지도 위에 그림. **빨간 점이 지도 벽과 겹치면 위치 추정이 맞음.** 5 cm 넘게 벗어나면 리모컨 1.
- 노트북에 ROS 2 lyrical · `nav2_map_server` · `rviz2` 와 Jetson ssh 키가 있어야 합니다. Jetson 주소가 다르면 `JETSON=<사용자@주소>`, 다른 지도면 `MAP=<지도.yaml>` 을 앞에 붙입니다.

### 단계 9 — J4: 시연 실행

로봇을 **출발 자리 (0.694, 0.950) · 앞이 신호등 쪽 (180°)** 에 2 ~ 3 cm 안으로 놓고:
```bash
./run_demo.sh real slow step init 0-0     # 첫 구간만
./run_demo.sh real slow step 1-9          # 나머지, 구간마다 Enter
./run_demo.sh real slow step init         # 처음부터 끝까지 구간마다 Enter
BOX_APPROACH=dry ./run_demo.sh real slow step init 0-0   # 상자 앞 다가가기는 재기만 (안 움직임)
BOX_APPROACH_AT=box1 ./run_demo.sh real slow step init 0-0   # box1 에서만 다가가기
```
- `run_demo.sh` 가 하는 일: 시연 Nav2 노드 (controller_server · behavior_server) 와 규칙 v2 필터를 띄우고 **active 인지 확인 (아니면 켬)** → `demo_follow.py` 가 계획을 따라감 → 끝나면 모두 끔.
- `slow` = 전진 0.20 · 후진 0.10 m/s · 제자리 0.4 rad/s (`full` = 0.30 · 0.15 · 0.5). `step` = 구간마다 Enter. `init` = 출발 자세를 AMCL 에 알림. `4-9` = 그 구간만.
- 환경 변수 `BOX_APPROACH` (기본 `1`) = 상자 정지점에서 앞 끝 ↔ 상자 5 cm 까지 다가감. `0` = 끔 · `dry` = 재고 로그만. `BOX_APPROACH_AT=box1,L2` = 그곳만 (빈 값 = 9 곳 전부). 아래 "상자 앞 다가가기".
- 시작 때 `✓ map_server active` · `✓ controller_server active` · `✓ behavior_server active` 가 나와야 합니다.
- **출발 확인:** `지금 위치 … 출발과 x cm · AMCL σ …` → RViz 에서 빨간 점이 벽과 겹치면 `y` + Enter. 출발과 3 cm 넘게 다르면 로봇을 다시 놓습니다.
- **정지점마다 팔 동작:** `▶ … Enter 출발` 이 나와 있는 동안 팔 동작을 하고 Enter.
- **안전장치 (11-15 밤 추가):** 전진 · 후진 중 AMCL 자리가 계획 경로에서 **10 cm** 넘게 벗어나면 그 조각을 취소하고 멈춥니다 (`✗ 경로에서 xx cm 벗어남 — 멈춤`). 실물에서 벽에 닿은 S6 (경로에서 24 cm) 때문에 넣은 것입니다.
- **방향 맞추기 건너뛰기 (11-16 추가):** 조각을 시작할 때 방향 맞추기 (작은 제자리 회전) 를 Nav2 가 거부하면 (`Collision Ahead` · 시간 초과) `· 시작 방향 맞추기 건너뜀 (…)` 을 찍고 조각을 그대로 진행합니다. 예전에는 그 구간이 실패로 끝났습니다.
- **되찾기 (11-16 추가):** 조각이 실패하면 (경로 이탈 멈춤 · Nav2 진행 실패 · 충돌 예상, Ctrl+C 는 제외) 그 조각을 최대 **2 번** 다시 합니다. 한 번마다:
  1. `↺ 되찾기 1/2 (이유)` 를 찍고 멈춥니다.
  2. 자기 위치를 다시 찾습니다 (AMCL 에 "움직이지 않아도 다시 계산" 요청 5 번, σ 를 알림).
  3. 그 조각에서 지나온 길을 거꾸로 **최대 0.30 m** 되돌아갑니다 (전진 조각이면 후진).
  4. 다시 위치를 찾고 그 조각을 다시 합니다.
  - 그래도 안 되면 step 모드에서는 `Enter 한 번 더 · s 이 조각 건너뛰기 · q 구간 끝` 을 묻습니다. step 이 아니면 예전처럼 그 구간이 실패입니다.
  - 끄려면 `NO_RECOVER=1 ./run_demo.sh …`. 결과 줄 끝에 `되찾기 n 번` 이 붙습니다.

**중간 구간부터 다시** — 로봇을 그 앞 구간의 정지점에 놓고 `init` 을 붙여 실행합니다 (`init` 이 그 정지점을 AMCL 에 알림):

| 구간 | 이름 | 끝 정지점 (지도 x, y) · 방향 | 그다음부터 다시 |
|---|---|---|---|
| S0 | 출발 → box1 | (1.805, 1.085) · 180° | `init 1-9` |
| S1 | box1 → box2 | (3.655, 2.885) · 90° | `init 2-9` |
| S2 | box2 → box6 | (3.290, 3.921) · 0° | `init 3-9` |
| S3 | box6 → box3 | (3.021, 4.930) · 90° | `init 4-9` |
| S4 | box3 → box5 | (1.918, 5.280) · 0° | `init 5-9` |
| S5 | box5 → box4 | (0.855, 4.985) · 90° | `init 6-9` |
| S6 | box4 → L3 | (3.658, 4.945) · 90° | `init 7-9` |
| S7 | L3 → L2 | (1.953, 3.675) · 0° | `init 8-9` |
| S8 | L2 → L1 | (3.370, 0.968) · 0° | `init 9-9` |
| S9 | L1 → 출발 구역 A1 | (0.734, 0.950) | – |

방향: 0° = 동쪽 (+x), 90° = 북쪽 (+y), 180° = 서쪽 (신호등 쪽).

| 보이는 것 | 원인 · 해결 |
|---|---|
| `AMCL 자세 없음` · `AMCL σ … 넘음` | 로봇을 정지점에 정확히 놓고 `init` 을 붙여 다시 |
| `출발 자리와 xx cm` | 로봇을 다시 놓음 (RViz 로 확인) |
| `✗ … action server 없음` | 시연 Nav2 노드가 안 켜짐 → 다시 실행 (스크립트가 activate 를 25 번까지 시도) |
| `✗ 경로에서 xx cm 벗어남 — 멈춤` | 위치 추정이 틀렸거나 실제로 밀린 것 → RViz 로 보고, 로봇을 그 구간 시작 정지점에 놓고 `init N-9` |
| `Failed to make progress` | E-stop (2 + 3) · 리모컨 방향 버튼 · 필터 잘림 (`nav.log` 의 `v2_filter 잘림 횟수`) |
| `✗ Sn …: 정지점 어긋남 5 ~ 8 cm` | 실물이 덜 꺾어서 생김. 다음 구간은 그대로 감 (실패 아님으로 봐도 됨) |
| 제자리 회전에서 멈칫 · 덜 돎 | 벨트 v7 토크 → 리모컨 1 → 기록을 제어 담당에게 |
| `⚠ 상자 앞 다가가기 안 함: 상자 못 찾음` | 앞 라이다 스캔면이 상자 높이에 안 걸림 → 라이다 높이 (바닥 위 약 60 mm) · URDF `laser_front` z (0.0525) 확인. `ros2 topic echo /scan_front --once` 에 상자 쪽 거리가 나오는지 |
| `✓ 상자 앞 틈` 이 줄자와 다름 (1 cm 넘게) | `base_link` → `laser_front` TF (x · yaw) 또는 라이다 거리 바이어스 → 아래 "실물 보정" |
| 다가가다 멈칫 · 덜 감 | v2_filter 의 정지 앞 감속 · 잘림 (`nav.log` 의 `v2_filter 잘림 횟수`) → 기록을 제어 담당에게. 그 자리는 `BOX_APPROACH_AT=` 에서 빼고 진행 |

기록: Jetson `~/demo_v2/runs/<월일_시각>_실물_…/` — `follow.log` (화면) · `nav.log` (Nav2 · 필터) · `track_live.csv` (0.1 s 마다 위치 · 명령, Ctrl+C 로 끊겨도 남음) · `track.csv` · `course_result.json` · `bag/` (rosbag, 아래).

#### rosbag 자동 기록 (11-21 추가)

- `run_demo.sh` 가 실행마다 기록 폴더 안 `bag/` 에 `ros2 bag record` 를 함께 띄웁니다 (로그 `bag.log`). 형식은 mcap (되면 zstd 압축), 안 되면 기본값입니다. 1 GiB 마다 파일을 나눕니다.
- 토픽: `/amcl_pose` · `/odometry/filtered` · `/scan_merged` · `/scan_front` · `/tf` · `/tf_static` · `/particle_cloud` · `/loc_guard/pose` · `/loc_guard/event` · `/cmd_vel_raw`. 시뮬은 `/odom` · `/scan` · `/clock` 을 더하고 sim time 으로 기록합니다.
- 끝날 때 (끝 · Ctrl+C) SIGINT 로 닫습니다. 끝에 `rosbag: …/bag (크기)` 가 나오면 된 것입니다. `⚠ rosbag 이 깔끔히 안 닫힘` 이면 `ros2 bag reindex <bag 폴더>` 로 살립니다.
- 끄기: `DEMO_BAG=0 ./run_demo.sh …`. `/particle_cloud` 만 빼기 (크기가 클 때): `DEMO_BAG_PARTICLES=0`.
- 노트북으로 가져와 재생:
```bash
scp -r <사용자@Jetson>:~/demo_v2/runs/<기록 폴더> 2026-11-12_기준주행/runs/
ros2 bag info  2026-11-12_기준주행/runs/<기록 폴더>/bag
ros2 bag play  2026-11-12_기준주행/runs/<기록 폴더>/bag --clock     # RViz 는 use_sim_time:=true
```
- 위치가 튈 때 비교할 토픽: `/amcl_pose` (AMCL 그대로) ↔ `/loc_guard/pose` (위치 보호를 거친 값) ↔ `/odometry/filtered` (EKF, 튀지 않는 기준). 튄 순간은 `/loc_guard/event` 와 `/particle_cloud` 퍼짐으로 봅니다.

### 주행 모드 / 미션 모드

bringup 은 `mode` 인자로 두 모드 중 하나로 뜹니다.

| | 주행 모드 `mode:=drive` | 미션 모드 (기본, 인자 없이) |
|---|---|---|
| 쓰는 곳 | A. 주행 시연 (단계 6) | B. 전체 미션 · 경기 |
| 팀 Nav2 · 계획기 | 안 띄움 (인자로 켜도 꺼짐) | 띄움 |
| mission_node | 안 띄움 | 띄움 |
| drive_node · manipulator_node · TFmini | 띄움 (켤 때 팔을 주행 자세로) | 띄움 |
| AMCL (`use_amcl_node` 기본) | `use_localization` 을 따름 | `use_nav2` 를 따름 |

- 지금 모드는 `/system/mode` (std_msgs/String, reliable · transient_local, 1 Hz) 로 나옵니다: `ros2 topic echo --once --qos-durability transient_local /system/mode`.
- `./run_demo.sh real` 은 `/system/mode` 가 `mission` 이면 `✗ 미션 모드 bringup 이 떠 있음` 을 찍고 시작하지 않습니다 (팀 Nav2 · mission_node 와 시연 노드가 함께 바퀴를 움직이지 않게). 단계 6 을 `mode:=drive` 로 다시 띄웁니다. 토픽을 못 받으면 (옛 bringup) 경고만 찍고 진행합니다.

#### 장애물 정지 · 막힘 우회 (11-19 추가)

- **`SAFETY` (기본 1)** — `run_demo.sh` 가 시연 Nav2 에 `params_demo_safety.yaml` 을 덧씌우고 라이다 장애물 층 · `collision_monitor` (v2_filter 뒤) · `planner_server` (Detour) 를 함께 띄웁니다. 정지 띠는 속도에 따라 바뀝니다 (전진 빠름 · 중간 · 느림 앞 띠, 후진 뒤 띠, 제자리 회전 띠).
  - `SAFETY=0 ./run_demo.sh …` 이면 예전 그대로 (정지 띠 · 우회 없음) 돌고, `demo_follow.py` 에 `--no-detour` 를 넘깁니다.
- **막힘 우회** (`demo_follow.py`, 전진 조각만) — 정지 띠에 걸려 멈추면:
  1. 최대 **5 s** 기다립니다 (그 사이 비키면 그대로 이어감).
  2. 지나온 길로 **0.15 m** 후진합니다.
  3. planner_server 로 우회 경로를 받습니다. 합류점은 조각에서 **1.1 · 1.3 · 1.5 m** 앞 (또는 조각 끝) 이고, 원래 조각에서 **0.4 m** 넘게 벗어나는 경로는 받지 않습니다.
  4. 우회 경로를 따라간 뒤 합류점부터 원래 조각을 이어 갑니다.
  - 구간당 최대 **2 번**. 후진 실패 · 우회 경로 없음 · 후진 조각이면 기존 되찾기로 넘어갑니다.
  - 끄려면 `DEMO_DETOUR=0 ./run_demo.sh …` (또는 `demo_follow.py --no-detour`). 시험용 `DEMO_TEST_BLOCK=1` = 첫 긴 전진 조각 (> 0.6 m) 이 3 s 뒤 막힌 것처럼 (전체에서 한 번).
- **한계:** 비켜 갈 옆 공간이 이어져 있는 곳은 시연 경로의 약 19 % 뿐입니다 (S0 · S1 · S3 · S5 는 0 %). 그 밖에서는 우회 경로가 안 나와 기다린 뒤 되찾기 (step 모드면 건너뛰기) 로 넘어갑니다.
- 기록: `course_result.json` 의 구간마다 `detour_n` (우회 횟수), `tries` 에 상태 `"detour"` 줄 (후진 · 경로 · 결과).

#### 상자 앞에서 정지 띠 · HOLD 끄기 (11-20 추가, 실물 미시험)

- 11-20 에 실물 앞 LDS-03 을 **1.5 cm 낮췄습니다** (`laser_front` z 0.0675 → 0.0525, 스캔면 바닥 위 약 60 mm). 이제 높이 70 mm 상자가 앞 라이다에 보입니다. 뒤 라이다는 그대로입니다.
- 시연 정지점은 대부분 상자 바로 앞입니다 (앞 끝 ↔ 상자 −8.8 ~ +2.5 cm). 그래서 다가가는 동안 정지 띠와 HOLD 가 상자에 걸려 우회로 빠집니다.
- 그래서 `demo_follow.py` 는 목적지가 상자 (`box1` ~ `box6`, `L1` ~ `L3`) 인 구간에서 로봇 (map 자세) 이 정지점 **0.40 m 안**에 들어오면 아래 두 가지를 끕니다.
  - `collision_monitor`: `/collision_monitor/toggle` 로 끔. 끄면 명령이 그대로 통과합니다.
  - HOLD 판단
- 다시 켜는 때: 0.42 m 밖으로 나갈 때 · 구간이 끝날 때 · Ctrl+C · 오류로 끝날 때. 켜고 끌 때마다 로그 한 줄 (`정지 띠 (collision_monitor) · HOLD 끔` / `다시 켬`) 을 남기고, `course_result.json` 의 `box_quiet_n` 에 끈 횟수를 적습니다.
- 거리는 `demo_follow.py --box-quiet-m 0.40` 으로 바꿉니다. `0` 이면 이 기능을 끕니다. S9 (→ A1) 에는 적용하지 않습니다.
- ⚠ **끈 동안 남는 보호:**
  - 정지 앞 감속 (v2_filter `v ≤ √(0.05² + 0.6·(d − 0.05))`): 0.40 m 에서 0.46 · 0.16 m 에서 0.26 · 0.10 m 에서 0.18 · 마지막 5 cm 는 0.05 m/s 입니다. 이 계산상 상한은 max 전진 0.45 m/s 보다 0.16 m 안에서만 낮습니다.
  - 경로 이탈 멈춤 (10 cm)
  - 리모컨
- 시연 base_bringup 은 `obstacle_lock:=false` 라 **TFmini 비상 정지가 없습니다**. 상자 앞 0.40 m 안에 사람이나 물건이 들어와도 로봇이 멈추지 않으니 리모컨을 손에 들고 지켜봅니다.

#### 상자 앞 다가가기 (11-21 추가, 실물 미시험)

**무엇을 하나** — 상자 구간 (`box1` ~ `box6`, `L1` ~ `L3`, S0 ~ S8) 의 정지점에서 상판 앞 끝 (`base_link` x 0.1328) 과 상자 앞면 사이를 **5 cm** 로 맞춥니다.

1. 마지막 전진 조각을 정지점 **0.30 m** 앞에서 끊습니다 (재는 자리). 마지막이 전진이 아니면 도착한 뒤 최대 **0.35 m** 곧게 후진합니다.
2. 그 자리에서 원본 `/scan_front` 3 ~ 5 장으로 상자 앞면을 잽니다. 선분을 맞추고, 길이가 205 ± 25 mm 인지 보고, 두 끝점 가운데를 씁니다.
3. 상자 앞 **0.15 m** 자리로 가면서 옆과 각도를 맞춥니다 (돌기 → 전진 → 앞면 법선으로 돌기).
4. 다시 잽니다. 마지막은 오도메트리로 곧게 다가가 틈 5 cm 에 섭니다. 속도는 0.05 m/s 이하입니다.

- 마지막을 라이다로 안 보는 까닭: LDS-03 최소 거리가 0.12 m 이고 라이다가 앞 끝과 거의 같은 자리라, 5 cm 에서는 상자가 안 보입니다.
- `/scan_merged` 를 안 쓰는 까닭: 차체 마스크 (x ≤ 0.285) 가 가까운 상자를 지웁니다.
- 정지점 어긋남 판정 (5 cm) 은 다가가기 **전** 값으로 합니다.

**켜기 · 끄기** — 기본은 켬입니다.

| 환경 변수 | 값 | `demo_follow.py` 인자 |
|---|---|---|
| `BOX_APPROACH` | `1` 켬 (기본) · `0` 끔 · `dry` 재고 로그만 | `--box-approach on\|off\|dry` |
| `BOX_APPROACH_AT` | `box1,L2` = 그곳만 · 빈 값 = 9 곳 전부 | `--box-approach-at box1,L2` |
| `BOX_APPROACH_LOOSE` | `1` 느슨한 판정 · `0` 기본 | `--box-loose` |
| – | 목표 틈 (기본 0.05 m) | `--box-gap 0.05` |
| (자동) | 회전 · 거리에 쓰는 오도메트리 (실물 `/odometry/filtered` · 시뮬 `/odom`) | `--odom-topic` |

- `BOX_APPROACH_LOOSE=1` 은 실물 dry 로그의 실패 이유에 찍히는 `RMS x mm` 가 늘 7 mm 를 넘을 때 켭니다 (라이다 잡음 약 1 cm). 대신 상자 옆 2 cm 틈에 놓인 퍽은 상자와 구분하지 못합니다.
- 재는 라이다는 `/scan_front` 고정입니다 (`--box-scan-topic`, 넘기지 않음).

- 기록 폴더 이름 끝에 `_접근끔` (0) · `_접근측정` (dry) 이 붙습니다. `settings.txt` 의 "추가 인자" 에도 적힙니다.
- step 모드에서는 움직이기 전에 `▶ … Enter 실행 · s 건너뛰기` 를 묻습니다.

**화면에 보이는 것**
```
    · 상자 정면 재기: 틈 24.3 cm · 옆 +1.8 cm · 기울기 −2.1° (점 42 개)
    ✓ 상자 앞 틈 5.2 cm · 옆 +0.3 cm
```
- 구간 요약 줄 끝에 ` · 상자 앞 틈 5.2 cm (보정 4.3 cm)` 가 붙습니다. `course_result.json` 의 구간마다 `approach` 에 틈 · 옆 · 기울기를 적습니다.

| `approach` 키 | 뜻 |
|---|---|
| `mode` · `target` | on / dry · 목적지 (예 `box1`) |
| `how` | `끊기` (마지막 전진을 끊음) · `후진` (도착 뒤 후진) |
| `status` · `why` | `done` · `fallback` (재기 실패 → 기존 동작) · `skip` (step 에서 s) · `dry` · 그 이유 |
| `pre` | 다가가기 전 자세 (어긋남 판정에 쓰는 값) |
| `before` · `mid` · `after` | 첫 재기 · 0.15 m 자리에서 다시 재기 · 끝 — 각각 `gap_cm` · `lat_cm` · `yaw_deg` |
| `plan` · `back_m` | 기동 계획 (돌기 · 전진 · 돌기) · 후진한 거리 [m] |

**안 움직이는 경우** — 아래면 움직이지 않고 `⚠ 상자 앞 다가가기 안 함: <이유> — 계획 정지점 그대로` 한 줄을 남깁니다. 그다음은 기존 동작 (남은 조각 · 정지점 방향 맞추기) 대로 갑니다. 구간 실패가 아닙니다.
- 상자를 못 찾음 · 앞면 길이가 205 ± 25 mm 밖 · 기울기 25° 넘음 · 옆 6 cm 넘음 · 시간 초과

**남은 위험**
- 계획 정지점과 실제 상자가 10 cm 넘게 다르면 상자를 못 찾고 기존 동작으로 갑니다.
- 회전은 오도메트리로 닫습니다. 5 cm 에서는 라이다로 다시 잴 수 없으니 끝 자세는 줄자로 확인합니다.

⚠ **안전**
- 상자 정지점 0.40 m 안에서는 정지 띠 · HOLD 가 꺼지고 TFmini 비상 정지도 없습니다 (위 "상자 앞에서 정지 띠 · HOLD 끄기"). 다가가는 동안 상자 앞에 손을 넣지 않습니다.
- 리모컨 담당은 **리모컨 1** 에 손을 얹고 지켜봅니다. 시작 전에 리모컨 전원 · 블루투스 LED 를 확인합니다.

**처음 시험 순서**
1. 로봇을 출발 자리에 놓고 `BOX_APPROACH=dry ./run_demo.sh real slow step init 0-0`. box1 앞에서 `· 상자 정면 재기` 가 찍히면 로봇을 그대로 둔 채 앞 끝 ↔ 상자 틈 · 옆을 줄자로 재어 비교합니다 (±1 cm 안).
2. 상자를 치우고 다시 dry → `⚠ … 상자 못 찾음` 이 나오는지 봅니다.
3. `BOX_APPROACH_AT=box1 ./run_demo.sh real slow step init 0-0` 로 한 곳만 실제로 다가갑니다. 멈춘 뒤 틈을 줄자로 잽니다.
4. 9 곳 `slow step` → `max step` → `max`.

**실물 보정** — 상자를 앞 라이다 정면 25 ~ 30 cm 에 두고 dry 로 잽니다.
- 기울기가 0° 에서 벗어나면 `laser_front` yaw, 틈이 줄자와 계속 같은 만큼 다르면 거리 바이어스입니다. 값을 제어 담당에게 알립니다.
- `ros2 topic echo /scan_front --once | grep range_min` 으로 최소 거리 (0.12 m 쯤) 를 확인합니다.

#### 반경 넓힌 계획 `demo_plan_wide.json` (11-19 추가, 실물 미시험)

정지점은 `demo_plan_max.json` 과 같고 구간 길만 `widen_plan.py` (하이브리드 A*) 로 새로 만든 계획입니다. 큰 반경 (전진 0.4 ~ 0.5 m, 상자 칸 들어가기만 0.25) · 제자리 회전 · 방향 바뀜 최소 · 정지점 앞 0.26 m 곧게 들어가기. S2 · S9 는 원래 길 그대로 (시뮬에서 새 길이 더 나빴음).
```bash
PLAN=~/demo_v2/demo_plan_wide.json ./run_demo.sh real max step init 0-0     # 실물 첫 구간
PLAN=$PWD/demo_plan_wide.json ./run_demo.sh sim max                          # 노트북 시뮬
```
| SAFETY=1 시뮬 (11-19) | `demo_plan_max.json` | `demo_plan_wide.json` |
|---|---|---|
| 전체 | 155.6 s | 136.6 s |
| 계획 제자리 회전 | 120° | 60° |
| 조각 시작 방향 맞추기 | 14 번 (127°) | 8 번 (73°) |
| 정지점 방향 맞추기 | 91° | 38° |
| HOLD · 되찾기 | S0 에서 2 · 2 (38 s) | 0 |

- 줄어든 시간의 대부분은 S0 (38.3 → 15.1 s). 원래 S0 은 출발 구역 작은 원이 벽에 붙어 collision_monitor 앞 띠에 걸립니다. 나머지 구간은 길이 길어져 1 ~ 2 s 씩 느립니다.
- 다시 만들기: `python3 widen_plan.py demo_plan_max.json demo_plan_wide.json --keep 2,9` (구간마다 몇 초 ~ 2 분, `--legs 3-5` = 그 구간만).
- 계획을 새로 만들 때 꼭 지킬 것: ① 안전 띠 (전진 앞 · 후진 뒤) 에 `map_amcl` 벽 · 기둥이 안 들어오게 ② 이어진 후진 ≤ 0.30 m (필터 0.40 − 따라가기 끝 밀기 0.10, 꽉 채우면 `Failed to make progress`) ③ 벽 여유 4 cm.
- 원래 S9 출발 후진은 뒤 띠에 1 cm 걸치는 점이 있습니다 (시뮬에선 안 걸림) — 실물에서 S9 출발 HOLD 가 나면 이것.

## 4. B. 전체 미션 시험 (팀 bringup)

> ⚠ 제어 담당은 11-15 에 실물로 전체 미션을 돌려 보지 못했습니다 (노트북 Gazebo 로만 확인, 5 장). 처음에는 한 단계씩 지켜보며 합니다.

### 단계 10 — J1: bringup 전체

A 의 터미널이 모두 꺼져 있는지 확인하고, 로봇을 출발 자리에 놓은 뒤:
```bash
ros2 launch bringup bringup.launch.py rc100:=true \
  lidar_front_port:=/dev/serial/by-path/platform-3610000.usb-usb-0:2.1.2:1.0-port0 \
  lidar_rear_port:=/dev/serial/by-path/platform-3610000.usb-usb-0:2.1.1:1.0-port0 \
  max_wz:=0.5 rotate_wz:=0.5 2>&1 | tee ~/mission_bringup.log
```
- 인자 없이 `ros2 launch bringup bringup.launch.py` 가 경기 명령입니다 (하드웨어 + 카메라 · 라이다 · TFmini + 인식 + 위치 추정 (AMCL · loc_monitor) + Nav2 + 계획기 + 미션).
- `rc100:=true`: 리모컨 사용 (bringup 이 넘겨받아 base_bringup 으로 감).
- `lidar_front_port` · `lidar_rear_port`: udev 이름이 없을 때만 라이다 USB 를 직접 줌 (`sensors.launch.py` 로 감). `/dev/lidar_*` 가 있으면 두 줄을 빼도 됩니다.
- `max_wz:=0.5 rotate_wz:=0.5`: 팀 Nav2 의 회전 속도 상한을 규칙 v2 (제자리 ≤ 0.5 rad/s) 로 낮춤. ⚠ 팀 Nav2 는 **제자리 회전 각도 (≤ 90°)** 는 막지 않습니다 (RotationShim 이 경로가 45° 넘게 틀어지면 제자리로 돎).
- ⚠ 켜질 때 팔이 대기 → 주행 자세로 펴집니다. Nav2 는 `/joint_states` 가 나온 뒤 10 s 뒤에 뜹니다.

### 단계 11 — 준비 확인

```bash
ros2 topic echo --once /mission/ready
ros2 lifecycle get /map_server; ros2 lifecycle get /amcl; ros2 lifecycle get /bt_navigator; ros2 lifecycle get /controller_server
```
- mission_node 가 J1 화면에 준비 표 (1 안전 · 2 컨트롤러 · 3 노드 서버 · 4 위치 추정 · 5 Nav2) 를 찍고, 모두 OK 면 `/mission/ready` = `READY`.
- `inactive` 인 노드가 있으면 `ros2 lifecycle set /<노드> activate` (응답 지연, 0 장 참고). 그래도 안 되면 J1 을 Ctrl+C 하고 다시.
- 위치 추정 (`/localization/status`) 이 `FAILED` · `LOST` 면 로봇이 출발 자리에 있는지 보고, RViz (단계 8) 로 빨간 점이 벽과 겹치는지 확인.
- 리모컨 2 + 3 으로 E-stop 풀기.

### 단계 12 — 시작 · 감시 · 멈춤 (J2)

```bash
ros2 service call /mission/start std_srvs/srv/Trigger      # 시작 (READY 일 때만 받음)
ros2 topic echo /mission/status                            # 진행 상태 (다른 터미널)
ros2 service call /mission/stop std_srvs/srv/Trigger       # 멈춤 (소프트웨어 정지)
```
- 흐름: 신호등 · 바코드 보기 → 측량 (상자 퍽 보기) → 계획 (`/mission_planner/plan`) → 상자마다 다가가기 · 집기 · 싣기 → L 마다 놓기 → 확인 → 출발 구역 복귀 → DONE.
- 기록: `~/Lv2_Cartlidar_Assignment/mission_log/<YYMMDD_HHMM>/` (mission_log.jsonl · 이동 전후 사진 · 계획기 기록).

| 위험 (아직 실물 미확인) | 내용 · 대처 |
|---|---|
| 팀 Nav2 제자리 회전 | 경로가 45° 넘게 틀어지면 제자리로 돎 (180° 도 가능) → 벨트. 회전이 길면 리모컨 1 |
| 상자 · L 자리 | 계획기는 발제 공식 좌표로 다가감. 실제 상자는 box1 이 18.6 cm 등 5 ~ 19 cm, L2 · L3 는 7 ~ 9 cm 다름 → 인식 (`measure_box`) 이 8 ~ 10 cm 까지 보정, 넘으면 그 퍽 건너뜀 (`on_fail.skip_puck`) |
| 위치 감시 거짓 LOST | loc_monitor 가 실물에서 예민 → 잠금 (`/localization/lock`) 으로 Nav2 가 멈출 수 있음 |
| lifecycle 응답 지연 | bt_navigator 가 안 켜지는 일 (#276) → 단계 11 |

## 5. C. 노트북 시뮬 (로봇 없이)

### 주행 시연 시뮬 (loopback, GPU 안 씀)

```bash
cd <저장소>/workspace/tools/drive_demo      # 또는 Jetson ~/demo_v2
./run_demo.sh sim slow                 # 10 구간, 약 5 분
SIM_MAP=<실측판 map_amcl.yaml> ./run_demo.sh sim slow    # 실측판 지도로
SIM_TURN=0.79,0.70 REV_MAX=0.50 ./run_demo.sh sim slow   # 실물형: 실물처럼 덜 돎 (꺾기 79 % · 제자리 70 %) + 실물 후진 한도
```
- 실물과 같은 스크립트를 가짜 로봇 (명령을 그대로 따라 함) 으로. 미끄럼은 없음.
- `SIM_TURN=꺾기,제자리` (11-16 추가): 회전 명령만 그 비율로 줄여 가짜 로봇에 넣습니다 (`sim_turn_scale.py`). 11-15 실물 (벨트 v7) 이 꺾기 약 79 % · 제자리 66 ~ 80 % 만 돌던 것을 흉내 낸 근사입니다 (미끄럼 · 지연 · 잡음 없음, 실물 검증 아님).
- 같은 시각에 여러 개를 돌릴 때는 `SIM_DOMAIN=81` 처럼 도메인을 다르게, `RUN_TAG=이름` 으로 기록 폴더 이름을 다르게 합니다.

### 팀 전체 미션 (Gazebo + 가짜 팔 · 인식)

```bash
cd ~/GIT/lv2_CartLidar/Lv2_Cartlidar_Assignment/workspace/ros2_ws && source install/setup.bash
src/mission_planner/test/gazebo_mission.sh /tmp/gazebo_mission     # 결과: /tmp/gazebo_mission/mock_out.json
```
- 진짜 = Gazebo 경기장 · Nav2 · AMCL · drive_node · mission_node · 계획기, 가짜 = 팔 · 인식 · 정렬.
- `Nav2 bringup 실패 (active 2/3)` 가 가끔 남 → 다시 실행.
- ⚠ 이 스크립트는 끝날 때 `map_server` · `lifecycle_manager` 같은 이름의 프로세스를 모두 끕니다 → **다른 시뮬 · scan_view 와 같이 돌리지 않습니다.**
- 실측판 미로로 돌리려면 제어 담당 노트북의 `~/Desktop/CartRider/2026-11-15_미로실측/README.md` (실측 SDF 로 덮어쓰기 공간을 만들어 그 터미널에서 실행).

### 11-16 오전 시뮬 — 오늘 계획을 되찾기 판으로 (이상 vs 실물형 회전)

| 시뮬 (느린, 계획 `7cm_실측L_어제순서`) | 결과 | 정지점 어긋남 | 정지점 밖 최소 여유 |
|---|---|---|---|
| 이상 (명령대로 움직임) | **10/10** | 0.8 ~ 4.5 cm | 4.7 cm |
| 실물형 회전 (꺾기 79 % · 제자리 70 %) · 후진 0.50 | 10 구간 모두 끝까지 감 · **정지점 3 곳 5 cm 넘음** (S4 6.9 · S5 5.9 · S7 6.2 cm) | 0.6 ~ 6.9 cm | 2.9 cm |
| 실물형 회전 최악 (75 % · 66 %) · 후진 0.50 | 같은 3 곳 (7.5 · 6.6 · 7.3 cm) | 0.6 ~ 7.5 cm | 2.7 cm |

- 실물형에서도 이탈 멈춤 · 되찾기는 한 번도 걸리지 않았습니다. 정지점 어긋남은 거의 모두 **옆으로** (5.8 ~ 6.9 cm) 입니다 → 정지점에서 앞뒤로 조금 움직여서는 못 고침. 11-15 실물 S2 · S3 의 5 ~ 8 cm 와 비슷한 크기입니다.
- S0 끝 box1 정지점 바로 앞에서, 덜 돈 방향 (약 22°) 으로 들어오며 차체 외곽 (Nav2 6 각형, 팔 끝 포함 · 보수적) 이 북쪽 기둥 (1.835, 1.317) 과 0.3 ~ 1.3 cm 겹칩니다. 11-15 실물 S0 네 번에서는 닿은 기록이 없습니다. 실물에서 box1 앞을 볼 것.

### 11-16 새벽 시뮬 묶음 (팀 전체 미션 12 회 · 주행 시연 9 회)

| 시뮬 | 조건 | 결과 |
|---|---|---|
| 팀 전체 미션 (Gazebo + 가짜 팔 · 인식) | 문제 5 가지 (Q1 · 바코드 내림 · 신호 바꿈 · 퍽 섞음 · 무작위 2) × 기본 · 실물형 (위치 감시 켬 + `max_wz:=0.5 rotate_wz:=0.5`) · 실측 벽 · 상자 어긋남 | **12 회 중 10 회 DONE** (646 ~ 957 s) |
| 실패 1 | 무작위 2 · 실물형 | box1 → box6 서쪽 접근 주행이 3 번 모두 `Controller patience exceeded` → **미션 전체 FAILED** (재시도 2 번 뒤 퍽을 건너뛰지 않음). 같은 조건 다시 · 기본 설정은 DONE → 가끔 나는 실패 |
| 실패 2 | 무작위 2 · 실측 벽 | 계획기가 60 s 안에 계획을 못 냄 (실측 벽 지도는 쓰지 않기로 함) |
| Nav2 켜기 | 15 번 중 5 번 다시 켬 | lifecycle 응답 지연 (실물과 같은 종류) |
| 주행 시연 (계획기 방문 순서 5 가지, 13 ~ 16 구간) | 느린 | 3 개 완주, 1 개 정지점 5.2 cm (계속 감), 1 개는 방향 맞추기 `Collision Ahead` 로 멈춤 → **건너뛰기로 고침** |
| 주행 시연 (오늘 계획 · 고친 판) | 느린 · 방향 맞추기 모두 거부 강제 | **10/10** · 10/10 (8 번 건너뛰고도 완주, 이탈 멈춤 0) |

### 11-15 밤 결과

| 시뮬 | 미로 | 결과 |
|---|---|---|
| 주행 시연 (loopback, 느린) | 원래 지도 | **10/10** · 정지점 어긋남 1.0 ~ 4.5 cm · 정지점 밖 최소 여유 4.7 cm |
| 주행 시연 (loopback, 느린) | **실측판** 지도 | **10/10** · 정지점 어긋남 0.8 ~ 4.5 cm · 실측판 벽 · 상자와 최소 여유 4.3 cm (정지점 밖), 2 cm 안 0 번 |
| 팀 전체 미션 (Gazebo + 가짜 팔 · 인식) | **실측판** 미로 (벽 · 상자 9 개 실측) | **DONE** · 451 s (시뮬 시각) · 주행 16 · 정렬 25 · 회전 기술 8 · 확인 3 |

- 시뮬에는 실물의 미끄럼 · 덜 도는 것 · AMCL 오차가 없습니다. Gazebo 미션의 팔 · 인식은 가짜라 실제 상자 보정 (`measure_box`) 은 확인되지 않았습니다.
- 실측판 = 칠판 숫자 중 서로 맞는 것을 넣은 판 (6 장). x3 → 동쪽 벽 1175 는 넣지 않았습니다.

## 6. 미로 실측 (11-15 칠판) vs 지도

![칠판 실측 vs 지도 (괄호 = 실측 − 지도 mm, 보라 점선 = 실측 L 상자)](미로_실측_비교.png)

- 같은 미로입니다. 25 개 중 12 개는 ±2 cm, 6 개는 ±4 cm 안. 4 cm 넘게 다른 7 개: x1 · x3 세로벽 위쪽 끝 (+13.8 · +9.3 cm), x3 → 동쪽 벽 (+7.7), 오른쪽 가운데 가로벽 사이 (−5.1), 오른쪽 아래 가로벽 끝 (−4.5), L2 (동 8.7), L3 (남 6.6).
- **팀 기준 지도는 발제 지도 그대로 씁니다 (11-15 결정, 실측은 더 하지 않음).** 실물 라이다가 발제 지도와 잘 맞았고, 크게 다른 숫자는 대부분 벽 끝이라 위치 추정에 영향이 작습니다. 칠판 숫자끼리 맞지 않는 것 (x3 → 동쪽 벽 1175) 도 있습니다.
- 시연 계획의 **L1 ~ L3 정지점만** 실측 상자 자리로 고쳤습니다. 상자는 라이다 지도에 없고 놓을 때마다 달라지므로 지도와 따로입니다.
- 실물에서 RViz 로 한 번 볼 곳: 오른쪽 가운데 통로 (box6 근처) · x3 · L3 근처 (S6 에서 닿은 곳). 빨간 점이 지도 벽에서 5 cm 넘게 떨어져 보이면 제어 담당에게.
- 실측판 SDF (참고 · 시뮬용, 저장소에는 안 올림) 는 제어 담당 노트북 `~/Desktop/CartRider/2026-11-15_미로실측/`.

## 7. 리모컨 RC-100

| 버튼 | 하는 일 |
|---|---|
| **1** | E-stop (모두 멈춤) |
| **2 + 3** 동시 | E-stop 풀기 (켤 때 한 번 필요) |
| U / D | 누르는 동안 앞 · 뒤 0.1 m/s |
| L / R | 누르는 동안 제자리 회전 0.5 rad/s (L = 왼쪽). 한 번에 90° 넘게 돌리지 않음 |
| 4 · 5 · 6 | 안 씀 |

## 8. 끝내기

1. 시연: J4 → J3 → J2 → J1 Ctrl+C. 미션: `/mission/stop` → J1 Ctrl+C. J1 을 끄면 팔은 펌웨어가 그 자리를 붙잡음 (HOLD).
2. 전원을 끄기 전에 팔을 대기 자세로 접습니다 (팔 담당 방법). 다음에 켤 때 대기 자세 검사 때문입니다.
3. 배터리를 빼고 충전합니다.
4. 기록은 Jetson `~/demo_v2/runs/` (시연) · `~/Lv2_Cartlidar_Assignment/mission_log/` (미션) 에 남습니다. 제어 담당에게 시각만 알려 주면 됩니다.
