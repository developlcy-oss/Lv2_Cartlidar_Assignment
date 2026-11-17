# 재현 시험 기록

목적: 만든 사람이 아닌 팀원 (또는 다른 장비) 이 같은 절차로 돌려 같은 결과가 나오는지 확인한다.
실물 / 모의를 반드시 나눠 적는다. 원본 로그 · 영상 경로와 커밋을 적지 못하면 그 줄은 `미확인` 으로 둔다.

## 양식

| 날짜 | 실행자 | 구분 (실물 · 모의) | 장비 | 설정 (명령 · 인자 · 계획 · 문제) | SW 버전 (ROS · 펌웨어 · 정책) | 커밋 | 실행 결과 (성공 · 실패 · 미확인) | 원본 로그 · 영상 | 비고 |
|---|---|---|---|---|---|---|---|---|---|
| | | | | | | | | | |

## 기록

### 1. 주행 시연 drive_demo — 실물 (2026-11-15 저녁, 한상준)

원본 기록: `project2/results/reproduction_20261115/` (Jetson `~/demo_v2/runs/` 에서 가져온 5 회).

| 항목 | 내용 |
|---|---|
| 날짜 | 2026-11-15 (일) 20:50 ~ 21:18 KST (`settings.txt` 의 `시각` · `nav.log` ROS 시각) |
| 실행자 | 한상준 (WindForce08, 주행 담당). **drive_demo 를 만든 사람** (PR #306 작성자) 이 직접 돌린 것이라, "만든 사람이 아닌 팀원의 독립 재현" 은 아니다. 이 기록이 재현하는 것은 **실물 로봇에서의 주행 부분** (계획 경로를 Nav2 로 따라가기) 뿐이고, 전체 시스템 (신호 · 바코드 관측 · 계획기 · 팔 집기 · 놓기 · 재검증 · 복귀) 은 아니다 |
| 구분 | 실물 |
| 장비 | 실물 로봇 (Jetson Orin Nano Super, 계정 `cart11` · 4 륜 스키드 XM430-W210 ×4 · 주행 / 팔 OpenCR 2 장 · LiDAR LDS-03 ×2). 벨트 v7 44T → 22T = **1 : 2.0** (`settings.txt` `실물 벨트: 2.0`). 위치 추정 AMCL · EKF. 팔 · 카메라는 쓰지 않음 |
| 설정 | `run_demo.sh real` (`settings.txt`): 계획 `길모음7cm_어제순서` (HOME → box1 → box2 → box6 → box3 → box5 → box4 → L3 → L2 → L1 → HOME, 각 폴더의 `길모음7cm_어제순서.yaml`) · RPP 전진 / 후진 0.20 / 0.10 m/s (slow) 또는 0.30 / 0.15 (full, 20:55) · Spin · 규칙 v2 필터 (제자리 ≤ 0.4 · 벨트 1.5 · 꺾는 중 모터 ≤ 3.0, full 은 제자리 ≤ 0.5, 21:13 부터 이어진 후진 ≤ 0.50 m 추가) · Nav2 `xy_goal_tolerance` 0.03 · 구간마다 Enter (`params.yml`). 출발 자세 (0.694, 0.950, 180°) · 위치 감시 꺼짐 · 운영자가 RViz 로 위치 확인 뒤 `y` |
| SW 버전 | ROS 2 lyrical (`nav.log` 의 `/opt/ros/lyrical`) · Nav2 RPP. Jetson 저장소는 11-15 20:24 에 dev **`1b9266c`** (PR #304 머지, 라이다 yaw 보정) 로 옮김 (Jetson `git reflog`). 시험은 저장소의 Nav2 노드 · URDF 만 쓰고 (`settings.txt` `저장소: Nav2 노드만 (시연 폴더 demo_v2)`), 시연 스크립트 (`run_demo.sh` · `demo_follow.py` · `v2_filter.py`) 는 노트북에서 복사한 `~/demo_v2` 판이다. 이 스크립트가 저장소에 올라간 것은 그 뒤 PR #306 (머지 `e34af1c`, 11-15 22:52 KST) 이므로 **시험 때 스크립트 판은 커밋으로 특정할 수 없음** (미확인). 주행 OpenCR 펌웨어 판은 기록 없음 (미확인) |
| 커밋 | 저장소 `1b9266c` (Jetson reflog) + 저장소 밖 `~/demo_v2` (커밋 없음) |
| 원본 로그 · 영상 | `reproduction_20261115/<회차>/` (`settings.txt` · `params.yml` · `follow.log` · `nav.log` · 계획 yaml, 끝난 회차는 `summary.txt` · `course_result.json` · `moves.json` · `track.csv`). 영상 없음 |

회차별 결과 (구간 합격 = 정지점 어긋남 5 cm 이하)

| 회차 (폴더) | 모드 · 구간 | 결과 | 내용 |
|---|---|---|---|
| `1115_2050_실물_slow_구간0-0` | slow · S0 만 | **0/1** | 출발 자세와 11.6 cm 어긋남 → 운영자가 위치 확인 안 함 → 출발 안 함 (`follow.log`) |
| `1115_2052_실물_slow_구간0-0` | slow · S0 만 | **1/1** | S0 signal → box1 35.0 s · 3.8 cm · 1.2° 성공 |
| `1115_2055_실물_full` | full (0.30 / 0.15) · S0 ~ S9 | **1/10** (S1 에서 끝) | S0 성공 (3.7 cm). S1 조각 3 시작 방향 맞추기 −15° 에서 Spin 시간 초과 (오류 701 `Exceeded time allowance before reaching the Spin goal`) → 회차 끝 |
| `1115_2104_실물_slow` | slow · S0 ~ S9 | **2/10** (S4 에서 끝) | S0 4.3 · S1 4.2 cm 성공. S2 7.7 · S3 5.3 cm 실패. S4 조각 1 (후진) 에서 Nav2 `Failed to make progress` (오류 105) → 정지점 133.1 cm · 146.6° 로 끝 |
| `1115_2113_실물_slow` | slow · 이어진 후진 ≤ 0.50 m 필터 추가 · S0 ~ S9 | **4/6 까지 기록, 결과 파일 없음** | S0 4.3 · S1 4.3 · S4 1.5 · S5 4.2 cm 성공, S2 8.3 · S3 5.2 cm 실패 (`follow.log`). S6 조각 2/3 (전진) 중 21:18:23 목표 취소 → 21:18:29 Ctrl+C 로 끝나 `summary.txt` · `course_result.json` · `track.csv` 가 남지 않음 (`nav.log` 175 ~ 177 줄). PR #306 본문의 "S0 · S1 · S4 · S5 ✓ · S2 · S3 5 ~ 8 cm · S6 벽 닿음" 과 구간 결과가 같아 이 회차로 보이나, 벽 접촉 자체는 로그로 확인 안 됨 (영상 없음) |

판정: **실패** — 5 회 중 어느 회차도 10 구간을 끝내지 못했다 (최고 21:13 의 4 구간 성공, S6 에서 중단). S0 (출발 → box1) 은 출발한 4 회 모두 성공 (3.7 ~ 4.3 cm) 으로 재현됨. S2 · S3 정지점 5 ~ 8 cm 실패는 21:04 · 21:13 에서 같은 모양으로 반복.

11-16 경기 당일 같은 도구 (max 모드 · 벨트 2.0 · 계획 `7cm_실측L_어제순서_끝90도`) 결과와 비교: 공식 2 회차 (12:25) 4/10 · S9 까지 주행 (`official_scoring.md` 3 장). `test-results.csv` R-27 · M-05 · M-08.

한계

- 실행자가 도구를 만든 사람이다 → 다른 팀원이 README 만 보고 같은 결과를 내는지는 **확인 안 됨**.
- 주행만 재현했다. 팔 · 인식 · 계획기 · 미션 노드는 돌리지 않았다.
- 시연 스크립트 판 (`~/demo_v2`) 이 커밋으로 남지 않았고, 주행 OpenCR 펌웨어 판도 기록 없음.
- 위치 · 정지점 어긋남은 AMCL 추정값이다 (줄자 실측 아님).
- 21:13 회차는 Ctrl+C 로 끝나 결과 파일이 없고 `follow.log` 로만 판정했다. 벽 접촉은 PR 본문 진술뿐 (로그 · 영상 근거 없음).
- 정지점 합격선 5 cm 는 drive_demo 기준이며 발제 채점 기준이 아니다.

### 2. bringup 컨트롤러 활성화 — 실물 (2026-11-15)

bringup 은 kanichong 이 만들었고, 아래는 만든 사람이 아닌 팀원 (Onbeom) 이 같은 절차로 돌린 기록이다.

| 날짜 | 실행자 | 구분 | 장비 | 설정 | SW 버전 | 커밋 | 실행 결과 | 원본 로그 · 영상 | 비고 |
|---|---|---|---|---|---|---|---|---|---|
| 2026-11-15 16:03 ~ 19:40 | Onbeom (인지 · 위치 추정 담당) | 실물 | Jetson cart11 · 실물 로봇 (정지 상태) | `use_teleop:=true rc100:=true use_perception:=false use_localization:=true use_amcl_node:=true use_nav2:=false use_planner:=false use_loc_lock:=false use_loc_gate:=false` (11-14 실패 때와 같은 인자) | ROS 2 Lyrical | 설치본 c453996 (dev 1767e0f + STOW J1 −1.287) | **성공**: 3 판 모두 컨트롤러 4/4 · spawner 실패 0 · `/joy` 20 Hz · `/tf` 71 ~ 78 Hz (19:36 설치본 fd6bcbf 한 판 더, 같은 결과) | [#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6893200035) | 인지 · Nav2 끈 구성. 처음 두 번은 남은 노드 때문에 판정에서 뺌 |
| 2026-11-15 22:48 ~ 23:45 | Onbeom | 실물 | Jetson cart11 · 실물 로봇 (정지 상태) | `use_teleop:=false use_localization:=true use_nav2:=true use_planner:=false` + 인지 모드 7, 판마다 bringup 을 새로 띄움 | ROS 2 Lyrical | 설치본 1b9266c | **성공**: 8 판 모두 첫 bringup 에서 컨트롤러 4/4 (11-14 오전 같은 구성 30 번 중 25 번 실패) | [#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6896389227) | 원본 로그는 Jetson 에만 있음 |

### 3. 그 밖

| 항목 | 상태 |
|---|---|
| 전체 미션 실물 재현 (만든 사람 외 팀원) | **작성 필요** — 기록 없음. 11-16 경기 로그 (`real_20261116/`) 는 실행자가 로그에 남지 않아 재현 기록으로 쓸 수 없음 |
| 팔 주사위 배치 (`dice.sh`) 실물 재현 | **작성 필요** — 기록은 팔 담당 본인 실행뿐 (R-21 · R-22 · R-25) |
| 계획기 제출물 다시 만들기 (`mission_submit --problem config/problem_q1_example.yaml`) | **작성 필요** — 절차는 `project2/problem/README.md`, 다른 팀원이 돌린 기록 없음 |
| Isaac 평가 (`pallet_place_check.py --place` 등) | **작성 필요** — kanichong PR #313 · #320 결과는 만든 사람 본인 실행 |
