# ④ 시험 · 결과 (project2/results)

기준: 2026-11-17, dev `0bbc537` (PR #320 머지). 이 폴더는 **실물 시험과 모의 (Isaac · Gazebo · mock · loopback) 시험을 나눠** 적는다.
모의 결과는 실물 성공으로 세지 않는다. 하지 않은 시험은 `미수행`, 결과를 판단할 근거가 없으면 `미확인` 으로 적는다.

## 1. 파일

| 파일 · 폴더 | 내용 | 실물 / 모의 |
|---|---|---|
| `test-results.csv` | 시험 한 줄 = 한 행 (UTF-8, 머리글 있음). 열: `test_id, date, category (실물 · 모의 · 단위), target, equipment, input_condition, expected, actual, verdict (성공 · 실패 · 미확인 · 미수행), attempts, error, evidence, code_version` | 둘 다 (`category` 로 나눔) |
| `official_scoring.md` | 회차 대응 (팀 결정: 시간 순서) 과 공식 1 회차 (12:15) · 2 회차 (12:25) 100 점 판정표 (항목 · 배점 · 원점수 · 근거 · 비고, 완주 여부 · 반영 점수 · 실패 원인) | 실물 |
| `signal.json` | 경기에서 로봇이 **실제로 본** 신호 · 바코드. 실물 관측이 없어 `status: 미관측` 과 이유 · 로그 경로만 적음 | 실물 (값 없음) |
| `allocation.json` | 실물 입력으로 만든 그룹 · 할당. 실물은 `status: 미계산 (실물)`. 참고로 모의 결과 (`sim_reference_content`) 와 그 중복 · 누락 검증 (`sim_reference_check`) 을 같이 둠 | 실물 (값 없음) + 모의 참고 |
| `reproduction.md` | 재현 시험 기록 (날짜 · 실행자 · 장비 · 설정 · SW 버전 · 커밋 · 실행 결과 · 한계). 11-15 저녁 한상준 (WindForce08) 실물 drive_demo 5 회 | 실물 · 모의 |
| `reproduction_20261115/` | 재현 시험 원본 기록: 2026-11-15 20:50 ~ 21:17 실물 drive_demo 5 회 (`1115_2050` · `2052` · `2055` · `2104` · `2113`, 각 `settings.txt` · `params.yml` · `follow.log` · `nav.log` · 계획 yaml, 끝까지 간 회차는 `summary.txt` · `course_result.json` · `moves.json` · `track.csv`) | 실물 |
| `sim_plan_q1/` | 계획기 `mission_plan` 에 `config/problem_q1_example.yaml` (발제 05 쪽 공개 연습 문제 Q1) 을 넣은 **모의** 출력 (`signal.json` · `allocation.json` · `plan.json`) | 모의 |
| `real_20261116/` | 2026-11-16 경기 당일 Jetson (`cart11`) 에서 가져온 원본 로그 | 실물 |
| `real_20261116/launch_logs/` | 데모 브링업 로그 `demo_261116_HHMMSS.log` (11:51 ~ 13:49), 팔 없는 주행용 `demo_base.log` · 위치 추정 `demo_loc.log` (12:22 ~ 12:30). `123702` 는 크기 때문에 gzip | 실물 |
| `real_20261116/drive_runs/` | drive_demo 주행 시연 3 회 (`summary.txt` · `course_result.json` · `follow.log` · `nav.log` · `track.csv` · `params.yml` · `settings.txt`) | 실물 |
| `real_20261116/mission_log/` | 전체 미션 에피소드 2 개 (`261116_1236` · `261116_1236_2`) 의 `mission_log.jsonl` | 실물 |

### test-results.csv 읽는 법

- `test_id` 머리 글자: `U` 단위 시험 · `S` 모의 · `R` 실물 (경기 전) · `M` 실물 (2026-11-16 경기 당일 로그).
- 출처
  - `submission/midterm/test_results.md` (중간 제출, 11-09) 의 모든 행을 옮김 (U-01 ~ U-06, S-01 ~ S-14, R-01 ~ R-13).
  - 팔 · 미션 담당 (이창엽, `developlcy-oss`) 개인 일지 `report/daily/*.md` (저장소 밖) 와 그 PR 본문 (R-14 ~ R-30, S-15 ~ S-29, U-07 ~ U-09).
  - 팀원 PR 본문: kanichong #313 · #320, WindForce08 #306 · #318 · #319 (S-30 ~ S-34, R-27).
  - 경기 당일 로그 `real_20261116/` (M-01 ~ M-18).
- `expected` 에 합격선이 원 기록에 없으면 `합격선 없음` 으로 두고 `verdict` 는 `미확인` (수치만 기록). 시뮬 합격선은 그 작업 때 적어 둔 목표 (11-11 · 11-15 일지의 98 %, 메모의 95 %) 를 썼다.
- `code_version`: PR 이 있으면 `작업 커밋 (PR #N 머지 커밋)`. 경기 당일 Jetson 저장소는 `fd03441` (dev, PR #319 머지 — Jetson `git reflog` 10:06 · 10:14 KST pull, #316 코드 포함) 이다. 팔 OpenCR 에 펌웨어 v6 (#316) 를 올렸는지는 미확인. drive_demo 는 `~/demo_v2` 복사본으로 돌려 그 판은 미확인.
- `evidence` 의 PR · Issue 는 `https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment` 아래 링크. 로그 경로는 저장소 뿌리 기준.
- **영상**: 저장소 · 이 폴더에 경기 영상은 없다. 로봇은 전체 영상을 남기지 않는다 (`submission/scoring_map.md` 1 장). 휴대전화 촬영본이 있으면 팀이 경로를 `evidence` 에 덧붙여야 한다.

## 2. 2026-11-16 실물 실행 (시간 순)

시각은 로그 안 ROS 시각 (KST) 기준. 실행자는 로그에 남지 않는다 (Jetson 계정 `cart11`).

회차 열: **회차 대응은 팀장 결정 (시간 순서) — 영상 · 심판 기록과 다르면 고칠 것.** 로봇이 실제로 출발한 시도를 시간 순서로 1 · 2 회차로 셌다 (`official_scoring.md` 0 장).

| 시각 | 띄운 것 | 결과 | 회차 | CSV | 로그 |
|---|---|---|---|---|---|
| 11:51:37 ~ 11:54:25 | 데모 브링업 전체 (22 프로세스) + `demo_start` | 준비 13/14 — E-stop active (조이스틱 리셋 안 됨). 시작 요청 16 번, 미션 시작 안 됨 | 출발 전 실패 (회차 아님) | M-01 | `launch_logs/demo_261116_115135.log` |
| 11:54:34 ~ 11:55:46 | 데모 브링업 | OpenCR `CMD 송신 실패` · 컨트롤러 없음 · 준비 4/14 | 출발 전 실패 (회차 아님) | M-02 | `launch_logs/demo_261116_115431.log` |
| 11:56:17 ~ 11:59:35 | 데모 브링업 | `/dev/ttyACM0` 없음 → OpenCRSystem configure 실패 · 준비 7/14 | 출발 전 실패 (회차 아님) | M-03 | `launch_logs/demo_261116_115615.log` |
| 12:04:32 | drive_demo 주행 시연 (실물 max · 벨트 2.0 · 모터 3.0) | **0/10** — 위치 확인 안 됨, 출발 안 함 | **출발 전 위치 확인 실패 — 시작 안 함 (회차 아님)** | M-04 | `drive_runs/1116_1204_실물_max_벨트2.0_모터3.0/` |
| 12:15:22 | drive_demo 주행 시연 | **0/10** — S0 Nav2 진행 실패 → 되찾기 1 번 → Ctrl+C (정지점 112.1 cm) | **공식 1 회차** | M-05 | `drive_runs/1116_1215_실물_max_벨트2.0_모터3.0/` |
| 12:17:52 | 데모 브링업 + `demo_start` | 시작 요청 직후 로그 끝 (23 줄) — 결과 판단 불가 | 회차 아님 | M-06 | `launch_logs/demo_261116_121749.log` |
| 12:20:45 · 12:21:38 · 12:22:02 | 데모 브링업 3 번 | `ttyACM0` 없음 (2 번) · `ttyACM2` busy → OpenCR 실패 | 회차 아님 | M-07 | `launch_logs/demo_261116_122042.log` · `122136` · `122200` |
| 12:22:04 ~ 12:30:30 | 팔 없는 base 브링업 + 위치 추정 (AMCL · EKF) | 12:25 주행 시연용 | (2 회차용 base · 위치 추정) | M-08 | `launch_logs/demo_base.log` · `demo_loc.log` |
| 12:25:21 | drive_demo 주행 시연 | **4/10** — S0 · S3 · S6 · S7 성공, 나머지 정지점 6.0 ~ 14.4 cm | **공식 2 회차** | M-08 | `drive_runs/1116_1225_실물_max_벨트2.0_모터3.0/` |
| 12:31:43 | 데모 브링업 | `ttyACM2` 없음 → OpenCR 실패 · 준비 4/14 | 공식 회차 뒤 (비공식) | M-09 | `launch_logs/demo_261116_123140.log` |
| 12:33:22 | 데모 브링업 | 팔이 STOW · DRIVE 어느 쪽도 아님 (0.23 rad) → 팔 안 움직임 · 위치 RECOVERING · 준비 13/14 | 공식 회차 뒤 (비공식) | M-10 | `launch_logs/demo_261116_123320.log` |
| 12:35:48 → **12:36:33 · 12:36:41** | 데모 브링업 → **전체 미션 2 번 시작** | 준비 **14/14** → 두 번 모두 observe 에서 `/manipulator/execute_skill goal rejected` (팔 켜질 때 정리 미완) 3 번 → **FAILED** | **공식 회차 뒤 추가 시도 (비공식)** | M-11 · M-12 | `mission_log/261116_1236/` · `261116_1236_2/` · `launch_logs/demo_261116_123545.log` |
| 12:37:04 ~ 12:39:47 | 데모 브링업 | 위치 FAILED · Nav2 lifecycle 미완 → 시작 거절 | 공식 회차 뒤 (비공식) | M-13 | `launch_logs/demo_261116_123702.log.gz` |
| 12:39:57 · 12:40:23 | 데모 브링업 2 번 | 위치 불안정 · `ttyACM0` busy | 공식 회차 뒤 (비공식) | M-14 | `launch_logs/demo_261116_123955.log` · `124020` |
| 12:40:55 ~ 12:41:36 | 데모 브링업 | 진단 ERROR **팔 배터리 6.78 V 0 % 전압 위험** (주행 12.02 V) → `SyncRead 응답 부족` · 준비 11/14 | 공식 회차 뒤 (비공식) | M-15 | `launch_logs/demo_261116_124053.log` |
| 13:41 (3 번) | 데모 브링업 | `Package 'bringup' not found` (작업 공간 setup 안 불러옴) | 공식 회차 뒤 (비공식) | M-16 | `launch_logs/demo_261116_134132.log` · `134142` · `134157` |
| 13:45:56 · 13:47:01 · 13:49:12 ~ 13:51:34 | 데모 브링업 + `demo_start` | 13:47 팔 `arm at DRIVE` 도달. 그러나 위치 RECOVERING · FAILED, 계획기 서버 없음 → 시작 거절. 13:49 시작 요청 20 번 뒤 끝 | 공식 회차 뒤 (비공식) | M-17 | `launch_logs/demo_261116_134554.log` · `134659` · `134910` |

요약: 공식 1 회차 (12:15) · 2 회차 (12:25) 는 둘 다 drive_demo 주행 시연 (Nav2 경로 따라가기만, 신호 · 바코드 인식과 팔 없음) 이라 과제 항목 근거 기록이 없다 → 두 회차 원점수 0, 반영 점수 0 / 100, 미완주 (`official_scoring.md`). 주행은 1 회차 0/10 (S0 에서 진행 실패 → Ctrl+C), 2 회차 4/10 구간. 로그에 남은 전체 미션은 공식 회차 뒤 **12:36 의 2 번뿐**이고 둘 다 observe 단계에서 2 초 안에 FAILED. 신호 · 바코드 관측, 할당, 놓기, verify, 복귀 기록은 없다 (`signal.json` · `allocation.json` 참고).

12:04 · 12:15 주행 시연 때의 base 브링업 로그는 가져오지 못했다 (`demo_base.log` 는 12:22 ~ 12:30 만).

> **회차 대응 = 팀 결정 (팀장, 시간 순서)**: 로그에는 "공식 회차" 표시가 없고 영상 · 심판 기록도 저장소에 없어, 경기 당일 **로봇이 실제로 출발한 시도의 시간 순서**로 정했다.
> 12:04 는 위치 확인이 안 돼 출발하지 않았으므로 회차가 아니다 → 처음 출발한 12:15 = 공식 1 회차, 12:25 = 공식 2 회차. 12:36 전체 미션 2 번과 그 뒤 브링업은 공식 회차 뒤 추가 시도 (비공식).
> **회차 대응은 팀장 결정 (시간 순서) — 영상 · 심판 기록과 다르면 고칠 것.**

## 3. 경기 전 실물 · 모의 요약 (자세한 것은 CSV)

| 영역 | 모의 | 실물 |
|---|---|---|
| 신호 | 렌더 재생 320/320 · 12 회 확정 100/100 (S-01 ~ S-03) | 실물 신호등 판정 **미수행** (R-01), 경기에서도 관측 못 함 (M-12) |
| 바코드 | R3 92/92 · 144/144 (S-04 · S-05) | 손에 든 Arducam 499 장 100 % (R-03). 로봇 장착 상태 **미수행** (R-05) |
| 집기 · 넣기 · 꺼내기 · 놓기 | 꺼내기 → 놓기 98.2 % (v3) · 98.3 % (v5), 집기 5 cm 원기둥 100 %. 노드 조건 그대로는 76.2 % (S-29) | 주사위 pick · place 1 회 성공 (R-21), 연속 배치 실패 (R-22 · R-25), RL 집기 실패 (R-17) |
| 팔 동작 | — | 덜컹 1.4° → 0.07° (R-19), 자동 STOW 성공 (R-14), 관절 1 180° 문제 (R-12 · R-29) |
| 주행 | loopback 10/10, Gazebo 전체 미션 DONE (S-32 · S-33) · 새벽 묶음 10/12 (S-34) | 11-15 S6 벽 닿음 (R-27), 11-16 최고 4/10 (M-08) |
| 전체 미션 | mock DONE · Gazebo DONE (S-12 · S-14) | 11-16 observe 에서 FAILED (M-11) |

## 4. 남은 확인 (팀)

1. 회차 대응 (팀 결정: 12:15 = 1 회차 · 12:25 = 2 회차) 이 영상 · 심판 기록과 맞는지 확인 → 다르면 `official_scoring.md` · 이 표 · CSV M-04 ~ M-11 고침. 2 회차 복귀를 심판이 어떻게 판정했는지도 확인 (`official_scoring.md` 3 장).
2. 경기 영상 경로를 `test-results.csv` 의 해당 행 `evidence` 에 추가.
3. 경기 때 Jetson 저장소 커밋은 `fd03441` 로 확인됨 (reflog). `~/demo_v2` (drive_demo 복사본) 이 어느 판인지는 미확인.
4. 팔 펌웨어 v6 (#316) 이 경기 전에 올라갔는지 (R-30).
