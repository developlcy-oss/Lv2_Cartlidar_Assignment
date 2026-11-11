# 채점표 대응 (경기 뒤 점검)

기준: Integration 브랜치 2026-11-11. 배점은 팀 채점표 (합 100). 항목마다 **어느 기록이 근거인지**, 그 기록을 **어디서 만드는지** 를 적는다.
에피소드 폴더 = bringup `log_dir` (기본 `~/Lv2_Cartlidar_Assignment/mission_log`) 아래 미션마다 `<YYMMDD_HHMM>/`.

## 1. 항목 · 근거

| 항목 | 배점 | 근거 기록 (에피소드 폴더) | 만드는 곳 | 비고 |
|---|---|---|---|---|
| 신호 순서 | 12 | `planner/signal.json` `signal.sequence` (12 개), `mission_log.jsonl` `observe` 의 `sequence` | perception `/perception/signal` CONFIRMED → 계획기 | 같은 순서 2 번 봐야 확정. 4 묶음 안에 못 맞추면 `observe ok:false` 기록 뒤 안전 정지 |
| 횟수 | 9 | `signal.json` `signal.counts` (R · Y · G) | 계획기 `count_signal` | 12 개가 아니거나 불명 색이면 추정하지 않고 오류 |
| 바코드 | 4 | `signal.json` `barcode.code` · `order`, `observe` 의 `barcode` | perception `/perception/barcode` READ | 미등록 코드 → plan 실패 · 안전 정지 |
| 그룹 | 9 | `planner/allocation.json` `groups` (L1 ~ L3 = 그룹 · 횟수) | 계획기 `order_groups` | P2-01 · 03 오름차순, P2-02 · 04 내림차순, 동률 R → Y → G |
| 색·모양 | 18 | `verify` 의 `cells[].color` · `shape` · `error`, 계획 쪽 `allocation.json` `colors` · `shapes` · `y_shape` | `/perception/check_dice` (verify 때 새 관측) | R = 빨강만, G = 초록만, Y = 모양 하나 |
| 주사위 눈 | 30 | `verify` 의 `passed`, `cells[].count` · `offset` | check_dice (`verify.tolerance: 0` = perception 기준 10 mm) | 장소당 눈 전부 맞아야 10 점 |
| 재검증 | 8 | `verify` 의 `ids` (칸별 퍽 ID · 위치, 2 번 이상 본 것만), `verify_ids` (중복 · 누락 · `passed`), `verify` 의 `marker` (장소 태그), `images/NN_after_<L>_k.jpg` | verify 동안 `/perception/pucks` 를 새로 쌓음 (목표 복사 아님) | 마지막 verify 뒤 `verify_ids` 한 줄 |
| 복귀 | 10 | `return` (`pose` · `inside` · `min_margin_m` · `moved_m` · `turned_deg`), `finish` `DONE` | mission_node `wait` 단계 (`return_check`, return_check.hpp) | 차체 네 모서리 (외곽 x −0.2135 ~ +0.1328, y ±0.1676) 가 출발 구역 (0.734, 0.95, 0.6 m 사각) 안인지 |

그 밖의 기록:
- 이동 전 사진 `images/NN_before_r<회차>_B<박스>_k.jpg` (survey 보기 자세), 퍽 등록표 사본 `pucks_survey.yaml`.
- **전체 영상은 로봇이 남기지 않는다.** 경기 때 휴대전화 촬영 또는 `ros2 bag record` 를 운영 절차서에 넣을 것.

## 2. 경기 뒤 점검 순서

1. 예상 점수:
   ```
   ros2 run mission_planner mission_score <에피소드 폴더> --problem <그 경기 문제 yaml (signal · barcode)> [--json score.json]
   ```
   항목마다 점수와 근거 한 줄을 낸다. 부분 점수 규칙은 **팀 가정** (`mission_planner/score.py` 머리말). 발제 원문이 다르면 그 파일을 고친다.
2. `mission_log.jsonl` 의 `finish` 가 FAILED 면 `reason` 과 그 앞 사건으로 멈춘 단계를 찾는다 (`step` = 단계 번호, `planner/mission_lines.txt` 의 줄).
3. 재검증 0 점이면 `verify` 의 `ids` 가 비었는지 (태그를 못 읽음), `verify_ids` 가 없는지 (마지막 verify 전에 멈춤) 를 본다.
4. 복귀 0 점이면 `return` 의 `inside` · `min_margin_m` 를 본다. Nav2 허용 오차 (0.05 m · 6°) 와 AMCL 오차를 더하면 여유가 약 2.5 cm 라 실물에서 자주 볼 것.

## 3. 남은 확인

| 항목 | 할 일 |
|---|---|
| 신호 · 바코드 | 실물 신호등 · 바코드 설치 위치에서 출발 자세로 보이는지 (현장) |
| 색·모양 · 눈 | 실물 팔 카메라로 check_dice 정확도 (도전 A 와 같이) |
| 복귀 | 실물 3 회 이상 `return` 여유 기록. 여유가 1 cm 아래면 HOME 목표에 drive 정밀 정렬을 더할지 정함 |
| 전체 | `planner_stop_after` 로 관측 → 박스 1 곳 → 장소 1 곳 → 전체 순서로 키우며 확인 |
