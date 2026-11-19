# ① 문제 패키지

| 파일 | 내용 |
|---|---|
| `problem.md` | 신호 · 바코드 · 장소 그룹, 퍽 12 개 시작 좌표 |
| `signals.json` | R/Y/G 12 회 순서 · 색별 횟수 · 바코드 · L1 ~ L3 그룹 |
| `placement.csv` | 퍽 ID · 색 · 모양 · 시작 박스 · 좌표 (mm, 발제 좌표) |
| `allocation_answers.md` · `.json` | 유효한 전체 퍽 할당 정답 전부 (L1 ~ L3 그룹 · 목적지별 퍽 ID) |
| `verification.md` | 발제 04 쪽 운영자 검증 ① ~ ④ 결과 |
| `dice_targets.csv` | 장소별 주사위 눈 좌표 (발제 03 쪽: 장소 중심 기준 X · Y −60 · 0 · +60 mm, 바닥 Z 70) 와 놓을 퍽 ID |
| `rules.md` | 경기 규칙 · 해석 · 100 점 채점표 · 초기화 · 운영 |
| `review.md` | 다른 팀 검토 기록 (작성 필요) |

## 이 문제는 무엇인가

발제 (mission.pdf 05 쪽 「팀별 제출 문제」) 는 팀마다 **실제 퍽 12 개의 속성 · 시작 좌표, 12 회 신호 순서, 바코드, 유효 할당 정답과 검증 결과** 를 내라고 한다.

- 퍽 12 개: 팀이 실물 경기장에 놓은 등록표 `workspace/docs/stage_test/pucks_stage.yaml` (2026-11-15, 실물 단계 시험) —
  빨강 원기둥 3 · 빨강 직육면체 3 · 초록 원기둥 3 · 초록 직육면체 3, 박스 1 ~ 6 에 2 개씩 (중심 X ± 40 mm). **실물을 이 표에 맞춰 놓았다.**
- 신호 · 바코드: 발제 05 쪽 공개 연습 문제 Q1 `RGRRGRGGYGYG` · `P2-01` (R 4 · Y 2 · G 6, 오름차순 → L1 Y · L2 R · L3 G).
  실전에서 운영자가 다른 시퀀스를 내면 그 값으로 다시 만든다 (아래 명령).
- 운영자 검증 ① ~ ④ 통과, 유효 전체 할당 6 개 (Y 를 원기둥 또는 직육면체 중 하나로 고르는 2 가지 × ID 조합).
- 주사위 눈 좌표 (`dice_targets.csv`) 는 계획기 (`mission_plan`, hybrid) 가 고른 할당 (`../results/sim_plan_q1/allocation.json`) 기준.
  눈 모양은 `mission_planner/poses.py` `DICE` (2 = ↖↘ 대각선, 4 = 네 모서리, 6 = 좌우 두 열, 지도 +Y 가 위).

## 만든 방법 (재현)

```bash
cd workspace/ros2_ws/src/mission_planner
python3 -m mission_planner.submission --problem ../../../docs/stage_test/pucks_stage.yaml --out ../../../../project2/problem
python3 -m mission_planner.cli --problem ../../../docs/stage_test/pucks_stage.yaml --out ../../../../project2/results/sim_plan_q1   # 계획 · 할당
```

설치된 환경에서는 `ros2 run mission_planner mission_submit …` · `mission_plan …` 과 같다. 규칙 위반이면 `verification.md` 에 실패 원인을 쓰고 종료 코드 2 로 끝난다.
신호 · 바코드만 바꿀 때: `python3 -m mission_planner.submission --placement placement.csv --signal <12 글자> --barcode <P2-0x> --out …`
`problem.md` 를 다시 만들면 끝의 `dice_targets.csv` · `rules.md` · `review.md` 줄이 지워지므로 다시 붙인다.
