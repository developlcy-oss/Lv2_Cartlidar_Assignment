# mission_rl — 픽 앤 플레이스를 포함한 전체 미션 시뮬 (Isaac, 2026-11-17)

경기 (11-16) 뒤, 주행만이 아니라 **집기 → 팔레트 → 장소 (L) 놓기까지 미션 전체를 한 시뮬에서 이어서** 돌려 단계별 성공률을 재는 준비 작업.
목표 순서: P0 기존 팔 기술 재현 → **P1 미션 시뮬 기준선** (지금) → P2 약한 단계를 미션 시뮬 안에서 학습 (예: 도킹 오차에 강한 놓기).
팀 파일 (`urdf_and_usd/policy/manipulator/`) 은 불러 쓰기만 하고 고치지 않는다. Isaac Lab 3.0 / Isaac Sim 6.1, 창 없이.

## P0 — 팀 팔 확인 스크립트 재현 (`p0_check.sh`)

```bash
./p0_check.sh all     # chain · place · unload 차례로 (Isaac 은 한 번에 하나). 결과 results/p0_*.json (로그 *.log 는 안 올림)
```

팀 `run_rl.sh` 는 팀 폴더의 결과 JSON 을 덮어쓰고 학습 기록 문서에 줄을 더해서, 파이썬을 직접 부르고 `--out` 으로 이 폴더에 씀.

| 시험 | 인자 | 결과 (11-17, 이 노트북 RTX 5060) |
|---|---|---|
| 집기 → 팔레트 칸 (`chain_pallet_eval.py`) | 192 대, 고정 동작 집기 σ 2 mm, 벌림 0.0042, 시드 2 | **173/192 (90.1 %)** = 집기 182/192 (94.8 %) × 넣기 95.1 %. 엄격 판정 (옆 퍽 · 칸 퍽 안 건드림) 169/192 (88.0 %). 실패: 집기 10 · 운반 중 미끄럼 8 · 칸 밖 1 |
| 팔레트 놓기 (`pallet_place_check.py`) | 10 칸 × 원기둥 · 직육면체 | 24/24 앉음 |
| 팔레트 빼기 (`--unload`) | 같은 칸, 3 열 속도 규칙 없이 | 24/24 쥠 |

## P1 — 미션 시뮬 (`mission_sim.py`)

```bash
# 미션 계획 (팀 계획기, Q1 예시 문제)
ros2 run mission_planner mission_plan --problem <mission_planner>/config/problem_q1_example.yaml --mode hybrid --out results/plan_q1
# 시뮬 (ROS 환경 변수는 빼고 Isaac 파이썬으로)
~/pa-course/IsaacLab-6.1/.venv/bin/python mission_sim.py --plan results/plan_q1/plan.json --num_envs 16 [--steps 0-13] [--gui]
```

- 바탕 = 팀 TopPick 환경 (`Waffle-TopPick-Direct`: 정밀 로봇 · kinematic 상자 · IK · 접촉). 계획의 미션 퍽 12 개 + 장소 상자 (L) 를 더 만듦.
- 로봇은 환경 원점에 두고, 단계마다 상자 · 퍽 · L 을 **도킹한 로봇에서 본 자리**로 옮김 (= 주행해 온 것과 같은 상대 배치). 주행 자체는 순간이동이고 도킹 오차만 뽑음 (σ 앞뒤 3 mm · 옆 8 mm · 방향 1°).
- 집기: 계획의 ±17° 회전 (회전 중심 center_x, σ 1°) → 퍽 다시 재기 (σ 2 mm) → ROS `toppick_tilt` 와 같은 고정 동작 (접근 12 cm · J1 쪽 3 cm · 반경 +2 mm · 들기 8.5 cm) → 앞 운반 자세 → 칸 궤적 (`pallet_timeline`, 처짐 보정 ki 4).
- 놓기: 칸 빼기 (놓기 궤적 거꾸로 = `pallet_place_check --unload`) → ROS `placeAt` 처럼 계획 셀 자리 (로봇이 믿는 도킹 자세 기준) 에 45° · 앞뒤 4 cm · 벌림 36.4 mm 로 놓음.
- 퍽은 단계 사이에 그대로 남음: 팔레트 · L 에 놓은 퍽, 같은 상자에 남은 퍽 (다시 들르면 지난번 자리), 잃은 퍽.
- 판정: 칸 = 칸 축 3 mm · 바닥 ±3 mm · 기울기 5° (팀 기준과 같음). L = 셀 목표에서 퍽 바닥 가운데 15 mm (30 mm 도 기록) · 서 있음 · 상자 위. 결과 `results/<이름>/summary.json` · `steps.csv`.

### 지금까지 결과 (4 대, 앞 14 단계 — 집기 5 번 · L2 놓기 3 번)

| 단계 | 결과 |
|---|---|
| 집기 B1:S × 2 · B2:S · B6:W (앞뒤 퍽) · B4:S | 다섯 번 모두 4/4 가 팔레트 칸 3 mm 안 (칸 오차 중앙 0.1 ~ 0.4 mm), 옆 퍽 쓰러짐 0 |
| L2 놓기 × 3 | 매번 셀 15 mm 안 3/4 · 30 mm 안 4/4 (셀 오차 중앙 5.3 ~ 6.1 mm), 먼저 놓은 퍽 밀려남 0 |

`results/시험_집기` · `results/시험_L2`. 한 단계에 실제 시간 약 35 s (4 대).

## 한계 · 할 일 ★

- 전체 미션 (24 단계) · 16 대 기준선은 아직 안 돌림.
- 놓기 자리는 로봇이 믿는 도킹 자세 기준이라 도킹 오차가 셀 오차에 그대로 들어감. 셀 오차를 도킹 · 팔 원인으로 나눠 보지는 않음.
- 주행 · 인식은 모델이 아님 (순간이동 · 참값 + 잡음). 누운 퍽 세우기는 건너뜀 (Q1 은 모두 서 있음).
- 집기 벌림 기본값 0.0125 (실물 열림 66.7 mm, `manipulator.yaml finger_m_at_open`). P0 chain 은 팀 시험과 같게 0.0042.
