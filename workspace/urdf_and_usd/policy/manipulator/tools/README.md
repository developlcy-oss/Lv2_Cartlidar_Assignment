# 시뮬 스킬 서버 (2026-11-13, 실물 시험 연습용)

Isaac Lab 창 1 개 (차체 + 팔레트 + 상자 위 퍽 2 개) 에서 manipulator_node 와 같은 이름의 스킬을 하나씩 실행 — `arm_bench/real/arm_cmd.sh sim …` 로 보냄.

| 파일 | 하는 일 |
|---|---|
| `sim_skill_server.py` | `/sim_skill/cmd` (String) 받아 실행 → `/sim_skill/result` (JSON `{cmd, ok, msg, t}`). 스킬: `toppick_tilt` (정책 기본 k9, 노드 조건: READY_POSE_45 시작 · 퍽 좌표 한 번 잰 값 · 관절 속도 상한 · 행동 지연 0 ~ 1) · `pallet_load/unload_<열>_<칸>_tilt` (처짐 보정 ki 4, 벌림 36.4 mm) · `place_at:base_link:x:y:z` (45°, 앞 4 cm 직선, 옆 1.8 mm) · 시뮬 전용 `transit · reset · cells · status · clear · fill` |
| `sim_skill.sh` | `start [--headless] [--policy …] [--act_delay 0,1]` · `stop` · `log` — 컨테이너 `isaac-lab-base` 안에서 띄움 (`.gpu_lock` 확인 · 기록, 끝나면 지움) |

## 쓰는 법 (PC)
```bash
tools/sim_skill.sh start            # "준비됨" 까지 기다림
../../../tools/arm_bench/real/arm_cmd.sh sim pick
../../../tools/arm_bench/real/arm_cmd.sh sim load 3 1
../../../tools/arm_bench/real/arm_cmd.sh sim unload 3 1
../../../tools/arm_bench/real/arm_cmd.sh sim cells      # 상자 위 3x3 칸 좌표
../../../tools/arm_bench/real/arm_cmd.sh sim place <x> <y> <z>
tools/sim_skill.sh stop
```

## 주의 — 지금은 IsaacLab 쪽 폴더에서 돌림
서버는 Isaac Lab 컨테이너의 `/workspace/isaaclab/scripts/lv2_urdfusd/policy/manipulator` (사용자 IsaacLab 폴더) 에서 실행되고, 이 저장소 복사본에 아직 없는 것에 기댐:
- `waffle_isaac/sidepick_env*.py` 의 `act_delay_steps` · `applied_actions` · `tgt_n` (k9 학습 코드)
- `tools_ik/unload_traj.py` (+ `td_ik.py` · `armbody.py` · `selfcol.py`)
- `policies/sidepick45_k9.pt` (배포본은 `robot_policy/config/models/sidepick45_k9.onnx`)
학습 코드 동기화 때 같이 올라오면 이 폴더에서도 바로 돎. ROS 는 Isaac Sim 내장 jazzy rclpy (`isaacsim.ros2.core`), PC 쪽 `sim_cmd.py` 는 `~/ros2_lyrical` — 같은 도메인 · fastrtps.
