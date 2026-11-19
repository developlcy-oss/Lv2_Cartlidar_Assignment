# real — 실물 팔 실측 · 시험 도구 (2026-11-11 ~ 12, cart11)

젯슨 홈에 복사해서 씀: `cp ~/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench/real/*.py ~/`
기록은 모두 `~/실측/<이름>_<시각>/` 에 남음 → PC 로 `scp -r cart11@cart11.local:~/실측/<폴더> .`

| 파일 | 하는 일 | 실행 (젯슨) |
|---|---|---|
| `kbd_joy.py` | 조이스틱 없이 키보드로 /joy: `r` 리셋 (REARM) · `x` 비상정지 · w/s/a/d 주행 · `q` 끝 (끝나면 브링업 잠김 = 안전) | `python3 ~/kbd_joy.py` |
| `meas_session.py` | 실측 메뉴 (처짐 · 집게 폭 · 칸 좌표 · 무게 · 영점 보정 자세 · 상자 위 퍽 맞추기 · STOW↔DRIVE · 떨림 측정 13 번). 이동 중 E-stop 이면 goal 취소, 리셋하면 남은 이동을 이어서 | 브링업: `python3 ~/meas_session.py` · 직접 (브링업 끔): `python3 ~/meas_session.py --direct /dev/ttyACM0` (팔 보드 아니면 거부) |
| `pick_test.py` | 인식 (READ_TAG) → J1 에 가장 가까운 퍽 target_look → toppick_tilt 한 번 + 팔 카메라 영상 · 관절 · 로그. `--load 열 칸` 이면 이어서 pallet_load, `--recheck` 면 처음 자리로 다시 가서 옆 퍽이 밀렸는지 잼 (2026-11-14). `--load auto` = 퍽 모양 · 계획기 규칙으로 칸 자동, `--occupied` 차 있는 칸, 노드가 시작 정리 중 거부면 10 s 뒤 다시, 설치된 노드 궤적이 실측 칸 맞춤과 다르면 경고 (2026-11-15) | `python3 ~/pick_test.py [--load 1 1] [--recheck] [메모]` |
| `load.sh` | 집기 → 팔레트 넣기 여러 번 (pick_test.py `--load auto` 를 이어 부름). 칸 = 계획기 `can_load` (뒤 열부터) · 퍽 모양 맞는 칸 · 막는 쌍 안 생기게, 성공한 칸을 다음 번 `--occupied` 로 넘김. `--slots` 고정 차례 · `--pause` · `--recheck` · `--keep_going` · `--dry` (2026-11-15) | `~/load.sh 2 --dry` · `~/load.sh 2 --pause` |
| `lie_test.py` | 누운 퍽: target_look → lie_upright 한 번 + 기록 | `python3 ~/lie_test.py <상자 x> <상자 y> <요 deg> [메모]` |
| `estop_rec.py` | E-stop 시험 기록: 관절 실제 · arm_controller 원하는 위치 · /e_stop (600 s) | `python3 ~/estop_rec.py` |

- 브링업: `ros2 launch bringup bringup.launch.py skill_enabled:=true sim:=false` → `kbd_joy.py` 에서 `r`.
- [2026-11-13] `meas_session.py` 메뉴 4 (칸 좌표) 의 칸은 **팔레트 v5 10 칸** — `~/pallet_place_traj.json` 또는 저장소 `ros2_ws/src/control/config/pallet_place_traj.json` 의 place 구간에서 읽음 (젯슨에 복사할 때 이 파일도 `~/` 로). 바닥 높이 `BASE_H` 7.25 mm (바퀴 반지름 0.0335).
  메뉴 10 · 4 · 12 결과 `6_보정자세.csv` 로 관절 영점 맞추기 → `../../arm_calib/` (`arm_calib_fit.py`).
- 팔은 관절 1 ~ 5 가 대기 자세 (STOW) 일 때만 켜짐 (그리퍼는 아무 값). 끄기 전 메뉴 9 로 STOW.
- 결과 정리: #190 (실측) · #192 (누운 퍽) · #165 (덜컹 · 펌웨어 #242 · #244).

## 팔 스킬 teleop — `arm_cmd.sh` · `dice.sh` (2026-11-13)
같은 명령으로 실물 (`real`, 젯슨: `/manipulator/execute_skill` 액션) 과 Isaac 시뮬 (`sim`, PC: 시뮬 스킬 서버 `/sim_skill/cmd`) 에 스킬 하나를 보냄. `dice.sh` 도 스킬은 모두 이걸로 보냄.

| 명령 | 스킬 |
|---|---|
| `arm_cmd.sh <sim\|real> pick` | `toppick_tilt` (실물은 먼저 `look`, 인식 모드 4 로 바꿈) |
| `arm_cmd.sh <sim\|real> load 3 1` / `unload 3 1` | `pallet_load_3_1_tilt` / `pallet_unload_3_1_tilt` |
| `arm_cmd.sh <sim\|real> place <x> <y> <z>` | `place_at:base_link:x:y:z` (z = 놓을 윗면 높이) |
| `arm_cmd.sh real look <x> <y> <z>` · `real place_cell L1 m1 p1` | `target_look:…` · `place_L1_m1_p1_tilt` |
| `arm_cmd.sh real pose DRIVE` | `/manipulator/pose` 이름 자세 (스킬 실패로 팔이 멈췄을 때 되돌리기) |
| `arm_cmd.sh sim transit\|reset\|cells\|status\|clear\|fill 3_0 2_1` | 시뮬 전용 |
| `dice.sh real L1 5 --slots "3_0 3_1 2_0 2_1 2_2" [--record]` · `dice.sh sim - 5` | 팔레트 칸에서 꺼내 장소 3×3 에 주사위 눈 (1~6) 배치 (mission DICE 와 같은 칸, 먼 칸부터). `--dry` 로 보낼 스킬만 확인, `--record` 면 ros2 bag → `~/실측/dice_*` |

- 끝 코드: 0 성공 · 1 실패 (실물은 결과 `success: true` 로 판정) · 2 서버 없음 · 3 시간 초과 (`ARM_TIMEOUT`, 기본 120 s). 실물은 `ROS_WS` 를 주면 그 `install/setup.bash` 를 불러옴.
- `sim` 은 PC 에서 `sim_cmd.py` (같은 폴더) 가 `~/ros2_lyrical` 로 보냄. 서버는 `workspace/urdf_and_usd/policy/manipulator/tools/README.md` 참고.

## 팔레트 칸 맞추기 · 실물 준비 도구 (2026-11-14)
| 파일 | 하는 일 | 실행 (젯슨) |
|---|---|---|
| `slot_teach.py` | 칸마다 칸에 넣은 퍽에 집게를 손으로 맞춰 기록 (`~/실측/slot_teach.json`) -> `--make` 로 노드 궤적 칸 근처 보정판 (`~/pallet_place_traj_taught.json`, 원본 `~/pallet_place_traj_orig.json` 기준). `--show` 칸별 차이, `--set 칸 방향 mm` 말로 보정 | 브링업 + `r` 뒤 `python3 ~/slot_teach.py [칸 …]` |
| `slot_teach.py --box` · `--fit_box` | 상자 위 놓는 자리 맞추기 (쥔 퍽 바닥이 윗면에 닿고 칸 가운데, 기본 9 칸 `~/실측/box_teach.json`) -> 노드 놓기 IK 영점 (`ik.joint_offsets`) 제안 · 셀별 오차 | 브링업 + `r` 뒤 `python3 ~/slot_teach.py --box` |
| `slot_teach.py --make_box` | (2026-11-17) 상자 기록 -> `~/place_box_teach.json` -> 저장소 `control/config/place_box_teach.json` + `manipulator.yaml` `place.teach.enabled: true`. 노드가 칸마다 모델 오차 (맞춘 자리 − 노드 FK) 를 빼고 놓기 IK (주변 칸 2 차 국소 맞춤, 처짐 보정 스트리밍 놓기에서) | `python3 ~/slot_teach.py --make_box` |
| `arm_only.sh` | 보드 확인 (읽기만: 팔 HOLD = 재부팅 안 됨 · 모터 못 읽음 구분) + 최소 브링업 (센서 없이, 포트 자동) | `~/arm_only.sh --check` · `~/arm_only.sh` |
| `arm_stow.py` | 브링업 끈 상태에서 대기 자세 (STOW) 범위 밖 관절만 범위 가운데로 (opencr_arm_tool 저속, 접는 차례 5 -> 1 -> 4 -> 2 -> 3) | `python3 ~/arm_stow.py [--check]` |
| `arm_free.py` | 칸 안에 멈춘 팔 빼내기 (벌림 -> 4 cm 위 -> 높은 자세 -> 정면) | `python3 ~/arm_free.py [--keep]` |
- 켜는 순서: 모터 전원 먼저 -> 팔 OpenCR 리셋 (켜진 채 모터만 재시작되면 토크가 안 걸림, 전류 0) -> `arm_only.sh --check` 팔 보드 `0x2`.
- 관절 1 큰 회전 중 USB 허브가 끊기면 ttyACM 번호가 바뀜 (예: 2 · 3) -> 브링업 `ports:=` 로 지정 (`arm_only.sh` 는 자동).
- 팔 속도 (2026-11-14): `manipulator.yaml` 의 `arm.max_joint_velocity` (0.8 rad/s) 하나가 기준. 노드 (이름 자세 · 팔레트 · 팔레트 진입 · 켜질 때 정리) 와
  시험 도구 (`meas_session` · `slot_teach` · `arm_free` 의 이동, `arm_stow`) 가 모두 이 값을 읽음. 정밀 구간 (3 열 쥔 채 넣기 `pallet.row3_hold_speed`,
  `meas_session` 손 맞춤 jog 한 걸음 1.5 s, 놓기 직선, 강화학습 정책 한계) 은 따로.
- 상자 놓기 손 맞춤 (2026-11-17, `place.teach`): 상자를 **가장 가까운 정지 자리 (5 cm)** 에 자 · 지그로 정확히 두고 9 칸 모두 맞춤 (상자 자리 오차는 그대로 들어감,
  J1 쪽 밖으로 늘리기는 약함). 다시 맞출 때: 팔 조립 · 영점 · 펌웨어가 바뀐 뒤. 확인: 첫 놓기 노드 로그 `place.teach: cell … model error` · `N / N cell(s) in use`
  -> `dice.sh real L1 5 --bench` 로 놓인 퍽 칸 오차를 자로 잼.
- 책상 시험 상자 자리 (2026-11-15): 로봇 앞 끝에서 **5 cm** (예전 12.72 cm). `dice.sh --bench` 가운데 칸 0.2853, `slot_teach.py --box` 같음. 6 cm 면 `--cell0 "0.2953 0.0 0.0633"` · `--cx 0.2953`.
