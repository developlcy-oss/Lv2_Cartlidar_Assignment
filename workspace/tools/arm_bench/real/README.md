# real — 실물 팔 실측 · 시험 도구 (2026-11-11 ~ 12, cart11)

젯슨 홈에 복사해서 씀: `cp ~/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench/real/*.py ~/`
기록은 모두 `~/실측/<이름>_<시각>/` 에 남음 → PC 로 `scp -r cart11@cart11.local:~/실측/<폴더> .`

| 파일 | 하는 일 | 실행 (젯슨) |
|---|---|---|
| `kbd_joy.py` | 조이스틱 없이 키보드로 /joy: `r` 리셋 (REARM) · `x` 비상정지 · w/s/a/d 주행 · `q` 끝 (끝나면 브링업 잠김 = 안전) | `python3 ~/kbd_joy.py` |
| `meas_session.py` | 실측 메뉴 (처짐 · 집게 폭 · 칸 좌표 · 무게 · 영점 보정 자세 · 상자 위 퍽 맞추기 · STOW↔DRIVE · 떨림 측정 13 번). 이동 중 E-stop 이면 goal 취소, 리셋하면 남은 이동을 이어서 | 브링업: `python3 ~/meas_session.py` · 직접 (브링업 끔): `python3 ~/meas_session.py --direct /dev/ttyACM0` (팔 보드 아니면 거부) |
| `pick_test.py` | 인식 (READ_TAG) → J1 에 가장 가까운 퍽 target_look → toppick_tilt 한 번 + 팔 카메라 영상 · 관절 · 로그. `--load 열 칸` 이면 이어서 pallet_load, `--recheck` 면 처음 자리로 다시 가서 옆 퍽이 밀렸는지 잼 (2026-11-14) | `python3 ~/pick_test.py [--load 1 1] [--recheck] [메모]` |
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
| `arm_cmd.sh sim transit\|reset\|cells\|status\|clear\|fill 3_0 2_1` | 시뮬 전용 |
| `dice.sh real L1 5 --slots "3_0 3_1 2_0 2_1 2_2" [--record]` · `dice.sh sim - 5` | 팔레트 칸에서 꺼내 장소 3×3 에 주사위 눈 (1~6) 배치 (mission DICE 와 같은 칸, 먼 칸부터). `--dry` 로 보낼 스킬만 확인, `--record` 면 ros2 bag → `~/실측/dice_*` |

- 끝 코드: 0 성공 · 1 실패 · 2 서버 없음 · 3 시간 초과 (`ARM_TIMEOUT`, 기본 120 s). 실물은 `ROS_WS` 를 주면 그 `install/setup.bash` 를 불러옴.
- `sim` 은 PC 에서 `sim_cmd.py` (같은 폴더) 가 `~/ros2_lyrical` 로 보냄. 서버는 `workspace/urdf_and_usd/policy/manipulator/tools/README.md` 참고.
