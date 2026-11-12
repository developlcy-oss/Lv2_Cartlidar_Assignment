# real — 실물 팔 실측 · 시험 도구 (2026-11-11 ~ 12, cart11)

젯슨 홈에 복사해서 씀: `cp ~/Lv2_Cartlidar_Assignment/workspace/tools/arm_bench/real/*.py ~/`
기록은 모두 `~/실측/<이름>_<시각>/` 에 남음 → PC 로 `scp -r cart11@cart11.local:~/실측/<폴더> .`

| 파일 | 하는 일 | 실행 (젯슨) |
|---|---|---|
| `kbd_joy.py` | 조이스틱 없이 키보드로 /joy: `r` 리셋 (REARM) · `x` 비상정지 · w/s/a/d 주행 · `q` 끝 (끝나면 브링업 잠김 = 안전) | `python3 ~/kbd_joy.py` |
| `meas_session.py` | 실측 메뉴 (처짐 · 집게 폭 · 칸 좌표 · 무게 · 영점 보정 자세 · 상자 위 퍽 맞추기 · STOW↔DRIVE · 떨림 측정 13 번). 이동 중 E-stop 이면 goal 취소, 리셋하면 남은 이동을 이어서 | 브링업: `python3 ~/meas_session.py` · 직접 (브링업 끔): `python3 ~/meas_session.py --direct /dev/ttyACM0` (팔 보드 아니면 거부) |
| `pick_test.py` | 인식 (READ_TAG) → target_look → toppick_tilt 한 번 + 팔 카메라 영상 · 관절 · 로그 | `python3 ~/pick_test.py [메모]` |
| `lie_test.py` | 누운 퍽: target_look → lie_upright 한 번 + 기록 | `python3 ~/lie_test.py <상자 x> <상자 y> <요 deg> [메모]` |
| `estop_rec.py` | E-stop 시험 기록: 관절 실제 · arm_controller 원하는 위치 · /e_stop (600 s) | `python3 ~/estop_rec.py` |

- 브링업: `ros2 launch bringup bringup.launch.py skill_enabled:=true sim:=false` → `kbd_joy.py` 에서 `r`.
- 팔은 관절 1 ~ 5 가 대기 자세 (STOW) 일 때만 켜짐 (그리퍼는 아무 값). 끄기 전 메뉴 9 로 STOW.
- 결과 정리: #190 (실측) · #192 (누운 퍽) · #165 (덜컹 · 펌웨어 #242 · #244).
