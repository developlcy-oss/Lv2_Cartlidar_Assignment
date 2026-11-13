# arm_calib — 팔 관절 영점 보정 맞추기 (2026-11-13)

실물 팔에서 잰 자세 (`6_보정자세.csv`) 로 `ros2_ws/src/control/config/manipulator.yaml` 의 `ik.joint_offsets` 를 맞추는 도구.
지금 값 (2026-11-12, 6 자세, RMS 6.7 mm) 은 **팔레트 v3 칸 자세**로 맞춘 처짐 포함 실효값이라, 팔레트 v5 로 바뀐 칸 자세에서는 다시 맞춰야 한다.

## 1. 재기 (젯슨, 팔 벤치 실측 메뉴)
`tools/arm_bench/real/meas_session.py` (사용법 `tools/arm_bench/real/README.md`):
- 메뉴 **10 영점 보정 자세**: P1 ~ P10 자세로 가서 **손끝** (두 손가락 끝 가운데) 에서 실 + 추를 내려 종이에 찍고 x (관절 1 축 점에서 앞 +) · y (왼 +) · z (바닥 위) mm.
- 메뉴 **4 칸 좌표** · **12 상자 위 퍽 맞추기**: 칸 · 상자 위 퍽에 집게를 손으로 맞춘 자세 → **잡는 점** (설계 칸 · 상자 자리).
- 결과 `~/실측/6_보정자세.csv` 를 PC 로 가져온다. 같은 자세를 2 ~ 3 번 재면 좋다.
- **팔레트 v5 칸** (메뉴 4 의 칸 목록이 v5 칸이어야 함) 과 **상자 위 가운데 · 좌 · 우** 를 꼭 넣는다 (실제 미션 자세 근처일수록 정확).
- 몇 줄은 맞추는 데 안 쓰고 확인용으로: CSV 에 열 `쓰임` 을 더하고 `확인` 이라 적는다.

CSV 꼴 (메뉴가 만드는 그대로):
```
이름,j1,j2,j3,j4,j5,x,y,z            # j = 읽은 관절 (도, 모터 값), x · y = 관절 1 축 점 기준 앞 · 왼, z = 바닥 위 (mm)
P1,0.12,-14.80,30.05,29.90,0.00,231.5,0.8,120.4
칸3:0,168.0,-15.7,39.5,21.2,0.0,-180.2,38.6,152.0
```
- 잰 점: 이름이 `칸` · `상자` 로 시작하면 **잡는 점** (gripper_roll_link 앞 0.140 m), 아니면 **손끝** (URDF 손가락 앞 끝, 0.1596 m). 열 `점` (`tcp` · `tip` · 숫자) 으로 바꿀 수 있음.
- ★ 메뉴 4 · 12 는 바닥 높이를 옛 값 6.75 mm 로 넣는다. 바퀴 반지름이 0.0335 m 로 바뀌어 지금은 7.25 mm → 이 도구 `--base_h 7.25` (기본) 로 맞춤 (실측했으면 그 값).

## 2. 맞추기 (PC)
```bash
cd ~/git/Lv2_Cartlidar_Assignment/workspace/tools/arm_calib
python3 arm_calib_fit.py ~/실측/6_보정자세.csv                 # 관절 1 ~ 4 영점
python3 arm_calib_fit.py ~/실측/6_보정자세.csv --sag 2,3       # + 관절 2 · 3 처짐 (자세 10 개 이상)
python3 arm_calib_fit.py ~/실측/6_보정자세.csv --z0            # + 바닥 높이 오차
```
나오는 것:
1. 자세별 끝 오차 — 보정 없음 · **지금 yaml 값** · 새 값, 새 값의 dx dy dz.
2. RMS (맞춘 자세 · 확인 자세) 와 **하나씩 빼고 맞춘 오차** (이것이 맞춘 자세 RMS 보다 많이 크면 자세를 더 잴 것).
3. 새 영점 (도) 과 지금 yaml 값 비교, (`--sag`) 처짐 크기.
4. **v5 팔레트 칸 놓는 자세 10 곳 · 상자 위 집기**에서 지금 yaml ↔ 새 값으로 명령했을 때 끝 오차 (mm).
5. 붙여 넣을 한 줄: `ik.joint_offsets: [ … ]`.

`--sag` 를 쓰면 영점 (e) 과 처짐 (k) 을 따로 맞춘 뒤, yaml 에는 하나의 값만 들어가므로 **미션 자세 (칸 · 상자 위) 의 평균 실효 영점**을 권한다.
남는 오차 (4 번 표) 는 자세마다 다른 처짐 — 놓기 처짐 보정 스트리밍 (`pallet.ki`) 이 줄여 준다.

## 3. 반영
1. `ros2_ws/src/control/config/manipulator.yaml` → `ik.joint_offsets` 에 붙여 넣고, 주석에 날짜 · 자세 수 · RMS 를 적는다.
2. `colcon build --packages-select control` → 실물: 칸 놓기 · 꺼내기 (3 · 2 · 1 열 각 1 칸), 상자 위 집기를 다시 시험.
3. CSV 는 `urdf_and_usd/policy/manipulator/기록/실측/` 에 남긴다 (다음에 다시 맞출 수 있게).

## 4. 모델 · 시험
- FK = 저장소 팀 URDF (`urdf_and_usd/waffle_4wd_description/urdf/waffle_4wd.urdf`) base_link → gripper_roll_link (관절 1 ~ 5). 관절 5 는 끝 위치에 영향이 없어 0.
- 실제 관절 각 = 읽은 각 − e (− k · L, L = 그 관절에서 잡는 점까지 수평 거리). 노드 명령 = 모델 각 + offset 이므로 offset = e.
- `python3 -m pytest -q test_arm_calib_fit.py` — 알고 넣은 영점 (관절 2 −27° 포함) · 처짐을 가짜 자료에서 되찾는지.
