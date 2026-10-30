# urdfusd_20261030v1 — waffle_4wd (4륜 + RM-X52 팔 + 퍽 플레이트) URDF · USD

시뮬레이션 (ROS 2 · Gazebo Sim, Isaac Sim 6.1 / Isaac Lab 3.0) 과 강화 학습에 쓰는 로봇 · 경기장 파일 묶음. 2026-10-30 판 (v1).

| 폴더 | 쓰는 곳 | 시작 |
|---|---|---|
| `waffle_4wd_description_261028/` | ROS 2 (RViz) · Gazebo Sim 패키지: `urdf/waffle_4wd.urdf` · `.urdf.xacro` (collision:=full/light), 메시, 센서 값, launch. 작업 공간 `src/` 에 넣고 `colcon build` | 그 폴더 `README.md`, 바뀐 것 `REVISION_261028.md` |
| `isaac/` | Isaac Sim / Isaac Lab: USD 2 개 (`usd/full` 정밀 · `usd/light` 가벼운), 경기장 장면 (`scenes/`), 로봇 · 센서 설정 (`waffle_4wd_isaac/`), 물리값 `robot_params.json`, 실행기 `run.sh` | 그 폴더 `README.md`, `./run.sh check` · `./run.sh maze` |
| `isaac/rl/` | 강화 학습 과제 (집기 · 옮기기 · 넣기) 코드 · 정책 · 체크포인트 | `isaac/rl/정책_학습기록.md`, `run_rl.sh` |
| `docs/puck_plate_placement/` | 퍽 플레이트 자리 계산 그림 (현재 / 제안, 위 · 옆, 완성 URDF, 경기장 출발 자리) | - |

## 주요 파일
| 파일 | 내용 |
|---|---|
| `waffle_4wd_description_261028/urdf/waffle_4wd.urdf` | 로봇 URDF (Gazebo 태그 포함). 메시는 `package://waffle_4wd_description_261028/meshes/...` |
| `isaac/urdf/waffle_4wd_{full,light}.urdf` | Isaac 변환용 URDF (gazebo 태그 뺌, 메시 상대 경로) |
| `isaac/usd/full/waffle_4wd_full/waffle_4wd_full.usda` | 로봇 USD 정밀 판 (퍽 넣기 시험) - `payloads/` 와 함께 옮길 것 |
| `isaac/usd/light/waffle_4wd_light/waffle_4wd_light.usda` | 로봇 USD 가벼운 판 (주행 · 학습) |
| `isaac/scenes/dice_mission_waffle_{full,light}.usda` | 주사위 경기장 + 로봇 (출발 구역, 신호등을 봄) |
| `isaac/scenes/dice_mission.usda` | 경기장만 |

## 이번 판 (v1, 2026-10-30) 에서 바뀐 것
- 퍽 보관통 3 개 + 레일 2 개 → **퍽 플레이트** (`puck_plate_link`, 원점 = 팔 joint1 축 · 상판 윗면 base_link (0.096, 0, 0.111) m, 컵 12 개). 전체 질량 3.8613 kg.
- Isaac USD 다시 변환 + 점검 (`isaac/results/`): 기존 점검 값 정상, 퍽 12/12 컵에 앉음 (원기둥 · 상자). Gazebo 도 12/12.
- 주사위 경기장 장면 (`isaac/scenes/`, `./run.sh maze`).
- 자세한 기록: `waffle_4wd_description_261028/REVISION_261028.md` 10 절, `isaac/README.md`.

## 주의
- 폴더 구조 그대로 둘 것: `isaac/urdf/*.urdf` 와 `isaac/puck_plate_check.py` 가 `../waffle_4wd_description_261028/` 를 상대 경로로 쓴다 (USD 는 메시가 안에 들어 있어 USD 폴더만 옮겨도 됨).
- 패키지 이름 `waffle_4wd_description_261028` 은 URDF 의 `package://` 경로라 바꾸지 않았다.
- 큰 파일: `waffle_4wd_description_261028/meshes/main/base_link.dae` 80 MB (GitHub 경고 50 MB · 한계 100 MB), USD `geometries.usd` 각 28 MB.
- `isaac/rl/` 의 옮기기 · 넣기 과제 목표는 아직 옛 보관통 3 개 기준 (퍽 플레이트 컵으로 바꿔야 함).
