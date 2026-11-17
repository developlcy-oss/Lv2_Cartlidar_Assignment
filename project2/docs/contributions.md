# 팀원별 기여 · 근거

저장소: https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment
기준 시각: 2026-11-17 (KST), `dev` · `Integration` 병합 기록과 GitHub PR · 이슈 기준.
같이 볼 문서: [architecture.md](architecture.md) (전체 구조) · [issues.md](issues.md) (이슈 표) · [contributions_log.md](contributions_log.md) (사람별 이슈 · 날짜별 커밋 전체)

팀원마다 **맡은 일** 과 근거 네 가지를 적는다. 근거는 할 일 목록이 아니라, 링크와 결과가 있는 것만 적는다.

| 근거 | 무엇을 적나 |
|---|---|
| 구현 | 직접 작성해 병합된 PR: 링크, 바뀐 기능, 효과 |
| 검증 | 직접 돌린 시험: 조건, 기대, 실제, 원자료 (영상 · 로그), 한계와 개선 |
| 리뷰 | 다른 팀원 PR · 작업에 준 의견: 링크, 구체적인 문제 · 의견, 반영 여부 (후속) |
| 설명 | 전체 흐름과, 내가 맡지 않은 코드 · 설정 · 시험 결과를 이해한다는 근거 |

팀 PR 집계 (GitHub, 2026-11-17)

| 팀원 | GitHub | 병합된 PR | 닫힌 PR (병합 안 됨) |
|---|---|---|---|
| 이창엽 (팀장) | developlcy-oss | 47 | 2 |
| 이홍주 | kanichong | 55 | 0 |
| 온창범 | Onbeom | 91 | 1 |
| 한상준 | WindForce08 | 16 | 0 |

---

## 0. 팀원별 저장소 기록 요약 (커밋 · 이슈 · 댓글 · PR)

`dev` 의 병합 제외 커밋 (작성자 이메일로 구분) 과 GitHub 이슈 · 이슈 댓글 · PR 을 사람별로 센 것이다 (2026-11-17).
줄 수는 메시 · 모델 · CSV · JSON · 이미지 · 로그 같은 데이터 파일을 뺀 **코드 · 문서** 줄이다. 전체 목록은 [contributions_log.md](contributions_log.md).

| | 이창엽 (developlcy-oss) | 이홍주 (kanichong) | 온창범 (Onbeom) | 한상준 (WindForce08) |
|---|---|---|---|---|
| 커밋 (dev) | 68 | 79 | **144** | 30 |
| 일한 날 (커밋 기준) | 15 일 (10-16 ~ 11-17) | 18 일 (10-22 ~ 11-17) | **23 일** (10-22 ~ 11-16) | 13 일 (10-22 ~ 11-16) |
| 코드 · 문서 줄 (+ / −) | +42,679 / −2,828 | **+68,042** / −30,720 | +25,646 / −2,471 | +17,448 / −584 |
| 많이 바꾼 곳 | URDF · Isaac 모델, `control` (manipulator), RL 정책, `tools/arm_bench`, `project2` | URDF · Isaac 모델 생성기, `mission_planner`, `control`, RL 정책 (TopPick) | `perception`, `mission`, `localization`, RL 정책 (lie_upright) | `opencr_hardware` · `opencr_firmware`, `tools/arm_bench` · `drive_demo` · `drive_rl`, `navigation` |
| 작성 PR (병합) | 49 (47) | 55 (55) | 93 (91) | 16 (16) |
| 올린 이슈 (열림) | 7 (0) | 30 (2) | **70** (14) | 3 (0) |
| 이슈 · PR 댓글 (다른 팀원 글에) | 62 (51) | **89** (56) | 59 (15) | 45 (40) |
| 리뷰한 다른 팀원 PR (본문 있는 리뷰 수) | **153** (2) | 3 (4) | 1 (1) | 6 (5) |
| 다른 팀원 PR 병합 | **161** | 1 | 0 | 2 |

누가 누구 글에 댓글을 달았나 (이슈 · PR 댓글 수)

| 댓글 쓴 사람 ↓ / 글 쓴 사람 → | 이창엽 | 이홍주 | 온창범 | 한상준 |
|---|---|---|---|---|
| 이창엽 | — | 4 | **46** | 1 |
| 이홍주 | 5 | — | **47** | 4 |
| 온창범 | 7 | 7 | — | 1 |
| 한상준 | 5 | 12 | **23** | — |

읽는 법: 온창범 이 이슈를 가장 많이 올렸고 (70 건, 인지 쪽 요청 · 실물 측정 결과), 나머지 세 명이 그 이슈에 답하며 일을 나눴다.
PR 리뷰는 팀장이 거의 다 승인 · 병합했고 (153 건 리뷰 · 161 건 병합) 글로 남긴 리뷰는 팀 전체에 적다 — 구체적인 검토 의견은 대부분 이슈 댓글에 있다.

### 0.1 이창엽 — 팀장 · 저장소 · 매니퓰레이터 · 강화학습 · 실물 팔

- **10-16 ~ 10-30 시작 · 뼈대**: 저장소 · 기본 트리 · 노드 구조, `center` → `mission` 이름 바꿈과 mission · control 노드 ([#4](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/4)), 4 륜 바퀴 펌웨어 실물 맞춤 ([#10](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/10)), 261028 URDF · Isaac USD · 매니퓰레이터 강화학습 첫 반영 ([#20](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/20)).
- **11-03 ~ 11-06 미션 · 팔 동작**: mission 의 인지 출력 사용 (신호등 · 바코드 · 장소 태그 · 주사위 · 퍽 측량, [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94)), 측량 다시 보기 거리 · 발제 좌표 위치표, observe · verify 계산 자세, RL 집기 시작 자세 · 재시도, 위치 잃음 동안 기다리기.
- **11-08 ~ 11-16 젯슨 실물 팔 · 정책**: 켜질 때 자동 STOW · HOLD 안전 정지, 처짐 보정, 45° 집기 정책 k7 → k8 (실측 영점) → k9 (실측 지연), 팔 덜컹 1.4° → 0.07°, E-stop 튐, 쥠 판정, 최소 저크 궤적, 팔 안전 검사, 수직 넣기, 실측 칸 좌표 · 팔 속도 통일, 관절 1 180° 대응 (#311 → #314 → #316). Isaac 꺼내기 → 놓기 98.2 ~ 98.3 %.
- **올린 이슈 7 건**은 모두 다른 파트에 걸친 문제를 정리한 것: 팔 카메라로 손 안 퍽 재기 ([#128](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/128)), 팔 카메라 보정 파일 없음 ([#143](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/143)), 팔 시작 정리 ([#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144)), RL 운영 · 진도 ([#185](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/185) · [#190](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/190)), 상자 앞 정차 간격 ([#194](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/194)), 칸 배정에 퍽 모양 ([#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230)).
- **댓글 62 개 중 46 개가 온창범 이슈**: 인지 → 매니퓰레이터 요청 (팔 카메라 자세, 정책 관측 오차, 누운 직육면체 다시 잡기 등) 에 답하고 정한 것.
- **통합 관리**: 다른 팀원 PR 153 건 리뷰 · 161 건 병합, `Integration` → `dev` 병합, 최종 제출물 (`project2/`, #321 · #322).

### 0.2 이홍주 — 미션 계획기 · 시뮬레이션 모델 · TopPick 강화학습 · 통합 시험

- **10-22 ~ 10-28 기반**: 초기 SYSTEM 구조, 증속벨트 · drive 구동 구성 선택, 주행 skill 장애물 처리 · 팔 정책 속도 제한, Isaac Sim 6.2 브리지 (TopicBasedSystem), TF · 카메라 프레임 규약 · URDF 시험 · USD 정적 검사.
- **11-02 ~ 11-05 미션 설계 · 계획기**: mission plan 모드 ↔ `mission_planner_node`, **할당 · 적재 · 경로 계획기 (GTSP)** · 문서 좌표 지도 · AMCL 설정, 모든 집기 · 놓기 45° · 상자 퍽 ±17° 회전, 팔레트 놓기 · 꺼내기 경로, 처짐 보정 스트리밍, TopPick 물리 무작위화 · 학습 기록, TopPick 정책 실기 연결 · 집기 → 칸 이어 평가, 통합 launch.
- **11-06 ~ 11-10 Nav2 · Gazebo 통합 시험 · 중간 제출**: Nav2 통합 실행 · loc_gate · Gazebo 미션 연동 시험, 좁은 통로 대기 자리 (#135), FINE 정렬 float32 거부 버그 · TFmini 비상 정지 유지 (#157 · #168), TP8 · TP9 정책, **중간 제출물 · 도전 B (`challenge/B`) · `mission_submit`**.
- **11-11 ~ 11-14 기록 · 채점 · 실물 대응**: 재검증 기록 중복 합치기 (#172), 복귀 기록 · **채점 스크립트 `mission_score`**, 부분 실물 시험 `stop_after`, 칸 배정에 퍽 모양 (#230), `avoid_slots` (#219), 시작 박스 회전 범위 (#226), 세운 퍽 먼저 (#193), B 안 모델 생성기 (증속벨트 v6.1 · v7 · 캐리어 v7 · LDS-03 브라켓), `odom_icr` C++ · 계획기 시작 33 → 3 s (#265), spawner 대기 · bringup 순서 (#275), 준비 확인 · 퍽 건너뛰기 · HOME 복귀.
- **11-15 ~ 11-17**: Nav2 한 프로세스 (`nav2_composed`), 무작위 퍽 자리 측량 · 계획, 떨어뜨린 퍽 다시 집기, TopPick TP14 ~ TP17, 장소 상자 놓기 Isaac 평가 `--place`.
- **올린 이슈 30 건**: 11-02 ~ 11-04 에 미션을 할 일 단위로 쪼갠 이슈 (skill 목록 #63, FINE 간격 #64, observe 자세 #65, 정책 배포 #69 · #78, 팔레트 속도 #75, 회전 skill #77, 실측 항목 #87 · #109, 시험 #110 ~ #116) 와 11-09 이후 통합 버그 (#135 · #151 · #168) · 실측 요청 (#266).
- **댓글 89 개 (팀 최다), 다른 팀원 글에 56**: 온창범 이슈 47 개에 답함 (인지 요청 → 계획기 · 미션 반영).

### 0.3 온창범 — 인지 (perception) · 위치 추정 · 실물 측정 · 누운 퍽 세우기

- **10-22 ~ 11-06 인지 전체**: 카메라 영상 토픽 · 라이다 합치기 · TFmini 비상 정지, **신호등 점멸 순서 세기 · 주사위 3×3 확인** (메시지 · 서비스 포함), 퍽 · 장소 태그 (16 mm · tag36h11 48 mm) · 끝면 모양, 같은 색 퍽 깊이로 나누기, **바코드 R3 C++ (D435 라벨 찾기 · 팔 카메라 zbar)**, 카메라 사이 퍽 병합, YOLO 엔진 (D → E2 → E3, FP16) · 엔진 해시 검사, B0 측량 (`perception_survey`) · 다시 보기, 뒤 D435 `rear_obstacle`, 팔 카메라 평면 교차 `/arm_range`, 장소 상자 윗면 찾기, 따뜻한 조명 신호등 문턱.
- **위치 추정**: `localization` 패키지 (loc_monitor · 지도 · AMCL), 거짓 LOST 고침, 11-16 위치 추정 노드 C++ 판.
- **11-09 ~ 11-10 실물 · 미션 기록**: 실물 IMU yaw −90°, 팔 카메라 태그 검색 · 촬영 시각 TF, **미션마다 기록 폴더 · 이동 전후 사진 · 계획기 기록 · 퍽 등록표 사본** (채점 근거), `/perception/snapshot`.
- **11-11 누운 직육면체 세우기 RL**: Isaac `Waffle-LieUpright` 환경 · 정책 r2e → r2g2 · 관절 속도 비율 축소 (#192 · #193).
- **11-12 ~ 11-16 실물 대응**: 시작 박스 재기 · 비스듬히 서기 (#226 · #229), 앞 · 뒤 D435 피치 실측, place_box 배율 · 시차 보정, udev (TFmini · 라이다), 팔 카메라 자세별 실측 보정 (#278), 라이다 TF 요 180° (#303), survey TF 리스너 CPU 절약.
- **올린 이슈 70 건 (팀 최다, 열림 14)**: 10-27 ~ 11-03 은 다른 파트에 인지 출력을 연결해 달라는 `[요청]` (mission · interfaces · bringup · Isaac 브리지 · drive), 11-09 이후는 실물에서 찾은 문제와 측정 결과 — 관절 1 영점 180° (#161), 팔 펌웨어 실기 확인 (#165), 정책 관측 오차 실측 (#191), ALIGN · DRIVE 자세 충돌 계산 (#218 · #219), 직진 옆 미끄럼 2.35° (#248), Jetson CPU 85 ~ 95 % (#265), spawner · bt_navigator 실패 (#275 · #276), 팔 카메라 높이 · 링크 길이 차이 (#278 · #295), Cartographer 지도 시도 (#317).

### 0.4 한상준 — OpenCR 펌웨어 · 팔 벤치 · 주행 (Nav2 · 주행 RL · 시연 도구)

- **10-22 ~ 10-27 펌웨어 · 팔 벤치**: 4 륜 전환 · 역할별 펌웨어 빌드 (wheel · arm), **펌웨어 v5 안전 코어 · SIL 시험**, 팔 벤치 도구 · 기록 (범위 · 게인 · 그리퍼 보호), pick RL 과제.
- **11-03 ~ 11-04 주행**: Isaac Lab 미로 주행 RL (`drive_rl`), `opencr_hardware` 백엔드 선택 (opencr · mock · isaac), **Nav2 패키지** (Smac 2D · RotationShim + MPPI · Gazebo 시험), 후진 때 뒤 카메라 확인 (#84), 출발 자세 통일.
- **11-09 ~ 11-13 실물 안전 · 주행 한계**: 팔이 대기 자세가 아니면 시작 거부 (#166), 실물 기본 지도 = 발제 SDF 지도 (#116), `/arm/skill_state` 등 삭제 (#86), footprint 에 주행 자세 팔, 바퀴 속도 한계 7.51 rad/s · 보드별 배터리 상태, diff_drive 회전 상한 1.0 rad/s (벨트 보호), RC-100 리모컨 주행.
- **11-15 ~ 11-16 경기 주행**: **`drive_demo` 규칙 v2 주행 시연 도구 · 팀원용 시험 안내**, 조각 실패 시 되찾기 · 실물형 회전 시뮬, 방향 맞추기 거부 시 건너뛰기 — 11-16 공식 1 · 2 회차에 쓴 도구 ([results/official_scoring.md](../results/official_scoring.md)), 11-15 실물 실행 기록은 [results/reproduction.md](../results/reproduction.md).
- **올린 이슈 3 건**은 모두 실물 통합 막힘: 팔 카메라가 READY · BOX_VIEW 에서 퍽을 못 봄 (#154), FINE 정렬 뒤 TFmini 비상 정지 재발 (#157), 빈 상자 장소 접근 실패 (#160).
- **댓글 45 개 중 40 개가 다른 팀원 글**: 온창범 이슈 23 · 이홍주 12 · 이창엽 5. 리뷰한 PR 6 건에 본문 있는 리뷰 5 개 (구체적인 것은 #153).

---

## 1. 이창엽 (팀장 · developlcy-oss)

### 1.1 맡은 일

| 영역 | 내용 | 주요 파일 |
|---|---|---|
| 미션 | `mission_node` (단계 실행 · 재시도 · PAUSED), 퍽 측량 · 다시 보기, 위치표 (발제 좌표) | `workspace/ros2_ws/src/mission/` |
| 팔 제어 | `manipulator_node` (pose · pick · execute_skill), IK, 켜질 때 자동 정리 (STOW → DRIVE), 스킬 각본 (넣기 · 꺼내기 · 주사위 놓기), 처짐 보정, E-stop · HOLD 처리, 쥠 판정, 최소 저크 이동 | `control/src/manipulator_node.cpp`, `control/include/control/*.hpp`, `control/config/manipulator.yaml` |
| 정책 배포 | 강화학습 집기 정책 k7 · k8 · k9 를 노드에 올림 (관측 33 조립, 정책별 설정) | `robot_policy/config/policies.yaml`, `robot_policy/config/models/` |
| 강화학습 · 평가 | Isaac Lab 학습 · 노드와 같은 조건 평가 (집기 → 넣기, 꺼내기 → 놓기, 주사위 놓기) | IsaacLab 작업본 (저장소 밖), 결과는 PR · 이슈 본문 |
| 펌웨어 | OpenCR 팔 펌웨어 (E-stop 목표 유지, 게인, HOLD 막힌 관절 풀기, 관절 1 π 보정 v6) | `opencr_firmware/`, `opencr_hardware/` |
| 실물 | 젯슨 브링업, 팔 실측 · 시험 도구 | `workspace/tools/arm_bench/real/` |
| 팀장 | `dev` · `Integration` 병합 (다른 팀원 PR 161 건 병합), 이슈 정리 | - |

### 1.2 구현 (작성 · 병합된 PR)

**미션**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#4](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/4) (10-25) | `center` → `mission` 패키지, `mission_node` 새로 씀 (1,110줄). 순서만 정하고 바퀴 · 팔은 Action 으로 맡김. 단계 문법 `navigate · search · approach · pick · pose · skill`, 실패 시 goal 취소 · 단계별 재시도, E-stop · OpenCR ERROR 면 PAUSED | 미션 · 주행 · 팔 노드의 책임 경계가 이때 정해짐. control 컨테이너 launch 도 같이 만듦 |
| [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94) (11-03) | mission 이 인지 출력 (신호등 · 바코드 · 장소 태그 · 주사위 · 퍽 측량) 을 받아 씀, 측량 실패 시 문제 칸만 다시 보기. manipulator 관절 한계 · STOW · 미션 skill, 팔레트 궤적 `pallet_place_traj.json` | 인지 담당 리뷰 2건 반영 뒤 병합 (아래 1.4 받은 리뷰) |
| [#98](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/98) · [#101](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/101) (11-04) | 다시 보기 거리 `survey.revisit_approach_distance` (0.20 m), 시작 검사 (≥ 0.13 m, TFmini 하한). 위치표를 발제 좌표 (map) 로 | 잘못된 거리면 노드가 시작 단계에서 실패 → 실행 중 사고 방지 |
| [#127](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/127) · [#132](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/132) · [#134](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/134) (11-06) | observe · verify 팔 자세를 계산 자세로 (`box_look` · `barcode_look`), 집기 정책 시작 자세 · 12 s 상한 · 실패 시 `target_look` 부터 재시도, 위치를 잃은 동안 Nav2 단계 대기 (90 s 넘으면 PAUSED) | 전: 위치 잃으면 약 6 분 뒤 FAILED (재개 불가) → 후: 기다렸다 이어 감, 넘으면 PAUSED 로 재개 가능 |
| [#202](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/202) (11-11) | FINE 정렬 팔 자세 READY → ALIGN, `skill.policies` 를 `[toppick_tilt]` 로 | READY 가 상단 카메라에 닿던 문제 (#144) 제거. 없는 모델 하나 때문에 정책 전체가 로드 안 되던 문제도 고침 |

**팔 제어 (manipulator_node · control)**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#189](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/189) (11-10) | 켜질 때 DRIVE 로 자동 정리 (J3 → J2 → J1 → J4, HOLD 해제 뒤에만), 스트리밍 (처짐 보정 · 정책) 은 HOLD · E-stop 동안 멈춤 | 11-08 실물에서 매번 손으로 팔을 접던 일을 없앰 (테스트 브랜치에서 실물 자동 정리 성공 후 dev 반영). 병합 당시 빌드 · 단위 시험은 아직이었음 |
| [#204](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/204) · [#205](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/205) · [#209](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/209) (11-11) | 주사위 놓기 집게 벌림 축소 · 처짐 보정, 놓기에도 처짐 보정 스트리밍 (`sag_comp.hpp`), 놓기 실측 도구 | 모의: 주사위 놓기 33.6 % → 90.0 %, 꺼내기 → 놓기 2.8 % → 95.8 % (아래 1.3) |
| [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) (11-11) | 집기 정책 k7 (관측 33) 노드 조립, 정책별 관측 기준 자세 · 시작 자세 · 손가락 값, `skill.toppick_tilt_policy` | 관측 36 만 받던 노드가 다른 관측 형식 정책도 받게 됨. 관측 시험 3개 통과 |
| [#231](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/231) (11-12) | k8 정책, 실측 관절 영점 보정 `ik.joint_offsets`, 정책별 관절 속도 상한 (4.0 rad/s), 놓기 옆 보정, ALIGN = E2 | 6 점 실측으로 남은 오차 평균 6.7 mm. 속도 상한 2.0 → 4.0 으로 k7 노드 조건 92.9 % → 96.6 % (원기둥, 모의) |
| [#236](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/236) (11-12) | 집게 쥠 판정을 위치 범위로, 연속 쥠 신호 `/manipulator/grasped` (30 Hz, 8 / 3 스텝 히스테리시스), 정책 집기 성공에도 쥠 판정 요구 | 실물에서 빈손을 "성공" 으로 보고하던 문제 (11-12 시험) 를 막음 |
| [#244](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/244) (11-12) | E-stop · REARM 때 팔 튐: 붙잡은 목표 유지, REARM 뒤 0.3 rad/s 로 천천히 따라감, 비상정지 중 동작 취소 | 젯슨에서 빌드 · 확인, 튐 줄어듦 (정량값 없음) |
| [#271](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/271) (11-13) | 팔 관절 이동을 최소 저크 곡선으로, 켜질 때 정리 0.1 → 0.3 rad/s | 정리 시간 약 55 s → 단축 (0.2 rad/s 실측 33.1 s) |
| [#273](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/273) (11-14) | 팔레트 궤적 출발 자세 검사, 시작 정리 끝나기 전 명령 거절 | 실물에서 팔꿈치가 퍽 스토리지를 내려친 사고 뒤 넣음 |
| [#282](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/282) (11-14) | 집은 퍽을 0.085 m 들고, 팔레트는 높이 이동 뒤 칸 위에서 수직 넣기 | 엄격 판정 모의: 노드 집기 → 넣기 59.0 % → 83.0 %, 꽉 찬 스토리지 8.0 % → 90.2 % |
| [#286](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/286) · [#287](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/287) (11-14) | STOW J1 1.554 → 2.200 (팀 URDF 충돌 계산, `fold_clearance.py`), `target_look` 퍽 높이를 상자 윗면 기준으로 | 예전 STOW 는 1 열 끝 칸 벽과 −3.5 mm 겹침 → 새 값 최소 9.2 mm. 팔 카메라 높이 오차 8 ~ 22 mm 를 집기 높이에서 뺌 |
| [#290](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/290) · [#291](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/291) · [#293](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/293) (11-14 ~ 15) | 팔레트 v5 실물 10 칸 좌표 반영, 팔 속도 기준 하나 (`arm.max_joint_velocity` 0.8), 칸 위 곧게 내려가기 · 쥠 판정 상한 0.863 | 실물 주사위 배치 실패 (원기둥을 쥐고도 'empty grasp', 칸 축 벗어난 하강) 원인을 고침 |
| [#297](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/297) · [#300](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/300) (11-15) | 상자 5 cm 정지 기준 주사위 놓기 칸별 보정, 고정 동작 집기 접근 높이 0.12 (풀리지 않으면 0.08 · 0.05) | 모의: 5 cm 놓기 87.5 % → 97.5 % (512 대), 세운 직육면체 다시 집기 68.0 % → 84.0 % |
| [#301](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/301) · [#311](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/311) · [#314](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/314) (11-15 ~ 16) | 관절 1 브라켓 재조립에 따라 STOW 값 변경 | 관절 1 모터가 모델과 180° 돌아 달린 것을 실물로 확정 (아래 #316 으로 정리). 한계 1.6 참고 |
| [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) (11-16) | TF 리스너를 작업 중에만 켬 (`tf_idle_off` 30 s) | PC 측정 3.22 → 2.62 % (−19 %). 리뷰에서 "변화가 작다" 는 반대 의견이 있었음 (1.4 받은 리뷰) |

**정책 · 펌웨어 · 도구 · 브링업**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#20](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/20) (10-30) | URDF 261028 · Isaac USD · 경기장 장면 · 매니퓰레이터 강화학습 코드 · 학습 기록 | 팀 Isaac 시뮬 기준 모델. 당시 집기 61 %, 이어 붙이기 0/64 를 그대로 기록 |
| [#245](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/245) (11-12) | 집기 정책 k9 (실측 지연 0 ~ 66 ms 학습), `skill.horizon` 1.0 시험 권장 | 지연을 건 평가에서 k8 원기둥 69.5 % → k9 75.7 %, 지연 0 ~ 33 ms 가정 91.4 / 93.1 % (모의) |
| [#10](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/10) (10-27) | 주행 펌웨어 바퀴 ID · 기어 · 속도 한계 (0.261 m/s), 바퀴 시험 도구 `opencr_wheel_tool.py` | ROS 없이 바퀴 단독 시험 가능 |
| [#233](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/233) · [#242](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/242) · [#243](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/243) (11-12) | `arm_controller` `interpolate_from_desired_state`, 펌웨어 E-stop 목표 유지 · 관절 2 · 3 I 게인 100 · 손목 가속 제한, 대기 자세 검사 그리퍼 범위 | 관절 2 · 3 덜컹 평균 1.3° → 0.07° (실물, 1.3 검증) |
| [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284) (11-14) | HOLD 중 막힌 관절 풀기, 모터별 HW 오류 · 온도 진단 (`/diagnostics`), 토크 ON 전 목표 쓰기 | 온창범 요청 #259 · #277 처리. SIL 3개 추가, colcon test 145 개 통과 |
| [#316](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/316) (11-16) | 팔 펌웨어 v6: 관절 1 만 확장 위치 모드로 π 보정, 역할별 버전 검사, STOW 모델 값 복귀, `j1_flip_bench.sh` | IK · 정책 · 이름 자세를 모델 각도 그대로 쓰게 함. 빌드 · 시험 통과, **업로드 · 실물 확인은 하지 않음** |
| [#246](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/246) · [#255](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/255) · [#298](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/298) | 실물 팔 실측 · 시험 도구 (`kbd_joy.py`, `meas_session.py`, `pick_test.py`), 팔 스킬 teleop `arm_cmd.sh` · `dice.sh` + Isaac 시뮬 스킬 서버, 영점 + 링크 길이 맞추기 `arm_calib_fit.py --links` | 시뮬과 실물을 같은 bash 명령으로 시험. 시뮬에서 집기 → 넣기 → 꺼내기 → 놓기 전 과정, 주사위 눈 6/6 성공 |
| [#238](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/238) · [#299](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/299) | 브링업 `sim` 인자 선언 순서 버그, `split_control:=true` (control 컨테이너를 프로세스 넷으로 나눠 노드별 CPU 측정, #265) | `sim:=` 없이 띄우면 실패하던 문제 해결. 젯슨 측정에서 manipulator_node 22.2 (코어 하나 = 100) 를 찾아 #312 로 이어짐 |

그 밖의 병합 PR: [#281](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/281), [#294](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/294). 닫힌 PR 2 건: [#254](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/254) (#255 로 합침), [#274](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/274) (커밋은 다른 경로로 dev 에 들어감).

### 1.3 검증 (직접 돌린 시험)

원자료 위치: 이슈 · PR 댓글 (링크), 저장소 [`project2/results/real_20261116/`](../results/real_20261116/). 시험 영상 · 학습 기록 일부는 **저장소 밖 IsaacLab 작업본** (`policy/manipulator/기록/`) 에만 있다 (한계 1.6).

**실물**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 켜질 때 자동 정리 (11-08) | 젯슨 cart11, 테스트 브랜치 `feat/test-lcy001`, 조이스틱 없이 `/joy` 리셋으로 HOLD 해제 | HOLD 해제 뒤 STOW 도착 | `stage 1/2 (12.3 s)` · `stage 2/2 (8.8 s)` → `arm at 'STOW'`. 손목은 혼 어긋남으로 묶음에서 뺌 | [#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144) | 테스트 브랜치에서만 확인 → dev 반영은 #189 (목표 DRIVE) |
| 팔 처짐 · 집게 폭 · 칸 좌표 실측 (11-11) | 바퀴 보드 FAULT 로 브링업 불가 → 팔 OpenCR 직접 연결, 줄자 ±10 mm, 캘리퍼스 | 설계 FK 와 비슷 | 0 자세 설계 311 → 실측 220 mm (−91), 집게 빈손 닫힘 23.8 mm (0.922 rad), 원기둥 쥠 0.571, 칸 3 곳이 한 덩어리로 x +28 · 높이 +52 mm 어긋남 | [#190 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/190#issuecomment-6798080681) | 처짐은 한 오프셋으로 안 맞음 (잔차 ±15 ~ 30 mm) → #231 영점 보정, #236 쥠 판정 범위 근거 |
| 강화학습 집기 실물 (11-12 오전) | `skill_enabled:=true`, 팔 Arducam 만, `pick_test.py` (영상 · 관절 CSV · 로그) | 퍽을 쥐고 들기 | **실패 2회**: 상자에 박고 빈손인데 `success: true`, 두 번째는 목표 z 0.171 m 보다 약 5 cm 아래로 파고들어 12 s 시간 끝. 인식 퍽 높이 +25 ~ +64 mm 치우침 | [#190 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/190#issuecomment-6811725884), 영상 `pick_1112_*/arm.mp4` (IsaacLab 작업본) | 모의 정책이 실물 처짐 · 지연 · 인식 치우침을 본 적 없음 → 빈손 성공은 #236, 높이는 #287, 지연은 k9 (#245) 로 대응. **k9 실물 재시험은 못 함** |
| 팔 덜컹 측정 (11-12 오후) | 앞 45° 자세, 관절 1 ~ 4 를 ±5° · 0.4 s, 각 · 전류 100 Hz (`meas_session.py` 메뉴 13) | 진동 · 덜컹 원인 찾기 | 모터 진동 없음 (정지 0.088°). 덜컹 원인 = 처진 실측 위치에서 새 동작 시작. 직접 모드 평균 1.3° / 최대 5.1° → 고친 뒤 **브링업 실물 평균 0.07° / 최대 0.08°** | [#165 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/165#issuecomment-6814553855), [#233 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/233#issuecomment-6820146134) | 처음 낸 "D 게인" 제안은 측정 뒤 스스로 철회. 손목 · 집게 관성 출렁임은 엔코더로 안 보여 남음 |
| **실물 미션 (11-16 12:36)** | 젯슨 브링업, `mission_source:=plan` | 첫 단계 observe (`barcode_look`) 팔 자세 | **FAILED**: `observe failed after 2 retries: manipulator: /manipulator/execute_skill goal rejected`. 팔 로그: `startup: arm is not at STOW (0.23 rad) nor 'DRIVE' ... -> not moving` → `execute_skill: startup to 'DRIVE' not finished -> reject` | [`mission_log/261116_1236/mission_log.jsonl`](../results/real_20261116/mission_log/261116_1236/mission_log.jsonl), [`launch_logs/demo_261116_123545.log`](../results/real_20261116/launch_logs/demo_261116_123545.log) | 내가 넣은 안전 검사 (#273 · 알려진 자세 근처에서만 정리) 는 설계대로 동작했지만, 그날 STOW 정의가 여러 번 바뀌어 (#311 → #314 → #316) 실제 팔 자리와 0.23 rad 어긋남 → 팔이 미션에 한 번도 들어가지 못함. 개선: 펌웨어 v6 업로드 · STOW 한 값으로 고정 · 시작 전 준비 확인에 "startup 끝남" 포함 |

**모의 (Isaac Lab, 노드와 같은 조건)**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 주사위 눈 놓기 (11-11) | 2,048 대, L1 ~ L3 × 눈 1 ~ 6, 상자 위치 ±5 mm, 노드 `placeAt` 재현 | 90 % 이상 | 노드 그대로 33.6 % → 벌림 37.6 mm 48.3 % → + 처짐 보정 **90.0 %** | [#204](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/204) 본문 | 원인 (옆 퍽 밀기 · 팔 처짐) 을 나눠 고침. 5 cm 정지 기준으로 다시 맞춤 → #297 (2,048 대 95.8 %, 엄격 판정) |
| 꺼내기 → 놓기 (11-11 ~ 12) | 노드 꺼내기 궤적 그대로, 팔레트 나머지 칸 무작위로 채움 | 98 % | 노드 2 mm 틈 2.8 % → 놓기 처짐 보정 95.8 % → 칸 차례 · 벌림 34.6 mm 까지 **98.2 % (2011/2048)**, v5 (10 칸) **98.3 %** | [#209](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/209), [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) | 칸 차례 조건은 계획기 담당에게 넘겨 반영됨 (#260). 노드 실제 벌림 36.4 mm 로 다시 잰 값은 시간 제한으로 끊김 |
| 집기 정책 k7 → k8 → k9 (11-12) | 노드 조건 (퍽 좌표 한 번 잰 값, 관절 변화 상한, 실측 지연 0 ~ 66 ms), 2,048 ~ 8,192 대 | 집기 → 넣기 95 % 이상 | 지연 없이 k8 원기둥 98.4 %. **지연을 제대로 걸면 k8 69.5 / 82.5 %**, k9 75.7 / 86.3 %, 지연 0 ~ 33 ms 가정 91.4 / 93.1 % | [#245](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/245), [#226 정정 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6817601451) | 처음 낸 면 어긋남별 결과에 지연이 빠져 있었음 → 같은 날 정정 · 재측정 댓글. 98 % 는 못 넘김 |
| 엄격 판정 집기 → 넣기 (11-14) | 옆 퍽 · 운반 중 접촉도 실패로 셈, 400 ~ 512 대 | 예전 판정과 비슷 | 예전 판정이 접촉을 세지 않았음을 찾음. 꽉 찬 스토리지 8.0 % (엄격 1.6 %) → 새 경로 90.2 %, 노드 동작 59.0 % → 83.0 % | [#282](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/282) | 남은 실패 대부분은 집기 단계 |
| 세운 직육면체 다시 집기 (11-15) | `chain_pallet_eval.py --spot`, 세워 두는 자리 D | 상자 접촉 줄이기 | 68.0 → 84.0 % (시드 3: 67.8 → 81.0 %), 상자 접촉 5.5 → 3.8 % | [#300](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/300), [#221 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/221) | 실물 미확인 |

**코드 시험**

| 시험 | 결과 | 근거 |
|---|---|---|
| 펌웨어 SIL · 보드 빌드 | #242 SIL 36 개 통과 · 업로드, #284 SIL 3 개 추가 · colcon test 145 개, #316 opencr_hardware 149 개 (10 skipped) · control 85 개 · 새 SIL 4 개 · `opencr_arm_v6.bin` 빌드 | [#242](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/242), [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284), [#316](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/316) |
| control 단위 시험 | `test_startup_stow`, 관측 33 조립 시험 3 개, control 85 개 (#287 · #297 · #312) | 각 PR 본문 |
| STOW 접기 충돌 계산 | 팀 URDF collision (팔 상자 200 개) 으로 접기 · 펴기 경로 최소 틈: 예전 −3.5 mm → 새 STOW 9.2 mm, 실은 퍽 14.8 mm | [#277 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/277) |

### 1.4 리뷰

**사실 그대로의 집계 (GitHub)**

- 다른 팀원 PR 153 건에 리뷰를 남김 (온창범 85 · 이홍주 52 · 한상준 16). 상태는 APPROVED 159 · DISMISSED 7 (한 PR 에 여러 번 포함).
- **본문이 있는 PR 리뷰는 2 건뿐** ([#1](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/1) "승인", [#2](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/2) "승인합니다") 이고, **줄 단위 리뷰 댓글은 0 건**이다. 팀장으로서 병합 승인은 대부분 본문 없이 했다.
- 다른 팀원 PR 161 건을 직접 병합했다.
- 실제 기술 의견은 PR 이 아니라, 그 PR 들이 가리키는 **이슈 스레드**에 모의 · 계산 근거와 함께 남겼다 (이슈 댓글 60 건). 아래는 그중 의견이 다른 팀원 코드로 이어진 것만 고른 것이다.

**이슈 스레드에서 준 의견 → 다른 팀원 PR 반영**

| 대상 (담당) | 문제 · 의견 (요지) | 반영 여부 · 후속 |
|---|---|---|
| 계획기 칸 배정 (이홍주) — [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) | hybrid 칸 배정이 퍽 모양을 안 봄 → 꺼내기 → 놓기 79 % (모양대로면 98 %). [답 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230#issuecomment-6819961887): 같은 열 차례는 자유, 단 '막는 쌍' (원기둥 칸을 꺼낼 때 닫는 축 쪽 직육면체 칸이 비어 있어야 함) 은 지킬 것, 표로 제시 | **반영**: 칸 모양 배정 → 막는 쌍 · 모양 벌점 [#260](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/260) → v5 로 다시 [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261). 담당자 계산 (Q1 + 무작위 100 문제) 에서 막힌 채 꺼내기 0 |
| 시작 박스 회전 범위 (이홍주 [#235](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/235), 온창범 [#247](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/247)) — [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226) | #235 의 전제 "회전으로 잘라서 남는 각은 집기 정책이 받음" 을 모의로 확인: 면 어긋남 20° 까지는 정렬과 비슷, 30° 넘으면 약 20 %p 떨어짐 → 남는 각이 20° 를 넘지 않게 요청. 첫 값에 지연이 빠진 것을 스스로 찾아 [정정](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6817601451) 후 [재측정](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6823669678) | **반영**: #247 이 "남는 각 20° 넘으면 mission_log 에 기록" 으로 이 기준을 씀. 회전으로 못 맞추는 경우는 [#251](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/251) 로 넘김 |
| `box_look` 7 필드 (온창범) — [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6813575193) | 거리 · 각 지정 필드 추가에 대해: 5 필드 동작 · `min_clearance` · 카메라 장착값이 그대로라 팔 쪽 문제 없음, 영점 보정이 들어가면 같은 거리 · 각에서도 카메라 자리가 조금 바뀔 수 있다고 주의 | **반영**: [#232](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/232) 승인 · 병합 |
| 주행 자세 DRIVE (이홍주 · 온창범) — [#219](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219#issuecomment-6812219900) | 실은 퍽과 관절 3 이 겹치는 문제에 대해, 계획기 우회 (`avoid_slots`) 대신 DRIVE 를 ALIGN 과 같은 E2 로 바꾸자고 제안 (G 줄 퍽 17.8 mm · 차체 23.4 mm). 라이다 · D435 시야와 footprint 확인 요청 | **반영**: DRIVE = E2 [#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270) (이홍주). `avoid_slots` 는 필요 없게 됨. 실물 확인은 #266 으로 넘김 |
| 누운 직육면체 세우기 정책 (온창범) — [#192](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/192), [#193](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/193) | 다시 잡는 피치 45° 고정, 잡는 점 = 퍽 중심 + 15 mm, 배포 규칙 "`execute_skill` 이름 = `policies.yaml` 키, 외부 데이터 없는 한 파일 ONNX" 를 명세로 줌. 실물 시험에서 젯슨에 옛 `r2e` 가 올라간 것을 찾음 | **반영**: [#215](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/215) · [#217](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/217) (r2g2), 11-16 실물 로그에서 `lieupright_r2g2.onnx` 로드 확인 |
| 손목 (J5) 으로 면 맞추기 제안 (온창범) — [#251](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/251#issuecomment-6876839570) | 관측 · 행동 · 노드 IK 가 모두 바뀌어 재학습이 필요하고, 실물 성공률은 지연 · 처짐에 더 좌우됨 → k9 실물 시험 뒤로 **보류** 판단 | **의도적으로 반영 안 함** (보류로 닫음, 다시 열 조건 명시) |
| 집기 넘어뜨림 · 쥐는 깊이 제안 (온창범) — [#141](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/141) | fit1 대신 k7 (관측 33) 로 정함, 미션 배치 2,048 대 집기 원기둥 98.4 % · 직육면체 97.9 % (지연 없는 조건) | **부분 반영**: 집기 전 퍽 밀림 벌점은 이홍주 [#206](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/206) 에 들어감 |
| 펌웨어 요청 (온창범) — [#259](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/259), [#277](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/277) | 모터별 HW 오류 · 온도 진단, HOLD 가 막힌 목표를 계속 유지하는 문제, STOW 접기 충돌 | **반영 (내가 구현)**: [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284), [#286](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/286) |

**받은 리뷰와 반영 (참고)**

| 내 PR | 리뷰어 · 지적 | 반영 여부 |
|---|---|---|
| [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94) | 온창범: `registered` 모드에서 측량이 실패해도 지난 실행 결과로 계획될 수 있음, `survey.poses` 가 계획기 파일의 복사본이라 어긋날 수 있음 | 반영 (커밋 a573322: 측량 시작 때 지난 등록표 삭제, `approach_poses.yaml` 직접 읽기). 같은 PR 의 이홍주 팔레트 skill 리뷰 6 건 (#75) 은 PR 에 남은 채 병합 — 후속은 #75 담당 이슈로 처리 |
| [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) | 이홍주: 옆 퍽 없음 (0, 0) 을 학습에서 봤는지, 행동 → IK 자세가 학습과 같은지, 관절 속도 상한 2.0 rad/s, 로드 실패가 조용함 | 관절 속도 상한은 [#231](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/231) 에서 정책별 4.0 rad/s 로 반영. 옆 퍽 없음 · IK 자세 일치 근거는 PR 에 답을 남기지 못함 (한계) |
| [#189](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/189) | 한상준 (#144 에서): 목표 자세 STOW → DRIVE, 순서 3 → 2 → 1 → 4 → 5 · 6, 0.1 rad/s | 반영 |
| [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) | 온창범: CPU 0.68 % 변화는 의미 없으니 반려 의견 | 반영하지 못하고 다른 팀원 (한상준) 이 병합. 의견을 놓고 합의한 기록은 없음 |

### 1.5 설명 (전체 흐름 이해)

전체 구조는 [architecture.md](architecture.md) 에 정리한다 (작성 중). 내가 이해하고 쓰는 흐름은 아래와 같다.

```
/mission/start ─▶ mission_node ──(/mission_planner/plan)──▶ mission_planner (칸 배정 · 싣기/내리기 차례)
                     │
                     ├─▶ drive_node : /drive/navigate (Nav2 navigate_to_pose · spin) · approach COARSE/FINE (TFmini 0.13 m)
                     ├─▶ manipulator_node : /manipulator/pose · pick · execute_skill
                     │        ├─ 각본 스킬 (pallet_load / unload, place_<L>, target_look, box_look)
                     │        └─ 정책 스킬 (robot_policy: policies.yaml 의 ONNX, 30 Hz)
                     │              ▼
                     │        ros2_control arm_controller ─▶ opencr_hardware ─▶ OpenCR 팔 펌웨어 (STATE/CMD, HOLD/E-stop)
                     └─◀ perception (/perception/pucks · place_box · 태그) · localization (/localization/status)
```

내가 맡지 않은 코드 · 설정 · 결과를 읽고 판단한 근거:

| 근거 | 내가 맡지 않은 부분 | 무엇을 이해 · 판단했나 |
|---|---|---|
| [#194](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/194) 정차 간격 (drive · planner · policy) | 주행 `drive.yaml`, 계획기 `FINE_GAP`, TopPick 학습 배치 | "코스트맵이 12 cm" 라는 질문을 추적해 원인이 Nav2 `inflation_radius` 가 아니라 TFmini 최소 거리로 정한 `min_target_distance 0.13` · `stop_margin 0.03` · 계획기 `0.1272` 임을 밝힘. 5 ~ 6 cm 로 줄이면 바뀌는 곳 (TFmini · obstacle_stop · 계획기 · 학습) 을 비교해 "정차는 그대로, 팔 정책을 0.1272 로 다시 학습" 으로 결론 (#195) |
| [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) · [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226) 계획기 | `mission_planner` 칸 배정 · 회전 범위 | 계획기 hybrid 운영 (#224) 을 읽고 모양 무시 문제를 찾음, 계획기 담당이 쓸 수 있는 조건 (막는 쌍 표, 남는 각 20°) 으로 바꿔 전달 |
| [#128](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/128) · [#143](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/143) 인지 | perception 팔 카메라 | 칸에 못 앉는 원인이 손 안 퍽 위치 오차임을 모의로 보이고 (오차 0 mm 면 세운 직육면체 30/1) 인지 쪽에 ~1 mm 측정 가능성을 물음. 실물에서 퍽이 안 보이는 원인을 `arm_camera_info_url` 이 비어 K = 0 camera_info 가 나가 perception 이 거부한 것으로 찾음 → 온창범 #146 으로 해결 |
| [#114](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/114) Nav2 | 라이다 장착 · Nav2 설정 | URDF 로 스캔 면 0.082 m vs 상자 윗면 0.070 m (여유 12 mm) 를 계산해 유령 점 위험을 알리고, 팔이 닿는 거리로 정지 거리 · footprint 값을 제안 |
| 11-09 담당 노드 검토 → [#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144) | 다른 노드와의 연결 (action · 메시지 필드 · yaml 키) | 연결은 문제없고, NaN 이 `withinLimits` 를 통과해 팔 명령까지 가는 점, E-stop 뒤 스트림이 최대 0.45 rad 앞서 남아 REARM 때 튈 수 있는 점을 찾음 → #189 · #244 |
| [#265](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265) CPU (bringup) | 브링업 컨테이너 구성 | 한 프로세스라 노드별 CPU 를 못 가르는 문제를 `split_control` 로 풀고 (#299), 젯슨 측정으로 manipulator `/tf` 구독이 큰 몫임을 찾아 #312 로 줄임 |
| 11-16 실물 로그 해석 | mission · planner · Nav2 브링업 | 같은 로그에서 계획기 서버 없음 · Nav2 lifecycle 대기로 처음 시작이 거절된 것과, 팔 시작 정리 거절로 observe 가 실패한 것을 구분 (1.3 표) |

### 1.6 한계 (솔직하게)

1. **11-16 실물 미션에서 팔이 한 번도 동작하지 못했다.** 첫 단계 observe 의 `execute_skill` 이 `goal rejected` 로 3 번 거절돼 미션이 FAILED. 원인은 팔이 STOW 에서 0.23 rad 어긋나 켜질 때 정리가 시작되지 않은 것이고, 그 배경은 같은 날 STOW 정의를 여러 번 바꾼 것 (#311 → #314 → #316) 이다.
2. **실물 집기 → 넣기 · 주사위 놓기를 처음부터 끝까지 성공시킨 기록이 없다.** 실물 강화학습 집기는 11-12 에 2 번 실패했고, 그 뒤 k9 · `skill.horizon 1.0` 실물 시험은 하지 못했다. 실물 주사위 배치는 11-13 에 pick · place 한 번 성공이 전부다.
3. 성공률 수치 (90 ~ 98 %) 는 대부분 **Isaac 모의** 다. 실측 지연을 빼고 잰 평가를 한 번 올렸다가 정정했다 (#226). 지연을 넣으면 집기는 아직 85 % 안팎이다.
4. **관절 1 모터 180° 문제** 를 펌웨어 v6 (#316) 로 정리했지만 업로드 · 실물 확인을 하지 못했다. 영점 · 링크 길이 재보정 (#292 · #295) 도구는 커밋하지 않고 stash 에 있다.
5. 계획기 정지 거리 (12.7 cm) 와 노드 놓기 보정 기준 (5 cm, #297) 이 아직 맞지 않는다.
6. PR 리뷰는 대부분 본문 없는 승인이었다. 의견은 이슈에 남겼지만 PR 화면에서 리뷰 근거가 보이지 않는다. 받은 리뷰 중 #214 의 일부 질문과 #312 의 반려 의견에는 답을 남기지 못했다.
7. 시험 영상 · 학습 기록 일부가 저장소 밖 (IsaacLab 작업본, 임시 릴리스 `rl-20261110`) 에 있어 다른 사람이 바로 열어 볼 수 없다.

---

## 2. 이홍주 (kanichong)

> 저장소 기록 (커밋 · PR · 이슈) 으로 정리함 — 본인 확인 필요
> git 작성자 `kanichong <kanichong1@gmail.com>` (dev 비병합 커밋 79 개), GitHub `kanichong`.

### 2.1 맡은 일

| 영역 | 내용 | 주요 파일 |
|---|---|---|
| 계획기 | `mission_planner`: 할당 (Y 모양 · ID 전부 나열), 일반화 TSP (Held-Karp), 운영 방식 `per_location` · `single_load` · `hybrid`, 팔레트 칸 배정 (모양 · 막는 쌍), 접근 자세표 · 지도 생성, 회전 범위, 부분 시험 `stop_after`, 제출물 `mission_submit` | `workspace/ros2_ws/src/mission_planner/` |
| 미션 (plan 모드) | `mission_node` plan 모드 단계 (`observe` · `plan` · `goto` · `pick_target` · `verify` · `measure_box` · `approach_box`), 복귀 기록 · 채점 스크립트, 시작 전 준비 확인 · 퍽 건너뛰기 · HOME 복귀, 떨어뜨림 다시 집기 | `mission/src/mission_node.cpp`, `mission/include/mission/give_up.hpp` |
| 주행 · 위치 추정 연결 | drive 제자리 회전 skill (`turn_*`), 박스 앞 정차 (`box_park`), 위치 확인 (`loc_gate`), EKF 회전 중심 보정 (`odom_icr`), twist_mux 위치 잃음 잠금, Nav2 통합 실행 · BT 복구 · 한 프로세스 실행 (`nav2_composed`), Gazebo 미션 시험 스크립트 | `control/src/drive_node.cpp`, `bringup/`, `navigation/`, `mission_planner/test/gazebo_*.sh` |
| 시뮬레이션 모델 | B안 URDF · USD (팔레트 v3 → v5, 증속벨트 → v7, LDS-03 브라켓 v2, joint1 브라켓 v3, 퍽 충돌 형상 · 실측 무게), Isaac 브리지, 실기 ↔ Isaac 모델 비교 시험 | `workspace/urdf_and_usd/`, `tools/isaac_sim/`, `opencr_hardware/test/test_model_compare.py` |
| 팔 강화학습 · Isaac 평가 | TopPick 학습 환경 (TP2 ~ TP17, 물리 무작위화, 보상 항목 기록), 팔레트 놓기 · 꺼내기 궤적 확인, 고정 동작 집기 평가, 이웃 퍽 · 장소 상자 놓기 평가 | `workspace/urdf_and_usd/policy/manipulator/` |
| 문서 · 제출물 | 초기 구조 (#1), 중간 제출물, 채점 대응표, 도전 B, TF 규약 | `submission/`, `challenge/B/`, `workspace/docs/frames_tf.md` |

### 2.2 구현 (작성 · 병합된 PR)

병합된 PR 55 건, 닫힌 PR 0 건.

**계획기 (mission_planner)**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#61](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/61) (11-02) | 계획기 새로 씀: 경기장 SDF 에서 지도 2 장 (`map_nav` · `map_amcl`), 접근 자세표, 할당 (Y 모양 · ID 전부 나열), 적재 방식 2 가지, 일반화 TSP (Held-Karp) + A* 경로 비용 | pytest 13 개 (도전 B 형식 할당 6 건 포함). AMCL 모의 위치 오차 평균 16 mm (nav2 기본값 32 mm) |
| [#79](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/79) (11-03) | 팔레트 12 칸 모두 45° 집기 · 놓기, 열별 속도, 상자 퍽 ±17° 제자리 회전, 접근 간격 조정 | Isaac 놓기 · 꺼내기 24/24, 놓기 173.9 → 157.8 s (CSV S-06) |
| [#138](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/138) (11-06) | 대기 자리 거리 (standoff) 를 0.01 m 단위로 골라 북쪽 통로에서 갇히지 않게 | 원인 = 통로 0.551 m < 차체 회전 지름 0.563 m. Gazebo 미션 105 단계 DONE ([#135](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/135)) |
| [#224](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/224) · [#227](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/227) (11-12) | 운영 방식 `hybrid` 기본값 (싣다가 한 장소 퍽이 다 모이면 내림), 같은 박스는 세운 퍽 먼저 | 회의 결정 ([#197 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/197#issuecomment-6811873397)) 반영. 할당 전부 약 0.01 s, 무작위 20 문제 위반 0 |
| [#235](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/235) · [#237](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/237) (11-12) | 시작 박스 회전 범위 `turn_range_deg`, 싣지 않을 칸 `avoid_slots` | 막히지 않은 박스 8 곳 모두 ±45° 회전 가능 ([#226 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6816084117)). `avoid_slots` 는 DRIVE = E2 ([#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270)) 뒤 쓸 필요 없어짐 ([#219 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219#issuecomment-6855678974)) |
| [#239](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/239) · [#260](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/260) (11-12 ~ 13) | 칸 배정에 퍽 모양 반영 (`slots.assign`), 꺼내기 막는 쌍 · 모양 벌점 28 m | 무작위 80 문제 hybrid 막힌 꺼내기 33 → 3. 팔 담당 모의 ([#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230)) 를 계획기 조건으로 옮김 |
| [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261) (11-13) | 팔레트 v5 (10 칸) · 막는 쌍 v5 · 벌점 분리 (모델 · control 궤적 · 계획기 함께) | 무작위 100 문제 막힌 꺼내기 0, 3 열 사용 0.85 → 0.57 칸. 시험 control 78 · opencr_hardware 104 · 계획기 54 통과 |
| [#263](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/263) · [#272](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/272) (11-13) | 시작 박스 · 비스듬히 서기 기하 값을 자세표 `geom` 과 `robot_geometry.yaml` 한 곳에서 읽기 ([#262](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/262)) | 숫자가 따로 있어 FINE 거리가 바뀌면 조준점이 1 cm 씩 어긋나던 위험 제거. 옛 자세표는 노드가 거부 |
| [#269](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/269) (11-13) | 외곽 판정에서 장애물 외곽을 한 번만 계산, `odom_icr` 를 C++ 로 | PC World 33 → 3.2 s. Jetson 실측 (온창범): `ready` 시작 뒤 22.1 s, odom_icr CPU 28 → 6.9 ([#265 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265#issuecomment-6869612498)) |
| [#211](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/211) · [#167](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/167) (11-09 ~ 11) | 부분 실물 시험 `stop_after`, 제출물 생성 `mission_submit`, 도전 B 비교 | 도전 B: 탐색 6/6, 순차 선택 3/6 ([`challenge/B/`](../../challenge/B/)) |

**미션 (mission_node)**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#67](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/67) (11-02) | plan 모드: observe (신호 · 바코드) → `/mission_planner/plan` → 단계 문장 실행, `mission_log.jsonl`, 가짜 로봇 `mock_robot.py` | mock Q1: 85 단계 DONE, skill 거절 → 재시도 뒤 FAILED, 미등록 바코드 → FAILED (CSV S-12) |
| [#104](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/104) (11-04) | 팔레트 꺼내기를 놓기 궤적 역순으로 (Isaac 확인 순서), 놓기 전 장소 상자 측정 `measure_box`, FINE 자리 그대로 집기 · 놓기 (안 A, `FINE_GAP` 0.1272) | 상자 2 cm · 5° 밀림 mock DONE (CSV S-13). YAML 이 `N` 을 불리언으로 읽어 plan 모드가 안 뜨던 버그도 고침 |
| [#213](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/213) · [#222](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/222) (11-11) | 복귀 판정 `return_check`, 채점 스크립트 `mission_score`, [`submission/scoring_map.md`](../../submission/scoring_map.md), 재검증에서 한 장소 같은 ID 합침 | [#222](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/222) 는 한상준 리뷰 ([#172 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/172#issuecomment-6757717891)) 반영 |
| [#289](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/289) (11-14) | 시작 전 준비 확인 표 · `/mission/ready`, 재시도 소진 시 그 퍽만 건너뛰기, 그 밖 실패는 HOME 복귀 뒤 FAILED, 경기 명령 기본값 | mock: 한 퍽 거절 → `DONE (skipped: pick 1_1)`, 놓기 거절 → HOME 복귀 FAILED. 11-16 실물 로그에서 준비 확인이 빠진 항목 (계획기 서버 · Nav2 · 위치 추정) 으로 시작을 거절한 판이 남음 ([`results/README.md`](../results/README.md) M-13 · M-17) |
| [#315](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/315) (11-16) | 들기 뒤 · 운반 중 떨어뜨림도 `target_look` 부터 다시 집기 · 그 퍽만 건너뛰기, 실패 이유 문구가 사라지던 버그 | mock 3 경우 DONE, mission 49 · control 85 통과. 실물 미확인 |

**주행 · 위치 추정 · Nav2 · bringup**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#11](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/11) · [#13](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/13) (10-27) | 주행 skill 장애물 처리, 증속벨트 기어비 · `drive` 인자 | 기어비 루프백 시험 |
| [#99](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/99) · [#105](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/105) (11-04) | 제자리 회전 skill `turn_p/m<deg>` ([#77](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/77)), `target_look` (45° 집기 전 내려다보고 재기, [#82](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/82)), EKF 회전 중심 옆 이동 보정 `odom_icr` ([#92](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/92)), twist_mux 위치 잃음 잠금 ([#93](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/93)), D435 노출 고정 | 온창범 요청 이슈 [#82](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/82) · [#92](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/92) · [#93](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/93) · [#89](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/89) 처리 |
| [#117](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/117) · [#136](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/136) (11-05 ~ 06) | 통합 launch 에 plan 미션 · 위치 추정 · 계획기 · Nav2, drive `loc_gate` ([#121](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/121)), `xy_goal_tolerance` 0.05 통일 | Gazebo 미션 연동 시험 (`gazebo_mission.sh`) 시작 |
| [#142](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/142) (11-08) | Nav2 BT 복사본: 플래너 복구 조건에 GOAL_OCCUPIED 206 등 추가 | 목표 칸이 잠깐 막히면 1 s 만에 중단하던 것. Q1 105 단계 DONE · 재시도 0 (바뀐 복구가 실제로 도는 장면은 못 봄) |
| [#169](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/169) (11-09) | 박스 앞 FINE 정렬 뒤 낮춘 비상 정지 거리 유지 · navigate 전 후진 (`box_park.hpp`), float32 0.13 거절 수정 ([#168](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/168)) | 한상준 이슈 [#157](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157) 해결. Gazebo `gazebo_box_park.sh` 통과, 한상준이 다시 돌려 확인 ([#157 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157#issuecomment-6761667521)) |
| [#200](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/200) (11-11) | 빈 장소 상자 앞은 퍽 없이 TFmini 로 접근 (`approach_box`, `AlignBase.box_only`) | 한상준 이슈 [#160](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/160) 해결 |
| [#152](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/152) · [#208](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/208) · [#268](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/268) (11-09 ~ 13) | Gazebo `loc_monitor` 지도 맞춤, 다시 넣는 자세 이동 보정, Gazebo 도 EKF odom | 다시 넣은 자세 오차 5 ~ 8 cm → 0.8 ~ 2.8 cm. Gazebo 바퀴 odom 이 회전을 21 ~ 24 % 크게 셈을 찾음 ([#151 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/151#issuecomment-6850403088)) |
| [#283](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/283) (11-14) | 컨트롤러 spawner 하나로 · 시간 초과 늘림, `bringup_repeat.sh`, `/joint_states` 뒤 무거운 노드 띄우기 | 온창범 이슈 [#275](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275): Jetson 30 번 중 25 번 실패 → 온창범 실물 확인 4/4 · 4 판, 무거운 구성 8 판 모두 첫 bringup 4/4 ([#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6893200035), [#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6896389227)) |
| [#307](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/307) (11-15) | Nav2 서버 6 개를 한 프로세스 `nav2_composed` 로, 팔 대기 자세 범위 넓힘 ([#305](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/305)) | 온창범 이슈 [#276](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/276). PC Gazebo 5/5. Jetson 무거운 구성 확인은 PR 에 "필요" 로 남음 |

**팔 (control) 쪽 기여**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#106](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/106) (11-04) | 팔레트 놓기 · 꺼내기 처짐 보정 스트리밍 (`sag_comp.hpp`) | mock 팔 bias 0.055 → 0.009 rad |
| [#170](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/170) (11-09) | `toppick_tilt` 를 위에서 내려오는 접근 (`approach_mode top`) | Isaac 실제 배치 top 84/88 vs 예전 축 접근 48/88 (측정 오차 0) |
| [#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270) (11-13) | 주행 자세 DRIVE = ALIGN E2, 켤 때 순서 J3 → J2 → J4 → J1, TopPick 칸 퍽 틈 벌점 | 팀 URDF 충돌 상자로 v5 칸 퍽과 틈 계산 (옛 DRIVE −9 mm 겹침 → 새 값 퍽 15 · 차체 23 mm). 이창엽 제안 ([#219](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219)) 반영 |

**시뮬레이션 모델 · Isaac**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#17](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/17) · [#62](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/62) (10-28 ~ 11-02) | TF · 카메라 프레임 규약 [`frames_tf.md`](../../workspace/docs/frames_tf.md), Isaac Sim 브리지 (관절 · 카메라 3 · 라이다 2 발행), `sensors.launch.py` | 온창범 요청 [#40](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/40) · [#41](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/41) · [#43](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/43) 처리 |
| [#50](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/50) · [#60](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/60) (11-02) | B안 차체 · 팔레트 v3 URDF · USD, 팔레트 충돌 형상 수정, 놓기 궤적 | 상자 퍽이 칸 바닥 위 13 ~ 20 mm 에 걸리던 문제 해결, 놓기 24/24 |
| [#149](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/149) · [#156](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/156) (11-09) | 모의 팔 카메라를 실측 보정값 (fx 865.4) 으로, 실기 ↔ Isaac URDF 비교 시험 (FK 프레임 11 개, self-test 7) | 모델 차이를 시험이 잡음 (이후 [#264](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/264) 라이다 24.3 mm 차이 발견) |
| [#264](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/264) · [#267](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/267) · [#285](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/285) · [#288](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/288) (11-13 ~ 14) | LDS-03 브라켓 v2, 퍽 실측 무게 13 · 17 g · 팔 영점 보정 도구 `arm_calib`, 증속벨트 v7 (1:2.0), joint1 브라켓 v3 | 무게 반영 뒤 고정 동작 집기 σ2 137 → 168/192. 벨트 v7 모의 제자리 회전 69.2° → 47.4° (실물 확인 필요로 적음) |

**팔 강화학습 · Isaac 평가**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#70](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/70) · [#99](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/99) · [#106](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/106) (11-03 ~ 04) | TopPick 보상 항목별 · 종료 원인 기록, 다시 학습 설정 (배치 A · B · C), 물리 무작위화 | 기존 정책 무작위화 끔 23 % · 켬 17 % → 다시 학습 필요 (CSV S-07), TP2 좌표 고정 0/46 (S-08) |
| [#170](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/170) · [#196](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/196) · [#206](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/206) (11-09 ~ 11) | TP8 · TP9 · TP10 등록, 이어 평가에 다시 쥐기 · 고정 동작 집기 | 고정 동작 집기 172/192 (σ 0) vs TP9 39 · TP8 23 → 미션은 고정 동작 집기로 |
| [#308](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/308) (11-15) | TP14 ~ TP17 기록, 벌림 50 mm 기본값 | TP14 · TP15 가 61.6 mm 로 잘못 학습된 것을 찾아 버림. TP17 202/384 로 개선 없음 → **RL 집기 학습 멈춤** |
| [#313](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/313) · [#320](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/320) (11-15 ~ 17) | 팔레트 이웃 퍽 시험 `--neighbors`, 장소 상자 놓기 평가 `--place` | 아래 2.3 |

그 밖의 병합 PR: [#1](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/1) (초기 SYSTEM base 구조, 패키지 뼈대 · 펌웨어 프로토콜 확인 · colcon test 53 개).

### 2.3 검증 (직접 돌린 시험)

원자료: PR 본문 · 이슈 댓글, Isaac 결과 JSON (`workspace/urdf_and_usd/policy/manipulator/` 의 `policies/evals/`, `정책_학습기록.md`), [`project2/results/test-results.csv`](../results/test-results.csv).

**모의 (Gazebo · mock)**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| Gazebo 미션 전체 (Q1) | Nav2 + AMCL + 진짜 drive_node, 팔 · 인식 mock | DONE | 처음 B3:N → L3 에서 갇혀 FAILED → standoff 수정 뒤 105 단계 DONE, goto 18 번 xy 중앙 48 ~ 49 mm, yaw 최대 5.5° | [#135](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/135), [#138](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/138), [#142](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/142), CSV S-14 | 팔 · 인식이 mock 이라 `/drive/align` 실패 ([#160](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/160)) 는 못 잡음 (한상준 지적) |
| Gazebo 위치 감시 켬 (LOC=1) | `loc_monitor` + twist_mux 잠금 + `loc_gate` | DONE · FAILED 0 | Q1 107 단계 DONE (522 s), LOST:residual 17 번 모두 1.3 ~ 1.5 s 복구 | [#151 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/151#issuecomment-6818481818) | 원인이던 Gazebo odom 회전 과대는 [#268](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/268) 로 따로 고침 |
| 박스 앞 정차 | `gazebo_box_park.sh`: FINE 0.13 → turn_p17 → turn_0 → navigate | 종료 코드 0 | FINE 6.2 s, turn 0.93° · −0.66°, navigate 18.3 s | [#169](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/169) | 한상준이 같은 스크립트로 재현 ([#157 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157#issuecomment-6761667521)) |
| 계획기 무작위 문제 | hybrid · per_location, 무작위 80 · 100 문제 | 막힌 꺼내기 0 | v3: 33 → 3 (80 문제), v5: 0 (Q1 + 100 문제) | [#260](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/260), [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261), [#230 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230#issuecomment-6849002125) | 모의 칸 쌍 시험 근거. 실물 시험 (#266 4 번) 전 |

**모의 (Isaac)**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 팔레트 놓기 · 꺼내기 | v3 12 칸 · v5 10 칸, 원기둥 · 직육면체 | 전부 성공 | v3 24/24 (놓기 157.8 s · 꺼내기 137.8 s), v5 24/24, 옆 칸 퍽 (pairs) 56/56 | [#79](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/79), [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261), [#313](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/313), CSV S-06 · S-30 | 옆 퍽이 밀린 쌍 1_0 ← 1_1 · 1_2 ← 1_3 확인, 1_1 ← 1_0 은 재현 안 됨 → 실물 (#266) 로 넘김 |
| 퍽 자리 측정 오차 σ 별 집기 | 고정 동작 집기, 192 대 × 시드 2 | — | σ0 98.3 % · σ2 96.1 % · σ3 93.4 % · σ4 84.0 % · σ6 66.3 %. σ3 mm 넘으면 운반 미끄럼 급증 | [#313](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/313), [#266 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/266#issuecomment-6898310022) | 이 결과로 [#315](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/315) (운반 중 떨어뜨림 다시 집기) 를 넣음. 팔 영점 · 링크 길이 오차는 안 넣음 |
| 장소 상자 놓기 | `--place`, 180 경우 × 시드 2 = 360 회, 5 cm 정차 | 기본 조건 전부 성공 | 기본 360/360 (p50 3.7 mm), σ 2 mm · 2° 353/360, 12.7 cm 정차 360/360, **이웃 8 칸 54/180** | [#320](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/320), [#266 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/266#issuecomment-6933544497), CSV S-31 | 이웃이 다 찬 경우 실패 → 먼 칸 먼저 놓는 순서가 꼭 필요하다는 결론 |
| TopPick 강화학습 | TP8 ~ TP17, 이어 평가 384 대 | 고정 동작보다 좋음 | TP17 최고 202/384 · 집기 67 ~ 72 % 그대로, 고정 동작 332 ~ 347/384 | [#308](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/308), CSV S-09 | 개선 없어 RL 집기 학습 멈춤 |

**실물**

| 시험 | 조건 | 결과 | 원자료 | 한계 |
|---|---|---|---|---|
| 라이다 브라켓 · 퍽 무게 · 대기 자세 거부 | 실물 브라켓 확인 (11-13), 퍽 무게 저울 (원기둥 13 · 직육면체 17 g), 11-15 실물 시작 거부 기록 | xacro · Isaac 모델에 반영, 대기 범위 J1 · J5 넓힘 (18:43 · 19:08 판은 통과, 18:53 J1 96.3° 는 여전히 밖) | [#264](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/264), [#267](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/267), [#307](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/307) | 직접 돌린 실물 주행 · 미션 시험 기록은 저장소에 없음 |

### 2.4 리뷰

**사실 그대로의 집계 (GitHub, 2026-11-17)**

- 다른 팀원 PR 에 남긴 PR 리뷰: **3 PR** — [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) (이창엽, 본문 있는 COMMENTED + 같은 본문 APPROVED), [#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284) · [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) (이창엽, "LGTM." 승인). 줄 단위 리뷰 댓글 0 건 (팀 전체 0 건).
- 다른 팀원 PR 대화 댓글 1 건: [#94 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94#issuecomment-6620155997).
- 다른 팀원 PR 병합 1 건 ([#284](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/284)).
- 이슈 댓글 89 건 (이슈 87 · PR 2). 대부분 다른 팀원이 올린 요청 · 문제 이슈에 원인 분석과 처리 결과를 남긴 것이다.

**PR 에 준 의견**

| 대상 PR (작성자) | 문제 · 의견 (요지) | 반영 여부 · 후속 |
|---|---|---|
| [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94) (이창엽) — [#94 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94#issuecomment-6620155997) | 팔레트 skill 6 가지: ① 꺼내기가 Isaac 에서 확인한 순서 (놓기 궤적 역순) 와 다름 ② 3 열 꺼내기 속도 0.4 vs 표 0.8 ③ 칸 안에서 끝까지 열면 칸 벽에 닿을 수 있음 (Isaac 은 37.6 mm) ④ 놓은 뒤 0.3 s 머물기 ⑤ STOW J1 91° 회전 ⑥ 궤적 사본 일치 시험 | ①~④ 는 **본인이 [#104](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/104) 로 반영** (꺼내기 역순, 쥔 구간만 0.4, `pallet.release_position` 0.0798 rad, `place_dwell` 0.3 s). ⑤ · ⑥ 은 반영 기록 없음 |
| [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) (이창엽) | ① 옆 퍽 없음 (0, 0) 을 학습에서 본 적이 있는지 (가장 중요) ② 행동 → IK 자세가 학습 환경과 같은지 ③ 정책 로드 실패가 조용함 ④ 관절 속도 상한 2.0 rad/s ⑤ 로그 문구 | ④ 는 [#231](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/231) 에서 정책별 4.0 rad/s 로 반영 (1.4 받은 리뷰). ① · ② 에는 PR 에 답이 없음 |

**이슈 스레드에서 다른 팀원 요청 · 문제를 받아 처리**

| 이슈 (올린 사람) | 의견 · 분석 (요지) | 반영 · 후속 |
|---|---|---|
| [#230](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230) (이창엽) | 칸 배정이 모양을 안 봐 Q1 도 12 개 중 8 개만 맞음 → 모양 · 막는 쌍을 계획기에 넣음. 팔레트 v3 기준이던 값을 v5 로 스스로 [정정](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/230#issuecomment-6849002125) | [#239](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/239) → [#260](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/260) → [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261) |
| [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226) · [#193](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/193) · [#197](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/197) (온창범) | 박스 접근별 회전 가능 범위 계산 ([#226 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6816084117)), 집기 순서는 mission 이 아닌 계획기에 넣는 것이 맞다고 판단 ([#193 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/193#issuecomment-6798225562)), 운영 방식은 회의 결정대로 hybrid ([#197 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/197#issuecomment-6811873397)) | [#235](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/235), [#227](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/227), [#224](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/224) |
| [#219](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219) · [#220](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/220) (온창범) | 팔꿈치-퍽 겹침에 계획기 우회책 `avoid_slots` 를 먼저 넣고, DRIVE = E2 뒤 필요 없음을 정리 ([#219 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219#issuecomment-6855678974)). Isaac loaded 판 퍽에 충돌 형상 추가 ([#220 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/220#issuecomment-6812960987)) | [#237](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/237), [#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270), [#227](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/227) |
| [#157](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157) · [#160](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/160) (한상준) | 박스 앞 정차 잠금 원인 · 안 A 수정 ([#157 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157#issuecomment-6740772654)), 빈 상자 접근을 `approach_box` 로 ([#160 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/160#issuecomment-6783179326)). `AlignBase` 변경이라 Controller 승인 요청 | [#169](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/169), [#200](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/200) |
| [#275](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275) · [#276](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/276) · [#265](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265) · [#305](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/305) (온창범) | spawner 파일 잠금 다툼 · 서비스 찾기 지연 원인을 코드로 찾음 ([#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6874209312)), PC 에서 재현 ([#275 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275#issuecomment-6875580386)), bt_navigator 활성화 실패 원인 ([#276 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/276#issuecomment-6896179878)), 계획기 시작 계산 · odom_icr 줄임 ([#265 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265#issuecomment-6852688133)) | [#283](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/283), [#307](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/307), [#269](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/269). 실물 확인은 온창범이 함 |
| [#145](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/145) · [#15](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/15) · [#112](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/112) (온창범 · 본인) | 모의 세 곳 팔 카메라 화각 ([#145 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/145#issuecomment-6733107255)), 바퀴 한계를 11.1 V 기준으로 통일 ([#15 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/15#issuecomment-6734881769)), 실기 ↔ Isaac 비교 시험 ([#112 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/112#issuecomment-6734561911)) | [#149](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/149), [#156](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/156) |
| [#278](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/278) (온창범) | 팔 카메라 장착 · 손눈 보정은 카메라를 쓰는 인지 쪽이 맡는 것이 맞다고 의견 ([#278 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/278#issuecomment-6872592831)) | 온창범이 맡아 [#296](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/296) |

### 2.5 설명 (전체 흐름 이해)

| 근거 | 내가 맡지 않은 부분 | 무엇을 이해 · 판단했나 |
|---|---|---|
| 중간 제출물 [`nodes_topics_states.md`](../../submission/midterm/nodes_topics_states.md) · [`test_results.md`](../../submission/midterm/test_results.md) · [`open_issues.md`](../../submission/midterm/open_issues.md) ([#167](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/167)) | 전 패키지 | 노드 구성 · 미션 경로 토픽 그래프 · 상태도 3 개 (실행 상태 · plan 모드 단계 · `loc_monitor`), 전 패키지 단위 시험을 다시 빌드해 실패 0 (perception 199 · control 49 · opencr_hardware 107 · mission_planner 19), 열린 이슈 21 건을 경기 영향 순으로 정리. 온창범 요청 ([#187](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/187)) 에 맞춰 기록 표를 이동 전후 사진 · 미션 폴더로 고침 ([#187 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/187#issuecomment-6768294131)) |
| [`scoring_map.md`](../../submission/scoring_map.md) ([#213](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/213)) | 채점 · 기록 전체 | 채점표 항목마다 근거 기록 · 만드는 노드 · 경기 뒤 점검 순서를 대응 |
| [#135](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/135) · [#142](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/142) Nav2 | 주행 (한상준 담당 Nav2 설정) | 북쪽 통로 (0.551 m) 가 차체 회전 지름 (0.563 m) 보다 좁고 대기 자리가 통로 가운데보다 88 mm 남쪽이라 MPPI 가 갇히는 것, 기본 BT 가 GOAL_OCCUPIED 206 을 복구 대상으로 안 보는 것을 Nav2 코드로 확인 |
| [#283](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/283) · [#307](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/307) bringup · Nav2 | 브링업 · Nav2 시작 | controller_manager spawner 가 파일 잠금 하나를 같이 쓰는 구조, bt_navigator 가 활성화 때 서버를 못 찾으면 스스로 포기하는 구조를 설치된 Nav2 1.5.0 코드에서 찾음 |
| [#168](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/168) · [#169](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/169) drive | drive 정렬 (`AlignBase`) | `float32` 0.13 이 0.1299999952 로 와서 `< 0.13` 비교에 거절되던 원인 |
| [#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270) 팔 | 팔 자세 (이창엽 담당) | 팀 URDF 충돌 상자로 DRIVE · 접기 순서별 퍽 · 차체 틈을 계산해 자세와 순서를 함께 바꿈 |

### 2.6 한계 (저장소 기록으로 본 것)

1. **실물 시험 기록이 적다.** 검증 대부분이 Gazebo · Isaac · mock 이다. 계획기 · plan 모드 미션을 실물로 끝까지 돌린 기록이 없고, 11-16 실물 미션은 observe 단계에서 FAILED 였다 ([`results/README.md`](../results/README.md) M-11).
2. Jetson 문제를 PC 에서 재현하지 못한 채 병합한 PR 이 있다. [#283](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/283) 은 이후 온창범이 실물로 확인했지만, [#307](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/307) 의 Jetson 무거운 구성 확인은 기록이 없다. 11-16 로그에도 Nav2 lifecycle 미완으로 시작이 거절된 판이 있다 (M-13, 원인은 가리지 못함).
3. 강화학습 집기 (TopPick) 는 TP17 까지 고정 동작 집기보다 나아지지 않아 멈췄다 ([#308](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/308)). 고정 동작 집기 · 장소 상자 놓기 수치도 모의다.
4. PR 리뷰는 3 PR, 그중 본문이 있는 것은 [#214](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/214) 하나다. 의견은 대부분 이슈 댓글로 남겼다.
5. 몇 이슈 댓글에 커밋 해시 대신 `<해시>` 자리표시가 남아 있다 ([#77](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/77) · [#78](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/78) · [#89](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/89)). 모델 파일이 든 큰 PR ([#50](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/50) +60,193 / −540,953, [#261](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/261), [#196](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/196)) 은 코드 변경을 따로 보기 어렵다.
6. 실측 이슈 [#87](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/87) (TFmini 보정) · [#266](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/266) (바퀴 · 팔 영점 · 팔레트 v5 실물 확인) 이 열려 있다.

---

## 3. 온창범 (Onbeom)

> 개인 보고서 원문: [contributions/온창범_보고서.md](contributions/온창범_보고서.md) (그림 15 장) · 모의 시험 영상 20 개: [results/sim_videos/](../results/sim_videos/)

### 3.1 맡은 일

| 영역 | 내용 | 주요 파일 |
|---|---|---|
| 인지 | `perception_node` (신호등 · 바코드 · 퍽 · 장소 태그 · 주사위 칸 · 장소 상자), 라이다 합치기 · TFmini 비상 정지 · 후방 장애물, YOLO 엔진 연동 · 배포 | `perception/` |
| 퍽 측량 | 시작 박스 퍽 등록표 `puck_survey`, 문제 칸 다시 보기 | `perception_survey/` |
| 위치 추정 | `loc_monitor` · 지도 · AMCL 입력, 위치 상태 `/localization/status`, C++ 판 | `localization/` |
| 실물 센서 · TF | 실물 TF (B안), IMU · 라이다 방향, D435 장착 피치 실측, udev, 팔 Arducam 드라이버 · 보정 | `opencr_hardware/urdf/`, `perception/udev/`, `bringup/` |
| 미션 (11-10 부터 이어받음) | 재검증 기록 (장소별 퍽 ID · 중복 · 누락), 이동 전후 사진, 상자 재기 (`measure_front`) · 비스듬히 서기, 시작 박스 회전 대응, 누운 직육면체 세우기 정책 연결 | `mission/`, `perception/`, `robot_policy/` |

### 3.2 구현 (작성 · 병합된 PR)

**인지**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#2](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/2) · [#3](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/3) (10-23 ~ 25) | 카메라를 직접 열던 구조를 영상 토픽 입력으로 바꿈, 앞뒤 라이다 합치기 · TFmini 비상 정지, 퍽 · 장소 태그 · 신호등 · 주사위 인지와 메시지 | 모의와 실물이 같은 노드를 씀 |
| [#5](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/5) · [#7](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/7) · [#8](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/8) (10-26 ~ 27) | 바코드 라벨 찾기 · 읽기 C++ (READ_BARCODE), 앞뒤 D435 퍽 병합, READ_TAG 장소 태그 · 팔 정렬 목표 유지 | 바코드 · 장소 태그를 미션이 받을 수 있게 됨 |
| [#26](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/26) · [#27](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/27) · [#28](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/28) (11-01) | 신호등 자리 고정 전 관측 보관, 바코드 회전 후보 읽기, 3000 K 조명 색별 문턱 | 모의 판정 R2′ 32/40 → 40/40, R3′ 60/72 → 72/72, 3000 K 통과 (S-01 ~ S-03) |
| [#18](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/18) · [#24](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/24) · [#29](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/29) · [#51](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/51) · [#54](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/54) · [#57](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/57) | 퍽 측량 등록표 `puck_survey`, 모양 판별 보완, 팔 태그 관측 우선, 문제 칸만 다시 보기 `/perception/survey/revisit` | 모의 측량 24/24 · 288/288 · 틀린 완료 0 |
| [#23](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/23) · [#25](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/25) · [#97](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/97) · [#158](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/158) | 촬영 시각과 먼 TF · 틀린 camera_info 로 위치를 내지 않음, 촬영 시각 TF 가 온 장 고르기 | 촬영 시각 TF 비율 6 위상 모두 1.0 (끄면 0.044 ~ 1.0) |
| [#22](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/22) · [#32](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/32) · [#100](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/100) · [#103](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/103) · [#120](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/120) · [#126](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/126) · [#210](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/210) | YOLO 4 클래스 후처리 (클래스별 NMS) · 입출력 검사, D435 480 · 팔 384 FP16 엔진 분리, 배포 엔진 D → E2 → E3, 엔진 해시 대조 | Jetson GPU FP32 53.1 % → FP16 18.9 %. E3 새 영상 앞 0.983 · 0.994, 뒤 0.996 · 0.996 (R-33) |
| [#96](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/96) | 팔 YOLO 박스로 정렬 목표 잠금 | 모의 A1 다른 퍽 잡음 3 → 0 |
| [#72](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/72) · [#223](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/223) · [#228](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/228) · [#234](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/234) · [#250](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/250) | 장소 상자 윗면 찾기 (place_box), 화면 밖 후보 제외, 앞 D435 상자 재기 서비스, 카메라 높이 오차 보정, 배율 후보 5 → 3 | 모의 240/240 (S-38), 높이 ±1 cm 에서 60 ~ 95 mm → 0.66 mm (S-40). 실물 상자 재기는 불통과 (R-44) |
| [#130](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/130) · [#131](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/131) · [#159](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/159) · [#171](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/171) · [#180](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/180) | 실물 팔 카메라 바코드 라벨 찾기 · 막대 방향 흐림, 태그 줄여 찾기, 태그 검색 단축 · 둘레 추적 | 실물 바코드 0/1038 → 4 라벨 100 % (R-02 · R-03), 태그 15 종 1.000 (R-32) |
| [#181](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/181) | 신호등 빨강 · 노랑을 켜진 칸 높이로도 가름 | HSV 만 쓰면 실습실에서 빨강 0/1006 · 3 판 틀린 확정 (R-38) → 3 판 모두 정답 · 틀린 확정 0 (R-39) |
| [#45](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/45) · [#42](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/42) · [#280](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/280) · [#296](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/296) | 뒤 D435 후방 장애물, TFmini 끊김 처리, TFmini 노드 중복 실행 고침, 팔 카메라 자세별 실측 보정 (기본 꺼짐) | 비상 정지가 끊긴 센서를 '멀다' 로 보지 않음 |

**위치 추정**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#91](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/91) (11-03) | `localization` 패키지: loc_monitor · 지도 · AMCL 입력 (AMCL 기본, Cartographer 는 인자로) | 모의 K1 0.52 s 감지 · 다시 찾은 오차 0.0156 m (S-35) |
| [#102](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/102) · [#129](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/129) | Jetson 에서 TF 기다림으로 막히던 것, 정지 중 거짓 LOST | loc_monitor CPU 47 → 33 % (R-31), LOST:gap 40 → 0 (S-37) |
| [#256](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/256) · [#310](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/310) | scan_body_mask 점 수 변화에 죽지 않게, 위치 추정 노드 C++ 판 · 퍽 조사 TF 필요할 때만 | 실물 라이다가 399 · 400 점을 번갈아 내도 동작, C++ 판 재생 비교 상태 바뀜 19 · lock 4 같음 (S-44) |

**실물 센서 · TF · 장치**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#56](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/56) · [#162](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/162) | 실물 TF 를 기준 모델 B안에, IMU 보드 방향 yaw −90° | 실물 센서 위치가 모델과 같은 기준 |
| [#240](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/240) · [#253](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/253) | 앞 D435 피치 35.20° → 32.62°, 뒤 D435 피치 실측값 | 앞 남은 피치 +0.09° · +0.16°, 뒤 +0.19° · +0.18° (R-40 · R-42) |
| [#304](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/304) | 실물 라이다 TF 요 180° (실물만) | 실물 지도 29 · 17 % → 58 · 43 % (R-46 · R-47) |
| [#140](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/140) · [#146](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/146) · [#147](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/147) · [#148](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/148) · [#150](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/150) · [#153](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/153) | 팔 Arducam OpenCV 노드 1280×720, 실측 보정 (fx 865), 거꾸로 장착이라 180° 회전 발행, 공유 메모리 · 버퍼 2 | 15.3 → 30.6 Hz (R-35 · R-36) |
| [#182](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/182) · [#252](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/252) · [#309](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/309) | D435 시리얼 기본값, udev 실제 값, 라이다 이름을 허브 칸 + 시리얼로 | 허브 포트를 바꿔도 장치 이름이 같음 |

**미션 · 강화학습 (11-10 부터)**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#174](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/174) · [#175](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/175) · [#186](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/186) · [#198](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/198) | 재검증 때 장소별 퍽 ID 기록 · 전체 중복 · 누락 판정, 기본 미션 plan 모드, 이동 전후 팔 카메라 사진 · 미션별 기록 폴더, 측량 pose 를 퍽 등록표로 | 발제 재검증 2 점 항목 (전체 중복 · 누락) 을 코드로 판정 |
| [#203](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/203) · [#212](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/212) · [#215](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/215) · [#217](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/217) | 누운 직육면체 세워 놓기 스킬 · 학습 코드 · 정책 r2g2 | 모의 노드 판정 98.9 % (S-43), 실물 미확인 |
| [#232](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/232) · [#247](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/247) | box_look 7 필드 (거리 · 각 지정), 시작 박스를 재서 회전각 · 집기 자리 맞추기 (기본 꺼짐) | 비스듬히 서기 224/224 (S-39), 시작 박스 9 경우 끝까지 (S-41) |

그 밖의 병합 PR: #9 · #16 · #19 · #21 · #33 · #44 · #46 · #47 · #48 · #58 · #68 · #80 · #90 · #95 · #122 · #123 · #124 · #125 · #137 · #164 · #176. 닫힌 PR 1 건: #52.

### 3.3 검증 (직접 돌린 시험)

원자료 위치: 이슈 · PR 본문 (링크), 시험 표 [test-results.csv](../results/test-results.csv) (내 시험 S-01 ~ S-05, R-01 ~ R-05, S-35 ~ S-44, R-31 ~ R-49). 실습실 영상 · Jetson 부하 기록 · YOLO 판정 영상 일부는 **저장소 밖 개인 기록** (`lv2_next_module/`) 에만 있다 (한계 3.6).

**실물**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 바코드 판독 (11-04 → 11-06) | Jetson · 팔 Arducam 1280×720 · 약 10 cm, 4 라벨 (로봇 없이) | ≥ 90 % · 틀린 문자열 0 · p95 ≤ 1000 ms | 1 차 0/1038 → 라벨을 먼저 찾아 펴게 고친 뒤 4 라벨 100 % (499 장) · 틀림 0 · p95 64.8 ms | [#130](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/130), R-02 · R-03 | 로봇에 단 출발 자세 (R-05) 는 못 함 |
| 태그 판독 (11-06) | 같은 장비, 퍽 16 mm · 장소 48 mm 15 종 | 태그별 ≥ 90 % · 다른 ID 0 | 1 차 L1 0.347 · L3 0.244 → 줄여 찾기 뒤 15 종 1.000 · 다른 ID 0 · 중앙 105.4 ms | [#131](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/131), R-32 | 노드 메시지 확인 (11-10) 4/4 · 12/12 · 3/3 (R-37), 거리 미기록 |
| YOLO E3 새 영상 (11-06) | 학습 · 고르기에 안 쓴 실물 녹화, 커튼 닫음 | 카메라마다 재현율 · 정밀도 ≥ 0.95 | 앞 0.983 · 0.994, 뒤 0.996 · 0.996 통과 · 팔 0.897 · 0.886 불통과 → '색 + 자리' 기준 0.967 · 0.975 | 개인 기록 `261105_2047_final_eval/`, R-33 · R-34 | 팔은 결과를 본 뒤 기준을 바꿈 (옛 불통과 유지) |
| 팔 카메라 입력 (11-09) | Jetson · D435 2 대와 함께 · 1280×720 | ≥ 25 Hz · stamp 나이 p99 ≤ 66 ms | 15.3 Hz (버퍼 1) → 30.6 Hz · p99 13 ms | [#150](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/150), R-36 | — |
| 신호등 실습실 판정 (11-11) | PC · 실물 D435 · 실습실 신호등, 3 판 | 12 회 정답 확정 · 틀린 확정 0 | HSV 만이면 빨강 0/1006 · 3 판 틀린 확정 (R-38) → 켜진 칸 높이를 함께 써 3 판 모두 약 25 s 에 정답 · 빨강 487/487 · 노랑 661/661 · 초록 446/446 | 개인 기록 `261106_1655_real_drive/` P5i · 영상, [#181](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/181), R-39 | 높이는 장착 기울기에 민감 (브래킷 10° 들리면 초록 놓침) |
| 앞 D435 장착 각 (11-12 ~ 13) | 책상 위 노트북 화면 태그 판 PnP · 깊이 책상 평면, 판 고정 자동 직진 | 남은 피치 ≤ 0.28° · \|요\| ≤ 0.7° | 피치 −2.55 · −2.65 · −2.56° → 32.62° 로 고친 뒤 +0.09 · +0.16°, 요 −0.60° | [#240](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/240) · [#253](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/253), R-40 · R-41 | 같은 측정에서 직진 옆 미끄럼 2.35° 를 찾아 [#248](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/248) 로 넘김 |
| 뒤 D435 장착 각 (11-13) | 같은 주행 · 깊이 바닥 평면 | 한계는 #253 본문 | 요 −0.67° 통과 · 피치 +1.25° 불통과 → 고친 뒤 +0.19 · +0.18° | [#253](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/253), R-42 | — |
| 실물 상자 재기 (11-13) | 앞 D435 · 과제 상자 4 배치 + 빈 바닥 | 빈 바닥 찾음 0 · 상자 중심 15 mm 안 | 빈 바닥 0/10 · 상자는 배율 1.0 으로 0/10, 배율을 풀면 10/10 · 중심 15 ~ 29 mm | 개인 기록 `261112_1221_front_d435_cal/` R2, R-44 | **불통과**. 낡은 상자 · 정답 기준 의심, 원인 미확정 |
| place_box 처리 시간 (11-13) | Jetson · 카메라 3 대 + 인지 부하 · 렌더 80 장 | 장당 ≤ 3 s | 앞 1.32 s 통과 · 팔 p95 4.77 s 불통과 | [#250](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/250), R-45 | 배율 후보 5 → 3 (계산상 약 2.9 s), **실물 재측정 못 함** |
| 라이다 방향 · 실물 미로 지도 (11-14 ~ 16) | 상자 6 자리로 가려 보기, 실물 bag 을 PC 에서 Cartographer c6 | 지도: 실물 ↔ SDF 5 cm 안 둘 다 ≥ 70 % | 라이다 요 약 180° 틀림 → [#304](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/304). 지도 29 · 17 % → 58 · 43 % → 11-16 재녹화 17 ~ 30 · 8 ~ 12 % | [#303](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/303) · [#317](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/317), R-46 · R-47 | **기준 미달** → 시연은 SDF 지도. 바퀴 odom 회전이 IMU 의 1.32 배 (원인 미확정) |
| Jetson 부하 (11-04) | 카메라 3 대 + YOLO + AMCL · loc_monitor, 녹화 재생 30 분 × 5 | CPU · GPU 평균 ≤ 60 % · RAM ≤ 5.5 GB | CPU 44.0 % · GPU 50.2 % · RAM 1.72 GB · YOLO 15 Hz | 개인 기록 `261103_1913_jetson_load2/`, [#102](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/102), R-31 | 주행 · 팔 없는 부하. 11-13 전체 구성 정지 상태는 CPU 85 ~ 95 % ([#265](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265)) |

**모의**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 신호등 · 바코드 판정 (10-28 → 11-01) | Isaac 렌더를 실제 perception_node 에 재생 | 틀린 확정 0 · 전부 | 첫 판정 R2′ 32/40 · R3′ 60/72 → 고친 뒤 320/320 · 100/100 · 144/144 | [#26](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/26) · [#27](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/27) · [#28](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/28), S-01 ~ S-05 | — |
| 위치 추정 방식 비교 (11-06) | 설계 45 Nav2, 각 3 판, 규칙을 실행 전에 정함 | 두 방식 판정 통과, RMSE 차 ≥ 0.005 m 면 작은 쪽 | AMCL 0.0154 ~ 0.0162 m · Cartographer 0.0072 ~ 0.0081 m | [#116 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/116#issuecomment-6673626701), S-35 · S-36 | 규칙으로는 Cartographer 였지만 팀 요청으로 AMCL |
| 상자 ±5 cm · ±45° (11-12) | Isaac 렌더 재생 | 상자 15 mm · 4° 안 | 앞 D435 재기 240/240 (최대 2.10 mm · 2.09°), 비스듬히 서기 place_box 224/224 · 태그 98.9 % | [#228](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/228) · [#234](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/234), S-38 ~ S-40 | 실물은 R-44 불통과 |
| 시작 박스 회전 (11-13) | mission mock · 계산, 9 가지 경우 | 끝까지 진행 | 9 경우 모두 끝까지 · 퍽 자리 0.07 mm 안 · 남는 면 어긋남 최대 27.7° | [#247](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/247), S-41 | 20° 넘는 476 경우 중 다시 서기 가능 64 (13 %) → 보류, 손목 면 맞춤은 [#251](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/251) (S-42) |
| 누운 직육면체 세우기 r2g2 (11-11) | Isaac Lab, 노드 관측 · 관절 속도 3.0 rad/s | 노드 판정 ≥ 95 % | 95.7 % · 노드 판정 98.9 % · 미션 순서 칸 넣기까지 81.8 % | [#217](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/217), S-43 | **실물 미확인** (11-12 팔 시험은 옛 r2e 로 실패, R-18) |

### 3.4 리뷰

**사실 그대로의 집계 (GitHub)**

- 다른 팀원 PR 에 본문 있는 리뷰를 남긴 것은 2 건이다 ([#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94), [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312)). 줄 단위 리뷰 댓글은 0 건이다.
- 기술 의견은 대부분 다른 팀원이 연 이슈 스레드에 계산 · 모의 근거와 함께 남겼다 (11 개 이슈). 아래는 그중 다른 팀원 코드 · 결정으로 이어진 것이다.

**PR 리뷰**

| 대상 (담당) | 문제 · 의견 (요지) | 반영 여부 · 후속 |
|---|---|---|
| [#94](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/94) (developlcy-oss) | `registered` 모드에서 측량이 실패해도 지난 실행 결과로 계획될 수 있음, `survey.poses` 가 계획기 파일의 복사본이라 어긋날 수 있음 | **반영** (커밋 a573322: 측량 시작 때 지난 등록표 삭제, `approach_poses.yaml` 직접 읽기) |
| [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) (developlcy-oss) | CPU 변화 (0.68 %) 가 작아 의미 없다는 반려 의견 | **반영 안 됨**: 다른 팀원이 병합, 합의 기록 없음 |

**이슈 스레드에서 준 의견 → 다른 팀원 코드 · 결정 반영**

| 대상 (담당) | 문제 · 의견 (요지) | 반영 여부 · 후속 |
|---|---|---|
| 시작 박스 회전 (kanichong) — [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6821994172) | 제안된 '회전각 = 기본각 + β' 를 계산해 보니 방향이 반대 (왼쪽 퍽 β +10° 에서 어긋남 14.7°, 그대로 두면 10.1°) | **반영**: 실제 회전 중심 기준 계산으로 [#247](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/247), ±45° 부호 모호는 mission_log 경고 |
| 시작 박스 · 비스듬히 서기 기하 (kanichong) — [#262](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/262#issuecomment-6861599525) | mock 미션 R0 (끔) · R1 (켬, 시작 박스 2 cm · −1.5 cm · 12°) 으로 확인 | 두 판 모두 DONE |
| FINE 정렬 자세 ALIGN_A (WindForce08 이슈, developlcy-oss #202) — [#154](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/154#issuecomment-6792349716) | ALIGN_A 자세가 미션 순서 모의에서 팔 - 몸체 접촉을 냄 | 결과 표를 [#218](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/218) · [#219](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219) 로 넘김 → DRIVE = E2 ([#270](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/270)) 로 이어짐 |
| 집기 정밀도 (developlcy-oss) — [#128](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/128#issuecomment-6707541504) | 약 1 mm 측정은 어렵지만 퍽 바닥 수평 위치 1 ~ 2 mm 만 재도 효과가 큼, 내부값 실측 전에는 놓기 IK 를 측정에 기대지 말 것 | 내부값 실측 [#143](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/143#issuecomment-6729718432) (fx 865) 으로 이어짐, [#146](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/146) 로 해결 |
| verify · 보기 자세 (kanichong) — [#65](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/65#issuecomment-6607739575) · [#82](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/82#issuecomment-6607738487) | 대기 자세에서는 윗면을 보기 어려우니 TFmini 로 다가간 뒤 보는 자세를 권함, 자세별 퍽 윗면 · 태그 화면 투영 | 참고 자료로 제공 |
| TFmini 거리 (kanichong) — [#87](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/87#issuecomment-6870406755) | 실물 TFmini Plus 를 줄자와 비교한 실측표 (11-13) | 실측값 제공 |
| 그 밖 | [#81](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/81#issuecomment-6607737926) 면 요 정의 · 헬퍼, [#85](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/85#issuecomment-6607739028) 위치 상태 연동, [#116](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/116#issuecomment-6673626701) 위치 추정 방식 확정, [#190](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/190#issuecomment-6782882177) 칸에 못 앉음 원인 재확인 | — |

**받은 리뷰와 반영 (참고)**

| 내 PR | 리뷰어 · 지적 | 반영 여부 |
|---|---|---|
| [#153](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/153) | WindForce08: 주점 고친 것 확인. 실물 인지는 왜곡을 편 영상 (newK fx 601.2 · fy 792.0) 을 쓰므로 Gazebo K 도 그 값이어야 실물과 같다, Jetson 의 newK 확인 부탁 | PR 에 답을 남기지 못함. 남은 일은 [#145](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/145) 에서 WindForce08 이 마무리 (11-10 닫힘) |
| [#2](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/2) | developlcy-oss: "승인합니다" | — |

### 3.5 설명 (전체 흐름 이해)

전체 흐름은 [architecture.md](architecture.md) 의 그림과 같다. 내가 맡은 인지 · 위치 추정은 `/perception/*` 과 `/localization/status` 로 mission · drive · manipulator 에 값을 준다.

```
D435 앞 · 뒤 · 팔 Arducam · 라이다 앞 · 뒤 · TFmini
   └─▶ perception_node ─(/perception/signal · /perception/barcode)─▶ mission · mission_planner
          ├─(/perception/pucks)─▶ puck_survey ─(퍽 등록표 파일)─▶ mission
          ├─(/perception/place_box · task_marker · check_dice)─▶ mission (measure_box · verify)
          └─(/perception/result · /arm_range)─▶ drive_node · manipulator_node
AMCL (map → odom) ─▶ loc_monitor ─(/localization/status · /localization/lock)─▶ mission · drive (loc_gate) · puck_survey · twist_mux
```

내가 맡지 않은 코드 · 설정 · 결과를 읽고 판단한 근거:

| 근거 | 내가 맡지 않은 부분 | 무엇을 이해 · 판단했나 |
|---|---|---|
| [#226](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/226#issuecomment-6821994172) | 계획기 회전 범위 · 팔 집기 정책 | 제자리 회전은 J1 자리만 옮기고 정책의 닫힘 방향은 J1 → 퍽 방향이라는 점에서 제안 식의 방향이 반대임을 계산으로 보임 |
| [#219](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/219#issuecomment-6800717726) | 계획기 칸 채우는 순서 · 팔 주행 자세 | `per_location` 이 처음 싣는 두 퍽을 G_2 · G_3 에 넣어 DRIVE 자세와 겹침을 짚음 |
| [#265](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/265) · [#275](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/275) · [#276](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/276) | bringup · Nav2 · 계획기 | Jetson 정지 상태 CPU 85 ~ 95 % 를 담당별로 나눠 요청, spawner · bt_navigator 실패를 재현 기록과 함께 알림 |
| [#248](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/248) · [#249](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/249) | 바퀴 · 모의 모델 | 카메라 측정에서 직진 옆 미끄럼 2.35° 를 찾아 가르는 시험 방법을 넘김, 실물 xacro 와 Isaac 모델의 라이다 자리 차 (22.9 · 8.0 mm) |
| [#278](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/278) · [#292](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/292) · [#295](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/295) | 팔 기구 · 관절 영점 | 팔 카메라가 설계보다 약 20 mm 낮은 원인을 장착 치수 · 영점 · 링크 길이로 나눠 실측 방법과 함께 요청 |

### 3.6 한계

1. 실물로 확인한 것은 인지 판독 (신호등 · 바코드 · 태그), 카메라 장착 각, Jetson 부하, 실물 미로 지도 (기준 미달) 까지다. 11-16 실물 미션은 첫 단계 observe 에서 멈춰 (팔 시작 정리 거절) 경기 중 신호등 · 바코드 관측 기록이 없다 (M-12).
2. 실물 미로 지도는 기준 70 % 에 못 미쳤다 (58 · 43 %). 11-16 재녹화는 더 나빠졌고 원인 (바퀴 odom 회전이 IMU 의 1.32 배) 은 확정하지 못했다.
3. 실물 상자 재기 (R-44) 와 팔 place_box 처리 시간 (R-45) 이 불통과다. #250 은 계산상 약 2.9 s 로 실물 재측정을 하지 못했다.
4. 위치 추정 · 상자 · 강화학습 성공률은 대부분 모의다. 누운 직육면체 r2g2 는 실물에서 확인하지 못했다.
5. YOLO 팔 기준 (R-34) 과 r2g2 일부 기준은 결과를 본 뒤 바꿨다. 바꾸기 전 불통과도 함께 남겼다.
6. 신호등 실습실 영상, Jetson 부하 기록, YOLO 판정 영상 등 원자료 일부가 저장소 밖 개인 기록 (`lv2_next_module/`) 에만 있다.
7. 다른 팀원 PR 에 남긴 리뷰가 2 건뿐이다. 의견 대부분은 이슈 스레드에 남겼고, 받은 리뷰 #153 의 질문에는 PR 에서 답하지 못했다.


---

## 4. 한상준 (WindForce08)

> 저장소 기록 (커밋 · PR · 이슈) 으로 정리하고 본인이 확인함 (2026-11-17)
> git 작성자 `WindForce08 <hsj38038@gmail.com>` (dev 비병합 커밋 30 개), GitHub `WindForce08`.

### 4.1 맡은 일

**하드웨어 · 주행 담당** — OpenCR 2 대 펌웨어와 호스트 (ros2_control), Nav2 주행 설정, 실물 주행 시험 · 운용.

| 영역 | 내용 | 주요 파일 |
|---|---|---|
| OpenCR 펌웨어 | 프로토콜 v5 (팔 HOLD · REARM · FAULT), 4 륜, 보드 역할별 빌드, 팔 벤치 이식 (범위 · 게인 · 그리퍼 막힘 보호), 바퀴 속도 한계, 보드별 배터리 경고, RC-100 리모컨 | `workspace/ros2_ws/src/opencr_firmware/` |
| opencr_hardware 호스트 | `backend` 선택 (opencr · mock · isaac), 팔 대기 자세 검사, 팔 벤치 도구 `opencr_arm_tool.py`, 펌웨어 SIL 시험 | `opencr_hardware/src/`, `opencr_hardware/test/`, `opencr_hardware/scripts/` |
| 주행 (Nav2) | `navigation` 패키지 (Smac 2D · RotationShim + MPPI), 출발 자세 통일, 후진 때 뒤 카메라, footprint, 속도 한계 인자 · 시작 지연 · 모서리 대책, 실물 기본 지도, 실물 첫 실행 문서 | `workspace/ros2_ws/src/navigation/` |
| 주행 도구 | Isaac 주행 RL `drive_rl`, 규칙 v2 주행 시연 `drive_demo` (경로 이탈 멈춤 · 되찾기) | `workspace/tools/drive_rl/`, `workspace/tools/drive_demo/` |
| 팔 벤치 (초기) | 팔 OpenCR 단독 시험, 슬라이더 · 모니터 · pick RL task | `workspace/tools/arm_bench/`, `workspace/docs/arm_bench_test_2026-10-2*.md` |
| 실물 통합 · 시험 | Jetson 에 OpenCR 2 대 연결 (udev · 전원 순서: 배터리 먼저 · RESET), 바퀴 ID · 방향 · 속도 한계 · 벨트 실측 (띄움 · 바닥), 라이다 2 대 USB 자리 · 방향 확인, Jetson 점검 스크립트, 11-15 · 11-16 실물 주행 운용 (근거 4.3) | `opencr_hardware/udev/99-opencr.rules`, `workspace/tools/drive_demo/` (`jetson_check.sh` · `scan_rotate.py` · README 전원 순서 · USB 자리) |
| 경기 뒤 (11-17) | 자율주행만 다시: 장애물 감시 · 즉시 멈춤 · 3 초 판단 · 다시 계획 · 정지점 건너뛰기 (Gazebo), 실물 bag 으로 미로 SLAM 지도, 회전 속도 상한 비교. 픽 앤 플레이스 포함 전체 미션 Isaac 시뮬 준비 | `workspace/tools/drive_avoid/`, `workspace/tools/mission_rl/` |

### 4.2 구현 (작성 · 병합된 PR)

병합된 PR 16 건, 닫힌 PR 0 건.

**펌웨어 · opencr_hardware**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#6](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/6) (10-26) | 펌웨어 프로토콜 v5: 팔 HOLD / REARM · FAULT (멈춰도 토크 유지, 리셋으로만 재개), 4 륜 (ID 1 ~ 4), 보드 역할 지정 빌드 | SIL 시험 77 건 통과. 인터페이스 변경은 [`opencr_v5_change_request.md`](../../workspace/docs/opencr_v5_change_request.md) 로 승인 요청 |
| [#12](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/12) (10-27) | 팔 벤치 펌웨어 이식: 관절 범위 · 게인 · 그리퍼 막힘 보호 (`GripGuard`), 벤치 도구 · pick RL task | 10-27 실기 시험 결과 반영 (4.3) |
| [#76](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/76) (11-03) | bringup `backend` 인자 (opencr · mock · isaac), 잘못된 불리언 인자 거부, diff_drive 6.x 에서 무시되던 감속 한계 이름 수정 | 장치 없이 mock 으로 컨트롤러 · twist_mux · e_stop 확인 가능. colcon test 92 건 실패 0 |
| [#166](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/166) (11-09) | 팔이 대기 자세 범위 밖이면 활성화 거부 (토크 · 명령 없이), 실물 첫 실행 문서 | 켤 때 팔이 튀는 것을 막음. 새 시험 6 개 · opencr_hardware 115 개 통과 ([#166 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/166#issuecomment-6739882520)) |
| [#179](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/179) (11-10) | 벤치 도구 `--stages` (관절 하나씩) · `--check-standby`, Nav2 footprint 에 주행 자세 팔, 켜기 전 대기 자세 안내 | 실물 대기 ↔ 주행 왕복: 오류 비트 없음, 최대 0.25 A, 끝 오차 ≤ 0.016 rad |
| [#207](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/207) (11-11) | 바퀴 속도 한계 7.91 → 7.51 rad/s, 두 보드 배터리 각각 읽기 · 부저 경고 · `/diagnostics` · `/battery/*` | 실물 바퀴 띄움 시험으로 바퀴끼리 어긋남 2.18 % → 0.04 %. colcon test 130 / 실패 0 |
| [#257](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/257) · [#258](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/258) (11-13) | RC-100 리모컨 주행 시험 (`rc100:=true`, 프로토콜 길이 · 버전 그대로), 수신 UART1 → UART2 | 실물 주행 보드에서 버튼 값 수신 확인 (UART1 0 바이트 · UART2 패킷) |

**주행 (Nav2) · 인터페이스**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#115](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/115) (11-05) | `navigation` 패키지 (Smac 2D, RotationShim + MPPI, `/cmd_vel_nav`), Gazebo 시험 launch, 출발 자세 base_link (0.69365, 0.95, π) 통일, 후진 때 `/perception/rear_obstacle` ([#84](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/84)) | Gazebo 대기 자세 12 곳 연속 12/12, AMCL 오차 평균 1.9 cm |
| [#163](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/163) (11-09) | 실물 기본 지도를 Isaac SLAM 지도 → 발제 SDF 지도 ([#116](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/116)) | SLAM 지도는 도면과 7 ~ 15 cm 달라 Gazebo 2/12 → SDF 12/12 |
| [#241](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/241) (11-12) | 속도 한계 launch 인자 (`max_vx` · `max_wz` · `rotate_wz`), lifecycle 시작 지연, MPPI PathAlign 14 → 28, 실물 회전 상한 2.0 → 1.0 rad/s (벨트 보호) | Nav2 시작 10 번 중 5 실패 → 15 번 중 0, 미션 Q1 FAILED 2/2 → DONE (Gazebo) |
| [#155](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/155) (11-09) | 쓰는 노드가 없는 `/arm/skill_state` · `/arm/safety_state` 삭제 ([#86](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/86) 결정 B) | control 49 · opencr_hardware 98 실패 0 |

**주행 도구 (ROS 패키지 밖)**

| PR | 바뀐 기능 | 효과 · 결과 |
|---|---|---|
| [#76](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/76) · [#302](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/302) (11-03 · 11-15) | Isaac 주행 RL `drive_rl`, 회전 규칙 v2 (제자리 ≤ 90° · 후진 ≤ 0.40 m · 반경 ≥ 0.20 m), 관측 55 | 벨트 v7 물리 전체 경로 1/58 → **실물에는 쓰지 않음** (본문에 명시) |
| [#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306) (11-15) | `drive_demo`: 규칙 v2 계획 경로를 Nav2 로 따라가는 시연 도구, 경로 이탈 멈춤 10 cm, 팀원용 안내 [`README.md`](../../workspace/tools/drive_demo/README.md) | 노트북 바탕화면에만 있던 도구를 팀원이 혼자 쓸 수 있게 함 |
| [#318](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/318) · [#319](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/319) (11-16) | 방향 맞추기 거부 시 건너뛰기, dev 11-16 맞춤 (팔 펌웨어 v6 판정 · 위치 감시 C++), 조각 실패 시 되찾기 (위치 다시 찾기 → 0.30 m 되돌아감 → 다시), 실물형 회전 시뮬 | loopback 10/10, 일부러 실패 10/10 · 되찾기 1 번씩. 되찾기 기록을 실패로 세던 버그를 시험에서 잡음 |

### 4.3 검증 (직접 돌린 시험)

원자료: [`results/reproduction_20261115/`](../results/reproduction_20261115/) (11-15 실물 drive_demo 기록), [`results/real_20261116/drive_runs/`](../results/real_20261116/drive_runs/) (11-16 drive_demo 기록), [`workspace/docs/arm_bench_test_2026-10-26.md`](../../workspace/docs/arm_bench_test_2026-10-26.md) · [`-10-27.md`](../../workspace/docs/arm_bench_test_2026-10-27.md), PR 본문 · 이슈 댓글.

**실물**

| 시험 | 조건 | 기대 | 실제 | 원자료 | 한계 · 개선 |
|---|---|---|---|---|---|
| 팔 OpenCR 단독 벤치 (10-26 · 27) | 팔 OpenCR 만 PC 에 연결, 단계마다 승인 | 관절 범위 · 방향 · 대기 자세 · 그리퍼 | 모터 6 대 ID · 범위 · + 방향 정리, 그리퍼 다시 결합 (열림 1550 · 닫힘 2650 tick), 퍽 쥐다 `MOTOR_HW` → PWM 400 · 막힘 보호 | arm_bench_test 문서, CSV R-06 | 물리 E-stop 없음을 기록. 과부하 모터는 확인 전에 전원을 꺼 못 가림 |
| 팔 펌웨어 · J1 혼 (11-09) | dev 빌드 업로드, 혼 재조립 | 부팅 점검 통과, 정면 2048 근처 | 업로드 CRC OK · 99 Hz, J1 정면 2029 tick (−1.7°), J2 ~ J5 기준 자세 · + 방향 URDF 와 같음 | [#139 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/139#issuecomment-6738448640), [#161 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/161#issuecomment-6739352224) | 이후 11-16 에 관절 1 모터 180° 문제가 다시 나옴 (1.6 이창엽 한계) |
| 대기 ↔ 주행 자세 왕복 | `opencr_arm_tool.py move --stages` | 오류 없음 | 오류 비트 없음 · 최대 0.25 A · 끝 오차 ≤ 0.016 rad · 한쪽 약 59 s | [#179](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/179) | 한 관절씩, 충돌 검사 없음 (벤치 도구) |
| 바퀴 속도 한계 (11-11) | 바퀴 띄움, 네 바퀴 6 s 회전 수 비교 | 바퀴끼리 어긋남 작게 | 7.91 rad/s 2.18 % (오뒤 +47°) → 7.51 rad/s 0.04 %, 업로드 뒤 7.49 rad/s 로 잘림 | [#207](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/207) | 배터리 11.9 V 아래면 다시 재야 함 (#49) |
| RC-100 수신 (11-13) | 주행 OpenCR 단독, 확인용 스케치 | 패킷 수신 | UART1 0 바이트 · UART2 패킷 → 펌웨어 수정 뒤 STATE 에 버튼 값, 누르는 동안 50 ~ 60 ms 마다 반복 | [#258](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/258) | 블루투스 포함 응답 시간 미측정, 소프트웨어 정지일 뿐 |
| **drive_demo 실물 (11-15 저녁)** | Jetson 실물, AMCL · EKF, 계획 `길모음7cm_어제순서`, RPP 전진 0.20 · 후진 0.10 | 구간 모두 정지점 ≤ 5 cm · 충돌 없음 | 20:52 S0 1/1 (3.8 cm). 20:55 빠르게 1/10 (S1 Spin 시간 초과). 21:04 2/10 (S2 7.7 · S3 5.3 cm, S4 `Failed to make progress`). 21:13 구간마다 Enter: S0 · S1 · S4 · S5 ✓, S2 8.3 · S3 5.2 cm, **S6 벽 닿음** | [`1115_2052`](../results/reproduction_20261115/1115_2052_실물_slow_구간0-0/summary.txt) · [`1115_2055`](../results/reproduction_20261115/1115_2055_실물_full/summary.txt) · [`1115_2104`](../results/reproduction_20261115/1115_2104_실물_slow/summary.txt) · [`1115_2113/follow.log`](../results/reproduction_20261115/1115_2113_실물_slow/follow.log), CSV R-27 | 실물이 명령 회전의 66 ~ 80 % 만 돎 → 실물 이어진 후진 0.50 m, 경로 이탈 멈춤 10 cm ([#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306)), 되찾기 ([#319](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/319)) 추가 |
| 경기 당일 drive_demo (11-16) | 같은 도구, max 모드 · 벨트 2.0 · 모터 3.0, 계획 `7cm_실측L_어제순서_끝90도` | 10/10 | 12:04 0/10 (위치 확인 안 해 출발 안 함), 12:15 0/10 (S0 진행 실패 → 되찾기 1 번 → Ctrl+C), 12:25 **4/10** (S0 2.3 · S3 4.2 · S6 2.2 · S7 4.0 cm, 나머지 6.0 ~ 14.4 cm) | [`drive_runs/`](../results/real_20261116/drive_runs/), CSV M-04 · M-05 · M-08 | 실행자는 로그에 남지 않음. 정지점 5 cm 넘는 구간이 많음 |
| 배터리 진단 (경기 당일) | 데모 브링업 | 팔 배터리 위험이면 ERROR | `opencr/battery` 진단이 팔 배터리 6.78 V · 0 % 를 ERROR 로 냄 → 그 판은 시작 안 됨 | [`demo_261116_124053.log`](../results/real_20261116/launch_logs/demo_261116_124053.log), M-15 | [#207](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/207) 의 두 보드 진단이 실물에서 동작한 기록 (부저는 로그로 확인 불가) |

**모의 · 코드 시험**

| 시험 | 조건 | 실제 | 원자료 |
|---|---|---|---|
| Gazebo Nav2 | SDF 지도, 대기 자세 12 곳 · 북쪽 통로 · 기둥 모서리 · 미션 Q1 | 12/12, 북쪽 통로 멈춤 3 → 0, B2:S → B6:W 멈춤 3 → 통과 3 (기둥 여유 6.2 ~ 7.9 cm), Q1 DONE | [#115](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/115), [#241](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/241) |
| 박스 앞 정차 재현 | 이홍주 `gazebo_box_park.sh` | FINE 5.8 s · turn 0.86° · 같은 결과 → [#157](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157) 닫음 | [#157 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/157#issuecomment-6761667521) |
| drive_demo loopback · Gazebo | 노트북 loopback, Gazebo 전체 미션 (가짜 팔 · 인식), 새벽 묶음 12 회 | 10/10 · 정지점 밖 최소 여유 4.7 cm, Gazebo DONE 451 s, 묶음 10/12 | [#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306), [#318](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/318), CSV S-32 ~ S-34 |
| 실물형 회전 시뮬 | 꺾기 79 % · 제자리 70 % | 10 구간 끝까지, 정지점 3 곳 5 cm 넘음 (6.9 · 5.9 · 6.2 cm) | [#319](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/319) |
| 펌웨어 SIL · colcon | 펌웨어 SIL · opencr_hardware | v5 SIL 77 건, 배터리 130 / 0, RC-100 141 중 실패 1 (기존 `test_real_vs_isaac`) · 펌웨어에서 리모컨 읽기를 빼면 실패함을 확인 | [#6](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/6), [#207](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/207), [#257](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/257) |

**시뮬레이션 → 적용 (정리)**

실물에 올리기 전에 시뮬로 먼저 확인하고, 결과로 설정 · 경로를 바꿨다. 시뮬 · 실물 주행 기록을 한 페이지에 모은 것: [미로 주행 기록](https://claude.ai/artifact/TEYkNqHvpLi5gUZuY7qvvB?sk=pgH8zpXgutNf41-f-OJaHg) (궤적 · 정지점 오차 · 규칙 v2 판정 · PPO 학습 곡선 · 11-17 장애물 시험).

| 시뮬레이션 (날짜) | 무엇을 확인 | 결과 | 적용 · 결정 | 근거 |
|---|---|---|---|---|
| Isaac 주행 RL `drive_rl` (11-02 ~ 11-15) | 미로 경로 (정지점 58) 를 PPO 로 따라가기 | 58/58 완주까지 갔으나 정책이 후진 74 % · 제자리 180° 회전에 기댐, 벨트 v7 물리에서 1/58 | 실물에는 안 씀 → 회전 규칙 v2 와 미리 계산한 경로로 방향 바꿈 | [#76](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/76) · [#302](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/302), 주행 기록 페이지 PPO 곡선 |
| Nav2 경로 시험 (11-13, Gazebo) | 제자리 회전을 줄인 Nav2 설정 (through-poses BT · Hybrid-A* Reeds-Shepp · MPPI Ackermann · Spin 뺀 BT) | 완주 10/10 인 판은 후진 78 %, 반경 0.20 판은 앞 8 구간 5/8 (B5:N → B6:W 벽 닿음) | Nav2 계획기만으로는 규칙 v2 를 못 지킴 → 경로는 미리 계산하고 Nav2 는 따라가기만 | 주행 기록 페이지 |
| 규칙 v2 경로 계획 (11-14 ~ 15, Hybrid A*) | 제자리 ≤ 90° · 후진 ≤ 0.40 m · 반경 ≥ 0.20 m 을 지키는 한 바퀴 | 209.7 s · 후진 5.8 % · 규칙 모두 지킴. 상자 실측 (box1 이 지도보다 북 18.6 cm) · 칠판 L 실측으로 다시 계획 | 경기 경로 = `drive_demo/legs.json` · `demo_plan.json` | [#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306) |
| 실물형 회전 시뮬 (11-15 ~ 16) | 벨트 v7 실물이 명령의 79 % (꺾기) · 66 ~ 80 % (제자리) 만 도는 것을 시뮬에 넣음 | 10 구간 끝까지, 정지점 3 곳 5 cm 넘음 | `params_demo_v7B/C.yaml`, 경로 이탈 멈춤 · 되찾기 | [#319](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/319) |
| drive_demo loopback · Gazebo 묶음 (11-15 ~ 16) | 경기 도구 전체 흐름 | loopback 10/10, Gazebo 전체 미션 DONE 451 s, 새벽 묶음 10/12 | 경기 당일 사용 | 위 표, CSV S-32 ~ S-34 |
| **장애물 멈춤 · 다시 계획** (11-17, Gazebo · 진짜 Nav2 · AMCL) | 경기에서 못 한 것: 길 위 장애물 → 멈춤 → 위치 다시 찾기 → 3 초 판단 → 원래 길 / 다시 계획 / 건너뛰기 | 치움 2/2 (멈춤 3.3 s 뒤 원래 길) · 그대로 2/2 (5.7 s 뒤 다시 계획, 장애물 여유 6.8 cm) · 통로 막힘 → box2 건너뛰고 box6 먼저 | 실물은 아직 (경기 뒤) | [`drive_avoid/README.md`](../../workspace/tools/drive_avoid/README.md), `runs/` |
| **갑자기 나타난 장애물** (11-17, 최고 속도 0.44 ~ 0.46 m/s) | 예전 감시 (경기 때 위치 감시 스캔 · 2 스캔 확인) 대 새 감시 (원래 스캔 · 가까우면 한 스캔 · TFmini) | 상자 0.3 m 앞: 예전 31.2 cm · 1.4 s 에 멈춰 **차체 닿음**, 새 20.5 cm · 0.5 s · 차체 여유 9.4 cm. 0.2 m 앞까지 차체 안 닿음. TFmini 거짓 멈춤 (Gazebo 집게 손가락이 빛을 가림) 2 → 0 번 | 감시 인자 확정 (`obstacle_guard.py`) | [`runs/sudden_table.json`](../../workspace/tools/drive_avoid/runs/sudden_table.json) |
| 실물 미로 SLAM (11-17) | 팀원 녹화 bag → slam_toolbox → 발제 좌표 | 벽 중앙 2.5 cm · 북벽 5 ~ 10 cm 어긋남, 라이다가 7 cm 상자 · L 을 봄, 그 녹화의 스캔은 180° 돌아가 있었음 | 장애물 감시의 '알려진 물체 지도' 후보 `maze_real_aligned` · `maze_real_walls` | [`drive_avoid/real_map/`](../../workspace/tools/drive_avoid/real_map/) |
| 회전 속도 상한 (11-17, 계산) | 직진 모터 7 에서 꺾기로 자연스럽게 이어지는 상한 4 안 | 한 바퀴 153.7 (지금) · 152.7 · 149.0 · 139.8 s, 대신 모터 3.0 넘는 꺾기 1.8 → 23.7 s | 아직 안 고름 (벨트 보호와 맞바꿈) | [`drive_avoid/turn_speed/`](../../workspace/tools/drive_avoid/turn_speed/) |
| **픽 앤 플레이스 포함 전체 미션** (11-17, Isaac) | 팀 팔 확인 재현 (P0) → 미션 계획 Q1 을 한 장면에서 집기 → 팔레트 → L 놓기로 이어서 (P1) | P0: 집기 → 칸 173/192 (90.1 %) · 놓기 24/24 · 빼기 24/24. P1 (4 대 · 앞 14 단계): 집기 5 번 모두 4/4 칸 3 mm 안, L2 놓기 셀 15 mm 안 3/4 | 다음: 전체 24 단계 기준선 → 약한 단계 학습 | [`mission_rl/README.md`](../../workspace/tools/mission_rl/README.md) |

### 4.4 리뷰

**사실 그대로의 집계 (GitHub, 2026-11-17)**

- 다른 팀원 PR 에 남긴 PR 리뷰: **6 PR** — [#153](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/153) (온창범, 본문 있는 COMMENTED), [#4](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/4) (이창엽, 짧은 확인 댓글), [#20](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/20) (이창엽, 본문 없는 승인), [#176](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/176) (온창범) · [#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312) (이창엽) · [#315](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/315) (이홍주) "확인했습니다" 승인. 줄 단위 리뷰 댓글 0 건.
- 다른 팀원 PR 대화 댓글 4 건 (모두 표 형식 리뷰): [#180 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/180#issuecomment-6761758924) · [#181 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/181#issuecomment-6761756396) · [#182 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/182#issuecomment-6761752376) (온창범), [#301 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/301#issuecomment-6893025993) (이창엽).
- 다른 팀원 PR 병합 2 건 ([#312](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/312), [#315](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/315)).
- 이슈 댓글 45 건 (이슈 39 · PR 6). [#172](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/172) 에는 리뷰 전에 병합된 PR 세 개 ([#171](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/171) · [#174](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/174) · [#175](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/175)) 리뷰를 모아 남겼다 ([#172 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/172#issuecomment-6757717891)).

**PR 에 준 의견**

| 대상 PR (작성자) | 문제 · 의견 (요지) | 반영 여부 · 후속 |
|---|---|---|
| [#153](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/153) (온창범) | 주점 180° 값은 맞지만, 실물 인지는 왜곡 편 영상 (newK fx 601.2 · fy 792.0 · cx 529.6) 을 쓰므로 Gazebo K 도 그 값이어야 화면 방향이 맞음 (실물 +10.4° vs PR +3.7°). Gazebo 에서 표지 3 개 0.6 px 안 확인 | **반영 안 됨** — dev 의 Gazebo 값은 PR 의 K (cx 584.12) 그대로. 본인이 [#145](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/145) 를 닫으며 '넣지 못한 것' 으로 남김 |
| [#174](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/174) (온창범) — [#172 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/172#issuecomment-6757717891) | 칸 경계 퍽이 영상마다 칸이 바뀌어 같은 ID 가 "중복" 으로 판정됨. 재현 시험 코드 첨부 | **반영** — 이홍주 [#222](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/222) (한 장소 같은 ID 합침) |
| [#175](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/175) (온창범) — 같은 댓글 | `coarse_distance` 주석의 상한 0.48 은 Nav2 허용 오차를 넣으면 0.47 | **반영 안 됨** — `mission.yaml` 주석 0.48 그대로 |
| [#171](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/171) (온창범) — 같은 댓글 | 추적 중 새 퍽 태그가 다음 전체 검색까지 안 나옴 (모드 바뀐 첫 장에 reset 제안), CPU 측정에 Nav2 포함 여부 | 답 · 반영 기록 없음 |
| [#180](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/180) · [#182](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/182) (온창범) | 문제 없음 + 선택: 줄여 · 키워 다시 찾기 단위 시험 추가, launch 사용법 주석 · 두 파일 기본값 중복 | 선택 항목은 반영 기록 없음 (시험 파일 · 주석 그대로) |
| [#181](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/181) (온창범) | 문제 없음. 노드 없이 높이 잡음 모의를 따로 돌려 판정 기준 확인 | 승인 성격 |
| [#301](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/301) (이창엽) | 병합 찬성 (tick 환산 · 펌웨어 범위 · 옛 값 남은 곳 · colcon 230 개 확인). 부탁: ① PR 제목 · 본문을 최종 값으로 ② 집게 1418 tick 이 열림 끝보다 9.9° 더 열림 → 닫힘 · 열림 끝 실측 ③ 켤 때 펴기 틈 −0.4 ~ +1.4 mm 지켜보기 | ① **반영 안 됨** (제목이 첫 커밋 값 J1 −1.287 그대로). ② · ③ 답 기록 없음 |
| [#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144) (이창엽 테스트 브랜치) — [#144 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144#issuecomment-6732687724) | 켜질 때 정리: 허용 여유 0.3 → 0.1 rad, 관절 공간 직선 이동 중 팔끼리 닿을 수 있음. 이후 목표를 STOW → DRIVE, 순서 3 → 2 → 1 → 4, 0.1 rad/s 로 정리 ([#144 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144#issuecomment-6761714359)) | 목표 · 순서 · 속도는 **반영** ([#189](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/189), 1.4 받은 리뷰). 여유 값 반영 여부는 확인 못 함 |

### 4.5 설명 (전체 흐름 이해)

| 근거 | 내가 맡지 않은 부분 | 무엇을 이해 · 판단했나 |
|---|---|---|
| [#183 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/183#issuecomment-6761699897) | 온창범 S1 시험 결과 | 조종 끊김 0.601 s 가 정상 범위인 이유를 `opencr_hardware.cpp` 줄 번호 (잠금 판정 100 Hz · `/e_stop` 100 ms 타이머) 로 설명하고 기준 문구를 제안 |
| [#160 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/160#issuecomment-6757676995) · [#154 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/154#issuecomment-6761739407) | mission · 계획기 · drive 정렬 | 놓기 전 접근이 퍽을 찾다 실패하는 원인을 `mission_lines.py` · `drive_node.cpp` 줄로 짚고, Gazebo 시험이 `/drive/align` 을 mock 해서 못 잡는다는 것을 찾음 → 이홍주 [#200](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/200). 팔 카메라 자세 계산을 온창범에게 넘김 |
| [#107 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/107#issuecomment-6756905722) | 팔 처짐 보정 스트리밍 (control) | manipulator 50 Hz → controller_manager 100 Hz → 호스트 → 펌웨어까지 주기를 구간별로 확인, E-stop 리셋 튐 문제를 [#144](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/144) 로 옮김 |
| [#114 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/114#issuecomment-6758215384) · [#113 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/113#issuecomment-6732735798) · [#86 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/86#issuecomment-6732727023) | Nav2 · 주행 정책 · 인터페이스 | footprint 후보 3 개를 Gazebo 미션으로 비교해 볼록 6 각형 채택, 주행 정책은 실기에서 쓰지 않기로 정리, 아무도 쓰지 않는 토픽 삭제 결정 |
| [#49](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/49) · [#165](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/165) 체크 목록 ([#49 댓글](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/issues/49#issuecomment-6757608581)) | 조립 뒤 실물 시험 전체 | 흩어진 이슈 (#107 · #108 · #109 · #114 · #157 · #183) 를 주행 · 팔 실물 체크 목록 두 개로 모음 |
| 문서 | 펌웨어 · 주행 운용 | [`firmware_safety_analysis.md`](../../workspace/docs/firmware_safety_analysis.md), [`opencr_dual_board_v4.md`](../../workspace/docs/opencr_dual_board_v4.md), navigation README "실물 첫 실행" ([#166](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/166)), drive_demo 팀원용 안내 0 ~ 8 장 ([#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306)) |

### 4.6 한계 (저장소 기록으로 본 것)

1. **실물 주행을 계획대로 끝까지 마친 기록이 없다.** 11-15 S6 에서 벽에 닿았고, 11-16 경기 당일 최고는 4/10 구간이며 정지점 5 cm 를 넘는 구간이 많았다 (M-08).
2. 경로 이탈 멈춤 · 실물 0.50 m 후진 · 되찾기는 실물 확인 전에 병합됐다 ([#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306) · [#318](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/318) · [#319](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/319) 본문). 11-16 12:15 에 되찾기가 실물에서 한 번 돌았지만 사람이 Ctrl+C 로 멈췄다 (M-05).
3. Isaac 주행 RL (`drive_rl`) 은 전체 경로 1/58 로 실물에 쓰지 않았다 ([#302](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/302)).
4. 저전압 부저 · `/battery/*` 발행 · 리모컨 응답 시간은 실물로 재지 않았다 ([#207](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/207) · [#257](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/257) 본문).
5. Gazebo 창 모드 Nav2 시작이 7 번 중 3 번 실패하는 문제를 남긴 채 병합했다 ([#241](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/241)). 이후 이홍주 [#307](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/307) 로 이어짐.
6. PR 리뷰에서 낸 부탁 일부 ([#301](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/301) 제목, [#153](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/153) newK, [#175](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/175) 상한) 가 반영되지 않았고, 그 뒤 따라간 기록이 없다.
7. `drive_demo` 는 ROS 패키지 밖 도구이고, 계획을 새로 만드는 도구는 제어 담당 노트북에만 있다 ([#306](https://github.com/developlcy-oss/Lv2_Cartlidar_Assignment/pull/306) 본문).
8. 11-17 장애물 회피 · 실물 SLAM 지도 · 미션 시뮬은 **경기 뒤 시뮬 결과뿐**이고 실물에서 돌려 보지 않았다. 미션 시뮬은 4 대 · 앞 14 단계만 돌렸다 (전체 기준선 아직).
