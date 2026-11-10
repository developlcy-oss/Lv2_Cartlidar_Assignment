# 노드 · 토픽 · 상태도 (중간 제출)

기준: Integration 브랜치 2026-11-09. 토픽 · 액션의 전체 표 (타입 · QoS) 는 [`workspace/docs/interfaces.md`](../../workspace/docs/interfaces.md), 프레임은 [`frames_tf.md`](../../workspace/docs/frames_tf.md), 노드 설계는 [`robot_nodes.html`](../../workspace/docs/robot_nodes.html).
이 문서는 주사위 기억 로봇 미션 (plan 모드) 에 쓰는 것만 추렸다.

## 1. 노드 구성

| 컨테이너 · 프로세스 | 노드 | 패키지 | 하는 일 |
|---|---|---|---|
| ① ros2_control_node | OpenCRHardware, diff_drive_controller, arm_controller, gripper_controller, joint_state_broadcaster | opencr_hardware | OpenCR 2 대 (주행 ID 1~4 · 팔 ID 11~16) USB, 잠금식 E-stop, /diagnostics |
| 센서 드라이버 | coin_d4_driver ×2, realsense2_camera ×2, arm_camera_node, tfmini_node | bringup `sensors.launch.py`, perception | LiDAR 앞뒤, D435 앞뒤, 팔 Arducam 1280×720, 1층 TFmini |
| ⑦ perception_container | perception_node, scan_merger, rear_obstacle | perception | 신호등 · 바코드 · 퍽 · 장소 태그 · 주사위 칸 · 장소 상자 인식, 스캔 합치기, 뒤 장애물 |
| ⑦ 옆 | puck_survey | perception_survey | 시작 박스 6 개 · 퍽 12 칸 측량 → 퍽 등록표 |
| ⑧ | obstacle_stop | perception | TFmini 박스 거리로 비상 정지 잠금 |
| localization | amcl, map_server, loc_monitor | localization | 문서 좌표 지도 (`map_amcl`) 위 위치 추정, 위치 잃음 판정 · 잠금 |
| nav2 | planner · controller · bt_navigator · collision_monitor … | navigation | 주행 (`map_nav`, 박스 70 mm 포함) |
| ⑧ control_container | drive_node, manipulator_node, mission_node | control, mission | 정렬 · 회전 skill / IK · 집기 · 놓기 skill / 미션 순서 |
| 단독 | mission_planner_node | mission_planner | 신호 · 바코드 · 퍽 → 할당 · 적재 · 경로 계획 |
| 기타 | twist_mux, ekf, odom_icr, robot_state_publisher, joy · teleop | bringup | 속도 명령 우선순위 · 잠금, 오도메트리 융합 |

## 2. 노드 · 토픽 그래프 (미션 경로)

```mermaid
flowchart LR
  subgraph 센서
    LDS[LiDAR 앞 · 뒤]
    D435[D435 앞 · 뒤]
    ARD[팔 Arducam]
    TFM[tfmini_node]
  end
  subgraph 인식
    SM[scan_merger]
    PER[perception_node]
    SUR[puck_survey]
    OBS[obstacle_stop]
  end
  subgraph 위치
    AMCL[amcl + map_server]
    LOC[loc_monitor]
    EKF[ekf · odom_icr]
  end
  subgraph 제어
    MIS[mission_node]
    PLN[mission_planner_node]
    DRV[drive_node]
    MAN[manipulator_node]
    NAV[nav2]
    MUX[twist_mux]
  end
  HW[OpenCRHardware<br/>ros2_control]

  LDS -->|/scan_front · /scan_rear| SM -->|/scan| AMCL & NAV & DRV
  D435 -->|image · depth| PER
  ARD -->|/arm_camera/image_raw| PER
  TFM -->|/tfmini/range| OBS & DRV
  PER -->|/perception/signal · /barcode| MIS & PLN
  PER -->|/perception/pucks| SUR & MAN
  PER -->|/perception/result| MIS & DRV & MAN
  SUR -->|/perception/puck_survey · 퍽 등록표 파일| MIS
  AMCL -->|/tf map→odom| LOC -->|/localization/status| MIS & DRV & SUR
  LOC -->|/localization/lock| MUX
  OBS -->|/perception/obstacle_stop| MUX
  HW -->|/odom · /imu/data| EKF -->|/odometry/filtered| NAV & DRV
  HW -->|/e_stop · /diagnostics| MIS & MUX
  MIS -->|/mission_planner/plan srv| PLN
  MIS -->|/drive/navigate · align · execute_skill| DRV
  MIS -->|/manipulator/pose · pick · execute_skill| MAN
  MIS -->|/perception/set_mode · check_dice srv| PER
  MIS -->|/perception/snapshot| PER
  DRV -->|navigate_to_pose| NAV -->|/cmd_vel_nav| MUX
  DRV -->|/cmd_vel_drive| MUX -->|/cmd_vel| HW
  MAN -->|follow_joint_trajectory · gripper_cmd| HW
```

명령 주체는 하나: 바퀴 = drive_node (nav2 도 drive 를 거쳐), 팔 · 그리퍼 = manipulator_node. mission_node 는 순서만 정하고 직접 움직이지 않는다.

## 3. 미션 상태도

### 3-1. 실행 상태 (`/mission/status`)

```mermaid
stateDiagram-v2
  [*] --> IDLE
  IDLE --> RUNNING: /mission/start
  RUNNING --> PAUSED: /e_stop · 진단 ERROR · 위치 잃음 max_lost_s 초과
  PAUSED --> RUNNING: 원인 해결 → 리셋 → /mission/start (현재 단계부터)
  RUNNING --> DONE: 마지막 단계 (wait) 끝
  RUNNING --> FAILED: 점멸 MISMATCH · 미등록 바코드 · 풀 수 없는 문제 (재시도 없음)<br/>단계 실패가 max_retries 초과
  RUNNING --> IDLE: /mission/stop
  PAUSED --> IDLE: /mission/stop
  DONE --> [*]
  FAILED --> [*]
```

### 3-2. 미션 단계 (plan 모드, 발제 05 쪽 ① ~ ⑧)

```mermaid
stateDiagram-v2
  direction LR
  state "observe<br/>바코드 (팔 카메라 barcode_look)<br/>+ 신호 12 회 (앞 D435)" as OBS
  state "survey<br/>박스 6 곳 퍽 측량 → 퍽 등록표" as SUR
  state "plan<br/>횟수 · 그룹 · 할당 · 경로" as PLAN
  state "운반 (박스마다)<br/>goto → approach → [turn] → target_look → toppick_tilt → pallet_load" as PICK
  state "놓기 (장소마다)<br/>goto → approach → measure_box → pallet_unload → place_L" as PLACE
  state "verify<br/>L1 · L2 · L3 칸 + 장소 태그 새 관측" as VER
  state "복귀<br/>goto 출발 구역 → wait 3" as RET
  [*] --> OBS
  OBS --> SUR: CONFIRMED + READ
  OBS --> FAIL: MISMATCH · 미등록 (안전 정지)
  SUR --> PLAN: 12 칸 확정
  SUR --> SUR: revisit (문제 칸만 각도 · 거리 바꿔)
  PLAN --> PICK
  PLAN --> FAIL: 풀 수 없음
  PICK --> PICK: 다음 박스
  PICK --> PLACE: 그 장소 퍽을 다 실음 (per_location)
  PLACE --> PICK: 다음 장소
  PLACE --> VER: 세 장소 끝
  VER --> RET: 틀려도 기록하고 계속
  RET --> [*]
  FAIL --> [*]
```

단계 실패 처리: 실패 · 타임아웃이면 진행 중 goal 을 모두 취소하고 그 단계를 `max_retries` 번 다시 한다. `toppick_tilt` 실패는 바로 앞 `target_look` 부터 (#69).

### 3-3. 위치 상태 (`/localization/status`, loc_monitor)

```mermaid
stateDiagram-v2
  direction LR
  OK --> SUSPECT: 스캔 잔차 나쁨 suspect_s 이상
  SUSPECT --> OK: 잔차 좋음
  OK --> LOST: 잔차 lost_s · 자세 jump · 추정 gap · 시작 뒤 무응답
  SUSPECT --> LOST
  LOST --> RECOVERING: 재위치 찾기 시작
  RECOVERING --> CONFIRM: 후보 자세로 다시 넣음
  RECOVERING --> FAILED: max_recover_s 초과
  CONFIRM --> OK: ok_confirm_s 동안 잔차 좋음
  CONFIRM --> LOST: confirm_timeout
```

OK · SUSPECT 가 아니면 `/localization/lock` 으로 Nav2 속도만 막고, mission 은 Nav2 단계를 기다린다 (시간에 세지 않음). Gazebo 에서는 회전 중 거짓 LOST 가 있어 기본으로 끔 (#151).

## 4. 기록 (채점 근거)

| 파일 | 만드는 곳 | 내용 |
|---|---|---|
| `signal.json` · `allocation.json` · `plan.json` · `mission_lines.txt` | 계획기 `log_dir` = `<log_dir>/planner`, plan 성공 뒤 에피소드 폴더 `planner/` 로 사본 | 관측한 색 12 개 · 횟수 · 바코드, 그룹 · 할당, 계획 단계 |
| `mission_log.jsonl` | 미션마다 `<log_dir>/<YYMMDD_HHMM>/` (bringup 기본 `~/Lv2_Cartlidar_Assignment/mission_log`) | observe · survey · plan · verify (칸 판정 · 칸별 퍽 ID · 위치) · verify_ids (ID 중복 · 누락) · snapshot · copy · finish |
| `pucks_survey.yaml` | `<log_dir>/pucks_survey.yaml` (mission_node survey_finish), survey 성공 뒤 에피소드 폴더에 사본 | 실제 측량한 퍽 등록표 → 계획기 입력 |
| 이동 전후 이미지 | 에피소드 폴더 `images/`, 팔 카메라 사건당 3 장 (`/perception/snapshot`, perception 이 READ_TAG · ARM_ALIGN 모드일 때) | 이동 전 `NN_before_r<회차>_B<박스>_<k>.jpg` (survey_view), 이동 후 `NN_after_<L>_<k>.jpg` (verify). 전체 영상은 따로 |
