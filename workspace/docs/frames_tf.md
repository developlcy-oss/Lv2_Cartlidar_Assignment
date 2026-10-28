# 좌표 (TF) · 카메라 연결 규약

실물과 Isaac Sim 이 같은 프레임 이름 · 토픽 · 축 규약을 쓰게 하는 기준이다 (2026-10-28, tutor-s-robot `docs/CAMERA_AND_TF.md` 형식 참고).
URDF 는 `opencr_hardware/urdf/robot.urdf.xacro`, 시뮬레이션 USD 참고본은 `~/Downloads/urdf_test/waffle_4wd_usd_261027_1711/waffle_4wd/waffle_4wd.usda`.

## 축

- `base_link`: +X 앞 · +Y 왼쪽 · +Z 위 (ROS REP 103). 원점 = 하판 윗면 중심, 바닥 위 56.25 mm (urdfv2 와 같음).
- 광학 프레임 (`*_optical_frame`): +X 영상 오른쪽 · +Y 영상 아래 · +Z 앞 (보는 방향). 몸체 프레임 기준 rpy (-π/2, 0, -π/2).
- USD 참고본은 base_link 원점이 다르다 (base_footprint 위 6.75 mm). **위치값은 그대로 옮기지 말고** 이름 · 축 · 장착 방향만 참고한다.

## 프레임

| 프레임 | 부모 | 발행 | 쓰는 곳 | 상태 |
|---|---|---|---|---|
| `map` → `odom` | — | slam_toolbox | nav2 | |
| `odom` → `base_link` | — | **ekf 만** (diff_drive `enable_odom_tf: false`) | perception · manipulator (`fixed_frame`) | |
| `base_footprint` | base_link | robot_state_publisher (URDF) | perception 찾기 · 바코드 (`base_frame`) | |
| `imu_link` | base_link | URDF | imu_sensor_broadcaster · ekf | 뒤집어 장착 반영 (roll π) |
| `laser_front` · `laser_rear` | base_link | URDF | LiDAR 드라이버 frame_id · slam_toolbox | ★ roll π 미적용 |
| `camera_front_link` · `camera_rear_link` | base_link | URDF | — | ★ C: 이름 불일치 |
| `top_camera_front_link` → `top_camera_front_color_optical_frame` (뒤도 같음) | (URDF 에 없음) | realsense2_camera (`<camera_name>_link` 아래만) | perception D435 | ★ C: base_link 와 끊김 |
| `arm_base_link` → `link1` ~ `gripper_roll_link` | base_link | URDF + /joint_states | manipulator (`arm_base_frame`) | |
| `arm_camera_link` → `arm_camera_optical_frame` | gripper_roll_link | URDF | Arducam UVC 드라이버 frame_id · perception 태그 | 그리퍼 장착 확인 (2026-10-28) |
| `tfmini_link` | gripper_roll_link | URDF | manipulator (`tfmini_frame`) | ★ 차체 1층 플레이트로 옮길 것 |
| `tool_link` | gripper_roll_link | URDF | manipulator | |

## 적용할 것 (주석으로 남김)

| 항목 | 내용 | 위치 |
|---|---|---|
| C | realsense 의 TF 뿌리는 `<camera_name>_link` (`top_camera_front_link` · `top_camera_rear_link`, 왼쪽 IR = 깊이 원점). URDF 는 `camera_front_link` · `camera_rear_link` 라 `base_link → top_camera_*_color_optical_frame` 이 끊겨 perception `tf_->fixed()` 가 실패한다. USD 참고본은 `top_camera_*_link` | robot.urdf.xacro, test_robot_description.cpp |
| LiDAR roll π | LDS-03 뒤집어 장착 (USD `lidar_*_scan` roll 180°, urdfv2 `lds03_*` 같음). 미적용이면 스캔 좌우가 뒤집힌다 | robot.urdf.xacro |
| TFmini | 차체 1층 플레이트 장착. urdfv2 `tfmini_joint` = base_link 기준 (0.132, 0, -0.02125). 옮기면 manipulator pick ① `range_offset` 가정을 다시 본다 | robot.urdf.xacro, manipulator.yaml, test_robot_description.cpp |
| A | manipulator STOW joint3 -2.3 이 펌웨어 범위 밖 → STOW 명령 거부 | manipulator.yaml |
| B | manipulator `lower/upper_limits` 가 URDF · config.h 보다 넓음 | manipulator.yaml |

## TF 발행 원칙

- 한 변환은 한 노드만 발행한다. `odom → base_link` 는 ekf, `map → odom` 은 slam_toolbox.
- realsense2_camera 는 `<camera_name>_link` 아래 (`_color_optical_frame` 등) 만 발행한다. URDF 는 그 아래 프레임을 만들지 않는다 (시험이 확인).
- 시뮬레이션의 정답 자세 (ground truth) 는 `/tf` 에 넣지 않는다. `sim_world` 프레임이나 별도 토픽으로 기록해 추정값과 비교한다.
- Isaac 은 `/joint_states` · `/clock` 을 발행하고, 로봇 TF 는 실물과 같게 robot_state_publisher 가 URDF 로 만든다.

## Isaac Sim 발행 규약

| 토픽 | 타입 | frame_id |
|---|---|---|
| `/top_camera_front/color/image_raw` · `/color/camera_info` | Image · CameraInfo | `top_camera_front_color_optical_frame` |
| `/top_camera_front/aligned_depth_to_color/image_raw` | Image (컬러와 같은 크기) | `top_camera_front_color_optical_frame` |
| `/top_camera_rear/...` | 위와 같음 | `top_camera_rear_color_optical_frame` |
| `/arm_camera/image_raw` · `/arm_camera/camera_info` | Image · CameraInfo | `arm_camera_optical_frame` |
| `/clock` | rosgraph_msgs/Clock | — |
| `/joint_states` | sensor_msgs/JointState | — |

- 토픽 · frame_id 는 실물 드라이버와 같은 이름 (perception.yaml 을 바꾸지 않고 쓰기 위해).
- CameraInfo 는 렌더러 값으로 계산한다: fx = 폭 × focalLength / horizontalAperture, fy = 높이 × focalLength / verticalAperture. D435 컬러 화각은 69° × 42° (drive_view.py). 실물 이전 때는 realsense 의 CameraInfo 를 쓴다.
- 영상 · 깊이 · CameraInfo 는 같은 시뮬레이션 스텝의 stamp (perception `depth.sync_tol_s` 0.02 s). 노드는 `use_sim_time: true`.
- Kit 뷰포트 창을 더 열지 않는다 (Isaac Sim 6.1 에서 모든 뷰포트가 지오메트리를 못 그림). 카메라는 RenderProduct / Camera 센서로.

## 점검

```bash
ros2 run tf2_ros tf2_echo base_link top_camera_front_color_optical_frame   # C 가 남아 있으면 실패
ros2 run tf2_ros tf2_echo base_link arm_camera_optical_frame
ros2 run tf2_tools view_frames                                            # 부모가 둘 · 끊긴 나무 확인
ros2 topic echo /top_camera_front/color/camera_info --once
ros2 topic hz /top_camera_front/color/image_raw
```

정적 검사:

- URDF: `colcon test --packages-select opencr_hardware` (test_robot_description — 나무 구조 · 프레임 · 광학 축 · 구동 설정).
- USD: `workspace/tools/validate_usd.py <usd>` (usd-core 만 필요, Isaac 없이). 결과 `PASS_STATIC_ONLY` 는 물리 · 렌더링 · ROS 연결을 보지 않았다는 뜻이다.
