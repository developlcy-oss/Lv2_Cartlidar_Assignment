"""Gazebo 시험: 출발 자세로 Gazebo → 팔 주행 자세 → (nav:=true) 5 s 뒤 Nav2 + AMCL.

  ros2 launch navigation gazebo_sim.launch.py                    # Gazebo 창 + RViz + Nav2
  ros2 launch navigation gazebo_sim.launch.py gui:=false         # 창 없이 (센서 렌더는 함)
  ros2 launch navigation gazebo_sim.launch.py nav:=false         # Gazebo · 팔 자세만 (Nav2 는 따로)
  ros2 launch navigation gazebo_sim.launch.py x:=1.8108 y:=0.95  # 다른 시작 자세 (AMCL 시작 자세도 같이 바뀜)
  ros2 launch navigation gazebo_sim.launch.py use_scan:=false    # /scan (perception scan_merger) 없이
  ros2 launch navigation gazebo_sim.launch.py use_loc_monitor:=false  # loc_monitor 없이 (기본 켬, loc_map 은 같은 SDF 지도, #133 · #151)
  ros2 launch navigation gazebo_sim.launch.py sim_turn:=0.79,0.70   # 실물형 회전 (꺾기 × 0.79 · 제자리 × 0.70, 아래)
  ros2 launch navigation gazebo_sim.launch.py map:=<localization share>/maps/slam_walls_001.yaml objects_map:=<...>/objects.yaml   # Isaac SLAM 지도로

waffle_4wd_description 은 ros2_ws/src 밖 패키지라 먼저 같이 빌드해야 한다 (README "Gazebo 시험").
Gazebo 는 twist_mux · ros2_control 을 거치지 않는다 → Nav2 를 cmd_vel_out:=/cmd_vel 로 띄운다.
odom: use_ekf:=true (기본) 면 실물처럼 bringup odom_icr + EKF (바퀴 vx · vy + IMU yaw rate) 가 /odometry/filtered 와 odom -> base_link TF 를 냄
  (Gazebo DiffDrive TF 는 끔). Gazebo 바퀴 odom 은 제자리 회전을 실제보다 약 22 % 크게 셈 (스키드 미끄러짐, 2026-11-13 측정) → 바퀴 odom 그대로 쓰면
  AMCL yaw 가 회전마다 어긋나 loc_monitor LOST 가 잦았음 (#151). use_ekf:=false 면 예전처럼 /odom (바퀴) 그대로.
sim_turn:=<꺾기>,<제자리> (기본 빈 값 = 끔): 실물처럼 덜 도는 로봇 흉내 (2026-11-21, tools/drive_demo SIM_TURN 과 같은 값 0.79,0.70).
  scripts/sim_turn_scale.py 가 /cmd_vel_turn 의 회전만 줄여 /cmd_vel (Gazebo 브리지) 로 냄. 켜면 cmd_vel_out 이 기본 /cmd_vel 일 때 Nav2 출력을
  /cmd_vel_turn 으로 돌림. 다른 출력 (cmd_vel_out:=/cmd_vel_nav + twist_mux · watchdog 등) 을 쓰면 /cmd_vel 로 내는 마지막 노드를
  /cmd_vel_turn 으로 remap 할 것 (mission_planner test/gazebo_mission.sh SIM_TURN=). 근사일 뿐 (미끄럼 · 지연 없음), 실물 검증 아님.
팔: Gazebo 관절은 0 (영 자세) 으로 시작하므로 팀 기본 시작 자세 (urdf_and_usd/isaac/robot_params.json poses.drive) 로 보낸다.
Nav2 는 팔 명령을 다 보낸 뒤 5 s 기다렸다 띄운다 (scan_body_mask 가 처음 1.9 s 정지 스캔으로 차체 가림을 배우므로 팔이 멈춘 뒤에).
지도: Gazebo 경기장과 같은 SDF 에서 만든 mission_planner 지도 (/map = map_amcl 스캔면, /map_objects = map_nav 벽 + 상자, 0.025 m).
  localization 기본 (Isaac SLAM slam_walls_001) 은 동쪽으로 갈수록 SDF 와 8 ~ 15 cm 어긋나 AMCL 이 치우치다 위치를 잃었다 (2026-11-04)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

TURN_IN = '/cmd_vel_turn'   # sim_turn 켤 때 scale 노드 입력 (출력 = /cmd_vel, Gazebo 브리지)


def sim_turn_node(context):
    """sim_turn:=<꺾기>,<제자리> → scripts/sim_turn_scale.py (TURN_IN → /cmd_vel). 빈 값이면 아무것도 안 띄움"""
    s = LC('sim_turn').perform(context).strip()
    if not s:
        return []
    try:
        k_curve, k_spin = (float(v) for v in s.split(','))
    except ValueError:
        raise RuntimeError(f'gazebo_sim: sim_turn:={s} 는 <꺾기>,<제자리> 두 숫자여야 함 (예 0.79,0.70)') from None
    if not (0.0 < k_curve <= 1.5 and 0.0 < k_spin <= 1.5):
        raise RuntimeError(f'gazebo_sim: sim_turn:={s} 비율은 0 ~ 1.5')
    return [Node(package='navigation', executable='sim_turn_scale.py', name='sim_turn_scale', output='screen',
                 parameters=[{'use_sim_time': True, 'k_curve': k_curve, 'k_spin': k_spin}],
                 remappings=[('cmd_vel_turn', TURN_IN), ('cmd_vel', '/cmd_vel')])]


ARM_DRIVE = {'joint1': 0.0, 'joint2': -1.35, 'joint3': 1.1497, 'joint4': 0.8094, 'joint5': 0.0, 'gripper_roll': 0.0}  # robot_params.json poses.drive


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('waffle_4wd_description'), 'launch', 'gazebo.launch.py'])),
        launch_arguments={'x': LC('x'), 'y': LC('y'), 'yaw': LC('yaw'), 'gui': LC('gui'), 'rviz': LC('rviz'),
                          'odom_tf': PythonExpression(["'false' if '", LC('use_ekf'), "' == 'true' else 'true'"])}.items())
    # 로봇이 Gazebo 에 생긴 뒤 (/joint_states 가 나오기 시작) 보낸다. 브리지 구독은 그 전부터 있어서 -w 1 만으로는
    # 생성 전에 보낸 명령이 버려진다 (2026-11-04: joint2 가 0 으로 남아 팔이 차체를 뚫음 — 이 모델은 팔 · 차체 자기 충돌 없음)
    # echo 에 메시지 형을 줘야 /joint_states 가 아직 없을 때 기다린다. 형이 없으면 "Could not determine the type" 로 바로 끝나
    # 팔 자세를 하나도 안 보냄 (2026-11-12 Gazebo 창 실행: 창을 띄우면 브리지가 늦게 떠서 남)
    arm = ExecuteProcess(
        cmd=['bash', '-c', 'ros2 topic echo --once /joint_states sensor_msgs/msg/JointState > /dev/null && sleep 1 && ' + ' && '.join(
            f'ros2 topic pub --times 3 -r 2 -w 1 /arm/{j}/cmd_pos std_msgs/msg/Float64 "{{data: {v}}}"' for j, v in ARM_DRIVE.items())],
        output='screen')
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('navigation'), 'launch', 'navigation.launch.py'])),
        # sim_turn 을 켜고 cmd_vel_out 이 기본 (/cmd_vel) 이면 Nav2 출력을 scale 노드 입력으로
        launch_arguments={'use_sim_time': 'true',
                          'cmd_vel_out': PythonExpression(["'", TURN_IN, "' if '", LC('sim_turn'), "'.strip() and '", LC('cmd_vel_out'),
                                                           "' == '/cmd_vel' else '", LC('cmd_vel_out'), "'"]),
                          'odom_topic': PythonExpression(["'/odometry/filtered' if '", LC('use_ekf'), "' == 'true' else '/odom'"]),
                          'use_loc_monitor': LC('use_loc_monitor'), 'map': LC('map'), 'objects_map': LC('objects_map'),
                          'loc_map': LC('loc_map'),
                          'start_x': LC('x'), 'start_y': LC('y'), 'start_yaw': LC('yaw')}.items(),
        condition=IfCondition(LC('nav')))
    # 실물 bringup ⑥ 과 같은 odom (#92 · #151): /odom (바퀴) + IMU ω → /odom_icr (vy = −icr_x·ω) → EKF (회전은 IMU 에서만)
    sim = {'use_sim_time': True}
    ekf = [Node(package='bringup', executable='odom_icr', name='odom_icr', output='screen',
                parameters=[PathJoinSubstitution([FindPackageShare('control'), 'config', 'robot_geometry.yaml']), sim],
                condition=IfCondition(LC('use_ekf'))),
           Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
                parameters=[PathJoinSubstitution([FindPackageShare('bringup'), 'config', 'ekf.yaml']), sim],
                condition=IfCondition(LC('use_ekf')))]
    # 실물 perception scan_merger 와 같은 /scan (앞 · 뒤 LDS 합침, base_link, 차체 상자 self_box 지움, lidar.yaml 그대로).
    # drive_node 의 먼 거리 정렬 · 정책 주행 · 후진 확인 · navigate 전 전진 (pullout) 이 /scan 을 본다. Nav2 · AMCL 은 /scan_merged (localization) 그대로
    scan = Node(package='perception', executable='scan_merger', name='scan_merger', output='screen',
                parameters=[PathJoinSubstitution([FindPackageShare('perception'), 'config', 'lidar.yaml']), sim],
                condition=IfCondition(LC('use_scan')))
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.69365', description='시작 base_link x (발제 좌표) = AMCL 시작 자세'),
        DeclareLaunchArgument('y', default_value='0.950'),
        DeclareLaunchArgument('yaw', default_value='3.14159265'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo 창'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('nav', default_value='true', description='Nav2 + AMCL 도 띄움'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([FindPackageShare('mission_planner'), 'maps', 'map_amcl.yaml']),
                              description='/map (AMCL · costmap 벽). SDF 에서 만든 지도 = Gazebo 경기장과 같음'),
        DeclareLaunchArgument('objects_map', default_value=PathJoinSubstitution([FindPackageShare('mission_planner'), 'maps', 'map_nav.yaml']),
                              description='/map_objects (costmap 물체: 벽 + 라이다에 안 보이는 상자 · 받침대)'),
        DeclareLaunchArgument('loc_map', default_value=PathJoinSubstitution([FindPackageShare('mission_planner'), 'maps', 'map_amcl.yaml']),
                              description='loc_monitor 잔차 · 탐색용 벽 지도. /map 과 같은 SDF 지도 (기본 Isaac slam_walls 는 Gazebo 와 어긋남, #133)'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel',
                              description='Nav2 최종 속도 명령. 기본 Gazebo 바로. twist_mux 를 따로 띄워 실물처럼 /localization/lock 을 걸려면 /cmd_vel_nav (#151)'),
        DeclareLaunchArgument('use_loc_monitor', default_value='true',
                              description='loc_monitor (위치 잃음 감지 · 다시 찾기). 기본 켬: use_ekf 로 Gazebo 잔차 p95 0.024 < 문턱 0.034, Q1 LOC=1 LOST 0 (#151). 예전 바퀴 odom 은 LOST 잦아 끔 (#133)'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='실물처럼 odom_icr + EKF (IMU yaw rate) 가 odom TF · /odometry/filtered 를 냄. false = Gazebo 바퀴 odom 그대로 (회전 약 22 % 큼, #151)'),
        DeclareLaunchArgument('use_scan', default_value='true',
                              description='실물처럼 perception scan_merger 가 /scan 을 냄 (drive_node 정렬 · 전진 · 후진 확인용). false = /scan 없음 (예전)'),
        DeclareLaunchArgument('sim_turn', default_value='',
                              description='실물형 회전 <꺾기>,<제자리> (예 0.79,0.70 = tools/drive_demo SIM_TURN). 빈 값 = 끔. '
                                          '켜면 /cmd_vel_turn → (회전만 줄임) → /cmd_vel, cmd_vel_out 기본이면 Nav2 출력도 /cmd_vel_turn'),
        gazebo,
        *ekf,
        OpaqueFunction(function=sim_turn_node),
        scan,
        arm,
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[TimerAction(period=5.0, actions=[nav])])),
    ])
