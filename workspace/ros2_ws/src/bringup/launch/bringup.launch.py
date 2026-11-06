# 퍽 운반 로봇 전체 실행 (Jetson 프로세스 ① ② ⑥ ⑦ ⑧)
#
#   ros2 launch bringup bringup.launch.py                        # 실물: 하드웨어 + 인식 + 판단 · 제어
#   ros2 launch bringup bringup.launch.py use_hardware:=false    # OpenCR 없이 (판단 · 제어 · 인식만)
#   ros2 launch bringup bringup.launch.py use_perception:=false  # 카메라 없이
#   ros2 launch bringup bringup.launch.py skill_enabled:=true    # ONNX Runtime 빌드에서만 (정책 실행)
#   ros2 launch bringup bringup.launch.py drive:=direct          # 직결 구동 (기본 belt = 증속벨트)
#   ros2 launch bringup bringup.launch.py sim:=true use_sim_time:=true use_teleop:=false use_tfmini:=false   # Isaac Sim (workspace/tools/isaac_sim 브리지)
#   ros2 launch bringup bringup.launch.py front_serial:=<시리얼> rear_serial:=<시리얼>   # D435 두 대 구분 (rs-enumerate-devices -s)
#   use_sensors:=false — 카메라 · 라이다 드라이버를 띄우지 않는다 (sim:=true 면 기본 false: Isaac 브리지가 같은 토픽을 냄)
#   use_tfmini:=false — TFmini(박스 거리) · obstacle_stop 을 띄우지 않고 twist_mux obstacle lock 도 뺀다 (TFmini 가 없는데 lock 이 있으면 늘 잠김)
#
#   미션 전체 (주사위 기억 로봇, plan 모드):
#   ros2 launch bringup bringup.launch.py mission_source:=plan use_localization:=true use_nav2:=true
#     mission_source:=plan — mission_node 에 mission_plan.yaml 을 덧씌우고 mission_planner_node(계획기)도 띄운다 (use_planner 기본 = plan 일 때)
#                            측량 결과 = 계획기 등록표가 같은 파일이 되게 pucks_file 하나로 둘 다 준다 (mission survey.pucks_out = planner pucks_file)
#                            mission/mission_bringup.launch.py 를 같이 띄우지 말 것 (mission_node 가 둘이 됨)
#     use_localization:=true — localization.launch.py (지도 · scan 합치기 · loc_monitor). use_loc_lock · use_loc_gate 기본도 이것을 따른다
#     use_nav2:=true        — navigation/navigation.launch.py (Smac 2D · RotationShim + MPPI → /cmd_vel_nav) 를 use_localization:=false 로.
#                            위치 추정은 위 localization 하나만 (둘 다 띄우면 /map · loc_monitor 가 두 번, #116). use_amcl_node 기본 = use_nav2
#     지도: map:= objects_map:= loc_map:= 를 이 launch 에 주면 localization 까지 그대로 간다. Isaac · Gazebo 경기장 (발제 SDF) 은
#           mission_planner/maps (map_amcl · map_nav) 를 써야 함 — 기본 SLAM 지도는 SDF 와 7 ~ 15 cm 다름 (#116). AMCL 과 loc_monitor 는 같은 지도로
#
# ① ros2_control_node · ② base_container : opencr_hardware/base_bringup.launch.py
# ⑦ perception_container                  : perception/perception.launch.py
# ⑧ control_container (component_container_mt) — MissionNode(mission) · DriveNode · ManipulatorNode(control) ·
#    TFminiNode · ObstacleStopNode(perception, use_tfmini) — 1층 TFmini 박스 거리 → drive FINE 정렬 · 비상 정지(twist_mux lock).
#    카메라 인식(⑦)을 끄더라도 안전 정지가 남도록 TFmini 와 같은 프로세스에 둔다
#    같은 프로세스 · intra-process 통신 · 노드마다 MutuallyExclusive 콜백 그룹 → 바퀴 · 팔 · 판단 콜백이 병렬
# ⑥ ekf (use_ekf) — 바퀴 vx + 회전 중심 옆 이동 vy (odom_icr, #92) + IMU yaw rate
# 센서 드라이버 (use_sensors) : bringup/sensors.launch.py — D435 두 대 · 팔 Arducam · LDS-03 두 대 (TFmini 는 위 ⑧)
# 위치 추정 (use_localization) : localization/localization.launch.py · 계획기 (use_planner) : mission_planner/planner.launch.py
# 주행 (use_nav2) : navigation/navigation.launch.py (마지막 출력만 /cmd_vel_nav → twist_mux)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw = FindPackageShare('opencr_hardware')
    per = FindPackageShare('perception')
    ctl = FindPackageShare('control')
    mis = FindPackageShare('mission')
    pol = FindPackageShare('robot_policy')
    me = FindPackageShare('bringup')
    loc = FindPackageShare('localization')
    mpl = FindPackageShare('mission_planner')
    nav = FindPackageShare('navigation')
    ipc = [{'use_intra_process_comms': True}]
    # 모든 노드가 같은 시계를 쓰게: 시뮬레이션이면 /clock (영상 · 라이다 stamp 가 시뮬레이션 시각이라 300 ms 규칙에 필요)
    sim = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    geometry = PathJoinSubstitution([ctl, 'config', 'robot_geometry.yaml'])   # drive · odom_icr 공용 (base.icr_x 한 곳)
    skill = {'skill.enabled':ParameterValue(LaunchConfiguration('skill_enabled'), value_type=bool),
             'skill.policies_file': LaunchConfiguration('policies_file')}
    # 카메라 없이 (use_perception:=false) 띄우면 뒤 D435 도 없음 → drive_node 후진 확인 끔 (켜 두면 값 없음 = 후진 막힘, #84)
    rear_camera = {'scan.rear_camera': ParameterValue(LaunchConfiguration('use_perception'), value_type=bool)}
    # loc_monitor 없이 띄우면 /localization/status 가 없음 → drive navigate 위치 확인 끔 (켜 두면 60 s 기다리다 실패, #121)
    loc_gate = {'navigate.loc_gate.enabled': ParameterValue(LaunchConfiguration('use_loc_gate'), value_type=bool)}

    def control_container(context):
        source = LaunchConfiguration('mission_source').perform(context)
        if source not in ('tag', 'plan'):
            raise RuntimeError(f'mission_source 는 tag 또는 plan: {source}')
        mission_params = [PathJoinSubstitution([mis, 'config', 'mission.yaml']),
                          PathJoinSubstitution([mis, 'config', 'mission_table.yaml'])]
        if source == 'plan':   # mission_bringup.launch.py source:=plan 과 같은 덧씌우기 + 측량 결과 경로 = 계획기 pucks_file
            mission_params += [PathJoinSubstitution([mis, 'config', 'mission_plan.yaml']),
                               {'survey.pucks_out': LaunchConfiguration('pucks_file')}]
        return [ComposableNodeContainer(
        name='control_container', namespace='', package='rclcpp_components',
        executable='component_container_mt', output='screen',
        composable_node_descriptions=[
            ComposableNode(package='mission', plugin='mission::MissionNode', name='mission_node',
                           parameters=mission_params + [sim], extra_arguments=ipc),
            ComposableNode(package='control', plugin='control::DriveNode', name='drive_node',
                           parameters=[geometry, PathJoinSubstitution([ctl, 'config', 'drive.yaml']), skill, rear_camera, loc_gate, sim],
                           extra_arguments=ipc),
            ComposableNode(package='control', plugin='control::ManipulatorNode', name='manipulator_node',
                           parameters=[PathJoinSubstitution([ctl, 'config', 'manipulator.yaml']), skill, sim], extra_arguments=ipc),
            ComposableNode(package='perception', plugin='perception::TFminiNode', name='tfmini_node',
                           parameters=[PathJoinSubstitution([per, 'config', 'tfmini.yaml']), sim], extra_arguments=ipc,
                           condition=IfCondition(LaunchConfiguration('use_tfmini'))),
            ComposableNode(package='perception', plugin='perception::ObstacleStopNode', name='obstacle_stop',
                           parameters=[PathJoinSubstitution([per, 'config', 'lidar.yaml']), sim], extra_arguments=ipc,
                           condition=IfCondition(LaunchConfiguration('use_tfmini'))),
        ])]

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='true', description='① ② OpenCR · 컨트롤러 · twist_mux · 조이스틱'),
        DeclareLaunchArgument('sim', default_value='false',
                              description='① 을 Isaac Sim 브리지에 연결 (TopicBasedSystem). use_sim_time:=true 와 함께'),
        DeclareLaunchArgument('use_teleop', default_value='true', description='조이스틱 (끄면 E-stop heartbeat 검사도 끔)'),
        DeclareLaunchArgument('use_perception', default_value='true', description='⑦ 카메라 3대 · 검출'),
        DeclareLaunchArgument('use_tfmini', default_value='true',
                              description='1층 TFmini(박스 거리) · obstacle_stop · twist_mux obstacle lock. 시뮬레이션 · TFmini 없음은 false'),
        DeclareLaunchArgument('mission_source', default_value='tag',
                              description='tag (미션표 mission_table.yaml) | plan (주사위 기억 로봇: mission_plan.yaml + 계획기)'),
        DeclareLaunchArgument('use_planner',
                              default_value=PythonExpression(["'true' if '", LaunchConfiguration('mission_source'), "' == 'plan' else 'false'"]),
                              description='mission_planner_node (계획기). 기본: mission_source:=plan 이면 true'),
        DeclareLaunchArgument('pucks_file', default_value='/tmp/mission_log/pucks_survey.yaml',
                              description='퍽 등록표: mission 측량 결과 (survey.pucks_out) = 계획기 pucks_file. 측량을 끄면 미리 만든 등록표'),
        DeclareLaunchArgument('planner_mode', default_value='per_location', description='계획기 mode: per_location | single_load'),
        DeclareLaunchArgument('use_localization', default_value='false',
                              description='localization.launch.py (지도 · scan 합치기 · loc_monitor). Nav2 는 따로 (#114)'),
        DeclareLaunchArgument('localization', default_value='amcl', description='위치 추정 방식: amcl | carto (localization.launch.py)'),
        DeclareLaunchArgument('use_nav2', default_value='false',
                              description='navigation.launch.py (Nav2, use_localization:=false 로). 위치 추정은 use_localization 하나'),
        DeclareLaunchArgument('use_amcl_node', default_value=LaunchConfiguration('use_nav2'),
                              description='AMCL 노드를 localization 쪽에서 띄움. 기본 = use_nav2 (Nav2 의 map → odom)'),
        DeclareLaunchArgument('use_loc_lock', default_value=LaunchConfiguration('use_localization'),
                              description='twist_mux 위치 잃음 lock (/localization/lock, #93). 기본 = use_localization'),
        DeclareLaunchArgument('use_loc_gate', default_value=LaunchConfiguration('use_loc_lock'),
                              description='drive navigate 를 /localization/status OK 로 막기 (#121). loc_monitor 가 있을 때만. 기본 = use_loc_lock'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='⑥ robot_localization ekf + odom_icr (#92)'),
        DeclareLaunchArgument('use_sensors',
                              default_value=PythonExpression(["'false' if '", LaunchConfiguration('sim'), "'.lower() == 'true' else 'true'"]),
                              description='카메라 · 라이다 드라이버 (sensors.launch.py). 기본: 실물 true · sim:=true 면 false'),
        DeclareLaunchArgument('front_serial', default_value='', description='★ 앞 D435 시리얼'),
        DeclareLaunchArgument('rear_serial', default_value='', description='★ 뒤 D435 시리얼'),
        DeclareLaunchArgument('ports', default_value='/dev/ttyACM0,/dev/ttyACM1',
                              description='두 OpenCR 후보 포트 (역할은 프로토콜 응답으로 판별)'),
        DeclareLaunchArgument('drive', default_value='belt', description='구동: belt (증속벨트) | direct (직결)'),
        DeclareLaunchArgument('skill_enabled', default_value='false',
                              description='정책 실행 — ONNX Runtime 빌드에서만 true (없으면 거절 → 미션 fallback)'),
        DeclareLaunchArgument('policies_file', default_value=PathJoinSubstitution([pol, 'config', 'policies.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='시뮬레이션 시계(/clock) 사용 — 모든 노드에 전달'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([hw, 'launch', 'base_bringup.launch.py'])),
            launch_arguments={'use_teleop': LaunchConfiguration('use_teleop'), 'ports': LaunchConfiguration('ports'),
                              'drive': LaunchConfiguration('drive'), 'sim': LaunchConfiguration('sim'),
                              'obstacle_lock': LaunchConfiguration('use_tfmini'),
                              'loc_lock': LaunchConfiguration('use_loc_lock')}.items(),
            condition=IfCondition(LaunchConfiguration('use_hardware'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([per, 'launch', 'perception.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'use_obstacle_stop': 'false'}.items(),   # obstacle_stop 은 위 control_container
            condition=IfCondition(LaunchConfiguration('use_perception'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([me, 'launch', 'sensors.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'front_serial': LaunchConfiguration('front_serial'),
                              'rear_serial': LaunchConfiguration('rear_serial')}.items(),
            condition=IfCondition(LaunchConfiguration('use_sensors'))),
        OpaqueFunction(function=control_container),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([loc, 'launch', 'localization.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'localization': LaunchConfiguration('localization'),
                              'use_amcl_node': LaunchConfiguration('use_amcl_node')}.items(),
            condition=IfCondition(LaunchConfiguration('use_localization'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([nav, 'launch', 'navigation.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'use_localization': 'false'}.items(),   # 위치 추정은 위 use_localization 하나만 (#116)
            condition=IfCondition(LaunchConfiguration('use_nav2'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([mpl, 'launch', 'planner.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'pucks_file': LaunchConfiguration('pucks_file'),
                              'mode': LaunchConfiguration('planner_mode')}.items(),
            condition=IfCondition(LaunchConfiguration('use_planner'))),
        # ⑥ /odom + IMU ω → /odom_icr (vy = −icr_x·ω, 스키드 회전 중심 옆 이동, #92) → ekf odom0
        Node(package='bringup', executable='odom_icr.py', name='odom_icr', output='screen',
             parameters=[geometry, sim], condition=IfCondition(LaunchConfiguration('use_ekf'))),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([me, 'config', 'ekf.yaml']), sim],
             condition=IfCondition(LaunchConfiguration('use_ekf'))),
    ])
