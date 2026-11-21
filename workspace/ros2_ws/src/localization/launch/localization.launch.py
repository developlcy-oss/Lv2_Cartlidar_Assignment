"""저장 지도 위 위치 추정: map → odom 과 위치 상태(/localization/status · /localization/lock), 주행용 지도(/map · /map_objects).

  ros2 launch localization localization.launch.py [use_sim_time:=true] [localization:=amcl|carto] [use_amcl_node:=true] [gate_in:=<토픽> gate_out:=<토픽>]
                                                  [loc_guard:=off|shadow|active]

map = 발제 좌표(팀 map, mission_planner 문서 좌표 지도와 같은 원점). odom → base_link 는 팀 EKF(bringup ekf.yaml)가 낸다.
입력: /scan_front · /scan_rear(LaserScan), TF odom → base_link · base_link → laser_*, /odometry/filtered(EKF).
localization:=amcl(기본) — AMCL. AMCL 노드는 주행(Nav2) 쪽 launch 가 띄운다. 이 launch 는 그 입력(/scan_merged · /map)과 감시만 낸다.
  scan_body_mask → scan_merge(/scan_merged). use_amcl_node:=true 면 amcl 도 여기서 띄움(config/amcl.yaml, 단독 시험용).
localization:=carto — Cartographer 순수 위치 추정(기본으로는 안 뜸)
  scan_body_mask(차체 가림 지움 → /scan_*_filtered) → cartographer_node(maps/carto_map.pbstream 고정, map_frame carto_map)
  → carto_start(출발 자세에서 궤적 시작) → /tracked_pose → map_odom_relay(map → odom). 정적 TF map → carto_map(지도 맞춤 값).
loc_monitor(use_loc_monitor, 기본 true): 위치 잃음 감지 · 다시 찾기. 두 방식 모두 지원(다시 넣기만 다름).
  gate_in 을 주면 그 속도 명령을 받아 OK · SUSPECT 가 아닐 때 0 으로 바꿔 gate_out 으로 낸다(기본 none = 게이트 끔, /localization/lock 만 냄).
lifecycle_delay(기본 3.0 s): 지도 서버 · AMCL 을 켜 주는 lifecycle_manager 를 이만큼 늦게 띄움. 노드가 한꺼번에 뜰 때 change_state 응답이
  DDS 에서 버려져("failed to send response … client will not receive response") 지도 서버가 안 켜지던 일(Gazebo 11-12, 10 번 중 2 번).
  navigation.launch.py 와 같은 인자 이름이라 명령 줄에서 한 번 주면 둘 다 같은 값.
loc_guard(localization:=amcl 일 때만, 11-21): AMCL 튐 감시 · 보정(config/loc_guard.yaml). loc_guard:=
  off    — 띄우지 않음(예전과 같음).
  shadow — 기본. 판정 · 기록 · /loc_guard/pose 만 내고 map → odom 은 AMCL 이 그대로 냄(TF 구조 안 바뀜).
  active — AMCL tf_broadcast false 로 덮어쓰고 loc_guard 가 map → odom 을 냄. loc_monitor pose_topic 은 /loc_guard/pose.
           AMCL 을 이 launch 가 띄울 때(use_amcl_node:=true)만 허용(다른 launch 의 AMCL 은 tf_broadcast 를 끌 수 없어 map → odom 이 둘이 됨).
  노드는 respawn(죽으면 다시 뜸). loc_monitor 가 켜져 있으면 /initialpose 는 loc_monitor 만 냄(loc_guard reinit_enabled false)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, EqualsSubstitution, LaunchConfiguration as LC, NotSubstitution, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    me = FindPackageShare('localization')
    cfg = PathJoinSubstitution([me, 'config'])
    maps = lambda f: PathJoinSubstitution([me, 'maps', f])
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    carto = IfCondition(EqualsSubstitution(LC('localization'), 'carto'))
    amcl = IfCondition(EqualsSubstitution(LC('localization'), 'amcl'))
    amcl_node = IfCondition(AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), LC('use_amcl_node')))
    align = {'align_x': ParameterValue(LC('align_x'), value_type=float), 'align_y': ParameterValue(LC('align_y'), value_type=float),
             'align_yaw': ParameterValue(LC('align_yaw'), value_type=float)}
    # loc_guard (off | shadow | active). active 일 때만 AMCL tf_broadcast false · loc_monitor 입력을 /loc_guard/pose 로
    guard_on = IfCondition(AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), NotSubstitution(EqualsSubstitution(LC('loc_guard'), 'off'))))
    guard_active = PythonExpression(["'", LC('localization'), "' == 'amcl' and '", LC('loc_guard'), "' == 'active'"])
    start = {'start_x': ParameterValue(LC('start_x'), value_type=float), 'start_y': ParameterValue(LC('start_y'), value_type=float),
             'start_yaw': ParameterValue(LC('start_yaw'), value_type=float)}

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('localization', default_value='amcl', description='amcl(기본) | carto'),
        DeclareLaunchArgument('use_amcl_node', default_value='false',
                              description='true 면 amcl 노드도 여기서 띄움(단독 시험용). 기본은 주행(Nav2) 쪽 launch 가 띄움'),
        DeclareLaunchArgument('use_loc_monitor', default_value='true', description='위치 잃음 감지 · 다시 찾기'),
        DeclareLaunchArgument('use_map_server', default_value='true', description='/map · /map_objects 발행(주행 costmap 정적 층)'),
        DeclareLaunchArgument('map', default_value=maps('slam_walls_001.yaml'), description='/map — SLAM 벽 지도(발제 좌표, 0.01 m)'),
        DeclareLaunchArgument('objects_map', default_value=maps('objects.yaml'), description='/map_objects — 라이다에 안 보이는 고정 물체(발제 배치, 0.01 m)'),
        DeclareLaunchArgument('loc_map', default_value=maps('slam_walls.yaml'), description='loc_monitor 잔차 · 탐색용 벽 지도(0.025 m)'),
        DeclareLaunchArgument('carto_state', default_value=maps('carto_map.pbstream'), description='Cartographer 저장 지도'),
        DeclareLaunchArgument('carto_config_dir', default_value=cfg),
        DeclareLaunchArgument('carto_config', default_value='carto_localization.lua'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='Cartographer odometry 입력(팀 EKF 출력)'),
        DeclareLaunchArgument('align_x', default_value='0.729', description='map(발제 좌표) ← carto_map 지도 맞춤 값(SLAM 지도 저장 때 잼)'),
        DeclareLaunchArgument('align_y', default_value='0.960'),
        DeclareLaunchArgument('align_yaw', default_value='3.143774'),
        DeclareLaunchArgument('start_x', default_value='0.69365', description='출발 자세(base_link, 발제 좌표) = 출발 구역 가운데 0.734 − 0.04035 (B안 외곽 가운데를 구역 가운데에, −X 봄). '
                                          'carto 궤적 시작 · use_amcl_node 때 AMCL 초기 자세'),
        DeclareLaunchArgument('start_y', default_value='0.950'),
        DeclareLaunchArgument('start_yaw', default_value='3.14159265'),
        DeclareLaunchArgument('future_dt', default_value='0.1', description='map_odom_relay 가 map → odom 을 앞당겨 낼 시간[s]'),
        DeclareLaunchArgument('loc_cpp', default_value='true', description='차체 가림 · 스캔 합치기 · loc_monitor(amcl) 를 C++ 판(perception)으로 (false = 예전 Python 판)'),
        DeclareLaunchArgument('mask_file', default_value='none', description='차체 가림 칸 yaml(none = 출발 전 0.5~1.9 s 정지 스캔에서 배움)'),
        DeclareLaunchArgument('amcl_params', default_value=PathJoinSubstitution([cfg, 'amcl.yaml'])),
        DeclareLaunchArgument('loc_res_mode', default_value='interp_p75'),
        DeclareLaunchArgument('loc_res_thr', default_value='0.034', description='잔차 문턱[m](interp_p75 기준, 모의에서 정함)'),
        DeclareLaunchArgument('loc_check_mode', default_value='pair'),
        DeclareLaunchArgument('loc_event_log', default_value='none', description='loc_monitor 사건 기록 jsonl(none = 안 씀)'),
        DeclareLaunchArgument('gate_in', default_value='none', description='loc_monitor 명령 게이트 입력(none = 끔)'),
        DeclareLaunchArgument('gate_out', default_value='/cmd_vel_nav'),
        DeclareLaunchArgument('loc_guard', default_value='shadow',
                              description='AMCL 튐 감시 · 보정: off | shadow(기본, 기록만 · TF 는 AMCL) | active(loc_guard 가 map → odom, AMCL tf_broadcast 끔)'),
        DeclareLaunchArgument('loc_guard_params', default_value=PathJoinSubstitution([cfg, 'loc_guard.yaml'])),
        DeclareLaunchArgument('lifecycle_delay', default_value='3.0',
                              description='지도 서버 · AMCL 의 lifecycle_manager 를 이만큼 늦게 시작[s](시작 때 DDS 응답 유실 줄이기, 0 = 바로)'),
    ]
    # lifecycle_delay 는 여기서(포함 범위 안에서) 숫자로 읽어 둠. TimerAction 에 LaunchConfiguration 을 그대로 주면 나중에 읽어
    # 다른 launch 에 포함됐을 때 "launch configuration 'lifecycle_delay' does not exist" 로 타이머가 안 터짐(Gazebo 11-12)
    delay = lambda node: OpaqueFunction(function=lambda context: [
        TimerAction(period=float(LC('lifecycle_delay').perform(context)), actions=[node])])
    map_servers = ['map_server', 'map_objects_server']
    loc_params = {'backend': LC('localization'), 'map': LC('loc_map'), 'mission_frame': 'map',
                  'carto_config_dir': LC('carto_config_dir'), 'carto_config': LC('carto_config'),
                  'res_mode': LC('loc_res_mode'), 'res_thr': ParameterValue(LC('loc_res_thr'), value_type=float),
                  'check_mode': LC('loc_check_mode'), 'event_log': LC('loc_event_log'),
                  'gate_in': LC('gate_in'), 'gate_out': LC('gate_out'),
                  # 빈 값 = amcl /amcl_pose(carto /tracked_pose). loc_guard active 면 보정한 자세를 감시
                  'pose_topic': ParameterValue(PythonExpression(["'/loc_guard/pose' if ", guard_active, " else ''"]), value_type=str)}

    def check_guard(context):
        mode = LC('loc_guard').perform(context)
        if mode not in ('off', 'shadow', 'active'):
            raise RuntimeError(f'loc_guard 는 off | shadow | active: {mode}')
        if mode == 'active' and LC('localization').perform(context) == 'amcl' and LC('use_amcl_node').perform(context).lower() != 'true':
            raise RuntimeError('loc_guard:=active 는 use_amcl_node:=true 일 때만(다른 launch 의 AMCL 이 map → odom 을 같이 냄). '
                               '되돌리기 = loc_guard:=shadow 또는 off')
        return []

    return LaunchDescription(args + [
        OpaqueFunction(function=check_guard),
        # 차체 가림 지우기 · AMCL 용 스캔 합치기는 C++ 판(perception 패키지)이 기본. loc_cpp:=false 면 예전 Python 판
        #   (Jetson 실물 11-13: Python 판 CPU 코어 하나의 약 18 % · 22 %, 11-16 PC 재생(11-14 bag)에서 상태 · lock · 다시 찾기 · 차체 가림 스캔 같음)
        Node(package='perception', executable='scan_body_mask', name='scan_body_mask', output='screen', condition=IfCondition(LC('loc_cpp')),
             parameters=[sim, {'mask_file': LC('mask_file')}]),
        Node(package='localization', executable='scan_body_mask', name='scan_body_mask', output='screen', condition=UnlessCondition(LC('loc_cpp')),
             parameters=[sim, {'mask_file': LC('mask_file')}]),
        # 주행용 지도
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', condition=IfCondition(LC('use_map_server')),
             parameters=[sim, {'yaml_filename': LC('map'), 'topic_name': 'map', 'frame_id': 'map'}]),
        Node(package='nav2_map_server', executable='map_server', name='map_objects_server', output='screen', condition=IfCondition(LC('use_map_server')),
             parameters=[sim, {'yaml_filename': LC('objects_map'), 'topic_name': 'map_objects', 'frame_id': 'map'}]),
        delay(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_maps', output='screen',
                   condition=IfCondition(LC('use_map_server')), parameters=[sim, {'autostart': True, 'node_names': map_servers}])),
        # Cartographer
        Node(package='cartographer_ros', executable='cartographer_node', name='cartographer_node', output='screen', parameters=[sim], condition=carto,
             arguments=['-configuration_directory', LC('carto_config_dir'), '-configuration_basename', LC('carto_config'),
                        '-load_state_filename', LC('carto_state'), '-load_frozen_state=true', '-start_trajectory_with_default_topics=false'],
             remappings=[('scan_1', '/scan_front_filtered'), ('scan_2', '/scan_rear_filtered'), ('odom', LC('odom_topic'))]),
        Node(package='localization', executable='carto_start', name='carto_start', output='screen', condition=carto,
             parameters=[sim, align, start, {'carto_config_dir': LC('carto_config_dir'), 'carto_config': LC('carto_config')}]),
        Node(package='localization', executable='map_odom_relay', name='map_odom_relay', output='screen', condition=carto,
             parameters=[sim, align, {'map_frame': 'map', 'future_dt': ParameterValue(LC('future_dt'), value_type=float)}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='carto_map_tf', output='screen', parameters=[sim], condition=carto,
             arguments=['--x', LC('align_x'), '--y', LC('align_y'), '--yaw', LC('align_yaw'), '--frame-id', 'map', '--child-frame-id', 'carto_map']),
        # AMCL
        Node(package='perception', executable='loc_scan_merge', name='scan_merge', output='screen', parameters=[sim],
             condition=IfCondition(AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), LC('loc_cpp')))),
        Node(package='localization', executable='scan_merge', name='scan_merge', output='screen', parameters=[sim],
             condition=IfCondition(AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), NotSubstitution(LC('loc_cpp'))))),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen', condition=amcl_node,
             parameters=[LC('amcl_params'), sim, {'initial_pose.x': ParameterValue(LC('start_x'), value_type=float),
                                                  'initial_pose.y': ParameterValue(LC('start_y'), value_type=float),
                                                  'initial_pose.yaw': ParameterValue(LC('start_yaw'), value_type=float),
                                                  # loc_guard active 면 map → odom 은 loc_guard 가 냄
                                                  'tf_broadcast': ParameterValue(PythonExpression(['not (', guard_active, ')']), value_type=bool)}]),
        delay(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_amcl', output='screen',
                   condition=amcl_node, parameters=[sim, {'autostart': True, 'node_names': ['amcl']}])),
        # 위치 잃음 감지 · 다시 찾기
        #   amcl 이고 loc_cpp 면 C++ 판(perception loc_monitor, 같은 인자 · 같은 동작, Jetson 실물 11-14 Python 판 CPU 코어 하나의 20~25 %).
        #   carto 는 Python 판만 있음(Jetson 에 cartographer_ros_msgs 없음)
        Node(package='perception', executable='loc_monitor', name='loc_monitor', output='screen',
             condition=IfCondition(AndSubstitution(LC('use_loc_monitor'), AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), LC('loc_cpp')))),
             parameters=[sim, align, loc_params]),
        Node(package='localization', executable='loc_monitor', name='loc_monitor', output='screen',
             condition=IfCondition(AndSubstitution(LC('use_loc_monitor'),
                                                   NotSubstitution(AndSubstitution(EqualsSubstitution(LC('localization'), 'amcl'), LC('loc_cpp'))))),
             parameters=[sim, align, loc_params]),
        # AMCL 튐 감시 · 보정 (loc_guard). 죽으면 다시 띄움(active 면 map → odom 이 끊겨 Nav2 · demo_follow 가 멈추므로)
        Node(package='localization', executable='loc_guard', name='loc_guard', output='screen', condition=guard_on,
             respawn=True, respawn_delay=1.0,
             parameters=[LC('loc_guard_params'), sim,
                         {'mode': LC('loc_guard'), 'map': LC('loc_map'),
                          # /initialpose 이중 발행 막기: loc_monitor 가 켜져 있으면 다시 넣기는 loc_monitor 만
                          'reinit_enabled': ParameterValue(PythonExpression(["'", LC('use_loc_monitor'), "'.lower() != 'true'"]), value_type=bool)}]),
    ])
