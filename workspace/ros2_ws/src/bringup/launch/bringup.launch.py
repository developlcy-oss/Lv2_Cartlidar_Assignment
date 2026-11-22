# 퍽 운반 로봇 전체 실행 (Jetson 프로세스 ① ② ⑥ ⑦ ⑧)
#
#   ros2 launch bringup bringup.launch.py                        # 경기 (실물): 하드웨어 + 인식 + 위치 추정 + Nav2 + 계획기 + 미션
#   ros2 launch bringup bringup.launch.py mode:=drive ...        # 주행 시연 로봇 쪽 (tools/drive_demo/README.md 단계 6): 팀 Nav2 · 계획기 · mission_node 없음
#   ros2 launch bringup bringup.launch.py use_localization:=false use_nav2:=false   # 벤치 시험 (주행 없이 팔 · 인식)
#   ros2 launch bringup bringup.launch.py use_survey:=false pucks_file:=<등록표.yaml>  # 측량 없이 미리 만든 등록표로 계획 (실물 단계 시험)
#   ros2 launch bringup bringup.launch.py use_hardware:=false    # OpenCR 없이 (판단 · 제어 · 인식만)
#   ros2 launch bringup bringup.launch.py use_perception:=false  # 카메라 없이
#   ros2 launch bringup bringup.launch.py skill_enabled:=true    # ONNX Runtime 빌드에서만 (정책 실행)
#   ros2 launch bringup bringup.launch.py drive:=direct          # 직결 구동 (기본 belt = 증속벨트)
#   ros2 launch bringup bringup.launch.py sim:=true use_sim_time:=true use_teleop:=false use_tfmini:=false   # Isaac Sim (workspace/tools/isaac_sim 브리지)
#   ros2 launch bringup bringup.launch.py front_serial:=<시리얼> rear_serial:=<시리얼>   # D435 두 대 구분 (rs-enumerate-devices -s)
#   use_sensors:=false — 카메라 · 라이다 드라이버를 띄우지 않는다 (sim:=true 면 기본 false: Isaac 브리지가 같은 토픽을 냄)
#   max_wz:=<rad/s> — Nav2 회전 한계 (navigation.launch.py max_wz: MPPI wz_max · RotationShim 제자리 회전 min(YAML 0.75, max_wz) · behavior
#                     max_rotational_vel · smoother 를 함께). 기본: 실물 (use_hardware:=true, sim:=false) 0.5 (2026-11-21, tools/drive_demo v2_filter 근거:
#                     실물은 꺾기 79 % · 제자리 66 ~ 80 % 만 돌고, 0.26 + 0.2045·0.5 = 0.362 m/s 로 바깥 바퀴 포화 0.425 대비 여유 15 %),
#                     Isaac (sim:=true) · use_hardware:=false 는 빈 값 = nav2_params.yaml (0.75). YAML 값을 실물에서 쓰려면 max_wz:=0.75
#   use_tfmini:=false — TFmini(박스 거리) · obstacle_stop 을 띄우지 않고 twist_mux obstacle lock 도 뺀다 (TFmini 가 없는데 lock 이 있으면 늘 잠김)
#
#   미션 전체 (주사위 기억 로봇, plan 모드 = 기본). 2026-11-14 부터 use_localization · use_nav2 기본 true (경기 명령 = 인자 없이):
#   ros2 launch bringup bringup.launch.py
#     시작 전 준비 확인 (R3): mission_node 가 1 안전 · 2 컨트롤러 · 3 노드 서버 · 4 위치 추정 · 5 Nav2 를 표로 찍고
#                            /mission/ready = READY 일 때만 시작을 받음. 아래 control_container 가 ready.* 를 넘김
#     mission_source:=plan (기본) — mission_node 에 mission_plan.yaml 을 덧씌우고 mission_planner_node(계획기)도 띄운다 (use_planner 기본 = plan 일 때)
#     mission_source:=tag — 옛 태그 미션표 (mission_table.yaml) 만 불러오고 계획기는 띄우지 않는다
#                            측량 결과 = 계획기 등록표가 같은 파일이 되게 pucks_file 하나로 둘 다 준다 (mission survey.pucks_out = planner pucks_file)
#     log_dir:=<경로>        — 미션 기록 루트 (기본 ~/Lv2_Cartlidar_Assignment/mission_log, 앞 ~ = HOME). 미션마다 <log_dir>/<YYMMDD_HHMM>/
#                            (mission_log.jsonl · images/ 이동 전후 사진 · planner/ 계획기 기록 사본 · 퍽 등록표 사본).
#                            계획기 log_dir = <log_dir>/planner, pucks_file 기본 = <log_dir>/pucks_survey.yaml. 빈 값이면 기록 안 함
#                            mission/mission_bringup.launch.py 를 같이 띄우지 말 것 (mission_node 가 둘이 됨)
#     use_localization:=true — localization.launch.py (지도 · scan 합치기 · loc_monitor). use_loc_lock · use_loc_gate 기본도 이것을 따른다
#     use_nav2:=true        — navigation/navigation.launch.py (Smac 2D · RotationShim + MPPI → /cmd_vel_nav) 를 use_localization:=false 로.
#                            위치 추정은 위 localization 하나만 (둘 다 띄우면 /map · loc_monitor 가 두 번, #116). use_amcl_node 기본 = use_nav2
#   mode:=mission (기본) | drive — 현재 모드를 /system/mode (std_msgs/String, reliable · transient_local, 1 Hz) 로 냄
#     drive: use_nav2 · use_planner 를 false 로 덮어씀 (인자로 true 를 줘도), control_container 에서 mission_node 를 뺌
#            (drive_node · manipulator_node (켤 때 팔을 주행 자세로) · tfmini 는 그대로). use_amcl_node 기본 = use_localization
#            tools/drive_demo/run_demo.sh real 은 /system/mode 가 mission 이면 거부함
#     지도: 기본 = 발제 도면 SDF 지도 mission_planner/maps (map · loc_map = map_amcl, objects_map = map_nav). 실물 · Isaac · Gazebo 공통.
#           localization 기본 SLAM 지도 (slam_walls*, Isaac 미로) 는 SDF 와 7 ~ 15 cm 다름 (#116). 바꿀 때는 map:= objects_map:= loc_map:= 를 함께
#           (AMCL 과 loc_monitor 는 같은 지도로). localization:=carto 면 carto_state 와 같은 세계의 지도를 줄 것
#     loc_guard:=shadow (기본) | active | off — AMCL 튐 감시 · 보정 (localization README). shadow = 기록만, active = loc_guard 가 map → odom (실물에서 이상하면 off)
#
# ① ros2_control_node · ② base_container : opencr_hardware/base_bringup.launch.py
# ⑦ perception_container                  : perception/perception.launch.py
# ⑧ control_container (component_container_mt) — MissionNode(mission) · DriveNode · ManipulatorNode(control) ·
#    TFminiNode · ObstacleStopNode(perception, use_tfmini) — 1층 TFmini 박스 거리 → drive FINE 정렬 · 비상 정지(twist_mux lock).
#    카메라 인식(⑦)을 끄더라도 안전 정지가 남도록 TFmini 와 같은 프로세스에 둔다
#    같은 프로세스 · intra-process 통신 · 노드마다 MutuallyExclusive 콜백 그룹 → 바퀴 · 팔 · 판단 콜백이 병렬
#    split_control:=true (측정용, #265): mission · drive · manipulator · tfmini+obstacle_stop 을 프로세스 넷으로 나눠 노드별 CPU 를 잼
#    (control_mission · control_drive · control_manipulator · control_tfmini). 같은 노드 · 파라미터, intra-process 이득만 없음
# ⑥ ekf (use_ekf) — 바퀴 vx + 회전 중심 옆 이동 vy (odom_icr, #92) + IMU yaw rate
# 센서 드라이버 (use_sensors) : bringup/sensors.launch.py — D435 두 대 · 팔 Arducam · LDS-03 두 대 (TFmini 는 위 ⑧)
# 위치 추정 (use_localization) : localization/localization.launch.py · 계획기 (use_planner) : mission_planner/planner.launch.py
# 주행 (use_nav2) : navigation/navigation.launch.py (마지막 출력만 /cmd_vel_nav → twist_mux)
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
                            SetLaunchConfiguration, TimerAction)
from launch.event_handlers import OnProcessExit
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
    # /diagnostics 끊김 감시 (#207): OpenCRHardware 가 있을 때만 (실물). sim · use_hardware:=false 에는 opencr/* 진단이 없음
    diag = {'diagnostics.timeout': ParameterValue(LaunchConfiguration('diag_timeout'), value_type=float)}
    geometry = PathJoinSubstitution([ctl, 'config', 'robot_geometry.yaml'])   # drive · odom_icr 공용 (base.icr_x 한 곳)
    skill = {'skill.enabled':ParameterValue(LaunchConfiguration('skill_enabled'), value_type=bool),
             'skill.policies_file': LaunchConfiguration('policies_file')}
    # 카메라 없이 (use_perception:=false) 띄우면 뒤 D435 도 없음 → drive_node 후진 확인 끔 (켜 두면 값 없음 = 후진 막힘, #84)
    rear_camera = {'scan.rear_camera': ParameterValue(LaunchConfiguration('use_perception'), value_type=bool)}
    # loc_monitor 없이 띄우면 /localization/status 가 없음 → drive navigate 위치 확인 끔 (켜 두면 60 s 기다리다 실패, #121)
    loc_gate = {'navigate.loc_gate.enabled': ParameterValue(LaunchConfiguration('use_loc_gate'), value_type=bool)}

    # 기록 루트 · 퍽 등록표 경로 (앞 ~ 를 풀어서 mission · 계획기에 같은 값을 넘김)
    def log_paths(context):
        log_dir = os.path.expanduser(LaunchConfiguration('log_dir').perform(context))
        pucks = os.path.expanduser(LaunchConfiguration('pucks_file').perform(context))
        if not pucks:
            if not log_dir:
                raise RuntimeError('pucks_file 이 비었으면 log_dir 이 있어야 함 (등록표 = <log_dir>/pucks_survey.yaml)')
            pucks = os.path.join(log_dir, 'pucks_survey.yaml')
        return log_dir, pucks

    def run_mode(context):
        mode = LaunchConfiguration('mode').perform(context)
        if mode not in ('mission', 'drive'):
            raise RuntimeError(f'mode 는 mission 또는 drive: {mode}')
        return mode

    # mode 확인 · drive 면 팀 Nav2 · 계획기 끔 · /system/mode 발행 (늦게 붙는 구독자도 받게 transient_local, 계속 1 Hz)
    def mode_setup(context):
        mode = run_mode(context)
        acts = []
        if mode == 'drive':   # 인자로 true 를 줘도 끔. use_amcl_node 기본은 선언 때 이미 use_localization 으로 계산됨
            acts += [SetLaunchConfiguration('use_nav2', 'false'), SetLaunchConfiguration('use_planner', 'false')]
        acts.append(ExecuteProcess(name='system_mode', output='log', cmd=[
            'ros2', 'topic', 'pub', '--qos-reliability', 'reliable', '--qos-durability', 'transient_local', '-r', '1',
            '/system/mode', 'std_msgs/msg/String', f'{{data: {mode}}}']))
        return acts

    def control_container(context):
        mission = run_mode(context) == 'mission'   # drive: mission_node 를 띄우지 않음 (미션 파라미터도 만들지 않음)
        source = LaunchConfiguration('mission_source').perform(context)
        if source not in ('tag', 'plan'):
            raise RuntimeError(f'mission_source 는 tag 또는 plan: {source}')
        if mission:
            groups = [('mission', [ComposableNode(package='mission', plugin='mission::MissionNode', name='mission_node',
                                                  parameters=mission_params(context, source) + [diag, sim], extra_arguments=ipc)])]
        else:
            groups = []
        groups += [
            ('drive', [ComposableNode(package='control', plugin='control::DriveNode', name='drive_node',
                                      parameters=[geometry, PathJoinSubstitution([ctl, 'config', 'drive.yaml']), skill, rear_camera,
                                                  loc_gate, sim],
                                      extra_arguments=ipc)]),
            ('manipulator', [ComposableNode(package='control', plugin='control::ManipulatorNode', name='manipulator_node',
                                            parameters=[PathJoinSubstitution([ctl, 'config', 'manipulator.yaml']), skill, sim],
                                            extra_arguments=ipc)]),
            # TFmini 와 비상 정지는 나눌 때도 한 프로세스 (안전 정지가 TFmini 와 같이 살아 있게)
            ('tfmini', [ComposableNode(package='perception', plugin='perception::TFminiNode', name='tfmini_node',
                                       parameters=[PathJoinSubstitution([per, 'config', 'tfmini.yaml']), sim], extra_arguments=ipc,
                                       condition=IfCondition(LaunchConfiguration('use_tfmini'))),
                        ComposableNode(package='perception', plugin='perception::ObstacleStopNode', name='obstacle_stop',
                                       parameters=[PathJoinSubstitution([per, 'config', 'lidar.yaml']), sim], extra_arguments=ipc,
                                       condition=IfCondition(LaunchConfiguration('use_tfmini')))]),
        ]
        container = lambda name, nodes: ComposableNodeContainer(
            name=name, namespace='', package='rclcpp_components', executable='component_container_mt', output='screen',
            composable_node_descriptions=nodes)
        if LaunchConfiguration('split_control').perform(context).lower() == 'true':   # 측정용 (#265): 노드 묶음마다 프로세스 하나 → top · tegrastats 로 노드별 CPU
            return [container(f'control_{g}', nodes) for g, nodes in groups]
        return [container('control_container', [n for _, nodes in groups for n in nodes])]

    def mission_params(context, source):
        # geometry: 차체 기하 base.* (front_offset · icr_x) 를 drive 와 같은 파일에서 (mission 비스듬히 서기 · 시작 박스, 값 한 곳)
        mission_params = [geometry, PathJoinSubstitution([mis, 'config', 'mission.yaml'])]
        if source == 'plan':   # mission_bringup.launch.py source:=plan 과 같은 덧씌우기 + 측량 결과 경로 = 계획기 pucks_file
            log_dir, pucks = log_paths(context)
            survey = LaunchConfiguration('use_survey').perform(context).lower() == 'true'
            if not survey and not LaunchConfiguration('pucks_file').perform(context):
                # 측량을 끄면 계획기가 등록표를 그대로 씀 → 기본 <log_dir>/pucks_survey.yaml (지난 미션 결과) 를 모르고 쓰지 않게
                raise RuntimeError('use_survey:=false 면 pucks_file:=<미리 만든 등록표.yaml> 을 꼭 줄 것 '
                                   '(형식 mission_planner/config/problem_q1_example.yaml)')
            mission_params += [PathJoinSubstitution([mis, 'config', 'mission_plan.yaml']),
                               {'survey.pucks_out': pucks, 'log_dir': log_dir, 'survey.enabled': survey}]
        else:                  # 옛 태그 미션표 (태그 → 단계 · 위치표). plan 모드는 쓰지 않음
            mission_params.append(PathJoinSubstitution([mis, 'config', 'mission_table.yaml']))
        # 시작 전 준비 확인 (R3): 띄운 것만 확인 목록에 넣음 (빈 목록은 넘기지 않음 — 빈 배열은 타입을 몰라 파라미터 오류)
        on = lambda name: LaunchConfiguration(name).perform(context).lower() == 'true'
        # 팔 정리 (manipulator 가 켜진 뒤 STOW 로 모음, /manipulator/startup_done): manipulator_node 는 bringup 이 늘 띄움
        ready = {'ready.localization': on('use_localization'), 'ready.arm_startup': True}
        if on('use_hardware'):   # base_bringup.launch.py spawner 와 같은 목록 (IMU 는 실물만, gripper_controller 는 bringup 에서 안 씀)
            ready['ready.controllers'] = ['joint_state_broadcaster', 'diff_drive_controller', 'arm_controller'] + \
                ([] if on('sim') else ['imu_sensor_broadcaster'])
        if on('use_nav2'):
            ready['ready.lifecycle_managers'] = ['lifecycle_manager_navigation']
        mission_params.append(ready)
        return mission_params

    def planner(context):   # 계획기: mission 과 같은 등록표, 기록은 <log_dir>/planner (mission 이 plan 성공 뒤 에피소드 폴더로 복사)
        if LaunchConfiguration('use_planner').perform(context).lower() != 'true':
            return []
        log_dir, pucks = log_paths(context)
        args = {'use_sim_time': LaunchConfiguration('use_sim_time'), 'pucks_file': pucks,
                'mode': LaunchConfiguration('planner_mode'), 'stop_after': LaunchConfiguration('planner_stop_after'),
                'avoid_slots': LaunchConfiguration('planner_avoid_slots')}
        if log_dir:
            args['log_dir'] = os.path.join(log_dir, 'planner')
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([mpl, 'launch', 'planner.launch.py'])),
            launch_arguments=args.items())]

    # 무거운 노드 묶음 (#275 · #276): 카메라 · 인지 · 위치 추정 · 계획기 → (nav2_delay 뒤) Nav2.
    #   Jetson 에서 전부 한꺼번에 뜨면 controller_manager 가 늦어 컨트롤러가 못 뜨고 (#275) bt_navigator 활성화도 실패 (#276).
    #   stage_start:=true (기본) 이고 use_hardware 면 /joint_states 가 나온 뒤 (최대 stage_timeout 초) 띄움
    heavy = [
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([loc, 'launch', 'localization.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'localization': LaunchConfiguration('localization'),
                              'use_amcl_node': LaunchConfiguration('use_amcl_node'),
                              'map': LaunchConfiguration('map'),
                              'objects_map': LaunchConfiguration('objects_map'),
                              'loc_map': LaunchConfiguration('loc_map'),
                              'loc_guard': LaunchConfiguration('loc_guard')}.items(),
            condition=IfCondition(LaunchConfiguration('use_localization'))),
        OpaqueFunction(function=planner),
        TimerAction(period=LaunchConfiguration('nav2_delay'), actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([nav, 'launch', 'navigation.launch.py'])),
                launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                                  'max_wz': LaunchConfiguration('max_wz'),   # 실물 기본 0.5 (아래 선언, 11-21)
                                  'use_localization': 'false'}.items(),   # 위치 추정은 위 use_localization 하나만 (#116)
                condition=IfCondition(LaunchConfiguration('use_nav2')))]),
    ]

    def staged(context):
        hw_on = LaunchConfiguration('use_hardware').perform(context).lower() == 'true'
        if not (hw_on and LaunchConfiguration('stage_start').perform(context).lower() == 'true'):
            return heavy
        t = LaunchConfiguration('stage_timeout').perform(context)
        gate = ExecuteProcess(name='wait_joint_states', output='screen', cmd=['bash', '-c',
            f'timeout {t} ros2 topic echo --once /joint_states sensor_msgs/msg/JointState > /dev/null '
            f'&& echo "[bringup] /joint_states 받음 -> 인지 · 위치 추정 · Nav2 시작" '
            f'|| echo "[bringup] /joint_states {t} s 안에 없음 -> 그래도 시작 (컨트롤러 확인, #275)"'])
        return [gate, RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=heavy))]

    return LaunchDescription([
        # mode 는 use_amcl_node 기본값 (PythonExpression) 이 읽으므로 맨 앞에 선언
        DeclareLaunchArgument('mode', default_value='mission',
                              description='mission (경기, 기본) | drive (주행 시연 로봇 쪽: 팀 Nav2 · 계획기 · mission_node 없음). /system/mode 로 발행'),
        DeclareLaunchArgument('use_hardware', default_value='true', description='① ② OpenCR · 컨트롤러 · twist_mux · 조이스틱'),
        # 2026-11-12 sim 은 diag_timeout 기본값 (PythonExpression) 이 읽으므로 그보다 먼저 선언 (sim:= 안 주면 'launch configuration sim does not exist')
        DeclareLaunchArgument('sim', default_value='false',
                              description='① 을 Isaac Sim 브리지에 연결 (TopicBasedSystem). use_sim_time:=true 와 함께'),
        DeclareLaunchArgument('diag_timeout',
                              default_value=PythonExpression(["'3.0' if '", LaunchConfiguration('use_hardware'), "'.lower() == 'true' and '",
                                                              LaunchConfiguration('sim'), "'.lower() != 'true' else '0.0'"]),
                              description='mission_node diagnostics.timeout [s]: opencr/* 진단이 이만큼 끊기면 안전 정지. 기본 실물 3.0 · sim · 하드웨어 없음 0 (끔)'),
        DeclareLaunchArgument('use_teleop', default_value='true', description='조이스틱 (끄면 E-stop heartbeat 검사도 끔)'),
        DeclareLaunchArgument('use_perception', default_value='true', description='⑦ 카메라 3대 · 검출'),
        DeclareLaunchArgument('split_control', default_value='false',
                              description='측정용 (#265): control_container 를 mission · drive · manipulator · tfmini 프로세스 넷으로 나눔 '
                                          '(노드별 CPU). 경기는 false (한 프로세스 · intra-process)'),
        DeclareLaunchArgument('use_tfmini', default_value='true',
                              description='1층 TFmini(박스 거리) · obstacle_stop · twist_mux obstacle lock. 시뮬레이션 · TFmini 없음은 false'),
        DeclareLaunchArgument('mission_source', default_value='plan',
                              description='plan (주사위 기억 로봇: mission_plan.yaml + 계획기) | tag (옛 태그 미션표 mission_table.yaml)'),
        DeclareLaunchArgument('use_planner',
                              default_value=PythonExpression(["'true' if '", LaunchConfiguration('mission_source'), "' == 'plan' else 'false'"]),
                              description='mission_planner_node (계획기). 기본: mission_source:=plan 이면 true'),
        DeclareLaunchArgument('log_dir', default_value='~/Lv2_Cartlidar_Assignment/mission_log',
                              description='미션 기록 루트: 미션마다 <log_dir>/<YYMMDD_HHMM>/ (기록 · 사진 · 사본), 계획기는 <log_dir>/planner. 빈 값 = 기록 안 함'),
        DeclareLaunchArgument('pucks_file', default_value='',
                              description='퍽 등록표: mission 측량 결과 (survey.pucks_out) = 계획기 pucks_file. 빈 값 = <log_dir>/pucks_survey.yaml. '
                                          '측량을 끄면 미리 만든 등록표'),
        DeclareLaunchArgument('planner_avoid_slots', default_value='',
                              description="계획기: 싣지 않을 팔레트 칸 '열:칸,…' (예 2:1,2:2 = G_2 · G_3, #219). 빈 값 = 모두 씀"),
        DeclareLaunchArgument('planner_mode', default_value='hybrid', description='계획기 mode: hybrid (기본) | per_location | single_load'),
        DeclareLaunchArgument('planner_stop_after', default_value='none',
                              description='부분 실물 시험: none | observe (관측만) | first_pick (박스 1 곳 집기) | first_place (장소 1 곳 놓기 + verify). 그 뒤 복귀'),
        DeclareLaunchArgument('use_localization', default_value='true',
                              description='localization.launch.py (지도 · scan 합치기 · loc_monitor). Nav2 는 따로 (#114). '
                                          '기본 true = 경기 (2026-11-14, 이전 false), 벤치 시험은 false'),
        DeclareLaunchArgument('localization', default_value='amcl', description='위치 추정 방식: amcl | carto (localization.launch.py)'),
        DeclareLaunchArgument('use_nav2', default_value='true',
                              description='navigation.launch.py (Nav2, use_localization:=false 로). 위치 추정은 use_localization 하나. '
                                          '기본 true = 경기 (2026-11-14, 이전 false)'),
        DeclareLaunchArgument('use_survey', default_value='true',
                              description='plan 모드 퍽 측량 (mission survey.enabled). false = pucks_file 의 미리 만든 등록표로 계획 '
                                          '(측량 자세 #154 를 정하기 전 실물 단계 시험). false 면 pucks_file 필수'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([mpl, 'maps', 'map_amcl.yaml']),
                              description='/map (AMCL · costmap 벽): 발제 도면 SDF 지도. localization 기본 SLAM 지도는 SDF 와 7 ~ 15 cm 다름 (#116)'),
        DeclareLaunchArgument('objects_map', default_value=PathJoinSubstitution([mpl, 'maps', 'map_nav.yaml']),
                              description='/map_objects (costmap: 벽 + 라이다에 안 보이는 상자 · 받침대): 발제 도면 SDF 지도'),
        DeclareLaunchArgument('loc_map', default_value=LaunchConfiguration('map'),
                              description='loc_monitor 잔차 · 탐색 지도 (기본 = map). AMCL 과 같은 지도여야 거짓 LOST 가 없음'),
        DeclareLaunchArgument('loc_guard', default_value='shadow',
                              description='AMCL 튐 감시 · 보정 (localization.launch.py): shadow (기본, 기록만 · map → odom 은 AMCL) | '
                                          'active (loc_guard 가 map → odom, AMCL tf_broadcast 끔, use_amcl_node 필요) | off'),
        DeclareLaunchArgument('use_amcl_node',
                              default_value=PythonExpression(["'", LaunchConfiguration('use_localization'), "' if '",
                                                              LaunchConfiguration('mode'), "' == 'drive' else '",
                                                              LaunchConfiguration('use_nav2'), "'"]),
                              description='AMCL 노드를 localization 쪽에서 띄움. 기본 = use_nav2 (Nav2 의 map → odom), mode:=drive 면 use_localization'),
        DeclareLaunchArgument('use_loc_lock', default_value=LaunchConfiguration('use_localization'),
                              description='twist_mux 위치 잃음 lock (/localization/lock, #93). 기본 = use_localization'),
        DeclareLaunchArgument('use_loc_gate', default_value=LaunchConfiguration('use_loc_lock'),
                              description='drive navigate 를 /localization/status OK 로 막기 (#121). loc_monitor 가 있을 때만. 기본 = use_loc_lock'),
        DeclareLaunchArgument('use_ekf', default_value='true', description='⑥ robot_localization ekf + odom_icr (#92)'),
        DeclareLaunchArgument('use_sensors',
                              default_value=PythonExpression(["'false' if '", LaunchConfiguration('sim'), "'.lower() == 'true' else 'true'"]),
                              description='카메라 · 라이다 드라이버 (sensors.launch.py). 기본: 실물 true · sim:=true 면 false'),
        DeclareLaunchArgument('front_serial', default_value='261822072247', description='앞 D435 시리얼 (sensors.launch.py 와 같게), 빈 값 = 아무 장치'),
        DeclareLaunchArgument('rear_serial', default_value='261822079801', description='뒤 D435 시리얼, 빈 값 = 아무 장치'),
        DeclareLaunchArgument('ports', default_value='auto',
                              description='두 OpenCR 포트. auto = /dev/ttyACM* 중 OpenCR 2 개를 찾아 씀 (보드 리셋으로 번호가 바뀌어도 됨, '
                                          '2 개가 아니면 찾은 목록과 함께 중단). 직접: ports:=/dev/ttyACM2,/dev/ttyACM3 (순서 무관, 역할은 STATE.role). '
                                          'RC-100 리모컨을 쓸 때는 rc100:=true 필수 — 없으면 리모컨 리셋 (2+3) 이 E-stop 을 못 풂'),
        DeclareLaunchArgument('drive', default_value='belt', description='구동: belt (증속벨트) | direct (직결)'),
        DeclareLaunchArgument('skill_enabled', default_value='false',
                              description='정책 실행 — ONNX Runtime 빌드에서만 true (없으면 거절 → 미션 fallback)'),
        DeclareLaunchArgument('policies_file', default_value=PathJoinSubstitution([pol, 'config', 'policies.yaml'])),
        DeclareLaunchArgument('stage_start', default_value='true',
                              description='true: /joint_states 가 나온 뒤 인지 · 위치 추정 · 계획기 · Nav2 를 띄움 (Jetson 시작 부하, #275 · #276). use_hardware 일 때만'),
        DeclareLaunchArgument('stage_timeout', default_value='90', description='/joint_states 를 기다릴 최대 [s]. 넘으면 그래도 띄움'),
        DeclareLaunchArgument('nav2_delay', default_value='10.0', description='무거운 묶음 시작 뒤 Nav2 를 띄울 때까지 [s] (bt_navigator 활성화 실패 #276)'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='시뮬레이션 시계(/clock) 사용 — 모든 노드에 전달'),
        # 2026-11-21 실물 회전 한계 (review_arch ③): 실물이면 0.5, Isaac · 하드웨어 없음은 빈 값 (= nav2_params.yaml 0.75). rotate_wz 는 안 줌
        #   (navigation 이 min(YAML 0.75, max_wz) = 0.5 로 맞춤. rotate_wz 기본을 같이 두면 max_wz:=0.4 같은 명령에서 rotate > max 로 거부됨)
        DeclareLaunchArgument('max_wz',
                              default_value=PythonExpression(["'0.5' if '", LaunchConfiguration('use_hardware'), "'.lower() == 'true' and '",
                                                              LaunchConfiguration('sim'), "'.lower() != 'true' else ''"]),
                              description='Nav2 회전 한계 [rad/s] (MPPI wz_max · 제자리 회전 · behavior max_rotational_vel · smoother). '
                                          '기본 실물 0.5 · sim:=true 또는 use_hardware:=false 는 빈 값 (YAML 0.75)'),
        OpaqueFunction(function=mode_setup),   # 아래 모든 것보다 먼저 (drive 면 use_nav2 · use_planner = false)

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([hw, 'launch', 'base_bringup.launch.py'])),
            launch_arguments={'use_teleop': LaunchConfiguration('use_teleop'), 'ports': LaunchConfiguration('ports'),
                              'drive': LaunchConfiguration('drive'), 'sim': LaunchConfiguration('sim'),
                              'obstacle_lock': LaunchConfiguration('use_tfmini'),
                              'loc_lock': LaunchConfiguration('use_loc_lock')}.items(),
            condition=IfCondition(LaunchConfiguration('use_hardware'))),
        OpaqueFunction(function=control_container),
        OpaqueFunction(function=staged),
        # ⑥ /odom + IMU ω → /odom_icr (vy = −icr_x·ω, 스키드 회전 중심 옆 이동, #92) → ekf odom0
        Node(package='bringup', executable='odom_icr', name='odom_icr', output='screen',
             parameters=[geometry, sim], condition=IfCondition(LaunchConfiguration('use_ekf'))),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[PathJoinSubstitution([me, 'config', 'ekf.yaml']), sim],
             condition=IfCondition(LaunchConfiguration('use_ekf'))),
    ])
