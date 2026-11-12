"""주행 (Nav2): 경로 계획 Smac 2D · 경로 추종 MPPI · behavior · velocity_smoother · collision_monitor + (선택) localization (AMCL).

  ros2 launch navigation navigation.launch.py                                     # 실물: AMCL 포함, 출력 /cmd_vel_nav (twist_mux nav 입력)
  ros2 launch navigation navigation.launch.py use_sim_time:=true odom_topic:=/odom cmd_vel_out:=/cmd_vel   # Gazebo (waffle_4wd_description)
  ros2 launch navigation navigation.launch.py use_localization:=false            # map → odom · /map 을 다른 곳에서 낼 때
  ros2 launch navigation navigation.launch.py max_wz:=0.8 rotate_wz:=0.6         # 속도 한계만 바꿔 실행 (아래)

설정: config/nav2_params.yaml (★ 임시값). AMCL 설정은 localization/config/amcl.yaml, 출발 자세 start_x/y/yaw 등 localization 인자는 그대로 넘어간다.
속도 한계 인자 (비우면 YAML 값 그대로): max_vx [m/s] · max_wz [rad/s] · rotate_wz [rad/s, 제자리 회전, ≤ max_wz].
  같은 한계가 4 노드 6 곳에 있어 (MPPI vx_max · wz_max, RotationShim rotate_to_heading_angular_vel, behavior max_rotational_vel,
  velocity_smoother max · min_velocity) 이 인자 하나로 함께 바꾼다. max_wz 만 주면 rotate_wz = min(YAML 값, max_wz).
  bringup · gazebo_sim 에 줘도 된다 (명령 줄 인자는 포함된 launch 까지 감). 실물 모터 = 로봇 회전 × 3.20 (11-11 벨트 시험).
속도 명령: controller_server · behavior_server → /nav2/cmd_vel_raw → velocity_smoother → /nav2/cmd_vel_smoothed
  → collision_monitor → cmd_vel_out (기본 /cmd_vel_nav). use_collision_monitor:=false 면 smoother 가 바로 cmd_vel_out 으로 낸다.
  Nav2 기본 launch 는 중간 출력을 cmd_vel_nav 로 쓰므로, 그대로 쓰면 twist_mux 가 smoother · collision_monitor 를 건너뛴 명령을 받는다."""
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

RAW = '/nav2/cmd_vel_raw'            # controller_server · behavior_server 출력
SMOOTHED = '/nav2/cmd_vel_smoothed'  # velocity_smoother 출력 = collision_monitor 입력 (nav2_params.yaml 과 같게)
# 실물 바퀴 앞 상한 (opencr_hardware diff_drive: drive_belt.yaml linear.x 0.34 · controllers_4wd.yaml angular.z 1.0). 넘으면 알림만 (Gazebo 는 안 거침)
DD_VX, DD_WZ = 0.34, 1.0


def speed_limits(context):
    """launch 인자 max_vx · max_wz · rotate_wz → 노드별 매개변수 덮어쓰기 {노드: {이름: 값}} 와 알림 문장들. 인자가 모두 비면 ({}, [])"""
    def arg(name):
        s = LC(name).perform(context).strip()
        if not s:
            return None
        try:
            v = float(s)
        except ValueError:
            raise RuntimeError(f'navigation: {name}:={s} 는 숫자가 아님') from None
        if not v > 0.0:
            raise RuntimeError(f'navigation: {name}:={s} 는 0 보다 커야 함')
        return v

    vx, wz, rot = arg('max_vx'), arg('max_wz'), arg('rotate_wz')
    if vx is None and wz is None and rot is None:
        return {}, []
    with open(LC('params_file').perform(context)) as f:   # 덮어쓰지 않는 값 (YAML 기본값) 을 읽음
        y = yaml.safe_load(f)
    fp = y['controller_server']['ros__parameters']['FollowPath']
    mppi = fp['primary_controller']
    beh = y['behavior_server']['ros__parameters']
    sm = y['velocity_smoother']['ros__parameters']
    vx = float(mppi['vx_max']) if vx is None else vx
    rot = (min(float(fp['rotate_to_heading_angular_vel']), wz) if wz is not None else float(fp['rotate_to_heading_angular_vel'])) \
        if rot is None else rot
    wz = float(mppi['wz_max']) if wz is None else wz
    if rot > wz:
        raise RuntimeError(f'navigation: rotate_wz {rot} 가 max_wz {wz} 보다 큼')
    if wz < float(beh['min_rotational_vel']):
        raise RuntimeError(f'navigation: max_wz {wz} 가 behavior min_rotational_vel {beh["min_rotational_vel"]} 보다 작음')
    over = {
        'controller_server': {'FollowPath.primary_controller.vx_max': vx, 'FollowPath.primary_controller.wz_max': wz,
                              'FollowPath.rotate_to_heading_angular_vel': rot},
        'behavior_server': {'max_rotational_vel': wz},
        'velocity_smoother': {'max_velocity': [vx, float(sm['max_velocity'][1]), wz],
                              'min_velocity': [float(sm['min_velocity'][0]), float(sm['min_velocity'][1]), -wz]},
    }
    notes = [f'navigation: 속도 한계 (launch 인자) vx {vx} m/s · wz {wz} rad/s · 제자리 회전 {rot} rad/s']
    if vx > DD_VX or wz > DD_WZ:
        notes.append(f'navigation: 실물 diff_drive 상한 (vx {DD_VX} · wz {DD_WZ}) 보다 큼 — 실물에서는 바퀴 앞에서 잘림')
    return over, notes


def launch_setup(context):
    use_cm = LC('use_collision_monitor').perform(context).lower() == 'true'
    out = LC('cmd_vel_out').perform(context)
    params = ParameterFile(LC('params_file'), allow_substs=True)   # bt_search_directories 의 $(find-pkg-share ...) 풀기
    sim = {'use_sim_time': ParameterValue(LC('use_sim_time'), value_type=bool)}
    odom = {'odom_topic': LC('odom_topic')}
    over, notes = speed_limits(context)
    try:
        delay = float(LC('lifecycle_delay').perform(context))
    except ValueError:
        raise RuntimeError(f"navigation: lifecycle_delay:={LC('lifecycle_delay').perform(context)} 는 숫자가 아님") from None
    servers = ['controller_server', 'planner_server', 'behavior_server', 'velocity_smoother'] + \
        (['collision_monitor'] if use_cm else []) + ['bt_navigator']

    nodes = [LogInfo(msg=n) for n in notes] + [
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
             parameters=[params, sim, odom, over.get('controller_server', {})], remappings=[('cmd_vel', RAW)]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params, sim]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params, sim, over.get('behavior_server', {})], remappings=[('cmd_vel', RAW)]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother', output='screen',
             parameters=[params, sim, odom, over.get('velocity_smoother', {})],
             remappings=[('cmd_vel', RAW), ('cmd_vel_smoothed', SMOOTHED if use_cm else out)]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, sim, odom]),
        # 켜 주는 노드는 서버들이 뜬 뒤 lifecycle_delay 만큼 늦게: 한꺼번에 뜰 때 change_state 응답이 DDS 에서 버려져
        # ("failed to send response … client will not receive response") 끝없이 기다리던 일 (Gazebo 11-12, 10 번 중 1 번) 을 줄임
        TimerAction(period=delay, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', output='screen',
                 parameters=[sim, {'autostart': True, 'node_names': servers}])]),
    ]
    if use_cm:
        nodes.append(Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor', output='screen',
                          parameters=[params, sim, {'cmd_vel_in_topic': SMOOTHED, 'cmd_vel_out_topic': out}]))
    return nodes


def generate_launch_description():
    me = FindPackageShare('navigation')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([me, 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('use_localization', default_value='true',
                              description='localization launch 를 use_amcl_node:=true 로 함께 (AMCL · /map · /map_objects · /scan_merged · loc_monitor)'),
        DeclareLaunchArgument('use_collision_monitor', default_value='true'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel_nav', description='최종 속도 명령 (실물 = twist_mux nav 입력, Gazebo = /cmd_vel)'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='실물 = EKF, Gazebo = /odom'),
        DeclareLaunchArgument('lifecycle_delay', default_value='3.0',
                              description='lifecycle_manager_navigation 을 서버들보다 이만큼 늦게 시작 [s] (시작 때 DDS 응답 유실 줄이기, 0 = 바로)'),
        DeclareLaunchArgument('max_vx', default_value='', description='전진 속도 한계 [m/s] (MPPI vx_max · smoother). 비우면 YAML'),
        DeclareLaunchArgument('max_wz', default_value='',
                              description='회전 속도 한계 [rad/s] (MPPI wz_max · behavior · smoother). 비우면 YAML. 실물 모터 = × 3.20'),
        DeclareLaunchArgument('rotate_wz', default_value='',
                              description='제자리 회전 속도 [rad/s] (RotationShim, ≤ max_wz). 비우면 YAML 값 (max_wz 를 주면 그보다 크지 않게)'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('localization'), 'launch', 'localization.launch.py'])),
            launch_arguments={'use_amcl_node': 'true', 'use_sim_time': LC('use_sim_time')}.items(),
            condition=IfCondition(LC('use_localization'))),
        OpaqueFunction(function=launch_setup),
    ])
