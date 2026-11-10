# ⑨ mission_container (component_container_mt) — MissionNode
# 사용: ros2 launch mission mission_bringup.launch.py [source:=plan|tag]
#   source:=plan (기본)  주사위 기억 로봇 발제 v5: mission_plan.yaml 을 덧씌움 (mission_planner_node 는 mission_planner 패키지 launch 로 따로)
#   source:=tag          옛 태그 미션표 (mission_table.yaml: 태그 → 단계). plan 모드는 미션표 · 위치표를 쓰지 않아 tag 일 때만 불러옴
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _nodes(context):
    config = os.path.join(get_package_share_directory('mission'), 'config')
    source = LaunchConfiguration('source').perform(context)
    if source not in ('tag', 'plan'):
        raise RuntimeError(f'source 는 plan 또는 tag: {source}')
    params = [os.path.join(config, 'mission.yaml')]
    params.append(os.path.join(config, 'mission_plan.yaml' if source == 'plan' else 'mission_table.yaml'))
    mission = ComposableNode(
        package='mission',
        plugin='mission::MissionNode',
        name='mission_node',
        parameters=params,
        extra_arguments=[{'use_intra_process_comms': True}],
    )
    container = ComposableNodeContainer(
        name='mission_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[mission],
        output='screen',
    )
    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('source', default_value='plan', description='plan (주사위 기억 로봇 계획기) | tag (옛 태그 미션표)'),
        OpaqueFunction(function=_nodes),
    ])
