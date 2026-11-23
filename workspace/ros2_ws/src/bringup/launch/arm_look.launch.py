"""주행 모드 (bringup.launch.py mode:=drive use_perception:=false) 에서 팔 카메라 (Arducam) 로 상자를 재기 위한 최소 인식 (11-22 밤).
tools/drive_demo run_demo.sh ARM_LOOK=1|dry 가 부르는 서비스 (/perception/set_mode · /perception/place_box/measure · /perception/place_box/status) 만 띄움.

  ros2 launch bringup arm_look.launch.py                      # perception_node 하나 + 팔 Arducam 드라이버
  ros2 launch bringup arm_look.launch.py use_arm_camera:=false  # Arducam 은 이미 다른 launch (sensors.launch.py) 가 띄웠을 때
  ros2 launch bringup arm_look.launch.py use_sim_time:=true   # (시험용)

bringup.launch.py use_perception:=true 와 같이 띄우지 말 것 (perception_node 가 둘이 됨). 이 launch 는 D435 · 라이다 · scan 합치기 · obstacle_stop ·
후방 장애물 · 퍽 측량을 띄우지 않음 (perception.launch.py 와 달리 PerceptionNode 하나만). 검출기는 색 검출로 고정 (TensorRT 엔진을 안 올려 GPU · 메모리 부하를 줄임 —
place_box 는 검출기를 쓰지 않음). 팔 Arducam 드라이버 = sensors.launch.py use_arm_camera (D435 · 라이다는 끔).
"""
import atexit
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _as_bool(text):
    return str(text).lower() in ('true', '1', 'yes')


def _setup(context):
    cfg = os.path.join(get_package_share_directory('perception'), 'config', 'perception.yaml')
    sim = _as_bool(context.launch_configurations['use_sim_time'])
    # perception.launch.py 와 같은 이유로 (설정 yaml 의 노드 이름 항목이 dict · -p 를 덮음) 임시 yaml 을 맨 뒤 파일로 넘김
    f = tempfile.NamedTemporaryFile('w', prefix='arm_look_perception_', suffix='.yaml', delete=False)
    atexit.register(lambda path=f.name: os.path.exists(path) and os.remove(path))
    yaml.safe_dump({'perception_node': {'ros__parameters': {'use_sim_time': sim, 'detector.type': 'color', 'arm.detector': 'color'}}}, f)
    f.close()
    return [ComposableNodeContainer(
        name='perception_container', namespace='', package='rclcpp_components', executable='component_container_mt', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='perception', plugin='perception::PerceptionNode', name='perception_node',
            parameters=[cfg, f.name], extra_arguments=[{'use_intra_process_comms': True}])])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_arm_camera', default_value='true', description='팔 Arducam 드라이버 (sensors.launch.py use_arm_camera, D435 · 라이다는 끔)'),
        OpaqueFunction(function=_setup),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'sensors.launch.py'])),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 'use_d435': 'false', 'use_lidar': 'false',
                              'use_arm_camera': 'true'}.items(),
            condition=IfCondition(LaunchConfiguration('use_arm_camera'))),
    ])
