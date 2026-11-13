"""Gazebo Sim (gz-sim 10) 에 waffle_4wd 를 놓고 센서 · 구동 토픽을 ROS 2 로 잇는다.

    ros2 launch waffle_4wd_description gazebo.launch.py                # 창 있음, 주사위 경기장, 출발 자리
    ros2 launch waffle_4wd_description gazebo.launch.py gui:=false     # 창 없이 (센서 렌더는 함)
    ros2 launch waffle_4wd_description gazebo.launch.py rviz:=true x:=1.0 y:=1.0 yaw:=0.0
    ros2 launch waffle_4wd_description gazebo.launch.py collision:=full   # 차체 충돌 볼록 메시 그대로 (퍽 · 보관함 시험, 느림)
    ros2 launch waffle_4wd_description gazebo.launch.py sliders:=true     # + 관절 슬라이더 창 (팔 관절을 Gazebo 로)

- 월드: worlds/dice_mission.sdf (주사위 기억 로봇 경기장, ~/GIT/dice_mission_gazebo 에서 생성한 것). world:= 로 바꿀 수 있음.
- 출발 자리 기본값: 발제 신호 관측 자리 (0.734, 0.950) m, 신호등을 봄 (yaw = π).
- 토픽 이름 · 센서 값은 config/sensors.yaml, 브리지 목록은 config/gz_bridge.yaml.
- odom_tf:=false: DiffDrive 의 odom -> base_link TF 를 ROS 로 잇지 않음 (config/gz_bridge_odom_tf.yaml). /odom 토픽은 그대로.
  실물처럼 EKF 가 TF 를 낼 때 (navigation gazebo_sim.launch.py use_ekf:=true). 바퀴만으로 센 회전은 실제보다 약 22 % 큼 (#151).
- collision:=light (기본): 차체 볼록 메시 423 개를 묶음별 상자 12 개로 (실시간 비율 0.76 -> 1.0, 2026-10-28 시험).
- 카메라 토픽은 lazy 브리지: ROS 에서 구독할 때만 렌더한다 (9 개 다 쓰면 실시간 비율 약 0.5).
- 키보드 조작: 다른 터미널에서 ros2 run waffle_4wd_description keyboard_teleop.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'waffle_4wd_description'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    arg = LaunchConfiguration
    # limit_margin: 관절 물리 한계를 1 mm · 0.001 rad 넓힘 (Gazebo DART 에서 한계 끝에 닿으면 걸림, URDF 한계값 명령은 그대로 됨)
    description = ParameterValue(Command(['xacro ', os.path.join(share, 'urdf', 'waffle_4wd.urdf.xacro'),
                                          ' collision:=', LaunchConfiguration('collision'), ' limit_margin:=0.001 pucks:=', LaunchConfiguration('pucks')]), value_type=str)
    gz_args = PythonExpression(["'-r ' + ('' if '", arg('gui'), "' == 'true' else '-s --headless-rendering ') + '", arg('world'), "'"])
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'dice_mission.sdf')),
        DeclareLaunchArgument('gui', default_value='true', description='false 면 창 없이 (gz sim -s --headless-rendering)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('sliders', default_value='false', description='true 면 관절 슬라이더 창 (팔 관절 -> /arm/<관절>/cmd_pos)'),
        DeclareLaunchArgument('pucks', default_value='false', description='true 면 팔레트에 퍽 12개 (주행용 질량, 겉모양 · 질량만)'),
        DeclareLaunchArgument('collision', default_value='light', description='light (상자, 빠름) | full (볼록 메시)'),
        DeclareLaunchArgument('odom_tf', default_value='true',
                              description='false 면 DiffDrive 바퀴 odom TF (odom -> base_link) 를 안 이음 (EKF 가 낼 때, #151)'),
        DeclareLaunchArgument('x', default_value='0.734'),
        DeclareLaunchArgument('y', default_value='0.950'),
        DeclareLaunchArgument('yaw', default_value='3.14159265'),
        # package://<이 패키지>/... 메시를 Gazebo 가 찾도록 (share 의 부모 폴더)
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(share)),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': description}, sim_time]),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'waffle_4wd', '-allow_renaming', 'false',
                        '-x', arg('x'), '-y', arg('y'), '-z', '0.012', '-Y', arg('yaw')]),   # 루트 base_link 는 바닥 위 6.75 mm (+ 5 mm 띄워 놓기)
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge.yaml')}, sim_time]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='odom_tf_bridge', output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'gz_bridge_odom_tf.yaml')}, sim_time],
             condition=IfCondition(arg('odom_tf'))),
        Node(package=PKG, executable='tfmini_scan_to_range.py', output='screen', parameters=[sim_time]),
        Node(package=PKG, executable='camera_info_fix.py', output='screen',
             parameters=[{'config': os.path.join(share, 'config', 'camera_info_fix.yaml')}, sim_time]),
        # 슬라이더 창 출력은 /slider/joint_states 로 (Gazebo 의 /joint_states 와 섞이지 않게), slider_to_gazebo.py 가 팔 명령으로 넘김
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', output='screen',
             remappings=[('joint_states', '/slider/joint_states')], condition=IfCondition(arg('sliders'))),
        Node(package=PKG, executable='slider_to_gazebo.py', output='screen', condition=IfCondition(arg('sliders'))),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'gazebo.rviz')],
             parameters=[sim_time], condition=IfCondition(arg('rviz'))),
    ])
