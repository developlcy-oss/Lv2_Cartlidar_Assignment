"""config/sensors.yaml 을 읽어 URDF 에 Gazebo (gz-sim 10) 센서 · 구동 설정 (<gazebo> 태그) 을 붙인다. make_revision.py 가 부른다.

- 센서: LDS-03 x 2 (gpu_lidar), TFmini (광선 1 개 gpu_lidar), D435 x 2 (컬러 camera + 깊이 depth_camera + IR 카메라 2), 팔 카메라, IMU x 2.
  센서는 frame 링크에 붙이고 (고정 관절로 합쳐져도 sdformat 이 자리를 옮김) gz_frame_id 로 ROS frame 이름을 맞춘다.
  카메라 frame 은 광학 좌표 (z 앞, x 오른쪽, y 아래) 라 Gazebo 카메라 좌표 (x 앞, y 왼쪽, z 위) 로 돌리는 pose 를 넣는다.
  gz_frame_id 는 SDF 사양에 없어 gz sdf 가 경고하지만 gz-sensors 가 직접 읽는다 (libgz-sensors 10).
- 구동: DiffDrive (바퀴 4), JointStatePublisher, 팔 관절마다 JointPositionController, OdometryPublisher (실제 위치).
- RViz · robot_state_publisher 는 <gazebo> 태그를 무시하므로 같은 URDF 를 그대로 쓴다.
"""
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

OPTICAL_TO_CAMERA = Rotation.from_matrix([[0, -1, 0], [0, 0, -1], [1, 0, 0]]).as_euler('xyz')   # 광학 frame 안에서 Gazebo 카메라 축


def f(v):
    return f'{float(v):.6g}'


def sub(parent, tag, text=None, **attrib):
    e = ET.SubElement(parent, tag, {k: str(v) for k, v in attrib.items()})
    if text is not None:
        e.text = str(text)
    return e


def gazebo(robot, reference=None):
    return sub(robot, 'gazebo', **({'reference': reference} if reference else {}))


def sensor(robot, reference, name, typ, topic, frame, rate, pose=(0, 0, 0, 0, 0, 0)):
    s = sub(gazebo(robot, reference), 'sensor', name=name, type=typ)
    sub(s, 'pose', ' '.join(f(v) for v in pose))
    sub(s, 'always_on', 1)
    sub(s, 'update_rate', rate)
    sub(s, 'visualize', 'false')
    sub(s, 'topic', topic.lstrip('/'))
    sub(s, 'gz_frame_id', frame)
    return s


def noise(parent, stddev, typ='gaussian'):
    n = sub(parent, 'noise')
    sub(n, 'type', typ)
    sub(n, 'mean', 0)
    sub(n, 'stddev', stddev)


def lidar(s, samples, a0, a1, rmin, rmax, stddev):
    li = sub(s, 'lidar')
    sc = sub(li, 'scan')
    h = sub(sc, 'horizontal')
    sub(h, 'samples', samples)
    sub(h, 'resolution', 1)
    sub(h, 'min_angle', f(a0))
    sub(h, 'max_angle', f(a1))
    v = sub(sc, 'vertical')
    sub(v, 'samples', 1)
    sub(v, 'min_angle', 0)
    sub(v, 'max_angle', 0)
    r = sub(li, 'range')
    sub(r, 'min', rmin)
    sub(r, 'max', rmax)
    sub(r, 'resolution', 0.001)
    noise(li, stddev)


def camera(s, hfov, w, h, fmt, clip, stddev, info_topic=None, depth_clip=None):
    c = sub(s, 'camera')
    if info_topic:                      # camera · depth_camera 는 따름 (rgbd_camera 는 무시하고 <topic>/camera_info 로 냄, gz-sensors 10)
        sub(c, 'camera_info_topic', info_topic.lstrip('/'))
    sub(c, 'horizontal_fov', f(hfov))
    im = sub(c, 'image')
    sub(im, 'width', w)
    sub(im, 'height', h)
    sub(im, 'format', fmt)
    cl = sub(c, 'clip')
    sub(cl, 'near', clip[0])
    sub(cl, 'far', clip[1])
    if depth_clip:
        dc = sub(sub(c, 'depth_camera'), 'clip')
        sub(dc, 'near', depth_clip[0])
        sub(dc, 'far', depth_clip[1])
    if stddev is not None:
        noise(c, stddev)


def plugin(robot, filename, name):
    return sub(gazebo(robot), 'plugin', filename=filename, name=name)


def add_frame(robot, name, parent, xyz):
    """질량 없는 고정 frame 링크 (IMU 자리)."""
    sub(robot, 'link', name=name)
    j = sub(robot, 'joint', name=f'{name.removesuffix("_link")}_joint', type='fixed')
    sub(j, 'origin', xyz=' '.join(f(v) for v in xyz), rpy='0 0 0')
    sub(j, 'parent', link=parent)
    sub(j, 'child', link=name)


def add_gazebo(robot, sensors_yaml):
    S = yaml.safe_load(open(sensors_yaml, encoding='utf-8'))
    links = {lk.get('name'): lk for lk in robot.findall('link')}
    cam_pose = (0, 0, 0) + tuple(OPTICAL_TO_CAMERA)

    # 팔 그리퍼 모터 각 링크: 질량이 없으면 sdformat 이 움직이는 관절째 버림 -> 아주 작은 질량 (1 g)
    gm = links['gripper_motor_link']
    if gm.find('inertial') is None:
        ine = sub(gm, 'inertial')
        sub(ine, 'origin', xyz='0 0 0', rpy='0 0 0')
        sub(ine, 'mass', value='0.001')
        sub(ine, 'inertia', ixx='1e-08', ixy='0', ixz='0', iyy='1e-08', iyz='0', izz='1e-08')

    # LDS-03
    L = S['lds03']['sim']
    step = 2 * np.pi / L['samples']
    for side, u in S['lds03']['units'].items():
        s = sensor(robot, u['frame'], f'lds03_{side}', 'gpu_lidar', u['topic'], u['frame'], L['update_rate_hz'])
        lidar(s, L['samples'], -np.pi, np.pi - step, *L['range_m'], L['noise_stddev_m'])

    # TFmini (광선 1 개)
    T = S['tfmini_plus']
    s = sensor(robot, T['frame'], 'tfmini_plus', 'gpu_lidar', T['sim']['scan_topic'], T['frame'], T['sim']['update_rate_hz'])
    lidar(s, 1, 0, 0, *T['sim']['range_m'], T['sim']['noise_stddev_m'])

    # D435 x 2: 컬러 (camera) + 같은 자리 · 같은 화각 깊이 (depth_camera = 컬러에 맞춘 깊이), IR 좌우 (L8)
    #   rgbd_camera 는 이 로봇에 붙이면 장면을 못 봄 (컬러 = 배경색, 깊이 = 전부 inf; 따로 둔 시험 월드에서는 정상, 2026-10-28) -> 둘로 나눔
    D = S['d435']['sim']
    for side, u in S['d435']['units'].items():
        base = u['prefix'].lstrip('/')
        frame = f'{base}_color_optical_frame'
        # 컬러 · 깊이 센서는 광축 (광학 frame +z) 으로 sensor_forward_m 앞: URDF frame 이 D435 몸체 메시 안 (앞면 5.6 mm 뒤) 이라
        # 깊이 카메라가 몸체 안쪽 면에 막혀 전부 -inf 가 됨 (2026-10-28 시험, 1 cm 앞으로 옮기면 정상). 컬러도 같이 옮겨 맞춘 상태 유지
        fwd_pose = (0, 0, D['sensor_forward_m']) + tuple(OPTICAL_TO_CAMERA)
        s = sensor(robot, frame, f'{base}_color', 'camera', f'{base}/color/image_raw', frame, D['update_rate_hz'], fwd_pose)
        camera(s, D['color_hfov_rad'], D['width'], D['height'], 'R8G8B8', D['depth_clip_m'], D['image_noise_stddev'], f'{base}/color/camera_info')
        s = sensor(robot, frame, f'{base}_depth', 'depth_camera', f'{base}/aligned_depth_to_color/image_raw', frame, D['update_rate_hz'], fwd_pose)
        camera(s, D['color_hfov_rad'], D['width'], D['height'], 'R_FLOAT32', D['depth_clip_m'], None, f'{base}/aligned_depth_to_color/camera_info')
        noise(s.find('camera'), D['depth_noise_stddev_m'])
        if D['ir_stereo']:
            for ir in ('infra1', 'infra2'):
                s = sensor(robot, f'{base}_{ir}_optical_frame', f'{base}_{ir}', 'camera', f'{base}/{ir}/image_rect_raw',
                           f'{base}_{ir}_optical_frame', D['update_rate_hz'], cam_pose)
                camera(s, D['ir_hfov_rad'], D['width'], D['height'], 'L8', D['depth_clip_m'], D['image_noise_stddev'], f'{base}/{ir}/camera_info')

    # 팔 카메라
    A = S['arm_camera']
    s = sensor(robot, A['frame'], 'arm_camera', 'camera', f'{A["topic_prefix"]}/image_raw', A['frame'], A['sim']['update_rate_hz'], cam_pose)
    camera(s, A['sim']['hfov_rad'], A['sim']['width'], A['sim']['height'], 'R8G8B8', A['sim']['clip_m'], A['sim']['image_noise_stddev'],
           f'{A["topic_prefix"]}/camera_info')

    # IMU x 2 (OpenCR 위)
    I = S['imu']['sim']
    for side, u in S['imu']['units'].items():
        add_frame(robot, u['frame'], u['parent'], I['xyz_m'])
        s = sensor(robot, u['frame'], f'imu_{side}', 'imu', u['topic'], u['frame'], I['update_rate_hz'])
        imu = sub(s, 'imu')
        for grp, sd in (('angular_velocity', I['gyro_noise_stddev_rad_s']), ('linear_acceleration', I['accel_noise_stddev_m_s2'])):
            g = sub(imu, grp)
            for ax in 'xyz':
                n = sub(sub(g, ax), 'noise', type='gaussian')      # IMU 잡음은 type 이 속성 (lidar · camera 는 자식 태그)
                sub(n, 'mean', 0)
                sub(n, 'stddev', sd)

    # 팀 imu_sensor_broadcaster 의 frame (두 IMU 평균값용): 두 IMU frame 가운데, 차체와 같은 방향. 센서는 붙이지 않음
    mid = []
    for u in S['imu']['units'].values():
        pj = next(j for j in robot.findall('joint') if j.find('child').get('link') == u['parent'])
        assert pj.find('parent').get('link') == 'base_link'
        mid.append(np.array([float(v) for v in pj.find('origin').get('xyz').split()]) + I['xyz_m'])
    add_frame(robot, S['imu']['mean_frame'], 'base_link', np.mean(mid, axis=0))

    # 바퀴 마찰
    Dr = S['drive']['sim']
    for j in Dr['left_joints'] + Dr['right_joints']:
        child = next(jj.find('child').get('link') for jj in robot.findall('joint') if jj.get('name') == j)
        g = gazebo(robot, child)
        sub(g, 'mu1', Dr['wheel_friction'][0])
        sub(g, 'mu2', Dr['wheel_friction'][1])

    # 구동 · 관절 상태 · 실제 위치
    p = plugin(robot, 'gz-sim-diff-drive-system', 'gz::sim::systems::DiffDrive')
    for j in Dr['left_joints']:
        sub(p, 'left_joint', j)
    for j in Dr['right_joints']:
        sub(p, 'right_joint', j)
    sub(p, 'wheel_separation', Dr['wheel_separation_m'])
    sub(p, 'wheel_radius', Dr['wheel_radius_m'])
    sub(p, 'max_linear_velocity', f(Dr['max_wheel_speed_rad_s'] * Dr['wheel_radius_m']))
    sub(p, 'odom_publish_frequency', Dr['odom_rate_hz'])
    sub(p, 'topic', Dr['cmd_topic'].lstrip('/'))
    sub(p, 'odom_topic', Dr['odom_topic'].lstrip('/'))
    sub(p, 'tf_topic', 'tf')
    sub(p, 'frame_id', Dr['odom_frame'])
    sub(p, 'child_frame_id', Dr['base_frame'])

    Ar = S['arm']['sim']
    p = plugin(robot, 'gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher')
    sub(p, 'topic', 'joint_states')
    sub(p, 'update_rate', Ar['joint_states_rate_hz'])

    p = plugin(robot, 'gz-sim-odometry-publisher-system', 'gz::sim::systems::OdometryPublisher')
    sub(p, 'odom_frame', 'world')
    sub(p, 'robot_base_frame', Dr['base_frame'])
    sub(p, 'odom_topic', Dr['ground_truth_topic'].lstrip('/'))
    sub(p, 'tf_topic', 'ground_truth/tf')
    sub(p, 'odom_publish_frequency', Dr['odom_rate_hz'])
    sub(p, 'dimensions', 3)

    types = {j.get('name'): j.get('type') for j in robot.findall('joint')}
    for j in Ar['joints']:
        kind = 'prismatic' if types[j] == 'prismatic' else 'revolute'
        kp = Ar['p_gain'][kind]
        vmax = f(min(float(next(jj.find('limit') for jj in robot.findall('joint') if jj.get('name') == j).get('velocity')), Ar['max_speed'][kind]))
        p = plugin(robot, 'gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController')
        sub(p, 'joint_name', j)
        sub(p, 'topic', Ar['cmd_topic_format'].format(joint=j).lstrip('/'))
        sub(p, 'use_velocity_commands', str(Ar['use_velocity_commands']).lower())
        sub(p, 'p_gain', kp)
        sub(p, 'i_gain', 0)
        sub(p, 'd_gain', 0)
        sub(p, 'cmd_max', vmax)
        sub(p, 'cmd_min', f'-{vmax}')
        sub(p, 'initial_position', 0)
    return S


def write_bridge(S, path):
    """ros_gz_bridge parameter_bridge 설정 (config_file) 을 sensors.yaml 의 토픽 이름으로 만든다."""
    G2R, R2G = 'GZ_TO_ROS', 'ROS_TO_GZ'
    rows = [('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', G2R)]
    for u in S['lds03']['units'].values():
        rows.append((u['topic'], u['topic'], 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', G2R))
    rows.append((S['tfmini_plus']['sim']['scan_topic'],) * 2 + ('sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', G2R))
    img, info = ('sensor_msgs/msg/Image', 'gz.msgs.Image'), ('sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo')
    fixes = []                                                  # camera_info 는 /gz/... 로 받아 camera_info_fix.py 가 Tx 를 고쳐 낸다
    # D435 깊이 영상 · 점구름도 /gz/... 로 받아 camera_info_fix.py 가 URDF optical frame 기준으로 고쳐 낸다:
    # 센서가 광축으로 sensor_forward_m 앞에 있어 깊이 + 그만큼, 점구름은 Gazebo 센서 축 (x 앞 · y 왼 · z 위) -> optical (x 오른 · y 아래 · z 앞)
    fwd = float(S['d435']['sim']['sensor_forward_m'])
    depth_fixes, point_fixes = [], []
    for u in S['d435']['units'].values():
        b = u['prefix']
        depth_fixes.append({'in': f'/gz{b}/aligned_depth_to_color/image_raw', 'out': f'{b}/aligned_depth_to_color/image_raw', 'add_m': fwd})
        point_fixes.append({'in': f'/gz{b}/depth/color/points', 'out': f'{b}/depth/color/points', 'add_m': fwd})

    def cinfo(ros, gz, baseline=0.0):
        fixes.append(dict(**{'in': f'/gz{ros}', 'out': ros, 'baseline_m': baseline}))
        return (f'/gz{ros}', gz) + info + (G2R,)
    for u in S['d435']['units'].values():
        b = u['prefix']
        rows += [(f'{b}/color/image_raw',) * 2 + img + (G2R,),
                 cinfo(f'{b}/color/camera_info', f'{b}/color/camera_info'),
                 (f'/gz{b}/aligned_depth_to_color/image_raw', f'{b}/aligned_depth_to_color/image_raw') + img + (G2R,),
                 cinfo(f'{b}/aligned_depth_to_color/camera_info', f'{b}/aligned_depth_to_color/camera_info'),
                 (f'/gz{b}/depth/color/points', f'{b}/aligned_depth_to_color/image_raw/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked', G2R)]
        if S['d435']['sim']['ir_stereo']:
            for ir in ('infra1', 'infra2'):
                bl = S['d435']['spec']['stereo_baseline_m'] if ir == 'infra2' else 0.0
                rows += [(f'{b}/{ir}/image_rect_raw',) * 2 + img + (G2R,), cinfo(f'{b}/{ir}/camera_info', f'{b}/{ir}/camera_info', bl)]
    a = S['arm_camera']['topic_prefix']
    rows += [(f'{a}/image_raw',) * 2 + img + (G2R,), cinfo(f'{a}/camera_info', f'{a}/camera_info')]
    for u in S['imu']['units'].values():
        rows.append((u['topic'],) * 2 + ('sensor_msgs/msg/Imu', 'gz.msgs.IMU', G2R))
    Dr = S['drive']['sim']
    rows += [(Dr['odom_topic'],) * 2 + ('nav_msgs/msg/Odometry', 'gz.msgs.Odometry', G2R),
             ('/tf', '/tf', 'tf2_msgs/msg/TFMessage', 'gz.msgs.Pose_V', G2R),
             (Dr['ground_truth_topic'],) * 2 + ('nav_msgs/msg/Odometry', 'gz.msgs.Odometry', G2R),
             ('/joint_states', '/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', G2R),
             (Dr['cmd_topic'],) * 2 + ('geometry_msgs/msg/TwistStamped', 'gz.msgs.Twist', R2G)]
    for j in S['arm']['sim']['joints']:
        t = S['arm']['sim']['cmd_topic_format'].format(joint=j)
        rows.append((t, t, 'std_msgs/msg/Float64', 'gz.msgs.Double', R2G))
    out = ['# ros_gz_bridge parameter_bridge 설정 - tools/make_revision.py 가 config/sensors.yaml 로 만듦 (직접 고치지 말 것)',
           '# 카메라 (영상 · 깊이 · 점구름 · camera_info) 는 lazy: ROS 쪽 구독이 있을 때만 Gazebo 가 렌더한다 (안 쓰는 카메라가 시뮬 속도를 깎지 않게)']
    for r, g, rt, gt, d in rows:
        out += [f'- ros_topic_name: "{r}"', f'  gz_topic_name: "{g.lstrip("/")}"', f'  ros_type_name: "{rt}"', f'  gz_type_name: "{gt}"',
                f'  direction: {d}']
        if rt in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CameraInfo', 'sensor_msgs/msg/PointCloud2'):
            out.append('  lazy: true')
    path.write_text('\n'.join(out) + '\n', encoding='utf-8')
    fix = path.parent / 'camera_info_fix.yaml'
    fix.write_text('# scripts/camera_info_fix.py 설정 - tools/make_revision.py 가 config/sensors.yaml 로 만듦 (직접 고치지 말 것)\n'
                   + yaml.safe_dump({'camera_info': fixes, 'depth_image': depth_fixes, 'points': point_fixes}, sort_keys=False, allow_unicode=True),
                   encoding='utf-8')
    return len(rows)
