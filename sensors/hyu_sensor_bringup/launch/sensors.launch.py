"""Vehicle sensor bringup: LiDAR + ZED + SBG under /sensors, plus the TF chain.

This is step 1 of the vehicle flow (fsk.sh pane ①). The simulator publishes the
gazebo driver topics; this file is what makes the real car look the same shape
to everything downstream.

Topics (the vehicle half of docs/topic_contract.md):
    /sensors/lidar/points                        PointCloud2 (frame: rslidar)
    /sensors/zed/left/color/rect/image           Image
    /sensors/zed/left/color/rect/camera_info     CameraInfo
    /sensors/zed/right/color/rect/image          Image
    /sbg/*                                       driver defaults

The camera topics are the ZED wrapper's own names placed under /sensors by
namespace only — nothing is renamed and nothing is relayed, so there is no
extra copy of a 30 Hz raw image. camera_name stays 'zed' because it names the
TF frames too (zed_left_camera_optical_frame), which must match the simulator.

TF:
    base_footprint -> rslidar                        config/vehicle_mount.yaml
    base_footprint -> zed_left_camera_optical_frame  mount o extrinsic^-1

The camera pose is COMPOSED, never configured: the mount yaml anchors the
LiDAR, the calibration says where the camera is relative to it. That is why
the ZED wrapper's own TF publishing is off here — two parents for the camera
frame would be a silent, drifting conflict.

Usage:
    ros2 launch hyu_sensor_bringup sensors.launch.py
    ros2 launch hyu_sensor_bringup sensors.launch.py tf:=false     # calib capture
    ros2 launch hyu_sensor_bringup sensors.launch.py extrinsic:=/path/to.yaml
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

PACKAGE = 'hyu_sensor_bringup'

# Default extrinsic: the workspace-level store that calib.sh writes and points
# 'active' at. Absolute because launch does not run from the workspace root.
DEFAULT_EXTRINSIC = os.path.expanduser('~/fsk/extrinsics/active')


def _rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def _mat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def _quat_from_mat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _static_tf(name, parent, child, xyz, quat, use_sim_time='false'):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time.lower() in ('true', '1')}],
        arguments=['--x', f'{xyz[0]:.6f}', '--y', f'{xyz[1]:.6f}',
                   '--z', f'{xyz[2]:.6f}',
                   '--qx', f'{quat[0]:.9f}', '--qy', f'{quat[1]:.9f}',
                   '--qz', f'{quat[2]:.9f}', '--qw', f'{quat[3]:.9f}',
                   '--frame-id', parent, '--child-frame-id', child])


def _tf_actions(context, share):
    """base->rslidar from the mount, base->camera composed with the extrinsic."""
    import yaml

    if context.launch_configurations['tf'].lower() not in ('true', '1'):
        return [LogInfo(msg='[sensors] tf:=false — no TF published '
                            '(calibration capture mode).')]

    mount_path = context.launch_configurations['mount']
    with open(mount_path) as f:
        lidar = (yaml.safe_load(f) or {})['lidar']
    lx, ly, lz = float(lidar['x']), float(lidar['y']), float(lidar['z'])
    lr = math.radians(float(lidar.get('roll_deg', 0.0)))
    lp = math.radians(float(lidar.get('pitch_deg', 0.0)))
    lyaw = math.radians(float(lidar.get('yaw_deg', 0.0)))

    actions = [
        LogInfo(msg=f'[sensors] mount {os.path.basename(mount_path)}: '
                    f'lidar xyz=({lx:.3f}, {ly:.3f}, {lz:.3f}) '
                    f'yaw={math.degrees(lyaw):+.2f}deg'),
        _static_tf('mount_tf_lidar', 'base_footprint', 'rslidar',
                   (lx, ly, lz), _rpy_to_quat(lr, lp, lyaw),
                   context.launch_configurations['use_sim_time']),
    ]

    ext_path = context.launch_configurations['extrinsic']
    if not os.path.exists(ext_path):
        actions.append(LogInfo(msg=(
            f'[sensors] extrinsic not found: {ext_path} — camera TF NOT '
            f'published. Perception will not project. Run ./calib.sh, or pass '
            f'extrinsic:=<yaml>.')))
        return actions

    with open(ext_path) as f:
        ext = yaml.safe_load(f) or {}
    # camera_to_lidar is the camera-frame pose OF the lidar; we need the
    # lidar-frame pose of the camera, i.e. rslidar -> camera, which is exactly
    # the stored lidar_to_camera block (parent camera, child rslidar) inverted
    # once more — the file already carries both, so read the one we need.
    c2l = ext.get('camera_to_lidar') or {}
    l2c = ext.get('lidar_to_camera') or {}
    if not l2c:
        actions.append(LogInfo(msg=f'[sensors] {ext_path} has no '
                                   f'lidar_to_camera block — camera TF skipped.'))
        return actions

    # lidar_to_camera: camera_point = R * lidar_point + t, i.e. the transform
    # that takes lidar coords to camera coords. As a POSE that is camera->lidar,
    # so the lidar-frame pose of the camera is its inverse: R^T, -R^T t.
    R = l2c['R']
    t = l2c['t']
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    cam_in_lidar = [-sum(Rt[i][k] * t[k] for k in range(3)) for i in range(3)]

    # base -> camera = (base -> rslidar) o (rslidar -> camera)
    Rm = _mat_from_rpy(lr, lp, lyaw)
    Rbc = _matmul(Rm, Rt)
    tbc = [lx, ly, lz]
    for i in range(3):
        tbc[i] += sum(Rm[i][k] * cam_in_lidar[k] for k in range(3))

    cam_frame = context.launch_configurations['camera_frame']
    stored_parent = l2c.get('parent_frame', '?')
    if stored_parent != cam_frame:
        actions.append(LogInfo(msg=(
            f"[sensors] NOTE: extrinsic names the camera frame "
            f"'{stored_parent}' but the stack uses '{cam_frame}'. Publishing "
            f"as '{cam_frame}' so TF resolves; fix the yaml to stop the drift.")))

    actions.append(_static_tf('mount_tf_camera', 'base_footprint', cam_frame,
                              tbc, _quat_from_mat(Rbc),
                              context.launch_configurations['use_sim_time']))
    actions.append(LogInfo(msg=(
        f'[sensors] camera TF from {os.path.basename(ext_path)}: '
        f'base_footprint -> {cam_frame} '
        f't=({tbc[0]:+.3f}, {tbc[1]:+.3f}, {tbc[2]:+.3f}) '
        f"metrics: plane={(ext.get('metrics') or {}).get('rmse_plane_mm', '-')}mm")))
    return actions


def _setup(context, *_args, **_kwargs):
    share = get_package_share_directory(PACKAGE)
    actions = []

    # --- RS-16: vendor node, team config (topics already under /sensors/lidar) ---
    if context.launch_configurations['lidar'].lower() != 'off':
        actions.append(Node(
            package='rslidar_sdk', executable='rslidar_sdk_node',
            name='lidar_driver', namespace='sensors', output='screen',
            parameters=[{'config_path': os.path.join(share, 'config',
                                                     'rslidar.yaml')}]))

    # --- ZED: vendor launch, unmodified, namespaced into /sensors/zed ---
    # namespace + camera_name together decide BOTH the topic root and the TF
    # frame prefix. camera_name must stay 'zed' so the frames match the sim.
    if context.launch_configurations['camera'].lower() != 'off':
        zed_launch = os.path.join(get_package_share_directory('zed_wrapper'),
                                  'launch', 'zed_camera.launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zed_launch),
            launch_arguments={
                'camera_model': context.launch_configurations['camera_model'],
                'namespace': 'sensors',
                'camera_name': 'zed',
                'enable_ipc': 'false',
                'publish_tf': 'false',      # we own the camera frame, see _tf_actions
                'publish_map_tf': 'false',
                'ros_params_override_path': os.path.join(share, 'config',
                                                         'zed_override.yaml'),
            }.items()))

    # --- SBG GNSS/INS (optional): 장치가 없으면 조용히 스킵 ---
    gnss = context.launch_configurations['gnss'].lower()
    if gnss != 'off':
        sbg_cfg = os.path.join(share, 'config', 'sbg.yaml')
        port = '/dev/ttyUSB0'
        try:
            import yaml
            with open(sbg_cfg) as f:
                port = yaml.safe_load(f)['/**']['ros__parameters']['uartConf']['portName']
        except Exception:
            pass
        if gnss == 'on' or os.path.exists(port):
            # NOT namespaced. The driver publishes relative topics (sbg/ekf_nav,
            # imu/data, ...), so a 'sensors' namespace would put them on
            # /sensors/sbg/* while hyu_localization's sbg_odometry_bridge and
            # wheel_odometry both subscribe /sbg/* -- the INS chain would sit
            # silent with the driver visibly running. docs/topic_contract.md
            # lists /sbg/* as the driver default; keep it that way.
            actions.append(Node(
                package='sbg_driver', executable='sbg_device',
                name='gnss_driver', output='screen',
                parameters=[sbg_cfg]))
        else:
            actions.append(LogInfo(msg=(
                f'[sensors] GNSS not connected ({port} absent) — skipping '
                f'sbg_driver (force with gnss:=on)')))

    # --- odometry conditioning ------------------------------------------------
    # Raw driver output -> odometry. These live here, not in step 2's INS
    # pipeline, because every input they read is a step-1 topic:
    #   wheel_odometry      /vehicle/wheel_speeds, /sbg/ekf_nav, /sbg/ekf_rot_accel_body
    #   sbg_odometry_bridge /sbg/ekf_nav, /sbg/ekf_euler
    # and because perception's LiDAR deskew subscribes /localization/wheel_odom.
    # Started in step 2 it arrived a whole step late and perception ran the
    # entire of step 1 with the scan uncorrected.
    if context.launch_configurations['odometry'].lower() not in ('false', '0'):
        use_sim_time = context.launch_configurations['use_sim_time'].lower() in ('true', '1')
        actions.append(Node(
            package='hyu_localization', executable='wheel_odometry',
            name='wheel_odometry', output='screen',
            parameters=[{'use_sim_time': use_sim_time}]))
        actions.append(Node(
            package='hyu_localization', executable='sbg_odometry_bridge',
            name='sbg_odometry_bridge', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'car_state_topic':
                             context.launch_configurations['ins_odom_topic']}]))

    actions.extend(_tf_actions(context, share))
    return actions


def generate_launch_description():
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='on',
                              choices=['on', 'off']),
        DeclareLaunchArgument('camera', default_value='on',
                              choices=['on', 'off']),
        DeclareLaunchArgument('camera_model', default_value='zed'),
        DeclareLaunchArgument('gnss', default_value='auto',
                              choices=['auto', 'on', 'off'],
                              description='auto = start only if the serial '
                                          'port exists'),
        DeclareLaunchArgument('tf', default_value='true',
                              description='false = sensors only, no TF '
                                          '(calibration capture)'),
        DeclareLaunchArgument('mount',
                              default_value=os.path.join(share, 'config',
                                                         'vehicle_mount.yaml'),
                              description='base_footprint -> rslidar geometry'),
        DeclareLaunchArgument('extrinsic', default_value=DEFAULT_EXTRINSIC,
                              description='lidar<->camera calibration yaml'),
        DeclareLaunchArgument('camera_frame',
                              default_value='zed_left_camera_optical_frame',
                              description='must match hyu_perception camera_frame'),
        # Bag replay drives every stamp off /clock; on the car this stays false.
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('odometry', default_value='true',
                              description='Start wheel_odometry and '
                                          'sbg_odometry_bridge. Step 2 must be '
                                          'told odometry:=false when this is on, '
                                          'or both run twice.'),
        DeclareLaunchArgument('ins_odom_topic',
                              default_value='/localization/ins_odom',
                              description='must match ins_pipeline'),
        OpaqueFunction(function=_setup),
    ])
