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
    base_footprint -> zed_left_camera_optical_frame  the extrinsic's `ground`
                                                     block (camera height / tilt
                                                     over the ground plane)
    base_footprint -> rslidar                        camera o extrinsic
    base_footprint -> zed_camera_link                the SAME camera pose
                                                     re-expressed at the ZED
                                                     URDF's root, so the wrapper's
                                                     own frames (right camera
                                                     included) hang off the car
                                                     instead of floating

base_footprint is DEFINED by the camera: the point on the ground directly
below the ZED's stereo centre, +x = camera forward projected onto the ground,
+z = ground normal. It is NOT the rear axle. Nothing here is a tape measure:
solve_mount.py reads the ground plane out of the LiDAR through the extrinsic
and stores height/roll/pitch next to it (calib.sh does this after every
solve), and the LiDAR pose is composed from that plus the extrinsic. Re-mount
the LiDAR -> re-calibrate -> everything follows. An extrinsic without a
`ground` block (pre-2026-08 files) falls back to config/vehicle_mount.yaml.

The ZED wrapper's own TF publishing is off here — two parents for the camera
frame would be a silent, drifting conflict.

Usage:
    ros2 launch hyu_sensor_bringup sensors.launch.py
    ros2 launch hyu_sensor_bringup sensors.launch.py tf:=false     # calib capture
    ros2 launch hyu_sensor_bringup sensors.launch.py extrinsic:=/path/to.yaml
    ros2 launch hyu_sensor_bringup sensors.launch.py ntrip:=true   # + RTK
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


# ROS body (x fwd, y left, z up) -> optical (x right, y down, z fwd): the ZED
# URDF's *_camera_frame -> *_camera_frame_optical joint, rpy (-pi/2, 0, -pi/2).
R_BODY_OPT = [[0.0, 0.0, 1.0],
              [-1.0, 0.0, 0.0],
              [0.0, -1.0, 0.0]]


def _compose(r_a, t_a, r_b, t_b):
    """(a o b): apply b, then a."""
    return (_matmul(r_a, r_b),
            [t_a[i] + sum(r_a[i][k] * t_b[k] for k in range(3)) for i in range(3)])


def _invert(r, t):
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    return rt, [-sum(rt[i][k] * t[k] for k in range(3)) for i in range(3)]


def _camera_over_ground(context, ext, ext_path):
    """(height, roll, pitch, where-from) of the left optical frame over the ground.

    Preferred source is the `ground` block solve_mount.py stores IN the
    extrinsic: the plane is expressed through that exact R,t, so the two can
    only ever be switched together (calib -e). config/vehicle_mount.yaml is the
    fallback for extrinsics that predate the block -- tape values, and it says
    so in the log.
    """
    import yaml
    g = ext.get('ground') or {}
    if 'camera_height_m' in g:
        return (float(g['camera_height_m']),
                math.radians(float(g.get('camera_roll_deg', 0.0))),
                math.radians(float(g.get('camera_pitch_deg', 0.0))),
                f"{os.path.basename(ext_path)} ground block "
                f"({g.get('source', '?')}, {g.get('n_frames', '?')} frames, "
                f"std {g.get('height_std_mm', '?')} mm)")
    mount_path = context.launch_configurations['mount']
    with open(mount_path) as f:
        cam = (yaml.safe_load(f) or {}).get('camera') or {}
    if 'height' not in cam:
        raise ValueError(f'{mount_path} has no camera.height')
    return (float(cam['height']),
            math.radians(float(cam.get('roll_deg', 0.0))),
            math.radians(float(cam.get('pitch_deg', 0.0))),
            f"{os.path.basename(mount_path)} FALLBACK -- the extrinsic has no "
            f"ground block; run solve_mount.py --write to measure it")


def _tf_actions(context, share):
    """base_footprint below the camera; the LiDAR composed with the extrinsic."""
    import yaml

    lc = context.launch_configurations
    if lc['tf'].lower() not in ('true', '1'):
        return [LogInfo(msg='[sensors] tf:=false — no TF published '
                            '(calibration capture mode).')]

    ext_path = lc['extrinsic']
    ext = {}
    if os.path.exists(ext_path):
        with open(ext_path) as f:
            ext = yaml.safe_load(f) or {}
    l2c = ext.get('lidar_to_camera') or {}

    try:
        h, roll, pitch, origin = _camera_over_ground(context, ext, ext_path)
    except (OSError, ValueError, KeyError) as exc:
        return [LogInfo(msg=(f'[sensors] no camera height available ({exc}) — '
                             f'NO TF published. Run ./calib.sh (solve_mount.py '
                             f'writes it) or fill config/vehicle_mount.yaml.'))]

    # base -> left optical, with the LEFT lens on the plumb line for now.
    r_bopt = _matmul(_mat_from_rpy(roll, pitch, 0.0), R_BODY_OPT)
    t_bopt = [0.0, 0.0, h]

    # Slide base so the STEREO CENTRE, not the left lens, is what sits above
    # it. Pure horizontal shift; the height stays that of the left optical
    # centre (the two differ by ~1 mm through the roll).
    actions = []
    geom = _zed_geometry(context)
    if geom is None:
        below = 'LEFT lens (ZED URDF not resolved; stereo centre is baseline/2 to its right)'
    else:
        below = 'stereo centre'
        _, t_center = _compose(r_bopt, t_bopt, *_invert(*geom['center_to_opt']))
        t_bopt[0] -= t_center[0]
        t_bopt[1] -= t_center[1]

    cam_frame = lc['camera_frame']
    actions.append(_static_tf('mount_tf_camera', 'base_footprint', cam_frame,
                              t_bopt, _quat_from_mat(r_bopt), lc['use_sim_time']))
    actions.append(LogInfo(msg=(
        f'[sensors] base_footprint := ground below the {below}; camera '
        f'{h:.3f} m up, roll {math.degrees(roll):+.2f} pitch '
        f'{math.degrees(pitch):+.2f} deg  <- {origin}')))

    # LiDAR: the extrinsic block IS the camera->rslidar pose (camera_point =
    # R lidar_point + t), so base -> rslidar = (base -> camera) o (R, t).
    if not l2c:
        actions.append(LogInfo(msg=(
            f'[sensors] extrinsic {ext_path} missing or without lidar_to_camera '
            f'— rslidar TF NOT published. Perception cannot place the cloud. '
            f'Run ./calib.sh, or pass extrinsic:=<yaml>.')))
    else:
        if l2c.get('parent_frame', cam_frame) != cam_frame:
            actions.append(LogInfo(msg=(
                f"[sensors] NOTE: extrinsic names the camera frame "
                f"'{l2c.get('parent_frame')}' but the stack uses '{cam_frame}'. "
                f"Publishing as '{cam_frame}' so TF resolves.")))
        r_bl, t_bl = _compose(r_bopt, t_bopt, l2c['R'], l2c['t'])
        actions.append(_static_tf('mount_tf_lidar', 'base_footprint', 'rslidar',
                                  t_bl, _quat_from_mat(r_bl), lc['use_sim_time']))
        actions.append(LogInfo(msg=(
            f'[sensors] rslidar from {os.path.basename(ext_path)}: '
            f'base_footprint -> rslidar t=({t_bl[0]:+.3f}, {t_bl[1]:+.3f}, '
            f'{t_bl[2]:+.3f}) yaw={math.degrees(math.atan2(r_bl[1][0], r_bl[0][0])):+.2f}deg '
            f"plane={(ext.get('metrics') or {}).get('rmse_plane_mm', '-')}mm  "
            f'(nose LiDAR ~1.7 m ahead of the hoop camera; z should match '
            f"ground.lidar_height_m={(ext.get('ground') or {}).get('lidar_height_m', '-')})")))

    if geom is not None:
        # Graft the wrapper's URDF island onto the car: base -> zed_camera_link
        # = (base -> left optical) o (zed_camera_link -> left optical)^-1.
        r_root, t_root = _compose(r_bopt, t_bopt, *_invert(*geom['link_to_opt']))
        actions.append(_static_tf('mount_tf_zed_urdf', 'base_footprint',
                                  geom['root_link'], t_root,
                                  _quat_from_mat(r_root), lc['use_sim_time']))
        actions.append(LogInfo(msg=(
            f"[sensors] ZED URDF anchored: base_footprint -> {geom['root_link']} "
            f't=({t_root[0]:+.3f}, {t_root[1]:+.3f}, {t_root[2]:+.3f})')))
    return actions


def _zed_geometry(context):
    """Fixed poses inside the ZED URDF, or None if it cannot be resolved.

    The wrapper's robot_state_publisher emits zed_camera_link -> ... ->
    zed_left/right_camera_frame_optical as a FLOATING ISLAND: its root has no
    parent here, because the calibrated camera pose lands on a different frame
    name (zed_left_camera_optical_frame). Two things are read out of the same
    xacro the wrapper loads (so a camera_model change follows automatically):

        center_to_opt   zed_camera_center -> left optical   (baseline/2 etc.)
        link_to_opt     zed_camera_link   -> left optical   (URDF root)

    The first puts base_footprint under the stereo centre, the second anchors
    the island so the URDF's own left optical frame lands EXACTLY on the
    calibrated one. Neither adds geometry; both only re-express it.

    Anything unexpected returns None and the caller degrades: an unparented
    subtree is untidy, a wrongly parented one lies.
    """
    import xml.etree.ElementTree as ET

    # Deliberately NOT gated on camera:=off: base_footprint's definition must
    # not depend on whether the driver runs (bagplay replays with camera:=off).
    # 'zed' is the camera_name handed to the wrapper below; it prefixes every
    # frame in the URDF, so the two must not drift apart.
    lc = context.launch_configurations
    name, model = 'zed', lc['camera_model']
    root_link, center, leaf = (f'{name}_camera_link', f'{name}_camera_center',
                               f'{name}_left_camera_frame_optical')
    try:
        import xacro
        doc = xacro.process_file(
            os.path.join(get_package_share_directory('zed_wrapper'), 'urdf',
                         'zed_descr.urdf.xacro'),
            mappings={'camera_name': name, 'camera_model': model})
        urdf = ET.fromstring(doc.toxml())
        by_child = {}
        for j in urdf.findall('joint'):
            org = j.find('origin')
            by_child[j.find('child').get('link')] = (
                j.find('parent').get('link'),
                [float(v) for v in (org.get('xyz') or '0 0 0').split()],
                [float(v) for v in (org.get('rpy') or '0 0 0').split()])

        chain, node = [], leaf                  # leaf -> root, one joint each
        while node != root_link:
            parent, xyz, rpy = by_child[node]   # KeyError = not our tree
            chain.append((parent, xyz, rpy))
            node = parent
        if not any(p == center for p, _, _ in chain):
            raise KeyError(center)
    except Exception as exc:                    # noqa: BLE001 - fail soft
        print(f'[sensors] ZED URDF not resolved ({exc})')
        return None

    def pose_from(start):
        rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        trans = [0.0, 0.0, 0.0]
        below = list(reversed(chain))
        idx = next(i for i, (p, _, _) in enumerate(below) if p == start)
        for _, xyz, rpy in below[idx:]:
            rot, trans = _compose(rot, trans, _mat_from_rpy(*rpy), xyz)
        return rot, trans

    return {'root_link': root_link,
            'link_to_opt': pose_from(root_link),
            'center_to_opt': pose_from(center)}

def _ntrip_actions(context):
    """RTK corrections, opt-in with ntrip:=true.

    Three things must line up or this is a no-op, and only one of them is here:
      1. this node, fetching RTCM into /ntrip_client/rtcm      <- ntrip:=true
      2. config/sbg.yaml  rtcm.subscribe: true                  (driver forwards
         the bytes to the device over the same serial link)
      3. the DEVICE's aiding/diffCorr/source = comA             (flash config,
         set with sbgEComApi -- see localization/docs/sbg_commands.md)

    Defaults are the NGII network-RTK values verified 2026-07-24 (RTS1, not
    RTS2: RTS2 returns 401 for this account, and its mountpoint differs).
    VRS mounts send nothing until the rover uplinks a GGA, so send_gga stays
    on and the initial lat/lon seed the first one before any fix exists.
    """
    lc = context.launch_configurations
    if lc['ntrip'].lower() not in ('true', '1'):
        return []

    return [
        Node(package='sbg_driver', executable='ntrip_client',
             name='ntrip_client', output='screen',
             parameters=[{
                 'host': lc['ntrip_host'],
                 'port': int(lc['ntrip_port']),
                 'mountpoint': lc['ntrip_mountpoint'],
                 'username': lc['ntrip_user'],
                 'password': lc['ntrip_pass'],
                 'send_gga': True,
                 'initial_latitude': float(lc['ntrip_lat']),
                 'initial_longitude': float(lc['ntrip_lon']),
             }]),
        LogInfo(msg=(f"[sensors] NTRIP {lc['ntrip_host']}:{lc['ntrip_port']}/"
                     f"{lc['ntrip_mountpoint']} -> /ntrip_client/rtcm "
                     f"(check: ros2 topic hz /ntrip_client/rtcm, then "
                     f"/sbg/gps_pos status.type 6=RTK float 7=RTK fixed)")),
    ]


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
            actions.extend(_ntrip_actions(context))
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
        # RTK is on by default: the car wants a fixed solution, and a GNSS
        # bringup that silently runs single-point is the failure we keep
        # rediscovering. Costs a warn every reconnect_delay (5 s) when there
        # is no network -- 'ntrip:=false' for a bench run without one.
        DeclareLaunchArgument('ntrip', default_value='true',
                              choices=['true', 'false'],
                              description='fetch RTCM corrections for RTK'),
        DeclareLaunchArgument('ntrip_host', default_value='RTS1.ngii.go.kr'),
        DeclareLaunchArgument('ntrip_port', default_value='2101'),
        DeclareLaunchArgument('ntrip_mountpoint', default_value='VRS-RTCM31'),
        DeclareLaunchArgument('ntrip_user', default_value='shchon11'),
        DeclareLaunchArgument('ntrip_pass', default_value='ngii'),
        # No seed by default: a VRS caster hands out a virtual base AT the
        # position you uplink, so a stale one (the device's localParam still
        # says Gwangju) picks a reference station hundreds of km away. Left
        # NaN, the client simply waits for the receiver's own first fix --
        # seconds outdoors. Pass both to prime it before any fix exists.
        DeclareLaunchArgument('ntrip_lat', default_value='nan',
                              description='seeds the first VRS GGA (deg)'),
        DeclareLaunchArgument('ntrip_lon', default_value='nan'),
        DeclareLaunchArgument('tf', default_value='true',
                              description='false = sensors only, no TF '
                                          '(calibration capture)'),
        DeclareLaunchArgument('mount',
                              default_value=os.path.join(share, 'config',
                                                         'vehicle_mount.yaml'),
                              description='camera height/tilt FALLBACK, used '
                                          'only when the extrinsic has no '
                                          'ground block'),
        DeclareLaunchArgument('extrinsic', default_value=DEFAULT_EXTRINSIC,
                              description='lidar<->camera calibration yaml '
                                          '(+ ground block from solve_mount.py)'),
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
