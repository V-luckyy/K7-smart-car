import hashlib
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build(context):
    arg = lambda key: LaunchConfiguration(key).perform(context)
    profile, algorithm, namespace = arg('profile'), arg('algorithm'), arg('namespace').strip('/')
    test_mode = arg('test_mode')
    if profile not in ('manual', 'trajectory', 'formation'):
        raise ValueError('profile must be manual, trajectory or formation')
    if algorithm != 'apf_stanley':
        raise ValueError('algorithm must be apf_stanley; other algorithms are not implemented')
    if test_mode not in ('none', 'no_perception'):
        raise ValueError('test_mode must be none or no_perception')
    if test_mode == 'no_perception' and profile not in ('trajectory', 'formation'):
        raise ValueError('no_perception test mode supports trajectory and formation profiles only')
    no_perception_test = test_mode == 'no_perception'
    config_file = os.path.abspath(arg('config'))
    with open(config_file, 'rb') as stream:
        config_bytes = stream.read()
    config = yaml.safe_load(config_bytes)
    prefix = namespace+'/' if namespace else ''
    base, odom = prefix+'base_link', prefix+'odom'
    common = config['common']
    nodes = []

    def node(package, executable, extra=None, common_keys=(), remappings=None):
        params = dict(config.get(executable, {}))
        params.update({k: common[k] for k in common_keys})
        params.update(extra or {})
        nodes.append(Node(package=package, executable=executable, name=executable,
            namespace=namespace, parameters=[params], remappings=remappings or [], output='screen'))

    node('k7_bringup', 'serial_bridge', dict(base_frame=base, odom_frame=odom, imu_frame=prefix+'gyro_link'),
         remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')])
    if not no_perception_test:
        # Capture/decode and matching are the only high-bandwidth processes. No splitter or cloud.
        camera = dict(config['usb_cam'])
        nodes.append(Node(package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
            namespace=namespace, parameters=[camera],
            remappings=[('image_raw', 'camera/image_raw'), ('camera_info', 'camera/camera_info')], output='screen'))
        node('k7_camera', 'stereo_obstacles', dict(base_frame=base, optical_frame=prefix+'camera_left_optical_frame'),
             ('sensor_timeout',))
    node('k7_control', 'command_guard', dict(base_frame=base, odom_frame=odom,
         no_perception_test=no_perception_test),
         ('track', 'wheel_limit', 'sensor_timeout', 'stop_distance'))
    node('k7_tasks', 'task_manager', dict(profile=profile, base_frame=base, odom_frame=odom,
        publish_fleet=arg('publish_fleet').lower() == 'true'), ('wheel_limit', 'sensor_timeout'))
    if profile != 'manual':
        node('k7_control', 'controller_node', dict(algorithm=algorithm, base_frame=base, odom_frame=odom,
             no_perception_test=no_perception_test, test_speed_limit=config['command_guard']['test_speed_limit']),
             ('track', 'wheel_limit', 'sensor_timeout', 'stop_distance'))
    node('k7_experiment', 'experiment_recorder', dict(enabled=arg('record').lower() == 'true',
         config_file=config_file, config_sha256=hashlib.sha256(config_bytes).hexdigest(),
         profile=profile, algorithm=algorithm, test_mode=test_mode))
    return nodes


def generate_launch_description():
    default = os.path.join(get_package_share_directory('k7_bringup'), 'config', 'robot.yaml')
    arguments = dict(profile='trajectory', algorithm='apf_stanley', namespace='car1',
                     config=default, publish_fleet='false', record='false', test_mode='none')
    return LaunchDescription([DeclareLaunchArgument(key, default_value=value) for key, value in arguments.items()]
                             + [OpaqueFunction(function=build)])
