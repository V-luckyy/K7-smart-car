# k7_bringup/launch/k7_twist_mux.launch.py
# 启动 twist_mux 多路速度仲裁：cmd_vel_joy + cmd_vel_key + cmd_vel_follower + cmd_vel_mpc → cmd_vel
# 优先级 joy(100) > keyboard(80) > follower(70) > mpc(50)，见 config/twist_mux.yaml
# namespace：单机留空（默认）；多机编队传 namespace:=leader/follower1/follower2 等。
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    bringup_dir = get_package_share_directory('k7_bringup')
    twist_mux_config = Path(bringup_dir, 'config', 'twist_mux.yaml')
    namespace = LaunchConfiguration('namespace')

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[str(twist_mux_config)],
        remappings=[('cmd_vel_out', 'cmd_vel')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='节点命名空间：单机留空；多机编队传 leader/follower1/follower2 等'),
        GroupAction(
            actions=[PushRosNamespace(namespace), twist_mux],
        ),
    ])
