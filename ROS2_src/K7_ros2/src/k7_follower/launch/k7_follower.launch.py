"""多机编队 —— 跟随者整车 launch（从机一键盘启动）。

在从机板上运行本 launch = 轻量串口节点 + twist_mux + 编队跟随节点，全部落在
namespace 下（默认 follower1）。主车 STM32 自主运行时使用 run/start_leader.sh，
ROS 侧只读取主车原始 odom，绝不向 STM32 下发速度。

用法（在 K7 板 K7_ros2 工作区根，source 后）：
    ros2 launch k7_follower k7_follower.launch.py namespace:=follower1
    ros2 launch k7_follower k7_follower.launch.py namespace:=follower2 distance:=1.2 \
        leader_topic:=/follower1/odom
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    distance = LaunchConfiguration('distance')
    leader_topic = LaunchConfiguration('leader_topic')

    bringup_share = get_package_share_directory('k7_bringup')
    follower_share = get_package_share_directory('k7_follower')
    follower_cfg = Path(follower_share, 'config', 'follower.yaml')

    # 1. 仅底盘串口节点。原始轮式里程计足够跟随，无需 EKF/TF/URDF。
    k7_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'k7_core.launch.py')),
        launch_arguments={
            'namespace': ns,
            'serial_only': 'true',
            'enable_downlink': 'true',
        }.items(),
    )

    # 2. twist_mux 仲裁（k7_bringup/k7_twist_mux.launch.py，带 namespace）
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'k7_twist_mux.launch.py')),
        launch_arguments={'namespace': ns}.items(),
    )

    # 3. 编队跟随节点（namespace 由其自身参数传入，避免与上面 include 双重 push）
    follower = Node(
        package='k7_follower',
        executable='follower_node',
        name='follower_node',
        namespace=ns,
        output='screen',
        parameters=[str(follower_cfg),
                    {'target_gap': distance, 'leader_topic': leader_topic}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='follower1',
            description='本从机命名空间：follower1 / follower2（须与其它车不同）'),
        DeclareLaunchArgument(
            'distance', default_value='1.2',
            description='跟随保持的纵向间距 (m)'),
        DeclareLaunchArgument(
            'leader_topic', default_value='/leader/odom',
            description='前车里程计话题；多级编队可指向上一台从车'),
        k7_core,
        twist_mux,
        follower,
    ])
