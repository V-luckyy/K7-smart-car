"""APF + Stanley 实车调参 —— 一键记录。

前置：STM32 已烧含 0xFB 调试帧的固件，且正在自跑 APF+Stanley 圆。
本 launch 在 K7 上做两件事：
  1. k7_serial_node 以 enable_downlink:=false（只收不发）解析串口上行，
     其中 0xFB 调试帧 -> /apf_debug（不发任何指令，不影响 STM32 自跑）
  2. apf_recorder 订阅 /apf_debug 写 CSV（含 cte/航向误差）

用法：
    ros2 launch k7_apf_debug record.launch.py                       # 默认参考=line（与当前固件直线一致）
    ros2 launch k7_apf_debug record.launch.py reference:=circle      # 固件改回参考圆时用 circle
    ros2 launch k7_apf_debug record.launch.py out_dir:=/home/kickpi/logs
记录后用 k7_apf_debug/plot_apf_log.py 绘图：
    ros2 run k7_apf_debug plot_apf <csv> --ref line --line-len 3    # 直线参考
    ros2 run k7_apf_debug plot_apf <csv> --ref circle --radius 0.6  # 圆参考
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    out_dir = LaunchConfiguration('out_dir')
    reference = LaunchConfiguration('reference')

    serial = Node(
        package='k7_bringup',
        executable='k7_serial_node',
        output='screen',
        parameters=[{
            'usart_port_name': '/dev/k7_controller',
            'serial_baud_rate': 115200,
            'odom_frame_id': 'odom_combined',
            'robot_frame_id': 'base_footprint',
            'gyro_frame_id': 'gyro_link',
            'cmd_vel_timeout_ms': 500,
            'enable_downlink': False,          # 只收不发：避免看门狗零速帧干扰 STM32 自跑
        }],
    )

    recorder = Node(
        package='k7_apf_debug',
        executable='apf_recorder',
        name='apf_recorder',
        output='screen',
        parameters=[{'out_dir': out_dir, 'reference': reference}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='命名空间：单机留空；多机编队里给从机传 follower1 等'),
        DeclareLaunchArgument(
            'out_dir', default_value=os.path.join(os.path.expanduser('~'), 'apf_log'),
            description='CSV 输出目录'),
        DeclareLaunchArgument(
            'reference', default_value='line',
            description="参考路径：line(直线 y=0 沿 +x,当前固件) / circle(参考圆)，须与固件 stanley.h 路径一致"),
        GroupAction(actions=[PushRosNamespace(ns), serial, recorder]),
    ])
