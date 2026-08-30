"""MPC 实车验证 launch（番外线）。

前置：k7_core.launch.py（串口 + EKF）已在运行；红外固件扩展帧未就绪时，
可配合 `ros2 run k7_mpc fake_ir_publisher` 联调控制链路。
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="k7_mpc",
            executable="mpc_node",
            name="k7_mpc_node",
            output="screen",
        ),
    ])
