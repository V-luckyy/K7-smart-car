"""MPC 实车验证 launch（番外线）。

前置：k7_core.launch.py（串口 + EKF）已在运行；红外固件扩展帧未就绪时，
可配合 `ros2 run k7_mpc fake_ir_publisher` 联调控制链路。

版本选择：默认 V5（全自适应 + CBF 安全滤波），回退 V1 用
`ros2 launch k7_mpc k7_mpc.launch.py version:=V1`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    version = LaunchConfiguration("version")
    return LaunchDescription([
        DeclareLaunchArgument(
            "version",
            default_value="V5",
            description="MPC 版本：V1（基线）/ V5（全自适应 + CBF）",
        ),
        Node(
            package="k7_mpc",
            executable="mpc_node",
            name="k7_mpc_node",
            output="screen",
            parameters=[{"version": version}],
        ),
    ])
