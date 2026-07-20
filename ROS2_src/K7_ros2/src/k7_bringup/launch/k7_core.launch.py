# K7 智能小车核心启动文件（ROS2 Jazzy）
# 移植自 wheeltec_ros2 turn_on_wheeltec_robot 的 launch 结构（Humble）：
#   base_serial.launch.py        -> k7_serial_node（设备名改为 /dev/k7_controller，去掉 v650 专用参数）
#   turn_on_wheeltec_robot.launch.py -> 静态 TF base_footprint->base_link / base_footprint->gyro_link、imu_filter_madgwick
#   wheeltec_ekf.launch.py       -> robot_localization ekf_node（carto_slam=false 分支）
#   robot_mode_description_*.launch.py -> robot_state_publisher（改用 k7_description 的 urdf/k7_robot.urdf）
# 差异说明：
# - static_transform_publisher 使用 Jazzy 新式参数（--x/--frame-id 等），wheeltec 旧式位置参数在 Jazzy 已废弃
# - base_footprint->base_link 平移 z=0.0625（K7 轮半径，见 URDF 注释）
# - imu_filter_madgwick 与 wheeltec 一样不做 remap：默认订阅 imu/data_raw、发布 imu/data，与 k7_serial_node 输出对得上
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    bringup_dir = get_package_share_directory('k7_bringup')
    ekf_config = Path(bringup_dir, 'config', 'ekf.yaml')
    imu_config = Path(bringup_dir, 'config', 'imu.yaml')

    urdf_path = os.path.join(
        get_package_share_directory('k7_description'), 'urdf', 'k7_robot.urdf')
    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # STM32(C50X) 串口通信节点：/cmd_vel -> 串口下发，串口上行 -> /odom、/imu/data_raw、/PowerVoltage
    k7_serial_node = launch_ros.actions.Node(
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
        }],
    )

    # URDF 模型发布（base_link 树：轮子/万向轮/相机 frame）
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # base_footprint -> base_link：平移 z=0.0625（轮半径），无旋转
    base_to_link = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_link',
        arguments=['--x', '0', '--y', '0', '--z', '0.0625',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
    )

    # base_footprint -> gyro_link：单位变换（IMU 位于旋转中心）
    base_to_gyro = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gyro',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'gyro_link'],
    )

    # EKF 融合 /odom + /imu/data_raw，输出 /odometry/filtered remap 到 /odom_combined（与 wheeltec 一致）
    robot_ekf = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', 'odom_combined')],
    )

    # Madgwick 滤波：订阅 imu/data_raw，发布 imu/data（无 remap，与 wheeltec 一致）
    imu_filter_node = launch_ros.actions.Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_config],
    )

    ld = LaunchDescription()
    ld.add_action(k7_serial_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(base_to_link)
    ld.add_action(base_to_gyro)
    ld.add_action(robot_ekf)
    ld.add_action(imu_filter_node)

    return ld
