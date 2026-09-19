"""双目深度/视差/点云 launch。

前置：usb_cam 已启动并发布 3840×1080 左右并排帧（话题 /camera/image_raw）。
本 launch 启动 stereo_splitter（拆分左右图）+ stereo_depth（立体匹配出深度/视差/点云）。

用法：
    ros2 launch k7_camera stereo_depth.launch.py
    ros2 launch k7_camera stereo_depth.launch.py scale:=0.5 num_disparities:=128

发布话题：
    /camera/left|right/image_raw      （splitter）
    /camera/depth/image_raw           （深度图 32FC1 米）
    /camera/disparity/image           （视差图 32FC1 像素）
    /camera/points2                   （点云 PointCloud2 xyz+rgb）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scale = LaunchConfiguration('scale')
    num_disparities = LaunchConfiguration('num_disparities')
    depth_max = LaunchConfiguration('depth_max')

    splitter = Node(
        package='k7_camera',
        executable='stereo_splitter',
        name='stereo_splitter',
        output='screen',
    )

    depth = Node(
        package='k7_camera',
        executable='stereo_depth',
        name='stereo_depth',
        output='screen',
        parameters=[{
            'scale': scale,
            'num_disparities': num_disparities,
            'depth_max': depth_max,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('scale', default_value='0.5',
                              description='降采样比例（1.0=全尺寸1920x1080，0.5=960x540）'),
        DeclareLaunchArgument('num_disparities', default_value='128',
                              description='SGBM 视差搜索范围（16 的倍数）'),
        DeclareLaunchArgument('depth_max', default_value='5.0',
                              description='深度/点云有效上限 (m)'),
        splitter,
        depth,
    ])
