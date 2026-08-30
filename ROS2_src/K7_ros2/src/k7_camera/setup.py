import glob

from setuptools import setup

package_name = 'k7_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 标定 yaml 装到 share/k7_camera/config/，splitter 节点从这里读 camera_info
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k7',
    maintainer_email='user@todo.todo',
    description='LRCP 2MV 双目相机：splitter + 标定 + launch',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_splitter = k7_camera.stereo_splitter_node:main',
        ],
    },
)
