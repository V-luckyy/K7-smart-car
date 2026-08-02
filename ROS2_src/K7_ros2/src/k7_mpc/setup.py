from glob import glob

from setuptools import setup

package_name = 'k7_mpc'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.mpc_lib'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k7',
    maintainer_email='user@todo.todo',
    description='MPC 避障实车验证包（仿真 five_version_progressive_r04_with_pid 移植，番外线）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = k7_mpc.mpc_node:main',
            'fake_ir_publisher = k7_mpc.fake_ir_publisher:main',
        ],
    },
)
