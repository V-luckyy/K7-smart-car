from setuptools import setup

package_name = 'k7_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # TODO(Phase 3): 加入 launch/ 与 config/（标定 yaml）的 data_files
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
            # TODO(Phase 3): 'stereo_splitter = k7_camera.stereo_splitter_node:main',
        ],
    },
)
