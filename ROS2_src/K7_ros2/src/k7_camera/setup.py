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
        # The runtime stereo node loads per-camera calibration here.
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k7',
    maintainer_email='user@todo.todo',
    description='Calibrated local stereo obstacle sectors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_obstacles = k7_camera.stereo_obstacles:main',
        ],
    },
)
