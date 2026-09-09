from glob import glob

from setuptools import setup

package_name = 'k7_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='k7',
    maintainer_email='user@todo.todo',
    description='多机编队跟随包（Leader-Follower v1）',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follower_node = k7_follower.convoy_follower:main',
        ],
    },
)
