from glob import glob

from setuptools import setup

package_name = 'k7_apf_debug'

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
    description='APF+Stanley 实车调参：/apf_debug 记录 CSV + 绘图',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apf_recorder = k7_apf_debug.apf_recorder:main',
            'plot_apf = k7_apf_debug.plot_apf_log:main',
        ],
    },
)
