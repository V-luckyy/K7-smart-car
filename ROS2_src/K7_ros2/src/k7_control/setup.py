from setuptools import setup

setup(name='k7_control', version='0.2.0', packages=['k7_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/k7_control']),
                  ('share/k7_control', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='k7',
      maintainer_email='k7@example.com', description='Single algorithm and command authority',
      license='MIT', entry_points={'console_scripts': [
          'controller_node = k7_control.controller_node:main',
          'command_guard = k7_control.command_guard:main']})
