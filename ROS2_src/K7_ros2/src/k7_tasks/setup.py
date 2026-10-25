from setuptools import setup

setup(name='k7_tasks', version='0.2.0', packages=['k7_tasks'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/k7_tasks']),
                  ('share/k7_tasks', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='k7',
      maintainer_email='k7@example.com', description='Exclusive motion task actions',
      license='MIT', entry_points={'console_scripts': [
          'task_manager = k7_tasks.task_manager:main',
          'ref_path = k7_tasks.ref_path:main']})
