from setuptools import setup

setup(name='k7_experiment', version='0.2.0', packages=['k7_experiment'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/k7_experiment']),
                  ('share/k7_experiment', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='k7',
      maintainer_email='k7@example.com', description='Opt-in bounded experiment logging',
      license='MIT', entry_points={'console_scripts': ['experiment_recorder = k7_experiment.recorder:main']})
