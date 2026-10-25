"""Check that a clean checkout has one buildable source and documentation tree."""
import ast
from pathlib import Path
import re
import unittest
from urllib.parse import unquote
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'ROS2_src/K7_ros2/src'


class WorkspaceLayoutTests(unittest.TestCase):
    def test_only_current_packages_and_launch_are_present(self):
        names = {ET.parse(p).findtext('name') for p in SRC.rglob('package.xml')}
        self.assertEqual(names, {'k7_msgs', 'k7_bringup', 'k7_camera', 'k7_control',
                                 'k7_tasks', 'k7_experiment', 'k7_description'})
        self.assertEqual(list(SRC.rglob('COLCON_IGNORE')), [])
        self.assertEqual([p.relative_to(SRC).as_posix() for p in SRC.rglob('*.launch.py')],
                         ['k7_bringup/launch/robot.launch.py'])

    def test_exported_interfaces_and_install_inputs_exist(self):
        cmake = (SRC / 'k7_msgs/CMakeLists.txt').read_text(encoding='utf-8')
        declared = set(re.findall(r'"((?:msg|srv|action)/[^\"]+)"', cmake))
        present = {p.relative_to(SRC / 'k7_msgs').as_posix()
                   for suffix in ('msg', 'srv', 'action')
                   for p in (SRC / 'k7_msgs' / suffix).glob('*.' + suffix)}
        self.assertEqual(declared, present)
        for file in SRC.rglob('CMakeLists.txt'):
            text = file.read_text(encoding='utf-8')
            for group in re.findall(r'install\(DIRECTORY\s+(.*?)\s+DESTINATION', text, re.S):
                for directory in group.split():
                    self.assertTrue((file.parent / directory).is_dir(), (file, directory))
            for source in re.findall(r'\b[\w/]+\.c(?:c|pp)\b', text):
                self.assertTrue((file.parent / source).is_file(), (file, source))

    def test_python_entry_points_exist(self):
        for setup in SRC.glob('*/setup.py'):
            calls = [n for n in ast.walk(ast.parse(setup.read_text(encoding='utf-8')))
                     if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                     and n.func.id == 'setup']
            entries = next(k.value for k in calls[0].keywords if k.arg == 'entry_points')
            for entry in ast.literal_eval(entries)['console_scripts']:
                module = entry.split('=', 1)[1].strip().split(':', 1)[0]
                self.assertTrue((setup.parent / (module.replace('.', '/') + '.py')).is_file(), entry)

    def test_optional_no_perception_mode_is_wired_and_bounded(self):
        launch = (SRC / 'k7_bringup/launch/robot.launch.py').read_text(encoding='utf-8')
        config = (SRC / 'k7_bringup/config/robot.yaml').read_text(encoding='utf-8')
        self.assertIn("test_mode='none'", launch)
        self.assertIn("test_mode == 'no_perception'", launch)
        self.assertIn('if not no_perception_test:', launch)
        self.assertIn('no_perception_test=no_perception_test', launch)
        self.assertIn('test_speed_limit: 0.05', config)

    def test_recording_uses_one_launch_switch(self):
        launch = (SRC / 'k7_bringup/launch/robot.launch.py').read_text(encoding='utf-8')
        self.assertIn("'experiment_recorder', dict(enabled=arg('record').lower() == 'true'", launch)
        self.assertNotIn("arg('logging')", launch)
        self.assertNotIn("logging=", launch)

    def test_keil_references_exist_and_retired_modules_are_absent(self):
        project = ROOT / 'STM32F407VET6_src/USER/WHEELTEC.uvprojx'
        tree = ET.parse(project)
        retired = {'apf.c', 'apf_task.c', 'stanley.c', 'gy53_pwm.c'}
        for target in tree.findall('./Targets/Target'):
            files = target.findall('.//File')
            self.assertTrue(files)
            for item in files:
                self.assertNotIn(item.findtext('FileName'), retired)
                source = project.parent / item.findtext('FilePath').replace('\\', '/')
                self.assertTrue(source.is_file(), str(source))
        firmware = ROOT / 'STM32F407VET6_src'
        for file in firmware.rglob('*'):
            if file.suffix in ('.c', '.h') and 'OBJ' not in file.parts:
                includes = re.findall(rb'#include\s+"([^\"]+)"', file.read_bytes())
                self.assertTrue(set(includes).isdisjoint(
                    {b'apf.h', b'apf_task.h', b'stanley.h', b'gy53_pwm.h'}), str(file))

    def test_current_document_links_resolve(self):
        documents = [ROOT / name for name in ('README.md', 'AGENTS.md', 'CLAUDE.md',
                     'ROS2_src/K7_ros2/README.md', 'logs/README.md')]
        documents.extend((ROOT / 'docs').glob('*.md'))
        for file in documents:
            for link in re.findall(r'\]\(([^)]+)\)', file.read_text(encoding='utf-8')):
                target = unquote(link.split('#', 1)[0])
                if target and not re.match(r'\w+://', target):
                    self.assertTrue((file.parent / target).exists(), (file, target))


if __name__ == '__main__':
    unittest.main()
