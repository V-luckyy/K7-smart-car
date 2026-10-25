"""Exercise the recorder without ROS, using real temporary log files."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import tempfile
import time
import unittest
import uuid


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'ROS2_src/K7_ros2/src'


class NodeStub:
    config = {}

    def __init__(self, name):
        pass

    def declare_parameter(self, key, default, descriptor):
        return SimpleNamespace(value=self.config.get(key, default))

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass

    def get_namespace(self):
        return '/car1'


tree = ast.parse((SRC / 'k7_experiment/k7_experiment/recorder.py').read_text(encoding='utf-8'))
recorder_class = next(n for n in tree.body if isinstance(n, ast.ClassDef))
scope = dict(Node=NodeStub, ParameterDescriptor=SimpleNamespace, SetBool=object,
             Odometry=object, TwistStamped=object, DiagnosticArray=object,
             Path=Path, hashlib=hashlib, csv=csv, json=json, shutil=shutil,
             time=time, uuid=uuid)
exec(compile(ast.Module(body=[recorder_class], type_ignores=[]), '<recorder>', 'exec'), scope)


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / 'robot.yaml'
        self.original = b'common:\n  wheel_limit: 0.10\n'
        self.config.write_bytes(self.original)
        NodeStub.config = dict(config_file=str(self.config),
            config_sha256=hashlib.sha256(self.original).hexdigest(),
            output_dir=str(self.root / 'logs'), variables=[], topics=[])

    def test_snapshot_is_startup_config_not_later_file(self):
        node = scope['Recorder']()
        self.addCleanup(node.stop)
        self.config.write_bytes(b'common:\n  wheel_limit: 0.35\n')
        node.start()
        self.assertEqual((node.directory / 'config.yaml').read_bytes(), self.original)
        node.stop()
        manifest = json.loads((node.directory / 'manifest.json').read_text())
        self.assertEqual(manifest['config_sha256'], NodeStub.config['config_sha256'])
        self.assertEqual(manifest['status'], 'closed')
        self.assertFalse((node.directory / 'bag').exists())

    def test_launch_snapshot_mismatch_is_rejected(self):
        self.config.write_bytes(b'changed: true\n')
        with self.assertRaisesRegex(ValueError, 'configuration changed'):
            scope['Recorder']()

    def test_cached_serial_parameters_are_read_only(self):
        source = (SRC / 'k7_bringup/src/serial_bridge.cpp').read_text()
        declarations = [line for line in source.splitlines() if 'declare_parameter<' in line]
        self.assertEqual(len(declarations), 7)
        self.assertIn('startup_only.read_only = true;', source)
        self.assertTrue(all('startup_only)' in line for line in declarations))


if __name__ == '__main__':
    unittest.main()
