"""Host-side regressions; no ROS installation or hardware required."""
import ast
import math
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
import sys
import time
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'ROS2_src/K7_ros2/src'
for package in ('k7_control', 'k7_tasks', 'k7_camera'):
    sys.path.insert(0, str(SRC / package))
from k7_control.core import (Authority, apf_stanley, control_ready, limit_drive, project,
                             stanley_only, valid_scan)
from k7_tasks.formation import LeaderTrail
from k7_camera.stereo_geometry import depth_from_disparity, obstacle_sectors


class ControlTests(unittest.TestCase):
    def test_wheel_limits_preserve_curvature(self):
        v, w = limit_drive(0.8, 1.0, 0.329, 0.35)
        self.assertAlmostEqual(w / v, 1 / 0.8)
        self.assertLessEqual(max(abs(v-w*.329/2), abs(v+w*.329/2)), .35)

    def test_forward_limit_no_reverse_or_pivot(self):
        for v, w in ((.1, 9), (.1, -9), (0, 1), (-1, 1)):
            v, w = limit_drive(v, w, .329, .35, True)
            self.assertGreaterEqual(v-abs(w)*.329/2, -1e-12)
        self.assertEqual(limit_drive(0, 1, .329, .35, True), (0, 0))

    def test_nonfinite_command_stops(self):
        self.assertEqual(limit_drive(math.nan, 1, .329, .35), (0, 0))

    def test_projection_sign_and_endpoint(self):
        self.assertAlmostEqual(project([(0, 0), (2, 0)], 1, .2)[1], -.2)
        self.assertAlmostEqual(project([(0, 0), (2, 0)], 3, 0)[3], 2)
        with self.assertRaises(ValueError):
            project([(0, 0), (0, 0)], 0, 0)

    def test_unknown_scan_is_not_free_space(self):
        self.assertTrue(valid_scan([.3, math.inf], .05, 4))
        for value in (math.nan, -math.inf, 0, 5):
            self.assertFalse(valid_scan([value], .05, 4))
        self.assertFalse(valid_scan([], .05, 4))

    def test_no_perception_test_still_requires_odom_and_hardware(self):
        self.assertTrue(control_ready(True, True, True, False, False, True))
        self.assertFalse(control_ready(False, True, True, False, False, True))
        self.assertFalse(control_ready(True, False, True, False, False, True))
        self.assertFalse(control_ready(True, True, False, False, False, True))
        self.assertFalse(control_ready(True, True, True, False, False, False))

    def test_stanley_corrects_heading_before_crossing(self):
        config = yaml.safe_load((SRC/'k7_bringup/config/robot.yaml').read_text())
        p = {**config['common'], **config['controller_node']}
        for pose in ((0, .2, 0), (0, 0, .2)):
            v, w, _ = apf_stanley([(0, 0), (2, 0)], pose, .2, .2, [0], [math.inf], p)
            self.assertGreater(v, 0)
            self.assertLess(w, 0)
        v, w, _ = apf_stanley([(0, 0), (2, 0)], (0, 0, 0), .2, .2, [0], [.6], p)
        self.assertGreater(v, 0)
        self.assertGreater(abs(w), 0)
        self.assertGreaterEqual(v-abs(w)*p['track']/2, -1e-12)
        self.assertEqual(apf_stanley([(0, 0), (2, 0)], (0, 0, 0), .2, .2, [0], [.2], p)[:2], (0, 0))

    def test_stanley_only_tracks_without_scan(self):
        config = yaml.safe_load((SRC/'k7_bringup/config/robot.yaml').read_text())
        p = {**config['common'], **config['controller_node']}
        v, w, debug = stanley_only([(0, 0), (2, 0)], (0, .2, 0), .05, 0, p)
        self.assertAlmostEqual(v, .05)
        self.assertLess(w, 0)
        self.assertEqual(debug['w_apf'], 0.0)


class AuthorityTests(unittest.TestCase):
    def setUp(self):
        self.a = Authority(.25)
        self.assertTrue(self.a.select(2, True, True))
        self.token = self.a.token

    def test_stale_candidate_revokes_and_cannot_restart(self):
        self.a.receive(self.token, .2, .1, 1)
        self.assertEqual(self.a.output(1.1, True, True), (.2, .1))
        self.assertEqual(self.a.output(1.3, True, True), (0, 0))
        self.a.receive(self.token, .2, 0, 1.4)
        self.assertEqual(self.a.output(1.4, True, True), (0, 0))

    def test_wrong_permission_and_sensor_loss(self):
        self.a.receive(bytes(16), .2, 0, 1)
        self.assertEqual(self.a.output(1, True, True), (0, 0))
        self.a.receive(self.token, .2, 0, 1)
        self.assertEqual(self.a.output(1, False, True), (0, 0))
        self.assertEqual(self.a.source, 0)

    def test_hold_release_does_not_resume(self):
        self.a.hold(True)
        self.assertFalse(self.a.select(2, True, True))
        self.a.hold(False)
        self.assertEqual(self.a.output(1, True, True), (0, 0))
        self.assertTrue(self.a.select(2, True, True))
        self.assertNotEqual(self.token, self.a.token)

    def test_not_ready_or_moving_cannot_arm(self):
        self.a.revoke()
        self.assertFalse(self.a.select(2, False, True))
        self.assertFalse(self.a.select(2, True, False))


class FormationTests(unittest.TestCase):
    def test_initial_gap_and_bounded_history(self):
        trail = LeaderTrail(capacity=8)
        trail.seed((0, 0, 0), (1, 0), 0, b'a', .15, .35)
        self.assertEqual(trail.reference((0, 0), 1, .2, .35, .5)[1], 0)
        trail.append((1.2, 0), b'a')
        points, speed, error = trail.reference((0, 0), 1, .2, .35, .5)
        self.assertAlmostEqual(points[-1][0], .2)
        self.assertAlmostEqual(error, .2)
        self.assertGreater(speed, 0)
        for i in range(10):
            trail.append((1.3+i*.1, 0), b'a')
        self.assertEqual(len(trail.points), 8)

    def test_wrong_alignment_restart_jump_rejected(self):
        trail = LeaderTrail()
        for leader in ((-1, 0), (1, 1)):
            with self.assertRaises(ValueError):
                trail.seed((0, 0, 0), leader, 0, b'a', .15, .35)
        trail.seed((0, 0, 0), (1, 0), 0, b'a', .15, .35)
        for point, session in (((1.1, 0), b'b'), ((2, 0), b'a'), ((math.nan, 0), b'a')):
            with self.assertRaises(ValueError):
                trail.append(point, session)


class StereoTests(unittest.TestCase):
    def test_both_disparity_signs_and_invalid_pixels(self):
        positive = depth_from_disparity(np.array([5., -1., 0.]), 0, 10., 0.)
        self.assertAlmostEqual(positive[0], 2)
        self.assertTrue(np.isnan(positive[1:]).all())
        negative = depth_from_disparity(np.array([-5., -65., 1.]), -64, -10., 0.)
        self.assertAlmostEqual(negative[0], 2)
        self.assertTrue(np.isnan(negative[1:]).all())

    def test_unknown_and_ground_ranges_are_conservative(self):
        config = yaml.safe_load((SRC/'k7_bringup/config/robot.yaml').read_text())
        p = {**config['stereo_obstacles'], 'sectors': 3, 'half_fov': .1,
             'roi_top': 0., 'roi_bottom': 1., 'min_samples': 1}
        intrinsics = (100, 100, 15, 15)
        self.assertTrue(all(math.isnan(r) for r in obstacle_sectors(np.full((30, 30), np.nan), p, intrinsics)))
        result = obstacle_sectors(np.full((30, 30), 2.), p, intrinsics)
        self.assertTrue(all(1.9 <= r <= 2.1 for r in result))


# Exercise real lifecycle methods without mocking the entire rclpy runtime.
tree = ast.parse((SRC/'k7_tasks/k7_tasks/task_manager.py').read_text())
klass = next(n for n in tree.body if isinstance(n, ast.ClassDef))
klass.bases = []
scope = {'time': time, 'math': math, 'finite': lambda *v: all(math.isfinite(x) for x in v),
         'yaw': lambda q: q.heading, 'stamp_fresh': lambda *args: True,
         'SelectSource': SimpleNamespace(Request=lambda **kw: kw)}
exec(compile(ast.Module(body=[klass], type_ignores=[]), '<task methods>', 'exec'), scope)
guard_tree = ast.parse((SRC/'k7_control/k7_control/command_guard.py').read_text())
guard_class = next(n for n in guard_tree.body if isinstance(n, ast.ClassDef))
guard_class.bases = []
exec(compile(ast.Module(body=[guard_class], type_ignores=[]), '<guard methods>', 'exec'), scope)


class StandstillTests(unittest.TestCase):
    def odometry(self, frame='odom'):
        return SimpleNamespace(header=SimpleNamespace(frame_id=frame, stamp=None),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                                                     orientation=SimpleNamespace(heading=0.))),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=SimpleNamespace(x=0.),
                                                       angular=SimpleNamespace(z=0.))))

    def test_gap_restarts_standstill_in_guard_and_task(self):
        for name in ('TaskManager', 'CommandGuard'):
            node = scope[name].__new__(scope[name])
            node.p = dict(odom_frame='odom', sensor_timeout=.4, standstill_speed=.02)
            old = time.monotonic()-10
            node.odom_time = node.still_since = old
            node.odom_cb(self.odometry())
            self.assertGreater(node.still_since, old+9)

    def test_invalid_odometry_breaks_standstill(self):
        for name in ('TaskManager', 'CommandGuard'):
            node = scope[name].__new__(scope[name])
            node.p = dict(odom_frame='odom', sensor_timeout=.4, standstill_speed=.02)
            node.still_since = time.monotonic()-1
            node.odom_cb(self.odometry('wrong_frame'))
            self.assertIsNone(node.still_since)


class TaskLifecycleTests(unittest.TestCase):
    def manager(self, mode):
        manager = scope['TaskManager'].__new__(scope['TaskManager'])
        manager.mode, manager.grant_pending = mode, True
        manager.manual_reply, manager.token = Future(), b'a'*16
        manager.publish_status = lambda: None
        manager.source = SimpleNamespace(call_async=lambda request: request)
        manager.leader_sub = None
        return manager

    def test_late_success_is_revoked(self):
        manager = self.manager('STOPPING')
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        manager.grant_done(future, 'STARTING')
        self.assertEqual(manager.revoke, {'source': 0})
        self.assertEqual(manager.mode, 'STOPPING')
        self.assertFalse(manager.grant_pending)

    def test_service_exception_finishes_manual_reply(self):
        manager = self.manager('STARTING_MANUAL')
        future = Future()
        future.set_exception(RuntimeError('lost service'))
        manager.grant_done(future, 'STARTING_MANUAL')
        self.assertEqual(manager.mode, 'STOPPING')
        self.assertFalse(manager.manual_reply.result()[0])
        self.assertEqual(manager.token, bytes(16))

    def test_manual_success_returns_permission(self):
        manager = self.manager('STARTING_MANUAL')
        future = Future()
        future.set_result(SimpleNamespace(success=True, permission_id=SimpleNamespace(uuid=[1]*16), reason='ok'))
        manager.grant_done(future, 'STARTING_MANUAL')
        self.assertEqual(manager.mode, 'MANUAL')
        self.assertEqual(manager.manual_reply.result(), (True, 'ok'))

    def test_stop_waits_for_pending_grant(self):
        manager = self.manager('STOPPING')
        now = time.monotonic()
        manager.p = dict(sensor_timeout=.4, standstill_time=.3, stop_timeout=3.)
        manager.revoke = Future()
        manager.revoke.set_result(SimpleNamespace(success=True))
        manager.odom_time, manager.still_since = now, now-1
        manager.stop_started = now
        manager.stop_code, manager.stop_reason = 1, 'canceled'
        finished = []
        manager.finish_stop = lambda *args: finished.append(args)
        manager.tick()
        self.assertEqual(finished, [])
        manager.grant_pending = False
        manager.tick()
        self.assertEqual(finished, [(1, 'canceled')])

    def test_pending_grant_blocks_next_task(self):
        manager = self.manager('IDLE')
        manager.goal = None
        self.assertFalse(manager.reserve('trajectory'))

    def test_late_success_after_stop_timeout_is_revoked(self):
        manager = self.manager('FAULT')
        future = Future()
        future.set_result(SimpleNamespace(success=True))
        manager.grant_done(future, 'STARTING_MANUAL')
        self.assertEqual(manager.mode, 'FAULT')
        self.assertEqual(manager.revoke, {'source': 0})


if __name__ == '__main__':
    unittest.main()
