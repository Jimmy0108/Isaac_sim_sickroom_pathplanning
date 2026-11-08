"""Focused offline app-state tests; not Isaac Sim physical validation."""
import asyncio
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from v105_app import Application


def run_inline(coroutine):
    """Exercise deterministic app transitions without a Windows socket event loop."""
    try:
        coroutine.send(None)
    except StopIteration as done:
        return done.value
    raise AssertionError('Unexpected suspended coroutine in a deterministic state test')


async def inline_thread(function, *args, **kwargs):
    return function(*args, **kwargs)


async def inline_shield(job):
    return await job


class ImmediateJob:
    def __init__(self, coroutine):
        self.value = run_inline(coroutine)
    def __await__(self):
        if False:
            yield
        return self.value


class AppStateTests(unittest.TestCase):
    def setUp(self):
        self.test_root = Path(__file__).resolve().parent/'test_artifacts'
        self.test_root.mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=self.test_root)
        self.project = Path(self.temp.name)
        config = json.loads(Path('v105_config.json').read_text())
        (self.project/'v105_config.json').write_text(json.dumps(config))
        self.app = Application(self.project)
        self.app.simulation_time = lambda: 10.
        self.writes = {}
        controller = SimpleNamespace(set=lambda attr, value: self.writes.__setitem__(attr, value))
        self.app.og = SimpleNamespace(Controller=controller)
        self.app.linear, self.app.angular = 'v', 'w'
        self.app.ready = True
        self.app.mode = 'MANUAL'
        self.app.timeline = SimpleNamespace(is_playing=lambda: True)
        self.app.odom = SimpleNamespace(pose=np.zeros(3), v=0., w=0.)
        self.app.odom_history.append((10., np.zeros(3)))
        self.app.scan = (10., np.array([[4., 4.]]*25), np.zeros(3))
        self.app.last_scan = 10.
        self.app.max_wheel_speed = 15.

    def tearDown(self):
        assert Path(self.temp.name).resolve().is_relative_to(self.test_root.resolve())
        self.temp.cleanup()

    def test_manual_moves_even_when_slam_is_missing_or_rejected(self):
        for result in (None, {'ok': False, 'failures': 100}):
            self.app.result = result
            self.app.mode = 'LOCALIZE'
            self.app.set_manual(1, 0)
            self.app.physics_pre()
            self.assertGreater(self.app.last_control[0], 0.)
            self.assertEqual(self.writes['w'], 0.)
            self.assertLess(self.writes['v'], 0.)  # physical front is base -X

    def test_fresh_scan_obstacle_or_timeout_stops_without_slow_speed_search(self):
        self.app.scan = (10., np.array([[.47, 0.]]*25), np.zeros(3))
        self.app.set_manual(1, 0)
        self.app.physics_pre()
        self.assertEqual(self.app.last_control, (0., 0.))
        self.assertIn('blocks', self.app.control_status)
        self.app.scan = (8., np.array([[4., 4.]]*25), np.zeros(3))
        self.app.set_manual(1, 0)
        self.app.physics_pre()
        self.assertEqual(self.app.last_control, (0., 0.))
        self.assertIn('fresh', self.app.control_status)

    def test_stop_cancels_queued_navigation(self):
        self.app.request('navigate')
        self.assertEqual(self.app.action, 'navigate')
        old_epoch = self.app.control_epoch
        self.app.stop()
        self.assertIsNone(self.app.action)
        self.assertGreater(self.app.control_epoch, old_epoch)
        self.assertEqual(self.writes, {'v': 0., 'w': 0.})

    def test_busy_map_operation_cannot_accept_manual_motion(self):
        self.app.mode = 'BUSY'
        self.app.set_manual(1, 0)
        self.assertEqual(self.app.manual_target, (0., 0.))

    def test_finish_saves_candidate_without_replacing_existing_map(self):
        target = self.project/'slam_maps'
        target.mkdir()
        (target/'room_map_latest.npz').write_bytes(b'KEEP OLD MAP')
        def save(folder, metadata):
            folder.mkdir()
            (folder/'slam_graph.npz').write_bytes(b'CANDIDATE')
        self.app.mapping = True
        self.app.localized = lambda now: True
        self.app.metadata = lambda: {}
        self.app.slam = SimpleNamespace(nodes=[np.zeros(3)], save=save, loaded=False, relocalized=False)
        with patch('v105_app.asyncio.to_thread', inline_thread), \
                patch('v105_app.asyncio.create_task', ImmediateJob), \
                patch('v105_app.asyncio.shield', inline_shield):
            run_inline(self.app.perform_action('finish_map', 10.))
        self.assertEqual((target/'room_map_latest.npz').read_bytes(), b'KEEP OLD MAP')
        self.assertTrue((self.app.candidate/'slam_graph.npz').is_file())
        self.assertTrue(self.app.slam.loaded and self.app.slam.relocalized)
        self.assertTrue(self.app.map_ready)

    def test_replan_keeps_original_B_even_if_marker_moved(self):
        self.app.map_ready = True
        self.app.localized = lambda now: True
        self.app.goal = np.array([1., 2., .8])
        self.app.goal_info = {'map_xy_m': [1., 2.]}
        self.app.display = SimpleNamespace(read_goal_pose=lambda: (_ for _ in ()).throw(AssertionError('Do not reread B')))
        self.app.slam = SimpleNamespace(snapshot=lambda: (np.zeros(2), .05, np.zeros((10, 10), bool)))
        async def run():
            with patch('v105_app.plan_owned', return_value=(None, [], {})), \
                    patch('v105_app.asyncio.to_thread', inline_thread), \
                    patch('v105_app.asyncio.create_task', ImmediateJob):
                await self.app.perform_action('replan', 10.)
                await self.app.plan_job
            np.testing.assert_allclose(self.app.plan_goal, [1., 2., .8])
        run_inline(run())

    def test_arrival_requires_heading_not_just_xy(self):
        self.app.mode = 'VERIFY'
        self.app.goal = np.array([0., 0., np.pi/2])
        self.app.verify_start = 9.
        self.app.verify_quiet = 9.5
        self.app.scanner = SimpleNamespace(read=lambda: None)
        self.app.last_draw = 10.
        self.app.localized = lambda now: True
        run_inline(self.app.update(10., 1/60))
        self.assertEqual(self.app.navigation_result, 'FINAL_POSE_OUTSIDE_TOLERANCE')
        self.assertEqual(self.app.last_control, (0., 0.))


if __name__ == '__main__':
    unittest.main()
