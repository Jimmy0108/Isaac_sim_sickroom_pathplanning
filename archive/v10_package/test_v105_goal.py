"""Goal-frame and mathematical Hybrid/RPP checks; no Isaac Sim physics claim.

Run with: python -m unittest test_v105_goal -v
The marker test doubles only stand in for USD reads. The trajectory test uses
an ideal planar kinematic integrator, not contacts, wheel dynamics, or SLAM.
"""
import json
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import amr_planner_core as planner
import isaac_io
from rpp_core import RPP
from slam_core import compose, wrap


class _Prim:
    def __init__(self, valid=True):
        self.valid = valid

    def IsValid(self):
        return self.valid


class _Stage:
    def __init__(self, valid=True):
        self.valid = valid

    def GetPrimAtPath(self, path):
        return _Prim(self.valid)


class GoalFrameTests(unittest.TestCase):
    def setUp(self):
        self.matrix = np.eye(4)
        owner = self

        class _Cache:
            def GetLocalToWorldTransform(self, prim):
                return owner.matrix

        self.pxr = patch.dict(sys.modules, {
            'pxr': types.SimpleNamespace(UsdGeom=types.SimpleNamespace(XformCache=_Cache))
        })
        self.pxr.start()
        self.addCleanup(self.pxr.stop)
        self.display = isaac_io.Display(_Stage(), np.array([1., 2., .3]), -.5, .01,
                                        '/World/PythonNav/GoalB')

    def set_marker(self, x=3., y=4., z=9.99, yaw=.8, pitch=.4, scale=2.):
        # USD row-vector form. Only +X's world XY projection is the yaw source.
        self.matrix[0, :3] = [scale*math.cos(yaw)*math.cos(pitch),
                              scale*math.sin(yaw)*math.cos(pitch), scale*math.sin(pitch)]
        self.matrix[3, :3] = np.array([x, y, z])/self.display.units

    def test_goal_world_map_roundtrip_and_non_metre_stage(self):
        self.set_marker()
        pose = self.display.read_goal_pose()
        np.testing.assert_allclose(compose(self.display.anchor, pose), [3., 4., .8], atol=1e-12)
        details = self.display.goal_details()
        np.testing.assert_allclose(details['world_xy_m'], [3., 4.])
        self.assertAlmostEqual(details['world_yaw_deg'], math.degrees(.8))
        self.assertAlmostEqual(details['map_yaw_rad'], .5)

    def test_height_and_tilt_do_not_become_planar_goal_constraints(self):
        self.set_marker(z=20., pitch=.2)
        first = self.display.read_goal_pose()
        self.set_marker(z=-30., pitch=.7)
        np.testing.assert_allclose(self.display.read_goal_pose(), first, atol=1e-12)

    def test_arrow_is_ground_projected_and_points_toward_logical_front(self):
        self.set_marker()
        starts, ends = self.display.goal_arrow(self.display.read_goal_pose(), length=.45)
        self.assertEqual(len(starts), 3)
        self.assertEqual(len(ends), 3)
        for point in starts+ends:
            self.assertAlmostEqual(point[2], (self.display.z+.025)/self.display.units)
        direction = np.asarray(ends[0])[:2]-np.asarray(starts[0])[:2]
        self.assertAlmostEqual(np.linalg.norm(direction), .45/self.display.units)
        self.assertAlmostEqual(math.atan2(direction[1], direction[0]), .8)

    def test_unreadable_or_undefined_marker_is_rejected(self):
        self.display.stage.valid = False
        with self.assertRaisesRegex(RuntimeError, 'Create and drag'):
            self.display.read_goal_pose()
        self.display.stage.valid = True
        self.matrix[0, :3] = [0., 0., 1.]
        with self.assertRaisesRegex(RuntimeError, 'no planar heading'):
            self.display.read_goal_pose()
        self.matrix[0, 0] = float('nan')
        with self.assertRaisesRegex(RuntimeError, 'nonfinite'):
            self.display.read_goal_pose()


class OrientedGoalTrajectoryTests(unittest.TestCase):
    def test_hybrid_and_rpp_keep_required_terminal_heading(self):
        config = json.loads((HERE/'v105_config.json').read_text(encoding='utf-8-sig'))
        body = config['body']
        cfg = planner.Config(length=body['length'], width=body['width'],
                             track=body['track'], wheel_radius=body['wheel_radius'],
                             margin=config['margin'], nominal_v=.4, nominal_w=.9)
        grid = planner.Grid(np.zeros((80, 80), dtype=bool), .05, (0., 0.), cfg)
        start = np.array([2., 2., 0.])
        goal = np.array([2.4, 2., math.pi/2])
        samples, stats = planner.plan(grid, start, goal)
        self.assertGreater(stats['expansions'], 0)
        np.testing.assert_allclose(samples[-1][0][:2], goal[:2], atol=1e-8)
        self.assertLess(abs(wrap(samples[-1][0][2]-goal[2])), 1e-8)
        self.assertTrue(any(abs(v) < 1e-12 and abs(w) > 0 for _, v, w, _ in samples))
        for pose, _, _, _ in samples:
            self.assertFalse(grid.collision(pose))

        follower = RPP(config['rpp'])
        follower.set_path(samples)
        self.assertEqual(follower.groups[-1]['kind'][0], 'spin')
        pose = start.copy()
        measured_v = measured_w = 0.
        dt = .02
        obstacles = np.empty((0, 2))
        saw_terminal_spin = False
        for _ in range(2000):
            v, w, state = follower.command(pose, measured_v, grid, obstacles, dt,
                                           measured_w=measured_w)
            self.assertFalse(state.startswith('replan'), state)
            if abs(v) < 1e-10 and abs(w) > .01:
                saw_terminal_spin = True
            if state == 'done':
                break
            pose = planner.integrate(pose, v, w, dt)
            self.assertFalse(grid.collision(pose))
            measured_v, measured_w = v, w
        else:
            self.fail('Ideal mathematical trajectory did not finish within 40 s')
        self.assertTrue(saw_terminal_spin)
        self.assertLessEqual(np.linalg.norm(pose[:2]-goal[:2]), .07)
        self.assertLessEqual(abs(wrap(pose[2]-goal[2])), math.radians(5))


if __name__ == '__main__':
    unittest.main()
