"""Small offline IO-boundary checks, not Isaac Sim physics validation."""
import json
import math
from types import SimpleNamespace
import unittest

import numpy as np
from v105_io import sample_diagnostics


class JointView:
    shared_metatype = SimpleNamespace(dof_names=['original_left', 'original_right'])

    def get_dof_positions(self):
        return np.array([[1., 2.]])

    def get_dof_velocities(self):
        # Intentionally not derived from positions or an odometry estimate.
        return np.array([[3., 7.]])


class BodyView:
    def __init__(self):
        self.x = 1.

    def get_transforms(self):
        return np.array([[self.x, 2., 3., 0., 0., 0., 1.]])

    def get_velocities(self):
        return np.array([[.2, .3, .4, .1, .2, .3]])


def application(graph_available=True):
    class Attribute:
        def __init__(self, path):
            self.path = path

        def is_valid(self):
            return graph_available

    class Controller:
        attribute = staticmethod(Attribute)

        @staticmethod
        def get(attr):
            return [10., 8.]

    return SimpleNamespace(
        paths={'controller': '/Graph/DifferentialController'},
        og=SimpleNamespace(Controller=Controller),
        cfg={'front_axis_sign': -1},
        encoders=SimpleNamespace(view=JointView(), left=1, right=0, lsign=-1, rsign=-1),
        scanner=SimpleNamespace(body_view=BodyView(), rotation_row=lambda q: np.eye(3), units=1.))


class DiagnosticBoundaryTests(unittest.TestCase):
    def test_direct_measured_velocity_and_explicit_logical_binding(self):
        result = sample_diagnostics(application(), 12.)
        self.assertEqual(result['logical_wheel_joint_names'], ['original_right', 'original_left'])
        self.assertEqual(result['actual_logical_wheel_positions_rad'], [-2., -1.])
        self.assertEqual(result['actual_logical_wheel_velocities_rad_s'], [-7., -3.])
        self.assertFalse(result['errors'])

    def test_graph_target_keeps_raw_controller_order(self):
        result = sample_diagnostics(application(), 12.)
        self.assertEqual(result['graph_wheel_targets_rad_s'], [10., 8.])
        self.assertIn('declared left,right', result['graph_output_order'])
        self.assertIn('not proof', result['graph_output_timing'])
        missing = sample_diagnostics(application(graph_available=False), 12.)
        self.assertIsNone(missing['graph_wheel_targets_rad_s'])
        self.assertIn('graph_output', missing['errors'])

    def test_body_is_read_again_with_new_timestamp_and_front_frame(self):
        app = application()
        first = sample_diagnostics(app, 12.)
        app.scanner.body_view.x = 4.
        second = sample_diagnostics(app, 13.)
        self.assertEqual(first['body']['physics_s'], 12.)
        self.assertEqual(second['body']['physics_s'], 13.)
        self.assertEqual(second['body']['world_position_m'], [4., 2., 3.])
        self.assertEqual(second['body']['linear_front_m_s'], [-.2, -.3, .4])
        self.assertAlmostEqual(abs(second['body']['physical_front_world_yaw_rad']), math.pi)
        json.dumps(second, allow_nan=False)

    def test_missing_body_does_not_reuse_previous_pose(self):
        app = application()
        self.assertIsNotNone(sample_diagnostics(app, 12.)['body'])
        app.scanner.body_view = None
        result = sample_diagnostics(app, 13.)
        self.assertIsNone(result['body'])
        self.assertIn('body', result['errors'])
        json.dumps(result, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
