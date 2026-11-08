"""Focused offline checks of direct command semantics, not physics validation."""
import math
import unittest
import numpy as np
from v105_control import ManualDrive, wheel_limit, local_swept_clear


class DirectDriveTests(unittest.TestCase):
    def test_four_directions_remain_pure(self):
        for target, axis, expected in [((.9,0),0,.9), ((-.9,0),0,-.9),
                                      ((0,.9),1,.9), ((0,-.9),1,-.9)]:
            drive = ManualDrive()
            for i in range(240):
                v,w,_ = drive.step(target,1/60,0,0,i/60)
                self.assertEqual((v,w)[1-axis],0.)
            self.assertAlmostEqual((v,w)[axis],expected)

    def test_spin_to_forward_requires_measured_settle(self):
        drive = ManualDrive()
        drive.step((0,.9),.1,0,0,0)
        self.assertEqual(drive.step((.9,0),.1,0,.2,.1)[:2],(0.,0.))
        self.assertEqual(drive.step((.9,0),.1,0,0,.2)[:2],(0.,0.))
        self.assertEqual(drive.step((.9,0),.1,0,0,.4)[:2],(0.,0.))
        v,w,_ = drive.step((.9,0),.1,0,0,.5)
        self.assertGreater(v,0.); self.assertEqual(w,0.)

    def test_reverse_and_spin_boundaries_are_stopped(self):
        for first,second in [((.9,0),(-.9,0)), ((.9,0),(0,.9)), ((0,.9),(0,-.9))]:
            drive=ManualDrive();drive.step(first,.1,0,0,0)
            self.assertEqual(drive.step(second,.1,.1,.1,.1)[:2],(0.,0.))

    def test_stop_is_immediate_and_does_not_carry_turn(self):
        drive=ManualDrive();drive.step((0,.9),.1,0,0,0)
        self.assertEqual(drive.step((0,0),.1,0,.2,.1)[:2],(0.,0.))
        self.assertEqual(drive.step((.9,0),.1,0,.2,.2)[:2],(0.,0.))

    def test_wheel_limit_preserves_curvature(self):
        v,w,scale=wheel_limit(.9,.9,.09,.514,10.)
        self.assertLess(scale,1.)
        self.assertAlmostEqual(w/v,1.)
        self.assertLessEqual(max(abs((v-w*.514/2)/.09),abs((v+w*.514/2)/.09)),10.+1e-12)
        self.assertEqual(wheel_limit(.9,0,.09,.514,0),(.9,0.,1.))

    def test_swept_collision_catches_future_and_rotation(self):
        self.assertTrue(local_swept_clear(np.array([[2.,0.]]),.9,0,.88,.56,.04)[0])
        self.assertFalse(local_swept_clear(np.array([[.8,0.]]),.9,0,.88,.56,.04)[0])
        self.assertTrue(local_swept_clear(np.array([[0.,.48]]),0,0,.88,.56,.04)[0])
        self.assertFalse(local_swept_clear(np.array([[0.,.48]]),0,.9,.88,.56,.04,horizon=1.)[0])

    def test_invalid_inputs_and_time_reset(self):
        self.assertFalse(local_swept_clear([[float('nan'),0]],0,0,.88,.56,.04)[0])
        drive=ManualDrive();drive.step((.9,0),.1,0,0,1.)
        with self.assertRaises(ValueError):drive.step((.9,0),.1,0,0,.5)
        self.assertEqual((drive.v,drive.w),(0.,0.))


if __name__=='__main__': unittest.main()
