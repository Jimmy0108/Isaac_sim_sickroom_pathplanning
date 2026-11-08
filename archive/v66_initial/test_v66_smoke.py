"""Focused offline migration checks; NOT evidence of PhysX motion/SLAM success."""
import ast
import asyncio
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from slam_core import WheelOdom
from v66_control import ManualDrive,wheel_limit
from v66_app import Application,build_grid

HERE=Path(__file__).resolve().parent
CFG=json.loads((HERE/'v66_nav_config.json').read_text())


class MigrationTests(unittest.TestCase):
    def test_drive_units_and_signed_wheel_odom(self):
        r,b=CFG['body']['wheel_radius'],CFG['body']['track']
        self.assertEqual(r,.08);self.assertAlmostEqual(b,.4769668332689978)
        odom=WheelOdom(r,b);odom.update(0,0,0.)
        for i in range(1,101):odom.update(i*.01/r,i*.01/r,i*.02)
        np.testing.assert_allclose(odom.pose,[1.,0.,0.],atol=1e-9)
        odom=WheelOdom(r,b);odom.update(0,0,0.)
        for i in range(1,101):
            theta=i*math.pi/200
            odom.update(-theta*b/2/r,theta*b/2/r,i*.02)
        np.testing.assert_allclose(odom.pose,[0,0,math.pi/2],atol=1e-9)
        # v=.9 and w=.9 require peak ~13.933 rad/s, not the old r=.09 value.
        v,w,scale=wheel_limit(.9,.9,r,b,12.)
        self.assertLess(scale,1.)
        self.assertLessEqual(max(abs((v-w*b/2)/r),abs((v+w*b/2)/r)),12.+1e-9)

    def test_manual_forward_has_no_angular_correction(self):
        drive=ManualDrive()
        for i in range(1,201):
            v,w,_=drive.step((.9,0),.02,0,0,i*.02)
            self.assertEqual(w,0.)
        self.assertAlmostEqual(v,.9)

    def make_app(self,folder):
        (folder/'v66_nav_config.json').write_text(json.dumps(CFG))
        app=Application(folder)
        app.simulation_time=lambda:10.
        app.og=SimpleNamespace(Controller=SimpleNamespace(set=lambda *args:None))
        app.linear='linear';app.angular='angular'
        app.odom=SimpleNamespace(pose=np.zeros(3),v=0.,w=0.)
        return app

    def test_old_map_rejected_before_estimator_is_loaded(self):
        with tempfile.TemporaryDirectory(dir=HERE) as temp:
            folder=Path(temp);app=self.make_app(folder)
            maps=folder/'slam_maps';maps.mkdir()
            np.savez(maps/'room_map_latest.npz',metadata_json=json.dumps(dict(scan_backend='PHYSX_COLLISION_RAYCAST')))
            app.scanner=SimpleNamespace(backend='PHYSX_COLLISION_RAYCAST')
            with self.assertRaisesRegex(RuntimeError,'vehicle contract'):
                # This branch refuses before its first await; no event loop or
                # Windows socket pair is needed for this offline contract check.
                coro=app.perform_action('load_map',10.)
                try:coro.send(None)
                finally:coro.close()

    def test_planner_uses_new_footprint_and_exposed_field_settings(self):
        occupied=np.zeros((100,100),bool)
        grid=build_grid((np.array([-2.5,-2.5]),.05,occupied),CFG['body'],CFG['margin'],CFG['rpp'],np.empty((0,2)),CFG['planner'])
        self.assertAlmostEqual(grid.cfg.width,.5698)
        self.assertAlmostEqual(grid.cfg.length,.898)
        self.assertAlmostEqual(grid.cfg.field_weight,3.)
        self.assertFalse(grid.collision(np.zeros(3)))

    def test_scene_startup_contains_no_old_physics_writes(self):
        tree=ast.parse((HERE/'v66_bootstrap.py').read_text())
        forbidden={'Set','Apply','SetActive','setup_simulation','set_dt','CreateRadiusAttr','CreateDampingAttr'}
        writes=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in forbidden]
        self.assertEqual(writes,[])
        audit=ast.parse((HERE/'V66_VehicleAudit.py').read_text())
        writes=[n.func.attr for n in ast.walk(audit) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr in forbidden]
        self.assertEqual(writes,[])

    def test_sensor_is_taken_on_post_physics_step(self):
        with tempfile.TemporaryDirectory(dir=HERE) as temp:
            app=self.make_app(Path(temp))
            app.ready=True
            app.clock_audit=SimpleNamespace(observe=lambda *args:True)
            app.timeline=SimpleNamespace(get_current_time=lambda:10.)
            app.odom=WheelOdom(.08,.4769668332689978)
            app.odom.update(0.,0.,10.-1/60.)
            app.encoders=SimpleNamespace(read=lambda:(0.,0.))
            app.last_stop_sample=app.last_diag=10.
            frame=dict(time_ns=10_000_000_000,x=np.arange(720),z=np.ones(720),hit_states=['EXTERNAL_HIT']*720,prim_paths=['/wall']*720)
            app.scanner=SimpleNamespace(read=lambda:(10_000_000_000,np.ones((30,2))),last_frame=frame)
            app.physics_post(1/60.)
            self.assertIsNone(app.physics_error)
            self.assertEqual(len(app.pending_sensor_frames),1)
            self.assertAlmostEqual(app.odom_history[-1][0],10.)


if __name__=='__main__':unittest.main()
