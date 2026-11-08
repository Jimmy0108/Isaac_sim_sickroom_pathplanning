"""Offline tests of the Isaac adapter math; NOT an Isaac runtime test."""
import ast
import math
from pathlib import Path
import unittest
import numpy as np

source=Path(__file__).with_name('AMR_Hybrid_Navigate_Isaac.py').read_text(encoding='utf-8-sig')
m=ast.parse(source)
names={'wrap','convex_hull','raster_polygon','group_segments','follow_command'}
ns=dict(np=np,math=math,POSITION_TOLERANCE=.06,YAW_TOLERANCE=math.radians(5),ASSUMED_DECELERATION=.4)
exec(compile(ast.Module(body=[n for n in m.body if isinstance(n,ast.FunctionDef) and n.name in names],type_ignores=[]),'adapter_math','exec'),ns)

class AdapterTests(unittest.TestCase):
    def test_raster_includes_intersecting_cells(self):
        xx,yy=np.meshgrid(np.array([0.,.1,.2]),np.array([0.,.1,.2]))
        poly=np.array([[.04,.04],[.06,.04],[.06,.06],[.04,.06]])
        mask=ns['raster_polygon'](xx,yy,poly,.1)
        self.assertTrue(mask[0,0]);self.assertTrue(mask[1,1]);self.assertFalse(mask[2,2])
    def test_rotated_proxy_not_world_aabb(self):
        poly=np.array([[0,1],[1,0],[0,-1],[-1,0]])
        mask=ns['raster_polygon'](np.array([[.9]]),np.array([[.9]]),poly,.05)
        self.assertFalse(mask[0,0])
    def test_convex_hull_discards_internal_and_duplicate_points(self):
        hull=ns['convex_hull']([[0,0],[1,0],[1,1],[0,1],[.5,.5],[0,0]])
        self.assertEqual(len(hull),4)
    def test_reverse_preserved(self):
        seg=dict(v=-.35,w=0,points=np.array([[-.1,0,0],[-.3,0,0]]))
        v,w,i,done=ns['follow_command'](np.array([0.,0.,0.]),seg,0,.4,.7)
        self.assertLess(v,0);self.assertAlmostEqual(w,0);self.assertFalse(done)
    def test_spin_does_not_translate(self):
        seg=dict(v=0,w=.7,points=np.array([[0,0,math.pi/2]]))
        v,w,i,done=ns['follow_command'](np.zeros(3),seg,0,.4,.7)
        self.assertEqual(v,0);self.assertGreater(w,0);self.assertFalse(done)
    def test_segments_do_not_skip_reversal(self):
        samples=[(np.zeros(3),0,0,0),(np.ones(3),.3,0,.1),(np.ones(3),0,.7,.1),(np.ones(3),-.3,0,.1)]
        seg=ns['group_segments'](samples)
        self.assertEqual([x['v'] for x in seg],[.3,0,-.3])
    def test_front_minus_x_tracking(self):
        seg=dict(v=.35,w=0,points=np.array([[-.1,0,math.pi],[-.3,0,math.pi]]))
        v,w,i,done=ns['follow_command'](np.array([0.,0.,math.pi]),seg,0,.4,.7)
        self.assertGreater(v,0);self.assertAlmostEqual(w,0,places=8)
        self.assertLess(-v,0) # original base_link controller command

if __name__=='__main__':unittest.main(verbosity=2)
