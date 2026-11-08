import math
import unittest
import numpy as np
from amr_planner_core import Config, Grid, integrate, wheels, plan

class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.cfg=Config(timeout_seconds=10)
    def empty(self):
        return Grid(np.zeros((100,100),bool),.05,(0,0),self.cfg)
    def test_wheel_track_not_body_width(self):
        l,r=wheels(0,1,self.cfg)
        self.assertAlmostEqual(l,-.514/(2*.09))
        self.assertAlmostEqual(r,.514/(2*.09))
        self.assertNotAlmostEqual(r,.56/(2*.09))
    def test_kinematics_spin_reverse_arc(self):
        p=np.array([2.,2.,0.])
        np.testing.assert_allclose(integrate(p,0,1,1)[:2],p[:2])
        self.assertLess(integrate(p,-.3,0,1)[0],p[0])
        q=integrate(p,.3,.8,1)
        self.assertGreater(q[1],2)
        self.assertAlmostEqual(q[2],.8)
    def test_field_depends_on_voronoi_distance(self):
        g=self.empty()
        self.assertTrue(np.any(g.voronoi))
        self.assertTrue(np.all(g.field[~g.occupied]>=0))
        self.assertTrue(np.all(g.field[~g.occupied]<=1))
        self.assertLess(np.max(g.field[g.voronoi]),1e-8)
        self.assertGreater(g.field[1,50],0)
    def test_rectangular_footprint_and_spin_sweep(self):
        occ=np.ones((100,100),bool)
        occ[39:61,5:95]=False  # 1.10 m corridor
        g=Grid(occ,.05,(0,0),self.cfg)
        self.assertFalse(g.collision(np.array([2.5,2.5,0.])))
        # Narrower corridor fits width but not a 90 degree rotated length.
        occ=np.ones((100,100),bool);occ[42:58,5:95]=False # .80 m
        g=Grid(occ,.05,(0,0),self.cfg)
        p=np.array([2.5,2.5,0.])
        self.assertFalse(g.collision(p))
        self.assertTrue(g.collision(np.array([2.5,2.5,math.pi/2])))
        self.assertIsNone(g.edge(p,0,1,math.pi/2))
    def test_translation_sweep_does_not_jump_thin_wall(self):
        occ=np.zeros((100,100),bool);occ[:,50]=True
        g=Grid(occ,.05,(0,0),self.cfg)
        self.assertIsNone(g.edge(np.array([1.,2.,0.]),1,0,3))
    def test_bad_goal_rejected(self):
        g=self.empty()
        with self.assertRaisesRegex(ValueError,'Goal footprint'):
            plan(g,[1,1,0],[.1,.1,0])
    def test_goal_heading_and_sampled_wheel_kinematics(self):
        g=self.empty();goal=np.array([2.3,2.0,math.pi/2])
        rows,stats=plan(g,[1.5,2.,0],goal)
        np.testing.assert_allclose(rows[-1][0],goal,atol=1e-9)
        for prev,row in zip(rows[:-1],rows[1:]):
            p,v,w,dt=row
            np.testing.assert_allclose(integrate(prev[0],v,w,dt),p,atol=1e-9)
            self.assertFalse(g.collision(p))
            l,r=wheels(v,w,self.cfg)
            self.assertAlmostEqual((l+r)*self.cfg.wheel_radius/2,v)
            self.assertAlmostEqual((r-l)*self.cfg.wheel_radius/self.cfg.track,w)
    def test_closed_barrier_no_path(self):
        cfg=Config(max_expansions=200,timeout_seconds=3)
        occ=np.zeros((80,80),bool);occ[:,40]=True
        g=Grid(occ,.05,(0,0),cfg)
        with self.assertRaisesRegex(RuntimeError,'No solution'):
            plan(g,[1.,2.,0],[3.,2.,0])

if __name__=='__main__':unittest.main(verbosity=2)
