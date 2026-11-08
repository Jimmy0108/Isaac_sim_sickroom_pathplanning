import ast,json,math,unittest
from pathlib import Path
import numpy as np
from slam_core import *
from rpp_core import RPP
import amr_planner_core as planner

class EstimationTests(unittest.TestCase):
 def test_se2_inverse_composition(self):
  p=np.array([2.,-1.,2.1]);np.testing.assert_allclose(compose(p,inverse(p)),[0,0,0],atol=1e-12)
 def test_odom_actual_angles_straight_reverse_spin(self):
  o=WheelOdom();o.update(0,0,0);np.testing.assert_allclose(o.update(1,1,.1),[.09,0,0],atol=1e-12)
  np.testing.assert_allclose(o.update(0,0,.2),[0,0,0],atol=1e-12)
  p=o.update(-.2,.2,.3);self.assertAlmostEqual(p[2],.4*.09/.514);self.assertAlmostEqual(p[0],0)
 def test_scan_time_interpolation(self):
  h=[(0.,np.array([0.,0.,3.1])),(.1,np.array([1.,0.,-3.1]))]
  p=interpolate_odom(h,.05);self.assertAlmostEqual(p[0],.5);self.assertGreater(abs(p[2]),3.1)
  with self.assertRaisesRegex(RuntimeError,'timestamp'):interpolate_odom(h,1.)
 def test_encoder_clock_reset_rejected(self):
  o=WheelOdom();o.update(0,0,1)
  with self.assertRaisesRegex(RuntimeError,'clock reset'):o.update(0,0,0)
 def test_icp_corrects_prior(self):
  ref=np.random.default_rng(2).uniform(-2,2,(500,2));truth=np.array([.12,-.06,.04]);scan=transform(ref,inverse(truth))
  m=match_scan(scan,ref,[.1,-.04,.03]);self.assertTrue(m.accepted);np.testing.assert_allclose(m.pose,truth,atol=.003)
 def test_unrelated_scan_rejected(self):
  rng=np.random.default_rng(1);m=match_scan(rng.uniform(10,11,(100,2)),rng.uniform(0,1,(100,2)),[0,0,0])
  self.assertFalse(m.accepted)
 def test_pose_graph_loop_reduces_drift(self):
  truth=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,0]],float)
  initial=truth.copy();initial[:,0]+=np.arange(5)*.08
  edges=[(i,i+1,between(initial[i],initial[i+1]),[.1,.1,.1],'odom') for i in range(4)]
  edges.append((0,4,np.zeros(3),[.01,.01,.01],'loop'))
  out=optimize_graph(initial,edges);self.assertLess(np.linalg.norm(out[-1,:2]),np.linalg.norm(initial[-1,:2]));np.testing.assert_allclose(out[0],truth[0])
 def test_save_load_localization(self):
  import tempfile
  from offline_validation import raycast
  cfg=json.loads(Path(__file__).with_name('slam_config.json').read_text());cfg['sensor_origin_front_m']=[0,0,.4]
  obj=Slam2D(cfg);scan=raycast(np.zeros(3));obj.process(scan,np.zeros(3))
  root=Path(__file__).parent/'test_outputs';root.mkdir(exist_ok=True);folder=tempfile.mkdtemp(dir=root)
  obj.save(folder,dict(stage_identifier='test'))
  loaded=Slam2D.load_localization(folder,cfg,[.03,.02,.01]);result=loaded.process(scan,np.zeros(3))
  self.assertTrue(result['ok']);self.assertTrue(loaded.loaded);self.assertEqual(len(loaded.nodes),1)
  self.assertLess(np.linalg.norm(result['pose'][:2]),.02)
 def test_map_odom_propagates_without_truth(self):
  o=np.array([1.,.2,.1]);m=np.array([.2,-.1,.05]);p=compose(m,o)
  np.testing.assert_allclose(compose(p,inverse(o)),m,atol=1e-12)

class RPPTests(unittest.TestCase):
 def setUp(self):
  self.cfg=json.loads(Path(__file__).with_name('slam_config.json').read_text())['rpp'];self.grid=planner.Grid(np.zeros((140,140),bool),.05,(0,0),planner.Config())
 def controller(self,reverse=False):
  p=np.array([2.,2.,0.]);sign=-1 if reverse else 1
  rows=[(p,0,0,0)]+[(np.array([2+sign*x,2.,0.]),sign*.35,0,.1) for x in np.linspace(.02,1,50)]
  c=RPP(self.cfg);c.set_path(rows);return c,p
 def test_forward_reverse(self):
  for rev in (False,True):
   c,p=self.controller(rev);v,w,state=c.command(p,0,self.grid,np.empty((0,2)),.1)
   self.assertGreater(v,0) if not rev else self.assertLess(v,0)
   self.assertAlmostEqual(w,0)
 def test_local_obstacle_stops(self):
  c,p=self.controller();v,w,state=c.command(p,0,self.grid,np.array([[2.3,2.]]),.1)
  self.assertEqual(v,0);self.assertTrue(state.startswith('replan'))
 def test_cost_regulates_speed(self):
  c,p=self.controller();c.last_v=.35
  a=c.command(p,.35,self.grid,np.empty((0,2)),.1)[0]
  c,p=self.controller();c.last_v=.35
  b=c.command(p,.35,self.grid,np.array([[2.,2.6]]),.1)[0]
  self.assertLess(b,a)
 def test_spin_segment_keeps_zero_linear(self):
  c=RPP(self.cfg);p=np.array([2.,2.,0.]);c.set_path([(p,0,0,0),(np.array([2.,2.,1.]),0,.7,1)])
  v,w,state=c.command(p,0,self.grid,np.empty((0,2)),.1);self.assertEqual(v,0);self.assertGreater(w,0)

class SourceBoundaryTests(unittest.TestCase):
 def test_no_truth_in_estimator_or_runtime(self):
  root=Path(__file__).parent
  for name in ('slam_core.py','rpp_core.py','isaac_slam_app.py'):
   s=(root/name).read_text(encoding='utf-8-sig')
   self.assertNotIn('GetLocalToWorldTransform',s);self.assertNotIn('get_root_transforms',s)
 def test_encoder_reads_positions_not_commands(self):
  tree=ast.parse(Path(__file__).with_name('isaac_io.py').read_text(encoding='utf-8-sig'))
  encoder=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='EncoderReader')
  text=ast.unparse(encoder);self.assertIn('get_dof_positions',text);self.assertNotIn('get_dof_position_targets',text)

if __name__=='__main__':unittest.main(verbosity=2)
