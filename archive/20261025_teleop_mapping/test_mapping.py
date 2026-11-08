import json,math,tempfile,unittest
from pathlib import Path
import numpy as np
from nav_toolkit import Mapper,bresenham,body_filter
BODY=dict(length=.88,width=.56,height=.45)
class MappingTests(unittest.TestCase):
 def test_ray_trace(self):
  self.assertEqual(bresenham(1,1,1,1),[(1,1)])
  self.assertEqual(bresenham(0,0,2,1)[-1],(1,2))
 def test_missing_angles_stay_unknown(self):
  m=Mapper((0,0),(100,100),.05)
  m.update(np.array([2.,2.,.4]),np.array([[4.,2.,.4]]),np.array([2.,2.,0.]),0.,BODY)
  self.assertTrue(m.seen[m.cell([3,2])]);self.assertFalse(m.seen[m.cell([2,3])])
  self.assertGreater(m.log[m.cell([4,2])],0)
 def test_hits_not_erased_by_other_rays(self):
  m=Mapper((0,0),(100,100),.05)
  cloud=np.array([[3.,2.,.4],[4.,2.,.4]])
  m.update(np.array([2.,2.,.4]),cloud,np.array([2.,2.,0.]),0.,BODY)
  self.assertGreater(m.log[m.cell([3,2])],0)
 def test_body_filter_rotates_with_robot(self):
  cloud=np.array([[1.,1.3,.4],[1.4,1.,.4]])
  out=body_filter(cloud,np.array([1.,1.,math.pi/2]),0,BODY)
  self.assertEqual(len(out),1);np.testing.assert_allclose(out[0],cloud[1])
 def test_mapping_from_two_robot_positions(self):
  m=Mapper((0,0),(140,140),.05)
  m.update(np.array([2.,2.,.4]),np.array([[4.,2.,.4]]),np.array([2.,2.,0]),0,BODY)
  m.update(np.array([2.,3.,.4]),np.array([[4.,3.,.4]]),np.array([2.,3.,0]),0,BODY)
  self.assertLess(m.log[m.cell([3,2])],0);self.assertLess(m.log[m.cell([3,3])],0)
 def test_save_unknown_blocked_and_roundtrip(self):
  m=Mapper((0,0),(100,100),.05)
  m.update(np.array([2.,2.,.4]),np.array([[4.,2.,.4]]),np.array([2.,2.,0]),0,BODY)
  output=Path(__file__).parent/'test_outputs';output.mkdir(exist_ok=True)
  folder=tempfile.mkdtemp(dir=output)
  if True:
   file=m.save(folder,dict(stage_identifier='test',sensor_world_z=.4))
   with np.load(file,allow_pickle=False) as d:
    self.assertTrue(d['occupied'][m.cell([2,3])]);self.assertFalse(d['occupied'][m.cell([3,2])])
   other,meta=Mapper.load(file,'test');np.testing.assert_allclose(m.log,other.log)
   self.assertTrue((Path(folder)/'room_map_latest.png').read_bytes().startswith(b'\x89PNG'))
   with self.assertRaisesRegex(RuntimeError,'different stage'):Mapper.load(file,'other-stage')
 def test_out_of_bounds_sensor_rejected(self):
  m=Mapper((0,0),(100,100),.05)
  with self.assertRaisesRegex(RuntimeError,'outside mapping'):
   m.update(np.array([8.,8.,.4]),np.empty((0,3)),np.array([8.,8.,0]),0,BODY)
if __name__=='__main__':unittest.main(verbosity=2)
