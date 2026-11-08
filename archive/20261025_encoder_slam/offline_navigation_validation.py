"""Run offline_validation.py first. Validate planner and RPP on its estimated map."""
from pathlib import Path
import json,math
import numpy as np
import amr_planner_core as core
from rpp_core import RPP

def run():
 root=Path(__file__).parent;out=root/'validation_outputs'
 cfg=json.loads((root/'slam_config.json').read_text())
 with np.load(out/'room_map_latest.npz',allow_pickle=False) as d:
  grid=core.Grid(d['occupied'],float(d['resolution']),d['origin'],core.Config(nominal_v=.35,nominal_w=.7))
 start=np.array([0.,0.,0.]);goal=np.array([4.,3.,math.pi/2]);rows,stats=core.plan(grid,start,goal)
 control=RPP(cfg['rpp']);control.set_path(rows);p=start.copy();vlast=0.;trajectory=[p.copy()]
 for i in range(6000):
  v,w,state=control.command(p,vlast,grid,np.empty((0,2)),.05)
  if state=='done':break
  if state.startswith('replan'):raise RuntimeError((state,i,p))
  p=core.integrate(p,v,w,.05);vlast=v;trajectory.append(p.copy())
 else:raise RuntimeError('RPP tracking did not finish')
 assert np.linalg.norm(p[:2]-goal[:2])<.07
 report=dict(mode='SYNTHETIC SLAM map -> Hybrid A* / Voronoi -> ideal RPP tracking; NOT Isaac Sim',expansions=stats['expansions'],steps=i,
  final_position_error_m=float(np.linalg.norm(p[:2]-goal[:2])),final_yaw_error_rad=abs(core.wrap(p[2]-goal[2])))
 (out/'end_to_end.json').write_text(json.dumps(report,indent=2))
 import matplotlib
 matplotlib.use('Agg')
 import matplotlib.pyplot as plt
 path=np.array([r[0] for r in rows]);tr=np.array(trajectory)
 fig,ax=plt.subplots(figsize=(9,7));ax.imshow(grid.occupied,origin='lower',cmap='Greys',extent=[grid.origin[0],grid.origin[0]+grid.w*grid.res,grid.origin[1],grid.origin[1]+grid.h*grid.res])
 ax.plot(path[:,0],path[:,1],color='limegreen',label='Hybrid A* / Voronoi');ax.plot(tr[:,0],tr[:,1],color='royalblue',lw=1,label='Ideal RPP tracking')
 ax.scatter([0,4],[0,3],c=['blue','red']);ax.set(xlim=(-.8,6),ylim=(-.8,4.8),xlabel='SLAM map X [m]',ylabel='SLAM map Y [m]',title='SYNTHETIC estimated map + navigation (not Isaac Sim)',aspect='equal');ax.legend();fig.savefig(out/'synthetic_navigation.png',dpi=150);plt.close(fig)
 print(json.dumps(report,indent=2));return report

if __name__=='__main__':run()
