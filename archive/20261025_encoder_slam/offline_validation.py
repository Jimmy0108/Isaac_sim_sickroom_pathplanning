"""Synthetic lidar/biased encoder validation. Ground truth is ONLY a test fixture."""
from pathlib import Path
import json,math
import numpy as np
from slam_core import *


def raycast(pose):
    rects=[(-1,-1,7,5),(2,1,3.7,2.3),(5,3.7,6,4.7),(-.5,3.5,.4,4.6)]
    walls=[]
    for x0,y0,x1,y1 in rects:
        q=np.array([[x0,y0],[x1,y0],[x1,y1],[x0,y1]])
        walls.extend(zip(q,np.roll(q,-1,axis=0)))
    az=np.linspace(-np.pi,np.pi,240,endpoint=False)
    d=np.c_[np.cos(az+pose[2]),np.sin(az+pose[2])]
    best=np.full(len(az),12.)
    def cross(a,b):return a[...,0]*b[...,1]-a[...,1]*b[...,0]
    for a,b in walls:
        edge=b-a;q=a-pose[:2];den=cross(d,edge);safe=np.where(abs(den)<1e-9,np.nan,den)
        t=cross(q,edge)/safe;u=cross(q,d)/safe
        valid=(t>.1)&(u>=0)&(u<=1)
        best=np.minimum(best,np.where(valid,t,12.))
    return np.c_[best*np.cos(az),best*np.sin(az)]


def run():
    cfg=json.loads(Path(__file__).with_name('slam_config.json').read_text());cfg['sensor_origin_front_m']=[0,0,.4]
    slam=Slam2D(cfg);odom=WheelOdom();odom.update(0,0,0);q=np.zeros(2);p=np.zeros(3);time=0;history=[]
    def frame(distance,angle):
        nonlocal p,q,time
        from amr_planner_core import integrate
        p=integrate(p,distance,angle,1.)
        delta=np.array([distance-angle*.514/2,distance+angle*.514/2])/.09
        q+=delta*np.array([1.005,.995]);time+=.2;od=odom.update(q[0],q[1],time)
        result=slam.process(raycast(p),od)
        history.append([*p,*od,*result['pose'],int(result['ok']),result['loops']])
    frame(0,0)
    for length in (4.,3.,4.,3.):
        n=int(math.ceil(length/.16))
        for _ in range(n):frame(length/n,0)
        for _ in range(10):frame(0,np.pi/20)
    out=Path(__file__).parent/'validation_outputs';out.mkdir(exist_ok=True)
    h=np.array(history);np.savez(out/'synthetic_session.npz',history=h)
    slam.save(out,dict(stage_identifier='SYNTHETIC TEST ONLY',display_anchor=[0,0,0],display_z=0,sensor_origin_front_m=[0,0,.4]))
    accepted=h[:,9].mean();odom_error=np.linalg.norm(h[-1,3:5]-h[-1,:2]);slam_error=np.linalg.norm(h[-1,6:8]-h[-1,:2])
    summary=dict(mode='SYNTHETIC TEST, NOT ISAAC SIM',frames=len(h),accepted_fraction=float(accepted),loop_closures=slam.loops,
                 final_odom_error_m=float(odom_error),final_slam_error_m=float(slam_error),keyframes=len(slam.nodes))
    (out/'synthetic_summary.json').write_text(json.dumps(summary,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(8,7));ax.plot(h[:,0],h[:,1],'k--',label='Synthetic ground truth');ax.plot(h[:,3],h[:,4],label='Biased encoder odom');ax.plot(h[:,6],h[:,7],label='Online SLAM estimate')
    poses=np.array(slam.nodes);ax.plot(poses[:,0],poses[:,1],'.',label='Optimized keyframes')
    ax.set(xlabel='map X [m]',ylabel='map Y [m]',title='SYNTHETIC lidar + encoder test (not Isaac Sim)',aspect='equal');ax.legend();ax.grid();fig.savefig(out/'synthetic_slam.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,indent=2));return summary

if __name__=='__main__':run()
