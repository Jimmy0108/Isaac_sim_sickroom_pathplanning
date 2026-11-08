"""Offline USD migration. No Isaac runtime success is inferred."""
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf,Vt
from pathlib import Path
import json,math,hashlib
W=Path(__file__).parent.parent;P=W/'outputs/Isaac_V6_6_Migration';P.mkdir(exist_ok=True)
geom=W/'outputs/revision_20261105_v6_6_caster_ground/TZAGV_B02_v6_6_geometry.usdc'
source=W/'AGV_configured_v4.usd';control=W/'AGV_control.usd'
hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [geom,source,control]}
old=Usd.Stage.Open(str(source));g=Usd.Stage.Open(str(geom));layer=Sdf.Layer.CreateNew(str(P/'AGV_configured_v66.usd'));s=Usd.Stage.Open(layer)
UsdGeom.Xform.Define(s,'/Root');s.SetDefaultPrim(s.GetPrimAtPath('/Root'));UsdGeom.SetStageMetersPerUnit(s,1);UsdGeom.SetStageUpAxis(s,'Z')
newprefix='/Root/TZAGV_B02_vendor_v4';oldprefix='/TZAGV_B02'
Sdf.CopySpec(g.GetRootLayer(),oldprefix,layer,newprefix)
# Remap material connections and relationship targets copied across root namespaces.
for p in s.Traverse():
 for a in p.GetAttributes():
  cs=a.GetConnections()
  if cs:a.SetConnections([q.ReplacePrefix(Sdf.Path(oldprefix),Sdf.Path(newprefix)) for q in cs])
 for r in p.GetRelationships():
  ts=r.GetTargets()
  if ts:r.SetTargets([q.ReplacePrefix(Sdf.Path(oldprefix),Sdf.Path(newprefix)) for q in ts])
ROOT=newprefix+'/TZAGV_B02_root';BASE=ROOT+'/base_link'
links=[p for p in s.Traverse() if p.GetName()=='base_link' or p.GetName().endswith('_link')]
assert len(links)==11,len(links)
massmap={}
for p in links:
 prior=old.GetPrimAtPath(ROOT+'/'+p.GetName());assert prior
 UsdPhysics.RigidBodyAPI.Apply(p).CreateRigidBodyEnabledAttr(True)
 mass=float(UsdPhysics.MassAPI(prior).GetMassAttr().Get());ma=UsdPhysics.MassAPI.Apply(p);ma.CreateMassAttr(mass);massmap[p.GetName()]=mass
 # Explicit simple inertias: baseline mass retained; no stale old tensor copied.
 if p.GetName()=='base_link':
  L,B,H=.898,.5698,.444;inertia=(mass*(B*B+H*H)/12,mass*(L*L+H*H)/12,mass*(L*L+B*B)/12);com=(0,0,.18)
 elif 'drive_wheel' in p.GetName():
  r,b=.08,.055;inertia=(mass*(3*r*r+b*b)/12,mass*r*r/2,mass*(3*r*r+b*b)/12);com=(0,0,0)
 elif 'wheel_link' in p.GetName():
  r,b=.0315,.0192629283;inertia=(mass*(3*r*r+b*b)/12,mass*r*r/2,mass*(3*r*r+b*b)/12);com=(0,0,0)
 else:inertia=(.002,.002,.002);com=(0,0,0)
 ma.CreateCenterOfMassAttr(Gf.Vec3f(*com));ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia));ma.CreatePrincipalAxesAttr(Gf.Quatf(1))
UsdPhysics.ArticulationRootAPI.Apply(s.GetPrimAtPath(BASE))
s.GetPrimAtPath(BASE).SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(list(s.GetPrimAtPath(BASE).GetAppliedSchemas())+['PhysxArticulationAPI']))
s.GetPrimAtPath(BASE).CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(False)
# Remove all geometry guide proxies; create one explicit active set only.
for path in sorted([str(p.GetPath()) for p in s.Traverse() if 'collision' in p.GetName().lower()],key=len,reverse=True):
 s.RemovePrim(path)
mat=UsdShade.Material.Define(s,ROOT+'/PhysicsMaterials/Wheel');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(.7);pm.CreateDynamicFrictionAttr(.6);pm.CreateRestitutionAttr(0)
def collider(p):
 UsdPhysics.CollisionAPI.Apply(p).CreateCollisionEnabledAttr(True);UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,UsdShade.Tokens.strongerThanDescendants,'physics');UsdGeom.Imageable(p).CreatePurposeAttr('guide')
wheelpaths=[]
for p in links:
 n=p.GetName()
 if 'wheel_link' not in n:continue
 r,b=(.08,.055) if 'drive' in n else (.0315,.019262928306058)
 c=UsdGeom.Cylinder.Define(s,p.GetPath().AppendChild('wheel_contact_collider'));c.CreateAxisAttr('Y');c.CreateRadiusAttr(r);c.CreateHeightAttr(b);c.CreateExtentAttr([Gf.Vec3f(-r,-b/2,-r),Gf.Vec3f(r,b/2,r)]);collider(c.GetPrim());wheelpaths.append(str(c.GetPath()))
def cube(name,lo,hi):
 c=UsdGeom.Cube.Define(s,BASE+'/Collision/'+name);c.CreateSizeAttr(1);x=UsdGeom.Xformable(c);x.AddTranslateOp().Set(Gf.Vec3d(*[(a+b)/2 for a,b in zip(lo,hi)]));x.AddScaleOp().Set(Gf.Vec3f(*[b-a for a,b in zip(lo,hi)]));collider(c.GetPrim())
# Compound chassis proxies preserve front/rear recess volume, unlike one convex hull.
cube('center',(-.325,-.28,.046),(.325,.28,.296))
for sign,label in [(-1,'front'),(1,'rear')]:
 xlo,xhi=sorted([sign*.325,sign*.405]);cube(label+'_lower',(xlo,-.28,.046),(xhi,.28,.1105));cube(label+'_upper',(xlo,-.28,.1915),(xhi,.28,.296))
cube('first_deck',(-.401,-.276,.296),(.401,.276,.301));cube('second_deck',(-.401,-.276,.441),(.401,.276,.444))
cube('front_bumper',(-.449,-.28,.055),(-.405,.28,.085));cube('rear_bumper',(.405,-.28,.055),(.449,.28,.085))
# Five enclosure walls: thin convex Mesh collisions from actual new wall geometry.
for wall in [p for p in s.Traverse() if p.IsA(UsdGeom.Mesh) and 'enclosure_pentagon_panel' in str(p.GetPath())]:
 dst=BASE+'/Collision/'+wall.GetParent().GetName();Sdf.CopySpec(layer,wall.GetParent().GetPath(),layer,dst)
 for q in Usd.PrimRange(s.GetPrimAtPath(dst)):
  UsdGeom.Imageable(q).CreatePurposeAttr('guide')
  if q.IsA(UsdGeom.Mesh):collider(q);UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr('convexHull')
reference=json.loads((W/'outputs/revision_20261105_v6_6_caster_ground/joint_migration_REFERENCE_ONLY.json').read_text(encoding='utf8'))
for j in reference['joints']:
 path=ROOT+'/'+j['body1']+'/'+j['name'];q=UsdPhysics.RevoluteJoint.Define(s,path);q.CreateBody0Rel().SetTargets([ROOT+'/'+j['body0']]);q.CreateBody1Rel().SetTargets([ROOT+'/'+j['body1']]);q.CreateAxisAttr(j['axis']);q.CreateLocalPos0Attr(Gf.Vec3f(*j['local_pos0_m']));q.CreateLocalPos1Attr(Gf.Vec3f(*j['local_pos1_m']));q.CreateLocalRot0Attr(Gf.Quatf(1));q.CreateLocalRot1Attr(Gf.Quatf(1));q.CreateCollisionEnabledAttr(False)
 if j['name'] in ['left_wheel_joint','right_wheel_joint']:
  drv=UsdPhysics.DriveAPI.Apply(q.GetPrim(),'angular');drv.CreateTypeAttr('force');drv.CreateStiffnessAttr(0);drv.CreateDampingAttr(100);drv.CreateMaxForceAttr(5);drv.CreateTargetVelocityAttr(0)
 # Passive caster joints: no target motors/heading compensation.
# Keep the old RTX sensor profile, but set new approximate mount. D555 stays geometry.
Sdf.CopySpec(old.GetRootLayer(),BASE+'/Lidar_2D',layer,BASE+'/Lidar_2D');lid=s.GetPrimAtPath(BASE+'/Lidar_2D');x=UsdGeom.Xformable(lid);x.ClearXformOpOrder();x.AddTranslateOp().Set(Gf.Vec3d(-.23,0,.345));x.AddRotateZOp().Set(180)
lid.SetCustomDataByKey('migration:mountStatus','Approximate optical plane; verify before mapping')
layer.Save()
# Copy source control, retain graph ownership, explicitly author all names and dimensions.
ctl=Sdf.Layer.CreateNew(str(P/'AGV_control_v66.usd'));ctl.TransferContent(Sdf.Layer.FindOrOpen(str(control)));cs=Usd.Stage.Open(ctl);UsdGeom.SetStageMetersPerUnit(cs,1)
gp='/Root/AGV_DifferentialGraph';dc=cs.GetPrimAtPath(gp+'/DifferentialController');dc.GetAttribute('inputs:wheelRadius').Set(.08);dc.GetAttribute('inputs:wheelDistance').Set(.4769668332689978)
for n in ['linearVelocity','angularVelocity']:dc.GetAttribute('inputs:'+n).Set(0)
ac=cs.GetPrimAtPath(gp+'/ArticulationController');target='/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root/base_link';ac.GetRelationship('inputs:targetPrim').SetTargets([target]);ac.GetAttribute('inputs:robotPath').Set(target)
for i,n in enumerate(['left_wheel_joint','right_wheel_joint']):cs.GetPrimAtPath(gp+'/ArrayNames').GetAttribute('inputs:input'+str(i)).Set(n)
ctl.Save()
# Standalone acceptance scene: one PhysicsScene and a simple floor, no ward edits.
test=Usd.Stage.CreateNew(str(P/'V66_TestScene.usda'));UsdGeom.SetStageMetersPerUnit(test,1);UsdGeom.SetStageUpAxis(test,'Z');world=UsdGeom.Xform.Define(test,'/World');test.SetDefaultPrim(world.GetPrim())
robot=UsdGeom.Xform.Define(test,'/World/AGV_configured_v4');robot.GetPrim().GetReferences().AddReference('./AGV_configured_v66.usd');robot.AddTranslateOp().Set(Gf.Vec3d(0,0,.01))
ctrl=UsdGeom.Xform.Define(test,'/World/AGV_control');ctrl.GetPrim().GetReferences().AddReference('./AGV_control_v66.usd')
ph=UsdPhysics.Scene.Define(test,'/World/PhysicsScene');ph.CreateGravityDirectionAttr(Gf.Vec3f(0,0,-1));ph.CreateGravityMagnitudeAttr(9.81)
ph.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit(['PhysxSceneAPI']));ph.GetPrim().CreateAttribute('physxScene:timeStepsPerSecond',Sdf.ValueTypeNames.UInt).Set(240);ph.GetPrim().CreateAttribute('physxScene:solverType',Sdf.ValueTypeNames.Token).Set('TGS')
floor=UsdGeom.Cube.Define(test,'/World/Ground');floor.CreateSizeAttr(1);xf=UsdGeom.Xformable(floor);xf.AddTranslateOp().Set(Gf.Vec3d(0,0,-.05));xf.AddScaleOp().Set(Gf.Vec3f(10,10,.1));UsdPhysics.CollisionAPI.Apply(floor.GetPrim());test.GetRootLayer().Save()
# Reopen composed scene: joints meet, all targets resolve, six native wheel bottoms equal.
s=Usd.Stage.Open(str(P/'AGV_configured_v66.usd'));cache=UsdGeom.XformCache();errors=[]
for p in s.Traverse():
 if p.IsA(UsdPhysics.Joint):
  j=UsdPhysics.Joint(p);a=s.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0]);b=s.GetPrimAtPath(j.GetBody1Rel().GetTargets()[0]);pa=cache.GetLocalToWorldTransform(a).Transform(Gf.Vec3d(j.GetLocalPos0Attr().Get()));pb=cache.GetLocalToWorldTransform(b).Transform(Gf.Vec3d(j.GetLocalPos1Attr().Get()));e=(pa-pb).GetLength();assert e<2e-6,(p.GetPath(),e);errors.append(e)
bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['guide']);bottoms=[]
for path in wheelpaths:
 z=bbox.ComputeWorldBound(s.GetPrimAtPath(path)).ComputeAlignedRange().GetMin()[2];assert abs(z)<2e-6,(path,z);bottoms.append(z)
test=Usd.Stage.Open(str(P/'V66_TestScene.usda'));assert test.GetPrimAtPath(target);assert len([p for p in s.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)])==11
assert len([p for p in s.Traverse() if p.IsA(UsdPhysics.Joint)])==10
missing=[]
for p in s.Traverse():
 for r in p.GetRelationships():
  for t in r.GetTargets():
   if not s.GetObjectAtPath(t):missing.append(str(t))
assert not missing,missing
for path,h in hashes.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==h
report=dict(source_hashes=hashes,offline_validation='PASS',rigid_bodies=11,joints=10,native_wheel_colliders=6,wheel_bottoms_m=bottoms,max_joint_anchor_error_m=max(errors),all_relationship_targets_resolve=True,test_scene_has_target=True,source_files_unchanged=True,isaac_runtime_tested=False,mass_policy='old masses preserved; box/cylinder inertia approximation and base COM180mm are explicit initial engineering assumptions, not CAD measured',wheel_friction='static0.7/dynamic0.6 provisional baseline, not tuned',drive='source maxForce5 damping100 preserved',lidar='old RTX profile preserved; mount optical Z345mm approximate; not scan validated',D555='geometry only',motion_control='source OnPlaybackTick chain retained, no manual heading compensation')
(P/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
(P/'vehicle_geometry_config.json').write_text(json.dumps(dict(wheel_radius=.08,track=.4769668332689978,length=.898,width=.5698,height=.444,front_axis_sign=-1,linear_command_sign=-1,angular_command_sign=1,logical_left_joint='right_wheel_joint',logical_right_joint='left_wheel_joint',encoder_sign_left=-1,encoder_sign_right=-1,lidar_xyz_m=[-.23,0,.345],note='Merge into matching existing config fields; not auto-read by old Python'),indent=2),encoding='utf8')
print('MIGRATION_OFFLINE_PASS',len(errors),max(errors),bottoms)

