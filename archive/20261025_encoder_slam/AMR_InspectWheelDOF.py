# Read-only physical articulation / wheel DOF check. Requires Play, sends NO motor commands.
from pathlib import Path
import sys,json
PROJECT=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav')
if str(PROJECT) not in sys.path:sys.path.insert(0,str(PROJECT))
import omni.usd,omni.timeline
from pxr import UsdPhysics
from isaac_io import find_robot,EncoderReader
stage=omni.usd.get_context().get_stage()
cfg=json.loads((PROJECT/'slam_config.json').read_text(encoding='utf-8-sig'))
if stage is None:print('Open a scene first.')
else:
 print('ArticulationRoot prims:')
 for p in stage.Traverse():
  if p.HasAPI(UsdPhysics.ArticulationRootAPI):print(str(p.GetPath()))
 if not omni.timeline.get_timeline_interface().is_playing():print('Press Play, then run this inspector to read actual DOF positions.')
 else:
  try:
   reader=EncoderReader(stage,find_robot(stage,cfg),cfg);print('Logical encoder angles [rad]:',reader.read())
  except Exception as exc:print('Encoder inspection:',type(exc).__name__,str(exc))
 print('No movement commands or geometry edits were made.')
