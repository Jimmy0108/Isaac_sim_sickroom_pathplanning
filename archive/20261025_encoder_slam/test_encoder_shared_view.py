"""Exercise encoder IO with simulated manager/tensor frontends, without Isaac."""
import ast
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock,patch
import numpy as np


class EncoderSharedViewTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).with_name('isaac_io.py').read_text(encoding='utf-8')
        node=next(n for n in ast.parse(source).body
                  if isinstance(n,ast.ClassDef) and n.name=='EncoderReader')
        namespace={'np':np}
        exec(compile(ast.Module(body=[node],type_ignores=[]),'<encoder test>','exec'),namespace)
        self.Reader=namespace['EncoderReader']
        pxr=types.ModuleType('pxr')
        pxr.UsdPhysics=types.SimpleNamespace()
        self.modules=patch.dict(sys.modules,{'pxr':pxr})
        self.modules.start()
        self.addCleanup(self.modules.stop)
        self.config={'front_axis_sign':-1,'encoders':{
            'articulation_path':'/World/AGV/base_link',
            'original_left_joint':'left_drive_wheel_joint',
            'original_right_joint':'right_drive_wheel_joint',
            'logical_left_sign':-1,'logical_right_sign':-1}}
        self.joints=types.SimpleNamespace(count=1,
            shared_metatype=types.SimpleNamespace(dof_names=[
                'left_drive_wheel_joint','right_drive_wheel_joint']),
            get_dof_positions=Mock(return_value=np.array([[1.,2.]])))
        self.sim=types.SimpleNamespace(is_valid=True,
            create_articulation_view=Mock(return_value=self.joints))

    def reader(self):
        return self.Reader(None,{},self.config,simulation_view=self.sim)

    def test_reuses_exact_attached_view_and_reads_measured_positions(self):
        reader=self.reader()
        self.assertIs(reader.sim,self.sim)
        self.sim.create_articulation_view.assert_called_once_with('/World/AGV/base_link')
        self.assertEqual(reader.read(),(-2.,-1.))

    def test_accepts_warp_and_gpu_torch_conversion(self):
        for methods in (['numpy'],['detach','cpu','numpy']):
            with self.subTest(methods=methods):
                calls=[]
                tensor=types.SimpleNamespace()
                for name in methods:
                    def convert(name=name):
                        calls.append(name)
                        return np.array([[1.,2.]]) if name=='numpy' else tensor
                    setattr(tensor,name,convert)
                self.joints.get_dof_positions.return_value=tensor
                self.assertEqual(self.reader().read(),(-2.,-1.))
                self.assertEqual(calls,methods)

    def test_invalid_simulation_view_fails_before_articulation_creation(self):
        self.sim.is_valid=False
        with self.assertRaisesRegex(RuntimeError,'invalid'):
            self.reader()
        self.sim.create_articulation_view.assert_not_called()

    def test_missing_manager_view_does_not_create_a_replacement(self):
        manager=types.ModuleType('isaacsim.core.simulation_manager')
        getter=Mock(return_value=None)
        manager.SimulationManager=types.SimpleNamespace(get_physics_simulation_view=getter)
        with patch.dict(sys.modules,{'isaacsim.core.simulation_manager':manager}):
            with self.assertRaisesRegex(RuntimeError,'unavailable'):
                self.Reader(None,{},self.config)
        getter.assert_called_once_with()

    def test_nonfinite_measured_angles_are_rejected(self):
        self.joints.get_dof_positions.return_value=np.array([[np.nan,2.]])
        with self.assertRaisesRegex(RuntimeError,'not finite'):
            self.reader().read()


if __name__=='__main__':unittest.main()
