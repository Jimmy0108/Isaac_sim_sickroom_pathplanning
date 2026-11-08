"""Offline tests for source loading without running the Isaac entry point."""
import ast
import os
from pathlib import Path
import py_compile
import sys
import tempfile
import types
import unittest


class SourceLoaderTests(unittest.TestCase):
    def setUp(self):
        source=Path(__file__).with_name('AMR_SLAM_RPP.py').read_text(encoding='utf-8')
        function=next(n for n in ast.parse(source).body
                      if isinstance(n,ast.FunctionDef) and n.name=='load_source_module')
        namespace={'Path':Path,'sys':sys}
        exec(compile(ast.Module(body=[function],type_ignores=[]),'<loader test>','exec'),namespace)
        self.load=namespace['load_source_module']
        self.name='_amr_source_loader_test'
        self.addCleanup(sys.modules.pop,self.name,None)

    def test_reads_saved_source_despite_stale_module_and_matching_pyc_timestamp(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            path=Path(directory)/'runtime.py'
            path.write_text('value = 1\n',encoding='utf-8')
            stamp=path.stat()
            py_compile.compile(str(path),doraise=True)
            stale=types.ModuleType(self.name)
            stale.value=1
            sys.modules[self.name]=stale
            path.write_text('value = 2\n',encoding='utf-8')
            os.utime(path,ns=(stamp.st_atime_ns,stamp.st_mtime_ns))
            current=self.load(self.name,path)
            self.assertEqual(current.value,2)
            self.assertIs(sys.modules[self.name],current)
            self.assertIsNot(current,stale)

    def test_failed_module_restores_previous_module(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as directory:
            path=Path(directory)/'runtime.py'
            path.write_text('raise RuntimeError("startup failed")\n',encoding='utf-8')
            previous=types.ModuleType(self.name)
            sys.modules[self.name]=previous
            with self.assertRaisesRegex(RuntimeError,'startup failed'):
                self.load(self.name,path)
            self.assertIs(sys.modules[self.name],previous)


if __name__=='__main__':unittest.main()
