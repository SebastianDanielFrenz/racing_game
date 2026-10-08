import json
from pathlib import Path
import tempfile
import unittest
from reuse_surface_mesh import reuse

class ReuseSafety(unittest.TestCase):
    def test_geometry_mismatch_rejected_before_copy(self):
        with tempfile.TemporaryDirectory() as folder:
            a=Path(folder)/'base';b=Path(folder)/'case';a.mkdir();b.mkdir()
            (a/'pilot_manifest.json').write_text(json.dumps({'stl_sha256':'old'}))
            (b/'pilot_manifest.json').write_text(json.dumps({'stl_sha256':'new'}))
            with self.assertRaisesRegex(ValueError,'base mesh differs'):reuse(a,b)
            self.assertFalse((b/'constant/polyMesh').exists())
    def test_already_layered_mesh_cannot_be_layered_again(self):
        with tempfile.TemporaryDirectory() as folder:
            a=Path(folder)/'base';b=Path(folder)/'case';a.mkdir();b.mkdir()
            for p in (a,b):(p/'pilot_manifest.json').write_text('{}')
            (a/'log.checkMesh').write_text('Mesh OK.')
            (a/'log.snappyHexMesh').write_text('overall thickness\nbody 100 1.2 .01 40\nEnd')
            with self.assertRaisesRegex(ValueError,'zero achieved prism layers'):reuse(a,b)
            self.assertFalse((b/'constant/polyMesh').exists())

if __name__=='__main__':unittest.main()
