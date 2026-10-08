import unittest
import hashlib
import json
from pathlib import Path
import tempfile

try:
    import numpy as np
    import trimesh
    from prepare_solid import fill_exterior, prepare
except ImportError:
    np=None

@unittest.skipIf(np is None,'solid preparation development dependencies not installed')
class SolidFillTests(unittest.TestCase):
    def shell(self):
        box=np.zeros((9,9,9),dtype=bool);box[2:7,2:7,2:7]=True;box[3:6,3:6,3:6]=False
        return box
    def test_closed_volume_fills(self):
        filled,pad=fill_exterior(self.shell(),0)
        self.assertTrue(filled[4+pad,4+pad,4+pad])
        self.assertEqual(filled.sum(),125)
    def test_intentionally_open_volume_is_not_blindly_filled(self):
        box=self.shell();box[2,3:6,3:6]=False
        filled,pad=fill_exterior(box,0)
        self.assertFalse(filled[4+pad,4+pad,4+pad])
    def test_single_voxel_seam_is_sealed(self):
        box=self.shell();box[2,4,4]=False
        filled,pad=fill_exterior(box,1)
        self.assertTrue(filled[4+pad,4+pad,4+pad])
        with self.assertRaises(ValueError):fill_exterior(box,-1)
    def test_prepared_box_has_valid_geometry_and_hashed_output(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);source=root/'source';source.mkdir()
            obj=source/'hypercar_rest_iso.obj'
            # Synthetic closed-body fixture, never supplied as hypercar/CFD evidence.
            obj.write_bytes(('o car_hyper_primitive_0\n'+trimesh.creation.box(extents=[.2,.2,.2]).export(file_type='obj')).encode())
            audit={'flow_selection_sha256':'synthetic test fixture','frame':'ISO chassis body origin',
                   'obj_sha256':hashlib.sha256(obj.read_bytes()).hexdigest()}
            (source/'audit.json').write_text(json.dumps(audit))
            result=prepare(source,root/'solid',pitch=.02)
            self.assertTrue(result['quality']['watertight'])
            self.assertTrue(result['quality']['winding_consistent'])
            self.assertGreater(result['quality']['volume_m3'],.001)
            self.assertFalse(result['cfd_validated'])
            self.assertEqual(result['stl_sha256'],hashlib.sha256((root/'solid'/'hypercar_cfd_rest.stl').read_bytes()).hexdigest())
            # Cache remains bound to identical input bytes and yields identical STL.
            cached=prepare(source,root/'cached',pitch=.02)
            self.assertEqual(cached['stl_sha256'],result['stl_sha256'])

@unittest.skipIf(np is None,'solid preparation development dependencies not installed')
class WheelClearanceIdentityTests(unittest.TestCase):
    def test_wrong_vehicle_cannot_define_clearance_geometry(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);vehicle=root/'vehicle.json';vehicle.write_text('{}')
            (root/'audit.json').write_text(json.dumps({'vehicle_sha256':'other vehicle'}))
            with self.assertRaisesRegex(ValueError,'vehicle identity mismatch'):
                prepare(root,root/'output',vehicle=vehicle,core_mode='wheel_clearance')
            self.assertFalse((root/'output').exists())
    def test_clearance_mode_requires_vehicle(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'audit.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'requires source vehicle'):
                prepare(root,root/'output',core_mode='wheel_clearance')

if __name__=='__main__':unittest.main()
