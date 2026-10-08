import tempfile
from pathlib import Path
import unittest
from report_openfoam import read_forces

class ForceParsing(unittest.TestCase):
    def parse(self,text):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'forces.dat';p.write_text(text);return read_forces(p)
    def test_pressure_and_viscous_vectors_are_summed_in_body_axes(self):
        line='((-10 2 3) (-1 -2 4)) ((5 6 7) (1 2 -3))'
        rows=self.parse('# header\n0 '+line+'\n1 '+line+'\n')
        self.assertEqual(rows[0],[0,-11,0,7,6,8,4])
    def test_missing_vector_is_rejected(self):
        with self.assertRaises(ValueError):self.parse('0 ((1 2 3))\n1 ((1 2 3))')
    def test_duplicate_sample_times_are_rejected(self):
        line='0 ((1 2 3) (0 0 0)) ((0 0 0) (0 0 0))\n'
        with self.assertRaises(ValueError):self.parse(line+line)


class WheelBoundaryTests(unittest.TestCase):
    def test_rotation_bottom_velocity_matches_moving_ground(self):
        from openfoam_case import wheel_velocity_boundary
        row={'name':'wheel_FL','radius_m':.35,'origin_iso_m':[1,.8,-.2],'axis_iso':[0,1,0]}
        text=wheel_velocity_boundary(row,50)
        self.assertIn('axis (0 1 0)',text)
        omega=float(text.split('omega ')[1].split(';')[0])
        self.assertAlmostEqual(-omega*row['radius_m'],-50)
        row['radius_m']=0
        with self.assertRaises(ValueError):wheel_velocity_boundary(row,50)
    def test_nested_dictionary_replacement_preserves_other_controls(self):
        from openfoam_case import replace_block
        text='geometry\n{ car { file "a.stl"; } box { type box; } }\ncontrols { limit 4; }'
        result=replace_block(text,'geometry','wheel { file "wheel.stl"; }')
        self.assertNotIn('a.stl',result)
        self.assertIn('controls { limit 4; }',result)
class WallResolutionTests(unittest.TestCase):
    def test_latest_snapshot_preserves_patch_statistics(self):
        from report_openfoam import read_yplus
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'yPlus.dat';p.write_text('# header\n0 body 1 20 10\n100 body 30 90 45\n100 wheel_FL 20 70 40\n')
            result=read_yplus(p)
            self.assertEqual(result['time'],100)
            self.assertEqual(result['patches']['wheel_FL']['average'],40)
            self.assertFalse(result['validated'])
    def test_impossible_wall_statistics_are_rejected(self):
        from report_openfoam import read_yplus
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'yPlus.dat';p.write_text('100 body 10 20 30\n')
            with self.assertRaises(ValueError):read_yplus(p)

class AchievedLayerTests(unittest.TestCase):
    def test_achieved_zero_layers_are_not_requested_layers(self):
        from report_openfoam import read_layer_summary
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'log';p.write_text('overall thickness\nbody 100 0 0 0\nwheel_FL 20 1.5 .002 50\nLayer mesh : cells:500')
            result=read_layer_summary(p)
            self.assertEqual(result['body']['average_layers'],0)
            self.assertEqual(result['wheel_FL']['average_thickness_m'],.002)
    def test_absent_layer_stage_is_explicit(self):
        from report_openfoam import read_layer_summary
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'log';p.write_text('no layer stage\nEnd')
            self.assertIsNone(read_layer_summary(p))

if __name__=='__main__':unittest.main()
