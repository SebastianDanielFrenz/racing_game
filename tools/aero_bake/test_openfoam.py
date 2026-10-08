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
if __name__=='__main__':unittest.main()
