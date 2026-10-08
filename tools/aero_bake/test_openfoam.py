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

if __name__=='__main__':unittest.main()
