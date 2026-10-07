import hashlib
import csv
import json
import tempfile
import unittest
from pathlib import Path
from bake import LOADS, configuration, cases, convert, prepare

class BakeTests(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
  (self.root/"geometry.bin").write_bytes(b"synthetic test geometry, not CFD")
  self.config={"format":"rg.aero-bake/1","axes":{"yaw_deg":[-180,0,180],"pitch_deg":[-90,0,90],"speed_m_s":[20,40]},"reference_area_m2":2,"reference_length_m":3,"reference_point_local_m":[0,0,0],"geometry_files":["geometry.bin"],"solver":{"name":"synthetic-test","version":"1"},"provenance":{"kind":"synthetic_test"}}
  self.config_path=self.root/"config.json";self.results=self.root/"loads.csv";self.output=self.root/"map.json"
  self.write_config();self.write_results()
 def tearDown(self):self.temp.cleanup()
 def write_config(self):self.config_path.write_text(json.dumps(self.config))
 def write_results(self,missing=False,duplicate=False,seam=False,nan=False):
  config,names=configuration(self.config_path);all_cases=list(cases(config,names))
  if missing:all_cases.pop()
  if duplicate:all_cases.append(all_cases[0])
  with self.results.open("w",newline="") as file:
   writer=csv.writer(file);writer.writerow([*names,"density_kg_m3",*LOADS])
   for key in all_cases:
    scale=.5*1.2*key[2]**2*2
    loads=[-0.3*scale,0.1*scale,-0.5*scale,0.02*scale*3,-0.03*scale*3,0.04*scale*3]
    if seam and key[0]==180:loads[0]*=2
    if nan:loads[0]=float("nan")
    writer.writerow([*key,1.2,*loads])
 def test_prepare_manifest_and_direction(self):
  manifest=prepare(self.config_path,self.root/"cases");self.assertEqual(manifest["case_count"],18)
  self.assertEqual(manifest["status"],"prepared_not_solved")
  self.assertEqual(len(manifest["geometry"][0]["sha256"]),64)
 def test_force_moment_normalization(self):
  result=convert(self.config_path,self.results,self.output)
  for actual,expected in zip(result["coefficients"][0],[-.3,.1,-.5,.02,-.03,.04]):self.assertAlmostEqual(actual,expected)
  proof=json.loads(Path(str(self.output)+".provenance.json").read_text())
  self.assertFalse(proof["validated"])
  self.assertEqual(proof["map_sha256"],hashlib.sha256(self.output.read_bytes()).hexdigest())
 def test_missing_case(self):
  self.write_results(missing=True)
  with self.assertRaisesRegex(ValueError,"missing"):convert(self.config_path,self.results,self.output)
 def test_duplicate_case(self):
  self.write_results(duplicate=True)
  with self.assertRaisesRegex(ValueError,"duplicate"):convert(self.config_path,self.results,self.output)
 def test_periodic_seam(self):
  self.write_results(seam=True)
  with self.assertRaisesRegex(ValueError,"seam"):convert(self.config_path,self.results,self.output)
 def test_pitch_pole_mismatch(self):
  text=self.results.read_text();lines=text.splitlines();fields=lines[7].split(",");fields[5]=str(float(fields[5])+1);lines[7]=",".join(fields);self.results.write_text("\n".join(lines)+"\n")
  with self.assertRaisesRegex(ValueError,"pole"):convert(self.config_path,self.results,self.output)
 def test_nonfinite_load(self):
  self.write_results(nan=True)
  with self.assertRaisesRegex(ValueError,"finite"):convert(self.config_path,self.results,self.output)
 def test_no_solver_fabrication(self):
  self.config["solver"]["name"]="unselected";self.write_config()
  with self.assertRaisesRegex(ValueError,"actual solver"):convert(self.config_path,self.results,self.output)
if __name__=="__main__":unittest.main()
