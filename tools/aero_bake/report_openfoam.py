"""Summarize exploratory OpenFOAM forces without producing a runtime map."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

NUMBER=r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?'
def read_forces(path):
    rows=[]
    for line in Path(path).read_text().splitlines():
        if not line.strip() or line.lstrip().startswith('#'): continue
        values=[float(v) for v in re.findall(NUMBER,line)]
        if len(values)!=13 or not all(math.isfinite(v) for v in values):
            raise ValueError('expected time plus pressure/viscous force and moment vectors')
        rows.append([values[0],*[values[1+i]+values[4+i] for i in range(3)],
                     *[values[7+i]+values[10+i] for i in range(3)]])
    if len(rows)<2 or any(a[0]>=b[0] for a,b in zip(rows,rows[1:])):
        raise ValueError('at least two strictly increasing samples required')
    return rows

def report(case,output,window=50):
    case=Path(case)
    manifest=json.loads((case/'pilot_manifest.json').read_text())
    mesh=(case/'log.checkMesh').read_text()
    if 'Mesh OK.' not in mesh or 'Failed ' in mesh:
        raise ValueError('mesh validation did not pass')
    sources=sorted((case/'postProcessing/loads').glob('*/forces.dat'),key=lambda p:float(p.parent.name))
    if not sources: raise ValueError('no force sampling segment')
    source=sources[-1] # do not merge overlapping restart segments
    rows=read_forces(source)
    if window<2 or len(rows)<2*window: raise ValueError('two full averaging windows required')
    recent,previous=rows[-window:],rows[-2*window:-window]
    mean=[statistics.mean(r[i] for r in recent) for i in range(1,7)]
    delta=[mean[i-1]-statistics.mean(r[i] for r in previous) for i in range(1,7)]
    solver_log=next(case/name for name in ('log.foamRun.parallel','log.foamRun.loads','log.foamRun') if (case/name).exists())
    result={'format':'rg.openfoam-pilot-results/1','validated':False,'runtime_map_eligible':False,
            'pilot':manifest,'force_file_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'force_segment_start':float(source.parent.name),
            'solver_log_sha256':hashlib.sha256(solver_log.read_bytes()).hexdigest(),
            'case_dictionary_sha256':{str(p.relative_to(case)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((case/'system').glob('*')) if p.is_file()},
            'samples':len(rows),'averaging_window':window,'sample_interval':[recent[0][0],recent[-1][0]],
            'mean_body_loads_SI':dict(zip(('fx_n','fy_n','fz_n','mx_nm','my_nm','mz_nm'),mean)),
            'previous_window_delta_SI':delta,
            'window_standard_deviation_SI':[statistics.pstdev(r[i] for r in recent) for i in range(1,7)],
            'solver_completed':'End' in solver_log.read_text().splitlines()[-3:],
            'note':'Exploratory '+('rotating-wall' if manifest.get('wheel_regions') else 'fixed-wheel')+'/fixed-wing-pose mesh and solver check; no coefficient certification.'}
    Path(output).write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case');p.add_argument('output');p.add_argument('--window',type=int,default=50)
    a=p.parse_args();print(json.dumps(report(a.case,a.output,a.window),indent=2))
