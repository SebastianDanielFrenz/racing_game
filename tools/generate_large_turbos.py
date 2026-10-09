from pathlib import Path
import copy,json,math,sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'external/physics_sim/tools/turbo_sim'))
from meanline import Generator,atomic_json
base=json.loads((root/'external/physics_sim/tools/turbo_sim/standard_geometry.json').read_text())
for diameter in (100,120):
    scale=diameter/66
    profile=copy.deepcopy(base)
    profile['name']=f'garage_single_{diameter}mm_preliminary'
    for key in ('inducer_mm','exducer_mm','hub_mm','exit_width_mm'): profile['compressor'][key]*=scale
    profile['turbine']['diameter_mm']*=scale
    profile['turbine']['effective_throat_mm2']*=scale**2
    profile['provenance']['size_variant']=f'DERIVED: geometrically scaled by {scale}; {diameter} mm compressor inducer. Assumed internals, not manufacturer calibrated.'
    g=Generator(profile,root/'out/turbo_point_cache')
    n=[240000/scale*i/8 for i in range(9)]
    terminal=2*math.sqrt(2*g.exhaust.cp*1000*g.t.nozzle_efficiency)/(g.t.diameter_mm*.0005)*60/math.tau
    if terminal>n[-1]:n.append(terminal)
    w=[.9*scale**2*i/12 for i in range(13)]
    a=(g.exhaust.gamma-1)/g.exhaust.gamma
    chi=math.sqrt(1-40**(-a))
    er=[(1-(chi*i/8)**2)**(-1/a) for i in range(9)]
    result=g.adaptive_generate(n,w,er,max_passes=5,max_speed_lines=129,max_flow_points=257,workers=20)
    assert not result['sampling']['budget_limited']
    atomic_json(root/f'data/turbo_maps/single_{diameter}mm.json',result)
    atomic_json(root/f'data/turbo_maps/single_{diameter}mm_geometry.json',profile)
    for suffix in ('','_n2o'):
        hw=json.loads((root/f'data/turbos/garage_standard{suffix}.json').read_text())
        hw['name']=f'garage_single_{diameter}mm{suffix}'
        hw['rotor_inertia_kgm2']*=scale**5
        hw['reference_rpm']/=scale
        for key in ('max_mass_flow_kg_s','turbine_area_m2','wastegate_area_m2','shaft_loss_w_at_reference'):hw[key]*=scale**2
        hw['rotor_inertia_source']=f'Derived by geometric similarity from 66/82 mm rotor: inertia scales with diameter to the fifth power; scale={scale}. Assumed wheel mass distribution.'
        hw['performance_map']=f'../turbo_maps/single_{diameter}mm.json'
        atomic_json(root/f"data/turbos/{hw['name']}.json",hw)
        cfg=json.loads((root/f'data/turbo_configurations/hyper_single_standard{suffix}.json').read_text())
        cfg['name']=f'hyper_single_{diameter}mm{suffix}'
        cfg['turbo']=f"../turbos/{hw['name']}.json"
        atomic_json(root/f"data/turbo_configurations/{cfg['name']}.json",cfg)
    print(diameter,'mm:',result['sampling']['audit'],flush=True)
path=root/'data/vehicles/setup_options.json'
doc=json.loads(path.read_text())
for option in doc['options']:
    if option['id'] not in ('turbo_install','turbo_install_n2o'):continue
    suffix='_n2o' if option['id'].endswith('_n2o') else ''
    for diameter in (100,120):
        part_id=f'hyper_single_{diameter}mm{suffix}'
        if any(p['id']==part_id for p in option['parts']):continue
        hw=json.loads((root/f'data/turbos/garage_single_{diameter}mm{suffix}.json').read_text())
        option['parts'].append({'id':part_id,'label':f'Single turbo · {diameter}/{diameter*82/66:.1f} mm',
            'path':f'../turbo_configurations/{part_id}.json','image':'res://assets/components/engines/turbo_single.png',
            'detail':f"Compressor inducer/exducer · 1 unit · {hw['max_mass_flow_kg_s']:.2f} kg/s reference flow · I {hw['rotor_inertia_kgm2']:.5f} kg m²"})
path.write_text(json.dumps(doc,indent=2)+'\n')
