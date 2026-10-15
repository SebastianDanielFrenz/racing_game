extends Node
const DriverState := preload("res://scripts/driver_state.gd") # preloaded once: this is called several times per frame
# Material adapter: emission colours come from the supplied car rig, the lens luminance per lamp
# kind from data/world/headlights.json (cd/m2, constant), shown x the scene exposure
# (light_exposure.gd): lit lenses glow at night and fade into daylight.
const LightExposure := preload("res://scripts/light_exposure.gd")
var simulation: Node
var vehicle_name: String
var lamps: Array = []
var lights: Node # headlights.gd driver: gets the lamp states for the rear light sources
var config: Dictionary = {} # data/world/headlights.json
const ACTIONS := {"indicator_left":"indicator_left","indicator_right":"indicator_right","brake":"brake_lamp","reverse":"reverse_lamp","head":"head_lamp","tail":"tail_lamp"}

static func attach_to(model: Node3D, sim: Node, car: String, path: String) -> Node:
	var driver = load("res://scripts/exterior_lamps.gd").new()
	driver.simulation = sim
	driver.vehicle_name = car
	driver.config = load("res://scripts/headlights.gd").load_config()
	var rig = JSON.parse_string(FileAccess.get_file_as_string(path.get_basename()+".rig.json"))
	if rig is Dictionary:
		# One material copy per surface; a material named by several kinds (tail and brake
		# often share one lens) lists them all and shows the strongest active one.
		var by_surface := {}
		for node in model.find_children("*","MeshInstance3D",true,false):
			for surface in range(node.mesh.get_surface_count()):
				var material = node.get_active_material(surface)
				if not material is StandardMaterial3D:
					continue
				for kind in ACTIONS:
					var spec: Dictionary = rig.get("lights",{}).get(kind,{})
					if not material.resource_name in spec.get("materials",[]):
						continue
					var key := str(node.get_instance_id()) + ":" + str(surface)
					if not by_surface.has(key):
						var copy = material.duplicate()
						node.set_surface_override_material(surface,copy)
						by_surface[key] = {"material":copy,"kinds":[]}
						driver.lamps.append(by_surface[key])
					by_surface[key]["kinds"].append({"action":ACTIONS[kind],"spec":spec,"nits":driver.lens_nits(kind,spec)})
		driver.lights = load("res://scripts/headlights.gd").attach_to(model,sim,car,rig)
	model.add_child(driver)
	driver.update_lamps({}, {})
	return driver

# Lens luminance (cd/m2) of a rig light kind: headlights.json lens_luminance_nits; a kind it
# does not list keeps the rig's own strength as its display value at the night exposure.
func lens_nits(kind: String, spec: Dictionary) -> float:
	var table: Dictionary = config.get("lens_luminance_nits", {})
	if table.has(kind):
		return float(table[kind])
	return float(spec.get("emissive_strength_on", 1.0)) / float(config.get("fallback_exposure", 0.0942))

func update_lamps(snapshot: Dictionary, channels: Dictionary) -> void:
	var states := {}
	var e := LightExposure.current(config)
	for lamp in lamps:
		var best: Dictionary = {}
		for k in lamp["kinds"]:
			var action: String = k["action"]
			if not states.has(action):
				states[action] = RgDriverActions.state(action,snapshot,channels)["active"]
			if states[action] and (best.is_empty() or float(k["nits"]) > float(best["nits"])):
				best = k
		var material: StandardMaterial3D = lamp["material"]
		material.emission_enabled = not best.is_empty()
		if best.is_empty():
			material.emission = Color.BLACK
			material.emission_energy_multiplier = 0.0
		else:
			var rgb: Array = best["spec"]["emissive_on"]
			material.emission = Color(rgb[0],rgb[1],rgb[2])
			material.emission_energy_multiplier = LightExposure.lens_energy(float(best["nits"]), e, material.emission)
	if lights != null:
		lights.set_states(states)

const FrameProbe := preload("res://scripts/frame_probe.gd")
func _process(_delta: float) -> void:
	var probe_t0 := Time.get_ticks_usec()
	_process_body(_delta)
	FrameProbe.add("exterior_lamps", Time.get_ticks_usec() - probe_t0)
func _process_body(_delta: float) -> void:
	if simulation != null:
		update_lamps(simulation.get_vehicle_powertrain(vehicle_name),DriverState.channels(simulation) if simulation.has_method("get_control") else {})
