extends Node
# Material adapter: emission colours and strengths come from the supplied car rig.
var simulation: Node
var vehicle_name: String
var lamps: Array = []
const ACTIONS := {"indicator_left":"indicator_left","indicator_right":"indicator_right","brake":"brake_lamp","reverse":"reverse_lamp"}

static func attach_to(model: Node3D, sim: Node, car: String, path: String) -> Node:
	var driver = load("res://scripts/exterior_lamps.gd").new()
	driver.simulation = sim
	driver.vehicle_name = car
	var rig = JSON.parse_string(FileAccess.get_file_as_string(path.get_basename()+".rig.json"))
	if rig is Dictionary:
		for kind in ACTIONS:
			var spec: Dictionary = rig.get("lights",{}).get(kind,{})
			for node in model.find_children("*","MeshInstance3D",true,false):
				for surface in range(node.mesh.get_surface_count()):
					var material = node.get_active_material(surface)
					if material is StandardMaterial3D and material.resource_name in spec.get("materials",[]):
						var copy = material.duplicate()
						node.set_surface_override_material(surface,copy)
						driver.lamps.append({"material":copy,"action":ACTIONS[kind],"spec":spec})
	model.add_child(driver)
	driver.update_lamps({}, {})
	return driver

func update_lamps(snapshot: Dictionary, channels: Dictionary) -> void:
	for lamp in lamps:
		var state: Dictionary = RgDriverActions.state(lamp["action"],snapshot,channels)
		var on: bool = state["active"]
		var rgb: Array = lamp["spec"]["emissive_on"]
		var material: StandardMaterial3D = lamp["material"]
		material.emission_enabled = on
		material.emission = Color(rgb[0],rgb[1],rgb[2]) if on else Color.BLACK
		material.emission_energy_multiplier = float(lamp["spec"]["emissive_strength_on"]) if on else 0.0

func _process(_delta: float) -> void:
	if simulation != null:
		update_lamps(simulation.get_vehicle_powertrain(vehicle_name),load("res://scripts/driver_state.gd").channels(simulation) if simulation.has_method("get_control") else {})
