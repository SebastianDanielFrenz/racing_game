extends Node
# Headlamps that light the road: one SpotLight3D per headlamp cluster (left /
# right), placed at the front face of the car rig's "head" lamp materials
# (<model>.rig.json lights.head.materials; no lamp sockets exist in the rigs),
# aimed forward and slightly down. On/off and dipped/main beam follow the
# shared DriverActions projection ("head_lamp", "main_beam" <- cabin.lights).
# Beam numbers: data/world/headlights.json (photometric: candela; on screen x the scene exposure,
# light_exposure.gd, so the beams light the road at night and are negligible in daylight). Rear: one red SpotLight3D at the midpoint of the tail/brake lamp
# materials (dim for the tail lamps, stronger while braking) and one white one at the reverse lamps, aimed
# backward and down; their on/off states are pushed by exterior_lamps.gd (set_states).
#
# Cost (gl_compatibility): every visible SpotLight3D adds one more draw pass
# for each object inside its range (terrain chunks, road ribbons, the car);
# Godot's per-object limit is rendering/limits/opengl/max_lights_per_object
# (default 8). Shadows off by default. "lights_per_car": 1 merges both lamps
# into one spot at their midpoint (half the extra passes).

const CONFIG_PATH := "res://../data/world/headlights.json"
const LightExposure := preload("res://scripts/light_exposure.gd")
var simulation: Node
var vehicle_name: String
var spots: Array[SpotLight3D] = []
var config: Dictionary = {}
var rear_spots := {} # "tail" / "reverse" -> SpotLight3D
var _level := -1
var _states := {}
var _rear_key := ""
var _exposure := -1.0 # exposure the energies were last set for

static func load_config() -> Dictionary:
	var cfg = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path(CONFIG_PATH)))
	return cfg if cfg is Dictionary else {}

# model: the glTF root (model space: +Z forward, +X left, +Y up).
static func attach_to(model: Node3D, sim: Node, car: String, rig: Dictionary) -> Node:
	var driver = load("res://scripts/headlights.gd").new()
	driver.name = "Headlights"
	driver.simulation = sim
	driver.vehicle_name = car
	driver.config = load_config()
	driver._exposure = LightExposure.current(driver.config)
	var materials: Array = rig.get("lights", {}).get("head", {}).get("materials", [])
	var clusters: Array = lamp_clusters(model, materials)
	if int(driver.config.get("lights_per_car", 2)) == 1 and clusters.size() == 2:
		clusters = [(clusters[0] + clusters[1]) * 0.5]
	for centre in clusters:
		var spot := SpotLight3D.new()
		spot.name = "HeadlampSpot"
		spot.position = centre
		spot.shadow_enabled = bool(driver.config.get("shadows", false))
		var rgb: Array = driver.config.get("colour", [1.0, 0.95, 0.85])
		spot.light_color = Color(rgb[0], rgb[1], rgb[2])
		spot.visible = false
		model.add_child(spot)
		driver.spots.append(spot)
	for kind in ["tail", "reverse"]:
		var spec: Dictionary = driver.config.get(kind, {})
		var lamp_kind := "brake" if kind == "tail" else "reverse"
		var rear_materials: Array = rig.get("lights", {}).get(lamp_kind, {}).get("materials", [])
		if kind == "tail":
			for m in rig.get("lights", {}).get("tail", {}).get("materials", []):
				if not m in rear_materials:
					rear_materials.append(m)
		var rear: Array = lamp_clusters(model, rear_materials, true)
		if spec.is_empty() or rear.is_empty():
			continue
		var centre := Vector3.ZERO
		for c in rear:
			centre += c
		var rear_spot := SpotLight3D.new()
		rear_spot.name = "RearLampSpot_" + kind
		rear_spot.position = centre / rear.size()
		rear_spot.shadow_enabled = false
		var rear_rgb: Array = spec.get("colour", [1.0, 0.1, 0.05])
		rear_spot.light_color = Color(rear_rgb[0], rear_rgb[1], rear_rgb[2])
		rear_spot.spot_range = float(spec.get("range_m", 12.0))
		rear_spot.spot_angle = float(spec.get("angle_deg", 60.0))
		rear_spot.spot_angle_attenuation = float(spec.get("angle_attenuation", 1.0))
		rear_spot.spot_attenuation = float(spec.get("attenuation", 1.0))
		var pitch := deg_to_rad(float(spec.get("pitch_down_deg", 15.0)))
		rear_spot.basis = Basis.looking_at(Vector3(0.0, -sin(pitch), -cos(pitch)), Vector3.UP)
		rear_spot.visible = false
		model.add_child(rear_spot)
		driver.rear_spots[kind] = rear_spot
	model.add_child(driver)
	return driver

# Front-face centres of the lamp geometry, one per side (x > 0 left, x < 0
# right), in model space. Empty when the rig names no head lamp material.
static func lamp_clusters(model: Node3D, materials: Array, rear := false) -> Array:
	var sides := {1: [], -1: []}
	var inverse := model.global_transform.affine_inverse() if model.is_inside_tree() else Transform3D.IDENTITY
	for node in model.find_children("*", "MeshInstance3D", true, false):
		var mi := node as MeshInstance3D
		if mi.mesh == null:
			continue
		var to_model: Transform3D = _relative(model, mi) if not model.is_inside_tree() else inverse * mi.global_transform
		for surface in range(mi.mesh.get_surface_count()):
			var material = mi.get_active_material(surface)
			if material == null or not (material.resource_name in materials):
				continue
			var arrays: Array = mi.mesh.surface_get_arrays(surface)
			var vertices: PackedVector3Array = arrays[Mesh.ARRAY_VERTEX] if arrays.size() > Mesh.ARRAY_VERTEX and arrays[Mesh.ARRAY_VERTEX] != null else PackedVector3Array()
			for v in vertices:
				var p: Vector3 = to_model * v
				sides[1 if p.x >= 0.0 else -1].append(p)
	var out := []
	for side in [1, -1]:
		var points: Array = sides[side]
		if points.is_empty():
			continue
		var sum := Vector3.ZERO
		var front := INF if rear else -INF
		for p in points:
			sum += p
			front = min(front, p.z) if rear else max(front, p.z)
		var centre: Vector3 = sum / points.size()
		centre.z = front + (-0.02 if rear else 0.02) # just outside the lens, beyond the body
		out.append(centre)
	return out

static func _relative(root: Node3D, node: Node3D) -> Transform3D:
	var t := Transform3D.IDENTITY
	var n: Node = node
	while n != null and n != root:
		if n is Node3D:
			t = (n as Node3D).transform * t
		n = n.get_parent()
	return t

func set_level(level: int) -> void:
	if level == _level:
		return
	_level = level
	var beam: Dictionary = config.get("main_beam" if level >= 2 else "dipped", {})
	var pitch := deg_to_rad(float(beam.get("pitch_down_deg", 1.5)))
	var direction := Vector3(0.0, -sin(pitch), cos(pitch))
	for spot in spots:
		spot.visible = level > 0
		spot.spot_range = float(beam.get("range_m", 60.0))
		spot.spot_angle = float(beam.get("angle_deg", 30.0))
		spot.spot_angle_attenuation = float(beam.get("angle_attenuation", 1.0))
		spot.spot_attenuation = float(beam.get("attenuation", 1.0))
		spot.light_energy = LightExposure.spot_energy(float(beam.get("intensity_cd", 0.0)), _exposure, spot.spot_attenuation, float(beam.get("reference_m", 1.0)))
		spot.basis = Basis.looking_at(direction, Vector3.UP)

# states: lamp action -> bool ("tail_lamp", "brake_lamp", "reverse_lamp"), from exterior_lamps.gd.
func set_states(states: Dictionary) -> void:
	if states == _states:
		return # unchanged (called every frame): no light parameter pushed to the RenderingServer
	_states = states
	_update_rear()

func _update_rear() -> void:
	var tail = rear_spots.get("tail")
	if tail != null:
		var spec: Dictionary = config.get("tail", {})
		var braking: bool = bool(_states.get("brake_lamp", false))
		tail.visible = braking or bool(_states.get("tail_lamp", false))
		tail.light_energy = LightExposure.spot_energy(float(spec.get("brake_intensity_cd" if braking else "intensity_cd", 0.0)), _exposure, tail.spot_attenuation, float(spec.get("reference_m", 1.0)))
	var reverse = rear_spots.get("reverse")
	if reverse != null:
		reverse.visible = bool(_states.get("reverse_lamp", false))
		reverse.light_energy = LightExposure.spot_energy(float(config.get("reverse", {}).get("intensity_cd", 0.0)), _exposure, reverse.spot_attenuation, float(config.get("reverse", {}).get("reference_m", 1.0)))

func _process(_delta: float) -> void:
	var e := LightExposure.current(config)
	if absf(e - _exposure) > 0.01 * _exposure: # the scene exposure moved (time of day, weather): rescale the beams
		_exposure = e
		var level := _level
		_level = -1
		if level >= 0:
			set_level(level)
		_update_rear()
	if simulation == null or spots.is_empty() or not simulation.has_method("get_control"):
		return
	set_level(int(round(simulation.get_control("cabin.lights"))))
