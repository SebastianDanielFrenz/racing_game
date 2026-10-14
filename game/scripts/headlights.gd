extends Node
# Headlamps that light the road: one SpotLight3D per headlamp cluster (left /
# right), placed at the front face of the car rig's "head" lamp materials
# (<model>.rig.json lights.head.materials; no lamp sockets exist in the rigs),
# aimed forward and slightly down. On/off and dipped/main beam follow the
# shared DriverActions projection ("head_lamp", "main_beam" <- cabin.lights).
# Beam numbers: data/world/headlights.json.
#
# Cost (gl_compatibility): every visible SpotLight3D adds one more draw pass
# for each object inside its range (terrain chunks, road ribbons, the car);
# Godot's per-object limit is rendering/limits/opengl/max_lights_per_object
# (default 8). Shadows off by default. "lights_per_car": 1 merges both lamps
# into one spot at their midpoint (half the extra passes).

const CONFIG_PATH := "res://../data/world/headlights.json"
var simulation: Node
var vehicle_name: String
var spots: Array[SpotLight3D] = []
var config: Dictionary = {}
var _level := -1

# model: the glTF root (model space: +Z forward, +X left, +Y up).
static func attach_to(model: Node3D, sim: Node, car: String, rig: Dictionary) -> Node:
	var driver = load("res://scripts/headlights.gd").new()
	driver.name = "Headlights"
	driver.simulation = sim
	driver.vehicle_name = car
	var cfg = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path(CONFIG_PATH)))
	driver.config = cfg if cfg is Dictionary else {}
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
	model.add_child(driver)
	return driver

# Front-face centres of the lamp geometry, one per side (x > 0 left, x < 0
# right), in model space. Empty when the rig names no head lamp material.
static func lamp_clusters(model: Node3D, materials: Array) -> Array:
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
		var front := -INF
		for p in points:
			sum += p
			front = max(front, p.z)
		var centre: Vector3 = sum / points.size()
		centre.z = front + 0.02 # just ahead of the lens, outside the body
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
		spot.light_energy = float(beam.get("energy", 4.0))
		spot.basis = Basis.looking_at(direction, Vector3.UP)

func _process(_delta: float) -> void:
	if simulation == null or spots.is_empty() or not simulation.has_method("get_control"):
		return
	set_level(int(round(simulation.get_control("cabin.lights"))))
