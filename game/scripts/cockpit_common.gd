extends Node3D
const DriverState := preload("res://scripts/driver_state.gd") # preloaded once: this is called several times per frame
# Visual-only GLB extraction and data-driven mounts. No driver/physics logic.
const LAYOUT := "res://../data/cockpits/car_hyper_common.json"
var simulation: Node
var vehicle_name := ""
var model_path := ""
var _mounted: Array[Node3D] = []
var _layout: Dictionary = {}
var _materials: Dictionary = {}
var _lamp_specs: Dictionary = {}
var _poses: Array = [] # {node, rest (Basis), axis (glTF), angles, channel}: controls posed from a control channel

static func attach_to(model: Node3D, sim: Node = null, car: String = "", source_path: String = "") -> Node3D:
	var existing := model.get_node_or_null("CommonCockpit")
	if existing != null: return existing
	var script := load("res://scripts/cockpit_common.gd")
	var node: Node3D = script.new()
	node.name = "CommonCockpit"
	node.set("simulation",sim)
	node.set("vehicle_name",car)
	node.set("model_path",source_path)
	model.add_child(node)
	return node

static func iso(v: Array) -> Vector3:
	return Vector3(float(v[1]),float(v[2]),float(v[0]))

static func mount_basis(normal: Array, up: Array) -> Basis:
	var z := iso(normal).normalized()
	var x := iso(up)
	x = (x-z*x.dot(z)).normalized()
	var y := z.cross(x).normalized()
	# Part glTF axes are ISO Y,Z,X, not ISO X,Y,Z.
	return Basis(y,z,x)

func _ready() -> void:
	var model := get_parent() as Node3D
	# Garage reflection clones already contain the mounted trees.
	if model.has_meta("common_cockpit_mounted"):
		set_process(false)
		return
	var path := ProjectSettings.globalize_path(LAYOUT)
	var layout_value = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not layout_value is Dictionary:
		push_error("Common cockpit: invalid layout %s" % path)
		return
	_layout = layout_value
	var base := path.get_base_dir()
	var rig: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(base.path_join(_layout["rig"])))
	for spec in rig.get("lights",{}).values():
		for material in spec["materials"]: _lamp_specs[material] = spec
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_file(base.path_join(_layout["model"]),state) != OK:
		push_error("Common cockpit: cannot load parts GLB")
		return
	var library := doc.generate_scene(state)
	if library == null: return
	# Read engine-neutral socket frames from the rig; GLTF metadata forwarding
	# is importer-dependent. Explicit node metadata can override it in tests/scenes.
	var socket_frames: Dictionary = {}
	var car_rig_path := model_path.get_basename()+".rig.json"
	if model_path != "" and FileAccess.file_exists(car_rig_path):
		var car_rig_value = JSON.parse_string(FileAccess.get_file_as_string(car_rig_path))
		if car_rig_value is Dictionary:
			for frame in car_rig_value.get("sockets",[]): socket_frames[frame["name"]] = frame
	for mount in _layout["mounts"]:
		var source := library.find_child("part_"+str(mount["part"]),true,false) as Node3D
		if source == null:
			push_error("Common cockpit: missing part %s" % mount["part"])
			continue
		var socket := model.find_child(str(mount["socket"]),true,false) as Node3D
		var parent: Node3D = socket if socket != null else model.find_child(str(mount["parent"]),true,false) as Node3D
		if parent == null and mount["parent"] == "car_hyper": parent = model
		if parent == null:
			push_error("Common cockpit: missing mount parent %s" % mount["parent"])
			continue
		var part := source.duplicate() as Node3D
		part.name = "cockpit_"+str(mount["id"])
		var normal: Array = mount["normal"]
		var up: Array = mount["up"]
		if socket != null:
			var frame: Dictionary = socket_frames.get(mount["socket"],{})
			normal = frame.get("normal",normal)
			up = frame.get("up",up)
			normal = socket.get_meta("normal",normal)
			up = socket.get_meta("up",up)
		part.transform = Transform3D(mount_basis(normal,up),Vector3.ZERO if socket != null else iso(mount["position"]))
		parent.add_child(part)
		_mounted.append(part)
		_copy_lamp_materials(part)
	library.free()
	for rule in _layout.get("poses",[]):
		for part in _mounted:
			var node := part.find_child(str(rule["node"]),true,false) as Node3D
			if node == null: continue
			_poses.append({"node":node,"rest":node.transform.basis,"axis":iso(rule["axis"]).normalized(),"angles":rule["angles_rad"],"channel":str(rule["channel"])})
	model.set_meta("common_cockpit_mounted",true)
	print("RG_COCKPIT_COMMON mounted=%d" % _mounted.size())
	set_process(simulation != null)

func _copy_lamp_materials(root: Node) -> void:
	var stack: Array[Node] = [root]
	while not stack.is_empty():
		var node := stack.pop_back() as Node
		for child in node.get_children(): stack.append(child)
		if not node is MeshInstance3D: continue
		var mesh_node := node as MeshInstance3D
		for surface in range(mesh_node.mesh.get_surface_count()):
			var material := mesh_node.get_active_material(surface) as StandardMaterial3D
			if material == null or not _lamp_specs.has(material.resource_name): continue
			var copy := material.duplicate() as StandardMaterial3D
			mesh_node.set_surface_override_material(surface,copy)
			if not _materials.has(copy.resource_name): _materials[copy.resource_name] = []
			_materials[copy.resource_name].append(copy)

var _lamp_applied: Dictionary = {} # lamp material name -> last applied on/off

func update_lamps(snapshot: Dictionary, channels: Dictionary = {}) -> void:
	var states: Dictionary = {}
	for rule in _layout.get("lamps",[]):
		var name: String = rule["material"]
		var on := false
		if rule.has("action"):
			var action: String = rule["action"]
			if not states.has(action): states[action] = RgDriverActions.state(action,snapshot,channels)
			on = (int(states[action]["lamp_segments"]) & (1 << int(rule["segment"]))) != 0
		else:
			# New driver functions arrive in C1; the existing start ring still
			# reads the published engine state, with no new game behaviour.
			var field: String = rule["field"]
			on = snapshot.has(field) and snapshot.get(field) == rule["equals"]
		if not _lamp_specs.has(name): continue
		# Material parameters are pushed to the RenderingServer only on a change: a setter call re-uploads the
		# material's uniforms even with the same value, every frame for every lamp (perf, 2026-10-15).
		if _lamp_applied.get(name, null) == on: continue
		_lamp_applied[name] = on
		var spec: Dictionary = _lamp_specs[name]
		var rgb: Array = spec["emissive_on"]
		for material in _materials.get(name,[]):
			material.emission_enabled = on
			material.emission = Color(float(rgb[0]),float(rgb[1]),float(rgb[2])) if on else Color.BLACK
			material.emission_energy_multiplier = float(spec["emissive_strength_on"]) if on else 0.0

const FrameProbe := preload("res://scripts/frame_probe.gd")
func _process(_delta: float) -> void:
	var probe_t0 := Time.get_ticks_usec()
	_process_body(_delta)
	FrameProbe.add("cockpit_common", Time.get_ticks_usec() - probe_t0)
func _process_body(_delta: float) -> void:
	if simulation != null and simulation.has_method("get_vehicle_powertrain"):
		var channels: Dictionary = DriverState.channels(simulation) if simulation.has_method("get_control") else {}
		update_lamps(simulation.get_vehicle_powertrain(vehicle_name),channels)
	if simulation != null and simulation.has_method("get_control"):
		update_poses(simulation)

# Poses the switches listed under "poses" from their control channel (the shared DriverActions state: the O key and
# any cockpit switch move the same channel). Visual only.
func update_poses(sim: Object) -> void:
	for p in _poses:
		var angles: Array = p["angles"]
		var index := clampi(int(round(float(sim.get_control(p["channel"])))),0,angles.size()-1)
		(p["node"] as Node3D).transform.basis = (p["rest"] as Basis) * Basis(p["axis"],float(angles[index]))

func pose_angle(node_name: String) -> float:
	for p in _poses:
		if (p["node"] as Node).name == node_name:
			var n := p["node"] as Node3D
			return ((p["rest"] as Basis).inverse() * n.transform.basis).get_euler().length()
	return NAN

func mounted_count() -> int:
	return _mounted.size()
