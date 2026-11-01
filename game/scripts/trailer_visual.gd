extends Node3D
# game/scripts/trailer_visual.gd - the towed trailer (rg/trailer.h): shows data/models/<kind>/<kind>.glb at the pose
# RgSimulation.get_trailer_transform() gives (render-interpolated like the chassis, floating-origin relative), its
# wheels spinning with the simulated per-wheel spin angle. No game logic: every number comes from get_trailer_state().
#
# Frames: the glb is ISO-consistent like the car models (gltf(x,y,z) = iso(y,z,x), see vehicle_visual.gd), so the model
# root gets the same axis permutation and sits at TrailerKind::model_offset (ISO, in the trailer vehicle's frame - the
# glb origin is the ground plane below the axle midpoint, the vehicle origin is the chassis centre). The wheel nodes are
# named wheel_<corner> after the vehicle file's wheel names ("1L" -> wheel_1L); their spin axis is the model's local X.
#
# The node hides while no trailer exists and rebuilds when the serial changes (a new trailer was spawned). Needs the
# `simulation` reference set before add_child (the pattern npc_truck.gd and body_visuals.gd use).

var simulation: Node

var _serial: int = -1
var _kind: String = ""
var _model_root: Node3D
var _wheels: Dictionary = {} # vehicle wheel name -> model node
var _model_ok: bool = false

func _ready() -> void:
	visible = false
	set_process(true)

# True once a trailer model is shown (the scripted tests wait for it).
func model_loaded() -> bool:
	return _model_ok and visible

func _model_path(kind: String) -> String:
	var root: String = ProjectSettings.globalize_path("res://")
	return root.path_join("../external/physics_sim/data/models").path_join(kind).path_join(kind + ".glb").simplify_path()

func _clear() -> void:
	if _model_root != null:
		_model_root.queue_free()
	_model_root = null
	_wheels.clear()
	_model_ok = false

func _build(state: Dictionary) -> void:
	_clear()
	_serial = int(state.get("serial", -1))
	_kind = str(state.get("kind", ""))
	var path := _model_path(_kind)
	if not FileAccess.file_exists(path):
		push_warning("trailer_visual.gd: model not found at '%s'" % path)
		return
	var doc := GLTFDocument.new()
	var gltf_state := GLTFState.new()
	if doc.append_from_file(path, gltf_state) != OK:
		push_warning("trailer_visual.gd: could not read '%s'" % path)
		return
	var scene := doc.generate_scene(gltf_state)
	if scene == null or not (scene is Node3D):
		push_warning("trailer_visual.gd: no scene in '%s'" % path)
		return
	_model_root = scene
	add_child(_model_root)
	# gltf-local -> ISO-local: the same pure axis permutation vehicle_visual.gd applies (determinant +1).
	_model_root.transform.basis = Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0))
	_model_root.position = state.get("model_offset", Vector3.ZERO)
	for w in state.get("wheels", []):
		var wheel_name := str(w.get("name", ""))
		var node := _model_root.find_child("wheel_" + wheel_name, true, false)
		if node is Node3D:
			_wheels[wheel_name] = node
	_model_ok = true

func _process(_delta: float) -> void:
	if simulation == null or not simulation.has_method("get_trailer_state"):
		visible = false
		return
	var state: Dictionary = simulation.get_trailer_state()
	if not bool(state.get("present", false)):
		if visible:
			visible = false
		if _serial != -1:
			_clear()
			_serial = -1
		return
	if int(state.get("serial", -1)) != _serial or str(state.get("kind", "")) != _kind:
		_build(state)
	visible = _model_ok
	if not _model_ok:
		return
	global_transform = simulation.get_trailer_transform()
	for w in state.get("wheels", []):
		var node: Node3D = _wheels.get(str(w.get("name", "")))
		if node != null:
			node.rotation = Vector3(fmod(float(w.get("spin_angle", 0.0)), TAU), 0.0, 0.0)
