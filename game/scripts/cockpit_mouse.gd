extends Node
# Mouse adapter of the mounted hyper cockpit controls (cockpit_common.gd). Thin on purpose: every decision (hit volumes,
# thresholds, travel, debounce, which action a control submits, availability) lives in rg::CockpitHit (RgCockpit,
# core/src/cockpit_hit.cpp, data/cockpits/car_hyper_common.json "control" blocks). This node only
#  - turns the viewport mouse ray into the car model's ISO frame (through the live model transform, so floating-origin
#    rebases and the seat position never enter) and tells the core the steering wheel angle,
#  - queues the core's events and hands them to the ONE shared RgDriverActions (source CockpitMouse) when main.gd calls
#    flush_to(); a paddle becomes a shift edge for consume_shift_up/down (the counters the keyboard also feeds),
#  - poses the pushed switch (travel from the core, axis/limit from the generated parts rig), swaps unavailable drive
#    modes and the EPB for the blank cap, and highlights the hovered control.
# Clicks count only while main.gd says the car is driven live, the world has input, the cursor is free (middle click
# toggles the capture, as for the other cockpits), no menu is open, no UI control is under the pointer and the camera sits
# within the layout's mouse.max_eye_distance_m of the driver eye.
const DriverState := preload("res://scripts/driver_state.gd")
const UiScale := preload("res://scripts/ui_scale.gd")
const BUTTON_PRIMARY := 0
const BUTTON_SECONDARY := 1
const HOVER_COLOUR := Color(1.0, 0.85, 0.3, 0.35)
const MENU_GROUPS := ["seat_adjustment_open", "traffic_settings_open", "address_teleport_open"]

var simulation: Node
var vehicle_name := ""
var driving_live := false     # set by main.gd every frame: the car is driven live (not paused, not unattended)
var world_input_active := true # set by main.gd every frame (input_map.is_world_input_active)
var wheel_angle_override := NAN # tests: steering wheel angle (rad) instead of the vehicle visual's
var pointer_override := Vector2(-1, -1) # tests: viewport PIXEL position used instead of the real mouse (x < 0 = real mouse)
var hit: RgCockpit
var _model: Node3D
var _parts: Dictionary = {}
var _blanks: Dictionary = {}
var _info: Array = []
var _joints: Array = []        # per control: {node, rest_position, rest_basis, axis, last}
var _blank_shown: Array = []
var _hover_boxes: Dictionary = {}
var _eye: Node3D
var _pending: Array = []
var _shift_up := 0
var _shift_down := 0
var _down := {}                # button -> true while the core got a pointer-down for it
var _max_eye_m := 0.0
var _margin_m := 0.0
var _hover := -1
var _ready_for_use := false

func _ready() -> void:
	# A duplicated model (garage reflection) carries this node without a setup(); it stays inert.
	set_process(false)
	set_process_input(false)
	set_process_unhandled_input(false)

# Loads the layout into the core and indexes the mounted parts. `parts`: mount id -> part Node3D, `blanks`: mount id -> blank cap clone.
func setup(layout_path: String, car_rig_path: String, model: Node3D, parts: Dictionary, blanks: Dictionary) -> bool:
	if not ClassDB.class_exists("RgCockpit"):
		return false
	hit = RgCockpit.new()
	var error: String = hit.load(layout_path, car_rig_path)
	if error != "":
		push_error("Cockpit mouse: " + error)
		hit = null
		return false
	_model = model
	_parts = parts
	_blanks = blanks
	var config: Dictionary = hit.mouse_config()
	_max_eye_m = float(config["max_eye_distance_m"])
	_margin_m = float(config["pick_margin_m"])
	_eye = model.find_child("socket_driver_eye", true, false) as Node3D
	for i in range(hit.get_control_count()):
		var info: Dictionary = hit.control_info(i)
		_info.append(info)
		_blank_shown.append(false)
		var entry: Dictionary = {}
		var part: Node3D = _parts.get(info["id"])
		if part != null and bool(info["has_joint"]):
			var node := part.find_child(str(info["joint_node"]), true, false) as Node3D
			if node != null:
				var a: Vector3 = info["joint_axis"]
				entry = {"node": node, "rest_position": node.position, "rest_basis": node.basis, "axis": Vector3(a.y, a.z, a.x).normalized(), "last": -1.0}
		_joints.append(entry)
	_ready_for_use = true
	set_process(true)
	set_process_input(true)
	set_process_unhandled_input(true)
	return true

# ---- frames ------------------------------------------------------------------------------------------------------

# glTF model-local (the model root's own axes) -> the core's ISO model frame.
static func gltf_to_iso(v: Vector3) -> Vector3:
	return Vector3(v.z, v.x, v.y)

static func iso_to_gltf(v: Vector3) -> Vector3:
	return Vector3(v.y, v.z, v.x)

func world_ray_to_model(origin: Vector3, direction: Vector3) -> Array:
	var inverse := _model.global_transform.affine_inverse()
	return [gltf_to_iso(inverse * origin), gltf_to_iso(inverse.basis * direction).normalized()]

func model_point_to_world(iso_point: Vector3) -> Vector3:
	return _model.global_transform * iso_to_gltf(iso_point)

func _wheel_angle() -> float:
	if not is_nan(wheel_angle_override):
		return wheel_angle_override
	var visual := _model.get_parent() if _model != null else null
	if visual != null and visual.has_method("get_steering_wheel_angle_rad"):
		return float(visual.get_steering_wheel_angle_rad())
	return 0.0

func _menu_open() -> bool:
	var tree := get_tree()
	if tree == null:
		return true
	for group in MENU_GROUPS:
		if tree.get_first_node_in_group(group) != null:
			return true
	return false

func _allowed(camera: Camera3D) -> bool:
	if not _ready_for_use or not driving_live or not world_input_active or camera == null:
		return false
	if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED or _menu_open():
		return false
	if _eye == null or camera.global_position.distance_to(_eye.global_position) > _max_eye_m:
		return false
	var viewport := get_viewport()
	if viewport.has_method("gui_get_hovered_control") and viewport.gui_get_hovered_control() != null:
		return false
	return true

# `at` in the camera's viewport pixels (UiScale.canvas_to_viewport_pixels for a real mouse position).
func _mouse_ray(camera: Camera3D, at: Vector2) -> Array:
	return world_ray_to_model(camera.project_ray_origin(at), camera.project_ray_normal(at))

func _pointer_pixels(at_canvas: Vector2) -> Vector2:
	return UiScale.canvas_to_viewport_pixels(get_viewport(), at_canvas)

# ---- input -------------------------------------------------------------------------------------------------------

func _unhandled_input(event: InputEvent) -> void:
	if not (event is InputEventMouseButton) or not event.pressed or not _ready_for_use:
		return
	var button := -1
	if event.button_index == MOUSE_BUTTON_LEFT:
		button = BUTTON_PRIMARY
	elif event.button_index == MOUSE_BUTTON_RIGHT:
		button = BUTTON_SECONDARY
	if button < 0:
		return
	var camera := get_viewport().get_camera_3d()
	if not _allowed(camera):
		return
	var ray := _mouse_ray(camera, _pointer_pixels(event.position))
	if hit.pointer_down(ray[0], ray[1], button) >= 0:
		_down[button] = true
		get_viewport().set_input_as_handled()

# Release anywhere (also outside the control or over a UI element) lets go.
func _input(event: InputEvent) -> void:
	if not (event is InputEventMouseButton) or event.pressed or not _ready_for_use:
		return
	var button := BUTTON_PRIMARY if event.button_index == MOUSE_BUTTON_LEFT else (BUTTON_SECONDARY if event.button_index == MOUSE_BUTTON_RIGHT else -1)
	if button >= 0 and _down.get(button, false):
		_down[button] = false
		hit.pointer_up(button)

# Tests and tools drive the core directly with a world ray.
func click_ray(world_origin: Vector3, world_direction: Vector3, button: int = BUTTON_PRIMARY) -> int:
	var ray := world_ray_to_model(world_origin, world_direction)
	var index: int = hit.pointer_down(ray[0], ray[1], button)
	if index >= 0:
		_down[button] = true
	return index

func release_button(button: int = BUTTON_PRIMARY) -> void:
	if _down.get(button, false):
		_down[button] = false
		hit.pointer_up(button)

# ---- per frame ---------------------------------------------------------------------------------------------------

# ---- diagnostics (RG_COCKPIT_PROBE=1) ------------------------------------------------------------------------------
var _probe_t := 0.0
var _probe_i := 0

# Why a click would not count right now ("" = it would). Mirrors _allowed().
func block_reason(camera: Camera3D, at: Vector2) -> String:
	if not _ready_for_use: return "not ready"
	if not driving_live: return "driving_live false"
	if not world_input_active: return "world input inactive"
	if camera == null: return "no camera"
	if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED: return "mouse captured"
	if _menu_open(): return "menu open"
	if _eye == null: return "no driver eye"
	var d := camera.global_position.distance_to(_eye.global_position)
	if d > _max_eye_m: return "camera %.2f m from the eye" % d
	var under := controls_under(at)
	if not under.is_empty(): return "UI under pointer: " + ", ".join(under)
	return ""

# Every visible Control with a mouse filter that is not IGNORE whose rect holds `at` (what would eat a click there).
func controls_under(at: Vector2) -> Array:
	var out: Array = []
	_collect_controls(get_tree().root, at, out)
	return out

func _collect_controls(node: Node, at: Vector2, out: Array) -> void:
	if node is CanvasItem and not (node as CanvasItem).is_visible_in_tree():
		return
	if node is Control:
		var c := node as Control
		if c.mouse_filter != Control.MOUSE_FILTER_IGNORE and c.get_global_rect().has_point(at):
			out.append("%s(%s,%s)" % [c.get_path(), c.get_class(), "STOP" if c.mouse_filter == Control.MOUSE_FILTER_STOP else "PASS"])
	for child in node.get_children():
		_collect_controls(child, at, out)

var _probe_at := Vector2.ZERO
var _probe_id := ""
var _probe_armed := false

# Every 2 s: aim at the next control and free the cursor (what the middle click does); 0.3 s later click it through
# Godot's input pipeline (GUI first, then _unhandled_input) and log what happened.
func _probe(delta: float) -> void:
	_probe_t += delta
	var main_node := get_tree().current_scene
	if main_node != null and main_node.get("_drive_view") != null:
		main_node.set("_drive_view", "cockpit")
	var camera := get_viewport().get_camera_3d()
	if _probe_armed and _probe_t >= 0.3:
		_probe_armed = false
		# parse_input_event takes WINDOW pixels (an OS click) and hands _unhandled_input canvas units, like a real click.
		var reason := block_reason(camera, _probe_at / get_tree().root.content_scale_factor)
		var ev := InputEventMouseButton.new()
		ev.button_index = MOUSE_BUTTON_LEFT
		ev.pressed = true
		ev.position = _probe_at
		ev.global_position = _probe_at
		Input.parse_input_event(ev)
		Input.flush_buffered_events()
		var reached := bool(_down.get(BUTTON_PRIMARY, false))
		var up := ev.duplicate() as InputEventMouseButton
		up.pressed = false
		Input.parse_input_event(up)
		Input.flush_buffered_events()
		print("COCKPIT_PROBE %s at %s block='%s' click_reached_core=%s hover=%s capture=%d" % [_probe_id, _probe_at, reason, reached, hovered_id(), Input.mouse_mode])
		return
	if _probe_t < 2.0 or camera == null or _info.is_empty():
		return
	_probe_t = 0.0
	var info: Dictionary = _info[_probe_i % _info.size()]
	_probe_i += 1
	var part: Node3D = _parts.get(info["id"])
	if part == null:
		return
	_probe_id = str(info["id"])
	_probe_at = camera.unproject_position(part.global_position)
	pointer_override = _probe_at
	Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	_probe_armed = true

func _process(delta: float) -> void:
	if not _ready_for_use:
		return
	if OS.get_environment("RG_COCKPIT_PROBE") == "1":
		_probe(delta)
	var camera := get_viewport().get_camera_3d()
	var allowed := _allowed(camera)
	hit.set_enabled(allowed)
	if not allowed:
		for button in _down.keys():
			_down[button] = false
	hit.set_wheel_angle(_wheel_angle())
	if allowed:
		var ray := _mouse_ray(camera, _pointer_pixels(get_viewport().get_mouse_position()) if pointer_override.x < 0.0 else pointer_override)
		hit.pointer_move(ray[0], ray[1])
	var snapshot: Dictionary = {}
	var channels: Dictionary = {}
	if simulation != null and simulation.has_method("get_vehicle_powertrain"):
		snapshot = simulation.get_vehicle_powertrain(vehicle_name)
		if simulation.has_method("get_control"):
			channels = DriverState.channels(simulation)
	for event in hit.update(delta, snapshot, channels):
		_pending.append(event)
	_pose()

func _pose() -> void:
	var hover: int = hit.get_hover()
	for i in range(_info.size()):
		var info: Dictionary = _info[i]
		var state: Dictionary = hit.control_state(i)
		var available := bool(state["available"])
		if bool(info["blank_when_unavailable"]):
			var blank := not available
			if blank != _blank_shown[i]:
				_blank_shown[i] = blank
				var part: Node3D = _parts.get(info["id"])
				var cap: Node3D = _blanks.get(info["id"])
				if part != null: part.visible = not blank
				if cap != null: cap.visible = blank
		var joint: Dictionary = _joints[i]
		if not joint.is_empty():
			var travel := float(state["travel"])
			if travel != float(joint["last"]):
				joint["last"] = travel
				var value := travel * float(info["joint_max"])
				var node: Node3D = joint["node"]
				if bool(info["joint_revolute"]):
					node.basis = Basis(joint["axis"], value) * (joint["rest_basis"] as Basis)
				else:
					node.position = (joint["rest_position"] as Vector3) + (joint["axis"] as Vector3) * value
	if hover != _hover:
		if _hover_boxes.has(_hover):
			(_hover_boxes[_hover] as Node3D).visible = false
		_hover = hover
		if hover >= 0 and bool(hit.control_state(hover)["available"]):
			_hover_box(hover).visible = true

func _hover_box(index: int) -> MeshInstance3D:
	if _hover_boxes.has(index):
		return _hover_boxes[index]
	var info: Dictionary = _info[index]
	var part: Node3D = _parts.get(info["id"])
	var holder: Node3D = null
	if part != null:
		holder = part.find_child("hit_" + str(info["part"]), true, false) as Node3D
	var box := MeshInstance3D.new()
	box.name = "HoverHighlight"
	var mesh := BoxMesh.new()
	var half: Vector3 = info["hit_half"]
	mesh.size = (iso_to_gltf(half) + Vector3.ONE * _margin_m) * 2.0
	box.mesh = mesh
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	material.albedo_color = HOVER_COLOUR
	material.no_depth_test = true
	material.render_priority = 10
	box.material_override = material
	box.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	box.visible = false
	(holder if holder != null else (part if part != null else self)).add_child(box)
	_hover_boxes[index] = box
	return box

# ---- hand-over to main.gd ----------------------------------------------------------------------------------------

# Submits the queued events (and the held levels, the horn) to the shared RgDriverActions just before it applies.
func flush_to(actions: Object) -> void:
	for event in _pending:
		if int(event["shift"]) > 0:
			_shift_up += 1
		elif int(event["shift"]) < 0:
			_shift_down += 1
		else:
			actions.submit(str(event["action_id"]), int(event["kind"]), float(event["value"]), int(event["source"]))
	_pending.clear()
	if hit != null:
		for held in hit.held_actions():
			actions.submit(str(held["action_id"]), int(held["kind"]), float(held["value"]), int(held["source"]))

# Events that never reach the car (paused, unattended) are dropped.
func discard_pending() -> void:
	_pending.clear()
	_shift_up = 0
	_shift_down = 0

func consume_shift_up() -> int:
	var n := _shift_up
	_shift_up = 0
	return n

func consume_shift_down() -> int:
	var n := _shift_down
	_shift_down = 0
	return n

func is_ready_for_use() -> bool:
	return _ready_for_use

func hovered_id() -> String:
	return str(_info[_hover]["id"]) if _hover >= 0 and _hover < _info.size() else ""
