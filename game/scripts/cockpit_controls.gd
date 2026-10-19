extends Node3D
# Physical cockpit controls of a cargen car (data/models/<car>/<car>.cockpit.json, rg.cockpit/1).
#
# Attached by vehicle_visual.gd under the loaded model root whenever a "<model>.cockpit.json" sits next to the
# .glb. Two jobs, both presentation and input mapping only (the physics never sees this node):
#  - every frame, pose the cockpit nodes from the live car: needles from speed/rpm/fuel/temperature, the
#    steering wheel, pedals and handbrake from the APPLIED control channels, the gear lever through its gate
#    from the selected gear, knobs/switches/stalks from their own state;
#  - make them clickable: a left click on a control acts on it (a switch toggles, a knob/stalk/slider steps,
#    a button presses, the gear lever shifts up), shift + click or the mouse wheel steps back / shifts down,
#    and pedals, levers and the steering wheel are dragged. Clicks only count while the active camera sits in
#    the car (near socket_driver_eye) and the mouse is not captured for looking around.
#
# Control channels that reach the physics (steer, throttle, brake, clutch, handbrake, gear shifts, ignition,
# starter) are not written here: main.gd's _forward_driving pulls them (get_overrides, consume_shift_up/down,
# consume_ignition_request, starter_held) so the one place that sets controls stays the one place.
# Cabin switches that DriverActions owns (hazard, indicator stalk, horn, light switch) submit ActionEvents through `driver_actions`
# (main.gd sets it to the shared instance; source CockpitMouse) and are POSED from the shared projection
# (DriverActions::state + the cabin.* channels), so a key press and a click show the same switch position. Without a
# driver_actions object (garage, pure-GDScript tests) they fall back to the local cosmetic state below.
# Everything else ("cosmetic:*" sources: fog, wipers, climate, radio, ...) is state kept here; get_cosmetic
# reads it (screens and lamps can follow it later).
#
# Frames: cockpit.json and rig.json are ISO 8855 (x forward, y left, z up); the .glb nodes use glTF axes,
# gltf(x, y, z) = iso(y, z, x). Every node's rest rotation is identity, so a joint value is a rotation about
# (revolute) or a shift along (prismatic) the node's own glTF-local axis.

var simulation: Node
const PowertrainFeed := preload("res://scripts/powertrain_feed.gd")
var _feed := PowertrainFeed.new("cockpit")
var _held_speed_kmh := 0.0
var vehicle_name: String = ""
var driver_actions: Object = null # RgDriverActions (submit/state), set by main.gd; null = local cosmetic state
var model_root: Node3D
var cockpit_path: String = ""
var rig_path: String = ""

const EYE_RADIUS_M := 0.9       # the camera must be this close to the driver eye for clicks to count
# RgDriverActions enum values (godot_ext/src/rg_driver_actions.h); literals so this script parses without the GDExtension
const KIND_RELEASE := 1
const KIND_PRESS := 0
const KIND_SET := 2
const SOURCE_COCKPIT_MOUSE := 3
const DRIVER_BACKED := ["hazard", "indicator", "horn", "lights"]   # channels with a DriverActions action
const PRESS_TIME_S := 0.18      # how long a pressed button stays down
const START_HOLD_S := 0.7       # an ignition key held in its start detent cranks this long, then springs back
const DRAG_PX_FULL := 220.0     # vertical mouse travel for a pedal/lever's full range
const WHEEL_PX_TURN := 600.0    # horizontal mouse travel for one steering wheel turn

var _controls: Array = []       # [{spec, node, axis, type, rest}]
var _by_node: Dictionary = {}   # node name -> entry
var _cosmetic: Dictionary = {}  # channel -> value
var _channel_values: Dictionary = {} # channel -> the values a stepped control offers (a toggle on it keeps them)
var _press_left: Dictionary = {} # node name -> seconds the button stays pressed
var _overrides: Dictionary = {} # control channel -> value while dragged
var _drag: Dictionary = {}      # {entry, start: Vector2, value0}
var _shift_up := 0
var _shift_down := 0
var _ignition_request := -1     # -1 none, 0 off, 1 on
var _starter_left := 0.0
var _horn_held := false         # the left mouse button is down on the horn: the level stays up until released
var _horn_left := 0.0           # seconds a clicked horn button keeps the horn level up
var _steer_lock_deg := 540.0    # steering wheel angle from centre to one lock, degrees (per-car, cockpit file; else cabin.json)
var _eye: Node3D
var _gear_pose := Vector2.ZERO  # smoothed (side, fore) of the gear lever
var _loaded := false
var _wheel_angle_rad := 0.0     # last posed steering wheel angle (rad), for vehicle_visual / diagnostics

func _ready() -> void:
	_loaded = load_spec()

func is_loaded() -> bool:
	return _loaded

# True when this node poses the named model node itself (the steering wheel of a car with a cockpit spec): vehicle_visual
# then leaves it alone, so the two never write the same transform in one frame.
func owns_node(node_name: String) -> bool:
	return _loaded and _by_node.has(node_name)

func get_steering_wheel_angle_rad() -> float:
	return _wheel_angle_rad

# ---- loading ----------------------------------------------------------------

func load_spec() -> bool:
	if model_root == null or cockpit_path == "" or not FileAccess.file_exists(cockpit_path):
		return false
	var ck: Variant = JSON.parse_string(FileAccess.get_file_as_string(cockpit_path))
	if not (ck is Dictionary) or str(ck.get("format", "")) != "rg.cockpit/1":
		push_warning("cockpit_controls.gd: %s is not an rg.cockpit/1 file" % cockpit_path)
		return false
	var joints := {}
	if rig_path != "" and FileAccess.file_exists(rig_path):
		var rig: Variant = JSON.parse_string(FileAccess.get_file_as_string(rig_path))
		if rig is Dictionary:
			for n in rig.get("nodes", []):
				if n.has("joint"):
					joints[str(n["name"])] = n["joint"]
	_eye = model_root.find_child("socket_driver_eye", true, false)
	_steer_lock_deg = float(ck.get("steering", {}).get("steering_wheel_lock_deg", _cabin_lock_deg()))
	_controls.clear()
	_by_node.clear()
	for spec in ck.get("controls", []):
		var node_name := str(spec.get("node", ""))
		var node: Node3D = model_root.find_child(node_name, true, false)
		var joint: Dictionary = joints.get(node_name, {})
		if node == null or joint.is_empty():
			continue
		var a: Array = joint.get("axis", [0, 0, 1])
		var entry := {
			"spec": spec,
			"node": node,
			"axis": iso_to_gltf(Vector3(float(a[0]), float(a[1]), float(a[2]))).normalized(),
			"type": str(joint.get("type", "revolute")),
			"min": float(joint.get("min", -INF)),
			"max": float(joint.get("max", INF)),
			"rest": node.transform,
		}
		_controls.append(entry)
		_by_node[node_name] = entry
		var act: Dictionary = spec.get("action", {})
		if str(act.get("type", "")) == "step" and not act.get("values", []).is_empty():
			_channel_values[str(act.get("channel", ""))] = act["values"]
		var source := str(spec.get("source", ""))
		if source.begins_with("cosmetic:") and not _cosmetic.has(source.substr(9)):
			_cosmetic[source.substr(9)] = _first_value(spec)
	for channel in _channel_values:
		if not _channel_values[channel].has(_cosmetic.get(channel)):
			_cosmetic[channel] = _off_value(_channel_values[channel])
	return not _controls.is_empty()

# The default lock for a car whose cockpit file does not carry one (data/controls/cabin.json, same field and unit).
static func _cabin_lock_deg() -> float:
	var path := ProjectSettings.globalize_path("res://../data/controls/cabin.json")
	if FileAccess.file_exists(path):
		var cfg: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
		if cfg is Dictionary and cfg.has("steering_wheel_lock_deg"):
			return float(cfg["steering_wheel_lock_deg"])
	return 540.0

func get_steering_wheel_lock_deg() -> float:
	return _steer_lock_deg

static func iso_to_gltf(v: Vector3) -> Vector3:
	return Vector3(v.y, v.z, v.x)

func _first_value(spec: Dictionary) -> Variant:
	var values: Array = spec.get("action", {}).get("values", [])
	if not values.is_empty():
		return values[0] if not values.has("off") else "off"
	return 0.0

# ---- queries main.gd and tests use --------------------------------------------

func control_names() -> Array:
	return _by_node.keys()

func get_cosmetic(channel: String) -> Variant:
	var shared: Variant = _shared_value(channel)
	return shared if shared != null else _cosmetic.get(channel, null)

# The switch position from the shared DriverActions projection, or null when this channel is not driver-backed.
func _shared_value(channel: String) -> Variant:
	if driver_actions == null or simulation == null or not channel in DRIVER_BACKED:
		return null
	var channels := {}
	for key in ["cabin.hazard", "cabin.horn", "cabin.indicator", "cabin.indicator_left", "cabin.indicator_right", "cabin.lights"]:
		channels[key] = simulation.get_control(key)
	if channel == "lights": # 0 off, 1 dipped, 2 main beam: the switch shows on for either beam
		return float(channels.get("cabin.lights", 0.0)) != 0.0
	if channel == "indicator":
		var dir := float(channels.get("cabin.indicator", 0.0)) # + = left (ISO), - = right, 0 = off
		return "left" if dir > 0.0 else ("right" if dir < 0.0 else "off")
	return bool(driver_actions.state(channel, {}, channels).get("active", false))

func _submit(id: String, kind: int, value: float) -> void:
	driver_actions.submit(id, kind, value, SOURCE_COCKPIT_MOUSE)

# main.gd calls this every frame just before DriverActions.apply: the horn is a held level and the keyboard adapter
# submits its own level each frame, so a clicked horn button has to keep asserting its level after that.
func submit_held() -> void:
	if driver_actions != null and (_horn_held or _horn_left > 0.0):
		_submit("horn", KIND_SET, 1.0)

# A left click that landed on the horn keeps it sounding until release_horn().
func hold_horn() -> void:
	if driver_actions != null:
		_horn_held = true

func is_horn_held() -> bool:
	return _horn_held

func release_horn() -> void:
	if not _horn_held:
		return
	_horn_held = false
	_horn_left = 0.0
	for k in _press_left.keys():
		if str(_by_node.get(k, {}).get("spec", {}).get("action", {}).get("channel", "")) == "horn":
			_press_left.erase(k)
	if driver_actions != null:
		_submit("horn", KIND_RELEASE, 0.0)

func _exit_tree() -> void:
	release_horn()

func _notification(what: int) -> void:
	if what == NOTIFICATION_APPLICATION_FOCUS_OUT or what == NOTIFICATION_WM_WINDOW_FOCUS_OUT:
		release_horn()

# Mouse-up anywhere (also outside the hit box or over a UI control) lets go of the horn.
func _input(event: InputEvent) -> void:
	if _horn_held and event is InputEventMouseButton and event.button_index == MOUSE_BUTTON_LEFT and not event.pressed:
		release_horn()

# Channel -> value for every control currently dragged (and a hand throttle that is set).
func get_overrides() -> Dictionary:
	var out := _overrides.duplicate()
	var hand := float(_cosmetic.get("hand_throttle", 0.0))
	if hand > 0.0:
		out["throttle"] = maxf(hand, float(out.get("throttle", 0.0)))
	return out

func consume_shift_up() -> int:
	var n := _shift_up
	_shift_up = 0
	return n

func consume_shift_down() -> int:
	var n := _shift_down
	_shift_down = 0
	return n

func consume_ignition_request() -> int:
	var r := _ignition_request
	_ignition_request = -1
	return r

func starter_held() -> bool:
	return _starter_left > 0.0

# ---- per frame: pose the controls from the car --------------------------------

const FrameProbe := preload("res://scripts/frame_probe.gd")
func _process(delta: float) -> void:
	var probe_t0 := Time.get_ticks_usec()
	_process_body(delta)
	FrameProbe.add("cockpit_controls", Time.get_ticks_usec() - probe_t0)
func _process_body(delta: float) -> void:
	if not _loaded:
		return
	for k in _press_left.keys():
		_press_left[k] = float(_press_left[k]) - delta
		if float(_press_left[k]) <= 0.0:
			_press_left.erase(k)
	if _horn_held:
		var cam := get_viewport().get_camera_3d() if is_inside_tree() else null
		if Input.mouse_mode == Input.MOUSE_MODE_CAPTURED or (cam != null and not _in_car(cam)):
			release_horn() # cursor taken away or the view left the cockpit
		else:
			for k in _by_node.keys():
				if str(_by_node[k]["spec"].get("action", {}).get("channel", "")) == "horn":
					_press_left[k] = PRESS_TIME_S # the button stays down while held
	if _horn_left > 0.0:
		_horn_left -= delta
		if _horn_left <= 0.0 and driver_actions != null:
			_submit("horn", KIND_RELEASE, 0.0)
	if _starter_left > 0.0:
		_starter_left -= delta
		if _starter_left <= 0.0:
			_spring_back_ignition()
	var live := _live_values()
	for entry in _controls:
		_pose(entry, live, delta)

func _live_values() -> Dictionary:
	var v := {"speed_kmh": 0.0, "engine_rpm": 0.0, "fuel_fraction": 0.0, "coolant_c": 20.0, "oil_pressure_bar": 0.0,
			"battery_v": 12.4, "gear": 0}
	var t := Time.get_time_dict_from_system()
	v["clock_m"] = float(t["minute"]) + float(t["second"]) / 60.0
	v["clock_h"] = fmod(float(t["hour"]), 12.0) + float(v["clock_m"]) / 60.0
	if simulation == null or not simulation.has_method("get_vehicle_powertrain"):
		return v
	var misses_before: int = _feed.misses
	var pt: Dictionary = _feed.read(simulation, vehicle_name)
	var rpm := float(pt.get("rpm", 0.0))
	var running := rpm > 300.0
	if _feed.misses == misses_before:
		_held_speed_kmh = absf(float(simulation.get_vehicle_ground_speed_mps(vehicle_name))) * 3.6
	v["speed_kmh"] = _held_speed_kmh
	v["engine_rpm"] = rpm
	v["gear"] = int(pt.get("gear", 0))
	# Traction battery / motor for an EV or hybrid cockpit file (needles bind to "battery_soc" 0..1 and "motor_rpm"); 0 on an ICE car.
	v["battery_soc"] = float(pt.get("battery_percent", 0)) / 100.0
	v["motor_rpm"] = float(pt.get("traction_motor_rpm", 0.0))
	var info: Dictionary = simulation.get_vehicle_gauge_info(vehicle_name)
	var capacity := float(info.get("fuel_capacity_kg", 0.0))
	v["fuel_fraction"] = clampf(float(pt.get("fuel_mass_kg", 0.0)) / capacity, 0.0, 1.0) if capacity > 0.0 else 0.0
	var thermal: Dictionary = simulation.get_vehicle_thermal(vehicle_name)
	v["coolant_c"] = float(thermal.get("coolant_c", 90.0 if running else 20.0)) if bool(thermal.get("present", false)) else (90.0 if running else 20.0)
	# No oil pressure or charging model in the physics: a plausible reading from the engine speed (display only).
	v["oil_pressure_bar"] = (1.2 + 3.0 * clampf(rpm / 4000.0, 0.0, 1.0)) if running else 0.0
	v["battery_v"] = 13.9 if running else (12.4 if simulation.get_control("ignition") > 0.5 else 11.0)
	return v

func _source_value(spec: Dictionary, live: Dictionary) -> Variant:
	var source := str(spec.get("source", ""))
	if source.begins_with("cosmetic:"):
		return get_cosmetic(source.substr(9)) if _cosmetic.has(source.substr(9)) else 0.0
	if source.begins_with("control:"):
		var channel := source.substr(8)
		if _overrides.has(channel):
			return _overrides[channel]
		if channel == "ignition":
			return _cosmetic.get("ignition_detent", _ignition_detent_from_sim(spec))
		if channel == "hand_throttle":
			return _cosmetic.get("hand_throttle", 0.0)
		return simulation.get_control(channel) if simulation != null else 0.0
	return live.get(source, 0.0)

func _pose(entry: Dictionary, live: Dictionary, delta: float) -> void:
	var spec: Dictionary = entry["spec"]
	var kind := str(spec.get("kind", ""))
	var m: Dictionary = spec.get("map", {})
	var value: Variant = _source_value(spec, live)
	var j := 0.0
	match kind:
		"needle", "pedal", "lever":
			var i: Array = m.get("in", [0.0, 1.0])
			var o: Array = m.get("out_deg", [0.0, 0.0])
			var f := clampf(inverse_lerp(float(i[0]), float(i[1]), float(value)), 0.0, 1.0)
			j = deg_to_rad(lerpf(float(o[0]), float(o[1]), f))
		"knob":
			var detents: Array = m.get("detents", [0])
			var o: Array = m.get("out_deg", [0.0, 0.0])
			var idx := maxi(detents.find(value), 0)
			var f := float(idx) / float(maxi(detents.size() - 1, 1))
			j = deg_to_rad(lerpf(float(o[0]), float(o[1]), f))
		"stalk":
			var positions: Array = m.get("positions", [])
			var idx := maxi(positions.find(value), 0)
			var centre := positions.find("off")
			if centre < 0:
				centre = 0
			j = deg_to_rad(float(m.get("throw_deg", 10.0))) * float(idx - centre)
		"toggle":
			j = deg_to_rad(float(m.get("on_deg", 15.0)) if _truthy(value) else float(m.get("off_deg", -15.0)))
		"slider":
			var positions: Array = m.get("positions", [0])
			var idx := maxi(positions.find(value), 0)
			j = float(m.get("travel_m", 0.05)) * float(idx) / float(maxi(positions.size() - 1, 1))
		"button":
			j = float(m.get("pressed_m", 0.003)) if (_press_left.has(str(spec["node"])) or _truthy(value) and str(spec.get("action", {}).get("type", "")) == "toggle") else 0.0
		"steering_wheel":
			j = float(simulation.get_control("steer") if simulation != null and not _overrides.has("steer") else _overrides.get("steer", 0.0)) * deg_to_rad(_steer_lock_deg)
			_wheel_angle_rad = j
		"gear_lever":
			var gate: Dictionary = m.get("gate", {})
			var g := int(live.get("gear", 0))
			var key := "R" if g < 0 else ("N" if g == 0 else str(g))
			var target: Array = gate.get(key, gate.get("N", [0, 0]))
			_gear_pose = _gear_pose.lerp(Vector2(float(target[0]), float(target[1])), clampf(delta * 14.0, 0.0, 1.0))
			j = deg_to_rad(float(m.get("deg_per_unit_side", 8.0))) * _gear_pose.x
			var fa: Dictionary = _by_node.get(str(m.get("fa_node", "")), {})
			if not fa.is_empty():
				_apply(fa, deg_to_rad(float(m.get("deg_per_unit_fore", 8.0))) * _gear_pose.y)
		"lever_axis":
			return # posed by its gear lever
	_apply(entry, j)

func _apply(entry: Dictionary, j: float) -> void:
	j = clampf(j, float(entry["min"]), float(entry["max"]))
	var node: Node3D = entry["node"]
	var rest: Transform3D = entry["rest"]
	if str(entry["type"]) == "prismatic":
		node.transform = Transform3D(rest.basis, rest.origin + entry["axis"] * j)
	else:
		node.transform = Transform3D(Basis(entry["axis"], j) * rest.basis, rest.origin)

static func _off_value(values: Array) -> Variant:
	return "off" if values.has("off") else values[0]

static func _on_value(values: Array) -> Variant:
	for v in values:
		if _truthy(v):
			return v
	return values[-1]

static func _truthy(v: Variant) -> bool:
	if v is bool:
		return v
	if v is String:
		return v != "" and v != "off"
	return float(v) > 0.5

# ---- ignition ----------------------------------------------------------------

func _ignition_spec() -> Dictionary:
	for entry in _controls:
		if str(entry["spec"].get("source", "")) == "control:ignition":
			return entry["spec"]
	return {}

# The detent that shows the physics' ignition: 0 when off, else the run detent (the one before "start" on a
# key with four positions, the last one otherwise).
func _ignition_detent_from_sim(spec: Dictionary) -> Variant:
	var detents: Array = spec.get("map", {}).get("detents", [0, 1])
	var on: bool = simulation != null and simulation.get_control("ignition") > 0.5
	return detents[_run_index(detents)] if on else detents[0]

static func _run_index(detents: Array) -> int:
	return detents.size() - 2 if detents.size() >= 4 else detents.size() - 1

func _spring_back_ignition() -> void:
	var spec := _ignition_spec()
	if spec.is_empty():
		return
	var detents: Array = spec.get("map", {}).get("detents", [0, 1])
	_cosmetic["ignition_detent"] = detents[_run_index(detents)]

func _set_ignition_detent(spec: Dictionary, index: int) -> void:
	var detents: Array = spec.get("map", {}).get("detents", [0, 1])
	index = clampi(index, 0, detents.size() - 1)
	_cosmetic["ignition_detent"] = detents[index]
	_ignition_request = 0 if index == 0 else 1
	if detents.size() >= 4 and index == detents.size() - 1:
		_starter_left = START_HOLD_S

# ---- clicking -----------------------------------------------------------------

# False when the node or one of its ancestors up to the model root is hidden (independent of the scene tree).
func _shown(node: Node) -> bool:
	var n: Node = node
	while n != null:
		if n is Node3D and not (n as Node3D).visible:
			return false
		n = n.get_parent()
	return true

# The control a ray (global space) hits, or "". Hit boxes come from cockpit.json and move with their node. The
# nearest box wins, except that a smaller box whose entry point lies inside it wins over it: buttons sit inside
# the steering wheel's box and must stay clickable.
func pick(origin: Vector3, direction: Vector3) -> String:
	var hits: Array = [] # {t, volume, name, point (global entry point), box: [inverse transform, centre, half]}
	for entry in _controls:
		var h: Dictionary = entry["spec"].get("hit", {})
		if h.is_empty() or not _shown(entry["node"]):
			continue # a control the setup took off the car (rg_core strip_visual.h) cannot be clicked
		var c: Array = h.get("center", [0, 0, 0])
		var e: Array = h.get("half_extents", [0.02, 0.02, 0.02])
		var centre := iso_to_gltf(Vector3(float(c[0]), float(c[1]), float(c[2])))
		var half := iso_to_gltf(Vector3(float(e[0]), float(e[1]), float(e[2]))).abs()
		var inv: Transform3D = (entry["node"] as Node3D).global_transform.affine_inverse()
		var o := inv * origin
		var d := inv.basis * direction
		var t := _ray_box(o - centre, d, half)
		if t >= 0.0 and str(entry["spec"].get("kind", "")) == "steering_wheel" and not _on_wheel(o - centre, d, half):
			t = -1.0 # through a gap between the rim and the hub: the stalks and dash behind stay clickable
		if t >= 0.0:
			hits.append({"t": t, "volume": half.x * half.y * half.z, "name": str(entry["spec"]["node"]),
					"point": origin + direction * t, "inv": inv, "centre": centre, "half": half})
	if hits.is_empty():
		return ""
	hits.sort_custom(func(a, b): return a["t"] < b["t"])
	var best: Dictionary = hits[0]
	for h in hits:
		if float(h["volume"]) >= float(best["volume"]):
			continue
		var p: Vector3 = (best["inv"] as Transform3D) * (h["point"] as Vector3) - (best["centre"] as Vector3)
		var b: Vector3 = best["half"]
		if absf(p.x) <= b.x + 1e-4 and absf(p.y) <= b.y + 1e-4 and absf(p.z) <= b.z + 1e-4:
			best = h
	return str(best["name"])

# The steering wheel's box is a disc: the ray counts only where it crosses the wheel plane on the rim (outer 18 %)
# or the hub (inner 35 %). The plane is normal to the box's thinnest axis.
static func _on_wheel(o: Vector3, d: Vector3, half: Vector3) -> bool:
	var n := 0
	for i in 3:
		if half[i] < half[n]:
			n = i
	if absf(d[n]) < 1e-6:
		return true
	var p := o + d * (-o[n] / d[n])
	p[n] = 0.0
	var radius := maxf(half[(n + 1) % 3], half[(n + 2) % 3])
	var r := p.length()
	return r > 0.82 * radius or r < 0.35 * radius

static func _ray_box(o: Vector3, d: Vector3, half: Vector3) -> float:
	var tmin := -INF
	var tmax := INF
	for i in 3:
		if absf(d[i]) < 1e-9:
			if absf(o[i]) > half[i]:
				return -1.0
			continue
		var t1 := (-half[i] - o[i]) / d[i]
		var t2 := (half[i] - o[i]) / d[i]
		tmin = maxf(tmin, minf(t1, t2))
		tmax = minf(tmax, maxf(t1, t2))
	if tmax < maxf(tmin, 0.0):
		return -1.0
	return maxf(tmin, 0.0)

# Acts on a control: step = +1 (click / wheel up) or -1 (shift + click / wheel down). Returns false when the
# control is unknown or only dragged.
func activate(node_name: String, step: int = 1) -> bool:
	var entry: Dictionary = _by_node.get(node_name, {})
	if entry.is_empty():
		return false
	var spec: Dictionary = entry["spec"]
	var action: Dictionary = spec.get("action", {})
	var channel := str(action.get("channel", ""))
	match str(action.get("type", "")):
		"toggle":
			if driver_actions != null and channel == "lights":
				_submit(channel, KIND_PRESS, 1.0) # the same cycle as the lights key: auto -> on -> off -> auto
				return true
			if driver_actions != null and channel == "hazard":
				_submit(channel, KIND_SET, 0.0 if _truthy(get_cosmetic(channel)) else 1.0)
				return true
			var on := not _truthy(_cosmetic.get(channel, false))
			var values: Array = _channel_values.get(channel, [])
			if values.is_empty():
				_cosmetic[channel] = on
			else: # e.g. a wiper switch beside a wiper stalk: off, or the first speed
				_cosmetic[channel] = _on_value(values) if on else _off_value(values)
		"press":
			_press_left[node_name] = PRESS_TIME_S
			if driver_actions != null and channel == "horn":
				_horn_left = PRESS_TIME_S
				_submit("horn", KIND_SET, 1.0)
				return true
			_cosmetic[channel] = int(_cosmetic.get(channel, 0)) + 1 # a press count other code can watch
		"step":
			var values: Array = action.get("values", [])
			if values.is_empty():
				return false
			if channel == "ignition":
				var cur: Variant = _source_value(spec, {})
				_set_ignition_detent(spec, maxi(values.find(cur), 0) + step)
				return true
			if driver_actions != null and channel == "indicator":
				var target: Variant = values[clampi(values.find(get_cosmetic(channel)) + step, 0, values.size() - 1)]
				if target == "off":
					_submit("indicator_left", KIND_SET, 0.0)
				else:
					_submit("indicator_left" if target == "left" else "indicator_right", KIND_SET, 1.0)
				return true
			var idx := values.find(_cosmetic.get(channel, values[0]))
			_cosmetic[channel] = values[clampi(idx + step, 0, values.size() - 1)]
		"gear":
			if step > 0:
				_shift_up += 1
			else:
				_shift_down += 1
		"drag":
			# a click without a drag moves a lever one quarter of its range; pedals only follow a drag
			if channel in ["hand_throttle", "spark"]:
				var r: Array = action.get("range", [0.0, 1.0])
				_cosmetic[channel] = clampf(float(_cosmetic.get(channel, 0.0)) + 0.25 * float(step) * (float(r[1]) - float(r[0])), float(r[0]), float(r[1]))
				return true
			return false
		"touch":
			_cosmetic[str(action.get("screen", node_name)) + "_touches"] = int(_cosmetic.get(str(action.get("screen", node_name)) + "_touches", 0)) + 1
		_:
			return false
	return true

func begin_drag(node_name: String, at: Vector2) -> bool:
	var entry: Dictionary = _by_node.get(node_name, {})
	var action: Dictionary = entry.get("spec", {}).get("action", {}) if not entry.is_empty() else {}
	var type := str(action.get("type", ""))
	if type != "drag" and type != "rotate":
		return false
	var channel := str(action.get("channel", ""))
	var v0 := 0.0
	if channel in _cosmetic:
		v0 = float(_cosmetic[channel])
	elif simulation != null:
		v0 = simulation.get_control(channel)
	_drag = {"entry": entry, "start": at, "value0": v0}
	return true

func drag_to(at: Vector2) -> void:
	if _drag.is_empty():
		return
	var action: Dictionary = _drag["entry"]["spec"]["action"]
	var channel := str(action.get("channel", ""))
	var r: Array = action.get("range", [0.0, 1.0])
	var moved: Vector2 = at - _drag["start"]
	var v: float
	if str(action.get("type", "")) == "rotate":
		# dragging right turns the wheel right (steer is positive to the left)
		v = float(_drag["value0"]) - moved.x / (WHEEL_PX_TURN * _steer_lock_deg / 360.0)
	else:
		v = float(_drag["value0"]) + moved.y / DRAG_PX_FULL * (float(r[1]) - float(r[0]))
	v = clampf(v, float(r[0]), float(r[1]))
	if channel in ["hand_throttle", "spark"]:
		_cosmetic[channel] = v
	else:
		_overrides[channel] = v

func end_drag() -> void:
	if _drag.is_empty():
		return
	var action: Dictionary = _drag["entry"]["spec"]["action"]
	var channel := str(action.get("channel", ""))
	if action.has("release_to") or str(action.get("type", "")) == "rotate":
		_overrides.erase(channel) # pedals spring back, the wheel returns to the player's own steering
	elif _overrides.has(channel) and channel == "handbrake":
		pass # a ratchet: the handbrake stays where it was pulled until dragged again
	_drag = {}

func is_dragging() -> bool:
	return not _drag.is_empty()

# ---- mouse --------------------------------------------------------------------

func _in_car(camera: Camera3D) -> bool:
	if camera == null or _eye == null:
		return false
	return camera.global_position.distance_to(_eye.global_position) < EYE_RADIUS_M

func _unhandled_input(event: InputEvent) -> void:
	if not _loaded or Input.mouse_mode == Input.MOUSE_MODE_CAPTURED:
		return
	var camera := get_viewport().get_camera_3d()
	if event is InputEventMouseMotion and is_dragging():
		drag_to(event.position)
		get_viewport().set_input_as_handled()
		return
	if not (event is InputEventMouseButton):
		return
	var mb := event as InputEventMouseButton
	if mb.button_index == MOUSE_BUTTON_LEFT and not mb.pressed and is_dragging():
		end_drag()
		get_viewport().set_input_as_handled()
		return
	if not mb.pressed or not _in_car(camera):
		return
	var hit := pick(camera.project_ray_origin(mb.position), camera.project_ray_normal(mb.position))
	if hit == "":
		return
	var handled := false
	match mb.button_index:
		MOUSE_BUTTON_LEFT:
			handled = begin_drag(hit, mb.position)
			if not handled:
				handled = activate(hit, -1 if mb.shift_pressed else 1)
				if handled and str(_by_node.get(hit, {}).get("spec", {}).get("action", {}).get("channel", "")) == "horn":
					hold_horn()
		MOUSE_BUTTON_WHEEL_UP:
			handled = activate(hit, 1)
		MOUSE_BUTTON_WHEEL_DOWN:
			handled = activate(hit, -1)
	if handled:
		get_viewport().set_input_as_handled()
