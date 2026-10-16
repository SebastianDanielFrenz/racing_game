extends SceneTree
# Headless check of cockpit_controls.gd on every car with a <model>.cockpit.json (no GDExtension needed):
#   godot --headless --path game --script res://scripts/cockpit_test.gd
# Loads each model like vehicle_visual.gd does, drives the cockpit from a stand-in simulation and checks that
# every control with a hit box is picked by a ray at it, that clicks act (switch, knob, gear lever, ignition key,
# buttons) and that the steering wheel, a needle and a pedal follow the car. Prints RG_COCKPIT lines and
# "RG_COCKPIT_DONE checks=N failures=M"; exits 1 on any failure.

const MODELS := ["res://../data/models/pico/pico.glb", "res://../data/models/ventura_gt/ventura_gt.glb",
		"res://../data/models/runabout_t/runabout_t.glb"]

class FakeSim:
	extends Node
	var controls := {"steer": 0.0, "throttle": 0.0, "brake": 0.0, "clutch": 0.0, "handbrake": 0.0, "ignition": 1.0}
	var rpm := 0.0
	var gear := 0
	var speed_mps := 0.0
	func get_control(c: String) -> float:
		return float(controls.get(c, 0.0))
	func get_vehicle_powertrain(_n: String) -> Dictionary:
		return {"rpm": rpm, "gear": gear, "fuel_mass_kg": 20.0}
	func get_vehicle_ground_speed_mps(_n: String) -> float:
		return speed_mps
	func get_vehicle_gauge_info(_n: String) -> Dictionary:
		return {"fuel_capacity_kg": 40.0}
	func get_vehicle_thermal(_n: String) -> Dictionary:
		return {"present": true, "coolant_c": 90.0}

# Stand-in for RgDriverActions: records submitted events and applies them to the stand-in simulation's cabin.* channels
# (what DriverActions::apply/advance do), so the cockpit's posed switches read the "projected" state back.
class FakeActions:
	extends RefCounted
	var sim: Node
	var events: Array = []
	var light_mode := 0 # 0 auto (a daylit stand-in: lamps off), 1 on, 2 off - the rg::LightMode cycle
	func submit(id: String, kind: int, value: float, source: int) -> bool:
		events.append({"id": id, "kind": kind, "value": value, "source": source})
		match id:
			"hazard": sim.controls["cabin.hazard"] = 1.0 if value != 0.0 else 0.0
			"indicator_left": sim.controls["cabin.indicator"] = 1.0 if value != 0.0 else 0.0
			"indicator_right": sim.controls["cabin.indicator"] = -1.0 if value != 0.0 else 0.0
			"horn": sim.controls["cabin.horn"] = 1.0 if (kind == 2 and value != 0.0) else 0.0
			"lights":
				if kind == 0: # Press cycles auto -> on -> off -> auto
					light_mode = (light_mode + 1) % 3
					sim.controls["cabin.lights"] = 1.0 if light_mode == 1 else 0.0
				else:
					sim.controls["cabin.lights"] = value
		return true
	func state(id: String, _snapshot: Dictionary, channels: Dictionary) -> Dictionary:
		return {"active": float(channels.get("cabin." + id, 0.0)) != 0.0}

var checks := 0
var failures := 0

func check(ok: bool, what: String) -> void:
	checks += 1
	if not ok:
		failures += 1
		print("RG_COCKPIT FAIL ", what)

func _initialize() -> void:
	_run.call_deferred()

func _run() -> void:
	for path in MODELS:
		await _test_model(ProjectSettings.globalize_path(path).simplify_path())
	print("RG_COCKPIT_DONE checks=%d failures=%d" % [checks, failures])
	quit(1 if failures > 0 else 0)

func _angle_about(node: Node3D, rest: Basis, axis: Vector3) -> float:
	var q := (rest.inverse() * node.transform.basis).get_rotation_quaternion()
	var a := q.get_angle()
	return a if q.get_axis().dot(axis) >= 0.0 else -a

func _test_model(path: String) -> void:
	var name := path.get_file().get_basename()
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	check(doc.append_from_file(path, state) == OK, name + ": glb loads")
	var root: Node3D = doc.generate_scene(state)
	root.transform.basis = Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0)) # as vehicle_visual.gd
	var holder := Node3D.new()
	holder.position = Vector3(10, 0, -5) # not at the origin: the hit test must use global transforms
	holder.rotation.y = 0.7
	get_root().add_child(holder)
	holder.add_child(root)
	var sim := FakeSim.new()
	get_root().add_child(sim)
	var ck := Node3D.new()
	ck.set_script(load("res://scripts/cockpit_controls.gd"))
	ck.set("simulation", sim)
	ck.set("vehicle_name", name)
	ck.set("model_root", root)
	ck.set("cockpit_path", path.get_basename() + ".cockpit.json")
	ck.set("rig_path", path.get_basename() + ".rig.json")
	root.add_child(ck)
	await process_frame
	check(ck.has_method("is_loaded") and ck.is_loaded(), name + ": cockpit loaded")
	if not ck.has_method("is_loaded"):
		return
	var spec: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path.get_basename() + ".cockpit.json"))
	var eye: Node3D = root.find_child("socket_driver_eye", true, false)
	# every clickable control is picked by a ray at its hit box: from 12 cm in front of the box and from the eye
	var from_eye := 0
	var clickable := 0
	for c in spec["controls"]:
		if not c.has("hit"):
			continue
		clickable += 1
		var node: Node3D = root.find_child(str(c["node"]), true, false)
		var hc: Array = c["hit"]["center"]
		var centre: Vector3 = node.global_transform * Vector3(float(hc[1]), float(hc[2]), float(hc[0]))
		var toward_eye := (eye.global_position - centre).normalized()
		var got := ""
		var side := toward_eye.cross(Vector3.UP).normalized()
		for dir in [toward_eye, (toward_eye + Vector3.UP).normalized(), (toward_eye - Vector3.UP).normalized(),
				(toward_eye + side).normalized(), (toward_eye - side).normalized()]:
			got = ck.pick(centre + dir * 0.12, -dir)
			if got == str(c["node"]):
				break
		check(got == str(c["node"]), "%s: pick %s from 12 cm got '%s'" % [name, c["node"], got])
		var eye_got: String = ck.pick(eye.global_position, (centre - eye.global_position).normalized())
		if eye_got == str(c["node"]):
			from_eye += 1
		elif OS.get_environment("RG_COCKPIT_VERBOSE") != "":
			print("RG_COCKPIT %s: %s from the eye picks '%s'" % [name, c["node"], eye_got])
	print("RG_COCKPIT %s controls=%d clickable=%d picked_from_eye=%d" % [name, ck.control_names().size(), clickable, from_eye])
	check(ck.pick(eye.global_position, (eye.global_position - root.global_position).normalized()) == "", name + ": a ray away from the dash picks nothing")
	# clicks
	for c in spec["controls"]:
		var action: Dictionary = c.get("action", {})
		var ch := str(action.get("channel", ""))
		match str(action.get("type", "")):
			"toggle":
				var before: bool = ck._truthy(ck.get_cosmetic(ch))
				check(ck.activate(str(c["node"]), 1) and ck._truthy(ck.get_cosmetic(ch)) != before, "%s: %s toggles" % [name, c["node"]])
			"step":
				if ch == "ignition":
					continue
				var values: Array = action["values"]
				var start: Variant = ck.get_cosmetic(ch)
				var want: Variant = values[mini(values.find(start) + 1, values.size() - 1)]
				ck.activate(str(c["node"]), 1)
				check(ck.get_cosmetic(ch) == want, "%s: %s steps to %s (got %s)" % [name, c["node"], want, ck.get_cosmetic(ch)])
				ck.activate(str(c["node"]), -1)
				check(ck.get_cosmetic(ch) == start, "%s: %s steps back" % [name, c["node"]])
			"press":
				check(ck.activate(str(c["node"]), 1) and int(ck.get_cosmetic(ch)) >= 1, "%s: %s presses" % [name, c["node"]])
			"gear":
				ck.activate(str(c["node"]), 1)
				ck.activate(str(c["node"]), 1)
				ck.activate(str(c["node"]), -1)
				check(ck.consume_shift_up() == 2 and ck.consume_shift_down() == 1, "%s: gear lever shifts" % name)
	await _test_shared_switches(ck, sim, spec, name)
	# ignition key: off, then (four-position keys) start -> starter held, springs back to run
	var key := ""
	var detents: Array = []
	for c in spec["controls"]:
		if str(c.get("source", "")) == "control:ignition":
			key = str(c["node"])
			detents = c["map"]["detents"]
	if key != "":
		for _i in detents.size():
			ck.activate(key, -1)
		check(ck.consume_ignition_request() == 0, name + ": key to off asks for ignition off")
		for _i in detents.size():
			ck.activate(key, 1)
		check(ck.consume_ignition_request() == 1, name + ": key forward asks for ignition on")
		if detents.size() >= 4:
			check(ck.starter_held(), name + ": start detent cranks")
			await create_timer(1.0).timeout
			check(not ck.starter_held(), name + ": the key springs back from start")
	# a pedal drag overrides while held and springs back on release
	check(ck.begin_drag("pedal_throttle", Vector2(100, 100)), name + ": throttle pedal drags")
	ck.drag_to(Vector2(100, 210))
	var thr := float(ck.get_overrides().get("throttle", 0.0))
	check(thr > 0.4 and thr < 0.6, "%s: half a pedal drag = half throttle (got %.2f)" % [name, thr])
	ck.end_drag()
	check(not ck.get_overrides().has("throttle") or name == "runabout_t", name + ": the pedal springs back")
	# posing from the car: the steering wheel turns with steer, the speedometer with speed, the brake pedal with brake
	var wheel: Node3D = root.find_child("steering_wheel", true, false)
	var wheel_rest := wheel.transform.basis
	sim.controls["steer"] = 0.25
	sim.controls["brake"] = 1.0
	sim.speed_mps = 10.0
	sim.gear = 1
	for _i in 30:
		await process_frame
	var turns := float(spec["steering"]["steering_wheel_lock_deg"]) / 360.0
	var axis_iso: Array = []
	var rig: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path.get_basename() + ".rig.json"))
	for n in rig["nodes"]:
		if n["name"] == "steering_wheel":
			axis_iso = n["joint"]["axis"]
	var axis := Vector3(float(axis_iso[1]), float(axis_iso[2]), float(axis_iso[0]))
	var got_angle := _angle_about(wheel, Basis.IDENTITY, axis)
	check(absf(got_angle - fmod(0.25 * turns * TAU, TAU)) < 0.02 or absf(got_angle - (0.25 * turns * TAU - TAU)) < 0.02,
			"%s: steering wheel at %.3f rad for steer 0.25 (want %.3f)" % [name, got_angle, 0.25 * turns * TAU])
	var needle: Node3D = root.find_child("needle_speed", true, false)
	check(not needle.transform.basis.is_equal_approx(Basis.IDENTITY), name + ": speedometer needle moved at 36 km/h")
	var pedal: Node3D = root.find_child("pedal_brake", true, false)
	check(not pedal.transform.basis.is_equal_approx(Basis.IDENTITY), name + ": brake pedal moved under full brake")
	var lever: Node3D = root.find_child("lever_gear", true, false)
	check(not lever.transform.basis.is_equal_approx(Basis.IDENTITY), name + ": gear lever left neutral in first gear")
	# optional model parts (model_options.gd): the Runabout's folding top, the Ventura's raised hoods
	var parts := preload("res://scripts/model_options.gd")
	var rig_file := path.get_basename() + ".rig.json"
	if name == "runabout_t":
		var top: Node3D = root.find_child("top", true, false)
		parts.apply(root, rig_file, [])
		check(not top.visible, "runabout_t: the top is off as delivered")
		parts.apply(root, rig_file, PackedStringArray(["roof"]))
		check(top.visible, "runabout_t: the folding top option shows it")
	if name == "ventura_gt":
		check(parts.hood_variant(rig_file, 577.0) == "", "ventura_gt: its own V8 fits under the flat hood")
		check(parts.hood_variant(rig_file, 760.0) == "hood_bulge", "ventura_gt: a 760 mm engine gets the bulge")
		check(parts.hood_variant(rig_file, 820.0) == "hood_scoop", "ventura_gt: an 820 mm engine gets the scoop")
		parts.apply(root, rig_file, ["hood_scoop"])
		check(root.find_child("hood_scoop", true, false).visible and not root.find_child("hood_bulge", true, false).visible, "ventura_gt: only the picked hood shows")
	holder.queue_free()
	sim.queue_free()
	await process_frame

# Hazard / indicator stalk / horn switches submit ActionEvents (source CockpitMouse = 3) and follow the projected state.
func _test_shared_switches(ck: Node3D, sim: FakeSim, spec: Dictionary, name: String) -> void:
	var fake := FakeActions.new()
	fake.sim = sim
	ck.set("driver_actions", fake)
	for c in spec["controls"]:
		var action: Dictionary = c.get("action", {})
		var ch := str(action.get("channel", ""))
		var node := str(c["node"])
		if ch == "hazard":
			fake.events.clear()
			check(ck.activate(node, 1), "%s: %s clicks" % [name, node])
			check(fake.events.size() == 1 and fake.events[0] == {"id": "hazard", "kind": 2, "value": 1.0, "source": 3}, "%s: hazard click submits Set 1 from the cockpit mouse (%s)" % [name, fake.events])
			check(ck.get_cosmetic("hazard") == true, name + ": hazard switch follows the projection (on)")
			sim.controls["cabin.hazard"] = 0.0 # a key press (or anything else) clears it: the switch follows
			check(ck.get_cosmetic("hazard") == false, name + ": hazard switch follows a change made elsewhere")
			ck.activate(node, 1)
			ck.activate(node, 1)
			check(fake.events[-1]["value"] == 0.0 and ck.get_cosmetic("hazard") == false, name + ": second click switches it off")
		elif ch == "lights":
			fake.events.clear()
			check(ck.activate(node, 1), "%s: %s clicks" % [name, node])
			check(fake.events.size() == 1 and fake.events[0] == {"id": "lights", "kind": 0, "value": 1.0, "source": 3}, "%s: light switch click submits a lights Press (the auto/on/off cycle) from the cockpit mouse (%s)" % [name, fake.events])
			check(ck.get_cosmetic("lights") == true, name + ": light switch follows the projection (on)")
			sim.controls["cabin.lights"] = 2.0 # O key held for main beam: still on
			check(ck.get_cosmetic("lights") == true, name + ": light switch shows on for main beam")
			sim.controls["cabin.lights"] = 0.0 # O key switched them off: the switch follows
			check(ck.get_cosmetic("lights") == false, name + ": light switch follows a change made elsewhere")
			ck.activate(node, 1)
			ck.activate(node, 1)
			check(fake.events.size() == 3 and fake.events[-1]["kind"] == 0 and ck.get_cosmetic("lights") == false, name + ": two more clicks cycle on -> off -> auto (daylight: off)")
		elif ch == "indicator":
			fake.events.clear()
			ck.activate(node, 1) # off -> left
			check(fake.events.size() == 1 and fake.events[0]["id"] == "indicator_left" and fake.events[0]["value"] == 1.0 and fake.events[0]["source"] == 3, "%s: stalk up submits indicator_left (%s)" % [name, fake.events])
			check(ck.get_cosmetic("indicator") == "left", name + ": stalk follows the projection (left)")
			sim.controls["cabin.indicator"] = 0.0 # self-cancel by wheel return
			check(ck.get_cosmetic("indicator") == "off", name + ": stalk follows the self-cancel")
			fake.events.clear()
			ck.activate(node, -1) # off -> right
			check(fake.events.size() == 1 and fake.events[0]["id"] == "indicator_right" and ck.get_cosmetic("indicator") == "right", name + ": stalk down submits indicator_right")
			ck.activate(node, 1) # right -> off
			check(fake.events[-1]["id"] == "indicator_left" and fake.events[-1]["value"] == 0.0 and ck.get_cosmetic("indicator") == "off", name + ": stalk back to off clears")
		elif ch == "horn":
			fake.events.clear()
			ck.activate(node, 1)
			check(fake.events.size() == 1 and fake.events[0] == {"id": "horn", "kind": 2, "value": 1.0, "source": 3}, "%s: horn click submits Set 1 (%s)" % [name, fake.events])
			check(ck.get_cosmetic("horn") == true, name + ": horn follows the projection")
			ck.submit_held()
			check(fake.events.size() == 2 and fake.events[1]["value"] == 1.0, name + ": held horn re-asserts its level")
			await create_timer(0.4).timeout
			check(fake.events[-1]["id"] == "horn" and fake.events[-1]["kind"] == 1 and ck.get_cosmetic("horn") == false, name + ": horn released after the press time")
			# button held: the level stays up across frames, mouse-up (anywhere) releases
			fake.events.clear()
			ck.activate(node, 1)
			ck.hold_horn()
			await create_timer(0.4).timeout
			ck.submit_held()
			check(ck.is_horn_held() and ck.get_cosmetic("horn") == true and fake.events[-1]["kind"] == 2 and fake.events[-1]["value"] == 1.0, name + ": held horn stays up past the press time")
			var up := InputEventMouseButton.new()
			up.button_index = MOUSE_BUTTON_LEFT
			up.pressed = false
			ck._input(up)
			check(not ck.is_horn_held() and fake.events[-1]["kind"] == 1 and ck.get_cosmetic("horn") == false, name + ": mouse-up releases the horn")
			ck.submit_held()
			check(fake.events[-1]["kind"] == 1, name + ": nothing re-asserts after release")
			ck.activate(node, 1)
			ck.hold_horn()
			ck._notification(Node.NOTIFICATION_APPLICATION_FOCUS_OUT)
			check(not ck.is_horn_held() and fake.events[-1]["kind"] == 1, name + ": focus loss releases the horn")
	# non driver-backed switches (fog, wipers) stay local
	fake.events.clear()
	for c in spec["controls"]:
		var ch2 := str(c.get("action", {}).get("channel", ""))
		if ch2 in ["wipers", "fog"]:
			ck.activate(str(c["node"]), 1)
	check(fake.events.is_empty(), name + ": fog/wiper switches submit nothing")
	ck.set("driver_actions", null)
