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
	var turns := 0.5 * float(spec["steering"]["lock_to_lock_turns"])
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
	holder.queue_free()
	sim.queue_free()
	await process_frame
