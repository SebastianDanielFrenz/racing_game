extends SceneTree
# Hyper cockpit clicking (cockpit_mouse.gd + RgCockpit, rg::CockpitHit). Headless:
#   Godot --headless --path game --script res://scripts/cockpit_mouse_test.gd
# Windowed (screenshots into out/hyper_cockpit_clicks/): ... -- --capture
const DriverState := preload("res://scripts/driver_state.gd")
const SOURCE_MOUSE := 3
class FakeSim extends Node:
	var controls := {"assist.traction_control": 1.0, "assist.drive_mode": 1.0}
	var powertrain := {
		"assist_fitted_bits": 8 | 16 | 64, # TC, launch control, aero control
		"traction_modes_available_bits": 31, "traction_mode_effective": 1,
		"drive_modes_available_bits": 7, "drive_mode_effective": 1, # eco, normal, sport offered; racing is not
		"launch_lamp": 1, "engine_state": "Running",
	}
	func get_vehicle_gauge_info(_v: String) -> Dictionary: return {}
	func get_vehicle_powertrain(_v: String) -> Dictionary: return powertrain
	func get_control(c: String) -> float: return float(controls.get(c, 0.0))
	func is_running() -> bool: return true
	func get_vehicle_wheel_count(_v: String) -> int: return 0
	func get_step_count() -> int: return 0
var failures := 0
var sim: FakeSim
var visual: Node3D
var mouse: Node
var camera: Camera3D
var actions: Object
func check(value: bool, message: String) -> void:
	if not value:
		failures += 1
		push_error(message)
func _initialize() -> void:
	call_deferred("run")

func frames(n: int, dt: float = 1.0 / 60.0) -> void:
	for i in range(n): mouse._process(dt)

func channels() -> Dictionary: return DriverState.channels(sim)

# Click the control with a ray from the eye through its hit centre; holds the button `hold` frames, then lets go and
# runs the frames that follow. Returns the control index the ray hit.
func click(id: String, button: int = 0, hold: int = 8, after: int = 12) -> int:
	var i: int = mouse.hit.find(id)
	var target: Vector3 = mouse.model_point_to_world(mouse.hit.hit_centre_model(i))
	var origin := camera.global_position
	var hit_index: int = mouse.click_ray(origin, (target - origin).normalized(), button)
	frames(hold)
	mouse.release_button(button)
	frames(after)
	return hit_index

# Hands the queued events to the shared RgDriverActions and applies them; returns {channel: value} writes.
func apply_now() -> Dictionary:
	mouse.flush_to(actions)
	var out := {}
	for w in actions.apply(sim.powertrain, channels(), true):
		out[str(w["channel"])] = float(w["value"])
	for w in actions.advance(sim.powertrain, channels(), true, 0.016):
		out[str(w["channel"])] = float(w["value"])
	return out

func run() -> void:
	sim = FakeSim.new()
	root.add_child(sim)
	visual = load("res://scripts/vehicle_visual.gd").new() as Node3D
	visual.simulation = sim
	visual.vehicle_name = "car_hyper"
	visual.model_absolute_path = ProjectSettings.globalize_path("res://../external/physics_sim/data/models/car_hyper/car_hyper.glb")
	root.add_child(visual)
	await process_frame
	await process_frame
	mouse = visual.get_cockpit_mouse()
	check(mouse != null, "the mounted hyper cockpit has a mouse adapter")
	if mouse == null:
		print("RG_COCKPIT_MOUSE_TEST failures=%d" % failures)
		quit(1)
		return
	var count: int = mouse.hit.get_control_count()
	check(count == 17, "17 controls (%d)" % count)
	actions = ClassDB.instantiate("RgDriverActions")
	var cabin: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/cabin.json")))
	actions.configure_cabin(cabin)
	camera = Camera3D.new()
	camera.near = 0.02
	camera.fov = 70
	root.add_child(camera)
	var eye := visual.find_child("socket_driver_eye", true, false) as Node3D
	check(eye != null, "driver eye socket")
	camera.global_position = eye.global_position
	camera.look_at(mouse.model_point_to_world(mouse.hit.hit_centre_model(mouse.hit.find("hazard"))), Vector3(0, 0, 1))
	camera.current = true
	mouse.driving_live = true
	mouse.world_input_active = true
	frames(2)

	# 1. The core's hit centres sit on the mounted hit_<part> nodes, also with the steering wheel turned.
	var wheel := visual.find_child("steering_wheel", true, false) as Node3D
	var wheel_rest := wheel.basis
	var axis := Vector3(0.0, 0.3011313679370974, -0.9535826651341417)
	for angle in [0.0, 0.6, -1.3]:
		wheel.basis = Basis(axis, angle) * wheel_rest
		mouse.wheel_angle_override = angle
		mouse._process(0.016)
		var worst := 0.0
		for i in range(count):
			var info: Dictionary = mouse.hit.control_info(i)
			var node := (mouse._parts[info["id"]] as Node3D).find_child("hit_" + str(info["part"]), true, false) as Node3D
			if node == null:
				check(false, "mounted hit node for " + str(info["id"]))
				continue
			worst = maxf(worst, node.global_position.distance_to(mouse.model_point_to_world(mouse.hit.hit_centre_model(i))))
		check(worst < 0.0015, "hit centres match the mounted nodes at wheel angle %.1f (worst %.4f m)" % [angle, worst])
	wheel.basis = wheel_rest
	mouse.wheel_angle_override = 0.0
	frames(1)

	# 2. Every control is hit by a ray from the eye through its centre (also the wheel-mounted ones at angle 0).
	for i in range(count):
		var id := str(mouse.hit.control_info(i)["id"])
		var target: Vector3 = mouse.model_point_to_world(mouse.hit.hit_centre_model(i))
		var picked: int = mouse.hit.pick(mouse.world_ray_to_model(camera.global_position, (target - camera.global_position).normalized())[0], mouse.world_ray_to_model(camera.global_position, (target - camera.global_position).normalized())[1])
		check(picked == i, "ray from the eye hits %s (got %s)" % [id, str(mouse.hit.control_info(picked)["id"]) if picked >= 0 else "nothing"])
	var up_ray: Array = mouse.world_ray_to_model(camera.global_position, Vector3(0, 0, 1))
	check(mouse.hit.pick(up_ray[0], up_ray[1]) < 0, "a ray at the roof misses")

	# 3. Clicks reach the shared DriverActions with the same results as the keys.
	check(click("tc") == mouse.hit.find("tc"), "tc hit")
	var writes := apply_now()
	check(writes.get("assist.traction_control", -1.0) == 2.0, "tc click cycles to the next mode: %s" % str(writes))
	sim.controls["assist.traction_control"] = 2.0
	check(click("tc", 1) >= 0, "tc secondary hit")
	writes = apply_now()
	check(writes.get("assist.traction_control", -1.0) == 0.0, "tc right click sets Off: %s" % str(writes))
	sim.controls["assist.traction_control"] = 0.0
	check(click("sport") >= 0, "sport hit")
	writes = apply_now()
	check(writes.get("assist.drive_mode", -1.0) == 2.0, "sport click selects drive mode 2: %s" % str(writes))
	check(click("lc") >= 0, "lc hit")
	check(apply_now().get("assist.launch_control", -1.0) == 1.0, "lc click arms launch control")
	check(click("aero") >= 0, "aero hit")
	check(apply_now().get("assist.aero_control", -1.0) == 1.0, "aero click")
	check(click("am") >= 0, "am hit")
	check(apply_now().get("assist.auto_shift", -1.0) == 1.0, "A/M click turns auto shift on")
	check(click("start") >= 0, "start hit")
	check(apply_now().has("ignition"), "start click writes the ignition")
	check(click("hazard") >= 0, "hazard hit")
	writes = apply_now()
	print("hazard writes ", writes)
	check(writes.get("cabin.hazard", -1.0) == 1.0, "hazard click turns the hazard lamps on: %s" % str(writes))
	click("hazard", 0, 8, 40) # debounce is over: the second click switches them off again
	check(apply_now().get("cabin.hazard", -1.0) == 0.0, "second hazard click turns them off")
	check(click("stalk_L") >= 0, "stalk_L hit")
	writes = apply_now()
	print("stalk writes ", writes)
	check(writes.get("cabin.indicator_left", -1.0) == 1.0, "left stalk switches the left indicator on: %s" % str(writes))
	check(click("stalk_L", 1, 8, 40) >= 0, "stalk_L secondary hit")
	writes = apply_now()
	check(writes.get("cabin.indicator_right", -1.0) == 1.0, "left stalk right click selects the right indicator: %s" % str(writes))
	check(click("lights") >= 0, "lights hit")
	writes = apply_now()
	print("lights writes ", writes)
	check(writes.has("cabin.light_mode") or writes.has("cabin.lights"), "light dial click writes the light state: %s" % str(writes))
	check(click("paddle_L") >= 0, "paddle_L hit")
	apply_now()
	check(mouse.consume_shift_down() == 1 and mouse.consume_shift_up() == 0, "left paddle is one shift-down edge")
	check(click("paddle_R") >= 0, "paddle_R hit")
	apply_now()
	check(mouse.consume_shift_up() == 1 and mouse.consume_shift_down() == 0, "right paddle is one shift-up edge")
	# the horn is a level while the button is down
	var horn_index: int = mouse.hit.find("horn")
	var horn_target: Vector3 = mouse.model_point_to_world(mouse.hit.hit_centre_model(horn_index))
	check(mouse.click_ray(camera.global_position, (horn_target - camera.global_position).normalized(), 0) == horn_index, "horn hit")
	frames(10)
	check(apply_now().get("cabin.horn", -1.0) == 1.0, "horn level while pressed")
	frames(10)
	check(apply_now().get("cabin.horn", -1.0) == 1.0, "horn level still held")
	mouse.release_button(0)
	frames(12)
	check(apply_now().get("cabin.horn", -1.0) == 0.0, "horn level released")

	# 4. Unavailable controls: racing is not offered and the EPB has no physics -> blank caps, clicks are inert.
	check(not (mouse._parts["racing"] as Node3D).visible and (mouse._blanks["racing"] as Node3D).visible, "racing shows the blank cap")
	check(not (mouse._parts["epb"] as Node3D).visible and (mouse._blanks["epb"] as Node3D).visible, "EPB shows the blank cap")
	check((mouse._parts["sport"] as Node3D).visible and not (mouse._blanks["sport"] as Node3D).visible, "offered mode keeps its button")
	click("racing")
	check(not apply_now().has("assist.drive_mode"), "an unoffered drive mode does nothing")
	click("epb")
	check(not apply_now().has("epb"), "EPB stays inert")

	# 5. Pressed button travels (pose from the core's travel, limit from the parts rig).
	var tc_joint := (mouse._parts["tc"] as Node3D).find_child("ctl_tc", true, false) as Node3D
	var rest_position := tc_joint.position
	var tc_target: Vector3 = mouse.model_point_to_world(mouse.hit.hit_centre_model(mouse.hit.find("tc")))
	mouse.click_ray(camera.global_position, (tc_target - camera.global_position).normalized(), 0)
	frames(20)
	check(tc_joint.position.distance_to(rest_position) > 0.0025, "pressed TC button is down (%.4f m)" % tc_joint.position.distance_to(rest_position))
	mouse.release_button(0)
	frames(20)
	check(tc_joint.position.distance_to(rest_position) < 1e-6, "TC button returns")
	apply_now()

	# 6. No clicks while the cursor is captured, a menu is open, or the camera is away from the driver.
	if DisplayServer.get_name() != "headless": # a headless display server ignores the capture request
		Input.mouse_mode = Input.MOUSE_MODE_CAPTURED
		frames(1)
		check(not mouse.hit.is_enabled(), "captured cursor: mouse look owns the mouse")
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
	var marker := Node.new()
	marker.add_to_group("seat_adjustment_open")
	root.add_child(marker)
	frames(1)
	check(not mouse.hit.is_enabled(), "menu open disables the cockpit")
	marker.free()
	frames(1)
	check(mouse.hit.is_enabled(), "back to enabled")
	var saved := camera.global_position
	camera.global_position = saved + Vector3(5, 0, 0)
	frames(1)
	check(not mouse.hit.is_enabled(), "camera away from the eye disables clicks")
	camera.global_position = saved
	mouse.driving_live = false
	frames(1)
	check(not mouse.hit.is_enabled(), "not driving live disables clicks")
	mouse.driving_live = true
	frames(1)

	if "--capture" in OS.get_cmdline_user_args():
		await capture()
	visual.free()
	sim.free()
	await process_frame
	print("RG_COCKPIT_MOUSE_TEST failures=%d" % failures)
	quit(1 if failures else 0)

func shot(name: String) -> void:
	for f in range(4): await process_frame
	await RenderingServer.frame_post_draw
	var image := root.get_texture().get_image()
	check(image != null and not image.is_empty(), "rendered " + name)
	if image != null:
		var dir := ProjectSettings.globalize_path("res://../out/hyper_cockpit_clicks")
		DirAccess.make_dir_recursive_absolute(dir)
		var path := dir.path_join(name + ".png")
		check(image.save_png(path) == OK, "saved " + name)
		print("RG_COCKPIT_MOUSE_CAPTURE " + path)

func aim_at(id: String, back: float, up: float = 0.0) -> Vector3:
	var target: Vector3 = mouse.model_point_to_world(mouse.hit.hit_centre_model(mouse.hit.find(id)))
	camera.global_position = target + Vector3(-back, 0, 0.35 + up)
	camera.look_at(target, Vector3(0, 0, 1))
	return target

func capture() -> void:
	root.size = Vector2i(1400, 1000)
	DisplayServer.window_set_size(root.size)
	var world := WorldEnvironment.new()
	world.environment = Environment.new()
	world.environment.background_mode = Environment.BG_COLOR
	world.environment.background_color = Color(0.15, 0.18, 0.23)
	world.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	world.environment.ambient_light_color = Color(0.7, 0.75, 0.85)
	world.environment.ambient_light_energy = 0.8
	root.add_child(world)
	var light := OmniLight3D.new()
	light.position = Vector3(0.0, 0.3, 1.2)
	light.omni_range = 3.0
	light.light_energy = 1.2
	root.add_child(light)
	camera.fov = 50
	# hover: the cursor over TC
	var target := aim_at("tc", 0.2)
	mouse.pointer_override = camera.unproject_position(target)
	frames(3)
	check(mouse.hovered_id() == "tc", "hover finds TC (%s)" % mouse.hovered_id())
	await shot("hover_tc")
	# pressed rows: hold the button down until the travel saturates
	for entry in [["pressed_left_row_tc", "tc"], ["pressed_right_row_sport", "sport"], ["pressed_centre_start", "start"], ["pressed_centre_hazard", "hazard"]]:
		var t := aim_at(entry[1], 0.2)
		mouse.pointer_override = camera.unproject_position(t)
		mouse.click_ray(camera.global_position, (t - camera.global_position).normalized(), 0)
		frames(25)
		await shot(entry[0])
		mouse.release_button(0)
		frames(25)
		apply_now()
	# the wheel controls from the driver side
	for entry in [["pressed_wheel_horn", "horn"], ["pressed_wheel_paddle_L", "paddle_L"]]:
		var t := aim_at(entry[1], 0.25, 0.1)
		mouse.pointer_override = camera.unproject_position(t)
		mouse.click_ray(camera.global_position, (t - camera.global_position).normalized(), 0)
		frames(25)
		await shot(entry[0])
		mouse.release_button(0)
		frames(25)
		apply_now()
	mouse.pointer_override = Vector2(-1, -1)
