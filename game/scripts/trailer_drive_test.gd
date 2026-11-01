extends SceneTree
# game/scripts/trailer_drive_test.gd - the scripted tow run (farm attachment plan A2), flat world, needs a real renderer
# for the screenshots (not for the numbers):
#   godot --path game --script res://scripts/trailer_drive_test.gd -- --flat --vehicle car_sedan --trailer-out <dir>
# Phases: bring the tipper trailer behind the car (standing) -> hook it on -> tow forward -> brake (the trailer brakes through
# the hitch) -> reverse -> stop and unhook -> bring it back (recover) -> relocate the coupled pair -> world reload (restart):
# the trailer comes back with the car. Prints one "RG_TRAILER_TEST ..." line per phase and a final PASS / FAIL line; takes
# <dir>/trailer_standing.png, trailer_towing.png and trailer_reverse.png (a camera of its own, 3/4 view from the rear right).
# Exit code 0 = PASS. Every number is the simulation's; nothing here decides physics.
# Recorded sabotages (both run, both fail as named): deleting "tow_brake" from data/vehicles/trailer_tipper.json fails "the trailer brake
# follows the car's"; making RgSimulation::capture_trailer_record return at once fails "after the reload the trailer is hooked on behind
# the car again" (no record survives the world load).

const TIMEOUT_S := 240.0
var main: Node
var sim: Node
var out_dir := ""
var failures: Array[String] = []
var _started_ms := 0
var _cam: Camera3D

func _initialize() -> void:
	_run.call_deferred()

func _arg(name: String) -> String:
	var a := OS.get_cmdline_user_args()
	var i := a.find(name)
	return a[i + 1] if i >= 0 and i + 1 < a.size() else ""

func _check(ok: bool, what: String) -> void:
	print("RG_TRAILER_TEST %s: %s" % ["ok" if ok else "FAIL", what])
	if not ok:
		failures.append(what)

func _trailer() -> Dictionary:
	return sim.get_trailer_state()

func _speed() -> float:
	return float(sim.get_steering_kinematics().get("forward_mps", 0.0))

# Waits (frames) until `cond` is true or `seconds` of wall time passed; true when it became true.
func _wait(cond: Callable, seconds: float) -> bool:
	var began := Time.get_ticks_msec()
	while not cond.call():
		await process_frame
		if (Time.get_ticks_msec() - began) / 1000.0 > seconds:
			return false
		if (Time.get_ticks_msec() - _started_ms) / 1000.0 > TIMEOUT_S:
			return false
	return true

# Waits `seconds` of SIMULATION time (the sim runs in real time on its own thread).
func _sim_wait(seconds: float) -> void:
	var t0 := float(sim.get_sim_time())
	while float(sim.get_sim_time()) - t0 < seconds:
		await process_frame
		if (Time.get_ticks_msec() - _started_ms) / 1000.0 > TIMEOUT_S:
			return

func _controls(throttle: float, brake: float, handbrake: float, extra: Dictionary = {}) -> void:
	var c := {"throttle": throttle, "brake": brake, "handbrake": handbrake, "steer": 0.0, "clutch": 0.0, "ignition": 1.0, "assist.auto_shift": 1.0}
	for k in extra:
		c[k] = extra[k]
	main.scripted_controls = c

func _shot(file_name: String) -> void:
	if out_dir == "":
		return
	if _cam == null:
		_cam = Camera3D.new()
		_cam.fov = 55.0
		_cam.far = 2000.0
		root.add_child(_cam)
	var car: Transform3D = sim.get_body_transform("chassis")
	var tr: Transform3D = sim.get_trailer_transform()
	var mid := car.origin
	if bool(_trailer().get("present", false)):
		mid = (car.origin + tr.origin) * 0.5
	var fwd: Vector3 = car.basis.x
	var left: Vector3 = car.basis.y
	_cam.global_position = mid - fwd * 2.0 - left * 11.0 + Vector3.UP * 4.0
	_cam.look_at(mid + Vector3.UP * 0.8, Vector3.UP)
	_cam.make_current()
	for _i in range(6):
		await process_frame
	var img := root.get_viewport().get_texture().get_image()
	img.save_png(out_dir.path_join(file_name))
	print("RG_TRAILER_TEST shot %s" % out_dir.path_join(file_name))

func _finish() -> void:
	main.scripted_controls = {}
	print("RG_TRAILER_TEST %s failures=%d" % ["PASS" if failures.is_empty() else "FAIL", failures.size()])
	quit(0 if failures.is_empty() else 1)

func _run() -> void:
	_started_ms = Time.get_ticks_msec()
	out_dir = _arg("--trailer-out")
	if out_dir != "":
		DirAccess.make_dir_recursive_absolute(out_dir)
	DirAccess.remove_absolute(OS.get_user_data_dir().path_join("trailer.json")) # a record of an earlier run would pre-spawn a trailer
	main = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	var ready_ok: bool = await _wait(func(): return main.get("world_state") == "running", 120.0)
	if not ready_ok:
		push_error("RG_TRAILER_TEST world did not start")
		quit(1)
		return
	sim = main.get_simulation()
	_check(bool(sim.has_trailer_config()), "the trailer config is loaded")
	var visual: Node = main.get_trailer_visual()
	# 1. bring the trailer behind the car (standing, not hooked)
	_controls(0.0, 1.0, 1.0)
	await _sim_wait(1.0)
	sim.request_trailer_spawn("", "", false)
	var standing: bool = await _wait(func(): return str(_trailer().get("state", "")) == "standing", 15.0)
	_check(standing, "a trailer stands behind the car (%s)" % str(_trailer().get("message", "")))
	var shown: bool = await _wait(func(): return visual != null and visual.model_loaded(), 15.0)
	_check(shown, "the trailer model is shown")
	await _sim_wait(2.0)
	var t: Dictionary = _trailer()
	print("RG_TRAILER_TEST standing kind=%s load=%s mass=%.0f hookup=%s %.2f m" % [t.get("kind"), t.get("load"), float(t.get("mass_kg", 0)), t.get("hookup_verdict"), float(t.get("hookup_distance_m", 0))])
	await _shot("trailer_standing.png")
	# 2. hook on (car still)
	sim.request_trailer_attach()
	var coupled: bool = await _wait(func(): return str(_trailer().get("state", "")) == "coupled", 10.0)
	_check(coupled, "hook-up works with the car standing (%s)" % str(_trailer().get("message", "")))
	await _sim_wait(2.0)
	t = _trailer()
	print("RG_TRAILER_TEST coupled class=%s towed_mass=%.0f tongue=%.0f N socket_error=%.1f mm findings=%s" % [t.get("class"), float(t.get("towed_mass_kg", 0)), float(t.get("tongue_load_n", 0)), float(t.get("socket_error_m", 0)) * 1000.0, str(t.get("findings"))])
	_check(float(t.get("socket_error_m", 1.0)) < 0.02, "the hitch socket error stays under 20 mm at rest")
	# 3. tow forward 12 s
	_controls(0.6, 0.0, 0.0)
	var max_yaw := 0.0
	var max_force := 0.0
	var t_end := float(sim.get_sim_time()) + 12.0
	var shot_taken := false
	while float(sim.get_sim_time()) < t_end:
		await process_frame
		t = _trailer()
		max_yaw = maxf(max_yaw, absf(float(t.get("yaw_deg", 0.0))))
		max_force = maxf(max_force, float(t.get("hitch_force_n", 0.0)))
		if not shot_taken and float(sim.get_sim_time()) > t_end - 3.0:
			shot_taken = true
			await _shot("trailer_towing.png")
	var v_tow := _speed()
	print("RG_TRAILER_TEST tow v=%.2f m/s max_yaw=%.2f deg max_hitch_force=%.0f N state=%s" % [v_tow, max_yaw, max_force, _trailer().get("state")])
	_check(v_tow > 4.0, "the pair drives forward (%.1f m/s)" % v_tow)
	_check(str(_trailer().get("state", "")) == "coupled", "the hitch holds in ordinary driving")
	_check(max_yaw < 10.0, "the articulation angle stays small on a straight line (%.1f deg)" % max_yaw)
	# 4. brake: the trailer brakes through the hitch
	_controls(0.0, 1.0, 0.0)
	await _sim_wait(0.7)
	var tow_brake := float(_trailer().get("tow_brake", 0.0))
	_check(tow_brake > 0.5, "the trailer brake follows the car's (%.2f)" % tow_brake)
	var stopped: bool = await _wait(func(): return absf(_speed()) < 0.3, 20.0)
	_check(stopped, "the pair stops (%.2f m/s)" % _speed())
	_check(str(_trailer().get("state", "")) == "coupled" and absf(float(_trailer().get("yaw_deg", 0.0))) < 20.0, "still coupled and straight after braking")
	# 5. reverse
	_controls(0.0, 0.0, 0.0, {"assist.auto_shift": 0.0, "gear_request": -1.0})
	await _sim_wait(1.0)
	_controls(0.4, 0.0, 0.0, {"assist.auto_shift": 0.0, "gear_request": -1.0})
	await _sim_wait(5.0)
	var v_rev := _speed()
	print("RG_TRAILER_TEST reverse v=%.2f m/s yaw=%.2f deg" % [v_rev, float(_trailer().get("yaw_deg", 0.0))])
	_check(v_rev < -0.3, "the pair reverses (%.2f m/s)" % v_rev)
	_check(str(_trailer().get("state", "")) == "coupled", "the hitch holds in reverse")
	await _shot("trailer_reverse.png")
	# 6. stop and unhook
	_controls(0.0, 1.0, 1.0, {"assist.auto_shift": 0.0, "gear_request": -1.0})
	await _wait(func(): return absf(_speed()) < 0.2, 20.0)
	sim.request_trailer_detach()
	var released: bool = await _wait(func(): return str(_trailer().get("state", "")) == "standing", 10.0)
	_check(released, "unhooking leaves the trailer standing (%s)" % str(_trailer().get("message", "")))
	# 7. the car drives off, the trailer stays
	var trailer_pos_before: Vector3 = sim.get_trailer_transform().origin
	_controls(0.5, 0.0, 0.0, {"gear_request": 1.0})
	await _sim_wait(4.0)
	var left_behind: float = trailer_pos_before.distance_to(sim.get_trailer_transform().origin)
	_check(left_behind < 1.5, "the unhooked trailer stays where it is (%.2f m)" % left_behind)
	_controls(0.0, 1.0, 1.0)
	await _wait(func(): return absf(_speed()) < 0.2, 20.0)
	# 8. bring it back and relocate the coupled pair
	sim.request_trailer_recover()
	var recovered: bool = await _wait(func(): return str(_trailer().get("state", "")) == "coupled", 10.0)
	_check(recovered, "recover puts the trailer behind the car and hooks it on (%s)" % str(_trailer().get("message", "")))
	await _sim_wait(2.0)
	var before_relocations := int(sim.get_streaming_status().get("relocations", 0))
	sim.relocate_vehicle(60.0, 25.0, 90.0)
	await _wait(func(): return int(sim.get_streaming_status().get("relocations", 0)) > before_relocations, 15.0)
	await _sim_wait(3.0)
	t = _trailer()
	_check(str(t.get("state", "")) == "coupled", "the pair is coupled after the car was relocated (%s)" % str(t.get("message", "")))
	_check(float(t.get("socket_error_m", 1.0)) < 0.02, "the hitch is intact after the relocation (%.1f mm)" % (float(t.get("socket_error_m", 0)) * 1000.0))
	# 9. restart: the world is torn down and loaded again; the trailer comes back with the car
	var serial_before := int(t.get("serial", 0))
	main._load_world(str(main.world_kind))
	var gone: bool = await _wait(func(): return main.get("world_state") != "running", 15.0)
	var back: bool = await _wait(func(): return main.get("world_state") == "running", 120.0)
	_check(gone and back, "the world reloads")
	sim = main.get_simulation()
	if back:
		_controls(0.0, 1.0, 1.0)
		var again: bool = await _wait(func(): return bool(_trailer().get("present", false)), 20.0)
		await _sim_wait(2.0)
		t = _trailer()
		print("RG_TRAILER_TEST restart present=%s state=%s serial=%d (was %d)" % [t.get("present"), t.get("state"), int(t.get("serial", 0)), serial_before])
		_check(again and str(t.get("state", "")) == "coupled", "after the reload the trailer is hooked on behind the car again")
	_finish()
