extends Node
# game/scripts/camera_switch_test.gd - `--camera-test` (tools/smoke_test.ps1
# -Cameras): the PHYS-008 camera-switch test, headless, flat world.
#
# What "no jump" means here. The rigs of the drive views (chase, bumper,
# cockpit, orbit, cinematic) are mounted on the car, so their camera sits at a
# fixed place relative to the chassis once it has settled; a rebase of the
# floating origin shifts the chassis AND every rig root by the same vector, so
# the camera-minus-chassis offset ("rel") is invariant under a rebase. With the
# simulation paused (the chassis does not move) the test measures rel and the
# camera orientation:
#
#  1. BASELINE: switch to each drive view from a fixed start (the chase view)
#     and record its camera one frame after the switch.
#  2. PAIRS: for every ordered pair (A, B) of drive views, run A, switch to B
#     and compare B's camera one frame later with its baseline. Done twice:
#     plain, and with an out-of-band floating-origin rebase (RgSimulation.
#     rebase_focus 900 m away, above the 500 m threshold) in the very frame of
#     the switch - the director then also rebases back to the active rig, so
#     two rebases cross the switch. The camera must not change (POS_TOL_M /
#     ROT_TOL_DEG) and must not drift on the frame after either.
#  3. STAY: every drive view stays put across a rebase without a switch (every
#     rig root, active or not, follows the origin).
#  4. FREE CAM: switching A -> free_cam starts where A's camera was (position
#     and view direction), with and without a rebase in the switch frame; and
#     free_cam -> B reproduces B's baseline.
#
#  5. DRONE (drone_follow, own car as the target): the drone does not sit at a
#     fixed place on activation - it starts where the previous camera was and
#     eases to its trailing position (exponential lag), so its checks differ:
#     (a) A -> drone: the first frame moves at most the lag's share of the way
#     (plus POS_TOL_M) from A's camera, also across a rebase in the switch
#     frame; (b) once settled the drone camera equals its baseline whatever A
#     was and whether or not a rebase crossed the switch; (c) drone -> B
#     reproduces B's baseline; (d) a settled drone stays put across a rebase.
#     The maths is rg::drone_* (tests/unit/test_camera_math.cpp [drone]).
#
#  6. FPV DRONE (fpv_drone mode, rg::FpvDrone): the camera is the drone's, so it
#     does not continue the previous camera - the flyer appears above the car
#     facing along it. Checks: (a) A -> fpv_drone, plain and with a rebase in
#     the switch frame: once the drone exists (the simulation is unpaused for
#     that, then paused) its camera equals the baseline whatever A was; (b)
#     fpv_drone -> B reproduces B's baseline; (c) flying forward moves the
#     camera along its view direction and the drone stops at the ground when
#     it descends; (d) leaving the mode removes the drone.
#
# Prints "RG_CAMERA_TEST ..." lines and a final PASS / FAIL line with the worst
# position/rotation error seen; exit code 1 on a failure. The walker rig is not
# in the matrix: it needs a spawned walker (a stepping simulation and a car
# below the get-out speed); drive_smoke.gd switches to it.

const POS_TOL_M := 0.05
const ROT_TOL_DEG := 0.5
const REBASE_OFFSET_M := 900.0
const TIMEOUT_S := 300.0
const DRONE_LAG := 4.0 # drone_rig.gd position_lag
const DRONE_SETTLE_S := 2.5 # exp(-4 * 2.5) = 4.5e-5 of the fly-in left

var main: Node
var _checks := 0
var _failures := 0
var _max_pos := 0.0
var _max_rot := 0.0
var _rebases_seen := 0

func _ready() -> void:
	_run.call_deferred()

func _check(condition: bool, what: String) -> bool:
	_checks += 1
	if not condition:
		_failures += 1
		print("RG_CAMERA_TEST FAIL: %s" % what)
	return condition

func _frames(n: int = 1) -> void:
	for i in range(n):
		await get_tree().process_frame

func _wait_until(condition: Callable, timeout_s: float) -> bool:
	var deadline := Time.get_ticks_msec() + int(timeout_s * 1000.0)
	while not condition.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await get_tree().process_frame
	return true

# The camera's pose relative to the chassis, in Godot axes (rebase invariant).
func _sample() -> Dictionary:
	var camera: Camera3D = main._director.active_camera()
	var chassis: Transform3D = main._simulation.get_body_transform("chassis")
	return {
		"rig": main._director.active_name,
		"rel": camera.global_position - chassis.origin,
		"basis": camera.global_transform.basis.orthonormalized(),
		"pos": camera.global_position,
	}

func _rot_deg(a: Basis, b: Basis) -> float:
	return rad_to_deg((a.get_rotation_quaternion().inverse() * b.get_rotation_quaternion()).get_angle())

# Compares two samples (rig-relative), records the worst error, returns ok.
func _compare(what: String, got: Dictionary, want: Dictionary) -> bool:
	var dp: float = (got["rel"] - want["rel"]).length()
	var dr: float = _rot_deg(got["basis"], want["basis"])
	_max_pos = maxf(_max_pos, dp)
	_max_rot = maxf(_max_rot, dr)
	return _check(dp <= POS_TOL_M and dr <= ROT_TOL_DEG,
		"%s: position error %.4f m (tol %.2f), rotation error %.3f deg (tol %.1f)" % [what, dp, POS_TOL_M, dr, ROT_TOL_DEG])

# Triggers an out-of-band floating-origin rebase of REBASE_OFFSET_M; returns
# the size of the origin change (0 when nothing happened).
func _force_rebase() -> float:
	var sim: Node = main._simulation
	var before: Vector3 = sim.get_world_origin_godot_position()
	var camera: Camera3D = main._director.active_camera()
	sim.rebase_focus(camera.global_position + Vector3(REBASE_OFFSET_M, 0.0, 0.0))
	var moved: float = (sim.get_world_origin_godot_position() - before).length()
	if moved > 0.0:
		_rebases_seen += 1
	return moved

func _view(view_name: String, rebase: bool) -> void:
	if rebase:
		var moved := _force_rebase()
		_check(moved >= REBASE_OFFSET_M * 0.9, "the out-of-band rebase moved the origin by %.1f m" % moved)
	main.set_drive_view(view_name)

func _to_view(view_name: String, settle_frames: int = 3) -> void:
	main.set_drive_view(view_name)
	await _frames(settle_frames)

# Waits wall-clock seconds (the drone's lag runs on frame deltas).
func _settle(seconds: float) -> void:
	var until := Time.get_ticks_msec() + int(seconds * 1000.0)
	while Time.get_ticks_msec() < until:
		await get_tree().process_frame

func _to_drone(rebase: bool) -> void:
	if rebase:
		var moved := _force_rebase()
		_check(moved >= REBASE_OFFSET_M * 0.9, "drone: the out-of-band rebase moved the origin by %.1f m" % moved)
	main._simulation.set_player_mode("drone_follow")

func _run() -> void:
	var guard := get_tree().create_timer(TIMEOUT_S)
	guard.timeout.connect(func():
		print("RG_CAMERA_TEST FAIL: timeout")
		get_tree().quit(1))
	var sim: Node = main._simulation
	var director: Node = main._director
	if not _check(await _wait_until(func(): return main.world_state == "running" and main.shell_screen() == "drive", 30.0), "the flat world runs and the Drive screen is up"):
		get_tree().quit(1)
		return
	await _frames(30)
	# A paused simulation keeps the chassis still: every measurement is static.
	sim.set_paused(true)
	await _frames(5)

	var views: PackedStringArray = RgCameraMath.drive_view_names()
	_check(views.size() == 5, "five driving views (%s)" % ", ".join(views))
	for view_name in views:
		_check(director.rigs.has(view_name), "the director has a rig for '%s'" % view_name)
	_check(director.rigs.has("free"), "the director has the free rig")
	_check(director.get_process_priority() == -1000, "the camera director runs at process priority -1000 (PHYS-008)")

	# ---- 1. baselines ----
	var baseline := {}
	for view_name in views:
		main.set_drive_view("chase")
		await _frames(3)
		main.set_drive_view(view_name)
		await _frames(1)
		var s := _sample()
		_check(s["rig"] == view_name, "the active rig is '%s' after the switch (is '%s')" % [view_name, s["rig"]])
		baseline[view_name] = s
		print("RG_CAMERA_TEST baseline %s rel=(%.3f, %.3f, %.3f)" % [view_name, s["rel"].x, s["rel"].y, s["rel"].z])
	# the rigs are not all the same camera: a cockpit that equals the chase view would hide a broken rig
	_check((baseline["chase"]["rel"] - baseline["cockpit"]["rel"]).length() > 1.0, "chase and cockpit sit in different places")
	_check((baseline["chase"]["rel"] - baseline["bumper"]["rel"]).length() > 1.0, "chase and bumper sit in different places")

	# ---- 2. every ordered pair, plain and across a rebase ----
	var pair_count := 0
	var rebase_pair_count := 0
	for with_rebase in [false, true]:
		for a in views:
			for b in views:
				if a == b:
					continue
				await _to_view(a)
				var rebase_count_before: int = director.rebase_count
				_view(b, with_rebase)
				await _frames(1)
				var first := _sample()
				var tag := "%s -> %s%s" % [a, b, " (rebase in the switch frame)" if with_rebase else ""]
				_check(first["rig"] == b, "%s: the active rig is %s" % [tag, b])
				_compare(tag, first, baseline[b])
				await _frames(2)
				_compare(tag + ", two frames later", _sample(), first)
				if with_rebase:
					_check(director.rebase_count > rebase_count_before, "%s: the director rebased onto the active rig too" % tag)
					rebase_pair_count += 1
				else:
					pair_count += 1
	print("RG_CAMERA_TEST pairs=%d rebase_pairs=%d" % [pair_count, rebase_pair_count])

	# ---- 3. a rebase without a switch ----
	for view_name in views:
		await _to_view(view_name)
		var before := _sample()
		_force_rebase()
		await _frames(2)
		_compare("%s stays put across a rebase" % view_name, _sample(), before)
		# the INACTIVE rigs followed the origin too: switch to the next view and
		# check it against its baseline (the pairs above did this from a rebase
		# in the switch frame; this one rebases a frame EARLIER while it is inactive)
	for view_name in views:
		await _to_view("chase")
		_force_rebase()
		await _frames(2) # every rig root shifted while chase was active
		main.set_drive_view(view_name)
		await _frames(1)
		_compare("chase -> %s after an earlier rebase" % view_name, _sample(), baseline[view_name])

	# ---- 4. free cam ----
	var free_checks := 0
	for with_rebase in [false, true]:
		for a in views:
			await _to_view(a)
			var from := _sample()
			var from_pos: Vector3 = from["pos"]
			if with_rebase:
				var moved := _force_rebase()
				_check(moved >= REBASE_OFFSET_M * 0.9, "free cam: the rebase moved the origin by %.1f m" % moved)
			sim.set_player_mode("free_cam")
			await _frames(1)
			var s := _sample()
			var tag := "%s -> free_cam%s" % [a, " across a rebase" if with_rebase else ""]
			_check(s["rig"] == "free", "%s: the free rig is active" % tag)
			# the free rig keeps the previous camera's view direction; it only moves
			# with input (none here), so rel equals the previous camera's rel
			_compare(tag, s, from)
			free_checks += 1
			sim.set_player_mode("drive")
			await _frames(1)
			main.set_drive_view("chase")
			await _frames(2)
			main.set_drive_view(a)
			await _frames(1)
			_compare("free_cam -> %s" % a, _sample(), baseline[a])
	print("RG_CAMERA_TEST free_cam_checks=%d rebases=%d rebase_count=%d" % [free_checks, _rebases_seen, director.rebase_count])

	# ---- 5. drone follow ----
	var drone_checks := 0
	sim.set_player_mode("drone_follow")
	await _settle(DRONE_SETTLE_S)
	var drone_base := _sample()
	_check(drone_base["rig"] == "drone", "drone_follow activates the drone rig (is '%s')" % drone_base["rig"])
	_check(director.rigs.has("drone"), "the director has the drone rig")
	sim.set_player_mode("drive")
	await _frames(2)
	for with_rebase in [false, true]:
		for a in views:
			main.set_drive_view("chase")
			await _frames(3)
			await _to_view(a)
			var from := _sample()
			var tag := "%s -> drone%s" % [a, " (rebase in the switch frame)" if with_rebase else ""]
			var t0 := Time.get_ticks_msec()
			_to_drone(with_rebase)
			await _frames(1)
			var dt_s := maxf(float(Time.get_ticks_msec() - t0) / 1000.0, 1.0 / 60.0)
			var first := _sample()
			_check(first["rig"] == "drone", "%s: the drone rig is active" % tag)
			# starts where the previous camera was: at most the lag's share of the fly-in (two frame times of slack)
			var fly: float = (drone_base["rel"] - from["rel"]).length()
			var allowed: float = fly * (1.0 - exp(-DRONE_LAG * dt_s * 2.0)) + POS_TOL_M
			var moved: float = (first["rel"] - from["rel"]).length()
			_max_pos = maxf(_max_pos, maxf(moved - allowed, 0.0))
			_check(moved <= allowed, "%s: first frame moved %.3f m of a %.2f m fly-in (allowed %.3f)" % [tag, moved, fly, allowed])
			await _settle(DRONE_SETTLE_S)
			_compare(tag + ", settled", _sample(), drone_base)
			drone_checks += 1
			sim.set_player_mode("drive")
			await _frames(1)
			main.set_drive_view("chase")
			await _frames(2)
			main.set_drive_view(a)
			await _frames(1)
			_compare("drone -> %s" % a, _sample(), baseline[a])
	# a settled drone stays put across a rebase (its rig root and smoothed state follow the origin)
	sim.set_player_mode("drone_follow")
	await _settle(DRONE_SETTLE_S)
	var before_rebase := _sample()
	_force_rebase()
	await _frames(2)
	_compare("settled drone stays put across a rebase", _sample(), before_rebase)
	sim.set_player_mode("drive")
	await _frames(2)
	print("RG_CAMERA_TEST drone_checks=%d" % drone_checks)

	# ---- 6. FPV drone ----
	var fpv_checks := 0
	var fpv_base := {}
	for with_rebase in [false, true]:
		for a in views:
			await _to_view(a)
			if with_rebase:
				var moved := _force_rebase()
				_check(moved >= REBASE_OFFSET_M * 0.9, "fpv: the out-of-band rebase moved the origin by %.1f m" % moved)
			sim.set_paused(false)
			sim.set_player_mode("fpv_drone")
			var got_drone: bool = await _wait_until(func(): return not sim.get_fpv_state().is_empty() and director.active_name == "fpv", 10.0)
			await _frames(3)
			sim.set_paused(true)
			await _frames(3)
			var tag := "%s -> fpv_drone%s" % [a, " (rebase in the switch frame)" if with_rebase else ""]
			_check(got_drone, "%s: the drone exists and the fpv rig is active" % tag)
			var s := _sample()
			if fpv_base.is_empty():
				fpv_base = s
				print("RG_CAMERA_TEST fpv baseline rel=(%.3f, %.3f, %.3f)" % [s["rel"].x, s["rel"].y, s["rel"].z])
				_check(float(s["rel"].y) > 1.5, "the drone hovers above the car (rel y %.2f)" % s["rel"].y)
			else:
				_compare(tag, s, fpv_base)
			fpv_checks += 1
			sim.set_player_mode("drive")
			sim.set_paused(false)
			await _frames(3)
			_check(sim.get_fpv_state().is_empty(), "%s: leaving the mode removes the drone" % tag)
			sim.set_paused(true)
			await _frames(2)
			main.set_drive_view("chase")
			await _frames(2)
			main.set_drive_view(a)
			await _frames(1)
			_compare("fpv_drone -> %s" % a, _sample(), baseline[a])
	# flying: forward input moves the camera along its view direction; descending ends on the ground
	sim.set_paused(false)
	sim.set_player_mode("fpv_drone")
	await _wait_until(func(): return not sim.get_fpv_state().is_empty(), 10.0)
	var st0: Dictionary = sim.get_fpv_state()
	sim.set_fpv_input(0.0, 0.0, 1.0, 0.0, 0.0)
	await _settle(1.0)
	var st1: Dictionary = sim.get_fpv_state()
	var moved_v: Vector3 = st1["position"] - st0["position"]
	_check(moved_v.length() > 3.0, "forward input moves the drone (%.1f m)" % moved_v.length())
	_check(moved_v.normalized().dot(st0["facing"]) > 0.95, "... along its heading")
	_check(float(st1["speed"]) <= 25.0 + 0.01, "... never above the maximum speed (%.2f m/s)" % float(st1["speed"]))
	sim.set_fpv_input(0.0, -1.0, 0.0, 0.0, 0.0)
	await _settle(4.0)
	var st2: Dictionary = sim.get_fpv_state()
	_check(bool(st2["on_ground"]), "descending ends on the ground (height %.2f m)" % float(st2["height_agl_m"]))
	sim.set_fpv_input(0.0, 0.0, 0.0, 0.0, 0.0)
	sim.set_player_mode("drive")
	await _frames(3)
	sim.set_paused(true)
	await _frames(2)
	print("RG_CAMERA_TEST fpv_checks=%d" % fpv_checks)

	sim.set_paused(false)
	if _failures == 0:
		print("RG_CAMERA_TEST PASS checks=%d max_pos_err_m=%.5f max_rot_err_deg=%.4f pos_tol_m=%.2f rot_tol_deg=%.1f" % [_checks, _max_pos, _max_rot, POS_TOL_M, ROT_TOL_DEG])
		get_tree().quit()
	else:
		print("RG_CAMERA_TEST FAIL failures=%d checks=%d max_pos_err_m=%.5f max_rot_err_deg=%.4f" % [_failures, _checks, _max_pos, _max_rot])
		get_tree().quit(1)
