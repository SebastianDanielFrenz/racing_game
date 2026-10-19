extends SceneTree
# game/scripts/hs_drive_test.gd - windowed high-speed drive over the real route
# with the shipped hypercar (owner complaint 2026-10-13: terrain does not
# resolve in time at speed, "I drove through the visible ground"). A SceneTree
# script like b8_contact_smoke.gd:
#   Godot --path game --script res://scripts/hs_drive_test.gd -- [options]
# Relocates to a station on data/routes/home_r1_drive.json, follows the route
# with a pure-pursuit steer at a target speed and measures, every 0.1 s:
#   l0_ahead / l1_ahead   how far ahead of the car the UPLOADED ground is
#                         covered by chunks of level <= 0 / <= 1 (along the
#                         velocity), and
#   ground_above          the highest visible render surface above the physics
#                         surface (carved L0 tiles) at the car and 5/20/60 m
#                         ahead - "visible ground above the wheel contact".
# main.gd's own RG_TERRAIN_STREAM / RG_ROAD_STREAM lines (1 Hz) run next to it.
# Prints one RG_HS_DONE line. Options: --hs-speed=70 (target m/s), --hs-start=1000
# (route station, m), --hs-seconds=40 (wall seconds of driving), --hs-baseline
# (no velocity lead, fixed 0.8 ms upload budget). --hs-fly: fly the kinematic FPV drone along the route instead of
# driving the car (the drone is physics interest point 0, so the same terrain streaming and the same gate run, without
# the car crashing into guard rails; needs RG_FPV_CONFIG=<json with a high max_speed_mps>). Stuck guard (car mode):
# speed < 5 km/h for > 5 s after a 10 s start grace aborts the run with RG_HS_ABORT.
var main: Node
var points: Array = []
var stations: Array[float] = []
var began := Time.get_ticks_msec()

func _initialize() -> void:
	_run.call_deferred()

func _fail(message: String) -> void:
	push_error("RG_HS_DRIVE " + message)
	quit(1)

func _target(station: float) -> Vector3:
	for i in range(1, stations.size()):
		if stations[i] >= station:
			var t := (station - stations[i - 1]) / (stations[i] - stations[i - 1])
			var a := Vector2(points[i - 1][0], points[i - 1][1])
			var b := Vector2(points[i][0], points[i][1])
			var p := a.lerp(b, t)
			return Vector3(p.x, p.y, rad_to_deg((b - a).angle()))
	return Vector3.ZERO

func _densify(src: Array, step_m: float) -> Array:
	var out: Array = []
	for i in range(src.size() - 1):
		var a := Vector2(src[i][0], src[i][1])
		var b := Vector2(src[i + 1][0], src[i + 1][1])
		var n := maxi(1, int(ceil(a.distance_to(b) / step_m)))
		for k in range(n):
			var p := a.lerp(b, float(k) / n)
			out.append([p.x, p.y])
	out.append(src[src.size() - 1])
	return out

func _percentile(values: Array, q: float) -> float:
	if values.is_empty():
		return 0.0
	var sorted := values.duplicate()
	sorted.sort()
	return float(sorted[clampi(int(ceil(q * sorted.size())) - 1, 0, sorted.size() - 1)])

func _run() -> void:
	var target_speed := 70.0
	var start_s := 1000.0
	var seconds := 40.0
	var baseline := false
	var fly := false
	var carve := -1
	var shot_dir := ""
	var route_file := "res://../data/routes/home_r1_drive.json"
	var view_name := ""
	var hide_terrain := false
	var hide_ribbons := false
	var shot_t0 := 3.0
	var shot_step := 6.0
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--hs-speed="):
			target_speed = float(a.get_slice("=", 1))
		elif a.begins_with("--hs-start="):
			start_s = float(a.get_slice("=", 1))
		elif a.begins_with("--hs-seconds="):
			seconds = float(a.get_slice("=", 1))
		elif a.begins_with("--hs-shots="):
			shot_dir = a.get_slice("=", 1)
		elif a.begins_with("--hs-carve="):
			carve = int(a.get_slice("=", 1))
		elif a.begins_with("--hs-route="):
			route_file = a.get_slice("=", 1)
		elif a.begins_with("--hs-view="):
			view_name = a.get_slice("=", 1)
		elif a.begins_with("--hs-shot-t0="):
			shot_t0 = float(a.get_slice("=", 1))
		elif a.begins_with("--hs-shot-step="):
			shot_step = float(a.get_slice("=", 1))
		elif a == "--hs-hide-terrain":
			hide_terrain = true
		elif a == "--hs-hide-ribbons":
			hide_ribbons = true
		elif a == "--hs-baseline":
			baseline = true
		elif a == "--hs-fly":
			fly = true
	var route = JSON.parse_string(FileAccess.get_file_as_string((route_file if route_file.is_absolute_path() else ProjectSettings.globalize_path(route_file))))
	points = route["waypoints"]
	points = _densify(points, 10.0) # sparse routes (1 s log samples) would leave the station search window empty
	stations.append(0.0)
	for i in range(1, points.size()):
		stations.append(stations[-1] + Vector2(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]).length())
	main = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	while main.world_state != "running":
		if main.world_state == "failed" or Time.get_ticks_msec() - began > 300000:
			_fail("world failed to become ready")
			return
		await process_frame
	var sim: Node = main.get_simulation()
	var view: Node = main.get_world_view()
	if baseline:
		main.get_director().lead_enabled = false
		view.set_upload_adaptive(false)
	if carve >= 0:
		view.set_carve_max_level(carve) # chunks built after this (the relocation's) use it
	if view_name != "":
		main.set_drive_view(view_name)
	main.debug_hide(hide_terrain, hide_ribbons)
	main.scripted_controls = {"throttle": 0.0, "brake": 1.0, "handbrake": 1.0, "steer": 0.0, "clutch": 0.0, "ignition": 1.0, "starter": 0.0, "assist.auto_shift": 1.0}
	var before: int = sim.get_streaming_status().get("relocations", 0)
	var target := _target(start_s)
	sim.relocate_vehicle(target.x, target.y, target.z)
	var mark := Time.get_ticks_msec()
	while int(sim.get_streaming_status().get("relocations", 0)) == before:
		if Time.get_ticks_msec() - mark > 60000:
			_fail("relocation timed out")
			return
		await process_frame
	# Let the view settle at the new place before measuring.
	mark = Time.get_ticks_msec()
	while not bool(view.is_stream_idle()) and Time.get_ticks_msec() - mark < 60000:
		await process_frame
	if fly:
		print("RG_HS_FLY mode -> %s" % sim.set_player_mode("fpv_drone"))
		mark = Time.get_ticks_msec()
		while sim.get_fpv_state().is_empty():
			if Time.get_ticks_msec() - mark > 30000:
				_fail("fpv drone did not spawn")
				return
			await process_frame
		mark = Time.get_ticks_msec()
		while not bool(view.is_stream_idle()) and Time.get_ticks_msec() - mark < 60000:
			await process_frame
	view.take_report_stats()
	print("RG_HS_DRIVE start station=%.0f target_speed_mps=%.0f mode=%s shots=%s" % [start_s, target_speed, "baseline" if baseline else "fix", shot_dir])
	var drive_start: float = sim.get_sim_time()
	var drive_wall := Time.get_ticks_msec()
	var station := start_s
	var freezes_before: int = int(sim.get_streaming_status().get("freeze_count", 0))
	var frozen_before: int = int(sim.get_streaming_status().get("frozen_attempts", 0))
	var l0: Array = []
	var l1: Array = []
	var above: Array = []
	var far_above: Array = []
	var bed_above: Array = []
	var bed_samples := 0
	var bed_worst := -1000.0
	var bed_worst_level := -1
	var bed_worst_carve := 0
	var speeds: Array = []
	var frame_ms: Array = []
	var next_sample := 0.0
	var max_lateral := 0.0
	var last_us := Time.get_ticks_usec()
	var max_speed := 0.0
	var shots_taken := 0
	var aborted := false
	var slow_since := -1.0
	var next_fly_log := 0.0
	if shot_dir != "":
		DirAccess.make_dir_recursive_absolute(shot_dir)
	while float(Time.get_ticks_msec() - drive_wall) / 1000.0 < seconds and station < stations[-1] - 400.0:
		await process_frame
		var now_us := Time.get_ticks_usec()
		frame_ms.append(float(now_us - last_us) / 1000.0)
		last_us = now_us
		var pos: Vector3
		var speed: float
		var heading: Vector2
		var pose: Dictionary
		var fly_state: Dictionary = {}
		if fly:
			fly_state = sim.get_fpv_state()
			if fly_state.is_empty():
				continue
			var origin: Vector3 = sim.get_render_origin_session()
			var gp: Vector3 = fly_state["position"]
			var gv: Vector3 = fly_state["velocity"]
			pos = Vector3(origin.x - gp.z, origin.y - gp.x, gp.y + origin.z)
			speed = Vector2(gv.x, gv.z).length()
			var yaw_iso := float(fly_state["yaw_rad"])
			heading = Vector2(cos(yaw_iso), sin(yaw_iso))
			pose = {"x": pos.x, "y": pos.y, "vx": -gv.z, "vy": -gv.x}
		else:
			pos = main.chassis_session_position()
			speed = sim.get_body_speed_mps("chassis")
			var forward: Vector3 = sim.get_body_transform("chassis").basis.x
			heading = Vector2(-forward.z, -forward.x).normalized()
			pose = sim.get_chassis_session_pose()
		var closest := 100000.0
		for i in range(points.size()):
			if stations[i] < station - 20 or stations[i] > station + 60:
				continue
			var distance := Vector2(pos.x - points[i][0], pos.y - points[i][1]).length()
			if distance < closest:
				closest = distance
				station = stations[i]
		max_lateral = maxf(max_lateral, closest)
		max_speed = maxf(max_speed, speed)
		var aim := _target(station + maxf(12.0, speed * 0.7))
		var direction := Vector2(aim.x - pos.x, aim.y - pos.y).normalized()
		if fly:
			var err := heading.angle_to(direction)
			var agl := float(fly_state.get("height_agl_m", 12.0))
			main.scripted_fpv_input = [0.0, clampf((12.0 - agl) * 0.5, -1.0, 1.0), clampf((target_speed - speed) * 0.25, -1.0, 1.0), clampf(err * 4.0, -1.0, 1.0), 0.0]
			# Progress log and guard: the flight must advance (100 m within 10 s of the start), else abort with a clear line.
			var fly_t := float(Time.get_ticks_msec() - drive_wall) / 1000.0
			if fly_t >= next_fly_log:
				next_fly_log += 1.0
				print("RG_HS_FLYLOG t=%.0f station=%.0f speed_mps=%.1f agl_m=%.1f lateral_m=%.1f yaw_err_rad=%.3f" % [fly_t, station, speed, agl, closest, err])
			if fly_t > 10.0 and station - start_s < 100.0:
				print("RG_HS_ABORT drone did not advance: station=%.0f speed_mps=%.1f after_s=%.1f input=%s" % [station, speed, fly_t, str(main.scripted_fpv_input)])
				aborted = true
				break
			main.scripted_controls = {"throttle": 0.0, "brake": 1.0, "handbrake": 1.0, "steer": 0.0, "clutch": 0.0, "ignition": 1.0, "starter": 0.0}
		else:
			var steer_limit := 0.35 * minf(1.0, 12.0 / maxf(speed, 12.0))
			var steer := clampf(heading.angle_to(direction) * 1.8 * minf(1.0, 12.0 / maxf(speed, 12.0)), -steer_limit, steer_limit)
			main.scripted_controls = {"throttle": clampf((target_speed - speed) * 0.15, 0.0, 1.0), "brake": clampf((speed - target_speed - 6.0) * 0.1, 0.0, 0.5),
				"handbrake": 0.0, "steer": steer, "clutch": 0.0, "ignition": 1.0, "starter": 1.0 if sim.get_sim_time() - drive_start < 1 else 0.0, "assist.auto_shift": 1.0}
			# Stuck guard: a car against a wall measures nothing about streaming at speed.
			var since_start := float(Time.get_ticks_msec() - drive_wall) / 1000.0
			if since_start > 10.0 and speed < 1.4:
				if slow_since < 0.0:
					slow_since = since_start
				elif since_start - slow_since > 5.0:
					print("RG_HS_ABORT stuck station=%.0f speed_mps=%.2f lateral_m=%.1f after_s=%.1f" % [station, speed, closest, since_start])
					aborted = true
					break
			else:
				slow_since = -1.0
		var t := float(Time.get_ticks_msec() - drive_wall) / 1000.0
		if shot_dir != "" and shots_taken < 6 and t >= shot_t0 + shots_taken * shot_step:
			shots_taken += 1
			var img: Image = root.get_viewport().get_texture().get_image()
			img.save_png(shot_dir.path_join("hs_%d_%.0fmps.png" % [shots_taken, speed]))
		if t >= next_sample:
			next_sample += 0.1
			var v := Vector2(pose.get("vx", 0.0), pose.get("vy", 0.0))
			if v.length() >= 2.0:
				var d := v.normalized()
				var cov: Dictionary = view.get_lod_coverage(pose["x"], pose["y"], d.x, d.y)
				l0.append(float(cov["l0_ahead_m"]))
				l1.append(float(cov["l1_ahead_m"]))
				speeds.append(v.length())
				var gc: Dictionary = view.get_ground_check(pose["x"], pose["y"], d.x, d.y)
				if int(gc["samples"]) > 0:
					above.append(float(gc["visible_above_m"]))
				if int(gc["far_samples"]) > 0:
					far_above.append(float(gc["far_above_m"]))
				var bc: Dictionary = view.get_bed_check(pose["x"], pose["y"], d.x, d.y, 600.0)
				if int(bc["samples"]) > 0:
					bed_above.append(float(bc["above_max_m"]))
					bed_samples += int(bc["samples"])
					if float(bc["above_max_m"]) > bed_worst:
						bed_worst = float(bc["above_max_m"])
						bed_worst_level = int(bc["worst_level"])
						bed_worst_carve = int(bc["worst_carve"])
	var ss: Dictionary = sim.get_streaming_status()
	var gc_final: Dictionary = view.get_ground_check(0.0, 0.0, 1.0, 0.0)
	var rs: Dictionary = view.take_report_stats()
	print("RG_HS_DONE aborted=%s fly=%s mode=%s station=%.0f->%.0f max_speed_mps=%.1f max_lateral_m=%.1f l0_ahead_p5_m=%.0f l0_ahead_min_m=%.0f l0_ahead_p50_m=%.0f l1_ahead_p5_m=%.0f l1_ahead_min_m=%.0f ground_above_max_m=%.3f ground_above_p95_m=%.3f ground_above_p50_m=%.3f ground_samples=%d far_above_max_m=%.3f far_above_p95_m=%.3f far_above_p50_m=%.3f far_samples=%d frame_ms_avg=%.2f frame_ms_p99=%.2f falls=%d misses=%d bed_above_max_m=%.3f bed_above_p95_m=%.3f bed_above_p50_m=%.3f bed_probes=%d bed_worst_level=%d bed_worst_carve=%d fallback_chunks_end=%d fallback_tiles_end=%d freezes=%d frozen_ticks=%d" % [
		str(aborted), str(fly), "baseline" if baseline else "fix", start_s, station, max_speed, max_lateral, _percentile(l0, 0.05), _percentile(l0, 0.0), _percentile(l0, 0.5),
		_percentile(l1, 0.05), _percentile(l1, 0.0), _percentile(above, 1.0), _percentile(above, 0.95), _percentile(above, 0.5), above.size(), _percentile(far_above, 1.0), _percentile(far_above, 0.95), _percentile(far_above, 0.5), far_above.size(),
		_avg(frame_ms), _percentile(frame_ms, 0.99), int(ss.get("falls", 0)), int(ss.get("fill_misses", 0)), bed_worst, _percentile(bed_above, 0.95), _percentile(bed_above, 0.5), bed_above.size(), bed_worst_level, bed_worst_carve, int(view.get_stream_stats().get("fallback_chunks", 0)), int(view.get_stream_stats().get("fallback_tiles", 0)), int(ss.get("freeze_count", 0)) - freezes_before, int(ss.get("frozen_attempts", 0)) - frozen_before])
	main.scripted_controls = {"throttle": 0.0, "brake": 1.0, "handbrake": 1.0, "steer": 0.0}
	main.scripted_fpv_input = []
	quit(1 if aborted else 0)

func _avg(values: Array) -> float:
	if values.is_empty():
		return 0.0
	var s := 0.0
	for v in values:
		s += float(v)
	return s / values.size()
