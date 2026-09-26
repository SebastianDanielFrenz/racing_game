extends Node
# game/scripts/drive_tour.gd — `--screenshots <dir>` (with or without
# --drive - see carvis brief 2026-09-26, "one on the flat scene": run with no
# --drive and the whole tour below runs unattended in the flat test scene
# instead, since world_state is already "running" by the time this node is
# added - the "loading" phase's `elif world_state == "running"` branch below
# already covers that, and RgTerrainView.is_fully_uploaded()/get_chunk_count()
# are both well-defined - trivially true/0 - on a view nothing ever
# initialize_shared()'d). Needs a real renderer (not --headless).
#  05_loading_overlay.png  the loading overlay, LOADING_SHOT_S into the load
#                          (skipped, and logged, if the load was faster - the
#                          flat world never shows one)
#  01_spawn_chase.png      the car at the spawn, chase cam, SETTLE_S of sim
#                          time after the world became drivable, with every
#                          initial terrain chunk uploaded
#  02_steering_close.png   car held stationary (brake+handbrake), full steer
#                          lock applied - front wheels turned, chase cam
#                          swung to a closer 3/4-rear angle (side_offset_m)
#                          so the near-side front wheel is actually visible
#                          (a dead-on rear view hides it behind the body)
#  03_drive_chase.png      after DRIVE_S of sim time at throttle 0.5, steering
#                          straight (main.scripted_controls - no autopilot)
#  04_free_cam_above.png   FreeCam mode (the car goes unattended and brakes; shot once stopped),
#                          the free rig placed above and behind the car,
#                          looking down at it
# Writes poses.txt with the numbers behind each shot (chassis session
# position, speed, wheel loads, engine state) and quits 0; quits 1 if the
# world fails to load or TIMEOUT_S passes.

const LOADING_SHOT_S := 0.6
const SETTLE_S := 2.0
const STEER_S := 1.0
const DRIVE_S := 4.0
const SHOT_FRAMES := 8
const TIMEOUT_S := 240.0
const FREE_BACK_M := 14.0
const FREE_UP_M := 22.0
# Close-up framing for 02_steering_close.png: the SAME chase_rig.gd formula
# used for every other chase shot (already proven correct there), just
# parameterized closer/lower AND with a lateral offset (chase_rig.gd's
# side_offset_m, added 2026-09-27 for exactly this shot) - a directly-behind
# camera can never show a front wheel at all (the body is face-on and
# occludes it completely, whatever the distance/height); a 3/4 angle is
# needed to see the near-side wheels' profile and steer angle.
#
# 2026-09-27 history: the first cut (distance 2.6, height 0.55, look_height
# 0.15, no side offset) put the camera INSIDE the car - car_sedan's own rig
# (car_sedan.rig.json overall_bounds) spans local x [-2.28, 2.3041] about its
# ground-plane origin, and the chassis rigid body's own origin sits ~0.27 m
# off that same axle-midpoint (data/vehicles/car_sedan.json wheel
# attachment_local: front +1.08, rear -1.62, mean -0.27) - so the visible
# body's rear extent from the CHASSIS origin chase_rig measures "behind" from
# is close to -2.0..-2.55 m, well past a 2.6 m follow_distance_m. Widening the
# distance alone (verified via a screenshot) fixed the clipping but was still
# a dead-on rear view with both front wheels hidden behind the body. Adding
# side_offset_m and aiming this shot from behind-and-to-one-side (a
# conventional "3/4 rear" angle) is what actually reveals a front wheel.
const CLOSE_DISTANCE_M := 3.4
const CLOSE_SIDE_M := 3.2
const CLOSE_HEIGHT_M := 1.1
const CLOSE_LOOK_HEIGHT_M := 0.25

var main: Node
var out_dir: String

var _phase := "loading"
var _started_ms: int = 0
var _frames: int = 0
var _log := PackedStringArray()
var _drive_start_t: float = 0.0
var _saved_rig_params: Dictionary = {}

func _ready() -> void:
	_started_ms = Time.get_ticks_msec()
	DirAccess.make_dir_recursive_absolute(out_dir)

# car_sedan's own wheel_radius (data/vehicles/car_sedan.json, same for all 4
# corners) - RgSimulation exposes attachment_local/compression but not this,
# and it is not worth a seventh accessor for one diagnostic log line; ride
# height below is a car_sedan-specific check anyway (VEHICLE_NAME already is).
const WHEEL_RADIUS_M := 0.317

func _numbers() -> String:
	var sim: Node = main.get_simulation()
	var p: Vector3 = main.chassis_session_position()
	var loads := PackedStringArray()
	var ride_heights := PackedStringArray()
	var load_sum := 0.0
	for i in range(sim.get_vehicle_wheel_count(main.VEHICLE_NAME)):
		var l: float = sim.get_wheel_load_n(main.VEHICLE_NAME, i)
		load_sum += l
		loads.append("%.0f" % l)
		# Wheel-bottom height above the local ground plane: chassis origin
		# height (p.z, session-relative) + attachment_local.z (wheel centre at
		# full suspension extension) + compression (get_wheel_compression,
		# WheelState::suspension_travel - 0 at full extension, grows as the
		# spring compresses toward the chassis) - wheel_radius. ~0 means the
		# tyre is touching the ground; positive is a gap (floating), negative
		# is penetration (sunk) - see carvis brief 2026-09-26's ride-height
		# requirement.
		var attach: Vector3 = sim.get_wheel_attachment_local(main.VEHICLE_NAME, i)
		var compression: float = sim.get_wheel_compression(main.VEHICLE_NAME, i)
		var ride_height: float = p.z + attach.z + compression - WHEEL_RADIUS_M
		ride_heights.append("%.4f" % ride_height)
	var pt: Dictionary = sim.get_vehicle_powertrain(main.VEHICLE_NAME)
	var cam: Camera3D = main.get_director().active_camera()
	return "session=(%.2f, %.2f, %.2f) speed_kmh=%.1f wheel_loads_n=[%s] sum=%.0f wheel_ride_height_m=[%s] engine=%s gear=%d rpm=%.0f surface=%s mode=%s cam_godot=%s chunks=%d fps=%.1f" % [
		p.x, p.y, p.z, sim.get_body_speed_mps("chassis") * 3.6, ", ".join(loads), load_sum, ", ".join(ride_heights),
		pt.get("engine_state", "?"), int(pt.get("gear", 0)), float(pt.get("rpm", 0.0)),
		sim.get_wheel_surface_name(main.VEHICLE_NAME, 0), sim.get_player_mode(),
		cam.global_position if cam != null else Vector3.ZERO, int(main.get_world_view().get_chunk_count()),
		Engine.get_frames_per_second()]

func _shot(file_name: String, numbers: String) -> void:
	var path := out_dir.path_join(file_name)
	var err := get_viewport().get_texture().get_image().save_png(path)
	_log.append("%s err=%d %s" % [file_name, err, numbers])
	print("drive_tour: wrote %s  %s" % [path, numbers])

func _quit(code: int) -> void:
	var f := FileAccess.open(out_dir.path_join("poses.txt"), FileAccess.WRITE)
	if f != null:
		f.store_string("\n".join(_log) + "\n")
	_phase = "quit"
	get_tree().quit(code)

func _process(_delta: float) -> void:
	if _phase == "quit":
		return
	var elapsed := (Time.get_ticks_msec() - _started_ms) / 1000.0
	if elapsed > TIMEOUT_S or main.world_state == "failed":
		push_error("drive_tour: world not drivable (state %s after %.0f s)" % [main.world_state, elapsed])
		_quit(1)
		return
	var sim: Node = main.get_simulation()
	match _phase:
		"loading":
			if main.world_state == "loading" and elapsed >= LOADING_SHOT_S:
				var st: Dictionary = sim.get_init_status()
				_shot("05_loading_overlay.png", "elapsed_s=%.2f stage=%s resident_l0=%d missing_required=%d inflight=%d prime=%d/%d" % [
					elapsed, st.get("stage", "?"), int(st.get("resident_l0", 0)), int(st.get("missing_required", 0)),
					int(st.get("inflight", 0)), int(st.get("prime_done", 0)), int(st.get("prime_total", 0))])
				_phase = "settle"
			elif main.world_state == "running":
				_log.append("05_loading_overlay.png not captured: the load finished in %.2f s (< %.1f s), or a flat-world run has no loading phase" % [elapsed, LOADING_SHOT_S])
				_phase = "settle"
		"settle":
			if main.world_state == "running" and bool(main.get_world_view().is_fully_uploaded()) \
					and float(sim.get_sim_time()) - main.ready_sim_time >= SETTLE_S:
				_frames += 1
				if _frames >= SHOT_FRAMES:
					_shot("01_spawn_chase.png", _numbers())
					# Held stationary (brake+handbrake), full steer lock - proves
					# the front wheels turn under a real per-wheel steer angle
					# (WheelState::steer_angle via get_wheel_steer_angle), not
					# just that the chassis moves.
					main.scripted_controls = {"throttle": 0.0, "steer": 0.6, "brake": 1.0, "handbrake": 1.0, "clutch": 0.0}
					_drive_start_t = float(sim.get_sim_time())
					_frames = 0
					_phase = "steer_close"
		"steer_close":
			if float(sim.get_sim_time()) - _drive_start_t >= STEER_S:
				# Shrink the SAME chase_rig and swing it out to one side (a 3/4
				# rear angle - see CLOSE_DISTANCE_M's own comment above for why a
				# dead-on rear view can never show a front wheel), rather than a
				# bespoke camera placement.
				var rig: Node3D = main.get_director().active_rig()
				if rig != null:
					if _saved_rig_params.is_empty():
						_saved_rig_params = {
							"follow_distance_m": rig.follow_distance_m,
							"follow_height_m": rig.follow_height_m,
							"look_height_m": rig.look_height_m,
							"side_offset_m": rig.side_offset_m,
						}
						rig.follow_distance_m = CLOSE_DISTANCE_M
						rig.follow_height_m = CLOSE_HEIGHT_M
						rig.look_height_m = CLOSE_LOOK_HEIGHT_M
						rig.side_offset_m = CLOSE_SIDE_M
						# activate() resets chase_rig's own _initialized flag, so its
						# next update_rig() SNAPS straight to the new (closer) desired
						# position instead of the usual exponential lag slowly
						# catching up over many frames (SHOT_FRAMES=8 is too few for
						# position_lag=6.0's lerp to visibly close a 7m -> 3.4m gap).
						rig.activate(rig.transform)
				_frames += 1
				if _frames >= SHOT_FRAMES:
					_shot("02_steering_close.png", _numbers())
					if rig != null and not _saved_rig_params.is_empty():
						rig.follow_distance_m = _saved_rig_params["follow_distance_m"]
						rig.follow_height_m = _saved_rig_params["follow_height_m"]
						rig.look_height_m = _saved_rig_params["look_height_m"]
						rig.side_offset_m = _saved_rig_params["side_offset_m"]
					main.scripted_controls = {"throttle": 0.5, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0}
					_drive_start_t = float(sim.get_sim_time())
					_frames = 0
					_phase = "drive"
		"drive":
			if float(sim.get_sim_time()) - _drive_start_t >= DRIVE_S:
				_shot("03_drive_chase.png", _numbers())
				main.scripted_controls = {}
				sim.set_player_mode("free_cam")
				_frames = 0
				_phase = "free_place"
		"free_place":
			_frames += 1
			# main.gd has applied the mode (the free rig is active); the
			# unattended car brakes - place the camera once it has stopped.
			if _frames >= 2 and (sim.get_body_speed_mps("chassis") < 0.5 or _frames > 900):
				var chassis: Transform3D = sim.get_body_transform("chassis")
				var fwd: Vector3 = chassis.basis.x
				fwd.y = 0.0
				fwd = fwd.normalized()
				var rig: Node3D = main.get_director().active_rig()
				var pos: Vector3 = chassis.origin - fwd * FREE_BACK_M + Vector3.UP * FREE_UP_M
				rig.place(pos, atan2(-fwd.x, -fwd.z), -atan2(FREE_UP_M, FREE_BACK_M))
				_frames = 0
				_phase = "free_shot"
		"free_shot":
			_frames += 1
			if _frames >= SHOT_FRAMES:
				_shot("04_free_cam_above.png", _numbers())
				_quit(0)
