extends Node
# game/scripts/drive_tour.gd — `--drive --screenshots <dir>`: the R9 proof
# shots for the owner. Needs a real renderer (not --headless).
#  04_loading_overlay.png  the loading overlay, LOADING_SHOT_S into the load
#                          (skipped, and logged, if the load was faster)
#  01_spawn_chase.png      the car at the spawn, chase cam, SETTLE_S of sim
#                          time after the world became drivable, with every
#                          initial terrain chunk uploaded
#  02_drive_chase.png      after DRIVE_S of sim time at throttle 0.5, steering
#                          straight (main.scripted_controls - no autopilot)
#  03_free_cam_above.png   FreeCam mode (the car goes unattended and brakes; shot once stopped),
#                          the free rig placed above and behind the car,
#                          looking down at it
# Writes poses.txt with the numbers behind each shot (chassis session
# position, speed, wheel loads, engine state) and quits 0; quits 1 if the
# world fails to load or TIMEOUT_S passes.

const LOADING_SHOT_S := 0.6
const SETTLE_S := 2.0
const DRIVE_S := 4.0
const SHOT_FRAMES := 8
const TIMEOUT_S := 240.0
const FREE_BACK_M := 14.0
const FREE_UP_M := 22.0

var main: Node
var out_dir: String

var _phase := "loading"
var _started_ms: int = 0
var _frames: int = 0
var _log := PackedStringArray()
var _drive_start_t: float = 0.0

func _ready() -> void:
	_started_ms = Time.get_ticks_msec()
	DirAccess.make_dir_recursive_absolute(out_dir)

func _numbers() -> String:
	var sim: Node = main.get_simulation()
	var p: Vector3 = main.chassis_session_position()
	var loads := PackedStringArray()
	var load_sum := 0.0
	for i in range(sim.get_vehicle_wheel_count(main.VEHICLE_NAME)):
		var l: float = sim.get_wheel_load_n(main.VEHICLE_NAME, i)
		load_sum += l
		loads.append("%.0f" % l)
	var pt: Dictionary = sim.get_vehicle_powertrain(main.VEHICLE_NAME)
	var cam: Camera3D = main.get_director().active_camera()
	return "session=(%.2f, %.2f, %.2f) speed_kmh=%.1f wheel_loads_n=[%s] sum=%.0f engine=%s gear=%d rpm=%.0f surface=%s mode=%s cam_godot=%s chunks=%d fps=%.1f" % [
		p.x, p.y, p.z, sim.get_body_speed_mps("chassis") * 3.6, ", ".join(loads), load_sum,
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
				_shot("04_loading_overlay.png", "elapsed_s=%.2f stage=%s resident_l0=%d missing_required=%d inflight=%d prime=%d/%d" % [
					elapsed, st.get("stage", "?"), int(st.get("resident_l0", 0)), int(st.get("missing_required", 0)),
					int(st.get("inflight", 0)), int(st.get("prime_done", 0)), int(st.get("prime_total", 0))])
				_phase = "settle"
			elif main.world_state == "running":
				_log.append("04_loading_overlay.png not captured: the load finished in %.2f s (< %.1f s)" % [elapsed, LOADING_SHOT_S])
				_phase = "settle"
		"settle":
			if main.world_state == "running" and bool(main.get_world_view().is_fully_uploaded()) \
					and float(sim.get_sim_time()) - main.ready_sim_time >= SETTLE_S:
				_frames += 1
				if _frames >= SHOT_FRAMES:
					_shot("01_spawn_chase.png", _numbers())
					main.scripted_controls = {"throttle": 0.5, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0}
					_drive_start_t = float(sim.get_sim_time())
					_frames = 0
					_phase = "drive"
		"drive":
			if float(sim.get_sim_time()) - _drive_start_t >= DRIVE_S:
				_shot("02_drive_chase.png", _numbers())
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
				_shot("03_free_cam_above.png", _numbers())
				_quit(0)
