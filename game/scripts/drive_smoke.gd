extends Node
# game/scripts/drive_smoke.gd — `--drive --drive-smoke` (tools/smoke_test.ps1
# -Drive / -DriveDelayMs): a scripted, headless-capable real-world drive.
#  1. waits for main.gd's world to become "running" (quits 1 if it fails);
#  2. drives straight at throttle 0.5 for DRIVE_S seconds of sim time
#     (main.scripted_controls replaces the player's driving inputs; auto-shift
#     picks 1st from neutral);
#  3. flips the car upright IN PLACE (RgSimulation.flip_vehicle_upright) and
#     waits: checks the streaming status's relocations counter advances and
#     the car stayed close to where it was (unlike a reset-to-spawn) - the
#     car is already upright here (this script has no way to roll it over),
#     so this is an end-to-end wiring check of the binding, not a coverage
#     test of the yaw recovery itself (that is tests/unit/test_session.cpp's
#     job);
#  4. only with --g2m-fetch-delay-ms: relocates the car (RgSimulation.
#     relocate_vehicle) to the waypoint RELOCATE_ALONG_M along
#     data/routes/home_r1_drive.json, facing the next waypoint. Those tiles are
#     not resident and every fetch is delayed, so the terrain gate must freeze
#     the clock; waits for the relocation to land, then checks that the tick
#     count advances again over AFTER_S seconds of wall time;
#  5. runtime switching, no restart: cycles the player mode to free_cam
#     (checks the free rig is active and the car unattended) and back to
#     drive; switches the world to flat, back to the real world, cancels that
#     load CANCEL_AFTER_FRAMES frames in with another switch (prints how long the
#     cancel-then-join took), and switches to the real world once more;
#  6. prints one "RG_DRIVE done ..." line (main.gd's status_line numbers of
#     the final real-world Session - a NEW Session after the round trip, so
#     its own freezes start at 0 - plus drove_m / freezes_before /
#     relocate_freezes (the first Session's freeze_count once the relocation
#     landed; -1 without a fetch delay) / advanced_after_relocate / switches /
#     cancel_ms) and quits 0.
# The smoke's assertions (falls=0 misses=0 on every RG_DRIVE numbers line,
# ticks > 0 at the end, freezes >= 1, ticks advancing) are made by
# tools/smoke_test.ps1; this script quits 1 only when something cannot
# proceed (load failure, a mode/world check failing, timeout).

const DRIVE_S := 6.0
const FLIP_WAIT_S := 1.0
const RELOCATE_ALONG_M := 3000.0
const AFTER_S := 3.0
const CANCEL_AFTER_FRAMES := 3 # a warm-cache load takes ~0.4 s: cancel well inside it
const TIMEOUT_S := 300.0
const ROUTE := "../data/routes/home_r1_drive.json"

var main: Node

var _phase := "wait_ready"
var _started_ms: int = 0
var _mark_ms: int = 0
var _mark_ticks: int = 0
var _drive_start := Vector3.ZERO
var _drove_m: float = 0.0
var _flip_before_pos := Vector3.ZERO
var _flip_relocations_before: int = 0
var _freezes_before: int = 0
var _relocate_before: int = 0
var _relocate_freezes: int = -1 # freeze_count once the relocation landed (the final Session is a new one)
var _advanced: int = -1
var _switches: int = 0
var _cancel_ms: int = -1
var _frames: int = 0

func _ready() -> void:
	_started_ms = Time.get_ticks_msec()

func _finish(code: int, why: String) -> void:
	if code != 0:
		push_error("drive smoke: %s" % why)
	main.scripted_controls = {}
	var line: String = main.status_line("RG_DRIVE done") if main.get_simulation() != null else "RG_DRIVE done"
	print("%s drove_m=%.1f freezes_before=%d relocate_freezes=%d advanced_after_relocate=%d switches=%d cancel_ms=%d result=%s" % [
		line, _drove_m, _freezes_before, _relocate_freezes, _advanced, _switches, _cancel_ms, "ok" if code == 0 else why])
	_phase = "quit"
	get_tree().quit(code)

func _relocate_target() -> Array:
	var path := ProjectSettings.globalize_path("res://").path_join(ROUTE).simplify_path()
	var text := FileAccess.get_file_as_string(path)
	var route = JSON.parse_string(text)
	if typeof(route) != TYPE_DICTIONARY or not route.has("waypoints"):
		return []
	var w: Array = route["waypoints"]
	var along := 0.0
	for i in range(1, w.size() - 1):
		along += Vector2(w[i][0], w[i][1]).distance_to(Vector2(w[i - 1][0], w[i - 1][1]))
		if along >= RELOCATE_ALONG_M:
			var yaw_deg := rad_to_deg(atan2(float(w[i + 1][1]) - float(w[i][1]), float(w[i + 1][0]) - float(w[i][0])))
			return [float(w[i][0]), float(w[i][1]), yaw_deg, along]
	return []

func _process(_delta: float) -> void:
	if _phase == "quit":
		return
	var sim: Node = main.get_simulation()
	if (Time.get_ticks_msec() - _started_ms) / 1000.0 > TIMEOUT_S:
		_finish(1, "timeout in phase %s" % _phase)
		return
	match _phase:
		"wait_ready":
			if main.world_state == "failed":
				_finish(1, "world did not load")
			elif main.world_state == "running" and main.world_kind == "real_world":
				main.scripted_controls = {"throttle": 0.5, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0}
				_drive_start = main.chassis_session_position()
				_phase = "drive"
		"drive":
			if float(sim.get_sim_time()) - main.ready_sim_time >= DRIVE_S:
				var p: Vector3 = main.chassis_session_position()
				_drove_m = Vector2(p.x, p.y).distance_to(Vector2(_drive_start.x, _drive_start.y))
				print(main.status_line("RG_DRIVE smoke drove %.1f m in %.1f s:" % [_drove_m, DRIVE_S]))
				# Flip upright IN PLACE, already upright (this script has no
				# way to roll the car over) - an end-to-end wiring check that
				# the binding relocates without sending the car back to spawn.
				main.scripted_controls = {}
				_flip_before_pos = p
				_flip_relocations_before = int(sim.get_streaming_status().get("relocations", 0))
				sim.flip_vehicle_upright()
				_mark_ms = Time.get_ticks_msec()
				_phase = "flip_upright"
		"flip_upright":
			if (Time.get_ticks_msec() - _mark_ms) / 1000.0 >= FLIP_WAIT_S:
				var ss0: Dictionary = sim.get_streaming_status()
				if int(ss0.get("relocations", 0)) < _flip_relocations_before + 1:
					_finish(1, "flip upright did not relocate (relocations=%d)" % int(ss0.get("relocations", 0)))
					return
				var p2: Vector3 = main.chassis_session_position()
				var moved := Vector2(p2.x, p2.y).distance_to(Vector2(_flip_before_pos.x, _flip_before_pos.y))
				if moved > 5.0:
					_finish(1, "flip upright moved the car %.1f m (expected in place)" % moved)
					return
				print("RG_DRIVE smoke flip upright ok: stayed within %.2f m" % moved)
				if main.fetch_delay_ms <= 0:
					_begin_mode_check(sim)
					return
				var target := _relocate_target()
				if target.is_empty():
					_finish(1, "no relocation target in %s" % ROUTE)
					return
				_freezes_before = int(ss0.get("freeze_count", 0))
				_relocate_before = int(ss0.get("relocations", 0))
				print("RG_DRIVE smoke relocating to (%.1f, %.1f) yaw_deg=%.1f, %.0f m along the route, freezes so far %d" % [
					target[0], target[1], target[2], target[3], _freezes_before])
				sim.relocate_vehicle(target[0], target[1], target[2])
				_phase = "relocating"
		"relocating":
			var ss: Dictionary = sim.get_streaming_status()
			if int(ss.get("relocate_failures", 0)) > 0:
				_finish(1, "relocation found no ground")
			elif int(ss.get("relocations", 0)) > _relocate_before:
				print(main.status_line("RG_DRIVE smoke relocation landed:"))
				_relocate_freezes = int(ss.get("freeze_count", 0))
				_mark_ticks = int(sim.get_step_count())
				_mark_ms = Time.get_ticks_msec()
				_phase = "after"
		"after":
			if (Time.get_ticks_msec() - _mark_ms) / 1000.0 >= AFTER_S:
				_advanced = int(sim.get_step_count()) - _mark_ticks
				_begin_mode_check(sim)
		"mode_free":
			_frames += 1
			if _frames >= 5:
				var ms: Dictionary = sim.get_mode_state()
				if str(ms.get("mode")) != "free_cam" or main.get_director().active_name != "free" 						or str(ms.get("vehicle_control")) != "unattended" or bool(ms.get("driving_inputs_live")):
					_finish(1, "free_cam mode state wrong: %s rig=%s" % [ms, main.get_director().active_name])
					return
				print("RG_DRIVE smoke mode free_cam ok: rig=free car=unattended")
				print("RG_DRIVE smoke mode -> %s" % sim.cycle_player_mode())
				_frames = 0
				_phase = "mode_drive"
		"mode_drive":
			_frames += 1
			if _frames >= 5:
				var ms: Dictionary = sim.get_mode_state()
				if str(ms.get("mode")) != "drive" or main.get_director().active_name != "chase" 						or str(ms.get("vehicle_control")) != "player":
					_finish(1, "drive mode state wrong: %s rig=%s" % [ms, main.get_director().active_name])
					return
				print("RG_DRIVE smoke mode drive ok: rig=chase car=player")
				_switch("to_flat")
		"to_flat":
			if _running("flat"):
				print("RG_DRIVE smoke switched to flat: ticks=%d mode=%s" % [sim.get_step_count(), sim.get_player_mode()])
				_switch("to_real_cancel")
				_mark_ms = Time.get_ticks_msec()
				_frames = 0
		"to_real_cancel":
			_frames += 1
			if main.world_state == "loading" and _frames >= CANCEL_AFTER_FRAMES:
				var t0 := Time.get_ticks_usec()
				_switch("to_flat_again")
				_cancel_ms = int((Time.get_ticks_usec() - t0) / 1000)
				print("RG_DRIVE smoke cancelled a real-world load %d ms in: the switch returned in %d ms" % [Time.get_ticks_msec() - _mark_ms, _cancel_ms])
			elif main.world_state != "loading":
				_finish(1, "real-world load finished before it could be cancelled (%s)" % main.world_state)
		"to_flat_again":
			if _running("flat"):
				_switch("to_real")
		"to_real":
			if _running("real_world"):
				print(main.status_line("RG_DRIVE smoke switched back to the real world:"))
				_finish(0, "")

func _begin_mode_check(sim: Node) -> void:
	main.scripted_controls = {}
	print("RG_DRIVE smoke mode -> %s" % sim.cycle_player_mode())
	_frames = 0
	_phase = "mode_free"

func _switch(next_phase: String) -> void:
	main.switch_world()
	_switches += 1
	_phase = next_phase

func _running(kind: String) -> bool:
	if main.world_state == "failed":
		_finish(1, "%s world did not load after a switch" % kind)
		return false
	return main.world_kind == kind and main.world_state == "running" and int(main.get_simulation().get_step_count()) > 0
