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
#     (checks the free rig is active and the car unattended), on to
#     drone_follow (R9b: the drone rig is active, the target is the own car,
#     the car stays Player-driven and a scripted throttle still reaches the
#     sim; then, when a traffic car or the NPC truck exists - the truck is
#     requested - cycles to it: car unattended, driving inputs dropped, two
#     physics interest points; returns to the own car; when the target is the
#     truck, removing it must fall back to the own car by itself), then the
#     on-foot round trip (R9c): getting out while the car is moving is refused,
#     after braking to a stop the cycle reaches on_foot (a walker spawns beside
#     the car, the walker rig is active, the car unattended, two interest
#     points), the walker runs a few metres away (scripted walking input), a
#     get-in attempt out of range is refused, the walker runs back to the
#     door, gets in (mode drive, one interest point, the walker gone) and the
#     car drives again; then back on drive; switches the world to flat, back to the real world, cancels that
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
const NPC_WAIT_S := 20.0 # wall time to wait for a traffic car / the truck in drone_follow
const TRUCK_ID := 4611686018427387904 # rg::Session::kNpcTruckVehicleId, 2^62
const ROUTE := "../data/routes/home_r1_drive.json"
const FOOT_PHASE_TIMEOUT_S := 25.0 # wall time per on-foot phase
const GET_OUT_MIN_SPEED_MPS := 3.0 # the car must be faster than this for the refusal check
const FOOT_RUN_S := 1.5 # wall time of the run away from the car

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
var _drone_lost_before: int = 0
var _drone_target: int = -1
var _foot_start := Vector3.ZERO
var _foot_refused_before: int = 0
var _foot_enter_refused_before: int = 0

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
				print("RG_SPEED_LIMIT spawn car=%s data=%s" % [main.VEHICLE_NAME, JSON.stringify(sim.get_vehicle_speed_limit(main.VEHICLE_NAME))])
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
				_phase = "mode_drone_own"
		"mode_drone_own":
			_frames += 1
			if _frames == 5:
				var ms: Dictionary = sim.get_mode_state()
				if str(ms.get("mode")) != "drone_follow" or main.get_director().active_name != "drone" \
						or str(ms.get("vehicle_control")) != "player" or not bool(ms.get("driving_inputs_live")) \
						or not bool(ms.get("drone_target_own")):
					_finish(1, "drone_follow (own car) mode state wrong: %s rig=%s" % [ms, main.get_director().active_name])
					return
				# The player keeps driving: a scripted throttle reaches the sim
				# (in free_cam the driving inputs were not forwarded at all).
				main.scripted_controls = {"throttle": 0.3, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0}
			elif _frames == 15:
				var thr := float(sim.get_control("throttle"))
				if absf(thr - 0.3) > 1e-6:
					_finish(1, "drone_follow own car: driving input did not reach the sim (throttle=%f)" % thr)
					return
				if sim.get_drone_target_transform() == null:
					_finish(1, "drone_follow own car: no target transform")
					return
				print("RG_DRIVE smoke mode drone_follow ok: rig=drone target=own car player-driven throttle=%.2f interest_points=%d" % [
					thr, int(sim.get_streaming_status().get("interest_points", 0))])
				main.scripted_controls = {}
				sim.request_npc_truck(true, 70.0)
				_mark_ms = Time.get_ticks_msec()
				_phase = "drone_wait_npc"
		"drone_wait_npc":
			var have_truck := bool(sim.get_npc_truck_state().get("active", false))
			var have_traffic: bool = sim.get_traffic_state().get("actors", []).size() > 0
			if have_truck or have_traffic:
				_drone_lost_before = int(sim.get_streaming_status().get("followed_lost", 0))
				_drone_target = int(sim.cycle_drone_target())
				if _drone_target < 0:
					_finish(1, "drone_follow: cycle_drone_target found no target although an NPC exists")
					return
				print("RG_DRIVE smoke drone target -> %d (truck=%s traffic=%s)" % [_drone_target, str(have_truck), str(have_traffic)])
				_frames = 0
				_phase = "mode_drone_other"
			elif (Time.get_ticks_msec() - _mark_ms) / 1000.0 >= NPC_WAIT_S:
				print("RG_DRIVE smoke drone: no NPC car or truck appeared within %.0f s - own-car path only" % NPC_WAIT_S)
				sim.request_npc_truck(false, 70.0)
				_begin_foot_check()
		"mode_drone_other":
			_frames += 1
			if _frames >= 30:
				var ms: Dictionary = sim.get_mode_state()
				var ss: Dictionary = sim.get_streaming_status()
				if bool(ms.get("drone_target_own")) or str(ms.get("vehicle_control")) != "unattended" \
						or bool(ms.get("driving_inputs_live")) or main.get_director().active_name != "drone":
					_finish(1, "drone_follow (NPC target %d) mode state wrong: %s rig=%s" % [_drone_target, ms, main.get_director().active_name])
					return
				if int(ss.get("followed_id", 0)) != _drone_target or int(ss.get("interest_points", 0)) != 2:
					_finish(1, "drone_follow (NPC target %d): expected 2 interest points, got %s" % [_drone_target, ss])
					return
				if sim.get_drone_target_transform() == null:
					_finish(1, "drone_follow (NPC target %d): no target transform" % _drone_target)
					return
				print("RG_DRIVE smoke mode drone_follow NPC ok: target=%s car=unattended interest_points=2" % ms.get("drone_target_label"))
				if _drone_target == TRUCK_ID:
					# The truck going away must return the camera to the own car by itself.
					sim.request_npc_truck(false, 70.0)
					_frames = 0
					_phase = "drone_truck_lost"
				else:
					sim.set_drone_target(-1)
					_frames = 0
					_phase = "drone_back_own"
		"drone_truck_lost":
			_frames += 1
			if _frames >= 60:
				var ms: Dictionary = sim.get_mode_state()
				var ss: Dictionary = sim.get_streaming_status()
				if not bool(ms.get("drone_target_own")) or str(ms.get("vehicle_control")) != "player" \
						or int(ss.get("followed_lost", 0)) != _drone_lost_before + 1:
					_finish(1, "drone_follow: the truck was removed but the mode did not fall back to the own car: %s lost=%s (before %d)" % [ms, ss.get("followed_lost"), _drone_lost_before])
					return
				print("RG_DRIVE smoke drone: followed truck removed -> back on the own car (followed_lost=%d)" % int(ss.get("followed_lost", 0)))
				_frames = 0
				_phase = "drone_back_own"
		"drone_back_own":
			_frames += 1
			if _frames >= 30:
				var ms: Dictionary = sim.get_mode_state()
				var ss: Dictionary = sim.get_streaming_status()
				if not bool(ms.get("drone_target_own")) or str(ms.get("vehicle_control")) != "player" \
						or not bool(ms.get("driving_inputs_live")) or int(ss.get("interest_points", 0)) != 1:
					_finish(1, "drone_follow: back on the own car, state wrong: %s interest_points=%s" % [ms, ss.get("interest_points")])
					return
				print("RG_DRIVE smoke drone_follow back on own car ok: car=player interest_points=1")
				sim.request_npc_truck(false, 70.0)
				_begin_foot_check()
		"foot_speedup":
			# Drive (drone_follow keeps the own car player-driven) until the car is
			# clearly moving, then getting out must be refused.
			if float(sim.get_body_speed_mps("chassis")) > GET_OUT_MIN_SPEED_MPS:
				var before := int(sim.get_mode_state().get("get_out_refusals", 0))
				var answer := str(sim.set_player_mode("on_foot"))
				var ms: Dictionary = sim.get_mode_state()
				if answer != "refused" or str(ms.get("mode")) != "drone_follow" or int(ms.get("get_out_refusals", 0)) != before + 1:
					_finish(1, "getting out at %.1f m/s was not refused: answer=%s state=%s" % [float(sim.get_body_speed_mps("chassis")), answer, ms])
					return
				print("RG_DRIVE smoke on_foot refused at %.1f m/s ok: mode stays drone_follow" % float(sim.get_body_speed_mps("chassis")))
				main.scripted_controls = {"throttle": 0.0, "steer": 0.0, "brake": 1.0, "handbrake": 1.0, "clutch": 0.0}
				_mark_ms = Time.get_ticks_msec()
				_phase = "foot_slowdown"
			elif _foot_timed_out():
				_finish(1, "foot_speedup: the car never exceeded %.1f m/s (speed %.2f)" % [GET_OUT_MIN_SPEED_MPS, float(sim.get_body_speed_mps("chassis"))])
		"foot_slowdown":
			if float(sim.get_body_speed_mps("chassis")) < 0.5:
				var answer := str(sim.cycle_player_mode())
				print("RG_DRIVE smoke mode -> %s" % answer)
				if answer != "on_foot":
					_finish(1, "the cycle from drone_follow did not reach on_foot at a standstill: %s" % answer)
					return
				_mark_ms = Time.get_ticks_msec()
				_phase = "foot_spawn"
			elif _foot_timed_out():
				_finish(1, "foot_slowdown: the car did not stop (speed %.2f)" % float(sim.get_body_speed_mps("chassis")))
		"foot_spawn":
			var ws: Dictionary = sim.get_walker_state()
			if not ws.is_empty():
				var ms: Dictionary = sim.get_mode_state()
				var ss: Dictionary = sim.get_streaming_status()
				if str(ms.get("mode")) != "on_foot" or main.get_director().active_name != "walker" \
						or str(ms.get("vehicle_control")) != "unattended" or bool(ms.get("driving_inputs_live")) \
						or not bool(ms.get("walking_inputs_live")):
					_finish(1, "on_foot mode state wrong: %s rig=%s" % [ms, main.get_director().active_name])
					return
				if not bool(ws.get("can_enter")) or float(ws.get("enter_distance_m")) > 1.5:
					_finish(1, "the walker did not spawn at the car's door: %s" % ws)
					return
				_frames += 1
				if _frames < 20: # a few frames: the interest points and the grounding settle
					return
				if int(ss.get("interest_points", 0)) != 2 or not bool(ws.get("grounded")) or bool(ws.get("hold")):
					_finish(1, "on_foot spawn state wrong: interest_points=%s walker=%s" % [ss.get("interest_points"), ws])
					return
				_foot_start = ws["position"]
				print("RG_DRIVE smoke on_foot ok: walker spawned %.2f m from the car, rig=walker car=unattended interest_points=2 grounded" % float(ws.get("enter_distance_m")))
				var away: Vector3 = ws["position"] - ws["car_position"]
				away.y = 0.0
				main.scripted_walk = {"move_right": 0.0, "move_forward": 1.0, "look": away.normalized(), "run": true}
				_mark_ms = Time.get_ticks_msec()
				_phase = "foot_walk"
			elif _foot_timed_out():
				_finish(1, "foot_spawn: no walker appeared (mode state %s)" % sim.get_mode_state())
		"foot_walk":
			if (Time.get_ticks_msec() - _mark_ms) / 1000.0 >= FOOT_RUN_S:
				main.scripted_walk = {"move_right": 0.0, "move_forward": 0.0, "run": false}
				var ws: Dictionary = sim.get_walker_state()
				var moved := Vector2(ws["position"].x - _foot_start.x, ws["position"].z - _foot_start.z).length()
				if moved < 2.0 or not bool(ws.get("grounded")) or bool(ws.get("hold")):
					_finish(1, "the walker did not run away: moved %.2f m walker=%s" % [moved, ws])
					return
				if bool(ws.get("can_enter")):
					_finish(1, "the walker still reports get-in range after running %.2f m" % moved)
					return
				_foot_enter_refused_before = int(sim.get_mode_state().get("walker_enter_refused", 0))
				sim.request_walker_enter()
				_frames = 0
				print("RG_DRIVE smoke on_foot walked %.1f m away (%.1f m/s)" % [moved, Vector2(ws["velocity"].x, ws["velocity"].z).length()])
				_phase = "foot_enter_far"
		"foot_enter_far":
			_frames += 1
			if _frames >= 20:
				var ms: Dictionary = sim.get_mode_state()
				if str(ms.get("mode")) != "on_foot" or int(ms.get("walker_enter_refused", 0)) != _foot_enter_refused_before + 1:
					_finish(1, "a get-in out of range was not refused: %s" % ms)
					return
				print("RG_DRIVE smoke on_foot get-in out of range refused ok")
				_mark_ms = Time.get_ticks_msec()
				_phase = "foot_return"
		"foot_return":
			var ws: Dictionary = sim.get_walker_state()
			if bool(ws.get("can_enter")):
				main.scripted_walk = {"move_right": 0.0, "move_forward": 0.0, "run": false}
				sim.request_walker_enter()
				_frames = 0
				_phase = "foot_enter"
				return
			var back: Vector3 = ws["car_position"] - ws["position"]
			back.y = 0.0
			main.scripted_walk = {"move_right": 0.0, "move_forward": 1.0, "look": back.normalized(), "run": true}
			if _foot_timed_out():
				_finish(1, "foot_return: never got within get-in range (%s)" % ws)
		"foot_enter":
			_frames += 1
			if _frames >= 30:
				var ms: Dictionary = sim.get_mode_state()
				var ss: Dictionary = sim.get_streaming_status()
				if str(ms.get("mode")) != "drive" or main.get_director().active_name != "chase" \
						or str(ms.get("vehicle_control")) != "player" or not sim.get_walker_state().is_empty() \
						or int(ss.get("interest_points", 0)) != 1:
					_finish(1, "after getting in, the state is wrong: %s interest_points=%s walker=%s" % [ms, ss.get("interest_points"), sim.get_walker_state()])
					return
				print("RG_DRIVE smoke on_foot get-in ok: mode=drive rig=chase car=player interest_points=1")
				main.scripted_walk = {}
				main.scripted_controls = {"throttle": 0.5, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0, "ignition": 1.0, "assist.auto_shift": 1.0}
				_mark_ms = Time.get_ticks_msec()
				_phase = "foot_drive"
		"foot_drive":
			if float(sim.get_body_speed_mps("chassis")) > 1.0:
				print("RG_DRIVE smoke on_foot round trip ok: the car drives again at %.1f m/s" % float(sim.get_body_speed_mps("chassis")))
				main.scripted_controls = {}
				_frames = 0
				_phase = "mode_drive"
			elif _foot_timed_out():
				_finish(1, "foot_drive: the car did not drive away after getting in (speed %.2f)" % float(sim.get_body_speed_mps("chassis")))
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

# Wall-time guard for one on-foot phase (drive_smoke's own TIMEOUT_S covers the whole run).
func _foot_timed_out() -> bool:
	return (Time.get_ticks_msec() - _mark_ms) / 1000.0 > FOOT_PHASE_TIMEOUT_S

func _begin_foot_check() -> void:
	main.scripted_controls = {"throttle": 1.0, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0, "ignition": 1.0, "assist.auto_shift": 1.0}
	_mark_ms = Time.get_ticks_msec()
	_frames = 0
	_phase = "foot_speedup"

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
