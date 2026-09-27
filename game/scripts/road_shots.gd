extends Node
# game/scripts/road_shots.gd — roads_plan.md R-4 (owner request 2026-09-27:
# "Next time I test drive, I at least want to see where roads are."):
# `--drive --road-shots <dir>` relocates the car to four points along the
# committed data/routes/home_r1_drive.json and takes one chase-cam screenshot
# at each, so the owner can see (and this session can honestly check) whether
# a road actually runs under the car and is visible ahead - roads_plan.md
# R-2's OSM road classes painted onto the render chunks.
#  05_road_b8_junction.png   s ~ 150 m: turning from Koenigsteiner Strasse onto the B 8
#  06_road_bridge.png        s ~ 2870 m: the route's only OSM-tagged bridge (source
#                            field: s 2847-2888 m, 13 m of B 8)
#  07_road_forest.png        s ~ 6000 m: a forested stretch of the B 8 trunk road
#  08_road_konigstein.png    s ~ 9800 m: approaching Koenigstein (the route's end,
#                            9854.5 m total)
# Route points are found by walking the route's own waypoints (same Euclidean
# arc-length convention rg::route_check uses) to the target s, same idea as
# drive_smoke.gd's _relocate_target but for an arbitrary s, and reused for all
# four targets in one run. RgSimulation.relocate_vehicle lands the car at rest
# (rg::Session::request_relocate); chase_rig.gd snaps instead of lagging after
# a jump bigger than SNAP_M (60 m - every one of these jumps is), so no extra
# camera settle time is needed beyond a few frames for the terrain view's own
# streaming gate.
#
# Writes poses.txt (session position/speed/surface per shot, one line each)
# and quits 0 once all four shots are taken; quits 1 on a relocation failure,
# a missing route point, or TIMEOUT_S.
#
# Coordinator review (2026-09-27, after the first four shots showed no road at
# s=6000/s=9800): the "settle" gate below used to check only
# RgTerrainView.is_fully_uploaded() (its own upload QUEUE is drained), which
# reads true even while TerrainViewStreamer is still BUILDING the new focus's
# chunk set in the background - is_fully_uploaded() only reflects whatever
# chunks_ already holds (the OLD, pre-relocate set, still fully uploaded)
# until the new diff actually arrives. A cold build for a never-before-visited
# region (every target past spawn/s=150 - see CLAUDE.md's "Visible roads
# (R-2)") can take single-digit seconds; SHOT_FRAMES=12 frames is nowhere
# near enough for that, so the shot was taken while still looking at stale
# (pre-relocate) terrain. Fixed by gating on RgTerrainView.is_stream_idle()
# instead (also requires the streamer's own phase == Idle, i.e. the new
# diff has actually been applied) and logging get_stream_stats() into
# poses.txt for verification.

const ROUTE := "../data/routes/home_r1_drive.json"
const SHOT_FRAMES := 12
const TIMEOUT_S := 240.0

const TARGETS := [
	{"s": 150.0, "name": "05_road_b8_junction.png"},
	{"s": 2870.0, "name": "06_road_bridge.png"},
	{"s": 6000.0, "name": "07_road_forest.png"},
	{"s": 9800.0, "name": "08_road_konigstein.png"},
]

var main: Node
var out_dir: String

var _phase := "wait_ready"
var _started_ms: int = 0
var _target_index: int = 0
var _frames: int = 0
var _log := PackedStringArray()
var _waypoints: Array = []
var _relocations_before: int = 0
var _failures_before: int = 0

func _ready() -> void:
	_started_ms = Time.get_ticks_msec()
	DirAccess.make_dir_recursive_absolute(out_dir)
	var path := ProjectSettings.globalize_path("res://").path_join(ROUTE).simplify_path()
	var text := FileAccess.get_file_as_string(path)
	var route = JSON.parse_string(text)
	if typeof(route) == TYPE_DICTIONARY and route.has("waypoints"):
		_waypoints = route["waypoints"]

# [x, y, yaw_deg] at cumulative arc length target_s along _waypoints
# (Euclidean distance between consecutive waypoints - same convention
# rg::route_check's own arc length uses); empty if _waypoints is empty or
# target_s is negative.
func _point_at_s(target_s: float) -> Array:
	if _waypoints.size() < 2 or target_s < 0.0:
		return []
	var along := 0.0
	for i in range(1, _waypoints.size()):
		var a := Vector2(_waypoints[i - 1][0], _waypoints[i - 1][1])
		var b := Vector2(_waypoints[i][0], _waypoints[i][1])
		var seg := a.distance_to(b)
		if along + seg >= target_s or i == _waypoints.size() - 1:
			var t := 0.0 if seg <= 0.0 else clampf((target_s - along) / seg, 0.0, 1.0)
			var p := a.lerp(b, t)
			var yaw_deg := rad_to_deg(atan2(b.y - a.y, b.x - a.x))
			return [p.x, p.y, yaw_deg]
		along += seg
	return []

func _numbers() -> String:
	var sim: Node = main.get_simulation()
	var p: Vector3 = main.chassis_session_position()
	var stream: Dictionary = main.get_world_view().get_stream_stats()
	return ("session=(%.2f, %.2f, %.2f) speed_kmh=%.1f surface=%s chunks=%d resident_l0=%d " +
			"stream_idle=%d streamer_phase=%d diffs_applied=%d selections_started=%d " +
			"last_build_ms=%.1f settle_frames=%d") % [
		p.x, p.y, p.z, sim.get_body_speed_mps("chassis") * 3.6,
		sim.get_wheel_surface_name(main.VEHICLE_NAME, 0), int(main.get_world_view().get_chunk_count()),
		int(sim.get_streaming_status().get("resident_l0", 0)),
		int(main.get_world_view().is_stream_idle()), int(stream.get("streamer_phase", 0)),
		int(stream.get("diffs_applied", 0)), int(stream.get("selections_started", 0)),
		float(stream.get("last_build_ms", 0.0)), _frames]

func _shot(file_name: String, target_s: float, numbers: String) -> void:
	var path := out_dir.path_join(file_name)
	var err := get_viewport().get_texture().get_image().save_png(path)
	_log.append("%s s=%.0f err=%d %s" % [file_name, target_s, err, numbers])
	print("road_shots: wrote %s (s=%.0f)  %s" % [path, target_s, numbers])

func _quit(code: int) -> void:
	var f := FileAccess.open(out_dir.path_join("poses.txt"), FileAccess.WRITE)
	if f != null:
		f.store_string("\n".join(_log) + "\n")
	_phase = "quit"
	get_tree().quit(code)

func _begin_relocate() -> void:
	var target: Dictionary = TARGETS[_target_index]
	var t := _point_at_s(target["s"])
	if t.is_empty():
		push_error("road_shots: no route point for s=%.0f (target %s)" % [target["s"], target["name"]])
		_quit(1)
		return
	var ss: Dictionary = main.get_simulation().get_streaming_status()
	_relocations_before = int(ss.get("relocations", 0))
	_failures_before = int(ss.get("relocate_failures", 0))
	main.get_simulation().relocate_vehicle(t[0], t[1], t[2])
	print("road_shots: relocating to s=%.0f (%.1f, %.1f) yaw_deg=%.1f for %s" % [target["s"], t[0], t[1], t[2], target["name"]])
	_frames = 0
	_phase = "relocating"

func _process(_delta: float) -> void:
	if _phase == "quit":
		return
	var elapsed := (Time.get_ticks_msec() - _started_ms) / 1000.0
	if elapsed > TIMEOUT_S or main.world_state == "failed":
		push_error("road_shots: world not drivable (state %s after %.0f s)" % [main.world_state, elapsed])
		_quit(1)
		return
	var sim: Node = main.get_simulation()
	match _phase:
		"wait_ready":
			if main.world_state == "running" and main.world_kind == "real_world" \
					and bool(main.get_world_view().is_stream_idle()):
				main.scripted_controls = {"throttle": 0.0, "steer": 0.0, "brake": 1.0, "handbrake": 1.0, "clutch": 0.0}
				_begin_relocate()
		"relocating":
			var ss: Dictionary = sim.get_streaming_status()
			if int(ss.get("relocate_failures", 0)) > _failures_before:
				push_error("road_shots: relocation to %s found no ground" % TARGETS[_target_index]["name"])
				_quit(1)
				return
			if int(ss.get("relocations", 0)) > _relocations_before:
				_frames = 0
				_phase = "settle"
		"settle":
			if bool(main.get_world_view().is_stream_idle()):
				_frames += 1
				if _frames >= SHOT_FRAMES:
					var target: Dictionary = TARGETS[_target_index]
					_shot(target["name"], target["s"], _numbers())
					_target_index += 1
					if _target_index >= TARGETS.size():
						_quit(0)
					else:
						_begin_relocate()
			else:
				_frames = 0 # a coalesced re-selection reopened the gate - restart the settle count
