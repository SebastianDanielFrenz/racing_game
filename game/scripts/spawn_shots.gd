extends Node
# game/scripts/spawn_shots.gd - `--drive --spawn-shots <dir>`: z-fighting diagnosis at the spawn (owner 2026-10-19,
# Unterliederbach). Once the world is drivable and the streamed terrain settled it takes the same chase-cam shot of the
# car at rest in several layer configurations: all drawn (01), ribbons hidden (02, terrain only), terrain hidden (03,
# ribbons only), then the same three from a low, close camera (04..06). poses.txt carries the numbers. Needs a window.
const SETTLE_S := 12.0
const SHOT_FRAMES := 10
const TIMEOUT_S := 300.0

var main: Node
var out_dir: String
var _phase := 0
var _frames := 0
var _started_ms := 0
var _log := PackedStringArray()
var _saved: Dictionary = {}

const CONFIGS := [
	{"name": "01_all.png", "terrain": false, "ribbons": false, "close": false},
	{"name": "02_terrain_only.png", "terrain": false, "ribbons": true, "close": false},
	{"name": "03_ribbons_only.png", "terrain": true, "ribbons": false, "close": false},
	{"name": "04_all_close.png", "terrain": false, "ribbons": false, "close": true},
	{"name": "05_terrain_only_close.png", "terrain": false, "ribbons": true, "close": true},
	{"name": "06_ribbons_only_close.png", "terrain": true, "ribbons": false, "close": true},
]

func _ready() -> void:
	_started_ms = Time.get_ticks_msec()
	DirAccess.make_dir_recursive_absolute(out_dir)

func _set_close(on: bool) -> void:
	var rig: Node3D = main.get_director().active_rig()
	if rig == null:
		return
	if _saved.is_empty():
		_saved = {"d": rig.follow_distance_m, "h": rig.follow_height_m, "l": rig.look_height_m}
	if on:
		rig.follow_distance_m = 4.0
		rig.follow_height_m = 0.9
		rig.look_height_m = 0.0
	else:
		rig.follow_distance_m = _saved["d"]
		rig.follow_height_m = _saved["h"]
		rig.look_height_m = _saved["l"]
	rig.activate(rig.transform)

func _quit(code: int) -> void:
	var f := FileAccess.open(out_dir.path_join("poses.txt"), FileAccess.WRITE)
	if f != null:
		f.store_string("\n".join(_log) + "\n")
	_phase = 999
	get_tree().quit(code)

func _process(_delta: float) -> void:
	if _phase >= 999:
		return
	var elapsed := (Time.get_ticks_msec() - _started_ms) / 1000.0
	if elapsed > TIMEOUT_S or main.world_state == "failed":
		_quit(1)
		return
	var sim: Node = main.get_simulation()
	if main.world_state != "running" or not bool(main.get_world_view().is_fully_uploaded()):
		return
	if float(sim.get_sim_time()) - main.ready_sim_time < SETTLE_S:
		return
	if _phase >= CONFIGS.size():
		main.debug_hide(false, false)
		_quit(0)
		return
	var cfg: Dictionary = CONFIGS[_phase]
	if _frames == 0:
		main.debug_hide(cfg["terrain"], cfg["ribbons"])
		_set_close(cfg["close"])
	_frames += 1
	if _frames >= SHOT_FRAMES:
		var path := out_dir.path_join(cfg["name"])
		var err := get_viewport().get_texture().get_image().save_png(path)
		var p: Vector3 = main.chassis_session_position()
		_log.append("%s err=%d session=(%.2f, %.2f, %.2f)" % [cfg["name"], err, p.x, p.y, p.z])
		_frames = 0
		_phase += 1
