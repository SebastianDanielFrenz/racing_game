# game/scripts/screenshot_tour.gd - --terrain-preview --screenshots <dir>:
# waits until RgTerrainView has uploaded every chunk, then puts the camera at
# a fixed list of poses (relative to the render origin = spawn, Godot Y-up,
# heights absolute metres ASL since the render origin's "up" is 0), writes
# one PNG per pose plus poses.txt, and quits. Needs a real renderer (not
# --headless). A timeout quits non-zero if loading never completes.
extends Node

const SETTLE_FRAMES := 10
const POSE_FRAMES := 4
const TIMEOUT_S := 120.0

# name, position, rotation_degrees (pitch, yaw, roll); yaw 0 = north (-Z),
# +90 = west.
const POSES := [
	["01_north_700m", Vector3(0.0, 700.0, 0.0), Vector3(-18.0, 0.0, 0.0)],
	["02_ridge_northwest_800m", Vector3(0.0, 800.0, 0.0), Vector3(-10.0, 45.0, 0.0)],
	["03_overview_4000m", Vector3(0.0, 4000.0, 3000.0), Vector3(-55.0, 0.0, 0.0)],
	["04_west_550m", Vector3(0.0, 550.0, 0.0), Vector3(-12.0, 90.0, 0.0)],
	["05_south_800m", Vector3(0.0, 800.0, 0.0), Vector3(-10.0, 180.0, 0.0)],
]

var camera: Camera3D
var terrain_view: Node
var out_dir: String

var _elapsed := 0.0
var _frames_since_loaded := -1
var _pose := 0
var _pose_frames := 0
var _log := PackedStringArray()

func _process(delta: float) -> void:
	_elapsed += delta
	if _frames_since_loaded < 0:
		if bool(terrain_view.is_fully_uploaded()):
			_frames_since_loaded = 0
		elif _elapsed > TIMEOUT_S:
			push_error("screenshot_tour: terrain never finished uploading")
			get_tree().quit(1)
		return
	if _frames_since_loaded < SETTLE_FRAMES:
		_frames_since_loaded += 1
		if _frames_since_loaded == SETTLE_FRAMES:
			_apply_pose()
		return
	_pose_frames += 1
	if _pose_frames < POSE_FRAMES:
		return
	var p: Array = POSES[_pose]
	var path := out_dir.path_join("%s.png" % p[0])
	var err := get_viewport().get_texture().get_image().save_png(path)
	_log.append("%s pos=%s rot=%s chunks=%d vertices=%d fps=%.1f err=%d" % [
		p[0], p[1], p[2], terrain_view.get_chunk_count(), terrain_view.get_total_vertex_count(),
		Engine.get_frames_per_second(), err])
	print("screenshot_tour: wrote %s" % path)
	_pose += 1
	if _pose >= POSES.size():
		var f := FileAccess.open(out_dir.path_join("poses.txt"), FileAccess.WRITE)
		if f != null:
			f.store_string("\n".join(_log) + "\n")
		get_tree().quit(0)
		return
	_apply_pose()

func _apply_pose() -> void:
	var p: Array = POSES[_pose]
	camera.position = p[1]
	camera.rotation_degrees = p[2]
	_pose_frames = 0
