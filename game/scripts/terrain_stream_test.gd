# game/scripts/terrain_stream_test.gd - --terrain-preview --stream-test
# (R2.2 R8 headless check, tools/smoke_test.ps1 -TerrainStream): once the
# initial preview is uploaded, moves RgTerrainView's LOD focus through a fixed
# list of session-local offsets from spawn (each > 128 m from the previous, so
# each step is exactly one streamed diff), waits until each diff is fully
# applied (adds uploaded, removals freed, committed), prints one line per step
# and a final "terrain stream test done" line, then quits. The smoke test
# asserts that line plus 0 ERROR lines - Godot prints every RID leaked at exit
# as an ERROR, so a clean exit after several add/remove diffs is the RID-leak
# check. The smoke's ERROR grep is case-insensitive, so nothing printed here
# may contain "error" (hence "missing_removals", not "errors"). Wall-clock timeout (not frame count: headless frames are uncapped).
extends Node

const STEPS := [
	Vector2(300.0, 0.0),
	Vector2(600.0, 0.0),
	Vector2(600.0, 300.0),
	Vector2(300.0, 600.0),
	Vector2(0.0, 0.0),
]
const TIMEOUT_MS := 180000

var terrain_view: Node
var spawn := Vector2.ZERO

var _start_ms := 0
var _step := -1 # -1: waiting for the initial preview upload
var _max_upload_ms := 0.0
var _added_total := 0
var _removed_total := 0

func _ready() -> void:
	_start_ms = Time.get_ticks_msec()

func _process(_delta: float) -> void:
	if Time.get_ticks_msec() - _start_ms > TIMEOUT_MS:
		push_error("terrain_stream_test: timed out at step %d" % _step)
		get_tree().quit(1)
		set_process(false)
		return
	if _step < 0:
		if bool(terrain_view.is_fully_uploaded()):
			_step = 0
			_max_upload_ms = 0.0
		else:
			return

	var focus: Vector2 = spawn + STEPS[_step]
	terrain_view.update_focus(focus.x, focus.y) # every frame, like a real caller
	var st: Dictionary = terrain_view.get_stream_stats()
	_max_upload_ms = maxf(_max_upload_ms, float(st["upload_ms_this_frame"]))
	if int(st["diffs_applied"]) < _step + 1 or not bool(terrain_view.is_stream_idle()):
		return

	_added_total += int(st["last_diff_added"])
	_removed_total += int(st["last_diff_removed"])
	print("terrain stream step %d: focus=(%.0f, %.0f) added=%d removed=%d chunks=%d build_ms=%.2f max_frame_ms=%.3f remove_ms=%.3f missing_removals=%d" % [
		_step, STEPS[_step].x, STEPS[_step].y, int(st["last_diff_added"]), int(st["last_diff_removed"]),
		int(st["resident_chunks"]), float(st["last_build_ms"]), _max_upload_ms, float(st["last_remove_ms"]),
		int(st["diff_errors"])])
	_max_upload_ms = 0.0
	_step += 1
	if _step >= STEPS.size():
		print("terrain stream test done: steps=%d diffs=%d added=%d removed=%d chunks=%d missing_removals=%d" % [
			STEPS.size(), int(st["diffs_applied"]), _added_total, _removed_total, int(st["resident_chunks"]),
			int(st["diff_errors"])])
		set_process(false)
		get_tree().quit(0)
