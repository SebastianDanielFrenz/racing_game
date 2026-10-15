# game/scripts/terrain_upload_bench.gd - main-thread cost of the terrain chunk upload (C9, 2026-10-15), no map store:
#   godot --headless --path game --script res://scripts/terrain_upload_bench.gd [-- --speed=100 --seconds=20 --distance=20000 --fps=120]
# Streams RgTerrainView.load_synthetic_preview (rolling hills, the game's LOD defaults) and moves the focus along +x at
# --speed m/s of real time (update_focus_moving, as camera_director.gd does for a car). Prints one RG_UPLOAD_BENCH line per
# phase: the initial bulk load and the streaming run, with the per-frame RgTerrainView adapter time (upload_ms of
# get_frame_costs: everything apply_diff does on the main thread) and the per-chunk split (pack / surface / instance).
# Without the GDExtension the script says so and quits 1.
extends SceneTree

var _view: Node
var _phase := "bulk"
var _frames: Array[float] = []
var _start_us := 0
var _speed := 100.0
var _seconds := 20.0
var _distance := 20000.0
var _x := 0.0
var _costs0: Dictionary = {}

func _initialize() -> void:
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--speed="): _speed = float(a.get_slice("=", 1))
		elif a.begins_with("--seconds="): _seconds = float(a.get_slice("=", 1))
		elif a.begins_with("--distance="): _distance = float(a.get_slice("=", 1))
		elif a.begins_with("--fps="): Engine.max_fps = int(a.get_slice("=", 1))
	if not ClassDB.class_exists("RgTerrainView"):
		print("RG_UPLOAD_BENCH skipped: no GDExtension")
		quit(1)
		return
	_view = ClassDB.instantiate("RgTerrainView")
	root.add_child(_view)
	var material := ShaderMaterial.new()
	material.shader = load("res://shaders/terrain.gdshader")
	_view.set_material(material.get_rid())
	_view.set_meta("material", material) # keep it alive
	_view.load_synthetic_preview(0.0, 0.0, _distance)
	_view.set_render_origin(Vector3.ZERO)
	print("RG_UPLOAD_BENCH selected chunks=%d vertices=%d build_ms=%.1f" % [_view.get_chunk_count(), _view.get_total_vertex_count(), _view.get_last_build_time_ms()])
	_start_us = Time.get_ticks_usec()
	_costs0 = _view.get_frame_costs()

func _report(label: String, elapsed_s: float) -> void:
	var sorted := _frames.duplicate()
	sorted.sort()
	var total := 0.0
	for f in sorted: total += f
	var n := sorted.size()
	var c: Dictionary = _view.get_frame_costs()
	var uploads := int(c.uploads_total) - int(_costs0.uploads_total)
	var per := func(key: String) -> float: return (float(c[key]) - float(_costs0[key])) / maxf(1.0, uploads)
	var st: Dictionary = _view.get_stream_stats()
	print("RG_UPLOAD_BENCH phase=%s frames=%d seconds=%.1f chunks_uploaded=%d frame_upload_ms_avg=%.3f p50=%.3f p99=%.3f max=%.3f per_chunk_ms pack=%.3f surface=%.3f instance=%.3f diffs=%d resident=%d" % [
		label, n, elapsed_s, uploads, total / maxf(1, n), sorted[int(n * 0.5)] if n > 0 else 0.0,
		sorted[mini(n - 1, int(n * 0.99))] if n > 0 else 0.0, sorted[n - 1] if n > 0 else 0.0,
		per.call("upload_pack_ms_total"), per.call("upload_surface_ms_total"), per.call("upload_instance_ms_total"),
		int(st.diffs_applied), int(st.resident_chunks)])
	_frames.clear()
	_costs0 = c

func _process(delta: float) -> bool:
	if _view == null:
		return false
	var c: Dictionary = _view.get_frame_costs()
	var elapsed := (Time.get_ticks_usec() - _start_us) / 1e6
	if _phase == "bulk":
		_frames.append(float(c.upload_ms))
		if _view.is_fully_uploaded():
			_report("bulk", elapsed)
			_phase = "stream"
			_start_us = Time.get_ticks_usec()
		return false
	_frames.append(float(c.upload_ms))
	_x += _speed * delta
	_view.update_focus_moving(_x, 0.0, _speed, 0.0, _x, 0.0)
	if elapsed >= _seconds:
		_report("stream", elapsed)
		_view.release()
		return true
	return false
