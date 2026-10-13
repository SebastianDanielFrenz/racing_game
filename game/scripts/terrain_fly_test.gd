# game/scripts/terrain_fly_test.gd - --terrain-preview --stream-fly
# Windowed measurement of the render terrain stream at speed (owner complaint
# 2026-10-13: at 400-580 km/h the visible terrain does not resolve in time).
# Moves RgTerrainView's focus CONTINUOUSLY at a constant speed along a straight
# line from spawn, like a car (no Session needed), and measures how far ahead of
# the moving point the UPLOADED ground is covered by chunks of level <= 0 / <= 1
# (rg::lod_coverage_ahead), the upload cost per frame, the chunk latency and the
# frame time. Needs a real renderer (not --headless: the dummy renderer has no
# upload cost). Prints one RG_FLY line per second and a final RG_FLY_DONE line
# with the percentiles, then quits.
#
# user args:  --fly-speed=160 (m/s)  --fly-seconds=40  --fly-dir=0 (deg, 0 = east)
#             --fly-baseline   the pre-fix behaviour: no velocity lead, no urgency
#                              order, fixed 0.8 ms upload budget
#             --fly-lead=1.0   look-ahead seconds (ignored with --fly-baseline)
extends Node

var terrain_view: Node
var spawn := Vector2.ZERO

var _speed := 160.0
var _seconds := 40.0
var _dir := Vector2.RIGHT
var _baseline := false
var _lead_s := 1.0
var _focus := RgRenderFocus.new()
var _pos := Vector2.ZERO
var _t := 0.0
var _warmup := 6.0 # s of hovering at spawn before the flight starts, lets the initial set settle
var _flying := false
var _fly_t := 0.0
var _next_sample := 0.0
var _next_report := 1.0
var _l0: Array = []
var _l1: Array = []
var _l2: Array = []
var _frame_ms: Array = []
var _upload_ms_avg: Array = []
var _upload_ms_max := 0.0
var _lat_p95: Array = []
var _last_diffs := 0
var _build_ms_sum := 0.0
var _added_sum := 0
var _diff_count := 0
var _gap_samples := 0
var _start_ms := 0

func _ready() -> void:
	_start_ms = Time.get_ticks_msec()
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--fly-speed="):
			_speed = float(a.get_slice("=", 1))
		elif a.begins_with("--fly-seconds="):
			_seconds = float(a.get_slice("=", 1))
		elif a.begins_with("--fly-dir="):
			_dir = Vector2.from_angle(deg_to_rad(float(a.get_slice("=", 1))))
		elif a.begins_with("--fly-lead="):
			_lead_s = float(a.get_slice("=", 1))
		elif a == "--fly-baseline":
			_baseline = true
	_pos = spawn
	_focus.set_lead_s(0.0 if _baseline else _lead_s)
	if _baseline:
		terrain_view.set_upload_adaptive(false)
	print("RG_FLY start speed_mps=%.0f seconds=%.0f dir=(%.2f,%.2f) mode=%s" % [
		_speed, _seconds, _dir.x, _dir.y, "baseline" if _baseline else "fix(lead=%.1fs, adaptive budget, urgency order)" % _lead_s])

func _percentile(values: Array, q: float) -> float:
	if values.is_empty():
		return 0.0
	var sorted := values.duplicate()
	sorted.sort()
	return float(sorted[clampi(int(ceil(q * sorted.size())) - 1, 0, sorted.size() - 1)])

func _process(delta: float) -> void:
	_t += delta
	if not _flying:
		if _t >= _warmup and bool(terrain_view.is_fully_uploaded()):
			_flying = true
			_fly_t = 0.0
			terrain_view.take_report_stats()
		else:
			terrain_view.update_focus(spawn.x, spawn.y)
			return
	_fly_t += delta
	_frame_ms.append(delta * 1000.0)
	_pos += _dir * _speed * delta
	var v := _dir * _speed
	if _baseline:
		terrain_view.update_focus(_pos.x, _pos.y)
	else:
		var f: Dictionary = _focus.update(_pos.x, _pos.y, v.x, v.y, delta, true)
		terrain_view.update_focus_moving(f["x"], f["y"], v.x, v.y, _pos.x, _pos.y)
	var st: Dictionary = terrain_view.get_stream_stats()
	_upload_ms_max = maxf(_upload_ms_max, float(st["upload_ms_this_frame"]))
	if int(st["diffs_applied"]) != _last_diffs:
		_last_diffs = int(st["diffs_applied"])
		_diff_count += 1
		_build_ms_sum += float(st["last_build_ms"])
		_added_sum += int(st["last_diff_added"])
	if _fly_t >= _next_sample:
		_next_sample += 0.1
		var cov: Dictionary = terrain_view.get_lod_coverage(_pos.x, _pos.y, _dir.x, _dir.y)
		_l0.append(float(cov["l0_ahead_m"]))
		_l1.append(float(cov["l1_ahead_m"]))
		_l2.append(float(cov["l2_ahead_m"]))
	if _fly_t >= _next_report:
		_next_report += 1.0
		var rs: Dictionary = terrain_view.take_report_stats()
		_upload_ms_avg.append(float(rs["upload_ms_avg"]))
		_lat_p95.append(float(rs["chunk_latency_p95_ms"]))
		print("RG_FLY t=%.0f dist_m=%.0f l0_ahead_m=%.0f l1_ahead_m=%.0f l2_ahead_m=%.0f pending_adds=%d pending_removals=%d adds_s=%d upload_ms_avg=%.2f upload_ms_max=%.2f chunk_lat_p95_ms=%.0f budget_ms=%.2f fps=%d resident=%d" % [
			_fly_t, (_pos - spawn).length(), _l0.back(), _l1.back(), _l2.back(), int(st["pending_adds"]), int(st["pending_removals"]),
			int(rs["chunks_uploaded"]), float(rs["upload_ms_avg"]), float(rs["upload_ms_max"]), float(rs["chunk_latency_p95_ms"]),
			float(rs["budget_ms_avg"]), int(Engine.get_frames_per_second()), int(st["resident_chunks"])])
	if _fly_t >= _seconds:
		var frame_avg := 0.0
		for f in _frame_ms:
			frame_avg += f
		frame_avg /= maxf(1.0, _frame_ms.size())
		print("RG_FLY_DONE mode=%s speed_mps=%.0f seconds=%.0f l0_ahead_p5_m=%.0f l0_ahead_min_m=%.0f l0_ahead_p50_m=%.0f l1_ahead_p5_m=%.0f l1_ahead_min_m=%.0f l1_ahead_p50_m=%.0f l2_ahead_p5_m=%.0f upload_ms_avg=%.2f upload_ms_max=%.2f chunk_lat_p95_ms=%.0f frame_ms_avg=%.2f frame_ms_p99=%.2f diffs=%d build_ms_avg=%.1f added_avg=%.1f build_ms_per_chunk=%.3f" % [
			"baseline" if _baseline else "fix", _speed, _seconds, _percentile(_l0, 0.05), _percentile(_l0, 0.0), _percentile(_l0, 0.5),
			_percentile(_l1, 0.05), _percentile(_l1, 0.0), _percentile(_l1, 0.5), _percentile(_l2, 0.05),
			_avg(_upload_ms_avg), _upload_ms_max, _percentile(_lat_p95, 0.5), frame_avg, _percentile(_frame_ms, 0.99),
			_diff_count, _build_ms_sum / maxf(1.0, _diff_count), float(_added_sum) / maxf(1.0, _diff_count),
			_build_ms_sum / maxf(1.0, _added_sum)])
		set_process(false)
		get_tree().quit(0)

func _avg(values: Array) -> float:
	if values.is_empty():
		return 0.0
	var s := 0.0
	for v in values:
		s += float(v)
	return s / values.size()
