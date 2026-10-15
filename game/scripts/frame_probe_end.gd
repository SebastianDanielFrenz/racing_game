extends Node
# Last _process of the frame (priority 100000): records how long the scripts of the whole frame took from main.gd's
# first _process (FrameProbe.frame_start_us) to here. RG_FRAME_STATS prints it as scene_span, and "outside" = frame
# delta - scene_span = time spent in the engine's render, the driver, vsync waits and the OS (diagnosis 2026-10-14).
#
# Frame phases (2026-10-15): the engine's main loop runs, per frame, [OS events + _input + frame limiter sleep]
# -> [physics steps] -> [_process of every node] -> [deferred calls, queue_free, RenderingServer sync of commands
# queued by worker threads] -> [draw: scene update, every viewport, buffer swap = GPU / vsync wait]. The signals
# below timestamp the boundaries, so "outside" is split into
#   ph_loop      post_draw -> first physics step (or process_frame): OS events, _input, max_fps sleep
#   ph_physics   first physics_frame -> process_frame: _physics_process + the physics server
#   ph_pre       process_frame -> main.gd's _process (first script of the frame)
#   ph_post      last _process -> frame_pre_draw: deferred calls, queue_free, RenderingServer.sync()
#   ph_draw      frame_pre_draw -> frame_post_draw: RenderingServer draw incl. the buffer swap
#   render_cpu / render_gpu  the main viewport's own measured render time (part of ph_draw)
#   ph_frame     post_draw -> post_draw: the real wall time of the frame (the process delta is clamped by Godot when
#                a frame takes longer than max_physics_steps_per_frame physics steps, 133 ms at the defaults)
# All are added to FrameProbe in the post_draw callback, so main.gd takes them with the same frame's script probes.
const FrameProbe := preload("res://scripts/frame_probe.gd")
var _t_post_prev := 0
var _t_phys := 0
var _t_proc := 0
var _t_scripts_end := 0
var _t_pre := 0
func _ready() -> void:
	process_priority = 100000
	get_tree().physics_frame.connect(_on_physics_frame)
	get_tree().process_frame.connect(_on_process_frame)
	RenderingServer.frame_pre_draw.connect(_on_pre_draw)
	RenderingServer.frame_post_draw.connect(_on_post_draw)
func _exit_tree() -> void:
	if RenderingServer.frame_pre_draw.is_connected(_on_pre_draw):
		RenderingServer.frame_pre_draw.disconnect(_on_pre_draw)
	if RenderingServer.frame_post_draw.is_connected(_on_post_draw):
		RenderingServer.frame_post_draw.disconnect(_on_post_draw)
func _process(_delta: float) -> void:
	_t_scripts_end = Time.get_ticks_usec()
	if FrameProbe.frame_start_us > 0:
		FrameProbe.add("scene_span", _t_scripts_end - FrameProbe.frame_start_us)
func _on_physics_frame() -> void:
	if _t_phys == 0:
		_t_phys = Time.get_ticks_usec()
func _on_process_frame() -> void:
	_t_proc = Time.get_ticks_usec()
func _on_pre_draw() -> void:
	_t_pre = Time.get_ticks_usec()
func _on_post_draw() -> void:
	var now := Time.get_ticks_usec()
	if _t_post_prev > 0 and _t_proc > 0:
		var loop_end := _t_phys if _t_phys > 0 else _t_proc
		FrameProbe.add("ph_frame", now - _t_post_prev)
		FrameProbe.add("ph_loop", maxi(0, loop_end - _t_post_prev))
		if _t_phys > 0:
			FrameProbe.add("ph_physics", maxi(0, _t_proc - _t_phys))
		if FrameProbe.frame_start_us >= _t_proc:
			FrameProbe.add("ph_pre", FrameProbe.frame_start_us - _t_proc)
		if _t_scripts_end >= _t_proc and _t_pre >= _t_scripts_end:
			FrameProbe.add("ph_post", _t_pre - _t_scripts_end)
		if _t_pre > 0:
			FrameProbe.add("ph_draw", maxi(0, now - _t_pre))
		var vp := get_viewport().get_viewport_rid()
		FrameProbe.add("render_cpu", int(RenderingServer.viewport_get_measured_render_time_cpu(vp) * 1000.0))
		FrameProbe.add("render_gpu", int(RenderingServer.viewport_get_measured_render_time_gpu(vp) * 1000.0))
	_t_post_prev = now
	_t_phys = 0
	_t_proc = 0
	_t_pre = 0
