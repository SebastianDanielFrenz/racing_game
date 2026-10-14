extends RefCounted
# Per-frame cost probes for RG_FRAME_SPIKE / RG_FRAME_STATS (render stutter diagnosis, 2026-10-14).
# A node times its own _process with
#     var t0 := Time.get_ticks_usec()
#     ... work ...
#     FrameProbe.add("name", Time.get_ticks_usec() - t0)
# main.gd takes the finished frame's costs at the start of the next frame (take_frame) and prints them next to a slow
# frame's delta, so a hitch is attributed to a subsystem - or to "none of them" (render/GPU/driver/OS).
static var _cur: Dictionary = {}

static func add(probe_name: String, usec: int) -> void:
	_cur[probe_name] = int(_cur.get(probe_name, 0)) + usec

static func take_frame() -> Dictionary:
	var out := _cur
	_cur = {}
	return out
