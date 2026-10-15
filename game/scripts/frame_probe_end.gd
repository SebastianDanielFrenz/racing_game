extends Node
# Last _process of the frame (priority 100000): records how long the scripts of the whole frame took from main.gd's
# first _process (FrameProbe.frame_start_us) to here. RG_FRAME_STATS prints it as scene_span, and "outside" = frame
# delta - scene_span = time spent in the engine's render, the driver, vsync waits and the OS (diagnosis 2026-10-14).
const FrameProbe := preload("res://scripts/frame_probe.gd")
func _ready() -> void:
	process_priority = 100000
func _process(_delta: float) -> void:
	if FrameProbe.frame_start_us > 0:
		FrameProbe.add("scene_span", Time.get_ticks_usec() - FrameProbe.frame_start_us)
