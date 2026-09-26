extends CanvasLayer
# game/scripts/loading_overlay.gd — the full-screen overlay shown while a
# real-world Session is starting up (RgSimulation.get_init_status() reports
# "loading"), and on a load error. Drawing only: main.gd hands it the status
# Dictionary every frame; this node decides nothing. The numbers are
# rg::StartupProgress's live atomics (R9): stage, resident L0 tiles, required
# tiles still missing, fetches in flight, failed fetches, priming ticks done
# of total. A future XR build shows the same numbers on a world-space panel.

var _panel: ColorRect
var _title: Label
var _body: Label

func _ready() -> void:
	layer = 10
	_panel = ColorRect.new()
	_panel.color = Color(0.05, 0.07, 0.09, 0.92)
	_panel.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(_panel)

	var box := VBoxContainer.new()
	box.set_anchors_preset(Control.PRESET_CENTER)
	box.grow_horizontal = Control.GROW_DIRECTION_BOTH
	box.grow_vertical = Control.GROW_DIRECTION_BOTH
	box.alignment = BoxContainer.ALIGNMENT_CENTER
	_panel.add_child(box)

	_title = Label.new()
	_title.add_theme_font_size_override("font_size", 34)
	_title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	box.add_child(_title)

	_body = Label.new()
	_body.add_theme_font_size_override("font_size", 18)
	_body.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	box.add_child(_body)
	visible = false

const STAGE_TEXT := {
	"opening": "opening the terrain store",
	"waiting_for_gate": "streaming the terrain around the spawn",
	"priming": "priming the physics tiles",
	"spawning": "placing the car",
	"done": "starting the simulation",
}

func show_loading(world_label: String, st: Dictionary, elapsed_s: float) -> void:
	visible = true
	_title.text = "LOADING %s..." % world_label.to_upper()
	var stage: String = str(st.get("stage", "?"))
	var lines := PackedStringArray()
	lines.append("%s  (%s)" % [STAGE_TEXT.get(stage, stage), stage])
	lines.append("L0 tiles resident %d   required missing %d   in flight %d   failed %d" % [
		int(st.get("resident_l0", 0)), int(st.get("missing_required", 0)),
		int(st.get("inflight", 0)), int(st.get("failed", 0))])
	lines.append("priming ticks %d / %d" % [int(st.get("prime_done", 0)), int(st.get("prime_total", 0))])
	lines.append("%.1f s" % elapsed_s)
	lines.append("")
	lines.append("F8 cancels and switches to the other world")
	_body.text = "\n".join(lines)

func show_error(world_label: String, message: String) -> void:
	visible = true
	_title.text = "%s FAILED TO LOAD" % world_label.to_upper()
	_body.text = "%s\n\nF8 switches to the other world" % message

func hide_overlay() -> void:
	visible = false
