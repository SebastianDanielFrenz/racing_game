extends CanvasLayer
# game/scripts/loading_overlay.gd - the full-screen loading screen shown while a
# world is starting up (RgSimulation.get_init_status() reports "loading") and
# on a load error. Drawing only: main.gd hands it the status Dictionary every
# frame; this node decides nothing. Everything shown is a real number from
# rg::StartupProgress (R9): the stage, resident L0 tiles, required tiles still
# missing, fetches in flight, failed fetches and priming ticks done of total.
# The four start-up stages are listed with their state (done / current /
# waiting) and the bar is a REAL fraction: spawn-area tiles resident out of the
# gate's key set (gate_total, its first missing count) while streaming, then
# priming ticks (prime_done / prime_total) - opening and spawning have no known
# total, so no bar is drawn for them rather than an invented one.
# The data attribution line (data/credits.json, via RgShell) sits at the foot.
# A future XR build shows the same numbers on a world-space panel.

# The attribution line (rg::attribution_line) shown at the foot; set by main.gd.
var attribution_line: String = ""

var _panel: ColorRect
var _title: Label
var _where: Label
var _steps: Label
var _bar: ProgressBar
var _body: Label
var _footer: Label
var _attribution: Label

func _ready() -> void:
	layer = 10
	_panel = ColorRect.new()
	_panel.color = Color(0.05, 0.07, 0.09, 0.96)
	_panel.set_anchors_preset(Control.PRESET_FULL_RECT)
	add_child(_panel)

	var box := VBoxContainer.new()
	box.set_anchors_preset(Control.PRESET_CENTER)
	box.grow_horizontal = Control.GROW_DIRECTION_BOTH
	box.grow_vertical = Control.GROW_DIRECTION_BOTH
	box.alignment = BoxContainer.ALIGNMENT_CENTER
	box.add_theme_constant_override("separation", 10)
	_panel.add_child(box)

	_title = Label.new()
	_title.add_theme_font_size_override("font_size", 34)
	_title.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	box.add_child(_title)

	_where = Label.new()
	_where.add_theme_font_size_override("font_size", 20)
	_where.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	box.add_child(_where)

	_steps = Label.new()
	_steps.add_theme_font_size_override("font_size", 18)
	_steps.horizontal_alignment = HORIZONTAL_ALIGNMENT_LEFT
	box.add_child(_steps)

	_bar = ProgressBar.new()
	_bar.custom_minimum_size = Vector2(520, 18)
	_bar.show_percentage = false
	_bar.min_value = 0.0
	_bar.max_value = 1.0
	_bar.visible = false
	box.add_child(_bar)

	_body = Label.new()
	_body.add_theme_font_size_override("font_size", 16)
	_body.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	box.add_child(_body)

	_footer = Label.new()
	_footer.add_theme_font_size_override("font_size", 16)
	_footer.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_footer.modulate = Color(1, 1, 1, 0.75)
	box.add_child(_footer)

	_attribution = Label.new()
	_attribution.add_theme_font_size_override("font_size", 13)
	_attribution.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	_attribution.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	_attribution.set_anchors_preset(Control.PRESET_BOTTOM_WIDE)
	_attribution.grow_vertical = Control.GROW_DIRECTION_BEGIN
	_attribution.offset_left = 24
	_attribution.offset_right = -24
	_attribution.offset_bottom = -14
	_attribution.modulate = Color(1, 1, 1, 0.7)
	_panel.add_child(_attribution)
	visible = false

# rg::StartupProgress stages, in the order a load goes through them.
const STAGES := ["opening", "waiting_for_gate", "priming", "spawning"]
const STAGE_TEXT := {
	"opening": "opening the terrain store",
	"waiting_for_gate": "streaming the terrain around the spawn",
	"priming": "priming the physics tiles",
	"spawning": "placing the car",
	"done": "starting the simulation",
}

# `footer`: the line about what the keys do ("" = none). `place`: where the car
# is going ("" = nothing to say, e.g. the flat scene).
func show_loading(world_label: String, st: Dictionary, elapsed_s: float, footer: String = "", place: String = "") -> void:
	visible = true
	_attribution.text = attribution_line
	_title.text = "LOADING %s..." % world_label.to_upper()
	_where.text = place
	_where.visible = place != ""
	var stage: String = str(st.get("stage", "?"))
	var current: int = STAGES.find(stage)
	var step_lines := PackedStringArray()
	for i in range(STAGES.size()):
		var mark := "[ ]"
		if stage == "done" or (current >= 0 and i < current):
			mark = "[x]"
		elif i == current:
			mark = "[>]"
		step_lines.append("%s %s" % [mark, STAGE_TEXT[STAGES[i]]])
	_steps.text = "\n".join(step_lines)
	var total: int = int(st.get("prime_total", 0))
	var gate_total: int = int(st.get("gate_total", 0))
	_bar.visible = (stage == "priming" and total > 0) or (stage == "waiting_for_gate" and gate_total > 0)
	if stage == "priming" and total > 0:
		_bar.value = clampf(float(st.get("prime_done", 0)) / float(total), 0.0, 1.0)
	elif _bar.visible:
		# Spawn-area tiles resident out of the gate's full key set (gate_total).
		_bar.value = clampf(float(gate_total - int(st.get("missing_required", 0))) / float(gate_total), 0.0, 1.0)
	var lines := PackedStringArray()
	lines.append("L0 tiles resident %d   required missing %d   in flight %d   failed %d" % [
		int(st.get("resident_l0", 0)), int(st.get("missing_required", 0)),
		int(st.get("inflight", 0)), int(st.get("failed", 0))])
	if stage == "waiting_for_gate" and gate_total > 0:
		lines.append("spawn area tiles %d / %d" % [gate_total - int(st.get("missing_required", 0)), gate_total])
	if total > 0:
		lines.append("priming ticks %d / %d" % [int(st.get("prime_done", 0)), total])
	lines.append("%.1f s" % elapsed_s)
	_body.text = "\n".join(lines)
	_footer.text = footer

func show_error(world_label: String, message: String, footer: String = "F8 switches to the other world") -> void:
	visible = true
	_attribution.text = attribution_line
	_title.text = "%s FAILED TO LOAD" % world_label.to_upper()
	_where.visible = false
	_steps.text = ""
	_bar.visible = false
	_body.text = message
	_footer.text = footer

func hide_overlay() -> void:
	visible = false
