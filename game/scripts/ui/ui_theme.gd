extends RefCounted
# game/scripts/ui/ui_theme.gd - the design language in Godot (docs/design_language.md). Static, so any script reaches it with
#     const UiTheme := preload("res://scripts/ui/ui_theme.gd")
# It holds no rules: the roles, tokens and palettes come from data/ui through RgUiTheme (rg/ui_theme.h), which has already
# refused every palette that breaks the role contract. What lives here is the Godot side of a swap:
# - role(&"accent"): the colour a role has in the running palette. Read it where you paint, never keep it in a constant.
# - The engine's default Theme is repainted in place, so plain Controls follow without a line of code: buttons, sliders, lists,
#   line edits, popups, on any CanvasLayer and in any SubViewport (a Theme set on a root Control stops at both). A Label takes a
#   text role with text(label, &"warning_text").
# - bind / bind_font / style: a colour outside the Theme that follows the palette from then on (ColorRect.color, a stylebox,
#   an override). Free the object and the binding goes with it.
# - changed: for _draw code that caches. follow(self) redraws a CanvasItem on every swap; generation() goes into a draw signature.

class Bus extends RefCounted:
	signal changed

const MISSING := Color.MAGENTA # a role nobody knows shows up, it does not pass as black
const DEFAULT_PALETTE_FALLBACK := "racing"

# The engine's stock theme uses a handful of colours; each maps to the role that does its job. Keyed by the stock value, so
# every control type follows, also the ones this game does not use yet.
const STOCK_BOX_ROLES := {
	"0.1,0.1,0.1,0.6": &"surface_control",
	"0.225,0.225,0.225,0.6": &"surface_control_hover",
	"0,0,0,0.6": &"surface_control_pressed",
	"0,0,0,0.51": &"surface_control_pressed",
	"0,0,0,0.5": &"surface_control_pressed",
	"0.1,0.1,0.1,0.3": &"surface_control_disabled",
	"0.25,0.25,0.25,1": &"surface_popup",
	"0.4,0.4,0.4,1": &"surface_popup_hover",
	"1,1,1,0.4": &"control_fill",
	"1,1,1,0.75": &"control_fill_hover",
	"0.75,0.75,0.75,0.75": &"control_fill_hover",
}
const STOCK_FONT_ROLES := {
	"0.875,0.875,0.875,1": &"text_control",
	"0.95,0.95,0.95,1": &"text",
	"1,1,1,1": &"text",
	"0.7,0.7,0.7,1": &"text_muted",
	"0.65,0.65,0.65,1": &"text_muted",
	"0.875,0.875,0.875,0.6": &"text_muted",
	"0.875,0.875,0.875,0.5": &"text_disabled",
	"1,1,1,0.3": &"text_disabled",
	"0.9,0.9,0.9,0.3": &"text_disabled",
	"0.4,0.4,0.4,0.8": &"text_disabled",
}
const STOCK_FOCUS_ITEMS := ["focus", "cursor", "cursor_unfocused", "tab_focus", "scroll_focus", "panel_focus", "picker_focus_circle", "picker_focus_rectangle", "preset_focus", "sample_focus", "slot_selected"]
# Labels a screen colours by role; each gets a type variation "Label_<role>" of the stock Label.
const TEXT_ROLES := [&"text", &"text_muted", &"text_on_scene", &"text_on_accent", &"text_on_status", &"accent", &"accent_text", &"good_text", &"warning_text", &"danger_text", &"info_text"]

static var _data: Dictionary = {}
static var _bus: Bus = null
static var _palette_id := ""
static var _colours: Dictionary = {}
static var _tokens: Dictionary = {}
static var _generation := 0
static var _bindings: Dictionary = {}
static var _bindings_swept_at := 0
static var _warned: Dictionary = {}
static var _stock_colours: Array = [] # [type, item, role]
static var _stock_boxes: Array = []   # [StyleBoxFlat, property, role]

static func _ensure() -> void:
	if not _data.is_empty():
		return
	_bus = Bus.new()
	var data_dir: String = ProjectSettings.globalize_path("res://").path_join("../data").simplify_path()
	_data = RgUiTheme.load_theme(data_dir)
	if not bool(_data.get("ok", false)):
		push_error("UiTheme: %s" % str(_data.get("error", "no data")))
		return
	for refused: Dictionary in _data["refused"]:
		for problem in refused["problems"]:
			push_warning("UiTheme: palette %s refused: %s" % [refused["id"], problem])
	_tokens = _data["tokens"]
	_snapshot_stock_theme()
	_use(str(_data["default_palette"]))

# ---- palettes ---------------------------------------------------------------------------------------------------------

# Accepted palettes, in the game's menu order.
static func palette_ids() -> PackedStringArray:
	_ensure()
	return _data.get("ids", PackedStringArray())

static func palette_label(id: String) -> String:
	_ensure()
	return str(_data.get("palettes", {}).get(id, {}).get("label", id))

static func current() -> String:
	_ensure()
	return _palette_id

static func default_palette() -> String:
	_ensure()
	return str(_data.get("default_palette", DEFAULT_PALETTE_FALLBACK))

# The role list of the contract: [{id, group, meaning}].
static func roles() -> Array:
	_ensure()
	return _data.get("roles", [])

# Swaps the palette at runtime. "" when it stands, else words saying why it does not (the running palette stays).
static func set_palette(id: String) -> String:
	_ensure()
	for refused: Dictionary in _data.get("refused", []):
		if str(refused["id"]) == id:
			return "palette %s was refused: %s" % [id, "; ".join(refused["problems"])]
	if not _data.get("palettes", {}).has(id):
		return "there is no palette %s (offered: %s)" % [id, ", ".join(palette_ids())]
	if id != _palette_id:
		_use(id)
	return ""

static func _use(id: String) -> void:
	_palette_id = id
	_colours = _data["palettes"][id]["colours"]
	_generation += 1
	_paint_stock_theme()
	_paint_bindings()
	_bus.changed.emit()

# ---- roles and tokens -------------------------------------------------------------------------------------------------

static func role(name: StringName) -> Color:
	if _colours.is_empty():
		_ensure()
	var found: Variant = _colours.get(name)
	if found == null:
		if not _warned.has(name):
			_warned[name] = true
			push_error("UiTheme: no role named %s" % name)
		return MISSING
	return found

# A role at a fraction of its own strength: a pulse, a fade, a dimmed lamp.
static func faded(name: StringName, fraction: float) -> Color:
	var colour := role(name)
	colour.a *= fraction
	return colour

# A design token ("type.body", "gauge.sweep_start_deg") as written in data/ui/tokens.json. Controls under the root's content
# scale use this: the canvas already scales them.
static func token(name: StringName) -> float:
	if _tokens.is_empty():
		_ensure()
	var found: Variant = _tokens.get(name)
	if found == null:
		if not _warned.has(name):
			_warned[name] = true
			push_error("UiTheme: no token named %s" % name)
		return 0.0
	return float(found)

# How much the design pixels grow on this viewport: 1 at the reference height, never below min_scale.
static func viewport_scale(viewport_height: float) -> float:
	_ensure()
	return maxf(viewport_height / float(_data.get("reference_height_px", 1080.0)), float(_data.get("min_scale", 0.4)))

# A token in real pixels of a viewport: _draw code and anything else the content scale does not reach (HUD dials, a headset view).
static func px(name: StringName, viewport_height: float) -> float:
	return token(name) * viewport_scale(viewport_height)

# ---- following the palette --------------------------------------------------------------------------------------------

static func bus() -> Bus:
	_ensure()
	return _bus

# Goes up with every swap: put it in a draw signature.
static func generation() -> int:
	return _generation

# A CanvasItem that caches its drawing redraws on every swap.
static func follow(item: CanvasItem) -> void:
	bus().changed.connect(item.queue_redraw)

# `object.property` follows the role from now on: a ColorRect's colour, a stylebox's bg_color, a modulate. Binding again replaces.
static func bind(object: Object, property: StringName, name: StringName) -> void:
	_ensure()
	object.set(property, role(name))
	_remember(object, property, [weakref(object), property, name, false])

# A theme colour override that follows the role: Label font_color and the like. Labels are better served by text().
static func bind_font(control: Control, name: StringName, items: Array = [&"font_color"]) -> void:
	_ensure()
	for item: StringName in items:
		control.add_theme_color_override(item, role(name))
		_remember(control, item, [weakref(control), item, name, true])

# Undoes bind_font.
static func unbind_font(control: Control, items: Array = [&"font_color"]) -> void:
	for item: StringName in items:
		control.remove_theme_color_override(item)
		_bindings.erase("%d/%s" % [control.get_instance_id(), item])

# A Label in a text role. Survives swaps with no binding: the variation lives in the Theme.
static func text(label: Control, name: StringName) -> void:
	_ensure()
	label.theme_type_variation = StringName("Label_%s" % name)

# A flat stylebox whose fill and edge are roles and follow the palette. Keep shape on the box as usual.
static func style(fill: StringName, edge: StringName = &"") -> StyleBoxFlat:
	var box := StyleBoxFlat.new()
	bind(box, &"bg_color", fill)
	if edge != &"":
		bind(box, &"border_color", edge)
	return box

static func _remember(object: Object, item: StringName, entry: Array) -> void:
	_bindings["%d/%s" % [object.get_instance_id(), item]] = entry
	if _bindings.size() > 2 * _bindings_swept_at + 256:
		_sweep_bindings()

static func _sweep_bindings() -> void:
	for key: String in _bindings.keys():
		if _bindings[key][0].get_ref() == null:
			_bindings.erase(key)
	_bindings_swept_at = _bindings.size()

static func _paint_bindings() -> void:
	_sweep_bindings()
	for entry: Array in _bindings.values():
		var object: Object = entry[0].get_ref()
		if entry[3]:
			(object as Control).add_theme_color_override(entry[1], role(entry[2]))
		else:
			object.set(entry[1], role(entry[2]))

# ---- the engine's theme -----------------------------------------------------------------------------------------------

static func _value_key(colour: Color) -> String:
	return "%s,%s,%s,%s" % [snappedf(colour.r, 0.001), snappedf(colour.g, 0.001), snappedf(colour.b, 0.001), snappedf(colour.a, 0.001)]

# Which stock colour does which job, read once: the stock values are overwritten by the first palette.
static func _snapshot_stock_theme() -> void:
	var theme: Theme = ThemeDB.get_default_theme()
	for type: String in theme.get_type_list():
		for item: String in theme.get_color_list(type):
			if not item.begins_with("font_") and item != "caret_color":
				continue
			if item.contains("outline") or item.contains("shadow"):
				continue
			var font_role: Variant = STOCK_FONT_ROLES.get(_value_key(theme.get_color(item, type)))
			if font_role != null:
				_stock_colours.append([type, item, font_role])
		for item: String in theme.get_stylebox_list(type):
			var box := theme.get_stylebox(item, type) as StyleBoxFlat
			if box == null:
				continue
			if item in STOCK_FOCUS_ITEMS:
				_stock_boxes.append([box, &"border_color", &"focus_ring"])
				continue
			var box_role: Variant = STOCK_BOX_ROLES.get(_value_key(box.bg_color))
			if box_role != null:
				_stock_boxes.append([box, &"bg_color", box_role])
	for name: StringName in TEXT_ROLES:
		var variation := StringName("Label_%s" % name)
		theme.set_type_variation(variation, &"Label")

static func _paint_stock_theme() -> void:
	var theme: Theme = ThemeDB.get_default_theme()
	theme.set_block_signals(true)
	for entry: Array in _stock_colours:
		theme.set_color(entry[1], entry[0], role(entry[2]))
	for entry: Array in _stock_boxes:
		entry[0].set(entry[1], role(entry[2]))
	for name: StringName in TEXT_ROLES:
		theme.set_color(&"font_color", StringName("Label_%s" % name), role(name))
	theme.set_color(&"selection_color", &"LineEdit", role(&"border_strong"))
	theme.set_block_signals(false)
	theme.emit_changed()
