extends Control
# game/scripts/car_browser_ui.gd - the car browser screen (PLAN.md R6c), shared by the garage
# (main menu) and the pause menu's "Change car". Drawing and input forwarding ONLY: what the
# grid holds (categories, filter, sort, which tile sits in which column and row, where the
# focus moves on an arrow key, which tiles are visible) is decided by rg::CarBrowser in rg_core
# and read through RgGarage.browser_*; this script instantiates exactly the tiles the view
# asks for (a catalog of hundreds of cars stays smooth) and forwards what the player did.
#
# Layout: stats of the focused car on the left (metric only), the horizontally scrolling grid of
# tiles (2-4 rows, from the height available) with category headers on the right, a filter/sort
# panel (F / gamepad Y / the on-screen button) on top of the grid. Each tile shows a lazily
# rendered preview (car_thumbnails.gd) with a placeholder until it exists.
#
# Input: arrows / D-pad / left stick move the focus, Enter / Space / A choose, Esc / B back,
# PageUp-PageDown / shoulder buttons jump between categories, F / Y open the panel, the mouse
# hovers (focus), clicks (choose) and scrolls with the wheel.
#
# Test hooks (tools/smoke_test.ps1 -CarBrowser): nav(), choose(), open_panel(), toggle_filter(),
# listed_ids(), focused_id(), instantiated_tile_count(), get_control(id).

signal car_chosen(id: String)
signal back_requested
signal car_focused(id: String)
signal state_changed # group/sort/filter changed: the caller persists RgGarage.browser_get_state()

const TILE_W := 300.0
const THUMB_H := 169.0
const TILE_H := 226.0
const GAP := 12.0
const COL_W := TILE_W + GAP
const HEADER_H := 36.0
const STATS_W := 400.0
const ACCENT := Color(0.91, 0.45, 0.17)
const OVERSCAN := 2
const STICK_THRESHOLD := 0.6
const STICK_REPEAT_S := 0.17

var garage: Node            # RgGarage
var thumbs: Node            # car_thumbnails.gd (may be null: placeholders only)
var screen_title: String = "Garage"
var choose_text: String = "Configure"
var initial_focus_id: String = ""

var _grid: Control
var _tiles: Dictionary = {}       # car id -> tile Control
var _headers: Dictionary = {}     # category index -> Label
var _stats_box: VBoxContainer
var _summary: Label
var _empty_label: Label
var _choose_button: Button
var _panel: PanelContainer
var _panel_box: VBoxContainer
var _filter_button: Button
var _controls: Dictionary = {}    # test hook id -> Control
var _rows := 3
var _vis_cols := 4
var _scroll_px := 0.0
var _last_fc := -1
var _focus_id := ""
var _stats_for := ""
var _mouse_motion_ms := -1000
var _stick_dir := ""
var _stick_t := 0.0
var _maxima: Dictionary = {}
var _built := false

func _ready() -> void:
	set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT) # in the tree already: plain set_anchors_preset keeps the current (empty) rect
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	if thumbs != null and thumbs.has_signal("thumbnail_ready"):
		thumbs.thumbnail_ready.connect(_on_thumbnail_ready)
	_build()

# ---- hooks / queries ------------------------------------------------------------

func focused_id() -> String:
	return str(garage.browser_get_focus_id())

func listed_ids() -> PackedStringArray:
	return garage.browser_get_listed_ids()

func instantiated_tile_count() -> int:
	return _tiles.size()

func rows() -> int:
	return _rows

func panel_open() -> bool:
	return _panel != null and _panel.visible

func get_control(id: String) -> Control:
	return _controls.get(id, null)

func control_ids() -> PackedStringArray:
	return PackedStringArray(_controls.keys())

# Esc / B: closes the filter panel first; true when it consumed the key.
func consume_back() -> bool:
	if panel_open():
		close_panel()
		return true
	return false

func nav(dir: String) -> void:
	if panel_open() or not garage.browser_move_focus(dir):
		return
	garage.browser_scroll_to_focus(_vis_cols)
	_after_focus_change()

func choose() -> void:
	if _focus_id != "":
		car_chosen.emit(_focus_id)

func focus_car(id: String) -> bool:
	if not garage.browser_set_focus(id):
		return false
	garage.browser_scroll_to_focus(_vis_cols)
	_after_focus_change()
	return true

func open_panel() -> void:
	if _panel == null:
		return
	_build_panel_contents()
	_panel.visible = true
	_focus_in_panel()

func close_panel() -> void:
	if _panel != null:
		_panel.visible = false
	if is_inside_tree():
		get_viewport().gui_release_focus() # the grid stays in charge of the keys

func toggle_filter(facet: String, value: String) -> void:
	garage.browser_toggle_filter(facet, value)
	_state_changed()

func set_group_by(id: String) -> void:
	garage.browser_set_group_by(id)
	_state_changed()

func set_sort(key: String, descending: bool) -> void:
	garage.browser_set_sort(key, descending)
	_state_changed()

# ---- build ----------------------------------------------------------------------

func _build() -> void:
	_maxima = garage.browser_get_maxima()
	var backdrop := ColorRect.new()
	backdrop.color = Color(0.045, 0.06, 0.08, 0.97)
	backdrop.set_anchors_preset(Control.PRESET_FULL_RECT)
	backdrop.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(backdrop)
	var margin := MarginContainer.new()
	margin.set_anchors_preset(Control.PRESET_FULL_RECT)
	for side in ["left", "right", "top", "bottom"]:
		margin.add_theme_constant_override("margin_" + side, 28)
	add_child(margin)
	var row := HBoxContainer.new()
	row.add_theme_constant_override("separation", 22)
	margin.add_child(row)

	# left: stats of the focused car
	var stats_panel := PanelContainer.new()
	stats_panel.custom_minimum_size = Vector2(STATS_W, 0)
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.03, 0.04, 0.055, 0.9)
	style.border_color = ACCENT
	style.border_width_left = 3
	style.set_corner_radius_all(6)
	style.set_content_margin_all(18)
	stats_panel.add_theme_stylebox_override("panel", style)
	row.add_child(stats_panel)
	_stats_box = VBoxContainer.new()
	_stats_box.add_theme_constant_override("separation", 6)
	stats_panel.add_child(_stats_box)

	# right: top bar, grid, footer
	var right := VBoxContainer.new()
	right.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	right.add_theme_constant_override("separation", 10)
	row.add_child(right)
	var top := HBoxContainer.new()
	top.add_theme_constant_override("separation", 14)
	right.add_child(top)
	var title_col := VBoxContainer.new()
	title_col.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	top.add_child(title_col)
	title_col.add_child(_make_label(screen_title, 34, false))
	_summary = _make_label("", 14, true)
	title_col.add_child(_summary)
	_filter_button = Button.new()
	_filter_button.text = "Filter and sort  (F)"
	_filter_button.add_theme_font_size_override("font_size", 18)
	_filter_button.focus_mode = Control.FOCUS_NONE
	_filter_button.pressed.connect(open_panel)
	top.add_child(_filter_button)
	_controls["filter_button"] = _filter_button

	_grid = Control.new()
	_grid.clip_contents = true
	_grid.size_flags_vertical = Control.SIZE_EXPAND_FILL
	_grid.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_grid.mouse_filter = Control.MOUSE_FILTER_PASS
	_grid.gui_input.connect(_on_grid_input)
	_grid.resized.connect(_relayout)
	right.add_child(_grid)
	_empty_label = _make_label("No car matches the filters.", 22, true)
	_empty_label.set_anchors_preset(Control.PRESET_CENTER)
	_empty_label.visible = false
	_grid.add_child(_empty_label)

	var footer := HBoxContainer.new()
	footer.add_theme_constant_override("separation", 12)
	right.add_child(footer)
	var back := Button.new()
	back.text = "Back"
	back.add_theme_font_size_override("font_size", 20)
	back.custom_minimum_size = Vector2(120, 0)
	back.focus_mode = Control.FOCUS_NONE
	back.pressed.connect(func(): back_requested.emit())
	footer.add_child(back)
	_controls["back"] = back
	var hint := _make_label("Arrows / D-pad move   Enter / A choose   F / Y filter and sort   Esc / B back   Page Up-Down / bumpers jump a category", 13, true)
	hint.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	hint.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	footer.add_child(hint)
	_choose_button = Button.new()
	_choose_button.text = choose_text
	_choose_button.add_theme_font_size_override("font_size", 20)
	_choose_button.custom_minimum_size = Vector2(180, 0)
	_choose_button.focus_mode = Control.FOCUS_NONE
	_choose_button.pressed.connect(choose)
	footer.add_child(_choose_button)
	_controls["choose"] = _choose_button

	_build_panel()
	_built = true
	# the initial focus: the requested car, else the selected one, else the first listed
	var want := initial_focus_id if initial_focus_id != "" else str(garage.get_selected_id())
	if not garage.browser_set_focus(want):
		# keep whatever the model focuses (the first listed car)
		pass
	_after_focus_change(true)
	call_deferred("_relayout")

func _make_label(text: String, size: int, dim: bool) -> Label:
	var label := Label.new()
	label.text = text
	label.add_theme_font_size_override("font_size", size)
	label.autowrap_mode = TextServer.AUTOWRAP_OFF
	if dim:
		label.modulate = Color(1, 1, 1, 0.7)
	return label

func _make_wrapped(text: String, size: int, dim: bool) -> Label:
	var label := _make_label(text, size, dim)
	label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	return label

# ---- layout -----------------------------------------------------------------------

func _relayout() -> void:
	if not _built or _grid == null:
		return
	var sz := _grid.size
	if sz.x < 40.0 or sz.y < 40.0:
		return
	var new_rows := int(garage.browser_rows_for_height(sz.y, TILE_H, GAP, HEADER_H))
	var new_cols := maxi(1, int(floor((sz.x + GAP) / COL_W)))
	var changed := new_rows != _rows or new_cols != _vis_cols or _last_fc < 0
	_rows = new_rows
	_vis_cols = new_cols
	garage.browser_set_rows(_rows)
	if changed:
		garage.browser_scroll_to_focus(_vis_cols)
		_scroll_px = float(garage.browser_get_first_column()) * COL_W
	_after_focus_change(true)

func _after_focus_change(force: bool = false) -> void:
	var id := str(garage.browser_get_focus_id())
	var focus_changed := id != _focus_id
	_focus_id = id
	var fc := int(garage.browser_get_first_column())
	if force or fc != _last_fc:
		_last_fc = fc
		_refresh_view()
	_mark_focus()
	if force or focus_changed:
		_fill_stats()
		if focus_changed:
			car_focused.emit(_focus_id)
	_update_summary()

func _refresh_view() -> void:
	var view: Dictionary = garage.browser_get_view(_vis_cols + 1, OVERSCAN)
	var wanted: Dictionary = {}
	for t in view.get("tiles", []):
		wanted[str(t["id"])] = t
	for id in _tiles.keys():
		if not wanted.has(id):
			(_tiles[id] as Control).queue_free()
			_tiles.erase(id)
	for id in wanted:
		var t: Dictionary = wanted[id]
		if not _tiles.has(id):
			_tiles[id] = _make_tile(str(id))
			_grid.add_child(_tiles[id])
		var tile: Control = _tiles[id]
		tile.set_meta("column", int(t["column"]))
		tile.set_meta("row", int(t["row"]))
	var wanted_headers: Dictionary = {}
	for h in view.get("headers", []):
		wanted_headers[int(h["category"])] = h
	for ci in _headers.keys():
		if not wanted_headers.has(ci):
			(_headers[ci] as Control).queue_free()
			_headers.erase(ci)
	for ci in wanted_headers:
		var h: Dictionary = wanted_headers[ci]
		if not _headers.has(ci):
			var label := _make_label("%s  (%d)" % [str(h["title"]), int(h["count"])], 20, false)
			label.add_theme_color_override("font_color", Color(1.0, 0.72, 0.45))
			label.mouse_filter = Control.MOUSE_FILTER_IGNORE
			_headers[ci] = label
			_grid.add_child(label)
		var header: Label = _headers[ci]
		header.visible = str(h["title"]) != ""
		header.set_meta("first_column", int(h["first_column"]))
		header.set_meta("column_count", int(h["column_count"]))
	_empty_label.visible = _tiles.is_empty()
	_layout_positions()

func _layout_positions() -> void:
	for id in _tiles:
		var tile: Control = _tiles[id]
		var col := int(tile.get_meta("column"))
		var row := int(tile.get_meta("row"))
		tile.position = Vector2(float(col) * COL_W - _scroll_px, HEADER_H + 8.0 + float(row) * (TILE_H + GAP))
	for ci in _headers:
		var header: Label = _headers[ci]
		var first := int(header.get_meta("first_column"))
		var count := int(header.get_meta("column_count"))
		var natural_x := float(first) * COL_W - _scroll_px
		var end_x := float(first + count) * COL_W - GAP - _scroll_px
		var label_w := header.get_minimum_size().x
		# sticky: stays at the left edge while its category is in view, pushed out by the next one
		var x := maxf(natural_x, minf(0.0, end_x - label_w))
		header.position = Vector2(x, 2.0)

func _process(delta: float) -> void:
	if not _built:
		return
	var fc := int(garage.browser_get_first_column())
	if fc != _last_fc:
		_last_fc = fc
		_refresh_view()
	var target := float(fc) * COL_W
	if absf(target - _scroll_px) > 0.4:
		_scroll_px = lerpf(_scroll_px, target, 1.0 - exp(-14.0 * delta))
		if absf(target - _scroll_px) <= 0.4:
			_scroll_px = target
		_layout_positions()
	_poll_stick(delta)

# ---- tiles -------------------------------------------------------------------------

func _make_tile(id: String) -> Control:
	var car: Dictionary = garage.browser_get_car(id)
	var tile := Control.new()
	tile.name = "Tile_" + id
	tile.custom_minimum_size = Vector2(TILE_W, TILE_H)
	tile.size = Vector2(TILE_W, TILE_H)
	tile.mouse_filter = Control.MOUSE_FILTER_STOP
	tile.set_meta("id", id)
	var bg := Panel.new()
	bg.name = "Bg"
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	bg.mouse_filter = Control.MOUSE_FILTER_IGNORE
	tile.add_child(bg)
	var thumb := TextureRect.new()
	thumb.name = "Thumb"
	thumb.position = Vector2(0, 0)
	thumb.size = Vector2(TILE_W, THUMB_H)
	thumb.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	thumb.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_COVERED
	thumb.mouse_filter = Control.MOUSE_FILTER_IGNORE
	tile.add_child(thumb)
	var placeholder := _make_label("...", 26, true)
	placeholder.name = "Placeholder"
	placeholder.position = Vector2(0, THUMB_H * 0.5 - 18.0)
	placeholder.size = Vector2(TILE_W, 36)
	placeholder.horizontal_alignment = HORIZONTAL_ALIGNMENT_CENTER
	placeholder.mouse_filter = Control.MOUSE_FILTER_IGNORE
	tile.add_child(placeholder)
	var title := _make_label(str(car.get("title", id)), 18, false)
	title.name = "Title"
	title.position = Vector2(10, THUMB_H + 6)
	title.size = Vector2(TILE_W - 20, 26)
	title.clip_text = true
	title.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	title.mouse_filter = Control.MOUSE_FILTER_IGNORE
	tile.add_child(title)
	var sub := _make_label(str(car.get("subtitle", "")), 13, true)
	sub.name = "Subtitle"
	sub.position = Vector2(10, THUMB_H + 32)
	sub.size = Vector2(TILE_W - 20, 20)
	sub.clip_text = true
	sub.text_overrun_behavior = TextServer.OVERRUN_TRIM_ELLIPSIS
	sub.mouse_filter = Control.MOUSE_FILTER_IGNORE
	tile.add_child(sub)
	var key := str(car.get("thumbnail_key", ""))
	tile.set_meta("thumb_key", key)
	_style_tile(tile, id == _focus_id)
	if thumbs != null:
		var tex: Texture2D = thumbs.request(key, str(car.get("model_path", "")), str(car.get("paint", "")), str(car.get("rim", "")))
		if tex != null:
			_set_tile_texture(tile, tex)
	tile.mouse_entered.connect(func(): _on_tile_hover(id))
	tile.gui_input.connect(func(event: InputEvent): _on_tile_input(id, event))
	return tile

func _set_tile_texture(tile: Control, tex: Texture2D) -> void:
	(tile.get_node("Thumb") as TextureRect).texture = tex
	(tile.get_node("Placeholder") as Control).visible = false

func _on_thumbnail_ready(key: String) -> void:
	for id in _tiles:
		var tile: Control = _tiles[id]
		if str(tile.get_meta("thumb_key", "")) == key and thumbs != null:
			var tex: Texture2D = thumbs.request(key, "", "", "")
			if tex != null:
				_set_tile_texture(tile, tex)

func _style_tile(tile: Control, focused: bool) -> void:
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.11, 0.13, 0.17, 1.0) if focused else Color(0.075, 0.09, 0.115, 1.0)
	style.set_corner_radius_all(6)
	style.set_border_width_all(3 if focused else 1)
	style.border_color = ACCENT if focused else Color(1, 1, 1, 0.10)
	(tile.get_node("Bg") as Panel).add_theme_stylebox_override("panel", style)

func _mark_focus() -> void:
	for id in _tiles:
		_style_tile(_tiles[id], id == _focus_id)

func _on_tile_hover(id: String) -> void:
	# Only a real mouse movement focuses: tiles sliding under a resting cursor while the keys scroll must not.
	if panel_open() or Time.get_ticks_msec() - _mouse_motion_ms > 150 or id == _focus_id:
		return
	if garage.browser_set_focus(id):
		_after_focus_change()

func _on_tile_input(id: String, event: InputEvent) -> void:
	if panel_open():
		return
	if event is InputEventMouseButton and event.pressed and event.button_index == MOUSE_BUTTON_LEFT:
		if garage.browser_set_focus(id):
			_after_focus_change()
		car_chosen.emit(id)
		accept_event()

func _on_grid_input(event: InputEvent) -> void:
	if panel_open():
		return
	if event is InputEventMouseButton and event.pressed:
		var delta := 0
		match event.button_index:
			MOUSE_BUTTON_WHEEL_UP, MOUSE_BUTTON_WHEEL_LEFT:
				delta = -1
			MOUSE_BUTTON_WHEEL_DOWN, MOUSE_BUTTON_WHEEL_RIGHT:
				delta = 1
		if delta != 0:
			garage.browser_scroll_by(delta * 2, _vis_cols)
			_after_focus_change()

# ---- stats --------------------------------------------------------------------------

func _fill_stats() -> void:
	if _focus_id == _stats_for and _stats_box.get_child_count() > 0:
		return
	_stats_for = _focus_id
	for child in _stats_box.get_children():
		_stats_box.remove_child(child)
		child.queue_free()
	if _focus_id == "":
		_stats_box.add_child(_make_wrapped("No car to show.", 16, true))
		_choose_button.disabled = true
		return
	_choose_button.disabled = false
	var v: Dictionary = garage.get_vehicle(_focus_id)
	if v.is_empty():
		return
	var title := _make_wrapped(str(v["title"]), 26, false)
	_stats_box.add_child(title)
	var tag := str(v.get("body_type", ""))
	var maker := str(v.get("manufacturer", ""))
	var line := tag.capitalize() if maker == "" else "%s - %s" % [maker, tag.capitalize()]
	_stats_box.add_child(_make_wrapped(line, 14, true))
	_stats_box.add_child(_make_wrapped(str(v["description"]), 13, true))
	_stats_box.add_child(HSeparator.new())
	var st: Dictionary = v["stats"]
	if not bool(st.get("ok", false)):
		_stats_box.add_child(_make_wrapped("stats unavailable: " + str(st.get("error", "")), 13, false))
		return
	var kw := float(st["peak_power_kw"])
	_stat_bar("Power", "%.0f kW at %.0f rpm" % [kw, float(st["peak_power_rpm"])], kw / maxf(float(_maxima.get("power_kw", 1.0)), 1.0))
	var nm := float(st["peak_torque_nm"])
	_stat_bar("Torque", "%.0f Nm at %.0f rpm" % [nm, float(st["peak_torque_rpm"])], nm / maxf(float(_maxima.get("torque_nm", 1.0)), 1.0))
	_stat_line("Mass", "%.0f kg" % float(st["mass_kg"]))
	if bool(st.get("displacement_known", false)):
		var litres := float(st["displacement_l"])
		_stat_line("Displacement", "%.2f L  (%.0f cm3)" % [litres, litres * 1000.0])
	else:
		_stat_line("Displacement", "-")
	_stat_line("Drive", "%s  (%d of %d wheels driven)" % [str(st["layout"]), int(st["driven_wheels"]), int(st["wheel_count"])])
	_stat_line("Gearbox", "%d-speed" % int(st["gear_count"]))
	_stat_line("Engine", str(st["engine_name"]))

func _stat_bar(label: String, value_text: String, ratio: float) -> void:
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 1)
	_stats_box.add_child(box)
	var line := HBoxContainer.new()
	box.add_child(line)
	var name_label := _make_label(label, 14, true)
	name_label.custom_minimum_size = Vector2(110, 0)
	line.add_child(name_label)
	var value_label := _make_label(value_text, 15, false)
	value_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	line.add_child(value_label)
	var bar := ProgressBar.new()
	bar.min_value = 0.0
	bar.max_value = 1.0
	bar.value = clampf(ratio, 0.0, 1.0)
	bar.show_percentage = false
	bar.custom_minimum_size = Vector2(0, 6)
	box.add_child(bar)

func _stat_line(label: String, value_text: String) -> void:
	var line := HBoxContainer.new()
	_stats_box.add_child(line)
	var name_label := _make_label(label, 14, true)
	name_label.custom_minimum_size = Vector2(110, 0)
	line.add_child(name_label)
	var value_label := _make_wrapped(value_text, 15, false)
	value_label.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	line.add_child(value_label)

func _update_summary() -> void:
	var layout: Dictionary = garage.browser_get_layout()
	var state: Dictionary = garage.browser_get_state()
	var text := "%d of %d cars" % [int(layout.get("listed", 0)), int(layout.get("total", 0))]
	text += "   |   grouped by %s" % str(state.get("group_label", "")).to_lower()
	text += "   |   sorted by %s %s" % [str(state.get("sort_label", "")).to_lower(), "descending" if bool(state.get("sort_desc", false)) else "ascending"]
	if bool(state.get("filter_active", false)):
		text += "   |   filtered"
	_summary.text = text
	_filter_button.text = "Filter and sort  (F)" + ("  *" if bool(state.get("filter_active", false)) else "")

# ---- the filter / sort panel ----------------------------------------------------------

func _build_panel() -> void:
	_panel = PanelContainer.new()
	_panel.visible = false
	_panel.set_anchors_preset(Control.PRESET_FULL_RECT)
	_panel.anchor_left = 0.5
	_panel.anchor_right = 1.0
	_panel.anchor_top = 0.0
	_panel.anchor_bottom = 1.0
	_panel.offset_left = 0
	_panel.offset_right = -28
	_panel.offset_top = 28
	_panel.offset_bottom = -28
	var style := StyleBoxFlat.new()
	style.bg_color = Color(0.025, 0.033, 0.048, 0.98)
	style.border_color = ACCENT
	style.set_border_width_all(2)
	style.set_corner_radius_all(8)
	style.set_content_margin_all(20)
	_panel.add_theme_stylebox_override("panel", style)
	add_child(_panel)
	var scroll := ScrollContainer.new()
	scroll.horizontal_scroll_mode = ScrollContainer.SCROLL_MODE_DISABLED
	_panel.add_child(scroll)
	_panel_box = VBoxContainer.new()
	_panel_box.size_flags_horizontal = Control.SIZE_EXPAND_FILL
	_panel_box.add_theme_constant_override("separation", 10)
	scroll.add_child(_panel_box)

func _build_panel_contents() -> void:
	for child in _panel_box.get_children():
		_panel_box.remove_child(child)
		child.queue_free()
	for key in _controls.keys():
		if str(key).begins_with("group:") or str(key).begins_with("sort") or str(key).begins_with("filter:") or key == "panel_close" or key == "panel_clear":
			_controls.erase(key)
	var state: Dictionary = garage.browser_get_state()
	_panel_box.add_child(_make_label("Filter and sort", 26, false))
	# group by
	_panel_box.add_child(_make_label("Group by", 14, true))
	var group_row := HFlowContainer.new()
	group_row.add_theme_constant_override("h_separation", 6)
	group_row.add_theme_constant_override("v_separation", 6)
	_panel_box.add_child(group_row)
	for choice in garage.browser_get_group_choices():
		var id := str(choice["id"])
		var b := _toggle(str(choice["label"]), id == str(state["group_by"]))
		b.pressed.connect(func(): set_group_by(id); _build_panel_contents(); _focus_in_panel())
		group_row.add_child(b)
		_controls["group:" + id] = b
	# sort
	_panel_box.add_child(_make_label("Sort by", 14, true))
	var sort_row := HFlowContainer.new()
	sort_row.add_theme_constant_override("h_separation", 6)
	sort_row.add_theme_constant_override("v_separation", 6)
	_panel_box.add_child(sort_row)
	var descending := bool(state["sort_desc"])
	for choice in garage.browser_get_sort_choices():
		var id := str(choice["id"])
		var b := _toggle(str(choice["label"]), id == str(state["sort_key"]))
		b.pressed.connect(func(): set_sort(id, descending); _build_panel_contents(); _focus_in_panel())
		sort_row.add_child(b)
		_controls["sort:" + id] = b
	var desc := _toggle("Descending", descending)
	desc.pressed.connect(func(): set_sort(str(state["sort_key"]), not descending); _build_panel_contents(); _focus_in_panel())
	sort_row.add_child(desc)
	_controls["sort_desc"] = desc
	# filters
	for facet in [["layout", "Drive layout"], ["body_type", "Body type"], ["power_band", "Power"]]:
		_panel_box.add_child(_make_label(str(facet[1]), 14, true))
		var frow := HFlowContainer.new()
		frow.add_theme_constant_override("h_separation", 6)
		frow.add_theme_constant_override("v_separation", 6)
		_panel_box.add_child(frow)
		for opt in garage.browser_get_filter_options(str(facet[0])):
			var value := str(opt["value"])
			var fid := str(facet[0])
			var b := _toggle("%s  (%d)" % [str(opt["label"]), int(opt["count"])], bool(opt["selected"]))
			b.pressed.connect(func(): toggle_filter(fid, value); _build_panel_contents(); _focus_in_panel())
			frow.add_child(b)
			_controls["filter:%s:%s" % [fid, value]] = b
	var footer := HBoxContainer.new()
	footer.add_theme_constant_override("separation", 10)
	_panel_box.add_child(footer)
	var clear := Button.new()
	clear.text = "Clear filters"
	clear.add_theme_font_size_override("font_size", 18)
	clear.pressed.connect(func(): garage.browser_clear_filter(); _state_changed(); _build_panel_contents(); _focus_in_panel())
	footer.add_child(clear)
	_controls["panel_clear"] = clear
	var close := Button.new()
	close.text = "Close"
	close.add_theme_font_size_override("font_size", 18)
	close.pressed.connect(close_panel)
	footer.add_child(close)
	_controls["panel_close"] = close

func _toggle(text: String, pressed: bool) -> Button:
	var b := Button.new()
	b.text = text
	b.toggle_mode = true
	b.button_pressed = pressed
	b.add_theme_font_size_override("font_size", 17)
	if pressed:
		b.add_theme_color_override("font_color", Color(1.0, 0.62, 0.3))
		b.add_theme_color_override("font_pressed_color", Color(1.0, 0.62, 0.3))
	return b

func _focus_in_panel() -> void:
	call_deferred("_grab_panel_focus")

func _grab_panel_focus() -> void:
	if not panel_open():
		return
	var first: Control = _controls.get("panel_close", null)
	for key in _controls:
		if str(key).begins_with("group:") and (_controls[key] as Button).button_pressed:
			first = _controls[key]
			break
	if first != null and first.is_inside_tree():
		first.grab_focus()

func _state_changed() -> void:
	garage.browser_scroll_to_focus(_vis_cols)
	_last_fc = -1
	_after_focus_change(true)
	state_changed.emit()

# ---- input ------------------------------------------------------------------------------

func _input(event: InputEvent) -> void:
	if not is_visible_in_tree() or not _built:
		return
	if event is InputEventMouseMotion:
		_mouse_motion_ms = Time.get_ticks_msec()
		return
	if panel_open():
		if event is InputEventKey and event.pressed and not event.echo and event.keycode == KEY_F:
			close_panel()
			get_viewport().set_input_as_handled()
		elif event is InputEventJoypadButton and event.pressed and (event.button_index == JOY_BUTTON_Y or event.button_index == JOY_BUTTON_B):
			close_panel()
			get_viewport().set_input_as_handled()
		return
	if event is InputEventKey and event.pressed:
		var handled := true
		match event.keycode:
			KEY_LEFT:
				nav("left")
			KEY_RIGHT:
				nav("right")
			KEY_UP:
				nav("up")
			KEY_DOWN:
				nav("down")
			KEY_PAGEUP:
				nav("prev_category")
			KEY_PAGEDOWN:
				nav("next_category")
			KEY_ENTER, KEY_KP_ENTER, KEY_SPACE:
				if not event.echo:
					choose()
			KEY_F:
				if not event.echo:
					open_panel()
			_:
				handled = false
		if handled:
			get_viewport().set_input_as_handled()
	elif event is InputEventJoypadButton and event.pressed:
		var handled := true
		match event.button_index:
			JOY_BUTTON_DPAD_LEFT:
				nav("left")
			JOY_BUTTON_DPAD_RIGHT:
				nav("right")
			JOY_BUTTON_DPAD_UP:
				nav("up")
			JOY_BUTTON_DPAD_DOWN:
				nav("down")
			JOY_BUTTON_LEFT_SHOULDER:
				nav("prev_category")
			JOY_BUTTON_RIGHT_SHOULDER:
				nav("next_category")
			JOY_BUTTON_A:
				choose()
			JOY_BUTTON_Y:
				open_panel()
			JOY_BUTTON_B:
				back_requested.emit()
			_:
				handled = false
		if handled:
			get_viewport().set_input_as_handled()

# The left stick moves the focus like the D-pad, repeating while held.
func _poll_stick(delta: float) -> void:
	if panel_open() or not is_visible_in_tree():
		_stick_dir = ""
		return
	var pads := Input.get_connected_joypads()
	if pads.is_empty():
		return
	var x := Input.get_joy_axis(pads[0], JOY_AXIS_LEFT_X)
	var y := Input.get_joy_axis(pads[0], JOY_AXIS_LEFT_Y)
	var dir := ""
	if absf(x) >= absf(y) and absf(x) > STICK_THRESHOLD:
		dir = "right" if x > 0.0 else "left"
	elif absf(y) > STICK_THRESHOLD:
		dir = "down" if y > 0.0 else "up"
	if dir == "":
		_stick_dir = ""
		return
	if dir != _stick_dir:
		_stick_dir = dir
		_stick_t = 0.0
		nav(dir)
		return
	_stick_t += delta
	if _stick_t >= STICK_REPEAT_S:
		_stick_t = 0.0
		nav(dir)
