extends Button
# game/scripts/ui/ui_image_tile.gd - the design language's image tile (docs/design_language.md): a picture you press, for the
# scrolling grids of a menu. The picture fills the tile; a caption is optional and sits on a band, so it reads on any picture.
#     const UiImageTile := preload("res://scripts/ui/ui_image_tile.gd")
#     grid.add_child(UiImageTile.new(thumbnail, "Tractor"))
#     tile.pressed.connect(_open)
# Size comes from tile.width and tile.height, the grid's gap from tile.gap. Focus and hover show the same way for mouse,
# keyboard and gamepad: a brighter card and the accent edge. The picture is content, a palette does not touch it.

const UiTheme := preload("res://scripts/ui/ui_theme.gd")

var _picture := TextureRect.new()
var _band := PanelContainer.new()
var _caption := Label.new()

func _init(texture: Texture2D = null, caption := "") -> void:
	custom_minimum_size = Vector2(UiTheme.token(&"tile.width"), UiTheme.token(&"tile.height"))
	focus_mode = Control.FOCUS_ALL
	clip_contents = true
	var inset := UiTheme.token(&"stroke.edge")
	_picture.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT, Control.PRESET_MODE_MINSIZE, int(inset))
	_picture.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_picture.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_COVERED
	_picture.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_picture)
	var band_box := UiTheme.style(&"surface_chip")
	band_box.content_margin_left = UiTheme.token(&"space.s")
	band_box.content_margin_right = UiTheme.token(&"space.s")
	_band.add_theme_stylebox_override(&"panel", band_box)
	_band.mouse_filter = Control.MOUSE_FILTER_IGNORE
	_band.anchor_top = 1.0
	_band.anchor_right = 1.0
	_band.anchor_bottom = 1.0
	_band.offset_left = inset
	_band.offset_right = -inset
	_band.offset_bottom = -inset
	_band.offset_top = -UiTheme.token(&"tile.label_band") - inset
	_caption.vertical_alignment = VERTICAL_ALIGNMENT_CENTER
	_caption.add_theme_font_size_override(&"font_size", int(UiTheme.token(&"type.small")))
	UiTheme.text(_caption, &"text")
	_band.add_child(_caption)
	add_child(_band)
	for item: StringName in [&"normal", &"disabled"]:
		add_theme_stylebox_override(item, _box(&"surface_card", &"border_subtle"))
	for item: StringName in [&"hover", &"focus"]:
		add_theme_stylebox_override(item, _box(&"surface_card_focus", &"accent"))
	for item: StringName in [&"pressed", &"hover_pressed"]:
		add_theme_stylebox_override(item, _box(&"surface_card_focus", &"border_selected"))
	setup(texture, caption)

func setup(texture: Texture2D, caption: String) -> void:
	_picture.texture = texture
	_caption.text = caption
	_band.visible = caption != ""
	tooltip_text = caption

func set_available(available: bool) -> void:
	disabled = not available
	modulate.a = 1.0 if available else UiTheme.token(&"state.disabled_alpha")

func _box(fill: StringName, edge: StringName) -> StyleBoxFlat:
	var box := UiTheme.style(fill, edge)
	box.set_corner_radius_all(int(UiTheme.token(&"radius.tile")))
	box.set_border_width_all(int(UiTheme.token(&"stroke.edge")))
	return box
