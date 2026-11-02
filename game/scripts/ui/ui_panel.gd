extends PanelContainer
# game/scripts/ui/ui_panel.gd - the design language's panel (docs/design_language.md): a surface with one accent edge, the
# padding and corner of the tokens. Put content in it as in any PanelContainer.
#     const UiPanel := preload("res://scripts/ui/ui_panel.gd")
#     var panel := UiPanel.new()                          # surface_panel, accent on the left
#     var clock := UiPanel.new(&"surface_widget", &"")    # no edge
# Over the running scene use surface_panel or surface_widget, thick enough to read over a sunlit field; inside a menu any surface.

const UiTheme := preload("res://scripts/ui/ui_theme.gd")

enum Edge { LEFT, RIGHT }

func _init(fill: StringName = &"surface_panel", edge: StringName = &"accent", side: Edge = Edge.LEFT) -> void:
	var box := UiTheme.style(fill, edge)
	box.set_corner_radius_all(int(UiTheme.token(&"radius.panel")))
	box.set_content_margin_all(UiTheme.token(&"panel.padding"))
	if edge != &"":
		var width := int(UiTheme.token(&"panel.accent_edge_width"))
		box.border_width_left = width if side == Edge.LEFT else 0
		box.border_width_right = width if side == Edge.RIGHT else 0
	add_theme_stylebox_override(&"panel", box)
