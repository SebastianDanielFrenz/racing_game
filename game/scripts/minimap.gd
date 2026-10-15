extends Control
# R4 minimap (thin): draws the polylines of RgNavigation's map view-model and
# the planned route, heading-up around the car. No logic: the geometry, its
# LOD (in-game "Map detail" setting) and the route all come from rg_core.
# Session frame: x east, y north; the car's yaw is the session heading.
#
# Cost (perf, 2026-10-15): the lines are drawn once, in session coordinates,
# by the child _MapLayer, and redrawn only when the map, the route or the
# size changes. A car move only sets the layer's transform, so the renderer
# replays the recorded canvas commands and no GDScript draw code runs per
# frame. Drawing every polyline every frame cost 3.8 ms per frame in the
# owner's 4K log.

var radius_m := 800.0:
	set(value):
		radius_m = value
		_redraw_layers()

var _lines: Array = []
var _route := PackedVector2Array()
var _car := Vector2.ZERO
var _yaw_rad := 0.0
var _layer: _MapLayer
var _marker: _CarMarker

const ROAD_COLOURS := [Color(0.55, 0.55, 0.55), Color(0.85, 0.85, 0.85), Color(1.0, 0.8, 0.35)]
const ROUTE_COLOUR := Color(0.25, 0.65, 1.0)
const BACKGROUND := Color(0.05, 0.06, 0.08, 0.7)

# The road and route lines in session coordinates; its transform maps them heading-up onto the minimap.
class _MapLayer extends Node2D:
	var map: Control
	func _draw() -> void:
		var probe_t0 := Time.get_ticks_usec()
		map._draw_lines(self)
		preload("res://scripts/frame_probe.gd").add("minimap_draw", Time.get_ticks_usec() - probe_t0)

# The car, always at the centre pointing up, drawn over the lines.
class _CarMarker extends Node2D:
	func _draw() -> void:
		draw_colored_polygon(PackedVector2Array([Vector2(0, -8), Vector2(6, 6), Vector2(-6, 6)]), Color(1, 0.3, 0.2))

func _ready() -> void:
	clip_contents = true
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	_layer = _MapLayer.new()
	_layer.map = self
	add_child(_layer)
	_marker = _CarMarker.new()
	add_child(_marker)
	resized.connect(_on_resized)
	_on_resized()

func set_map(map: Dictionary) -> void:
	_lines = map.get("lines", [])
	_redraw_layers()

func set_route(polyline: PackedVector2Array) -> void:
	_route = polyline
	_redraw_layers()

func set_car(session_xy: Vector2, yaw_deg: float) -> void:
	_car = session_xy
	_yaw_rad = deg_to_rad(yaw_deg)
	_place_layer()

func _draw() -> void:
	draw_rect(Rect2(Vector2.ZERO, size), BACKGROUND)

func _on_resized() -> void:
	if _marker != null:
		_marker.position = size * 0.5
	_redraw_layers()
	queue_redraw()

func _redraw_layers() -> void:
	if _layer == null:
		return
	_layer.queue_redraw()
	_place_layer()

# Session metres -> minimap pixels.
func _scale() -> float:
	var r: float = min(size.x, size.y) * 0.5
	return r / max(radius_m, 1.0)

func _place_layer() -> void:
	if _layer == null:
		return
	var s := _scale()
	# Heading-up: the car's heading points up, the session y axis follows.
	var c := cos(-_yaw_rad)
	var si := sin(-_yaw_rad)
	var x_axis := Vector2(-s * si, -s * c)
	var y_axis := Vector2(-s * c, s * si)
	var origin := size * 0.5 - x_axis * _car.x - y_axis * _car.y
	_layer.transform = Transform2D(x_axis, y_axis, origin)

# Called by _MapLayer._draw: widths are in session metres, so they come out in pixels after the layer's scale.
func _draw_lines(layer: Node2D) -> void:
	var s: float = max(_scale(), 1e-6)
	for line in _lines:
		var rank: int = int(line["rank"])
		var colour: Color = ROAD_COLOURS[0] if rank < 4 else (ROAD_COLOURS[1] if rank < 7 else ROAD_COLOURS[2])
		layer.draw_polyline(line["points"], colour, (1.0 + rank * 0.25) / s)
	if _route.size() >= 2:
		layer.draw_polyline(_route, ROUTE_COLOUR, 3.0 / s)
