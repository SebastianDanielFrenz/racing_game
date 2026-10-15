extends Control
const FrameProbe := preload("res://scripts/frame_probe.gd")
# R4 minimap (thin): draws the polylines of RgNavigation's map view-model and
# the planned route, heading-up around the car. No logic: the geometry, its
# LOD (in-game "Map detail" setting) and the route all come from rg_core.
# Session frame: x east, y north; the car's yaw is the session heading.

var radius_m := 800.0
var _lines: Array = []
var _route := PackedVector2Array()
var _car := Vector2.ZERO
var _yaw_rad := 0.0

const ROAD_COLOURS := [Color(0.55, 0.55, 0.55), Color(0.85, 0.85, 0.85), Color(1.0, 0.8, 0.35)]
const ROUTE_COLOUR := Color(0.25, 0.65, 1.0)

func _ready() -> void:
	clip_contents = true
	mouse_filter = Control.MOUSE_FILTER_IGNORE

func set_map(map: Dictionary) -> void:
	_lines = map.get("lines", [])
	queue_redraw()

func set_route(polyline: PackedVector2Array) -> void:
	_route = polyline
	queue_redraw()

func set_car(session_xy: Vector2, yaw_deg: float) -> void:
	_car = session_xy
	_yaw_rad = deg_to_rad(yaw_deg)
	queue_redraw()

# _draw runs in the deferred canvas redraw after every _process (not inside the _process probe): timed on its own.
func _draw() -> void:
	var probe_t0 := Time.get_ticks_usec()
	_draw_body()
	FrameProbe.add("minimap_draw", Time.get_ticks_usec() - probe_t0)

func _draw_body() -> void:
	var centre := size * 0.5
	var r: float = min(size.x, size.y) * 0.5
	draw_rect(Rect2(Vector2.ZERO, size), Color(0.05, 0.06, 0.08, 0.7))
	var s: float = r / max(radius_m, 1.0)
	# Heading-up: the car's heading points up, the session y axis follows.
	var c := cos(-_yaw_rad)
	var si := sin(-_yaw_rad)
	var x_axis := Vector2(-s * si, -s * c)
	var y_axis := Vector2(-s * c, s * si)
	var origin := centre - x_axis * _car.x - y_axis * _car.y
	draw_set_transform_matrix(Transform2D(x_axis, y_axis, origin))
	for line in _lines:
		var rank: int = int(line["rank"])
		var colour: Color = ROAD_COLOURS[0] if rank < 4 else (ROAD_COLOURS[1] if rank < 7 else ROAD_COLOURS[2])
		draw_polyline(line["points"], colour, (1.0 + rank * 0.25) / s)
	if _route.size() >= 2:
		draw_polyline(_route, ROUTE_COLOUR, 3.0 / s)
	draw_set_transform_matrix(Transform2D.IDENTITY)
	# The car, always at the centre pointing up.
	draw_colored_polygon(PackedVector2Array([centre + Vector2(0, -8), centre + Vector2(6, 6), centre + Vector2(-6, 6)]), Color(1, 0.3, 0.2))
