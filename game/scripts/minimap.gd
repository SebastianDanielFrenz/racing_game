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
var _mesh: ArrayMesh

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
# Every line goes into ONE triangle mesh with vertex colours (one draw call): each draw_polyline is its own canvas
# draw call in the GL Compatibility renderer, ~1400 per frame on a real-world map (owner's 4K log, 2026-10-15).
func _draw_lines(layer: Node2D) -> void:
	var s: float = max(_scale(), 1e-6)
	var verts := PackedVector2Array()
	var colours := PackedColorArray()
	for line in _lines:
		var rank: int = int(line["rank"])
		var colour: Color = ROAD_COLOURS[0] if rank < 4 else (ROAD_COLOURS[1] if rank < 7 else ROAD_COLOURS[2])
		_strip(line["points"], colour, (1.0 + rank * 0.25) / s, verts, colours)
	if _route.size() >= 2:
		_strip(_route, ROUTE_COLOUR, 3.0 / s, verts, colours)
	if verts.is_empty():
		return
	var arrays := []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = verts
	arrays[Mesh.ARRAY_COLOR] = colours
	# Held by the minimap: the canvas command only references the mesh, which must outlive the recorded draw.
	_mesh = ArrayMesh.new()
	_mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	layer.draw_mesh(_mesh, null)

# Port of Godot's canvas_item_add_polyline strip (width > 0, not antialiased; renderer_canvas_cull.cpp): clamped miter
# joints (|1/sin| <= 3), end offsets from the first / last non-zero segment, closed lines joined at the seam.
static func _strip(points: PackedVector2Array, colour: Color, width: float, verts: PackedVector2Array, colours: PackedColorArray) -> void:
	var n := points.size()
	if n < 2:
		return
	var loop := points[0].is_equal_approx(points[n - 1])
	var first_dir := Vector2.ZERO
	for i in range(1, n):
		first_dir = (points[i] - points[i - 1]).normalized()
		if not first_dir.is_zero_approx():
			break
	var last_dir := Vector2.ZERO
	for i in range(n - 1, 0, -1):
		last_dir = (points[i] - points[i - 1]).normalized()
		if not last_dir.is_zero_approx():
			break
	var left := PackedVector2Array()
	var right := PackedVector2Array()
	left.resize(n)
	right.resize(n)
	var prev_dir := Vector2.ZERO
	for i in n:
		var dir := prev_dir
		if i < n - 1:
			dir = (points[i + 1] - points[i]).normalized()
			if dir.is_zero_approx():
				dir = prev_dir
		if i == 0 and loop:
			prev_dir = last_dir
		elif i == n - 1 and loop:
			prev_dir = first_dir
		var base: Vector2
		if i == 0 and not loop:
			base = first_dir.orthogonal()
		elif i == n - 1 and not loop:
			base = last_dir.orthogonal()
		else:
			base = _edge_offset(dir, prev_dir)
		var offset := base * (width * 0.5)
		left[i] = points[i] + offset
		right[i] = points[i] - offset
		prev_dir = dir
	for i in n - 1:
		verts.append_array([left[i], right[i], left[i + 1], right[i], right[i + 1], left[i + 1]])
		for k in 6:
			colours.append(colour)

static func _edge_offset(dir: Vector2, prev_dir: Vector2) -> Vector2:
	var length := 1.0
	var bisector := (prev_dir * dir.length() - dir * prev_dir.length()).normalized()
	var angle := atan2(bisector.cross(prev_dir), bisector.dot(prev_dir))
	var sin_angle := sin(angle)
	if not is_zero_approx(sin_angle) and not dir.is_equal_approx(prev_dir):
		length = clampf(1.0 / sin_angle, -3.0, 3.0)
	else:
		bisector = dir.orthogonal()
	if bisector.is_zero_approx():
		bisector = dir.orthogonal()
	return bisector * length
