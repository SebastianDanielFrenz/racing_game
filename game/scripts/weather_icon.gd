extends Control
# Procedural weather icon (no image files): sun, sun_cloud, cloud, moon,
# moon_cloud, rain, snow, hail, thunder - the names rg::weather_icon_name
# returns. `background` must be the colour behind the icon (the moon's
# crescent is cut with it).

const ICONS := ["sun", "sun_cloud", "cloud", "moon", "moon_cloud", "rain", "snow", "hail", "thunder"]
const SUN := Color(1.0, 0.8, 0.2)
const MOON := Color(0.92, 0.92, 0.8)
const CLOUD := Color(0.86, 0.88, 0.92)
const DARK_CLOUD := Color(0.5, 0.52, 0.58)
const RAIN := Color(0.4, 0.65, 1.0)
const BOLT := Color(1.0, 0.85, 0.1)

var icon := "sun":
	set(value):
		icon = value
		queue_redraw()
var background := Color(0.08, 0.09, 0.11)

func _draw() -> void:
	var s: float = min(size.x, size.y)
	var c := size * 0.5
	match icon:
		"sun": _sun(c, s * 0.22)
		"moon": _moon(c, s * 0.3)
		"sun_cloud":
			_sun(c + Vector2(-0.14, -0.14) * s, s * 0.17)
			_cloud(c + Vector2(0.06, 0.1) * s, s * 0.62, CLOUD)
		"moon_cloud":
			_moon(c + Vector2(-0.14, -0.14) * s, s * 0.22)
			_cloud(c + Vector2(0.06, 0.1) * s, s * 0.62, CLOUD)
		"cloud": _cloud(c, s * 0.8, CLOUD)
		"rain":
			_cloud(c - Vector2(0, 0.12) * s, s * 0.75, DARK_CLOUD)
			for i in 3:
				var p := c + Vector2((i - 1) * 0.2, 0.16) * s
				draw_line(p, p + Vector2(-0.06, 0.2) * s, RAIN, max(2.0, s * 0.05))
		"snow":
			_cloud(c - Vector2(0, 0.12) * s, s * 0.75, DARK_CLOUD)
			for i in 3:
				_flake(c + Vector2((i - 1) * 0.22, 0.27 + 0.05 * (i % 2)) * s, s * 0.07)
		"hail":
			_cloud(c - Vector2(0, 0.12) * s, s * 0.75, DARK_CLOUD)
			for i in 3:
				draw_circle(c + Vector2((i - 1) * 0.2, 0.25 + 0.06 * (i % 2)) * s, s * 0.05, Color.WHITE)
		"thunder":
			_cloud(c - Vector2(0, 0.12) * s, s * 0.75, DARK_CLOUD)
			var bolt := PackedVector2Array([Vector2(0.04, 0.02), Vector2(-0.1, 0.24), Vector2(0.0, 0.24), Vector2(-0.06, 0.44), Vector2(0.12, 0.16), Vector2(0.02, 0.16), Vector2(0.1, 0.02)])
			for i in bolt.size():
				bolt[i] = c + bolt[i] * s
			draw_colored_polygon(bolt, BOLT)

func _sun(c: Vector2, r: float) -> void:
	for i in 8:
		var d := Vector2.RIGHT.rotated(i * PI / 4.0)
		draw_line(c + d * r * 1.35, c + d * r * 1.8, SUN, max(2.0, r * 0.18))
	draw_circle(c, r, SUN)

func _moon(c: Vector2, r: float) -> void:
	draw_circle(c, r, MOON)
	draw_circle(c + Vector2(r * 0.55, -r * 0.3), r * 0.85, background)

func _cloud(c: Vector2, w: float, color: Color) -> void:
	draw_circle(c + Vector2(-0.22, 0.05) * w, w * 0.2, color)
	draw_circle(c + Vector2(0.0, -0.08) * w, w * 0.27, color)
	draw_circle(c + Vector2(0.24, 0.06) * w, w * 0.19, color)
	draw_rect(Rect2(c + Vector2(-0.22, 0.05) * w, Vector2(0.46, 0.19) * w), color)

func _flake(c: Vector2, r: float) -> void:
	for i in 3:
		var d := Vector2.RIGHT.rotated(i * PI / 3.0) * r
		draw_line(c - d, c + d, Color.WHITE, max(1.5, r * 0.3))
