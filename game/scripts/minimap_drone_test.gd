extends SceneTree
# game/scripts/minimap_drone_test.gd - the minimap's drone view, headless, no world:
#   godot --headless --path game --script res://scripts/minimap_drone_test.gd
#
# 220 px across 2 * 1000 m is 0.11 px/m. The session frame is x east, y north,
# yaw 0 = east; the drone's heading points up, so with yaw 0 north is on the left.
# Prints RG_MINIMAP_TEST PASS checks=N, or a RG_MINIMAP_TEST FAIL line per failed check.

var _checks := 0
var _failed := 0

func _initialize() -> void:
	call_deferred("_test")

func _check(what: String, ok: bool) -> void:
	_checks += 1
	if not ok:
		_failed += 1
		print("RG_MINIMAP_TEST FAIL ", what)

func _near(what: String, got: Vector2, want: Vector2) -> void:
	_check("%s: %s, want %s" % [what, got, want], got.distance_to(want) < 1e-3)

func _nose(marker: Node2D) -> Vector2:
	return marker.transform.basis_xform(Vector2.UP)

func _test() -> void:
	var map := Control.new()
	map.set_script(load("res://scripts/minimap.gd"))
	root.add_child(map)
	map.size = Vector2(220, 220)
	map.radius_m = 1000.0
	var car: Node2D = map._marker
	var drone: Node2D = map._drone_marker
	var middle := Vector2(110, 110)

	map.set_car(Vector2(100, 200), 30.0)
	_near("car mode: car at the centre", car.position, middle)
	_near("car mode: car points up", _nose(car), Vector2.UP)
	_check("car mode: no drone", not drone.visible)

	map.set_drone(Vector2.ZERO, 0.0, Vector2(100, 0), 0.0)
	_check("drone mode: drone shown", drone.visible)
	_near("drone at the centre", drone.position, middle)
	_near("car ahead of the drone is above it", car.position, middle + Vector2(0, -11))
	_near("car heading the same way points up", _nose(car), Vector2.UP)
	_check("car on the map is fully opaque", car.modulate.a == 1.0)

	map.set_drone(Vector2.ZERO, 0.0, Vector2(0, 100), 90.0)
	_near("north is left with the drone facing east", car.position, middle + Vector2(-11, 0))
	_near("a car heading north points left", _nose(car), Vector2.LEFT)

	map.set_drone(Vector2(500, 500), 90.0, Vector2(500, 600), 0.0)
	_near("the drone turning north turns the map", car.position, middle + Vector2(0, -11))
	_near("a car heading east then points right", _nose(car), Vector2.RIGHT)

	map.set_drone(Vector2.ZERO, 0.0, Vector2(100000, 0), 0.0)
	_near("a far car is pinned inside the edge", car.position, Vector2(110, 10))
	_check("a pinned car is faded", car.modulate.a < 1.0)
	_check("a pinned car stays shown", car.visible and car.modulate.a > 0.0)

	map.set_drone(Vector2.ZERO, 0.0, Vector2(100000, 100000), 90.0)
	_near("a far diagonal car keeps its direction", car.position, Vector2(10, 10))
	_near("a pinned car still shows its heading", _nose(car), Vector2.LEFT)

	map.size = Vector2(300, 300)
	_near("the drone stays centred on a resize", drone.position, Vector2(150, 150))
	map.size = Vector2(220, 220)

	map.set_car(Vector2.ZERO, 0.0)
	_near("back in the car: centred", car.position, middle)
	_near("back in the car: points up", _nose(car), Vector2.UP)
	_check("back in the car: opaque", car.modulate.a == 1.0)
	_check("back in the car: no drone", not drone.visible)

	if _failed == 0:
		print("RG_MINIMAP_TEST PASS checks=", _checks)
	quit(1 if _failed > 0 else 0)
