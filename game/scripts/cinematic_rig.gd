extends Node3D
# game/scripts/cinematic_rig.gd - the cinematic view (rg::DriveView::Cinematic,
# PLAN.md R5): roadside cameras ahead of the car that track it as it passes and
# cut to the next one. A rig like chase_rig.gd (root Node3D + Camera3D child;
# camera_director.gd shifts the root on a floating-origin rebase and calls
# update_rig() only while this rig is active).
#
# Every decision is rg_core's (rg::CinematicDirector via RgSimulation.
# update_cinematic): where the next camera stands (beside the road ahead of the
# car from the client's road data - FrameSnapshot::road_ahead - or, where there
# is none (the flat world, off-road), beside the car's own predicted path;
# "chase_fallback" = no clear spot (buildings/terrain in the way), framed from behind;
# "source" says which), how high, which field of view and when to cut. A shot is
# a SESSION-frame point: this script converts it to the Godot frame every frame
# with RgSimulation.session_to_godot, so a floating-origin rebase can never move
# a shot (PHYS-008); shift_origin() only has to keep the held ground height in
# step. The camera stands on the ground (RgSimulation.get_camera_ground_height
# where terrain is loaded, else the flat world's z = 0 plane) at the shot's
# height above it and looks at the car.

const GROUND_FOLLOW := 6.0 # smoothing of the held ground height (1/s)
const LOOK_HEIGHT_M := 0.9

var camera: Camera3D

var _needs_reset: bool = true
var _serial: int = -1
var _ground_y: float = 0.0
var _have_ground: bool = false
var last_shot: Dictionary = {} # the director's last answer (tests and diagnostics read it)

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.near = 0.1
	camera.far = 43000.0
	camera.fov = 45.0
	add_child(camera)

func set_base_fov(_base_fov_deg: float) -> void:
	pass # a shot's field of view is chosen from its distance by rg_core

func activate(_from: Transform3D) -> void:
	_needs_reset = true # a fresh shot; the old one may be far behind
	_have_ground = false

func shift_origin(delta: Vector3) -> void:
	position += delta
	_ground_y += delta.y

func update_rig(delta: float, simulation: Node, _input_map: Node, _input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		return
	if _needs_reset:
		simulation.reset_cinematic()
		_needs_reset = false
	var shot: Dictionary = simulation.update_cinematic(delta)
	if shot.is_empty():
		return
	last_shot = shot
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var flat: Vector3 = simulation.session_to_godot(Vector3(float(shot["session_x"]), float(shot["session_y"]), 0.0))
	var ground = simulation.get_camera_ground_height(flat)
	var ground_y: float = flat.y
	if ground != null:
		ground_y = float(ground)
	elif simulation.is_terrain_mode():
		ground_y = chassis.origin.y - 0.4 # tile not resident yet: stay at the car's own height
	if not _have_ground or int(shot["serial"]) != _serial:
		_ground_y = ground_y
		_have_ground = true
	else:
		_ground_y = lerpf(_ground_y, ground_y, 1.0 - exp(-GROUND_FOLLOW * delta))
	_serial = int(shot["serial"])
	position = Vector3(flat.x, _ground_y + float(shot["height_above_ground_m"]), flat.z)
	camera.fov = float(shot["fov_deg"])
	var target: Vector3 = chassis.origin + Vector3.UP * LOOK_HEIGHT_M
	if position.distance_to(target) > 0.01:
		look_at(target, Vector3.UP)
