extends Node3D
# game/scripts/body_visuals.gd â€” places the placeholder/real meshes of the
# sim's bodies (the chassis; the flat world's static ground box) from
# RgSimulation.get_body_transform() every frame. Default process priority, so
# it runs AFTER camera_director.gd (-1000): the transforms it reads are
# already in this frame's rebased floating-origin frame. The real-world
# terrain is drawn by RgTerrainView, not here.
#
# Chassis visual (carvis brief, 2026-09-26): _chassis_root is the "floating-
# origin root" for the car - the one node whose transform is set every frame
# from get_body_transform("chassis") (already origin-rebase-corrected, see
# RgSimulation::get_body_transform). Two children hang off it and never move
# independently: _chassis_box (the red placeholder, MeshInstance3D) and
# VehicleVisual (vehicle_visual.gd, the real car_sedan.glb) - a rebase moves
# both together because they are only ever positioned relative to
# _chassis_root. The box is hidden once the real model reports loaded
# (VehicleVisual.model_loaded()); it stays visible (fallback) if the model
# failed to load, and vehicle_model_ok() below lets hud.gd surface that as a
# HUD warning alongside vehicle_visual.gd's own console push_warning.

const CHASSIS_HALF_EXTENTS := Vector3(2.0, 0.4, 0.15) # SessionConfig's default (x fwd, y left, z up)
const GROUND_HALF_EXTENT_M := 1000.0 # SessionConfig::ground_half_extent_m's default

var simulation: Node
var show_ground: bool = false

# Set by main.gd before add_child (mirrors physics_sim's own main.gd wiring
# vehicle_visual.gd's model_absolute_path/vehicle_name) - vehicle_name is
# ALSO the model directory/id under external/physics_sim/data/models/ for
# every shipped model (index.json: "car_sedan/car_sedan.glb", etc.), so a
# future car_hyper reuse only ever changes this one string.
var vehicle_name: String = ""
var model_absolute_path: String = ""
var paint_colour: Color = Color(0, 0, 0, 0) # alpha 0: keep the model's own colours
var rim_colour: Color = Color(0, 0, 0, 0)

var _chassis_root: Node3D
var _chassis_box: MeshInstance3D
var _vehicle_visual: Node3D
var _ground_anchor: Node3D

func _ready() -> void:
	# Ground: a box whose top face is the sim's static ground plane (the body
	# sits at the session origin, get_body_transform("ground")).
	_ground_anchor = Node3D.new()
	_ground_anchor.name = "GroundAnchor"
	add_child(_ground_anchor)
	var ground := MeshInstance3D.new()
	ground.name = "Ground"
	# Size/position are authored in ISO order (x fwd, y left, z up) - like
	# CHASSIS_HALF_EXTENTS/chassis_box below, NOT native Godot (x right, y up,
	# z back) - because _ground_anchor.transform (see _process()) is
	# get_body_transform("ground")'s ISO->Godot basis, the same convention the
	# chassis box relies on. Bug found 2026-09-27 (owner report: "a green
	# plane sideways through the middle of the car"): this mesh was authored
	# Y-up (native Godot), so that basis rotated the flat plate onto its edge
	# (thin-in-Godot-X, ~2000 units tall) - invisible from most angles, a
	# looming vertical wall from others. Z is the thin/up axis here, matching
	# the sim's own ground box (session.cpp: BoxShape half-extents
	# {ground_half_extent_m, ground_half_extent_m, 0.5}, pose.position
	# {0, 0, -0.5} - top face at local z=0).
	var ground_mesh := BoxMesh.new()
	ground_mesh.size = Vector3(GROUND_HALF_EXTENT_M * 2.0, GROUND_HALF_EXTENT_M * 2.0, 1.0)
	ground.mesh = ground_mesh
	ground.position = Vector3(0.0, 0.0, -0.5)
	var ground_mat := StandardMaterial3D.new()
	ground_mat.albedo_color = Color(0.25, 0.32, 0.22)
	ground.material_override = ground_mat
	_ground_anchor.add_child(ground)

	# Chassis root: the floating-origin-corrected transform carrier (see file
	# header comment). get_body_transform()'s basis columns are the sim
	# body's own local axes expressed in Godot space (frame_convert.h: column
	# 0 = ISO x forward, 1 = y left, 2 = z up), so the BoxMesh takes the ISO
	# extents unswapped. (R0's chase_cam-era main.gd swapped them, which drew
	# the plank standing on end.)
	_chassis_root = Node3D.new()
	_chassis_root.name = "ChassisRoot"
	add_child(_chassis_root)

	_chassis_box = MeshInstance3D.new()
	_chassis_box.name = "ChassisBox"
	var chassis_box := BoxMesh.new()
	chassis_box.size = CHASSIS_HALF_EXTENTS * 2.0
	_chassis_box.mesh = chassis_box
	var chassis_mat := StandardMaterial3D.new()
	chassis_mat.albedo_color = Color(0.75, 0.1, 0.1)
	_chassis_box.material_override = chassis_mat
	_chassis_root.add_child(_chassis_box)

	if model_absolute_path != "":
		_make_vehicle_visual()

	_chassis_root.visible = false
	_ground_anchor.visible = false

func _make_vehicle_visual() -> void:
	_vehicle_visual = Node3D.new()
	_vehicle_visual.name = "VehicleVisual"
	_vehicle_visual.set_script(load("res://scripts/vehicle_visual.gd"))
	_vehicle_visual.simulation = simulation
	_vehicle_visual.vehicle_name = vehicle_name
	_vehicle_visual.model_absolute_path = model_absolute_path
	_vehicle_visual.paint_colour = paint_colour
	_vehicle_visual.rim_colour = rim_colour
	_chassis_root.add_child(_vehicle_visual)

# R6: the garage's car for the next world (a different model, colours from its setup).
# The same car again only re-binds; a different one replaces the visual.
func set_vehicle(name_in: String, model_path: String, paint_hex: String, rim_hex: String) -> void:
	var same := _vehicle_visual != null and model_path == model_absolute_path
	vehicle_name = name_in
	model_absolute_path = model_path
	paint_colour = Color.html(paint_hex)
	rim_colour = Color.html(rim_hex)
	if same:
		_vehicle_visual.vehicle_name = name_in
		_vehicle_visual.paint_colour = paint_colour
		_vehicle_visual.rim_colour = rim_colour
		_vehicle_visual.apply_colours()
		return
	if _vehicle_visual != null:
		_chassis_root.remove_child(_vehicle_visual)
		_vehicle_visual.queue_free()
		_vehicle_visual = null
	_make_vehicle_visual()

# Called by main.gd right after a Session starts running (flat: immediately
# after start(); real world: _attach_world_view()'s first tick) - a world
# switch (R7/R9) rebuilds rg::Session from under the same RgSimulation node,
# so the wheel count/attachment points VehicleVisual cached at its own
# _ready() may now belong to a destroyed Session. Cheap (no .glb reload).
func on_session_ready() -> void:
	if _vehicle_visual != null:
		_vehicle_visual.rebuild()

# hud.gd's HUD warning (console already gets vehicle_visual.gd's own
# push_warning either way): true once the real car_sedan.glb art is up,
# false while still showing the red placeholder fallback.
func vehicle_model_ok() -> bool:
	return _vehicle_visual != null and _vehicle_visual.model_loaded()

# carvis steering-proof fix (2026-09-27): see vehicle_visual.gd's own doc
# comment - forwarded so drive_tour.gd does not need a direct reference to
# the VehicleVisual child.
func get_wheel_visual_steer_angle_rad(wheel_index: int) -> float:
	return _vehicle_visual.get_wheel_visual_steer_angle_rad(wheel_index) if _vehicle_visual != null else 0.0

func _process(_delta: float) -> void:
	var live: bool = simulation != null and int(simulation.get_step_count()) > 0
	_chassis_root.visible = live
	_ground_anchor.visible = live and show_ground
	if not live:
		return
	_chassis_root.transform = simulation.get_body_transform("chassis")
	_chassis_box.visible = not vehicle_model_ok() # fallback: only shown while the real model isn't up
	if show_ground:
		_ground_anchor.transform = simulation.get_body_transform("ground")

# ISO chassis axes: X forward, Y left, Z up. Shared by desktop and XR rigs.
var seat_offset := Vector3(0.20, 0.0, 0.0)

func driver_eye_local() -> Vector3:
	var base: Vector3 = _vehicle_visual.driver_eye_local() if _vehicle_visual != null else Vector3(-0.35, 0.38, 0.6)
	return base + seat_offset
