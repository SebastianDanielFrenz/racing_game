extends Node3D
# game/scripts/body_visuals.gd — places the placeholder meshes of the sim's
# bodies (the chassis box; the flat world's static ground box) from
# RgSimulation.get_body_transform() every frame. Default process priority, so
# it runs AFTER camera_director.gd (-1000): the transforms it reads are
# already in this frame's rebased floating-origin frame. The real-world
# terrain is drawn by RgTerrainView, not here.

const CHASSIS_HALF_EXTENTS := Vector3(2.0, 0.4, 0.15) # SessionConfig's default (x fwd, y left, z up)
const GROUND_HALF_EXTENT_M := 1000.0 # SessionConfig::ground_half_extent_m's default

var simulation: Node
var show_ground: bool = false

var _chassis: MeshInstance3D
var _ground_anchor: Node3D

func _ready() -> void:
	# Ground: a box whose top face is the sim's static ground plane (the body
	# sits at the session origin, get_body_transform("ground")).
	_ground_anchor = Node3D.new()
	_ground_anchor.name = "GroundAnchor"
	add_child(_ground_anchor)
	var ground := MeshInstance3D.new()
	ground.name = "Ground"
	var ground_mesh := BoxMesh.new()
	ground_mesh.size = Vector3(GROUND_HALF_EXTENT_M * 2.0, 1.0, GROUND_HALF_EXTENT_M * 2.0)
	ground.mesh = ground_mesh
	ground.position = Vector3(0.0, -0.5, 0.0)
	var ground_mat := StandardMaterial3D.new()
	ground_mat.albedo_color = Color(0.25, 0.32, 0.22)
	ground.material_override = ground_mat
	_ground_anchor.add_child(ground)

	# Chassis: get_body_transform()'s basis columns are the sim body's own
	# local axes expressed in Godot space (frame_convert.h: column 0 = ISO x
	# forward, 1 = y left, 2 = z up), so the BoxMesh takes the ISO extents
	# unswapped. (R0's chase_cam-era main.gd swapped them, which drew the
	# plank standing on end.)
	_chassis = MeshInstance3D.new()
	_chassis.name = "Chassis"
	var chassis_box := BoxMesh.new()
	chassis_box.size = CHASSIS_HALF_EXTENTS * 2.0
	_chassis.mesh = chassis_box
	var chassis_mat := StandardMaterial3D.new()
	chassis_mat.albedo_color = Color(0.75, 0.1, 0.1)
	_chassis.material_override = chassis_mat
	add_child(_chassis)
	_chassis.visible = false
	_ground_anchor.visible = false

func _process(_delta: float) -> void:
	var live: bool = simulation != null and int(simulation.get_step_count()) > 0
	_chassis.visible = live
	_ground_anchor.visible = live and show_ground
	if not live:
		return
	_chassis.transform = simulation.get_body_transform("chassis")
	if show_ground:
		_ground_anchor.transform = simulation.get_body_transform("ground")
