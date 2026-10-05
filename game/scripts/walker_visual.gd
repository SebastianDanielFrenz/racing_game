extends Node3D
# game/scripts/walker_visual.gd — placeholder render of the on-foot player
# (R9c): a capsule the size of the simulated one (radius 0.3 m, height 1.75 m)
# with a small "nose" box showing the facing direction. Display only: the pose
# comes from RgSimulation.get_walker_state() (origin-relative Godot frame, feet
# position + facing), nothing is decided here. Hidden while no walker exists
# and in first person (the camera is inside the capsule). A real character
# model replaces this node later (data/models is not touched by this repo's
# walker work).

@export var radius_m: float = 0.3
@export var height_m: float = 1.75

var simulation: Node
var director: Node

var _body: MeshInstance3D

func _ready() -> void:
	name = "WalkerVisual"
	visible = false
	_body = MeshInstance3D.new()
	var capsule := CapsuleMesh.new()
	capsule.radius = radius_m
	capsule.height = height_m
	_body.mesh = capsule
	_body.position = Vector3(0.0, height_m * 0.5, 0.0)
	var material := StandardMaterial3D.new()
	material.albedo_color = Color(0.95, 0.55, 0.12)
	_body.material_override = material
	add_child(_body)
	var nose := MeshInstance3D.new()
	var box := BoxMesh.new()
	box.size = Vector3(0.16, 0.1, 0.22)
	nose.mesh = box
	nose.position = Vector3(0.0, height_m * 0.5 + 0.55, -radius_m)
	var nose_material := StandardMaterial3D.new()
	nose_material.albedo_color = Color(0.1, 0.1, 0.12)
	nose.material_override = nose_material
	add_child(nose)

func _process(_delta: float) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		visible = false
		return
	var ws: Dictionary = simulation.get_walker_state()
	if ws.is_empty():
		visible = false
		return
	var first_person := false
	if director != null:
		var rig: Node3D = director.active_rig()
		first_person = director.active_name == "walker" and rig != null and bool(rig.first_person)
	visible = not first_person
	position = ws["position"]
	var facing: Vector3 = ws["facing"]
	# The nose sits on -Z, so look_at-style yaw: -Z -> facing.
	rotation = Vector3(0.0, atan2(-facing.x, -facing.z), 0.0)
