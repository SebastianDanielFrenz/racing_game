extends Node3D
# game/scripts/fpv_rig.gd - the FpvDrone mode camera rig (rg::CameraRig::Fpv). Same rig contract as free_rig.gd: a ROOT
# Node3D plus a Camera3D child; camera_director.gd moves the ROOT for floating-origin rebases (shift_origin) and calls
# update_rig() only while this rig is active.
#
# Display only: the flight (velocity, limits, ground clamp, yaw/pitch) is rg::FpvDrone in rg_core; this rig places the
# camera at RgSimulation.get_fpv_state() and aims it with the drone's own yaw and pitch. No roll, ever.

@export var fov_deg: float = 100.0 # an FPV camera is wide

var camera: Camera3D

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.fov = fov_deg
	camera.near = 0.1
	camera.far = 43000.0
	add_child(camera)

# The drone appears above the car; until its first state arrives the rig stays where the previous camera was.
func activate(from: Transform3D) -> void:
	position = from.origin

func shift_origin(delta: Vector3) -> void:
	position += delta

func update_rig(_delta: float, simulation: Node, _input_map: Node, _camera_input_live: bool) -> void:
	if simulation == null or int(simulation.get_step_count()) <= 0:
		return
	var state: Dictionary = simulation.get_fpv_state()
	if state.is_empty():
		return
	position = state["position"]
	# Godot: yaw about Y (ISO yaw 0 = east = -Z... the binding's facing vector is (-sin yaw, 0, -cos yaw), which is a
	# rotation of -Z by yaw about Y), pitch about X, nothing else.
	rotation = Vector3(0.0, float(state["yaw_rad"]), 0.0)
	camera.rotation = Vector3(float(state["pitch_rad"]), 0.0, 0.0)
