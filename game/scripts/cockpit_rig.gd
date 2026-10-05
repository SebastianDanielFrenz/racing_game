extends Node3D
# Driver-eye socket and rigid chassis mounting mirror physics_sim's cockpit_cam.
# The camera director owns rebasing and streaming focus for this rig.
@export var look_deadzone: float = 0.2
@export var look_lag: float = 14.0
@export var look_pitch_deg: float = -3.0
var body_visuals: Node
var camera: Camera3D
var _yaw: float = 0.0

func _ready() -> void:
	camera = Camera3D.new()
	camera.name = "Camera"
	camera.near = 0.03
	camera.far = 43000.0
	camera.fov = 75.0
	add_child(camera)

# The cockpit view is a little wider than the chase view: the setting
# "camera.fov_deg" is the base, the offset keeps the original 75 at the default 70.
const FOV_OFFSET_DEG := 5.0

func set_base_fov(base_fov_deg: float) -> void:
	camera.fov = base_fov_deg + FOV_OFFSET_DEG

func activate(_from: Transform3D) -> void:
	_yaw = 0.0

func shift_origin(delta: Vector3) -> void:
	position += delta

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	if simulation == null or simulation.get_step_count() <= 0:
		return
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var eye := Vector3(-0.35, 0.38, 0.6)
	if body_visuals != null:
		eye = body_visuals.driver_eye_local()
	var target := 0.0
	if input_live and input_map != null:
		var keys: Vector2 = input_map.get_camera_key_look()
		var stick: Vector2 = input_map.get_camera_stick()
		if absf(keys.x) > 0.5:
			target = signf(keys.x)*PI*0.5
		elif keys.y < -0.5:
			target = PI
		elif stick.length() > look_deadzone:
			target = atan2(stick.x,stick.y)
	_yaw = target if look_lag <= 0.0 else lerp_angle(_yaw,target,1.0-exp(-look_lag*delta))
	var up: Vector3 = chassis.basis.z.normalized()
	var direction: Vector3 = chassis.basis.x.normalized().rotated(up,-_yaw)
	var side: Vector3 = chassis.basis.y.normalized().rotated(up,-_yaw)
	direction = direction.rotated(side,-deg_to_rad(look_pitch_deg))
	position = chassis*eye
	basis = Basis.looking_at(direction,up)
