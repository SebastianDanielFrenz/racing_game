extends Node3D
# Driver-eye socket and rigid chassis mounting mirror physics_sim's cockpit_cam.
# The camera director owns rebasing and streaming focus for this rig.
var look_pitch_deg: float = 0.0
var _config: Dictionary = {}
var body_visuals: Node
var camera: Camera3D
var _yaw: float = 0.0

func _ready() -> void:
	_config = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/cabin.json")))
	look_pitch_deg = float(_config["cockpit_pitch_default_deg"])
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
	look_pitch_deg = float(_config["cockpit_pitch_default_deg"])

func shift_origin(delta: Vector3) -> void:
	position += delta

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	if simulation == null or simulation.get_step_count() <= 0:
		return
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var eye := Vector3(-0.35, 0.38, 0.6)
	if body_visuals != null:
		eye = body_visuals.driver_eye_local()
	var look := Vector2.ZERO
	var mouse := Vector2.ZERO
	if input_live and input_map != null:
		look = input_map.get_camera_look_rate()
		mouse = input_map.consume_mouse_delta()
	var next: Dictionary = RgCameraMath.cockpit_look_step({"yaw_deg":rad_to_deg(_yaw),"pitch_deg":look_pitch_deg},{"look_x":look.x,"look_y":look.y,"mouse_x":mouse.x,"mouse_y":mouse.y,"live":input_live},delta,_config)
	_yaw = deg_to_rad(next["yaw_deg"])
	look_pitch_deg = next["pitch_deg"]
	var up: Vector3 = chassis.basis.z.normalized()
	var direction: Vector3 = chassis.basis.x.normalized().rotated(up,-_yaw)
	var side: Vector3 = chassis.basis.y.normalized().rotated(up,-_yaw)
	direction = direction.rotated(side,-deg_to_rad(look_pitch_deg))
	position = chassis*eye
	basis = Basis.looking_at(direction,up)
