extends XROrigin3D
# Tracked cockpit/free-flight rig; the director owns floating-origin rebasing.
var camera: XRCamera3D
var body_visuals: Node
var free_flight := false
var _recenter_down := false

func _ready() -> void:
	camera = XRCamera3D.new()
	camera.name = "XRCamera"
	camera.near = 0.03
	camera.far = 25000.0
	add_child(camera)
	var xr := XRServer.find_interface("OpenXR")
	if xr != null:
		xr.pose_recentered.connect(recenter)
		xr.session_begun.connect(_session_begun)

func _session_begun() -> void:
	await get_tree().create_timer(1.0).timeout
	if is_inside_tree():
		recenter()

func recenter() -> void:
	XRServer.center_on_hmd(XRServer.RESET_BUT_KEEP_TILT, false)

func activate(from: Transform3D) -> void:
	if free_flight:
		global_transform = from * camera.transform.affine_inverse()
	current = true

func shift_origin(delta: Vector3) -> void:
	position += delta

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	var down := Input.is_physical_key_pressed(KEY_F9)
	if down and not _recenter_down:
		recenter()
	_recenter_down = down
	if free_flight:
		if input_live:
			var movement: Vector3 = input_map.get_camera_move()
			var forward := -camera.global_basis.z
			forward.y = 0.0
			forward = forward.normalized()
			var right := forward.cross(Vector3.UP)
			var speed := 40.0 * (6.0 if input_map.get_camera_fast() else 1.0)
			position += (right * movement.x + forward * movement.z + Vector3.UP * movement.y).limit_length() * speed * delta
		return
	var chassis: Transform3D = simulation.get_body_transform("chassis")
	var eye := Vector3(-0.35, 0.38, 0.6)
	if body_visuals != null:
		eye = body_visuals.driver_eye_local()
	position = chassis * eye
	basis = Basis.looking_at(chassis.basis.x.normalized(), chassis.basis.z.normalized())
