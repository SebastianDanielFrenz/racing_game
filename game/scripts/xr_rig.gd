extends XROrigin3D
# Tracked cockpit/free-flight rig; the director owns floating-origin rebasing.
var camera: XRCamera3D
var body_visuals: Node
var free_flight := false
# Drone follow in VR ("xr_drone"): the origin trails the drone target with the
# same rg::drone_* maths as drone_rig.gd (lag on the target-relative offset,
# zoom, snap, terrain clearance), but the world is never rotated for the
# player - no orbit input, no look-at: the origin yaw is set ONCE per
# activation to face the target and the headset turns freely after that.
var drone_follow := false
var _drone_state: Dictionary = {}
var _drone_face_pending := false
const DRONE_PARAMS := {"orbit_lag": 0.0} # orbit unused in VR (live=false below)
var _recenter_down := false
# The driver's virtual body (camera_director.gd, RgCameraMotion view
# "xr_cockpit"): head TRANSLATION only - VR never gets a rotation, shake or FOV
# change it did not make (rg_core forces those gains to zero).
var motion: Dictionary = {}
var motion_view := "xr_cockpit"
var motion_enabled := true

func _ready() -> void:
	camera = XRCamera3D.new()
	camera.name = "XRCamera"
	camera.near = 0.03
	camera.far = 43000.0
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
	if drone_follow:
		_drone_state = RgCameraMath.drone_initial_state(DRONE_PARAMS)
		_drone_state = RgCameraMath.drone_activate(_drone_state, from.origin)
		_drone_face_pending = true
	current = true
	camera.current = true

func shift_origin(delta: Vector3) -> void:
	position += delta
	if drone_follow and not _drone_state.is_empty():
		_drone_state = RgCameraMath.drone_shift_origin(_drone_state, delta)

# Origin placement so the HEAD (camera) sits at head_pos; the yaw is only
# touched when `face` is given (the horizontal direction the headset should
# look toward at that moment).
func _place_head(head_pos: Vector3, face: Variant) -> void:
	if face != null:
		var d: Vector3 = face
		var want_yaw := atan2(-d.x, -d.z)
		var f := -camera.basis.z
		var head_yaw := atan2(-f.x, -f.z)
		basis = Basis(Vector3.UP, want_yaw - head_yaw)
	position += head_pos - camera.global_position

func _update_drone(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	if int(simulation.get_step_count()) <= 0:
		_drone_state = RgCameraMath.drone_hold(_drone_state)
		return
	if input_map != null: # zoom only; the wheel is always drained
		_drone_state = RgCameraMath.drone_zoom_step(_drone_state, float(input_map.consume_zoom_steps()),
			float(input_map.get_camera_zoom_key()), input_live, delta, DRONE_PARAMS)
	var target: Variant = simulation.get_drone_target_transform()
	if target == null:
		return
	var xform: Transform3D = target
	var frame: Dictionary = RgCameraMath.drone_follow_step(_drone_state, {
		"target_position": xform.origin, "target_forward": xform.basis.x,
		"key_look": Vector2.ZERO, "stick": Vector2.ZERO, "live": false}, delta, DRONE_PARAMS)
	_drone_state = frame
	var pos: Vector3 = frame["position"]
	var grounds: Array = []
	for probe in RgCameraMath.drone_probe_points(pos):
		grounds.append(simulation.get_camera_ground_height(probe))
	pos = RgCameraMath.drone_apply_clearance(pos, grounds, DRONE_PARAMS)
	var face: Variant = null
	if _drone_face_pending:
		_drone_face_pending = false
		var look: Vector3 = frame["look_at"]
		var dir := Vector3(look.x - pos.x, 0.0, look.z - pos.z)
		if dir.length() > 0.01:
			face = dir.normalized()
	_place_head(pos, face)

func update_rig(delta: float, simulation: Node, input_map: Node, input_live: bool) -> void:
	var down := Input.is_physical_key_pressed(KEY_F9)
	if down and not _recenter_down:
		recenter()
	_recenter_down = down
	if drone_follow:
		_update_drone(delta, simulation, input_map, input_live)
		return
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
	if motion_enabled and not motion.is_empty():
		eye += motion["offset"]
	position = chassis * eye
	basis = Basis.looking_at(chassis.basis.x.normalized(), chassis.basis.z.normalized())
