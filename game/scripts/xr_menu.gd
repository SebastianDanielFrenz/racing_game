extends Node3D
# Existing shell/garage canvases on a fixed, headset-visible interaction panel.
const PIXELS := Vector2i(1920,1080)
const METRES := Vector2(3.2,1.8)
var main: Node
var viewport: SubViewport
var origin: XROrigin3D
var camera: XRCamera3D
var panel: MeshInstance3D
var cursor: MeshInstance3D
var hands: Array[XRController3D] = []
var active := false
var pointer := Vector2(-1,-1)
var held := false
var hand_index := 1
var scroll_delay := 0.0
var alignment_pending := false
var tracking_frames := 0

func _ready() -> void:
	process_priority = 1000
	viewport = SubViewport.new()
	viewport.name = "MenuViewport"
	viewport.size = PIXELS
	viewport.disable_3d = true
	viewport.handle_input_locally = true
	viewport.gui_embed_subwindows = true
	viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(viewport)
	origin = XROrigin3D.new()
	add_child(origin)
	camera = XRCamera3D.new()
	camera.near = 0.03
	origin.add_child(camera)
	for tracker in ["left_hand","right_hand"]:
		var hand := XRController3D.new()
		hand.tracker = tracker
		hand.pose = "aim"
		origin.add_child(hand)
		hands.append(hand)
		hand.button_pressed.connect(func(action: String): _button(action,true,hands.find(hand)))
		hand.button_released.connect(func(action: String): _button(action,false,hands.find(hand)))
		hand.input_float_changed.connect(func(action: String,value: float):
			if action == "trigger":
				if value>0.65 and not held: _button("trigger_click",true,hands.find(hand))
				elif value<0.35 and held and hand_index==hands.find(hand): _button("trigger_click",false,hand_index))
	panel = MeshInstance3D.new()
	var quad := QuadMesh.new()
	quad.size = METRES
	panel.mesh = quad
	var material := StandardMaterial3D.new()
	material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	material.albedo_texture = viewport.get_texture()
	material.cull_mode = BaseMaterial3D.CULL_DISABLED
	material.no_depth_test = true
	material.render_priority = 126
	panel.material_override = material
	add_child(panel)
	cursor = MeshInstance3D.new()
	var dot := SphereMesh.new()
	dot.radius = 0.009
	dot.height = 0.018
	cursor.mesh = dot
	var dot_material := StandardMaterial3D.new()
	dot_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	dot_material.albedo_color = Color(1,0.65,0.15)
	dot_material.no_depth_test = true
	dot_material.render_priority = 127
	cursor.material_override = dot_material
	add_child(cursor)
	panel.hide()
	cursor.hide()
	camera.current = false
	var xr := XRServer.find_interface("OpenXR")
	if xr != null:
		xr.pose_recentered.connect(_request_alignment)
		xr.session_begun.connect(_request_alignment)

func _request_alignment() -> void:
	if not active: return
	alignment_pending = true
	tracking_frames = 0
	_place_panel()
	panel.show()

func _place_panel() -> void:
	var forward := -camera.global_basis.z
	forward.y = 0
	if forward.length_squared() < 0.01: forward = Vector3.FORWARD
	forward = forward.normalized()
	panel.global_position = camera.global_position + forward*2.6
	panel.global_basis = Basis.looking_at(forward,Vector3.UP)
	print("RG_VR_MENU aligned eye=%s panel=%s origin_current=%s" % [camera.global_position,panel.global_position,origin.current])

func attach(layer: CanvasLayer) -> void:
	if layer.get_parent() != viewport:
		layer.reparent(viewport)

func show_screen(screen: String) -> void:
	var next := screen != "drive"
	if next and not active:
		var old := get_viewport().get_camera_3d()
		if old != null and old.get_parent() is XROrigin3D:
			origin.global_transform = old.get_parent().global_transform
		origin.current = true
		var xr := XRServer.find_interface("OpenXR")
		alignment_pending = xr != null and xr.is_initialized()
		tracking_frames = 0
		_place_panel()
	if not next and held: _click(false)
	active = next
	panel.visible = active
	viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS if active else SubViewport.UPDATE_DISABLED
	if active:
		origin.current = true
		camera.current = true
	else:
		cursor.hide()
		var world_camera: Camera3D = main._director.active_camera()
		if world_camera != null:
			if world_camera.get_parent() is XROrigin3D: world_camera.get_parent().current = true
			world_camera.current = true

func ray_pixel(from: Vector3, direction: Vector3) -> Vector2:
	var local_from := panel.global_transform.affine_inverse()*from
	var local_direction := panel.global_basis.inverse()*direction
	if local_direction.z >= -0.00001: return Vector2(-1,-1)
	var distance := -local_from.z/local_direction.z
	if distance < 0: return Vector2(-1,-1)
	var hit := local_from+local_direction*distance
	var uv := Vector2(hit.x/METRES.x+0.5,0.5-hit.y/METRES.y)
	if uv.x<0 or uv.x>1 or uv.y<0 or uv.y>1: return Vector2(-1,-1)
	return uv*Vector2(PIXELS)

func _process(delta: float) -> void:
	if not active: return
	origin.current = true
	camera.current = true
	if alignment_pending:
		# XRCamera3D is a Camera3D, not an XRNode3D: it has no
		# get_is_active(). Never hide the menu behind a tracking predicate.
		tracking_frames += 1
		if tracking_frames>=3:
			_place_panel()
			alignment_pending = false
	var hand := hands[hand_index]
	var source: Node3D = hand if hand.get_is_active() else camera
	var next := ray_pixel(source.global_position,-source.global_basis.z)
	cursor.visible = next.x>=0
	if next.x>=0:
		cursor.global_position = panel.global_transform*Vector3((next.x/PIXELS.x-0.5)*METRES.x,(0.5-next.y/PIXELS.y)*METRES.y,0.012)
		var motion := InputEventMouseMotion.new()
		motion.position = next
		motion.global_position = next
		motion.relative = next-pointer if pointer.x>=0 else Vector2.ZERO
		motion.button_mask = MOUSE_BUTTON_MASK_LEFT if held else 0
		viewport.push_input(motion,true)
	pointer = next
	scroll_delay -= delta
	var stick := hand.get_vector2("primary") if hand.get_is_active() else Vector2.ZERO
	if absf(stick.y)>0.65 and scroll_delay<=0 and pointer.x>=0:
		var wheel := InputEventMouseButton.new()
		wheel.position = pointer
		wheel.button_index = MOUSE_BUTTON_WHEEL_UP if stick.y>0 else MOUSE_BUTTON_WHEEL_DOWN
		wheel.pressed = true
		viewport.push_input(wheel,true)
		scroll_delay = 0.12

func _click(pressed: bool) -> void:
	if pressed and pointer.x<0: return
	held = pressed
	var event := InputEventMouseButton.new()
	event.position = pointer if pointer.x>=0 else Vector2(-1,-1)
	event.global_position = event.position
	event.button_index = MOUSE_BUTTON_LEFT
	event.pressed = pressed
	viewport.push_input(event,true)

func _button(action: String, pressed: bool, index: int) -> void:
	if not active:
		if pressed and action == "menu_button": main._on_escape()
		return
	if action == "trigger_click":
		if held == pressed or (not pressed and index != hand_index): return
		hand_index = index
		var hand := hands[index]
		pointer = ray_pixel(hand.global_position,-hand.global_basis.z)
		_click(pressed)
	elif pressed and action in ["by_button","menu_button"]:
		main._on_escape()
	elif action in ["ax_button","select_button"]:
		var event := InputEventAction.new()
		event.action = "ui_accept"
		event.pressed = pressed
		viewport.push_input(event,true)

func _input(event: InputEvent) -> void:
	if not active or event is InputEventMouse: return
	if event is InputEventKey and event.pressed and event.keycode == KEY_F9:
		XRServer.center_on_hmd(XRServer.RESET_BUT_KEEP_TILT,false)
		_request_alignment()
		return
	# Back stays with main's existing ESC/B flow; avoid a duplicate transition.
	if event.is_action("ui_cancel"): return
	if event is InputEventKey or event is InputEventJoypadButton or event is InputEventJoypadMotion:
		viewport.push_input(event,true)
