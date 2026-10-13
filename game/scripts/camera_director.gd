extends Node
# game/scripts/camera_director.gd — owns the camera rigs (chase_rig.gd,
# free_rig.gd; later drone/seat/walker/XR rigs) and the floating-origin
# bookkeeping every rig shares. Process priority -1000 (PHYS-008): it runs
# before every node that places itself from a sim pose this frame, because
# the rebase below may move the shared origin.
#
# Per frame:
#  1. every rig ROOT follows any origin change since last frame (a world
#     switch builds a new Session with its own origin) - inactive rigs too,
#     so switching back never jumps;
#  2. the ACTIVE rig updates (only the active one reads input);
#  3. ONLY the active rig rebases (PHYS-008 "only the current camera
#     rebases"): RgSimulation.rebase_focus(active root position); the
#     resulting origin delta moves every rig ROOT (never a hard-coded
#     Camera3D - an XROrigin3D root works the same way);
#  4. the same frame, if the session-frame render origin changed,
#     RgTerrainView.set_render_origin(sim.get_render_origin_session());
#  5. render streaming follows the ACTIVE camera (rg/player_mode.h's rule):
#     RgTerrainView.update_focus(active camera position in session XY) -
#     VELOCITY-LED while the active rig follows the player car (chase, bumper,
#     cockpit, orbit): focus = p + v_hat * min(|v| * 1 s, 150 m), the chassis
#     velocity in the session frame (rg/render_focus.h). Free cam, drone and
#     on foot get no lead. road_stream.gd / building_stream.gd read the same
#     focus through render_focus_session() / render_velocity_session().
# Which rig is active is decided by the mode framework (rg_core,
# get_mode_state().camera_rig); main.gd calls set_active().

var simulation: Node
var input_map: Node
var terrain_view: Node # null unless a real-world view is loaded
var camera_input_live: bool = true
var rigs: Dictionary = {} # rig name ("chase", "free", ...) -> rig root
var active_name: String = ""
var rebase_count: int = 0

var _last_origin_godot: Vector3 = Vector3.ZERO
var _pushed_render_origin: Variant = null
var _focus_tracker: RgRenderFocus = RgRenderFocus.new()
var _focus_session: Vector2 = Vector2.ZERO # the (led) render focus, session XY
var _camera_session: Vector2 = Vector2.ZERO # the active camera, session XY
var _velocity_session: Vector2 = Vector2.ZERO # the chassis velocity, session XY (0 when the rig does not lead)
var _focus_valid: bool = false
var lead_enabled: bool = true # --no-render-lead switches the velocity lead off (A/B measurements)

func _ready() -> void:
	set_process_priority(-1000)

func add_rig(rig_name: String, rig: Node3D) -> void:
	rigs[rig_name] = rig

func active_rig() -> Node3D:
	return rigs.get(active_name, null)

func active_camera() -> Camera3D:
	var rig := active_rig()
	return rig.camera if rig != null else null

# The "camera.fov_deg" setting: every rig that has a base field of view takes it
# (the cinematic rig chooses its own per shot; XR rigs have none).
func set_base_fov(base_fov_deg: float) -> void:
	for r in rigs.values():
		if r.has_method("set_base_fov"):
			r.set_base_fov(base_fov_deg)

# The render focus the streamers use (session XY), led by the car's velocity
# when the active rig follows the car. Valid after the first _process frame.
func render_focus_session() -> Vector2:
	return _focus_session

func render_camera_session() -> Vector2:
	return _camera_session

# The chassis velocity (session XY) while the active rig leads, else zero.
func render_velocity_session() -> Vector2:
	return _velocity_session

func render_focus_lead_m() -> float:
	return _focus_tracker.get_lead_length_m()

func render_focus_valid() -> bool:
	return _focus_valid

func set_active(rig_name: String) -> void:
	if rig_name == active_name or not rigs.has(rig_name):
		return
	var from := Transform3D.IDENTITY
	var old := active_camera()
	if old != null:
		from = old.global_transform
	active_name = rig_name
	var rig: Node3D = rigs[rig_name]
	rig.activate(from)
	rig.camera.current = true

# A newly loaded terrain view gets the render origin pushed this frame.
func set_terrain_view(view: Node) -> void:
	terrain_view = view
	_pushed_render_origin = null

func _shift_all(delta: Vector3) -> void:
	if delta == Vector3.ZERO:
		return
	for r in rigs.values():
		r.shift_origin(delta)

func _process(delta: float) -> void:
	var t0 := Time.get_ticks_usec()
	var rig := active_rig()
	if simulation == null or rig == null:
		return

	var origin_now: Vector3 = simulation.get_world_origin_godot_position()
	_shift_all(origin_now - _last_origin_godot)

	rig.update_rig(delta, simulation, input_map, camera_input_live)

	var before: Vector3 = rig.global_position
	var after: Vector3 = simulation.rebase_focus(before)
	var origin_after: Vector3 = simulation.get_world_origin_godot_position()
	if after != before:
		rebase_count += 1
		_shift_all(origin_after - origin_now)
	_last_origin_godot = origin_after

	if terrain_view != null:
		var render_origin: Vector3 = simulation.get_render_origin_session()
		if _pushed_render_origin == null or render_origin != _pushed_render_origin:
			terrain_view.set_render_origin(render_origin)
			_pushed_render_origin = render_origin
		var focus: Vector3 = terrain_view.godot_to_session(rig.camera.global_position)
		var pose: Dictionary = simulation.get_chassis_session_pose()
		var leads: bool = lead_enabled and RgRenderFocus.rig_leads_focus(active_name) and not pose.is_empty()
		var vx: float = pose.get("vx", 0.0) if leads else 0.0
		var vy: float = pose.get("vy", 0.0) if leads else 0.0
		var f: Dictionary = _focus_tracker.update(focus.x, focus.y, vx, vy, delta, leads)
		_camera_session = Vector2(focus.x, focus.y)
		_focus_session = Vector2(f["x"], f["y"])
		_velocity_session = Vector2(vx, vy)
		_focus_valid = true
		if leads:
			terrain_view.update_focus_moving(f["x"], f["y"], vx, vy, focus.x, focus.y)
		else:
			terrain_view.update_focus(f["x"], f["y"])

	simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)
