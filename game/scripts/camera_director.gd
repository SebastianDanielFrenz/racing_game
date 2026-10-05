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
#     RgTerrainView.update_focus(active camera position in session XY).
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
		terrain_view.update_focus(focus.x, focus.y)

	simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)
