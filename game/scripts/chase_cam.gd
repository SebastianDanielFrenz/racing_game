extends Camera3D
# game/scripts/chase_cam.gd — reused near-verbatim from physics_sim's own
# adapters/godot/demo/scripts/chase_cam.gd (read-only reference, not a
# submodule file): it depends only on four RgSimulation methods this repo's
# godot_ext/src/rg_simulation.h/.cpp implements with the SAME names/
# signatures as physics_sim's own PsSimulation (get_world_origin_godot_
# position, get_body_transform, rebase_focus, add_adapter_time_us), so no
# script-level change was needed beyond this header comment. See that
# method-parity choice in this repo's CLAUDE.md.
#
# Follows the vehicle chassis's OWN Transform3D (already race-free/origin-
# relative via RgSimulation.get_body_transform), smoothed with an
# exponential lag so small physics-tick jitter doesn't show, then rebases
# itself through RgSimulation.rebase_focus every frame - this IS the
# origin-rebasing loop PLAN.md 11.3 asks for ("only the active camera
# rebases"): once this camera's own Godot position drifts past
# OriginRebase's threshold (500 m, ps_godot::OriginRebase's own default),
# the origin snaps to it and rebase_focus's return value is `position`
# re-expressed in the new frame. `_smoothed_pos` must be shifted by that
# same delta right after the rebase_focus call, or it silently keeps living
# in the old frame - physics_sim's own file comment documents the exact bug
# this caused there (a visible one-frame "car bugs back and bounces back"
# glitch) and the fix, both preserved verbatim below since the same failure
# mode applies here unchanged.

@export var simulation_path: NodePath
@export var chassis_body_name: String = ""
@export var follow_distance_m: float = 7.0
@export var follow_height_m: float = 2.5
@export var look_height_m: float = 1.0
@export var position_lag: float = 6.0 # higher = snappier
@export var rotation_lag: float = 4.0
# Right-stick look-around: the stick's POSITION sets the orbit angle (not a
# rate). The camera looks the way the stick points - up = normal chase view,
# right = look right, down = look back - scaled by deflection past the
# deadzone, so releasing the stick returns to the chase view.
@export var orbit_deadzone: float = 0.2
@export var orbit_lag: float = 14.0 # higher = snappier; <= 0 = no smoothing

var _simulation: Node
var _smoothed_pos: Vector3 = Vector3.ZERO
var _smoothed_basis: Basis = Basis.IDENTITY
var _initialized: bool = false
var _orbit_yaw: float = 0.0 # radians, + = looking right of the car's heading
# Godot-space position of the sim's absolute origin as of this camera's
# previous frame.
var _last_origin_godot: Vector3 = Vector3.ZERO

func _target_orbit_yaw() -> float:
	var pads := Input.get_connected_joypads()
	if pads.is_empty():
		return 0.0
	# Godot's stick Y is +down, so "up" is -Y.
	var stick := Vector2(Input.get_joy_axis(pads[0], JOY_AXIS_RIGHT_X),
			-Input.get_joy_axis(pads[0], JOY_AXIS_RIGHT_Y))
	var mag: float = min(stick.length(), 1.0)
	if mag <= orbit_deadzone:
		return 0.0
	var weight: float = smoothstep(orbit_deadzone, 1.0, mag)
	return atan2(stick.x, stick.y) * weight

func _ready() -> void:
	# Run BEFORE every other node that reads a pose this frame - see file
	# comment: rebase_focus below may move the shared floating origin, and
	# anything that already placed itself this frame with the OLD origin
	# would then be drawn up to the rebase threshold away from this camera
	# for exactly one frame.
	set_process_priority(-1000)
	if simulation_path != NodePath():
		_simulation = get_node_or_null(simulation_path)

func _process(delta: float) -> void:
	var t0 := Time.get_ticks_usec()
	if _simulation == null or chassis_body_name == "":
		return

	var origin_now: Vector3 = _simulation.get_world_origin_godot_position()
	if _initialized:
		_smoothed_pos += origin_now - _last_origin_godot

	var chassis_xform: Transform3D = _simulation.get_body_transform(chassis_body_name)
	var chassis_pos: Vector3 = chassis_xform.origin
	var chassis_basis: Basis = chassis_xform.basis

	# chassis_basis.x is the car's FORWARD direction in Godot space (see
	# godot_ext/src/frame_convert.h's iso_to_godot_basis: column i is "the
	# sim's own local axis i, expressed in Godot world space", ISO 8855
	# x-forward) - "behind" is its negation.
	var behind := -chassis_basis.x.normalized() * follow_distance_m
	var desired_pos := chassis_pos + behind + Vector3.UP * follow_height_m
	var desired_look_at := chassis_pos + Vector3.UP * look_height_m

	if not _initialized:
		_smoothed_pos = desired_pos
		_initialized = true

	var pos_alpha: float = 1.0 - exp(-position_lag * delta)
	_smoothed_pos = _smoothed_pos.lerp(desired_pos, pos_alpha)

	var target_yaw := _target_orbit_yaw()
	if orbit_lag <= 0.0:
		_orbit_yaw = target_yaw
	else:
		_orbit_yaw = lerp_angle(_orbit_yaw, target_yaw, 1.0 - exp(-orbit_lag * delta))
	var offset := _smoothed_pos - chassis_pos
	position = chassis_pos + offset.rotated(Vector3.UP, -_orbit_yaw)
	look_at(desired_look_at, Vector3.UP)

	# Origin rebasing - only the CURRENT camera drives the floating origin.
	if current:
		var rebased_pos: Vector3 = _simulation.rebase_focus(position)
		var rebase_delta: Vector3 = rebased_pos - position
		position = rebased_pos
		_smoothed_pos += rebase_delta
	_last_origin_godot = _simulation.get_world_origin_godot_position()

	var elapsed_us := Time.get_ticks_usec() - t0
	_simulation.add_adapter_time_us(elapsed_us)
