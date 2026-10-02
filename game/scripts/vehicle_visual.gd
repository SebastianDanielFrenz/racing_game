extends Node3D
# game/scripts/vehicle_visual.gd â€” near-copy port of physics_sim's
# external/physics_sim/adapters/godot/demo/scripts/vehicle_visual.gd (read-
# only reference): shows the real car_sedan.glb art instead of the red
# placeholder box, wheels following suspension travel/spin/steer from the
# REAL simulated per-tick state. Ported for the "borrow physics_sim's car
# visual" brief (2026-09-26).
#
# Differences from the reference, both narrowing rather than widening scope:
#  - `simulation` is a plain Node reference set directly by body_visuals.gd
#    before add_child (the pattern this repo already uses for body_visuals.gd
#    itself, tach_gauge.gd, hud.gd's *_path exports aside) rather than a
#    NodePath + get_node_or_null - this node is created procedurally by its
#    direct owner, never placed in a hand-authored scene.
#  - No client-side spin integration: rg::vehicle::WheelState (ps/vehicle/
#    wheel_state.h) already carries an integrated spin_angle - RgSimulation.
#    get_wheel_spin_angle reads it straight from the race-free FrameSnapshot
#    (rg::Session::capture_frame_snapshot, sim-thread-only, published through
#    a triple buffer - see RgSimulation.get_wheel_compression/_spin_angle/
#    _steer_angle's own doc comment in rg_simulation.h). physics_sim's own
#    core did not have this field when its script was written, hence its
#    get_wheel_omega + client-side integration; racing_game's Session already
#    forwards the richer WheelState, so there is nothing to integrate here.
#  - rebuild(): re-runs _bind_wheel_nodes()/_align_model_to_physics_wheels()
#    without reloading the .glb - called by body_visuals.gd whenever a world
#    switch (R7/R9, flat <-> real world) rebuilds the underlying rg::Session,
#    since a fresh Session means a freshly re-parsed VehicleDesc (attachment
#    points, wheel count) even though today it is always the same
#    data/vehicles/car_sedan.json on both sides.
#
# MODEL-SPACE FIX AND EVERYTHING BELOW is otherwise the reference's own,
# unchanged: the glb is already ISO-consistent (gltf(x,y,z) = iso(y,z,x) per
# data/models/README.md and car_sedan.rig.json's own "frame" field), and
# there is NO -90 deg X rotation - see the reference file's header comment
# for the full derivation (verified directly against car_sedan.glb's bytes).
# Node names below match data/models/car_sedan/car_sedan.rig.json exactly
# ("susp_<corner>"/"steer_<corner>"/"wheel_<corner>", corner in
# {FL,FR,RL,RR}); rear wheels have no steer_<corner> node (not steered),
# handled by the steered guard in _process below.

var simulation: Node
var vehicle_name: String = ""
var model_absolute_path: String = "" # set by body_visuals.gd before add_child

var _wheel_nodes: Array = []
var _model_root: Node3D
var _load_ok: bool = false

func _ready() -> void:
	_load_model()
	_bind_wheel_nodes()
	set_process(true)

func model_loaded() -> bool:
	return _load_ok

# carvis steering-proof fix (2026-09-27): the steer_<corner> node's own local
# Y rotation, exactly as _process below last set it (or 0.0 if that wheel has
# no steer node/is not steered) - read back by drive_tour.gd for poses.txt so
# a sign/axis mismatch between the physics steer angle
# (RgSimulation.get_wheel_steer_angle) and what actually got applied to the
# visual shows up as two numbers side by side, not just a screenshot.
func get_wheel_visual_steer_angle_rad(wheel_index: int) -> float:
	if wheel_index < 0 or wheel_index >= _wheel_nodes.size():
		return 0.0
	var w = _wheel_nodes[wheel_index]
	if w["steer"] == null or not w["steered"]:
		return 0.0
	return w["steer"].rotation.y

# Re-binds wheel nodes and recomputes the model-to-physics alignment against
# whatever Session `simulation` currently holds - cheap (no .glb reload), see
# file header comment. Safe to call with no vehicle yet (wheel_count 0).
func rebuild() -> void:
	_wheel_nodes.clear()
	_bind_wheel_nodes()

func _load_model() -> void:
	if model_absolute_path == "" or not FileAccess.file_exists(model_absolute_path):
		push_warning("vehicle_visual.gd: model not found at '%s'" % model_absolute_path)
		return
	var gltf_doc := GLTFDocument.new()
	var gltf_state := GLTFState.new()
	var err: int = gltf_doc.append_from_file(model_absolute_path, gltf_state)
	if err != OK:
		push_warning("vehicle_visual.gd: GLTFDocument.append_from_file failed (err=%d) for %s" % [err, model_absolute_path])
		return
	_model_root = gltf_doc.generate_scene(gltf_state)
	if _model_root == null:
		push_warning("vehicle_visual.gd: GLTFDocument.generate_scene returned null for %s" % model_absolute_path)
		return
	add_child(_model_root)
	# gltf-local -> ISO-local permutation (see file header comment):
	# iso.x (fwd) = gltf.z, iso.y (left) = gltf.x, iso.z (up) = gltf.y.
	# As a basis, column i is where gltf's local axis i (x,y,z) ends up:
	# gltf+X (left) -> (0,1,0), gltf+Y (up) -> (0,0,1), gltf+Z (fwd) -> (1,0,0).
	# A pure axis permutation (determinant +1: an even permutation of the
	# three axes), so this never mirrors the mesh.
	_model_root.transform.basis = Basis(Vector3(0, 1, 0), Vector3(0, 0, 1), Vector3(1, 0, 0))
	_load_ok = true
	print("rg_godot vehicle_visual.gd: loaded model ", model_absolute_path)

func _bind_wheel_nodes() -> void:
	if simulation == null or _model_root == null or vehicle_name == "":
		return
	var wheel_count: int = simulation.get_vehicle_wheel_count(vehicle_name)
	if wheel_count == 0:
		# Bug found 2026-09-27 (poses.txt's wheel_steer_visual_rad stuck at
		# 0.0 for every wheel, every frame): the flat world's _load_world()
		# calls on_session_ready() (-> rebuild() -> here) synchronously right
		# after Session::start(), same frame - but the sim itself runs on its
		# own thread (PLAN.md D3) and does not create its vehicle until its
		# FIRST tick, so get_vehicle_wheel_count() here still reads 0 at that
		# exact instant (has_vehicle() false) and _wheel_nodes silently stays
		# empty forever - no warning fires (this path returns before the
		# missing-node check below, which is for a DIFFERENT problem: a node
		# absent from the .glb, not zero wheels at all). _process() below
		# retries this same call every frame while _wheel_nodes is still
		# empty, so the real world's own already-correct step_count-gated
		# on_session_ready() call (main.gd's _attach_world_view) keeps working
		# unchanged and the flat world self-heals the frame the vehicle
		# actually exists (get_step_count() > 0) instead of staying empty.
		return
	var missing := 0
	for i in range(wheel_count):
		var wname: String = simulation.get_wheel_name(vehicle_name, i)
		var susp: Node3D = _model_root.find_child("susp_" + wname, true, false)
		var steer: Node3D = _model_root.find_child("steer_" + wname, true, false)
		var wheel: Node3D = _model_root.find_child("wheel_" + wname, true, false)
		var entry := {
			"susp": susp,
			"steer": steer,
			"wheel": wheel,
			"rest_susp_pos": (susp.position if susp != null else Vector3.ZERO),
			"is_front": simulation.get_wheel_is_front(vehicle_name, i),
			"steered": simulation.get_wheel_steered(vehicle_name, i),
		}
		_wheel_nodes.append(entry)
		if susp == null or wheel == null:
			missing += 1
	if missing > 0:
		push_warning("vehicle_visual.gd: %d/%d wheel(s) missing a susp_*/wheel_* node in the model - those stay at their static rest position" % [missing, wheel_count])
	_align_model_to_physics_wheels()

# Mount the model so its wheel centres coincide with the PHYSICS wheel
# centres. The rig's origin is the ground point below the wheel centres
# (car_sedan.rig.json "origin"), but the chassis body's origin is wherever
# body_visuals.gd put the chassis box - see SessionConfig::chassis_z_m.
# Physics: a wheel centre sits at attachment_local + axis*compression
# (wheel_instance.cpp, axis = ISO +z here). Model: at rest_susp_up + susp
# offset. So the model root shifts by the mean of (attach - rest_susp_pos,
# re-permuted into ISO) per axis, and each susp node carries its own small
# residual bias, making every wheel exact even if the art and the vehicle
# file disagree a little (same formula as the reference file's own comment).
func _align_model_to_physics_wheels() -> void:
	var diffs: Array = [] # ISO-local (dx, dy, dz) per wheel, or null
	for i in range(_wheel_nodes.size()):
		var w = _wheel_nodes[i]
		if w["susp"] == null:
			diffs.append(null)
			continue
		var attach: Vector3 = simulation.get_wheel_attachment_local(vehicle_name, i)
		# susp_* is a direct child of the model's body node at the origin,
		# its position in the file's own gltf axes. gltf(x,y,z) = iso(y,z,x)
		# (car_sedan.rig.json "frame"), so re-permuting rest_susp_pos back
		# into ISO gives (rest_susp_pos.z, rest_susp_pos.x, rest_susp_pos.y).
		var rest_susp_pos: Vector3 = w["rest_susp_pos"]
		var rest_iso := Vector3(rest_susp_pos.z, rest_susp_pos.x, rest_susp_pos.y)
		diffs.append(attach - rest_iso)
	var sum := Vector3.ZERO
	var n := 0
	for d in diffs:
		if d != null:
			sum += d
			n += 1
	if n == 0:
		return
	var d_mean: Vector3 = sum / n
	_model_root.position = d_mean # ISO body-local, like the basis above
	for i in range(_wheel_nodes.size()):
		_wheel_nodes[i]["susp_bias"] = (diffs[i] - d_mean) if diffs[i] != null else Vector3.ZERO

func _process(delta: float) -> void:
	if simulation == null or vehicle_name == "":
		return
	# Self-heal the "bound too early" race documented on _bind_wheel_nodes'
	# own wheel_count==0 branch above: retry every frame while empty, cheap
	# (one int RPC) and self-limiting (stops retrying the instant it binds).
	if _wheel_nodes.is_empty():
		_bind_wheel_nodes()
		if _wheel_nodes.is_empty():
			return
	var t0 := Time.get_ticks_usec()

	for i in range(_wheel_nodes.size()):
		var w = _wheel_nodes[i]
		var compression: float = simulation.get_wheel_compression(vehicle_name, i)
		var spin_angle: float = simulation.get_wheel_spin_angle(vehicle_name, i)

		if w["susp"] != null:
			# Suspension axis is ISO [0,0,1] (up) for every corner of this
			# rig. susp_*'s own local translation is a RAW gltf-local
			# offset (child nodes keep the file's own, un-permuted axes -
			# see _load_model's comment), and the file's up axis is its
			# own +Y (gltf is Y-up; iso.z=gltf.y). Real simulated state
			# (WheelState::suspension_travel), plus this wheel's own
			# residual (dx, dy, dz) bias left over after
			# _align_model_to_physics_wheels' common per-axis mean was
			# removed (usually exactly zero), re-permuted from ISO back
			# into the file's own gltf-local axes (inverse of the
			# iso(x,y,z)=gltf(z,x,y) permutation used when the bias was
			# computed: gltf.x=iso.y, gltf.y=iso.z, gltf.z=iso.x).
			var bias_iso: Vector3 = w.get("susp_bias", Vector3.ZERO)
			var bias_gltf := Vector3(bias_iso.y, bias_iso.z, bias_iso.x)
			w["susp"].position = w["rest_susp_pos"] + Vector3(0, compression, 0) + bias_gltf

		if w["steer"] != null and w["steered"]:
			# Real simulated per-wheel steer angle (WheelState::steer_angle,
			# ackermann_wheel_angle()'s actual result for THIS wheel -
			# correct for any ackermann_fraction, front or rear). Steer axis
			# ISO [0,0,1] (up) -> gltf-local +Y (see susp_* comment above).
			var steer_angle: float = simulation.get_wheel_steer_angle(vehicle_name, i)
			w["steer"].rotation = Vector3(0, steer_angle, 0)

		if w["wheel"] != null:
			# Real simulated, already-integrated spin angle (WheelState::
			# spin_angle - unbounded, wrapped here only for float precision
			# at long drives, same as fmod'ing it would do). Wheel joint
			# axis ISO [0,1,0] (left/right axle axis, "+ = rolling forward")
			# -> gltf-local +X (iso.y=gltf.x - see _load_model's comment).
			w["wheel"].rotation = Vector3(fmod(spin_angle, TAU), 0, 0)

	if simulation.has_method("add_adapter_time_us"):
		simulation.add_adapter_time_us(Time.get_ticks_usec() - t0)

func has_driver_eye() -> bool:
	return _model_root != null and _model_root.find_child("socket_driver_eye", true, false) != null

func driver_eye_local() -> Vector3:
	if not has_driver_eye():
		return Vector3(-0.35, 0.38, 0.6)
	var eye := _model_root.find_child("socket_driver_eye", true, false) as Node3D
	return global_transform.affine_inverse() * eye.global_position
