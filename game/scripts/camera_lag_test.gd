extends SceneTree
# game/scripts/camera_lag_test.gd - camera lag audit of the drive-view rigs
# (and the drone rig), headless, no physics:
#   godot --headless --path game --script res://scripts/camera_lag_test.gd
#
# A fake simulation node moves a chassis transform at 50 / 150 / 300 km/h,
# straight and in a steady turn (radius v^2 / 8 m/s^2, 0.8 g), at 60 frames
# per second. Each rig runs its real update_rig() for 8 s; then the camera is
# compared with the rig's IDEAL pose for that chassis transform (the pose the
# rig aims for with every smoothing switched off): position error in metres and
# the horizontal angle between the camera's view direction and the ideal one.
# Prints one RG_CAMERA_LAG line per (rig, speed, path) and a final
# RG_CAMERA_LAG_DONE line; exits 1 when a rig trails the car on the straight
# (error over MAX_STRAIGHT_ERR_M: lag that grows with speed, the old orbit/drone
# behaviour) or swings wide in the turn (over MAX_TURN_ERR_M; the chase/drone
# rigs smooth their heading on purpose, which shows as a bounded swing error).
# Head motion and speed effects are switched off (they are a deliberate eye
# offset, not lag); camera_motion is measured by the core tests.

const FPS := 60.0
const RUN_S := 8.0
const LAT_ACCEL := 8.0
const MAX_STRAIGHT_ERR_M := 0.05
const MAX_TURN_ERR_M := 2.0

class FakeSim:
	extends Node
	var xform := Transform3D.IDENTITY
	func get_step_count() -> int: return 1
	func get_body_transform(_name: String) -> Transform3D: return xform
	func get_drone_target_transform() -> Variant: return xform
	func get_camera_ground_height(_p: Vector3) -> Variant: return null
	func get_bumper_camera(_rig_path: String = "", _offset: Vector3 = Vector3.ZERO) -> Dictionary:
		return {"eye_local": Vector3(2.4, 0.0, -0.05), "pitch_down_deg": 1.5, "fov_deg": 80.0, "source": "fake"}

func _car(t: float, v: float, turn: bool) -> Transform3D:
	var pos := Vector3.ZERO
	var fwd := Vector3(0, 0, -1)
	if not turn:
		pos = fwd * v * t
	else:
		var r := v * v / LAT_ACCEL
		var a := v * t / r # left turn (counter-clockwise seen from above)
		pos = Vector3(-r + r * cos(a), 0.0, -r * sin(a))
		fwd = Vector3(-sin(a), 0.0, -cos(a))
	var left := Vector3.UP.cross(fwd)
	return Transform3D(Basis(fwd, left, Vector3.UP), pos + Vector3.UP * 0.5)

func _ideal(rig_name: String, rig: Node3D, c: Transform3D) -> Array:
	var fwd: Vector3 = c.basis.x
	var flat := Vector3(fwd.x, 0, fwd.z).normalized()
	match rig_name:
		"chase":
			var p: Vector3 = c.origin - fwd * rig.follow_distance_m + Vector3.UP * rig.follow_height_m
			return [p, (c.origin + Vector3.UP * rig.look_height_m) - p]
		"drone":
			var d: float = rig.follow_distance_m
			var p2: Vector3 = c.origin - flat * d + Vector3.UP * d * rig.height_per_distance
			return [p2, (c.origin + Vector3.UP * rig.look_height_m) - p2]
		"orbit":
			var o: Vector3 = rig._state["offset"]
			var focus: Vector3 = c.origin + Vector3.UP
			var p3: Vector3 = focus + flat * o.x + Vector3.UP.cross(flat) * o.y + Vector3.UP * o.z
			return [p3, focus - p3]
		"bumper":
			return [c * (rig._mount["eye_local"] as Vector3), fwd]
		"cockpit":
			return [c * Vector3(-0.35, 0.38, 0.6), fwd]
	return [Vector3.ZERO, fwd]

func _hangle(a: Vector3, b: Vector3) -> float:
	var x := Vector2(a.x, a.z).normalized()
	var y := Vector2(b.x, b.z).normalized()
	return rad_to_deg(absf(x.angle_to(y)))

func _initialize() -> void:
	_run.call_deferred()

func _run() -> void:
	await process_frame
	var sim := FakeSim.new()
	root.add_child(sim)
	var worst := 0.0
	var failed := false
	for rig_name in ["chase", "bumper", "cockpit", "orbit", "drone"]:
		for kph in [50.0, 150.0, 300.0]:
			for turn in [false, true]:
				var rig := Node3D.new()
				rig.set_script(load("res://scripts/%s_rig.gd" % rig_name))
				if "motion_enabled" in rig:
					rig.motion_enabled = false
				root.add_child(rig) # _ready builds the camera
				var v: float = kph / 3.6
				var dt := 1.0 / FPS
				var steps := int(RUN_S * FPS)
				sim.xform = _car(0.0, v, turn)
				rig.activate(sim.xform)
				for i in range(steps + 1):
					sim.xform = _car(i * dt, v, turn)
					rig.update_rig(dt, sim, null, false)
				var ideal := _ideal(rig_name, rig, sim.xform)
				var cam: Vector3 = rig.camera.global_position
				var pos_err: float = cam.distance_to(ideal[0])
				var ang_err: float = _hangle(-rig.camera.global_basis.z, ideal[1])
				print("RG_CAMERA_LAG rig=%s kph=%d path=%s pos_err_m=%.3f ang_err_deg=%.2f" % [rig_name, int(kph), "turn" if turn else "straight", pos_err, ang_err])
				worst = maxf(worst, pos_err)
				if pos_err > (MAX_TURN_ERR_M if turn else MAX_STRAIGHT_ERR_M):
					failed = true
				rig.queue_free()
				await process_frame
	print("RG_CAMERA_LAG_DONE worst_pos_err_m=%.3f result=%s" % [worst, "FAIL" if failed else "PASS"])
	quit(1 if failed else 0)
