extends SceneTree

class MockSimulation extends Node:
	func get_body_transform(_name: String) -> Transform3D:
		# ISO axes expressed in Godot coordinates, translated far from origin.
		return Transform3D(Basis(Vector3(0,0,-1), Vector3(-1,0,0), Vector3(0,1,0)), Vector3(100,20,300))

class MockInput extends Node:
	func get_camera_move() -> Vector3:
		return Vector3(0,0,1)
	func get_camera_fast() -> bool:
		return false

func _initialize() -> void:
	call_deferred("_test")

func _test() -> void:
	var backend: RefCounted = ClassDB.instantiate("PsSpatialAudio")
	assert(not backend.open(0, 1), "invalid audio rate accepted")
	assert(not backend.open(48000, 0), "zero channels accepted")
	backend.close()
	backend.close()
	assert(not backend.is_active())
	var sim := MockSimulation.new()
	var inputs := MockInput.new()
	var rig := XROrigin3D.new()
	rig.set_script(load("res://scripts/xr_rig.gd"))
	root.add_child(rig)
	rig.update_rig(0.0, sim, inputs, true)
	var chassis := sim.get_body_transform("chassis")
	assert(rig.position.distance_to(chassis * Vector3(-0.35,0.38,0.6)) < 0.001)
	assert((-rig.basis.z).distance_to(chassis.basis.x) < 0.001)
	var previous: Vector3 = rig.position
	rig.shift_origin(Vector3(-100,0,-300))
	assert(rig.position.distance_to(previous + Vector3(-100,0,-300)) < 0.001)
	rig.free_flight = true
	rig.activate(Transform3D.IDENTITY)
	rig.update_rig(0.25, sim, inputs, true)
	assert(rig.position.distance_to(Vector3(0,0,-10)) < 0.001)
	rig.queue_free()
	sim.free()
	inputs.free()

	var native: Node = ClassDB.instantiate("RgSimulation")
	root.add_child(native)
	var data := ProjectSettings.globalize_path("res://../external/physics_sim/data/")
	var vehicle := "car_hyper" if "--hyper-voice" in OS.get_cmdline_user_args() else "car_sedan_pipes"
	assert(native.initialize(data + "vehicles/" + vehicle + ".json", data + "surfaces/surfaces.json"))
	native.set_control("ignition", 1.0)
	native.start()
	await create_timer(0.1).timeout
	var info: Dictionary = native.get_vehicle_gauge_info(vehicle)
	assert(float(info.idle_rpm) > 0.0)
	assert(float(info.limiter_rpm) > float(info.idle_rpm))
	var voice: Dictionary = native.start_engine_audio()
	assert(voice.get("active", false), str(voice))
	assert(native.get_engine_audio_positions().size() == int(voice.channels))
	assert(native.read_engine_audio(-1).is_empty())
	var samples := 0
	var peak := 0.0
	for iteration in range(60):
		native.update_engine_audio()
		var pcm: PackedFloat32Array = native.read_engine_audio(2048)
		samples += pcm.size()
		for value in pcm:
			assert(is_finite(value))
			peak = maxf(peak, absf(value))
		await create_timer(1.0/60.0).timeout
	assert(samples > 0, "live voice produced no PCM")
	assert(peak > 0.0, "live voice was silent")
	native.stop_engine_audio()
	native.stop_engine_audio()
	native.stop()
	assert(native.read_engine_audio(100).is_empty())
	native.queue_free()
	await process_frame
	print("RG_PRESENTATION_TEST ok: XR seat/orientation/rebase/free flight, spatial validation, live voice car=", vehicle, " PCM samples=", samples, " peak=", peak)
	quit(0)
