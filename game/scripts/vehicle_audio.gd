extends Node3D
# Same tyre model, live engine voice and Windows object backend as physics_sim.
# Each source is relative to the active camera, including the tracked VR head.
const TyreModel = preload("res://scripts/tyre_sound_model.gd")
const RATE := 48000
var simulation: Node
var director: Node
var vehicle_name := "car_hyper"
var tyres: Array = []
var tyre_points := PackedVector3Array()
var tyre_values := PackedFloat32Array()
var tyre_spatial: RefCounted
var engine_spatial: RefCounted
var fallback_players: Array[AudioStreamPlayer3D] = []
var engine_players: Array[AudioStreamPlayer3D] = []
var engine_points := PackedVector3Array()
var engine_rate := 0
var _sample_even := true
var _capacity := 0
var _engine_capacity := 0
var _initialized := false
var _shutdown := false

func _open_spatial(rate: int, channels: int) -> RefCounted:
	if "--audio=godot" in OS.get_cmdline_user_args() or "--m1-audio=godot" in OS.get_cmdline_user_args():
		return null
	var backend: RefCounted = ClassDB.instantiate("PsSpatialAudio")
	backend.set_reference_distance_m(8.0)
	backend.set_reflections_gain(0.15)
	var opened: bool = backend.open(rate, channels)
	print("RG_AUDIO ", backend.get_status())
	if not opened:
		backend.close()
		return null
	return backend

func _player(rate: int, point: Vector3) -> AudioStreamPlayer3D:
	var stream := AudioStreamGenerator.new()
	stream.mix_rate = rate
	stream.buffer_length = 0.15
	var player := AudioStreamPlayer3D.new()
	player.stream = stream
	player.position = point
	player.unit_size = 8.0
	player.max_distance = 100.0
	add_child(player)
	player.play()
	return player

func _initialize_audio() -> void:
	_initialized = true
	if not "--no-tyre-audio" in OS.get_cmdline_user_args():
		for wheel in range(simulation.get_vehicle_wheel_count(vehicle_name)):
			tyres.append(TyreModel.new(1234 + wheel * 997))
			var point: Vector3 = simulation.get_wheel_attachment_local(vehicle_name, wheel)
			point.z -= simulation.get_wheel_radius(vehicle_name, wheel)
			tyre_points.append(point)
			tyre_values.append(0.0)
		if not tyres.is_empty():
			tyre_spatial = _open_spatial(RATE, tyres.size())
			if tyre_spatial == null:
				for point in tyre_points:
					fallback_players.append(_player(RATE, point))
	var voice: Dictionary = simulation.start_engine_audio()
	print("RG_AUDIO engine voice: ", voice.get("reason", "unknown"))
	if voice.get("active", false):
		engine_rate = int(voice.rate)
		engine_points = simulation.get_engine_audio_positions()
		engine_spatial = _open_spatial(engine_rate, int(voice.channels))
		if engine_spatial == null:
			for point in engine_points:
				engine_players.append(_player(engine_rate, point))

func _relative(points: PackedVector3Array) -> PackedVector3Array:
	var result := PackedVector3Array()
	var listener: Camera3D = director.active_camera()
	if listener == null:
		return result
	var inverse := listener.global_transform.affine_inverse()
	for point in points:
		result.append(inverse * (global_transform * point))
	return result

func _fallback_needed(players: Array[AudioStreamPlayer3D], rate: int, engine: bool) -> int:
	if players.is_empty():
		return 0
	var playback := players[0].get_stream_playback() as AudioStreamGeneratorPlayback
	if playback == null:
		return 0
	var available := playback.get_frames_available()
	if engine:
		_engine_capacity = maxi(_engine_capacity, available)
		return mini(available, maxi(0, rate / 20 - (_engine_capacity - available)))
	_capacity = maxi(_capacity, available)
	return mini(available, maxi(0, rate / 20 - (_capacity - available)))

func _output(samples: PackedFloat32Array, channels: int, backend: RefCounted, players: Array[AudioStreamPlayer3D]) -> void:
	if backend != null:
		backend.push_audio(samples)
		return
	for channel in range(players.size()):
		var playback := players[channel].get_stream_playback() as AudioStreamGeneratorPlayback
		if playback == null:
			continue
		var buffer := PackedVector2Array()
		buffer.resize(samples.size() / channels)
		for frame in range(buffer.size()):
			var sample := samples[frame * channels + channel]
			buffer[frame] = Vector2(sample, sample)
		playback.push_buffer(buffer)

func _process(_delta: float) -> void:
	if not simulation.is_running() or simulation.get_step_count() == 0:
		return
	if not _initialized:
		_initialize_audio()
	global_transform = simulation.get_body_transform("chassis")
	# An endpoint failure falls back once, preserving a single consumer.
	if tyre_spatial != null and not tyre_spatial.is_active():
		print("RG_AUDIO tyre fallback: ", tyre_spatial.get_status())
		tyre_spatial.close()
		tyre_spatial = null
		for point in tyre_points:
			fallback_players.append(_player(RATE, point))
	if engine_spatial != null and not engine_spatial.is_active():
		print("RG_AUDIO engine fallback: ", engine_spatial.get_status())
		engine_spatial.close()
		engine_spatial = null
		for point in engine_points:
			engine_players.append(_player(engine_rate, point))
	var frozen: bool = simulation.get_streaming_status().get("frozen", false)
	for wheel in range(tyres.size()):
		var radius: float = simulation.get_wheel_radius(vehicle_name, wheel)
		tyres[wheel].update(simulation.get_body_speed_mps("chassis"), simulation.get_wheel_omega(vehicle_name, wheel) * radius,
			simulation.get_wheel_slip_ratio(vehicle_name, wheel), simulation.get_wheel_slip_angle(vehicle_name, wheel),
			0.0 if frozen else simulation.get_wheel_load_n(vehicle_name, wheel), simulation.get_wheel_fx(vehicle_name, wheel),
			simulation.get_wheel_fy(vehicle_name, wheel), simulation.get_wheel_surface_name(vehicle_name, wheel), 0.0)
	var needed := 0
	if tyre_spatial != null:
		tyre_spatial.set_positions(_relative(tyre_points))
		needed = tyre_spatial.get_frames_needed()
	else:
		needed = _fallback_needed(fallback_players, RATE, false)
	var pcm := PackedFloat32Array()
	pcm.resize(needed * tyres.size())
	for frame in range(needed):
		for wheel in range(tyres.size()):
			# Preserve the demo's 24 kHz sample pitch when feeding 48 kHz output.
			if _sample_even:
				tyre_values[wheel] += (tyres[wheel].sample(1.0 / 24000.0) - tyre_values[wheel]) * 0.65
			pcm[frame * tyres.size() + wheel] = tyre_values[wheel]
		_sample_even = not _sample_even
	if not tyres.is_empty():
		_output(pcm, tyres.size(), tyre_spatial, fallback_players)
	if engine_rate > 0:
		simulation.update_engine_audio()
		if engine_spatial != null:
			engine_spatial.set_positions(_relative(engine_points))
			needed = engine_spatial.get_frames_needed()
		else:
			needed = _fallback_needed(engine_players, engine_rate, true)
		_output(simulation.read_engine_audio(needed), engine_points.size(), engine_spatial, engine_players)

func shutdown() -> void:
	if _shutdown:
		return
	_shutdown = true
	set_process(false)
	for backend in [tyre_spatial, engine_spatial]:
		if backend != null:
			backend.close()
	for player in fallback_players + engine_players:
		player.stop()
	if is_instance_valid(simulation):
		simulation.stop_engine_audio()
	tyres.clear()
	fallback_players.clear()
	engine_players.clear()
	tyre_spatial = null
	engine_spatial = null

func _exit_tree() -> void:
	shutdown()
