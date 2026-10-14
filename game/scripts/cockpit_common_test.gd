extends SceneTree
const Cockpit := preload("res://scripts/cockpit_common.gd")
class IdleSimulation extends Node:
	func get_vehicle_gauge_info(_vehicle: String) -> Dictionary: return {}
	func get_vehicle_powertrain(_vehicle: String) -> Dictionary: return {}
	func is_running() -> bool: return false
var failures := 0
func check(value: bool, message: String) -> void:
	if not value:
		failures += 1
		push_error(message)
func load_car() -> Node3D:
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_file(ProjectSettings.globalize_path("res://../external/physics_sim/data/models/car_hyper/car_hyper.glb"),state)
	check(err == OK,"car GLB loads")
	return doc.generate_scene(state)
func _initialize() -> void:
	call_deferred("run")
func run() -> void:
	check(load("res://scripts/vehicle_visual.gd") != null,"drive visual script parses")
	check(load("res://scripts/garage_scene.gd") != null,"garage scene script parses")
	var car := load_car()
	root.add_child(car)
	var cockpit := Cockpit.attach_to(car)
	await process_frame
	check(cockpit.mounted_count() == 18,"all common mounts loaded")
	check(car.find_child("cockpit_tc",true,false) != null,"tunnel TC present")
	check(car.find_child("cockpit_hazard",true,false) != null,"hazard present")
	check(car.find_child("cockpit_start",true,false) != null,"start present")
	check(car.find_child("part_n2o",true,false) == null,"N2O excluded")
	check(car.find_child("part_shifter_seq",true,false) == null,"selector excluded")
	var wheel := car.find_child("steering_wheel",true,false) as Node3D
	var horn := car.find_child("cockpit_horn",true,false) as Node3D
	var paddle := car.find_child("cockpit_paddle_L",true,false) as Node3D
	check(horn.get_parent() == wheel and paddle.get_parent() == wheel,"wheel-mounted controls parented to wheel")
	var relative := wheel.global_transform.affine_inverse()*paddle.global_transform
	wheel.rotate_x(0.5)
	check((wheel.global_transform.affine_inverse()*paddle.global_transform).is_equal_approx(relative),"paddle follows wheel")
	check(Cockpit.attach_to(car) == cockpit,"attach is idempotent")
	cockpit.update_lamps({"assist_fitted_bits":8,"traction_modes_available_bits":31,"traction_mode_effective":3,"drive_modes_available_bits":7,"drive_mode_effective":2,"engine_state":"Running"})
	check(cockpit._materials["lamp_tc_led3"][0].emission_enabled,"effective TC lamp")
	check(not cockpit._materials["lamp_tc_led1"][0].emission_enabled,"other TC lamps off")
	check(cockpit._materials["lamp_drive_mode_sport_led0"][0].emission_enabled,"Sport lamp")
	check(cockpit._materials["lamp_start_ring"][0].emission_enabled,"Running ring")
	cockpit.update_lamps({})
	check(not cockpit._materials["lamp_tc_led3"][0].emission_enabled,"missing telemetry stays unlit")
	var clone := car.duplicate() as Node3D
	root.add_child(clone)
	await process_frame
	check(clone.find_children("cockpit_*","Node3D",true,false).size()==18,"garage reflection does not remount parts")
	# The same layout automatically uses a new socket's position and orientation.
	var car2 := load_car()
	var socket := Node3D.new()
	socket.name = "socket_btn_L1"
	socket.position = Vector3(0.2,0.5,0.3)
	socket.set_meta("normal",[-1,0,0])
	socket.set_meta("up",[0,0,1])
	car2.add_child(socket)
	root.add_child(car2)
	Cockpit.attach_to(car2)
	await process_frame
	var tc := car2.find_child("cockpit_tc",true,false) as Node3D
	check(tc.get_parent()==socket and tc.position==Vector3.ZERO,"actual socket overrides fallback")
	check(tc.basis.is_equal_approx(Cockpit.mount_basis([-1,0,0],[0,0,1])),"socket orientation applied")
	check(absf(tc.basis.determinant()-1)<0.00001,"mount basis preserves handedness")
	# Exercise the actual driving loader, not only the standalone mounting helper.
	var visual := load("res://scripts/vehicle_visual.gd").new() as Node3D
	var idle := IdleSimulation.new()
	root.add_child(idle)
	visual.simulation = idle
	visual.model_absolute_path = ProjectSettings.globalize_path("res://../external/physics_sim/data/models/car_hyper/car_hyper.glb")
	root.add_child(visual)
	await process_frame
	check(visual.find_children("cockpit_*","Node3D",true,false).size()==18,"actual drive loader integrates common parts")
	visual.free()
	idle.free()
	# Run only the garage model loader, avoiding setup/dyno/native-engine changes.
	var garage: CanvasLayer = load("res://scripts/garage_scene.gd").new()
	var car_pivot := Node3D.new()
	var mirror_pivot := Node3D.new()
	root.add_child(car_pivot); root.add_child(mirror_pivot)
	garage._car_pivot = car_pivot
	garage._mirror_pivot = mirror_pivot
	garage.vehicle_id = "car_hyper"
	check(garage._load_car({"model_path":ProjectSettings.globalize_path("res://../external/physics_sim/data/models/car_hyper/car_hyper.glb")}),"actual garage loader succeeds")
	await process_frame
	check(car_pivot.find_children("cockpit_*","Node3D",true,false).size()==18,"actual garage car has common controls")
	check(mirror_pivot.find_children("cockpit_*","Node3D",true,false).size()==18,"actual garage reflection has exactly one set")
	garage.free(); car_pivot.free(); mirror_pivot.free()
	clone.free(); car2.free()
	if "--capture" in OS.get_cmdline_user_args():
		wheel.rotation = Vector3.ZERO
		await capture(car)
	car.free()
	await process_frame
	print("RG_COCKPIT_COMMON_TEST failures=%d" % failures)
	quit(1 if failures else 0)

func capture(_car: Node3D) -> void:
	root.size = Vector2i(1400,1000)
	DisplayServer.window_set_size(root.size)
	var world := WorldEnvironment.new()
	world.environment = Environment.new()
	world.environment.background_mode = Environment.BG_COLOR
	world.environment.background_color = Color(0.15,0.18,0.23)
	world.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	world.environment.ambient_light_color = Color(0.7,0.75,0.85)
	world.environment.ambient_light_energy = 0.8
	root.add_child(world)
	var light := OmniLight3D.new()
	light.position = Vector3(0.3,1.2,0.0)
	light.omni_range = 3.0
	light.light_energy = 1.2
	root.add_child(light)
	var camera := Camera3D.new()
	camera.near = 0.02
	camera.fov = 75
	camera.position = Vector3(0.33,0.93,-0.08)
	root.add_child(camera)
	camera.look_at(Vector3(0.0,0.48,0.18),Vector3.UP)
	camera.current = true
	for frame in range(8): await process_frame
	await RenderingServer.frame_post_draw
	var image := root.get_texture().get_image()
	check(image != null and not image.is_empty(),"cockpit rendered")
	if image != null:
		var path := ProjectSettings.globalize_path("res://../out/cockpit_common_in_car.png")
		check(image.save_png(path)==OK,"cockpit screenshot saved")
		print("RG_COCKPIT_COMMON_CAPTURE "+path)
