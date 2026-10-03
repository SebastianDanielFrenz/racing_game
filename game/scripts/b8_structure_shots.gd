extends SceneTree
var main: Node
var camera: Camera3D
const POINTS := [
	[466008.193,5552282.039,"01_ramp_bridges"],
	[465373.418,5552475.811,"02_track_bridge"],
	[464427.504,5552838.622,"03_b8_bridge"],
	[463684.336,5553262.153,"04_l3014_bridge"],
	[463287.123,5553871.644,"05_tunnel_roof"],
	[462834.583,5554537.481,"06_tunnel_roof"],
	[461889.184,5555387.768,"07_track_bridge"]]
func _initialize() -> void:
	_run.call_deferred()
func _godot(p: Vector3,origin: Vector3) -> Vector3:
	var d := p-origin
	return Vector3(-d.y,d.z,-d.x)
func _run() -> void:
	main=load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	var began := Time.get_ticks_msec()
	while main.world_state!="running":
		if main.world_state=="failed" or Time.get_ticks_msec()-began>300000:
			push_error("RG_B8_SHOTS world startup failed");quit(1);return
		await process_frame
	main.scripted_controls={"throttle":0.0,"brake":1.0,"handbrake":1.0,"steer":0.0}
	main.get_director().set_process(false)
	camera=Camera3D.new();camera.fov=65;camera.near=.2;camera.far=25000
	main.add_child(camera);camera.current=true
	var sim: Node=main.get_simulation()
	var view: Node=main.get_world_view()
	var directory := ProjectSettings.globalize_path("res://../out/b8_structures")
	DirAccess.make_dir_recursive_absolute(directory)
	for point in POINTS:
		var x: float=point[0]-464000
		var y: float=point[1]-5559000
		var before: int=sim.get_streaming_status().get("relocations",0)
		sim.relocate_vehicle(x,y,150)
		while int(sim.get_streaming_status().get("relocations",0))==before:
			await process_frame
		var origin: Vector3=sim.get_render_origin_session()
		view.set_render_origin(origin);view.update_focus(x,y)
		for frame in range(30):
			await process_frame
		while not view.is_stream_idle():
			await process_frame
		var car: Vector3=main.chassis_session_position()
		var target := Vector3(x,y,car.z+3)
		camera.position=_godot(target+Vector3(55,-55,38),origin)
		camera.look_at(_godot(target,origin),Vector3.UP)
		await process_frame
		await RenderingServer.frame_post_draw
		var path := directory.path_join(str(point[2])+".png")
		var result := root.get_texture().get_image().save_png(path)
		print("RG_B8_SHOTS file=%s result=%d" % [path,result])
		if result!=OK:
			quit(1);return
	print("RG_B8_SHOTS result=ok")
	quit(0)
