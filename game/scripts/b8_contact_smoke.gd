extends SceneTree
# Drives across the three original B8 depressions using the shipped hypercar.
var main: Node
var points: Array = []
var stations: Array[float] = []
var began := Time.get_ticks_msec()

func _initialize() -> void:
	_run.call_deferred()

func _fail(message: String) -> void:
	push_error("RG_B8_CONTACT " + message)
	quit(1)

func _target(station: float) -> Vector3:
	for i in range(1, stations.size()):
		if stations[i] >= station:
			var t := (station-stations[i-1]) / (stations[i]-stations[i-1])
			var a := Vector2(points[i-1][0],points[i-1][1])
			var b := Vector2(points[i][0],points[i][1])
			var p := a.lerp(b,t)
			return Vector3(p.x,p.y,rad_to_deg((b-a).angle()))
	return Vector3.ZERO

func _run() -> void:
	var route = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/routes/home_r1_drive.json")))
	points = route["waypoints"]
	stations.append(0.0)
	for i in range(1,points.size()):
		stations.append(stations[-1]+Vector2(points[i][0]-points[i-1][0],points[i][1]-points[i-1][1]).length())
	main = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	while main.world_state != "running":
		if main.world_state == "failed" or Time.get_ticks_msec()-began>300000:
			_fail("world failed to become ready")
			return
		await process_frame
	var sim: Node = main.get_simulation()
	for start_s in [2800.0,4390.0,5200.0]:
		main.scripted_controls={"throttle":0.0,"brake":1.0,"handbrake":1.0,"steer":0.0,"clutch":0.0,"ignition":1.0,"starter":0.0,"assist.auto_shift":1.0}
		var before: int = sim.get_streaming_status().get("relocations",0)
		var target := _target(start_s)
		sim.relocate_vehicle(target.x,target.y,target.z)
		var mark := Time.get_ticks_msec()
		while int(sim.get_streaming_status().get("relocations",0)) == before:
			if Time.get_ticks_msec()-mark>60000:
				_fail("relocation timed out")
				return
			await process_frame
		var drive_start: float = sim.get_sim_time()
		var drive_wall := Time.get_ticks_msec()
		var drive_ticks: int = sim.get_step_count()
		var station: float = start_s
		var contact_loss := 0.0
		var max_contact_loss := 0.0
		var last_time: float = drive_start
		var max_lateral := 0.0
		while station < start_s+140:
			await process_frame
			if Time.get_ticks_msec()-drive_wall>35000:
				_fail("drive timed out at station %.1f" % station)
				return
			var pos: Vector3 = main.chassis_session_position()
			var closest := 100000.0
			for i in range(points.size()):
				if stations[i]<station-20 or stations[i]>station+30:
					continue
				var distance := Vector2(pos.x-points[i][0],pos.y-points[i][1]).length()
				if distance<closest:
					closest=distance
					station=stations[i]
			max_lateral=maxf(max_lateral,closest)
			var aim := _target(station+12)
			var direction := Vector2(aim.x-pos.x,aim.y-pos.y).normalized()
			var forward: Vector3 = sim.get_body_transform("chassis").basis.x
			var heading := Vector2(-forward.z,-forward.x).normalized()
			var steer := clampf(heading.angle_to(direction)*1.8,-.35,.35)
			var speed: float = sim.get_body_speed_mps("chassis")
			main.scripted_controls={"throttle":clampf((11.0-speed)*.10,0.0,.6),"brake":clampf((speed-12.0)*.2,0.0,.3),"handbrake":0.0,"steer":steer,"clutch":0.0,"ignition":1.0,"starter":1.0 if sim.get_sim_time()-drive_start<1 else 0.0,"assist.auto_shift":1.0}
			var load_sum := 0.0
			for wheel in range(sim.get_vehicle_wheel_count(main.VEHICLE_NAME)):
				load_sum += sim.get_wheel_load_n(main.VEHICLE_NAME,wheel)
			var now: float = sim.get_sim_time()
			contact_loss=contact_loss+now-last_time if load_sum<1000 else 0.0
			max_contact_loss=maxf(max_contact_loss,contact_loss)
			last_time=now
		var ss: Dictionary = sim.get_streaming_status()
		var hz: float = (sim.get_step_count()-drive_ticks)*1000.0/(Time.get_ticks_msec()-drive_wall)
		print("RG_B8_CONTACT station=%.1f:%.1f max_contact_loss_s=%.3f max_lateral_m=%.2f measured_hz=%.1f falls=%d misses=%d" % [start_s,station,max_contact_loss,max_lateral,hz,ss.get("falls",0),ss.get("fill_misses",0)])
		if max_contact_loss>.25 or max_lateral>5 or int(ss.get("falls",0))>0 or int(ss.get("fill_misses",0))>0:
			_fail("B8 contact or route containment check failed")
			return
	main.scripted_controls={"throttle":0.0,"brake":1.0,"handbrake":1.0,"steer":0.0}
	print("RG_B8_CONTACT result=ok")
	quit(0)
