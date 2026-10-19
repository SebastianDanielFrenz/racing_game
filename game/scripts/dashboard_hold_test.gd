extends SceneTree
# Owner 2026-10-18: the dashboard dropped to "N, 0 km/h, 0 rpm" in D with the hybrid car. RgSimulation answers an EMPTY
# powertrain dictionary when it cannot find the car for a frame; every dashboard now holds the last valid reading
# (powertrain_feed.gd) and logs the first miss once per drive. This drives the real digital cluster with a simulation that
# loses the car for a few frames.
const Cluster := preload("res://scripts/digital_cluster.gd")
const Feed := preload("res://scripts/powertrain_feed.gd")

class FakeSim extends Node:
	var present := true
	var speed_mps := 30.0
	func is_running() -> bool: return true
	func get_step_count() -> int: return 100
	func get_vehicle_gauge_info(_n: String) -> Dictionary: return {"fuel_capacity_kg": 0.0}
	func get_streaming_status() -> Dictionary: return {}
	func get_vehicle_thermal(_n: String) -> Dictionary: return {}
	func get_control(_c: String) -> float: return 0.0
	func get_vehicle_ground_speed_mps(_n: String) -> float: return speed_mps if present else 0.0
	func get_vehicle_powertrain(_n: String) -> Dictionary:
		if not present:
			return {}
		return {"engine_state": "Running", "rpm": 5200.0, "gear": 3, "drive_power_kw": 250.0}

var failures := 0
func check(value: bool, message: String) -> void:
	if not value:
		failures += 1
		push_error(message)

func _initialize() -> void:
	call_deferred("run")

func run() -> void:
	# The feed on its own.
	var feed := Feed.new("test")
	check(feed.filter({}, "car").is_empty() and feed.misses == 1, "an empty reading before any valid one stays empty")
	var good := {"engine_state": "Running", "gear": 4}
	check(feed.filter(good, "car") == good, "a valid reading passes through")
	check(feed.filter({}, "car") == good, "an empty reading returns the last valid one")
	check(feed.misses == 2 and feed.logged, "misses counted, logged once")
	feed.reset()
	check(feed.last.is_empty() and feed.misses == 0 and not feed.logged, "reset starts a new drive")

	# The real digital cluster.
	var sim := FakeSim.new()
	root.add_child(sim)
	var cluster := Cluster.new()
	cluster.simulation = sim
	cluster.vehicle_name = "car"
	root.add_child(cluster)
	cluster._process_body(0.1)
	check(is_equal_approx(cluster.speed, 108.0) and cluster.gear == 3 and cluster.rpm > 5000.0, "cluster shows speed/gear/rpm (%s %s %s)" % [cluster.speed, cluster.gear, cluster.rpm])
	sim.present = false
	for i in range(5):
		cluster._process_body(0.1)
	check(is_equal_approx(cluster.speed, 108.0), "speed held while the car is not found (%s)" % cluster.speed)
	check(cluster.gear == 3 and cluster.rpm > 5000.0, "gear and rpm held (%s, %s)" % [cluster.gear, cluster.rpm])
	check(cluster.feed.misses == 5 and cluster.feed.logged, "five misses, logged once")
	sim.present = true
	sim.speed_mps = 40.0
	cluster._process_body(0.1)
	check(is_equal_approx(cluster.speed, 144.0), "live values resume (%s)" % cluster.speed)
	print("dashboard_hold_test: ", "PASS" if failures == 0 else "FAIL (%d)" % failures)
	quit(1 if failures > 0 else 0)
