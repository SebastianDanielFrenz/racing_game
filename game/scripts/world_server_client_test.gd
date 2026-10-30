extends SceneTree

func _initialize() -> void:
	run.call_deferred()

func run() -> void:
	var repo := ProjectSettings.globalize_path("res://..").simplify_path()
	var sky = ClassDB.instantiate("RgSky")
	if not sky.initialize(repo.path_join("data/world/sky.json"), repo.path_join("data/world/world_config.json")):
		push_error(sky.get_error())
		quit(1)
		return
	var deadline := Time.get_ticks_msec() + 10000
	while Time.get_ticks_msec() < deadline:
		sky.advance(0.02)
		var state: Dictionary = sky.get_state()
		if state.get("server_status", "") == "connected":
			if not state.get("server_authoritative", false):
				quit(1)
				return
			for item in sky.get_menu():
				if item.enabled:
					push_error("Local weather control stayed enabled")
					quit(1)
					return
			if sky.choose("weather:clear"):
				quit(1)
				return
			var before := float(state.local_time_h)
			sky.set_local_time_h(0.0)
			if absf(float(sky.get_state().local_time_h) - before) > 0.001:
				quit(1)
				return
			print("RG_WORLD_GODOT_PASS authority controls clock weather")
			sky = null
			quit(0)
			return
		await create_timer(0.02).timeout
	push_error("World client failed: " + str(sky.get_state().get("server_status", "")))
	sky = null
	quit(1)
