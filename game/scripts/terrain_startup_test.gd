# Real-store startup regression: run with --headless --script res://scripts/terrain_startup_test.gd.
# Set RG_G2M_DERIVED to a new directory to exercise cold derivation; leaves player settings untouched.
extends SceneTree
func _initialize():
    run.call_deferred()
func run():
    var sim = ClassDB.instantiate("RgSimulation")
    root.add_child(sim)
    var args = OS.get_cmdline_user_args()
    var config = args[0] if args.size() > 0 else ProjectSettings.globalize_path("res://../data/world/world_config_rhein_main_corridor.json")
    var began = Time.get_ticks_msec()
    if not sim.initialize_terrain(config, ProjectSettings.globalize_path("res://../data/vehicles/car_hyper.json"), ProjectSettings.globalize_path("res://../data/surfaces/surfaces.json")):
        push_error(sim.get_last_error()); quit(1); return
    while true:
        var status = sim.get_init_status()
        if status.state == "ready":
            print("STARTUP_TEST_READY elapsed_ms=%d status=%s" % [Time.get_ticks_msec()-began, JSON.stringify(status)])
            sim.queue_free()
            await process_frame
            quit(0); return
        if status.state == "error" or Time.get_ticks_msec()-began > 60000:
            push_error("STARTUP_TEST_FAILED %s" % JSON.stringify(status))
            sim.queue_free()
            await process_frame
            quit(1); return
        await process_frame