extends SceneTree
# Standalone regression: --headless --path game --script res://scripts/driver_assists_test.gd
# -- --shell-user-dir <isolated test directory>. Exercises real install buttons and empty-category hiding.
class EmptyGarage extends Node:
    func get_options() -> Array:
        return []
var checks := 0
func check(ok: bool, message: String) -> bool:
    checks += 1
    if not ok:
        push_error("ASSIST_UI " + message)
        quit(1)
    return ok
func _initialize() -> void:
    run.call_deferred()
func run() -> void:
    if not "--shell-user-dir" in OS.get_cmdline_user_args():
        push_error("driver_assists_test requires an isolated --shell-user-dir")
        quit(2)
        return
    var main: Node = load("res://scenes/main.tscn").instantiate()
    root.add_child(main)
    var started := Time.get_ticks_msec()
    while main.shell_screen() != "main_menu":
        if Time.get_ticks_msec() - started > 30000:
            quit(1)
            return
        await process_frame
    var ui: CanvasLayer = main.get_shell_ui()
    var garage: Node = main.get_garage()
    ui.get_button("garage").pressed.emit()
    main._on_vehicle_chosen("car_hyper")
    ui._garage_path.assign(["Appearance & assists"])
    ui._draw_garage_options()
    if not check(ui.get_button("tile:Driver assists") != null, "driver assist tile missing"): return
    if not check(ui.get_button("tile:Paint") != null and ui.get_button("tile:Rims") != null, "appearance controls missing"): return
    ui.get_button("tile:Driver assists").pressed.emit()
    for id in ["assists_street_abs_esc", "assists_racing_abs_esc", "assists_abs"]:
        var button: Button = ui.get_button("part:assist_unit:" + id)
        if button == null and id == "assists_racing_abs_esc": button = ui.get_button("part:assist_unit:stock")
        if not check(button != null, "assist card missing " + id): return
        button.pressed.emit()
        if not check(bool(garage.get_validation().get("ok", false)), "assist install invalid " + id): return
    garage.reset_all()
    var hybrid: Dictionary = garage.set_option("powertrain_install", "hybrid_battery")
    if not check(bool(hybrid.get("accepted", false)), "hybrid conversion failed"): return
    ui._draw_garage_options()
    var race: Button = ui.get_button("part:assist_unit:assists_racing_abs_esc")
    if race == null: race = ui.get_button("part:assist_unit:stock")
    if not check(race != null, "hybrid assist card missing"): return
    race.pressed.emit()
    if not check(bool(garage.get_validation().get("ok", false)), "hybrid assist installation invalid"): return
    # No option can be edited: every root category must stay hidden, including with Show all.
    var fake := EmptyGarage.new()
    ui.garage = fake
    for show_all in [false, true]:
        main.get_shell().set_setting("garage.show_all_upgrades", show_all)
        ui._garage_path.clear()
        ui._draw_garage_options()
        for category in ui.UPGRADE_CATEGORIES:
            if not check(ui.get_button("tile:" + str(category)) == null, "empty category visible " + str(category)): return
    ui.garage = garage
    fake.free()
    print("ASSIST_UI PASS checks=%d" % checks)
    main._close_garage()
    main._quit_game()

