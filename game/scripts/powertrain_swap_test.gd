extends SceneTree
var failures := 0
func check(ok: bool, label: String) -> void:
 print("RG_SWAP_TEST ",label," ",ok)
 if not ok: failures+=1
func _initialize() -> void: run.call_deferred()
func run() -> void:
 var repo := ProjectSettings.globalize_path("res://..").simplify_path()
 var work := repo.path_join("out/swap_acceptance_"+str(Time.get_ticks_usec()))
 DirAccess.make_dir_recursive_absolute(work)
 var garage = ClassDB.instantiate("RgGarage")
 root.add_child(garage)
 check(garage.initialize(repo,work,work.path_join("work"),"").get("ok",false),"initialise")
 check(garage.begin_edit("car_electric").get("ok",false),"edit EV")
 var ui = load("res://scripts/shell_ui.gd").new()
 ui.garage=garage
 root.add_child(ui)
 var box := VBoxContainer.new()
 root.add_child(box)
 ui._option_box=box
 ui._garage_path.assign(["Swaps","Powertrain"])
 ui._draw_upgrades()
 for category in ["Internal combustion","EV","Hybrid"]: check(ui.get_button("tile:"+category)!=null,category+" category")
 ui._garage_path.assign(["Swaps","Powertrain","Internal combustion"])
 ui._draw_garage_options()
 for fuel in ["Gasoline","Diesel","Hydrogen"]: check(ui.get_button("tile:"+fuel)!=null,fuel+" category")
 ui._garage_path.assign(["Swaps","Powertrain","EV"])
 ui._draw_garage_options()
 check(ui.get_button("part:powertrain_install:fuel_cell_ev")!=null,"fuel cell choice")
 ui._garage_path.assign(["Swaps","Powertrain","Hybrid"])
 ui._draw_garage_options()
 var hybrid: Button=ui.get_button("part:powertrain_install:hybrid_dual_hydrogen")
 check(hybrid!=null,"dual hydrogen choice")
 if hybrid: hybrid.pressed.emit()
 check(garage.prepare_drive("car_electric").get("ok",false),"hybrid from EV materialises")
 for car in ["car_sedan","car_electric","car_hyper_n2o"]:
  for kind in ["ice","battery_ev","fuel_cell_ev","hybrid_battery","hybrid_dual_hydrogen","ice"]:
   check(garage.begin_edit(car).get("ok",false),"edit "+car)
   var changed: Dictionary=garage.set_option("powertrain_install",kind)
   check(changed.get("accepted",false),car+" accepts "+kind+" "+str(changed))
   check(garage.prepare_drive(car).get("ok",false),car+" materialises "+kind)
   check(garage.save().get("ok",false),car+" saves "+kind)
   var reloaded = ClassDB.instantiate("RgGarage")
   root.add_child(reloaded)
   check(reloaded.initialize(repo,work,work.path_join("reload_work"),"").get("ok",false),"reload initialise")
   check(reloaded.begin_edit(car).get("ok",false),"reload "+car)
   var restored := ""
   for option in reloaded.get_options():
    if str(option.get("id",""))=="powertrain_install":restored=str(option.get("value",""))
   check(restored==kind,car+" restores "+kind+" got "+restored)
   check(reloaded.prepare_drive(car).get("ok",false),car+" saved conversion materialises")
   reloaded.queue_free()
   await process_frame
 ui.queue_free()
 garage.queue_free()
 await process_frame
 print("RG_SWAP_TEST failures=",failures)
 quit(1 if failures else 0)
