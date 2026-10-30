extends SceneTree
# Headless acceptance: shared induction types, hardware filtering, legacy saves and factory restoration.
var failures := 0
func check(ok: bool, label: String) -> void:
 print("RG_TURBO_CATALOGUE_TEST ",label," ",ok)
 if not ok: failures += 1
func _initialize() -> void: run.call_deferred()
func run() -> void:
 var repo := ProjectSettings.globalize_path("res://..").simplify_path()
 var work := repo.path_join("out/turbo_catalogue_acceptance_"+str(Time.get_ticks_usec()))
 DirAccess.make_dir_recursive_absolute(work)
 var garage = ClassDB.instantiate("RgGarage")
 root.add_child(garage)
 check(garage.initialize(repo,work,work.path_join("work"),"").get("ok",false),"initialise")
 var ui = load("res://scripts/shell_ui.gd").new()
 ui.garage = garage
 root.add_child(ui)
 var box := VBoxContainer.new()
 root.add_child(box)
 ui._option_box = box
 ui._garage_mode = "upgrades"
 for car in ["car_hyper","car_hyper_n2o","car_city"]:
  check(garage.begin_edit(car).get("ok",false),"edit "+car)
  ui._garage_path.assign(["Swaps","Forced induction"])
  ui._draw_garage_options()
  for kind in ["NA","Single turbo","Twin turbo","Quad turbo"]:
   check(ui.get_button("induction:"+kind)!=null,car+" offers "+kind)
  var quad: Button = ui.get_button("induction:Quad turbo")
  if quad: quad.pressed.emit()
  ui._garage_path.assign(["Engine","Turbo"])
  ui._draw_garage_options()
  var hardware: Button = ui.get_button("part:turbo_kit:quad")
  check(hardware!=null,car+" quad hardware")
  check(ui.get_button("part:turbo_kit:sport_single")==null,car+" filters single hardware")
  check(garage.prepare_drive(car).get("ok",false),car+" quad materialises")
  check(garage.save().get("ok",false),car+" saves")
  check(garage.set_option("turbo_kit","stock").get("accepted",false),car+" restores factory")
  check(garage.prepare_drive(car).get("ok",false),car+" factory materialises")
 check(garage.begin_edit("car_hyper").get("ok",false),"edit legacy hypercar")
 check(garage.set_option("turbo_install","hyper_single_large").get("accepted",false),"legacy hardware")
 ui._garage_path.assign(["Engine","Turbo"])
 ui._draw_garage_options()
 check(ui.get_button("part:turbo_kit:legacy:turbo_install:hyper_single_large")!=null,"legacy installed hardware visible")
 check(garage.set_option("turbo_kit","stock").get("accepted",false),"factory clears legacy")
 ui._draw_garage_options()
 check(ui.get_button("part:turbo_kit:stock")!=null,"factory twin visible again")
 ui.queue_free()
 garage.queue_free()
 await process_frame
 print("RG_TURBO_CATALOGUE_TEST failures=",failures)
 quit(1 if failures else 0)
