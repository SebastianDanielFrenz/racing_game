extends Control
const UiTheme := preload("res://scripts/ui/ui_theme.gd")
const PowertrainFeed := preload("res://scripts/powertrain_feed.gd")
var feed := PowertrainFeed.new("cluster")
var simulation: Node
var vehicle_name: String
var speed := 0.0
var rpm := 0.0
var drive_kw := 0.0
var boost_bar := -1.0
var gear := 0
var park_engaged := false
var selector_label := "" # selector cars: P R N D S M<n> plus the running gear (rg::selector_display), "" on a manual
var cabin_lamps: Dictionary = {}
var capacity := 0.0
var density := 0.0
var fuel := 0.0
var initial_fuel := -1.0
var distance := 0.0
var flow_lph := 0.0
var elapsed := 0.0
var tc_fitted := false
var tc_on := false
var tc_intervening := false
var tc_mode_name := "Off"
var aero_fitted := false
var aero_active := false
var aero_target_deg := 0.0
var font := ThemeDB.fallback_font
var thermal: Dictionary = {}
const ThermalReadout := preload("res://scripts/thermal_readout.gd")
# Drive-mode theme colours (ECO blue / NORMAL gray / SPORT orange); a car without drive modes keeps the default look.
const DriveModeTheme := preload("res://scripts/drive_mode_theme.gd")
const DriverState := preload("res://scripts/driver_state.gd")
var drive_mode_label := ""
var launch_lamp := 0
var launch_lamp_text := ""
var regen_fitted := false
var regen_active := false
var regen_label := ""
const HybridDial := preload("res://scripts/hybrid_dial.gd")
const GaugeLogic := preload("res://scripts/gauge_logic.gd")
var tach_loaded := false
var tach_kind := "ice"       # core (rg::tach_scales): "ice" one dial, "ev" one motor dial, "hybrid" ICE dial + inner motor scale
var limiter_rpm := 0.0
var motor_dial_max := 0.0
var motor_dial_tick := 0.0
var motor_rpm := 0.0         # traction motor (hybrid inner needle)
var pt_now: Dictionary = {}

func _ready() -> void:
	UiTheme.follow(self)
	var info: Dictionary = simulation.get_vehicle_gauge_info(vehicle_name)
	tc_fitted = "TC" in info.get("fitted_assists", [])
	capacity = info.get("fuel_capacity_kg",0.0)
	density = info.get("fuel_density_kg_m3",0.0)

const FrameProbe := preload("res://scripts/frame_probe.gd")
func _process(delta: float) -> void:
	var probe_t0 := Time.get_ticks_usec()
	_process_body(delta)
	FrameProbe.add("cluster", Time.get_ticks_usec() - probe_t0)
func _process_body(delta: float) -> void:
	if not simulation.is_running() or simulation.get_step_count() <= 0:
		return
	if capacity <= 0.0 or not tach_loaded:
		var info: Dictionary = simulation.get_vehicle_gauge_info(vehicle_name)
		capacity = float(info.get("fuel_capacity_kg",0.0))
		if not info.is_empty():
			tach_loaded = true
			tach_kind = str(info.get("tach_kind","ice"))
			limiter_rpm = float(info.get("limiter_rpm",0.0))
			motor_dial_max = float(info.get("motor_dial_max_rpm",0.0))
			motor_dial_tick = float(info.get("motor_dial_tick_rpm",0.0))
		density = float(info.get("fuel_density_kg_m3",0.0))
	if simulation.get_streaming_status().get("frozen",false):
		delta = 0.0
	var ground_speed: float = simulation.get_vehicle_ground_speed_mps(vehicle_name)
	distance += maxf(ground_speed,0.0)*delta
	elapsed += delta
	if elapsed < 0.05:
		return
	var dt := elapsed
	elapsed = 0.0
	var misses_before: int = feed.misses
	var pt: Dictionary = feed.read(simulation,vehicle_name)
	var controls: Dictionary = DriverState.read(simulation,pt)
	cabin_lamps = controls
	thermal = simulation.get_vehicle_thermal(vehicle_name)
	tc_fitted = controls["tc"]["fitted"]
	tc_on = controls["tc"]["active"]
	tc_intervening = controls["tc"]["blink"]
	tc_mode_name = controls["tc"]["label"]
	aero_fitted = controls["aero"]["fitted"]
	aero_active = controls["aero"]["value"] != 0
	aero_target_deg = controls["aero"]["aero_target_deg"]
	drive_mode_label = controls["dm"]["label"]
	launch_lamp = controls["lc"]["lamp_state"]
	launch_lamp_text = controls["lc"]["label"]
	if feed.misses == misses_before: speed = ground_speed*3.6 # an empty reading keeps the last speed too
	rpm = pt.get("rpm",0.0)
	motor_rpm = float(pt.get("traction_motor_rpm",0.0))
	pt_now = pt
	drive_kw = pt.get("drive_power_kw",0.0)
	regen_fitted = bool(pt.get("regen_fitted",false))
	regen_active = bool(pt.get("regen_active",false))
	regen_label = str(pt.get("regen_label",""))
	boost_bar = pt.get("boost_bar",-1.0)
	gear = int(pt.get("gear",0))
	park_engaged = controls["park"]["active"]
	selector_label = str(controls["transmission"]["label"]) if controls.has("transmission") and controls["transmission"]["fitted"] else ""
	fuel = pt.get("fuel_mass_kg",0.0)
	if capacity>0.0 and (initial_fuel<0.0 or fuel>initial_fuel+0.01):
		initial_fuel = fuel
		distance = 0.0
	if density>0.0:
		var flow: float = pt.get("fuel_flow_g_s",0.0)*3600.0/density
		flow_lph = lerpf(flow_lph,flow,1.0-exp(-dt/0.5))
	queue_redraw()

func text(value: String, at: Vector2, size: int, color := UiTheme.role(&"text")) -> void:
	at.x -= font.get_string_size(value,HORIZONTAL_ALIGNMENT_LEFT,-1,size).x/2.0
	draw_string(font,at,value,HORIZONTAL_ALIGNMENT_LEFT,-1,size,color)

func dial(center: Vector2,value: float,maximum: float,color: Color,is_rpm: bool,label_step := 0.0,redline_from := -1.0) -> void:
	var a := deg_to_rad(135.0)
	var b := deg_to_rad(405.0)
	draw_arc(center,125,a,b,100,UiTheme.role(&"cluster_track"),12,true)
	draw_arc(center,125,a,a+(b-a)*clampf(value/maximum,0,1),100,color,12,true)
	if is_rpm:
		# Default (an ICE car): the red zone is the last ninth. Hybrid: from the ICE limiter; EV: none (redline_from 0).
		var red := 8.0/9.0 if redline_from < 0.0 else (redline_from/maximum if redline_from > 0.0 else 2.0)
		if red < 1.0:
			draw_arc(center,125,a+(b-a)*red,b,20,UiTheme.role(&"gauge_redline"),12,true)
	var count := 10
	if is_rpm and label_step > 0.0:
		count = int(round(maximum/label_step))+1
	for k in range(count):
		var angle := a+(b-a)*k/float(count-1)
		var direction := Vector2(cos(angle),sin(angle))
		draw_line(center+direction*105,center+direction*114,UiTheme.role(&"cluster_tick"),2,true)
		var tick_label := str(k*40)
		if is_rpm:
			tick_label = str(k) if label_step <= 0.0 else str(int(round(k*label_step/1000.0)))
		text(tick_label,center+direction*86+Vector2(0,6),18,UiTheme.role(&"cluster_tick"))

# The hybrid's inner scale (owner sketch out/owner_hybrid_tach_sketch.png): the traction motor in x1000 rpm, ticks and numbers
# inside the ICE numbers, a short needle. Max and tick spacing come from core (rg::tach_scales).
func hybrid_inner(center: Vector2) -> void:
	if motor_dial_max <= 0.0:
		return
	var a := deg_to_rad(135.0)
	var b := deg_to_rad(405.0)
	for tick_rpm: float in HybridDial.inner_ticks(motor_dial_max,motor_dial_tick):
		var angle := a+(b-a)*tick_rpm/motor_dial_max
		var direction := Vector2(cos(angle),sin(angle))
		draw_line(center+direction*70,center+direction*76,UiTheme.role(&"gauge_motor"),1.5,true)
		text(str(int(round(tick_rpm/1000.0))),center+direction*58+Vector2(0,4),11,UiTheme.role(&"gauge_motor"))
	var needle_angle := a+(b-a)*clampf(motor_rpm/motor_dial_max,0.0,1.0)
	var needle := Vector2(cos(needle_angle),sin(needle_angle))
	draw_line(center+needle*14,center+needle*68,UiTheme.role(&"gauge_motor"),3,true)
	draw_circle(center,5,UiTheme.role(&"gauge_motor"))

func small_dial(center: Vector2, value: String, title: String, fraction: float, fill := UiTheme.role(&"cluster_small_dial")) -> void:
	draw_arc(center,45,deg_to_rad(135),deg_to_rad(405),48,UiTheme.role(&"cluster_track"),5,true)
	if fraction>=0:
		draw_arc(center,45,deg_to_rad(135),deg_to_rad(135)+deg_to_rad(270)*clampf(fraction,0,1),48,fill,5,true)
	text(value,center+Vector2(0,3),28)
	text(title,center+Vector2(0,25),15,UiTheme.role(&"cluster_label"))

# _draw runs in the deferred canvas redraw after every _process (not inside the _process probe): timed on its own.
func _draw() -> void:
	var probe_t0 := Time.get_ticks_usec()
	_draw_body()
	FrameProbe.add("cluster_draw", Time.get_ticks_usec() - probe_t0)

func _draw_body() -> void:
	draw_rect(Rect2(0,0,1024,400),UiTheme.role(&"surface_cluster"))
	var blue := UiTheme.role(&"cluster_label")
	var drive_mode_accent := DriveModeTheme.accent_state(cabin_lamps.get("dm",{}),blue)
	var has_fuel := capacity>0 and density>0
	var fraction := clampf(fuel/capacity,0,1) if has_fuel else 0.0
	text("SCULPTED A",Vector2(503,29),18,blue)
	# The sketch: vertical fuel strip, two dominant dials and stacked auxiliaries.
	text("FUEL",Vector2(48,59),18,blue)
	draw_rect(Rect2(38,78,20,180),UiTheme.role(&"cluster_track"))
	draw_rect(Rect2(38,78+180*(1-fraction),20,180*fraction),UiTheme.role(&"warning") if fraction<0.15 else UiTheme.role(&"cluster_fuel"))
	text("F",Vector2(75,93),16,blue)
	text("E",Vector2(75,259),16,blue)
	text("%.0f%%" % (fraction*100) if has_fuel else "--",Vector2(48,286),20)
	# A drive-modes car (assists I7) draws its dials in the colour of the mode that runs; any other car keeps the defaults.
	var themed := drive_mode_label != ""
	dial(Vector2(245,170),speed,360,drive_mode_accent if themed else UiTheme.role(&"cluster_speed"),false)
	var rpm_color := drive_mode_accent if themed else UiTheme.role(&"cluster_rpm")
	if tach_kind == "hybrid":
		# ICE dial scaled to the ICE limiter (outer, main needle) plus the traction motor on an inner scale (core: rg::tach_scales).
		var ice_max := GaugeLogic.max_rpm_for_limiter(limiter_rpm)
		dial(Vector2(735,170),rpm,ice_max,rpm_color,true,1000.0,limiter_rpm)
		hybrid_inner(Vector2(735,170))
	elif tach_kind == "ev":
		dial(Vector2(735,170),rpm,motor_dial_max,rpm_color,true,motor_dial_tick,0.0)
	else:
		dial(Vector2(735,170),rpm,9000,rpm_color,true)
	text("%.0f" % speed,Vector2(245,180),68)
	text("km/h",Vector2(245,214),22,blue)
	text("%.0f" % rpm,Vector2(735,180) if tach_kind != "hybrid" else Vector2(735,206),58 if tach_kind != "hybrid" else 34)
	text("RPM" if tach_kind != "ev" else "MOTOR RPM",Vector2(735,214) if tach_kind != "hybrid" else Vector2(735,230),22 if tach_kind != "hybrid" else 16,blue)
	if themed:
		# Drive mode, small, under the tach dial (between its lower end and the information strip).
		text(drive_mode_label,Vector2(735,299),22,drive_mode_accent)
	if selector_label != "":
		# Selector cars (automatic, dual-clutch, hybrid): the label is rg::selector_display, the running gear after D/S included.
		var shown := selector_label
		text(shown,Vector2(490,174),90 if shown.length() < 2 else 70)
	else:
		text("P" if park_engaged else ("R" if gear<0 else ("N" if gear==0 else str(gear))),Vector2(490,174),90)
	text("GEAR",Vector2(490,207),18,blue)
	if not cabin_lamps.is_empty():
		text("<",Vector2(330,32),28,UiTheme.role(&"good") if cabin_lamps["left"]["active"] else UiTheme.role(&"cluster_blinker_off"))
		text(">",Vector2(650,32),28,UiTheme.role(&"good") if cabin_lamps["right"]["active"] else UiTheme.role(&"cluster_blinker_off"))
		if cabin_lamps["epb"]["label"] != "":
			text(cabin_lamps["epb"]["label"],Vector2(490,330),20,UiTheme.role(&"warning"))
	if tc_fitted:
		# The mode that actually runs (physics_sim's assist unit: Off / On / Relaxed / Sport / Drift).
		var tc_label := "TC OFF" if not tc_on else "TC " + tc_mode_name.to_upper()
		if tc_intervening:
			tc_label += " ACTIVE"
		var tc_color := UiTheme.role(&"warning") if tc_intervening or not tc_on else UiTheme.role(&"gauge_fill")
		text(tc_label,Vector2(490,266),20,tc_color)
	if not cabin_lamps.is_empty():
		# ABS / ESC (assist unit): green = on, amber = off or intervening (blink state comes from the shared ControlState).
		var amber := UiTheme.role(&"warning")
		var green := UiTheme.role(&"gauge_fill")
		if cabin_lamps["abs"]["fitted"]:
			text(str(cabin_lamps["abs"]["label"]),Vector2(330,330),18,green if cabin_lamps["abs"]["colour_role"] == "ok" else amber)
		if cabin_lamps["esc"]["fitted"]:
			text(str(cabin_lamps["esc"]["label"]),Vector2(330,354),18,green if cabin_lamps["esc"]["colour_role"] == "ok" else amber)
	if launch_lamp > 0:
		# Launch control (assists I4): gray = off, green = on, blinking amber = armed, solid amber = launching.
		var lc_color := UiTheme.role(&"cluster_lamp_off")
		if launch_lamp == 2:
			lc_color = UiTheme.role(&"gauge_fill")
		elif launch_lamp == 3:
			lc_color = UiTheme.role(&"warning") if fmod(Time.get_ticks_msec()/1000.0,0.4)<0.2 else UiTheme.role(&"cluster_lamp_off")
		elif launch_lamp >= 4:
			lc_color = UiTheme.role(&"warning")
		text(launch_lamp_text,Vector2(490,240),18,lc_color)
	if aero_fitted:
		var aero_label := "AERO AUTO %+.0f DEG" % aero_target_deg if aero_active else "AERO OFF"
		text(aero_label,Vector2(490,292),16,UiTheme.role(&"gauge_fill") if aero_active else UiTheme.role(&"warning"))
	# Engine temperature, above the gear: only for a car with a thermal model (the hypercar has none yet), else nothing is drawn.
	ThermalReadout.draw_bar(self,font,Vector2(490,66),1.0,thermal,fmod(Time.get_ticks_msec()/1000.0,0.5)<0.25)
	var small_fill := drive_mode_accent if themed else UiTheme.role(&"cluster_small_dial")
	small_dial(Vector2(935,109),"%.0f" % drive_kw,"DRIVE kW",maxf(drive_kw,0)/650.0,small_fill)
	small_dial(Vector2(935,241),"%.1f" % boost_bar if boost_bar>=0 else "--","BOOST bar",boost_bar/3.0 if boost_bar>=0 else -1,small_fill)
	# Traction battery (rg::battery_hud): SoC in percent and a small bar under the speed dial; amber/red below the configured thresholds.
	HybridDial.draw_battery(self,font,245,268,pt_now,18,Vector2(120,7))
	if regen_fitted:
		# Brake regeneration (electric cars): power into the pack and the motors' share of the braking.
		text(regen_label,Vector2(245,299),20,UiTheme.role(&"gauge_fill") if regen_active else UiTheme.role(&"cluster_label"))
	# Dynamic information strip, reserved for selectable trip pages later.
	draw_rect(Rect2(102,311,903,78),UiTheme.role(&"surface_cluster_strip"))
	text("TRIP",Vector2(181,338),16,blue)
	text("%.1f km" % (distance/1000),Vector2(181,369),24)
	var instant := "%.1f L/100km" % (flow_lph*100/speed) if speed>=5 else "%.1f L/h" % flow_lph
	text("INSTANT",Vector2(362,338),16,blue)
	text(instant if has_fuel else "--",Vector2(362,369),24)
	var used := maxf(initial_fuel-fuel,0)*1000/density if has_fuel else 0.0
	var average := used*100000/distance if distance>=100 else 0.0
	text("AVERAGE",Vector2(562,338),16,blue)
	text("%.1f L/100km" % average if average>0 else "--",Vector2(562,369),24)
	text("RANGE",Vector2(753,338),16,blue)
	text("%.0f km" % (fuel*1000/density*100/average) if average>0 else "--",Vector2(753,369),24)
	text("REMAINING",Vector2(922,338),16,blue)
	text("%.1f L" % (fuel*1000/density) if has_fuel else "--",Vector2(922,369),24)
