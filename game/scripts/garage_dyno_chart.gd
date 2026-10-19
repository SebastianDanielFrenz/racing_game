extends Control
# Native engine output only. Old complete points remain during recalculation.
var points: Array = []
var provisional: Array = []
var revision := -1
var vehicle_id := ""
var busy := false
var progress := 0
var error := ""
# Electrified cars (rg::DynoResult): what the curve means and the shares drawn on the kW axis.
var topology := "combustion"
var axis := "engine_rpm"
var reference_gear := 0
var motor_labels: Array = []
var unsupported_reason := ""
const SHARE_ENGINE := Color("d9d9d9")
const SHARE_ELECTRIC := [Color("7fd47a"), Color("3fa65a"), Color("b5e8a0"), Color("2b7a43")]
const SHARE_STACK := Color("c58cf0")

func _ready() -> void:
	custom_minimum_size = Vector2(0,180)
	mouse_filter = Control.MOUSE_FILTER_IGNORE
	resized.connect(queue_redraw)

func update_result(result: Dictionary) -> void:
	var next_vehicle := str(result.get("vehicle_id",""))
	if next_vehicle != vehicle_id:
		vehicle_id = next_vehicle
		points = []
		revision = -1
	var next_revision := int(result.get("revision",0))
	if next_revision != revision and not result.get("points",[]).is_empty():
		points = result["points"].duplicate(true)
		revision = next_revision
	provisional = result.get("provisional_points",[]).duplicate(true)
	busy = bool(result.get("busy",false))
	progress = int(result.get("progress",0))
	error = str(result.get("error",""))
	topology = str(result.get("topology","combustion"))
	axis = str(result.get("axis","engine_rpm"))
	reference_gear = int(result.get("reference_gear",0))
	motor_labels = result.get("motor_labels",[]).duplicate()
	unsupported_reason = str(result.get("unsupported_reason",""))
	if error.begins_with("No combustion engine") or not unsupported_reason.is_empty():
		points = []
	queue_redraw()

# The share curves (label, colour, kW per point) of an electrified result; [] for a combustion car.
func _shares(displayed: Array) -> Array:
	if topology == "combustion" or displayed.is_empty():
		return []
	var out: Array = []
	if topology == "hybrid":
		out.append(["Engine", SHARE_ENGINE, displayed.map(func(p): return float(p.get("engine_power_kw",0.0)))])
	var per_motor: bool = topology != "hybrid" and motor_labels.size() > 1
	if per_motor:
		for i in motor_labels.size():
			out.append([_motor_name(str(motor_labels[i])), SHARE_ELECTRIC[i % SHARE_ELECTRIC.size()],
				displayed.map(func(p): return float(p["motors"][i]["power_kw"]) if p.get("motors",[]).size() > i else 0.0)])
	else:
		out.append(["Electric", SHARE_ELECTRIC[0], displayed.map(func(p): return float(p.get("electric_power_kw",0.0)))])
	if topology == "fuel_cell_electric" or displayed.any(func(p): return float(p.get("fuel_cell_power_kw",0.0)) > 0.5):
		out.append(["Fuel cell", SHARE_STACK, displayed.map(func(p): return float(p.get("fuel_cell_power_kw",0.0)))])
	return out

func _motor_name(id: String) -> String:
	return id.replace("_"," ").capitalize()

func _ordinal(n: int) -> String:
	var suffix := "th"
	if n % 100 < 11 or n % 100 > 13:
		suffix = ["th","st","nd","rd","th","th","th","th","th","th"][n % 10]
	return "%d%s" % [n, suffix]

func _draw() -> void:
	var font := ThemeDB.fallback_font
	var torque_color := Color("ef973e")
	var power_color := Color("66c5ef")
	var displayed: Array = provisional if not provisional.is_empty() else points
	var shares := _shares(displayed)
	# Legend (electrified only): system torque / power, then each share (on the kW axis), wrapped below the rpm label.
	var legend: Array = []
	if not shares.is_empty():
		var entries: Array = [["Torque", torque_color], ["System", power_color]]
		for share in shares:
			entries.append([share[0], share[1]])
		var lx := 0.0
		var row := 0
		for entry in entries:
			var w: float = 22.0+font.get_string_size(entry[0],HORIZONTAL_ALIGNMENT_LEFT,-1,10).x
			if lx > 0.0 and lx+w > size.x:
				lx = 0.0
				row += 1
			legend.append([entry[0], entry[1], lx, row])
			lx += w
	var legend_rows := 0 if legend.is_empty() else int(legend[-1][3])+1
	custom_minimum_size.y = 180+14*legend_rows
	var plot := Rect2(34,28,maxf(40,size.x-68),maxf(40,size.y-75-14*legend_rows))
	var max_rpm := 1000.0
	var max_torque := 1.0
	var max_power := 1.0
	var min_torque := 0.0
	var min_power := 0.0
	for share in shares:
		for value in share[2]:
			max_power = maxf(max_power,float(value))
	for point in displayed:
		max_rpm = maxf(max_rpm,float(point["rpm"]))
		max_torque = maxf(max_torque,float(point["torque_nm"]))
		max_power = maxf(max_power,float(point["power_kw"]))
		min_torque = minf(min_torque,float(point["torque_nm"]))
		min_power = minf(min_power,float(point["power_kw"]))
	var lower := minf(min_torque/max_torque,min_power/max_power)
	var span := 1.0-lower
	draw_string(font,Vector2(0,16),"Nm",HORIZONTAL_ALIGNMENT_LEFT,-1,12,torque_color)
	draw_string(font,Vector2(size.x-25,16),"kW",HORIZONTAL_ALIGNMENT_LEFT,-1,12,power_color)
	for tick in range(5):
		var fraction := float(tick)/4.0
		var y := plot.end.y-fraction*plot.size.y
		draw_line(Vector2(plot.position.x,y),Vector2(plot.end.x,y),Color(0.3,0.33,0.38,0.45))
		if not displayed.is_empty():
			draw_string(font,Vector2(0,y+4),"%.0f" % (max_torque*(lower+span*fraction)),HORIZONTAL_ALIGNMENT_LEFT,-1,10,torque_color)
			draw_string(font,Vector2(plot.end.x+4,y+4),"%.0f" % (max_power*(lower+span*fraction)),HORIZONTAL_ALIGNMENT_LEFT,-1,10,power_color)
		var x := plot.position.x+fraction*plot.size.x
		draw_string(font,Vector2(x-13,plot.end.y+15),"%.0f" % (max_rpm*fraction),HORIZONTAL_ALIGNMENT_LEFT,-1,10,Color(0.7,0.73,0.78))
	var axis_label := "RPM"
	if topology != "combustion":
		if axis == "motor_rpm":
			axis_label = "%s RPM" % (_motor_name(str(motor_labels[0])) if not motor_labels.is_empty() else "Motor")
		elif reference_gear > 0:
			axis_label = "Engine RPM in %s" % _ordinal(reference_gear)
	draw_string(font,Vector2(plot.end.x-font.get_string_size(axis_label,HORIZONTAL_ALIGNMENT_LEFT,-1,10).x,plot.end.y+29),axis_label,HORIZONTAL_ALIGNMENT_LEFT,-1,10,Color(0.7,0.73,0.78))
	for entry in legend:
		var ly: float = plot.end.y+40+14*float(entry[3])
		draw_line(Vector2(entry[2],ly-4),Vector2(entry[2]+12,ly-4),entry[1],2)
		draw_string(font,Vector2(entry[2]+15,ly),entry[0],HORIZONTAL_ALIGNMENT_LEFT,-1,10,Color(0.82,0.84,0.88))
	if lower < 0.0:
		var zero_y := plot.end.y+lower/span*plot.size.y
		draw_line(Vector2(plot.position.x,zero_y),Vector2(plot.end.x,zero_y),Color(0.65,0.68,0.72,0.7))
	if not unsupported_reason.is_empty():
		draw_rect(plot,Color(0.2,0.21,0.24,0.55))
		draw_multiline_string(font,Vector2(plot.position.x+8,plot.position.y+20),unsupported_reason,HORIZONTAL_ALIGNMENT_LEFT,plot.size.x-16,11,-1,Color(0.62,0.64,0.68))
	elif displayed.is_empty():
		draw_string(font,Vector2(plot.position.x+8,plot.get_center().y),"Computing dyno…" if busy else "No dyno data",HORIZONTAL_ALIGNMENT_LEFT,-1,12,Color(0.7,0.73,0.78))
	else:
		var torque_line := PackedVector2Array()
		var power_line := PackedVector2Array()
		for point in displayed:
			var x := plot.position.x+float(point["rpm"])/max_rpm*plot.size.x
			torque_line.append(Vector2(x,plot.end.y-(float(point["torque_nm"])/max_torque-lower)/span*plot.size.y))
			power_line.append(Vector2(x,plot.end.y-(float(point["power_kw"])/max_power-lower)/span*plot.size.y))
		if displayed.size() > 1:
			for share in shares:
				var line := PackedVector2Array()
				for i in displayed.size():
					var x := plot.position.x+float(displayed[i]["rpm"])/max_rpm*plot.size.x
					line.append(Vector2(x,plot.end.y-(float(share[2][i])/max_power-lower)/span*plot.size.y))
				draw_polyline(line,share[1],1.5,true)
			draw_polyline(torque_line,torque_color,2,true)
			draw_polyline(power_line,power_color,2,true)
