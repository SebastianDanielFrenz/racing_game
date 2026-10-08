extends Control
# Native engine output only. Old complete points remain during recalculation.
var points: Array = []
var revision := -1
var vehicle_id := ""
var busy := false
var progress := 0
var error := ""

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
	busy = bool(result.get("busy",false))
	progress = int(result.get("progress",0))
	error = str(result.get("error",""))
	queue_redraw()

func _draw() -> void:
	var font := ThemeDB.fallback_font
	var torque_color := Color("ef973e")
	var power_color := Color("66c5ef")
	var plot := Rect2(34,28,maxf(40,size.x-68),maxf(40,size.y-75))
	var max_rpm := 1000.0
	var max_torque := 1.0
	var max_power := 1.0
	for point in points:
		max_rpm = maxf(max_rpm,float(point["rpm"]))
		max_torque = maxf(max_torque,float(point["torque_nm"]))
		max_power = maxf(max_power,float(point["power_kw"]))
	draw_string(font,Vector2(0,16),"Nm",HORIZONTAL_ALIGNMENT_LEFT,-1,12,torque_color)
	draw_string(font,Vector2(size.x-25,16),"kW",HORIZONTAL_ALIGNMENT_LEFT,-1,12,power_color)
	for tick in range(5):
		var fraction := float(tick)/4.0
		var y := plot.end.y-fraction*plot.size.y
		draw_line(Vector2(plot.position.x,y),Vector2(plot.end.x,y),Color(0.3,0.33,0.38,0.45))
		if not points.is_empty():
			draw_string(font,Vector2(0,y+4),"%.0f" % (max_torque*fraction),HORIZONTAL_ALIGNMENT_LEFT,-1,10,torque_color)
			draw_string(font,Vector2(plot.end.x+4,y+4),"%.0f" % (max_power*fraction),HORIZONTAL_ALIGNMENT_LEFT,-1,10,power_color)
		var x := plot.position.x+fraction*plot.size.x
		draw_string(font,Vector2(x-13,plot.end.y+15),"%.0f" % (max_rpm*fraction),HORIZONTAL_ALIGNMENT_LEFT,-1,10,Color(0.7,0.73,0.78))
	draw_string(font,Vector2(plot.end.x-22,plot.end.y+29),"RPM",HORIZONTAL_ALIGNMENT_LEFT,-1,10,Color(0.7,0.73,0.78))
	if points.is_empty():
		draw_string(font,Vector2(plot.position.x+8,plot.get_center().y),"Computing dyno…" if busy else "No dyno data",HORIZONTAL_ALIGNMENT_LEFT,-1,12,Color(0.7,0.73,0.78))
	else:
		var torque_line := PackedVector2Array()
		var power_line := PackedVector2Array()
		for point in points:
			var x := plot.position.x+float(point["rpm"])/max_rpm*plot.size.x
			torque_line.append(Vector2(x,plot.end.y-clampf(float(point["torque_nm"])/max_torque,0,1)*plot.size.y))
			power_line.append(Vector2(x,plot.end.y-clampf(float(point["power_kw"])/max_power,0,1)*plot.size.y))
		if points.size() > 1:
			draw_polyline(torque_line,torque_color,2,true)
			draw_polyline(power_line,power_color,2,true)
