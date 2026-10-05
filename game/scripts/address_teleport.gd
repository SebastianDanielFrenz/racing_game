extends CanvasLayer
# Manual address searches only; no request on each keystroke.
var main: Node
var _panel: PanelContainer
var _query: LineEdit
var _results: ItemList
var _status: Label
var _teleport: Button
var _http: HTTPRequest
var _matches: Array = []
var _cache := ConfigFile.new()
var _config: Dictionary = {}
var _world: Dictionary = {}
var _pending_query := ""
var _last_request_ms := -2000
var _mouse_before := Input.MOUSE_MODE_VISIBLE

func _ready() -> void:
 layer = 40
 _cache.load("user://address_search_cache.cfg")
 var parsed = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/geocoding.json")))
 if parsed is Dictionary:
  _config = parsed
 _http = HTTPRequest.new()
 _http.use_threads = true
 _http.timeout = 15.0
 _http.body_size_limit = 2097152
 add_child(_http)
 _http.request_completed.connect(_request_complete)
 _panel = PanelContainer.new()
 _panel.custom_minimum_size = Vector2(660, 0)
 add_child(_panel)
 var box := VBoxContainer.new()
 box.add_theme_constant_override("separation", 12)
 _panel.add_child(box)
 var title := Label.new()
 title.text = "Teleport to address (F10)"
 box.add_child(title)
 _query = LineEdit.new()
 _query.placeholder_text = "Street, house number, town — or latitude, longitude"
 box.add_child(_query)
 _query.text_submitted.connect(func(_text: String): _search())
 var search := Button.new()
 search.text = "Search"
 search.pressed.connect(_search)
 box.add_child(search)
 _results = ItemList.new()
 _results.custom_minimum_size = Vector2(640, 190)
 _results.item_selected.connect(func(_index: int): _teleport.disabled = false)
 _results.item_activated.connect(func(_index: int): _jump())
 box.add_child(_results)
 _status = Label.new()
 _status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
 box.add_child(_status)
 var attribution := Label.new()
 attribution.text = "Address search: Photon / © OpenStreetMap contributors"
 box.add_child(attribution)
 _teleport = Button.new()
 _teleport.text = "Teleport to selected location"
 _teleport.disabled = true
 _teleport.pressed.connect(_jump)
 box.add_child(_teleport)
 var close := Button.new()
 close.text = "Cancel"
 close.pressed.connect(func(): _toggle(false))
 box.add_child(close)
 _panel.hide()
 get_viewport().size_changed.connect(_center)

func _center() -> void:
 _panel.position = (get_viewport().get_visible_rect().size - _panel.size) * 0.5

func _input(event: InputEvent) -> void:
 if event is InputEventKey and event.pressed and not event.echo:
  if event.keycode == KEY_F10:
   _toggle(not _panel.visible)
   get_viewport().set_input_as_handled()
  elif event.keycode == KEY_ESCAPE and _panel.visible:
   _toggle(false)
   get_viewport().set_input_as_handled()

# The shell's "Search for an address" start (main.gd) opens the panel once the
# world is running.
func open_dialog() -> void:
 _toggle(true)

func _toggle(open: bool) -> void:
 if open:
  if main == null or main.world_state != "running" or main.world_kind != "real_world":
   print("RG_TELEPORT address search requires a running real-world map")
   return
  var parsed = JSON.parse_string(FileAccess.get_file_as_string(main._world_config_path()))
  if not parsed is Dictionary:
   return
  _world = parsed
  _mouse_before = Input.mouse_mode
  add_to_group("address_teleport_open")
  _panel.show()
  _status.text = "Enter an address and choose a result. Terrain must exist in the current map region."
  Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
  _query.grab_focus()
  call_deferred("_center")
 else:
  _http.cancel_request()
  _pending_query = ""
  _panel.hide()
  remove_from_group("address_teleport_open")
  if get_tree().get_nodes_in_group("seat_adjustment_open").is_empty() and get_tree().get_nodes_in_group("traffic_settings_open").is_empty():
   Input.mouse_mode = _mouse_before

func _search() -> void:
 var query := _query.text.strip_edges()
 if query.is_empty():
  _status.text = "Enter an address first."
  return
 _http.cancel_request()
 _pending_query = ""
 _results.clear()
 _matches.clear()
 _teleport.disabled = true
 var coordinates := query.split(",", false)
 if coordinates.size() == 2 and coordinates[0].strip_edges().is_valid_float() and coordinates[1].strip_edges().is_valid_float():
  _show_matches([{"label": query, "lat": float(coordinates[0]), "lon": float(coordinates[1])}])
  return
 var cache_key := query.to_lower()
 if _cache.has_section_key("search", cache_key):
  var cached = _cache.get_value("search", cache_key)
  if cached is Array:
   _show_matches(cached)
   return
 if Time.get_ticks_msec() - _last_request_ms < 1000:
  _status.text = "Please wait a second before searching again."
  return
 _last_request_ms = Time.get_ticks_msec()
 var endpoint := str(_config.get("endpoint", "https://photon.komoot.io/api/"))
 var url := endpoint + ("&" if "?" in endpoint else "?") + "q=" + query.uri_encode() + "&limit=5&lang=de"
 var spawn: Dictionary = _world.get("spawn", {})
 if spawn.has("latitude") and spawn.has("longitude"):
  url += "&lat=%s&lon=%s" % [spawn.latitude,spawn.longitude]
 _pending_query = cache_key
 _status.text = "Searching…"
 var error := _http.request(url, ["User-Agent: racing_game-address-teleport/1.0", "Accept: application/json"])
 if error != OK:
  _pending_query = ""
  _status.text = "Could not start address search. Try again."

func _request_complete(result: int, code: int, _headers: PackedStringArray, body: PackedByteArray) -> void:
 if _pending_query.is_empty() or not _panel.visible:
  return
 var query := _pending_query
 _pending_query = ""
 if result != HTTPRequest.RESULT_SUCCESS or code != 200:
  _status.text = "Address search failed (HTTP %d). Try again, or enter latitude, longitude." % code
  return
 var parsed = JSON.parse_string(body.get_string_from_utf8())
 if not parsed is Dictionary or not parsed.get("features") is Array:
  _status.text = "Address service returned an invalid response."
  return
 var matches: Array = []
 for feature in parsed.features:
  if not feature is Dictionary:
   continue
  var geometry = feature.get("geometry", {})
  if not geometry is Dictionary:
   continue
  var coordinates = geometry.get("coordinates", [])
  if not coordinates is Array or coordinates.size() < 2:
   continue
  if not (coordinates[0] is float or coordinates[0] is int) or not (coordinates[1] is float or coordinates[1] is int):
   continue
  var properties = feature.get("properties", {})
  if not properties is Dictionary:
   continue
  var parts: Array[String] = []
  var street := str(properties.get("street", properties.get("name", "")))
  var number := str(properties.get("housenumber", ""))
  if not street.is_empty():
   parts.append((street + " " + number).strip_edges())
  for key in ["postcode", "city", "state", "country"]:
   var part := str(properties.get(key, ""))
   if not part.is_empty():
    parts.append(part)
  matches.append({"label": ", ".join(parts), "lat": float(coordinates[1]), "lon": float(coordinates[0])})
 if not matches.is_empty():
  _cache.set_value("search", query, matches)
  _cache.save("user://address_search_cache.cfg")
 _show_matches(matches)

func _show_matches(matches: Array) -> void:
 _matches = matches
 _results.clear()
 for match in matches:
  _results.add_item(str(match.label))
 _status.text = "No matching address found. Include the town and house number." if matches.is_empty() else "Choose a destination, then teleport."
 if not matches.is_empty():
  _results.select(0)
 _teleport.disabled = matches.is_empty()

# WGS84 transverse Mercator, the same northern UTM convention as WorldConfig.
# Subtract the session origin before converting the result to Godot Vector2.
static func session_coordinates(latitude: float, longitude: float, origin: Dictionary) -> Vector2:
 var lat := deg_to_rad(latitude)
 var lon := deg_to_rad(longitude)
 var zone := int(origin.get("zone", 32))
 var central := deg_to_rad((zone - 1) * 6.0 - 180.0 + 3.0)
 var e2 := 0.0066943799901413165
 var ep2 := e2 / (1.0 - e2)
 var n := 6378137.0 / sqrt(1.0 - e2 * pow(sin(lat), 2))
 var t := pow(tan(lat), 2)
 var c := ep2 * pow(cos(lat), 2)
 var a := cos(lat) * (lon - central)
 var m := 6378137.0 * ((1.0-e2/4.0-3.0*e2*e2/64.0-5.0*pow(e2,3)/256.0)*lat - (3.0*e2/8.0+3.0*e2*e2/32.0+45.0*pow(e2,3)/1024.0)*sin(2.0*lat) + (15.0*e2*e2/256.0+45.0*pow(e2,3)/1024.0)*sin(4.0*lat) - 35.0*pow(e2,3)/3072.0*sin(6.0*lat))
 var east := 0.9996*n*(a+(1.0-t+c)*pow(a,3)/6.0+(5.0-18.0*t+t*t+72.0*c-58.0*ep2)*pow(a,5)/120.0)+500000.0
 var north := 0.9996*(m+n*tan(lat)*(a*a/2.0+(5.0-t+9.0*c+4.0*c*c)*pow(a,4)/24.0+(61.0-58.0*t+t*t+600.0*c-330.0*ep2)*pow(a,6)/720.0))
 return Vector2(east-float(origin.get("e0",0.0)),north-float(origin.get("n0",0.0)))

func _jump() -> void:
 var selected := _results.get_selected_items()
 if selected.is_empty() or main.world_state != "running" or main.world_kind != "real_world":
  return
 var match: Dictionary = _matches[selected[0]]
 var latitude := float(match.lat)
 var longitude := float(match.lon)
 if not is_finite(latitude) or not is_finite(longitude) or latitude < 0.0 or latitude >= 84.0 or longitude < -180.0 or longitude > 180.0:
  _status.text = "Destination is outside this map's northern UTM coordinate range."
  return
 var origin: Dictionary = _world.get("session_origin_utm", {})
 var point := session_coordinates(latitude, longitude, origin)
 if point.length() > float(_config.get("maximum_distance_from_origin_m", 30000.0)):
  _status.text = "Destination is outside the current map region."
  return
 main._simulation.relocate_vehicle(point.x, point.y, float(_world.get("spawn", {}).get("yaw_deg", 0.0)))
 print("RG_ADDRESS_TELEPORT ", JSON.stringify({"label":match.label,"latitude":latitude,"longitude":longitude,"session_m":[point.x,point.y]}))
 _toggle(false)
