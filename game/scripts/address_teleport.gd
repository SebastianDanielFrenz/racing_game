extends CanvasLayer
# Manual address searches only; no request on each keystroke.
var main: Node
var _root: Control
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

const PANEL_WIDTH := 660.0
const RESULTS_HEIGHT := 190.0
const SCREEN_MARGIN := 16.0

func _ready() -> void:
 layer = 40
 _cache.load("user://address_search_cache.cfg")
 # Query parsing, the search URL, the response parsing and the destination check are rg_core's
 # (rg/address_search.h through RgAddressSearch); this panel owns the widgets, the request and the cache file.
 _config = RgAddressSearch.load_config(ProjectSettings.globalize_path("res://../data/controls/geocoding.json"))
 if not bool(_config.get("ok", false)):
  push_warning("Address search: " + str(_config.get("message", "")))
 _http = HTTPRequest.new()
 _http.use_threads = true
 _http.timeout = 15.0
 _http.body_size_limit = 2097152
 add_child(_http)
 _http.request_completed.connect(_request_complete)
 # Full-rect root + CenterContainer, like shell_ui.gd: the dialog stays centred
 # in the canvas whatever the window size and the ui_scale.gd content scale.
 # (It used to be placed by hand from get_visible_rect() and _panel.size, which
 # left it partly off-screen on a scaled 4K window.)
 _root = Control.new()
 _root.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
 _root.mouse_filter = Control.MOUSE_FILTER_IGNORE
 add_child(_root)
 var center := CenterContainer.new()
 center.set_anchors_and_offsets_preset(Control.PRESET_FULL_RECT)
 center.mouse_filter = Control.MOUSE_FILTER_IGNORE
 _root.add_child(center)
 _panel = PanelContainer.new()
 _panel.custom_minimum_size = Vector2(PANEL_WIDTH, 0)
 center.add_child(_panel)
 var box := VBoxContainer.new()
 box.add_theme_constant_override("separation", 12)
 _panel.add_child(box)
 var title := Label.new()
 title.text = "Teleport to address (F10)"
 title.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
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
 _results.custom_minimum_size = Vector2(0, RESULTS_HEIGHT)
 _results.item_selected.connect(func(_index: int): _teleport.disabled = false)
 _results.item_activated.connect(func(_index: int): _jump())
 box.add_child(_results)
 _status = Label.new()
 _status.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
 box.add_child(_status)
 var attribution := Label.new()
 attribution.text = "Address search: Photon / © OpenStreetMap contributors"
 attribution.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
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
 _root.resized.connect(_fit)
 _fit()

# Shrink the panel (width, then the result list) on a small canvas so it never
# exceeds the window; the CenterContainer does the centring.
func _fit() -> void:
 var avail := _root.size - Vector2(SCREEN_MARGIN, SCREEN_MARGIN) * 2.0
 if avail.x <= 0.0 or avail.y <= 0.0:
  return
 _panel.custom_minimum_size.x = minf(PANEL_WIDTH, avail.x)
 _results.custom_minimum_size.y = RESULTS_HEIGHT
 var overflow := _panel.get_combined_minimum_size().y - avail.y
 if overflow > 0.0:
  _results.custom_minimum_size.y = maxf(48.0, RESULTS_HEIGHT - overflow)

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
  call_deferred("_fit")
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
 var point: Dictionary = RgAddressSearch.parse_coordinates(query)
 if not point.is_empty():
  _show_matches([point])
  return
 var cache_key: String = RgAddressSearch.cache_key(query)
 if _cache.has_section_key("search", cache_key):
  var cached = _cache.get_value("search", cache_key)
  if cached is Array:
   _show_matches(cached)
   return
 if not bool(_config.get("ok", false)):
  _status.text = "Address search is not configured (data/controls/geocoding.json)."
  return
 if Time.get_ticks_msec() - _last_request_ms < int(float(_config.get("min_request_interval_s", 1.0)) * 1000.0):
  _status.text = "Please wait a second before searching again."
  return
 _last_request_ms = Time.get_ticks_msec()
 var url: String = RgAddressSearch.search_url(_config, query, _world.get("spawn", {}))
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
 var parsed: Dictionary = RgAddressSearch.parse_response(body)
 if not bool(parsed.get("ok", false)):
  _status.text = "Address service returned an invalid response."
  return
 var matches: Array = parsed.get("matches", [])
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

func _jump() -> void:
 var selected := _results.get_selected_items()
 if selected.is_empty() or main.world_state != "running" or main.world_kind != "real_world":
  return
 var match: Dictionary = _matches[selected[0]]
 var latitude := float(match.lat)
 var longitude := float(match.lon)
 var target: Dictionary = RgAddressSearch.check_destination(latitude, longitude, _world.get("session_origin_utm", {}), float(_config.get("maximum_distance_from_origin_m", 0.0)))
 if target.status == "outside_utm":
  _status.text = "Destination is outside this map's northern UTM coordinate range."
  return
 if target.status != "ok":
  _status.text = "Destination is outside the current map region."
  return
 var point := Vector2(float(target.x), float(target.y))
 main._simulation.relocate_vehicle(point.x, point.y, float(_world.get("spawn", {}).get("yaw_deg", 0.0)))
 print("RG_ADDRESS_TELEPORT ", JSON.stringify({"label":match.label,"latitude":latitude,"longitude":longitude,"session_m":[point.x,point.y]}))
 _toggle(false)
