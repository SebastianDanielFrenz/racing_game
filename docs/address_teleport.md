# Address teleport

Press F10 in a running real-world session. Enter a full address (street, house
number and town), press Enter/Search, select a result, then click Teleport or
double-click the result. Latitude, longitude input also works without network
access. Escape/Cancel closes the dialog. Driving controls are held neutral
with brake applied while the dialog is open; typed R/F/V/T do not trigger
vehicle reset, flip, mode changes or the manual truck.

Address searches use the configurable Photon endpoint in
data/controls/geocoding.json. Only explicit searches make requests, at most
one per second; successful searches are cached in
user://address_search_cache.cfg. Searches send the entered text and map-spawn
location bias to that service. Photon/OSM attribution is shown in the dialog.
The public service is suitable for light personal usage without an availability
guarantee; a private Photon endpoint can be configured for larger deployments:
https://github.com/komoot/photon

WGS84 coordinates project into the configured northern UTM zone and subtract
the fixed Session origin, independently of the floating render origin. The
existing relocation/terrain-loading mechanism places the car at rest and upright
on the ground. This does not alter the configured R-reset spawn. Address points
are exact geocoding destinations, not a nearest-road navigation calculation.
Destinations farther than maximum_distance_from_origin_m (default 30 km) are
rejected; actual terrain availability still depends on the current map data.
RG_ADDRESS_TELEPORT records the selected destination for diagnostics.
