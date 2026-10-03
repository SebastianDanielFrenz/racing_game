# Cameras and engine audio

Tab or right-stick click switches the desktop driving camera between chase
and cockpit. V still switches drive/free-camera mode. The cockpit uses the
model's `socket_driver_eye` through `body_visuals.driver_eye_local()`, rigid
chassis mounting (including pitch/roll), a 3 cm near plane, and a slight
downward gaze, following physics_sim's cockpit camera reference. Right-stick
look stays continuous around the full circle; releasing returns ahead.
Tracked VR keeps its existing cockpit/free rigs.

Chase view probes the already-cached shared driving terrain below the final
lagged/orbited camera and its near-plane footprint, enforcing 0.6 m ground
clearance. This does not query the live physics world or trigger synchronous
tile generation on the render thread. Missing/NoData samples are ignored.
It prevents terrain penetration; it does not handle arbitrary building or
bridge-slab obstruction.

`data/controls/presentation.json` exposes engine levels in decibels:
chase -12 dB, cockpit -8 dB, free camera -6 dB. Restart after edits. Levels
apply to PCM before either Dolby/Windows object audio or Godot output, with
a smooth fade on view changes. Tyre audio retains its existing level.
Native build and script compilation are checked; subjective camera/audio
verification is left to the owner.

## Shared frame poses, live dashboard and bookmarks

The adapter latches one immutable physics snapshot per Godot process frame.
Camera, car/wheel rendering, gauges and other snapshot getters use that same
tick even when the 240 Hz physics thread advances between render callbacks.
A new world invalidates the latch. This prevents the camera/body relative
pose mismatch that is especially visible from a rigid cockpit camera.

The supplied hypercar now mounts the reference demo's `hyper_live_cluster`
and `digital_cluster` display on the same model-local panel. Live telemetry
populates speed, RPM, gear, boost, drive power, fuel and consumption/trip/range
fields. Fuel capacity/density and fuel/boost/power snapshot values are exposed
through the existing vehicle gauge APIs. Derived trip/range values become
available after sufficient driving.

Middle mouse click bookmarks the car's current location (not a cursor-picked
point) as `RG_OWNER_MARK` in the drive log and appends it to
`user://drive_marks.jsonl`. Marks include session/UTM coordinates, wall and
simulation time, tick, speed, view, build and wheel load/slip/surface data.
This archive persists across normal Godot log rotation.

## Configurable spawn and R reset

`data/world/world_config.json` exposes `spawn.latitude`, `spawn.longitude`,
and `spawn.yaw_deg` (degrees counter-clockwise from east in the UTM frame).
Startup and R reset both use this point; restart or reload the real world
after editing. Legacy UTM `spawn.e`/`spawn.n` remain supported, but do not
mix coordinate formats. The configured road-centre point beside the old spawn point is 50.12359409, 8.51546541, with heading 4.05 degrees. It is projected
from the OSM address onto the street to avoid the building.

Manual cars (`manual_tcu`) start with auto-shift disabled. The existing F5 key /
D-pad-left toggle still enables it on demand and survives world reloads for the
same car. Automatic/non-manual cars retain the previous enabled default.
