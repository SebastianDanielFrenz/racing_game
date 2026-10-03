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
