# Controls (R5b)

The control configuration menu and the input model under it (PLAN.md R5b, owner request 2026-10-06,
brief `out/brief_controls_menu.md`). Logic lives in `rg_core` (`rg::Controls`, `rg::BindingCapture`,
`rg::AxisCalibrator`); `RgControls` (GDExtension) converts Variants; `input_map.gd` gathers raw device
state and publishes action values; `controls_ui.gd` is the screen. No Godot type in `rg_core`.

## Where it is

Main menu / pause -> Settings -> "Controls...". The pause menu has a "controls" entry as well. The screen
lists every known device (connected ones first, then remembered ones as "not connected"); the selected
device shows its action rows grouped by section (Driving, Camera, On foot, Menus, Global), a live monitor
(raw axes/buttons/keys and what the game receives), conflict notes ("Also used by: ...") and the fixed
keys that cannot be rebound (`data/controls/fixed_keys.json`).

Per row: bind (click, then press the input), add a second binding, clear, reset the action. Per analogue
binding: deadzone, saturation, curve, sensitivity, invert, calibrate (move through the whole travel) and
"combine as pedals" (one axis becomes throttle and brake with opposite halves). Per device: reset all
(two presses, armed for 4 s).

Capture rules: inputs held at the start are ignored; Esc cancels; on a pad B cancels on a short press and
binds on a hold of 0.8 s; an axis binds after moving more than 0.6 from its resting value, the direction
choosing the half; a pedal resting at +-1 is bound calibrated (rest -> opposite end).

## Data

- `data/controls/actions.json` (`rg.control_actions/1`): the one action schema, 35 actions. Fields: `id`,
  `label`, `group` (driving/camera/on_foot/menus/global), `kind` (`button` | `axis`), `mode`
  (`held` | `edge`, buttons), `range` (`signed` | `unit` | `delta`, axes), `negative_label`/
  `positive_label`, `help`, `modes` (the player modes the action is live in; two bindings of one
  device conflict only when their actions' modes overlap). Strictly validated; a bad file refuses to
  start the controls layer.
- `data/controls/defaults/{keyboard,mouse,gamepad}.json` (`rg.control_profile/1`): the built-in default per
  device class. They equal the bindings the game had before R5b (frozen in `test_controls.cpp`), so
  nothing changes for a player who never opens the screen. A Generic pad (unknown SDL mapping) has no
  default and does nothing until the player binds it.
- `data/controls/fixed_keys.json` (`rg.fixed_keys/1`): description of the keys handled directly in scripts.
  Kept in step by hand.
- `<user dir>/controls.json` (`rg.controls/1`): the player's profiles, **overrides only** (action -> list of
  bindings, an empty list = cleared). Missing file -> defaults. A syntax error, wrong structure or unknown
  version -> defaults, the bad file is kept as `controls.json.bak`; an entry that does not validate is
  dropped and reported, the rest loads. Saved when the screen is left and on window close.

Nitrous (S1): the action `toggle_nitrous` (driving group, edge) toggles the plain 0/1 control channel `nitrous_arm`; default pad Y, key N. A car without a nitrous kit ignores it.

Assists (physics_sim assist unit): the action `traction_control` (key F4) steps the requested traction-control mode through the modes the car's assist unit offers (Off, On, Relaxed, Sport, Drift; a car without a unit offers Off and On; a mode the car does not offer is never selected - `rg::next_traction_mode`). The action `toggle_aero` (key H) toggles `assist.aero_control`, the rear-wing automation of a Racing unit; a car without it ignores the key. Neither channel has a game-side default: they start at the car's own (the hypercars carry the Racing unit: TC On, aero automation on).

Launch control (physics_sim assists I4/I7b): the action `toggle_launch_control` (key L) flips the `assist.launch_control` channel from its CURRENT value (`rg::toggled_launch_control`; a car without launch control ignores it). A drive-mode change also writes the mode's defaults into `assist.traction_control` and `assist.launch_control` itself (assists I7c; car_hyper_n2o: launch control on in SPORT, off in ECO and NORMAL), and the Session adopts those writes after the tick (`rg/channel_mirror.h`), so the HUD lamps follow without a key press and a key press made meanwhile wins.

Drive modes (physics_sim assists I7): the action `drive_mode` (key M, no gamepad button) cycles the requested drive mode ECO, NORMAL, SPORT through the modes the car's `drive_modes` part offers (`rg::next_drive_mode`; a mode the car does not offer is never selected, and on a car without the part the key does nothing). Only car_hyper_n2o carries the part (ECO / NORMAL / SPORT, default NORMAL). The game writes only the `assist.drive_mode` channel: physics_sim sets the traction-control request of the new mode itself (ECO and NORMAL On, SPORT Relaxed) and gates launch control and the N2O arm to SPORT, so the game never writes `assist.traction_control` on a mode change; F4 still takes over TC afterwards. The running mode is shown by a theme colour (ECO blue, NORMAL gray, SPORT orange; `game/scripts/drive_mode_theme.gd`) on the tach gauge, the hypercar cluster and the HUD readout, with the mode name bottom right under the tach dial. A launch-control lamp (`LC`) follows the snapshot's `launch_phase` (off, on, armed, launching); the game has no key for the `assist.launch_control` switch yet.

A binding: `{type: key|joy_button|joy_axis|mouse_button|mouse_motion, key|button|axis, span:
full|positive|negative, sign, deadzone, saturation, curve, sensitivity, invert, calibrated, cal_min,
cal_max}`. Axis pipeline in order: calibration -> invert -> span -> deadzone/saturation -> curve ->
sensitivity (see `core/include/rg/control_binding.h`).

## Device identity

- `keyboard` and `mouse` each have one profile.
- A pad is `joy:<guid>#<n>`: the SDL GUID (`Input.get_joy_guid`: bus/vendor/product/version, the same for
  two units of one model - there is no per-unit serial) plus an ordinal among devices of that GUID. The
  ordinal is the lowest one not held by a currently connected device of the same GUID, so a lone pad is
  always #1, a reconnected pad gets its profile back, and two identical pads are #1 and #2 in connection
  order.
- Profile resolution: own profile -> the GUID's `#1` profile (a second identical pad starts from the first
  one's setup; its own profile is created, copying that setup, on its first edit) -> the class default.
- Hot-plug: `Input.joy_connection_changed` -> `joypad_connected/disconnected`; an unplugged device keeps
  its profile and stays listed "not connected".
- Several devices act at once: each profile binds actions on its own device; per action the larger
  magnitude wins.

## Tests

- Core: `tests/unit/test_controls.cpp` (41 cases: schema validation, defaults equal to the old bindings,
  axis pipeline incl. calibration, combined pedals, ordinals and fallback, hot-plug, conflicts, persistence
  and corrupt-file policy, larger-magnitude-wins), `tests/unit/test_control_capture.cpp` (capture and
  calibration), `test_shell_flow.cpp` (Screen::Controls).
- Headless: `tools\smoke_test.ps1 -Controls` runs Godot twice - `--controls-test` (62 checks: device list,
  binding, conflict display, capture of key/pad button/axis, calibration, reset, hot-plug) writes
  `controls.json`, then `--controls-verify` in a NEW process on the same `--shell-user-dir` (14 checks)
  confirms the choices survived the restart. `-Shell` covers the pause -> Controls -> Esc path.
- Screenshots: `tools\controls_shots.ps1 [-OutDir out/controls_screens] [-Resolution 1600x900]`
  (`--controls-shots <dir>`), forward slashes in `-OutDir`.

## Known limits

- Steer rows of a pad show the side rows as unbound when the full axis is bound.
- The raw axis bars show half fill at 0.
- Real devices (wheel, pedals, two identical pads, hot-plug, mouse look) could only be simulated headless;
  see the report's hand-check list.
