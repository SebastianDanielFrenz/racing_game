# VR menus

`tools/run.ps1 -VR` now starts the normal menu flow. Spawn selection, the car
browser, garage upgrades/tuning, exit confirmation and loading are displayed on
a fixed panel in the headset. The existing showroom is rendered within that
panel; this is not yet a walk-around, stereo garage room.

Point either tracked controller and squeeze its trigger to select or drag a
slider. The primary stick scrolls the panel under the pointer. B/Y or the
controller menu button goes back. A/X confirms keyboard/controller focus.
Keyboard and gamepad navigation also work; ESC/B retain the existing submenu
and save-and-exit behavior. Without an active tracked controller, the pointer
follows head gaze. The panel hides and stops rendering when driving starts.

Dropdowns remain inside the panel. The loading screen uses the same viewport,
and the tracked driving camera is restored after leaving the menus. The panel
is drawn over world geometry so the car or terrain cannot obscure it.

For the previous direct launch, use `tools/run.ps1 -VR -Drive`. This retains
the A5/Rosbach direct-start preset. `-VR -Flat` still starts the flat test world.

Automated check: run Godot with `--headless --path game --script
res://scripts/xr_menu_test.gd`. It checks script compilation, panel ray mapping,
GUI click forwarding, controller back and screen visibility transitions.
These checks do not validate headset optics, controller bindings on every
device, or comfort; those require testing with an OpenXR headset.
