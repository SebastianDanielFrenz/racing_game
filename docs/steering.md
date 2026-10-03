# Adaptive steering

Keyboard and gamepad inputs pass through `game/scripts/adaptive_steering.gd`.
This translates driver input only; it never steers automatically. Settings
live in `data/controls/steering.json` and apply after restarting the game.

Normal steering uses a progressive stick curve (exponent 1.8) and a
speed-dependent limit based on vehicle wheelbase, maximum wheel angle, and
`normal_lateral_accel_mps2` (18). The limit bottoms out at
`normal_min_fraction` (0.14): approximately 4.5 degrees for the supplied
hypercar's 32-degree maximum. Full input can exceed tyre grip at high speed
and initiate a drift without commanding full steering lock. Parking retains
full lock. Wheelbase and steering lock come from the active vehicle JSON.

Slide angle is measured from the chassis's actual body-frame longitudinal
and lateral velocity. Countersteering is input towards that velocity, with
yaw-rate anticipation (`yaw_anticipation_s`, 0.15). Above 5 m/s forward speed,
recovery authority blends in between 3 and 20 degrees of slip, restoring up
to direct/full input for severe corrections. It never gives additional
authority to input pointing into the slide, including during keyboard
direction reversals. Recovery fades out at `recovery_fall_per_s` (4), rises
at `recovery_rise_per_s` (12), and is disabled when the chassis is overturned
or reversing. This does not guarantee recovery of every physical slide.

Keyboard steering builds at `keyboard_rise_per_s` (4), returns at
`keyboard_return_per_s` (8), and builds faster during recovery using
`recovery_keyboard_per_s` (18). The controller's analog position is used
directly before the progressive curve. World changes and relocations reset
filter state. Scripted physical steering controls bypass this translation.

`input_profile`: `auto` detects common wheel names and preserves their direct
steering; `wheel` forces direct analog steering for an unrecognised wheel;
`controller` forces adaptive analog steering. Keyboard always uses adaptive
translation when enabled. Set `enabled` to false for direct legacy input.
Driving verification and final tuning are left to the owner; native build
and Godot script compilation are checked before delivery.
