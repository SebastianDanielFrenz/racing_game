# Hypercar engine calibration

Physics pin: bc6fdc41db4675e3bb56c9f1cb8b418cbc9724fd, pulled from S:/claude_code/physics_sim.
The default car continues to use the supplied cycle-generated simulated V8,
including fuel use, turbo dynamics and live engine audio.

The calibration retains 92 mm bore, 95.25 mm stroke, eight cylinders,
9:1 compression, 8250 RPM limiter, 1.8 bar charge-boost target and E5 fuel.
Game chassis mass remains 1500 kg. No global torque multiplier is used.

Assumed valve sizes/lift, cam timing and flow dimensions were changed to improve
high-RPM filling. Turbo delivery and intercooling were recalibrated, with
RPM-dependent high-load ignition adjustment to control midrange output.

| RPM | Previous net torque, Nm | Calibrated net torque, Nm | Calibrated power, kW |
| ---: | ---: | ---: | ---: |
| 3000 | 654.5 | 1230.3 | 386.5 |
| 6000 | 1336.3 | 1371.0 | 861.4 |
| 7500 | 1021.9 | 1273.5 | 1000.2 |
| 8000 | 904.1 | 1184.3 | 992.1 |

Same native held-crank dyno: fresh engine per point, 15 seconds at full throttle,
last second averaged, sea-level 101325 Pa / 288.15 K. Net crank figures include
smooth and Coulomb friction. The previous net 7500-RPM power was 802.6 kW;
the older 824-kW documentation omitted the fixed Coulomb loss in its quoted output.
The new output is approximately 1360 PS, a 24.6% gain at that point.
These are sampled steady results; transient driving, altitude and tyre grip
still affect acceleration and speed.

Detailed parameters, full curve and native reproduction instructions:
[library calibration](../external/physics_sim/data/engines/HYPER_ONE1.md).
The underlying cams, ports and turbo data are authored assumptions. Optimistic
steady intercooling and the absence of validated knock limits mean this is a
calibrated game model, not measured manufacturer internals.

Restart the game after the update. Content-keyed engine maps regenerate for the
changed cycle and are then reused by the existing cache. Driving acceptance is
left to the owner.
