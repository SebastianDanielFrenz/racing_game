# Aerodynamics integration

The physics library supplies a deterministic quasi-steady coefficient model,
not a CFD solver. Local airflow includes wind, angular velocity at the pressure
point and other vehicles' expanding wakes. Drag, side forces, signed lift and
moments are integrated with the vehicle substep/shadow wrench. Aerodynamic force
is applied at each surface position, so it changes real tyre loads and chassis
pitch, roll and yaw. Legacy drag-only definitions remain supported without
adding duplicate drag.

The supplied hypercar has authored body drag, a splitter, underfloor and rear
wing. Coefficients are fictional and not a manufacturer or wind-tunnel fit.
Ground effect uses a probe pointing toward the nearby local surface; missing
support leaves the effect at its baseline rather than inventing a floor. The
same orientation-independent model supports positive-lift wings, inverted cars
and aircraft. It does not guarantee upside-down adhesion: the available force
must overcome gravity while the wheels maintain valid surface contact.

Wakes are a finite expanding-cone entrainment/turbulence approximation with
bounded multi-source combination, not a resolved vortex field. Vehicle sources
and explicitly supplied non-vehicle/truck sources use the same sampling model.
The game currently has one player vehicle; traffic gameplay and aircraft flight
controls are separate features, not introduced by aerodynamic integration.

## Controls and telemetry

The game stepping thread drives the `rear_wing` actuator automatically. At normal
forward driving speeds it ramps toward the configured cruise offset; braking
above the configured threshold and minimum speed requests an airbrake offset.
The engine rate-limits the actuator, so target changes do not instantly rotate
the aerodynamic surface. The supplied art wing rotates from the ACTUAL simulated
offset, read from the same latched frame snapshot as chassis and wheel poses.
Wing-lift struts follow the actual simulated actuator lift, including its changing pressure-point height.

`get_aero_state()` exposes wind/density, actual body airspeed including wake,
net force/moment, drag/downforce, axle-based front balance, wing offset, optional
fan power/remaining energy and per-surface angle of attack, sideslip, lift/drag
coefficients, dynamic pressure, ground clearance and multiplier. Vectors use
world/session ISO (east,north,up) axes. The HUD shows the main values. Middle-click
coordinate bookmarks also contain this aero state. Axle balance is derived from
net local pitch moment and actual axle positions; it can indicate unloading,
rather than categorising every pressure point ahead of the CoM as front-axle force.

## Optional suction fans

Fan definitions belong to a vehicle's `aero.fans` array. The stock hypercar has
none. The native model computes pressure-area suction toward a configured local
surface, with leakage as the gap opens, command slew limits and an explicit
energy reservoir. A missing surface produces no suction, while an operating
motor still spends energy. It can operate without road speed when a seal and
power supply exist.

The game can credit a configured finite auxiliary battery once at Session
creation using `fan_battery_energy_j`, and commands fitted fans using
`fan_command` in environment.json. Both default to zero. This is an explicit
stored-energy budget, not a free grip force or an automatic engine-recharge model.
Resetting position does not recreate the Session or refill this energy budget.
No fan model/geometry is added to the stock car automatically.

## Environment

`data/world/environment.json` defines mean wind and optional smooth deterministic
gusts in session/world ISO axes: east, north, up, in metres per second. Wind points
in the direction the air moves, not the meteorological 'wind from' direction.
Default wind and gust amplitudes are zero. Gust evolution uses simulation time,
so a terrain streaming pause also pauses the environment. This is a bounded wind
input, not a turbulent computational fluid dynamics simulation.

Sea-level pressure and temperature, optional altitude-dependent density and the
temperature lapse rate are configurable. Pressure uses the hydrostatic constant-
lapse approximation and density uses the ideal-gas law. Altitude is limited to
-500..11,000 m for this tropospheric approximation. Zero lapse uses the isothermal
formula. The same sampled pressure/temperature is supplied to simulated engines.

Reference physics: [NASA aerodynamic forces](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/aerodynamic-forces/)
and [NASA atmosphere model](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/earth-atmosphere-equation-english/).

## Settings

Reload the real world or restart after editing environment.json. `enabled` controls
atmospheric overrides, not whether vehicle aerodynamic surfaces exist. Missing
file preserves the legacy sea-level/no-wind environment; malformed settings stop
loading with an error. Flat test mode uses the native sea-level/no-wind environment
and default automatic wing policy.

| Setting | Default | Meaning |
| --- | ---: | --- |
| altitude_density | true | Use chassis altitude in the atmosphere model |
| sea_level_pressure_pa | 101325 | Pressure reference |
| sea_level_temperature_k | 288.15 | Temperature reference |
| lapse_k_per_m | 0.0065 | Temperature decrease per metre |
| wind_world_m_s | [0,0,0] | Mean east/north/up wind velocity |
| gust_amplitude_m_s | [0,0,0] | Bounded smooth gust amplitudes |
| gust_period_s | 12 | Base gust period |
| automatic_rear_wing | true | Drive the named rear_wing actuator |
| cruise_wing_offset_deg | 4 | Additional pitch relative to art/physics rest angle |
| airbrake_wing_offset_deg | 45 | Maximum requested braking offset |
| airbrake_min_speed_m_s | 20 | Minimum forward airspeed for airbrake |
| airbrake_threshold | 0.45 | Brake pedal threshold; progressively increases command |
| fan_command | 0 | Command for fitted suction fans, 0..1 |
| fan_battery_energy_j | 0 | Initial finite auxiliary energy, debited by native fan model |

Target offsets are limited by each surface's configured mechanical range. Wind
and actuator inputs are applied only on the simulation thread; presentation reads
snapshots rather than live mutable physics objects.

## Continuous atmosphere performance

The initial integration exposed a native simulated-engine hot path: tiny chassis
altitude changes rebuilt the entire steady throttle/inverse table every tick,
raising the real-world hypercar tick from about 0.3 ms to 105 ms. Physics pin
5ec8b7d0a39953b0bfe8f57e1abc3808b5b37ede removes that per-tick full-table rebuild while retaining continuous
pressure/temperature inputs. The local hidden `[aero_perf]` regression compares
the same spawn with ground effect on/off and continuous atmosphere on. It requires
`RG_G2M_HOME` pointing at the owner terrain cache.
Game real-spawn regression after the fix: mean tick 0.389 ms with atmosphere enabled (previously 105.586 ms), 20 ticks; ground probe mean 0.001 ms. Driving acceptance remains with the owner.
