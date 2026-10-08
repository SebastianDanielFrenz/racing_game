# Rolling resistance model

Set `physics.rolling_resistance_model` in `data/world/world_config.json` to
`quadratic` (default) or `fourth_power`, then start a new game session.
The setting applies to the player's tyres in both flat and real-world modes,
including car changes. Traffic currently uses its simpler driving model.

Quadratic uses a constant baseline plus a nonnegative speed-squared term,
calibrated to the same baseline and rated-speed resistance as the tyre's
original model. The original fractional-speed calibration point is not an
additional constraint on this curve. Fourth power restores the existing
Pacejka/Magic Formula linear-plus-fourth-power speed terms unchanged.
Surface resistance, longitudinal-force dependence, wheel load and the
near-zero-speed sign ramp remain part of the shared native wheel calculation.

This setting controls tyre rolling resistance; aerodynamic drag remains
separate. A running session keeps its selected model until reinitialized.
For explicit QSY coefficients or imported .tir tyres without rated-speed anchors,
the quadratic curve matches the original speed term at the tyre's reference
speed v0. The original QSY coefficients are retained for reversible switching.
