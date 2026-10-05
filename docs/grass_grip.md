# Grass traction

The game loads data/surfaces/surfaces.json rather than modifying the read-only
physics_sim surface table. Grass lambda_mu is 0.63. It was doubled from 0.42 to 0.84 (RACE-002) and
reduced to 0.63 on 2026-10-05 after the owner found it "a bit too grippy now"
(halfway back; dirt is 0.6, so grass is still marginally above dirt - the owner
judges the value by driving, and wet grass will multiply the dry value later). This scales longitudinal and lateral tyre friction; rolling resistance
stays 0.08. lambda_mu is relative to the tyre model's dry-asphalt friction, not
an absolute coefficient or guaranteed chassis acceleration.

Cenek, Jamieson and McLarin, *Frictional Characteristics of Roadside Grass Types*,
Table 4, measured locked-wheel braking from 35–45 km/h on hard underlying soil:

| Surface | Dry deceleration | Wet deceleration |
| --- | --- | --- |
| Clover | 0.21 g / 2.06 m/s² | 0.17 g / 1.67 m/s² |
| Long rye-grass | 0.36 g / 3.53 m/s² | 0.21 g / 2.06 m/s² |
| Short rye-grass | 0.38 g / 3.73 m/s² | 0.24 g / 2.35 m/s² |

Source: https://saferroadsconference.com/wp-content/uploads/2016/05/Peter-Cenek-Frictional-Characteristics-Roadside-Grass-Types.pdf

These are sliding-braking measurements, not optimum-slip powered acceleration
measurements. As a first-order traction estimate, a ≈ mu*g when all weight is
carried by driven tyres; driven-axle weight, transfer, rolling resistance,
aerodynamics, tyre slip and soil deformation change acceleration. The study's
lightly loaded tyre-drag numbers (0.68–0.80 dry) are explicitly unsuitable as
full-car friction values. The game grip is an owner-requested handling choice above the measured sliding
values, not a realism calibration.
No wet-grass weather switching is added by this change. Restart to load it.
