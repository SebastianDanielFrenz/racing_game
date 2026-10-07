# Garage UI stages

## 2026-10-07: upgrade/tuning layout

The configurator now has vehicle information on the left, the showroom car in the middle, and parts or tuning controls on the right. Upgrades shows a three-column grid of actual compatible front/rear tyre definitions; selecting a card installs that definition in the existing validated working setup. Tuning adjusts installed components and shows the installed tyres without a second installation picker. Areas continue to choose the showroom camera. Non-replaceable installed component groups link to Tuning. Save, reset, invalid-setup handling and Drive use the existing garage model.

Panels resize with the logical canvas and the showroom camera accounts for both panels. At small window sizes content scrolls. Part selections are textual cards; no fabricated component catalog or performance bonuses are introduced.

The performance panel displays existing declared engine figures, mass, layout and gear count. Acceleration times, top speed with gear/RPM, and maximum longitudinal/lateral acceleration are explicitly unmeasured. A setup-specific native benchmark/cache is still required to populate them; the layout alone does not complete this part of Step 1. Acceleration results should show reached 100 km/h milestones through 500 in the first column, then 600–1000 in the second. Changing physical parts/tuning must invalidate or select the corresponding setup's results.

Remaining stages:

1. Native performance benchmarking and setup-specific results; additional real replacement-part catalogs as they become available.
2. Tyre customization beyond the current compatible-definition installation (physical values with validated ranges).
3. Engine customization with actual engine/part compatibility and physical definitions; coordinate with the physics library owner rather than arbitrary output multipliers.

Verification: real garage scripted screenshots at 1920x1080 and 1152x648; garage smoke passed 118 checks including tyre-card installation/restoration, tuning, rejection of invalid ratios, saved setup reload, Drive and node cleanup. Dedicated logs: out/garage_layout_test2.log and out/garage_layout_test2_engine.log. Screenshots: out/garage_layout_final.
## Component imagery

Upgrade tyre cards now use transparent product thumbnails, with visibly different road and track tread patterns. Explicit definition-to-presentation metadata lives in game/assets/components/tyres/catalog.json. Drag and slick artwork is prepared separately; those styles are not selectable until actual compatible physical definitions are supplied. See the asset README for the visual/physical scope.


## Step 2 requirements: tyre families, speed regimes and widths

Each compound/tread family must offer widths in configurable 10 or 20 mm increments (for example 235,245,255,...355 and beyond where the body allows). The upper limit comes from per-body, per-axle clearance, not a global tyre cap or a compound cap. Rim compatibility and diameter/clearance must also be checked. Width should update the tyre definition, wheel width and rendered component together.

Wider variants must have increased grip and resistance. Use a documented, tunable width response with diminishing grip returns rather than treating friction as proportional to contact-patch area. Account separately for rolling resistance, rotating mass and any aerodynamic consequences. Performance results must be keyed to the actual width, compound and tread setup.

Tyre families also need intended speed regimes and explicit speed ratings. A drag tyre is not automatically suitable for sustained high-speed cornering; a track semi-slick is different from a drag tyre and a fully smooth racing slick. Display the rating and suitability in the UI. The current rolling_resistance.rated_speed_kmh anchor is displayed on cards (hyper 500, sedan 270); it controls existing resistance calculations and is not itself a validated tyre-failure or speed-dependent grip model.

The current native compatibility selector requires wheel width to match tyre width within 6 mm. Width customization therefore requires extending the validated setup compiler and body fitment metadata before wider variants can be installed. Do not merely add menu widths that leave physical wheels unchanged. Current imagery work does not yet implement this physical width system.

Component imagery verification: 1920x1080 screenshot reviewed, no script/image errors, garage smoke still passes 118 checks. Logs: out/garage_component_shots_engine.log and out/garage_component_test.log.

