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