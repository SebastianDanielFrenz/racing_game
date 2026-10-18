# Powertrain swaps and terrain handoff

Owner cutoff: stop implementation at 85% used of the five-hour Codex usage window. Latest checkpoint: 72%; no reset credit used.

## Implemented locally
Game checkout: S:/claude_code/racing_game_branches_for_testing/pr33_ready, branch codex/powertrain-swaps.
Physics checkout: S:/claude_code/physics_sim_jacket_ready, branch codex/hybrid-fuel-cell, committed through a165761d0a93dd740228bb3eed9436adda705930. Game submodule pins this commit locally. Physics branch has been pushed to origin; its commit is fetchable for the game submodule.

- Swaps -> Powertrain -> Internal combustion / EV / Hybrid. ICE subcategories: Gasoline / Diesel / Hydrogen. Five conversion packages: ICE, battery EV, hydrogen fuel-cell EV, battery hybrid, dual-hydrogen hybrid.
- Dual hydrogen is hydrogen combustion plus hydrogen fuel-cell electric drive, with separate hydrogen stores. Diesel and hydrogen I4 torque curves are authored development engines, not measured production data.
- Conversions retain body, suspension, wheels, brakes and physical aero. Hybrid uses a post-gearbox traction motor and a separate crank generator; the engine can disconnect mechanically while electric propulsion continues.
- Battery and fuel cell have explicit power/energy limits and telemetry. Fuel-cell chemical/electric/loss accounting and hydrogen mass update are simulated. Fixed stack efficiency and reference hydrogen density; no electrochemistry, thermal stack model or real-gas tank model.
- ECO uses electric wheel drive. Normal starts ICE above electric power potential. Sport/Racing keep ICE running. Automatic schedule load uses combined electric and running-ICE power at current RPM.
- Low SOC (default 20%) latches ICE charging until 80%. Charging occurs with the mechanical transmission in neutral. A separate crank generator, separate traction motor and disconnect-capable transmission are required; unsupported hybrid topology is rejected. ECO can run disconnected ICE for charging.
- Hybrid READY/start-stop, modes, battery/fuel-cell/H2 dashboard telemetry and existing electric motor audio are wired. EV dyno explicitly reports that motor dyno is pending, clearing stale combustion curves. Hybrid engine dyno remains combustion-only.
- Fixed authored hydrogen starter free speed (700 RPM) so the CVT neutral load does not hold cranking below firing speed.

## Validation
- Scoped physics CI: tools/ci.ps1 -Topic vehicle,drivetrain,io -Only release -SkipWsl passed all 8 steps (981 selected tests, 2 walltime tests, include/math greps). Cross-platform hash compare was skipped explicitly; no full or sanitizer CI claim.
- Physics EV tests: 13 cases, 94 assertions passed, including hybrid/fuel-cell worker-count determinism and a fuel-cell DC supply cap. Fuel + EV suite: 71 cases, 110009 assertions passed. Includes fuel-cell energy closure, READY/empty/full, all hybrid modes, charge hysteresis to 80%, shutdown and rejection of unsupported hardware.
- Game conversion/gearbox tests: 5 cases, 21796 assertions passed. Covers conversion loading and AT/DCT/CVT electric launch, Sport ICE startup and stationary neutral charging to 80%, followed by ICE shutdown; three repeated READY/respawn/ICE restart cycles per gearbox; electronic parking brake retention on conversions.
- Godot editor parse completed. Existing electric-drive/audio acceptance: 24 checks, zero failures.
- New game/scripts/powertrain_swap_test.gd: 155 checks, actual category buttons plus materialisation, save/reload and return-to-ICE conversion for all five packages on sedan, electric sedan and N2O hypercar; zero failures. Uses isolated out/ user/work files.
- Broader garage/automatic/EV/audio/driver regression: 126 cases, 37230 assertions passed (out/hybrid_game_regressions.log). Legacy ICE-only gearbox/layout loops now exclude the EV; trim and option-count expectations respect its explicit catalogue allowlist.

## Remaining work and limits
- No public PR or merge for this batch yet. Game edits are checkpointed locally; physics branch is published and the game pin is fetchable.
- Racing boost preparation is intentionally not implemented; it currently shares Sport ICE keep-running behavior.
- EV/combined hybrid dyno, detailed source thermal/storage models, and physical conversion packaging/fit constraints beyond the four-road-wheel topology remain future work.
- Electronic parking brake parts are now retained across EV and hybrid conversions. Conversion electronics use hybrid TC/mode control; existing ICE-specific launch/aero automation electronics are replaced. Physical aero remains. Preserve or reintroduce compatible independent aero control in a follow-up.
- Hybrid requested full-pedal system power is an authored package calibration (300 kW), not automatically derived for each swapped engine/upgrade. Extend calibration for proportional pedal-demand behavior across arbitrary high-output engine combinations (full throttle still requests full ICE output).
- A direct shift-schedule comparison across ICE startup remains to be added; mode, gearbox, restart, determinism and save/reload transitions now have coverage.
- Garage vehicle-browser specification figures are still stock-engine figures; converted propulsion and mass are used by the actual simulation. Combined installed-system ratings need a dedicated presentation.
- Do not hide known issues by loosening assertions, clamping plotted power or claiming stack output bypasses its ratings.
## Outstanding terrain fix (investigated, not fixed)
Detailed report: out/highway_cave_in_investigation.md; archive out/highway_cave_in_logs_20261018_002325/godot.log. Owner mark169.95s at50.098796,8.487708 (UTM32N E463360.94 N5549741.04,z116.86m). No starvation/missing required tiles/new freeze during event.
Exact served geometry probe built against geo2map4fb8e8627273516ab2245c9ca5de7e68684e2ede with independent derived cache reproduced:
- A66 westbound way313879319, GroundCompleted provenance, artificial drop about117m ->112.67m then approximately+9.9% uphill ramp; matches logged dive/rebound and four zero wheel loads at169.908s,vz+8.03m/s.
- Connected way323855192 has no profile after completion; retained decline estimated endpoint slope exceeds class bound. Missing span202.77m from node3306247908 (E463374.298 N5549746.392) to39751651 (E463211.709 N5549626.412).
- Both are ordinary A66 motorway OSM ways, no bridge tag on these ways; served tile utm32n/2/452/5419 has zero underpass-deck records. Whether source tagging or support inference needs correction remains open.
- Fix connected motorway elevation/support reconstruction and missing profile, add route regression proving coverage and height/grade continuity; validate wheel contact/rendering. Do not hide with TC/tyre tuning, global flat-road clamping or an airborne-car patch.
Reproducer: out/highway_probe_source/served.cpp + out/highway_probe_build/highway_served.exe. Outputs highway_served_entries.csv, highway_served_samples.csv, highway_served.log, highway_mark_samples.csv. Strict-only road_probe omits completion/context and is insufficient evidence.

## Terrain diagnosis refined (2026-10-18)
The portable committed repro is tools/diagnostics/highway_profile_probe. With a fresh private cache it confirms way313879319 crosses one lower road, but RoadUnderpassFinder rejects the mask: crossings=1, served=0, reaches_end=1; no shallow/no_rim/too_long/DEM-missing rejection. The next way323855192 has crossings=0. Thus the OSM boundary splits the supported depression: the first way drops into the bare-earth tunnel cutting and the next begins on an invalid steep DEM plane.
Nearby lower service way5436745 is tagged tunnel=yes,layer=-1,maxheight=4; adjacent service approaches5436746 and54276755 carry cutting=yes. Bare DEM cross sections show the narrow dip: approximately115.54m at E463400 ->110.82m at E463380 ->115.18m at E463370 (approximate reference, not exact lane trajectory).
Zero served decks is not itself a bug: D61 explicitly says a tunnel-only crossing is covered, so it should get a profile mask/fill, not an exposed bridge deck. The actual bug is rejecting mask support at the upper-way boundary and failing to share corrected endpoint support with its neighbour.
Next fix must compute shared support from crossing/connected-way context in a tile-invariant way, extend covered masks across the way split, and use corrected common endpoint planes before strict/completion grade checks. Preserve measured support/rims and ownership invariance; do not loosen all grade bounds or remove clearance unconditionally. Check the tilted-bumpy fixture (G2M-019 explains why uncapped endpoint easing produces large junction gradients). No terrain production code changed yet.
## Already delivered
EV physics PR20 and game PR40 merged; motor audio PR41 merged and installed in pr33_ready. Existing launcher out/run_home_region.ps1, garage Sedan E, K toggles READY. Last accepted EV tests25checks with Windows spatial and Godot fallback; audio native tests2cases. New swap drafts must not regress these.
