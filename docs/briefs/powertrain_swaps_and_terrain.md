# Powertrain swaps and terrain handoff

Owner cutoff: stop implementation at 85% used of the five-hour Codex usage window. Latest checkpoint: 53%; no reset credit used.

## Implemented locally
Game checkout: S:/claude_code/racing_game_branches_for_testing/pr33_ready, branch codex/powertrain-swaps.
Physics checkout: S:/claude_code/physics_sim_jacket_ready, branch codex/hybrid-fuel-cell, committed through 6152dfe0052a7a11ded19fd0e6d4393a5a9f7ec9. Game submodule pins this commit locally. These commits must be pushed before a public game PR can fetch them.

- Swaps -> Powertrain -> Internal combustion / EV / Hybrid. ICE subcategories: Gasoline / Diesel / Hydrogen. Five conversion packages: ICE, battery EV, hydrogen fuel-cell EV, battery hybrid, dual-hydrogen hybrid.
- Dual hydrogen is hydrogen combustion plus hydrogen fuel-cell electric drive, with separate hydrogen stores. Diesel and hydrogen I4 torque curves are authored development engines, not measured production data.
- Conversions retain body, suspension, wheels, brakes and physical aero. Hybrid uses a post-gearbox traction motor and a separate crank generator; the engine can disconnect mechanically while electric propulsion continues.
- Battery and fuel cell have explicit power/energy limits and telemetry. Fuel-cell chemical/electric/loss accounting and hydrogen mass update are simulated. Fixed stack efficiency and reference hydrogen density; no electrochemistry, thermal stack model or real-gas tank model.
- ECO uses electric wheel drive. Normal starts ICE above electric power potential. Sport/Racing keep ICE running. Automatic schedule load uses combined electric and running-ICE power at current RPM.
- Low SOC (default 20%) latches ICE charging until 80%. Charging occurs with the mechanical transmission in neutral. A separate crank generator, separate traction motor and disconnect-capable transmission are required; unsupported hybrid topology is rejected. ECO can run disconnected ICE for charging.
- Hybrid READY/start-stop, modes, battery/fuel-cell/H2 dashboard telemetry and existing electric motor audio are wired. EV dyno explicitly reports that motor dyno is pending, clearing stale combustion curves. Hybrid engine dyno remains combustion-only.
- Fixed authored hydrogen starter free speed (700 RPM) so the CVT neutral load does not hold cranking below firing speed.

## Validation
- Physics EV tests: 11 cases, 88 assertions passed. Fuel + EV suite: 71 cases, 110009 assertions passed. Includes fuel-cell energy closure, READY/empty/full, all hybrid modes, charge hysteresis to 80%, shutdown and rejection of unsupported hardware.
- Game conversion/gearbox tests: 3 cases, 21725 assertions passed. Covers conversion loading and AT/DCT/CVT electric launch, Sport ICE startup and stationary neutral charging to 80%, followed by ICE shutdown.
- Godot editor parse completed. Existing electric-drive/audio acceptance: 24 checks, zero failures.
- New game/scripts/powertrain_swap_test.gd: 56 checks: actual category buttons plus materialisation of all five packages on sedan, electric sedan and N2O hypercar; zero failures. Uses isolated out/ user/work files.
- Broader garage/automatic/EV/audio/driver regression: 126 cases, 37230 assertions passed (out/hybrid_game_regressions.log). Legacy ICE-only gearbox/layout loops now exclude the EV; trim and option-count expectations respect its explicit catalogue allowlist.

## Remaining work and limits
- No public PR or merge for this batch yet. Game edits are checkpointed locally; physics branch must be pushed before the game submodule pin is published.
- Racing boost preparation is intentionally not implemented; it currently shares Sport ICE keep-running behavior.
- EV/combined hybrid dyno, detailed source thermal/storage models, and physical conversion packaging/fit constraints beyond the four-road-wheel topology remain future work.
- Conversion electronics use hybrid TC/mode control; existing ICE-specific launch/aero automation electronics are replaced. Physical aero remains. Preserve or reintroduce compatible independent aero control in a follow-up.
- Hybrid requested full-pedal system power is an authored package calibration (300 kW), not automatically derived for each swapped engine/upgrade. Extend calibration for proportional pedal-demand behavior across arbitrary high-output engine combinations (full throttle still requests full ICE output).
- Add explicit hybrid respawn/READY cycling and deterministic-worker regressions, save/reload bidirectional conversion acceptance, and direct shift-schedule comparison across ICE startup. Existing mode/gearbox tests cover normal running but not all those transitions.
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

## Already delivered
EV physics PR20 and game PR40 merged; motor audio PR41 merged and installed in pr33_ready. Existing launcher out/run_home_region.ps1, garage Sedan E, K toggles READY. Last accepted EV tests25checks with Windows spatial and Godot fallback; audio native tests2cases. New swap drafts must not regress these.
