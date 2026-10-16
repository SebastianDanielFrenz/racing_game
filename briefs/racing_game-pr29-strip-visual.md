# racing_game #29: Strip visual: show weight-reduction removals on the car

Before this PR the 11 weight-reduction options (driver/passenger/rear seats, audio, door trim, dashboard, sound deadening, carpets, spare wheel, fasteners, bonnet) only changed car mass. Now a data file (`data/vehicles/strip_visual.json`) says per car model which model nodes each removed part hides, in the drive, the cockpit and the garage (car and its mirror twin). A hidden cockpit control can no longer be clicked; undoing a choice shows the nodes again. The weight-reduction option for a seat is now offered only if the car has that seat (catalog `interior` map, `rg::part_choice_offered`). Lighter same-shape parts (carbon bonnet, titanium fasteners, bucket seats) change nothing on screen. cargen now builds removable parts as separate nodes, but NO regenerated .glb is committed yet: most of the visible effect appears only after you regenerate (section 7).

## 1. State at hand-off

| Item | Value |
|---|---|
| Repo | SebastianDanielFrenz/racing_game |
| PR | https://github.com/SebastianDanielFrenz/racing_game/pull/29 |
| Head | `claude/strip-visual`, sha `b522ab3` (17 commits) |
| Base | `claude/part-mass` (PR #22 branch) - STACKED. PR metadata records base sha 8344bda, but `origin/claude/part-mass` has since moved to `88896b3` (it merged #27, #28 and plan-work-q2a5oy). |
| Draft/ready | Draft |
| Mergeable | GitHub says "unknown"; owner commented "Resolve conflicts" (2026-10-16 14:45). My `git merge-tree` of current part-mass + strip-visual: ONE conflict, `data/vehicles/catalog.json` (setup_options.json auto-merges). |
| Size | 45 changed files per PR API (+3371 / -86); 3-dot diff vs current part-mass lists ~57 files because part-mass moved |
| physics_sim pin | `f38075e` (same on head and base). It is on physics_sim `origin/master` (and on claude/block-cooling, claude/cooling-capacity) = default branch, merged (PR #12). |
| geo2map_engine pin | `2571f43` (same on head and base). On geo2map_engine `origin/main` = default branch. |

## 2. What is done

- Core (Godot-free): `core/include/rg/strip_visual.h`, `core/src/strip_visual.cpp` (loader + hiding rules); `Garage::working_hidden_nodes()`, `DriveSelection::hidden_nodes` (garage.h/.cpp); `rg::part_choice_offered`, catalog `interior` map (vehicle_catalog.*, vehicle_setup.*); `Garage::effective_setup` drops a saved choice for a slot the car lacks; `compile_setup` skips it instead of failing. `rg_godot` binding: `godot_ext/src/rg_garage.cpp`.
- Data: `data/vehicles/strip_visual.json` (format `rg.strip_visual/1`; per model `rules` {slot, part, hide, optional `after_regeneration`}, plus `not_separate` explanations); `catalog.json` interior maps; `setup_options.json`, `part_masses.json` changes; `car_*.json` small edits.
- GDScript (visibility only): `game/scripts/model_options.gd`, `vehicle_visual.gd`, `body_visuals.gd`, `garage_scene.gd`, `cockpit_controls.gd` (hidden control not clickable), `main.gd`, `shell_ui.gd`, `car_mesh_merge.gd`.
- What hides TODAY with the committed models: Pico audio removed (screen_center, steering-wheel volume +/- and source buttons); Ventura GT audio removed (5 radio presets, volume and tune knobs); Runabout T nothing; sedan family, hypercar family, Toyama XR, generic SUV, Phoenix 455 nothing (single merged body mesh, not our models).
- cargen source (`tools/cargen/cargen/common.py`, `cars/pico.py`, `ventura_gt.py`, `runabout_t.py`) emits new nodes with rig extra `strip`:
  - Pico: seat_driver, seat_passenger, seat_rear, door_card_FL/FR/RL/RR, carpets, sound_deadening, dash_console, dash_glovebox
  - Ventura GT: seat_driver, seat_passenger, door_card_L/R, carpets, sound_deadening, dash_console, dash_glovebox, spare_wheel
  - Runabout T: seat_rear, carpets, dash_glovebox, spare_wheel
  - Rules for these are marked `after_regeneration: true` in strip_visual.json; a node the loaded model lacks is silently ignored.
- Seat options: Runabout T front bench is one part (new `seat_vintage_bench`, 18 kg in the driver-seat slot, same total mass) so no passenger-seat removal there; Ventura GT has no rear seats (14 kg rear-deck trim moved into body mass, same total); two-seat hypercars already had none; Phoenix 455 keeps rear seats (single mesh, cannot tell). Pico spare wheel not modelled (under boot floor). Stripped dashboard keeps dash body, gauges, switches, climate knobs, gear lever.
- Tests (from the PR body, Linux release, on a merge of part-mass 8344bda + plan-work-q2a5oy 07581be): `tests/unit/test_strip_visual.cpp` `[strip_visual]` 7 cases (loader, rules, shipped data vs setup_options.json, catalog slot coverage, rig files, cargen sources, garage/drive hand-over) and two `[seats]` cases; `test_part_mass.cpp` updated (slot the car lacks is ignored); full rg_unit_tests 758 cases / 224328 assertions passed; `game/scripts/cockpit_test.gd` headless 224 checks 0 failures; `hash_check` vehicle_step_steer `0xace4d38ea725e065` unchanged. Author also ran `build.py` and `validate.py` locally for all three cars (closed shells OK) and restored the model files afterwards.

## 3. What is unfinished / next steps (ordered)

1. Resolve the merge conflict with the current `claude/part-mass` (conflict in `data/vehicles/catalog.json`; the PR text notes the Runabout's option list drops `trim_passenger_seat` and gains the four `cooling_*` options from #28 - keep both sides' intent). Re-run `[strip_visual]`, `[seats]`, `[part_mass]`, `[vehicle_catalog]`, `[vehicle_setup]` afterwards.
2. Build `rg_godot` on Windows (never built in the cloud) and run the full unit suite.
3. Regenerate the three generated cars (section 7) and commit all files together.
4. Look at the result in a rendered Godot window: garage (car + mirror twin) and drive/cockpit view with each removal; check placement of new nodes (spare wheel on the tail, glovebox lids), that seat nodes sit where the old seats were, and that a removed audio/dash node disables clicks in the cockpit.
5. Take before/after screenshots and upload to PR-images (section 8); then take the PR out of draft.
6. Garage thumbnails (`car_thumbnails.gd`) still show the stock car - known, not addressed (only decide whether to fix).

## 4/5. Questions for Sebastian

Open question, never posted in the PR yet: should the imported Phoenix 455 keep a rear-seat removal option? Today it keeps its rear seats, because its single merged mesh cannot show whether it has any. Post this question in the PR with a recommendation (suggested: keep it without the option until the model is split into parts) and wait for Sebastian's answer. The other open item is his "Resolve conflicts" request (step 1).

Already answered (decisions, do not re-ask; his comment 2026-10-16 12:35, numbered as in the PR's question list):
1. Climate knobs on a stripped dashboard: "dunno" - so they STAY; stripped dashboard hides only the new console and glovebox nodes.
2. Bonnet tint: no.
3. A seat the car does not have must not be removable ("the passenger seat should not be removeable if a passenger seat does not exist").
4. Yes - cargen splits out carpets, sound deadening, dash console/glovebox and spare wheel as their own nodes.
5. Phone button: no.

## 6. Owner rules that apply beyond this PR

- A weight-reduction/removal option exists only for parts the car really has.
- Cars not made by us (physics_sim models, imported cars) are not modified: if their model is one mesh, nothing is hidden (explained in `not_separate`).
- Stacked PR: base is #22's branch `claude/part-mass`; merge order #22 (and whatever it needs) before this.

## 7. Known issues / risks and the model regeneration (be precise)

Which models must be regenerated: exactly the three cargen cars - `pico`, `ventura_gt`, `runabout_t`. Nothing else (sedan, hypercar, Toyama XR, generic SUV, Phoenix 455 are not cargen output). Per car, regenerate and commit ALL of: `data/models/<id>/<id>.glb`, `<id>_lod1.glb`, `<id>_lod2.glb`, `<id>.rig.json` (carries the `strip` extras), `<id>.cockpit.json`, `<id>.flow.json` (pinned to the glb sha256, so it must change with the glb). The PR says commit glb, rig, cockpit and flow files together.
- LFS note (inferred from the repo, not stated in PR): on `claude/strip-visual` `.gitattributes` marks only `data/models/downloaded/*.glb` and `data/models/imported/*/*.glb` as LFS; the cargen glbs under `data/models/<id>/` are checked in as plain binary glTF (I read the committed pico.glb: real glTF content, not an LFS pointer). The PR itself says regeneration "needs the LFS checkout" - presumably because cargen imports physics_sim assetgen / some inputs. Run `git lfs pull` anyway before regenerating. The cloud could not push LFS objects; Windows checkout is where this must be done and pushed.
- Do not commit until `validate.py` is clean (closed shells, flow, cockpit) and `--determinism` passes.
- After regeneration the unit test `[strip_visual]` rig-file case reads the rig files: re-run it; it may also confirm that `after_regeneration` nodes now exist (inferred). If so, consider dropping the `after_regeneration` flag from strip_visual.json (inferred cleanup; the flag only documents status, a missing node is ignored).
- Merge conflict in catalog.json (above). Base moved after the PR's recorded base.
- Runabout T has no committed-model effect at all until regeneration.
- The strip decision depends on node names: if regeneration renames anything, `strip_visual.json` rules go silently dead (missing nodes are ignored) - check by eye.

## 8. Local build, run, verify on Windows

```powershell
git fetch origin; git checkout claude/strip-visual
git merge origin/claude/part-mass          # resolve data/vehicles/catalog.json
git -c protocol.file.allow=always submodule update --init --recursive
git lfs pull
cmake --preset debug -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build --preset debug
ctest --preset debug --output-on-failure
out\build\debug\tests\unit\rg_unit_tests.exe "[strip_visual]"   # path may differ; also "[seats]" "[part_mass]" "[garage]"
python tools\cargen\build.py pico ventura_gt runabout_t
python tools\cargen\validate.py pico ventura_gt runabout_t --determinism
git status data/models                       # expect the 3 cars x (glb, lod1, lod2, rig, cockpit, flow) changed
tools\smoke_test.ps1 -Garage                 # headless garage flow
godot --headless --path game --script res://scripts/cockpit_test.gd   # expect 224 checks, 0 failures
tools\ci.ps1                                 # full debug+release+smokes
tools\run.ps1                                # visual check: Garage -> pick Pico/Ventura GT/Runabout T -> Configure -> Weight reduction options; drive and cockpit view
```
Optional preview: `python3 tools/cargen/preview.py data/models/pico/pico.glb out/cargen_preview/pico` (software render) or the Godot render command in `tools/cargen/README.md`.
Needs no map store (RG_G2M_HOME): everything above. Needs the regenerated models: the visual checks. A real-world drive is not needed.
Screenshots to take (before = current glb, after = regenerated): each car in the garage with passenger seat, rear seats, door trim, carpets, sound deadening, dashboard, spare wheel removed, vs stock; Pico/Ventura cockpit with audio removed (screen and buttons gone). Upload to racing_game branch `PR-images` in folder `strip-visual/` and link as `https://github.com/SebastianDanielFrenz/racing_game/blob/PR-images/strip-visual/<file>?raw=true`. Put any question for Sebastian in the PR, not chat.

## 9. NOT verified in the cloud

Nothing was run in a rendered Godot window; no screenshots. `rg_godot` GDExtension not built. The new cargen nodes were never loaded in the game; placement of the spare wheel and glovebox lids not checked by eye. Tests ran on Linux release on a merge of part-mass 8344bda + plan-work-q2a5oy 07581be, not on the current part-mass tip 88896b3 and not on b522ab3 itself as merged with the owner's conflict-free state. No Windows build, no map store, no LFS models.

## 10. Files to read first

1. `data/vehicles/strip_visual.json`
2. `core/include/rg/strip_visual.h` and `core/src/strip_visual.cpp`
3. `tests/unit/test_strip_visual.cpp`
4. `game/scripts/model_options.gd`
5. `core/include/rg/garage.h` (`working_hidden_nodes`, `DriveSelection::hidden_nodes`, `effective_setup`)
6. `tools/cargen/cargen/common.py` and `tools/cargen/cars/pico.py`
7. `tools/cargen/README.md`
8. `docs/garage_ui.md` (strip-visual section added by this PR)
9. `data/vehicles/catalog.json` (conflict file; `interior` map)
