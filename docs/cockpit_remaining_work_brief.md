# Remaining cockpit work — implementation brief

Baseline: racing_game `e870894`, inspected 2026-10-13. This brief covers the remaining hypercar cockpit-controls, model, interaction and hands plan, including its physics and VR dependencies. It does not replace the unrelated world/traffic/engine/CFD project backlogs.

## Read first and repository rules

- Authoritative game: `S:/claude_code/racing_game`.
- Authoritative physics/model source: `S:/claude_code/physics_sim`.
- Never edit `racing_game/external/physics_sim` or `external/geo2map_engine`; these are read-only imports. Commit authorized model changes in the authoritative physics repository, then update the game pin separately after reviewing the upstream changes.
- Current game imports: physics_sim `8cb0316a31c8623225f61b970b2333ffc07d7797`, geo2map_engine `7d6957703f1febc645ceb710c9d2a6f9920dea41`. Upstream physics HEAD was `fa128a3` at this inspection; do not blindly upgrade to it or assume its pending plans are implemented.
- Read both repositories' applicable instructions. Physics `AGENTS.md` requires `RUNTIME_LOCKS.md` and runtime ownership checks before replacing the demo DLL. This task does not require a physics demo DLL rebuild merely to generate car assets.
- The owner explicitly assigned this session the **car model updater** role. The old plan/brief statements that another session owns M1 are superseded. The physics core is being worked on separately; avoid unrelated core changes and coordinate narrowly if an API is missing.
- The owner confirmed no other session was working on racing_game. Recheck build/runtime ownership before a new build; do not assume this remains true indefinitely.
- Engine-neutral behaviour, layouts, hit tests, gesture interpretation and hands scheduling belong in `rg_core`; Godot adapters convert values, render and forward inputs.
- Dimensions, limits, thresholds and timings belong in data. Existing cabin settings are in `data/controls/cabin.json`; part interaction settings are in `data/cockpits/parts_spec.json` and the generated rig.
- Do not install/download tools or assets without existing authorization. Blender is already installed at `S:/programs/Blender 5.2/blender.exe`. Generated hands avoid third-party asset licensing.
- One build owner and one vault writer at a time. Windows builds use PowerShell, `tools/common.ps1` and `Enter-VsDevShell`. Required release check: `tools/ci.ps1 -Only release -SkipSmoke`.
- Stage a changed submodule pointer before CI: the CI setup can reset imports to their recorded pins.
- Commit each increment separately; no push. Use `Co-Authored-By: GPT-6 <noreply@anthropic.com>` for this model. Leave `game/scripts/traction_control_test.gd.uid` untracked; commit other new script UIDs.
- Never touch `S:/claude_code/racing_game_releases`. A verified dev Godot locking the game DLL may be closed under the earlier owner authorization; report it. Do not terminate unknown/other-session processes.

## Verified completed work — reuse it

| Increment | State and evidence |
|---|---|
| P1 reusable parts | `2c90a38`: 22 parts, 29 joints, 26 hit boxes, 14,388 triangles under the 15,000 budget. Deterministic GLB/rig generation, native asset validation and independent Blender rest/posed sheets. |
| Common P2 visual subset | `781222a`: 18 common mounts in drive and garage, with reflected-clone/idempotency tests. Horn/paddles follow the wheel. Actual sockets override provisional fallbacks. No variable shifter/N2O or interactive controls yet. |
| C0 | `ce0dd77`: native `DriverActions`, shared `ControlState`, thin `RgDriverActions`, keyboard/HUD/cluster/common-lamp integration. Current-channel chaining and eight source kinds tested. |
| C1 park/EPB interfaces | `174fb45`: P requests the fitted physics pawl; shifts release it; cluster P follows actual engagement. Session reserves EPB and transmission channels. EPB itself is unavailable with the imported physics. |
| C1 cabin functions | `fe865f5`: tap start/stop with timed cranking, hazard/indicators and self-cancel, held generated horn, exterior indicator/brake/reverse emission, shared cabin lamps. 550 native tests passed. |
| C2 | `e870894`: native mouse/stick look integration, configurable pitch -60 to +10 degrees, wheel rotation from steer × configured lock, cockpit cursor capture, remappable Delete markers. Release CI 552/552 passed, plus real-window/headless adapter tests and screenshot/pose evidence. |

Current shortcuts: F11 start/stop, F12 hazards, Z/X indicators, held J horn, P park, I ignition, held K starter. **F6/F7 remain seat/traffic panels**; C1's earlier F6/F7 start/hazard defaults were corrected in C2. Middle click toggles cockpit cursor capture; Delete logs a coordinate bookmark. Existing player remaps must remain intact.

Important files:

- `core/include/rg/driver_actions.h`, `core/src/driver_actions.cpp`; `godot_ext/src/rg_driver_actions.*`.
- `core/include/rg/camera_math.h`, `core/src/camera_math.cpp`; `godot_ext/src/rg_camera_math.*`.
- `game/scripts/input_map.gd`, `driver_state.gd`, `cockpit_common.gd`, `exterior_lamps.gd`, `cockpit_rig.gd`, `vehicle_visual.gd`, `vehicle_audio.gd`.
- `data/cockpits/car_hyper_common.json`: provisional placement, not the complete future layout schema.
- `tools/cockpitgen/{build,validate,render_sheet}.py`, `tools/cockpitgen/README.md`; generated parts in `data/models/cockpit_parts/`.
- Reports: `out/c0/c0_report.md`, `out/c1_park_report.md`, `out/c1_cabin_report.md`, `out/c2_report.md`.
- Images: `out/cockpit_parts_sheet.png`, `out/cockpit_parts_posed.png`, `out/cockpit_common_in_car.png`, `out/c1_hazards.png`, `out/c2_cockpit_down.png`.

The original `out/hyper_cockpit_controls_plan.md` has stale “nothing built” and early ownership/layout statements. Use its final **Owner answers** sections, this brief, current code and the vault instead of restarting C0–C2/P1.

## Owner decisions that override early proposals

- Two five-slot tunnel rows, one on each side of the shifter, running along ISO X. Row centrelines Y +0.075 and -0.075 m; slot pitch 32 mm; generic `socket_btn_L1..L5` and `socket_btn_R1..R5`.
- Driver row front-to-rear: TC, LC, AERO, A/M, EPB. Passenger row: ECO, NORMAL, SPORT, RACING, blank.
- Hazard goes on the **dash face between two centre air vents**, VW-style double triangle; the current below-screen fallback is temporary. Add the appropriate vents/furniture.
- Start/stop sits on the tunnel front/waterfall. Guarded N2O sits behind the lever on the tunnel.
- TC and LC only on the tunnel. Horn and paddles are wheel-mounted. Existing column stalk mounts stay fixed to the car/column; do not accidentally rotate the stalks with the wheel because an early plan table grouped them with wheel sockets.
- Shifter follows the installed gearbox: sequential, H-pattern, or BMW-style monostable automatic selector with M gate. Default hypercar is intended to become 7-speed DCT when physics ships it; today it is the assisted-manual/sequential presentation.
- Door handle triggers get-out with a configured low-speed refusal and feedback.
- EPB has **no game-owned latch and no dynamic normal-mode stop**. ECO/NORMAL/SPORT are park-only, refusing at significant speed. RACING/drift uses a direct rear handbrake.
- Simple generated hands; optional simple two-bone arms in VR, **default on**, with comfort confirmed in V3. This overrides the earlier hands-only recommendation.
- Middle click frees/recaptures a normally hidden captured cockpit cursor. Marker action remains remappable, default Delete.

## Recommended order

1. M1 permanent car furniture/sockets and source-asset/CFD validation.
2. Finish P2 native layout, fitted-state/gearbox variants and complete action coverage.
3. P3 flat-screen interaction.
4. V1 visible hands/shared XR sources, then V2 physical interactions/haptics.
5. V3 seat-range reach/comfort audit and owner headset checks.
6. L1 lights/wipers once night/rain support exists; F1/F2 flat-screen hands afterward.

EPB and automatic/DCT physics integration can be added when available; their absence does not block manual/sequential layout, mouse interaction or visible hands. V1 can start before P2/P3 if useful, but keep ownership/build constraints.

## M1 — permanent car furniture and sockets (next)

Work in authoritative physics model tooling, principally `tools/assetgen/assetgen/vehicles/{car_hyper,hyper_interior}.py`, plus scene/rig export and validators as needed. Do not change the engine/traction core.

Deliver:

- Dash centre vents with correctly placed hazard mounting surface; waterfall/tunnel-front start mounting.
- Ten generic tunnel-row sockets and shared shifter/N2O/light/stalk/paddle/horn sockets, with final names matching the layout and generated parts.
- Each socket has identity rest rotation and explicit ISO `normal` and `up` in **both** rig JSON and GLB extras. Document the additive rig contract; validators must check finite, nonzero, nonparallel vectors and matching parent/position/orientation data.
- Remove placeholder lever/knob and two decorative metal knobs **when the replacement shifter is installed**, avoiding an intermediate car with no visible shifter.
- Separate revolute `pedal_clutch`, `pedal_brake`, `pedal_throttle` nodes and independently jointed left/right inner door handles parented to their respective doors. Drive their later animation through actual channels/actions.
- Preserve driver-eye/seat geometry, existing doors/body joints, exterior materials and current camera alignment unless a measured reach/clash issue justifies a documented change.
- Keep horn/paddle sockets under `steering_wheel`; generic button rows and column stalks belong to static cabin/column geometry.

Acceptance:

- Deterministic rebuild and `tools/assetgen/validate.py --determinism car_hyper` from authoritative physics. Confirm build/validator CLI syntax from the scripts before running.
- Update `hyper_topology_check.py`'s intentional node/socket baseline; it currently compares against HEAD and rejects any new nodes/sockets. Do not simply remove topology or pose checks to make additions pass.
- Existing closed visual geometry audits remain green; car triangle/byte budgets remain within their configured guards (last documented hyper guard 185,000 triangles/6.5 MB). If exceeded, make an explicit budget decision before claiming acceptance.
- Independent Blender driver-eye and posed-joint renders; no row/lever/vent/hand-space clashes.
- Regenerate `car_hyper.glb`, `.rig.json` and `.flow.json`. Revalidate final primitive ranges and GLB hash against the game's exporter/CFD selection. Interior-only work still changes GLB SHA; stale flow selections/tables must never be silently treated as valid. Do not fabricate CFD results or set `cfd_ready=true` from visual closure alone.
- Commit upstream model changes, review/bump the game import and verify both drive/garage paths. Check current upstream work before choosing the pin.

## P2 — complete layout, parts selection, state and action wiring

The current `cockpit_common.gd` is a provisional rendering subset. Implement the planned engine-neutral `rg::CockpitLayout` and a thin `RgCockpit`/render adapter; avoid creating a second dispatcher next to `main.get_driver_actions()`.

Deliver:

- A versioned `data/cockpits/car_hyper.json` mapping socket → part → action, local offsets, installed-gearbox presentation and fitted rules. Migrate or clearly delegate the common fallback layout; final supported car sockets must exist and validate.
- Correct shifter variant for installed gearbox, compatible paddles and N2O only when fitted. Blank unavailable controls, including EPB until physically available, and unavailable drive-mode buttons/assists; do not show a functional-looking switch that cannot act.
- Detect gearbox presentation from real installed data. Current game `GearboxKind` has Manual/Sequential/Automatic/Cvt, no Dct. Add DCT support only with an honest source contract. Automatic selector can be rendered using an explicit test fixture but remains unavailable for live driving until physics implements it.
- Extend the generated H-pattern legend/gates if an installed manual has more gears than the current six-speed-plus-reverse asset represents.
- Complete shared action coverage for shift counters, absolute gear/selector requests, drive-mode and TC Set values, indicator/stalk actions, get-out and future lights/wipers. Register cockpit-only absolute actions in `actions.json` without default keys. Existing TC/drive-mode IDs already accept Set values in DriverActions; reuse that implementation or add consistent named aliases.
- Treat `gear_request=-100` as **no request**, not neutral. Neutral is `0`; the old plan table says otherwise and is wrong. Release park when shifting away. `transmission.selector` remains reserved/inert until its physics contract exists.
- Feed effective `ControlState` into all LEDs, selector/gear displays and fitting state; animate push travel, detents, returns, pedals and relevant handles. Read interaction travel/thresholds from the generated rig, not duplicated literals.
- Guarded N2O: cover-gated interaction, separate toggle, closing cover forces arm off. Test the fitted non-N2O/N2O configurations and mechanical state transitions.
- Review input-source parity and conflicts. C2 currently handles capture via mouse-button events; any additional keyboard/pad capture remaps must be honoured consistently without firing a second edge for the same mouse press.
- Resolve config scope: the current cabin settings are shared across cars. Use vehicle/layout overrides where wheel lock, ratio, fitting or geometry differ; do not force hypercar dimensions on a later sedan.

Acceptance: every action exists in the schema; every final socket exists; exactly one compatible shifter; fitted rules and same-frame input chaining; tested Manual/Sequential and honest automatic fixture. Screenshots for sequential hypercar, six-speed H swap and fixture automatic selector; TC and offered drive-mode LEDs; non-N2O has no N2O toggle; unavailable EPB is blank.

## Physics integration follow-ups (coordinate, do not implement in the wrong repository)

### EPB / fourth drive mode

Neither the imported physics nor the inspected upstream core exposed the planned `epb_state`/`epb_refused` contract. The game currently exports `epb_fitted=false`; it must be replaced with actual physical fitment/telemetry when available.

- Channel `epb` is the switch **level**. In park_only mode a rising edge below the part's configured `max_engage_speed_mps` latches; above it the request is refused/dropped while high, requiring a release/repress. Falling edge releases a latch.
- RACING/handbrake mode applies the rear handbrake directly while high.
- Published state: released/latched/handbrake plus refusal; align exact enum/field names with the upstream API before conversion.
- Wire imported data/telemetry and verify real refusal blink/message and rear-handbrake behaviour. Core fake-state mapping already exists; it is not evidence of a working EPB.
- Add car_hyper's offered drive-modes part/fourth mode when physics provides it. Cycling already uses available bits; the cockpit row must show only offered modes.

### Automatic/DCT/CVT and pedal contracts

Coordinate the separate transmission/pedal plan through PHYS-074/PHYS-033 and the physics owner. Do not assume channels/data/kinds exist because a plan describes them. Integrate the actual selector/unlock/park/manual-gate contract and default DCT only once available. A/M remains supported until physics deliberately removes auto-shift for the relevant gearbox; then blank/replace it rather than leaving a dead action. No fake functioning automatic swap.

## P3 — flat-screen clicking and lever manipulation

Implement `rg::CockpitHit`/interaction state in chassis-local coordinates from part hit volumes and joint metadata, exposed through thin `RgCockpit` methods.

- Hover highlight, ray selection and button click.
- Drag sequential/H/selector levers, stalks, EPB and guard/door handles through their configured gates, detents and return behaviour.
- Debounce/hysteresis, approach/active volumes, cover gating and one event per crossing. Keep spring-centred and latching controls distinct.
- Submit to the SAME DriverActions instance with `CockpitMouse` source. Mouse look is disabled while free/clicking; middle click restores capture. No double firing through both GUI and 3D paths.
- Preserve all keyboard/pad controls and menu/seat/traffic/teleport text-input isolation.

Acceptance: synthetic ray per control produces the same writes as equivalent keyboard/Set input; deliberate miss/occlusion/duplicate-edge/drag-boundary tests; hover screenshot and captured/free-cursor smoke. Geometry transforms must handle the moving wheel, doors, seat and floating-origin rebases correctly.

## V1 — visible VR hands and shared tracking sources

- Generate a simple driving-glove hand GLB (left/right, 26 named OpenXR joint segments, no asset download), plus simple arm segments for optional analytic two-bone IK. VR arms default on per owner; provide a way to disable them for comfort.
- One shared controller pair and hand-source owner used by both menu and cockpit. `xr_menu.gd` currently creates its own pair: migrate ownership, preserving menu ray/click/scroll/back/pause functionality and current XR-origin handoff.
- Controllers provide canned/blended open/point/pinch/fist poses; good-confidence XR hand tracking drives real joints. Use valid hand tracker → controller → hidden per hand. Tracking selection, loss/fade/hiding, source transitions and head-gaze fallback must be explicit and tested. Preserve controller haptics only for controller sources.
- Verify the owner's actual runtime capabilities; do not infer tracking support from an action-map profile. Do not repeat the nonexistent `XRCamera3D.get_is_active()` call: controller activity and camera/head pose validity have different APIs.

Acceptance: scripted tracker/controller poses within 1 mm, correct mirrored side, clean source switches/loss handling and no duplicate controller pair. Owner headset check for visibility, latency, arms and tracking capability. Flat/headless tests cannot certify headset visibility/performance.

## V2 — VR poke, grab, ray and feedback

- Use the same native hit/joint/gesture system as P3; no independent Godot-only action implementation.
- Fingertip sphere presses with configured direction cone, travel threshold, release hysteresis and debounce; one control per finger prevents neighbouring presses.
- Grip/pinch/curl grabs, authored hand poses, motion-to-joint projection, detents and release/return for each lever/rocker/handle/guard.
- Short aim-ray fallback and drag when pointing/out of fingertip range; preserve trigger hysteresis and menu ownership.
- Controller pulses from part data for press, detent, guard and refused actions. Bare hands use non-haptic feedback.
- Do not implement VR wheel steering yet: the wheel remains driven by existing steer input. Rim grabbing is a later feature.

Acceptance: scripted fingertip paths press exactly once without neighbours; sequential pull yields one upshift; H neutral/gear semantics; EPB Set levels; guard safety; ray dial/button actions; haptic counts on a stub. Real headset review remains required.

## V3 — reach, visibility and comfort

- Native reach/clash/sight checks over the complete seat-adjustment range, including wheel rotation, installed shifter variant and hands/arms. Use the plan's shoulder/reach assumptions as configurable starting data, not certified anatomy.
- Check controls visible within the intended downward field of view and reachable in a seated/belted posture. Validate arm IK and attachment, side/height extremes, accidental-neighbour activation and glove/control clipping.
- Adjust data offsets first; change a permanent socket only if necessary and repeat M1 hash/topology verification when doing so.
- Owner headset checklist for every control: reached without leaning, first-try actuation, no adjacent actuation, comfortable arms and readable state. Also confirm existing VR spawn selector/garage/menu placement, recenter and origin switching; those earlier fixes were automated, not headset-certified.

## L1 and F1/F2 — later functions and hands

**L1 lights/wipers:** after integrated night/rain/weather support, wire dial/stalk absolute values, flash and one-wipe actions, actual exterior light sources and wiper geometry/animation. Respect per-car fitting and mode state. Test night headlamp images and moving wipers in rain. Existing C1 indicator/brake/reverse emission is complete and should not be rebuilt from scratch. A final licensed/measured horn source is optional later; the generated placeholder already works.

**F1 flat-screen hands:** reuse generated hand/arm meshes; engine-neutral DriverHands targets/finger poses and two-bone IK. Hands follow wheel grip sockets; configurable re-grip for large angles; screenshot 0/90/180 degrees, IK reach/elbow tests.

**F2 flat-screen reach animations:** key/button/gear actions schedule reaches, paddle flicks and returns through shared action events. Busy driving skips or reduces animation; one pending reach, timings and thresholds from data. Actions always apply immediately—animation must never delay an input. Test scheduler/skip/replacement and record examples.

**Beyond this plan:** sedan installation much later with its own sockets/layout; VR hand steering via rim grab; skinned/realistic glove assets only with a separate decision/licence review. Immersive stereo walk-around garage is not implemented by the current monoscopic VR garage panel and is separate optional work.

## Known validation gaps to carry forward

- C0's game hash was shown unchanged by that increment, but game Session `hash_check` versus separate `ps_run` scenario already differed. RACE-020 and `out/c0` hold evidence; investigate before claiming cross-runner parity.
- C2 drive_tour validated wheel poses, but its throttle phase stayed at rest in neutral. Fix/extend the tour's shift sequence if using it as driving-performance evidence; do not label the current image run a successful road drive.
- Headset visibility, controller mapping, reach/comfort and hand capabilities require the owner wearing the headset; no automated result substitutes for those checks.
- EPB/automatic selector functionality, full fitted-state blanking, clickable controls and VR hands remain unimplemented despite existing meshes/state placeholders.
- A fresh machine may not have ignored `out/` plans/images/reports. This brief is tracked; regenerate artefacts from source and use the vault for durable decisions.

## Relevant vault tickets

The authoritative ticket directory is `S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/`. Search and update through the knowledge-manager CLI, never by editing ticket/index files manually. Some ticket bodies predate later increments: read their latest updates and the owner overrides.

| Ticket | Why it matters |
|---|---|
| [RACE-020](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/RACE-020.md) | C0–C2 action/state/cabin/camera implementation, commits, tests, channels, corrected shortcuts, remaining work and hash-parity caveat. Primary game wiring record. |
| [RACE-019](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/RACE-019.md) | P1 generator/rig/interaction/lamp contract; common placement subset, provisional mounts and extraction/idempotency details. |
| [PHYS-005](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/PHYS-005.md) | Assetgen, ISO-to-glTF conventions, geometry/rig validation, deterministic builds, hyper topology repair, budgets and source ownership history. |
| [PHYS-067](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/PHYS-067.md) | CFD coefficient-map provenance, geometry SHA/flow selection, scope and fallback rules; crucial when M1 changes the GLB hash. |
| [PHYS-033](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/PHYS-033.md) | Powertrain, shaft network, park pawl and gear-request context; distinguish real components/contracts from future gearbox plans. |
| [PHYS-074](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/PHYS-074.md) | Assist grades/fitment, TC/LC/aero/drive-mode defaults and physics channel writeback; transmission/pedal follow-up context. |
| [RACE-011](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/RACE-011.md) | Garage setups/installed configurations, node teardown, VR menu/spawn/garage projection and XR-origin handoff. Headset confirmation still pending. |
| [PHYS-057](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/PHYS-057.md) | SteamVR/OpenXR Vulkan Mobile requirement, worn-headset visibility/performance and desktop-focus pitfalls. |
| [PHYS-008](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/PHYS-008.md) | Large-world/floating-origin coordinates; apply one consistent frame/rebase conversion to cameras, hands and hits. |
| [RACE-003](S:/claude_knowledge_base/knowledge-manager/Vault/Tickets/RACE-003.md) | Controller/camera-input history; useful when checking right-stick conventions and input regression cases. |

Example searches (run separately):

```powershell
& S:/claude_knowledge_base/knowledge-manager/gradlew.bat -p S:/claude_knowledge_base/knowledge-manager run --quiet --args="search 'cockpit layout sockets interaction hands' 3"
& S:/claude_knowledge_base/knowledge-manager/gradlew.bat -p S:/claude_knowledge_base/knowledge-manager run --quiet --args="search 'park EPB transmission channel writeback' 3"
```

## Report required for each remaining increment

Give commit(s), authoritative source/pin changes, tests and their limits, screenshots/pose evidence, updated vault ticket(s), remaining dependencies and the precise place reached in this plan. No automatic claim of headset validation, CFD readiness, working EPB or working automatics from placeholder/fixture tests.
