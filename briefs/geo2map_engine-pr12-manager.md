# geo2map_engine #12: Manager: open issues and feature requests

This is a standing DRAFT "manager" PR. It carries no code. Its only file is the tracker `docs/manager.md`, which lists what geo2map work is in progress, what waits on Sebastian, what is next, documentation debt and what was done recently. New requests/questions are posted as comments on this PR; every real piece of work gets its own PR, linked from the tracker. The tracker is updated in this branch whenever an item changes state. Merging is optional and would only land the tracker file. A developer sees a single current to-do list; a player sees nothing.

## State at hand-off

| Field | Value |
|---|---|
| Repo | SebastianDanielFrenz/geo2map_engine |
| PR | https://github.com/SebastianDanielFrenz/geo2map_engine/pull/12 |
| Head branch | `claude/manager` |
| Base branch | `main` (not stacked) |
| Head sha | `9c3a830` ("manager: #16 merged"); PR record read as 9c3a8307 |
| Draft/ready | Draft, open |
| Mergeable | clean (GitHub `mergeable_state`) |
| Size | 1 file (`docs/manager.md`), +61 / -0, 12 commits |
| Branch vs main | main has moved on past the branch point (inferred: `main` is not an ancestor of the branch); only a tracker file, so rebase/merge is trivial |
| Submodule pins | geo2map_engine has no submodules. `bridges/physics_sim/physics_sim.pin` = `b90a531f`. For reference, racing_game `origin/main` pins external/geo2map_engine `3092d66b`, external/physics_sim `1e07c0e8` (not checked against submodule default branches here) |

## What is done
- `docs/manager.md` written with five sections: In progress, Waiting on the owner, Next up, Documentation debt, Done recently (as of 2026-10-16).
- "Done recently" lists: #16 G4 lane hints (turn:lanes); #14 G4 routing slice 3 (turn restrictions in `roads.graph`, encoding 6, via-way rules); #15 P2 slice 11 render class window from `g2m.terrain.class` (D62); #13 underpass decks (D53-D61); #11 long tunnels follow one straight grade (D52, 200 m); #9/#10 sett/paving junction split; #8 G6 protocol v1 over HTTP + `g2m_server` sidecar; #3-#7 terrain migration phases 3a-6.
- "In progress" table is currently EMPTY (the underpass work it originally named was merged as #13).
- Cloud session posted the underpass-census instructions as a comment (see below).

## What is unfinished / next steps (ordered)
1. Sebastian runs the underpass census (below) and pastes `census.txt` (or the last count + `underpass_census` lines) as a PR comment. Then the agent de-duplicates by way pair (crossings near tile edges appear in two tiles) and marks which crossings D61 tag rules (parking storeys, covered tunnels) remove.
2. Phase 7 carve switch: run the real-store roadcmp probe (terrain_migration_plan.md, "Phase 7: carve switch") after warming the stores. racing_game deletes its own profile completion only when the probe shows zero game-only entries (standing rule).
3. Decide racing_game over HTTP (opt-in `g2m_server` sidecar vs in-process server); PR #8 left it open.
4. Work the "Next up" list in the order Sebastian confirms: G4 routing follow-ups (timing on the real store); long tunnels split over several OSM ways (D52 per-stretch default kept in #11); Koenigstein `roads.geom` tile `utm32n/2/452/5428` that the deriver rejects (racing_game logs `RG_ROAD_SURFACE unavailable reason=deriver_rejected`); racing_game consumer switches (class window opt-in is racing_game #27 draft; served carved heights = phase 7); G3-E/F (ribbons, markings, bridge bodies, 20-junction route acceptance); P2 slices 10-12 + P3 markings; G5 land cover/buildings/water/vegetation; G7 hosted server (TLS via reverse proxy, auth/rate limits, HTTP upstream chaining, "503 while queued", CDN/static-pack upload).
5. Documentation debt (pipeline/README.md section 9): items 5 (stale CLAUDE.md parts), 6 (PLAN.md vs terrain code), 7 (PLAN.md vs CLI), 9 (unify height quantiser), 11 (physics_sim pin values), 12 (are roads baked for home store), 14 (curvature exceptions decided?).
6. Keep updating `docs/manager.md` as PRs land; refresh before merging.

## Open questions for Sebastian
Still unanswered (from PR body):
1. Is the "Next up" order right, or should something move to the front? (No recommendation given in the PR; inferred sensible default: finish the two probes, then Koenigstein tile rejection, since it is a concrete bug.)
2. The two probes need the home store on his machine. Can he run them, or should an agent prepare one script that runs both and writes one log? Partly answered: he asked for the census command (below); the roadcmp script offer is still open. Recommendation: prepare a single script.
3. racing_game over HTTP (tracker, "Waiting on the owner"): unanswered.

Already answered (decisions; do not re-ask):
- He wants the underpass census command spelled out: which shell, which commands, which commit (comment 2026-10-15 21:41). It was provided (below). He has not yet posted census results.
- Earlier (tracker): per-stretch D52 default for long tunnels kept (#11); underpass mask with deck limited to the crossing span approved on racing_game #5.

## Owner decisions and rules from review comments
- Tracker rule: requests/questions go as comments on this PR; code goes in separate PRs; decisions keep D-numbers (e.g. D52) and are recorded in the owning design doc as well.
- Standing rule: racing_game deletes its own profile completion only after the real-store roadcmp probe shows zero game-only entries.
- No inline review comments or reviews exist on the PR.

## Known issues / risks
- Tracker "In progress" is empty; the PR description still says the underpass work is in progress (stale, #13 merged).
- PR body says "Done recently: #8 through #11"; the file has since grown to #3-#16. Update PR description if merging.
- Census scope caveat (from the comment): it lists all geometric crossing candidates and does NOT apply D61 tag rules, so it is a "before" set.
- The census box (x 438..472, y 5412..5460) is stated to cover E 448-484 km, N 5541-5591 km; those two ranges do not obviously match the tile indices (inferred; level-2 tiles are 1024 m, so check the numbers before trusting coverage). Tiles outside the store fail fast and are skipped.
- Census script's `$store` is a guess (`S:\claude_code\racing_game\cache\g2m\home-r1`); set it to whatever `RG_G2M_HOME` points at.

## Local build, run and verify on Windows
Developer PowerShell for VS 2022 (same as `tools\ci.ps1`). Commit named in the comment: geo2map `main` = `16a8ec6` (includes #13). Note: current origin/main has since moved to the merge of #16 (`cd76bcb`), so verify with `git log --oneline -1` and use whatever main is, or check out `16a8ec6` for an exact match.

```powershell
cd S:\claude_code\geo2map_engine
git switch main; git pull
cmake --preset release
cmake --build --preset release --target g2m_road_probe

$exe   = "S:\claude_code\geo2map_engine\out\build\release\apps\g2m_tiler\g2m_road_probe.exe"
$store = "S:\claude_code\racing_game\cache\g2m\home-r1"   # = RG_G2M_HOME
$out   = "S:\claude_code\underpass_census"
New-Item -ItemType Directory -Force $out | Out-Null
Remove-Item "$out\census.txt" -ErrorAction SilentlyContinue
foreach ($x in 438..472) { foreach ($y in 5412..5460) {
  $t = "utm32n/2/$x/$y"
  & $exe $store home-2026-09r1 $t "$out\t_${x}_${y}.csv" --underpasses 2>$null |
    Select-String '^underpass' | ForEach-Object { "$t $($_.Line)" } |
    Add-Content "$out\census.txt"
}}
(Select-String '^\S+ underpass ' "$out\census.txt").Count
```
- Needs the map store (home region). Read-only on the store. About 1-2 s per tile with roads; under an hour total (per comment).
- Output: one `underpass_census` line per tile (crossings, same-layer, short-side, count dipping >0.3 m, max dip) and one `underpass` line per crossing (upper/lower way, lower layer + tunnel flag, dip, lat/lon). Post `census.txt` back on the PR.
- Roadcmp probe: see docs/terrain_migration_plan.md phase 7 for the command (not reproduced in the PR).
- CI for the repo: `tools\ci.ps1`. A docs-only PR needs no build.

## What was NOT verified in the cloud
- Neither probe was run (they need the real home store); no census numbers exist.
- The PR is docs only; nothing was built or tested for it. The census script has not been run by anyone (cloud had no store or Windows).

## Files to read first
1. `/home/user/geo2map_engine/docs/manager.md` (on branch `claude/manager`)
2. `docs/terrain_migration_plan.md` (phase 7 carve switch)
3. `docs/pipeline/README.md` (section 9, documentation debt)
4. `docs/routing_design.md`
5. `docs/street_fidelity/plan.md` and `docs/street_fidelity/p2_junction_design.md`
6. `tools/road_probe/` (source of `g2m_road_probe`)
7. racing_game PR #5 and #27 (consumer side)
