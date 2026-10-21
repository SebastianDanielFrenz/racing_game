# A66 profile boundary diagnostic

Reproduces the reported depression and missing westbound continuation at UTM32N E463374 N5549746 using the actual TerrainService completion pipeline. Source store is read-only; cache and CSV output go to the supplied output directory. No source database is included.

From the repository root, in a compiler environment:

```powershell
cmake -S tools/diagnostics/highway_profile_probe -B out/highway_profile_probe -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build out/highway_profile_probe --target highway_profile_probe --parallel 12
out/highway_profile_probe/highway_profile_probe.exe S:/claude_code/geo2map_cache/rhein-main-hessen-r1 rhein-main-hessen-2026-10r1 out/highway_boundary_repro
```

Arguments are source-store directory, source scope, output directory. Use a private output/cache directory when comparing pipeline revisions. The probe uses the pinned geo2map submodule.

Expected current failure:

- Way 313879319: MASK_STAT crossings=1, served=0, reaches_end=1.
- Way 323855192: MASK_STAT crossings=0; its served entry has no profile.
- A tunnel-tagged service road passes below the motorway near the join. Underpass mask support extends into an OSM way boundary, where the current clearance rule rejects it. The next way sees no crossing and cannot recover the missing support.

Outputs: served profile presence/declines, profile samples near the driven line, bare DEM cross-sections, and nearby source-way tags. Cross-section stations use an approximate straight reference, so they are diagnostic samples, not a wheel-path replay.

Required fix: reconstruct shared, tile-invariant endpoint support and masking across the connected way boundary. For a lower road tagged tunnel, D61 calls for filling the upper road over the tunnel, not serving an exposed bridge deck. Keep ordinary-road endpoint and slope policies intact. Test both way coverage, shared endpoint height/grade, tilted terrain and ownership invariance before publishing a terrain fix. Merely disabling end clearance or increasing grade limits is insufficient.
