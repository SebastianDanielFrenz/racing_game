# Real-route terrain prefetch measurement

Build the `prefetch_probe` target with `relwithdebinfo`, then run
`out/build/relwithdebinfo/tools/prefetch_probe/prefetch_probe.exe [workers]`
from the repository root. Workers default to 4 and must be 1..24.
The tool reads the configured home store and replays the first 3 km of
`data/routes/home_r1_drive.json` at 100 m/s, paced at 240 Hz, off then on.
It writes per-tick CSVs under `out/` and fails on hash differences, missing
inputs, starvation, too few crossings, short routes or loader timeouts.

This is a streaming-focus replay without a vehicle or renderer. It measures
the real height/road fetch, snapshot, build and installation path in isolation;
it does not prove wheel-force, graphics frame-time or full driving acceptance.
The 26 priming ticks and initial ring build are excluded from runtime timing.
`lifetime_wait_max_ms` includes startup; runtime wait counters subtract startup.
Crossing samples cover the changed-focus tick and the next six ticks (the
five newly needed tiles plus settling), using nearest-rank p99.

## Measurement: 2026-10-02

Windows clang-cl RelWithDebInfo, physics b90a531, geo2map 44a3525,
256-sample heightfields, radius 400 m, one fill/tick, ring margin 1/cap 24.
Each run has 7,200 ticks and 15 changed tile centres on the cached home route.

| Workers | Prefetch | Crossing p99 ms | Crossing max ms | Terrain ticks > 4.167 ms | Runtime late waits |
|---|---|---:|---:|---:|---:|
| 4 | off | 4.557 | 4.988 | 10 | 0 |
| 4 | on | 0.022 | 0.027 | 0 | 0 |
| 24 | off | 9.074 | 9.263 | 70 | 0 |
| 24 | on | 0.024 | 0.025 | 0 | 0 |

Both on runs installed 75 prepared shapes, with no stale inputs, loader gate
freezes, fill misses or starved tiles. Every off/on per-tick hash matched.
Startup had 24 late waits, with lifetime maximum waits of 4.497 ms (4 workers)
and 8.140 ms (24 workers). Startup remains a separate optimization opportunity.
These are individual local runs, not a statistical hardware performance bound.

Integration verification: 150 debug tests twice, release CI (150 tests),
RelWithDebInfo tests, normal/bindings Godot smoke and corrected delayed-fetch
real-world smoke. The latter observed one relocation freeze and 726 subsequent
ticks, zero falls/misses/errors. A release drive process once exited without an
assertion report; direct and full-suite reruns passed. A later optimized
relocation test timed out once; its focused rerun passed in 4.7 s. Those two
intermittent failures are recorded without claiming an established cause.
