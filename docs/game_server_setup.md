# Headless world services

Initial implementation (2026-10-22), branch `codex/game-server`. This is the
environment/tile-service foundation for `docs/game_server.md`, not yet its
S1 multiplayer relay. No Godot process, renderer or vehicle simulation runs.
The executable currently links `rg_core`; splitting its environment dependencies
into a smaller library is a later build optimisation.

## Build and run

Windows, from a Developer PowerShell with clang-cl and Ninja available:

```powershell
cmake --preset relwithdebinfo -DRG_BUILD_SERVER=ON -DRG_BUILD_GODOT_EXTENSION=OFF
cmake --build out/build/relwithdebinfo --target rg_server
out/build/relwithdebinfo/server/rg_server.exe --root . --config data/server/server.json
```

Linux (including native ARM64, using the existing architecture detection; Linux execution has not yet been validated for this increment):

```bash
cmake --preset linux-release -DRG_BUILD_SERVER=ON -DRG_BUILD_GODOT_EXTENSION=OFF
cmake --build out/build/linux-release --target rg_server -j
out/build/linux-release/server/rg_server --root "$PWD" --config data/server/server.json
```

All paths in the config are resolved relative to `--root`, including state,
source and derived stores. Copy the sample config before changing it. Port `0`
selects an available port for tests. `--run-for N` requests a clean stop after
N real seconds. SIGINT/SIGTERM stop cleanly and save. Readiness is logged as
`RG_SERVER_READY port=... instance=... map=...`; errors exit nonzero.

The default config serves only the environment on loopback TCP 47101.
Set `map.enabled` to true and supply an existing store's `source` and `scope`
to serve real tiles too, on TCP 47102. No map data is downloaded automatically.
Source stores are opened read-only. Derived products are written only to the
separate `map.derived` store. Layer flags and world seed determine release ID;
they must match the clients' intended release. `public_url` advertises the
map service's externally reachable **base URL without `/g2m/v1`**; when empty,
it uses the bound address and port (suitable for loopback tests).

The tile and environment listeners use separate worker pools. Slow cold tile
derivations do not block the environment clock or HTTP request workers.
Both listeners share the process lifetime in this first increment; either
startup failure prevents readiness. Geo2map remains independently deployable.

## Read-only endpoints

| Endpoint | Payload |
|---|---|
| `GET /rg/v1/health` | Readiness and process instance ID |
| `GET /rg/v1/server` | Service discovery, map base URL, pinned release ID, feature flags |
| `GET /rg/v1/environment` | One coherent clock/weather snapshot |
| Map listener: `/g2m/v1/...` | Existing geo2map manifest, releases, tiles, packs, hashes; GET/HEAD, ETags |

Environment replies use `Cache-Control: no-store`. `server_us` is monotonic
microseconds since this process started, sampled with the game clock at up to
20 Hz. `game_seconds_j2000` is the corresponding game time in seconds since
J2000 UT; extrapolate with `rate` and the elapsed server time. The local civil
date/time and UTC offset are also included. The `weather` field is the existing
`rg.weather_broadcast/1` message, with transition origin/duration, next change
and revision. `air_temperature_c` is derived by the existing climate model.
Clients must discard their clock/revision history when `instance_id` changes.
Ping-based clock estimation and an actual game client adapter are still pending;
polling this API alone does not synchronize existing game sessions.

There are no remote mutation/admin endpoints. HTTP is plaintext; hosted access
should use the planned private WireGuard network or an appropriate reverse
proxy. Binding listeners beyond loopback is an explicit config choice.

## Persistence

The server saves at startup, every `checkpoint_interval_s`, and on clean exit.
Checkpoint replacement is atomic; a crash may lose time since the last save.
Restart resumes the saved civil instant without counting downtime. The seeded
weather chain is reconstructed from its original epoch, retaining scheduled
transitions rather than starting a new forecast on every restart.

An OS file lock prevents two instances using the same state file. Corrupt
state or changed sky/climate/schedule/location input fails startup explicitly.
Use a different `state_file` to start a new world after intentional content
changes; do not silently discard the old world's state.

This checkpoint currently covers only the clock and weather schedule. Physics
surface-condition grids, traffic and players are not persisted or replicated.
The plan's SQLite store will replace/extend this initial JSON checkpoint when
those entities are implemented.

## Verification

```powershell
python tools/server_smoke.py --root . --exe out/build/relwithdebinfo/server/rg_server.exe
python tools/server_smoke.py --root . --exe out/build/relwithdebinfo/server/rg_server.exe --source S:/claude_code/geo2map_cache/home-r1 --scope home-2026-09r1
```

The second command tests a real home-r1 tile, its ETag and release discovery.
All checkpoints/derived data use temporary directories; player saves and source
store contents are untouched. The test checks clock progression, weather,
exclusive checkpoint ownership, restart continuity, rejected mutation and
corrupt-state failure. Existing `[weather_authority]` tests cover the seeded
schedule and client broadcast reconstruction in more depth.

Next increments: client endpoint/configuration and time synchronization;
environment content identity/checkpoints; ENet player state relay and remote
visuals (S1/S2); shared traffic; authenticated administration and persistence.
