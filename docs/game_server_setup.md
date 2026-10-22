# Headless world services

Initial implementation (2026-10-22), branch `codex/game-server`. This is the
environment/tile-service foundation for `docs/game_server.md`, not yet its
S1 multiplayer relay. No Godot process, renderer or vehicle simulation runs.
The executable currently links `rg_core`; splitting its environment dependencies
into a smaller library is a later build optimisation.

## Build and run

Windows, from an ordinary PowerShell:

```powershell
tools/run_server.ps1
```

The launcher enters the Visual Studio toolchain environment, configures with
absolute compiler paths, builds only `rg_server`, and launches it. Use
`-BuildOnly` to build without launching, `-SkipBuild` to launch the existing
binary, or `-RunFor 5` for a bounded smoke run. If a previous direct CMake
configure left a broken compiler cache, repair it once with
`tools/run_server.ps1 -Fresh`. `-Fresh` resets CMake cache options; it retains
compiled objects and server world data.

Manual commands, only after entering a Developer PowerShell with clang-cl available:

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
The game client adapter below uses these snapshots with a minimum-RTT estimate; the later UDP clock protocol remains pending.

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

Next increments: UDP clock synchronization and player replication;
environment content identity/checkpoints; ENet player state relay and remote
visuals (S1/S2); shared traffic; authenticated administration and persistence.

## Game client (2026-10-22)

Build the extension from the same `codex/game-server` checkout (its geo2map pin
includes the additive remote `TerrainService` transport seam):

```powershell
cmake --preset relwithdebinfo -DRG_BUILD_SERVER=ON -DRG_BUILD_GODOT_EXTENSION=ON
cmake --build out/build/relwithdebinfo --target rg_server rg_godot
$env:RG_SERVER_URL = 'http://127.0.0.1:47101'
tools/run.ps1 -SkipBuild
```

Alternatively, set **Settings > Network > World server URL**. It applies when
the next world loads. An explicit `RG_SERVER_URL` overrides the saved setting.
Clear the setting and remove that environment variable to use local single
player again. The endpoint is a plain HTTP **base URL**, without `/rg/v1`.
HTTP support now builds with the game client as well as the dedicated server.

While connected:

- The sky, lighting and physics weather follow the server clock and broadcast.
  HTTP polling runs on a dedicated worker, not on a rendering or physics tick.
- The client checks that its sky and climate files match the server's advertised
  content. It uses the authority's climate location. Mismatches stay disconnected
  rather than quietly producing different weather.
- Time/weather menus and the time slider are read-only. The weather widget shows
  connection status; its status tooltip contains failure details.
- Clock samples use a minimum-RTT midpoint estimate. Later corrections hold or
  advance the clock, never rewind it within one server instance. A restarted
  server establishes a fresh clock/revision namespace.
- After three seconds without a valid snapshot the clock and weather hold. They
  resume after reconnection. This is initial HTTP synchronization (500 ms polling,
  up to 50 ms server snapshot age), not the later UDP clock protocol.
- Real-world initialization discovers the map URL and **pins its release ID**.
  All terrain, source OSM, road and rail requests use the existing asynchronous
  tile scheduler over HTTP. Remote mode does not open a local source store or
  derive missing server tiles locally. Missing required layers fail explicitly.
- Validated tile containers are cached by release/layer/tile in a bounded 5 GiB
  disk cache under `<derived_store.dir>/remote`. Already cached tiles remain
  usable through an outage. New world initialization still needs discovery and
  the release manifest; it currently requires the server to be reachable.

Enable `map.enabled` on the server for real-world driving; environment-only
servers can still supply time/weather to the flat world. The selected client
world config must describe the intended region/session frame and have its
required layers enabled on the server. This increment does not distribute a
world config, surface-condition checkpoints, traffic or other players.

Acceptance (requires an existing home-r1 source store and Godot console):

```powershell
cmake --build out/build/relwithdebinfo --target rg_world_client_probe
python tools/world_client_smoke.py --root . --source S:/claude_code/geo2map_cache/home-r1 --godot <godot-console.exe> --full-startup
```

This harness uses only its own temporary checkpoints, caches and processes. It
checks remote terrain without a local source store, the actual `RgSky` binding,
authoritative controls, cache availability after server shutdown and stale-clock
hold. The optional full startup also initializes the actual physics terrain
gate. `--bindings-test` and the existing weather/settings/terrain unit tests
cover the local single-player path. Linux runtime acceptance is still pending.
