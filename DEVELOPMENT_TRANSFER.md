# Continue development on another machine

The checked-in submodule URLs refer to the original machine's local repositories.
Use the three Git bundles produced by `tools/export_dev_bundle.ps1` to transfer
the game and the exact physics/geo2map revisions without needing those S: paths.
Bundles include Git history, tracked source, models, generated turbo maps and
custom nitrous parts. `manifest.json` records revisions and SHA-256 checksums.

## Restore on Windows

Copy the entire `out/development-transfer` folder to the destination machine.
In PowerShell, adapt these two paths:

```powershell
$transfer = (Resolve-Path 'D:\transfer\development-transfer').Path
$checkout = 'D:\development\racing_game'
git clone (Join-Path $transfer 'racing_game.bundle') $checkout
git -C $checkout submodule init
git -C $checkout config submodule.external/physics_sim.url (Join-Path $transfer 'physics_sim.bundle')
git -C $checkout config submodule.external/geo2map_engine.url (Join-Path $transfer 'geo2map_engine.bundle')
git -C $checkout -c protocol.file.allow=always submodule update --init --recursive
git -C $checkout status --short
git -C $checkout submodule status
```

These local configuration overrides preserve the original machine's setup.
Keep the bundle folder accessible while using the bundle URLs. Do not run
`git submodule sync`, which would restore the original S: URLs. Replace the
local URLs with shared repository URLs when hosting is established. A bundle
is a transfer snapshot, not a server to push future development to.

## Build and run

Install Git, CMake, Ninja, Visual Studio 2022 Build Tools with the C++ toolchain
and clang-cl, and Godot. See `tools/setup_dev_env.ps1` and `tools/common.ps1` for
the existing setup/lookup helpers. Python 3.10+ is needed to regenerate turbo
maps; the generated maps already ship in the repository.

```powershell
Set-Location $checkout
tools\run.ps1 -Flat
```

Build output and engine-map caches are deliberately excluded from Git. The
first build can fetch CMake dependencies, requiring network access. An
existing geo2map dependency checkout can be supplied with
`-DRG_G2M_DEPS_DIR=<path-to-_deps>` during CMake configuration.

## Real-world terrain and personal settings

The terrain stores are large external datasets and are not in these bundles.
For the corridor world, copy
`S:\claude_code\geo2map_cache\rhein-main-corridor-r2`, including its `baked`
directory, to the new machine. For the Hessen world, copy
`S:\claude_code\geo2map_cache\rhein-main-hessen-r1` similarly.
Edit `source_store.dir` and `derived_store.dir` in the corresponding world configuration to
match the destination paths, or make a local
copy of the configuration and select it with `RG_WORLD_CONFIG`.
Flat mode works without those datasets. The bundles do not transfer saved
garage setups, control bindings, logs, the knowledge vault or other personal
application data; copy those separately if wanted.

The increased engine-generation worker build still needs a DLL rebuild if the
original machine's running game prevented installation. Fresh builds compile
the committed change automatically. `RG_ENGINE_MAP_WORKERS` overrides garage
generation worker count (1–64); the default leaves four logical CPUs free.
