# Continue development on another machine

The normal route is now GitHub: clone `racing_game` with `--recursive` and run
`setup.cmd` - see README.md. The submodule URLs in `.gitmodules` are the GitHub
repositories. The bundles below remain an offline alternative.

Use the three Git bundles produced by `tools/export_dev_bundle.ps1` to transfer
the game and the exact physics/geo2map revisions without network access.
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
Keep the bundle folder accessible while using the bundle URLs. Running
`git submodule sync` switches them to the GitHub URLs in `.gitmodules`. A bundle
is a transfer snapshot, not a server to push future development to.

## Build and run

Run `setup.cmd` (installs the toolchain via winget - see README.md). Python 3.10+ is needed to regenerate turbo
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
directory, into a `geo2map_cache` folder next to `racing_game` on the new
machine, or anywhere with `RG_G2M_CACHE` set to the folder holding it. For the
Hessen world, copy `rhein-main-hessen-r1` the same way. The home world's store
goes to `racing_game\cache\g2m\home-r1` (or set `RG_G2M_HOME`).
Flat mode works without those datasets. The bundles do not transfer saved
garage setups, control bindings, logs, the knowledge vault or other personal
application data; copy those separately if wanted.

The increased engine-generation worker build still needs a DLL rebuild if the
original machine's running game prevented installation. Fresh builds compile
the committed change automatically. `RG_ENGINE_MAP_WORKERS` overrides garage
generation worker count (1–64); the default leaves four logical CPUs free.
