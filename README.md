# racing_game

An open-world racing game on real map data (Godot 4.7), built on:
- `physics_sim` (vehicle and rigid-body physics), pinned as a submodule;
- `geo2map_engine` (map layers, road graph, map server), pinned as a submodule.

The home region comes first: Main-Taunus-Kreis, Hochtaunuskreis and
Frankfurt-Höchst. The design and milestone plan for both repos is in
`PLAN.md`. This is a private project, and all rights are reserved.
Map data attribution is listed in-game under credits.

## Install or update

One script per platform downloads the latest commit on `main`, or one
specific commit on any branch. It builds the game and leaves a launcher and an
updater behind. Re-running it updates the install, and it never touches local
changes or a branch you switched to yourself.

**Windows** (PowerShell; installs Git and the build toolchain via winget if needed):

```powershell
irm https://raw.githubusercontent.com/SebastianDanielFrenz/racing_game/main/tools/install.ps1 -OutFile $env:TEMP\rg_install.ps1
powershell -ExecutionPolicy Bypass -File $env:TEMP\rg_install.ps1
```

- Default folder: `%LOCALAPPDATA%\racing_game`. Change it with `-InstallDir D:\games\racing_game`
  (or the `RG_INSTALL_DIR` environment variable).
- Install a specific commit with `-Commit <sha>`.
- Afterwards, `<install dir>\.local-install\play.cmd` starts the game; it takes the arguments of
  `tools\run.cmd`, e.g. `-Flat`, `-Drive` or `-VR`.
- `update.cmd` in the same folder updates to the latest main; with `-Commit <sha>` it switches
  to that commit instead.

**Linux** (ARM64 incl. Steam Frame, or x86_64; needs git, CMake, Ninja, clang (or another C++20
compiler), curl, unzip and Python 3, and fetches Git LFS itself if it is missing;
`--install-deps` installs the rest through apt or pacman):

```bash
curl -fsSLO https://raw.githubusercontent.com/SebastianDanielFrenz/racing_game/main/tools/install_linux.sh
bash install_linux.sh
```

- Default folder: `~/racing_game`. Change it with `--install-dir PATH` (or `RG_INSTALL_DIR`).
- Install a specific commit with `--commit SHA`.
- The script downloads its own Godot. It adds a Steam shortcut; skip that with `--no-steam`.
- Start the game with `~/racing_game/.local-install/launch.py`, and update with
  `bash ~/racing_game/.local-install/update.sh`. The updater also accepts `--commit SHA`.
- The launcher defaults to VR on ARM64 and flat-screen on x86_64; `--desktop` or `--vr` changes
  that. `bash install_linux.sh --help` lists all options.

A short SHA works for a commit that is on a branch. For a commit that is on no
branch (an unmerged pull request), give the full 40-character SHA. Map data is
not part of either install (see below). The flat test world runs without it.

## Set up a development checkout (Windows)

1. Install Git if it is missing: `winget install --id Git.Git -e`.
2. Clone with the submodules and run the setup script:

   ```bat
   git clone --recursive https://github.com/SebastianDanielFrenz/racing_game.git
   racing_game\setup.cmd
   ```

   `setup.cmd` installs what is missing via winget (VS 2022 Build Tools with
   clang-cl, CMake, Ninja, Godot 4.7.2, Python 3.13), checks out
   `external/physics_sim` and `external/geo2map_engine`, and reports which
   worlds have map data. Re-running it is safe. `setup.cmd -CheckOnly` only
   verifies; `setup.cmd -WithSiblings` also clones `physics_sim` and
   `geo2map_engine` next to `racing_game` for working on them directly.
3. First build and run: `racing_game\tools\run.cmd -Flat` (flat test scene,
   no map data needed). The first build fetches the CMake dependencies.

Map data is not in Git. Copy it from a machine that has it:

| World | Store location (default) | Override |
|---|---|---|
| home (default) | `racing_game\cache\g2m\home-r1` | `RG_G2M_HOME` |
| rhein_main_corridor (VR default), rhein_main_hessen | `<parent>\geo2map_cache\rhein-main-corridor-r2`, `...\rhein-main-hessen-r1` | `RG_G2M_CACHE` (the folder holding the stores) |

Copy each store with its `baked` folder. Then `tools\run.cmd` starts the
home world and `tools\run.cmd -VR` the corridor world.
