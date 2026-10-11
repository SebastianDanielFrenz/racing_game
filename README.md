# racing_game

An open-world racing game on real map data (Godot 4.7), built on:
- `physics_sim` (vehicle and rigid-body physics), pinned as a submodule;
- `geo2map_engine` (map layers, road graph, map server), pinned as a submodule.

The home region comes first: Main-Taunus-Kreis, Hochtaunuskreis and
Frankfurt-Höchst. The design and milestone plan for both repos is in
`PLAN.md`. This is a private project, and all rights are reserved.
Map data attribution is listed in-game under credits.

## Set up on a new machine (Windows)

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
