# Native Linux ARM64 install/update

Current GitHub main has Linux ARM64 target support. On 2026-10-15 a fresh cross-build of main 81e9f21 produced librg_godot.arm64.so; tools/frame/check_symbols.sh passed glibc 2.31 symbol/dlopen checks, and official Godot 4.7.2 ARM64 passed the game bindings test under qemu-aarch64. This is build/loading compatibility evidence, not a live headset/OpenXR/performance certification. Existing Windows-to-Frame deployment remains tools/frame_install.ps1.

## Run on the device

Copy tools/install_linux.sh to the device as a file. It is self-contained; it can clone main even when main does not yet contain the installer. Run as the normal Steam user:

```bash
bash install_linux.sh
```

Requires native Linux ARM64, a C++20 compiler with modern libstdc++, CMake >= 3.25, Ninja, Git+Git LFS, Python >= 3.9, curl, unzip and standard Linux coreutils/flock. Default game mode is OpenXR VR; use --desktop for a monitor. Existing Steam installations normally supply graphics runtime dependencies. For a writable Debian/Ubuntu/Arch OS, --install-deps installs build tools with sudo. The installer never unlocks a read-only SteamOS root; its build tools must already be provided by a suitable development environment.

```bash
bash install_linux.sh --desktop --jobs 2
bash install_linux.sh --map-dir "$HOME/geo2map_cache"
```

The dedicated checkout and all game/runtime/build files live in ~/racing_game. No game installation files are written outside home. Only the explicitly requested --install-deps step installs OS packages. Custom --install-dir paths must resolve beneath home. It clones/fetches the public repository, accepts fast-forward main updates only, updates pinned submodules, retrieves 22 LFS assets, compiles rg_godot, downloads the repo-tested Godot version with SHA512 verification, imports resources and runs a headless bindings check. Compiler capacity is tested early. The native build targets the device's installed toolchain/runtime; it is not the existing x86-host cross preset and does not claim the cross-build's old-glibc portability. Use native Steam without forcing Proton or an incompatible older compatibility container.

Local tracked changes, extra local commits, another branch, a running game or a concurrent installer stop the update rather than discarding work. Installation is incremental; a build failure is not an atomic release rollback. The script snapshots itself outside tracked files before git update so an update cannot overwrite the executing Bash source. User settings and engine/map caches are retained.

## Launch and update

```bash
~/racing_game/.local-install/launch.py
~/racing_game/.local-install/launch.py --desktop
~/racing_game/.local-install/launch.py --desktop --flat
bash ~/racing_game/.local-install/update.sh
```

The saved updater remembers install location, map location and mode. Additional installer options can be passed to it. Launch.py accepts Godot arguments directly; game arguments follow --, for example launch.py --desktop -- --drive. The --flat convenience argument selects Flat World directly.

Real-world map stores are NOT in GitHub. Copy the existing rhein-main-corridor-r2 directory into ~/geo2map_cache (or select its parent with --map-dir). Home-region data, if wanted, goes in home-r1. Without those files use Flat World; the installer does not pretend to download the missing maps. These stores can be large.

## Steam library and artwork

Fully exit Steam before registration; it caches shortcuts.vdf and can overwrite concurrent edits. The installer never kills Steam. If Steam is running after the build, the game is installed but registration stops with instructions:

```bash
bash ~/racing_game/.local-install/update.sh --steam-only
```

This creates/updates Racing Game in the selected native Steam user's non-Steam library, with original generated artwork: landscape header, portrait capsule, hero and icon. It also writes a desktop application entry. It preserves other shortcuts and user-edited launch options/tags/title, retains an existing AppID and backs up shortcuts.vdf before changes. Unknown unsupported binary VDF types or ambiguous accounts cause a refusal before rewriting the file. With multiple accounts it reads the MostRecent login; --steam-user ACCOUNT_ID selects an explicit userdata directory. Start Steam again to see the shortcut. Steam library registration was fixture-tested, not live-tested on the headset.

The game is installed even if Steam is absent; --no-steam deliberately skips registration. Normal native Steam paths are supported; sandboxed Flatpak Steam is not covered by this installer.

## Logs and verification

Installer/build output: ~/racing_game/.local-install/install-YYYYMMDD-HHMMSS.log. Import and bindings output: .local-install/import.log and bindings.log. Installed revision: .local-install/installed-commit. Runtime output stays in the launching terminal; Godot's own Linux log is normally ~/.local/share/godot/app_userdata/racing_game R0/logs/godot.log.

Focused local checks:

```bash
bash -n tools/install_linux.sh
python3 tools/tests/test_install_linux.py
```

Tests exercise binary VDF roundtrip/preservation, repeat installation, AppID/artwork paths, Steam-running refusal, malformed-file refusal, account selection and original vector generation. Raster artwork was rendered using Godot and visually reviewed. No actual device installation or native on-device compilation has been claimed.
