@echo off
rem setup.cmd - one-step machine setup for racing_game: installs the toolchain
rem (winget), checks out the submodules and reports the map data. Arguments
rem pass through (-CheckOnly, -WithSiblings, -SkipPython); see
rem tools\setup_dev_env.ps1.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\setup_dev_env.ps1" %*
