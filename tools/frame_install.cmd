@echo off
rem frame_install.cmd - thin wrapper so "frame_install" works from cmd.exe (and by
rem double-click) the same way ".\frame_install.ps1" works from PowerShell. See
rem frame_install.ps1's header for switches (-SkipBuild, -StageOnly, -NoMaps, -QemuCheck ...).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0frame_install.ps1" %*
