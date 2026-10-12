@echo off
rem make_release.cmd - thin wrapper so "make_release" works from cmd.exe (and by
rem double-click) the same way ".\make_release.ps1" works from PowerShell. See
rem make_release.ps1's header for switches (-SkipBuild, -AllowDirty, -Keep N).
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0make_release.ps1" %*
