@echo off
rem run.cmd - thin wrapper so "run" works from cmd.exe (and by double-click)
rem the same way ".\run.ps1" works from PowerShell. See run.ps1's own header
rem comment for switches and the extra-args pass-through convention.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run.ps1" %*
