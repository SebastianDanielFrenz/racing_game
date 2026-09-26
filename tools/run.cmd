@echo off
rem run.cmd - thin wrapper so "run" works from cmd.exe (and by double-click)
rem the same way ".\run.ps1" works from PowerShell. See run.ps1's own header
rem comment for switches and the extra-args pass-through convention.
rem Default start: the real world in Drive mode; "run -Flat" starts the flat
rem scene. Every argument is passed through unchanged.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0run.ps1" %*
