#Requires -Version 5.1
<#
.SYNOPSIS
    Takes the controls screen screenshots (PLAN.md R5b) into out/controls_screens/.

.DESCRIPTION
    Needs a real window (no --headless) and an already built game/bin/librg_godot.dll
    (tools/run.ps1 or tools/smoke_test.ps1 builds it). One launch of
    `--controls-shots <dir>` (game/scripts/controls_shots.gd) with a simulated pad:
      01_device_list.png, 02_action_list_keyboard.png, 02_action_list_pad.png,
      03_capture_prompt.png, 04_axis_tuning_monitor.png, 05_calibration.png.
    The user dir is a scratch dir, so the player's controls.json is untouched.
    Do not run it while the owner's Godot is open (the DLL must not be locked by another
    build and the window would compete for the GPU).
#>
[CmdletBinding()]
param(
    [string]$OutDir = '',
    [string]$Resolution = '1600x900'
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutDir) { $OutDir = Join-Path $repoRoot 'out\controls_screens' }
. (Join-Path $PSScriptRoot 'common.ps1')

$godotExe = Find-GodotExe
$gameDir = Join-Path $repoRoot 'game'
New-Item -ItemType Directory -Force $OutDir | Out-Null
$scratch = Join-Path $repoRoot 'out\build\controls_shots_user'
if (Test-Path $scratch) { Remove-Item -Recurse -Force $scratch }
New-Item -ItemType Directory -Force $scratch | Out-Null

$args = @('--path', $gameDir, '--resolution', $Resolution, '--', '--shell-user-dir', $scratch, '--controls-shots', $OutDir)
Write-Host "$godotExe $($args -join ' ')"
$prev = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
& $godotExe @args 2>&1 | Out-Host
$ErrorActionPreference = $prev

Get-ChildItem $OutDir -Filter *.png | ForEach-Object { Write-Host $_.FullName }
