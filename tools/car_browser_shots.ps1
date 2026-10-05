#Requires -Version 5.1
<#
.SYNOPSIS
    Takes the car browser screenshots (PLAN.md R6c) into out/car_browser_screens/.

.DESCRIPTION
    Needs a real window (no --headless) and an already built game/bin/librg_godot.dll
    (tools/run.ps1 or tools/smoke_test.ps1 builds it). Two launches:
      1. the real catalog: browser_by_body_type.png, browser_filter_panel.png,
         change_car_browser.png (the pause menu's Change car browser),
         change_car_in_world.png (the world after the swap);
      2. a synthetic 200-car catalog (tools/make_synthetic_catalog.ps1):
         browser_big_catalog.png, browser_big_catalog_scrolled.png.
    The user dir is a scratch dir, so the player's settings and garage are untouched.
    Do not run it while the owner's Godot is open (the DLL must not be locked by another
    build and the window would compete for the GPU).
#>
[CmdletBinding()]
param(
    [string]$OutDir = ''
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $OutDir) { $OutDir = Join-Path $repoRoot 'out\car_browser_screens' }
. (Join-Path $PSScriptRoot 'common.ps1')

$godotExe = Find-GodotExe
$gameDir = Join-Path $repoRoot 'game'
New-Item -ItemType Directory -Force $OutDir | Out-Null
$scratch = Join-Path $repoRoot 'out\build\car_browser_shots_user'
if (Test-Path $scratch) { Remove-Item -Recurse -Force $scratch }
New-Item -ItemType Directory -Force $scratch | Out-Null

function Run-Godot([string[]]$extra) {
    $args = @('--path', $gameDir, '--resolution', '1600x900', '--', '--shell-user-dir', $scratch) + $extra
    Write-Host "$godotExe $($args -join ' ')"
    $prev = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    & $godotExe @args 2>&1 | Out-Host
    $ErrorActionPreference = $prev
}

Run-Godot @('--car-browser-shots', $OutDir)

$bigCatalog = Join-Path $repoRoot 'out\build\car_browser_big_catalog.json'
& (Join-Path $PSScriptRoot 'make_synthetic_catalog.ps1') -OutFile $bigCatalog -Total 200 | Out-Host
Run-Godot @('--car-browser-shots', $OutDir, '--car-browser-big', '--catalog', $bigCatalog)

Get-ChildItem $OutDir -Filter *.png | ForEach-Object { Write-Host $_.FullName }
