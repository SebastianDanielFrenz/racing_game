#Requires -Version 5.1
<#
.SYNOPSIS
    R0 headless smoke test: configures + builds racing_game (Godot
    extension ON), runs rg_unit_tests via ctest, then runs game/ headless
    for a few seconds and asserts Godot printed no error and the HUD
    reported the sim thread ticking. Modelled directly on physics_sim's
    own adapters/godot/smoke_test.ps1 (read-only reference - a fresh copy,
    not dot-sourced/edited, same reasoning as tools/common.ps1's header).

.DESCRIPTION
    Trimmed relative to the reference: R0 has no scripted autopilot/M1
    driving-check equivalent (main.gd has no --m1-autopilot flag - out of
    R0 scope, PLAN.md 12's R0 acceptance only asks for "a headless smoke
    test ... non-zero exit on failure", not a driving assertion), so this
    script stops at "no error lines + sim thread observed ticking",
    matching the reference's OWN first (non-M1) headless-run section
    exactly. A later milestone that adds scripted control input can extend
    this the same way the reference's M1 section was added on top of its
    own P2 baseline.

.PARAMETER QuitAfterFrames
    How many process frames the headless run executes before quitting
    (Godot's own --quit-after). Default 300 (~5 s at 60 fps).

.PARAMETER SkipBuild
    Skip the configure/build step (use an already-built out/build/debug +
    game/bin/librg_godot.dll).
#>
[CmdletBinding()]
param(
    [int]$QuitAfterFrames = 300,
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$gameDir  = Join-Path $repoRoot 'game'
$buildDir = Join-Path $repoRoot 'out\build\debug'

$script:Failures = New-Object System.Collections.Generic.List[string]

function Report-Fail {
    param([string]$Message)
    Write-Host "FAIL: $Message" -ForegroundColor Red
    $script:Failures.Add($Message)
}

function Report-Ok {
    param([string]$Message)
    Write-Host "ok: $Message" -ForegroundColor Green
}

. (Join-Path $PSScriptRoot 'common.ps1')

Write-Host "=== racing_game R0 headless smoke test ===" -ForegroundColor Cyan
Write-Host "repo root: $repoRoot"
Write-Host "build dir: $buildDir"

Update-Submodules -RepoRoot $repoRoot

if (-not $SkipBuild) {
    Enter-VsDevShell

    Write-Host "`n-- configure (debug, RG_BUILD_GODOT_EXTENSION=ON) --" -ForegroundColor Cyan
    $configureExitCode = Invoke-RgCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir -Preset 'debug' -WithGodotExtension
    if ($configureExitCode -ne 0) { Report-Fail "cmake configure failed (exit $configureExitCode)" }

    if ($script:Failures.Count -eq 0) {
        Write-Host "`n-- build (rg_godot, rg_unit_tests) --" -ForegroundColor Cyan
        $buildExitCode = Invoke-RgCMakeBuild -BuildDir $buildDir -Targets @('rg_godot', 'rg_unit_tests')
        if ($buildExitCode -ne 0) { Report-Fail "cmake build failed (exit $buildExitCode)" }
    }
}

if ($script:Failures.Count -eq 0) {
    Report-Ok "build"
}

# ---------------------------------------------------------------------------
# rg_core unit tests via ctest.
# ---------------------------------------------------------------------------
if ($script:Failures.Count -eq 0) {
    Write-Host "`n-- ctest --" -ForegroundColor Cyan
    Push-Location $buildDir
    try {
        & ctest --output-on-failure | Out-Host
        $ctestExit = $LASTEXITCODE
    } finally {
        Pop-Location
    }
    if ($ctestExit -eq 0) {
        Report-Ok "ctest (exit 0)"
    } else {
        Report-Fail "ctest exited $ctestExit"
    }
}

# ---------------------------------------------------------------------------
# Headless Godot run.
# ---------------------------------------------------------------------------
if ($script:Failures.Count -eq 0) {
    $dll = Join-Path $gameDir 'bin\librg_godot.dll'
    if (-not (Test-Path $dll)) {
        Report-Fail "librg_godot.dll not found at $dll after build"
    } else {
        Report-Ok "librg_godot.dll present at $dll"
    }
}

if ($script:Failures.Count -eq 0) {
    $godotExe = Find-GodotConsoleExe
    Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotExe
    Write-Host "`n-- headless run: $godotExe --quit-after $QuitAfterFrames --" -ForegroundColor Cyan

    # stdout/stderr to separate files - see physics_sim's own smoke_test.ps1
    # comment: merging with '*>' intermittently dropped the stdout half
    # entirely against this native console executable.
    $stdoutFile = Join-Path $buildDir 'smoke_test_godot_stdout.log'
    $stderrFile = Join-Path $buildDir 'smoke_test_godot_stderr.log'
    $previousEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $godotExe --headless --path $gameDir --quit-after $QuitAfterFrames 1>$stdoutFile 2>$stderrFile
    $godotExit = $LASTEXITCODE
    $ErrorActionPreference = $previousEap
    $logContent = @(Get-Content $stdoutFile) + @(Get-Content $stderrFile)
    $logContent | ForEach-Object { Write-Host $_ }

    $errorLines = $logContent | Select-String -Pattern 'ERROR|SCRIPT ERROR|Unhandled exception|Segmentation fault'
    if ($errorLines) {
        Report-Fail "Godot printed error line(s):`n$($errorLines -join "`n")"
    } else {
        Report-Ok "no ERROR/SCRIPT ERROR lines in Godot output"
    }
    if ($godotExit -ne 0) {
        Report-Fail "Godot exited with code $godotExit"
    } else {
        Report-Ok "Godot exited 0"
    }

    $tickLine = $logContent | Select-String -Pattern 'measured sim tick rate|sim thread running' | Select-Object -Last 1
    if ($tickLine) {
        Report-Ok "HUD reported the sim thread ticking: $tickLine"
    } else {
        Report-Fail "no tick-rate line found in Godot output - RgSimulation/hud.gd may not have started"
    }
}

Write-Host "`n=== summary ===" -ForegroundColor Cyan
if ($script:Failures.Count -eq 0) {
    Write-Host "PASS: headless smoke test" -ForegroundColor Green
    exit 0
} else {
    Write-Host "FAIL: $($script:Failures.Count) failure(s):" -ForegroundColor Red
    $script:Failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}
