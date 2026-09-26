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

.PARAMETER TerrainPreview
    Run the PLAN.md R2.1 terrain-preview headless check instead of the
    normal RgSimulation/HUD one: launches Godot headless with
    `-- --terrain-preview` (main.gd's own cmdline-user-arg branch, see that
    file), asserts no ERROR/SCRIPT ERROR line (this also catches Godot's own
    "N RID allocations ... were leaked at exit" message - always an
    ERROR-prefixed line - so a separate RID-leak check is unnecessary: 0
    ERROR lines already means 0 RID leaks), a clean exit code, and that
    main.gd's own "terrain preview selected: chunks=N ..." line reports at
    least 150 chunks (PLAN.md R2.1's own smoke-test acceptance bar - the
    real committed data/world/world_config.json's 20000 m LOD default
    selects 488 around its own spawn point, well above this).

.PARAMETER TerrainStream
    PLAN.md R2.2 R8 streaming check, implies -TerrainPreview: launches with
    `-- --terrain-preview --stream-test` (game/scripts/terrain_stream_test.gd)
    which, once the preview is uploaded, moves RgTerrainView's LOD focus
    through 5 steps (each > 128 m) and waits for every streamed diff to be
    fully applied (adds uploaded, removed RIDs freed). Asserts everything
    -TerrainPreview does plus the script's own "terrain stream test done:
    steps=5 diffs=5 ... missing_removals=0" line; the 0-ERROR-lines check is then the
    RID-leak check after several add/remove diffs. --quit-after is raised to
    200000 frames (the script quits itself; it has its own 180 s wall-clock
    timeout, headless frames being uncapped).

.PARAMETER BindingsTest
    R2.2 R7 task 3: launches with `-- --bindings-test` (main.gd's own
    _run_bindings_test) instead of the normal scene, which proves the new
    RgSimulation (initialize_terrain/get_init_status/get_streaming_status/
    get_render_origin_session/retry_failed_tiles/is_terrain_mode) and
    RgTerrainView (initialize_shared/release) methods exist and that a flat
    initialize() -> initialize() re-init cycle runs without a crash - all in
    FLAT mode only (no RG_G2M_HOME/geo2map cache needed, so this runs in CI).
    Asserts no ERROR/SCRIPT ERROR line, a clean exit code, and the script's
    own "bindings test: ok ..." line.
#>
[CmdletBinding()]
param(
    [int]$QuitAfterFrames = 300,
    [switch]$SkipBuild,
    [switch]$TerrainPreview,
    [switch]$TerrainStream,
    [switch]$BindingsTest
)

$ErrorActionPreference = 'Stop'
if ($TerrainStream) {
    $TerrainPreview = $true
    if (-not $PSBoundParameters.ContainsKey('QuitAfterFrames')) { $QuitAfterFrames = 200000 }
}
if ($BindingsTest -and $TerrainPreview) {
    throw "smoke_test.ps1: -BindingsTest and -TerrainPreview/-TerrainStream are mutually exclusive"
}
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

    $godotArgs = @('--headless', '--path', $gameDir, '--quit-after', $QuitAfterFrames)
    if ($TerrainPreview) {
        $godotArgs += @('--', '--terrain-preview')
        if ($TerrainStream) { $godotArgs += @('--stream-test') }
    } elseif ($BindingsTest) {
        $godotArgs += @('--', '--bindings-test')
    }
    Write-Host "`n-- headless run: $godotExe $($godotArgs -join ' ') --" -ForegroundColor Cyan

    # stdout/stderr to separate files - see physics_sim's own smoke_test.ps1
    # comment: merging with '*>' intermittently dropped the stdout half
    # entirely against this native console executable.
    $stdoutFile = Join-Path $buildDir 'smoke_test_godot_stdout.log'
    $stderrFile = Join-Path $buildDir 'smoke_test_godot_stderr.log'
    $previousEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $godotExe @godotArgs 1>$stdoutFile 2>$stderrFile
    $godotExit = $LASTEXITCODE
    $ErrorActionPreference = $previousEap
    $logContent = @(Get-Content $stdoutFile) + @(Get-Content $stderrFile)
    $logContent | ForEach-Object { Write-Host $_ }

    # Godot's own "N RID allocations of type '...' were leaked at exit"
    # message (printed once per leaked RID at process shutdown) is always
    # ERROR-prefixed, so this one check also IS the "0 RID leaks" assertion
    # for -TerrainPreview - no separate pattern needed.
    $errorLines = $logContent | Select-String -Pattern 'ERROR|SCRIPT ERROR|Unhandled exception|Segmentation fault'
    if ($errorLines) {
        Report-Fail "Godot printed error line(s):`n$($errorLines -join "`n")"
    } else {
        Report-Ok "no ERROR/SCRIPT ERROR lines in Godot output (0 RID leaks included)"
    }
    if ($godotExit -ne 0) {
        Report-Fail "Godot exited with code $godotExit"
    } else {
        Report-Ok "Godot exited 0"
    }

    if ($TerrainPreview) {
        $selectedLine = $logContent | Select-String -Pattern 'terrain preview selected: chunks=(\d+) vertices=(\d+) build_ms=([\d.]+)' | Select-Object -Last 1
        $loadedLine = $logContent | Select-String -Pattern 'terrain preview loaded: chunks=(\d+) vertices=(\d+) upload_ms=([\d.]+)' | Select-Object -Last 1
        if (-not $selectedLine) {
            Report-Fail "no 'terrain preview selected' line found in Godot output - main.gd's --terrain-preview branch may not have run"
        } else {
            $chunkCount = [int]$selectedLine.Matches[0].Groups[1].Value
            Write-Host "terrain preview: $($selectedLine.Line)"
            if ($chunkCount -lt 150) {
                Report-Fail "terrain preview selected only $chunkCount chunks (< 150)"
            } else {
                Report-Ok "terrain preview selected $chunkCount chunks (>= 150)"
            }
        }
        if (-not $loadedLine) {
            Report-Fail "no 'terrain preview loaded' line found in Godot output - RgTerrainView upload never finished within $QuitAfterFrames frames"
        } else {
            Write-Host "terrain preview: $($loadedLine.Line)"
            Report-Ok "terrain preview fully uploaded (0 RID leaks, see the ERROR-line check above)"
        }
        if ($TerrainStream) {
            $logContent | Select-String -Pattern 'terrain stream step ' | ForEach-Object { Write-Host "terrain stream: $($_.Line)" }
            $doneLine = $logContent | Select-String -Pattern 'terrain stream test done: steps=(\d+) diffs=(\d+) added=(\d+) removed=(\d+) chunks=(\d+) missing_removals=(\d+)' | Select-Object -Last 1
            if (-not $doneLine) {
                Report-Fail "no 'terrain stream test done' line found - terrain_stream_test.gd did not finish"
            } else {
                $g = $doneLine.Matches[0].Groups
                Write-Host "terrain stream: $($doneLine.Line)"
                $steps = [int]$g[1].Value; $diffs = [int]$g[2].Value; $added = [int]$g[3].Value
                $removed = [int]$g[4].Value; $errs = [int]$g[6].Value
                if ($steps -ne 5 -or $diffs -ne 5) {
                    Report-Fail "terrain stream: expected steps=5 diffs=5, got steps=$steps diffs=$diffs"
                } elseif ($errs -ne 0) {
                    Report-Fail "terrain stream: $errs removal(s) of a non-resident chunk"
                } elseif ($added -le 0 -or $removed -le 0) {
                    Report-Fail "terrain stream: no chunk streamed (added=$added removed=$removed)"
                } else {
                    Report-Ok "terrain stream: 5 diffs applied, added=$added removed=$removed, 0 errors"
                }
            }
        }
    } elseif ($BindingsTest) {
        $doneLine = $logContent | Select-String -Pattern 'bindings test: ok' | Select-Object -Last 1
        if ($doneLine) {
            Report-Ok "bindings test: $($doneLine.Line)"
        } else {
            Report-Fail "no 'bindings test: ok' line found in Godot output - main.gd's --bindings-test branch may not have completed"
        }
    } else {
        $tickLine = $logContent | Select-String -Pattern 'measured sim tick rate|sim thread running' | Select-Object -Last 1
        if ($tickLine) {
            Report-Ok "HUD reported the sim thread ticking: $tickLine"
        } else {
            Report-Fail "no tick-rate line found in Godot output - RgSimulation/hud.gd may not have started"
        }
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
