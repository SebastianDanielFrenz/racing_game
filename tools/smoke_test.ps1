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
    Skip the configure/build step (use an already-built out/build/<Preset> +
    game/bin/librg_godot.dll).

.PARAMETER Preset
    CMake preset to build/run (default 'debug' - this script's own
    long-standing default; pass 'relwithdebinfo' to smoke-test the
    optimised DLL tools/run.ps1 now builds by default). Uses the same
    stamp-file guard as run.ps1 (tools/common.ps1's Remove-RgGodotStaleDll/
    Set-RgGodotPresetStamp) so a DLL left over from a different preset's
    build is deleted and relinked instead of silently reused; with
    -SkipBuild a stamp naming a different preset only warns, it does not
    fail.

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

.PARAMETER Drive
    R2.2 R9 real-world drive check: launches with
    `-- --drive --drive-smoke` (game/scripts/drive_smoke.gd). Needs the
    geo2map store (tiles.sqlite3 in $env:RG_G2M_HOME, or in the default
    <repo>/cache/g2m/home-r1 when RG_G2M_HOME is unset); without it the
    headless part prints SKIP and the script exits 0. The scripted run loads
    the real world, drives straight for 6 s of sim time, cycles the player
    mode drive -> free_cam -> drive, switches the world real -> flat -> real,
    cancels one real-world load mid-way with another switch, and loads the
    real world once more. Asserts everything the default run does (no
    ERROR lines, exit 0) plus: an "RG_DRIVE ready" line; falls=0 misses=0 on
    every RG_DRIVE numbers line; ticks > 0 and result=ok on the final
    "RG_DRIVE done" line. --quit-after is raised to 200000 (the script
    quits itself; 300 s wall-clock timeout).

.PARAMETER Shell
    R5 headless UI flow test: launches with `-- --shell-test --shell-user-dir <dir>`
    (game/scripts/shell_flow_test.gd; no direct-start flag, so the game boots into
    the shell). The script presses the real buttons: boot -> main menu -> settings
    (change a value, Back saves, a second RgShell reloads it) -> credits -> Free roam
    -> flat world -> drive -> Esc pause -> Resume -> Reset car -> Main menu (world
    torn down) -> Free roam again in the same process -> Main menu -> Quit. Needs no
    geo2map store (flat world), so it runs everywhere. Asserts the usual 0 ERROR
    lines / exit 0 plus "RG_SHELL_TEST PASS" and no "RG_SHELL_TEST FAIL" line.
    The settings go to a scratch dir under the build dir, never to user://.

.PARAMETER Garage
    R6 headless garage acceptance test: launches with `-- --garage-test --shell-user-dir <dir>`
    (game/scripts/garage_test.gd). Presses the real buttons and slider controls: Free roam
    with the stock hyper car (suspension compression read from the physics) -> Main menu ->
    Garage -> vehicle select -> Configure -> front springs slider -> an invalid gear set is
    rejected with the loader's message (Save/Drive disabled) -> Save -> a second RgGarage
    reads the setup -> Drive -> flat world, the physics shows the stiffer front springs ->
    pause -> Garage (respawn) -> the sedan -> Main menu: no world, no garage node, no work
    file left. Asserts 0 ERROR lines / exit 0 plus "RG_GARAGE_TEST PASS" and no FAIL line.

.PARAMETER Cameras
    R5 PHYS-008 camera-switch test: launches with `-- --camera-test` (flat world,
    game/scripts/camera_switch_test.gd). Every ordered pair of the five driving
    views (and the free cam) is switched, plain and across a floating-origin
    rebase in the switch frame, with the sim paused; the camera must land on its
    baseline within the script's tolerances. Asserts 0 ERROR lines / exit 0 plus
    "RG_CAMERA_TEST PASS".

.PARAMETER DriveDelayMs
    Implies -Drive and forwards `--g2m-fetch-delay-ms N` (every tile fetch
    is delayed by N ms), and drive_smoke.gd then relocates the car ~3 km
    along the route into non-resident terrain. Additionally asserts that the
    terrain gate froze the clock at least once (relocate_freezes >= 1) and
    that the tick count advanced after the relocation landed
    (advanced_after_relocate > 0). Typical: -DriveDelayMs 200.
#>
[CmdletBinding()]
param(
    [int]$QuitAfterFrames = 300,
    [switch]$SkipBuild,
    [string]$Preset = 'debug',
    [switch]$TerrainPreview,
    [switch]$TerrainStream,
    [switch]$BindingsTest,
    [switch]$Drive,
    [switch]$Shell,
    [switch]$Garage,
    [switch]$Cameras,
    [int]$DriveDelayMs = 0
)

$ErrorActionPreference = 'Stop'
if ($TerrainStream) {
    $TerrainPreview = $true
    if (-not $PSBoundParameters.ContainsKey('QuitAfterFrames')) { $QuitAfterFrames = 200000 }
}
if ($DriveDelayMs -gt 0) { $Drive = $true }
if (($Drive -or $Shell -or $Garage -or $Cameras) -and -not $PSBoundParameters.ContainsKey('QuitAfterFrames')) { $QuitAfterFrames = 200000 }
if ((@($BindingsTest, $TerrainPreview, $Drive, $Shell, $Garage, $Cameras) | Where-Object { $_ }).Count -gt 1) {
    throw "smoke_test.ps1: -BindingsTest, -TerrainPreview/-TerrainStream, -Drive/-DriveDelayMs, -Shell, -Garage and -Cameras are mutually exclusive"
}
$repoRoot = Split-Path -Parent $PSScriptRoot
$gameDir  = Join-Path $repoRoot 'game'
$buildDir = Join-Path $repoRoot "out\build\$Preset"
$dllPath  = Join-Path $gameDir 'bin\librg_godot.dll'

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
    # Stale-DLL guard (tools/common.ps1): game/bin/librg_godot.dll is
    # written by whichever build dir last linked it, whatever preset that
    # was - delete it first if it was not this preset's own, so ninja is
    # forced to relink instead of silently leaving the wrong DLL in place.
    Remove-RgGodotStaleDll -DllPath $dllPath -Preset $Preset

    Enter-VsDevShell

    Write-Host "`n-- configure ($Preset, RG_BUILD_GODOT_EXTENSION=ON) --" -ForegroundColor Cyan
    $configureExitCode = Invoke-RgCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir -Preset $Preset -WithGodotExtension
    if ($configureExitCode -ne 0) { Report-Fail "cmake configure failed (exit $configureExitCode)" }

    if ($script:Failures.Count -eq 0) {
        Write-Host "`n-- build (rg_godot, rg_unit_tests) --" -ForegroundColor Cyan
        $buildExitCode = Invoke-RgCMakeBuild -BuildDir $buildDir -Targets @('rg_godot', 'rg_unit_tests')
        if ($buildExitCode -ne 0) { Report-Fail "cmake build failed (exit $buildExitCode)" }
        if ($script:Failures.Count -eq 0 -and (Test-Path $dllPath)) {
            Set-RgGodotPresetStamp -DllPath $dllPath -Preset $Preset
        }
    }
} else {
    $builtPreset = Get-RgGodotBuiltPreset -DllPath $dllPath
    if ($builtPreset -and $builtPreset -ne $Preset) {
        Write-Host "warning: -SkipBuild given but $dllPath was last built by preset '$builtPreset', not '$Preset' - using it anyway." -ForegroundColor Yellow
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
    if (-not (Test-Path $dllPath)) {
        Report-Fail "librg_godot.dll not found at $dllPath after build"
    } else {
        Report-Ok "librg_godot.dll present at $dllPath"
    }
}

# -Drive needs the geo2map store (world_config.json's ${RG_G2M_HOME},
# defaulting to <repo>/cache/g2m/home-r1 - rg_core's world_config.cpp).
$script:DriveSkipped = $false
if ($Drive -and $script:Failures.Count -eq 0) {
    $g2mHome = if ($env:RG_G2M_HOME) { $env:RG_G2M_HOME } else { Join-Path $repoRoot 'cache\g2m\home-r1' }
    $store = Join-Path $g2mHome 'tiles.sqlite3'
    if (-not (Test-Path $store)) {
        Write-Host "`nSKIP: -Drive needs the geo2map store, none at $store (set RG_G2M_HOME)" -ForegroundColor Yellow
        $script:DriveSkipped = $true
    } else {
        Write-Host "`n-Drive: geo2map store $store"
    }
}

if ($script:Failures.Count -eq 0 -and -not $script:DriveSkipped) {
    $godotExe = Find-GodotConsoleExe
    Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotExe

    $godotArgs = @('--headless', '--path', $gameDir, '--quit-after', $QuitAfterFrames)
    if ($TerrainPreview) {
        $godotArgs += @('--', '--terrain-preview')
        if ($TerrainStream) { $godotArgs += @('--stream-test') }
    } elseif ($BindingsTest) {
        $godotArgs += @('--', '--bindings-test')
    } elseif ($Drive) {
        $godotArgs += @('--', '--drive', '--drive-smoke')
        if ($DriveDelayMs -gt 0) { $godotArgs += @('--g2m-fetch-delay-ms', $DriveDelayMs) }
    } elseif ($Shell) {
        # No direct-start flag: the game boots into the shell. A scratch user dir
        # keeps the test's settings.json out of the real user://.
        $shellUserDir = Join-Path $buildDir 'shell_test_user'
        if (Test-Path $shellUserDir) { Remove-Item -Recurse -Force $shellUserDir }
        New-Item -ItemType Directory -Force $shellUserDir | Out-Null
        $godotArgs += @('--', '--shell-test', '--shell-user-dir', $shellUserDir)
        # With a geo2map store the test also loads the real world through the menu.
        $shellStoreHome = if ($env:RG_G2M_HOME) { $env:RG_G2M_HOME } else { Join-Path $repoRoot 'cache\g2m\home-r1' }
        $script:ShellReal = Test-Path (Join-Path $shellStoreHome 'tiles.sqlite3')
        if ($script:ShellReal) { $godotArgs += @('--shell-real') } else { Write-Host "note: no geo2map store at $shellStoreHome - the shell test skips its real-world round" -ForegroundColor Yellow }
    } elseif ($Garage) {
        $garageUserDir = Join-Path $buildDir 'garage_test_user'
        if (Test-Path $garageUserDir) { Remove-Item -Recurse -Force $garageUserDir }
        New-Item -ItemType Directory -Force $garageUserDir | Out-Null
        $godotArgs += @('--', '--garage-test', '--shell-user-dir', $garageUserDir)
    } elseif ($Cameras) {
        $godotArgs += @('--', '--camera-test')
    } else {
        # A plain launch now opens the shell; the plain smoke checks the flat sim.
        $godotArgs += @('--', '--flat')
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
    # -CaseSensitive: Windows PowerShell 5.1 wraps the first stderr line in an
    # ErrorRecord whose text contains "FullyQualifiedErrorId" (matched 'ERROR'
    # case-insensitively although Godot printed no error).
    $errorLines = $logContent | Select-String -CaseSensitive -Pattern 'ERROR|SCRIPT ERROR|Unhandled exception|Segmentation fault'
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
    } elseif ($Drive) {
        $readyLine = $logContent | Select-String -Pattern 'RG_DRIVE ready' | Select-Object -First 1
        if ($readyLine) {
            Report-Ok "real world became drivable: $($readyLine.Line)"
        } else {
            Report-Fail "no 'RG_DRIVE ready' line - the real world never became drivable"
        }
        $numberLines = @($logContent | Select-String -Pattern 'RG_DRIVE .*falls=')
        $badLines = @($numberLines | Where-Object { $_.Line -notmatch 'falls=0 misses=0' })
        if ($numberLines.Count -eq 0) {
            Report-Fail "no RG_DRIVE line carries falls=/misses= numbers"
        } elseif ($badLines.Count -gt 0) {
            Report-Fail "a fall or a wheel-cast miss was reported:`n$($badLines -join "`n")"
        } else {
            Report-Ok "falls=0 misses=0 on all $($numberLines.Count) RG_DRIVE numbers lines"
        }
        # G2.5a-grip R-c: the spawn point on data/routes/home_r1_drive.json is
        # a paved road (test_route_grip.cpp's realdata proof) - wheel 0's
        # surface at the spawn check must read asphalt, never a fallback
        # uniform/off-road value.
        $spawnCheckLine = $logContent | Select-String -Pattern 'RG_DRIVE spawn_check .*surface0=(\S+)' | Select-Object -Last 1
        if (-not $spawnCheckLine) {
            Report-Fail "no 'RG_DRIVE spawn_check' line carries surface0= - main.gd's _report() spawn check may not have run"
        } else {
            $spawnSurface = $spawnCheckLine.Matches[0].Groups[1].Value
            if ($spawnSurface -eq 'asphalt') {
                Report-Ok "spawn surface is asphalt (surface0=$spawnSurface)"
            } else {
                Report-Fail "spawn surface0=$spawnSurface (expected asphalt)"
            }
        }
        # R9c on-foot round trip (drive_smoke.gd): refused while moving, spawn at
        # the door, run away, get-in out of range refused, run back, get in, drive.
        foreach ($step in @(
                @('on_foot refused at', 'getting out of a moving car is refused'),
                @('on_foot ok: walker spawned', 'the walker spawns beside the car (rig walker, car unattended, 2 interest points)'),
                @('on_foot walked', 'the walker runs away from the car'),
                @('on_foot get-in out of range refused ok', 'a get-in out of range is refused'),
                @('on_foot get-in ok', 'the walker gets back in (drive, 1 interest point)'),
                @('on_foot round trip ok', 'the car drives again after the round trip'))) {
            $stepLine = $logContent | Select-String -SimpleMatch -Pattern ('RG_DRIVE smoke ' + $step[0]) | Select-Object -First 1
            if ($stepLine) { Report-Ok "on foot: $($step[1])" } else { Report-Fail "on foot: no 'RG_DRIVE smoke $($step[0])' line ($($step[1]))" }
        }
        $doneLine = $logContent | Select-String -Pattern 'RG_DRIVE done .*' | Select-Object -Last 1
        if (-not $doneLine) {
            Report-Fail "no 'RG_DRIVE done' line - drive_smoke.gd did not finish"
        } else {
            $done = $doneLine.Line
            Write-Host "drive smoke: $done"
            $ticks = if ($done -match ' ticks=(\d+)') { [int]$Matches[1] } else { -1 }
            if ($ticks -gt 0) { Report-Ok "ticks=$ticks > 0 at the end" } else { Report-Fail "ticks=$ticks at the end (expected > 0)" }
            if ($done -match 'result=ok') { Report-Ok "drive smoke result=ok" } else { Report-Fail "drive smoke did not report result=ok" }
            if ($DriveDelayMs -gt 0) {
                $freezes = if ($done -match 'relocate_freezes=(-?\d+)') { [int]$Matches[1] } else { -1 }
                $advanced = if ($done -match 'advanced_after_relocate=(-?\d+)') { [int]$Matches[1] } else { -1 }
                if ($freezes -ge 1) { Report-Ok "the terrain gate froze the clock (relocate_freezes=$freezes)" } else { Report-Fail "relocate_freezes=$freezes (expected >= 1 with a $DriveDelayMs ms fetch delay)" }
                if ($advanced -gt 0) { Report-Ok "ticks advanced after the relocation (advanced_after_relocate=$advanced)" } else { Report-Fail "advanced_after_relocate=$advanced (expected > 0)" }
            }
        }
    } elseif ($Shell) {
        $failLines = @($logContent | Select-String -SimpleMatch -Pattern 'RG_SHELL_TEST FAIL')
        $okLines = @($logContent | Select-String -SimpleMatch -Pattern 'RG_SHELL_TEST ok:')
        $passLine = $logContent | Select-String -Pattern 'RG_SHELL_TEST PASS checks=(\d+)' | Select-Object -Last 1
        if ($failLines.Count -gt 0) {
            Report-Fail "shell flow test reported failure(s):`n$($failLines -join "`n")"
        } elseif (-not $passLine) {
            Report-Fail "no 'RG_SHELL_TEST PASS' line - shell_flow_test.gd did not finish (is main.gd's --shell-test node running?)"
        } else {
            Report-Ok "shell flow test: $($passLine.Line) ($($okLines.Count) ok lines)"
        }
        if ($script:ShellReal) {
            $realOk = @($logContent | Select-String -SimpleMatch -Pattern 'RG_SHELL_TEST ok: real:')
            if ($realOk.Count -ge 8) { Report-Ok "shell flow test: the real-world round ran ($($realOk.Count) checks)" } else { Report-Fail "shell flow test: only $($realOk.Count) 'real:' checks ran (expected the real-world round)" }
        }
        $shellLines = @($logContent | Select-String -Pattern 'RG_SHELL \S+ -> \S+')
        Write-Host "shell transitions: $($shellLines.Count)"
    } elseif ($Garage) {
        $failLines = @($logContent | Select-String -SimpleMatch -Pattern 'RG_GARAGE_TEST FAIL')
        $okLines = @($logContent | Select-String -SimpleMatch -Pattern 'RG_GARAGE_TEST ok:')
        $passLine = $logContent | Select-String -Pattern 'RG_GARAGE_TEST PASS checks=(\d+)' | Select-Object -Last 1
        if ($failLines.Count -gt 0) {
            Report-Fail "garage test reported failure(s):`n$($failLines -join "`n")"
        } elseif (-not $passLine) {
            Report-Fail "no 'RG_GARAGE_TEST PASS' line - garage_test.gd did not finish (is main.gd's --garage-test node running?)"
        } else {
            Report-Ok "garage test: $($passLine.Line) ($($okLines.Count) ok lines)"
        }
    } elseif ($Cameras) {
        $failLines = @($logContent | Select-String -SimpleMatch -Pattern 'RG_CAMERA_TEST FAIL')
        $passLine = $logContent | Select-String -Pattern 'RG_CAMERA_TEST PASS .*' | Select-Object -Last 1
        $pairsLine = $logContent | Select-String -Pattern 'RG_CAMERA_TEST pairs=(\d+) rebase_pairs=(\d+)' | Select-Object -Last 1
        if ($failLines.Count -gt 0) {
            Report-Fail "camera switch test reported failure(s):`n$($failLines -join "`n")"
        } elseif (-not $passLine) {
            Report-Fail "no 'RG_CAMERA_TEST PASS' line - camera_switch_test.gd did not finish"
        } elseif (-not $pairsLine -or [int]$pairsLine.Matches[0].Groups[1].Value -lt 20 -or [int]$pairsLine.Matches[0].Groups[2].Value -lt 20) {
            Report-Fail "camera switch test: fewer than 20 plain + 20 rebase pairs ran ($($pairsLine.Line))"
        } else {
            Report-Ok "camera switch test: $($passLine.Line); $($pairsLine.Line)"
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
if ($script:Failures.Count -eq 0 -and $script:DriveSkipped) {
    Write-Host "SKIP: -Drive headless run skipped (no geo2map store); build and ctest passed" -ForegroundColor Yellow
    exit 0
} elseif ($script:Failures.Count -eq 0) {
    Write-Host "PASS: headless smoke test" -ForegroundColor Green
    exit 0
} else {
    Write-Host "FAIL: $($script:Failures.Count) failure(s):" -ForegroundColor Red
    $script:Failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}
