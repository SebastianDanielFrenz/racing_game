#Requires -Version 5.1
<#
.SYNOPSIS
    racing_game CI: Windows legs only (clang-cl debug, release), each
    configuring with RG_BUILD_GODOT_EXTENSION=ON, building rg_core/hash_check/
    rg_godot/rg_unit_tests and running ctest. Trimmed relative to
    physics_sim's much larger tools/ci.ps1 (no path-based -Affected selection,
    no WSL/sanitizer legs, no asan/msvc-release legs).

.DESCRIPTION
    TWO LEVELS (plus a by-hand topic run) - pick the smallest that fits:

      tools\ci.ps1                    DEFAULT, per change and per merge batch:
          the release leg only (configure + build + ctest -LE long, parallel).
          Every test tagged [long] is skipped (the Catch2 tags become CTest
          labels via catch_discover_tests ADD_TAGS_AS_LABELS), no debug leg,
          no smoke test. Target: a few minutes.
      tools\ci.ps1 -Topic garage,dyno same build, but only the tests carrying
          one of these Catch2 tags (ctest -L '^(garage|dyno)$'), still without
          [long]. -WithLong includes the [long] ones of those tags.
      tools\ci.ps1 -Smoke             default plus tools/smoke_test.ps1 (all
          the Godot-driven smoke legs below). Use when Godot-facing code,
          the GDExtension or game/ scripts changed.
      tools\ci.ps1 -Full              everything: debug AND release legs, every
          test including [long], and the smoke runs (smoke_test.ps1 normal,
          -BindingsTest, -Shell, -Garage, -CarBrowser, -CarBrowserBig,
          -Cameras, -Controls; it targets out/build/debug). Only before a
          phase counts done, after foundational changes (core/, CMake,
          presets, tools/ci.ps1, physics_sim bumps) or when a [long] test's
          area changed.

    A test that takes >= 5 s in a release build gets the [long] tag (same
    rule as physics_sim). Where a [long] test is the only coverage of
    something cheap and important, a short variant without [long] sits next
    to it (e.g. the sedan + electric car instead of every catalog entry).

.PARAMETER Only
    Run only the named leg(s) (debug, release). Default: release; with -Full:
    debug and release. Tests are filtered the same way as for the level
    (-LE long unless -Full/-WithLong); smoke runs only with -Smoke/-Full.

.PARAMETER Full
    Both legs, all tests including [long], and the smoke runs.

.PARAMETER Smoke
    Add the smoke runs (tools/smoke_test.ps1; builds out/build/debug on its
    own) to a non-Full run.

.PARAMETER SkipSmoke
    Skip tools/smoke_test.ps1 even with -Full/-Smoke (faster iteration on
    build/test failures alone).

.PARAMETER Topic
    Catch2 tag name(s) (without brackets): run only the tests carrying one of
    them. Implies no smoke unless -Smoke is given.

.PARAMETER WithLong
    Include the [long] tests without -Full (e.g. -Topic turbo_kit -WithLong).

.PARAMETER Jobs
    ctest --parallel value (default 8; the cost data in out/test_costs.tsv
    was measured at 8).
#>
[CmdletBinding()]
param(
    [string[]]$Only,
    [switch]$Full,
    [switch]$Smoke,
    [switch]$SkipSmoke,
    [string[]]$Topic,
    [switch]$WithLong,
    [int]$Jobs = 8
)

if (-not $Only -or $Only.Count -eq 0) {
    $Only = if ($Full) { @('debug', 'release') } else { @('release') }
}
$runSmoke = ($Full -or $Smoke) -and -not $SkipSmoke
$includeLong = $Full -or $WithLong

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot

. (Join-Path $PSScriptRoot 'common.ps1')

$script:Failures = New-Object System.Collections.Generic.List[string]
$script:LegResults = @{}

function Report-Fail {
    param([string]$Message)
    Write-Host "FAIL: $Message" -ForegroundColor Red
    $script:Failures.Add($Message)
}

function Report-Ok {
    param([string]$Message)
    Write-Host "ok: $Message" -ForegroundColor Green
}

Write-Host "=== racing_game R0 CI ===" -ForegroundColor Cyan
Write-Host "repo root: $repoRoot"
Write-Host "legs: $($Only -join ', ')"
$ctestFilter = @()
if ($Topic -and $Topic.Count -gt 0) { $ctestFilter += @('-L', ('^(' + (($Topic | ForEach-Object { [regex]::Escape($_) }) -join '|') + ')$')) }
if (-not $includeLong) { $ctestFilter += @('-LE', 'long') }
Write-Host ("ctest filter: " + $(if ($ctestFilter.Count) { $ctestFilter -join ' ' } else { '(none: every test)' }) + "  parallel: $Jobs")
Write-Host ("smoke: " + $(if ($runSmoke) { 'yes' } else { 'no' }))

Write-Host "`n-- submodule --" -ForegroundColor Cyan
Update-Submodules -RepoRoot $repoRoot

Enter-VsDevShell

foreach ($preset in $Only) {
    $legSw = [System.Diagnostics.Stopwatch]::StartNew()
    $buildDir = Join-Path $repoRoot "out\build\$preset"
    Write-Host "`n=== leg: $preset ($buildDir) ===" -ForegroundColor Cyan

    Write-Host "-- configure --" -ForegroundColor Cyan
    $configureExit = Invoke-RgCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir -Preset $preset -WithGodotExtension
    if ($configureExit -ne 0) {
        Report-Fail "[$preset] cmake configure failed (exit $configureExit)"
        $script:LegResults[$preset] = 'FAIL (configure)'
        continue
    }

    Write-Host "-- build --" -ForegroundColor Cyan
    $buildExit = Invoke-RgCMakeBuild -BuildDir $buildDir
    if ($buildExit -ne 0) {
        Report-Fail "[$preset] cmake build failed (exit $buildExit)"
        $script:LegResults[$preset] = 'FAIL (build)'
        continue
    }
    Report-Ok "[$preset] build"

    Write-Host "-- ctest --" -ForegroundColor Cyan
    Push-Location $buildDir
    try {
        & ctest --output-on-failure --parallel $Jobs @ctestFilter | Out-Host
        $ctestExit = $LASTEXITCODE
    } finally {
        Pop-Location
    }
    if ($ctestExit -eq 0) {
        Report-Ok "[$preset] ctest"
    } else {
        Report-Fail "[$preset] ctest exited $ctestExit"
    }

    $legSw.Stop()
    $legFailed = $script:Failures | Where-Object { $_ -like "[$preset]*" }
    $script:LegResults[$preset] = if ($legFailed) { "FAIL ($($legSw.Elapsed.ToString('mm\:ss')))" } else { "PASS ($($legSw.Elapsed.ToString('mm\:ss')))" }
}

if ($runSmoke) {
    Write-Host "`n=== leg: smoke_test ===" -ForegroundColor Cyan
    $smokeSw = [System.Diagnostics.Stopwatch]::StartNew()
    & (Join-Path $PSScriptRoot 'smoke_test.ps1') -SkipBuild:$false -SkipTests
    $smokeExit = $LASTEXITCODE
    $smokeSw.Stop()
    if ($smokeExit -eq 0) {
        Report-Ok "smoke_test"
        $script:LegResults['smoke_test'] = "PASS ($($smokeSw.Elapsed.ToString('mm\:ss')))"
    } else {
        Report-Fail "smoke_test exited $smokeExit"
        $script:LegResults['smoke_test'] = "FAIL ($($smokeSw.Elapsed.ToString('mm\:ss')))"
    }

    # R2.2 R7 task 3: the new RgSimulation/RgTerrainView bindings smoke check
    # (flat-mode only, no RG_G2M_HOME/geo2map cache needed) - reuses the
    # binary the leg above just built (-SkipBuild), same "already-built DLL"
    # pattern as -TerrainPreview would use.
    Write-Host "`n=== leg: smoke_test (bindings) ===" -ForegroundColor Cyan
    $bindingsSw = [System.Diagnostics.Stopwatch]::StartNew()
    & (Join-Path $PSScriptRoot 'smoke_test.ps1') -SkipBuild -SkipTests -BindingsTest
    $bindingsExit = $LASTEXITCODE
    $bindingsSw.Stop()
    if ($bindingsExit -eq 0) {
        Report-Ok "smoke_test (bindings)"
        $script:LegResults['smoke_test_bindings'] = "PASS ($($bindingsSw.Elapsed.ToString('mm\:ss')))"
    } else {
        Report-Fail "smoke_test (bindings) exited $bindingsExit"
        $script:LegResults['smoke_test_bindings'] = "FAIL ($($bindingsSw.Elapsed.ToString('mm\:ss')))"
    }

    # R5: the headless UI flow test of the game shell (-Shell), R6: the garage
    # acceptance flow (-Garage), R5b: the control configuration (-Controls, two processes), R6c: the car browser and the in-world car switch (-CarBrowser,
    # -CarBrowserBig with a synthetic 200-car catalog) and the PHYS-008
    # camera-switch test (-Cameras). Both run flat-world only (no geo2map store)
    # and reuse the built binary, a few seconds each.
    foreach ($extra in @(@('Shell', 'smoke_test_shell'), @('Garage', 'smoke_test_garage'), @('CarBrowser', 'smoke_test_browser'), @('CarBrowserBig', 'smoke_test_browser_big'), @('Cameras', 'smoke_test_cameras'), @('Controls', 'smoke_test_controls'))) {
        Write-Host "`n=== leg: smoke_test (-$($extra[0])) ===" -ForegroundColor Cyan
        $extraSw = [System.Diagnostics.Stopwatch]::StartNew()
        $extraArgs = @{ SkipBuild = $true; SkipTests = $true }
        $extraArgs[$extra[0]] = $true
        & (Join-Path $PSScriptRoot 'smoke_test.ps1') @extraArgs
        $extraExit = $LASTEXITCODE
        $extraSw.Stop()
        if ($extraExit -eq 0) {
            Report-Ok "smoke_test (-$($extra[0]))"
            $script:LegResults[$extra[1]] = "PASS ($($extraSw.Elapsed.ToString('mm\:ss')))"
        } else {
            Report-Fail "smoke_test (-$($extra[0])) exited $extraExit"
            $script:LegResults[$extra[1]] = "FAIL ($($extraSw.Elapsed.ToString('mm\:ss')))"
        }
    }
}

Write-Host "`n=== summary ===" -ForegroundColor Cyan
foreach ($key in $script:LegResults.Keys) {
    $color = if ($script:LegResults[$key] -like 'PASS*') { 'Green' } else { 'Red' }
    Write-Host ("  {0,-12} {1}" -f $key, $script:LegResults[$key]) -ForegroundColor $color
}

if ($script:Failures.Count -eq 0) {
    Write-Host "`nPASS: racing_game CI" -ForegroundColor Green
    exit 0
} else {
    Write-Host "`nFAIL: $($script:Failures.Count) failure(s):" -ForegroundColor Red
    $script:Failures | ForEach-Object { Write-Host " - $_" -ForegroundColor Red }
    exit 1
}
