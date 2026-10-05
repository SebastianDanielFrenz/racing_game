#Requires -Version 5.1
<#
.SYNOPSIS
    racing_game R0 CI: Windows legs only (clang-cl debug, release), each
    configuring with RG_BUILD_GODOT_EXTENSION=ON, building rg_core/hash_check/
    rg_godot/rg_unit_tests, running ctest, then running tools/smoke_test.ps1
    twice (against the debug build - smoke_test.ps1 always targets
    out/build/debug, see its own header): once normally, once with
    -BindingsTest (R2.2 R7 task 3 - reuses the same build, -SkipBuild).
    Trimmed relative to physics_sim's
    much larger tools/ci.ps1 (no -Affected/-Topic path-based selection, no
    WSL/sanitizer legs, no asan/msvc-release legs) - R0 scope per PLAN.md
    12's "tools/ci.ps1 (Windows legs at least: clang-cl debug/release;
    build + unit tests + smoke test)".

.PARAMETER Only
    Run only the named leg(s) (debug, release). Default: both. NOTE: smoke_test.ps1 targets the debug build dir, so the smoke legs below also run for -Only release.

.PARAMETER SkipSmoke
    Skip tools/smoke_test.ps1 (faster iteration on build/test failures
    alone - the smoke test also rebuilds/reconfigures out/build/debug on
    its own, so this flag only matters when debugging non-smoke failures).
#>
[CmdletBinding()]
param(
    [string[]]$Only = @('debug', 'release'),
    [switch]$SkipSmoke
)

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
        & ctest --output-on-failure | Out-Host
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

if (-not $SkipSmoke) {
    Write-Host "`n=== leg: smoke_test ===" -ForegroundColor Cyan
    $smokeSw = [System.Diagnostics.Stopwatch]::StartNew()
    & (Join-Path $PSScriptRoot 'smoke_test.ps1') -SkipBuild:$false
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
    & (Join-Path $PSScriptRoot 'smoke_test.ps1') -SkipBuild -BindingsTest
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
    # acceptance flow (-Garage), R6c: the car browser and the in-world car switch (-CarBrowser,
    # -CarBrowserBig with a synthetic 200-car catalog) and the PHYS-008
    # camera-switch test (-Cameras). Both run flat-world only (no geo2map store)
    # and reuse the built binary, a few seconds each.
    foreach ($extra in @(@('Shell', 'smoke_test_shell'), @('Garage', 'smoke_test_garage'), @('CarBrowser', 'smoke_test_browser'), @('CarBrowserBig', 'smoke_test_browser_big'), @('Cameras', 'smoke_test_cameras'))) {
        Write-Host "`n=== leg: smoke_test (-$($extra[0])) ===" -ForegroundColor Cyan
        $extraSw = [System.Diagnostics.Stopwatch]::StartNew()
        $extraArgs = @{ SkipBuild = $true }
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
