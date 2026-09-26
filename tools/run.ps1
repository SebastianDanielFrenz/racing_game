#Requires -Version 5.1
<#
.SYNOPSIS
    Builds the rg_godot GDExtension (incrementally) and launches Godot on
    game/ - "build it and go drive" in one command, modelled directly on
    physics_sim's own drive.ps1 (read-only reference; this is a fresh copy,
    not a dot-sourced/edited one - see tools/common.ps1's own header
    comment for why).

.DESCRIPTION
    Default build is INCREMENTAL: cmake configure only runs the first time
    (no out/build/debug/build.ninja yet); afterwards a plain
    `cmake --build` is enough even after a CMakeLists.txt edit, because
    Ninja's own generated build re-runs cmake itself when needed.

    Before building, checks whether game/bin/librg_godot.dll is currently
    locked (a running Godot instance likely still has the project open)
    and stops with a clear message instead of a cryptic link.exe error.

    PASS-THROUGH CONVENTION for extra Godot arguments: same hand-rolled
    $args parsing as physics_sim's drive.ps1 (a literal "--" cannot be
    bound by PowerShell's own parameter binder alongside a param() block -
    confirmed there by testing). In practice all of the following work:
        run.ps1
        run.ps1 -SkipBuild
        run.ps1 -NoConsole
        run.ps1 -Flat
        run.cmd -- --some-godot-flag

    WHAT STARTS (R9): by default the game starts in the real world
    (home region, needs the geo2map store - see game/scripts/main.gd) in
    Drive mode, i.e. `--drive` is forwarded to Godot. -Flat starts the flat
    test scene instead (no store needed). Either way the world and the
    player mode switch at runtime (F8 / V) - the flag picks only the start.
    Nothing is added when the forwarded args already choose a start
    (--drive, --terrain-preview, --bindings-test).

.PARAMETER -SkipBuild
    Skip the configure/build step entirely and launch whatever is already
    built. Errors if librg_godot.dll does not exist yet.

.PARAMETER -NoConsole
    Launch the non-console Godot executable (no extra terminal window).

.PARAMETER -Flat
    Start in the flat test scene instead of the real world (Drive mode
    either way).

.PARAMETER -Editor
    Open the Godot editor on game/ instead of running it.

.PARAMETER -DryRun
    Print what would happen and do nothing - no build, no launch.

.PARAMETER -Preset
    CMake preset to build (default 'debug').
#>

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$gameDir  = Join-Path $repoRoot 'game'
$dllPath  = Join-Path $gameDir 'bin\librg_godot.dll'

. (Join-Path $PSScriptRoot 'common.ps1')

$SkipBuild = $false
$NoConsole = $false
$Editor    = $false
$Flat      = $false
$DryRun    = $false
$Preset    = 'debug'
$GodotArgs = New-Object System.Collections.Generic.List[string]
$sawSeparator = $false
$expectPresetValue = $false

foreach ($a in $args) {
    if ($expectPresetValue) { $Preset = $a; $expectPresetValue = $false; continue }
    if ($sawSeparator) { $GodotArgs.Add($a); continue }
    switch -Regex ($a) {
        '^--?SkipBuild$' { $SkipBuild = $true; continue }
        '^--?NoConsole$' { $NoConsole = $true; continue }
        '^--?Editor$'    { $Editor    = $true; continue }
        '^--?Flat$'      { $Flat      = $true; continue }
        '^--?DryRun$'    { $DryRun    = $true; continue }
        '^--?Preset$'    { $expectPresetValue = $true; continue }
        '^--$'           { $sawSeparator = $true; continue }
        default          { $GodotArgs.Add($a) }
    }
}

$buildDir = Join-Path $repoRoot "out\build\$Preset"

# R9: the start world. Real-world Drive unless -Flat or the forwarded args
# already choose a start themselves.
$choosesStart = @($GodotArgs | Where-Object { $_ -in @('--drive', '--terrain-preview', '--bindings-test') }).Count -gt 0
if (-not $Flat -and -not $choosesStart) {
    $GodotArgs.Insert(0, '--drive')
}
$startLabel = if ($Flat) { 'flat scene' } elseif ($GodotArgs -contains '--drive') { 'real world' } else { 'chosen by the forwarded args' }

Write-Host "=== run: build + launch racing_game ($startLabel) ===" -ForegroundColor Cyan
Write-Host "repo root: $repoRoot"
Write-Host "build dir: $buildDir"
Write-Host "game dir:  $gameDir"
if ($GodotArgs.Count -gt 0) {
    Write-Host "forwarding to Godot: $($GodotArgs -join ' ')"
}
if ($DryRun) {
    Write-Host "(dry run - nothing will actually be built or launched)" -ForegroundColor Yellow
}

Write-Host "`n-- submodule --" -ForegroundColor Cyan
if ($DryRun) {
    Write-Host "would run: git -c protocol.file.allow=always submodule update --init --recursive"
} else {
    Update-Submodules -RepoRoot $repoRoot
}

if (-not $SkipBuild) {
    $dllLocked = Test-DllLocked -Path $dllPath
    if ($dllLocked) {
        if ($DryRun) {
            Write-Host "note: $dllPath appears to be locked (likely a running Godot instance has the project open) - a real run would stop here." -ForegroundColor Yellow
        } else {
            throw "librg_godot.dll appears to be locked - a Godot process likely already has the project open:`n  $dllPath`nClose Godot and re-run run.ps1 / run.cmd."
        }
    }
}

$buildElapsed = $null
if (-not $SkipBuild) {
    $needsConfigure = -not (Test-RgConfigured -BuildDir $buildDir)
    if ($DryRun) {
        $what = if ($needsConfigure) { 'configure + build' } else { 'build (already configured)' }
        Write-Host "`n-- would $what (preset $Preset) in $buildDir --" -ForegroundColor Cyan
    } else {
        Enter-VsDevShell
        $sw = [System.Diagnostics.Stopwatch]::StartNew()

        if ($needsConfigure) {
            Write-Host "`n-- configure ($Preset, RG_BUILD_GODOT_EXTENSION=ON) --" -ForegroundColor Cyan
            $configureExitCode = Invoke-RgCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir -Preset $Preset -WithGodotExtension
            if ($configureExitCode -ne 0) {
                throw "cmake configure failed (exit $configureExitCode) - not launching Godot"
            }
        }

        Write-Host "`n-- build --" -ForegroundColor Cyan
        $buildExitCode = Invoke-RgCMakeBuild -BuildDir $buildDir -Targets @('rg_godot')
        $sw.Stop()
        $buildElapsed = $sw.Elapsed

        if ($buildExitCode -ne 0) {
            throw "cmake build failed (exit $buildExitCode) after $($buildElapsed.ToString('mm\:ss\.ff')) - not launching Godot on a stale DLL"
        }
        if (-not (Test-Path $dllPath)) {
            throw "build reported success but $dllPath is still missing - not launching Godot"
        }
        Write-Host "build ok in $($buildElapsed.ToString('mm\:ss\.ff'))" -ForegroundColor Green
    }
} else {
    Write-Host "`n-- skipping build (-SkipBuild) --" -ForegroundColor Cyan
    if (-not $DryRun -and -not (Test-Path $dllPath)) {
        throw "-SkipBuild given but $dllPath does not exist yet - build at least once first (run.ps1 without -SkipBuild)"
    }
}

$godotExe = if ($NoConsole) { Find-GodotExe } else { Find-GodotConsoleExe }

if (-not $Editor -and -not $DryRun) {
    # -Editor already runs its own import scan on open; a non-editor launch
    # against a checkout that has never been opened in the editor needs the
    # same one-shot --import pass smoke_test.ps1 uses (see
    # Ensure-GodotProjectImported's own comment in common.ps1) or the
    # GDExtension silently never loads.
    Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotExe
}

$launchArgs = New-Object System.Collections.Generic.List[string]
$launchArgs.Add('--path')
$launchArgs.Add($gameDir)

if ($Editor) {
    $launchArgs.Add('--editor')
    if ($GodotArgs.Count -gt 0) {
        Write-Host "note: -Editor ignores forwarded Godot args: $($GodotArgs -join ' ')" -ForegroundColor Yellow
    }
} elseif ($GodotArgs.Count -gt 0) {
    $launchArgs.Add('--')
    foreach ($a in $GodotArgs) { $launchArgs.Add($a) }
}

$launchArgsArr = $launchArgs.ToArray()

Write-Host "`n-- launch --" -ForegroundColor Cyan
Write-Host "$godotExe $($launchArgsArr -join ' ')"

if ($DryRun) {
    Write-Host "`n(dry run - not launching Godot)" -ForegroundColor Yellow
    exit 0
}

& $godotExe @launchArgsArr
exit $LASTEXITCODE
