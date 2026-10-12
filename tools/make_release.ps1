#Requires -Version 5.1
<#
.SYNOPSIS
    Builds rg_godot (relwithdebinfo, like run.ps1) and writes a self-contained,
    playable SNAPSHOT of the game into S:\claude_code\racing_game_releases\
    <yyyyMMdd-HHmm>_<shortsha>\ so the owner can play a fixed version while
    agents keep changing the repo.

.DESCRIPTION
    1. Build: the same incremental configure/build as tools/run.ps1
       (Enter-VsDevShell, Remove-RgGodotStaleDll, Invoke-RgCMake*, preset stamp).
       Stops with a clear message when game/bin/librg_godot.dll is locked (a
       running Godot has the project open). -SkipBuild uses the DLL as it is.
    2. Refuses a dirty TRACKED tree (including a submodule at another commit
       or with local changes) unless -AllowDirty. Untracked files are fine.
       The commit sha and both submodule shas go into RELEASE.txt.
    3. Copies, keeping the repo-relative layout the game resolves at runtime
       (res://../data, res://../external/physics_sim/data, ...):
         game/                                  incl. bin/ DLL+PDB and .godot/ import cache
         data/
         external/physics_sim/data
         external/physics_sim/adapters/godot/demo/audio   (tyre squeal sample, tyre_sound_model.gd)
         external/physics_sim/out/godot_engine_cache      (warm .psmaps engine-map cache, if present)
         out/godot_engine_cache                           (same, for racing_game's own vehicles)
         external/geo2map_engine/data
         CMakeLists.txt                                   (STUB: rg::load_world_config finds the
                                                           "repo root" as the nearest ancestor holding
                                                           a CMakeLists.txt; without it surface_map
                                                           / palette paths cannot resolve)
       NOT copied: .git, out/build, cache/ (the big geo2map stores).
    4. Writes play.cmd / play_drive.cmd (Godot on the snapshot's game/;
       RG_G2M_CACHE / RG_G2M_HOME default to the real shared locations, resolved
       now, unless the caller already set them) and RELEASE.txt.
    5. Updates <OutRoot>\latest.cmd (+ latest_drive.cmd) and keeps only the
       newest -Keep snapshots (default 3; the new one is never deleted).

    The snapshot is built in "<name>.partial" and renamed when complete, so
    latest.cmd never points at a half-copied directory.

    Shared with the dev checkout on purpose: the geo2map stores (the derived
    store is written to by whoever runs) and Godot's user dir (settings,
    garage work files, thumbnails: same project name).

.PARAMETER SkipBuild    Use the existing game/bin/librg_godot.dll.
.PARAMETER AllowDirty   Snapshot even with modified tracked files.
.PARAMETER Keep         Number of snapshots to keep (default 3).
.PARAMETER Preset       CMake preset to build (default relwithdebinfo).
.PARAMETER OutRoot      Snapshot root (default S:\claude_code\racing_game_releases).
#>
[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [switch]$AllowDirty,
    [int]$Keep = 3,
    [string]$Preset = 'relwithdebinfo',
    [string]$OutRoot = 'S:\claude_code\racing_game_releases'
)

$ErrorActionPreference = 'Stop'

$repoRoot = Split-Path -Parent $PSScriptRoot
$gameDir  = Join-Path $repoRoot 'game'
$dllPath  = Join-Path $gameDir 'bin\librg_godot.dll'
$buildDir = Join-Path $repoRoot "out\build\$Preset"

. (Join-Path $PSScriptRoot 'common.ps1')
. (Join-Path $PSScriptRoot 'release_common.ps1')

if ($Keep -lt 1) { throw "-Keep must be at least 1" }

function Invoke-Git {
    param([Parameter(ValueFromRemainingArguments)][string[]]$GitArgs)
    $out = & git -C $repoRoot @GitArgs
    if ($LASTEXITCODE -ne 0) { throw "git $($GitArgs -join ' ') failed (exit $LASTEXITCODE)" }
    return $out
}

Write-Host "=== make_release: snapshot of racing_game ===" -ForegroundColor Cyan

# --- 1. dirty check (before the long build) --------------------------------
$dirtyLines = @(Invoke-Git status --porcelain --untracked-files=no | Where-Object { $_ })
$isDirty = $dirtyLines.Count -gt 0
if ($isDirty -and -not $AllowDirty) {
    throw "tracked tree is dirty - commit/stash first or pass -AllowDirty:`n  $($dirtyLines -join "`n  ")"
}
if ($isDirty) { Write-Host "warning: dirty tracked tree (-AllowDirty):`n  $($dirtyLines -join "`n  ")" -ForegroundColor Yellow }

# --- 2. build, exactly as run.ps1 -------------------------------------------
$subStatus = @(Invoke-Git submodule status)
if (-not $SkipBuild) {
    if (@($subStatus | Where-Object { $_ -match '^-' }).Count -gt 0) {
        # Only when a submodule is missing: update --init could move an already
        # checked-out submodule off a deliberately different commit (-AllowDirty).
        Update-Submodules -RepoRoot $repoRoot
        $subStatus = @(Invoke-Git submodule status)
    }
    if (Test-DllLocked -Path $dllPath) {
        throw "librg_godot.dll appears to be locked - a Godot process likely has the project open:`n  $dllPath`nClose Godot and re-run make_release (or pass -SkipBuild to snapshot the DLL as it is)."
    }
    Remove-RgGodotStaleDll -DllPath $dllPath -Preset $Preset
    Enter-VsDevShell
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    if (-not (Test-RgConfigured -BuildDir $buildDir)) {
        Write-Host "`n-- configure ($Preset, RG_BUILD_GODOT_EXTENSION=ON) --" -ForegroundColor Cyan
        $rc = Invoke-RgCMakeConfigure -RepoRoot $repoRoot -BuildDir $buildDir -Preset $Preset -WithGodotExtension
        if ($rc -ne 0) { throw "cmake configure failed (exit $rc) - no snapshot written" }
    }
    Write-Host "`n-- build --" -ForegroundColor Cyan
    $rc = Invoke-RgCMakeBuild -BuildDir $buildDir -Targets @('rg_godot')
    $sw.Stop()
    if ($rc -ne 0) { throw "cmake build failed (exit $rc) - no snapshot written" }
    if (-not (Test-Path $dllPath)) { throw "build reported success but $dllPath is missing - no snapshot written" }
    Set-RgGodotPresetStamp -DllPath $dllPath -Preset $Preset
    Write-Host "build ok in $($sw.Elapsed.ToString('mm\:ss\.ff'))" -ForegroundColor Green
} else {
    Write-Host "`n-- skipping build (-SkipBuild) --" -ForegroundColor Cyan
    if (-not (Test-Path $dllPath)) { throw "-SkipBuild given but $dllPath does not exist" }
}
$builtPreset = Get-RgGodotBuiltPreset -DllPath $dllPath
if ($SkipBuild -and $builtPreset -and $builtPreset -ne $Preset) {
    Write-Host "warning: the DLL was last built by preset '$builtPreset', not '$Preset'" -ForegroundColor Yellow
}

# The snapshot's .godot/ import cache (extension_list.cfg) must exist.
$godotConsole = Find-GodotConsoleExe
$godotGui     = Find-GodotExe
Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotConsole

# --- 3. identity -------------------------------------------------------------
$sha      = (Invoke-Git rev-parse HEAD | Select-Object -First 1).Trim()
$shortSha = (Invoke-Git rev-parse --short=7 HEAD | Select-Object -First 1).Trim()
$subject  = (Invoke-Git log -1 --format=%s | Select-Object -First 1).Trim()
$stamp    = Get-Date -Format 'yyyyMMdd-HHmm'
$name     = "${stamp}_$shortSha" + $(if ($isDirty) { '-dirty' } else { '' })
$final    = Join-Path $OutRoot $name
$partial  = "$final.partial"

New-Item -ItemType Directory -Force -Path $OutRoot | Out-Null
if (Test-Path $final) { throw "snapshot $final already exists (same minute and commit) - wait a minute or delete it" }
# Leftovers of an interrupted earlier run.
Get-ChildItem -Path $OutRoot -Directory -Filter '*.partial' -ErrorAction SilentlyContinue | ForEach-Object {
    if ($_.FullName.StartsWith($OutRoot, [System.StringComparison]::OrdinalIgnoreCase)) { Remove-Item -Recurse -Force $_.FullName }
}

# --- 4. copy -----------------------------------------------------------------
Write-Host "`n-- copy -> $final --" -ForegroundColor Cyan
New-Item -ItemType Directory -Force -Path $partial | Out-Null
$copied = Copy-RgReleaseTrees -RepoRoot $repoRoot -Destination $partial

# --- 5. launchers + RELEASE.txt ----------------------------------------------
$g2mCache = if ($env:RG_G2M_CACHE) { $env:RG_G2M_CACHE } else { [System.IO.Path]::GetFullPath((Join-Path $repoRoot '..\geo2map_cache')) }
$g2mHome  = if ($env:RG_G2M_HOME)  { $env:RG_G2M_HOME }  else { Join-Path $repoRoot 'cache\g2m\home-r1' }

function New-LauncherText([string]$extraArgs) {
    $lines = @(
        '@echo off',
        'rem racing_game release snapshot launcher (written by tools/make_release.ps1).',
        'rem Runs Godot on THIS snapshot''s game\ folder. The geo2map stores are shared, not copied;',
        'rem set RG_G2M_CACHE / RG_G2M_HOME beforehand to point somewhere else.',
        'setlocal',
        "if not defined RG_G2M_CACHE set ""RG_G2M_CACHE=$g2mCache""",
        "if not defined RG_G2M_HOME set ""RG_G2M_HOME=$g2mHome""",
        "set ""RG_GODOT=$godotGui""",
        'if not exist "%RG_GODOT%" (',
        '  echo Godot not found at %RG_GODOT% - edit RG_GODOT in this file.',
        '  pause',
        '  exit /b 1',
        ')',
        ('"%RG_GODOT%" --path "%~dp0game" -- ' + $extraArgs + '%*')
    )
    return ($lines -join "`r`n") + "`r`n"
}
Set-Content -Path (Join-Path $partial 'play.cmd')       -Encoding ASCII -Value (New-LauncherText '')
Set-Content -Path (Join-Path $partial 'play_drive.cmd') -Encoding ASCII -Value (New-LauncherText '--drive ')

# Store info (best effort): which source scope the default world expects and
# which derived release ids (scopes) the derived store already holds.
$storeInfo = New-Object System.Collections.Generic.List[string]
try {
    $wcPath = Join-Path $repoRoot 'data\world\world_config_rhein_main_corridor.json'
    $wc = Get-Content -Raw -Path $wcPath | ConvertFrom-Json
    $expand = { param($s) $s.Replace('${RG_G2M_CACHE}', $g2mCache).Replace('${RG_G2M_HOME}', $g2mHome) }
    $srcDir = & $expand $wc.source_store.dir
    $derDir = & $expand $wc.derived_store.dir
    $derDb  = Join-Path $derDir ($wc.derived_store.name + '.sqlite3')
    $storeInfo.Add("default world config : data/world/world_config_rhein_main_corridor.json")
    $storeInfo.Add("source store         : $([System.IO.Path]::GetFullPath($srcDir)) (scope $($wc.source_store.scope))")
    $storeInfo.Add("derived store        : $derDb")
    $py = Get-Command python -ErrorAction SilentlyContinue
    if ($py -and (Test-Path $derDb)) {
        $code = "import sqlite3,sys`nc=sqlite3.connect('file:'+sys.argv[1].replace('\\','/')+'?mode=ro',uri=True,timeout=5)`nfor s,n in c.execute('select scope,count(*) from tiles group by scope'): print(s[:12],n)"
        $rows = @(& $py.Source -I -c $code $derDb)
        if ($LASTEXITCODE -eq 0 -and $rows.Count -gt 0) {
            $storeInfo.Add("derived release ids present (id prefix, derived tile count). The id this build uses is the")
            $storeInfo.Add("  geo2map pin's; a release id missing here is derived on first use (slow cold start):")
            foreach ($r in $rows) { $storeInfo.Add("    $r") }
        }
    }
} catch {
    $storeInfo.Add("(store info unavailable: $($_.Exception.Message))")
}

$subLines = @($subStatus | ForEach-Object { "  " + $_.Trim() })
$sizeBytes = (Get-ChildItem -Path $partial -Recurse -File -Force | Measure-Object -Property Length -Sum).Sum
$sizeMb = [math]::Round($sizeBytes / 1MB, 1)
$release = @(
    "racing_game release snapshot",
    "",
    "created      : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    "commit       : $sha",
    "subject      : $subject",
    "dirty tree   : $(if ($isDirty) { 'YES (-AllowDirty): ' + ($dirtyLines -join '; ') } else { 'no' })",
    "submodules   :",
    $subLines,
    "dll preset   : $(if ($builtPreset) { $builtPreset } else { 'unknown' })$(if ($SkipBuild) { ' (-SkipBuild)' } else { '' })",
    "size         : $sizeMb MB",
    "",
    "HOW TO PLAY",
    "  play.cmd          game shell (boot splash + main menu)",
    "  play_drive.cmd    straight into the real world, Drive mode (--drive)",
    "  extra arguments are passed on to the game, e.g. play.cmd --flat",
    "  ..\latest.cmd     (next to the snapshots) starts the newest snapshot",
    "  Godot is $godotGui",
    "",
    "SHARED WITH THE DEV CHECKOUT (not copied)",
    "  RG_G2M_CACHE = $g2mCache",
    "  RG_G2M_HOME  = $g2mHome",
    "  (both are only defaults in play.cmd: set them before starting to use other stores)",
    "  The derived store is written to by whoever runs; Godot's user dir (settings, garage work,",
    "  thumbnails) is the same as the dev checkout's (same project name).",
    $storeInfo,
    "",
    "CONTENTS",
    ($copied | ForEach-Object { "  $_" }),
    "  play.cmd, play_drive.cmd, RELEASE.txt",
    "NOT copied: .git, out/build, cache/ (geo2map stores)."
)
Set-Content -Path (Join-Path $partial 'RELEASE.txt') -Encoding ASCII -Value ($release | ForEach-Object { $_ })

# --- 6. publish, latest.cmd, prune -------------------------------------------
Rename-Item -Path $partial -NewName $name
foreach ($pair in @(@('latest.cmd', 'play.cmd'), @('latest_drive.cmd', 'play_drive.cmd'))) {
    $text = "@echo off`r`nrem written by tools/make_release.ps1 - starts the newest snapshot: $name`r`ncall `"$final\$($pair[1])`" %*`r`n"
    Set-Content -Path (Join-Path $OutRoot $pair[0]) -Encoding ASCII -Value $text
}

$snaps = @(Get-ChildItem -Path $OutRoot -Directory | Where-Object { $_.Name -match '^\d{8}-\d{4}_[0-9a-f]{7}' -and $_.Name -notlike '*.partial' } |
    Sort-Object Name -Descending)
$pruned = @()
for ($i = $Keep; $i -lt $snaps.Count; $i++) {
    $victim = $snaps[$i]
    if ($victim.Name -eq $name) { continue }
    if (-not $victim.FullName.StartsWith($OutRoot, [System.StringComparison]::OrdinalIgnoreCase)) { continue }
    try { Remove-Item -Recurse -Force -Path $victim.FullName; $pruned += $victim.Name }
    catch { Write-Host "warning: could not prune $($victim.Name) (in use?): $($_.Exception.Message)" -ForegroundColor Yellow }
}

Write-Host "`n=== snapshot ready ===" -ForegroundColor Green
Write-Host "path   : $final"
Write-Host "size   : $sizeMb MB"
Write-Host "commit : $sha$(if ($isDirty) { ' (dirty)' })"
Write-Host "launch : $final\play.cmd   or   $OutRoot\latest.cmd"
if ($pruned.Count -gt 0) { Write-Host "pruned : $($pruned -join ', ')" }
