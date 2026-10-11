#Requires -Version 5.1
<#
.SYNOPSIS
    Sets up (or verifies) a Windows machine for racing_game development.

.DESCRIPTION
    New machine, from a fresh clone:

        git clone --recursive https://github.com/SebastianDanielFrenz/racing_game.git
        racing_game\setup.cmd

    Install mode (default) installs via winget whatever is missing: VS 2022
    Build Tools (C++ workload + clang-cl), CMake, Ninja, Godot 4.7.2 and
    Python 3.13 (Python only regenerates data; the game does not need it).
    It then checks out both submodules (external/physics_sim,
    external/geo2map_engine) and reports which worlds have their map data.
    Re-running is safe: installed tools are skipped.

    Map data is not in Git (large, partly licence-bound). Copy it from an
    existing machine:
      - home world:        <repo>\cache\g2m\home-r1          (RG_G2M_HOME)
      - rhein_main worlds: <repo>\..\geo2map_cache\<store>   (RG_G2M_CACHE)
    The flat test scene needs no map data.

    -CheckOnly installs nothing and exits non-zero if a tool is missing.

.PARAMETER CheckOnly
    Verify only; install nothing.

.PARAMETER WithSiblings
    Also clone physics_sim and geo2map_engine next to this repo
    (<repo>\..\physics_sim, <repo>\..\geo2map_engine) for working on them
    directly. The game itself only needs the submodules.

.PARAMETER SkipPython
    Do not install Python.
#>
[CmdletBinding()]
param(
    [switch]$CheckOnly,
    [switch]$WithSiblings,
    [switch]$SkipPython
)

$ErrorActionPreference = 'Stop'
$script:AllOk = $true
$RepoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$ParentDir = Split-Path $RepoRoot -Parent
$GodotVersion = '4.7.2'
$GitHubOwner = 'https://github.com/SebastianDanielFrenz'

function Write-Section($title) { Write-Host ""; Write-Host "=== $title ===" -ForegroundColor Cyan }
function Write-Ok($msg) { Write-Host "  [OK]   $msg" -ForegroundColor Green }
function Write-Info($msg) { Write-Host "  [--]   $msg" -ForegroundColor Yellow }
function Write-Missing($msg) { Write-Host "  [MISS] $msg" -ForegroundColor Red; $script:AllOk = $false }

function Update-SessionPath {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

function Get-ToolVersion([string]$Command) {
    try { return (& $Command --version 2>&1 | Select-Object -First 1) } catch { return $null }
}

function Get-VsInstallPath {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { return $null }
    return & $vswhere -products * -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
}

function Get-ClangClPath {
    $vsPath = Get-VsInstallPath
    if (-not $vsPath) { return $null }
    $clangCl = Join-Path $vsPath 'VC\Tools\Llvm\x64\bin\clang-cl.exe'
    if (Test-Path $clangCl) { return $clangCl }
    return $null
}

# Same lookup as tools/common.ps1 (winget's portable-package folder).
function Find-GodotConsoleExe {
    Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter 'Godot_v*_console.exe' -ErrorAction SilentlyContinue |
        Select-Object -First 1
}

function Install-WingetPackage {
    param([string]$Id, [string[]]$ExtraArgs = @())
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "winget not found - install 'App Installer' from the Microsoft Store, then re-run"
    }
    Write-Host "  installing $Id via winget..."
    $wingetArgs = @('install', '--id', $Id, '-e', '--accept-package-agreements',
                    '--accept-source-agreements', '--disable-interactivity') + $ExtraArgs
    & winget @wingetArgs
    # -1978335189 = APPINSTALLER_CLI_ERROR_UPDATE_NOT_APPLICABLE (already installed, no newer version)
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne -1978335189) {
        throw "winget install of $Id failed with exit code $LASTEXITCODE"
    }
}

# ---------------------------------------------------------------------------
# Toolchain
# ---------------------------------------------------------------------------
Update-SessionPath

if (-not $CheckOnly) {
    Write-Section "Installing missing tools"
    if (-not (Get-ClangClPath)) {
        # clang-cl comes from the Build Tools' own LLVM components (same as
        # physics_sim/tools/setup_dev_env.ps1). Expect a UAC prompt.
        Install-WingetPackage -Id 'Microsoft.VisualStudio.2022.BuildTools' -ExtraArgs @(
            '--override', '"--wait --quiet --norestart --nocache --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended --add Microsoft.VisualStudio.Component.VC.Llvm.Clang --add Microsoft.VisualStudio.Component.VC.Llvm.ClangToolset"')
    }
    if (-not (Get-ToolVersion 'cmake')) { Install-WingetPackage -Id 'Kitware.CMake' }
    if (-not (Get-ToolVersion 'ninja')) { Install-WingetPackage -Id 'Ninja-build.Ninja' }
    if (-not (Find-GodotConsoleExe)) {
        try {
            Install-WingetPackage -Id 'GodotEngine.GodotEngine' -ExtraArgs @('--version', $GodotVersion)
        } catch {
            Write-Info "Godot $GodotVersion not available from winget ($($_.Exception.Message)); installing the latest"
            Install-WingetPackage -Id 'GodotEngine.GodotEngine'
        }
    }
    if (-not $SkipPython -and -not (Get-Command py -ErrorAction SilentlyContinue) -and -not (Get-Command python -ErrorAction SilentlyContinue)) {
        Install-WingetPackage -Id 'Python.Python.3.13'
    }
    Update-SessionPath
}

Write-Section "Toolchain"
foreach ($tool in 'git', 'cmake', 'ninja') {
    $v = Get-ToolVersion $tool
    if ($v) { Write-Ok "$tool -> $v" } else { Write-Missing $tool }
}
$vsPath = Get-VsInstallPath
if ($vsPath) { Write-Ok "VS 2022 Build Tools -> $vsPath" } else { Write-Missing "VS 2022 Build Tools (C++ workload)" }
$clangCl = Get-ClangClPath
if ($clangCl) { Write-Ok "clang-cl -> $clangCl" } else { Write-Missing "clang-cl (VS components VC.Llvm.Clang / VC.Llvm.ClangToolset)" }
$godot = Find-GodotConsoleExe
if ($godot) {
    if ($godot.Name -notlike "Godot_v$GodotVersion-*") { Write-Info "Godot $($godot.Name) is not the tested $GodotVersion" }
    Write-Ok "Godot -> $($godot.FullName)"
} else {
    Write-Missing "Godot (winget GodotEngine.GodotEngine)"
}
if (Get-Command py -ErrorAction SilentlyContinue) { Write-Ok "python -> $(& py --version 2>&1)" }
elseif (Get-Command python -ErrorAction SilentlyContinue) { Write-Ok "python -> $(& python --version 2>&1)" }
else { Write-Info "python not found (optional: only data generators need it)" }

# ---------------------------------------------------------------------------
# Source: submodules (and optional sibling clones)
# ---------------------------------------------------------------------------
Write-Section "Source"
if (-not $CheckOnly) {
    # protocol.file.allow: harmless for the GitHub URLs, needed where a
    # machine's local config points the submodules at local repositories.
    & git -C $RepoRoot -c protocol.file.allow=always submodule update --init --recursive
    if ($LASTEXITCODE -ne 0) { throw "git submodule update failed (exit $LASTEXITCODE)" }
}
foreach ($sub in 'external/physics_sim', 'external/geo2map_engine') {
    if (Test-Path (Join-Path $RepoRoot "$sub\CMakeLists.txt")) { Write-Ok "$sub checked out" }
    else { Write-Missing "$sub not checked out (git submodule update --init --recursive)" }
}
foreach ($repo in 'physics_sim', 'geo2map_engine') {
    $dir = Join-Path $ParentDir $repo
    if (Test-Path (Join-Path $dir '.git')) {
        Write-Ok "sibling $dir"
    } elseif ($WithSiblings -and -not $CheckOnly) {
        & git clone "$GitHubOwner/$repo.git" $dir
        if ($LASTEXITCODE -ne 0) { throw "git clone of $repo failed (exit $LASTEXITCODE)" }
        Write-Ok "cloned $dir"
    } else {
        Write-Info "no sibling $dir (only needed to work on $repo directly; -WithSiblings clones it)"
    }
}

# ---------------------------------------------------------------------------
# Map data (copied by hand; never in Git)
# ---------------------------------------------------------------------------
Write-Section "Map data"
Write-Ok "flat test scene: needs no map data (tools\run.cmd -Flat)"
$homeStore = if ($env:RG_G2M_HOME) { $env:RG_G2M_HOME } else { Join-Path $RepoRoot 'cache\g2m\home-r1' }
$cacheRoot = if ($env:RG_G2M_CACHE) { $env:RG_G2M_CACHE } else { Join-Path $ParentDir 'geo2map_cache' }
$worlds = @(
    @{ Name = 'home (default world)'; Dir = $homeStore },
    @{ Name = 'rhein_main_corridor (VR default)'; Dir = (Join-Path $cacheRoot 'rhein-main-corridor-r2') },
    @{ Name = 'rhein_main_hessen'; Dir = (Join-Path $cacheRoot 'rhein-main-hessen-r1') }
)
foreach ($w in $worlds) {
    if (Test-Path (Join-Path $w.Dir 'baked\tiles.sqlite3')) { Write-Ok "$($w.Name): $($w.Dir)" }
    else { Write-Info "$($w.Name): no baked store at $($w.Dir) - copy it from a machine that has it" }
}

# ---------------------------------------------------------------------------
Write-Section "Result"
if ($script:AllOk) {
    Write-Host "Ready. First build and run (builds everything, fetches CMake dependencies):" -ForegroundColor Green
    Write-Host "  tools\run.cmd -Flat        flat test scene"
    Write-Host "  tools\run.cmd              real world (needs the home map data)"
    exit 0
} else {
    $hint = if ($CheckOnly) { 'Re-run without -CheckOnly to install.' } else { 'See the [MISS] lines above.' }
    Write-Host "Something is missing. $hint" -ForegroundColor Red
    exit 1
}
