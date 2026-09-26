#Requires -Version 5.1
<#
.SYNOPSIS
    Shared helpers for run.ps1, tools/smoke_test.ps1 and tools/ci.ps1: VS
    2022 dev-env entry, Godot executable lookup, cmake configure/build
    wrappers, and the submodule-update helper. Dot-sourced, not run
    directly - modelled directly on physics_sim's own
    adapters/godot/godot_common.ps1 (read-only reference: same
    Enter-VsDevShell/Find-Godot*Exe bodies, since both repos need the exact
    same VS-dev-shell entry and WinGet Godot lookup).

.DESCRIPTION
    physics_sim's own godot_common.ps1 is NOT dot-sourced directly (that
    repo is READ-ONLY from this session per this task's own rules, and
    another session actively commits to it) - the functions below are a
    fresh copy, kept here instead.
#>

function Enter-VsDevShell {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (-not (Test-Path $vswhere)) { throw "vswhere.exe not found; VS 2022 Build Tools not installed" }
    $vsPath = & $vswhere -products * -latest -property installationPath
    if (-not $vsPath) { throw "vswhere found no VS installation" }
    $vcvars = Join-Path $vsPath "VC\Auxiliary\Build\vcvars64.bat"
    if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found under $vsPath" }
    # See physics_sim's godot_common.ps1 for why stderr is redirected to
    # $null here: vcvars64.bat's own internal vswhere.exe call can print a
    # benign "not recognized" line to stderr that would otherwise abort
    # this script under $ErrorActionPreference = 'Stop'.
    $previousEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    $envLines = cmd.exe /c "`"$vcvars`" && set"
    $ErrorActionPreference = $previousEap
    foreach ($line in $envLines) {
        if ($line -match '^([^=]+)=(.*)$') {
            [Environment]::SetEnvironmentVariable($Matches[1], $Matches[2], 'Process')
        }
    }
    if (-not (Get-Command clang-cl -ErrorAction SilentlyContinue)) {
        throw "vcvars64.bat ran but clang-cl is still not on PATH"
    }
}

function Find-GodotConsoleExe {
    $candidates = Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter "Godot_v*_console.exe" -ErrorAction SilentlyContinue
    if (-not $candidates -or $candidates.Count -eq 0) {
        throw "Godot console executable not found under WinGet packages (expected Godot_v4.7.2-stable_win64_console.exe)"
    }
    return $candidates[0].FullName
}

function Find-GodotExe {
    $candidates = Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter "Godot_v*.exe" -ErrorAction SilentlyContinue |
        Where-Object { $_.Name -notlike '*_console.exe' }
    if (-not $candidates -or $candidates.Count -eq 0) {
        throw "Godot executable not found under WinGet packages (expected Godot_v4.7.2-stable_win64.exe)"
    }
    return $candidates[0].FullName
}

# ---------------------------------------------------------------------------
# cmake configure/build wrappers. Piped through Out-Host for the same
# reason as physics_sim's own godot_common.ps1 (see that file's comment):
# without it, cmake's own stdout lines get concatenated into this
# function's `return $LASTEXITCODE` value instead of printing live.
# ---------------------------------------------------------------------------
function Invoke-RgCMakeConfigure {
    param(
        [Parameter(Mandatory)][string]$RepoRoot,
        [Parameter(Mandatory)][string]$BuildDir,
        [string]$Preset = 'debug',
        [switch]$WithGodotExtension
    )
    $godotFlag = if ($WithGodotExtension) { 'ON' } else { 'OFF' }
    & cmake -S $RepoRoot -B $BuildDir --preset $Preset -DRG_BUILD_GODOT_EXTENSION=$godotFlag | Out-Host
    return $LASTEXITCODE
}

function Invoke-RgCMakeBuild {
    param(
        [Parameter(Mandatory)][string]$BuildDir,
        [string[]]$Targets
    )
    if ($Targets -and $Targets.Count -gt 0) {
        & cmake --build $BuildDir --target @Targets | Out-Host
    } else {
        & cmake --build $BuildDir | Out-Host
    }
    return $LASTEXITCODE
}

function Test-RgConfigured {
    param([Parameter(Mandatory)][string]$BuildDir)
    return Test-Path (Join-Path $BuildDir 'build.ninja')
}

# Same file-lock probe as physics_sim's godot_common.ps1's Test-DllLocked -
# run.ps1's pre-build check so a running Godot instance produces a clear
# message instead of a cryptic link.exe failure partway through the build.
function Test-DllLocked {
    param([Parameter(Mandatory)][string]$Path)
    if (-not (Test-Path $Path)) { return $false }
    try {
        $stream = [System.IO.File]::Open($Path, [System.IO.FileMode]::Open, [System.IO.FileAccess]::ReadWrite, [System.IO.FileShare]::None)
        $stream.Close()
        return $false
    } catch [System.IO.IOException] {
        return $true
    }
}

# A fresh checkout's game/ has no .godot/ cache. Godot's GDExtension loader
# reads the list of .gdextension files to load from game/.godot/extension_list.cfg,
# which only the EDITOR writes (during its own project-import scan) - a
# --headless (non-editor) run never populates it and, discovered the hard
# way during R0, silently loads NO extension at all if it's missing (no
# warning/error either way - ClassDB.instantiate("RgSimulation") just
# returns null later, with no diagnostic pointing back at this cause). Godot
# 4's own --import CLI flag ("start the editor, wait for import, quit") is
# the documented one-shot fix; on this project it reliably writes
# extension_list.cfg but then the editor's OWN shutdown sequence can crash
# (STATUS_ACCESS_VIOLATION, observed during R0 - after the cache file is
# already written, so it doesn't matter here) - exit code is therefore
# deliberately NOT checked, only the cache file's presence afterward.
function Ensure-GodotProjectImported {
    param(
        [Parameter(Mandatory)][string]$GameDir,
        [Parameter(Mandatory)][string]$GodotExe
    )
    $extensionList = Join-Path $GameDir '.godot\extension_list.cfg'
    if (Test-Path $extensionList) { return }
    Write-Host "-- importing Godot project (no .godot/extension_list.cfg yet - first run against this checkout) --" -ForegroundColor Cyan
    $previousEap = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    & $GodotExe --headless --path $GameDir --import *>$null
    $ErrorActionPreference = $previousEap
    if (-not (Test-Path $extensionList)) {
        throw "Godot --import did not produce $extensionList - GDExtension will not load headless"
    }
}

# Ensures BOTH submodules (external/physics_sim, external/geo2map_engine -
# PLAN.md R2.0) are checked out, ALWAYS passing -c protocol.file.allow=always
# (D9: both submodules' URLs are local absolute file:// paths - git >= 2.38
# refuses "transport 'file' not allowed" without this flag on every
# clone/update, confirmed empirically during R0 setup; every script in this
# repo that touches either submodule must pass it). No path argument is
# passed to `submodule update`, so this one git invocation updates every
# submodule listed in .gitmodules - a single function has covered both since
# geo2map_engine was added, nothing else needed to change here.
function Update-Submodules {
    param([Parameter(Mandatory)][string]$RepoRoot)
    Push-Location $RepoRoot
    try {
        & git -c protocol.file.allow=always submodule update --init --recursive | Out-Host
        if ($LASTEXITCODE -ne 0) {
            throw "git submodule update failed (exit $LASTEXITCODE) - external/physics_sim / external/geo2map_engine not checked out"
        }
    } finally {
        Pop-Location
    }
}
