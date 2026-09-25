#Requires -Version 5.1
<#
.SYNOPSIS
    Verifies the racing_game development toolchain on Windows.

.DESCRIPTION
    -CheckOnly ONLY (there is no install mode in this repo, unlike
    physics_sim's own tools/setup_dev_env.ps1 - R0's dev machine already has
    everything this repo needs: VS 2022 Build Tools/clang-cl and Godot were
    installed for physics_sim's own development, CMake/Ninja/Git likewise).
    Prints each tool's version and exits non-zero if anything required is
    missing, so CI/onboarding has one command to run first.

.PARAMETER CheckOnly
    Accepted for interface parity with physics_sim's own script (which has
    an install mode); this script always behaves as -CheckOnly and ignores
    the switch's value - present so a caller can pass it unconditionally.
#>
[CmdletBinding()]
param(
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$script:AllOk = $true

function Write-Section($title) {
    Write-Host ""
    Write-Host "=== $title ===" -ForegroundColor Cyan
}

function Write-Ok($msg) {
    Write-Host "  [OK]   $msg" -ForegroundColor Green
}

function Write-Missing($msg) {
    Write-Host "  [MISS] $msg" -ForegroundColor Red
    $script:AllOk = $false
}

function Test-CommandVersion {
    param(
        [string]$Name,
        [string]$Command,
        [string[]]$ArgumentList = @('--version')
    )
    try {
        $out = & $Command @ArgumentList 2>&1 | Select-Object -First 1
        if ($out) {
            Write-Ok "$Name -> $out"
            return $true
        } else {
            Write-Missing "$Name (no output from '$Command $ArgumentList')"
            return $false
        }
    } catch {
        Write-Missing "$Name (command '$Command' not found or failed: $($_.Exception.Message))"
        return $false
    }
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

# Same winget-package-directory probe as physics_sim's own
# tools/setup_dev_env.ps1's Godot check.
function Find-GodotConsoleExe {
    $godotDir = Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Directory -Filter 'GodotEngine.GodotEngine*' -ErrorAction SilentlyContinue
    return $godotDir | Get-ChildItem -Filter '*_console.exe' -Recurse -ErrorAction SilentlyContinue | Select-Object -First 1
}

Write-Section "Windows toolchain"
Test-CommandVersion -Name 'git'   -Command 'git'   -ArgumentList @('--version') | Out-Null
Test-CommandVersion -Name 'cmake' -Command 'cmake' -ArgumentList @('--version') | Out-Null
Test-CommandVersion -Name 'ninja' -Command 'ninja' -ArgumentList @('--version') | Out-Null

$vsPath = Get-VsInstallPath
if ($vsPath) {
    Write-Ok "VS 2022 Build Tools -> $vsPath"
} else {
    Write-Missing "VS 2022 Build Tools (VC.Tools.x86.x64 workload) not found via vswhere"
}

$clangCl = Get-ClangClPath
if ($clangCl) {
    $ver = & $clangCl --version | Select-Object -First 1
    Write-Ok "clang-cl -> $clangCl ($ver)"
} else {
    Write-Missing "clang-cl not found under VC\Tools\Llvm\x64\bin (VC.Llvm.Clang / VC.Llvm.ClangToolset components)"
}

$godotExe = Find-GodotConsoleExe
if ($godotExe) {
    Write-Ok "Godot -> $($godotExe.FullName)"
} else {
    Write-Missing "Godot (GodotEngine.GodotEngine winget package) not found"
}

# git ≥2.38's file-transport protocol guard: this repo's external/physics_sim
# submodule uses a local absolute file:// URL (decision D9), which every
# submodule update MUST pass -c protocol.file.allow=always for, or it fails
# with "fatal: transport 'file' not allowed" - checked here so a missing/old
# git that predates the guard, or one some other config has locked down
# further, is caught before the first clone rather than mid-CI.
try {
    $gitVerOut = (& git --version) -replace '[^\d\.]', ' '
    $gitVer = [version]($gitVerOut.Trim().Split(' ')[0])
    if ($gitVer -ge [version]'2.38.0') {
        Write-Ok "git submodule file-transport guard applies (git $gitVer >= 2.38) - run.ps1/ci.ps1 always pass -c protocol.file.allow=always"
    } else {
        Write-Ok "git $gitVer predates the file-transport guard (< 2.38) - -c protocol.file.allow=always is harmless but unnecessary"
    }
} catch {
    Write-Missing "could not determine git version to check the submodule file-transport guard"
}

Write-Section "Result"
if ($script:AllOk) {
    Write-Host "All tools present." -ForegroundColor Green
    exit 0
} else {
    Write-Host "One or more tools are missing. This script installs nothing - see physics_sim/tools/setup_dev_env.ps1 for the install-mode pattern if a fresh machine ever needs it." -ForegroundColor Red
    exit 1
}
