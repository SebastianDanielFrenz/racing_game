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

.PARAMETER Frame
    Also install / verify the Steam Frame (Linux ARM64) toolchain, used by
    tools\frame_install.ps1 (all from official sources: Valve, godotengine,
    Ubuntu):
      - WSL packages: clang lld cmake ninja-build rsync python3 curl qemu-user
      - Valve's Steam Runtime 3c ARM64 SDK sysroot (~/sdk/steamrt3c-arm64-sdk in
        WSL; glibc 2.31, the oldest runtime a Frame devkit title runs in), SHA256
        checked against Valve's SHA256SUMS, absolute symlinks made relative
        (tools/frame/install_sysroot.sh)
      - the official Godot 4.7.2 linux.arm64 binary (godotengine/godot release,
        SHA512 checked) under <repo>\out\frame
        With -CheckOnly nothing is installed. Without -Frame the Frame toolchain is
    only reported (as info, never as missing).
#>
[CmdletBinding()]
param(
    [switch]$CheckOnly,
    [switch]$WithSiblings,
    [switch]$SkipPython,
    [switch]$Frame
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
# Steam Frame (Linux ARM64) toolchain: tools\frame_install.ps1
# ---------------------------------------------------------------------------
$FrameDir = Join-Path $RepoRoot 'out\frame'   # under the repo (gitignored out/), so WSL and Windows both see it
$FrameGodotName = "Godot_v$GodotVersion-stable_linux.arm64"
$FrameGodotZipUrl = "https://github.com/godotengine/godot/releases/download/$GodotVersion-stable/$FrameGodotName.zip"
$FrameGodotShaUrl = "https://github.com/godotengine/godot/releases/download/$GodotVersion-stable/SHA512-SUMS.txt"
$FrameWslPackages = @('clang', 'lld', 'cmake', 'ninja-build', 'rsync', 'python3', 'curl', 'qemu-user')

function Invoke-FrameWsl([string]$Script, [switch]$Root) {
    # bash script text -> a temp file WSL can read (avoids quoting through wsl -e).
    $tmp = Join-Path $env:TEMP ("rg_frame_" + [guid]::NewGuid().ToString('N') + '.sh')
    [System.IO.File]::WriteAllText($tmp, $Script.Replace("`r`n", "`n"), (New-Object System.Text.UTF8Encoding($false)))
    try {
        $wslPath = (& wsl wslpath -a $tmp.Replace('\', '/')) | Select-Object -First 1
        if ($Root) { $out = & wsl -u root -- bash $wslPath 2>&1 } else { $out = & wsl -- bash $wslPath 2>&1 }
        $global:FrameWslExit = $LASTEXITCODE
        return $out
    } finally { Remove-Item -Force $tmp -ErrorAction SilentlyContinue }
}

Write-Section "Steam Frame (Linux ARM64) toolchain$(if (-not $Frame) { ' (info only; -Frame installs)' })"
$frameOk = $true
function Write-FrameMissing($msg) { if ($Frame) { Write-Missing $msg } else { Write-Info "$msg (install: tools\setup_dev_env.ps1 -Frame)" }; $script:frameOk = $false }

$haveWsl = $false
try { $haveWsl = [bool]((wsl -l -q 2>$null) -replace "`0", "" | Select-String 'Ubuntu') } catch {}
if (-not $haveWsl) {
    Write-FrameMissing "WSL Ubuntu distro"
} else {
    # 1. WSL packages
    $pkgList = $FrameWslPackages -join ' '
    $check = "for p in $pkgList; do dpkg -s `$p >/dev/null 2>&1 || echo `$p; done"
    $missing = @(Invoke-FrameWsl $check | Where-Object { $_ })
    if ($missing.Count -gt 0 -and $Frame -and -not $CheckOnly) {
        Write-Host "apt-get install (as root in WSL): $($missing -join ' ')"
        Invoke-FrameWsl "apt-get install -y $($missing -join ' ')" -Root | Out-Host
        $missing = @(Invoke-FrameWsl $check | Where-Object { $_ })
    }
    if ($missing.Count -eq 0) { Write-Ok "WSL packages: $pkgList" } else { Write-FrameMissing "WSL packages: $($missing -join ' ')" }

    # 1b. the newest GCC cross libstdc++ (headers + libstdc++.a, Ubuntu package
    # libstdc++-N-dev-arm64-cross): the sniper SDK's own GCC 10 libstdc++ is too
    # old for the C++20 library features the code base uses; it is linked statically.
    $cxLs = 'ls -d /usr/lib/gcc-cross/aarch64-linux-gnu/*/ 2>/dev/null | while read d; do [ -f "$d/libstdc++.a" ] && echo "$d"; done | sort -V | tail -1'
    $cxHave = @(Invoke-FrameWsl $cxLs | Where-Object { $_ }) | Select-Object -First 1
    if (-not $cxHave -and $Frame -and -not $CheckOnly) {
        $cxPkg = @(Invoke-FrameWsl 'apt-cache pkgnames libstdc++- 2>/dev/null | grep -E "^libstdc\+\+-[0-9]+-dev-arm64-cross$" | sort -V | tail -1' | Where-Object { $_ }) | Select-Object -First 1
        if ($cxPkg) {
            Write-Host "apt-get install (as root in WSL): $cxPkg"
            Invoke-FrameWsl "apt-get install -y $cxPkg" -Root | Out-Host
            $cxHave = @(Invoke-FrameWsl $cxLs | Where-Object { $_ }) | Select-Object -First 1
        }
    }
    if ($cxHave) { Write-Ok "aarch64 cross libstdc++ (static) -> $cxHave" } else { Write-FrameMissing "aarch64 cross libstdc++ (apt package libstdc++-N-dev-arm64-cross in WSL)" }

    # 2. Steam Runtime 3c ARM64 SDK sysroot
    $sysrootScript = "/mnt/" + (($RepoRoot.Substring(0,1).ToLower())) + $RepoRoot.Substring(2).Replace('\', '/') + "/tools/frame/install_sysroot.sh"
    $chk = Invoke-FrameWsl "bash '$sysrootScript' --check"
    if ($FrameWslExit -ne 0 -and $Frame -and -not $CheckOnly) {
        Write-Host "installing Valve's Steam Runtime 3c ARM64 SDK sysroot (about 730 MB download, 2.5 GB on disk)..."
        Invoke-FrameWsl "bash '$sysrootScript'" | Out-Host
        if ($FrameWslExit -ne 0) { throw "sysroot install failed (exit $FrameWslExit)" }
        $chk = Invoke-FrameWsl "bash '$sysrootScript' --check"
    }
    if ($FrameWslExit -eq 0) { Write-Ok "ARM64 sysroot (WSL) -> $($chk | Select-Object -Last 1)" } else { Write-FrameMissing "Steam Runtime 3c ARM64 SDK sysroot (WSL ~/sdk/steamrt3c-arm64-sdk)" }
}

# 3. Godot linux.arm64
$frameGodot = Join-Path $FrameDir "godot-$GodotVersion-linux.arm64\$FrameGodotName"
if (-not (Test-Path $frameGodot) -and $Frame -and -not $CheckOnly) {
    New-Item -ItemType Directory -Force -Path (Join-Path $FrameDir 'dl') | Out-Null
    $zip = Join-Path $FrameDir "dl\$FrameGodotName.zip"
    $sums = Join-Path $FrameDir 'dl\SHA512-SUMS.txt'
    Write-Host "downloading $FrameGodotName.zip from godotengine/godot..."
    & curl.exe -fsSL -o $zip $FrameGodotZipUrl
    if ($LASTEXITCODE -ne 0) { throw "download of $FrameGodotZipUrl failed" }
    & curl.exe -fsSL -o $sums $FrameGodotShaUrl
    if ($LASTEXITCODE -ne 0) { throw "download of $FrameGodotShaUrl failed" }
    $want = ((Get-Content $sums | Where-Object { $_ -match [regex]::Escape("$FrameGodotName.zip") + '$' }) -split '\s+')[0].ToLower()
    $got = (Get-FileHash -Algorithm SHA512 $zip).Hash.ToLower()
    if (-not $want -or $want -ne $got) { throw "SHA512 mismatch for $FrameGodotName.zip (want '$want', got '$got')" }
    $dest = Join-Path $FrameDir "godot-$GodotVersion-linux.arm64"
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    Expand-Archive -Path $zip -DestinationPath $dest -Force
}
if (Test-Path $frameGodot) { Write-Ok "Godot $GodotVersion linux.arm64 -> $frameGodot" } else { Write-FrameMissing "Godot $GodotVersion linux.arm64 binary ($FrameGodotZipUrl)" }

$sshExe = Get-Command ssh.exe -ErrorAction SilentlyContinue
if ($sshExe) { Write-Ok "ssh.exe -> $($sshExe.Source) (frame_install.ps1 also needs `ssh steamos@frame` to work without a password)" } else { Write-FrameMissing "OpenSSH client (ssh.exe)" }
if ($Frame -and -not $frameOk) { $script:AllOk = $false }

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
