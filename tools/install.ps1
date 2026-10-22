#Requires -Version 5.1
<#
.SYNOPSIS
    Installs or updates racing_game on Windows from GitHub: the latest commit on
    main (default), or one specific commit on whichever branch it sits.

.DESCRIPTION
    Self-contained: download this one file and run it; it needs nothing from a
    checkout. In order, it:
      1. installs Git (winget) when it is missing;
      2. clones https://github.com/SebastianDanielFrenz/racing_game.git into
         -InstallDir, or updates the clone that is already there. It refuses a
         folder holding something else, a different origin, tracked local
         changes and a local branch other than main;
      3. checks out main (fast-forward to origin/main) or, with -Commit, that
         commit as a detached HEAD. All branches are fetched first, and a full
         SHA that sits on no branch (an unmerged pull request) is fetched
         directly. Then it updates the submodules and Git LFS files;
      4. runs the checkout's tools\setup_dev_env.ps1 (VS Build Tools + clang-cl,
         CMake, Ninja, Godot 4.7.2, Python via winget; skip with -SkipToolchain);
      5. builds the rg_godot GDExtension (preset relwithdebinfo, like
         tools\run.ps1) and runs Godot's one-time import;
      6. writes <InstallDir>\.local-install\play.cmd (start the game; the
         arguments are those of tools\run.cmd, e.g. -Flat, -Drive, -VR) and
         update.cmd (re-runs this installer: main again, or -Commit <sha>).

    Map data is not in Git. Copy the geo2map stores separately (README.md);
    the flat test world (-Flat) works without them.

.PARAMETER Commit
    A commit ID (full or abbreviated SHA) to install instead of main. A tag or
    origin/<branch> also works. Omit it to install the latest main.
.PARAMETER InstallDir
    The checkout folder. Default: $env:RG_INSTALL_DIR, else
    %LOCALAPPDATA%\racing_game.
.PARAMETER SkipToolchain
    Do not run tools\setup_dev_env.ps1 (the toolchain is already installed).
.PARAMETER SkipBuild
    Only fetch and check out; do not build.

.EXAMPLE
    # latest main into the default folder
    powershell -ExecutionPolicy Bypass -File install.ps1
.EXAMPLE
    # one specific commit into D:\games\racing_game
    powershell -ExecutionPolicy Bypass -File install.ps1 -Commit 82ca206 -InstallDir D:\games\racing_game
#>
[CmdletBinding()]
param(
    [string]$Commit = '',
    [string]$InstallDir = '',
    [switch]$SkipToolchain,
    [switch]$SkipBuild
)

$ErrorActionPreference = 'Stop'
$Repo = 'https://github.com/SebastianDanielFrenz/racing_game.git'
$Preset = 'relwithdebinfo'

function Write-Step($text) { Write-Host ""; Write-Host "=== $text ===" -ForegroundColor Cyan }

function Update-SessionPath {
    $env:Path = [Environment]::GetEnvironmentVariable('Path', 'Machine') + ';' +
                [Environment]::GetEnvironmentVariable('Path', 'User')
}

# git with the checkout as working directory; throws on a non-zero exit.
function Invoke-Git {
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'   # git writes progress to stderr
    # "$_" turns stderr records into plain lines (PowerShell 5.1 prints them as red NativeCommandErrors).
    try { & git -C $InstallDir @args 2>&1 | ForEach-Object { "$_" } | Out-Host } finally { $ErrorActionPreference = $previous }
    if ($LASTEXITCODE -ne 0) { throw "git $($args -join ' ') failed (exit $LASTEXITCODE)" }
}

# git output as a trimmed string, '' on failure (for queries).
function Get-Git {
    $previous = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { $out = & git -C $InstallDir @args 2>$null } finally { $ErrorActionPreference = $previous }
    if ($LASTEXITCODE -ne 0) { return '' }
    return (($out | Out-String).Trim())
}

if (-not $InstallDir) {
    $InstallDir = if ($env:RG_INSTALL_DIR) { $env:RG_INSTALL_DIR } else { Join-Path $env:LOCALAPPDATA 'racing_game' }
}
$InstallDir = [IO.Path]::GetFullPath($InstallDir)
$StateDir = Join-Path $InstallDir '.local-install'
$target = if ($Commit) { "commit $Commit" } else { 'latest main' }
# godot-cpp (fetched by CMake) has paths beyond Windows' 260-character limit below a deep
# install folder. core.longpaths for this process and its children only, not the user's git config.
$n = if ($env:GIT_CONFIG_COUNT) { [int]$env:GIT_CONFIG_COUNT } else { 0 }
Set-Item "Env:GIT_CONFIG_KEY_$n" 'core.longpaths'
Set-Item "Env:GIT_CONFIG_VALUE_$n" 'true'
$env:GIT_CONFIG_COUNT = $n + 1
Write-Host "racing_game installer: $target -> $InstallDir" -ForegroundColor Cyan

# ---- 1. Git -------------------------------------------------------------------------------------------------------
Write-Step 'Git'
if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw "Git is missing and winget is not available - install Git (https://git-scm.com) and re-run."
    }
    & winget install --id Git.Git -e --accept-package-agreements --accept-source-agreements --disable-interactivity
    Update-SessionPath
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw "Git was installed but is not on PATH yet - open a new terminal and re-run." }
}
& git --version

# ---- 2. clone or check the existing checkout ---------------------------------------------------------------------
Write-Step 'Checkout'
if (-not (Test-Path (Join-Path $InstallDir '.git'))) {
    if ((Test-Path $InstallDir) -and @(Get-ChildItem -Force $InstallDir | Where-Object { $_.Name -ne '.local-install' }).Count -gt 0) {
        throw "$InstallDir exists and is not a racing_game checkout - choose an empty or new folder (-InstallDir)."
    }
    New-Item -ItemType Directory -Force $InstallDir | Out-Null
    $env:GIT_LFS_SKIP_SMUDGE = '1'   # LFS files come with 'git lfs pull' below
    try { Invoke-Git clone $Repo $InstallDir } finally { Remove-Item Env:GIT_LFS_SKIP_SMUDGE -ErrorAction SilentlyContinue }
}
if ((Get-Git rev-parse --show-toplevel).Replace('/', '\') -ne $InstallDir.TrimEnd('\')) { throw "$InstallDir is not the top of a git checkout." }
$origin = Get-Git remote get-url origin
if ($origin.TrimEnd('/') -replace '\.git$', '' -ne ($Repo -replace '\.git$', '')) { throw "Unexpected origin '$origin' in $InstallDir - refusing to update." }
if (Get-Git status --porcelain --untracked-files=no) { throw "Tracked local changes in $InstallDir - commit or discard them, then re-run." }
New-Item -ItemType Directory -Force $StateDir | Out-Null
$exclude = Join-Path $InstallDir '.git\info\exclude'
if (-not (Select-String -Path $exclude -SimpleMatch '/.local-install/' -Quiet -ErrorAction SilentlyContinue)) {
    Add-Content -Path $exclude -Value "`n/.local-install/"
}
$dll = Join-Path $InstallDir 'game\bin\librg_godot.dll'
if ((Test-Path $dll) -and -not $SkipBuild) {
    try { [IO.File]::Open($dll, 'Open', 'ReadWrite', 'None').Close() } catch { throw "Close the running game before updating ($dll is in use)." }
}

# ---- 3. fetch and select the revision -----------------------------------------------------------------------------
Write-Step "Select $target"
Invoke-Git fetch --prune --tags origin '+refs/heads/*:refs/remotes/origin/*'
$branch = Get-Git branch --show-current
if ($Commit) {
    $sha = Get-Git rev-parse --verify --quiet "$Commit^{commit}"
    if (-not $sha) {
        # Not on any branch (e.g. an unmerged pull request): GitHub serves a full SHA directly.
        Invoke-Git fetch origin $Commit
        $sha = Get-Git rev-parse --verify --quiet 'FETCH_HEAD^{commit}'
    }
    if (-not $sha) { throw "Commit '$Commit' not found on GitHub (an abbreviated SHA must be on a branch; give the full SHA otherwise)." }
    Invoke-Git -c advice.detachedHead=false switch --detach $sha
} else {
    if ($branch -and $branch -ne 'main') { throw "The checkout is on branch '$branch', not main - refusing to switch your branch." }
    if (-not $branch) {
        if (Get-Git rev-parse --verify --quiet refs/heads/main) { Invoke-Git switch main } else { Invoke-Git switch -c main --track origin/main }
    }
    Invoke-Git merge --ff-only origin/main
    if ((Get-Git rev-parse HEAD) -ne (Get-Git rev-parse origin/main)) { throw "Local main has commits that are not on GitHub - refusing to report it as GitHub main." }
}
Invoke-Git submodule sync --recursive
Invoke-Git -c protocol.file.allow=always submodule update --init --recursive
if (Get-Git lfs version) {
    Invoke-Git lfs install --local
    Invoke-Git lfs pull
} else {
    Write-Host "  Git LFS not found: imported car models stay pointer files (the game runs, those cars do not load)." -ForegroundColor Yellow
}
$sha = Get-Git rev-parse HEAD
Write-Host "  at $sha ($(Get-Git log -1 --format=%s))"

# ---- 4. toolchain ---------------------------------------------------------------------------------------------------
if (-not $SkipToolchain) {
    Write-Step 'Toolchain (tools\setup_dev_env.ps1)'
    & powershell -NoProfile -ExecutionPolicy Bypass -File (Join-Path $InstallDir 'tools\setup_dev_env.ps1') -SkipPython
    if ($LASTEXITCODE -ne 0) { throw "tools\setup_dev_env.ps1 reported missing tools (exit $LASTEXITCODE) - see above, then re-run." }
    Update-SessionPath
}

# ---- 5. build and import --------------------------------------------------------------------------------------------
if (-not $SkipBuild) {
    Write-Step "Build rg_godot ($Preset)"
    if (-not ((Test-Path (Join-Path $InstallDir 'CMakeLists.txt')) -and (Test-Path (Join-Path $InstallDir 'tools\common.ps1')) -and (Test-Path (Join-Path $InstallDir 'game\project.godot')))) {
        throw "Revision $($sha.Substring(0, 9)) has no buildable game (no CMakeLists.txt, tools\common.ps1 or game\project.godot); choose another commit."
    }
    . (Join-Path $InstallDir 'tools\common.ps1')
    $buildDir = Join-Path $InstallDir "out\build\$Preset"
    Remove-RgGodotStaleDll -DllPath $dll -Preset $Preset
    Enter-VsDevShell
    if (-not (Test-RgConfigured -BuildDir $buildDir)) {
        $code = Invoke-RgCMakeConfigure -RepoRoot $InstallDir -BuildDir $buildDir -Preset $Preset -WithGodotExtension
        if ($code -ne 0) { throw "cmake configure failed (exit $code)" }
    }
    $code = Invoke-RgCMakeBuild -BuildDir $buildDir -Targets @('rg_godot')
    if ($code -ne 0) { throw "cmake build failed (exit $code)" }
    if (-not (Test-Path $dll)) { throw "build reported success but $dll is missing" }
    Set-RgGodotPresetStamp -DllPath $dll -Preset $Preset
    Ensure-GodotProjectImported -GameDir (Join-Path $InstallDir 'game') -GodotExe (Find-GodotConsoleExe)
}

# ---- 6. launchers ---------------------------------------------------------------------------------------------------
Write-Step 'Launchers'
Copy-Item -Force $PSCommandPath (Join-Path $StateDir 'install.ps1.new')
$checkoutInstaller = Join-Path $InstallDir 'tools\install.ps1'
# On main the checkout's copy is the newest installer; a pinned old commit may predate it, so keep the running one.
$source = if (-not $Commit -and (Test-Path $checkoutInstaller)) { $checkoutInstaller } else { Join-Path $StateDir 'install.ps1.new' }
Copy-Item -Force $source (Join-Path $StateDir 'install.ps1')
Remove-Item -Force (Join-Path $StateDir 'install.ps1.new')
Set-Content -Encoding ascii -Path (Join-Path $StateDir 'play.cmd') -Value @"
@echo off
rem Starts racing_game from $InstallDir. Arguments as for tools\run.cmd: -Flat, -Drive, -VR, -- <godot args>.
call "%~dp0..\tools\run.cmd" -SkipBuild %*
"@
Set-Content -Encoding ascii -Path (Join-Path $StateDir 'update.cmd') -Value @"
@echo off
rem Updates to the latest main; pass -Commit <sha> for one specific commit.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" -InstallDir "$InstallDir" %*
"@
Set-Content -Encoding ascii -Path (Join-Path $StateDir 'installed-commit') -Value $sha

Write-Host ""
Write-Host "Installed $sha into $InstallDir" -ForegroundColor Green
Write-Host "  play:   $StateDir\play.cmd          (-Flat: test world without map data)"
Write-Host "  update: $StateDir\update.cmd        (-Commit <sha> for a specific commit)"
if (-not (Test-Path (Join-Path $InstallDir 'cache\g2m\home-r1'))) {
    Write-Host "  No map data yet: copy the geo2map stores as README.md describes; -Flat works without them." -ForegroundColor Yellow
}
