#Requires -Version 5.1
<#
.SYNOPSIS
    Installs racing_game NATIVELY (Linux ARM64) on the Valve Steam Frame so it
    starts from the Steam Library (Library > Non-Steam, "Devkit Game: racing_game").

.DESCRIPTION
    Re-runnable end to end; nothing outside ~/devkit-game/<RemoteDir> on the
    headset is written (plus the Steam shortcut the devkit tool registers).

      1. BUILD     librg_godot.arm64.so in WSL (preset linux-arm64-release: clang
                   cross build against Valve's Steam Runtime 3c ARM64 SDK sysroot,
                   glibc 2.31, static libstdc++), then tools/frame/check_symbols.sh
                   (highest GLIBC_ version + dlopen under qemu with the glibc 2.31
                   loader). -SkipBuild reuses game/bin/librg_godot.arm64.so.
      2. STAGE     out\frame\stage\ = the exact tree that ends up on the headset:
                   game/ (with the .godot import cache), data/, external/*/data,
                   engine-map caches, the repo-root marker CMakeLists.txt (all via
                   tools/release_common.ps1, shared with make_release.ps1), the
                   official Godot 4.7.2 linux.arm64 binary, start.sh, RELEASE.txt.
                   No Windows DLL/PDB. Mirror copy: stale files never accumulate.
      3. DEVICE    ssh preflight (key login only; this script never creates or
                   installs keys and never types a password), free-space check,
                   rsync from WSL through the Windows ssh.exe (so the Windows
                   ssh key/agent is used): the stage tree, then the map stores
                   (-Maps corridor,home) into <RemoteDir>/maps/. Source stores are
                   only READ (the *.sqlite3-wal/-shm files and home-r1's unreferenced
                   1 GB derived.sqlite3 are not copied). Map copies are incremental
                   and never delete on the device (the game writes its derived store
                   there).
      4. REGISTER  the devkit shortcut (~/devkit-utils/steam-client-create-shortcut,
                   same mechanism as the SteamOS devkit upload), once; -ReRegister
                   repeats it.
      5. VERIFY    a headless run ON the headset: `start.sh --desktop --headless
                   -- --bindings-test` and the scripted real-world drive smoke
                   (`-- --drive --drive-smoke`); the log is saved to out\frame\ and
                   its tail printed. -NoVerify skips it.

    Launching: from the Steam Library on the headset; start.sh by default starts
    OpenXR (VR) mode. `start.sh --desktop [-- game args]` runs a flat-screen
    window instead (used by the verification).

    Exit codes: 0 done; 1 failure; 2 staged but the headset is unreachable /
    refuses key login (re-run when it is on the network); 3 uploaded but the
    on-device verification failed.

.PARAMETER SkipBuild    Use the existing game/bin/librg_godot.arm64.so.
.PARAMETER StageOnly    Build + stage, do not touch the headset.
.PARAMETER NoMaps       Skip the map stores.
.PARAMETER Maps         Which stores: corridor (rhein-main-corridor-r2), home (home-r1). Default both.
.PARAMETER Remote       ssh target (default steamos@frame; Developer Mode, key login).
.PARAMETER RemoteDir    Install directory relative to the remote home (default devkit-game/racing_game).
.PARAMETER GameId       Devkit game id / shortcut name (default racing_game).
.PARAMETER CompatTool   Steam compatibility tool for the shortcut. Default: autodetected from the
                        headset's installed SteamLinuxRuntime*arm64* directories (none found = the
                        game runs natively without a container, which its glibc 2.31 baseline allows).
.PARAMETER NoRegister   Upload only, no shortcut.
.PARAMETER ReRegister   Create the shortcut again even if this script registered one before.
.PARAMETER NoVerify     Skip the on-device headless run.
.PARAMETER QemuCheck    Also run the bindings test locally under qemu-aarch64 with the glibc 2.31
                        sysroot on the staged tree (no headset needed).
#>
[CmdletBinding()]
param(
    [switch]$SkipBuild,
    [switch]$StageOnly,
    [switch]$NoMaps,
    [ValidateSet('corridor', 'home')][string[]]$Maps = @('corridor', 'home'),
    [string]$Remote = 'steamos@frame',
    [string]$RemoteDir = 'devkit-game/racing_game',
    [string]$GameId = 'racing_game',
    [string]$CompatTool = '',
    [switch]$NoRegister,
    [switch]$ReRegister,
    [switch]$NoVerify,
    [switch]$QemuCheck
)

$ErrorActionPreference = 'Stop'

$repoRoot  = Split-Path -Parent $PSScriptRoot
$gameDir   = Join-Path $repoRoot 'game'
$soPath    = Join-Path $gameDir 'bin\librg_godot.arm64.so'
$frameRoot = Join-Path $repoRoot 'out\frame'
$stageDir  = Join-Path $frameRoot 'stage'
$workDir   = Join-Path $frameRoot 'work'
$GodotVersion = '4.7.2'
$godotName = "Godot_v$GodotVersion-stable_linux.arm64"
$godotBin  = Join-Path $frameRoot "godot-$GodotVersion-linux.arm64\$godotName"

. (Join-Path $PSScriptRoot 'common.ps1')
. (Join-Path $PSScriptRoot 'release_common.ps1')

$script:Timings = New-Object System.Collections.Specialized.OrderedDictionary
$script:TotalSw = [System.Diagnostics.Stopwatch]::StartNew()

function Write-Step([string]$Text) { Write-Host "`n=== $Text ===" -ForegroundColor Cyan }
function Write-Ok([string]$Text)   { Write-Host "  [OK]   $Text" -ForegroundColor Green }
function Write-Warn([string]$Text) { Write-Host "  [WARN] $Text" -ForegroundColor Yellow }
function Write-Fail([string]$Text) { Write-Host "  [FAIL] $Text" -ForegroundColor Red }
function Format-Size([double]$Bytes) {
    if ($Bytes -ge 1GB) { return ('{0:N2} GB' -f ($Bytes / 1GB)) }
    if ($Bytes -ge 1MB) { return ('{0:N1} MB' -f ($Bytes / 1MB)) }
    return ('{0:N0} KB' -f ($Bytes / 1KB))
}
function Stop-Phase([string]$Name, $Sw) { $Sw.Stop(); $script:Timings[$Name] = $Sw.Elapsed }

function ConvertTo-WslPath([string]$Path) {
    $full = [System.IO.Path]::GetFullPath($Path)
    return '/mnt/' + $full.Substring(0, 1).ToLower() + $full.Substring(2).Replace('\', '/')
}

function ConvertTo-NativeArg([string]$Arg) {
    if ($Arg -eq '') { return '""' }
    if ($Arg -notmatch '[\s"]') { return $Arg }
    return '"' + ($Arg -replace '(\\*)"', '$1$1\"' -replace '(\\+)$', '$1$1') + '"'
}

# Runs a native program, captures stdout/stderr separately (no PowerShell
# NativeCommandError handling involved), returns @{ExitCode; Out; Err; TimedOut}.
function Invoke-Native {
    param([string]$File, [string[]]$Arguments = @(), [int]$TimeoutSec = 0)
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $File
    $psi.Arguments = ($Arguments | ForEach-Object { ConvertTo-NativeArg $_ }) -join ' '
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.CreateNoWindow = $true
    $p = [System.Diagnostics.Process]::Start($psi)
    $outTask = $p.StandardOutput.ReadToEndAsync()
    $errTask = $p.StandardError.ReadToEndAsync()
    $timedOut = $false
    if ($TimeoutSec -gt 0) {
        if (-not $p.WaitForExit($TimeoutSec * 1000)) { $timedOut = $true; try { $p.Kill() } catch {} ; $p.WaitForExit() }
    } else { $p.WaitForExit() }
    return @{ ExitCode = $p.ExitCode; Out = $outTask.Result; Err = $errTask.Result; TimedOut = $timedOut }
}

# Runs a bash script text in WSL (Ubuntu). -Stream shows the output live,
# otherwise it is captured into $workDir\<Name>.log and its tail printed.
function Invoke-WslScript {
    param([string]$Name, [string]$Text, [switch]$Stream, [int]$TailLines = 25)
    New-Item -ItemType Directory -Force -Path $workDir | Out-Null
    $sh  = Join-Path $workDir "$Name.sh"
    $log = Join-Path $workDir "$Name.log"
    [System.IO.File]::WriteAllText($sh, $Text.Replace("`r`n", "`n"), (New-Object System.Text.UTF8Encoding($false)))
    $wslArgs = @('-e', 'bash', (ConvertTo-WslPath $sh))
    if ($Stream) {
        $p = Start-Process -FilePath wsl.exe -ArgumentList $wslArgs -NoNewWindow -Wait -PassThru
        return $p.ExitCode
    }
    $errLog = "$log.err"
    $p = Start-Process -FilePath wsl.exe -ArgumentList $wslArgs -NoNewWindow -Wait -PassThru -RedirectStandardOutput $log -RedirectStandardError $errLog
    $lines = @(Get-Content -Path $log -ErrorAction SilentlyContinue) + @(Get-Content -Path $errLog -ErrorAction SilentlyContinue | Where-Object { $_ })
    $lines | Select-Object -Last $TailLines | ForEach-Object { Write-Host "    $_" }
    if ($p.ExitCode -ne 0) { Write-Host "    (full log: $log)" -ForegroundColor DarkGray }
    return $p.ExitCode
}

# Runs a /bin/sh script text on the headset (sent base64-encoded on the command
# line, so no quoting problems and no file is written there). Extra words
# become $1..$n of the script.
function Invoke-Ssh {
    param([string]$ScriptText, [string[]]$ScriptArgs = @(), [int]$TimeoutSec = 60, [int]$ConnectTimeout = 10)
    $b64 = [Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($ScriptText.Replace("`r`n", "`n")))
    $argText = ($ScriptArgs | ForEach-Object { "'" + ($_ -replace "'", "'\''") + "'" }) -join ' '
    # sh -c "<script>" keeps stdin free for the commands inside the script.
    $remoteCmd = 'sh -c "$(echo ' + $b64 + ' | base64 -d)" sh ' + $argText
    $sshArgs = @('-o', 'BatchMode=yes', '-o', "ConnectTimeout=$ConnectTimeout", '-o', 'ServerAliveInterval=15', $Remote, $remoteCmd)
    return Invoke-Native -File 'ssh.exe' -Arguments $sshArgs -TimeoutSec $TimeoutSec
}

function Write-OwnerSshHelp([string]$Reason) {
    Write-Host ""
    Write-Host "The headset cannot be reached with key login ($Reason)." -ForegroundColor Yellow
    Write-Host "This script never creates or installs ssh keys and never types a password. The staged"
    Write-Host "install is ready in $stageDir. Owner steps:"
    Write-Host "  1. Headset on, awake, same network; Settings > Developer: Developer Mode on (pair this PC there)."
    Write-Host "  2. Check:   ssh $Remote echo ok"
    Write-Host "     If it asks for a password / says 'Permission denied (publickey)', install YOUR key once"
    Write-Host "     (you type the headset password yourself):"
    Write-Host "       ssh-keygen -t ed25519            (only if you have no key yet)"
    Write-Host "       type `$env:USERPROFILE\.ssh\id_ed25519.pub | ssh $Remote `"mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys`""
    Write-Host "  3. Re-run:  tools\frame_install.ps1 -SkipBuild     (the build and the stage are reused)"
}

# ---------------------------------------------------------------------------
# 0. preconditions
# ---------------------------------------------------------------------------
Write-Step "preconditions"
$setupFix = 'run tools\setup_dev_env.ps1 -Frame'
if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) { throw "wsl.exe not found - $setupFix" }
if (-not (Test-Path $godotBin)) { throw "Godot $GodotVersion linux.arm64 not found at $godotBin - $setupFix" }
Write-Ok "Godot $GodotVersion linux.arm64: $godotBin"
$chk = Invoke-WslScript -Name 'sysroot_check' -Text ("bash '" + (ConvertTo-WslPath (Join-Path $PSScriptRoot 'frame\install_sysroot.sh')) + "' --check") -TailLines 2
if ($chk -ne 0) { throw "Steam Runtime 3c ARM64 SDK sysroot missing in WSL - $setupFix" }
$subDirty = @(& git -C $repoRoot submodule status 2>$null | Where-Object { $_ -match '^[+\-U]' })
if ($subDirty.Count -gt 0) { Write-Warn "submodule(s) not at the recorded pin: $($subDirty -join '; ')" }

# ---------------------------------------------------------------------------
# 1. build
# ---------------------------------------------------------------------------
$sw = [System.Diagnostics.Stopwatch]::StartNew()
if (-not $SkipBuild) {
    Write-Step "build librg_godot.arm64.so (WSL, preset linux-arm64-release)"
    $repoW = ConvertTo-WslPath $repoRoot
    $build = @"
set -e
cd '$repoW'
[ -f ~/build/racing_game/linux-arm64-release/CMakeCache.txt ] || cmake --preset linux-arm64-release
cmake --build --preset linux-arm64-release
"@
    $rc = Invoke-WslScript -Name 'build_arm64' -Text $build -TailLines 12
    if ($rc -ne 0) { throw "arm64 build failed (exit $rc), log: $workDir\build_arm64.log" }
} else {
    Write-Step "build skipped (-SkipBuild)"
}
if (-not (Test-Path $soPath)) { throw "$soPath does not exist - build first (run without -SkipBuild)" }
$soItem = Get-Item $soPath
Write-Ok "$($soItem.Name) $(Format-Size $soItem.Length), built $($soItem.LastWriteTime.ToString('yyyy-MM-dd HH:mm'))"

Write-Step "library check (glibc symbol versions, dlopen under qemu with the glibc 2.31 loader)"
$symText = "bash '" + (ConvertTo-WslPath (Join-Path $PSScriptRoot 'frame\check_symbols.sh')) + "' '" + (ConvertTo-WslPath $soPath) + "'"
$rc = Invoke-WslScript -Name 'check_symbols' -Text $symText -TailLines 30
if ($rc -ne 0) { throw "library check failed - not installing a library the headset runtime may not load" }
Stop-Phase 'build + library check' $sw

# ---------------------------------------------------------------------------
# 2. stage
# ---------------------------------------------------------------------------
$sw = [System.Diagnostics.Stopwatch]::StartNew()
Write-Step "stage -> $stageDir"
# The import cache (.godot/extension_list.cfg, imported textures) must exist and
# be current; ETC2/ASTC variants are enabled in project.godot for ARM GPUs.
$godotConsole = Find-GodotConsoleExe
Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotConsole

New-Item -ItemType Directory -Force -Path $stageDir | Out-Null
$copied = Copy-RgReleaseTrees -RepoRoot $repoRoot -Destination $stageDir -Mirror `
    -ExcludeGameBinFiles @('*.dll', '*.pdb', '*.preset', '*.lib', '*.exp', 'librg_godot.so')
# editor-only state of the dev checkout
$editorState = Join-Path $stageDir 'game\.godot\editor'
if (Test-Path $editorState) { Remove-Item -Recurse -Force $editorState }

New-Item -ItemType Directory -Force -Path (Join-Path $stageDir 'godot') | Out-Null
Copy-Item -Force $godotBin (Join-Path $stageDir "godot\$godotName")

$startSh = @'
#!/bin/sh
# racing_game launcher for the Steam Frame (native Linux ARM64).
# Written by tools/frame_install.ps1; started by the Steam Library entry.
#   start.sh                    OpenXR (VR) mode, game shell menus
#   start.sh -- --drive         VR, straight into the real world
#   start.sh --desktop [args]   flat-screen window, args go to Godot (e.g. --headless -- --bindings-test)
# Map stores live in ./maps (rhein-main-corridor-r2, home-r1).
HERE=$(cd "$(dirname "$0")" && pwd) || exit 1
export RG_G2M_CACHE="${RG_G2M_CACHE:-$HERE/maps}"
export RG_G2M_HOME="${RG_G2M_HOME:-$HERE/maps/home-r1}"
GODOT="$HERE/godot/Godot_v4.7.2-stable_linux.arm64"
if [ "${RG_FRAME_ISOLATE:-0}" = "1" ]; then
  # verification runs keep Godot's user data inside the install directory
  export XDG_DATA_HOME="$HERE/.verify_home/data" XDG_CONFIG_HOME="$HERE/.verify_home/config" XDG_CACHE_HOME="$HERE/.verify_home/cache"
  mkdir -p "$XDG_DATA_HOME" "$XDG_CONFIG_HOME" "$XDG_CACHE_HOME"
fi
MODE=vr
if [ "$1" = "--desktop" ]; then MODE=desktop; shift; fi
cd "$HERE" || exit 1
if [ "$MODE" = "vr" ]; then
  exec "$GODOT" --path "$HERE/game" --xr-mode on --rendering-method mobile --rendering-driver vulkan "$@"
fi
exec "$GODOT" --path "$HERE/game" "$@"
'@
[System.IO.File]::WriteAllText((Join-Path $stageDir 'start.sh'), $startSh.Replace("`r`n", "`n"), (New-Object System.Text.UTF8Encoding($false)))

$sha     = (& git -C $repoRoot rev-parse HEAD | Select-Object -First 1)
$subject = (& git -C $repoRoot log -1 --format=%s | Select-Object -First 1)
$subs    = @(& git -C $repoRoot submodule status | ForEach-Object { '  ' + $_.Trim() })
$dirty   = @(& git -C $repoRoot status --porcelain --untracked-files=no | Where-Object { $_ })
$stageBytes = Get-RgDirSizeBytes -Path $stageDir
$releaseTxt = @(
    "racing_game on the Steam Frame (native Linux ARM64)", "",
    "created    : $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')",
    "commit     : $sha", "subject    : $subject",
    "dirty tree : $(if ($dirty.Count -gt 0) { 'YES: ' + ($dirty -join '; ') } else { 'no' })",
    "submodules :", $subs,
    "library    : game/bin/librg_godot.arm64.so ($(Format-Size $soItem.Length)), glibc <= 2.30 symbols, static libstdc++",
    "godot      : $godotName (godotengine.org official release)",
    "", "start.sh                  VR (OpenXR) mode; maps in ./maps",
    "start.sh --desktop [args]  flat-screen window / headless runs",
    "", "CONTENTS", ($copied | ForEach-Object { "  $_" }), "  godot/, start.sh, RELEASE.txt, maps/ (separate copy)"
)
[System.IO.File]::WriteAllText((Join-Path $stageDir 'RELEASE.txt'), (($releaseTxt | ForEach-Object { $_ }) -join "`n") + "`n", (New-Object System.Text.UTF8Encoding($false)))
Write-Ok "stage tree $(Format-Size $stageBytes): $($copied.Count) trees + godot + start.sh"
Stop-Phase 'stage' $sw

if ($QemuCheck) {
    Write-Step "local check: bindings test under qemu-aarch64 (glibc 2.31 sysroot)"
    $stageW = ConvertTo-WslPath $stageDir
    $qw = ConvertTo-WslPath $workDir
    $q = @"
set -u
SR=`$HOME/sdk/steamrt3c-arm64-sdk
cd '$stageW'
export RG_G2M_CACHE='$stageW/maps' RG_G2M_HOME='$stageW/maps/home-r1'
export XDG_DATA_HOME=/tmp/rg_qemu_xdg/data XDG_CONFIG_HOME=/tmp/rg_qemu_xdg/config XDG_CACHE_HOME=/tmp/rg_qemu_xdg/cache
mkdir -p /tmp/rg_qemu_xdg/data /tmp/rg_qemu_xdg/config /tmp/rg_qemu_xdg/cache
timeout 600 qemu-aarch64 -L "`$SR" './godot/$godotName' --headless --path game -- --bindings-test > '$qw/qemu_bindings.out' 2>&1
echo "exit=`$?"
grep -c 'ERROR' '$qw/qemu_bindings.out'
grep -E 'bindings test: ok|ERROR|RG_' '$qw/qemu_bindings.out' | head -20
"@
    $rc = Invoke-WslScript -Name 'qemu_check' -Text $q -TailLines 25
    Write-Host "  qemu check script exit: $rc (log tail above; full output $workDir\qemu_bindings.out)"
}

if ($StageOnly) {
    Write-Step "stage only (-StageOnly): headset not touched"
    Write-Host "stage: $stageDir ($(Format-Size $stageBytes))"
    exit 0
}

# ---------------------------------------------------------------------------
# 3. device: preflight
# ---------------------------------------------------------------------------
Write-Step "headset $Remote"
$probeScript = @'
echo "arch=$(uname -m)"
echo "kernel=$(uname -sr)"
[ -r /etc/os-release ] && . /etc/os-release && echo "os=$PRETTY_NAME"
echo "glibc=$(ldd --version 2>&1 | head -n 1)"
echo "home=$HOME"
echo "free_kb=$(df -Pk "$HOME" | awk 'NR==2{print $4}')"
echo "rsync=$(command -v rsync || echo none)"
if [ -d "$HOME/devkit-utils" ]; then echo "devkit_utils=$(ls "$HOME/devkit-utils" | tr '\n' ' ')"; else echo "devkit_utils=none"; fi
for d in "$HOME/.local/share/Steam/steamapps/common" "$HOME/.steam/steam/steamapps/common"; do
  [ -d "$d" ] && ls "$d" | grep -i -E 'SteamLinuxRuntime|SteamVR' | sed 's/^/steamapp=/'
done
exit 0
'@
$pr = Invoke-Ssh -ScriptText $probeScript -TimeoutSec 40 -ConnectTimeout 10
if ($pr.TimedOut -or $pr.ExitCode -ne 0) {
    $reason = ($pr.Err.Trim() -split "`n" | Select-Object -Last 1)
    if (-not $reason) { $reason = "exit $($pr.ExitCode)" }
    Write-Fail "ssh $Remote : $reason"
    Write-OwnerSshHelp $reason
    exit 2
}
$info = @{}; $steamApps = New-Object System.Collections.Generic.List[string]
foreach ($l in ($pr.Out -split "`n")) {
    if ($l -match '^([a-z_]+)=(.*)$') {
        if ($Matches[1] -eq 'steamapp') { $steamApps.Add($Matches[2].Trim()) } else { $info[$Matches[1]] = $Matches[2].Trim() }
    }
}
Write-Ok "connected: $($info['os']) / $($info['kernel']) / $($info['arch']) / $($info['glibc'])"
if ($info['arch'] -ne 'aarch64') { Write-Fail "the headset reports architecture '$($info['arch'])', expected aarch64"; exit 1 }
$remoteHome = $info['home']
$remoteAbs  = "$remoteHome/$RemoteDir"
Write-Host "  install dir : $remoteAbs"
Write-Host "  devkit-utils: $($info['devkit_utils'])"
Write-Host "  runtimes    : $(if ($steamApps.Count -gt 0) { $steamApps -join ', ' } else { '(none found)' })"
if ($info['rsync'] -eq 'none') { Write-Fail "rsync is not installed on the headset (nothing is installed there by this script)"; exit 1 }

# sizes to transfer
$mapPlan = New-Object System.Collections.Generic.List[hashtable]
if (-not $NoMaps) {
    $g2mCache = if ($env:RG_G2M_CACHE) { $env:RG_G2M_CACHE } else { [System.IO.Path]::GetFullPath((Join-Path $repoRoot '..\geo2map_cache')) }
    $g2mHome  = if ($env:RG_G2M_HOME)  { $env:RG_G2M_HOME }  else { Join-Path $repoRoot 'cache\g2m\home-r1' }
    if ($Maps -contains 'corridor') { $mapPlan.Add(@{ Name = 'corridor'; Src = (Join-Path $g2mCache 'rhein-main-corridor-r2'); Dst = 'rhein-main-corridor-r2'; Exclude = @() }) }
    if ($Maps -contains 'home')     { $mapPlan.Add(@{ Name = 'home';     Src = $g2mHome;                                       Dst = 'home-r1';                  Exclude = @('/derived.sqlite3') }) }
    foreach ($m in $mapPlan) {
        if (-not (Test-Path (Join-Path $m.Src 'tiles.sqlite3'))) { throw "map store '$($m.Name)' not found at $($m.Src) (tiles.sqlite3 missing)" }
        $wal = Get-ChildItem -Path $m.Src -Recurse -File -Filter '*-wal' -ErrorAction SilentlyContinue | Where-Object { $_.Length -gt 0 }
        if ($wal) { Write-Warn "map store $($m.Name) has a non-empty write-ahead log ($($wal[0].FullName)) - a game or tool is using it; the copy may be inconsistent (re-run after closing it)" }
        $bytes = 0L
        foreach ($f in (Get-ChildItem -Path $m.Src -Recurse -File -Force -ErrorAction SilentlyContinue)) {
            $rel = $f.FullName.Substring($m.Src.Length).Replace('\', '/')
            if ($f.Name -like '*-wal' -or $f.Name -like '*-shm') { continue }
            if ($m.Exclude -contains $rel) { continue }
            $bytes += $f.Length
        }
        $m.Bytes = $bytes
    }
}
$needBytes = $stageBytes + $(if ($mapPlan.Count -gt 0) { ($mapPlan | ForEach-Object { $_.Bytes } | Measure-Object -Sum).Sum } else { 0 })
$freeBytes = [int64]$info['free_kb'] * 1024
Write-Host "  to transfer : stage $(Format-Size $stageBytes)$(foreach ($m in $mapPlan) { ' + ' + $m.Name + ' ' + (Format-Size $m.Bytes) }) = up to $(Format-Size $needBytes); free on the headset $(Format-Size $freeBytes)"
if ($freeBytes -lt ($needBytes + 2GB)) { Write-Fail "not enough free space on the headset"; exit 1 }

# the devkit way of creating the game directory (falls back to mkdir -p)
$prep = @'
D="$HOME/$1"
mkdir -p "$D" || exit 1
if [ -x "$HOME/devkit-utils/steamos-prepare-upload" ]; then
  "$HOME/devkit-utils/steamos-prepare-upload" --gameid "$2" 2>&1 | head -n 5 | sed 's/^/prepare: /'
fi
echo "dir_ok=$D"
'@
$r = Invoke-Ssh -ScriptText $prep -ScriptArgs @($RemoteDir, $GameId) -TimeoutSec 60
if ($r.ExitCode -ne 0) { Write-Fail "preparing $remoteAbs failed: $($r.Err.Trim())"; exit 1 }
($r.Out -split "`n") | Where-Object { $_ -match '^prepare:' } | ForEach-Object { Write-Host "  $_" }

# ---------------------------------------------------------------------------
# 3b. upload (rsync from WSL through the Windows ssh.exe)
# ---------------------------------------------------------------------------
$sw = [System.Diagnostics.Stopwatch]::StartNew()
Write-Step "upload (rsync over ssh.exe)"
$stageW = ConvertTo-WslPath $stageDir
$sshOpt = "ssh.exe -o BatchMode=yes -o ConnectTimeout=15 -o ServerAliveInterval=15"
$rs = New-Object System.Text.StringBuilder
[void]$rs.AppendLine('set -e')
[void]$rs.AppendLine("RSH='$sshOpt'")
[void]$rs.AppendLine("echo '--- game tree ($stageW)'")
[void]$rs.AppendLine("rsync -rlt --delete --chmod=D755,F644 --filter='P /maps/' --filter='P /.verify_home/' --filter='P /.rg_*' --stats -e `"`$RSH`" '$stageW/' '${Remote}:$RemoteDir/'")
foreach ($m in $mapPlan) {
    $srcW = ConvertTo-WslPath $m.Src
    $ex = ($m.Exclude | ForEach-Object { "--exclude='$_'" }) -join ' '
    [void]$rs.AppendLine("echo '--- map store $($m.Name) ($srcW)'")
    [void]$rs.AppendLine("rsync -rt --partial --inplace --info=progress2 --stats --exclude='*-wal' --exclude='*-shm' $ex -e `"`$RSH`" '$srcW/' '${Remote}:$RemoteDir/maps/$($m.Dst)/'")
}
$ssh = Invoke-Ssh -ScriptText "mkdir -p `"`$HOME/$RemoteDir/maps`"; exit 0" -TimeoutSec 30
$rc = Invoke-WslScript -Name 'upload' -Text $rs.ToString() -Stream
if ($rc -ne 0) { Write-Fail "rsync failed (exit $rc). Re-run to resume (map copies are --partial/incremental)."; exit 1 }
$fix = @'
D="$HOME/$1"
chmod 755 "$D/start.sh" "$D"/godot/Godot_v* 2>&1
du -sh "$D" 2>/dev/null | awk '{print "installed_total=" $1}'
du -sh "$D"/maps/* 2>/dev/null | awk '{print "installed_map=" $2 " " $1}'
exit 0
'@
$r = Invoke-Ssh -ScriptText $fix -ScriptArgs @($RemoteDir) -TimeoutSec 120
$installedLines = @(($r.Out -split "`n") | Where-Object { $_ -match '^installed_' })
Stop-Phase 'upload' $sw
$installedLines | ForEach-Object { Write-Host "  $_" }

# ---------------------------------------------------------------------------
# 4. Steam shortcut
# ---------------------------------------------------------------------------
$registered = 'skipped (-NoRegister)'
if (-not $NoRegister) {
    Write-Step "Steam shortcut (devkit)"
    $compat = $CompatTool
    if (-not $compat) {
        $arm = @($steamApps | Where-Object { $_ -match 'SteamLinuxRuntime' -and $_ -match 'arm64' })
        $pick = @($arm | Where-Object { $_ -match 'sniper|_3' }) + @($arm) | Select-Object -First 1
        if ($pick) { $compat = $pick }
    }
    Write-Host "  compat tool : $(if ($compat) { $compat } else { '(none - native, no container)' })"
    $settings = [ordered]@{ steam_play = '0' }
    if ($compat) { $settings['compat_tool'] = $compat }
    $parms = [ordered]@{
        gameid = $GameId; directory = $remoteAbs; argv = @("$remoteAbs/start.sh"); env = @{}
        settings = $settings; clear_settings = $true; force_appid = ''; lepton_args = ''
    }
    $parmsJson = $parms | ConvertTo-Json -Compress -Depth 5
    $parmsB64 = [Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($parmsJson))
    $reg = @'
D="$HOME/$1"
B="$HOME/devkit-utils/steam-client-create-shortcut"
MARK="$D/.rg_registered"
PARMS=$(echo "$2" | base64 -d)
if [ "$3" != "force" ] && [ -f "$MARK" ] && [ "$(cat "$MARK")" = "$PARMS" ]; then echo "register=already"; exit 0; fi
if [ ! -x "$B" ]; then echo "register=missing-tool"; exit 0; fi
"$B" --parms "$PARMS" 2>&1 | head -n 10 | sed 's/^/devkit: /'
echo "$PARMS" > "$MARK"
echo "register=done"
'@
    $r = Invoke-Ssh -ScriptText $reg -ScriptArgs @($RemoteDir, $parmsB64, $(if ($ReRegister) { 'force' } else { 'normal' })) -TimeoutSec 60
    ($r.Out -split "`n") | Where-Object { $_ -match '^devkit:' } | ForEach-Object { Write-Host "  $_" }
    $state = (($r.Out -split "`n") | Where-Object { $_ -match '^register=' } | Select-Object -Last 1)
    switch -Regex ($state) {
        'done'         { $registered = 'created'; Write-Ok "shortcut created: Library > Non-Steam > 'Devkit Game: $GameId' (check on the headset; the devkit tool's exit state is only what it printed above)" }
        'already'      { $registered = 'already registered by an earlier run'; Write-Ok $registered }
        'missing-tool' { $registered = 'NOT created: ~/devkit-utils/steam-client-create-shortcut not found'; Write-Warn "$registered - add the game via the Steam devkit tab or re-run when SteamOS provides it" }
        default        { $registered = "unknown ($($r.Err.Trim()))"; Write-Warn $registered }
    }
}

# ---------------------------------------------------------------------------
# 5. on-device headless verification
# ---------------------------------------------------------------------------
$verifyState = 'skipped (-NoVerify)'; $exitCode = 0
if (-not $NoVerify) {
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    Write-Step "verify on the headset (headless Godot, same start.sh)"
    $ver = @'
cd "$HOME/$1" || { echo "VERIFY dir-missing"; exit 9; }
export RG_FRAME_ISOLATE=1
mkdir -p .verify_home
L1=.verify_home/bindings.log
L2=.verify_home/drive.log
echo "== bindings test"
timeout 240 ./start.sh --desktop --headless -- --bindings-test > "$L1" 2>&1; rc1=$?
tail -n 12 "$L1"
echo "== drive smoke (real world, home map)"
timeout 600 ./start.sh --desktop --headless --quit-after 200000 -- --drive --drive-smoke > "$L2" 2>&1; rc2=$?
grep -E 'RG_DRIVE (ready|done)|bindings|ERROR|sim thread' "$L2" | tail -n 14
b=fail; d=fail
[ "$rc1" = "0" ] && grep -q 'bindings test: ok' "$L1" && b=ok
[ "$rc2" = "0" ] && grep -q 'RG_DRIVE done' "$L2" && grep 'RG_DRIVE done' "$L2" | grep -q 'result=ok' && d=ok
echo "VERIFY bindings=$b(rc=$rc1,errors=$(grep -c 'ERROR' "$L1")) drive=$d(rc=$rc2,errors=$(grep -c 'ERROR' "$L2"))"
exit 0
'@
    $r = Invoke-Ssh -ScriptText $ver -ScriptArgs @($RemoteDir) -TimeoutSec 1200
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $vlog = Join-Path $frameRoot "verify_$stamp.log"
    New-Item -ItemType Directory -Force -Path $frameRoot | Out-Null
    Set-Content -Path $vlog -Encoding ASCII -Value ($r.Out + "`n--- stderr ---`n" + $r.Err)
    ($r.Out -split "`n") | Select-Object -Last 40 | ForEach-Object { Write-Host "    $_" }
    Write-Host "  full log: $vlog"
    $vline = (($r.Out -split "`n") | Where-Object { $_ -match '^VERIFY ' } | Select-Object -Last 1)
    if ($vline -match 'bindings=ok' -and $vline -match 'drive=ok') { $verifyState = "PASS ($vline)"; Write-Ok $verifyState }
    else { $verifyState = "FAIL ($vline)"; Write-Fail $verifyState; $exitCode = 3 }
    Stop-Phase 'verify' $sw
}

# ---------------------------------------------------------------------------
# summary
# ---------------------------------------------------------------------------
$script:TotalSw.Stop()
Write-Step "summary"
Write-Host "commit        : $sha"
Write-Host "library       : $($soItem.Name) $(Format-Size $soItem.Length)"
Write-Host "stage tree    : $(Format-Size $stageBytes)  ($stageDir)"
foreach ($m in $mapPlan) { Write-Host "map $($m.Name.PadRight(9)) : $(Format-Size $m.Bytes)  <- $($m.Src)" }
Write-Host "installed at  : ${Remote}:$remoteAbs"
$installedLines | ForEach-Object { Write-Host "                $_" }
Write-Host "shortcut      : $registered"
Write-Host "verification  : $verifyState"
foreach ($k in $script:Timings.Keys) { Write-Host ("time {0,-22}: {1}" -f $k, $script:Timings[$k].ToString('hh\:mm\:ss')) }
Write-Host ("time total                 : {0}" -f $script:TotalSw.Elapsed.ToString('hh\:mm\:ss'))
exit $exitCode
