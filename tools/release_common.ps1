#Requires -Version 5.1
<#
.SYNOPSIS
    Shared snapshot-assembly helpers for tools/make_release.ps1 (Windows
    snapshot) and tools/frame_install.ps1 (Steam Frame staging). Dot-sourced,
    not run directly.

.DESCRIPTION
    Both snapshots have the same repo-relative layout (the game resolves
    res://../data, res://../external/physics_sim/data, ... at runtime), so the
    copy plan and the repo-root stub live here once. A new runtime file
    dependency outside game/ + data/ + external/*/data must be added to
    Get-RgReleaseCopyPlan, and both scripts pick it up.
#>

# robocopy wrapper. $ExcludeDirs -> /XD, $ExcludeFiles -> /XF (name patterns).
# $Mirror switches /E to /MIR (removes files no longer in the source: used for
# the Frame staging dir so it never accumulates stale files). Returns $false
# when the source does not exist.
function Copy-Tree {
    param(
        [string]$Source,
        [string]$Destination,
        [string[]]$ExcludeDirs = @(),
        [string[]]$ExcludeFiles = @(),
        [switch]$Mirror
    )
    if (-not (Test-Path $Source)) { return $false }
    $a = @($Source, $Destination, $(if ($Mirror) { '/MIR' } else { '/E' }), '/MT:8', '/R:2', '/W:1', '/NFL', '/NDL', '/NJH', '/NJS', '/NP')
    if ($ExcludeDirs.Count -gt 0)  { $a += '/XD'; $a += $ExcludeDirs }
    if ($ExcludeFiles.Count -gt 0) { $a += '/XF'; $a += $ExcludeFiles }
    & robocopy @a | Out-Null
    if ($LASTEXITCODE -ge 8) { throw "robocopy $Source -> $Destination failed (exit $LASTEXITCODE)" }
    return $true
}

# The repo-relative directories a snapshot contains. $ExcludeGameBinFiles are
# name patterns skipped inside game\bin (the Frame has no use for the Windows
# DLL/PDB).
function Get-RgReleaseCopyPlan {
    param([string[]]$ExcludeGameBinFiles = @())
    return @(
        @{ rel = 'game';                                              xd = @('__pycache__'); xf = $ExcludeGameBinFiles },
        @{ rel = 'data';                                              xd = @('__pycache__'); xf = @() },
        @{ rel = 'external\physics_sim\data';                         xd = @(); xf = @() },
        @{ rel = 'external\physics_sim\adapters\godot\demo\audio';    xd = @(); xf = @() },
        @{ rel = 'external\physics_sim\out\godot_engine_cache';       xd = @(); xf = @() },
        @{ rel = 'out\godot_engine_cache';                            xd = @(); xf = @() },
        @{ rel = 'external\geo2map_engine\data';                      xd = @(); xf = @() }
    )
}

# Copies the plan from $RepoRoot into $Destination and writes the repo-root
# stub CMakeLists.txt. Returns the list of copied entries (repo-relative, '/').
function Copy-RgReleaseTrees {
    param(
        [Parameter(Mandatory)][string]$RepoRoot,
        [Parameter(Mandatory)][string]$Destination,
        [string[]]$ExcludeGameBinFiles = @(),
        [switch]$Mirror
    )
    New-Item -ItemType Directory -Force -Path $Destination | Out-Null
    $copied = New-Object System.Collections.Generic.List[string]
    foreach ($p in (Get-RgReleaseCopyPlan -ExcludeGameBinFiles $ExcludeGameBinFiles)) {
        if (Copy-Tree -Source (Join-Path $RepoRoot $p.rel) -Destination (Join-Path $Destination $p.rel) -ExcludeDirs $p.xd -ExcludeFiles $p.xf -Mirror:$Mirror) {
            $copied.Add($p.rel.Replace('\', '/'))
        } else {
            Write-Host "  (skipped, not present: $($p.rel))" -ForegroundColor DarkGray
        }
    }
    # rg::load_world_config's repo-root marker: the nearest ancestor of a world
    # config holding a CMakeLists.txt is taken as the repo root.
    Set-Content -Path (Join-Path $Destination 'CMakeLists.txt') -Encoding ASCII -Value @(
        '# STUB. Not a buildable project: a racing_game release snapshot.',
        '# rg::load_world_config (core/src/world_config.cpp find_repo_root) takes the nearest',
        '# ancestor of a world config holding a CMakeLists.txt as the repo root; this file is that marker.'
    )
    $copied.Add('CMakeLists.txt (stub marker)')
    return $copied
}

function Get-RgDirSizeBytes {
    param([Parameter(Mandatory)][string]$Path)
    $m = Get-ChildItem -Path $Path -Recurse -File -Force -ErrorAction SilentlyContinue | Measure-Object -Property Length -Sum
    if ($m.Sum) { return [int64]$m.Sum } else { return [int64]0 }
}
