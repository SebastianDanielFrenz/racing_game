#Requires -Version 5.1
<#
.SYNOPSIS
    Builds and runs the headless world server from an ordinary PowerShell.
.EXAMPLE
    tools\run_server.ps1
.EXAMPLE
    tools\run_server.ps1 -BuildOnly
#>
[CmdletBinding()]
param(
    [ValidateSet('relwithdebinfo', 'release', 'debug')]
    [string]$Preset = 'relwithdebinfo',
    [string]$Config = 'data/server/server.json',
    [switch]$BuildOnly,
    [switch]$SkipBuild,
    [switch]$Fresh,
    [ValidateRange(0, 2147483647)]
    [double]$RunFor = 0
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $repoRoot "out/build/$Preset"
$serverExe = Join-Path $buildDir 'server/rg_server.exe'
Push-Location $repoRoot
try {
    if (-not $SkipBuild) {
        Enter-VsDevShell
        $compilerPath = (Get-Command clang-cl -ErrorAction Stop).Source
        # Explicit absolute paths override the preset's bare clang-cl, and repair
        # a cache left incomplete by configuring outside the VS environment.
        $configureArgs = @('--preset', $Preset, "-DCMAKE_CXX_COMPILER=$compilerPath", "-DCMAKE_C_COMPILER=$compilerPath", '-DRG_BUILD_SERVER=ON')
        if ($Fresh) { $configureArgs += '--fresh' }
        & cmake @configureArgs
        if ($LASTEXITCODE -ne 0) { throw "Server configure failed (exit $LASTEXITCODE)" }
        & cmake --build $buildDir --target rg_server
        if ($LASTEXITCODE -ne 0) { throw "Server build failed (exit $LASTEXITCODE)" }
    }
    if ($BuildOnly) { return }
    if (-not (Test-Path -LiteralPath $serverExe)) { throw "Server executable missing; run without -SkipBuild first." }
    $serverArgs = @('--root', $repoRoot, '--config', $Config)
    if ($RunFor -gt 0) { $serverArgs += @('--run-for', $RunFor.ToString([Globalization.CultureInfo]::InvariantCulture)) }
    & $serverExe @serverArgs
    if ($LASTEXITCODE -ne 0) { throw "Server exited with code $LASTEXITCODE" }
} finally {
    Pop-Location
}
