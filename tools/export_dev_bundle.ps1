#Requires -Version 5.1
param([string]$Destination)
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not $Destination) { $Destination = Join-Path $repoRoot 'out/development-transfer' }
$Destination = [System.IO.Path]::GetFullPath($Destination)
New-Item -ItemType Directory -Path $Destination -Force | Out-Null
$repositories = @(
    @{ Path=$repoRoot; Name='racing_game' },
    @{ Path=(Join-Path $repoRoot 'external/physics_sim'); Name='physics_sim' },
    @{ Path=(Join-Path $repoRoot 'external/geo2map_engine'); Name='geo2map_engine' }
)
$manifest = @()
foreach ($repository in $repositories) {
    $pending = @(& git -C $repository.Path status --porcelain)
    if ($LASTEXITCODE -ne 0 -or $pending.Count -gt 0) {
        throw "$($repository.Name) has uncommitted files; commit them before exporting."
    }
    $bundle = Join-Path $Destination ($repository.Name + '.bundle')
    & git -C $repository.Path bundle create $bundle --all HEAD
    if ($LASTEXITCODE -ne 0) { throw "Bundle creation failed for $($repository.Name)" }
    & git -C $repository.Path bundle verify $bundle
    if ($LASTEXITCODE -ne 0) { throw "Bundle verification failed for $($repository.Name)" }
    $commit = & git -C $repository.Path rev-parse HEAD
    $manifest += @{ repository=$repository.Name; commit=$commit; sha256=(Get-FileHash -LiteralPath $bundle -Algorithm SHA256).Hash }
}
$manifest | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $Destination 'manifest.json') -Encoding UTF8
Copy-Item -LiteralPath (Join-Path $repoRoot 'DEVELOPMENT_TRANSFER.md') -Destination $Destination
Write-Host "Verified development bundles: $Destination"
