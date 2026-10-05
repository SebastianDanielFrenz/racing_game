#Requires -Version 5.1
<#
.SYNOPSIS
    Writes a synthetic vehicle catalog for the car browser tests and screenshots
    (PLAN.md R6c): the real catalog's base cars plus N generated presets.

.DESCRIPTION
    The result is a normal rg.vehicle_catalog/1 file, loaded with
    `--catalog <path>`. Every generated entry is a preset of one of the real base
    cars (so the physics files, models and setup whitelist are real), with its own
    id, title, subtitle, body type, manufacturer, a spring multiplier inside the
    whitelist range and a paint colour - enough variety to exercise grouping,
    sorting, filtering and scrolling with hundreds of tiles.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory)][string]$OutFile,
    [int]$Total = 200
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$real = Get-Content -Raw (Join-Path $repoRoot 'data\vehicles\catalog.json') | ConvertFrom-Json

$bases = @($real.vehicles | Where-Object { -not $_.preset_of })
$out = [ordered]@{
    format  = $real.format
    source  = "synthetic test catalog (tools/make_synthetic_catalog.ps1): the real base cars plus generated presets"
    default = $real.default
    vehicles = @()
}
$list = New-Object System.Collections.Generic.List[object]
foreach ($b in $bases) { $list.Add($b) }

$bodyTypes = @('sedan', 'sports', 'hatchback', 'coupe', 'wagon', 'hypercar', 'rally', 'pickup')
$makers = @('Alpha Motors', 'Bravo Cars', 'Charlie Works', 'Delta Auto', 'Echo Marque')
$paints = @('#c9262e', '#2848a8', '#3f7a4a', '#e8742a', '#e0b43a', '#6b3fa0', '#1b1d22', '#e8e8ea')
$n = $Total - $bases.Count
for ($i = 0; $i -lt $n; $i++) {
    $base = $bases[$i % $bases.Count]
    $spring = [math]::Round(0.75 + 0.5 * (($i * 37) % 100) / 100.0, 2)
    $entry = [ordered]@{
        id           = ('synthetic_{0:000}' -f $i)
        title        = ('Synthetic {0:000} {1}' -f $i, $base.title)
        subtitle     = ('Preset {0} of {1}' -f $i, $base.id)
        description  = 'A generated preset for the car browser tests.'
        body_type    = $bodyTypes[$i % $bodyTypes.Count]
        manufacturer = $makers[$i % $makers.Count]
        preset_of    = $base.id
        preset_setup = [ordered]@{
            spring_front = $spring
            paint        = $paints[$i % $paints.Count]
        }
    }
    $list.Add([pscustomobject]$entry)
}
$out.vehicles = $list.ToArray()
New-Item -ItemType Directory -Force (Split-Path -Parent $OutFile) | Out-Null
$out | ConvertTo-Json -Depth 12 | Set-Content -Encoding UTF8 $OutFile
Write-Host "wrote $($list.Count) entries to $OutFile"
