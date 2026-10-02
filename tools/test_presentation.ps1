#Requires -Version 5.1
# Run after building rg_godot (tools/run.ps1 or tools/smoke_test.ps1).
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$repoRoot = Split-Path -Parent $PSScriptRoot
$gameDir = Join-Path $repoRoot 'game'
$logPath = Join-Path $repoRoot 'out/presentation_test.log'
$godotExe = Find-GodotConsoleExe
Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotExe
& $godotExe --headless --path $gameDir --quit-after 1000 --script res://scripts/test_presentation.gd *> $logPath
$testExit = $LASTEXITCODE
Get-Content $logPath
if ($testExit -ne 0 -or
    (Select-String -Path $logPath -Pattern 'SCRIPT ERROR|^ERROR') -or
    -not (Select-String -Path $logPath -Pattern 'RG_PRESENTATION_TEST ok:')) {
    throw 'Presentation tests failed; see out/presentation_test.log'
}
