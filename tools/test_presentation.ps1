#Requires -Version 5.1
# Run after building rg_godot (tools/run.ps1 or tools/smoke_test.ps1).
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'common.ps1')
$repoRoot = Split-Path -Parent $PSScriptRoot
$gameDir = Join-Path $repoRoot 'game'
$logPath = Join-Path $repoRoot 'out/presentation_test.log'
$godotExe = Find-GodotConsoleExe
Ensure-GodotProjectImported -GameDir $gameDir -GodotExe $godotExe
$minimapLog = Join-Path $repoRoot 'out/minimap_drone_test.log'
& $godotExe --headless --path $gameDir --quit-after 1000 --script res://scripts/minimap_drone_test.gd *> $minimapLog
$testExit = $LASTEXITCODE
Get-Content $minimapLog
if ($testExit -ne 0 -or
    (Select-String -Path $minimapLog -Pattern 'SCRIPT ERROR|^ERROR') -or
    -not (Select-String -Path $minimapLog -Pattern 'RG_MINIMAP_TEST PASS')) {
    throw 'Minimap drone test failed; see out/minimap_drone_test.log'
}
& $godotExe --headless --path $gameDir --quit-after 1000 --script res://scripts/test_presentation.gd *> $logPath
$testExit = $LASTEXITCODE
Get-Content $logPath
if ($testExit -ne 0 -or
    (Select-String -Path $logPath -Pattern 'SCRIPT ERROR|^ERROR') -or
    -not (Select-String -Path $logPath -Pattern 'RG_PRESENTATION_TEST ok:')) {
    throw 'Presentation tests failed; see out/presentation_test.log'
}
