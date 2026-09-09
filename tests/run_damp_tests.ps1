param([string]$Python = 'python')
$ErrorActionPreference = 'Stop'
$sources = Get-ChildItem "$PSScriptRoot/../src/gen/damp/*.cpp" | ForEach-Object FullName
$output = Join-Path $PSScriptRoot '../bin/damp-tests'
New-Item -ItemType Directory -Force $output | Out-Null
foreach ($test in (Get-ChildItem "$PSScriptRoot/damp_*test.cpp")) {
    $exe = Join-Path $output ($test.BaseName + '.exe')
    & g++ -std=c++20 -O2 -Wall -Wextra -Wpedantic -I "$PSScriptRoot/../include" $test.FullName @sources -o $exe
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($test.Name)" }
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($test.Name)" }
}
& $Python -m unittest discover -s "$PSScriptRoot" -p test_damp_calibration.py -v
if ($LASTEXITCODE -ne 0) { throw 'Calibration tests failed' }
