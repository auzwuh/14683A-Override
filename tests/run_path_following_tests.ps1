param(
    [switch]$Tune  # also build and run tests/tune_ramsete_lqr.cpp (~20000-sample random search, a few seconds)
)

$ErrorActionPreference = 'Stop'

$Sources = Get-ChildItem "$PSScriptRoot/../src/gen/path_following/*.cpp" | ForEach-Object FullName
$Common = @('-std=c++20', '-O2', '-Wall', '-Wextra', '-Wpedantic', '-I', "$PSScriptRoot/../include")
$MathExe = "$PSScriptRoot/path_following_math_test.exe"
$BenchmarkExe = "$PSScriptRoot/path_following_benchmark.exe"
$TuneExe = "$PSScriptRoot/tune_ramsete_lqr.exe"

try {
    & g++ @Common "$PSScriptRoot/path_following_math_test.cpp" @Sources -o $MathExe
    if ($LASTEXITCODE -ne 0) { throw "math test compilation failed ($LASTEXITCODE)" }
    & $MathExe
    if ($LASTEXITCODE -ne 0) { throw "math test failed ($LASTEXITCODE)" }

    & g++ @Common "$PSScriptRoot/path_following_benchmark.cpp" @Sources -o $BenchmarkExe
    if ($LASTEXITCODE -ne 0) { throw "benchmark compilation failed ($LASTEXITCODE)" }
    & $BenchmarkExe
    if ($LASTEXITCODE -ne 0) { throw "benchmark failed ($LASTEXITCODE)" }

    if ($Tune) {
        & g++ @Common "$PSScriptRoot/tune_ramsete_lqr.cpp" @Sources -o $TuneExe
        if ($LASTEXITCODE -ne 0) { throw "tuning search compilation failed ($LASTEXITCODE)" }
        & $TuneExe
        if ($LASTEXITCODE -ne 0) { throw "tuning search failed ($LASTEXITCODE)" }
    }
} finally {
    Remove-Item -LiteralPath $MathExe, $BenchmarkExe, $TuneExe -Force -ErrorAction SilentlyContinue
}
