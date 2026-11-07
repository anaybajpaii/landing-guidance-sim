Set-Location (Split-Path -Parent $PSScriptRoot)
New-Item -ItemType Directory -Force build | Out-Null

$tests = [ordered]@{
    "physics"    = @("src/lander.cpp")
    "guidance"   = @("src/guidance.cpp")
    "pid"        = @("src/pid.cpp")
    "controller" = @("src/controller.cpp", "src/pid.cpp")
    "touchdown"  = @("src/touchdown.cpp")
    "config"     = @("src/config.cpp")
    "runner"     = @("src/runner.cpp", "src/config.cpp", "src/lander.cpp", "src/logger.cpp", "src/guidance.cpp", "src/pid.cpp", "src/controller.cpp", "src/touchdown.cpp")
    "system"     = @("src/lander.cpp", "src/guidance.cpp", "src/pid.cpp", "src/controller.cpp", "src/touchdown.cpp")
}

$failed = 0
foreach ($name in $tests.Keys) {
    $sources = $tests[$name]
    & g++ -std=c++17 -Wall -Wextra -Isrc "tests/test_$name.cpp" @sources -o "build/test_$name.exe"
    if ($LASTEXITCODE -ne 0) {
        Write-Host "BUILD FAILED  $name"
        exit 1
    }

    $output = & "./build/test_$name.exe"
    $code = $LASTEXITCODE
    $summary = $output | Select-Object -Last 1
    if ($code -ne 0) {
        $failed++
        Write-Host "FAIL  $name  $summary"
    } else {
        Write-Host "ok    $name  $summary"
    }
}

if ($failed -gt 0) {
    Write-Host "`n$failed of $($tests.Count) test programs failed"
    exit 1
}
Write-Host "`nall $($tests.Count) test programs passed"