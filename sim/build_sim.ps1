$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..")
$BuildDir = Join-Path $ScriptDir "build"
$Output = Join-Path $BuildDir "dm4340_sim.dll"

New-Item -ItemType Directory -Force $BuildDir | Out-Null

$gccCommand = Get-Command gcc -ErrorAction Stop

$includes = @(
    "-I$ScriptDir\host",
    "-I$RepoRoot\Task",
    "-I$RepoRoot\Algorithm\kinematic",
    "-I$RepoRoot\Algorithm\dynamics"
)

$sources = @(
    "$ScriptDir\host\sim_core.c",
    "$RepoRoot\Task\arm_task.c",
    "$RepoRoot\Task\protocol_handler.c",
    "$RepoRoot\Algorithm\kinematic\arm.c",
    "$RepoRoot\Algorithm\dynamics\arm_g.c"
)

& $gccCommand.Source -shared -O2 -DHOST_SIM -o $Output @includes @sources -lm
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

Write-Host "Built $Output"
