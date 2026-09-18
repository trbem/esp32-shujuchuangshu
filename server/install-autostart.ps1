param(
    [string]$TaskName = "ESP32 Telemetry Collector"
)

$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Run this script in an elevated PowerShell window."
}

$runner = Join-Path $PSScriptRoot "run-production.cmd"
if (-not (Test-Path $runner)) { throw "Missing $runner" }

$action = New-ScheduledTaskAction -Execute $runner
$trigger = New-ScheduledTaskTrigger -AtStartup
$taskPrincipal = New-ScheduledTaskPrincipal -UserId "SYSTEM" -RunLevel Highest
$settings = New-ScheduledTaskSettingsSet -RestartCount 3 -RestartInterval (New-TimeSpan -Minutes 1) `
    -ExecutionTimeLimit (New-TimeSpan -Days 3650) -StartWhenAvailable
Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger -Principal $taskPrincipal `
    -Settings $settings -Description "Starts the local-only ESP32 telemetry FastAPI server at Windows boot" -Force | Out-Null
Write-Host "Installed '$TaskName'. It starts server/run-production.cmd at boot."
