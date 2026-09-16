$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location (Split-Path -Parent $root)
if (-not (Test-Path "$root\.venv")) {
    py -3 -m venv "$root\.venv"
}
& "$root\.venv\Scripts\python.exe" -m pip install -r "$root\requirements.txt"
& "$root\.venv\Scripts\python.exe" -m uvicorn server.app:app --host 0.0.0.0 --port 8080
