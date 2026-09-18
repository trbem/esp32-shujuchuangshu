param(
    [Parameter(Mandatory = $true)]
    [string]$TunnelToken
)

$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = New-Object Security.Principal.WindowsPrincipal($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "Run this script in an elevated PowerShell window."
}

$cloudflared = Get-Command cloudflared -ErrorAction SilentlyContinue
if (-not $cloudflared) {
    throw "Install cloudflared first, then rerun this script."
}

& $cloudflared.Source service install $TunnelToken
if ($LASTEXITCODE -ne 0) { throw "cloudflared service installation failed" }
Write-Host "Cloudflare Tunnel service installed. Configure both public hostnames to http://127.0.0.1:8080 in Cloudflare."
