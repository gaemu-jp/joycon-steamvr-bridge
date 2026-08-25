param(
    [Parameter(Mandatory = $true)]
    [string]$ConfigPath,
    [int]$Port = 39771
)

$ErrorActionPreference = 'Stop'
$config = (Resolve-Path -LiteralPath $ConfigPath).Path
$backup = "$config.bak-$(Get-Date -Format 'yyyyMMdd-HHmmss')"

if (-not (Test-Path -LiteralPath $config)) {
    throw "XR Animator config not found: $config"
}

Copy-Item -LiteralPath $config -Destination $backup -Force
$raw = Get-Content -LiteralPath $config -Raw
$pattern = '(%22VMC%22%3A%7B%22send%22%3A%7B%22port%22%3A)\d+'
if ($raw -notmatch $pattern) {
    throw 'VMC sender port entry was not found; no change was made.'
}

$updated = [regex]::Replace($raw, $pattern, "`${1}$Port", 1)
Set-Content -LiteralPath $config -Value $updated -Encoding UTF8 -NoNewline
Write-Output "VMC sender port set to $Port"
Write-Output "Backup: $backup"
