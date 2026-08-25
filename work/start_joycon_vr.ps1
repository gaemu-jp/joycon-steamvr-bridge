$ErrorActionPreference = 'Stop'

$Bridge = (Resolve-Path (Join-Path $PSScriptRoot 'joycon_vr_bridge.py')).Path
$Python = (Get-Command python).Source
$Log = Join-Path $PSScriptRoot 'bridge-fusion.log'
$ErrorLog = Join-Path $PSScriptRoot 'bridge-fusion.err.log'

# Multiple bridges integrate the same gyro independently and fight over the
# SteamVR pose. Keep exactly one sender for this bridge script.
$existing = Get-CimInstance Win32_Process | Where-Object {
    $_.Name -match '^python(w)?\.exe$' -and
    $_.CommandLine -and
    $_.CommandLine -like "*$Bridge*"
}
foreach ($process in $existing) {
    Stop-Process -Id $process.ProcessId -Force -ErrorAction SilentlyContinue
}

Start-Sleep -Milliseconds 300
Remove-Item $Log, $ErrorLog -ErrorAction SilentlyContinue

$arguments = @(
    $Bridge,
    '--input', 'fusion',
    '--tcp-host', '127.0.0.1',
    '--tcp-port', '26760',
    '--listen-port', '39771',
    '--steamvr-port', '39772',
    '--trace',
    '--alpha', '1.0',
    '--quiet'
)

$bridgeProcess = Start-Process -FilePath $Python -ArgumentList $arguments `
    -WorkingDirectory (Split-Path $PSScriptRoot -Parent) `
    -RedirectStandardOutput $Log -RedirectStandardError $ErrorLog -PassThru

Start-Sleep -Seconds 1
if ($bridgeProcess.HasExited) {
    throw "Joy-Con bridge exited. See $ErrorLog"
}

Write-Host "Joy-Con VR bridge started: PID $($bridgeProcess.Id)"
Write-Host "Log: $Log"
