$ErrorActionPreference = 'Stop'

$Root = Resolve-Path (Join-Path $PSScriptRoot '..')
$SteamVr = 'C:\Program Files (x86)\Steam\steamapps\common\SteamVR'
$SteamVrDriverRoot = Join-Path $SteamVr 'drivers\joycon'
$DriverName = 'joycon'
$DriverRoot = Join-Path $Root "work\steamvr-installed\$DriverName"
$DriverBin = Join-Path $DriverRoot 'bin\win64'
$DriverResources = Join-Path $DriverRoot 'resources'
$SdkRoot = Join-Path $Root 'work\openvr'
$SdkHeaders = Join-Path $SdkRoot 'headers'
$SdkLib = Join-Path $SdkRoot 'lib\win64'
$BuildRoot = Join-Path $Root 'work\build-joycon-driver'
$OutputDll = Join-Path $BuildRoot 'driver_joycon.dll'
$CMake = 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe'
$VsDevCmd = 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\Tools\VsDevCmd.bat'
$Cl = 'C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Tools\MSVC\14.51.36231\bin\Hostx64\x64\cl.exe'

Write-Host '== Checking SteamVR =='
if (-not (Test-Path $SteamVr)) {
  throw "SteamVR not found at $SteamVr"
}
if (-not (Test-Path $Cl)) { throw "cl.exe not found at $Cl" }

Write-Host '== Checking OpenVR SDK =='
if (-not (Test-Path $SdkHeaders)) {
  Write-Host 'Cloning OpenVR SDK...'
  git clone https://github.com/ValveSoftware/openvr $SdkRoot
}

Write-Host '== Preparing driver folder =='
New-Item -ItemType Directory -Force -Path $DriverBin | Out-Null
New-Item -ItemType Directory -Force -Path $DriverResources | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $DriverResources 'icons') | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $DriverResources 'input') | Out-Null
New-Item -ItemType Directory -Force -Path (Join-Path $DriverResources 'localization') | Out-Null

Copy-Item -Force (Join-Path $Root 'work\steamvr_driver\driver.vrdrivermanifest') (Join-Path $DriverRoot 'driver.vrdrivermanifest')
Copy-Item -Force (Join-Path $Root 'work\steamvr_driver\resources\input\joycon_controller_profile.json') (Join-Path $DriverResources 'input\joycon_controller_profile.json')

Write-Host '== Configuring build =='
if (-not (Test-Path $CMake)) { throw "cmake not found at $CMake" }
if (-not (Test-Path $VsDevCmd)) { throw "Visual Studio dev prompt not found at $VsDevCmd" }

if (Test-Path $BuildRoot) {
  Remove-Item -Recurse -Force $BuildRoot
}
New-Item -ItemType Directory -Force -Path $BuildRoot | Out-Null

cmd /c "`"$VsDevCmd`" -arch=x64 -host_arch=x64 && set CXX=$Cl && set CC=$Cl && `"$CMake`" -S `"$Root\work\steamvr_driver`" -B `"$BuildRoot`" -G Ninja -DOPENVR_SDK_ROOT=`"$SdkRoot`" -DCMAKE_CXX_COMPILER=`"$Cl`" -DCMAKE_C_COMPILER=`"$Cl`" && `"$CMake`" --build `"$BuildRoot`""

if (-not (Test-Path $OutputDll)) {
  $built = Get-ChildItem $BuildRoot -Recurse -Filter driver_joycon.dll | Select-Object -First 1
  if (-not $built) {
    throw 'driver_joycon.dll was not produced'
  }
  Copy-Item -Force $built.FullName (Join-Path $DriverBin 'driver_joycon.dll')
} else {
  Copy-Item -Force $OutputDll (Join-Path $DriverBin 'driver_joycon.dll')
}

if (Test-Path $SteamVrDriverRoot) {
  Write-Host '== Syncing SteamVR runtime driver =='
  New-Item -ItemType Directory -Force -Path (Join-Path $SteamVrDriverRoot 'bin\win64') | Out-Null
  Copy-Item -Force (Join-Path $DriverBin 'driver_joycon.dll') (Join-Path $SteamVrDriverRoot 'bin\win64\driver_joycon.dll')
  Copy-Item -Force (Join-Path $DriverRoot 'driver.vrdrivermanifest') (Join-Path $SteamVrDriverRoot 'driver.vrdrivermanifest')
  New-Item -ItemType Directory -Force -Path (Join-Path $SteamVrDriverRoot 'resources\input') | Out-Null
  Copy-Item -Force (Join-Path $DriverResources 'input\joycon_controller_profile.json') (Join-Path $SteamVrDriverRoot 'resources\input\joycon_controller_profile.json')
}

Write-Host '== Registering driver =='
$VrPathReg = Join-Path $SteamVr 'bin\win64\vrpathreg.exe'
# Keep exactly one joycon registration. Duplicate registrations create two
# controller pairs, while only one driver instance can own the UDP socket.
& $VrPathReg removedriver $DriverRoot 2>$null
& $VrPathReg adddriver $SteamVrDriverRoot
& $VrPathReg show

Write-Host '== Done =='
Write-Host "Driver installed to: $DriverRoot"
Write-Host 'Next: start SteamVR, then run:'
Write-Host "python `"$Root\work\joycon_vr_bridge.py`" --input vmc --listen-port 39771 --steamvr-port 39772 --quiet"
