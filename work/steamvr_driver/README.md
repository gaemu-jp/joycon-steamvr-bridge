# Joy-Con SteamVR driver

This folder contains the SteamVR/OpenVR server driver that consumes UDP packets
emitted by `../joycon_vr_bridge.py`.

What this driver does:
- create a right-hand or left-hand controller device in SteamVR
- read pose packets from `127.0.0.1:39772`
- forward quaternion pose data and basic button values to SteamVR

Important:
- This is a native SteamVR driver, so it must be built against the OpenVR SDK
  and loaded by SteamVR.
- The repository here does not include the OpenVR headers or compiled DLL.
- `setup_joycon_steamvr.ps1` builds and registers the driver.

Packet format:
```json
{
  "device": "right",
  "timestamp": 1234567890.123,
  "role": "right",
  "pose": {
    "quat": [0, 0, 0, 1],
    "euler_deg": [0, 0, 0]
  },
  "inputs": {
    "trigger": 0.0,
    "grip": 0.0,
    "menu": false,
    "application_menu": false,
    "stick": {"x": 0.0, "y": 0.0}
  }
}
```

Build notes:
- Put the OpenVR SDK headers under a local include path.
- Build a `driver_joycon.dll` that exports `HmdDriverFactory`.
- Place the driver directory under SteamVR's driver folder or register it with
  `vrpathreg`.

Expected output layout:
```text
joycon/
  bin/win64/driver_joycon.dll
  driver.vrdrivermanifest
  resources/
```
