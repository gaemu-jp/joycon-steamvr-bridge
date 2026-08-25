# Joy-Con SteamVR Bridge

Experimental Windows prototype for exposing two Nintendo Joy-Cons as tracked
SteamVR controllers.

The project combines:

- Joy-Con rotation, buttons, and sticks from Bluetooth HID
- hand positions and slow rotation-drift correction from XR Animator VMC
- a Python UDP bridge
- a native OpenVR server driver that registers left and right controllers

This is an archived prototype, not a finished replacement for a tracked VR
controller. Position quality depends on XR Animator camera tracking, Joy-Con
yaw can drift, and the orientation mapping was tuned experimentally.

> [!WARNING]
> This project installs an experimental third-party driver into SteamVR and
> starts background bridge processes. Review the scripts before running them,
> close SteamVR before installing or replacing the driver, and use this project
> at your own risk. It may cause SteamVR startup failures, controller conflicts,
> tracking errors, or require manual driver removal. It is not affiliated with
> or endorsed by Nintendo, Valve, or XR Animator.

## Data Flow

```text
Joy-Con Bluetooth HID ---- rotation/buttons/sticks ---+
                                                       +--> Python bridge
XR Animator VMC :39771 -- hand position/correction ---+         |
                                                                 v
                                                        UDP :39772
                                                                 |
                                                                 v
                                                        OpenVR driver
                                                                 |
                                                                 v
                                                              SteamVR
```

## Repository Layout

- `work/joycon_vr_bridge.py`: input parsing, pose fusion, calibration, and UDP output
- `work/steamvr_driver/`: native OpenVR driver source and input profile
- `work/setup_joycon_steamvr.ps1`: builds, installs, and registers the driver
- `work/start_joycon_vr.ps1`: starts exactly one fusion bridge process
- `work/set_xr_animator_vmc_port.ps1`: updates an XR Animator config backup safely

The OpenVR SDK, build output, installed driver, generated DLLs, caches, and
runtime logs are intentionally excluded from Git.

## Requirements

- Windows 10 or 11
- Steam and SteamVR
- Python 3.12
- paired left and right Joy-Cons
- Visual Studio C++ Build Tools with CMake and Ninja
- XR Animator configured to send VMC data to UDP port `39771`

Install Python dependencies:

```powershell
python -m pip install -r requirements.txt
```

## Build And Install

The setup script clones Valve's OpenVR SDK when it is missing, builds
`driver_joycon.dll`, copies the driver into SteamVR, and registers it with
`vrpathreg`.

```powershell
powershell -ExecutionPolicy Bypass -File .\work\setup_joycon_steamvr.ps1
```

The script was written for the original machine's default SteamVR location and
Visual Studio 2026 Build Tools layout. Adjust the paths near the top of the
script for a different installation.

Start SteamVR, start XR Animator motion capture/VMC manually, then run:

```powershell
powershell -ExecutionPolicy Bypass -File .\work\start_joycon_vr.ps1
```

## Runtime Ports

- `26760/UDP`: optional BetterJoy DSU fallback
- `39771/UDP`: XR Animator VMC input
- `39772/UDP`: Python bridge to native SteamVR driver

## Controller Recenter

Direct HID starts with an arbitrary physical orientation. Hold each controller
in its intended neutral pose before recentering:

- right Joy-Con: ABXY face up, R edge forward, press `PLUS + right stick click`
- left Joy-Con: button face up, L edge forward, press `MINUS + left stick click`

Recenter also resets the VMC drift-correction reference for that hand.

## Current Limitations

- Camera-derived positions are not equivalent to lighthouse or inside-out controller tracking.
- Joy-Con IMUs have no absolute yaw reference and require correction/recentering.
- The VMC correction is deliberately slow so camera rotation does not replace Joy-Con motion.
- Input bindings use Vive compatibility and may need per-game customization.
- The orientation conversion contains empirical sign corrections from the original hardware tests.

## Privacy And Repository Hygiene

Runtime logs can contain timestamps, controller input, tracking poses, local
paths, and device diagnostics. Do not attach or commit logs without reviewing
them first. The repository ignores logs, generated DLLs, SDK downloads, build
directories, caches, and installed-driver output by default.

## License

Released under the [MIT License](LICENSE). The software is provided as-is,
without warranty. Third-party projects and dependencies remain subject to
their own licenses and trademarks.
