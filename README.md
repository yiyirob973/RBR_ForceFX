# RBR ForceFX

ForceFX is a plugin for Richard Burns Rally that adds configurable force feedback and optional audio and visual effects using NGP telemetry.

> **Compatibility:** Built for the RallySimFans (RSF) version of RBR with NGP7. Other setups are not officially supported.
>
> **Caution:** ForceFX sends force feedback directly to your wheel. Begin with low settings and increase gradually. Use at your own risk.

## Features

- Additional feedback for engine vibration, road surfaces, braking, water, weight transfer and landings.
- Optional dynamic audio for tyres, impacts and road conditions.
- Optional cockpit camera movement and experimental windscreen effects.
- In-game settings, diagnostics and per-car profiles.

## Requirements

- Windows, 32-bit RBR, RallySimFans and NGP7.
- NGP UDP telemetry enabled.
- A DirectInput force-feedback wheel configured in RBR for force feedback features.
- Microsoft Visual C++ 2015–2022 Redistributable (x86).

## Install

1. Download the latest `ForceFX-<version>.zip` from [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases).
2. Extract its contents into RBR's `Plugins` folder. Keep the DLL and configuration files together.
3. Enable NGP UDP telemetry.
4. Launch RBR, open **Plugins > ForceFX**, select your wheel and connect. To connect automatically on startup, set `AutoConnect=1` in `ForceFX.ini`.

## Configuration

Use **Plugins > ForceFX** for common settings. Additional options and their descriptions are in `ForceFX.ini`; per-car settings are in `ForceFXCars.ini`.

If the output feels too strong, reduce **Total output** in the menu or lower `TotalMaxMagnitude` in the configuration file. Numpad **+** and **-** adjust the live level; **Numpad \*** mutes. These keys can be changed in the configuration.

## Optional features

**Dynamic audio** is off by default. Enable it in the menu or set `EnableDynamicAudio=1`. To use sounds from your RBR installation, run `ForceFXAudio\Copy-RbrSounds.bat`; sound settings are in the `[Audio]` section of `ForceFX.ini`.

**Camera movement** is off by default. Enable it in the menu or set `EnableCamera=1` in `ForceFX.ini`.

**Windscreen effects** are experimental and off by default. Enable them in the menu or set `EnableWindscreen=1`. They may affect performance or stability.

## Troubleshooting

- **No telemetry:** Confirm that NGP UDP telemetry is enabled. Check the packet count and source in the ForceFX menu.
- **No force feedback:** Confirm force feedback is enabled in RBR and the correct wheel is selected. Reduce output if it feels too strong.
- **Unexpected behaviour:** Check `ForceFX.log` beside the plugin DLL. For camera or windscreen issues, also include `ForceFXCamera.log`.

When reporting an issue, include the relevant log and a brief description of your RBR and NGP setup. Remove personal information from logs before sharing.

## Build from source

Build the 32-bit Windows DLL with MSVC. From an x86 Visual Studio command prompt, run `build.bat`; the output is `out\RBR_ForceFX.dll`. Unit tests are in `tests/` and `audio/tests/`.

## Credits

The project includes plugin interface headers from the RBR SDK and [MinHook](https://github.com/TsudaKageyu/minhook) (BSD 2-Clause). See `thirdparty/` for license details.
