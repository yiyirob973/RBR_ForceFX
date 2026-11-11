# RBR_ForceFX

> **Disclaimer:** AI tools were used during the development of this project. ForceFX was built for the RallySimFans (RSF) version of Richard Burns Rally (RBR), using NGP7. It may work with other RBR versions or setups, but compatibility has not been verified and is not guaranteed. Testing has been performed, and every effort has been made to reduce the risk of hardware damage. However, ForceFX sends force-feedback output directly to a wheel, so hardware damage is possible. Use this software at your own risk.

ForceFX is a force-feedback plugin for Richard Burns Rally (RBR). It uses NGP telemetry to add configurable vibration and impact detail to a compatible DirectInput force-feedback wheel while RBR is running.

## Features

- Configurable engine, road/suspension, braking, and landing feedback.
- An in-game menu for connecting a wheel, adjusting settings, and checking telemetry diagnostics.
- Optional per-car profiles for tuning the feedback character of individual cars.
- Output limits and gradual derating intended to reduce sustained load on the wheel.

The precise effect behavior and available settings may change between releases. Use the settings menu and the comments in the shipped configuration files as the current reference.

## Requirements

- Windows and a 32-bit RBR installation.
- An RBR setup with a compatible NGP version and UDP telemetry support. ForceFX is developed for RallySimFans (RSF) with NGP7. Other setups or packet layouts may require configuration and are not guaranteed to work.
- A DirectInput force-feedback wheel recognized by RBR.

A standard RBR installation without NGP telemetry is not sufficient. ForceFX does not provide or configure NGP itself.

## Install and configure

1. Download the release archive from the repository's [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases) page.
2. Copy the files from the archive into RBR's `Plugins` folder. Keep the DLL and its configuration files together.
3. Enable NGP UDP telemetry in your RBR/NGP setup. The destination does not matter: ForceFX picks the packets up inside the RBR process as NGP sends them, so you can leave NGP pointed at SimHub or another telemetry application.
4. Start RBR and open **Plugins > ForceFX**. Select the wheel RBR is using and connect. Enable `AutoConnect` in `ForceFX.ini` if you want ForceFX to connect automatically to the saved wheel on startup.
5. Begin with the shipped settings and increase feedback gradually. If telemetry is not received, check that NGP UDP telemetry is enabled and use the troubleshooting notes below.

ForceFX settings are stored in `ForceFX.ini`. The comments in that file describe the available settings and ranges; the in-game menu provides controls for commonly adjusted settings. Advanced telemetry settings may require configuration for non-RSF setups.

`ForceFXCars.ini` contains optional per-car profiles. Profiles can adjust selected values and fall back to the global settings for unspecified values. You can edit or remove profiles, or disable per-car profiles in the configuration.

## Safety

ForceFX is developed for RallySimFans/NGP7 setups. Other versions or setups may work, but compatibility is not guaranteed. ForceFX adds force feedback to a device already used by RBR. Its settings include output ceilings, frequency bounds, and gradual derating for sustained output; the plugin also fades its effects when appropriate. These measures are intended to reduce risk, but they cannot guarantee that damage will not occur. Start with conservative settings, increase feedback gradually, and stop using the plugin if the wheel behaves unexpectedly. Because feedback is sent directly to your wheel, use ForceFX at your own risk.

## Troubleshooting

- **No telemetry is received:** Check that NGP UDP telemetry is enabled, then open the ForceFX menu. The telemetry line shows the packet count and where packets come from (NGP or the UDP port), and diagnostic information.
- **The wheel does not connect or effects are unavailable:** Confirm force feedback is enabled in RBR and select the same DirectInput wheel that RBR uses. Check `ForceFX.log` for connection or device errors.
- **Telemetry arrives but feedback is incorrect:** The telemetry layout may differ from the configured offsets or scaling. Review the advanced settings in `ForceFX.ini` and use the diagnostics page to inspect the received values.
- **Feedback is too strong or busy:** Lower the combined output ceiling or live level first, then adjust individual effect levels. Start conservatively and test one effect at a time.

The plugin writes `ForceFX.log` and, after receiving telemetry, `ForceFX_packet.txt` beside the DLL. Include relevant log details and your NGP/telemetry setup when reporting a problem; remove any personal information before sharing logs.

## Build from source

The build workflow compiles a 32-bit DLL on Windows using the MSVC toolchain and packages it with the configuration files. To build locally, use an x86 Visual Studio developer command prompt with the required MSVC tools installed.

Pushing a version tag beginning with `v` creates a release archive. Pushing to the `noudp` branch (the telemetry hook work) builds the plugin and attaches the zip to the workflow run as a downloadable artifact.

## Credits and warranty

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK.

This project is provided as is, without warranty. Use it at your own risk.
