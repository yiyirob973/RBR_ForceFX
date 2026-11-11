# RBR_ForceFX

ForceFX is a force-feedback plugin for Richard Burns Rally (RBR). It uses NGP telemetry to add configurable vibration and impact detail to a compatible DirectInput force-feedback wheel while RBR is running. It reads the telemetry as NGP sends it, so it does not need a UDP port of its own.

## Features

- Configurable engine, road/suspension, braking, and landing feedback.
- An in-game menu for connecting a wheel, adjusting settings, and checking telemetry diagnostics.
- Optional per-car profiles for tuning the feedback character of individual cars.
- Output limits and gradual derating intended to reduce sustained load on the wheel.

The precise effect behavior and available settings may change between releases. Use the settings menu and the comments in the shipped configuration files as the current reference.

## Requirements

- Windows and a 32-bit RBR installation.
- An RBR setup with a compatible NGP version and UDP telemetry support. The project is developed for RallySimFans (RSF) with NGP7; other setups or packet layouts may require configuration and are not guaranteed to work.
- A DirectInput force-feedback wheel recognized by RBR.

A standard RBR installation without NGP telemetry is not sufficient. ForceFX does not provide or configure NGP itself.

## Install and configure

1. Download the release archive from the repository's [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases) page.
2. Copy the files from the archive into RBR's `Plugins` folder. Keep the DLL and its configuration files together.
3. Enable NGP UDP telemetry in your RBR/NGP setup. The destination does not matter: ForceFX picks the packets up inside the RBR process as NGP sends them, so you can leave NGP pointed at SimHub or any other tool and nothing else needs configuring. (If the hook cannot see NGP's packets on your setup, ForceFX also listens on the backup port `UdpPort` in `ForceFX.ini`. Point NGP at that port as an extra endpoint, or see `TelemetrySource` in the same file.)
4. Start RBR and open **Plugins > ForceFX**. Select the wheel RBR is using and connect. Enable `AutoConnect` in `ForceFX.ini` if you want ForceFX to connect automatically to the saved wheel on startup.
5. Begin with the shipped settings and increase feedback gradually. If telemetry is not received, check that NGP UDP telemetry is enabled and use the troubleshooting notes below.

ForceFX settings are stored in `ForceFX.ini`. The comments in that file describe the available settings and ranges; the in-game menu provides controls for commonly adjusted settings. Advanced telemetry offsets and scaling options are provided for compatible packet layouts. Changes made through the menu are saved to the configuration file; manual edits take effect after RBR is restarted.

`ForceFXCars.ini` contains optional per-car profiles. Profiles can adjust selected values and fall back to the global settings for unspecified values. You can edit or remove profiles, or disable profile use through the configuration. See the comments at the top of the file for the profile format and matching behavior. Cars without a matching profile use the global settings.

## Safety

ForceFX adds force feedback to a device already used by RBR. Its settings include output ceilings, frequency bounds, and gradual derating for sustained output; the plugin also fades its effects when it shuts down. These protections do not eliminate the possibility of discomfort, excessive force, or hardware wear. The landing feature can modify RBR's own steering force. Use conservative settings, especially with high-torque wheels, and stop if the wheel behaves unexpectedly.

## Troubleshooting

- **No telemetry is received:** Check that NGP UDP telemetry is enabled, then open the ForceFX menu. The telemetry line shows the packet count and where packets come from (NGP or the UDP port), and the diagnostics page shows how the send hook was installed and how many sends it has seen. If the hook shows no sends, point NGP at `UdpPort` as an extra endpoint and make sure no other application uses that port. Setting `TelemetrySource` to `2` turns the hook off entirely if you suspect it.
- **The wheel does not connect or effects are unavailable:** Confirm force feedback is enabled in RBR and select the same DirectInput wheel that RBR uses. Check `ForceFX.log` for connection or device errors.
- **Telemetry arrives but feedback is incorrect:** The telemetry layout may differ from the configured offsets or scaling. Review the advanced settings in `ForceFX.ini` and use the diagnostics page and `ForceFX_packet.txt` dump to help identify the issue.
- **Feedback is too strong or busy:** Lower the combined output ceiling or live level first, then adjust individual effect levels. Start conservatively and test one effect at a time.

The plugin writes `ForceFX.log` and, after receiving telemetry, `ForceFX_packet.txt` beside the DLL. Include relevant log details and your NGP/telemetry setup when reporting a problem; remove any personal information before sharing diagnostic files.

## Build from source

The build workflow compiles a 32-bit DLL on Windows using the MSVC toolchain and packages it with the configuration files. To build locally, use an x86 Visual Studio developer command prompt with the Windows SDK and compile the plugin as a 32-bit DLL with the static runtime. The workflow file in `.github/workflows` is the authoritative reference for the current build and packaging steps.

Pushing a version tag beginning with `v` creates a release archive. Pushing to the `noudp` branch (the telemetry hook work) builds the plugin and attaches the zip to the workflow run as a downloadable artifact, without creating a release. The archive contents and workflow behavior may change; use the Releases page for published builds.

## Credits and warranty

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK.

This project is provided as is, without warranty. Use it at your own risk.
