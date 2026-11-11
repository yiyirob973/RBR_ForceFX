# RBR_ForceFX

ForceFX is a force-feedback plugin for Richard Burns Rally (RBR). It uses NGP UDP telemetry to add configurable vibration and impact detail to a compatible DirectInput force-feedback wheel while RBR is running.

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

A standard RBR installation without NGP telemetry is not sufficient. ForceFX receives telemetry over UDP; it does not provide or configure NGP itself.

## Install and configure

1. Download the release archive from the repository's [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases) page.
2. Copy the files from the archive into RBR's `Plugins` folder. Keep the DLL and its configuration files together.
3. Enable NGP UDP telemetry in your RBR/NGP setup and configure it to send packets to the same computer and UDP port configured by `UdpPort` in `ForceFX.ini`. The shipped file contains the default port. Make sure that port is not already in use by another application.
4. Start RBR and open **Plugins > ForceFX**. Select the wheel RBR is using and connect. Enable `AutoConnect` in `ForceFX.ini` if you want ForceFX to connect automatically to the saved wheel on startup.
5. Begin with the shipped settings and increase feedback gradually. If telemetry is not received, check that the telemetry destination, UDP port, and packet layout are compatible.

ForceFX settings are stored in `ForceFX.ini`. The comments in that file describe the available settings and ranges; the in-game menu provides controls for commonly adjusted settings. Advanced telemetry offsets and scaling options are provided for compatible packet layouts. Changes made through the menu are saved to the configuration file; manual edits take effect after RBR is restarted.

`ForceFXCars.ini` contains optional per-car profiles. Profiles can adjust selected values and fall back to the global settings for unspecified values. You can edit or remove profiles, or disable profile use through the configuration. See the comments at the top of the file for the profile format and matching behavior. Cars without a matching profile use the global settings.

## Safety

ForceFX adds force feedback to a device already used by RBR. Its settings include output ceilings, frequency bounds, and gradual derating for sustained output; the plugin also fades its effects when it shuts down. These protections do not eliminate the possibility of discomfort, excessive force, or hardware wear. The landing feature can modify RBR's own steering force. Use conservative settings, especially with high-torque wheels, and stop if the wheel behaves unexpectedly.

## Troubleshooting

- **No telemetry is received:** Check the NGP telemetry output is enabled and directed to the same local UDP port as `UdpPort` in `ForceFX.ini`. Ensure another application is not occupying that port. Use the in-game diagnostics to inspect the packet count and telemetry values.
- **The wheel does not connect or effects are unavailable:** Confirm force feedback is enabled in RBR and select the same DirectInput wheel that RBR uses. Check `ForceFX.log` for connection or device errors.
- **Telemetry arrives but feedback is incorrect:** The telemetry layout may differ from the configured offsets or scaling. Review the advanced settings in `ForceFX.ini` and use the diagnostics page and `ForceFX_packet.txt` dump to help identify the issue.
- **Feedback is too strong or busy:** Lower the combined output ceiling or live level first, then adjust individual effect levels. Start conservatively and test one effect at a time.

The plugin writes `ForceFX.log` and, after receiving telemetry, `ForceFX_packet.txt` beside the DLL. Include relevant log details and your NGP/telemetry setup when reporting a problem; remove any personal information before sharing diagnostic files.

## Build from source

The release workflow builds a 32-bit DLL on Windows using the MSVC toolchain and packages it with the configuration files. To build locally, use an x86 Visual Studio developer command prompt with the Windows SDK and compile the plugin as a 32-bit DLL with the static runtime. The workflow file in `.github/workflows` is the authoritative reference for the current build and packaging steps.

Pushing a version tag beginning with `v` runs the release workflow and creates a release archive. The archive contents and workflow behavior may change; use the Releases page for published builds.

## Credits and warranty

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK.

This project is provided as is, without warranty. Use it at your own risk.
