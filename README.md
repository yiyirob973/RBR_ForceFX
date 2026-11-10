# RBR_ForceFX

RBR_ForceFX is a community-developed force feedback enhancement plugin for Richard Burns Rally (RBR) using NGP telemetry. It adds extra vibration and impact detail on top of RBR's native steering force.

It is designed to improve the feedback from:

- engine vibration and roughness
- road surface feel and tyre load response
- braking judder and lock-up cues
- landing and impact events
- per-car tuning profiles

## Why use it?

RBR already does a great job with steering force, but many sim racers want a richer sense of the car, the road, and the suspension. RBR_ForceFX adds layered effects that react to telemetry such as wheel speed, engine RPM, braking, suspension movement, and tyre load.

The result is a more detailed and immersive wheel experience, with a focus on road feel, weight transfer, and more convincing landing impacts.

## Features

- Road and tyre feedback with side-aware vibration and texture variation
- Engine vibration tuned across the rev range
- Brake judder and front-lock cues
- Landing boost: a detected landing or hard hit scales up RBR's own steering force (no extra effect is layered on top)
- Load-sensitive suspension and weight feel adjustments
- Per-car profiles matched by car name
- In-game settings menu in RBR for live tuning
- Safety limits and thermal derating to protect the wheel and hardware

## Requirements

- Windows
- The latest RallySimFans (RSF) version of Richard Burns Rally, with NGP7
- A DirectInput-compatible force feedback wheel
- NGP7 UDP telemetry enabled and configured to send data to RBR_ForceFX

## Installation

1. Install and launch the latest RallySimFans (RSF) version of RBR. NGP7 is provided as part of the RSF installation; a standard RBR installation without NGP7 is not supported.
2. Copy `RBR_ForceFX.dll`, `ForceFX.ini`, and `ForceFXCars.ini` into your RBR `Plugins` directory (or the corresponding plugin location used by your RSF installation).
3. Enable NGP UDP telemetry in the RSF Launcher. Open its NGP/Telemetry settings, turn on UDP telemetry, and set the telemetry destination to `127.0.0.1` (this PC) and port `6778`. Save/apply the settings. Launcher labels can vary between RSF versions; make sure UDP telemetry output is enabled, not just another telemetry feature.
4. Open `ForceFX.ini` and check `UdpPort=6778`. The port must match the port configured for NGP telemetry. If you change it in one place, use the same port in the other.
5. Launch RBR and open the plugin settings from **RBR main menu > Plugins > ForceFX**. Select your wheel and adjust the settings as needed.

The repository includes:

- `RBR_ForceFX.dll`
- `ForceFX.ini`
- `ForceFXCars.ini`

## Configuration

The main setup file is `ForceFX.ini`. It controls:

- overall strength and safety ceilings
- engine, road, brake, and impact gain
- frequency ranges
- landing sensitivity
- live tuning level and hotkeys
- telemetry offsets and connection settings, including the UDP port

The car-specific setup is in `ForceFXCars.ini`, which allows tuning by vehicle type and name.

## In-game controls

The plugin supports live adjustment from RBR while driving. Common features include:

- live gain controls
- menu-based tuning of strength and frequency settings
- quick reset and diagnostics support

The exact hotkeys and settings can be adjusted in `ForceFX.ini`.

## Safety and design notes

This plugin enhances RBR's existing force feedback; it does not replace or override the simulator's steering force. Added effects are zero-mean vibrations layered on top of the wheel's normal steering force, except for the landing boost, which briefly scales RBR's own steering force.

The plugin includes safety protections such as:

- hard output ceilings
- frequency bounds
- gain clamping
- thermal derating under sustained heavy load

## Important note

This project is intended for enthusiasts and advanced users. It relies on telemetry and uses aggressive vibration shaping, so calibration is personal and may require tuning depending on the wheel and hardware.

## License and support

This project is provided as-is for the community. Please use it responsibly and ensure your hardware and game setup are compatible before running it for long sessions.

## Project status

RBR_ForceFX is a feature-rich plugin designed for players who want a stronger and more detailed sense of road, engine, suspension, and impact events in Richard Burns Rally.
