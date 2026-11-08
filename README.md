# RBR_ForceFX

RBR_ForceFX is a community-developed force feedback enhancement plugin for Richard Burns Rally (RBR) using NGP telemetry. It adds extra vibration and impact detail on top of RBR's native steering force, making the wheel feel more alive without replacing the base sim's steering model.

It is designed to improve the feedback from:

- engine vibration and roughness
- road surface feel and tyre load response
- braking judder and lock-up cues
- landing and impact events
- per-car tuning profiles

## Why use it?

RBR already does a great job with steering force, but many sim racers want a richer sense of the car, the road, and the suspension. RBR_ForceFX adds layered effects that react to telemetry such as wheel load, tyre roughness, engine speed, suspension travel, and car identity.

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

- Richard Burns Rally
- NGP7 telemetry enabled and outputting data to the plugin
- A DirectInput-compatible force feedback wheel
- Windows

## Installation

1. Place the plugin files in the RBR plugins directory or the appropriate install location used by your RBR setup.
2. Ensure the telemetry port configured in NGP matches the plugin's UDP port (`ForceFX.ini`).
3. Launch RBR and open the plugin settings from:
   - RBR main menu > Plugins > ForceFX
4. Select your wheel and adjust settings as needed.

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
- telemetry offsets and connection settings

The car-specific setup is in `ForceFXCars.ini`, which allows tuning by vehicle type and name.

## In-game controls

The plugin supports live adjustment from RBR while driving. Common features include:

- live gain controls
- menu-based tuning of strength and frequency settings
- quick reset and diagnostics support

The exact hotkeys and settings can be adjusted in `ForceFX.ini`.

## Safety and design notes

This plugin enhances RBR's existing force feedback; it does not replace or override the simulator's steering force. All added effects are zero-mean vibrations layered on top of the wheel's normal steering output.

The plugin includes safety protections such as:

- hard output ceilings
- frequency bounds
- gain clamping
- thermal derating under sustained heavy load

## Important note

This project is intended for enthusiasts and advanced users. It relies on telemetry and uses aggressive vibration shaping, so calibration is often personal and may require tuning depending on wheel hardware, seating, and preference.

## License and support

This project is provided as-is for the community. Please use it responsibly and ensure your hardware and game setup are compatible before running it for long sessions.

## Project status

RBR_ForceFX is a feature-rich plugin designed for players who want a stronger and more detailed sense of road, engine, suspension, and impact events in Richard Burns Rally.

