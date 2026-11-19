# RBR_ForceFX

Extra force feedback for Richard Burns Rally. ForceFX reads NGP telemetry and adds engine, road, braking and landing detail to your wheel on top of RBR's own steering force.

> **Use at your own risk.** ForceFX sends force feedback straight to your wheel. Output limits and gradual derating are built in, but they cannot guarantee that hardware will not be damaged. Start with the shipped settings, raise things slowly, and stop if the wheel behaves strangely.
>
> It is built and tested for the RallySimFans (RSF) version of RBR with NGP7. Other setups may work but are not supported. AI tools were used in its development.

## Features

- **Engine:** vibration that follows rpm, throttle and the rev limit.
- **Road:** vibration that follows the surface under each tyre (gravel, snow, ice, grass, tarmac, cobbles) together with speed, wheel slip and how rough the tyres measure.
- **Braking:** front-end judder that gets harsh on lock-up.
- **Landings:** jumps and hard hits make RBR's own steering force pull harder for a moment.
- **Optional dynamic surface audio:** tyre and ground sounds that follow each wheel (separate download, off by default).
- An in-game menu (**Plugins > ForceFX**) with per-effect switches, settings and diagnostics.
- Optional per-car profiles.

## Requirements

- Windows and 32-bit RBR.
- RallySimFans RBR with NGP7 and UDP telemetry enabled.
- A DirectInput force-feedback wheel that RBR already uses.

## Install

1. Download a release from the [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases) page:
   - `ForceFX-<version>.zip` includes the optional dynamic audio.
   - `ForceFX-<version>-NoAudio.zip` is force feedback only.

   Use one, not both.
2. Copy the contents into RBR's `Plugins` folder and keep the DLL and `.ini` files together.
3. Turn on NGP UDP telemetry. The destination doesn't matter, so it can keep pointing at SimHub or similar.
4. Start RBR, open **Plugins > ForceFX**, pick your wheel and connect. Set `AutoConnect=1` in `ForceFX.ini` to connect automatically next time.

## Settings

Settings live in `ForceFX.ini`, which describes every option. The most useful ones are also in the in-game menu.

- **Road gain** (`RoadMaxMagnitude`) is the one setting for how strong the road feel is. The surface and what the car is doing decide what it feels like.
- **Road feel delivery** (`RoadViaRbr`, also in the menu): by default the road vibration is added to RBR's own steering force, so the wheel gets no extra road effect. Set it to 0 to play the road as one sine effect of its own: less detailed, but easy on the wheel. It does this by itself when RBR's force updates aren't available. Use the default if bumps make the wheel feel heavy or sticky to turn; the road may then need a higher road gain.
- **Total output** (`TotalMaxMagnitude`) caps all effects together. If the wheel feels too strong or busy, lower this first.
- **Live level** scales everything down while you drive: Numpad **+** and **-** change it, Numpad **\*** mutes the added effects (keys are configurable in `ForceFX.ini`). It can only turn effects down, never past your limits.
- **Per-car profiles** are in `ForceFXCars.ini`. A profile can change a car's engine and road strength and set its `Drivetrain=RWD|FWD|AWD`. Cars without an entry are treated as RWD. The drivetrain decides which tyres can have wheelspin.

## Road feel

ForceFX reads which material is under each tyre from RBR's memory (read-only) and combines it with the telemetry. Loose ground is grainy, soft ground is dull, ice is hard and sharp, and cobbles hit in time with your speed. Standing still gives no vibration.

- If a material isn't recognised, it is treated as plain hard ground and the road still follows the telemetry.
- Cobblestones and wooden bridges have no material of their own in the game. Cobbles are detected from the telemetry, and any material ID can be assigned with `Map=` in the `[Surface]` section of `ForceFX.ini`, for example `Map=201:cobble,202:wood`. Restart RBR after changing it.
- The ground is only read while a stage is running. It relies on the memory layout of the RSF `RichardBurnsRally_SSE.exe`.
- The Diagnostics page shows the detected surface and the material IDs under the tyres. `ForceFX.log` records surface changes.

## Dynamic surface audio

Audio build only. It is off by default: switch on **Dynamic audio** in the menu, or set `EnableDynamicAudio=1`.

1. Run `ForceFXAudio\Copy-RbrSounds.bat` to copy matching sounds from your RBR `Audio` folder. See the short readme beside it. You can also put your own WAV files in `ForceFXAudio\sounds`.
2. Mix levels and the output device are in `ForceFXAudio.ini`.

`audio/surfaces.md` lists the sound names ForceFX looks for.

## Troubleshooting

- **No telemetry:** check that NGP UDP telemetry is enabled. The ForceFX menu shows the packet count and source.
- **Wheel won't connect or has no effects:** make sure force feedback is on in RBR and that you picked the same wheel RBR uses. Check `ForceFX.log`.
- **Road doesn't react to the ground:** open the Diagnostics page. "looking for the tyres" means the tyre data wasn't found, usually because the RBR build is different. "unknown" means the material isn't recognised; assign it with `Map=`.
- **Too strong or busy:** lower the total output or the live level first, then the individual effects.

When reporting a problem, attach `ForceFX.log` (it is beside the DLL) and describe your NGP setup. Remove any personal information first.

## Build from source

32-bit Windows DLL, built with MSVC. From an x86 Visual Studio command prompt run `build.bat`. It builds both variants into `out\audio` and `out\noaudio`. The audio module is behind the `FORCEFX_AUDIO` define. A version tag starting with `v` builds a release with both archives.

Unit tests for the logic headers are in `tests` and `audio/tests`. Each file has its build line at the top.

## Credits

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK. Provided as is, without warranty.
