# RBR_ForceFX

Extra force feedback for Richard Burns Rally. ForceFX reads NGP telemetry and adds engine, road, braking and landing detail to your wheel on top of RBR's own steering force. Optional modules add dynamic surface audio and cockpit camera movement.

> **Use at your own risk.** ForceFX sends force feedback straight to your wheel. Output limits and gradual derating are built in, but they cannot guarantee that hardware will not be damaged. Start with the shipped settings, raise things slowly, and stop if the wheel behaves strangely.
>
> It is built and tested for the RallySimFans (RSF) version of RBR with NGP7. Other setups may work but are not supported. AI tools were used in its development.

## Features

- **Engine:** vibration that follows rpm, throttle and the rev limit.
- **Road:** vibration that follows the surface under each tyre (gravel, snow, ice, grass, tarmac, cobbles) together with speed, wheel slip and how rough the tyres measure.
- **Braking:** front-end judder that gets harsh on lock-up.
- **Landings:** jumps make RBR's own steering force pull harder for a moment. Bumps and rough ground never do.
- **Optional dynamic surface audio:** tyre and ground sounds that follow each wheel (off by default).
- **Optional camera movement:** your head moves with the car in the cockpit view: leaning into braking and corners, nodding, bouncing over bumps, shaking on crashes and landings.
- **Optional windscreen effects:** rain, snow and mud on the glass, with wipers (off by default).
- An in-game menu (**Plugins > ForceFX**) with per-effect switches, settings and diagnostics.
- Optional per-car profiles.

## Requirements

- Windows and 32-bit RBR.
- RallySimFans RBR with NGP7 and UDP telemetry enabled.
- A DirectInput force-feedback wheel that RBR already uses (not needed for the camera).
- The camera builds also need the Microsoft Visual C++ 2015-2022 x86 Redistributable. Most gaming PCs already have it.

## Install

1. Download a release from the [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases) page:
   - `ForceFX-<version>-Full.zip` has dynamic audio and the camera.
   - `ForceFX-<version>-Audio.zip` has dynamic audio only.
   - `ForceFX-<version>-Camera.zip` has the camera only.
   - `ForceFX-<version>-Basic.zip` is force feedback only.

   Use one, not both. If you used the standalone `RBR_CameraFX.dll`, delete it first: both would hook the same game functions.
2. Copy the contents into RBR's `Plugins` folder and keep the DLL and `.ini` files together.
3. Turn on NGP UDP telemetry. The destination doesn't matter, so it can keep pointing at SimHub or similar. The camera reads the same telemetry, so it needs no extra endpoint.
4. Start RBR, open **Plugins > ForceFX**, pick your wheel and connect. Set `AutoConnect=1` in `ForceFX.ini` to connect automatically next time. The camera works without connecting a wheel.

## Settings

Settings live in `ForceFX.ini`, which describes every option. The most useful ones are also in the in-game menu.

- **Road gain** (`RoadMaxMagnitude`) is the one setting for how strong the road feel is. The surface and what the car is doing decide what it feels like.
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
2. Mix levels and the output device are in the `[Audio]` section of `ForceFX.ini`.

`audio/surfaces.md` lists the sound names ForceFX looks for.

## Camera movement

Camera builds only. It is on by default. Switch it with **Camera movement** in the menu or `EnableCamera` in `ForceFX.ini`; it changes at once.

- `Intensity` in the `[Camera]` section of `ForceFX.ini` is the main strength setting: 0.5 is very subtle, 1.0 natural, 1.5 to 2.0 pronounced. The other camera sections are described in the file.
- With RBR in focus, hold Ctrl and press **F9** to reload the camera settings from `ForceFX.ini` while you drive, **F10** to switch the camera on or off.
- It reads the same telemetry as the force feedback, so a wheel doesn't have to be connected. If telemetry comes only from ForceFX's own UDP port (`TelemetrySource=2`), the camera needs the wheel connected.
- It hooks fixed addresses in the RallySimFans `RichardBurnsRally_SSE.exe` and stays off in any other executable.
- It patches the Direct3D matrices the way openRBRVR does, so the game's own camera and wipers are untouched. With openRBRVR loaded, or if those hooks don't start, it switches to a simpler method that has no head roll and can stop the wipers.
- If a direction feels wrong, set `Test=1` and sit in the cockpit: the head runs scripted moves and `ForceFXCamera.log` (with `Log=1`) says what you should see. Flip the matching `Invert...` or `Sign...` value.

## Windscreen effects

Camera builds only. Off by default. Turn on **Windscreen effects** in the menu or `EnableWindscreen=1` in `ForceFX.ini`, then restart RBR: its Direct3D hooks are only installed at startup when it is on.

- Rain and snow follow the stage's weather, read from the game. Mud is thrown on rough, wet ground, on landings and on impacts.
- Wipers clear the glass. The plugin finds the car's real wipers after about 12 seconds of rain and follows them; until then it uses emulated blades.
- Drops, mud, haze, wipers and the game's own wet-windscreen effect have their own settings in the `[Windscreen]` section of `ForceFX.ini`. F8 (with Ctrl) switches it on and off while driving.
- It costs some frame rate. If that is a problem, lower `Micro`, `Drops` and `Mud`, or set `Refraction=0`.

## Troubleshooting

- **No telemetry:** check that NGP UDP telemetry is enabled. The ForceFX menu shows the packet count and source.
- **Wheel won't connect or has no effects:** make sure force feedback is on in RBR and that you picked the same wheel RBR uses. Check `ForceFX.log`.
- **Road doesn't react to the ground:** open the Diagnostics page. "looking for the tyres" means the tyre data wasn't found, usually because the RBR build is different. "unknown" means the material isn't recognised; assign it with `Map=`.
- **Too strong or busy:** lower the total output or the live level first, then the individual effects.
- **The camera doesn't move:** open the Diagnostics page and read the camera line, and look for lines starting with `Camera:` in `ForceFX.log`. "not the RallySimFans executable" means the game isn't `RichardBurnsRally_SSE.exe`. "telemetry not seen" means NGP isn't sending UDP telemetry. For more detail set `Log=1` in the `[Camera]` section of `ForceFX.ini`.

When reporting a problem, attach `ForceFX.log` (it is beside the DLL), plus `ForceFXCamera.log` for camera problems, and describe your NGP setup. Remove any personal information first.

## Build from source

32-bit Windows DLL, built with MSVC. From an x86 Visual Studio command prompt run `build.bat`. It builds four variants into `out\full`, `out\audio`, `out\camera` and `out\basic`. The audio module is behind the `FORCEFX_AUDIO` define and the camera behind `FORCEFX_CAMERA`. The camera variants use `/MD` because the bundled MinHook library is built for the dynamic runtime; the others use `/MT`. A version tag starting with `v` builds a release with all four archives.

The version shown on the in-game ForceFX pages and in `ForceFX.log` is the git tag the build was made from (the release workflow passes it to `build.bat` as `FORCEFX_VERSION`, for example `v0.6.0`). A build without a tag says `Dev build` and the date it was compiled.

Unit tests for the logic headers are in `tests` and `audio/tests`. Each file has its build line at the top.

## Credits

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK. The camera module uses [MinHook](https://github.com/TsudaKageyu/minhook) by Tsuda Kageyu (BSD 2-Clause, see `thirdparty/minhook/LICENSE.txt`) and the approach of openRBRVR for patching the Direct3D matrices. Provided as is, without warranty.
