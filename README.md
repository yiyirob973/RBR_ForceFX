# RBR_ForceFX

Extra force feedback for Richard Burns Rally. ForceFX reads NGP telemetry and adds engine, road, braking and landing detail to your wheel on top of RBR's own steering force. Optional modules add dynamic surface audio, camera movement and windscreen effects.

> **Use at your own risk.** ForceFX sends force feedback straight to your wheel. Output limits and gradual derating are built in, but they cannot guarantee that hardware will not be damaged. Start with low settings and stop immediately if the wheel behaves unexpectedly.
>
> It is built and tested for the RallySimFans (RSF) version of RBR with NGP7. Other setups may work but are not supported. AI tools were used in its development.

## Features

- **Engine:** vibration that follows rpm, throttle and the rev limit.
- **Road:** vibration that follows the surface under each tyre (gravel, snow, ice, grass, tarmac, cobbles) together with speed, wheel slip and how rough the tyres measure.
- **Braking:** front-end judder that gets harsh on lock-up.
- **Landings:** jumps make RBR's own steering force pull harder for a moment. Bumps and rough ground never do.
- **Water:** a thump and a tug on the wheel when a tyre hits standing water (harder the faster you go), a churning rumble while you wade, and a smaller kick on the way out. With dynamic audio on you also hear the splash, and with the windscreen effects on the glass takes a splash too. The steering gets heavy while the front tyres push through water (adjustable, `WaterHeaviness`).
- **Light steering from the front load:** when the front tyres carry less weight than they do standing still (hard acceleration, an uphill), RBR's steering force is scaled down in proportion, measured from the strut forces and working from the first metre of a launch (switch `EnableAccelLight`, gain `AccelLightness`, never more than 60% off).
- **Optional dynamic surface audio:** tyre and ground sounds that follow each wheel (off by default).
- **Optional camera movement:** your head moves with the car in the cockpit view: leaning into braking and corners, nodding, bouncing over bumps, shaking on crashes and landings.
- **Alpha feature — optional windscreen effects:** rain, snow and mud on the glass, with wipers (disabled by default).
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

   Choose the package that provides the features you need. If you used the standalone `RBR_CameraFX.dll`, delete it first: both would hook the same game functions.
2. Copy the contents into RBR's `Plugins` folder and keep the DLL and `.ini` files together.
3. Turn on NGP UDP telemetry. The destination doesn't matter, so it can keep pointing at SimHub or similar. The camera reads the same telemetry, so it needs no extra endpoint.
4. Start RBR, open **Plugins > ForceFX**, pick your wheel and connect. Set `AutoConnect=1` in `ForceFX.ini` to connect automatically next time. The camera works without connecting a wheel.

## Settings

Settings live in `ForceFX.ini`, which describes every option. The most useful ones are also in the in-game menu.

- **Road gain** (`RoadMaxMagnitude`) is the one setting for how strong the road feel is. The surface and what the car is doing decide what it feels like.
- **Road feel delivery** (`RoadViaRbr`, also in the menu): by default the road vibration is added to RBR's own steering force, so the wheel gets no extra road effect. Set it to 0 to play the road through its own DirectInput effect.
- **Total output** (`TotalMaxMagnitude`) caps all effects together. If the wheel feels too strong or busy, lower this first.
- **Live level** scales everything down while you drive: Numpad **+** and **-** change it, Numpad **\*** mutes the added effects (keys are configurable in `ForceFX.ini`). It can only turn effects down; it never raises the settings in the file.
- **Per-car profiles** are in `ForceFXCars.ini`. A profile can change a car's engine and road strength and set its `Drivetrain=RWD|FWD|AWD`. Cars without an entry are treated as RWD. The drivetrain changes engine vibration on the driven wheels.

## Road feel

ForceFX reads which material is under each tyre from RBR's memory (read-only) and combines it with the telemetry. Loose ground is grainy, soft ground is dull, ice is hard and sharp, and cobbles hit in a repeating pattern.

- If a material isn't recognised, it is treated as plain hard ground and the road still follows the telemetry.
- Cobblestones and wooden bridges have no material of their own in the game. Cobbles are detected from the telemetry, and any material ID can be assigned with `Map=` in the `[Surface]` section of `ForceFX.ini`.
- The ground is only read while a stage is running. It relies on the memory layout of the RSF `RichardBurnsRally_SSE.exe`.
- The Diagnostics page shows the detected surface and the material IDs under the tyres. `ForceFX.log` records surface changes.

## Dynamic surface audio

Audio build only. It is off by default: switch on **Dynamic audio** in the menu, or set `EnableDynamicAudio=1`.

1. Run `ForceFXAudio\Copy-RbrSounds.bat` to copy matching sounds from your RBR `Audio` folder. See the short readme beside it. You can also put your own WAV files in `ForceFXAudio\sounds`.
2. Mix levels and the output device are in the `[Audio]` section of `ForceFX.ini`.

`audio/surfaces.md` lists the sound names ForceFX looks for. For water it uses `water_splash` (a tyre hits standing water), `water_loop` (rolling through it) and `water_hit` (flung stones). Any of these your sounds folder lacks, ForceFX makes itself, so a ford never sounds like gravel; your own `water_*.wav` files replace them.
Crashes (a tree, a big rock, a wall) get their own sounds too: one thump by how hard the hit was, with crumpling metal, twigs on nose-first hits and glass on the worst, from RBR's `Impact` folders (the copy script handles them). `CrashMinG` in the `[Audio]` section sets how sensitive it is, and each crash is written to `ForceFX.log` with its peak g.

## Camera movement

Camera builds only. It is on by default. Switch it with **Camera movement** in the menu or `EnableCamera` in `ForceFX.ini`; it changes at once.

- `Intensity` in the `[Camera]` section of `ForceFX.ini` is the main strength setting: 0.5 is very subtle, 1.0 natural, 1.5 to 2.0 pronounced. The other camera sections are described in the file.
- With RBR in focus, hold Ctrl and press **F9** to reload the camera settings from `ForceFX.ini` while you drive, **F10** to switch the camera on or off.
- It reads the same telemetry as the force feedback, so a wheel doesn't have to be connected. If telemetry comes only from ForceFX's own UDP port (`TelemetrySource=2`), the camera needs the wheel connection to supply the telemetry.
- It hooks fixed addresses in the RallySimFans `RichardBurnsRally_SSE.exe` and stays off in any other executable.
- It patches the Direct3D matrices the way openRBRVR does, so the game's own camera and wipers are untouched. With openRBRVR loaded, or if those hooks don't start, it switches to a simpler method that does not patch the matrices.
- If a direction feels wrong, set `Test=1` and sit in the cockpit: the head runs scripted moves and `ForceFXCamera.log` (with `Log=1`) says what you should see. Flip the matching `Invert...` or `Sign...` settings until the moves match.

## Windscreen effects — alpha

> **Alpha feature:** Windscreen FX is experimental, may affect performance or stability, and is disabled by default.

Camera builds only. Enable it in the menu with **Windscreen effects** or set `EnableWindscreen=1` in `ForceFX.ini`, then restart RBR. Its Direct3D hooks are only installed at startup while it is enabled.

- Rain and snow follow the stage's weather, read from the game. Mud is thrown on rough, wet ground, on landings and on impacts.
- Wipers move the water, they do not erase it: each blade pushes the beads and the film in front of it (the beads it meets run together into bigger ones), leaves a thin wet film and a few small beads behind, and what it pushes past the edge of the glass is gone. In heavy rain a thick film also sags and runs down the glass in streams. The plugin finds the car's real wipers after about 12 seconds of rain and follows them; until then it uses emulated blades.
- Drops, mud, haze, wipers and the game's own wet-windscreen effect have their own settings in the `[Windscreen]` section of `ForceFX.ini`. Hold Ctrl and press F8 to switch it on and off while driving.
- With `Ground=1` (default) the dirt follows the surface scan: pale dust on dry gravel, dark mud on wet dirt, grey road spray on wet tarmac, slush on snow, nothing on clean tarmac. `CarryOver` keeps mud on the glass between the stages of one rally (cleaned for a new car, driver, retired stage or after `CarryMinutes`). `GrimeLife` makes dirt in places the wipers cannot reach dry off instead of building up. The wipers sweep the area between the blade's real positions on the glass; `WiperMeasure=1` reads the blade's length and direction from its mesh.
- It costs some frame rate. If that is a problem, lower `Micro`, `Drops` and `Mud`, or set `Refraction=0`.

## Troubleshooting

- **No telemetry:** check that NGP UDP telemetry is enabled. The ForceFX menu shows the packet count and source.
- **Wheel won't connect or has no effects:** make sure force feedback is on in RBR and that you picked the same wheel RBR uses. Check `ForceFX.log`.
- **Road doesn't react to the ground:** open the Diagnostics page. "looking for the tyres" means the tyre data wasn't found, usually because the RBR build is different. "unknown" means the material isn't recognised.
- **Too strong or busy:** lower the total output or the live level first, then the individual effects.
- **The camera doesn't move:** open the Diagnostics page and read the camera line, and look for lines starting with `Camera:` in `ForceFX.log`. "not the RallySimFans executable" means the game isn't being launched as the supported executable.

When reporting a problem, attach `ForceFX.log` (it is beside the DLL), plus `ForceFXCamera.log` for camera or windscreen problems, and describe your NGP setup. Review logs before sharing and remove paths, usernames or other personal information.

## Build from source

32-bit Windows DLL, built with MSVC. From an x86 Visual Studio command prompt run `build.bat`. It builds four variants into `out\full`, `out\audio`, `out\camera` and `out\basic`. The audio module is built with CMake/MinGW in `audio/native`.

The version shown on the in-game ForceFX pages and in `ForceFX.log` is the git tag the build was made from (the release workflow passes it to `build.bat` as `FORCEFX_VERSION`, for example `v0.6.0`). A plain `build.bat` run has no tag and shows `dev`.

Unit tests for the logic headers are in `tests` and `audio/tests`. Each file has its build line at the top.

## Credits

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK. The camera module uses [MinHook](https://github.com/TsudaKageyu/minhook) by Tsuda Kageyu (BSD 2-Clause, see `thirdparty/minhook/LICENSE.txt`).