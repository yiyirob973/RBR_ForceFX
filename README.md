# RBR_ForceFX

Extra force feedback for Richard Burns Rally. ForceFX reads NGP telemetry and adds engine, road, braking, water and landing detail to your wheel on top of RBR's own steering force. Optional modules add dynamic audio, camera movement and windscreen effects.

> **Use at your own risk.** ForceFX sends force feedback straight to your wheel. Output limits and gradual derating are built in, but they cannot guarantee that hardware will not be damaged. Start with low settings and stop if the wheel behaves unexpectedly.
>
> It is built and tested for the RallySimFans (RSF) version of RBR with NGP7. Other setups may work but are not supported. AI tools were used in its development.

## Features

- **Engine:** vibration that follows rpm, throttle and the rev limit.
- **Road:** vibration that follows the ground under each tyre, your speed, wheel slip and how rough the tyres measure.
- **Braking:** front-end judder, harsher on lock-up.
- **Landings:** after a real jump, RBR's own steering force pulls harder for a moment. Bumps never trigger it.
- **Water:** a thump and a tug when a tyre hits standing water, a rumble while you wade, heavier steering while the front tyres push through.
- **Light steering under hard acceleration:** the steering loses force in proportion to the weight the front tyres lose (never more than 60%).
- **Dynamic audio (optional, off by default):** tyre squeal, slides, rolling noise, stones, bumps, landings, water and crashes, following each wheel.
- **Camera movement (optional):** your head moves with the car in the cockpit view.
- **Windscreen effects (optional, alpha, off by default):** rain, snow and mud on the glass, with wipers.
- An in-game menu (**Plugins > ForceFX**) with per-effect switches, settings and diagnostics, and optional per-car profiles.

## Requirements

- Windows and 32-bit RBR with RallySimFans and NGP7, with NGP's UDP telemetry turned on.
- A DirectInput force feedback wheel that RBR already uses (the camera works without one).
- The camera packages need the Microsoft Visual C++ 2015-2022 x86 Redistributable. Most gaming PCs have it.

## Install

1. Download a package from [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases):
   - `ForceFX-<version>-Full.zip`: audio and camera
   - `ForceFX-<version>-Audio.zip`: audio only
   - `ForceFX-<version>-Camera.zip`: camera only
   - `ForceFX-<version>-Basic.zip`: force feedback only
2. Copy the contents into RBR's `Plugins` folder. Keep the DLL and the `.ini` files together.
3. Turn on NGP UDP telemetry. It can keep pointing at SimHub or similar; ForceFX needs no port of its own.
4. Start RBR, open **Plugins > ForceFX**, pick your wheel and connect. Set `AutoConnect=1` in `ForceFX.ini` to do it automatically next time.

## Settings

Everything is in `ForceFX.ini`, which describes each option, and the main ones are in the in-game menu.

- **Road gain** (`RoadMaxMagnitude`) is the one setting for road strength. The ground and the car's behaviour decide how it feels.
- **Total output** (`TotalMaxMagnitude`) caps all effects together. If the wheel feels too strong or busy, lower this first.
- **Live level:** Numpad **+** and **-** turn everything down or up (never above the ini values), Numpad **\*** mutes. Keys are configurable.
- **Car profiles** are in `ForceFXCars.ini`: engine and road strength per car, and `Drivetrain=RWD|FWD|AWD`. Cars without an entry count as RWD.
- **Road feel delivery** (`RoadViaRbr`): by default the road vibration is added to RBR's own force. Set it to 0 to play it as a separate effect.

## How the road feel works

ForceFX reads which material is under each tyre from RBR's memory (read-only, RSF `RichardBurnsRally_SSE.exe` only, and only while a stage is running) and combines it with the telemetry. Loose ground is grainy, soft ground is dull, ice is hard and sharp, cobbles hit in a repeating pattern. Unknown ground is treated as plain hard ground.

Cobbles and wooden bridges have no material of their own in the game. Cobbles are detected from the telemetry, and you can assign any material ID with `Map=` in the `[Surface]` section of `ForceFX.ini`. The Diagnostics page shows the material IDs under the tyres, and `ForceFX.log` records surface changes.

## Dynamic audio

Audio packages only. Switch on **Dynamic audio** in the menu, or set `EnableDynamicAudio=1`.

1. Run `ForceFXAudio\Copy-RbrSounds.bat` to copy matching sounds from your RBR `Audio` folder (see the short readme beside it). You can also put your own WAV files in `ForceFXAudio\sounds`.
2. Levels and the output device are in the `[Audio]` section of `ForceFX.ini`. `audio/surfaces.md` lists the sound names ForceFX looks for.

Water sounds the sounds folder lacks are made by ForceFX itself, and your own `water_*.wav` files replace them. Each crash is written to `ForceFX.log` with its peak g; `CrashMinG` sets how sensitive crash detection is.

## Camera movement

Camera packages only, on by default. Switch it in the menu or with `EnableCamera`; it changes at once.

- `Intensity` in `[Camera]` is the main strength: 0.5 subtle, 1.0 natural, 1.5 to 2.0 pronounced.
- With RBR in focus, **Ctrl+F9** reloads the camera settings and **Ctrl+F10** switches the camera on or off.
- It reads the same telemetry as the force feedback, so no wheel is needed. If telemetry only comes from ForceFX's own UDP port (`TelemetrySource=2`), the wheel must be connected to supply it.
- It hooks fixed addresses in the RSF executable and stays off in any other. It works like openRBRVR's matrix patch, so the game's own camera and wipers are untouched. With openRBRVR loaded, or if those hooks don't start, it uses a simpler method.
- If a direction feels wrong, set `Test=1` in `[Camera]` and sit in the cockpit: the head runs scripted moves and `ForceFXCamera.log` (with `Log=1`) says what you should see. Flip the matching `Invert...` or `Sign...` setting until they match.

## Windscreen effects (alpha)

Camera packages only. Experimental, may affect performance or stability. Enable it with **Windscreen effects** in the menu or `EnableWindscreen=1`, then restart RBR.

- Rain and snow follow the stage's weather. Mud is thrown on rough, wet ground, landings and impacts.
- Wipers push the water aside rather than erasing it. Heavy rain also runs down the glass in streams.
- Dirt follows the surface scan: dust on dry gravel, mud on wet dirt, spray on wet tarmac, slush on snow. Mud stays on the glass between the stages of a rally and is cleaned for a new car or driver.
- All options are in `[Windscreen]`. **Ctrl+F8** switches it on and off while driving. If it costs too much frame rate, lower `Micro`, `Drops` and `Mud`, or set `Refraction=0`.

## Troubleshooting

- **No telemetry:** check that NGP UDP telemetry is on. The ForceFX menu shows the packet count and source.
- **Wheel won't connect, or no effects:** make sure force feedback is on in RBR and that you picked the wheel RBR uses. Check `ForceFX.log`.
- **Road ignores the ground:** open the Diagnostics page. "looking for the tyres" means the tyre data wasn't found, usually because the RBR build differs. "unknown" means the material isn't recognised.
- **Too strong or busy:** lower the total output or the live level first.
- **Camera doesn't move:** read the camera line on the Diagnostics page and the `Camera:` lines in `ForceFX.log`. "not the RallySimFans executable" means the game isn't running the supported executable.

When reporting a problem, attach `ForceFX.log` (beside the DLL), plus `ForceFXCamera.log` for camera or windscreen problems, and describe your NGP setup. Remove paths, usernames and other personal information first.

## Build from source

32-bit Windows DLL, built with MSVC. From an x86 Visual Studio command prompt run `build.bat`. It builds `out\full`, `out\audio`, `out\camera` and `out\basic`.

The version shown in the game and in `ForceFX.log` is the git tag the build was made from (the release workflow passes it to `build.bat` as `FORCEFX_VERSION`, for example `v0.7.0`). A plain `build.bat` run shows `dev`.

Unit tests for the logic headers are in `tests` and `audio/tests`; each file has its build line at the top.

## Credits

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK. The camera module uses [MinHook](https://github.com/TsudaKageyu/minhook) by Tsuda Kageyu (BSD 2-Clause, see `thirdparty/minhook/LICENSE.txt`).
