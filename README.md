# RBR_ForceFX

Extra force feedback for Richard Burns Rally. ForceFX reads NGP telemetry and adds engine, road, braking, water and landing detail to your wheel on top of RBR's own steering force. Dynamic audio, camera movement and windscreen effects are built in but switched off until you turn them on.

> **Use at your own risk.** ForceFX sends force feedback straight to your wheel. Output limits and gradual derating are built in, but they cannot guarantee that hardware will not be damaged. Start with low settings and stop if the wheel behaves unexpectedly.
>
> It is built and tested for the RallySimFans (RSF) version of RBR with NGP7. Other setups may work but are not supported. AI tools were used in its development.

## Features

- **Engine:** vibration that follows rpm, throttle and the rev limit.
- **Road:** vibration that follows the ground under each tyre, your speed, wheel slip and how rough the tyres measure.
- **Braking:** front-end judder that grows with the pedal and the weight on the front tyres, a coarser scrub when the front wheels lock (its pitch follows the ground), and RBR's own steering force going light while they are locked. It rides on RBR's force like the road feel. Heavier steering in hard braking is available but off, since RBR already does that (`BrakeHeaviness`).
- **Crashes:** when the car hits a tree, a rock or a wall, the wheel is yanked for a moment (RBR's own force, boosted). No effect of its own.
- **Landings:** after a real jump, RBR's own steering force pulls harder for a moment. Bumps never trigger it.
- **Water:** a thump and a tug when a tyre hits standing water, a rumble while you wade, heavier steering while the front tyres push through.
- **Light steering under hard acceleration:** the steering loses force in proportion to the weight the front tyres lose (never more than 60%).
- **Sideways weight:** the steering gets a little heavier as the weight builds up on the outside tyres in a corner, and swells while the weight swings across the car in a direction change. It only scales RBR's own force.
- **Dynamic audio (off by default):** tyre squeal, slides, rolling noise, stones, bumps, landings, water and crashes, following each wheel.
- **Camera movement (off by default):** your head moves with the car in the cockpit view.
- **Windscreen effects (alpha, off by default):** rain, snow and mud on the glass, with wipers.
- An in-game menu (**Plugins > ForceFX**) with per-effect switches, settings and diagnostics, and optional per-car profiles.

## Requirements

- Windows and 32-bit RBR with RallySimFans and NGP7, with NGP's UDP telemetry turned on.
- A DirectInput force feedback wheel that RBR already uses (the camera works without one).
- The Microsoft Visual C++ 2015-2022 x86 Redistributable (most gaming PCs have it).

## Install

1. Download `ForceFX-<version>.zip` from [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases).
2. Copy the contents into RBR's `Plugins` folder. Keep the DLL and the `.ini` files together.
3. Turn on NGP UDP telemetry. It can keep pointing at SimHub or similar; ForceFX needs no port of its own.
4. Start RBR, open **Plugins > ForceFX**, pick your wheel and connect. Set `AutoConnect=1` in `ForceFX.ini` to do it automatically next time.

## Settings

Everything is in `ForceFX.ini`, which describes each option, and the main ones are in the in-game menu.

- **Road gain** (`RoadMaxMagnitude`) is the one setting for road strength. The ground and the car's behaviour decide how it feels.
- **Total output** (`TotalMaxMagnitude`) caps all effects together. If the wheel feels too strong or busy, lower this first.
- **Live level:** Numpad **+** and **-** turn everything down or up (never above the ini values), Numpad **\*** mutes. Keys are configurable.
- **Car profiles** are in `ForceFXCars.ini`: engine and road strength per car, and `Drivetrain=RWD|FWD|AWD`. Cars without an entry count as RWD.
- **How the road and brake feel reach the wheel:** they are added to RBR's own force, so the wheel gets no extra effect. Without RBR's force updates (menus, test mode) they play as effects of their own.
- **Limits:** landing, water, light steering, brake lock-up and crash all scale RBR's force. Together they never go past `MaxForceMultiplier` (3 by default), and the multiplier cannot jump in one step.

## How the road feel works

ForceFX reads which material is under each tyre from RBR's memory (read-only, RSF `RichardBurnsRally_SSE.exe` only, and only while a stage is running) and combines it with the telemetry. Loose ground is grainy, soft ground is dull, ice is hard and sharp. Unknown ground is treated as plain hard ground.

You can assign any material ID to a surface type with `Map=` in the `[Surface]` section of `ForceFX.ini`. The Diagnostics page shows the material IDs under the tyres, and `ForceFX.log` records surface changes.

## Dynamic audio

Switch on **Dynamic audio** in the menu, or set `EnableDynamicAudio=1`.

1. Run `ForceFXAudio\Copy-RbrSounds.bat` to copy matching sounds from your RBR `Audio` folder (see the short readme beside it). You can also put your own WAV files in `ForceFXAudio\sounds`.
2. Levels and the output device are in the `[Audio]` section of `ForceFX.ini`. `audio/surfaces.md` lists the sound names ForceFX looks for.

Water sounds the sounds folder lacks are made by ForceFX itself, and your own `water_*.wav` files replace them. Each crash is written to `ForceFX.log` with its peak g; `CrashMinG` sets how sensitive crash detection is.

## Camera movement

Off by default. Switch it on in the menu or with `EnableCamera=1`; it changes at once.

- `Intensity` in `[Camera]` is the main strength: 0.5 subtle, 1.0 natural, 1.5 to 2.0 pronounced.
- With RBR in focus, **Ctrl+F9** reloads the camera settings and **Ctrl+F10** switches the camera on or off.
- It reads the same telemetry as the force feedback, so no wheel is needed. If telemetry only comes from ForceFX's own UDP port (`TelemetrySource=2`), the wheel must be connected to supply it.
- It hooks fixed addresses in the RSF executable and stays off in any other. It works like openRBRVR's matrix patch, so the game's own camera and wipers are untouched. With openRBRVR loaded, or if those hooks don't start, it uses a simpler method.
- If a direction feels wrong, set `Test=1` in `[Camera]` and sit in the cockpit: the head runs scripted moves and `ForceFXCamera.log` (with `Log=1`) says what you should see. Flip the matching `Invert...` or `Sign...` setting until they match.

## Windscreen effects (alpha)

Experimental, may affect performance or stability. Enable it with **Windscreen effects** in the menu or `EnableWindscreen=1`, then restart RBR.

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

32-bit Windows DLL, built with MSVC. From an x86 Visual Studio command prompt run `build.bat`; it writes `out\RBR_ForceFX.dll`.

The version shown in the game and in `ForceFX.log` is the git tag the build was made from (the release workflow passes it to `build.bat` as `FORCEFX_VERSION`, for example `v0.7.0`). A plain `build.bat` run shows "Dev build" and the date it was compiled.

Unit tests for the logic headers are in `tests` and `audio/tests`; each file has its build line at the top. The release workflow runs them (with g++) before it builds anything; `camera_packet_test` needs the Windows headers and runs only on a Windows machine.

**Surface for road and audio.** The tyre reader decides what each tyre is on (the game's material id, its class with `Map=` corrections, wetness and how loose it is) and hands the same answer to the road feel and the audio. `ForceFXAudio\materials.csv` corrects both. Damp and wet ground change the tyre sounds from the dry recordings: duller, a little quieter loose-ground loops, a spray hiss that grows with speed and fewer stones (`[Audio]` `Wet*` settings, `AudioWetLevel` in the menu). A real recording named `..._wet_loop` / `..._damp_loop` is used as it is and gets none of that.

**Inside the car.** The plugin reads the game's camera. In the cockpit view its sounds are heard through the cabin (muffled, boomy, with short reflections) and the interior makes its own noise, made in code: the heater fan, wind round the glass, panels buzzing with the engine, and rattles over bumps and landings. In an outside view none of it applies. It is automatic, not a setting; `AudioCabinLevel` in the menu sets how loud the interior noises are. `ForceFX.log` records which camera type counts as inside or outside whenever you change view (types 3 to 6 are assumed to be inside; `CabinCameraMin` / `CabinCameraMax` in `[Audio]` change that).

All telemetry field offsets are the `Offset*` keys in `[ForceFX]` of `ForceFX.ini` (`telemetry_layout.hpp`): the force feedback, the audio and the camera read the packet through the same table. `ForceFX.log` starts fresh each run; the previous one is kept as `ForceFX.old.log`.

## Credits

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK. The camera module uses [MinHook](https://github.com/TsudaKageyu/minhook) by Tsuda Kageyu (BSD 2-Clause, see `thirdparty/minhook/LICENSE.txt`).
