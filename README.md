# RBR_ForceFX

> **Disclaimer**
>
> - AI tools were used during the development of this project.
> - This project was built for the RallySimFans (RSF) version of Richard Burns Rally (RBR) and is intended for the RSF/NGP7 setup.
> - It may work with other versions of RBR or other setups, but compatibility is not guaranteed.
> - Testing has been completed and every effort has been made to reduce the risk of hardware damage.
> - However, because ForceFX sends force-feedback output directly to the wheel, hardware damage is still possible.
> - Use this software at your own risk.

ForceFX is a force-feedback plugin for Richard Burns Rally (RBR). It uses NGP telemetry to add configurable vibration and impact detail to a compatible DirectInput force-feedback wheel while RBR is running.

## Features

- Configurable engine, road/suspension, braking, and landing feedback.
- Surface feel: the plugin reads which ground is under the tyres and gives gravel, snow, ice and grass their own road-vibration character (sharp grain on gravel, soft and slow on snow, hard cracks on ice).
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

1. Download a release archive from the repository's [Releases](https://github.com/yiyirob973/RBR_ForceFX/releases) page. There are two: `ForceFX-<version>.zip` includes the optional dynamic surface audio, and `ForceFX-<version>-NoAudio.zip` is force feedback only. Use one or the other, not both.
2. Copy the files from the archive into RBR's `Plugins` folder. Keep the DLL and its configuration files together.
3. Enable NGP UDP telemetry in your RBR/NGP setup. The destination does not matter: ForceFX picks the packets up inside the RBR process as NGP sends them, so you can leave NGP pointed at SimHub or another telemetry application.
4. Start RBR and open **Plugins > ForceFX**. Select the wheel RBR is using and connect. Enable `AutoConnect` in `ForceFX.ini` if you want ForceFX to connect automatically to the saved wheel on startup.
5. Begin with the shipped settings and increase feedback gradually. If telemetry is not received, check that NGP UDP telemetry is enabled and use the troubleshooting notes below.

ForceFX settings are stored in `ForceFX.ini`. The comments in that file describe the available settings and ranges; the in-game menu provides controls for commonly adjusted settings. Advanced telemetry settings may require configuration for non-RSF setups.

`ForceFXCars.ini` contains optional per-car profiles. Profiles can adjust selected values and fall back to the global settings for unspecified values. You can edit or remove profiles, or disable per-car profiles in the configuration.

## Road feel

The road feel is not a list of "gravel feels like this, tarmac like that". ForceFX reads the contact data RBR keeps for each tyre (read-only; it never writes to the game) and takes the name of the material under it, for example `Gravel_Coarse_Deep_Wet`. From the name it works out how hard the ground is, how much loose material lies on it, how coarse it is and how slippery it is. The road vibration then follows from those together with what the car is doing, so the same ground feels different depending on how you drive on it:

- **Strength** follows how rough the tyres actually measure, from the strut forces and damper speed in the telemetry.
- **Loose ground** (gravel, dirt, snow) is grainy, with gaps and the odd flung stone. How much of that you feel depends on speed and wheel slip: a gentle roll is calm, a slide or wheelspin is violent. Standing still, nothing is thrown.
- **Soft ground** (deep gravel, snow, grass) absorbs and dulls the vibration, with a lower pitch and a slower rise and fall.
- **Hard, slippery ground** (snow on ice, wet tarmac) is quiet between hits and cracks sharply. Wet tarmac is only slightly different from dry.
- **Cobblestones and other ridged ground** are recognised from the telemetry: hard ground whose tyres shake in a regular pattern gets strong, crisp hits in time with your speed (one per ridge). No material id is needed.
- All four tyres are read separately, so a car with two wheels on grass and two on tarmac feels the difference on each side. Each side mixes its front and rear tyre, and the ground fades in over a fraction of a second.

**The one setting is the road gain** (`RoadMaxMagnitude`, or `Road gain` in **Plugins > ForceFX**). `Effect: Surface feel` (`EnableSurfaceFeel`) switches the ground-aware part off, so the road feels the same on every surface. Unknown ground is treated as plain hard ground. The old per-surface values, `SurfaceStrength`, `RoadGrain` and `RoadDirectional` are gone; they are ignored if they are still in your `ForceFX.ini`.

For people who want to dig further, the `[Surface]` section of `ForceFX.ini` has the tyre order, the front/rear weighting, how often the ground is read, the cobble detection thresholds (`CobbleDetect`, `CobbleDetectLow`, `CobbleDetectHigh`) and `Map=`. `Map=N:type` replaces the ground of a single material id with a typical ground of that type (tarmac, gravel, snow, ice, soft, cobble, wood or none), which also covers an id the game's material table doesn't have. Edits there take effect when RBR starts. To help tune it from your own driving, set `SurfaceProbe=1` and send `ForceFX_probe.csv`.

- The ground is read only while a stage is running. Finding the tyre data takes a short scan of the game's memory at the start of each stage, in the background.
- The Diagnostics page shows the detected surface and the material ids under the four tyres. `ForceFX.log` records each surface change.
- It relies on the memory layout of the RallySimFans `RichardBurnsRally_SSE.exe`. On a build with a different layout the plugin simply finds no tyre data, the ground counts as plain hard ground, and the road feel still follows the telemetry.

## How wheel slip is measured

NGP's telemetry has no slip value. Each wheel only reports a segment counter that steps once per eighth of a turn, so ForceFX counts the turns over a window of at least four revolutions (about a quarter of a second at speed) and compares them with the car's real speed over the ground (the velocity vector, not the speedometer, which follows the driven wheels). Every reading carries the error of the count, and slip only counts beyond it, so a wheel that is simply rolling is never read as slipping. A shorter window finds lock-up and wheelspin quickly. NGP's step counter (`OffsetSteps`) is used as the clock when it behaves. Road feel, front-wheel lock-up and the audio's squeal and slide all use this. The tyre size is learned while coasting, so the first few seconds of a stage use a cautious guess. `tests/wheel_slip_test.cpp` and `audio/tests/slip_test.cpp` cover it.

## Dynamic surface audio

The dynamic audio is an optional module that is only in the audio build (`ForceFX-<version>.zip`). In the `-NoAudio` build it is left out of the DLL entirely: no audio menu entries, no audio settings and no sound code.

ForceFX can also play tyre and ground sounds that follow each wheel: rolling noise, squeal, sliding, flying stones, bumps and landings, with the sound of whatever surface each tyre is on. It is **off by default**. Turn it on with the first switch in **Plugins > ForceFX** (`Dynamic audio`) or with `EnableDynamicAudio=1` in `ForceFX.ini`.

- It uses the same telemetry and the same ground detection as the surface feel, so there is no extra UDP port to set up. The ground is read per tyre; when the exact material isn't known it falls back to what the telemetry suggests.
- To fill the sounds folder from your RBR installation, run `Plugins\ForceFXAudio\Copy-RbrSounds.bat` (it copies the matching sounds from RBR's own Audio folder; see the short readme beside it). You can also put your own sounds in `Plugins\ForceFXAudio\sounds` (WAV files, any subfolders). Names follow the surface, for example `gravel_loop`, `gravel_squeal`, `gravel_slide`, `gravel_hit`, plus `debris_stone`, `debris_impact`, `debris_landing` and `debris_rumble`. A plain pack of `tarmac`, `gravel`, `grass` and `snow` covers every surface; add finer names (`gravel_wet`, `ice`, `dirt`...) to make single surfaces distinct. `audio/surfaces.md` in the repository lists the exact search order for every material.
- Mix levels, the cabin sound and the output device are in `ForceFXAudio.ini`, read each time the audio is switched on.
- Sounds load in the background when you switch it on, so the wheel never waits. Switching it off frees everything again.
- The audio runs from ForceFX's telemetry loop, which starts when ForceFX is connected to a wheel.
- **Stones and debris come from the ground and the tyres, not from a setting.** The game's own material says how much loose material there is (clean tarmac: none, tarmac with gravel sprinkled or spread on it: some, gravel, dirt, rough ground, snow over gravel: plenty), and the telemetry says how hard each tyre works it (surface speed, slip, throttle). Clean tarmac throws nothing whatever the car does; with no game surface the telemetry decides alone. `StoneLevel` (and the menu's debris level) only set the volume.
- **Suspension sounds are rare events, not texture.** A creak needs real weight transfer (the tyre's load well away from normal) together with a slow stroke of a few centimetres, and the car makes one at most every 1.5 s. A bottom-out thump needs the load to jump to about three times normal in one fast compression, at most one at a time, and it replaces the plain impact sound. Normal driving, small bumps and rough ground with fast strokes make neither. They have no rate setting; `SqueakLevel`, `BottomOutLevel` and the menu's suspension level set the volume.
- **A sound that sticks:** the audio now goes quiet half a second after the last real telemetry frame, not after the last packet of any kind. If something still sticks, set `DebugLog = 1` in `ForceFXAudio.ini`: ForceFX.log gets a line a second with the speed, each tyre's slip and the loop levels, which shows what the sound is being told.

## Safety

- ForceFX is developed for RallySimFans/NGP7 setups.
- Other versions or setups may work, but compatibility is not guaranteed.
- ForceFX adds force feedback to a device already used by RBR.
- Its settings include output ceilings, frequency bounds, and gradual derating for sustained output; the plugin also fades its effects when appropriate.
- These measures are intended to reduce risk, but they cannot guarantee that damage will not occur.
- Start with conservative settings, increase feedback gradually, and stop using the plugin if the wheel behaves unexpectedly.
- Because feedback is sent directly to your wheel, use ForceFX at your own risk.

## Troubleshooting

- **No telemetry is received:** Check that NGP UDP telemetry is enabled, then open the ForceFX menu. The telemetry line shows the packet count and where packets come from (NGP or the UDP port), and diagnostic information.
- **The wheel does not connect or effects are unavailable:** Confirm force feedback is enabled in RBR and select the same DirectInput wheel that RBR uses. Check `ForceFX.log` for connection or device errors.
- **Telemetry arrives but feedback is incorrect:** The telemetry layout may differ from the configured offsets or scaling. Review the advanced settings in `ForceFX.ini` and use the diagnostics page to inspect the received values.
- **The road feel doesn't react to the ground:** Open the Diagnostics page. `Surface: looking for the tyres` means the tyre data was not found (check `ForceFX.log` for `Surface:` lines; a different RBR build can have a different layout). `Surface: unknown` means the ground under the car is not one the plugin recognises, so it counts as plain hard ground; the Diagnostics page shows the material ids, and `Map=` in `[Surface]` can assign them.
- **Feedback is too strong or busy:** Lower the combined output ceiling or live level first, then adjust individual effect levels. Start conservatively and test one effect at a time.

The plugin writes `ForceFX.log` and, after receiving telemetry, `ForceFX_packet.txt` beside the DLL. Include relevant log details and your NGP/telemetry setup when reporting a problem; remove any personal information before sharing logs.

## Build from source

The audio is a build-time module. Everything audio-related sits behind one define, `FORCEFX_AUDIO`, and `audio_module.hpp` is the only place the plugin touches it. Without the define the plugin compiles against an empty stand-in, so the audio code and `winmm.lib` are not part of the DLL.

The build workflow compiles both variants as 32-bit DLLs on Windows with MSVC and packages each with its configuration files:

- `ForceFX-<version>.zip`: built with `/DFORCEFX_AUDIO`. Includes `ForceFXAudio.ini` and the `ForceFXAudio` folder with the sound-copy script.
- `ForceFX-<version>-NoAudio.zip`: built without it. Force feedback and surface feel only.

To build both locally, open an x86 Visual Studio developer command prompt and run `build.bat`. The DLLs land in `out\audio` and `out\noaudio`. To build one by hand, use the `cl` line from `build.bat` and add or leave out `/DFORCEFX_AUDIO` (and `winmm.lib`).

Pushing a version tag beginning with `v` creates a release with both archives. Pushing to the `audio` branch builds both and attaches them to the workflow run as a downloadable artifact.

## Credits and warranty

`IRBRGame.h` and `IPlugin.h` are plugin interface headers from the RBR SDK.

This project is provided as is, without warranty. Use it at your own risk.
