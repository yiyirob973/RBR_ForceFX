# RBR_ForceFX
RBR_ForceFX: plugin for RBR with NGP7 that enhances FFB with additional effects (formerly RBR_ffb_aux / RBR_TrueforcePlugin)


## v4: safety limits and live gain

- Hard ceilings in code: total output capped at 5000/10000, vibration frequencies 12-120 Hz, all gains 0..1. The `.ini` cannot exceed them.
- Slow thermal derate (`SustainedLimit`): sustained heavy output is gradually reduced.
- UDP telemetry listens on 127.0.0.1 only.
- Effects settle to zero before release; worker-thread exceptions are contained; the DLL pins itself if another tool hooked DirectInput on top of it.
- **Live gain while driving** (RBR must be the foreground window):
  - `Ctrl` + `Numpad +` / `Numpad -`: overall vibration level up / down in equal-loudness steps (x or / `GainRatio`, default 0.85; beep pitch = level)
  - `Ctrl` + `Numpad *`: mute / unmute
  - The level can only reduce from the configured ceilings and is saved to `UserGain` in `ForceFX.ini`.
  - Keys are Windows virtual-key codes in the `.ini` (`KeyGainUp`, `KeyGainDown`, `KeyMute`, `KeyModifier`, `GainRatio`, `GainBeep`); 0 disables a key.
- Existing `ForceFX.ini` files keep their old values (clamped to the new ceilings). Delete it once to get the new, lower defaults.
- Gain structure is now: per-layer `*MaxMagnitude` (the mix) -> `TotalMaxMagnitude` (fixed safety ceiling) -> `UserGain` (live level). `MasterGain` and the per-layer `*Gain` keys were removed because they duplicated these. The thermal derate measures the level actually delivered, including `UserGain`.

## v4.1: granular road, directional impacts, rougher high revs

- **Road is now two zero-mean sawtooth effects with opposite sharp edges**, each fed by one side's tyre roughness. An impact on the left tyres snaps one way, on the right tyres the other. Because both waveforms average to zero, nothing is added to RBR's own force. `RoadDirectional` (0..1) blends between identical sides and fully separate sides; `RoadSideInvert=1` swaps the polarity if left/right feel backwards on your wheel.
- **Granular road:** amplitude and pitch are re-randomised every ~12 ms per side, and the two sides are detuned so they never lock in phase. `RoadGrain` (0..1) sets how coarse it gets.
- **Rougher high revs:** random firing-strength variation, a little pitch jitter, and the half-order effect turns into a detuned second oscillator (x1.37) at high revs, so it beats against the main one. `EngineGrit` (0..1) sets how much.
- TEST mode phase 3 now alternates left-side and right-side hits every 1.5 s so you can check direction without driving.

## v5: weight / suspension load, per-car profiles, shipped default inis

- **Defaults** are now the tuned values (engine 1200, road 3000, brake 3000, total 4000, no hotkey modifier, auto-connect to the SIMAGIC Alpha EVO). `ForceFX.ini` and `ForceFXCars.ini` live in the repo as reference defaults to copy next to the DLL. The build does not use them, and the plugin writes `ForceFX.ini` itself on first run if it is missing. If you add a key in `SyncConfig`, add it to the repo ini by hand.
- **Weight / load model** (relative to each car's own static load, so units don't matter; static load is re-learned at stage start):
  - the more heavily loaded side transmits more road (`LoadInfluence`)
  - compression and landing thump per side (`ThumpGain`), stronger and longer for a heavier car (`WeightFeel`)
  - road and engine go quiet when the wheels leave the ground (`AirborneFade`)
  - braking judder grows with the load on the front tyres
- **Per-car profiles, matched by car name:** the plugin reads the car's name text the way RSF does (`rallysimfans.ini` `[cars] slot`, then `Cars\Cars.ini` `[Car0<slot>]` `CarName` / `RSFCarID`; vanilla installs fall back to the telemetry slot and the car's model file name). Each section in `ForceFXCars.ini` with a `Name=` is a profile; names are matched loosely (case, punctuation, `Grp`/`Group`, missing manufacturer words), but years, model numbers and roman numerals must agree. `Match=` adds aliases and `RSFCarID=` gives an exact match. Keys: `Preset` (inline4, inline4turbo, boxer4, inline5, v6, flat6, v8), `Cylinders`, `RpmMax`, `EngineScale`, `RoadScale`, `LumpScale`, `Grit`, `Unevenness`, `WeightFeel`. The file is re-read when it changes; the menu page and `ForceFX.log` show the detected name and the matched profile. A car change (or a new stage) resets the learned rev limit and load.

## v5.1: landing impacts, car switching fix, new defaults

- **Landings and hard hits** now have their own effect layer (`ImpactMaxMagnitude`, `ImpactGain`). A landing is detected from the raw strut load (not the smoothed one), the time the wheels were off the ground and the heave spike; the hit attacks instantly (square burst, 24-60 Hz, lower for bigger hits and heavier cars) and rings out over about 0.2 s. While it plays, the other layers duck so it is not diluted, and it stays inside `TotalMaxMagnitude` and the hard ceilings. It is symmetric (zero-mean), so it never pushes RBR's steering force. `ImpactGain` 0..2 sets sensitivity (0 = off). TEST mode phase 5 plays big and medium landing hits.
- **Car switching fixed:** the car identity used to be checked only on a few events, so a car changed in the menus kept the old profile. It is now polled about once a second (also when telemetry returns after a pause, on stage start and when the slot changes), and `ForceFXCars.ini` edits are picked up live. A momentarily unreadable `Cars.ini` keeps the current car instead of dropping its profile.
- Shipped `ForceFX.ini` / `ForceFXCars.ini` and the built-in defaults are now the tuned personal set.

## Note

This plugin enhances Richard Burns Rally's own force feedback; it never replaces or overrides it. All effects are zero-mean vibrations added to the wheel alongside RBR's steering force, the wheel's gain and autocenter are never touched, and RBR's force is unaffected when the plugin is muted or unloaded.

## v5.2: landings rewritten

- Landing detection no longer needs 0.1 s of full airtime. Severity now combines three cues with `max()` (peak strut load, how fast the total strut load jumped from its recent minimum, airtime + heave spike), so short hops, crest drops and bottoming out register too, and the severity is continuous: a small hop is a tap, a big jump is a full-strength hit.
- The hit curve is more concave (`^0.6`) and `ImpactMaxMagnitude` default is 5000 (the hard ceiling). The hit may use its own ceiling; the other layers duck to make room.
- Every detected landing / hit is written to `ForceFX.log` with its severity and the cues behind it. Existing `ForceFX.ini` files keep their old `ImpactMaxMagnitude`; set it to 5000 or delete the file.

## v6: renamed to RBR_ForceFX, in-game settings menu

- **Renamed everywhere:** `RBR_ForceFX.cpp`, `RBR_ForceFX.def`, `RBR_ForceFX.dll`, `ForceFX.ini` (section `[ForceFX]`), `ForceFXCars.ini`, `ForceFX.log`, `ForceFX_loaded.txt`, `ForceFX_packet.txt`. The plugin is listed as "ForceFX" in RBR's plugin menu. On first start, if `ForceFX.ini` / `ForceFXCars.ini` do not exist but the old `TrueforceRumble.ini` / `TrueforceCars.ini` do, your settings and car profiles are carried over automatically. Delete the old `RBR_TrueforcePlugin.dll` from `Plugins` so only one copy loads.
- **In-game settings menu** (RBR main menu > Plugins > ForceFX). Up/Down choose a row, Left/Right change it, Select acts on the Wheel / TEST / Diagnostics / Reset rows. Changes reach the wheel immediately (use the TEST row to hear them without driving) and are written to `ForceFX.ini` at once.
  - Gains: live level, total ceiling, engine, road, brake and landing gain, landing sensitivity, landing thump.
  - Frequencies: engine min/max Hz, road min/max Hz, brake pitch and landing pitch (multipliers, new keys `BrakeHzScale` and `ImpactHzScale`, default 1).
  - Every edit goes through the same clamps as the ini (hard ceilings, 12-120 Hz), so the menu cannot exceed the safety limits. Car profiles still scale engine and road gain on top of the menu values.
  - The old raw-packet view moved to the Diagnostics row.

- **Priority ducking:** landing/impact is the highest priority, road second, engine lowest. While a higher layer is active the lower ones fall back (road by the landing, engine by landing and road), so the strongest cue is the one you feel. `PriorityDuck` (menu: "Priority ducking", default 85%) sets how far they fall back, 0 = off. A brake judder is only pushed back by a landing.
- **More sensitive landings:** lower thresholds for the airtime, load-spike and no-airtime hit detectors, a stronger curve for small hops and a slightly longer ring-out.
- **Landing kick:** a landing used to be only a short symmetric buzz, which a direct-drive wheel hardly registers. It is now split into that buzz plus a one-direction constant-force pulse (a new "jolt" effect). `LandingJolt` (menu: "Landing kick", default 35%) is the share of the landing budget played as the kick; 0 = buzz only. The direction follows the more heavily loaded side (flipped by `RoadSideInvert`) and alternates when both sides are equal. Buzz and kick together never exceed `ImpactMaxMagnitude` or the hard ceiling. The buzz also got a higher, slightly jittered pitch, a 40% minimum strength and a longer ring-out. Lock-up is unchanged.
- **Menu text fix:** RBR's text drawing treats `%` as a format character, which garbled the live-gain line; every menu string now escapes it.
- **Landing buzz matched to the lock-up:** same sine waveform (square effects can be weak on some bases), lower and wider pitch (about 18-55 Hz, jittered), a 60% minimum strength, a longer ring-out (real landings and the TEST hits), and the kick share defaults to 35% so most of the budget is sustained buzz.
- **Fewer false landings:** the lowered detection thresholds were also firing on weight transfer from braking while cornering, so the original thresholds are back, the 60% minimum strength is gone and the ring-out is shorter. New menu row "Landing trigger level" (`LandingThreshold`, default x1.0, 0.5-3.0): raise it if ordinary braking/cornering still triggers a landing, lower it if real jumps are missed. It gates the no-airtime hit detector; real airtime landings are not affected.
