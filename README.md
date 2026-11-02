# RBR_ffb_aux
plugin for RBR with NGP7 that enhances FFB with additional effects


## v4: safety limits and live gain

- Hard ceilings in code: total output capped at 5000/10000, vibration frequencies 12-120 Hz, all gains 0..1. The `.ini` cannot exceed them.
- Slow thermal derate (`SustainedLimit`): sustained heavy output is gradually reduced.
- UDP telemetry listens on 127.0.0.1 only.
- Effects settle to zero before release; worker-thread exceptions are contained; the DLL pins itself if another tool hooked DirectInput on top of it.
- **Live gain while driving** (RBR must be the foreground window):
  - `Ctrl` + `Numpad +` / `Numpad -`: overall vibration level up / down in equal-loudness steps (x or / `GainRatio`, default 0.85; beep pitch = level)
  - `Ctrl` + `Numpad *`: mute / unmute
  - The level can only reduce from the configured ceilings and is saved to `UserGain` in `TrueforceRumble.ini`.
  - Keys are Windows virtual-key codes in the `.ini` (`KeyGainUp`, `KeyGainDown`, `KeyMute`, `KeyModifier`, `GainRatio`, `GainBeep`); 0 disables a key.
- Existing `TrueforceRumble.ini` files keep their old values (clamped to the new ceilings). Delete it once to get the new, lower defaults.
- Gain structure is now: per-layer `*MaxMagnitude` (the mix) -> `TotalMaxMagnitude` (fixed safety ceiling) -> `UserGain` (live level). `MasterGain` and the per-layer `*Gain` keys were removed because they duplicated these. The thermal derate measures the level actually delivered, including `UserGain`.

## v4.1: granular road, directional impacts, rougher high revs

- **Road is now two zero-mean sawtooth effects with opposite sharp edges**, each fed by one side's tyre roughness. An impact on the left tyres snaps one way, on the right tyres the other. Because both waveforms average to zero, nothing is added to RBR's own force. `RoadDirectional` (0..1) blends between identical sides and fully separate sides; `RoadSideInvert=1` swaps the polarity if left/right feel backwards on your wheel.
- **Granular road:** amplitude and pitch are re-randomised every ~12 ms per side, and the two sides are detuned so they never lock in phase. `RoadGrain` (0..1) sets how coarse it gets.
- **Rougher high revs:** random firing-strength variation, a little pitch jitter, and the half-order effect turns into a detuned second oscillator (x1.37) at high revs, so it beats against the main one. `EngineGrit` (0..1) sets how much.
- TEST mode phase 3 now alternates left-side and right-side hits every 1.5 s so you can check direction without driving.

## v5: weight / suspension load, per-car profiles, shipped default inis

- **Defaults** are now the tuned values (engine 1200, road 3000, brake 3000, total 4000, no hotkey modifier, auto-connect to the SIMAGIC Alpha EVO). `TrueforceRumble.ini` and `TrueforceCars.ini` live in the repo as reference defaults to copy next to the DLL. The build does not use them, and the plugin writes `TrueforceRumble.ini` itself on first run if it is missing. If you add a key in `SyncConfig`, add it to the repo ini by hand.
- **Weight / load model** (relative to each car's own static load, so units don't matter; static load is re-learned at stage start):
  - the more heavily loaded side transmits more road (`LoadInfluence`)
  - compression and landing thump per side (`ThumpGain`), stronger and longer for a heavier car (`WeightFeel`)
  - road and engine go quiet when the wheels leave the ground (`AirborneFade`)
  - braking judder grows with the load on the front tyres
- **Per-car profiles, matched by car name:** the plugin reads the car's name text the way RSF does (`rallysimfans.ini` `[cars] slot`, then `Cars\Cars.ini` `[Car0<slot>]` `CarName` / `RSFCarID`; vanilla installs fall back to the telemetry slot and the car's model file name). Each section in `TrueforceCars.ini` with a `Name=` is a profile; names are matched loosely (case, punctuation, `Grp`/`Group`, missing manufacturer words), but years, model numbers and roman numerals must agree. `Match=` adds aliases and `RSFCarID=` gives an exact match. Keys: `Preset` (inline4, inline4turbo, boxer4, inline5, v6, flat6, v8), `Cylinders`, `RpmMax`, `EngineScale`, `RoadScale`, `LumpScale`, `Grit`, `Unevenness`, `WeightFeel`. The file is re-read when it changes; the menu page and `TrueforceRumble.log` show the detected name and the matched profile. A car change (or a new stage) resets the learned rev limit and load.

## v5.1: landing impacts, car switching fix, new defaults

- **Landings and hard hits** now have their own effect layer (`ImpactMaxMagnitude`, `ImpactGain`). A landing is detected from the raw strut load (not the smoothed one), the time the wheels were off the ground and the heave spike; the hit attacks instantly (square burst, 24-60 Hz, lower for bigger hits and heavier cars) and rings out over about 0.2 s. While it plays, the other layers duck so it is not diluted, and it stays inside `TotalMaxMagnitude` and the hard ceilings. It is symmetric (zero-mean), so it never pushes RBR's steering force. `ImpactGain` 0..2 sets sensitivity (0 = off). TEST mode phase 5 plays big and medium landing hits.
- **Car switching fixed:** the car identity used to be checked only on a few events, so a car changed in the menus kept the old profile. It is now polled about once a second (also when telemetry returns after a pause, on stage start and when the slot changes), and `TrueforceCars.ini` edits are picked up live. A momentarily unreadable `Cars.ini` keeps the current car instead of dropping its profile.
- Shipped `TrueforceRumble.ini` / `TrueforceCars.ini` and the built-in defaults are now the tuned personal set.

## Note

This plugin enhances Richard Burns Rally's own force feedback; it never replaces or overrides it. All effects are zero-mean vibrations added to the wheel alongside RBR's steering force, the wheel's gain and autocenter are never touched, and RBR's force is unaffected when the plugin is muted or unloaded.

## v5.2: landings rewritten

- Landing detection no longer needs 0.1 s of full airtime. Severity now combines three cues with `max()` (peak strut load, how fast the total strut load jumped from its recent minimum, airtime + heave spike), so short hops, crest drops and bottoming out register too, and the severity is continuous: a small hop is a tap, a big jump is a full-strength hit.
- The hit curve is more concave (`^0.6`) and `ImpactMaxMagnitude` default is 5000 (the hard ceiling). The hit may use its own ceiling; the other layers duck to make room.
- Every detected landing / hit is written to `TrueforceRumble.log` with its severity and the cues behind it. Existing `TrueforceRumble.ini` files keep their old `ImpactMaxMagnitude`; set it to 5000 or delete the file.
