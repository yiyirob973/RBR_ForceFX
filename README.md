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
