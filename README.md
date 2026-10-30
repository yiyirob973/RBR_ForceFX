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
