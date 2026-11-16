# ForceFX dynamic audio: copying the RBR sounds

This folder comes with the **audio** build of ForceFX (`ForceFX-<version>.zip`). The no-audio build
(`ForceFX-<version>-NoAudio.zip`) does not need it.

ForceFX plays its sounds from WAV files in `Plugins\ForceFXAudio\sounds`, found by name.
`Copy-RbrSounds.bat` fills that folder from your RBR installation. It only reads RBR's `Audio` folder and never changes it.

| From RBR's `Audio` folder | Copied as |
|---|---|
| `Surfaces\Normal` (tarmac, gravel, snow, water) | `<surface>_loop` |
| `Surfaces\Skid` (tarmac, gravel, snow, water) | `<surface>_squeal` and `<surface>_slide` |
| `Surfaces\Normal` and `Skid` `generic` | `grass_loop`, `grass_squeal`, `grass_slide` |
| `Impact\Grus` | `debris_stone_1`, `_2`, ... (flying stones) |
| `Impact\Medium` | `debris_impact_1`, `_2`, ... (bumps) |
| `Impact\Heavy` | `debris_landing_1`, `_2`, ... (landings) |
| `Cars\Effects\rumble.wav` | `debris_rumble` |
| `Cars\Effects\SuspSqee` | `susp_squeak_1`, `_2`, ... (suspension creaks) |
| `Cars\Effects\Suspen` | `susp_bottom_1`, `_2`, ... (bottoming out) |

Where RBR has two versions of a surface sound (such as `gravel.wav` and `normgravel.wav`), the script uses the one that
`Audio\Cars\Effects.ini` points at, so you get what the game itself plays. RBR has one skid sound per surface, so it
serves as both the squeal and the slide. The stone, impact and landing sounds are picked by folder name, so if one
sounds wrong, replace the matching `debris_*` files with your own.

## Run it

1. Extract the ForceFX zip into RBR's `Plugins` folder, so this folder ends up as `Plugins\ForceFXAudio`.
2. Start `Copy-RbrSounds.bat` in one of these ways. Keep it next to `Copy-RbrSounds.ps1`, which it runs.
   - **Double-click** it, then type your RBR folder when asked. This is the folder that contains the `Audio` folder,
     for example `C:\Games\RichardBurnsRally`.
   - **Drag** your RBR folder onto `Copy-RbrSounds.bat`.
   - From a **command prompt**: `Copy-RbrSounds.bat "C:\Games\RichardBurnsRally"`
3. Read the list it prints, then press a key to close the window.

Options go after the folder in a command prompt:

- `-OnlyMissing` leaves sounds that are already in the sounds folder alone. Without it, sounds with the same name are
  overwritten, so use this if you have replaced some with your own.
- `-DryRun` shows what would be copied without copying anything.
- `-SoundsDir "D:\MySounds"` copies somewhere else. Only needed if you set `SoundsFolder` in `ForceFXAudio.ini`.

Example: `Copy-RbrSounds.bat "C:\Games\RichardBurnsRally" -OnlyMissing`

If it says there is no `Audio` folder, RBR may only have an `Audio.dat` file. Extract that first (for example with the
RBR DAT tool) so the `Audio` folder exists. Files in `Audio\Surfaces` that are not WAV are skipped and listed, because
ForceFX only reads WAV.

## Then turn the audio on

Start RBR, open **Plugins > ForceFX** and switch **Dynamic audio** on. It is off by default. Sound names, mix levels and
the output device are explained in `ForceFXAudio.ini` and in `audio/surfaces.md` in the repository.
