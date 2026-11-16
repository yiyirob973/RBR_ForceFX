# ForceFX dynamic audio: copying the RBR sounds

This folder comes with the **audio** build of ForceFX (`ForceFX-<version>.zip`). The no-audio build
(`ForceFX-<version>-NoAudio.zip`) does not need it.

ForceFX plays its tyre and ground sounds from WAV files in `Plugins\ForceFXAudio\sounds`.
`Copy-RbrSounds.bat` fills that folder from your RBR installation. It reads the game's own tyre sounds in
`Audio\Surfaces` (`Normal` = rolling, `Skid` = sliding) and copies them under the names ForceFX looks for, for example
`Normal\gravel.wav` becomes `gravel_loop.wav` and `Skid\gravel.wav` becomes `gravel_squeal.wav` and `gravel_slide.wav`.
RBR's own files are never changed.

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
