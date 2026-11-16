<#
.SYNOPSIS
    Copies Richard Burns Rally's own tyre sounds into the ForceFX dynamic audio sounds folder.

.DESCRIPTION
    RBR keeps its ground sounds in  <RBR>\Audio\Surfaces:
        Normal\<surface>.wav   the sound of a tyre rolling on that surface
        Skid\<surface>.wav     the sound of a tyre sliding on it
    ForceFX looks for  <surface>_loop.wav  (rolling)  and  <surface>_squeal.wav / <surface>_slide.wav  (sliding),
    so each file is copied under that name into  <this folder>\sounds.  RBR's own files are only read, never changed.

    Run it through Copy-RbrSounds.bat, or directly:
        powershell -ExecutionPolicy Bypass -File Copy-RbrSounds.ps1 -RbrDir "C:\Games\RichardBurnsRally"

.PARAMETER RbrDir
    Your RBR folder, the one that contains the Audio folder.

.PARAMETER SoundsDir
    Where to put the sounds. Default: the "sounds" folder next to this script. Only needed if you set SoundsFolder in ForceFXAudio.ini.

.PARAMETER OnlyMissing
    Leave any sound that is already in the sounds folder alone (keeps sounds you replaced with your own).

.PARAMETER DryRun
    Show what would be copied without copying anything.
#>
param(
    [Parameter(Mandatory = $true)][string]$RbrDir,
    [string]$SoundsDir = '',
    [switch]$OnlyMissing,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

function Say([string]$text, [string]$color = '') {
    if ($color) { Write-Host $text -ForegroundColor $color } else { Write-Host $text }
}

# Rolling sound -> <name>_loop, sliding sound -> <name>_squeal and <name>_slide. RBR has one skid sound per surface,
# and ForceFX blends between a squeal (light slip) and a slide (locked or sliding hard), so the one file serves both.
$jobs = @(
    @{ Folder = 'Normal'; Suffixes = @('loop') },
    @{ Folder = 'Skid';   Suffixes = @('squeal', 'slide') }
)

try {
    if (-not (Test-Path -LiteralPath $RbrDir -PathType Container)) {
        throw "The folder '$RbrDir' does not exist."
    }
    $audio = Join-Path $RbrDir 'Audio'
    if (-not (Test-Path -LiteralPath $audio -PathType Container)) {
        throw ("There is no Audio folder in '$RbrDir'. Pick the folder that contains RichardBurnsRally.exe. " +
               "If RBR only has an Audio.dat file there, extract it first (for example with the RBR DAT tool) so the Audio folder exists.")
    }
    $surfaces = Join-Path $audio 'Surfaces'
    if (-not (Test-Path -LiteralPath $surfaces -PathType Container)) {
        throw "There is no Surfaces folder in '$audio', so there are no tyre sounds to copy."
    }

    if (-not $SoundsDir) { $SoundsDir = Join-Path $PSScriptRoot 'sounds' }
    if (-not $DryRun -and -not (Test-Path -LiteralPath $SoundsDir)) {
        New-Item -ItemType Directory -Path $SoundsDir | Out-Null
    }

    Say "RBR sounds : $surfaces"
    Say "Copying to : $SoundsDir"
    if ($DryRun) { Say 'Dry run: nothing will be copied.' 'Yellow' }
    Say ''

    $copied = 0; $skipped = 0; $notWav = @()
    foreach ($job in $jobs) {
        $src = Join-Path $surfaces $job.Folder
        if (-not (Test-Path -LiteralPath $src -PathType Container)) {
            Say "No $($job.Folder) folder in Surfaces, skipping it." 'Yellow'
            continue
        }
        foreach ($file in Get-ChildItem -LiteralPath $src -File) {
            if ($file.Extension -ne '.wav') { $notWav += $file.FullName; continue }
            $name = ($file.BaseName.ToLowerInvariant() -replace '[^a-z0-9]+', '_').Trim('_')
            if (-not $name) { continue }
            foreach ($suffix in $job.Suffixes) {
                $target = Join-Path $SoundsDir ("{0}_{1}.wav" -f $name, $suffix)
                if ($OnlyMissing -and (Test-Path -LiteralPath $target)) {
                    Say ("  kept     {0}" -f (Split-Path $target -Leaf)); $skipped++; continue
                }
                if (-not $DryRun) { Copy-Item -LiteralPath $file.FullName -Destination $target -Force }
                Say ("  {0,-8} {1}\{2}  ->  {3}" -f $(if ($DryRun) { 'would' } else { 'copied' }), $job.Folder, $file.Name, (Split-Path $target -Leaf))
                $copied++
            }
        }
    }

    Say ''
    if ($notWav.Count -gt 0) {
        Say ("{0} file(s) were not WAV and were skipped, because ForceFX only reads WAV files:" -f $notWav.Count) 'Yellow'
        $notWav | ForEach-Object { Say "  $_" 'Yellow' }
        Say ''
    }
    if ($copied -eq 0 -and $skipped -eq 0) {
        Say 'No sounds were found to copy. Check that Audio\Surfaces\Normal and Audio\Surfaces\Skid contain .wav files.' 'Red'
        exit 1
    }
    $verb = if ($DryRun) { 'would be copied' } else { 'copied' }
    Say ("Done. {0} file(s) {1}, {2} left as they were." -f $copied, $verb, $skipped) 'Green'
    if (-not $DryRun) { Say 'Start RBR and switch Dynamic audio on in the ForceFX menu.' }
}
catch {
    Say ''
    Say "Could not copy the sounds: $($_.Exception.Message)" 'Red'
    exit 1
}
