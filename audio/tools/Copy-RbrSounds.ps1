<#
.SYNOPSIS
    Copies Richard Burns Rally's own sounds into the ForceFX dynamic audio sounds folder.

.DESCRIPTION
    ForceFX plays WAV files from  <this folder>\sounds  and finds them by name (see audio\surfaces.md).
    This script takes the matching sounds from RBR's Audio folder and copies them there under those names.
    RBR's own files are only read, never changed.

        RBR (under <RBR>\Audio)                      ForceFX name
        Surfaces\Normal\<surface>                    <surface>_loop
        Surfaces\Skid\<skid file>                    <surface>_squeal (the file Effects.ini names, e.g. skidtarmac)
        Surfaces\Skid\<other file>                   <surface>_slide  (RBR's second skid sound: tarm, gravel, snow)
        Impact\Grus\*                                debris_stone_1, _2 ...
        Impact\Medium\*                              debris_impact_1, _2 ...
        Impact\Heavy\*                               debris_landing_1, _2 ...
        Impact\Soft, SoftMet, Addon, Branch, Shatter  crash_light, crash_metal, crash_crumple, crash_branch, crash_glass (crashes)
        Cars\Effects\rumble.wav                      debris_rumble
        Cars\Effects\SuspSqee\*                      susp_squeak_1, _2 ...
        Cars\Effects\Suspen\*                        susp_bottom_1, _2 ...

    Surfaces are tarmac, gravel, snow and water, plus RBR's "generic" ground sound, which becomes grass.
    Where RBR has two versions of a surface sound (for example gravel.wav and normgravel.wav), the one that
    Audio\Cars\Effects.ini points at is used, so you get what the game itself plays.

    Squeal (wheelspin under power) and slide (sideways or locked tyre) are separate sounds in ForceFX. RBR's Skid folder
    holds two files for most surfaces, so each gets its own: the one Effects.ini names becomes the squeal, the other one
    the slide. Water, and any surface with only one skid file, uses it for both. If they sound the wrong way round, use -SwapSkid.

    Run it through Copy-RbrSounds.bat, or directly:
        powershell -ExecutionPolicy Bypass -File Copy-RbrSounds.ps1 -RbrDir "C:\Games\RichardBurnsRally"

.PARAMETER RbrDir
    Your RBR folder, the one that contains the Audio folder.

.PARAMETER SoundsDir
    Where to put the sounds. Default: the "sounds" folder next to this script. Only needed if you set SoundsFolder in the [Audio] section of ForceFX.ini.

.PARAMETER OnlyMissing
    Leave any sound that is already in the sounds folder alone (keeps sounds you replaced with your own).

.PARAMETER SwapSkid
    Give the squeal the sound the slide would get and the other way round, if you prefer them that way.

.PARAMETER DryRun
    Show what would be copied without copying anything.
#>
param(
    [Parameter(Mandatory = $true)][string]$RbrDir,
    [string]$SoundsDir = '',
    [switch]$OnlyMissing,
    [switch]$SwapSkid,
    [switch]$DryRun
)

$ErrorActionPreference = 'Stop'

# ---- What to copy -----------------------------------------------------------------------------------------------
# Surface sounds: ForceFX name -> RBR file names to try, best first (without .wav, inside Surfaces\Normal or Surfaces\Skid).
# Whatever Effects.ini names for that surface is tried before these.
$surfaceFiles = [ordered]@{
    'tarmac' = @{ Normal = @('normtarmac');           Skid = @('skidtarmac', 'tarm') }
    'gravel' = @{ Normal = @('normgravel', 'gravel'); Skid = @('skidgravel', 'gravel') }
    'snow'   = @{ Normal = @('normsnow', 'snow');     Skid = @('skidsnow', 'snow') }
    'water'  = @{ Normal = @('normwater', 'water');   Skid = @('skidwater') }
    'grass'  = @{ Normal = @('generic');              Skid = @('generic') }
}
# The rolling sound gets one name. The skid files get two different ones: the first file found is the squeal, the next
# different file the slide (RBR's Skid folder has two for tarmac, gravel and snow). With only one file it serves as both.

# Sets of variants: ForceFX name -> folder under Audio. Every .wav in it is copied as <name>_1, <name>_2 ...
$variantSets = @(
    @{ Name = 'debris_stone';   Folder = 'Impact\Grus' },
    @{ Name = 'debris_impact';  Folder = 'Impact\Medium' },
    @{ Name = 'debris_landing'; Folder = 'Impact\Heavy' },
    @{ Name = 'crash_light';    Folder = 'Impact\Soft' },
    @{ Name = 'crash_metal';    Folder = 'Impact\SoftMet' },
    @{ Name = 'crash_crumple';  Folder = 'Impact\Addon' },
    @{ Name = 'crash_branch';   Folder = 'Impact\Branch' },
    @{ Name = 'crash_glass';    Folder = 'Impact\Shatter' },
    @{ Name = 'susp_squeak';    Folder = 'Cars\Effects\SuspSqee' },
    @{ Name = 'susp_bottom';    Folder = 'Cars\Effects\Suspen' }
)
$maxVariants = 32

# Single files: ForceFX name -> file under Audio.
$singleFiles = @(
    @{ Name = 'debris_rumble'; File = 'Cars\Effects\rumble.wav' }
)

# ---- Helpers ----------------------------------------------------------------------------------------------------
function Say([string]$text, [string]$color = '') {
    if ($color) { Write-Host $text -ForegroundColor $color } else { Write-Host $text }
}

$script:copied = 0
$script:kept = 0

function Put([string]$sourceFile, [string]$destName) {
    $target = Join-Path $SoundsDir ($destName + '.wav')
    $shortSource = $sourceFile.Substring($audio.Length + 1)
    if ($OnlyMissing -and (Test-Path -LiteralPath $target)) {
        Say ("  kept    {0}" -f (Split-Path $target -Leaf)); $script:kept++; return
    }
    if (-not $DryRun) { Copy-Item -LiteralPath $sourceFile -Destination $target -Force }
    $verb = 'copied'; if ($DryRun) { $verb = 'would' }
    Say ("  {0,-7} {1}  ->  {2}" -f $verb, $shortSource, (Split-Path $target -Leaf))
    $script:copied++
}

# Numbers in a name sort as numbers, so 2.wav comes before 10.wav.
function NaturalKey([string]$name) {
    [regex]::Replace($name, '\d+', [System.Text.RegularExpressions.MatchEvaluator]{ param($m) $m.Value.PadLeft(8, '0') })
}

try {
    if (-not (Test-Path -LiteralPath $RbrDir -PathType Container)) {
        throw "The folder '$RbrDir' does not exist."
    }
    $audio = (Resolve-Path -LiteralPath (Join-Path $RbrDir 'Audio') -ErrorAction SilentlyContinue)
    if (-not $audio) {
        throw ("There is no Audio folder in '$RbrDir'. Pick the folder that contains RichardBurnsRally.exe. " +
               "If RBR only has an Audio.dat file there, extract it first (for example with the RBR DAT tool) so the Audio folder exists.")
    }
    $audio = $audio.Path.TrimEnd('\')

    if (-not $SoundsDir) { $SoundsDir = Join-Path $PSScriptRoot 'sounds' }
    if (-not $DryRun -and -not (Test-Path -LiteralPath $SoundsDir)) {
        New-Item -ItemType Directory -Path $SoundsDir | Out-Null
    }

    Say "RBR audio  : $audio"
    Say "Copying to : $SoundsDir"
    if ($DryRun) { Say 'Dry run: nothing will be copied.' 'Yellow' }

    # What the game itself plays for each surface: lines like  NormalGravel = Audio\Surfaces\Normal\gravel
    $fromIni = @{}
    $iniPath = Join-Path $audio 'Cars\Effects.ini'
    if (Test-Path -LiteralPath $iniPath) {
        foreach ($line in Get-Content -LiteralPath $iniPath) {
            if ($line -match '^\s*(Normal|Skid)([A-Za-z0-9]+)\s*=\s*(\S.*?)\s*(;.*)?$') {
                $kind = $Matches[1]; $surface = $Matches[2].ToLowerInvariant(); $path = $Matches[3]
                if ($surface -eq 'generic') { $surface = 'grass' }
                $leaf = ($path -replace '/', '\').Split('\')[-1]
                if ($leaf.ToLowerInvariant().EndsWith('.wav')) { $leaf = $leaf.Substring(0, $leaf.Length - 4) }
                $fromIni["$kind/$surface"] = $leaf
            }
        }
    }

    # Surfaces
    foreach ($kind in @('Normal', 'Skid')) {
        Say ''
        Say "Surface sounds ($kind)"
        $dir = Join-Path $audio "Surfaces\$kind"
        if (-not (Test-Path -LiteralPath $dir -PathType Container)) {
            Say "  No Surfaces\$kind folder, skipping." 'Yellow'; continue
        }
        foreach ($surface in $surfaceFiles.Keys) {
            $candidates = @()
            if ($fromIni.ContainsKey("$kind/$surface")) { $candidates += $fromIni["$kind/$surface"] }
            $candidates += $surfaceFiles[$surface][$kind]
            $found = $null
            foreach ($c in $candidates) {
                $p = Join-Path $dir ($c + '.wav')
                if (Test-Path -LiteralPath $p -PathType Leaf) { $found = $p; break }
            }
            if (-not $found) { Say "  none     $surface (no matching file in Surfaces\$kind)" 'Yellow'; continue }
            if ($kind -eq 'Normal') { Put $found ("{0}_loop" -f $surface); continue }
            # Skid: the squeal and the slide are different sounds when RBR has two for this surface
            $other = $null
            foreach ($c in $candidates) {
                $p = Join-Path $dir ($c + '.wav')
                if ((Test-Path -LiteralPath $p -PathType Leaf) -and ($p -ne $found)) { $other = $p; break }
            }
            if (-not $other) { $other = $found }
            $squealFile = $found; $slideFile = $other
            if ($SwapSkid) { $squealFile = $other; $slideFile = $found }
            Put $squealFile ("{0}_squeal" -f $surface)
            Put $slideFile ("{0}_slide" -f $surface)
        }
    }

    # Sets of variants
    foreach ($set in $variantSets) {
        Say ''
        Say ("{0}  (from {1})" -f $set.Name, $set.Folder)
        $dir = Join-Path $audio $set.Folder
        if (-not (Test-Path -LiteralPath $dir -PathType Container)) {
            Say "  No $($set.Folder) folder, skipping." 'Yellow'; continue
        }
        $files = @(Get-ChildItem -LiteralPath $dir -File | Where-Object { $_.Extension -eq '.wav' } | Sort-Object { NaturalKey $_.Name })
        if ($files.Count -eq 0) { Say '  No .wav files there.' 'Yellow'; continue }
        if ($files.Count -gt $maxVariants) { $files = $files[0..($maxVariants - 1)] }
        $n = 0
        foreach ($f in $files) { $n++; Put $f.FullName ("{0}_{1}" -f $set.Name, $n) }
    }

    # Single files
    Say ''
    Say 'Other'
    foreach ($single in $singleFiles) {
        $p = Join-Path $audio $single.File
        if (-not (Test-Path -LiteralPath $p -PathType Leaf)) { Say "  none     $($single.Name) ($($single.File) not found)" 'Yellow'; continue }
        Put $p $single.Name
    }

    Say ''
    if ($script:copied -eq 0 -and $script:kept -eq 0) {
        Say 'No sounds were found to copy. Check that the Audio folder is RBR''s own (it should contain Surfaces, Impact and Cars).' 'Red'
        exit 1
    }
    $word = 'copied'; if ($DryRun) { $word = 'would be copied' }
    Say ("Done. {0} file(s) {1}, {2} left as they were." -f $script:copied, $word, $script:kept) 'Green'
    if (-not $DryRun) { Say 'Start RBR and switch Dynamic audio on in the ForceFX menu.' }
}
catch {
    Say ''
    Say "Could not copy the sounds: $($_.Exception.Message)" 'Red'
    exit 1
}
