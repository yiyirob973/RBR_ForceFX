# RBR_ForceFX

ForceFX adds extra force feedback detail to Richard Burns Rally. It listens to RBR's telemetry and layers engine, road and braking vibration onto your wheel, and it makes landings hit harder. RBR's own steering force stays in charge. ForceFX only adds to it.

## What it does

- **Engine vibration** that follows rpm and throttle, and gets rougher near the rev limit.
- **Road feel.** Gravel, dirt, kerbs and bumps come through as texture, and the left and right sides are felt separately.
- **Braking judder**, with a harsher shake when the front wheels lock up.
- **Landing boost.** When the car lands from a jump or takes a hard hit, RBR's own steering force pulls harder for a moment. ForceFX doesn't stack a separate effect on top of it.
- **Per-car profiles** for engine type, rev limit and weight feel, matched by car name.
- **An in-game settings menu** for tuning while you play, with a short tooltip for every setting.
- **Safety limits** so the plugin can't push the wheel past hard ceilings.

## Requirements

- Windows
- The latest RallySimFans (RSF) version of Richard Burns Rally, which includes NGP7
- A DirectInput force feedback wheel
- NGP7 UDP telemetry sent to ForceFX (see step 3 below)

A plain RBR install without NGP7 isn't supported.

## Installation

1. Download the latest zip from the Releases page.
2. Copy `RBR_ForceFX.dll`, `ForceFX.ini` and `ForceFXCars.ini` into your RBR `Plugins` folder (or wherever your RSF install keeps plugins).
3. In the RSF Launcher, turn on NGP UDP telemetry and send it to `127.0.0.1` (this PC) on port `6778`. The exact labels vary between RSF versions, so look for the UDP telemetry output rather than any other telemetry option.
4. Check that `UdpPort=6778` in `ForceFX.ini` matches the port you chose. If you change one, change the other. Don't reuse a port that another tool (SimHub, for example) already listens on.
5. Start RBR and open **Plugins > ForceFX** from the main menu. Pick your wheel, press Select to connect, and tune to taste. Set `AutoConnect=1` in `ForceFX.ini` if you want it to connect on its own next time.

## Tuning

Everything in the in-game menu is saved to `ForceFX.ini`, and each row has a tooltip explaining what it does. The menu also has:

- on/off switches for each effect, so you can feel them one at a time
- a built-in test mode that plays engine, gravel, dirt and braking vibration without driving
- a diagnostics page with live telemetry values and the raw packet, for when something looks off

If the wheel feels too busy or too strong, lower **Total ceiling** first.

While driving, **Numpad +** and **Numpad -** raise and lower the overall level, and **Numpad \*** mutes or unmutes ForceFX. The level can only go down from your configured limits, never above them. You can change the keys in `ForceFX.ini`, and the hotkeys only work while RBR is the active window.

## Car profiles

`ForceFXCars.ini` holds a profile for each car you want to tune: cylinder count, rev limit, how rough the engine feels, and how heavy the car feels. Cars are matched by name, and the match is forgiving about capitalisation, punctuation and "Grp" versus "Group". Years and model numbers do have to agree. The file's header lists every option. Cars without a profile just use the values from `ForceFX.ini`.

## Safety

The vibration effects average out to zero, so they never pull the wheel to one side. The landing boost is the one thing that touches RBR's own steering force, and it only lasts a fraction of a second. You can switch it off in the menu.

ForceFX also protects your hardware with:

- hard output ceilings that no setting can exceed
- frequency limits
- gradual derating if the output stays high for a long time
- a fade to zero when the plugin shuts down

It is still a tool for people who like to tune things, and it works your wheel hard in places. Start with the defaults, turn the strength up slowly, and be careful with powerful direct drive wheels.

## Troubleshooting

- **Nothing happens.** Open the ForceFX menu and check the UDP packet count. If it stays at zero, telemetry isn't reaching the plugin, so recheck the port in the RSF Launcher and in `ForceFX.ini`.
- **The menu says it is waiting for RBR to use the wheel.** Make sure force feedback is enabled in RBR and that you picked the same wheel RBR uses.
- **Something else is wrong.** Look at `ForceFX.log` next to the DLL. `ForceFX_packet.txt` is a dump of the first telemetry packet, useful if your NGP version uses a different layout (the offsets are in `ForceFX.ini`).

## Building from source

RBR is a 32-bit game, so build an x86 DLL with the static runtime from a Visual Studio x86 developer prompt:

```
cl /std:c++17 /EHsc /LD /MT /O2 RBR_ForceFX.cpp /Fe:RBR_ForceFX.dll /link /DEF:RBR_ForceFX.def
```

Pushing a tag that starts with `v` runs the GitHub Actions workflow, which builds the DLL and attaches a release zip.

## Credits and license

`IRBRGame.h` and `IPlugin.h` are the plugin interface headers from the RBR SDK. The road feel draws on the same idea RbrSurfaceAudio uses for tyre roughness.

This project is provided as is, without any warranty. Use it at your own risk.
