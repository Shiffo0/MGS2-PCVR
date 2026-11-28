# MGS2 PCVR v0.6.2 — Steam Frame test setting and Two Eye start-up reports

A small update to v0.6.1. Nothing changes unless you switch the new setting on. Everything from [v0.6.0](https://github.com/Shiffo0/MGS2-PCVR/releases/tag/v0.6.0) and [v0.6.1](https://github.com/Shiffo0/MGS2-PCVR/releases/tag/v0.6.1) is included.

## New

**`vr_fov_symmetric=1` (off by default)** — for [#10](https://github.com/Shiffo0/MGS2-PCVR/issues/10)

On the Steam Frame with SteamVR, the world can seem to zoom or swim when you turn your head. The Frame shows much more below the centre of each eye than above it (about 60° down and 49° up), and the effect disappears when that is made equal. With this setting the mod renders and submits the same amount above and below the centre.

- To try it: close the game, add the line `vr_fov_symmetric=1` to `dg_hook.on`, and start again. The log confirms it with a line starting `xr: vr_fov_symmetric=1`.
- It is a test setting. It has not been tried on a Steam Frame yet, so we do not know whether it cures the effect. If you have a Frame, please tell us in #10.
- Headsets that already look right (Quest, PS VR2) do not need it.

## Better problem reports

**VR Rendering stuck on WAITING.** When Two Eye rendering cannot start, the headset keeps showing AER and the Home panel says WAITING. `logs\dg_hook.log` now says why:

- `second_eye: a frame could not start: <reason>` when the mod refuses to start a second-eye frame.
- `second_eye: no frame started in this window; draws ended at: …` when a stretch of gameplay passes without one.

Please attach the log if you see WAITING with MGSHDFix installed.

## Housekeeping

Log texts and labels no longer use names of functions and variables from the original game. This changes wording in `dg_hook.log` only.

## MGSHDFix required

**[MGSHDFix](https://github.com/ShizCalev/MGSHDFix#installation) is required for Two Eye rendering, and also to get 4K resolution in the headset.** Install it separately. For 4K, set the resolution to **3840 × 2160** in the MGSHDFix Config Tool (in the game's `plugins` folder), and set Internal Resolution and Internal Upscaling to Default in the game's launcher. Without MGSHDFix, switch to AER with **Home → VR Rendering**.

## Installation and updating

Close the game and back up your existing mod files. Extract **MGS2-PCVR-v0.6.2.zip** into the folder containing `METAL GEAR SOLID2.exe`. Coming from v0.6.0 or v0.6.1, only `dg_hook.asi` changed; your `dg_hook.on` can stay. Keep `dg_hand_calibration.bin` and `dg_favorites.dat`.

## Known limitations

- An occasional flickering surface can still appear with Two Eye rendering. Switch to AER if it bothers you.
- Steam Frame: distorted stereo in the right eye is still open ([#10](https://github.com/Shiffo0/MGS2-PCVR/issues/10)). Mono view is the workaround for now.
- Plant / Raiden support is still incomplete. See the README for controls and other gameplay limitations.

## Source

The `src` folder contains the full source of the mod itself. It contains no Metal Gear Solid 2 source code and no game assets.
