# MGS2 PCVR v0.6.1 — Mono view fixes and better problem reports

A small update to v0.6.0. Everything from [v0.6.0](https://github.com/Shiffo0/MGS2-PCVR/releases/tag/v0.6.0) is included: Two Eye rendering by default, two-handed weapons, hand reloading, Scope and Camera to the eye, laser pointer menus.

## Fixed

**Mono view** (Home → View → MONO, or `stereo=0` in `dg_hook.on`) — [#11](https://github.com/Shiffo0/MGS2-PCVR/issues/11)
- The intro, title screen and menus now show in the headset. Before, the headset stayed empty there while the sound played.
- Controller input now works in those menus.
- The ammo counter on the weapon now shows in Mono too.

Stereo and Two Eye rendering behave as in v0.6.0.

## Better problem reports

`logs\dg_hook.log` now records three things that make setup problems much easier to pin down. If you report a problem, please attach that file.

- **What your headset reports for each eye:** field of view, and how each eye sits relative to the head (lines starting with `xr: eye 0 view` and `xr: eye 1 view`). This is for headsets we cannot test ourselves, such as the Steam Frame ([#10](https://github.com/Shiffo0/MGS2-PCVR/issues/10)).
- **Which controller profile the VR runtime gives the game** (`xr: controller profile`), and a line when the runtime takes controller input away from the game (`xr: input focus lost`). With SteamVR this happens while its dashboard or an overlay is open: you can still look around, but the controllers do nothing ([#9](https://github.com/Shiffo0/MGS2-PCVR/issues/9)).
- **Whether each frame reached the headset**, and if not, why (`OpenXR capture` lines). These were missing in v0.6.0.

## MGSHDFix required

**[MGSHDFix](https://github.com/ShizCalev/MGSHDFix#installation) is required for Two Eye rendering, and also to get 4K resolution in the headset.** Install it separately. For 4K, set the resolution to **3840 × 2160** in the MGSHDFix Config Tool (in the game's `plugins` folder), and set Internal Resolution and Internal Upscaling to Default in the game's launcher. Without MGSHDFix, switch to AER with **Home → VR Rendering**.

## Installation and updating

Close the game and back up your existing mod files. Extract **MGS2-PCVR-v0.6.1.zip** into the folder containing `METAL GEAR SOLID2.exe`. Coming from v0.6.0, only `dg_hook.asi` changed; your `dg_hook.on` can stay. Keep `dg_hand_calibration.bin` and `dg_favorites.dat`.

## Known limitations

- An occasional flickering surface can still appear with Two Eye rendering. Switch to AER if it bothers you.
- Steam Frame: distorted stereo and a view that seems to zoom when you turn your head are still open ([#10](https://github.com/Shiffo0/MGS2-PCVR/issues/10)). Mono view is the workaround for now.
- Plant / Raiden support is still incomplete. See the README for controls and other gameplay limitations.

## Source

The `src` folder contains the full source of the mod itself. It contains no Metal Gear Solid 2 source code and no game assets.
