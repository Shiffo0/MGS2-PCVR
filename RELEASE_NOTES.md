# MGS2 PCVR v0.6.0 — Two Eye rendering by default, hands-on weapons

## Highlights

- **Two Eye rendering is the default.** Both eyes get a fresh image from every game frame: up to **60 images per second per eye**, where AER gives each eye about 30. It is no longer marked experimental.
- **No more vanishing surfaces in one eye.** With some MGSHDFix setups, v0.5.0 skipped part of the scene in the second eye, so surfaces could briefly disappear. Every draw now reaches both eyes.
- **AER is still there.** Switch with **Home → VR Rendering → AER**, or set `vr_second_eye=0` in `dg_hook.on`. Use it if you do not have MGSHDFix.

## MGSHDFix required

**[MGSHDFix](https://github.com/ShizCalev/MGSHDFix#installation) is required for Two Eye rendering, and also to get 4K resolution in the headset.** Install it separately. For 4K, set the resolution to **3840 × 2160** in the MGSHDFix Config Tool (in the game's `plugins` folder), and set Internal Resolution and Internal Upscaling to Default in the game's launcher. MGSHDFix and this mod use the same ASI loader (`winmm.dll`), so one copy serves both.

## Also new since v0.5.0

**Rifles**
- **Two-handed grip** for the AKS-74u, M4 and PSG-1: put your left hand on the front of the rifle. Move your hands apart to let go.
- **Reload with your hands:** when the magazine is empty, a blue magazine hint marks the magazine well. Bring your left hand there to reload.
- **PSG-1 scope to your eye:** raise the rifle until the scope is at your eye to look through it; zoom with the grips. You can walk with the rifle at the hip.

**Pistols**
- **Two-handed grip** for the M9, USP and SOCOM: bring your left hand under the grip.
- **Magazine from below:** during a manual reload, slide the magazine into the grip from below.
- **M9 slide:** grab the slide with your left hand and pull it back.

**Scope and Camera**
- Raise **Raiden's Scope** or **Snake's Camera** to your eye to look through it. Right grip zooms in, left grip zooms out, and with the Camera the right trigger takes a photo. Lower it to put the viewfinder away.

**HF blade (Raiden)**
- **Hits land during the swing** itself, instead of after it.
- **Your body follows** when you turn with the stick while holding the blade, so your arms stay in front of you.
- Improved arm placement while holding the blade.

**Menus**
- **Laser pointer:** point at an entry in OPTIONS, the main menu or the LOAD list and press the right trigger.
- **VR SETTINGS** in the game's OPTIONS menu, below EXIT.
- The Home panel entry **VR Rendering** now shows **TWO EYE** or **AER**.

**Other improvements**
- The left hand follows your controller more steadily when you reach.

## Defaults

- **Two Eye rendering** (`stereo=1`, `vr_second_eye=1`, `stereo_test=2`).
- The desktop window shows the **Right Eye**; **Both Eyes (EXP)** stays experimental.
- The new weapon, item and blade features are on in the supplied `dg_hook.on`.

## Installation and updating

Close the game and back up your existing mod files. Install MGSHDFix if you have not already. Extract **MGS2-PCVR-v0.6.0.zip** into the folder containing `METAL GEAR SOLID2.exe`, and use the supplied `dg_hook.on` to get the new defaults and features. Keep `dg_hand_calibration.bin` (wrist alignment) and `dg_favorites.dat` (radial favourites) when updating.

If you keep your own `dg_hook.on` from v0.5.0, it contains `vr_second_eye=0` and keeps AER. Change it to `vr_second_eye=1` for Two Eye rendering, and copy the new lines from the supplied file to turn on the new features.

## Known limitations

- An occasional flickering surface can still appear with Two Eye rendering. Switch to AER if it bothers you.
- The new weapon and item features are fresh: hand positions can be a few centimetres off, and your own arm can sometimes block part of the view through the Scope or Camera.
- Plant / Raiden support is still incomplete. See the README for controls and other gameplay limitations.

## Source

The `src` folder contains the full source of the mod itself. It contains no Metal Gear Solid 2 source code and no game assets. The player build is compiled from it with `src\build.bat`; development measurement tools are not part of it.
