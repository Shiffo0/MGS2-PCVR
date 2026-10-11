# MGS2 PCVR v0.5.0 — Huge performance increase with second-eye rendering

## Highlights

- **Two Eye rendering (Experimental).** The second eye is now drawn from the same game frame instead of the next one. Both eyes get a fresh image every game frame: up to **60 images per second per eye**, where AER gives each eye about 30. Motion is much smoother, with far less judder and ghosting.
- **Effects in both eyes.** Rain, light pools, glow layers and the sky are drawn at the correct depth in the second eye.
- **Matching colours.** The second eye gets the same MGSHDFix colour correction as the first.
- **No vanishing walls at the edge of view.** In Two Eye rendering, the game renders a field of view that covers both eyes, and each eye is shown its own part of it.

## Also new since v0.4.0

**HF blade (Raiden, experimental)**
- **No button needed:** with the blade in hand in VR, swing to attack. Swing across, up or down to slash, and thrust forward to stab. Lifting the sword before a chop counts as a wind-up, so the chop itself is what attacks.
- **Two-handed block:** put your left hand on the blade and hold the left grip to block. Release the grip or move your hand away to stop blocking. Your left hand stays on the blade while you block.
- **Full blade model** in first person instead of the stretched one.

**Desktop observer**
- The desktop window now shows what you see in the headset, by default the **Right Eye**, cropped to 16:9.
- **Both Eyes (EXP)** combines both eyes into one wider desktop view (Experimental) and avoids cutting your hands and arms at the seam where it can.
- Switch it with **Home → Desktop Observer**.

**Weapons and gameplay**
- **Pistol aiming:** aim transitions keep your controller's rotation, while reload, wall, melee and other special animations stay native. This reduces arm jumps when you start aiming.
- **Reload feedback:** a green M9 slide cue and a SOCOM magazine cue during manual reloads. With Two Eye rendering, the USP also shows green markers at the magazine opening.
- **Stinger:** picking it up into your hand is more reliable.
- **Fission Mailed:** the fake game-over screen is shown flat, and your controls keep working.

**Home menu:** two new entries, **Desktop Observer** and **VR Rendering** (AER / BOTH EYES (EXP)).

## Defaults

- **AER (Alternate Eye Rendering) remains the default** (`stereo=1`, `vr_second_eye=0`).
- Switch on **Two Eye rendering** in-game with **Home → VR Rendering → BOTH EYES (EXP)**; `(EXP)` means **Experimental**. Home changes last for the session. To start with it every time, set `vr_second_eye=1` in `dg_hook.on`.
- The desktop window shows the **Right Eye** by default.

## MGSHDFix required

**[MGSHDFix](https://github.com/ShizCalev/MGSHDFix#installation) is required for Two Eye rendering, and also to get 4K resolution in the headset.** Install it separately. For 4K, set the resolution to **3840 × 2160** in the MGSHDFix Config Tool (in the game's `plugins` folder), and set Internal Resolution and Internal Upscaling to Default in the game's launcher. MGSHDFix and this mod use the same ASI loader (`winmm.dll`), so one copy serves both.

## Installation and updating

Close the game and back up your existing mod files. Extract **MGS2-PCVR-v0.5.0.zip** into the folder containing `METAL GEAR SOLID2.exe`. Use the supplied `.on` files to get the new defaults. Keep `dg_hand_calibration.bin` to preserve your wrist alignment.

If you keep your own `dg_hook.on`, add `stereo_test=2` and check `stereo=1` and `vr_second_eye=0`. The wider field of view is only used while Two Eye rendering is on; AER renders exactly as in v0.4.0.

## Known limitations

- Two Eye rendering is **Experimental**. Occasional flickering surfaces or stray quads can still appear. Switch back to AER if this bothers you.
- Plant / Raiden support is still incomplete. See the README for controls and other gameplay limitations.

## Source

The `src` folder contains the full source of the mod itself. It contains no Metal Gear Solid 2 source code and no game assets.
