# MGS2 PCVR Public Beta Hotfix 1 — 9A439F87

Based on public beta build `479DF688`. This hotfix changes only the OpenXR projection-swapchain resize lifecycle.

## Hotfix

- Detects when MGS2 changes its backbuffer resolution after the OpenXR projection swapchains were created.
- Rebuilds both projection swapchains on the XR frame thread before the next copy/submission.
- Cleans up partially created swapchains before retrying.
- Adds a bounded log line showing the old and new dimensions during recovery.

This targets issue #2, where SteamVR displayed **Waiting** during Tanker gameplay while codec and cutscenes remained visible. The supplied log showed both `2560x1440` and `1280x720` backbuffers. The old build kept its first swapchain size and correctly refused incompatible copies, leaving gameplay with zero submitted layers.

## Hotfix validation

- Shipping release build completed successfully.
- Release profile: 30 checks, 0 failures.
- Release GPU profile: 11 checks, 0 failures.
- The corresponding development XR/WARP resize regression passed as part of an 870-check capture/frame run.
- Existing `f2l` compiler warnings C4013/C4142 remain.

**SteamVR and physical-headset acceptance are not yet claimed.** Reporter confirmation is requested for Tanker gameplay, codec enter/exit and the 2560x1440 / 1280x720 transition.

## Baseline

Welcome to the first public beta of **MGS2 PCVR**, an unofficial PC VR mod for **METAL GEAR SOLID 2: Sons of Liberty — Master Collection Version**.

**Tested on Meta Quest 3 with Touch Plus controllers.** Requires the Steam game, a Windows gaming PC and a working OpenXR PC VR connection.

## What to expect

- Headset tracking and motion-controller controls, using **AER (Alternate Eye Rendering)**.
- Switch between **VR and Third Person** with **A**; hold **Y** to recenter.
- Wrist radar and LIFE display, with controller menus for weapons and items.
- A **Tanker-focused beta**. Plant / Raiden support is still in development.

## Download and install

Download **`MGS2-PCVR-v0.3.1-beta.3-479DF688.zip`** under **Assets** below. Close the game, back up any existing mod files, and extract the ZIP beside **`METAL GEAR SOLID2.exe`**. Launch through Steam with your headset connected.

GitHub's automatic **Source code** downloads are for contributors and are not the playable mod package.

**[Read the setup guide and controls before playing](https://github.com/Shiffo0/MGS2-PCVR#installation).**

## Before you play

- **One-hour VR session limit:** save and restart before the hour is up, or close the game and increase `seconds=3600` in `dg_hook.on` before playing. The game itself stays open when the mod's session ends.
- **Security cameras:** switch to Third Person to get past camera sections.
- **Hanging and moving while hanging:** experimental; use Third Person.
- **Grenades:** not supported in VR yet.
- **Plant / Raiden:** not ready for a complete VR playthrough. Weapon and interaction support is still being developed.
- **AER:** eyes update on alternating frames, which can cause ghosting or judder. At 60 game frames per second, each eye receives roughly 30 fresh images per second.

Found a problem? [Report it here](https://github.com/Shiffo0/MGS2-PCVR/issues), including your setup, location in the game and steps to reproduce it.

`MGS2-PCVR-v0.3.1-beta.3-hotfix.1-9A439F87.zip` is the playable release. Extract beside the game executable with the game closed, following the README installation steps. GitHub's automatic Source code archives are not playable packages.

`dg_hook.asi` SHA256: `9A439F870367BDCEC4F3228853271F9EE7CA98E7AA70D9B11191305ED2CEF6A7`

MIT covers contributors' own work. Third-party notices remain included. No game files, saves, debugging symbols or measurement output are bundled.
