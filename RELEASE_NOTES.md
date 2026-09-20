# MGS2 PCVR — Public Beta

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
