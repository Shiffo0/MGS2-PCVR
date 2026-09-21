# MGS2 PCVR — v0.3.3

Playable on PS VR 2 / SteamVR.

## What changed

- Fixed the SteamVR gameplay “Waiting” problem caused by graphics-device initialization.
- Retained unlimited VR sessions with the included `seconds=0` setting.
- Retained recovery when rendering resolution changes, including interrupted swapchain creation.
- Includes the existing movement, weapon, interaction, wrist radar and LIFE features.

## Install or update

Download **MGS2-PCVR-v0.3.3-62E9EEAF.zip** below. Close the game and back up existing mod files before extracting beside **METAL GEAR SOLID2.exe**.

Existing users can replace **dg_hook.asi** and set **seconds=0** in their existing **dg_hook.on** to keep their settings. The included loaders are unchanged from the previous release.

Use **SteamVR as the active OpenXR runtime for PS VR2**. For Quest Link / Air Link, use Meta's OpenXR runtime. The README control table uses Quest button labels; PS VR2 assignments can differ through SteamVR.

Development recording and GPU measurement probes are disabled in this player build. Bounded error and support logging remains available.

This remains a Tanker-focused mod. Plant/Raiden, hanging and grenades retain the limitations described in the [README](https://github.com/Shiffo0/MGS2-PCVR#known-limitations).
