# MGS2 PCVR — Public Beta

> [!IMPORTANT]
> **Player-build candidate `479DF688` — Quest 3 validation pending.** This branch adds a release profile with heavy measurements disabled at compile time. See [release notes](RELEASE_NOTES.md). The Beta 3 download linked below remains the existing build until headset validation is complete.

**MGS2 in PC VR — Quest 3 development baseline.**

An unofficial PC VR mod for **METAL GEAR SOLID 2: Sons of Liberty — Master Collection Version** on Windows, using OpenXR and **AER (Alternate Eye Rendering)**.

**[Download Public Beta 3](https://github.com/Shiffo0/MGS2-PCVR-Beta/releases/tag/v0.3.0-beta.3)** · [Installation](#installation) · [Controls](#controls) · [Known limitations](#known-limitations) · [Report a problem](https://github.com/Shiffo0/MGS2-PCVR-Beta/issues)

> [!IMPORTANT]
> **The earlier Tanker beta was reported fully playable with the workarounds below.** This update includes experimental Raiden/weapon work and needs a fresh headset regression test. Plant is not yet supported as a complete VR campaign.

## Compatibility

| Item | Beta status |
| --- | --- |
| Headset baseline | **Meta Quest 3**, connected to a Windows gaming PC; new combined build awaits headset acceptance |
| Controllers | Quest Touch Plus; default right-handed layout |
| Game | Steam Master Collection version of MGS2 (App 2131640) |
| Local game baseline | Steam build 21578573 |
| Mod build | **5AC0F9C8** — Public Beta 3 |
| Tanker / Snake | Earlier beta reported fully playable; this update needs a regression playthrough |
| Plant / Raiden | In development; not supported as a complete campaign |
| Other headsets / game versions | Not validated for this beta |

Quest 3 and Tanker playability were reported for the earlier beta build 12683251. Build 5AC0F9C8 passed its recorded build and desk checks and was deployed, but a new headset acceptance test for this combined build has not been recorded. This is a PC mod, not a standalone Quest application.

## Included in build 5AC0F9C8

This release packages the complete selected mod binary and its buildable source baseline, including Stinger controller/aim integration, HF Blade support, native HUD and wrist-radar work, coolant changes, M9 slide handling and pistol-reload work. These inclusions describe implementation, not completed headset acceptance. HF Blade, M9 slide, pistol reload, wrist radar and UI settings are enabled in the supplied configuration; diagnostic recording/probes are disabled.

Additional changes include M9 hand contact during slide manipulation, radar and LIFE placement along the rendered forearm, more permissive wrist-look activation, and menu/recovery fixes. Wrist activation uses head orientation, not eye tracking. Radar comfort and hanging behavior still need headset validation.

## Known limitations

- **Getting past security cameras:** this does not work in VR. Switch to **Third Person** for these sections, then return to VR.
- **Hanging:** experimental camera, heading and arm-visibility fixes are included. Entering and moving while hanging still require headset validation; use Third Person if needed.
- **Grenades:** do not work in VR yet.
- **Raiden:** further controller and weapon mapping is required. See the roadmap below.

## Installation

1. Install and launch your own Steam copy of **MGS2 — Master Collection Version** once, then close the game.
2. Connect your Quest 3 to your PC, start your PC VR connection software and make sure its OpenXR runtime is active. Confirm that PC VR works before starting the game.
3. Download **`MGS2-PCVR-v0.3.0-beta.3-5AC0F9C8.zip`** from [Public Beta 3](https://github.com/Shiffo0/MGS2-PCVR-Beta/releases/tag/v0.3.0-beta.3). GitHub's automatic **Source code** ZIP/TAR links are repository snapshots, not the playable mod package.
4. In Steam, right-click the game → **Manage → Browse local files**. The destination is the folder containing **`METAL GEAR SOLID2.exe`**, normally `steamapps/common/MGS2`.
5. Back up any existing files with the same names. Extract the release ZIP directly into that folder. Keep its folder structure. If another mod already provides `winmm.dll` or `openxr_loader.dll`, keep a backup and test this beta on a clean mod setup; combinations have not been validated.
6. Start the game normally through Steam with your headset connected. Use the left controller's **Menu** button for Start. In gameplay, **A** switches VR / Third Person. Face forward in your normal playing position and **hold Y** to recenter.
7. Start with the **Tanker** chapter. Read the controls and limitations before playing.

The ZIP includes `dg_hook.asi`, `openxr_loader.dll`, `winmm.dll`, `dg_hook.on`, `dg_present.on`, `dg_xr.on`, and documentation/licenses. It includes no game executable, game assets, saves, source files or debugging symbols.

### Updating or uninstalling

Close the game before changing files. Back up your existing mod files and settings before updating. To disable this mod, move `dg_hook.asi` out of the game folder. For full removal, remove the files supplied by this ZIP and restore your backups. Remove shared loaders only if no other mod uses them. Your game saves are not part of this package.

## Controls

![Quest 3 controller mappings](media/quest3-controls.png)

The illustration shows the core controls. In this build, **Codec additionally requires the weapon hand near the corresponding ear**. The table covers contextual actions for the **default right-handed** layout.

| Input | Action |
| --- | --- |
| Left stick | Move; up/down on ladders when engaged |
| Right stick | Turn; camera zoom when using the photo camera |
| Right **A** | Switch VR / Third Person during gameplay |
| Right **B** | Posture / crouch action |
| Left **X** | Native interaction: doors, handles and contextual actions; hold where the interaction requires it |
| Hold left **Y** | Recenter view and tracking reference |
| Left **Menu** | Start / pause |
| Right trigger | Weapon action; pistols follow their aim / fire / release behavior |
| Left trigger | Melee / contextual choke action |
| Left grip | Contextual grab / hold |
| Right grip, with the right hand near the right ear | Codec; move the hand into position before pressing the grip |
| Right trigger + **B** | Aim calibration; posture is suppressed during this chord |
| Right stick click | Weapon selection radial |
| Left stick click | Item selection radial |

**Radial selection:** start with the stick centered and triggers released. Hold the relevant stick click, move that stick toward the entry, and press the **same controller's trigger** to confirm. Releasing the stick click while an entry is highlighted also selects it. The opposite trigger cancels. Release the controls and return the stick to center before opening another selection. Release the selection trigger before using the weapon.

**Menus:** use the right stick to navigate, **A or right trigger** to confirm, and **B** to go back. Gameplay actions depend on context and are not all available in menus or scripted scenes.

**Mod settings (keyboard):** with the game focused during gameplay, press **Home** to open or close the settings panel. Use **Up/Down** to select View (stereo/mono), VR Pistol Reload or M9 Slide; use **Enter** or **Left/Right** to toggle an available option. After closing, release the buttons and center both sticks to resume control.

**Photo camera:** begin at minimum zoom and let it settle briefly before zooming. Its zoom is controlled with the right stick. Security-camera traversal remains a separate limitation.

## AER and performance

**AER means Alternate Eye Rendering.** The mod renders the left and right eye on successive game frames. This is also described as alternate-frame stereo (AFR).

The two eyes are therefore not freshly rendered together from one simulation frame. At 60 game frames per second, this means roughly **30 fresh images per eye per second**. A higher headset refresh rate or compositor reprojection does not increase the number of fresh game-rendered images. Motion can reveal temporal differences between the eyes, ghosting or judder.

This beta does not claim native simultaneous stereo. Keep the game frame rate stable and use settings your PC can sustain. AER comfort varies between players.

## Roadmap: Raiden and Plant

Future work includes, among other things:

- Raiden's **SOCOM** mapping.
- **Coolant** validation and remaining interaction fixes; this build includes the current implementation and diagnostic work.
- **Scope** support.
- **Sniper rifle** support.
- **Stinger** controller alignment, lock-on and firing validation; an experimental adapter is now included. **Nikita** remains future work.
- Hanging, movement while hanging, and grenade support.
- Further Plant campaign testing and polish.

There is no release date for complete Raiden support yet.

## Troubleshooting and feedback

- **No VR image:** check that your headset is connected to PC VR, the intended OpenXR runtime is active, and the ZIP was extracted beside the game executable. Check that the `.on` files did not gain an extra `.txt` extension.
- **View offset or incorrect facing:** face forward and hold **Y** to recenter.
- **Blocked by a security-camera section:** press **A** to use Third Person.
- **Session stops after about an hour:** this beta retains the tested 3600-second session setting. Save and restart the game, or increase `seconds` in `dg_hook.on` with the game closed.
- **Unexpected behavior with other mods:** reproduce with this beta as the only gameplay mod before reporting.

Open an [issue](https://github.com/Shiffo0/MGS2-PCVR-Beta/issues) with the mod build, game version, headset, PC VR connection/runtime, GPU, chapter, exact location and steps to reproduce. State whether it happens in VR, Third Person or both. Review logs before sharing them; do not upload game files or personal data.

## Credits and distribution

Created by **Shiffo0**. Unofficial fan project; not affiliated with or endorsed by Konami or Meta. You must own the game.

The mod implementation is available in [src/](https://github.com/Shiffo0/MGS2-PCVR-Beta/tree/main/src), with build instructions. Playable downloads are provided separately through Releases.

The original mod code and accompanying documentation are licensed under the **[MIT License](LICENSE)**. This grant covers the contributors' own work only. The game, game assets and trademarks are not covered. Third-party components retain their respective licenses; their notices are included in the release package.

- [Ultimate ASI Loader](https://github.com/ThirteenAG/Ultimate-ASI-Loader) by ThirteenAG — MIT license; notice included in the download.
- [OpenXR SDK / loader](https://github.com/KhronosGroup/OpenXR-SDK) by Khronos and contributors — license included in the download.

Controller reference artwork was generated with AI and checked against the selected build's mappings. Use the written control table for contextual details.
