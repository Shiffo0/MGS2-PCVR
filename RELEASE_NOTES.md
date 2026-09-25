# MGS2 PCVR v0.4.0

## Huge performance increase in VR

The mod checked memory validity with a Windows system call before almost every read of game state. In VR stereo those checks consumed most of the game thread's time, which kept the game below 60 frames per second even though the GPU was largely idle. The mod now remembers recent answers for a few milliseconds and falls back to a full check whenever an answer is missing, negative or expired.

On the maintainer's PC (Meta Quest 3, 4K render resolution, stereo), the playtest build of this code went from 43–56 fps with frame spikes of 33–50 ms to a steady 60 fps. Flat and mono modes also benefit. Results on other PCs will vary.

## Other changes since v0.3.5

- **PSG-1 / PSG-1T:** in the first-person scope, hold the **right grip** to zoom in and the **left grip** to zoom out; zoom continues while the grip is held. The shot stays aimed with your head.
- **Nikita:** the launcher sits in your VR hand. Hold the **right grip** to open the native Nikita sight and fire with the trigger, then steer the missile with the left stick. Firing no longer leaves the trigger or the menu blocked afterwards.
- **Stinger:** the launcher stays in your hand instead of opening in the zoomed view. Holding the **right grip** opens the sight. The sight and lock-on have not been verified in the headset yet.
- **Thermal goggles (experimental):** with the goggles in your inventory, press the left grip with your left hand near your left ear to toggle them. This has not been validated in the headset.
- **Fire handling:** a trigger action the game does not accept is now abandoned after a short time instead of blocking later input.
- **Player build and mod source:** development measurements, recording and weapon diagnostics are excluded. Comments that named internal game functions have been removed from the mod source.

Retains v0.3.5 Game Over recovery, HUD/cutscene handling, persistent wrist alignment, SteamVR support and unlimited sessions. AER still alternates eyes.

## Installation and limitations

Extract the mod ZIP beside the game executable with the game closed. Back up your existing mod files first. Keep `dg_hand_calibration.bin` to preserve your wrist alignment.

The Tanker chapter remains the focus. Plant/Raiden, hanging and grenades retain the limitations described in the README. A full campaign regression is not claimed.

## Build

The maintainer played and accepted this gameplay code on a playtest build (05F4F9F6): PSG-1 zoom, Nikita arm, firing, steering and menu, the Stinger in hand, and 60 fps in stereo. This release is **AEF2EC93**, a new binary rebuilt from that source after removing the development code. The rebuilt binary itself has not been tested in a headset.

The release-profile, XR runtime, HUD/cutscene and GPU checks passed on the published source. Private test harnesses and development results are not included.

Binary SHA256: `AEF2EC9304365DF292263C5041B10BDA6AC9EBA36884AF65BD72BF3DFE9D8A23`.
