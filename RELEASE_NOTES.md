# Public Beta 3 — build 5AC0F9C8

Updated playable package and matching buildable mod source for MGS2 Master Collection on Windows/OpenXR, using AER (Alternate Eye Rendering).

## Changes

- Radar and LIFE follow the rendered forearm, with the radar corner aligned near the wrist and its long axis pointing toward the elbow.
- Wrist-look activation tolerates a more relaxed viewing angle while retaining visibility hysteresis and rejecting the back of the panel. This uses head orientation, not eye tracking.
- M9 support-hand contact follows the slide during manipulation and blends back on release.
- Experimental hanging camera, heading and first-person arm-visibility fixes.
- Menu confirmation and recovery improvements, including movement after getting up.
- Includes the preceding HF Blade, Stinger, coolant, pistol-reload, native HUD and controller work.
- All 177 shipping C/C++ and local include files are included in src/. Matching release settings are in config/; recording and diagnostic probes are disabled.

## Validation and limitations

The packaged binary matches the installed build and the selected artifact byte for byte. All 323 entries in the artifact source manifest were verified. Recorded checks for this baseline passed: radar gaze (87 checks), WARP runtime harness (1199 checks), main suite, IK suite and recorder (9/9). The curated source also builds successfully; existing f2l compiler warnings remain.

**Quest 3 is the development and testing headset. A complete headset regression test for this exact build has not been recorded.** The earlier Tanker beta was reported fully playable with workarounds; that does not establish acceptance of every new feature. Radar comfort, hanging behavior and the expanded weapon interactions require further live validation. Plant/Raiden is not yet supported as a complete VR campaign.

Use Third Person to get past security-camera sections. Hanging and movement while hanging remain experimental. Grenades remain unsupported. Raiden work still includes SOCOM, coolant, scope, sniper rifle and Stinger validation, plus Nikita support.

## Download and installation

Download **MGS2-PCVR-v0.3.0-beta.3-5AC0F9C8.zip** and follow the [installation guide](https://github.com/Shiffo0/MGS2-PCVR-Beta#installation). GitHub's automatic Source code archives are repository snapshots, not playable packages.

The unchanged release dg_hook.asi has SHA256:

`5AC0F9C8A55DC5EDDFC7588884FB99F196B21868768A0E427E5B442D7DA963CE`

Source curation changes documentation comments and three optional diagnostic path literals only. A rebuild is a separate artifact and is not claimed byte-identical to the release binary. MIT covers contributors' own work; third-party notices remain included. No game files, saves or debug symbols are bundled.
