# Public Beta 2 — build E4267D3E

Updated mod binary, buildable source baseline and configuration for MGS2 Master Collection on Windows/OpenXR.

## Included

- Combined HF Blade, Stinger, native HUD and coolant work.
- Stinger first-person model/controller integration and aim-directed lock-on adapter; native animation, ammunition, damage and missile guidance are retained.
- M9 slide handling, pistol reload and wrist-radar/UI settings from the selected baseline.
- Full shipping implementation and local include dependencies in src/, including the C++ draw-trial module.
- Matching release settings in config/; recording, diagnostic probes and capture requests are disabled for the package.

## Validation and limitations

The selected baseline has recorded passes for the main, IK, recorder, Stinger (56 checks), Blade (470 checks), aim-capture (2408 checks) and aim-target desk suites. Its source manifest and binary hash were verified before packaging. The curated source copy also builds successfully; existing f2l compiler warnings remain.

**A new headset acceptance test for this combined build has not been recorded.** Quest 3/Tanker playability was reported for the earlier beta; it is not a completed regression test for E4267D3E. Stinger alignment/lock-on/firing, HF Blade gestures, regular weapons/reload, wrist HUD and coolant still need live validation.

Use Third Person to get past security-camera sections. Hanging, movement while hanging and grenades remain listed limitations. Raiden/Plant is not yet a complete supported VR campaign.

## Download

Download **MGS2-PCVR-v0.2.0-beta.2-E4267D3E.zip** for installation. GitHub's Source code archives contain the source snapshot for this release, not the playable package. See the [installation guide](https://github.com/Shiffo0/MGS2-PCVR-Beta#installation).

The packaged dg_hook.asi is unchanged:

E4267D3E8D856B1563229444020B718C49FA4596B0934A7403E33C9BAD1FEAD7

The curated source differs only in documentation comments and three optional diagnostic path strings; a rebuild is a separate artifact. MIT applies to the contributors' own work, with third-party licenses retained separately. No game files, saves or debug symbols are bundled.
