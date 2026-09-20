# MGS2 PCVR — SteamVR support candidate and unlimited sessions

Version `v0.3.2-beta.3-rc.1`, build `4F95FE0F`, based on public beta `479DF688`.

This is a test candidate for issue #2, not a confirmed fix for SteamVR's gameplay Waiting screen. The supplied log shows different resolutions across separate runtime startups, not an observed resize inside one startup.

## Changes

- `seconds=0` disables the automatic session timer. The supplied configuration now uses zero. Positive values retain timed sessions. When preserving older settings, change seconds after installing this binary; older binaries do not support unlimited mode.
- Projection swapchains rebuild using one synchronized size snapshot, including when Present changes size during creation. Partial creation is cleaned up before retrying.
- Up to 31 three-line OpenXR capture reports, spaced at least ten seconds apart, expose producer refusals, copy validation, quad/projection/zero-layer counters and frame-end results. Heavy GPU probes remain disabled. The normal process log cap still applies.

## Installation and test

Download `MGS2-PCVR-v0.3.2-beta.3-rc.1-4F95FE0F.zip`. Close the game and back up existing mod files. Existing beta users can replace only `dg_hook.asi` and change `seconds=0` in their existing `dg_hook.on`; loaders and other settings are unchanged.

For issue #2: start a fresh game process, reach affected Tanker gameplay promptly, wait 15 seconds, open codec for 15 seconds, close it and wait another 15 seconds. Exit the game, review the log for personal information and attach the new `dg_hook.log` to the issue. Report what the headset shows at each step. Do not change resolution or runtime during this first test.

## Verification and limitations

- Release profile/timer: 39 checks passed.
- Release GPU: 11 checks passed.
- Production resize lifecycle with WARP textures and mocked OpenXR: 48 checks passed, including resize recovery, a size change during creation, failed second-eye creation, retry, stereo copy/release and support-log rate/quantity bounds.
- Shipping build succeeded. Existing compiler warnings remain.
- No physical-headset acceptance or hour-long live test is claimed. Automated timer boundary tests cover cutoff logic.

Security-camera sections and experimental hanging may require Third Person. Grenades and a complete Plant/Raiden VR campaign remain unsupported. AER updates eyes on alternating game frames.

Binary SHA256: `4F95FE0F114CA314BCA1FAD41E6BB725B1901408F9C80399C80F197C81523AD4`.
