# MGS2 PCVR v0.3.5

## Changes since v0.3.4

- **Game Over controls:** the displayed Continue screen accepts controller navigation and confirmation even when a scripted failure leaves cutscene flags set. The preceding death sequence stays blocked, and queued input is discarded when the screen closes. Addresses [issue #4](https://github.com/Shiffo0/MGS2-PCVR/issues/4).
- **HUD and cutscenes:** VR HUD placement stops during cutscenes and stale gameplay states, restores the original GPU constants, and resumes when gameplay returns. The texture filter preserves eligible third-person HUD elements while excluding the scene image.
- **Coolant spray:** a held trigger reaches the native spray handler after the game's pad processing; release and stale input cancel the handoff.
- **Coolant and directional microphone:** standing movement and independent turning are supported, with corrected standing camera height and shoulder anchoring. Crouch, prone and scripted microphone actions retain their native behavior.
- **USP/SOCOM reload:** reload and mobile-tool motion now share their hook so one feature no longer prevents the other from installing. Native ammunition bookkeeping and reload timing are preserved.
- **Hand attachment:** improves selection of verified rigid weapon attachments, including the HF Blade, when the draw matrix lags behind the tracked hand.
- **Player build and mod source:** development recording, measurement output and optional GPU probes are excluded. The corresponding mod source includes the shipping dependency closure, without the removed measurement branches, unused measurement helpers or private source-reference comments. Bounded startup/error logging remains.

Retains v0.3.4 persistent wrist alignment, SteamVR support, unlimited sessions and swapchain resize recovery. AER still alternates eyes; this release does not claim to fix all stereo ghosting or animation-speed problems.

## Installation and limitations

Extract the mod ZIP beside the game executable with the game closed. Back up your existing mod files first. Keep `dg_hand_calibration.bin` to preserve your wrist alignment.

The Tanker chapter remains the focus. Plant/Raiden, hanging and grenades retain the limitations described in the README. The previously reported first-person locker-entry problem remains unverified; use Third Person if affected. A full campaign regression and headset acceptance for this rebuilt release are not claimed.

## Build

This release is rebuilt from the 64B10AA2 gameplay source used by the 30D7CA3C playtest build, with publication-source cleanup. It is a new binary, **1007DD0B**, not the previously installed 30D7CA3C.

Release-profile, XR and GPU/HUD checks passed. Private test harnesses and development results are not included in the download or mod-source tree.

Binary SHA256: `1007DD0B6574DA6B8329F256BCB7EA17C293CA821741BB64BA6A7E4EDF73462C`.
