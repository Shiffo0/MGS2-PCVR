# MGS2 PCVR v0.3.4

Playable on PS VR 2 / SteamVR.

Includes the persistent hand-alignment changes from candidate **0237AC10**:

- Automatic defaults position the hands from controller tracking.
- **Right trigger + B while unarmed** saves a one-time wrist-alignment override for both hands. Hold both hands in a comfortable neutral pose with both controllers tracked. A later deliberate calibration replaces the override.
- The override persists across game restarts in `dg_hand_calibration.bin`.
- Weapon changes, animations, first/third-person switches, new levels, cutscenes and tracking loss preserve the calibration/default.

Native animations can temporarily take control of the hands, and reach limits still apply. Headset testing remains necessary to confirm the visible result across those transitions.

**Known regression: this release breaks entering lockers from first-person (FPS) mode.**

Retains the v0.3.3 SteamVR gameplay Waiting fix, unlimited sessions (`seconds=0`) and synchronized projection-swapchain resize recovery with partial-creation cleanup.

Development measurements and recordings are compiled out of the player build; bounded support logging remains. Source downloads contain the corresponding mod source, excluding original game source and source references. The playable ZIP contains no source code.

Build: **0237AC10**. SHA256: `0237AC10E1BE3136A565945E87C156CC938F5ED995F1B421BA5218F382D2C7EE`.
