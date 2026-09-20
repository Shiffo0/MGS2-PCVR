# Beta 3 player-build candidate — 479DF688

Based on Beta 3 (5AC0F9C8), with heavy diagnostics excluded at compile time. **Draft: awaiting a short Quest 3 regression test.** This build has not been installed into the game folder or tested in a headset.

## Changed

- Release builds default to `DG_ENABLE_DIAGNOSTICS=0`. Old probe, recorder, dump and debug settings cannot activate measurements, including mixed-case keys.
- Flight recording, aim observation, eye/near captures, pixel probes, phase/pair traces, HUD memory watches and diagnostic GPU draw trials are disabled at their entry points.
- Diagnostic GPU query/copy hooks and the experimental MGSHDFix measurement adapter are not installed. Normal draw forwarding, HUD, wrist radar and AER remain enabled.
- Diagnostic recording buffers are reduced; disabled measurement implementations are removed by compilation/linking. The release binary is 715264 bytes, versus 901120 bytes for the corresponding diagnostic build. This is not a measured FPS improvement.
- File logging is limited to matching startup/error messages, capped at 256 lines per process. Repeated detailed capture and bridge summaries are excluded.
- `src/build.bat diagnostic` explicitly enables development measurements and produces `dg_hook-diagnostic.asi`; the default command produces `dg_hook.asi`.

## Validation

- Published release tests: 30 profile checks and 11 GPU checks passed. Old opt-in settings are ignored; diagnostic entry points are inert; query/copy slots stay unchanged and the gameplay shader hook remains installed.
- Release XR/WARP harness: 1075 checks passed, including rejection of the pixel-probe environment flag, inactive near capture, production image copies, frame submission, LIFE and radar paths.
- Release HUD/radar WARP harness: 5263 checks passed, including inactive copy tracing.
- Development main suite and IK suite passed; recorder 9/9; development XR/WARP 1199 checks; development HUD/radar 5263 checks passed.
- Both binary profiles compile. Existing f2l warnings C4013/C4142 remain.
- The source and package are screened for protected references and accidental development artifacts. The final download is verified by SHA256.

## Quest 3 check before publishing

Check game startup, VR/Third Person switching, AER view, movement and turning, recenter, menus/radials, LIFE and wrist radar, M9 slide/pistol reload, and a short Tanker segment. Confirm no measurement files are created when old diagnostic settings/markers are present. Keep the existing Beta 3 available until this check passes.

Quest 3 remains the development headset. Security-camera traversal requires Third Person; hanging remains experimental; grenades and the complete Plant/Raiden campaign remain unsupported.

## Package

`MGS2-PCVR-v0.3.1-beta.3-479DF688.zip` is the playable candidate. Extract beside the game executable with the game closed, following the README installation steps. GitHub's automatic Source code archives are not playable packages.

`dg_hook.asi` SHA256: `479DF688C67DE8C1008DB055AF7936E08C1295583EAA614F1FB52DB2450A3544`

MIT covers contributors' own work. Third-party notices remain included. No game files, saves, debugging symbols or measurement output are bundled.
