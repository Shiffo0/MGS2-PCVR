#ifndef DG_RADIAL_H
#define DG_RADIAL_H
#include <stdint.h>
/* Pure selection, called by the central game-tick control producer.
 * Caller supplies a fresh coherent snapshot on EVERY step (including release).
 * safe includes focus/session, pause/menu/codec/cutscene and input ownership.
 * epoch changes on lifecycle, context changes on player/stage, version changes
 * on catalog ordering/content changes. IDs are resolved by caller only after
 * revalidating the returned tokens at the eventual game-side commit seam.
 * hand 0/1 are binding-independent channels; the adapter maps catalogs/hands.
 * Selection uses one 0.10 radial threshold, without hysteresis.
 * primary is a logical hold, mapped upstream to thumbstick_click, not A.
 * Opening requires selected channel's stick magnitude below 0.35;
 * a deflected opening cancels until release and a neutral rearm sample.
 * No engine reads or writes.
 */
typedef struct dg_radial_input {
    int safe, primary[2], trigger[2];
    uint64_t now_ms;
    unsigned quick_ms; /* 0 preserves immediate radial-only behavior */
    float x[2], y[2];
    uint64_t epoch, context, version;
    unsigned count[2], eligible[2]; /* 6..8 slots, bit per eligible slot */
} dg_radial_input;
typedef struct dg_radial_state {
    int armed, hand, selected; /* hand -1 = closed; init requires neutral */
    uint64_t epoch, context, version;
    unsigned count;
    uint64_t opened_ms;
    unsigned quick_ms;
    int deflected;
    int suppress_release; /* a trigger-confirmed gesture closes without re-equipping */
} dg_radial_state;
typedef struct dg_radial_result {
    int intent, hand, slot; /* intent is a single-call pulse, never a write */
    int quick; /* neutral short release: slot is -1, caller resolves toggle */
    int trigger_confirm; /* open menu's own trigger; never a gameplay press */
    uint64_t epoch, context, version;
} dg_radial_result;
void dg_radial_init(dg_radial_state *s);
dg_radial_result dg_radial_step(dg_radial_state *s, const dg_radial_input *in);
#endif
