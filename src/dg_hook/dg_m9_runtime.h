#ifndef DG_M9_RUNTIME_H
#define DG_M9_RUNTIME_H
#include "dg_m9_slide.h"
typedef struct {
    uint64_t source, sequence;
    int valid, allowed, grip, trigger;
    double local[3], units_per_metre;
    double origin[3], aim_q[4]; /* raw LOCAL right grip and aim, same sample */
} DG_M9_SAMPLE;
typedef struct {
    uint64_t stamp, sequence;
    int valid, allowed, state, grip;
    double origin[3], aim_q[4], anchor[3], left[3];
} DG_GRIP_DEBUG;
int dg_bridge_grip_debug_snapshot(DG_GRIP_DEBUG *out);
/* Feedback consumed by the XR control producer; no OpenXR calls on game thread. */
unsigned dg_bridge_m9_events(void);
#endif
