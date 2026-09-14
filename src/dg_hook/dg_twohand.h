/* Proximity latch. Time is OpenXR nanoseconds, never camera invocation count. */
#ifndef DG_TWOHAND_H
#define DG_TWOHAND_H
#include <string.h>
#include <math.h>
/* Grip origins do not coincide when the physical controller shells touch.
   The recorded close gesture bottomed out at 126 mm, outside the old 120 mm
   gate. Keep a distinct release boundary to avoid chatter at attachment. */
#define DG_TWOHAND_ENTER_M 0.18
#define DG_TWOHAND_EXIT_M  0.24
#define DG_TWOHAND_DWELL_NS 50000000LL
#define DG_TWOHAND_BLEND_S 0.075
typedef struct {
    unsigned long long seq;
    long long time, near_since;
    int engaged;
    double blend;
} DG_TWOHAND;
static void dg_twohand_reset(DG_TWOHAND *s) { memset(s, 0, sizeof *s); }
static double dg_twohand_step(DG_TWOHAND *s, int valid, int aiming,
                              double distance_m, unsigned long long seq,
                              long long time)
{
    double dt;
    if (!valid || !aiming || !isfinite(distance_m) || distance_m < 0 ||
        !seq || time <= 0 || (s->seq && (seq < s->seq || time < s->time))) {
        dg_twohand_reset(s); return 0;
    }
    if (distance_m >= DG_TWOHAND_EXIT_M) {
        s->engaged = 0; s->near_since = 0;
    }
    if (seq == s->seq || time == s->time) return s->blend;
    dt = s->time ? (double)(time - s->time) * 1e-9 : 0;
    if (dt > 0.1) { dg_twohand_reset(s); dt = 0; }
    s->seq = seq; s->time = time;
    if (!s->engaged) {
        if (distance_m <= DG_TWOHAND_ENTER_M) {
            if (!s->near_since) s->near_since = time;
            if (time - s->near_since >= DG_TWOHAND_DWELL_NS) s->engaged = 1;
        } else s->near_since = 0;
    }
    s->blend += (s->engaged ? 1 : -1) * dt / DG_TWOHAND_BLEND_S;
    if (s->blend < 0) s->blend = 0;
    if (s->blend > 1) s->blend = 1;
    return s->blend;
}
#endif
