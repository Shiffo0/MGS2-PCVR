/* Proximity latch. Time is OpenXR nanoseconds, never camera invocation count. */
#ifndef DG_TWOHAND_H
#define DG_TWOHAND_H
#include <string.h>
#include <math.h>
#include "dg_grip_blend.h"
/* Grip origins do not coincide when the physical controller shells touch.
   The recorded close gesture bottomed out at 126 mm, outside the old 120 mm
   gate. Keep a distinct release boundary to avoid chatter at attachment. */
#define DG_TWOHAND_ENTER_M 0.18
#define DG_TWOHAND_EXIT_M  0.24
#define DG_TWOHAND_DWELL_NS 50000000LL
#define DG_TWOHAND_BLEND_S 0.075
/* Long weapons: the distance is the support hand to the predicted foregrip,
   not controller to controller (they sit 25-45 cm apart on a rifle). */
#define DG_TWOHAND_RIFLE_ENTER_M 0.10
#define DG_TWOHAND_RIFLE_EXIT_M  0.15
typedef struct {
    unsigned long long seq;
    long long time, near_since;
    int engaged;
    double blend;   /* visible weight: dg_grip_blend_output(phase) */
    double phase;   /* time-linear progress of the shared grip blend */
} DG_TWOHAND;
static void dg_twohand_reset(DG_TWOHAND *s) { memset(s, 0, sizeof *s); }
/* `times` NULL = the latch's original transition (75 ms linear).
   enter_m/exit_m: the attach / release distances (pistols: DG_TWOHAND_ENTER_M/EXIT_M). */
static double dg_twohand_step_gate(DG_TWOHAND *s, int valid, int aiming,
                              double distance_m, unsigned long long seq,
                              long long time, const DG_GRIP_TIMES *times,
                              double enter_m, double exit_m)
{
    static const DG_GRIP_TIMES legacy = {DG_GRIP_LEGACY_SUPPORT_MS, DG_GRIP_LEGACY_SUPPORT_MS, 1};
    double dt;
    if (!valid || !aiming || !isfinite(distance_m) || distance_m < 0 ||
        !seq || time <= 0 || (s->seq && (seq < s->seq || time < s->time))) {
        dg_twohand_reset(s); return 0;
    }
    if (distance_m >= exit_m) {
        s->engaged = 0; s->near_since = 0;
    }
    if (seq == s->seq || time == s->time) return s->blend;
    dt = s->time ? (double)(time - s->time) * 1e-9 : 0;
    if (dt > 0.1) { dg_twohand_reset(s); dt = 0; }
    s->seq = seq; s->time = time;
    if (!s->engaged) {
        if (distance_m <= enter_m) {
            if (!s->near_since) s->near_since = time;
            if (time - s->near_since >= DG_TWOHAND_DWELL_NS) s->engaged = 1;
        } else s->near_since = 0;
    }
    if (!times) times = &legacy;
    s->phase = dg_grip_blend_advance(s->phase, s->engaged, dt, times);
    s->blend = dg_grip_blend_output(s->phase, times);
    return s->blend;
}
static double dg_twohand_step(DG_TWOHAND *s, int valid, int aiming,
                              double distance_m, unsigned long long seq,
                              long long time, const DG_GRIP_TIMES *times)
{
    return dg_twohand_step_gate(s, valid, aiming, distance_m, seq, time, times,
                                DG_TWOHAND_ENTER_M, DG_TWOHAND_EXIT_M);
}
#endif
