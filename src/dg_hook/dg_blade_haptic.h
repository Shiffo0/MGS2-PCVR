#ifndef DG_BLADE_HAPTIC_H
#define DG_BLADE_HAPTIC_H
#include <stdint.h>
#define DG_BLADE_HAPTIC_HIT     1u
#define DG_BLADE_HAPTIC_DEFLECT 2u
#define DG_BLADE_HAPTIC_LEFT    1
#define DG_BLADE_HAPTIC_RIGHT   2
#define DG_BLADE_HAPTIC_MAX_AGE_MS 100   /* older events are dropped, like the M9 queue */
typedef struct { int hands; float amplitude; int64_t duration_ns; } DG_BLADE_PULSE;
/* A deflect wins over a hit in the same drain: it is the rarer, stronger
   message (both hands, longer, full strength); a hit is a short right-hand tick. */
static int dg_blade_haptic_plan(unsigned events, DG_BLADE_PULSE *p) {
    if(!p)return 0;
    p->hands=0;p->amplitude=0;p->duration_ns=0;
    if(events & DG_BLADE_HAPTIC_DEFLECT) {
        p->hands=DG_BLADE_HAPTIC_LEFT|DG_BLADE_HAPTIC_RIGHT;p->amplitude=1.0f;p->duration_ns=90000000;
        return 1;
    }
    if(events & DG_BLADE_HAPTIC_HIT) {
        p->hands=DG_BLADE_HAPTIC_RIGHT;p->amplitude=0.6f;p->duration_ns=35000000;
        return 1;
    }
    return 0;
}
#endif
