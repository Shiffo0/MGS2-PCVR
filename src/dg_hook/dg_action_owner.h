#ifndef DG_ACTION_OWNER_H
#define DG_ACTION_OWNER_H
#include <stdint.h>
/* ACTION has one writer: pre-mask NORMAL status. Native code owns edges. */
typedef struct {
    uint64_t sample, source, actor, now_ms;
    unsigned age_ms;
    int valid, down, denied, shutter_down, shutter_denied;
    float zoom_y;
    int zoom_active;
} DG_ACTION_SAMPLE;
typedef struct {
    DG_ACTION_SAMPLE physical;
    int context; /* 0 closed, 1 normal, 2 hatch turning, 3 hatch opening */
    int pad_allowed, native_down;
} DG_ACTION_INPUT;
typedef struct {
    uint64_t sample,source,actor,time;
    int seen, raw, blocked, held, context;
} DG_ACTION_OWNER;
static unsigned dg_action_step(DG_ACTION_OWNER *s,const DG_ACTION_INPUT *in) {
    const DG_ACTION_SAMPLE *p=&in->physical;
    int fresh=!s->seen || p->sample>s->sample;
    int identity=s->seen && (s->source!=p->source || s->actor!=p->actor);
    int valid=p->valid && p->sample && p->source && p->actor && p->age_ms<=100 &&
        (!s->seen || (p->now_ms>=s->time && (identity || p->sample>=s->sample))) &&
        (!s->seen || identity || p->sample!=s->sample || p->down==s->raw);
    int transfer=s->held && s->context==1 && in->context==2 && !identity;
    if (!s->seen || identity) {s->blocked=1;s->held=0;fresh=1;}
    if (s->context!=in->context && !transfer) {s->blocked=1;s->held=0;}
    if (!valid || !in->pad_allowed || p->denied || in->native_down ||
        (in->context!=1 && in->context!=2)) {s->blocked=1;s->held=0;}
    else {
        if (fresh && !p->down) s->blocked=0;
        s->held=p->down && !s->blocked && (fresh || s->held);
    }

    if (!s->seen || identity || p->sample>s->sample) s->sample=p->sample;
    if (p->now_ms>s->time) s->time=p->now_ms;
    s->source=p->source;s->actor=p->actor;s->seen=1;s->raw=p->down;
    s->context=in->context;
    return s->held?0x10u:0u;
}
#endif
