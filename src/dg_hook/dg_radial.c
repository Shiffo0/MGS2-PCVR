#include "dg_radial.h"
#include <math.h>
#include <float.h>
#include <string.h>
static int finite_axis(float f) { return f >= -1.0f && f <= 1.0f; }
static void cancel(dg_radial_state *s) {
    s->armed = 0; s->hand = -1; s->selected = -1;
}
void dg_radial_init(dg_radial_state *s) {
    memset(s, 0, sizeof(*s)); cancel(s);
}
static int sector(float x, float y, unsigned n) {
    double a;
    if ((double)x*x + (double)y*y < 0.10*0.10) return -1;
    a = atan2((double)x, (double)y); /* up = 0, clockwise */
    if (a < 0) a += 6.2831853071795864769;
    return ((int)floor(a*n/6.2831853071795864769 + 0.5)) % (int)n;
}
dg_radial_result dg_radial_step(dg_radial_state *s, const dg_radial_input *in) {
    dg_radial_result r;
    int h, i;
    memset(&r, 0, sizeof(r)); r.hand = r.slot = -1;
    if (!s || !in) { if (s) cancel(s); return r; }
    if (!in->safe || ((in->trigger[0] || in->trigger[1]) &&
        (s->hand<0 || in->trigger[1-s->hand])) ||
        (in->primary[0] && in->primary[1])) { cancel(s); return r; }
    for (i=0; i<2; ++i) {
        if (!finite_axis(in->x[i]) || !finite_axis(in->y[i])) {
            cancel(s); return r;
        }
    }
    if (!s->armed) {
        if (!in->primary[0] && !in->primary[1]) {
            s->armed = 1;
            s->epoch = in->epoch; s->context = in->context;
            s->version = in->version;
        }
        return r;
    }
    if (in->epoch != s->epoch || in->context != s->context ||
        in->version != s->version) { cancel(s); return r; }
    if (s->hand < 0) {
        if (!in->primary[0] && !in->primary[1]) return r;
        h = in->primary[1] ? 1 : 0;
        if (in->count[h] < 6 || in->count[h] > 8) { cancel(s); return r; }
        if ((double)in->x[h]*in->x[h] + (double)in->y[h]*in->y[h] >= 0.35*0.35) {
            cancel(s); return r;
        }
        s->hand = h; s->epoch = in->epoch; s->context = in->context;
        s->version = in->version; s->count = in->count[h];
        s->opened_ms = in->now_ms; s->quick_ms = in->quick_ms;
        s->deflected = 0;
        s->suppress_release = 0;
    }
    h = s->hand;
    if (in->epoch != s->epoch || in->context != s->context ||
        in->version != s->version || in->count[h] != s->count ||
        in->primary[1-h]) { cancel(s); return r; }
    if (in->now_ms < s->opened_ms || in->quick_ms != s->quick_ms) {
        cancel(s); return r;
    }
    s->selected = sector(in->x[h], in->y[h], s->count);
    if (s->selected >= 0) s->deflected = 1;
    if (!in->primary[h] || in->trigger[h]) {
        if(!in->primary[h] && s->suppress_release) {cancel(s);return r;}
        if (s->selected >= 0 && (in->eligible[h] & (1u << s->selected))) {
            r.intent = 1; r.hand = h; r.slot = s->selected;
            r.trigger_confirm = in->trigger[h]!=0;
            r.epoch = s->epoch; r.context = s->context; r.version = s->version;
        }
        if (!in->trigger[h] && !s->deflected && s->quick_ms &&
            in->now_ms - s->opened_ms < s->quick_ms) {
            r.intent = r.quick = 1; r.hand = h; r.slot = -1;
            r.epoch = s->epoch; r.context = s->context; r.version = s->version;
        }
        cancel(s); /* explicit neutral observation before another open */
    }
    return r;
}
