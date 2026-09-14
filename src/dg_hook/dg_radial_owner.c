#include "dg_radial_owner.h"
#include <string.h>
#include <math.h>
static int axis_ok(float v) { return v >= -1.0f && v <= 1.0f; }
static int neutral(const dg_radial_owner_input *in, int h) {
    double x=in->select.x[h], y=in->select.y[h];

    return h==0 ? sqrt(x*x+y*y) <= in->move_deadzone :
        (x<0 ? -x : x) <= in->turn_deadzone;
}
void dg_radial_owner_init(dg_radial_owner *s) {
    memset(s, 0, sizeof(*s)); dg_radial_init(&s->select);
    s->triggers = 3u; s->weapon_hand = -1;
}
dg_radial_owner_output dg_radial_owner_step(dg_radial_owner *s,
                                          const dg_radial_owner_input *in) {
    dg_radial_owner_output out;
    dg_radial_input input;
    unsigned old_axes;
    int h, valid, changed, was_open;
    memset(&out, 0, sizeof(out)); out.selection.hand = out.selection.slot = -1;
    if (!s) return out;
    valid = in && in->have_sample && in->select.safe &&
        in->weapon_hand >= 0 && in->weapon_hand <= 1 &&
        in->move_deadzone >= 0 && in->move_deadzone <= 0.9 &&
        in->turn_deadzone >= 0 && in->turn_deadzone <= 0.9 &&
        in->sample_ms <= in->now_ms && in->now_ms-in->sample_ms <= 100u;
    if (valid) for (h=0; h<2; ++h)
        if (!axis_ok(in->select.x[h]) || !axis_ok(in->select.y[h])) valid=0;
    if (valid && s->seen && (in->now_ms < s->now_ms || in->sequence < s->sequence)) valid=0;
    /* Equip eligibility changes while carrying a body. With no radial
     * gesture or axis claim, that catalog update owns no gameplay input.
     * Invalidate menu selection only; preserve grip/fire/movement routing.
     * Actual epoch/context/hand/deadzone changes still reset below. */
    if (valid && s->seen && s->select.hand < 0 && !s->axes &&
        in->select.version != s->version) {
        dg_radial_init(&s->select);
        s->version = in->select.version;
    }
    changed = valid && s->seen && (in->select.epoch != s->epoch ||
        in->select.context != s->context || in->select.version != s->version ||
        in->weapon_hand != s->weapon_hand ||
        in->move_deadzone != s->move_deadzone || in->turn_deadzone != s->turn_deadzone);
    if (!valid || changed) {
        dg_radial_init(&s->select); s->triggers=3u;
        /* Unknown axes cannot release an existing claim. */
        if (in) for (h=0; h<2; ++h)
            if (in->select.primary[h]) s->axes |= 1u << h;
        if (valid) {
            s->epoch=in->select.epoch; s->context=in->select.context;
            s->version=in->select.version; s->weapon_hand=in->weapon_hand;
            s->sequence=in->sequence; s->now_ms=in->now_ms;
            s->move_deadzone=in->move_deadzone; s->turn_deadzone=in->turn_deadzone;
        }
        goto result;
    }
    /* Safety/context validation precedes duplicate suppression. A duplicate
     * produces neither an edge nor a neutral observation nor an intent. */
    if (s->seen && in->sequence == s->sequence) goto result;
    s->seen=1; s->sequence=in->sequence; s->now_ms=in->now_ms;
    s->epoch=in->select.epoch; s->context=in->select.context;
    s->version=in->select.version; s->weapon_hand=in->weapon_hand;
    s->move_deadzone=in->move_deadzone; s->turn_deadzone=in->turn_deadzone;
    old_axes=s->axes; was_open=s->select.hand >= 0;
    for (h=0; h<2; ++h) {
        if (!in->select.trigger[h]) s->triggers &= ~(1u << h);
        if (in->select.primary[h]) s->axes |= 1u << h;
        if ((old_axes & (1u << h)) && !in->select.primary[h] && neutral(in,h))
            s->axes &= ~(1u << h);
        if (was_open && in->select.trigger[h]) s->triggers |= 1u << h;
    }
    input=in->select;
    input.now_ms=in->now_ms;
    /* Existing fire owns its safe completion. Never synthesize its release.
     * A closing axis claim prevents another opening until fresh neutral. */
    if (!in->fire_idle || (s->triggers &&
        !(was_open && s->triggers==(1u<<s->select.hand) && input.trigger[s->select.hand])) ||
        (!was_open && old_axes)) input.safe=0;
    if (!in->selection_allowed) input.safe=0;
    if (!was_open) {
        if (!s->select.armed && (!neutral(in,0) || !neutral(in,1))) input.safe=0;
        for (h=0;h<2;++h) if (input.primary[h] && !neutral(in,h)) input.safe=0;
    }
    out.selection=dg_radial_step(&s->select,&input);
    if (out.selection.intent)
        out.equip_kind=out.selection.hand==in->weapon_hand ? 1 : 2;
    out.route_valid=1;
    out.sequence=in->sequence; out.epoch=in->select.epoch;
    out.context=in->select.context; out.version=in->select.version;
    out.move_deadzone=in->move_deadzone; out.turn_deadzone=in->turn_deadzone;
    out.weapon_hand=in->weapon_hand;
    if (!((old_axes | s->axes) & 1u)) { out.move_x=input.x[0]; out.move_y=input.y[0]; }
    if (!((old_axes | s->axes) & 2u)) out.turn_x=input.x[1];
result:
    out.consume_axes=s->axes;
    out.consume_clicks=3u; /* dedicated mapping, including rejected gestures */
    out.deny_new_fire=s->triggers | (s->select.hand >= 0 ? 3u : 0u);
    return out;
}
