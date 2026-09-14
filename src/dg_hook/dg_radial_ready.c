#include "dg_radial_ready.h"
#include <math.h>
static int axis_ok(float v) { return v >= -1.0f && v <= 1.0f; }
int dg_radial_controls_equip_ready(const dg_radial_owner_input *in,
                                  const dg_radial_owner_output *out) {
    int h, left_neutral, right_neutral, routed_left_neutral, routed_right_neutral;
    if (!in || !out || !out->route_valid || !in->have_sample || !in->select.safe ||
        !in->fire_idle || in->weapon_hand<0 || in->weapon_hand>1 ||
        in->sample_ms>in->now_ms || in->now_ms-in->sample_ms>100u ||
        !(in->move_deadzone>=0.0 && in->move_deadzone<=0.9) ||
        !(in->turn_deadzone>=0.0 && in->turn_deadzone<=0.9)) return 0;
    if (out->sequence!=in->sequence || out->epoch!=in->select.epoch ||
        out->context!=in->select.context || out->version!=in->select.version ||
        out->weapon_hand!=in->weapon_hand || out->move_deadzone!=in->move_deadzone ||
        out->turn_deadzone!=in->turn_deadzone) return 0;
    for(h=0; h<2; ++h) {
        int confirmation=out->selection.intent && out->selection.trigger_confirm &&
            out->selection.hand==h && (out->consume_axes&(1u<<h)) &&
            (out->deny_new_fire&(1u<<h));
        if ((!confirmation && (in->select.primary[h] || in->select.trigger[h])) ||
            !axis_ok(in->select.x[h]) || !axis_ok(in->select.y[h])) return 0;
    }
    if (!axis_ok(out->move_x) || !axis_ok(out->move_y) || !axis_ok(out->turn_x)) return 0;
    left_neutral=sqrt((double)in->select.x[0]*in->select.x[0] +
        (double)in->select.y[0]*in->select.y[0]) <= in->move_deadzone;
    right_neutral=in->select.x[1]>=-in->turn_deadzone && in->select.x[1]<=in->turn_deadzone;
    routed_left_neutral=sqrt((double)out->move_x*out->move_x+
        (double)out->move_y*out->move_y)<=in->move_deadzone;
    routed_right_neutral=out->turn_x>=-in->turn_deadzone && out->turn_x<=in->turn_deadzone;
    if (!(left_neutral && routed_left_neutral) &&
        !((out->consume_axes&1u) && out->move_x==0 && out->move_y==0)) return 0;
    if (!(right_neutral && routed_right_neutral) &&
        !((out->consume_axes&2u) && out->turn_x==0)) return 0;
    return 1;
}
