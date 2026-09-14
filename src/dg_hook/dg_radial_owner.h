#ifndef DG_RADIAL_OWNER_H
#define DG_RADIAL_OWNER_H
#include "dg_radial.h"
/* CPU-only, single serialized caller. Never feeds game pad/fire directly.
 * Physical channels: 0 left, 1 right. primary[] is stick-click HOLD.
 * The caller versions every catalog/hand/context change and supplies coherent
 * timestamps. safe includes gameplay, native takeover and action validity;
 * selection_allowed adds presentation/catalog/engine admission availability.
 * Pending equip therefore need not invalidate safe routing. Not just FPS safe.
 * A/FPS and item-use have no fields here and retain their separate owners.
 * Caller must supply actual effective downstream deadzones (finite 0..0.9).
 * move_deadzone gates left sqrt(x*x+y*y); turn_deadzone gates right abs(x).
 * Right y has no downstream consumer. Threshold equality is neutral, as in
 * dg_move.c and dg_hook.c; inputs are floats promoted to double, without epsilon.
 * Current caller defaults are both 150/1000.0. Config changes cancel ownership
 * gestures before deduplication. The frame releasing an axis claim outputs 0.
 */
typedef struct dg_radial_owner_input {
    dg_radial_input select;
    uint64_t sequence, now_ms, sample_ms;
    int have_sample, fire_idle, weapon_hand;
    int selection_allowed; /* admission gate only; 0 does not invalidate routing */
    double move_deadzone, turn_deadzone;
} dg_radial_owner_input;
typedef struct dg_radial_owner {
    dg_radial_state select;
    uint64_t sequence, now_ms, epoch, context, version;
    unsigned axes, triggers;
    int seen, weapon_hand;
    double move_deadzone, turn_deadzone;
} dg_radial_owner;
typedef struct dg_radial_owner_output {
    dg_radial_result selection;
    int equip_kind; /* 0 none, 1 weapon, 2 item; never use */
    unsigned consume_axes, consume_clicks, deny_new_fire;
    float move_x, move_y, turn_x;
    int route_valid;
    uint64_t sequence, epoch, context, version; /* valid only with route_valid */
    double move_deadzone, turn_deadzone;
    int weapon_hand;
} dg_radial_owner_output;
void dg_radial_owner_init(dg_radial_owner *s);
dg_radial_owner_output dg_radial_owner_step(dg_radial_owner *s,
                                          const dg_radial_owner_input *in);
#endif
