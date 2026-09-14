#include "dg_radial_commit.h"
#include <string.h>
static void observe_tick(dg_radial_commit_state *s, uint64_t seq) {
    if (!s->seen_tick || seq>s->last_tick_seq) s->last_tick_seq=seq;
    s->seen_tick=1;
}
void dg_radial_commit_init(dg_radial_commit_state *s) {
    if (s) memset(s, 0, sizeof(*s));
}
int dg_radial_commit_offer(dg_radial_commit_state *s,
    const dg_radial_commit_intent *intent, uint64_t now_ms) {
    if (!s || !intent || !intent->seq || intent->seq <= s->last_seq) return 0;
    s->last_seq=intent->seq;
    if (s->phase != DG_RADIAL_COMMIT_IDLE || intent->id < 0 ||
        (intent->kind != DG_RADIAL_EQUIP_WEAPON && intent->kind != DG_RADIAL_EQUIP_ITEM)) return 0;
    s->pending=*intent; s->offered_ms=now_ms;
    s->phase=DG_RADIAL_COMMIT_PENDING;
    return 1;
}
dg_radial_commit_result dg_radial_commit_step(dg_radial_commit_state *s,
    const dg_radial_commit_tick *t) {
    dg_radial_commit_result r;
    int valid, stale_tick;
    memset(&r, 0, sizeof(r));
    if (!s) return r;
    if (!t) {
        if (s->phase==DG_RADIAL_COMMIT_AWAIT_ACK) s->phase=DG_RADIAL_COMMIT_BLOCKED;
        else if (s->phase==DG_RADIAL_COMMIT_PENDING) s->phase=DG_RADIAL_COMMIT_IDLE;
        return r;
    }
    stale_tick=s->seen_tick && t->tick_seq<=s->last_tick_seq;
    if (s->phase!=DG_RADIAL_COMMIT_PENDING && s->phase!=DG_RADIAL_COMMIT_AWAIT_ACK) {
        observe_tick(s,t->tick_seq);
        return r;
    }
    valid=t->observation_safe && t->epoch==s->pending.epoch &&
        t->context==s->pending.context && t->version==s->pending.version;
    if (s->phase==DG_RADIAL_COMMIT_PENDING) {
        s->phase=DG_RADIAL_COMMIT_IDLE; /* refusal consumes, never retries */
        observe_tick(s,t->tick_seq);
        if (!valid || !t->all_validation_ready || t->now_ms<s->offered_ms || t->now_ms-s->offered_ms>100) return r;
        if (stale_tick) return r; /* spent, not deferred to another tick */
        r.propose_write=1; r.intent=s->pending;
        s->proposed_ms=s->last_tick_ms=t->now_ms;
        s->proposal_tick_seq=s->last_tick_seq=t->tick_seq;
        s->phase=DG_RADIAL_COMMIT_AWAIT_ACK;
        return r;
    }
    if (!valid || t->now_ms<s->last_tick_ms || t->now_ms-s->proposed_ms>100) {
        s->phase=DG_RADIAL_COMMIT_BLOCKED;
        return r;
    }
    if (t->tick_seq<s->last_tick_seq) {
        s->phase=DG_RADIAL_COMMIT_BLOCKED;
        return r;
    }
    s->last_tick_ms=t->now_ms;
    if (t->tick_seq==s->last_tick_seq) return r;
    s->last_tick_seq=t->tick_seq;
    if ((s->pending.kind==DG_RADIAL_EQUIP_WEAPON ? t->observed_weapon : t->observed_item)==s->pending.id) {
        r.acknowledged=1; r.intent=s->pending;
        s->phase=DG_RADIAL_COMMIT_IDLE;
    }
    return r;
}
