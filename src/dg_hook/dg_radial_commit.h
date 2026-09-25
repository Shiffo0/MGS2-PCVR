#ifndef DG_RADIAL_COMMIT_H
#define DG_RADIAL_COMMIT_H
#include <stdint.h>
/* Single-thread CPU model. No engine access, rollback, retry or item use.
 * Caller serializes calls and provides monotonic millisecond timestamps.
 * tick_seq identifies game ticks and must increase for a subsequent tick.
 * Duplicate tick calls cannot acknowledge; decreasing tick_seq blocks while
 * awaiting acknowledgement. Safety/token/deadline checks precede deduplication.
 * all_validation_ready means ALL retail, safety, eligibility and ownership
 * checks passed freshly at this tick; this module cannot establish that fact.
 * observation_safe means fresh safe context/player and readable actor identity;
 * it applies before proposal and throughout acknowledgement. Engine readiness
 * (all_validation_ready) applies only before proposal, since normal equip may
 * temporarily make the engine unavailable while acknowledgement is pending.
 * An emitted proposal is treated as possibly written, even if the caller fails
 * to apply it. Only a subsequent tick observing the desired actor acknowledges
 * it. Observation is not proof of causality (the actor may already match).
 * Context/safety loss, clock regression or >100ms without acknowledgement
 * after proposal latches BLOCKED until explicit init/session recreation.
 * Nonnegative IDs; zero is native NONE/unequip. No item-use operation.
 */
enum dg_radial_commit_kind { DG_RADIAL_EQUIP_WEAPON=1, DG_RADIAL_EQUIP_ITEM=2 };
enum dg_radial_commit_phase { DG_RADIAL_COMMIT_IDLE, DG_RADIAL_COMMIT_PENDING,
    DG_RADIAL_COMMIT_AWAIT_ACK, DG_RADIAL_COMMIT_BLOCKED };
typedef struct dg_radial_commit_intent {
    int kind, id;
    int thermal_toggle, expected_item; /* additional native toggle revalidation */
    uint64_t epoch, context, version, seq;
} dg_radial_commit_intent;
typedef struct dg_radial_commit_state {
    int phase, seen_tick;
    uint64_t last_seq, offered_ms, proposed_ms, last_tick_ms;
    uint64_t last_tick_seq, proposal_tick_seq;
    dg_radial_commit_intent pending;
} dg_radial_commit_state;
typedef struct dg_radial_commit_tick {
    uint64_t epoch, context, version, now_ms, tick_seq;
    int observation_safe, all_validation_ready;
    int observed_weapon, observed_item;
} dg_radial_commit_tick;
typedef struct dg_radial_commit_result {
    int propose_write, acknowledged;
    dg_radial_commit_intent intent;
} dg_radial_commit_result;
void dg_radial_commit_init(dg_radial_commit_state *s);
/* Every new positive seq is spent even if invalid/busy; busy is never queued.
 * seq zero and non-increasing seq are rejected until explicit init. */
int dg_radial_commit_offer(dg_radial_commit_state *s,
    const dg_radial_commit_intent *intent, uint64_t now_ms);
dg_radial_commit_result dg_radial_commit_step(dg_radial_commit_state *s,
    const dg_radial_commit_tick *tick);
#endif
