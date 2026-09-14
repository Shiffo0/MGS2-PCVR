/* Native button forwarding, separate from the optional target/gesture policy.
 * The game decides what ACTION/PUNCH/CAPTURE mean in its current state. */
#ifndef DG_INTERACT_ADAPTER_H
#define DG_INTERACT_ADAPTER_H
#include <stdint.h>
enum { DG_IA_ACTION=1, DG_IA_POSTURE=2, DG_IA_MELEE=4, DG_IA_CAPTURE=8,
       DG_IA_CODEC=16, DG_IA_UP=32, DG_IA_DOWN=64,
       DG_IA_PEEP_LEFT=128,DG_IA_PEEP_RIGHT=256,DG_IA_ALL=511 };
enum { DG_IA_LADDER_CONTEXT=3,DG_IA_BEYOND_CONTEXT=5,DG_IA_LOCKER_CONTEXT=6 };
typedef struct {
    uint64_t sample, epoch, choke_seq;
    unsigned age_ms, levels, suppressed;
    int valid, ladder, special;
} DG_INTERACT_SAMPLE;
typedef struct {
    DG_INTERACT_SAMPLE input;
    uint64_t tick, player_identity;
    unsigned native_status, native_press, native_release, capture_mask;
    unsigned native_peep_left,native_peep_right;
    int safe, unarmed;
} DG_INTERACT_NATIVE_INPUT;
typedef struct {
    uint64_t epoch, sample, tick, choke_seq, player_identity;
    unsigned held, blocked, raw;
    int seen,special;
} DG_INTERACT_ADAPTER;
typedef struct { unsigned status, press, release; int capture_pressure, codec_press; }
    DG_INTERACT_NATIVE_OUTPUT;
#endif
