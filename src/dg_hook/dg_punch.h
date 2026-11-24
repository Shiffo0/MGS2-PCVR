/* Punch gesture: a fast jab of either hand along the view, without a button.
 * Pure: no game memory, no OpenXR. Tracking-space metres, sample time in ns.
 *
 * Recipe (taken over from the author's companion VR mod): a free hand away
 * from the other hand and the head, grip open, pushed forward faster than a
 * threshold, re-armed only after it slowed down again. New here: the hand
 * holding a weapon may punch too, behind stricter gates against the moves of
 * aiming and shooting (see dg_punch_gate).
 *
 * A stroke starts when the forward speed passes DG_PUNCH_START; it fires once,
 * as soon as the peak and the forward travel since the start both reach the
 * hand's threshold, so the fist is still moving forward when the hit line is
 * laid. It re-arms below DG_PUNCH_REARM. Two calls with the same sample (both
 * eyes of one pair) advance nothing. */
#ifndef DG_PUNCH_H
#define DG_PUNCH_H
#include <stdint.h>
#include <string.h>
#include <math.h>

#define DG_PUNCH_START        0.8    /* m/s forward: a stroke begins */
#define DG_PUNCH_REARM        1.0    /* m/s forward: below this a hand is armed again */
#define DG_PUNCH_FREE_SPEED   2.0    /* free hand */
#define DG_PUNCH_FREE_TRAVEL  0.15   /* m forward since the stroke began */
#define DG_PUNCH_WEAPON_SPEED 2.5    /* the hand holding a weapon */
#define DG_PUNCH_WEAPON_TRAVEL 0.20
#define DG_PUNCH_OTHER_HAND   0.25   /* m: closer to the other hand = holding something together */
#define DG_PUNCH_HEAD         0.22   /* m: closer to the head = not a jab */
#define DG_PUNCH_AIM_HOLD_MS  300    /* weapon hand: quiet after aiming/firing ends */
#define DG_PUNCH_SWAP_MS      300    /* both hands: quiet after a weapon change */
#define DG_PUNCH_GAP_NS       100000000LL  /* a longer sample gap restarts the hand */

enum { DG_PUNCH_LEFT = 0, DG_PUNCH_RIGHT = 1 };
enum {
    DG_PUNCH_R_FIRED = 0,
    DG_PUNCH_R_NOT_ALLOWED,   /* not first person / menu / radial / codec */
    DG_PUNCH_R_GRIP,          /* free hand with the grip closed */
    DG_PUNCH_R_HEAD,
    DG_PUNCH_R_OTHER_HAND,
    DG_PUNCH_R_AIMING,        /* weapon hand while aiming/firing or just after */
    DG_PUNCH_R_LATCHED,       /* two-hand latch: both hands hold the weapon */
    DG_PUNCH_R_WEAPON,        /* blade (its own recogniser), PSG, Stinger, Nikita */
    DG_PUNCH_R_SWAP,          /* just changed weapon */
    DG_PUNCH_R_COUNT
};

typedef struct {
    uint64_t source, sequence;
    int64_t time_ns;
    uint64_t now_ms;
    int allowed;                 /* VR first person, gameplay, no menu/radial/codec */
    int head_valid;
    double head[3], head_forward[3];
    int hand_valid[2];
    double hand[2][3];
    int grip[2];                 /* squeeze held */
    int weapon_hand;             /* hand holding the weapon: DG_PUNCH_RIGHT, or -1 unarmed */
    int weapon;
    int aiming;                  /* the fire machine is drawing/holding/firing */
    int latched;                 /* the support hand is on the weapon */
    int input_refused;
} DG_PUNCH_SAMPLE;

typedef struct {
    int have, armed, in_stroke;
    int64_t t;
    double p[3], start[3], peak;
} DG_PUNCH_HAND;

typedef struct {
    uint64_t source, sequence;
    int weapon, have_weapon;
    uint64_t aim_until_ms, swap_until_ms;
    DG_PUNCH_HAND h[2];
    unsigned long fired[2], refused[DG_PUNCH_R_COUNT];
} DG_PUNCH_STATE;

typedef struct {
    int hand;                    /* -1 none */
    int reason;                  /* DG_PUNCH_R_FIRED or why a full-speed stroke was refused */
    double speed, travel;        /* peak forward m/s, forward m */
} DG_PUNCH_EVENT;

static void dg_punch_reset(DG_PUNCH_STATE *s) { memset(s, 0, sizeof *s); }

static int dg_punch_weapon_excluded(int weapon)
{
    return weapon == 13 || weapon == 4 || weapon == 19 || weapon == 7 || weapon == 6;
}

static double dg_punch_dist(const double a[3], const double b[3])
{
    double d[3] = {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    return sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

/* Why hand `h` may not punch now (DG_PUNCH_R_FIRED = it may). */
static int dg_punch_gate(const DG_PUNCH_STATE *s, const DG_PUNCH_SAMPLE *in, int h)
{
    int armed_hand = in->weapon_hand == h;
    if (!in->allowed) return DG_PUNCH_R_NOT_ALLOWED;
    if (in->now_ms < s->swap_until_ms) return DG_PUNCH_R_SWAP;
    if (in->weapon_hand >= 0 && dg_punch_weapon_excluded(in->weapon)) return DG_PUNCH_R_WEAPON;
    if (in->latched) return DG_PUNCH_R_LATCHED;
    if (armed_hand && (in->aiming || in->now_ms < s->aim_until_ms)) return DG_PUNCH_R_AIMING;

    if (in->input_refused) return DG_PUNCH_R_NOT_ALLOWED;
    if (!armed_hand && in->grip[h]) return DG_PUNCH_R_GRIP;
    if (in->head_valid && dg_punch_dist(in->hand[h], in->head) < DG_PUNCH_HEAD) return DG_PUNCH_R_HEAD;
    if (in->hand_valid[!h] && dg_punch_dist(in->hand[h], in->hand[!h]) < DG_PUNCH_OTHER_HAND)
        return DG_PUNCH_R_OTHER_HAND;
    return DG_PUNCH_R_FIRED;
}

/* One sample. Returns 1 and fills *ev when a stroke reached its threshold in
   this sample (ev->reason FIRED = punch; anything else = a full-speed stroke
   that a gate refused, for counting false punches). One event per call at
   most; the right hand is judged first. */
static int dg_punch_step(DG_PUNCH_STATE *s, const DG_PUNCH_SAMPLE *in, DG_PUNCH_EVENT *ev)
{
    double fwd[3], n;
    int h, k, out = 0;
    ev->hand = -1; ev->reason = DG_PUNCH_R_FIRED; ev->speed = ev->travel = 0;
    if (!in->source || !in->sequence || in->time_ns <= 0 ||
        (s->source && in->source != s->source)) {
        uint64_t src = in->source;
        dg_punch_reset(s); s->source = src;
        return 0;
    }
    if (in->sequence == s->sequence) return 0;           /* the other eye of this pair */
    s->source = in->source; s->sequence = in->sequence;
    /* Aiming ends: the weapon hand stays quiet a moment (recoil, lowering). */
    if (in->aiming) s->aim_until_ms = in->now_ms + DG_PUNCH_AIM_HOLD_MS;
    if (s->have_weapon && in->weapon != s->weapon) {
        s->swap_until_ms = in->now_ms + DG_PUNCH_SWAP_MS;
        memset(s->h, 0, sizeof s->h);
    }
    s->weapon = in->weapon; s->have_weapon = 1;
    if (!in->head_valid) { memset(s->h, 0, sizeof s->h); return 0; }
    n = sqrt(in->head_forward[0] * in->head_forward[0] + in->head_forward[1] * in->head_forward[1] +
             in->head_forward[2] * in->head_forward[2]);
    if (!(n > 0.5) || !isfinite(n)) { memset(s->h, 0, sizeof s->h); return 0; }
    for (k = 0; k < 3; k++) fwd[k] = in->head_forward[k] / n;
    for (h = DG_PUNCH_RIGHT; h >= DG_PUNCH_LEFT; h--) {
        DG_PUNCH_HAND *g = &s->h[h];
        double v, travel, speed_need, travel_need;
        int64_t dt;
        if (!in->hand_valid[h] || !isfinite(in->hand[h][0]) || !isfinite(in->hand[h][1]) ||
            !isfinite(in->hand[h][2])) { memset(g, 0, sizeof *g); continue; }
        if (!g->have || (dt = in->time_ns - g->t) <= 0 || dt > DG_PUNCH_GAP_NS) {
            memset(g, 0, sizeof *g);
            g->have = 1; g->t = in->time_ns; memcpy(g->p, in->hand[h], sizeof g->p);
            continue;
        }
        v = ((in->hand[h][0] - g->p[0]) * fwd[0] + (in->hand[h][1] - g->p[1]) * fwd[1] +
             (in->hand[h][2] - g->p[2]) * fwd[2]) / ((double)dt * 1e-9);
        g->t = in->time_ns; memcpy(g->p, in->hand[h], sizeof g->p);
        if (v < DG_PUNCH_REARM) g->armed = 1;
        if (!g->in_stroke) {
            if (v > DG_PUNCH_START && g->armed) {
                g->in_stroke = 1; g->peak = v;
                memcpy(g->start, in->hand[h], sizeof g->start);   /* travel counts from here */
            }
            continue;
        }
        if (v < DG_PUNCH_START) { g->in_stroke = 0; g->peak = 0; continue; }
        if (v > g->peak) g->peak = v;
        travel = (in->hand[h][0] - g->start[0]) * fwd[0] + (in->hand[h][1] - g->start[1]) * fwd[1] +
                 (in->hand[h][2] - g->start[2]) * fwd[2];
        speed_need = in->weapon_hand == h ? DG_PUNCH_WEAPON_SPEED : DG_PUNCH_FREE_SPEED;
        travel_need = in->weapon_hand == h ? DG_PUNCH_WEAPON_TRAVEL : DG_PUNCH_FREE_TRAVEL;
        if (g->peak >= speed_need && travel >= travel_need) {
            int why = dg_punch_gate(s, in, h);
            g->in_stroke = 0; g->armed = 0;                /* one event per stroke */
            if (!out) {
                ev->hand = h; ev->reason = why; ev->speed = g->peak; ev->travel = travel; out = 1;
            }
            if (why == DG_PUNCH_R_FIRED) s->fired[h]++; else s->refused[why]++;
        }
    }
    return out;
}

static const char *dg_punch_reason_name(int r)
{
    static const char *n[DG_PUNCH_R_COUNT] = {"fired", "not-allowed", "grip", "head", "other-hand",
                                              "aiming", "latched", "weapon", "weapon-change"};
    return r >= 0 && r < DG_PUNCH_R_COUNT ? n[r] : "?";
}
#endif
