#ifndef DG_REACH_CENTRE_H
#define DG_REACH_CENTRE_H
#include <math.h>

#define DG_REACH_CENTRE_TAU_MS   3500.0
#define DG_REACH_CENTRE_JUMP_MM  30.0
#define DG_REACH_CENTRE_GAP_MS   500u

typedef struct {
    double c[3];
    int valid;
    unsigned long long ms;
    unsigned long long weapon;
    unsigned long resets, uses;
} DG_REACH_CENTRE;

static int dg_reach_centre_step(DG_REACH_CENTRE *s, const double live[3], unsigned long long now_ms,
                                unsigned long long weapon, int free_hand, double out[3])
{
    double d[3], dist, a;
    int k, reset;
    if (!s || !live || !out || !isfinite(live[0]) || !isfinite(live[1]) || !isfinite(live[2])) return 0;
    for (k = 0; k < 3; k++) d[k] = live[k] - s->c[k];
    dist = sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    reset = !s->valid || s->weapon != weapon || now_ms < s->ms || now_ms - s->ms > DG_REACH_CENTRE_GAP_MS ||
            !(dist <= DG_REACH_CENTRE_JUMP_MM);
    if (reset) {
        for (k = 0; k < 3; k++) s->c[k] = live[k];
        s->valid = 1; s->weapon = weapon; s->resets++;
    } else {
        a = 1.0 - exp(-(double)(now_ms - s->ms) / DG_REACH_CENTRE_TAU_MS);
        for (k = 0; k < 3; k++) s->c[k] += d[k] * a;
    }
    s->ms = now_ms;
    if (!free_hand) return 0;
    for (k = 0; k < 3; k++) out[k] = s->c[k];
    s->uses++;
    return 1;
}

static void dg_reach_centre_rotate(const double q[4], const double v[3], double o[3])
{
    double t[3] = {2*(q[1]*v[2]-q[2]*v[1]), 2*(q[2]*v[0]-q[0]*v[2]), 2*(q[0]*v[1]-q[1]*v[0])};
    o[0] = v[0] + q[3]*t[0] + q[1]*t[2] - q[2]*t[1];
    o[1] = v[1] + q[3]*t[1] + q[2]*t[0] - q[0]*t[2];
    o[2] = v[2] + q[3]*t[2] + q[0]*t[1] - q[1]*t[0];
}
static int dg_reach_centre_tracking(DG_REACH_CENTRE *s, const double shoulder_world[3], const double origin[3],
                                    const double q[4], unsigned long long now_ms, unsigned long long weapon,
                                    int free_hand, double centre_world[3])
{
    double d[3], rel[3], c[3], back[3], qi[4];
    int k;
    if (!shoulder_world || !origin || !q || !centre_world) return 0;
    for (k = 0; k < 4; k++) if (!isfinite(q[k])) return 0;
    if (fabs(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3] - 1.0) > 1e-6) return 0;
    qi[0] = -q[0]; qi[1] = -q[1]; qi[2] = -q[2]; qi[3] = q[3];
    for (k = 0; k < 3; k++) d[k] = shoulder_world[k] - origin[k];
    dg_reach_centre_rotate(qi, d, rel);
    if (!dg_reach_centre_step(s, rel, now_ms, weapon, free_hand, c)) return 0;
    dg_reach_centre_rotate(q, c, back);
    for (k = 0; k < 3; k++) { centre_world[k] = origin[k] + back[k]; if (!isfinite(centre_world[k])) return 0; }
    return 1;
}
#endif
