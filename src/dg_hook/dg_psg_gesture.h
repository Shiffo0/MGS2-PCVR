#ifndef DG_PSG_GESTURE_H
#define DG_PSG_GESTURE_H
#include <math.h>

typedef struct { double open_m, close_m, open_deg, close_deg; } DG_PSG_GESTURE_CFG;
typedef struct {
    int have, open;
    double last_m, last_deg;     /* last valid sample */
    double min_m, min_deg;       /* session minima (smallest seen) */
    unsigned long opens, closes;
} DG_PSG_GESTURE;
enum { DG_PSG_GESTURE_INVALID = 0, DG_PSG_GESTURE_HIP = 1, DG_PSG_GESTURE_SCOPE = 2 };

static void dg_psg_gesture_defaults(DG_PSG_GESTURE_CFG *c)
{
    c->open_m = 0.20; c->close_m = 0.30; c->open_deg = 30.0; c->close_deg = 45.0;
}
/* Whole centimetres/degrees from the marker: 5..60 cm open, open..100 cm
   close, 5..90 deg open, open..120 deg close. */
static int dg_psg_gesture_cfg_set(DG_PSG_GESTURE_CFG *c, int open_cm, int close_cm,
                                  int open_deg, int close_deg)
{
    if (open_cm < 5 || open_cm > 60 || close_cm < open_cm || close_cm > 100 ||
        open_deg < 5 || open_deg > 90 || close_deg < open_deg || close_deg > 120) return 0;
    c->open_m = open_cm / 100.0; c->close_m = close_cm / 100.0;
    c->open_deg = open_deg; c->close_deg = close_deg;
    return 1;
}
/* OpenXR -Z (forward) of a unit quaternion (x, y, z, w). */
static void dg_psg_forward(const double q[4], double f[3])
{
    double x = q[0], y = q[1], z = q[2], w = q[3];
    f[0] = -2.0 * (x * z + w * y);
    f[1] = -2.0 * (y * z - w * x);
    f[2] = -(1.0 - 2.0 * (x * x + y * y));
}
static int dg_psg_unit_quat(const double q[4])
{
    double n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    return n >= 0.99 && n <= 1.01;   /* positive test also refuses NaN */
}
static int dg_psg_gesture_step(DG_PSG_GESTURE *s, const DG_PSG_GESTURE_CFG *c, int valid,
                               const double head_p[3], const double head_q[4],
                               const double hand_p[3], const double aim_q[4])
{
    double hf[3], af[3], d[3], m, dot, deg;
    int i;
    if (!valid || !s || !c || !head_p || !head_q || !hand_p || !aim_q ||
        !dg_psg_unit_quat(head_q) || !dg_psg_unit_quat(aim_q)) return DG_PSG_GESTURE_INVALID;
    for (i = 0; i < 3; i++) {
        d[i] = hand_p[i] - head_p[i];
        if (!(d[i] >= -100.0 && d[i] <= 100.0)) return DG_PSG_GESTURE_INVALID;
    }
    m = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    dg_psg_forward(head_q, hf); dg_psg_forward(aim_q, af);
    dot = hf[0] * af[0] + hf[1] * af[1] + hf[2] * af[2];
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;
    deg = acos(dot) * (180.0 / 3.14159265358979323846);
    if (!s->have) { s->have = 1; s->open = 0; s->min_m = m; s->min_deg = deg; }
    if (m < s->min_m) s->min_m = m;
    if (deg < s->min_deg) s->min_deg = deg;
    s->last_m = m; s->last_deg = deg;
    if (s->open) {
        if (m > c->close_m || deg > c->close_deg) { s->open = 0; s->closes++; }
    } else if (m <= c->open_m && deg <= c->open_deg) { s->open = 1; s->opens++; }
    return s->open ? DG_PSG_GESTURE_SCOPE : DG_PSG_GESTURE_HIP;
}
#endif
