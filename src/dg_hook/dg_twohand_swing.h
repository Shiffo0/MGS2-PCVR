/* Support-hand steering for long weapons. Pure: no game memory, no OpenXR.
 *
 * Recipe (taken over from the author's companion VR mod, which re-implemented
 * the geometric contract of a published VR mod; no code was copied): while
 * the support hand holds the foregrip, turn the weapon about the right-wrist
 * pivot by the SHORTEST arc that points the wrist->foregrip direction at the
 * support hand. Only a rotation: the pivot does not move, nothing is scaled,
 * and roll about the barrel is not added (the arc axis is perpendicular to
 * both directions). The swing is a hand-LOCAL rotation, always computed from
 * the unswung hand, so it never accumulates over frames; the weight fades it
 * in and out with the grip blend. A lever outside the band is refused rather
 * than inventing a direction.
 *
 * Quaternions are [x, y, z, w], Hamilton product, active rotation
 * (world = hand * local), as everywhere in this mod. Lengths in game units
 * (1 unit = 1 mm). */
#ifndef DG_TWOHAND_SWING_H
#define DG_TWOHAND_SWING_H
#include <math.h>
#include <string.h>

#define DG_SWING_LEVER_MIN_MM 100.0   /* wrist -> foregrip, and wrist -> support hand */
#define DG_SWING_LEVER_MAX_MM 700.0
#define DG_SWING_REACH_MAX_MM 850.0   /* support hand may sit up to the latch exit beyond */
#define DG_SWING_MAX_DEG      75.0    /* a hand this far off the line is not on the foregrip */

static void dg_swing_qmul(const double a[4], const double b[4], double out[4])
{
    double r[4];
    r[0] = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    r[1] = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    r[2] = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    r[3] = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    memcpy(out, r, sizeof r);
}
static void dg_swing_rotate(const double q[4], const double v[3], double out[3])
{
    double cx = q[1] * v[2] - q[2] * v[1], cy = q[2] * v[0] - q[0] * v[2], cz = q[0] * v[1] - q[1] * v[0];
    double ccx = q[1] * cz - q[2] * cy, ccy = q[2] * cx - q[0] * cz, ccz = q[0] * cy - q[1] * cx, r[3];
    r[0] = v[0] + 2.0 * (q[3] * cx + ccx);
    r[1] = v[1] + 2.0 * (q[3] * cy + ccy);
    r[2] = v[2] + 2.0 * (q[3] * cz + ccz);
    memcpy(out, r, sizeof r);
}
static int dg_swing_finite(const double *v, int n)
{
    int i;
    for (i = 0; i < n; i++) if (!isfinite(v[i])) return 0;
    return 1;
}

/* Shortest arc taking direction a onto direction b (lengths ignored).
   0 for degenerate or (nearly) opposite directions: there is no unique arc. */
static int dg_swing_between(const double a[3], const double b[3], double q[4])
{
    double la, lb, c[3], d, n;
    if (!dg_swing_finite(a, 3) || !dg_swing_finite(b, 3)) return 0;
    la = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    lb = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (!(la > 1e-9) || !(lb > 1e-9)) return 0;
    d = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb);
    if (d < -0.999) return 0;
    c[0] = (a[1] * b[2] - a[2] * b[1]) / (la * lb);
    c[1] = (a[2] * b[0] - a[0] * b[2]) / (la * lb);
    c[2] = (a[0] * b[1] - a[1] * b[0]) / (la * lb);
    q[0] = c[0]; q[1] = c[1]; q[2] = c[2]; q[3] = 1.0 + d;
    n = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!(n > 1e-12)) return 0;
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    return 1;
}

/* The local swing L for an UNSWUNG hand orientation `hand` (world), the
   foregrip offset in that hand's frame (`anchor`, game units) and the vector
   from the wrist to the support hand in world (`to_support`). Afterwards
   rotate(hand * L, anchor) points along to_support. Returns 0 (and identity)
   for a lever outside the band, a degenerate input or too large an angle. */
static int dg_twohand_swing_local(const double hand[4], const double anchor[3],
                                  const double to_support[3], double L[4])
{
    double inv[4], b[3], la, lb, q[4];
    L[0] = L[1] = L[2] = 0; L[3] = 1;
    if (!dg_swing_finite(hand, 4) || !dg_swing_finite(anchor, 3) || !dg_swing_finite(to_support, 3)) return 0;
    la = sqrt(anchor[0] * anchor[0] + anchor[1] * anchor[1] + anchor[2] * anchor[2]);
    lb = sqrt(to_support[0] * to_support[0] + to_support[1] * to_support[1] + to_support[2] * to_support[2]);
    if (la < DG_SWING_LEVER_MIN_MM || la >= DG_SWING_LEVER_MAX_MM ||
        lb < DG_SWING_LEVER_MIN_MM || lb >= DG_SWING_REACH_MAX_MM) return 0;
    if (fabs(hand[0] * hand[0] + hand[1] * hand[1] + hand[2] * hand[2] + hand[3] * hand[3] - 1) > 1e-6) return 0;
    inv[0] = -hand[0]; inv[1] = -hand[1]; inv[2] = -hand[2]; inv[3] = hand[3];
    dg_swing_rotate(inv, to_support, b);
    if (!dg_swing_between(anchor, b, q)) return 0;
    if (2.0 * acos(fmin(1.0, fabs(q[3]))) > DG_SWING_MAX_DEG * 3.14159265358979323846 / 180.0) return 0;
    memcpy(L, q, sizeof q);
    return 1;
}

/* slerp(identity, L, w): exact identity at w <= 0 (and NaN), exact L at w >= 1. */
static void dg_twohand_swing_weighted(const double L[4], double w, double out[4])
{
    double q[4], ang, s, k, n;
    if (!(w > 0) || !dg_swing_finite(L, 4)) { out[0] = out[1] = out[2] = 0; out[3] = 1; return; }
    memcpy(q, L, sizeof q);
    if (q[3] < 0) { q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3]; }
    if (w >= 1) { memcpy(out, q, sizeof q); return; }
    ang = acos(fmin(1.0, q[3]));
    s = sin(ang);
    if (s < 1e-9) { out[0] = out[1] = out[2] = 0; out[3] = 1; return; }
    k = sin(w * ang) / s;
    out[0] = q[0] * k; out[1] = q[1] * k; out[2] = q[2] * k; out[3] = cos(w * ang);
    n = sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2] + out[3] * out[3]);
    out[0] /= n; out[1] /= n; out[2] /= n; out[3] /= n;
}

/* The steered hand target: aim * weighted(L). */
static void dg_twohand_swing_apply(const double aim[4], const double L[4], double w, double out[4])
{
    double lw[4];
    dg_twohand_swing_weighted(L, w, lw);
    dg_swing_qmul(aim, lw, out);
}
#endif
