/* dg_ik.c - the two-bone solve. See dg_ik.h for the shape and the contract.
 *
 * Order of business:
 *
 *   1. small vector helpers, all local;
 *   2. the pole plane - picking the direction the elbow bends in, including
 *      what happens when the pole says nothing;
 *   3. the position solve;
 *   4. the orientation helper.
 *
 * The one thing worth reading twice is the pole fallback in section 2. It is
 * the only place in here where the answer is not forced by the input, and a
 * fallback that flips between frames would make the elbow snap on-screen.
 */

#include <math.h>
#include <string.h>
#include "dg_ik.h"

#if defined(_MSC_VER)
#include <float.h>
#define DG_IK_FINITE(v) (_finite(v) != 0)
#else
#define DG_IK_FINITE(v) (isfinite(v) != 0)
#endif

#define DG_IK_PI 3.14159265358979323846

/* The forbidden inner radius is |upper - fore|, where the triangle collapses to
   a line and the elbow direction stops being defined. Sitting exactly on it
   would leave sin(shoulder angle) at zero and the pole plane meaningless, so
   the clamp stops just outside, scaled by the chain so that it means the same
   thing for a 10-unit test rig as for a 535-unit arm. */
#define DG_IK_INNER_EPS_FRAC 1e-6

/* Below this much lateral information the pole is treated as saying nothing;
   above it, it is taken at face value. These are sines of the angle between
   the pole vector and the shoulder-to-target axis: roughly 1.1 and 5.7
   degrees. The band exists rather than a single threshold because a pole
   hovering at a threshold would otherwise flip the elbow from one side of the
   arm to the other on alternate frames. */
#define DG_IK_POLE_LO 0.02
#define DG_IK_POLE_HI 0.10

/* atan2 names the negative fallback direction as either +pi or -pi depending
   on the sign of a rounding-sized side component. Multiplying that angle by a
   partial fallback weight turns those equivalent endpoint names into opposite
   elbow bends. Treat a direction indistinguishable from exactly opposite as
   +pi, deterministically. This is an algebraic branch choice, not temporal
   smoothing: identical inputs still produce identical outputs and no prior
   frame is consulted. */
#define DG_IK_POLE_ANTIPARALLEL_EPS 1e-12

/* ==================================================== vector helpers ===== */

static void v_sub(const double a[3], const double b[3], double out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static double v_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double v_len(const double a[3])
{
    return sqrt(v_dot(a, a));
}

static void v_cross(const double a[3], const double b[3], double out[3])
{
    double x = a[1] * b[2] - a[2] * b[1];
    double y = a[2] * b[0] - a[0] * b[2];
    double z = a[0] * b[1] - a[1] * b[0];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

static void v_scale(const double a[3], double s, double out[3])
{
    out[0] = a[0] * s;
    out[1] = a[1] * s;
    out[2] = a[2] * s;
}

static void v_zero(double a[3])
{
    a[0] = 0.0;
    a[1] = 0.0;
    a[2] = 0.0;
}

static int v_finite(const double a[3])
{
    return DG_IK_FINITE(a[0]) && DG_IK_FINITE(a[1]) && DG_IK_FINITE(a[2]);
}

/* Returns 0 and leaves out zeroed when the vector has no length to speak of.
   Callers treat that as "this direction is not available", never as an error
   to ignore. */
static int v_normalize(const double a[3], double out[3])
{
    double len = v_len(a);
    if (!DG_IK_FINITE(len) || len <= 0.0) {
        v_zero(out);
        return 0;
    }
    v_scale(a, 1.0 / len, out);
    return 1;
}

static double clamp01(double v)
{
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

/* Hermite ramp, zero derivative at both ends. Used only to fade the pole
   fallback in, where a linear ramp would still produce a visible kink in the
   elbow path at each end of the band. */
static double smoothstep01(double t)
{
    t = clamp01(t);
    return t * t * (3.0 - 2.0 * t);
}

/* acos with the argument pinned. Law-of-cosines quotients land a few ulps
   outside [-1, 1] all the time when the triangle is nearly degenerate, and
   acos(1.0000000000000002) is a NaN that would reach the game as a joint
   rotation. */
static double acos_clamped(double c)
{
    if (c <= -1.0) return DG_IK_PI;
    if (c >= 1.0) return 0.0;
    return acos(c);
}

/* ======================================================= the pole plane == */

/* A direction perpendicular to `axis`, chosen from the world basis vector that
   `axis` is least aligned with. Deterministic and never degenerate: the least
   aligned basis vector has |axis . e| <= 1/sqrt(3), so the perpendicular part
   is always at least sqrt(2/3) long.

   There is no continuous choice available here - a nowhere-zero continuous
   tangent field on the sphere does not exist - so this necessarily jumps
   somewhere. It jumps where two components of the axis are exactly equal in
   magnitude, which is as far as it can be pushed from the cases that matter,
   and it is a pure function of the axis: the same axis always yields the same
   direction, with no memory and no hysteresis. */
static void perp_of(const double axis[3], double out[3])
{
    double basis[3];
    double d;
    int k = 0;
    double best = fabs(axis[0]);
    double ay = fabs(axis[1]);
    double az = fabs(axis[2]);

    if (ay < best) { best = ay; k = 1; }
    if (az < best) { k = 2; }

    basis[0] = (k == 0) ? 1.0 : 0.0;
    basis[1] = (k == 1) ? 1.0 : 0.0;
    basis[2] = (k == 2) ? 1.0 : 0.0;

    d = v_dot(basis, axis);
    out[0] = basis[0] - d * axis[0];
    out[1] = basis[1] - d * axis[1];
    out[2] = basis[2] - d * axis[2];
    v_normalize(out, out);
}

/* Decides which way the elbow bends: a unit vector perpendicular to `axis`,
   in the half-plane the pole points into. `axis` must already be unit length.
   Sets *used_fallback when the pole contributed less than everything.

   How the degenerate case is handled, and why it is done this way:

   The pole is only meaningful through its component perpendicular to the
   shoulder-to-target axis. As the pole swings towards that axis, that
   component shrinks to nothing and its DIRECTION becomes numerically wild -
   long before it becomes zero. Switching to a fallback at a fixed threshold
   would therefore hand the elbow between two unrelated directions whenever the
   pole loiters near the threshold, once per frame, which reads on-screen as
   the elbow snapping back and forth.

   So instead of a switch there is a rotation. The pole's lateral direction is
   measured as an angle around the axis relative to the fallback direction, and
   that angle is scaled towards zero as the pole degenerates. Full pole at one
   end of the band, exactly the fallback at the other, and a bounded rate of
   change in between - the elbow sweeps rather than jumps. Nothing here is a
   comparison against a near-zero quantity, and nothing is remembered between
   calls, so two identical inputs always give the same elbow. */
static void bend_direction(const double axis[3], const double pole_rel[3],
                           double out[3], int *used_fallback)
{
    double fallback[3];
    double side[3];
    double perp[3];
    double along;
    double pole_len;
    double perp_len;
    double sine;
    double weight;
    double angle;
    double ca;
    double sa;

    perp_of(axis, fallback);
    /* Completes a right-handed frame in the plane the elbow lives in. Both are
       unit and perpendicular to the axis, so an angle in this frame is an
       honest rotation about the axis. */
    v_cross(axis, fallback, side);

    along = v_dot(pole_rel, axis);
    perp[0] = pole_rel[0] - along * axis[0];
    perp[1] = pole_rel[1] - along * axis[1];
    perp[2] = pole_rel[2] - along * axis[2];

    pole_len = v_len(pole_rel);
    perp_len = v_len(perp);

    /* sin(angle between pole and axis). Scale-free on purpose: a caller that
       puts the pole a metre away and one that puts it a centimetre away in the
       same direction must get the same elbow. */
    sine = (pole_len > 0.0) ? (perp_len / pole_len) : 0.0;
    weight = 1.0 - smoothstep01((sine - DG_IK_POLE_LO) /
                                (DG_IK_POLE_HI - DG_IK_POLE_LO));

    if (perp_len > 0.0) {
        double side_component = v_dot(perp, side) / perp_len;
        double fallback_component = v_dot(perp, fallback) / perp_len;
        if (fallback_component < 0.0 &&
            fabs(side_component) <= DG_IK_POLE_ANTIPARALLEL_EPS)
            angle = DG_IK_PI;
        else
            angle = atan2(side_component, fallback_component);
    } else {
        /* A pole exactly on the axis, or a pole at the shoulder itself. There
           is no angle to measure; weight is 1 here anyway, so this only avoids
           atan2(0, 0). */
        angle = 0.0;
    }

    angle *= (1.0 - weight);
    ca = cos(angle);
    sa = sin(angle);
    out[0] = ca * fallback[0] + sa * side[0];
    out[1] = ca * fallback[1] + sa * side[1];
    out[2] = ca * fallback[2] + sa * side[2];

    *used_fallback = (weight > 0.0) ? 1 : 0;
}

/* ====================================================== position solve === */

/* Distance between shoulder and wrist for a given interior elbow angle. The
   angle limits are enforced by converting them into a distance range and
   clamping the distance, which keeps the whole solve to one clamp on one
   scalar instead of two separate corrections that could disagree. */
static double dist_for_elbow(double upper, double fore, double deg)
{
    double rad = deg * (DG_IK_PI / 180.0);
    double v = upper * upper + fore * fore - 2.0 * upper * fore * cos(rad);
    if (v <= 0.0) return 0.0;
    return sqrt(v);
}

static int input_is_sane(const DG_IK_IN *in)
{
    if (!v_finite(in->shoulder) || !v_finite(in->target) || !v_finite(in->pole))
        return 0;
    if (!DG_IK_FINITE(in->upper) || !DG_IK_FINITE(in->fore)) return 0;
    if (in->upper <= 0.0 || in->fore <= 0.0) return 0;
    if (!DG_IK_FINITE(in->max_reach_frac)) return 0;
    /* Above 1 would ask for a distance the two bones cannot span, and the only
       way to honour that is to stretch one of them - which is the one thing
       this solver promises never to do. Refusing is the honest answer; quietly
       clamping to 1 would hide a caller bug behind a locked-straight elbow. */
    if (in->max_reach_frac <= 0.0 || in->max_reach_frac > 1.0) return 0;
    if (!DG_IK_FINITE(in->min_elbow_deg) || !DG_IK_FINITE(in->max_elbow_deg))
        return 0;
    if (in->min_elbow_deg > in->max_elbow_deg) return 0;
    return 1;
}

int dg_ik_solve(const DG_IK_IN *in, DG_IK_OUT *out)
{
    double to_target[3];
    double axis[3];
    double pole_rel[3];
    double bend[3];
    double span;
    double inner;
    double d_raw;
    double d;
    double reach_min;
    double reach_max;
    double angle_min_deg;
    double angle_max_deg;
    double d_angle_lo;
    double d_angle_hi;
    double d_lo;
    double d_hi;
    double cos_shoulder;
    double a_shoulder;
    double cos_elbow;
    unsigned flags = 0;
    int fallback = 0;
    int i;

    /* Zeroed first, so every early return leaves the caller with something it
       can safely copy and obviously must not use. */
    v_zero(out->elbow);
    v_zero(out->wrist);
    out->elbow_deg = 0.0;
    out->reach_used = 0.0;
    out->clamped = 0;

    if (!in || !input_is_sane(in)) {
        out->clamped = DG_IK_BAD_INPUT;
        return 0;
    }

    v_sub(in->target, in->shoulder, to_target);
    d_raw = v_len(to_target);
    /* A target sitting exactly on the shoulder gives no axis at all, so there
       is no plane to put the elbow in and no direction to point the arm. There
       is no sane guess to make here - refuse and let the caller keep last
       frame's pose. */
    if (!v_normalize(to_target, axis)) {
        out->clamped = DG_IK_BAD_INPUT;
        return 0;
    }

    span = in->upper + in->fore;
    inner = fabs(in->upper - in->fore);
    reach_min = inner + span * DG_IK_INNER_EPS_FRAC;
    reach_max = span * in->max_reach_frac;

    /* Angle limits are pinned just inside the open interval: 0 and 180 both
       name a collapsed triangle, and a caller passing them means "no limit"
       rather than "put the arm on a line". */
    angle_min_deg = in->min_elbow_deg;
    angle_max_deg = in->max_elbow_deg;
    if (angle_min_deg < 0.0) angle_min_deg = 0.0;
    if (angle_max_deg > 180.0) angle_max_deg = 180.0;
    if (angle_max_deg < angle_min_deg) angle_max_deg = angle_min_deg;
    d_angle_lo = dist_for_elbow(in->upper, in->fore, angle_min_deg);
    d_angle_hi = dist_for_elbow(in->upper, in->fore, angle_max_deg);

    /* Each bound is reported against the ORIGINAL distance, before the bounds
       are intersected. A caller that is both out of reach and past the elbow
       limit is told both, because those two mean different things: one is the
       player's arm being longer than the character's, the other is the pose
       hitting a limit we chose. */
    if (d_raw > reach_max) flags |= DG_IK_CLAMP_REACH;
    if (d_raw < reach_min) flags |= DG_IK_CLAMP_TOO_CLOSE;
    if (d_raw < d_angle_lo || d_raw > d_angle_hi)
        flags |= DG_IK_CLAMP_ELBOW_LIMIT;

    d_lo = (reach_min > d_angle_lo) ? reach_min : d_angle_lo;
    d_hi = (reach_max < d_angle_hi) ? reach_max : d_angle_hi;
    /* Contradictory limits - a max_reach_frac so small it lands inside the
       minimum elbow angle, say. The inner bound wins, because violating it is
       the one that has no triangle at all. */
    if (d_hi < d_lo) d_hi = d_lo;

    d = d_raw;
    if (d < d_lo) d = d_lo;
    if (d > d_hi) d = d_hi;

    v_sub(in->pole, in->shoulder, pole_rel);
    bend_direction(axis, pole_rel, bend, &fallback);
    if (fallback) flags |= DG_IK_POLE_FALLBACK;

    /* Re-orthogonalise against the axis. bend_direction builds it from a frame
       that is orthonormal by construction, but the rounding in that frame is
       exactly what would show up as a stretched bone below, and one Gram-
       Schmidt pass costs nothing next to being wrong. */
    {
        double drift = v_dot(bend, axis);
        bend[0] -= drift * axis[0];
        bend[1] -= drift * axis[1];
        bend[2] -= drift * axis[2];
        if (!v_normalize(bend, bend))
            perp_of(axis, bend);
    }

    /* Law of cosines at the shoulder. d is inside [|upper-fore|, upper+fore]
       by the clamp above, so this quotient is inside [-1, 1] up to rounding,
       which acos_clamped absorbs. */
    cos_shoulder = (in->upper * in->upper + d * d - in->fore * in->fore) /
                   (2.0 * in->upper * d);
    a_shoulder = acos_clamped(cos_shoulder);

    for (i = 0; i < 3; i++) {
        out->wrist[i] = in->shoulder[i] + d * axis[i];
        out->elbow[i] = in->shoulder[i] +
                        in->upper * (cos(a_shoulder) * axis[i] +
                                     sin(a_shoulder) * bend[i]);
    }

    cos_elbow = (in->upper * in->upper + in->fore * in->fore - d * d) /
                (2.0 * in->upper * in->fore);
    out->elbow_deg = acos_clamped(cos_elbow) * (180.0 / DG_IK_PI);
    out->reach_used = d;
    out->clamped = flags;
    return 1;
}

/* ======================================================= orientation ===== */

int dg_ik_swing_quat(const double from[3], const double to[3], double q[4],
                     int *flipped)
{
    double a[3];
    double b[3];
    double axis[3];
    double w;
    double len;

    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 1.0;
    if (flipped) *flipped = 0;

    if (!from || !to || !v_finite(from) || !v_finite(to)) return 0;
    if (!v_normalize(from, a)) return 0;
    if (!v_normalize(to, b)) return 0;

    v_cross(a, b, axis);
    w = 1.0 + v_dot(a, b);

    /* Opposite directions: the cross product vanishes and w is zero, so the
       half turn is real but its axis is not determined by the input. Any
       perpendicular does the job; perp_of gives the same one every time for
       the same bone, which is what stops the joint spinning on its own. */
    if (w <= 1e-12) {
        perp_of(a, axis);
        q[0] = axis[0];
        q[1] = axis[1];
        q[2] = axis[2];
        q[3] = 0.0;
        if (flipped) *flipped = 1;
        return 1;
    }

    q[0] = axis[0];
    q[1] = axis[1];
    q[2] = axis[2];
    q[3] = w;
    len = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!DG_IK_FINITE(len) || len <= 0.0) {
        q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
        return 0;
    }
    q[0] /= len;
    q[1] /= len;
    q[2] /= len;
    q[3] /= len;
    return 1;
}

static void quat_rotate(const double q[4], const double v[3], double out[3])
{
    double u[3] = { q[0], q[1], q[2] };
    double uv[3];
    double uuv[3];

    v_cross(u, v, uv);
    v_cross(u, uv, uuv);
    out[0] = v[0] + 2.0 * (q[3] * uv[0] + uuv[0]);
    out[1] = v[1] + 2.0 * (q[3] * uv[1] + uuv[1]);
    out[2] = v[2] + 2.0 * (q[3] * uv[2] + uuv[2]);
}

/* The twist component of q about a unit axis: the rotation about that axis
   which, followed by a pure swing, reproduces q. Identity when q turns about
   something perpendicular - there is then nothing to twist. */
static void orient_twist_about(const double q[4], const double axis[3],
                               double out[4])
{
    double proj = q[0] * axis[0] + q[1] * axis[1] + q[2] * axis[2];
    double len = sqrt(proj * proj + q[3] * q[3]);
    if (!DG_IK_FINITE(len) || len <= 1e-12) {
        out[0] = 0.0; out[1] = 0.0; out[2] = 0.0; out[3] = 1.0;
        return;
    }
    out[0] = axis[0] * proj / len;
    out[1] = axis[1] * proj / len;
    out[2] = axis[2] * proj / len;
    out[3] = q[3] / len;
}

/* Slide q along its bone circle to the point nearest prev. The circle is
   {twist-about-bone times q}; the member closest to prev is reached by the
   twist component of prev relative to q. The result still takes rest onto
   the bone EXACTLY - only the free parameter moved - and it is continuous
   in q, bone and prev, which the shortest arc alone is not. */
static void orient_nearest_on_circle(const double q[4], const double bone[3],
                                     const double prev[4], double out[4])
{
    double inv[4], rel[4], tw[4];
    inv[0] = -q[0]; inv[1] = -q[1]; inv[2] = -q[2]; inv[3] = q[3];
    dg_ik_quat_mul(prev, inv, rel);
    orient_twist_about(rel, bone, tw);
    dg_ik_quat_mul(tw, q, out);
}

/* The roll about `axis` that brings v as near to target as a roll can, as a
   quaternion. Both vectors are projected onto the plane across the axis and
   the angle between the projections is the answer; 0 is returned when either
   projection vanishes - then the roll genuinely changes nothing about their
   relationship and the caller must decide the free parameter some other way.

   This is what gives the upper arm's roll a REFERENCE. Without one, the roll
   was pinned only to the previous pair, and continuity has no restoring
   force: a hand walked around a closed loop returns to its direction but not
   to its roll, and the difference accumulates every lap - 52.8 degrees per
   circle in front of the shoulder, measured 2026-08-21 on the desk, smooth
   at under 1 degree per step so every jump-hunting instrument called it
   calm. That is the arm slowly winding up, which is what the headset sees as
   tumbling. Anchored to the pose instead, the same arm pose always yields
   the same roll and accumulation is not merely small but impossible. */
static int roll_to_align(const double axis[3], const double v[3],
                         const double target[3], double q[4])
{
    double vp[3], tp[3], vn[3], tn[3], cr[3];
    double dv, dt, c, s, ang, h;
    int k;

    q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
    if (!v_finite(axis) || !v_finite(v) || !v_finite(target)) return 0;
    dv = v_dot(v, axis);
    dt = v_dot(target, axis);
    for (k = 0; k < 3; k++) {
        vp[k] = v[k] - dv * axis[k];
        tp[k] = target[k] - dt * axis[k];
    }
    /* A vector along the axis has no projection to turn: the roll is free
       and this function declines to invent one. The bar is a real length,
       not merely nonzero - v_normalize would happily turn a 1e-17 residue
       into a unit vector and hand back a direction made of rounding noise,
       which reads as a deterministic anchor while jumping half a turn
       between neighbouring poses. */
    if (!(v_len(vp) > 1e-6) || !(v_len(tp) > 1e-6)) return 0;
    if (!v_normalize(vp, vn) || !v_normalize(tp, tn)) return 0;
    c = v_dot(vn, tn);
    v_cross(vn, tn, cr);
    s = v_dot(cr, axis);
    if (!DG_IK_FINITE(c) || !DG_IK_FINITE(s)) return 0;
    ang = atan2(s, c);
    if (!DG_IK_FINITE(ang)) return 0;
    h = 0.5 * ang;
    q[0] = axis[0] * sin(h);
    q[1] = axis[1] * sin(h);
    q[2] = axis[2] * sin(h);
    q[3] = cos(h);
    return 1;
}

int dg_ik_orient(const DG_IK_ORIENT_IN *in, DG_IK_ORIENT_OUT *out)
{
    double upper_dir[3];
    double fore_dir[3];
    double fore_after_upper[3];
    double bone[3];
    int continued = 0;
    int anchored = 0;
    int anchor_k;
    int flip_upper = 0;
    int flip_fore = 0;

    out->upper_quat[0] = 0.0; out->upper_quat[1] = 0.0;
    out->upper_quat[2] = 0.0; out->upper_quat[3] = 1.0;
    out->fore_quat[0] = 0.0; out->fore_quat[1] = 0.0;
    out->fore_quat[2] = 0.0; out->fore_quat[3] = 1.0;
    out->flags = 0;

    if (!in || !v_finite(in->shoulder) || !v_finite(in->elbow) ||
        !v_finite(in->wrist) || !v_finite(in->rest_upper) ||
        !v_finite(in->rest_fore)) {
        out->flags = DG_IK_ORIENT_BAD_INPUT;
        return 0;
    }

    v_sub(in->elbow, in->shoulder, upper_dir);
    v_sub(in->wrist, in->elbow, fore_dir);

    /* These are swing-only rotations: twist about a bone is not recoverable
       from two points, and inventing it here would silently roll the hand.
       Joint 5 is a child of joint 4, so its rest direction first inherits the
       upper rotation.  The elbow output is only the residual world swing from
       that intermediate direction; solving both joints independently would
       apply the upper swing twice to the forearm. */
    if (!dg_ik_swing_quat(in->rest_upper, upper_dir, out->upper_quat,
                          &flip_upper)) {
        out->flags = DG_IK_ORIENT_BAD_INPUT;
        return 0;
    }
    /* The upper's free roll, anchored to the pose: choose it so the child's
       rest direction lands as near the actual forearm as a roll can put it.
       That is what a shoulder does when the elbow has to bend a particular
       way, it is a pure function of shoulder/elbow/wrist, and - the point -
       it cannot accumulate, because the same pose always returns the same
       roll. It also leaves the forearm's residual swing SMALL by
       construction, which keeps the fore solve away from the antiparallel
       seam that the continuity memory was introduced to survive. */
    if (v_normalize(upper_dir, bone)) {
        double p0[3], carried[3], normal[3], rest_normal[3], rollq[4];
        double anchored_q[4];
        /* The reference pair, both perpendicular to the bone by
           construction so the roll can align them EXACTLY and the
           conditioning is that of the arm plane itself:
             - carried: a fixed rest-space perpendicular of the upper bone
               (perp_of is a pure function of its input, no memory), turned
               by the swing we just solved, so it lands perpendicular to
               the live bone;
             - normal: the plane the real arm bends in.
           Aligning them says "the shoulder rolls so the elbow points where
           it actually points", which is the anatomical statement and, more
           to the point, a function of THIS pose alone.

           Both normals, deliberately, and nothing else. Two earlier
           references were tried and are recorded here because each failed
           in a way the other would not have shown:
             - the child REST direction: on a rig whose rest arm is
               straight it lies along the bone, so its projection is
               rounding noise and the anchor jumped half a turn between
               neighbouring poses;
             - perp_of(rest_upper): perp_of is a pure function but a
               DISCONTINUOUS one by construction (it jumps where two
               components of its input match in magnitude), and rest_upper
               is read from the live skeleton, so a yawing body walks its
               input straight through that jump - three ~90 degree steps in
               a 60-pair turn probe, 2026-08-21. A reference must be
               continuous in the pose, not merely deterministic. */
        v_cross(upper_dir, fore_dir, normal);
        v_cross(in->rest_upper, in->rest_fore, rest_normal);
        /* Both planes must be genuinely OPEN, and the bar is the sine of
           the bend rather than a raw length: |a x b| = |a||b| sin, so a
           plain length test would call a long straight arm well-formed and
           a short bent one degenerate. Below this the cross product is
           cancellation error and normalising it yields a direction made of
           rounding noise - an anchor that looks deterministic and points
           somewhere different for every neighbouring pose. 1e-4 is a bend
           of six thousandths of a degree; every rig and every animated
           pose is orders of magnitude clear of it. */
        v_zero(carried);
        if (v_len(rest_normal) >
                1e-4 * v_len(in->rest_upper) * v_len(in->rest_fore) &&
            v_len(normal) > 1e-4 * v_len(upper_dir) * v_len(fore_dir) &&
            v_normalize(rest_normal, p0)) {
            quat_rotate(out->upper_quat, p0, carried);
        }
        if (roll_to_align(bone, carried, normal, rollq)) {
            dg_ik_quat_mul(rollq, out->upper_quat, anchored_q);
            if (dg_ik_quat_normalize(anchored_q)) {
                for (anchor_k = 0; anchor_k < 4; anchor_k++)
                    out->upper_quat[anchor_k] = anchored_q[anchor_k];
                anchored = 1;
                out->flags |= DG_IK_ORIENT_ANCHORED;
            }
        }
    }
    /* Only where the anchor is degenerate - the arm folded so the forearm
       lies along the upper bone, where no roll changes anything - is there a
       free parameter left, and there the previous pair is still the best
       available answer. */
    if (!anchored && in->have_prev && v_normalize(upper_dir, bone)) {
        orient_nearest_on_circle(out->upper_quat, bone,
                                 in->prev_upper_quat, out->upper_quat);
        if (!dg_ik_quat_normalize(out->upper_quat)) {
            out->flags = DG_IK_ORIENT_BAD_INPUT;
            return 0;
        }
        continued = 1;
    }
    /* The fore residual leans on the CONTINUED upper: its circle is built
       from the direction the upper actually leaves the forearm's rest in. */
    quat_rotate(out->upper_quat, in->rest_fore, fore_after_upper);
    if (!dg_ik_swing_quat(fore_after_upper, fore_dir, out->fore_quat,
                          &flip_fore)) {
        out->flags = DG_IK_ORIENT_BAD_INPUT;
        return 0;
    }
    /* With the upper anchored, the fore residual is the minimal swing and
       nothing more: pose-only, so it cannot accumulate either. Sliding it
       along its own circle toward the previous pair is exactly the mechanism
       that wound the arm up, so it is done ONLY on the degenerate path,
       where the upper had no anchor to give. */
    if (!anchored && continued && v_normalize(fore_dir, bone)) {
        orient_nearest_on_circle(out->fore_quat, bone,
                                 in->prev_fore_quat, out->fore_quat);
        if (!dg_ik_quat_normalize(out->fore_quat)) {
            out->flags = DG_IK_ORIENT_BAD_INPUT;
            return 0;
        }
        out->flags |= DG_IK_ORIENT_CONTINUED;
    }

    if (flip_upper) out->flags |= DG_IK_ORIENT_UPPER_FLIP;
    if (flip_fore) out->flags |= DG_IK_ORIENT_FORE_FLIP;
    return 1;
}

/* ==================================================== hand orientation === */

static void q_copy(const double a[4], double out[4])
{
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2]; out[3] = a[3];
}

void dg_ik_quat_mul(const double a[4], const double b[4], double out[4])
{
    double x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
    double y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
    double z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
    double w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
    out[0] = x; out[1] = y; out[2] = z; out[3] = w;
}

void dg_ik_quat_conj(const double q[4], double out[4])
{
    out[0] = -q[0]; out[1] = -q[1]; out[2] = -q[2]; out[3] = q[3];
}

int dg_ik_quat_normalize(double q[4])
{
    double len;
    int i;

    for (i = 0; i < 4; i++) {
        if (!DG_IK_FINITE(q[i])) {
            q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
            return 0;
        }
    }
    len = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (!DG_IK_FINITE(len) || len <= 1e-12) {
        q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
        return 0;
    }
    for (i = 0; i < 4; i++) q[i] /= len;
    return 1;
}

double dg_ik_quat_angle(const double a[4], const double b[4])
{
    double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (d < 0.0) d = -d;
    return 2.0 * acos_clamped(d);
}

int dg_ik_basis_quat(const double basis[3][3], double q[4])
{
    double e[3][3];
    double proj[3];
    double t[3];
    double m[3][3];
    double tr, s;
    int r, c;

    q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
    if (!basis) return 0;
    for (r = 0; r < 3; r++)
        if (!v_finite(basis[r])) return 0;

    /* Gram-Schmidt first. The live matrices are orthonormal only to float
       precision, and the trace formula below turns a slightly skewed basis
       into something that is not quite a rotation - which then compounds every
       pair it is multiplied through. */
    if (!v_normalize(basis[0], e[0])) return 0;
    v_scale(e[0], v_dot(e[0], basis[1]), proj);
    v_sub(basis[1], proj, t);
    if (!v_normalize(t, e[1])) return 0;
    v_cross(e[0], e[1], e[2]);

    /* A left-handed basis is a mirror, not a rotation. The third row is the
       one that says which, so it is checked rather than discarded. */
    if (v_dot(e[2], basis[2]) <= 0.0) return 0;

    /* basis rows are the world directions of the local axes, so the matrix
       that takes a local column vector to world is the transpose of them. */
    for (r = 0; r < 3; r++)
        for (c = 0; c < 3; c++) m[r][c] = e[c][r];

    tr = m[0][0] + m[1][1] + m[2][2];
    if (tr > 0.0) {
        s = sqrt(tr + 1.0) * 2.0;
        q[3] = 0.25 * s;
        q[0] = (m[2][1] - m[1][2]) / s;
        q[1] = (m[0][2] - m[2][0]) / s;
        q[2] = (m[1][0] - m[0][1]) / s;
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q[3] = (m[2][1] - m[1][2]) / s;
        q[0] = 0.25 * s;
        q[1] = (m[0][1] + m[1][0]) / s;
        q[2] = (m[0][2] + m[2][0]) / s;
    } else if (m[1][1] > m[2][2]) {
        s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q[3] = (m[0][2] - m[2][0]) / s;
        q[0] = (m[0][1] + m[1][0]) / s;
        q[1] = 0.25 * s;
        q[2] = (m[1][2] + m[2][1]) / s;
    } else {
        s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q[3] = (m[1][0] - m[0][1]) / s;
        q[0] = (m[0][2] + m[2][0]) / s;
        q[1] = (m[1][2] + m[2][1]) / s;
        q[2] = 0.25 * s;
    }
    return dg_ik_quat_normalize(q);
}

int dg_ik_hand_adjust(const DG_IK_HAND_IN *in, double out[4])
{
    double live[4], pu[4], pf[4], ph[4], nu[4], nf[4], want[4];
    double chain[4], animated[4], inv[4];

    out[0] = 0.0; out[1] = 0.0; out[2] = 0.0; out[3] = 1.0;
    if (!in) return 0;
    q_copy(in->live, live);
    q_copy(in->prev_upper, pu);
    q_copy(in->prev_fore, pf);
    q_copy(in->prev_hand, ph);
    q_copy(in->new_upper, nu);
    q_copy(in->new_fore, nf);
    q_copy(in->desired, want);
    if (!dg_ik_quat_normalize(live) || !dg_ik_quat_normalize(pu) ||
        !dg_ik_quat_normalize(pf) || !dg_ik_quat_normalize(ph) ||
        !dg_ik_quat_normalize(nu) || !dg_ik_quat_normalize(nf) ||
        !dg_ik_quat_normalize(want)) return 0;

    /* Strip last pair's three adjustments back off the matrix we just read. */
    dg_ik_quat_mul(ph, pf, chain);
    dg_ik_quat_mul(chain, pu, chain);
    dg_ik_quat_conj(chain, inv);
    dg_ik_quat_mul(inv, live, animated);

    /* What the hand will inherit on the next pass, before its own adjust. */
    dg_ik_quat_mul(nf, nu, chain);
    dg_ik_quat_mul(chain, animated, chain);
    dg_ik_quat_conj(chain, inv);
    dg_ik_quat_mul(want, inv, out);
    return dg_ik_quat_normalize(out);
}

static void q_identity(double q[4])
{
    q[0] = 0.0; q[1] = 0.0; q[2] = 0.0; q[3] = 1.0;
}

static void q_axis_angle(const double axis[3], double angle, double q[4])
{
    double h = 0.5 * angle;
    double s = sin(h);
    q[0] = axis[0] * s;
    q[1] = axis[1] * s;
    q[2] = axis[2] * s;
    q[3] = cos(h);
}

static double q_clamp_signed(double v, double limit)
{
    if (v > limit) return limit;
    if (v < -limit) return -limit;
    return v;
}

/* One turn of the multivalued angle: the branch of `angle` nearest ref.
   Eight steps bound the walk for any ref a caller could have accumulated
   without ever hiding a genuinely non-finite input. */
static double unwrap_toward(double angle, double ref)
{
    int guard;
    for (guard = 0; guard < 8 && angle - ref > DG_IK_PI; guard++)
        angle -= 2.0 * DG_IK_PI;
    for (guard = 0; guard < 8 && ref - angle > DG_IK_PI; guard++)
        angle += 2.0 * DG_IK_PI;
    return angle;
}

/* How much longer than the short arc a walked branch may grow before the
   fold renames it. A branch is bookkeeping: b and b - 360 are the same
   rotation, and the clamps render whichever name they are handed. Without
   a margin a demand hovering on the 180 seam would be renamed every pair
   and, under caps below 180, the applied would snap by two caps at 22 Hz
   (the 2026-08-21 evening tumble: cmd-drift worst 145.9, desk sweep
   sweep_seam.c). Live noise is a few degrees; a genuine crossing walks
   straight through. Thirty is far above the one, far below the other. */
#define DG_IK_UNWRAP_HYST_RAD (30.0 * DG_IK_PI / 180.0)

/* The branch choice: keep walking the branch the caller was on (raw_ref,
   the unwrapped angle it last reported; the applied reference when there
   is none). Inside the cap the walked name is rendered exactly, whichever
   name it is, so nothing is renamed there. Once the walked name is
   outside the cap AND longer than the short arc by the margin, it is
   renamed to the short arc: the short arc is never further outside the
   caps than any other name of the same rotation, so with caps summing
   past 180 (the live 170 + 55) the hand renders the rotation it was
   asked for on both sides of the rename (the forearm's share of the twist
   does move - the split is not a function of the rotation - by 2 x 170 -
   360 at least, once per crossing). What this replaces was the
   nearest-to-APPLIED rule, and that rule latched: in run 11 (2026-09-03)
   a body turn walked the raw branch to -346, the applied sat pinned on
   the -225 cap, and against that cap the nearest name of a neutral hand
   (0) is -360, not 0 - the hand stayed 135 degrees wrong until the player
   turned 45 the other way, which the video shows as the gun lying flat
   across the forearm. The bound here is |branch| <= max(cap, 180 +
   margin), so the reference fed back cannot wind up and a garbage wound
   reference is abandoned on the first read. (Under that bound the applied
   reference alone would name the same branch for any cap above the
   margin; the raw reference is kept as the plain continuity memory.) */
static double unwrap_branch(double angle, double app_ref,
                            int have_raw, double raw_ref, double cap)
{
    double ref = (have_raw && DG_IK_FINITE(raw_ref)) ? raw_ref : app_ref;
    double pick = unwrap_toward(angle, ref);
    if (fabs(pick) > cap &&
        fabs(angle) + DG_IK_UNWRAP_HYST_RAD < fabs(pick))
        pick = angle;
    return pick;
}

int dg_ik_hand_stabilize(const DG_IK_HAND_STABILIZE_IN *in,
                         DG_IK_HAND_STABILIZE_OUT *out)
{
    double raw[4], axis[3], twist[4], swing[4], inv[4];
    double swing_axis[3], limited_swing[4], wrist_twist[4];
    double axis_len, twist_len, swing_vec_len, prev_axis_len;
    double twist_angle, fore_angle, remain, wrist_angle;
    double swing_angle, limited_swing_angle, projection;
    int k, have_swing_axis;

    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    q_identity(out->fore_twist_world);
    q_identity(out->wrist_world);
    if (!in || !DG_IK_FINITE(in->max_fore_twist_rad) ||
        !DG_IK_FINITE(in->max_wrist_swing_rad) ||
        !DG_IK_FINITE(in->max_wrist_twist_rad) ||
        in->max_fore_twist_rad < 0.0 ||
        in->max_wrist_swing_rad < 0.0 ||
        in->max_wrist_twist_rad < 0.0) return 0;

    q_copy(in->raw_world, raw);
    if (!dg_ik_quat_normalize(raw) || !v_finite(in->fore_axis_world)) return 0;
    axis_len = sqrt(v_dot(in->fore_axis_world, in->fore_axis_world));
    if (!DG_IK_FINITE(axis_len) || axis_len <= 1e-12) return 0;
    for (k = 0; k < 3; k++) axis[k] = in->fore_axis_world[k] / axis_len;

    /* q and -q encode the same rotation. Pick w >= 0 so all reported and
       limited angles follow the deterministic short arc. */
    if (raw[3] < 0.0)
        for (k = 0; k < 4; k++) raw[k] = -raw[k];

    projection = raw[0] * axis[0] + raw[1] * axis[1] + raw[2] * axis[2];
    twist[0] = axis[0] * projection;
    twist[1] = axis[1] * projection;
    twist[2] = axis[2] * projection;
    twist[3] = raw[3];
    twist_len = sqrt(projection * projection + raw[3] * raw[3]);
    if (twist_len <= 1e-12) {
        /* A 180-degree pure swing has no unique twist. Identity is the only
           deterministic, sign-independent choice. */
        q_identity(twist);
    } else {
        for (k = 0; k < 4; k++) twist[k] /= twist_len;
    }
    dg_ik_quat_conj(twist, inv);
    dg_ik_quat_mul(raw, inv, swing);
    if (!dg_ik_quat_normalize(swing)) return 0;
    if (swing[3] < 0.0)
        for (k = 0; k < 4; k++) swing[k] = -swing[k];

    twist_angle = 2.0 * atan2(twist[0] * axis[0] +
                              twist[1] * axis[1] +
                              twist[2] * axis[2], twist[3]);
    /* The branch choice. atan2 hands back (-PI, PI]; the caller's previous
       twist names which turn of the multivalued angle this pair is actually
       on, and the raw reference adds the fold hysteresis - see
       unwrap_branch. */
    if (in->have_prev_twist && DG_IK_FINITE(in->prev_twist_rad))
        twist_angle = unwrap_branch(twist_angle, in->prev_twist_rad,
                                    in->have_prev_twist_raw,
                                    in->prev_twist_raw_rad,
                                    in->max_fore_twist_rad +
                                    in->max_wrist_twist_rad);
    fore_angle = q_clamp_signed(twist_angle, in->max_fore_twist_rad);
    remain = twist_angle - fore_angle;
    wrist_angle = q_clamp_signed(remain, in->max_wrist_twist_rad);

    swing_vec_len = sqrt(swing[0] * swing[0] + swing[1] * swing[1] +
                         swing[2] * swing[2]);
    swing_angle = 2.0 * atan2(swing_vec_len, swing[3]);
    have_swing_axis = 0;
    if (swing_vec_len > 1e-12) {
        for (k = 0; k < 3; k++) swing_axis[k] = swing[k] / swing_vec_len;
        have_swing_axis = 1;
    } else {
        for (k = 0; k < 3; k++) swing_axis[k] = 0.0;
    }
    /* The swing's branch choice, mirroring the twist's above. The short arc
       re-picks its axis every call, and across 180 that axis reverses while
       the angle stays positive - same rotation, opposite clamp. Held to the
       reference hemisphere the angle turns signed, and the same 8-step walk
       puts it on the branch the caller was actually on. */
    prev_axis_len = 0.0;
    if (in->have_prev_swing && DG_IK_FINITE(in->prev_swing_rad) &&
        v_finite(in->prev_swing_axis))
        prev_axis_len = sqrt(v_dot(in->prev_swing_axis,
                                   in->prev_swing_axis));
    if (DG_IK_FINITE(prev_axis_len) && prev_axis_len > 0.5) {
        if (!have_swing_axis) {
            /* An identity swing has no axis of its own; the reference
               carries the sign convention across it unchanged. */
            for (k = 0; k < 3; k++)
                swing_axis[k] = in->prev_swing_axis[k] / prev_axis_len;
            have_swing_axis = 1;
            swing_angle = 0.0;
        } else if (swing_axis[0] * in->prev_swing_axis[0] +
                   swing_axis[1] * in->prev_swing_axis[1] +
                   swing_axis[2] * in->prev_swing_axis[2] < 0.0) {
            for (k = 0; k < 3; k++) swing_axis[k] = -swing_axis[k];
            swing_angle = -swing_angle;
        }
        swing_angle = unwrap_branch(swing_angle, in->prev_swing_rad,
                                    in->have_prev_swing_raw,
                                    in->prev_swing_raw_rad,
                                    in->max_wrist_swing_rad);
        limited_swing_angle = q_clamp_signed(swing_angle,
                                             in->max_wrist_swing_rad);
    } else {
        limited_swing_angle = swing_angle;
        if (limited_swing_angle > in->max_wrist_swing_rad)
            limited_swing_angle = in->max_wrist_swing_rad;
    }
    if (have_swing_axis)
        q_axis_angle(swing_axis, limited_swing_angle, limited_swing);
    else
        q_identity(limited_swing);

    q_axis_angle(axis, fore_angle, out->fore_twist_world);
    q_axis_angle(axis, wrist_angle, wrist_twist);
    dg_ik_quat_mul(limited_swing, wrist_twist, out->wrist_world);
    if (!dg_ik_quat_normalize(out->fore_twist_world) ||
        !dg_ik_quat_normalize(out->wrist_world)) {
        q_identity(out->fore_twist_world);
        q_identity(out->wrist_world);
        return 0;
    }

    out->raw_swing_rad = swing_angle;
    out->raw_twist_rad = twist_angle;
    out->fore_twist_rad = fore_angle;
    out->wrist_swing_rad = limited_swing_angle;
    out->wrist_twist_rad = wrist_angle;
    if (have_swing_axis)
        for (k = 0; k < 3; k++) out->wrist_swing_axis[k] = swing_axis[k];
    if (fabs(fore_angle) > 1e-12) out->flags |= DG_IK_HAND_F_FORE_TWIST;
    if (fabs(swing_angle) > fabs(limited_swing_angle) + 1e-12)
        out->flags |= DG_IK_HAND_F_SWING_LIMITED;
    if (fabs(remain) > in->max_wrist_twist_rad + 1e-12)
        out->flags |= DG_IK_HAND_F_TWIST_LIMITED;
    return 1;
}

/* =================================================== PS2 angle channel === */

/* One PS2 angle unit in radians. See the header for where 4096 comes from. */
#define DG_IK_PS_UNIT_RAD (2.0 * DG_IK_PI / (double)DG_IK_PS_UNITS_PER_TURN)

/* How far a quaternion's norm may sit from 1 before it stops being treated as
   a rotation. Live float matrices and repeated products drift by a few ulps,
   which is six orders of magnitude inside this; anything looser than 1e-6 and
   a genuinely unnormalised input - the shape a half-initialised struct has -
   would sail through. */
#define DG_IK_PS_UNIT_TOL 1e-6

/* |sin(pitch)| past which the roll/yaw pair is solved by the degenerate split
   instead of the general formulas.

   The general formulas are two atan2 calls whose four arguments are all
   proportional to cos(pitch); their absolute error is a rounding of 1, so the
   angle they return is wrong by roughly 1e-16 / cos(pitch) radians. At this
   threshold cos(pitch) is about 4.5e-5, so that error is 2e-12 rad - about a
   billionth of the 1.5e-3 rad grid the answer is rounded to. The threshold is
   deliberately this tight: everything outside it is solved exactly, and the
   split below is only ever reached where there is genuinely nothing to
   solve. */
#define DG_IK_PS_SING_SIN 0.999999999

/* Round to the nearest unit, ties away from zero. Truncating would bias every
   angle towards zero by up to a whole unit, and three of those per frame is a
   pose that consistently under-reaches rather than one that is merely
   quantised. */
static int ps_round_units(double rad)
{
    double u = rad / DG_IK_PS_UNIT_RAD;
    if (u >= 0.0) return (int)(u + 0.5);
    return -(int)(-u + 0.5);
}

/* Into [-2048, 2047], the canonical half-turn-either-way naming of an angle
   that is only ever meaningful modulo a full turn. Keeps the written short
   small, which matters downstream: dg_ik_ps_precompensate scales it up by
   D/(D-1) and a value near 32767 would saturate for no reason. The explicit
   sign fix is because C89 leaves the sign of % on negatives to the
   implementation. */
static int ps_wrap_units(int n)
{
    n = n % DG_IK_PS_UNITS_PER_TURN;
    if (n < 0) n += DG_IK_PS_UNITS_PER_TURN;
    if (n >= DG_IK_PS_UNITS_PER_TURN / 2) n -= DG_IK_PS_UNITS_PER_TURN;
    return n;
}

int dg_ik_quat_to_ps_angles(const double q[4], short out[3], unsigned *flags)
{
    double n[4];
    double len;
    double sin_p;
    double roll;
    double pitch;
    double yaw;
    unsigned f = 0;
    int vx, vy, vz;
    int i;

    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    if (flags) *flags = 0;

    if (!q) {
        if (flags) *flags = DG_IK_PS_BAD_INPUT;
        return 0;
    }
    for (i = 0; i < 4; i++) {
        if (!DG_IK_FINITE(q[i])) {
            if (flags) *flags = DG_IK_PS_BAD_INPUT;
            return 0;
        }
        n[i] = q[i];
    }
    len = sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2] + n[3] * n[3]);
    if (!DG_IK_FINITE(len) || fabs(len - 1.0) > DG_IK_PS_UNIT_TOL) {
        if (flags) *flags = DG_IK_PS_BAD_INPUT;
        return 0;
    }
    /* Unit to within tolerance is not unit; the last few ulps are divided out
       so the asin argument below cannot land outside [-1, 1] by more than
       rounding. */
    for (i = 0; i < 4; i++) n[i] /= len;

    /* For q = Rz Ry Rx the matrix entry that carries the pitch alone is
       -sin(pitch) = -2*(w*y - z*x); the other two angles are read off the two
       pairs of entries that are proportional to cos(pitch). */
    sin_p = 2.0 * (n[3] * n[1] - n[2] * n[0]);
    if (sin_p > 1.0) sin_p = 1.0;
    if (sin_p < -1.0) sin_p = -1.0;
    pitch = asin(sin_p);

    if (fabs(sin_p) >= DG_IK_PS_SING_SIN) {
        /* Gimbal lock. At pitch = +90 the quaternion depends on vx and vz only
           through vx - vz, and at -90 only through vx + vz, so no amount of
           care recovers them separately - one of the two has to be named.
           Putting the whole free parameter in vx and zeroing vz is chosen
           because with vz = 0 the SAME expression solves both poles: at +90,
           w = k*cos(vx/2) and x = k*sin(vx/2), and at -90 the signs of y and z
           flip but w and x are unchanged. One formula with no inner branch is
           one fewer place for a sign to be wrong in the case that is already
           the hardest to see on screen. The alternative - vx = 0 and the free
           parameter in vz - needs the pole's sign in the formula. */
        roll = 2.0 * atan2(n[0], n[3]);
        yaw = 0.0;
        f |= DG_IK_PS_SINGULAR;
    } else {
        roll = atan2(2.0 * (n[3] * n[0] + n[1] * n[2]),
                     1.0 - 2.0 * (n[0] * n[0] + n[1] * n[1]));
        yaw = atan2(2.0 * (n[3] * n[2] + n[0] * n[1]),
                    1.0 - 2.0 * (n[1] * n[1] + n[2] * n[2]));
    }

    /* roll and yaw come from atan2 and are at most 2*PI, so at most 4096 units
       before wrapping; pitch comes from asin and is already inside
       [-1024, 1024]. Nothing here can approach the short range, which is what
       makes the casts below safe rather than merely usually safe. */
    vx = ps_wrap_units(ps_round_units(roll));
    vy = ps_round_units(pitch);
    vz = ps_wrap_units(ps_round_units(yaw));







    if (vy == 0) {
        vy = (sin_p < 0.0) ? -1 : 1;
        f |= DG_IK_PS_VY_CLAMPED;
    }
    if (vy == DG_IK_PS_UNITS_PER_TURN / 4 || vy == -DG_IK_PS_UNITS_PER_TURN / 4)
        f |= DG_IK_PS_NEAR_POLE;

    out[0] = (short)vx;
    out[1] = (short)vy;
    out[2] = (short)vz;
    if (flags) *flags = f;
    return 1;
}

static int ps_pull_once(int from, int divisor)
{
    int diff = 0 - from;
    if (diff > -divisor && diff < divisor)
        diff = (diff > 0) ? divisor : -divisor;
    return from + diff / divisor;
}

static int ps_abs(int v)
{
    return (v < 0) ? -v : v;
}

/* How far the search below reaches either side of the closed-form estimate.
   For |X| >= D the pull is X - X/D, so X = want*D/(D-1) to within D/(D-1) <= 2;
   below D it is the fixed one-unit step and X = want +/- 1. Sixteen is an
   order of magnitude of headroom over both, and the loop is 33 integer
   comparisons per component. */
#define DG_IK_PS_SEARCH 16

int dg_ik_ps_precompensate(const short want[3], int pull_divisor, short out[3])
{
    int exact = 1;
    int i;

    if (!out) return 0;
    if (!want) {
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        return 0;
    }

    /* Not "a divisor of 1", which the recurrence turns into a hard zero, but
       "the pull has not been measured, so do not pretend to compensate for
       it". Baking 4 in here instead would make every result silently wrong the
       day the live measurement says something else. */
    if (pull_divisor <= 1) {
        out[0] = want[0];
        out[1] = want[1];
        out[2] = want[2];
        return 1;
    }

    for (i = 0; i < 3; i++) {
        int w = (int)want[i];
        double est = (double)w * (double)pull_divisor /
                     (double)(pull_divisor - 1);
        int centre;
        int lo, hi, x;
        int best = 0;
        int have = 0;

        if (est >= 32767.0) centre = 32767;
        else if (est <= -32768.0) centre = -32768;
        else centre = (est >= 0.0) ? (int)(est + 0.5) : -(int)(-est + 0.5);

        lo = centre - DG_IK_PS_SEARCH;
        hi = centre + DG_IK_PS_SEARCH;
        if (lo < -32768) lo = -32768;
        if (hi > 32767) hi = 32767;

        for (x = lo; x <= hi; x++) {
            if (ps_pull_once(x, pull_divisor) != w) continue;
            /* Writing a literal 0 for a non-zero target is legal arithmetic and
               a trap in the vy slot, where 0 changes which conversion the
               engine runs at all. There is always another preimage: at any
               divisor, -2 pulls to -1 just as 0 does. */
            if (w != 0 && x == 0) continue;
            if (!have || ps_abs(x) < ps_abs(best) ||
                (ps_abs(x) == ps_abs(best) && x > best)) {
                best = x;
                have = 1;
            }
        }

        /* Nothing in a short pulls this far: at D == 4 the reachable band is
           [-24576, 24576]. Saturate towards the target and say so, because a
           hand that stops 5000 units short of where it was told to go is a
           tracking bug the caller will otherwise spend a day looking for
           somewhere else. */
        if (!have) {
            best = (w > 0) ? 32767 : -32768;
            exact = 0;
        }
        out[i] = (short)best;
    }
    return exact;
}

/* ========================================== the channel's reach (run 13) === */

int dg_ik_ps_reach_units(int pull_divisor)
{
    int half = DG_IK_PS_UNITS_PER_TURN / 2 - 1;
    if (pull_divisor <= 1) return half;
    return ps_pull_once(half, pull_divisor);
}

static int ps_within(const short a[3], int reach)
{
    int i;
    for (i = 0; i < 3; i++)
        if ((int)a[i] > reach || (int)a[i] < -reach) return 0;
    return 1;
}

int dg_ik_quat_to_ps_reach(const double q[4], int reach, short out[3],
                           unsigned *flags)
{
    unsigned f = 0;
    short alt[3];
    const int half = DG_IK_PS_UNITS_PER_TURN / 2;

    if (reach < 1) reach = 1;
    if (!dg_ik_quat_to_ps_angles(q, out, &f)) {
        if (flags) *flags = f;
        return 0;
    }
    if (ps_within(out, reach)) {
        if (flags) *flags = f;
        return 1;
    }
    /* Rz(y) Ry(p) Rx(r) == Rz(y + 180) Ry(180 - p) Rx(r + 180): the second
       solution of the ZYX decomposition (Rz(180) Ry(180) is Rx(180), and
       conjugating Ry(-p) by it gives Ry(p)). |out[1]| <= 1024 puts the
       alternate pitch in [1024, 2048], never 0, so the XYZ path holds. */
    alt[0] = (short)ps_wrap_units((int)out[0] + half);
    alt[1] = (short)ps_wrap_units(half - (int)out[1]);
    alt[2] = (short)ps_wrap_units((int)out[2] + half);
    if (ps_within(alt, reach)) {
        out[0] = alt[0];
        out[1] = alt[1];
        out[2] = alt[2];
        f |= DG_IK_PS_ALT_NAMING;
        f &= ~(unsigned)(DG_IK_PS_VY_CLAMPED | DG_IK_PS_NEAR_POLE);
        if (alt[1] == DG_IK_PS_UNITS_PER_TURN / 4 ||
            alt[1] == -DG_IK_PS_UNITS_PER_TURN / 4)
            f |= DG_IK_PS_NEAR_POLE;
        if (flags) *flags = f;
        return 1;
    }
    f |= DG_IK_PS_UNREACHABLE;
    if (flags) *flags = f;
    return 1;
}

/* q shortened to `fraction` of its angle about its own axis; q must be unit
   and is taken in its w >= 0 naming so the angle is the short one. */
static void ps_scale_rotation(const double q[4], double fraction, double out[4])
{
    double v[3], len, angle, half, s;
    int k;
    for (k = 0; k < 3; k++) v[k] = q[k];
    len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (q[3] < 0.0) {
        for (k = 0; k < 3; k++) v[k] = -v[k];
        angle = 2.0 * atan2(len, -q[3]);
    } else {
        angle = 2.0 * atan2(len, q[3]);
    }
    if (len < 1e-12) {
        out[0] = out[1] = out[2] = 0.0;
        out[3] = 1.0;
        return;
    }
    half = 0.5 * angle * fraction;
    s = sin(half) / len;
    for (k = 0; k < 3; k++) out[k] = v[k] * s;
    out[3] = cos(half);
}

int dg_ik_ps_fit(const double q[4], int reach, double fitted[4],
                 short out[3], unsigned *flags, double *fraction)
{
    unsigned f = 0;
    double lo = 0.0, hi = 1.0, trial[4], n[4];
    int i, k;

    if (fitted) {
        fitted[0] = fitted[1] = fitted[2] = 0.0;
        fitted[3] = 1.0;
    }
    if (fraction) *fraction = 0.0;
    if (!dg_ik_quat_to_ps_reach(q, reach, out, &f)) {
        if (flags) *flags = f;
        return 0;
    }
    if (!(f & DG_IK_PS_UNREACHABLE)) {
        if (fitted) {
            for (k = 0; k < 4; k++) fitted[k] = q[k];
            dg_ik_quat_normalize(fitted);
        }
        if (fraction) *fraction = 1.0;
        if (flags) *flags = f;
        return 1;
    }
    for (k = 0; k < 4; k++) n[k] = q[k];
    dg_ik_quat_normalize(n);
    for (i = 0; i < 20; i++) {
        double mid = 0.5 * (lo + hi);
        unsigned tf = 0;
        short tmp[3];
        ps_scale_rotation(n, mid, trial);
        if (dg_ik_quat_to_ps_reach(trial, reach, tmp, &tf) &&
            !(tf & DG_IK_PS_UNREACHABLE))
            lo = mid;
        else
            hi = mid;
    }
    ps_scale_rotation(n, lo, trial);
    if (!dg_ik_quat_to_ps_reach(trial, reach, out, &f)) {
        if (flags) *flags = f;
        return 0;
    }
    f |= DG_IK_PS_SCALED;
    if (fitted) for (k = 0; k < 4; k++) fitted[k] = trial[k];
    if (fraction) *fraction = lo;
    if (flags) *flags = f;
    return 1;
}

/* Forward XYZ conversion for a reachable, nonzero-vy channel triple. */
static void ps_triple_quat(const short a[3], double q[4])
{
    double r = a[0] * DG_IK_PS_UNIT_RAD * 0.5;
    double p = a[1] * DG_IK_PS_UNIT_RAD * 0.5;
    double y = a[2] * DG_IK_PS_UNIT_RAD * 0.5;
    double cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p);
    double cy = cos(y), sy = sin(y);
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
    q[3] = cr * cp * cy + sr * sp * sy;
    dg_ik_quat_normalize(q);
}

int dg_ik_ps_project(const double q[4], int reach, double fitted[4],
                    short out[3], unsigned *flags, double *retained)
{
    short candidates[4][3];
    unsigned f = 0, best_flags = 0;
    double n[4], trial[4], best[4], best_dot = -1.0;
    int count = 2, i, k, best_i = 0;
    const int half = DG_IK_PS_UNITS_PER_TURN / 2;
    const int pole = DG_IK_PS_UNITS_PER_TURN / 4;

    if (fitted) {
        fitted[0] = fitted[1] = fitted[2] = 0.0;
        fitted[3] = 1.0;
    }
    if (retained) *retained = 0.0;
    if (!dg_ik_quat_to_ps_reach(q, reach, out, &f)) {
        if (flags) *flags = f;
        return 0;
    }
    for (k = 0; k < 4; k++) n[k] = q[k];
    dg_ik_quat_normalize(n);
    if (!(f & DG_IK_PS_UNREACHABLE)) {
        if (fitted) for (k = 0; k < 4; k++) fitted[k] = n[k];
        if (retained) *retained = 1.0;
        if (flags) *flags = f;
        return 1;
    }
    if (reach < 1) reach = 1;
    for (k = 0; k < 3; k++) candidates[0][k] = out[k];
    candidates[1][0] = (short)ps_wrap_units(out[0] + half);
    candidates[1][1] = (short)ps_wrap_units(half - out[1]);
    candidates[1][2] = (short)ps_wrap_units(out[2] + half);
    for (i = 0; i < 2; i++) for (k = 0; k < 3; k++) {
        if (candidates[i][k] > reach) candidates[i][k] = (short)reach;
        if (candidates[i][k] < -reach) candidates[i][k] = (short)-reach;
    }
    /* The pole has a whole family of Euler names, not just two. Consider
       its nearest rotation throughout the neighborhood, so approaching a
       pole cannot introduce a large artificial roll/yaw clamp error.
       At +90, delta = roll-yaw; at -90, delta = roll+yaw. Splitting delta
       keeps both components within 90 degrees. Round delta only once. */
    if (reach >= pole) {
        for (i = 0; i < 2; i++) {
            int sign = i == 0 ? 1 : -1;
            double a = n[0] - sign * n[2], b = n[3] + sign * n[1];
            int delta = ps_wrap_units(ps_round_units(2.0 * atan2(a, b)));
            int roll = ps_round_units(0.5 * delta * DG_IK_PS_UNIT_RAD);
            candidates[2 + i][0] = (short)roll;
            candidates[2 + i][1] = (short)(sign * pole);
            candidates[2 + i][2] = (short)(sign * (roll - delta));
        }
        count = 4;
    }
    for (i = 0; i < count; i++) {
        double dot = 0.0;
        ps_triple_quat(candidates[i], trial);
        for (k = 0; k < 4; k++) dot += n[k] * trial[k];
        if (fabs(dot) > best_dot) {
            best_dot = fabs(dot);
            best_i = i;
            for (k = 0; k < 4; k++) best[k] = trial[k];
        }
    }
    for (k = 0; k < 3; k++) out[k] = candidates[best_i][k];
    best_flags = DG_IK_PS_PROJECTED;
    if (best_i == 1) best_flags |= DG_IK_PS_ALT_NAMING;
    if (out[1] == pole || out[1] == -pole) best_flags |= DG_IK_PS_NEAR_POLE;
    if (best_i == 0) best_flags |= f & DG_IK_PS_VY_CLAMPED;
    if (fitted) for (k = 0; k < 4; k++) fitted[k] = best[k];
    if (retained) {
        double angle = 2.0 * acos(fabs(n[3]) > 1.0 ? 1.0 : fabs(n[3]));
        double error = 2.0 * acos(best_dot > 1.0 ? 1.0 : best_dot);
        *retained = angle > 1e-12 ? 1.0 - error / angle : 0.0;
        if (*retained < 0.0) *retained = 0.0;
    }
    if (flags) *flags = best_flags;
    return 1;
}
