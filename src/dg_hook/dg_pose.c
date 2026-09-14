/* dg_pose.c - the policy. See dg_pose.h for the shape and the contract.
 *
 * Order of business:
 *
 *   1. small helpers, all local;
 *   2. the quaternion blend, which is the only real maths in here;
 *   3. input validation - what counts as a pose at all;
 *   4. the blend weight and the rate limiter - what a usable pose becomes;
 *   5. dg_pose_step: the frame sequence and the eye pair latch.
 *
 * Section 5 is the one to read twice. Everything else in this file could be
 * slightly wrong and produce an arm that looks slightly wrong; section 5 being
 * wrong produces two different arms in the two eyes, which is not a degraded
 * picture but a broken one.
 */

#include <math.h>
#include "dg_pose.h"

#if defined(_MSC_VER)
#include <float.h>
#define DG_POSE_FINITE(v) (_finite(v) != 0)
#else
#define DG_POSE_FINITE(v) (isfinite(v) != 0)
#endif

#define DG_POSE_PI 3.14159265358979323846

/* How far off unit an incoming quaternion may be. OpenXR hands out floats and
   the conversion into doubles here leaves a length error of order 1e-7, so this
   is loose by three orders of magnitude on purpose: it is a garbage detector,
   not a precision check, and a threshold tight enough to catch rounding would
   reject perfectly good tracking. */
#define DG_POSE_QUAT_TOL 1e-3

/* Above this |dot| the two quaternions are close enough that sin(theta) in the
   slerp weights is heading for zero and the division stops being meaningful.
   A straight lerp is within a rounding error of slerp at that separation. */
#define DG_POSE_SLERP_LINEAR 0.999999

/* ======================================================== small helpers == */

static double clamp01(double v)
{
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

static int finite3(const double a[3])
{
    return DG_POSE_FINITE(a[0]) && DG_POSE_FINITE(a[1]) && DG_POSE_FINITE(a[2]);
}

static int finite4(const double a[4])
{
    return DG_POSE_FINITE(a[0]) && DG_POSE_FINITE(a[1]) &&
           DG_POSE_FINITE(a[2]) && DG_POSE_FINITE(a[3]);
}

static void copy3(const double a[3], double out[3])
{
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
}

static void copy4(const double a[4], double out[4])
{
    out[0] = a[0];
    out[1] = a[1];
    out[2] = a[2];
    out[3] = a[3];
}

static void identity_quat(double q[4])
{
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 1.0;
}

static double quat_len(const double q[4])
{
    return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

/* Renormalises in place, falling back to identity when there is nothing to
   normalise. Never leaves a non-unit quaternion behind: the joint writer
   downstream refuses those outright (dg_bridge counts them in
   c_arm_quat_refused), and a refused frame is a dropped frame - the arm would
   stutter for reasons invisible at the call site. */
static void quat_normalize(double q[4])
{
    double len = quat_len(q);
    if (!DG_POSE_FINITE(len) || len <= 0.0) {
        identity_quat(q);
        return;
    }
    q[0] /= len;
    q[1] /= len;
    q[2] /= len;
    q[3] /= len;
}

static double quat_dot(const double a[4], const double b[4])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

/* acos with the argument pinned, for the same reason dg_ik.c pins it: a dot
   product of two unit quaternions lands a few ulps outside [-1, 1] constantly,
   and acos(1.0000000000000002) is a NaN that would become a joint rotation. */
static double acos_clamped(double c)
{
    if (c <= -1.0) return DG_POSE_PI;
    if (c >= 1.0) return 0.0;
    return acos(c);
}

/* ====================================================== quaternion blend = */

void dg_pose_quat_blend(const double a[4], const double b[4], double t,
                        double out[4])
{
    double bb[4];
    double dot;
    double theta;
    double sin_theta;
    double wa;
    double wb;
    int i;

    if (!out) return;
    if (!a || !b || !finite4(a) || !finite4(b) || !DG_POSE_FINITE(t)) {
        /* Nothing here can be salvaged into a rotation. Identity is the only
           answer that is certainly harmless, and the caller sees it as the arm
           sitting at its rest orientation rather than as a NaN reaching the
           engine's matrix code. */
        identity_quat(out);
        return;
    }

    /* The endpoints are exact rather than "whatever the general formula happens
       to give at t = 0 and t = 1". The rate limiter leans on that: a step it
       did not clamp must reproduce the input sample bit for bit, or two
       supposedly identical frames would differ in the last bit and the eye-pair
       identity check would be a lie. */
    if (t <= 0.0) {
        copy4(a, out);
        quat_normalize(out);
        return;
    }
    if (t >= 1.0) {
        copy4(b, out);
        quat_normalize(out);
        return;
    }

    copy4(b, bb);
    dot = quat_dot(a, b);
    /* q and -q are the same rotation, so the two endpoints name FOUR arcs
       between them and only the short one is the one a hand should travel. A
       negative dot product means the long way round is what the formula would
       give; flipping the far endpoint picks the short arc instead. Without
       this, half of all blends sweep the hand through most of a full turn to
       reach an orientation that was a few degrees away. */
    if (dot < 0.0) {
        bb[0] = -bb[0];
        bb[1] = -bb[1];
        bb[2] = -bb[2];
        bb[3] = -bb[3];
        dot = -dot;
    }

    if (dot > DG_POSE_SLERP_LINEAR) {
        /* Nearly the same rotation: sin(theta) is on its way to zero and the
           slerp weights below turn into 0/0. Lerp and renormalise; at this
           separation the two agree to well inside a float. */
        for (i = 0; i < 4; i++)
            out[i] = a[i] + t * (bb[i] - a[i]);
        quat_normalize(out);
        return;
    }

    theta = acos_clamped(dot);
    sin_theta = sin(theta);
    if (!DG_POSE_FINITE(sin_theta) || sin_theta <= 0.0) {
        for (i = 0; i < 4; i++)
            out[i] = a[i] + t * (bb[i] - a[i]);
        quat_normalize(out);
        return;
    }

    /* True slerp rather than a normalised lerp, because the rate limiter asks
       for a fraction of an ANGLE and expects to get exactly that fraction.
       nlerp would deliver a smaller step than asked for in the middle of a
       large arc, which is the case the limiter exists for. */
    wa = sin((1.0 - t) * theta) / sin_theta;
    wb = sin(t * theta) / sin_theta;
    for (i = 0; i < 4; i++)
        out[i] = wa * a[i] + wb * bb[i];
    quat_normalize(out);
}

/* Angle of the short arc between two unit quaternions, in radians. */
static double quat_angle(const double a[4], const double b[4])
{
    double dot = quat_dot(a, b);
    if (dot < 0.0) dot = -dot;
    return 2.0 * acos_clamped(dot);
}

/* ============================================================ config ===== */

void dg_pose_cfg_default(DG_POSE_CFG *cfg)
{
    if (!cfg) return;

    cfg->max_age_ms = 100.0;
    /* Asymmetric on purpose. Coming back should be quick enough that the player
       does not notice the hand arriving late; going away should be slow enough
       that a single dropped tracking sample does not read as a flicker, because
       a hand that blinks is more distracting than one that fades. */
    cfg->blend_in_s = 0.15;
    cfg->blend_out_s = 0.20;
    /* 3 m/s at the wrist. A fast human punch is around that; anything quicker
       arriving from a tracker is the tracker, not the player. */
    cfg->max_speed_mm_s = 3000.0;
    cfg->max_ang_speed_deg_s = 720.0;
    cfg->max_dt_s = 0.10;
}

static double cfg_positive(double v, double fallback)
{
    if (!DG_POSE_FINITE(v) || v <= 0.0) return fallback;
    return v;
}

static double cfg_non_negative(double v, double fallback)
{
    if (!DG_POSE_FINITE(v) || v < 0.0) return fallback;
    return v;
}

void dg_pose_init(DG_POSE_STATE *s, const DG_POSE_CFG *cfg)
{
    DG_POSE_CFG def;
    int i;

    if (!s) return;

    /* Field by field rather than memset: the struct is all doubles and ints
       whose zero representation is what is wanted, but writing it out means a
       field added later without a decision about its initial value shows up as
       a compile-time omission rather than as a silent zero. */
    for (i = 0; i < 3; i++) s->hold_pos[i] = 0.0;
    identity_quat(s->hold_quat);
    s->weight = 0.0;
    s->pending_dt = 0.0;
    s->last_frame = 0;
    s->have_frame = 0;
    s->pair_open = 0;
    s->have_hold = 0;

    s->latch.write = 0;
    for (i = 0; i < 3; i++) s->latch.pos[i] = 0.0;
    identity_quat(s->latch.quat);
    s->latch.weight = 0.0;
    s->latch.flags = DG_POSE_F_RELEASED;

    dg_pose_cfg_default(&def);
    if (!cfg) {
        s->cfg = def;
        return;
    }
    s->cfg.max_age_ms = cfg_positive(cfg->max_age_ms, def.max_age_ms);
    /* Zero is meaningful for the blend times - "no blend, switch instantly" -
       so these accept it where the others do not. */
    s->cfg.blend_in_s = cfg_non_negative(cfg->blend_in_s, def.blend_in_s);
    s->cfg.blend_out_s = cfg_non_negative(cfg->blend_out_s, def.blend_out_s);
    s->cfg.max_speed_mm_s = cfg_positive(cfg->max_speed_mm_s,
                                         def.max_speed_mm_s);
    s->cfg.max_ang_speed_deg_s = cfg_positive(cfg->max_ang_speed_deg_s,
                                              def.max_ang_speed_deg_s);
    s->cfg.max_dt_s = cfg_positive(cfg->max_dt_s, def.max_dt_s);
}

/* ======================================================== the pose ======= */

/* What this frame's sample is worth. Returns 1 when the pose may steer the
   arm; sets *refused for input that is not a pose at all and *stale for a pose
   that is real but too old. The two are separated because they mean different
   things to whoever is reading the counters: refused is a bug or a garbage
   tracker, stale is a runtime that stopped delivering. */
static int sample_usable(const DG_POSE_CFG *cfg, const DG_POSE_IN *in,
                         int *refused, int *stale)
{
    double len;

    *refused = 0;
    *stale = 0;

    if (!finite3(in->pos) || !finite4(in->quat) ||
        !DG_POSE_FINITE(in->age_ms)) {
        *refused = 1;
        return 0;
    }
    /* A negative age is a clock that ran backwards or a field that was never
       filled in. Either way the number cannot be used to decide freshness, and
       guessing that it means "very fresh" is the guess that hurts. */
    if (in->age_ms < 0.0) {
        *refused = 1;
        return 0;
    }
    len = quat_len(in->quat);
    /* A non-unit quaternion is not a rotation. It is usually a struct that was
       never written, a component ordering mistake, or three good components and
       one piece of memory that was something else - none of which should be
       fixed up and used, because a normalised piece of garbage is still
       garbage and it would drive the arm somewhere confidently wrong. */
    if (!DG_POSE_FINITE(len) || fabs(len - 1.0) > DG_POSE_QUAT_TOL) {
        *refused = 1;
        return 0;
    }
    if (in->age_ms > cfg->max_age_ms) {
        *stale = 1;
        return 0;
    }
    if (!in->valid) return 0;
    return 1;
}

/* Moves w towards target at a fixed rate, never overshooting. A zero blend time
   means the caller asked for no blend at all, which is honoured rather than
   turned into a divide by zero. */
static double advance_weight(double w, double target, double dt, double time)
{
    double step;

    /* No time passed, so nothing moved. Checked before the zero-blend case so
       that a refused dt cannot make an instant blend fire on a frame that did
       not happen. */
    if (dt <= 0.0) return w;
    if (time <= 0.0) return target;
    step = dt / time;
    if (target > w) {
        w += step;
        if (w > target) w = target;
    } else if (target < w) {
        w -= step;
        if (w < target) w = target;
    }
    return w;
}

/* Takes the held pose towards the sample, clamped so that neither translation
   nor rotation can exceed the configured speed over dt. Returns 1 if a clamp
   was applied.

   The clamp is a single fraction applied to both channels, not two independent
   clamps. Clamping position while letting orientation jump would put the wrist
   halfway to the new place with the hand already fully turned, which looks
   worse than either error on its own - the hand appears to detach from the
   arm. */
static int rate_limit(DG_POSE_STATE *s, const DG_POSE_IN *in, double dt)
{
    double d[3];
    double dist;
    double max_dist;
    double ang;
    double max_ang;
    double frac = 1.0;
    double f;
    double blended[4];
    int i;

    /* Nothing on screen to move away from: the joint is released, so the first
       accepted pose after a loss must ARRIVE where the controller is rather
       than crawl there from wherever the hand was last seen. The blend in is
       what hides the arrival; rate limiting it as well would drag the arm
       across the room at walking pace in full view. */
    if (!s->have_hold) {
        copy3(in->pos, s->hold_pos);
        copy4(in->quat, s->hold_quat);
        quat_normalize(s->hold_quat);
        s->have_hold = 1;
        return 0;
    }

    d[0] = in->pos[0] - s->hold_pos[0];
    d[1] = in->pos[1] - s->hold_pos[1];
    d[2] = in->pos[2] - s->hold_pos[2];
    dist = sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    max_dist = s->cfg.max_speed_mm_s * dt;
    if (dist > max_dist && dist > 0.0) {
        f = max_dist / dist;
        if (f < frac) frac = f;
    }

    ang = quat_angle(s->hold_quat, in->quat);
    max_ang = s->cfg.max_ang_speed_deg_s * (DG_POSE_PI / 180.0) * dt;
    if (ang > max_ang && ang > 0.0) {
        f = max_ang / ang;
        if (f < frac) frac = f;
    }

    if (frac >= 1.0) {
        /* Copied, not interpolated with t = 1. An unclamped frame has to
           reproduce the sample exactly; a multiply-add that lands one ulp away
           would make two identical inputs produce two different poses. */
        copy3(in->pos, s->hold_pos);
        copy4(in->quat, s->hold_quat);
        quat_normalize(s->hold_quat);
        return 0;
    }

    for (i = 0; i < 3; i++)
        s->hold_pos[i] = s->hold_pos[i] + frac * d[i];
    dg_pose_quat_blend(s->hold_quat, in->quat, frac, blended);
    copy4(blended, s->hold_quat);
    return 1;
}

/* Publishes whatever the state currently holds. Every exit from dg_pose_step
   that produces a fresh pose goes through here, so there is exactly one place
   that decides what a weight of zero means. */
static void emit(const DG_POSE_STATE *s, double prev_weight, unsigned flags,
                 DG_POSE_OUT *out)
{
    copy3(s->hold_pos, out->pos);
    copy4(s->hold_quat, out->quat);
    out->weight = s->weight;
    out->write = (s->weight > 0.0) ? 1 : 0;

    if (s->weight > prev_weight) flags |= DG_POSE_F_BLEND_IN;
    else if (s->weight < prev_weight) flags |= DG_POSE_F_BLEND_OUT;
    /* Released is about the joint, not about the blend: it says the animation
       owns this joint again, which is exactly the frames where nothing is
       written. */
    if (!out->write) flags |= DG_POSE_F_RELEASED;
    out->flags = flags;
}

/* The flags that describe the POSE rather than the frame's input. Those are
   the ones a replay inherits: the pose it republishes really is mid-blend, or
   really was rate limited, and hiding that on every second frame would make the
   counters read half of what happened. The rest - stale, refused, sequence -
   describe an input the replay path deliberately never looked at, so inheriting
   them would report a frame as anomalous on the strength of a different
   frame's problems, and double every anomaly in the log. */
#define DG_POSE_POSE_FLAGS (DG_POSE_F_BLEND_IN | DG_POSE_F_BLEND_OUT | \
                            DG_POSE_F_RELEASED | DG_POSE_F_RATE_LIMIT)

/* Replays the pair's latched pose. The input for this frame is deliberately not
   consulted at all - not its pose, not its validity, not its age. That is the
   whole point: the second frame of an eye pair shows the player the same arm
   the first frame did, and anything that could make it differ has to be kept
   out of this path. */
static void replay(const DG_POSE_STATE *s, unsigned extra, DG_POSE_OUT *out)
{
    *out = s->latch;
    out->flags = (out->flags & DG_POSE_POSE_FLAGS) | DG_POSE_F_LATCHED | extra;
}

/* ============================================================ step ======= */

void dg_pose_step(DG_POSE_STATE *s, const DG_POSE_IN *in, DG_POSE_OUT *out)
{
    unsigned flags = 0;
    unsigned long delta;
    double dt;
    double prev_weight;
    double target;
    int refused = 0;
    int stale = 0;
    int usable;
    int eye;
    int mono;
    int next_pair_open;

    if (!out) return;
    if (!s) {
        /* No state, so there is nothing to hold and nothing to blend. Say so
           and write nothing to the game. */
        out->write = 0;
        out->pos[0] = 0.0; out->pos[1] = 0.0; out->pos[2] = 0.0;
        identity_quat(out->quat);
        out->weight = 0.0;
        out->flags = DG_POSE_F_REFUSED | DG_POSE_F_RELEASED;
        return;
    }
    if (!in) {
        /* A frame with no input record at all: no pose, no dt and - the part
           that matters - no eye, so there is no way to tell whether this frame
           opens a pair or closes one. Publishing anything new without knowing
           that could put a second pose inside a perceived image, so the latch
           is replayed and the pair bookkeeping is left exactly as it was. */
        replay(s, DG_POSE_F_REFUSED, out);
        return;
    }

    /* ---- frame sequence ------------------------------------------------ */

    dt = in->dt;
    if (!DG_POSE_FINITE(dt) || dt <= 0.0) {
        /* A frame that took no time did not happen, or the caller's clock is
           broken. Either way there is no interval to blend over or to rate
           limit against, and inventing one would make the arm's speed depend
           on a number that is already known to be wrong. */
        flags |= DG_POSE_F_REFUSED;
        dt = 0.0;
    } else if (dt > s->cfg.max_dt_s) {
        /* A hitch, an alt-tab or a loading screen. It is still one frame, but
           it is not a frame's worth of time, and handing the rate limiter the
           real number would licence exactly the teleport it exists to stop. */
        dt = s->cfg.max_dt_s;
        flags |= DG_POSE_F_SEQ;
    }

    if (s->have_frame) {
        /* Unsigned difference, so a counter that wraps reads as a small step
           forward rather than as an enormous step backwards. Half the range is
           the dividing line, which at 90 Hz is years away from being reached by
           anything except a genuine reordering. */
        delta = in->frame - s->last_frame;
        if (delta == 0 || delta > (~0UL >> 1)) {
            /* A repeated frame number, or one from the past. Both mean the pose
               this frame shows was already decided: a repeat is the same
               perceived image again, and a frame from the past carries a
               sample the arm has already moved beyond. Publishing anything new
               here is the one thing that cannot be allowed, so the latch is
               replayed and the time is banked for the next real frame.
               last_frame deliberately keeps the highest value seen, so a burst
               of out-of-order frames all hold instead of dragging the sequence
               backwards with them. */
            s->pending_dt += dt;
            replay(s, DG_POSE_F_SEQ | (flags & DG_POSE_F_REFUSED), out);
            return;
        }
        /* A gap: frames were dropped between that one and this. Nothing is
           broken by it - the pair those frames belonged to is simply gone - but
           the caller is told, because a stream full of gaps is a performance
           problem wearing a tracking problem's clothes. */
        if (delta > 1) flags |= DG_POSE_F_SEQ;
    }
    s->last_frame = in->frame;
    s->have_frame = 1;

    /* ---- which frame of which pair ------------------------------------- */

    eye = in->eye;
    mono = 0;
    if (eye == DG_POSE_EYE_MONO) {
        mono = 1;
    } else if (eye != DG_POSE_EYE_LEFT && eye != DG_POSE_EYE_RIGHT) {
        /* next_eye() in dg_hook.c collapses any malformed value to LEFT or
           RIGHT, so this cannot come from the renderer - it is a caller bug.
           Treating it as mono is the safe reading: every frame gets its own
           fresh pose, which can look like latency but can never show the two
           eyes different arms. */
        mono = 1;
        flags |= DG_POSE_F_SEQ;
    }

    if (mono) {
        next_pair_open = 0;
    } else if (eye == DG_POSE_EYE_LEFT) {
        /* LEFT opens a pair. That is not a convention chosen here: dg_hook.c
           sets g_current_eye to DG_EYE_LEFT when the hook arms and toggles it
           once per Present, so the first frame of the sequence is LEFT and
           every pair therefore begins on one. */
        if (s->pair_open) {
            /* The previous LEFT never got its RIGHT. Its pair was drawn to one
               eye only, which is a dropped frame somewhere upstream. */
            flags |= DG_POSE_F_SEQ;
        }
        next_pair_open = 1;
    } else if (s->pair_open) {
        /* The ordinary case: RIGHT closes the pair LEFT opened. */
        s->pending_dt += dt;
        s->pair_open = 0;
        replay(s, flags & (DG_POSE_F_SEQ | DG_POSE_F_REFUSED), out);
        return;
    } else {
        /* A RIGHT with no LEFT in front of it: either the LEFT was dropped or
           this stream was joined mid-pair. Holding the previous pair's pose
           would be safe for this frame but would freeze the arm outright if the
           pattern repeats, so it is treated as a pair of its own - one frame
           long, with nothing to be identical to - and reported. */
        flags |= DG_POSE_F_SEQ;
        next_pair_open = 0;
    }

    /* ---- sample, blend, publish ---------------------------------------- */

    dt += s->pending_dt;
    s->pending_dt = 0.0;

    usable = sample_usable(&s->cfg, in, &refused, &stale);
    if (refused) flags |= DG_POSE_F_REFUSED;
    if (stale) flags |= DG_POSE_F_STALE;

    prev_weight = s->weight;
    target = usable ? 1.0 : 0.0;
    s->weight = clamp01(advance_weight(s->weight, target, dt,
                                       usable ? s->cfg.blend_in_s
                                              : s->cfg.blend_out_s));

    if (usable) {
        if (rate_limit(s, in, dt)) flags |= DG_POSE_F_RATE_LIMIT;
    } else if (s->weight <= 0.0) {
        /* Fully released. The anchor is dropped with it, so when tracking
           returns the arm appears at the controller instead of being rate
           limited across the gap from wherever it was standing when the
           tracking died - which could be minutes and a room away. */
        s->have_hold = 0;
    }

    emit(s, prev_weight, flags, out);
    s->latch = *out;
    s->pair_open = next_pair_open;
}
