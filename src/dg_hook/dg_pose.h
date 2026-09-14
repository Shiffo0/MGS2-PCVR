

#ifndef DG_POSE_H
#define DG_POSE_H

/* These must agree with DG_EYE_* in dg_xr.h, which are the OpenXR view indices.
   They are restated rather than included because this file must build with
   nothing but a C compiler - dg_xr.h drags in the runtime's world. If those two
   ever disagree, the arm latches on the wrong frame of the pair and the fault
   shows up as one eye lagging the other, so they are named identically here to
   make the mismatch obvious to anyone reading both. */
#define DG_POSE_EYE_LEFT   0
#define DG_POSE_EYE_RIGHT  1
#define DG_POSE_EYE_MONO  (-1)      /* stereo off: every frame is its own pair */

/* Everything the module does that the caller cannot see from the pose alone is
   reported here. A pose that was silently held, silently clamped or silently
   faded is indistinguishable at the call site from tracking that is simply
   working, and the difference is exactly what anyone debugging a broken arm
   needs to know.

   Two kinds are mixed in this list. BLEND_IN, BLEND_OUT, RELEASED and
   RATE_LIMIT describe the POSE being published, so the second frame of an eye
   pair repeats them - the pose it shows really is mid-blend. STALE, REFUSED and
   SEQ describe THIS FRAME'S INPUT, and a frame that only replayed the latch
   never looked at its input, so it reports only what went wrong with itself.
   Counting SEQ therefore counts anomalies, not anomalies times two. */
enum {
    /* This frame did not sample anything: it replayed the pose its pair was
       opened with. Expected on roughly half of all frames in stereo - that is
       the module working, not a fault. */
    DG_POSE_F_LATCHED     = 1u << 0,
    /* Blend weight is rising towards 1 - tracking has come back. */
    DG_POSE_F_BLEND_IN    = 1u << 1,
    /* Blend weight is falling towards 0 - tracking is gone, stale or refused,
       and the joint is on its way back to the animation. */
    DG_POSE_F_BLEND_OUT   = 1u << 2,
    /* Weight reached 0. write is 0 and the joint belongs to the animation
       again. Always paired with write == 0. */
    DG_POSE_F_RELEASED    = 1u << 3,
    /* The step from the previous published pose to this sample exceeded
       max_speed_mm_s or max_ang_speed_deg_s and was clamped. One or two of
       these is a tracking glitch; a continuous stream of them means the limit
       is set below what a human arm actually does. */
    DG_POSE_F_RATE_LIMIT  = 1u << 4,
    /* age_ms exceeded max_age_ms. The sample was real but too old to steer a
       hand with, so it is treated as a tracking loss. */
    DG_POSE_F_STALE       = 1u << 5,
    /* The input was refused outright: non-finite, a non-unit quaternion, a
       negative age, a non-positive dt, or a NULL pointer. Refused input never
       reaches the pose; it drives the blend out, same as a loss. */
    DG_POSE_F_REFUSED     = 1u << 6,
    /* The frame stream was not the clean L,R,L,R the renderer is supposed to
       produce: a repeat, a step backwards, a gap, a right eye with no left
       before it, an unrecognised eye value, or a dt so large the frame is a
       hitch rather than a frame. Handling is always deterministic and always
       errs towards holding the pair's pose. */
    DG_POSE_F_SEQ         = 1u << 7
};

typedef struct {

    double max_age_ms;
    double blend_in_s;          /* time for weight 0 -> 1 */
    double blend_out_s;         /* time for weight 1 -> 0 */
    double max_speed_mm_s;      /* wrist translation limit, game units/second */
    double max_ang_speed_deg_s; /* wrist rotation limit */
    /* dt above this is treated as a frame gap: it is clamped and flagged.
       Without it a three second alt-tab would hand the rate limiter a three
       second budget, which is a licence to teleport exactly when the pose is
       least trustworthy. */
    double max_dt_s;
} DG_POSE_CFG;

typedef struct {
    int           valid;    /* the tracker reported a usable pose this frame */
    double        pos[3];   /* wrist target, game units */
    double        quat[4];  /* orientation, {x,y,z,w}, w last, unit length */
    double        age_ms;   /* how old that sample is */
    unsigned long frame;    /* monotonic frame counter from the renderer */
    int           eye;      /* DG_POSE_EYE_*: which eye this frame draws */
    double        dt;       /* seconds since the previous frame */
} DG_POSE_IN;

typedef struct {
    int      write;         /* 1 = write to the game, 0 = leave the joint alone */
    double   pos[3];
    double   quat[4];       /* always unit length on a write */
    double   weight;        /* 0..1, the blend weight actually applied */
    unsigned flags;         /* DG_POSE_F_* */
} DG_POSE_OUT;

/* Caller-owned. Zeroed and filled by dg_pose_init; nothing else may touch it.
   Laid out in the header rather than hidden behind a pointer because there is
   no allocator in this module by design and a test needs to be able to put a
   hundred of these on the stack. */
typedef struct {
    DG_POSE_CFG   cfg;
    DG_POSE_OUT   latch;        /* what this pair publishes, both frames of it */
    double        hold_pos[3];  /* last published target: the rate-limit anchor */
    double        hold_quat[4];
    double        weight;
    /* Seconds banked by frames that only replayed the latch. The blend has to
       run on wall-clock time, not on pairs, or its duration would depend on
       whether stereo happens to be on. */
    double        pending_dt;
    unsigned long last_frame;
    int           have_frame;   /* last_frame means something */
    int           pair_open;    /* a left frame is waiting for its right frame */
    int           have_hold;    /* hold_pos/hold_quat carry a real pose */
} DG_POSE_STATE;

/* Fills cfg with the defaults documented above: 100 ms, 0.15 s in, 0.20 s out,
   3000 mm/s, 720 deg/s, 0.10 s max dt. */
void dg_pose_cfg_default(DG_POSE_CFG *cfg);

/* Zero-initialises s and adopts cfg. A NULL cfg means the defaults.
   Non-finite or non-positive config entries are replaced by their default
   rather than refused: init has no way to report, and a module that refused
   every frame for the rest of the session because one marker line was mistyped
   would be a worse failure than one that runs with a sane number. blend times
   of exactly 0 ARE honoured - that means "no blend", which is a legitimate
   thing to ask for. */
void dg_pose_init(DG_POSE_STATE *s, const DG_POSE_CFG *cfg);

/* One frame. Never fails, never refuses to produce an answer: on any input at
   all *out is fully written, and out->write says whether the answer is one the
   caller may hand to the game. */
void dg_pose_step(DG_POSE_STATE *s, const DG_POSE_IN *in, DG_POSE_OUT *out);

/* Shortest-arc constant-speed blend from a to b, written to out as
   {x, y, z, w} and always unit length.

   Public because it is the one piece of maths in here worth testing on its own,
   and because getting it wrong is invisible until it is not: q and -q are the
   same rotation, so a blend that does not check the sign of the dot product
   takes the long way round for half of all input pairs and spins the hand
   through most of a full turn to reach an orientation a few degrees away.
   Downstream refuses non-unit quaternions and a refused frame is a dropped
   frame, so the result is renormalised unconditionally. */
void dg_pose_quat_blend(const double a[4], const double b[4], double t,
                        double out[4]);

#endif
