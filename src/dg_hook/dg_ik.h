#ifndef DG_IK_H
#define DG_IK_H

/* Clamp and diagnostic flags. Every clamp the solver applies is reported -
   a silently clamped target is indistinguishable from a tracking failure at
   the call site, and the caller needs to be able to tell "your arm is longer
   than Raiden's" from "the solver broke". */
enum {

    DG_IK_CLAMP_REACH       = 1u << 0,
    /* |target - shoulder| was inside |upper - fore|, where no triangle exists
       at all. Happens when the target is drawn in against the chest. */
    DG_IK_CLAMP_TOO_CLOSE   = 1u << 1,
    /* The interior elbow angle would have left [min_elbow_deg, max_elbow_deg],
       so the target distance was pulled back to the limit. */
    DG_IK_CLAMP_ELBOW_LIMIT = 1u << 2,
    /* The pole hint carried little or no information about which way the
       elbow should point - it was parallel to the shoulder-to-target axis, or
       zero length - so a deterministic fallback direction contributed to the
       answer. See the fallback note in dg_ik.c. */
    DG_IK_POLE_FALLBACK     = 1u << 3,
    /* Non-finite input, a non-positive bone length, a max_reach_frac outside
       (0, 1], reversed elbow limits, or a zero-length shoulder-to-target
       vector. Always paired with a 0 return: nothing usable was produced. */
    DG_IK_BAD_INPUT         = 1u << 4
};

typedef struct {
    double shoulder[3];     /* world position of joint 4 */
    double target[3];       /* desired world position of the wrist */
    double pole[3];         /* world-space hint: which way the elbow points */
    double upper;           /* bone length shoulder->elbow */
    double fore;            /* bone length elbow->wrist */
    double max_reach_frac;  /* e.g. 0.99: never let the elbow lock straight */
    double min_elbow_deg;   /* interior elbow angle limits, e.g. 15 and 175 */
    double max_elbow_deg;
} DG_IK_IN;

typedef struct {
    double elbow[3];        /* solved world position of joint 5 */
    double wrist[3];        /* the target actually used, after clamping */
    double elbow_deg;       /* interior angle achieved at the elbow */
    double reach_used;      /* |wrist - shoulder| after clamping */
    unsigned clamped;       /* DG_IK_CLAMP_* / DG_IK_POLE_FALLBACK bits */
} DG_IK_OUT;

/* Solves the chain. Returns 1 on a usable solution, 0 if the input was
   degenerate enough that nothing should be written to the game - in which
   case *out is zeroed apart from DG_IK_BAD_INPUT in `clamped`.

   On a 1 return the bone lengths are exact to within double rounding:
   |elbow - shoulder| == upper and |wrist - elbow| == fore, whatever the input
   was. That invariant is the whole contract. A solver that stretches a bone
   produces a visibly broken arm rather than a merely misplaced one. */
int dg_ik_solve(const DG_IK_IN *in, DG_IK_OUT *out);

/* ------------------------------------------------------- orientation ----- */

/* Orientation flags, reported the same way and for the same reason. */
enum {
    /* The bone direction was exactly opposite its rest direction, so the
       rotation is a half turn about an axis the input does not pin down and a
       deterministic one was chosen. */
    DG_IK_ORIENT_UPPER_FLIP = 1u << 0,
    DG_IK_ORIENT_FORE_FLIP  = 1u << 1,
    DG_IK_ORIENT_BAD_INPUT  = 1u << 2,
    /* have_prev was set and honoured: both outputs are the points on their
       bone circles nearest the previous pair's quaternions. Raised only on
       the degenerate path now - see DG_IK_ORIENT_ANCHORED. */
    DG_IK_ORIENT_CONTINUED  = 1u << 3,

    DG_IK_ORIENT_ANCHORED   = 1u << 4
};

typedef struct {
    double shoulder[3];
    double elbow[3];
    double wrist[3];
    double rest_upper[3];   /* direction the upper bone points at rest */
    double rest_fore[3];    /* direction the forearm points at rest */
    /* A swing that takes rest onto the bone is one point on a whole CIRCLE
       of rotations that do (the free parameter is twist about the bone), and
       the shortest arc picks its point with no memory. Far from opposite
       that pick moves smoothly; near opposite it swings wildly while the arm
       barely moves - the walking-turn probe measured 175 degrees of joint
       jump per pair against 3 degrees of body yaw, which the hierarchy hands
       straight to joint 6 as visible tumbling. With have_prev set, each
       output is instead the point on ITS OWN circle nearest the previous
       pair's quaternion: still exactly rest onto bone, but continuous in
       every input. Zeroed have_prev is the old behaviour, bit for bit. */
    int have_prev;
    double prev_upper_quat[4];
    double prev_fore_quat[4];
} DG_IK_ORIENT_IN;

typedef struct {

    double upper_quat[4];   /* world swing of the shoulder joint */
    double fore_quat[4];    /* residual world swing after upper_quat */
    unsigned flags;         /* DG_IK_ORIENT_* */
} DG_IK_ORIENT_OUT;

/* Shortest-arc rotation taking unit-ish direction `from` onto `to`, written to
   q as {x, y, z, w}. Returns 1 on success, 0 on non-finite or zero-length
   input (q is then set to identity). *flipped, when non-NULL, is set to 1 if
   the two directions were opposite and the half-turn axis had to be chosen. */
int dg_ik_swing_quat(const double from[3], const double to[3], double q[4],
                     int *flipped);

/* The two hierarchical world-space joint swings implied by a solved chain.
   upper_quat takes rest_upper to the upper bone. fore_quat then takes the
   already-upper-rotated rest_fore to the forearm bone. Kept apart from
   dg_ik_solve so it can be tested on hand-built triangles that never went
   through the solver. Returns 1 on success, 0 on bad input. */
int dg_ik_orient(const DG_IK_ORIENT_IN *in, DG_IK_ORIENT_OUT *out);

/* --------------------------------------------------- hand orientation ---- */

/* Quaternion algebra, {x, y, z, w} with w LAST throughout, matching the rest
   of this header and the engine's own FVECTOR layout. The product is the
   ordinary Hamilton product: dg_ik_quat_mul(a, b, out) is the rotation that
   applies b first and a second. Aliasing is allowed - out may be a or b. */
void dg_ik_quat_mul(const double a[4], const double b[4], double out[4]);

/* Inverse of a unit quaternion. */
void dg_ik_quat_conj(const double q[4], double out[4]);

/* Normalises in place. Returns 0 and leaves identity behind if the input was
   non-finite or too short to carry a direction: a non-unit quaternion is not
   a rotation, and letting one through would scale the hand as well as turn
   it. */
int dg_ik_quat_normalize(double q[4]);

/* The unit quaternion of a rotation given as three basis rows, which is how
   the engine stores a joint matrix: basis[r] is the WORLD direction of the
   joint's local axis r. The result satisfies "rotating local axis r by q
   gives basis[r]", so no caller has to decide whether the stored matrix wants
   transposing - the one decision that would produce a confidently wrong pose
   rather than an obviously broken one.

   Returns 0 if the rows are non-finite, degenerate or left-handed. A mirrored
   basis has no rotation quaternion at all, and quietly picking the nearest one
   would turn a bad read into a plausible hand. */
int dg_ik_basis_quat(const double basis[3][3], double q[4]);

/* Angle in radians of the short arc between two unit quaternions, for
   reporting how far an achieved rotation missed the one that was asked for. */
double dg_ik_quat_angle(const double a[4], const double b[4]);

typedef struct {
    double live[4];         /* joint 6's world rotation, read this pair */
    double prev_upper[4];   /* what was written to joint 4 on the last pair */
    double prev_fore[4];    /* joint 5, last pair */
    double prev_hand[4];    /* joint 6, last pair */
    double new_upper[4];    /* what is about to be written to joint 4 */
    double new_fore[4];     /* joint 5, this pair */
    double desired[4];      /* the world rotation the hand should end up in */
} DG_IK_HAND_IN;

/* Solves for the joint-6 adjustment. On a 1 return, out is the unit
   quaternion for which

       out (x) new_fore (x) new_upper (x) animated == desired

   holds to within double rounding, with `animated` recovered from the inputs
   as described above. Returns 0 and leaves identity in out if any input was
   not a usable rotation. */
int dg_ik_hand_adjust(const DG_IK_HAND_IN *in, double out[4]);

/* Stateless visual safety policy for the adjustment returned above. The raw
   hand adjustment is decomposed about the solved forearm's WORLD axis:

       raw = swing (x) twist

   A bounded part of twist is moved to joint 5, where rotating about the
   solved forearm axis cannot move the wrist point. The remaining wrist swing
   and twist are independently bounded. These are explicit visual envelopes,
   not medical/anatomical measurements; callers choose the limits.

   On success, the bounded chain adjustment is

       wrist_world (x) fore_twist_world

   and is exactly raw_world whenever its swing and combined twist fit within
   the supplied envelopes. Returns 0 and identities on invalid input. */
enum {
    DG_IK_HAND_F_FORE_TWIST = 1u << 0,
    DG_IK_HAND_F_SWING_LIMITED = 1u << 1,
    DG_IK_HAND_F_TWIST_LIMITED = 1u << 2
};

typedef struct {
    double raw_world[4];
    double fore_axis_world[3];
    double max_fore_twist_rad;
    double max_wrist_swing_rad;
    double max_wrist_twist_rad;

    int have_prev_twist;
    double prev_twist_rad;

    int have_prev_twist_raw;
    double prev_twist_raw_rad;

    int have_prev_swing;
    double prev_swing_rad;
    double prev_swing_axis[3];
    /* And the swing's fold hysteresis reference beside it, the exact
       mirror of prev_twist_raw_rad: out.raw_swing_rad fed straight back.
       The swing's fold midpoint sits at cap+180 = 250 the same way. */
    int have_prev_swing_raw;
    double prev_swing_raw_rad;
} DG_IK_HAND_STABILIZE_IN;

typedef struct {
    double fore_twist_world[4];
    double wrist_world[4];
    double raw_swing_rad;       /* signed and unwrapped when prev was given;
                                   the plain [0,180] short arc otherwise */
    double raw_twist_rad;       /* signed; unwrapped when prev was given */
    double fore_twist_rad;      /* signed amount transferred to joint 5 */
    double wrist_swing_rad;     /* signed like raw_swing_rad; the APPLIED
                                   swing, the caller's next prev_swing_rad */
    double wrist_twist_rad;     /* signed amount left on joint 6 */
    /* The unit axis wrist_swing_rad's signed reading is about - the
       caller's next prev_swing_axis. All zero when the swing is identity
       and no reference was given to carry the convention across it. */
    double wrist_swing_axis[3];
    unsigned flags;
} DG_IK_HAND_STABILIZE_OUT;

int dg_ik_hand_stabilize(const DG_IK_HAND_STABILIZE_IN *in,
                         DG_IK_HAND_STABILIZE_OUT *out);

/* -------------------------------------------------- PS2 angle channel ---- */

#define DG_IK_PS_UNITS_PER_TURN 4096

/* Diagnostics from dg_ik_quat_to_ps_angles. Reported for the same reason the
   solver reports its clamps: an angle triple that quietly lost a degree of
   freedom looks exactly like one that did not, and the caller is the only one
   who can decide whether that matters. */
enum {
    /* Gimbal lock. |pitch| was within a rounding of 90 degrees, where this
       Euler convention pins only vx - vz (at +90) or vx + vz (at -90) and not
       the two separately. A deterministic split was used: the whole of the
       free parameter went into out[0] and out[2] was set to 0. The rotation
       that comes back is still the one asked for - only the naming of it is a
       choice this code made rather than one the input forced. */
    DG_IK_PS_SINGULAR    = 1u << 0,
    /* out[1] came out as exactly +/-1024, so whatever this code computed, the
       triple the ENGINE will reconstruct is at gimbal lock and only the sum or
       difference of out[0] and out[2] survives. Always set when
       DG_IK_PS_SINGULAR is, and also set in the band where the exact pitch was
       still well determined but quantisation put it on the pole anyway. */
    DG_IK_PS_NEAR_POLE   = 1u << 1,
    /* out[1] rounded to 0 and was pushed to +/-1. See the note on the
       function: 0 in that slot sends the engine down a different conversion
       entirely. */
    DG_IK_PS_VY_CLAMPED  = 1u << 2,
    /* Non-finite, or a norm further than 1e-6 from 1. Paired with a 0 return
       and a zeroed out[]. */
    DG_IK_PS_BAD_INPUT   = 1u << 3,
    /* dg_ik_quat_to_ps_reach: the primary ZYX naming had a component beyond
       the reach and the SECOND solution of the decomposition was used -
       (roll + 180, 180 - pitch, yaw + 180), the same rotation under another
       name. The engine reconstructs the rotation asked for; only the triple
       differs. */
    DG_IK_PS_ALT_NAMING  = 1u << 4,

    DG_IK_PS_UNREACHABLE = 1u << 5,
    /* dg_ik_ps_fit: the rotation was shortened along its own axis until a
       naming fit. out[] and fitted[] describe the shortened rotation, which
       IS reachable; *fraction says how much of the ask survived. */
    DG_IK_PS_SCALED      = 1u << 6,
    /* dg_ik_ps_project selected a nearby reachable rotation. Unlike SCALED,
       this need not preserve the input axis. */
    DG_IK_PS_PROJECTED   = 1u << 7
};

int dg_ik_quat_to_ps_angles(const double q[4], short out[3], unsigned *flags);

int dg_ik_ps_precompensate(const short want[3], int pull_divisor, short out[3]);

int dg_ik_ps_reach_units(int pull_divisor);

/* dg_ik_quat_to_ps_angles with the reach applied. Tries the primary naming,
   then the second solution of the ZYX decomposition, and returns the first
   whose three components all sit inside [-reach, reach]; flags carry
   DG_IK_PS_ALT_NAMING for the second. When neither fits, out[] is the primary
   triple and DG_IK_PS_UNREACHABLE is set - the return is still 1, because
   the triple is a faithful naming; it is the engine that cannot hold it.
   Returns 0 exactly when dg_ik_quat_to_ps_angles would. A reach below one
   unit is raised to one: the XYZ path needs a non-zero vy. */
int dg_ik_quat_to_ps_reach(const double q[4], int reach, short out[3],
                           unsigned *flags);

int dg_ik_ps_fit(const double q[4], int reach, double fitted[4],
                 short out[3], unsigned *flags, double *fraction);

int dg_ik_ps_project(const double q[4], int reach, double fitted[4],
                    short out[3], unsigned *flags, double *retained);

#endif
