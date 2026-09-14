/* dg_ik.h - a standalone two-bone IK solver, world space only.
 *
 * The chain this exists for was measured live in the running game and is a
 * plain two-bone arm:
 *
 *     joint 4 (shoulder) --upper--> joint 5 (elbow) --fore--> joint 6 (wrist)
 *
 * Bone lengths differ per character (Raiden 290.00 / 245.61, Snake 295.00 /
 * 233.45, in game units where 1 unit = 1 mm), so they are inputs and never
 * constants. Nothing in here knows which character it is solving for.
 *
 * Everything below is pure: no globals, no allocation, no I/O, no windows.h,
 * no game memory. The same input always produces the same output, which is
 * what makes a desk test worth anything and what keeps the elbow from
 * wandering between frames on identical input.
 *
 * The solver stops at world space on purpose. Converting a world orientation
 * into the engine's per-joint adjust[] channel is a delta on top of the
 * animated pose whose exact space still has to be measured live; guessing at
 * it here would bury an unverified assumption inside verified math.
 */

#ifndef DG_IK_H
#define DG_IK_H

/* Clamp and diagnostic flags. Every clamp the solver applies is reported -
   a silently clamped target is indistinguishable from a tracking failure at
   the call site, and the caller needs to be able to tell "your arm is longer
   than Raiden's" from "the solver broke". */
enum {
    /* |target - shoulder| exceeded (upper + fore) * max_reach_frac. Expected
       constantly in normal use: a human arm in a headset is not the length of
       the character's arm. */
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
    /* The upper's free roll was pinned by the POSE (the child's rest
       direction turned as near the real forearm as a roll can put it)
       rather than by the previous pair. This is the normal path, and the
       property it buys is that the same arm pose always yields the same
       roll: a hand walked around a closed loop comes back to the roll it
       started with. Continuity alone did not - it has no restoring force,
       so every lap added roll (52.8 degrees per shoulder-height circle,
       measured 2026-08-21) at under a degree per step, which is the
       tumbling the headset saw and the instruments called calm. */
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

/* Everything needed to work out what to write to the hand joint.

   The engine applies a joint's adjust quaternion on the LEFT of that joint's
   absolute rotation, and a child inherits its parents' adjustments, so the
   hand's world rotation as read at the camera seam is

       live = prev_hand (x) prev_fore (x) prev_upper (x) animated

   where `animated` is the rotation the animation alone would have produced.
   That is why the previous pair's three quaternions are inputs: the matrices
   read at the seam already contain them, and treating `live` as the animation
   is exactly the feedback that made the arm spin.

   A write lands on the NEXT hierarchy pass, alongside the new shoulder and
   elbow quaternions, so the compensation must be computed against those and
   not against the ones still sitting in the matrices. */
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
    /* The twist angle is only defined modulo a full turn, and the short arc
       re-picks its branch every call: a raw twist drifting past 180 flips
       to -180, the clamps flip from +caps to -caps, and the hand snaps by
       twice the cap sum in one pair - the crouch tumble of 2026-08-20,
       measured at raw twist 166 with both clamps pinned and alternating.
       With have_prev_twist set, the angle is unwrapped to the branch
       nearest prev_twist_rad before anything is clamped or reported, so a
       twist that crosses 180 keeps walking instead of teleporting.

       prev_twist_rad is the twist the caller last APPLIED - the clamped
       fore + wrist sum, bounded by the caps - and deliberately NOT the raw
       reported angle. A raw reference can wind up a whole turn during a bad
       episode and then hold the clamps pinned forever: a true twist of -4
       reads as +356 against a 356 reference, which is the 2026-08-20 late
       latch the player could not shake off by re-aligning. Bounded by the
       caps, the reference keeps continuity across the 180 seam and folds a
       wound-up branch back on its own. Zeroed fields are the old behaviour,
       bit for bit. */
    int have_prev_twist;
    double prev_twist_rad;
    /* The continuity reference: the unwrapped raw twist this call
       reported LAST time (out.raw_twist_rad fed straight back). The
       branch walks on from it across the 180 seam, and folds back to the
       short arc once the walked name is outside the caps and longer than
       the short one by a 30-degree margin (2026-09-03, run 11): a branch
       is only a name for the rotation, inside the caps every name renders
       the same, and outside them the short name is never further out
       than the long one. Naming by nearest-to-APPLIED instead, as this
       did before, latched - with the applied pinned on a cap the nearest
       name of a neutral hand is a full turn away, and the hand stayed
       135 degrees wrong through a whole body turn. The margin is what
       keeps a demand hovering on the seam from being renamed every pair
       (under caps below 180 that rename snaps the applied by two caps -
       the 2026-08-21 evening tumble). The branch returned never exceeds
       180 + margin, so feeding it back cannot wind up. Zeroed fields
       walk from the applied reference instead, bit for bit the same
       branch for any cap above the margin. */
    int have_prev_twist_raw;
    double prev_twist_raw_rad;
    /* The swing has the same seam one axis over. Its short arc is always
       read as a POSITIVE angle, so when the raw swing crosses 180 the AXIS
       flips direction instead of the sign - and because the swing is
       clamped to max_wrist_swing_rad, the applied wrist teleports by twice
       the cap in one pair. That flap is the 2026-08-21 body-turn tumble:
       rotating the body sweeps the relative demand through 180, cmd-drift
       read spikes of 118 degrees between consecutive published commands,
       and rotating back swept it out again.

       With have_prev_swing set, the axis is first held to the hemisphere of
       prev_swing_axis (flipping axis and sign together names the SAME
       rotation - only the branch bookkeeping changes), and the now-signed
       angle is unwrapped to the branch nearest prev_swing_rad before the
       clamp, exactly like the twist above. prev_swing_rad is the swing the
       caller last APPLIED - bounded by the cap - for the same no-latch
       reason as the twist: against a bounded reference a wound-up demand
       folds back to the short reading the moment the player realigns.
       Zeroed fields are the old behaviour, bit for bit. */
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
    /* dg_ik_quat_to_ps_reach: neither naming fits inside the reach. out[] is
       the primary triple and the engine WILL NOT reproduce it: GV_NearExp4PV
       measures its way back to zero along the short arc of the turn, so a
       precompensated short past a half turn is folded before it is pulled
       and lands three quarters of a turn from the ask - the 90-degree hand
       flips of run 13 (2026-09-03, 278 dirty slot echoes at swing cap 150).
       dg_ik_ps_fit is the caller's way of never publishing this. */
    DG_IK_PS_UNREACHABLE = 1u << 5,
    /* dg_ik_ps_fit: the rotation was shortened along its own axis until a
       naming fit. out[] and fitted[] describe the shortened rotation, which
       IS reachable; *fraction says how much of the ask survived. */
    DG_IK_PS_SCALED      = 1u << 6,
    /* dg_ik_ps_project selected a nearby reachable rotation. Unlike SCALED,
       this need not preserve the input axis. */
    DG_IK_PS_PROJECTED   = 1u << 7
};

/* The inverse of GM_RotToQuat: fills out[] with the three shorts that the
   engine's own forward conversion turns back into q. Returns 1 on success, 0
   with out[] zeroed if q is non-finite or is not a unit quaternion to within
   1e-6 - a non-unit input is not a rotation, and normalising it silently would
   turn a caller's bug into a plausible pose. *flags, when non-NULL, receives
   the DG_IK_PS_* bits; it is written on failure too.

   Accuracy. The three angles land on a grid of 2*PI/4096 radians, so each is
   off by at most half a unit. Rotation angle is subadditive under composition,
   and perturbing one factor of Rz Ry Rx by d changes the product by at most
   |d| (conjugation preserves angle), so the round trip is out by at most
   1.5 units = 3*PI/4096 rad = 0.132 degrees. When DG_IK_PS_VY_CLAMPED is set
   the pitch error is a whole unit rather than half, giving 2 units = 0.176
   degrees. That is the floor imposed by the storage format, not by this code.

   out[1] is never 0, and that is deliberate rather than incidental. The engine
   branches on it - `if (ArmCamRotateShift.vy != 0) GM_RotToQuat(...) else
   GM_RotToQuatXAfterY(...)` - and the else branch is a different, two
   degree-of-freedom conversion that forces quat.vy = 0. A vy that rounded to 0
   would therefore not be a small angle, it would be a different function. It
   is clamped to +/-1, about 0.088 degrees, which is far below anything visible
   and keeps the full XYZ path. */
int dg_ik_quat_to_ps_angles(const double q[4], short out[3], unsigned *flags);

int dg_ik_ps_precompensate(const short want[3], int pull_divisor, short out[3]);

/* How far a component can be after the engine's pull. The pull runs on the
   SVECTOR as a 4096-unit angle: GV_NearExp4PV takes the way back to zero
   along the short arc, so whatever short is written, the value the pull
   starts from is inside a half turn, and after one pull of 1/D it is inside
   (2047 - 2047/D). At D == 4 that is 1536 units, 135 degrees. Writing a
   larger precompensated short does not reach further - it folds, and the
   engine lands on the far side (run 13, 2026-09-03). pull_divisor <= 1 means
   no pull, and the reach is the half turn itself. */
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

/* The rotation the channel can carry that is closest to q along q's own
   axis. Within reach (either naming) it is q itself, *fraction = 1 and the
   flags are those of dg_ik_quat_to_ps_reach. Otherwise the angle is shortened
   by bisection on [0, 1] - twenty halvings, a resolution of a millionth -
   to the largest fraction whose rotation about the same axis fits,
   DG_IK_PS_SCALED is set, fitted[] is that rotation (unit, w >= 0) and out[]
   its triple. Zero always fits (the identity is (0, +/-1, 0)), so the search
   cannot fail; the Euler components are not monotone in the fraction, so
   the fraction found is a fitting one with no fitting fraction above it in
   the bisection's path, which is all a graceful shortfall needs. Returns 0,
   with fitted[] the identity and *fraction 0, only for a non-rotation
   input. fitted and fraction may be NULL. */
int dg_ik_ps_fit(const double q[4], int reach, double fitted[4],
                 short out[3], unsigned *flags, double *fraction);

/* Runtime channel fitting. Preserve a reachable q; otherwise choose the
   least angular error among clamped primary/alternate ZYX triples and both
   nearest pitch-pole families (when reach >= 1024). This avoids the radial
   search's jump across unreachable gaps before an alternate naming fits.
   It is a finite candidate projection, not a global closest-point solver
   or a temporal smoother; candidate ties away from reach can still switch.
   Projected fitted[] is the exact XYZ rotation of the chosen integer out[].
   retained = max(0, 1 - angular_error / input_angle), or 1 for an unchanged
   request. It is diagnostic, NOT a scale for reconstructing the input.
   Validation and optional outputs follow dg_ik_ps_fit. */
int dg_ik_ps_project(const double q[4], int reach, double fitted[4],
                    short out[3], unsigned *flags, double *retained);

#endif
