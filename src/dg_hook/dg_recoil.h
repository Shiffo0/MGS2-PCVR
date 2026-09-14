/* dg_recoil - the kick the game cannot give us any more.
 *
 * Deliberately standalone: no windows.h, no OpenXR, no game. Same rule as
 * dg_fire - the reasoning lives here so it can be tested at a desk, and the
 * bridge does the writing.
 *
 * Why this file has to exist at all. MGS2 animates recoil onto the arm joints
 * like any other motion, and we overwrite those joints every camera frame from
 * the controller. Measured, not assumed: in the run of 2026-08-19, twenty-four
 * consecutive frames at 60 fps spanning several shots show the muzzle flash
 * appear and vanish with the forearm and hand completely motionless. We win
 * that fight every tick, so the shot has no physical consequence at all.
 *
 * So the kick is ours to make. That is a real distinction and worth stating
 * plainly: everything else this project writes is a translation of something
 * the player or the game already did. This is motion we invent. It therefore
 * gets the strictest form of the usual rules - it must return to exactly zero,
 * it must never point the muzzle below where the player aimed, and it must be
 * impossible for it to leave a standing bias in the aim.
 *
 * The model is a critically damped spring driven by an impulse, which is what
 * a wrist actually is. A shot adds velocity; the spring pulls the hand back to
 * where the player is pointing. Critically damped means the amplitude rises,
 * peaks and returns without ever crossing zero - so the muzzle climbs and
 * settles, and never dips under the aim line on the way back. Rapid fire adds
 * impulses to a system that is still moving, which is exactly how a real one
 * behaves, and the accumulation is capped so a magazine dump cannot fold the
 * wrist.
 */
#ifndef DG_RECOIL_H
#define DG_RECOIL_H

/* Angular frequency in radians per game tick. 1/3 puts the peak of a single
   shot about three ticks (50 ms) after it, and a critically damped impulse is
   down to a thousandth of it a little past thirty ticks - half a second from
   shot to still. Both numbers are asserted by the desk tests rather than left
   as intentions. */
#define DG_RECOIL_OMEGA         0.33333333333333333

/* One shot is one unit of amplitude at the peak. The caller multiplies that by
   the degrees and millimetres it wants, so this file never knows about either
   and the tuning lives in the marker. The factor is omega*e, which is what
   turns an impulse of one into a peak of one for the continuous solution
   x(t) = v0*t*exp(-omega*t). dg_recoil_step advances that solution in closed
   form rather than integrating it, so the peak is exactly one - which is what
   the tests check, to the third decimal. */
#define DG_RECOIL_E             2.7182818284590452

/* Three shots' worth of climb, and no more, however fast the trigger is
   pulled. Past this the wrist would be doing something a wrist does not do,
   and the hand envelope in dg_ik would start clamping - which would be a
   silent limit rather than a stated one. */
#define DG_RECOIL_MAX           3.0

/* Below this the spring is done. Snapped to exactly zero rather than left to
   decay asymptotically, because a residual amplitude is a permanent aim
   error, and "very small" is not the same as "none". A thousandth of a shot
   is a thousandth of a degree at any sane setting - well under the resolution
   of the PS2 angle channel this eventually travels down. */
#define DG_RECOIL_EPS           1.0e-3

/* What the tests hold the envelope to: a single shot is back to exactly zero
   within this many ticks. Not a mechanism - a promise. */
#define DG_RECOIL_SETTLE_TICKS  40

/* A forearm this close to vertical has no horizontal axis to climb about, so
   there is no answer rather than a badly conditioned one. About 3 degrees. */
#define DG_RECOIL_MIN_SIN       0.05

typedef struct {
    double x;   /* amplitude now: 0 at rest, 1 at the peak of one shot */
    double v;   /* its rate, in amplitude per tick */
} DG_RECOIL;

void   dg_recoil_reset(DG_RECOIL *r);

/* One shot. `shots` is normally 1.0; it exists so a weapon can be given a
   heavier or lighter kick without touching this file. */
void   dg_recoil_fire(DG_RECOIL *r, double shots);

/* One game tick. Must be called on the tick seam and nowhere else: the camera
   seam runs several times per tick, and an envelope stepped five times a tick
   is an envelope five times too fast. */
void   dg_recoil_step(DG_RECOIL *r);

double dg_recoil_amplitude(const DG_RECOIL *r);

/* The geometry, kept here so the sign is derived rather than guessed.
 *
 * The pistol rides on the forearm, so a climb is a rotation about a horizontal
 * axis perpendicular to it: axis = normalize(fore x up). Rotating by a
 * positive angle about that axis moves a point on the forearm by
 * angle * (axis x fore), and (fore x up) x fore is the component of `up`
 * perpendicular to the forearm - which points up. So a positive climb_rad
 * always lifts the muzzle, whichever way the arm happens to be held.
 *
 * Returns 0 - and writes nothing - when the forearm is too close to vertical
 * for that cross product to mean anything.
 */
int dg_recoil_climb_quat(const double fore_axis[3], const double up[3],
                         double climb_rad, double q_out[4]);

/* The other half of the kick: the hand driven back toward the shoulder.
 *
 * Always a shortening of the reach and never a lengthening. That is the
 * physical direction, and it has a second consequence worth having on
 * purpose: a kick can only ever make the target easier to reach, so recoil
 * can never be the thing that puts a pair out of range and costs a frame of
 * hand control.
 *
 * Returns 0 - and writes nothing - when the target is already so close to the
 * root that the push would carry it past.
 */
int dg_recoil_pull_back(double target[3], const double root[3],
                        double push_mm);

#endif /* DG_RECOIL_H */
