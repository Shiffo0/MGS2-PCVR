#include <math.h>
#include "dg_recoil.h"

void dg_recoil_reset(DG_RECOIL *r)
{
    if (!r) return;
    r->x = 0.0;
    r->v = 0.0;
}

double dg_recoil_amplitude(const DG_RECOIL *r)
{
    return r ? r->x : 0.0;
}

void dg_recoil_fire(DG_RECOIL *r, double shots)
{
    if (!r) return;
    /* Non-finite or negative asks are refused rather than clamped. A negative
       kick would drive the muzzle below the aim line, which is the one thing
       this file promises never to do. */
    if (!(shots > 0.0)) return;          /* also catches NaN */
    if (shots > DG_RECOIL_MAX) shots = DG_RECOIL_MAX;
    r->v += shots * DG_RECOIL_OMEGA * DG_RECOIL_E;
}

void dg_recoil_step(DG_RECOIL *r)
{
    double a, decay, nx, nv;

    if (!r) return;

    /* The exact solution, advanced one tick - not an integrator. A critically
       damped spring is x(t) = (x0 + a*t)*exp(-w*t) with a = v0 + w*x0, so a
       tick is a closed-form linear map and the shape does not depend on how
       often it is stepped. That matters more here than the two multiplies it
       costs: an Euler step at dt = 1 tick with w = 1/3 is nowhere near the
       curve it is meant to be following - it swallows most of the impulse in
       the first step and peaks at a third of the asked-for height. The header
       promises a peak of one at three ticks, and only the closed form keeps
       that promise. */
    a = r->v + DG_RECOIL_OMEGA * r->x;
    decay = exp(-DG_RECOIL_OMEGA);
    nx = (r->x + a) * decay;
    nv = (a * (1.0 - DG_RECOIL_OMEGA) - DG_RECOIL_OMEGA * r->x) * decay;
    r->x = nx;
    r->v = nv;

    /* The muzzle never goes below the aim line. Critical damping should make
       this unreachable, and the tests say it is; the clamp is here so that a
       future change to omega cannot turn a tuning decision into a wrong-way
       flick that nobody notices. */
    if (r->x < 0.0) {
        r->x = 0.0;
        if (r->v < 0.0) r->v = 0.0;
    }
    if (r->x > DG_RECOIL_MAX) {
        r->x = DG_RECOIL_MAX;
        if (r->v > 0.0) r->v = 0.0;
    }

    /* Done is done. An amplitude that merely tends to zero is a standing aim
       error with a long name. */
    if (r->x < DG_RECOIL_EPS && r->v > -DG_RECOIL_EPS &&
        r->v < DG_RECOIL_EPS) {
        r->x = 0.0;
        r->v = 0.0;
    }

    if (!(r->x == r->x) || !(r->v == r->v)) {   /* NaN cannot be aimed with */
        r->x = 0.0;
        r->v = 0.0;
    }
}

int dg_recoil_pull_back(double target[3], const double root[3],
                        double push_mm)
{
    double back[3], d;
    int k;

    if (!target || !root) return 0;
    if (!(push_mm > 0.0)) return 0;              /* also catches NaN */
    for (k = 0; k < 3; k++) {
        back[k] = root[k] - target[k];
        if (!(back[k] == back[k])) return 0;
    }
    d = sqrt(back[0] * back[0] + back[1] * back[1] + back[2] * back[2]);
    /* One millimetre of margin so the target can never land exactly on the
       root, where the arm solver has no direction left to work with. */
    if (!(d > push_mm + 1.0)) return 0;
    for (k = 0; k < 3; k++) target[k] += back[k] * (push_mm / d);
    return 1;
}

int dg_recoil_climb_quat(const double fore_axis[3], const double up[3],
                         double climb_rad, double q_out[4])
{
    double axis[3], n, half, s;
    int k;

    if (!fore_axis || !up || !q_out) return 0;
    for (k = 0; k < 3; k++)
        if (!(fore_axis[k] == fore_axis[k]) || !(up[k] == up[k])) return 0;
    if (!(climb_rad == climb_rad)) return 0;

    /* fore x up. Its length is |fore||up|sin(angle between them), so testing
       it against the inputs' own lengths is the same as testing how far the
       forearm is from vertical - without needing either to be normalised. */
    axis[0] = fore_axis[1] * up[2] - fore_axis[2] * up[1];
    axis[1] = fore_axis[2] * up[0] - fore_axis[0] * up[2];
    axis[2] = fore_axis[0] * up[1] - fore_axis[1] * up[0];
    n = sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
    {
        double fn = sqrt(fore_axis[0] * fore_axis[0] +
                         fore_axis[1] * fore_axis[1] +
                         fore_axis[2] * fore_axis[2]);
        double un = sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
        if (!(fn > 1e-9) || !(un > 1e-9)) return 0;
        if (!(n > DG_RECOIL_MIN_SIN * fn * un)) return 0;
    }
    for (k = 0; k < 3; k++) axis[k] /= n;

    half = climb_rad * 0.5;
    s = sin(half);
    q_out[0] = cos(half);
    q_out[1] = axis[0] * s;
    q_out[2] = axis[1] * s;
    q_out[3] = axis[2] * s;
    return 1;
}
