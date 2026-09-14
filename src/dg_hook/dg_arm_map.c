/* dg_arm_map.c - pure position calibration and anatomical reach policy. */

#include <math.h>
#include <string.h>
#include "dg_arm_map.h"

#define DG_ARM_MAP_EPS 1.0e-9
/* World translations around 127 m are stored as float in the game, so a live
   200-300 mm bone can quantise by hundredths of a millimetre between poses.
   One permille tolerates that while a model swap is independently identified
   by arm/objs/topology and a real length change remains fail-closed. */
#define DG_ARM_MAP_BONE_EPS 1.0e-3
#define DG_ARM_MAP_MAX_REACH_FRAC 0.99
/* Visual safety envelope, not a claim about human anatomy. The input still
   uses the established .99 calibration scale; only the final shoulder reach
   is compressed. A cubic Hermite segment has slope 1 at SOFT_START and 0 at
   MAX_REACH, so entering or saturating the zone cannot create a velocity
   discontinuity. */
#define DG_ARM_MAP_SOFT_START_FRAC 0.86
#define DG_ARM_MAP_SOFT_END_FRAC 0.95
#define DG_ARM_MAP_FINITE(v) (isfinite(v) != 0)

static int finite3(const double v[3])
{
    return v && DG_ARM_MAP_FINITE(v[0]) && DG_ARM_MAP_FINITE(v[1]) &&
           DG_ARM_MAP_FINITE(v[2]);
}

static void sub3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

static double dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double len3(const double v[3])
{
    return sqrt(dot3(v, v));
}

static int close_bone(double a, double b)
{
    double scale = fabs(a) > fabs(b) ? fabs(a) : fabs(b);
    if (scale < 1.0) scale = 1.0;
    return fabs(a - b) <= DG_ARM_MAP_BONE_EPS * scale;
}

static double soften_outer_reach(double reach, double span, double max_reach)
{
    double start = span * DG_ARM_MAP_SOFT_START_FRAC;
    double end = span * DG_ARM_MAP_SOFT_END_FRAC;
    double width = max_reach - start;
    double t, t2, t3;

    if (reach <= start) return reach;
    if (reach >= max_reach) return end;
    t = (reach - start) / width;
    t2 = t * t;
    t3 = t2 * t;
    /* Cubic Hermite: start value/slope = start/1, end value/slope = end/0. */
    return (2.0 * t3 - 3.0 * t2 + 1.0) * start +
           (t3 - 2.0 * t2 + t) * width +
           (-2.0 * t3 + 3.0 * t2) * end;
}

static int valid_common(const DG_ARM_MAP_IN *in)
{
    double span;

    if (!in || !finite3(in->controller_view) ||
        !finite3(in->shoulder_view) || !finite3(in->native_wrist_view) ||
        !DG_ARM_MAP_FINITE(in->upper) || !DG_ARM_MAP_FINITE(in->fore) ||
        in->upper <= 0.0 || in->fore <= 0.0) return 0;
    if(in->explicit_target && !finite3(in->desired_view)) return 0;
    /* Only checked when it is going to be read. An offset-anchor caller has no
       reason to fill it in, and demanding it would make this module reject
       the configuration it was originally written for. */
    if (in->anchor_shoulder &&
        (!finite3(in->player_shoulder) ||
         !DG_ARM_MAP_FINITE(in->player_reach) ||
         in->player_reach <= DG_ARM_MAP_EPS)) return 0;

    span = in->upper + in->fore;
    return DG_ARM_MAP_FINITE(span) && span > DG_ARM_MAP_EPS;
}

void dg_arm_map_reset(DG_ARM_MAP_STATE *state)
{
    if (state) memset(state, 0, sizeof(*state));
}

int dg_arm_map_step(DG_ARM_MAP_STATE *state, const DG_ARM_MAP_IN *in,
                    DG_ARM_MAP_OUT *out)
{
    double source_vec[3];
    double wrist_vec[3];
    double candidate[3];
    double ray[3];
    double source_span;
    double wrist_reach;
    double reach;
    double max_reach;
    double min_reach;
    double scale;
    double wanted;
    unsigned flags = 0;
    int i;

    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    if (!state || !valid_common(in)) {
        out->flags = DG_ARM_MAP_F_BAD_INPUT;
        return 0;
    }

    /* Match dg_ik's deliberate 0.99 reach ceiling. Mapping to the mathematical
       full span would make every mapped outer clamp become an IK clamp too,
       defeating the pair-level "did calibration avoid solver clamps" metric
       and allowing a visually locked elbow. */
    max_reach = (in->upper + in->fore) * DG_ARM_MAP_MAX_REACH_FRAC;
    min_reach = fabs(in->upper - in->fore);

    if (!state->calibrated) {
        /* The span the scale divides by. Under the shoulder anchor both ends
           of this vector are the PLAYER's, so it measures the player's arm;
           under the offset anchor it keeps its original meaning. */
        /* The span the scale divides by. Supplied under the shoulder anchor
           and measured under the offset one, for the reason in the header. */
        sub3(in->controller_view, in->shoulder_view, source_vec);
        sub3(in->native_wrist_view, in->shoulder_view, wrist_vec);
        source_span = in->anchor_shoulder ? in->player_reach
                                          : len3(source_vec);
        wrist_reach = len3(wrist_vec);

        if (!DG_ARM_MAP_FINITE(source_span) ||
            !DG_ARM_MAP_FINITE(wrist_reach) ||
            source_span <= DG_ARM_MAP_EPS ||
            wrist_reach <= DG_ARM_MAP_EPS ||
            wrist_reach < min_reach - DG_ARM_MAP_BONE_EPS ||
            wrist_reach > (in->upper + in->fore) + DG_ARM_MAP_BONE_EPS) {
            out->flags = DG_ARM_MAP_F_BAD_INPUT;
            return 0;
        }

        for (i = 0; i < 3; ++i) {
            state->controller0_view[i] = in->controller_view[i];
            state->wrist0_view[i] = in->native_wrist_view[i];
            state->shoulder0_view[i] = in->shoulder_view[i];
            out->target_view[i] = in->native_wrist_view[i];
        }
        state->source_span = source_span;
        state->upper = in->upper;
        state->fore = in->fore;
        state->anchor_shoulder = in->anchor_shoulder ? 1 : 0;
        state->calibrated = 1;

        out->scale = max_reach / source_span;
        out->source_span = source_span;
        out->reach_used = wrist_reach;
        out->flags = DG_ARM_MAP_F_CALIBRATED | DG_ARM_MAP_F_NOT_READY;
        return 0;
    }

    /* A character/rig switch needs an explicit reset: retaining one rig's
       wrist anchor while silently adopting another rig's scale is a jump. */
    if (!close_bone(in->upper, state->upper) ||
        !close_bone(in->fore, state->fore) ||
        (in->anchor_shoulder ? 1 : 0) != state->anchor_shoulder ||
        !DG_ARM_MAP_FINITE(state->source_span) ||
        state->source_span <= DG_ARM_MAP_EPS) {
        out->flags = DG_ARM_MAP_F_BAD_INPUT;
        return 0;
    }

    /* Quantisation-tolerated live measurements must not modulate the scale
       every frame. The calibrated character lengths are the stable contract. */
    max_reach = (state->upper + state->fore) * DG_ARM_MAP_MAX_REACH_FRAC;
    min_reach = fabs(state->upper - state->fore);

    scale = max_reach / state->source_span;
    if (!DG_ARM_MAP_FINITE(scale) || scale <= 0.0) {
        out->flags = DG_ARM_MAP_F_BAD_INPUT;
        return 0;
    }

    for (i = 0; i < 3; ++i) {
        /* Reported under both anchors, because "how far have you moved since
           calibration" is what a log reader wants either way. Only the offset
           anchor actually builds the target out of it. */
        out->controller_delta[i] =
            in->controller_view[i] - state->controller0_view[i];
        candidate[i] = in->explicit_target ? in->desired_view[i] : in->anchor_shoulder
            /* The anatomical reference belongs to this calibration's root
               frame. Native aim/reload may move the LIVE shoulder, but must
               not move a reachable controller target with it. The caller
               still carries this root-local target with locomotion/turning;
               the live shoulder below still owns reach and the IK origin. */
            ? state->shoulder0_view[i] +
              (in->controller_view[i] - in->player_shoulder[i]) * scale
            : state->wrist0_view[i] + out->controller_delta[i] * scale;
    }
    if (!finite3(out->controller_delta) || !finite3(candidate)) {
        memset(out, 0, sizeof(*out));
        out->flags = DG_ARM_MAP_F_BAD_INPUT;
        return 0;
    }

    sub3(candidate, in->shoulder_view, ray);
    reach = len3(ray);
    if (!DG_ARM_MAP_FINITE(reach)) {
        memset(out, 0, sizeof(*out));
        out->flags = DG_ARM_MAP_F_BAD_INPUT;
        return 0;
    }

    wanted = reach;
    if (reach > (state->upper + state->fore) *
                DG_ARM_MAP_SOFT_START_FRAC) {
        wanted = soften_outer_reach(reach, state->upper + state->fore,
                                    max_reach);
        flags |= DG_ARM_MAP_F_SOFT_REACH;
    }
    if (reach >= max_reach) {
        flags |= DG_ARM_MAP_F_CLAMP_REACH;
    } else if (reach < min_reach) {
        wanted = min_reach;
        flags |= DG_ARM_MAP_F_CLAMP_TOO_CLOSE;
    }

    /* A non-zero minimum reach needs a direction.  Choosing a fallback axis
       here would be an unreported arm jump, so the exact-shoulder case fails
       closed instead.  Equal-length bones legitimately have min_reach == 0. */
    if (reach <= DG_ARM_MAP_EPS && wanted > DG_ARM_MAP_EPS) {
        memset(out, 0, sizeof(*out));
        out->flags = DG_ARM_MAP_F_BAD_INPUT;
        return 0;
    }

    if (reach > DG_ARM_MAP_EPS && wanted != reach) {
        double k = wanted / reach;
        for (i = 0; i < 3; ++i)
            candidate[i] = in->shoulder_view[i] + ray[i] * k;
        reach = wanted;
    }

    for (i = 0; i < 3; ++i) out->target_view[i] = candidate[i];
    out->scale = scale;
    out->source_span = state->source_span;
    out->reach_used = reach;
    out->flags = flags;
    return 1;
}
