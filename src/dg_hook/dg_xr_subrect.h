#pragma once
/* Union-FOV render, per-eye presentation (10 Oct 2026).
   stereo_test=2 renders both eyes with the union FOV: the game's culling then covers the replayed eye and
   walls stop vanishing at its edge (simulator: 33 -> 0-2 drop-probe transitions). Submitted as such, the
   Quest showed a doubled image (user, 10 Oct; also seen in July, MGS2_PCVR_V3_PLAN.md S4d) - the runtime
   path does not present a union-FOV layer the way it presents its own per-eye FOV.
   An eye's own frustum, from the same eye position, is an axis-aligned rectangle of the union image plane
   (both are central projections onto the same plane). So each eye is submitted with its own FOV and that
   rectangle as the sub-image: exactly what the runtime shows correctly today, cut from the wider render.
   Angles in radians, OpenXR convention (left/down negative). The rectangle is rounded to whole pixels and
   the returned FOV is that of the rounded rectangle, so pixels map exactly. Returns 0 (caller keeps the
   stored FOV and the full image) unless the eye FOV lies inside the stored one and differs from it. */
#include <math.h>
typedef struct { double left, right, up, down; } DgFov;
typedef struct { int x, y, w, h; } DgRect;
static int dgEyeSubRect(const DgFov *stored, const DgFov *eye, int width, int height, DgRect *rect, DgFov *out) {
    const double eps = 1e-6;
    double sl = tan(stored->left), sr = tan(stored->right), su = tan(stored->up), sd = tan(stored->down);
    double el = tan(eye->left), er = tan(eye->right), eu = tan(eye->up), ed = tan(eye->down);
    if (!(width > 0 && height > 0 && sr > sl && su > sd && er > el && eu > ed)) return 0;
    if (el < sl - eps || er > sr + eps || eu > su + eps || ed < sd - eps) return 0;           /* not inside */
    if (fabs(el - sl) < eps && fabs(er - sr) < eps && fabs(eu - su) < eps && fabs(ed - sd) < eps) return 0;
    {
        int x0 = (int)floor((el - sl) / (sr - sl) * width + 0.5), x1 = (int)floor((er - sl) / (sr - sl) * width + 0.5);
        int y0 = (int)floor((su - eu) / (su - sd) * height + 0.5), y1 = (int)floor((su - ed) / (su - sd) * height + 0.5);
        if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0; if (x1 > width) x1 = width; if (y1 > height) y1 = height;
        if (x1 - x0 < 16 || y1 - y0 < 16) return 0;
        rect->x = x0; rect->y = y0; rect->w = x1 - x0; rect->h = y1 - y0;
        out->left = atan(sl + (sr - sl) * x0 / width);  out->right = atan(sl + (sr - sl) * x1 / width);
        out->up = atan(su - (su - sd) * y0 / height);   out->down = atan(su - (su - sd) * y1 / height);
    }
    return 1;
}
