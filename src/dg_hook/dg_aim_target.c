#include "dg_aim_target.h"
#include "dg_ik.h"
#include <math.h>
#ifdef _MSC_VER
#include <float.h>
#define AIM_FINITE(x) _finite(x)
#else
#define AIM_FINITE(x) isfinite(x)
#endif

static int aim_unit(double q[4])
{
    double length;
    int i;
    for (i = 0; i < 4; i++) if (!AIM_FINITE(q[i])) return 0;
    length = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (!AIM_FINITE(length) || fabs(length - 1.0) > 1e-6) return 0;
    return dg_ik_quat_normalize(q);
}

int dg_aim_target_build(const double camera_world[3][3],
                        const double view_raw[4], const double aim_raw[4],
                        double hand_world[4])
{
    const double b[4] = {1.0, 0.0, 0.0, 0.0};
    /* C maps hand -Y to XR aim -Z and hand +Z to XR aim +Y. */
    const double c[4] = {0.0, 0.70710678118654752440, 0.70710678118654752440, 0.0};
    double camera[3][3], view[4], aim[4], cq[4], inverse[4], relative[4], result[4];
    int i, j, k;
    if (!hand_world) return 0;
    if (!camera_world || !view_raw || !aim_raw) {
        hand_world[0] = hand_world[1] = hand_world[2] = 0.0;
        hand_world[3] = 1.0;
        return 0;
    }
    /* Copy before initializing the output so aliasing cannot alter inputs. */
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) camera[i][j] = camera_world[i][j];
    for (i = 0; i < 4; i++) { view[i] = view_raw[i]; aim[i] = aim_raw[i]; }
    hand_world[0] = hand_world[1] = hand_world[2] = 0.0;
    hand_world[3] = 1.0;
    if (!aim_unit(view) || !aim_unit(aim)) return 0;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
        double dot = 0.0;
        if (!AIM_FINITE(camera[i][j])) return 0;
        for (k = 0; k < 3; k++) dot += camera[i][k] * camera[j][k];
        if (!AIM_FINITE(dot) || fabs(dot - (i == j ? 1.0 : 0.0)) > 1e-4) return 0;
    }
    /* The helper also rejects a reflected basis; it only orthonormalizes
       floating-point noise after the explicit rigid-basis check above. */
    if (!dg_ik_basis_quat(camera, cq)) return 0;
    dg_ik_quat_conj(view, inverse);
    dg_ik_quat_mul(inverse, aim, relative);
    dg_ik_quat_mul(cq, b, result);
    dg_ik_quat_mul(result, relative, result);
    dg_ik_quat_mul(result, c, result);
    if (!dg_ik_quat_normalize(result)) return 0;
    for (i = 0; i < 4; i++) hand_world[i] = result[i];
    return 1;
}
