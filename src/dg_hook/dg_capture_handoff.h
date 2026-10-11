#ifndef DG_CAPTURE_HANDOFF_H
#define DG_CAPTURE_HANDOFF_H
#include "dg_xr.h"
#include "dg_magwell.h"
typedef struct {
    int valid,eye;
    DG_MAGWELL magwell;
    DG_XR_RAW_POSE raw;
    DG_PROJ_FOV fov;
    DG_NEAR_META near_meta;
    /* Same camera invocation; copied with the stage/consumer stamp. */
    int pair_valid;
    MAT pair_ep[2];
    /* The other eye of the same head sample (vr_second_eye): its layer pose
       (LOCAL, as reported) and the FOV it is submitted with. */
    int other_valid;
    DG_XR_RAW_POSE other_raw;
    DG_PROJ_FOV other_fov;
} DG_HOOK_HANDOFF;
#endif
