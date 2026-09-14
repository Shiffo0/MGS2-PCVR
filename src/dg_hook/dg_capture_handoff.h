#ifndef DG_CAPTURE_HANDOFF_H
#define DG_CAPTURE_HANDOFF_H
#include "dg_xr.h"
typedef struct {
    int valid,eye;
    DG_XR_RAW_POSE raw;
    DG_PROJ_FOV fov;
    DG_NEAR_META near_meta;
} DG_HOOK_HANDOFF;
#endif
