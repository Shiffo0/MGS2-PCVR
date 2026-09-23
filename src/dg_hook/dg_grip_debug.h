#ifndef DG_GRIP_DEBUG_H
#define DG_GRIP_DEBUG_H
#include "dg_m9_runtime.h"
#include "dg_health.h" /* reuse bounded, tested upload lifecycle */
#define DG_GRIPDBG_SIZE 256u
#define DG_GRIPDBG_PIXELS (256u*256u)
/* Inverse of dg_m9_relative, including the native/XR basis permutation.
 * These are the actual hit-test coordinates, not an anatomical estimate. */



















/* Quad X follows the measured segment; its normal faces the viewer. */


















/* Atlas: ring TL, controller dot TR, distance-line BL, numeric label BR.
 * Straight RGBA. Green=attached, yellow=in radius, red=outside, gray=blocked. */


























#endif
