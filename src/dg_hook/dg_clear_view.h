#ifndef DG_CLEAR_VIEW_H
#define DG_CLEAR_VIEW_H

#include <math.h>
enum { DG_CV_IGNORE = 0, DG_CV_FORGET, DG_CV_NOTE };
typedef struct { long left, top, right, bottom; } DG_CV_RECT;   /* D3D11_RECT layout */
static int dg_clear_view_decide(int multisampled, unsigned width, unsigned height,
                                const DG_CV_RECT *rects, unsigned num_rects, const float colour[4]) {
    unsigned k;
    if (!multisampled || !colour) return DG_CV_IGNORE;
    for (k = 0; k < 4; k++) if (!isfinite(colour[k])) return DG_CV_FORGET;
    if (num_rects == 0) return DG_CV_NOTE;
    if (num_rects == 1 && rects && width && height &&
        rects[0].left == 0 && rects[0].top == 0 &&
        rects[0].right == (long)width && rects[0].bottom == (long)height) return DG_CV_NOTE;
    return DG_CV_FORGET;
}
#endif
