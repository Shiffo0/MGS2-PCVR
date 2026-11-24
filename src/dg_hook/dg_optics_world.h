#ifndef DG_OPTICS_WORLD_H
#define DG_OPTICS_WORLD_H

#include <float.h>
#include "dg_optics_attachment.h"

/* Map the same XR LOCAL pose used for the ocular test into the game's
   row-vector model matrix. Uses the absolute left-wrist mapping's camera,
   basis and scale, including the unscaled eye translation. */
static int dg_optics_world_matrix(const DG_OPTICS_MODEL_POSE *pose,
                                 const DG_POSITION_INPUT *camera, float output[4][4])
{
    float world[4][4] = {{0}};
    double basis[4];
    if (!pose || !camera || !output || !camera->enabled || !dg_position_frame(camera,basis))
        return 0;
    for (int row = 0; row < 4; ++row) {
        const double *source = row == 3 ? pose->origin : pose->model_axes[row];
        double local[3], mapped[3];
        for (int i = 0; i < 3; ++i)
            local[i] = (row == 3 ? source[i] - camera->view[4+i] : source[i]) * camera->units;
        dg_position_rotate(basis,local,mapped);
        for (int i = 0; i < 3; ++i) {
            if (row == 3) mapped[i] += camera->camera[12+i];
            if (!(mapped[i] >= -FLT_MAX && mapped[i] <= FLT_MAX)) return 0;
            world[row][i] = (float)mapped[i];
        }
    }
    world[3][3] = 1;
    memcpy(output,world,sizeof world);
    return 1;
}

#endif
