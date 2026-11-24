#ifndef DG_OPTICS_GESTURE_H
#define DG_OPTICS_GESTURE_H
#include "dg_psg_gesture.h"
#include <stdint.h>
#include <string.h>

/* Metres in one OpenXR space. The renderer supplies the actual eye positions
   and the ocular point and orientation of the drawn item. Local -Z looks out
   through the instrument. Owner must change on equip/actor/session changes. */
typedef struct {
    uint64_t owner;
    DG_PSG_GESTURE gesture;
    int neutral_grips;
} DG_OPTICS_GESTURE;

static void dg_optics_reset(DG_OPTICS_GESTURE *state) {
    memset(state, 0, sizeof *state);
}

static int dg_optics_eye_step(DG_OPTICS_GESTURE *state, uint64_t owner,
        const DG_PSG_GESTURE_CFG *cfg, int valid, const double eyes[2][3],
        const double head_q[4], const double ocular[3], const double ocular_q[4]) {
    double distance[2] = {0,0}, forward[3], optical_axis[3];
    double depth = 0, axial = 0, lateral_squared = 0;
    int eye, axis, nearest, result;
    if (!state) return 0;
    if (!valid || !owner || !cfg || !eyes || !head_q || !ocular || !ocular_q ||
        !dg_psg_unit_quat(head_q) || !dg_psg_unit_quat(ocular_q)) {
        dg_optics_reset(state);
        return 0;
    }
    if (owner != state->owner) {
        dg_optics_reset(state);
        state->owner = owner;
    }
    for (eye = 0; eye < 2; ++eye) {
        for (axis = 0; axis < 3; ++axis) {
            double delta = ocular[axis] - eyes[eye][axis];
            if (!(delta >= -100 && delta <= 100)) {
                dg_optics_reset(state);
                return 0;
            }
            distance[eye] += delta * delta;
        }
    }
    nearest = distance[1] < distance[0];
    dg_psg_forward(head_q, forward);
    dg_psg_forward(ocular_q, optical_axis);
    for (axis = 0; axis < 3; ++axis) {
        depth += (ocular[axis] - eyes[nearest][axis]) * forward[axis];
        axial += (ocular[axis] - eyes[nearest][axis]) * optical_axis[axis];
    }
    for (axis = 0; axis < 3; ++axis) {
        double lateral = ocular[axis] - eyes[nearest][axis] - axial * optical_axis[axis];
        lateral_squared += lateral * lateral;
    }
    /* Initial eye-box radii in metres. Reject an instrument beside the eye
       even when its optical axis happens to point parallel to the gaze. */
    double radius = state->gesture.open ? .055 : .035;
    if (depth < 0 || axial < 0 || lateral_squared > radius * radius) {
        dg_optics_reset(state);
        return 0;
    }
    result = dg_psg_gesture_step(&state->gesture,cfg,1,eyes[nearest],head_q,ocular,ocular_q);
    if (result != DG_PSG_GESTURE_SCOPE) state->neutral_grips = 0;
    return result == DG_PSG_GESTURE_SCOPE;
}

/* Start zoom only after neutral while actually looking through the optic. */
static int dg_optics_grip_zoom(DG_OPTICS_GESTURE *state, int left, int right) {
    if (!state || !state->gesture.open) return 0;
    if (!left && !right) state->neutral_grips = 1;
    return state->neutral_grips ? (!!right - !!left) : 0;
}
#endif
