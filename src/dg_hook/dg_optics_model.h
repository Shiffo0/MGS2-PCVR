#ifndef DG_OPTICS_MODEL_H
#define DG_OPTICS_MODEL_H

#include "dg_optics_gesture.h"

/* The renderer supplies the drawn origin and each model axis in XR LOCAL.
   Each axis vector is the displacement in metres for one native model unit.
   optical_q represents the drawn item with optical forward along XR -Z. */
typedef struct {
    double origin[3];
    double model_axes[3][3];
    double optical_q[4];
} DG_OPTICS_MODEL_POSE;

typedef struct {
    DG_OPTICS_GESTURE gesture;
    int item, ocular, eye;
} DG_OPTICS_MODEL_GESTURE;

static int dg_optics_grip_item(int item)
{
    return item==14 || item==15 || item==21;
}

static int dg_optics_model_hidden(int leased, int open, int native_hidden)
{
    return leased ? !!open : !!native_hidden;
}

static int dg_optics_model_oculars(int item, const DG_OPTICS_MODEL_POSE *model,
                                  double points[2][3]) {
    static const double scope[2][3] = {{45,-93,79}, {100,-93,79}};
    /* Rear surface on the camera lens axis. Verify against the drawn model. */
    static const double camera[1][3] = {{108,-99,36}};
    const double (*offsets)[3];
    int count;
    switch (item) {
        case 14: offsets = scope; count = 2; break;
        case 15:
        case 21: offsets = camera; count = 1; break;
        default: return 0;
    }
    for (int local = 0; local < 3; ++local) {
        const double *v = model->model_axes[local];
        double length_squared = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
        if (!(length_squared >= 1e-12 && length_squared <= .01)) return 0;
    }
    for (int ocular = 0; ocular < count; ++ocular) {
        for (int axis = 0; axis < 3; ++axis) {
            points[ocular][axis] = model->origin[axis];
            for (int local = 0; local < 3; ++local)
                points[ocular][axis] += offsets[ocular][local] * model->model_axes[local][axis];
        }
    }
    return count;
}

static int dg_optics_model_pair_step(DG_OPTICS_GESTURE *gesture, uint64_t owner,
        const DG_PSG_GESTURE_CFG *cfg, const double eye[3], const double head_q[4],
        const double ocular[3], const double optical_q[4]) {
    /* Evaluate a specific eye. The model step ranks all successful pairs. */
    double pair[2][3];
    memcpy(pair[0], eye, sizeof pair[0]);
    memcpy(pair[1], eye, sizeof pair[1]);
    return dg_optics_eye_step(gesture,owner,cfg,1,pair,head_q,ocular,optical_q);
}

static int dg_optics_model_step(DG_OPTICS_MODEL_GESTURE *state, uint64_t owner, int item,
        int valid, const DG_PSG_GESTURE_CFG *cfg, const double eyes[2][3],
        const double head_q[4], const DG_OPTICS_MODEL_POSE *model) {
    double points[2][3], best_distance = 0;
    int count = valid && owner && cfg && eyes && head_q && model
        ? dg_optics_model_oculars(item,model,points) : 0;
    DG_OPTICS_MODEL_GESTURE best = {0};
    if (!state) return 0;
    if (count && state->item == item && state->gesture.owner == owner &&
        state->gesture.gesture.open &&
        dg_optics_model_pair_step(&state->gesture,owner,cfg,eyes[state->eye],head_q,
                                  points[state->ocular],model->optical_q))
        return 1;

    for (int ocular = 0; ocular < count; ++ocular) {
        for (int eye = 0; eye < 2; ++eye) {
            DG_OPTICS_GESTURE candidate = {0};
            if (dg_optics_model_pair_step(&candidate,owner,cfg,eyes[eye],head_q,
                                          points[ocular],model->optical_q) &&
                (!best.item || candidate.gesture.last_m < best_distance)) {
                best = (DG_OPTICS_MODEL_GESTURE){candidate,item,ocular,eye};
                best_distance = candidate.gesture.last_m;
            }
        }
    }
    *state = best;
    return best.item != 0;
}

#endif
