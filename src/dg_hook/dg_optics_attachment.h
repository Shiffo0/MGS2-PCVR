#ifndef DG_OPTICS_ATTACHMENT_H
#define DG_OPTICS_ATTACHMENT_H
#include "dg_optics_model.h"
#include "dg_position_target.h"

/* The calibrated pivot is the point on the item that meets the grip.
   model_to_grip_q rotates item-local axes into grip-local axes (xyzw).
   All XR positions here share the same LOCAL origin and predicted time. */
static int dg_optics_attachment_pose(const double grip_m[3], const double grip_q[4],
        const double pivot_units[3], const double model_to_grip_q[4],
        double metres_per_unit, DG_OPTICS_MODEL_POSE *result) {
    DG_OPTICS_MODEL_POSE pose = {0};
    if (!result || !grip_m || !pivot_units || !grip_q || !model_to_grip_q ||
        !dg_psg_unit_quat(grip_q) || !dg_psg_unit_quat(model_to_grip_q) ||
        !(metres_per_unit >= .000001 && metres_per_unit <= .1)) return 0;
    dg_ik_quat_mul(grip_q,model_to_grip_q,pose.optical_q);
    if (!dg_ik_quat_normalize(pose.optical_q)) return 0;
    for (int i=0;i<3;++i) {
        if (!isfinite(grip_m[i]) || !isfinite(pivot_units[i])) return 0;
        pose.origin[i]=grip_m[i];
    }
    for (int i=0;i<3;++i) {
        double basis_m[3]={0,0,0};
        basis_m[i]=metres_per_unit;
        dg_position_rotate(pose.optical_q,basis_m,pose.model_axes[i]);
        for (int j=0;j<3;++j) pose.origin[j]-=pivot_units[i]*pose.model_axes[i][j];
    }
    for (int i=0;i<3;++i) if (!isfinite(pose.origin[i])) return 0;
    *result=pose;
    return 1;
}

/* Measured mesh support points. These sit below and behind the front lens,
   inside the body the left hand supports. Aim -Z supplies optical forward
   and the standard interaction profile keeps its roll consistent. */
static int dg_optics_palm_pose(int item,const double palm[3],const double aim[4],
        double metres_per_unit,DG_OPTICS_MODEL_POSE *out)
{
    static const double scope[3]={72.5,-75,12},camera[3]={52,-74,33};
    static const double identity[4]={0,0,0,1};
    return dg_optics_grip_item(item) &&
        dg_optics_attachment_pose(palm,aim,item==14?scope:camera,identity,metres_per_unit,out);
}
#endif
