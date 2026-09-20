#ifndef DG_MODEL_ARM_H
#define DG_MODEL_ARM_H
#include <stdint.h>
typedef struct {
    int valid;
    uint64_t ms, sequence;
    unsigned long stream, pair;
    /* Actual solved elbow/wrist and forearm surface normal in XR LOCAL,
       converted with the same camera/raw-eye mapping as that game's pose. */
    double elbow[3],wrist[3],normal[3];
} DG_MODEL_ARM;
void dg_bridge_model_arm_clear(void);
void dg_bridge_model_arm_publish(const DG_MODEL_ARM *pose);
int dg_bridge_model_arm_snapshot(DG_MODEL_ARM *pose);
#endif
