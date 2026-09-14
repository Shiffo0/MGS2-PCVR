#ifndef DG_AIM_REPLAY_H
#define DG_AIM_REPLAY_H
#include "dg_pose.h"

/* DGREC6 absolute-direction input, before quaternion construction or filtering.
   Matrices are camera axes in world. View and AIM are exact LOCAL xyzw/xyz.
   Selection IDs describe the bounded live guard's result, never replay pointers. */
typedef struct {
    double camera[3][3], view[7], aim[7], dt;
    unsigned long long sample_seq;
    long long sample_time;
    unsigned long long camera_id, arm, subobject, subobjs, hand, model;
    unsigned int frame, eye, stream, age_ms;
    unsigned int pose_flags, hand_tag, kind_tag, source_valid;
    unsigned int reset, main_pose_flags;
} DG_AIM_REPLAY_IN;

typedef struct {
    DG_POSE_STATE pose;
    unsigned long long sample_seq;
    long long sample_time;
} DG_AIM_REPLAY_STATE;

typedef struct {
    DG_POSE_OUT pose;
    unsigned long long sample_seq;
    long long sample_time;
    unsigned int write, input_valid;
} DG_AIM_REPLAY_OUT;

typedef struct {
    DG_AIM_REPLAY_IN input;
    DG_AIM_REPLAY_STATE before; /* seed only at replay start; thereafter checked */
    DG_AIM_REPLAY_OUT observed; /* comparison oracle, never fed into the solve */
    unsigned int present, pad;
} DG_REC_AIM_REPLAY;

typedef char dg_aim_input_layout[(sizeof(DG_AIM_REPLAY_IN)==296)?1:-1];
typedef char dg_aim_state_layout[(sizeof(DG_AIM_REPLAY_STATE)==232)?1:-1];
typedef char dg_aim_output_layout[(sizeof(DG_AIM_REPLAY_OUT)==104)?1:-1];
typedef char dg_aim_record_layout[(sizeof(DG_REC_AIM_REPLAY)==640)?1:-1];

#endif
