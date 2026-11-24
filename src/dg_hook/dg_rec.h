#include "dg_build_profile.h"

#ifndef DG_REC_H
#define DG_REC_H

#include <stdio.h>
#include "dg_xr.h"
#include "dg_aim_replay.h"

#define DG_REC_CAP DG_DIAGNOSTIC_CAPACITY(16384)
#define DG_REC_MAGIC  "DGREC6"
#define DG_REC_MAGIC_V5 "DGREC5"

#define DG_REC_MAGIC_V4 "DGREC4"
#define DG_REC_MAGIC_V3 "DGREC3"
#define DG_REC_MAGIC_V2 "DGREC2"
#define DG_REC_MAGIC_V1 "DGREC1"

/* What read_open hands back and read_next expects: which on-disk layout the
   file carries. 0 is the current one; the nonzero codes name the past. */
#define DG_REC_COMPAT_NONE 0
#define DG_REC_COMPAT_V1   1
#define DG_REC_COMPAT_V2   2
#define DG_REC_COMPAT_V3   3
#define DG_REC_COMPAT_V4   4
#define DG_REC_COMPAT_V5   5

/* One tracked pose as recorded: the raw LOCAL pose plus everything the
   pipeline's validity gates read. 80 bytes, no padding holes - the dedupe
   and the file both compare raw bytes, so a hole would be nondeterminism. */
typedef struct {
    double   q[4];              /* qx qy qz qw */
    double   p[3];              /* px py pz, metres */
    unsigned long long sample_seq;
    long long xr_time;
    unsigned int flags;         /* bit0 position_valid  bit1 orientation_valid
                                   bit2 active           bit3 tracked */
    unsigned int pose_age_ms;
} DG_REC_POSE;

/* One hand: both poses and the full non-pose input surface. 200 bytes. */
typedef struct {
    DG_REC_POSE grip;
    DG_REC_POSE aim;
    unsigned long long trigger_press_seq;
    unsigned long long trigger_release_seq;
    float trigger_value;
    float squeeze_value;
    float thumbstick_x;
    float thumbstick_y;
    unsigned int buttons;       /* bit0 trigger_click  bit1 squeeze_click
                                   bit2 thumbstick_click  bit3 primary
                                   bit4 secondary  bit5 menu */
    unsigned int pad0;          /* explicit, so the byte image is total */
} DG_REC_HAND;

typedef struct DG_REC_PAIRSTATE {
    double base_now[4];    /* animated hand base, our adjusts stripped */
    double live_hand[4];   /* hand basis as read from the hierarchy */
    double root_q[4];      /* body root frame */
    double root_q0[4];     /* calibration root (the F10 yaw zero) */
    double rest_view[4];   /* captured animation rest (hand_rest_view) */
    double ctrl_rest[4];   /* captured controller rest (hand_ctrl_rest) */
    double demand[4];      /* the adjust command published this pass */
    double desired[4];     /* the desired world orientation behind it */
    unsigned int wtype;    /* weapon word (wp_set->type) as last seen */
    unsigned int flags;    /* DG_REC_PAIR_F_* */

    float rest_drift_deg;

    unsigned int frame_word;

    float joint_world[4][3];  /* clean joints 3..6 (adjusts stripped), world mm */
    float ik_elbow[3];        /* the solver's elbow, world mm */
    float ik_wrist[3];        /* the solver's wrist (target after clamp), world mm */
    float ik_target[3];       /* the composed target the solver chased, world mm */
    float q4_world[4];        /* the published upper-arm adjust, world */
    float q5_world[4];        /* the published forearm adjust (twist share in), world */
    float fore_axis[3];       /* the envelope's forearm axis, unit, world */
    float raw_twist_deg;      /* the envelope's twist demand before the caps */
    float raw_swing_deg;      /* the envelope's swing demand before the caps */
    float fit_frac;           /* 1 = delivered whole; CHANNEL_PROJECT selects
                                1-error/input_angle, otherwise radial fraction */
    float head_yaw_deg;       /* the mapped head yaw the view is drawn from */
    float stick_yaw_deg;      /* the stick's software turn */
    float aim_yaw_deg;        /* the aim pose's yaw */

    float camera_world[3][3]; /* camera-to-world basis, rows as received */
    float camera_proj[3];     /* projection m00, m11, m23 */
} DG_REC_PAIRSTATE;

#define DG_REC_PAIR_F_FRAMES    1u  /* root_q / live_hand read this pass */
#define DG_REC_PAIR_F_REST      2u  /* rest_view / ctrl_rest standing */
#define DG_REC_PAIR_F_HAND_ON   4u  /* the hand drive was enabled */
#define DG_REC_PAIR_F_PUBLISHED 8u  /* demand / desired published this pass */
#define DG_REC_PAIR_F_PHASE2   16u  /* arm map calibrated (phase 2) */
#define DG_REC_PAIR_F_BASE     32u  /* base_now recovered this pass */
#define DG_REC_PAIR_F_FRAME    64u  /* frame_word valid (rot.vy captured) */
#define DG_REC_PAIR_F_FRAME_LIVE 128u
#define DG_REC_PAIR_F_ARM      256u /* joint_world / ik_* / q4_world / q5_world stand */
#define DG_REC_PAIR_F_ENVELOPE 512u /* fore_axis / raw_twist_deg / raw_swing_deg / fit_frac stand */
#define DG_REC_PAIR_F_FACING  1024u /* head_yaw_deg / stick_yaw_deg / aim_yaw_deg stand */
#define DG_REC_PAIR_F_CAMERA  2048u /* camera tail is valid at this seam */
#define DG_REC_PAIR_F_CHANNEL_PROJECT 4096u /* fit_frac is angular retention,
                                             not a radial scale factor */
#define DG_REC_FRAME_WORD_VALID 0x80000000u

typedef struct {
    long long qpc;
    double head[7];             /* qx qy qz qw px py pz, raw LOCAL */
    DG_REC_HAND hand[2];        /* 0 = left, 1 = right */
    DG_REC_PAIRSTATE pair;      /* game-side half, see above */
    unsigned int present_frame;
    unsigned int eye;
    unsigned int stream_id;
    unsigned int pair_id;
    /* Same camera visit's absolute input and observed target. Unlike pair,
       this is not one seam behind. Present also records refused inputs. */
    DG_REC_AIM_REPLAY absolute;
} DG_REC_FRAME;

/* The compiler is told the exact layout this file format assumes. If either
   struct ever grows a hole or a field, old recordings must be refused by a
   size mismatch, not misread - so the build fails here first. */
typedef char dg_rec_pose_size_assert[(sizeof(DG_REC_POSE) == 80) ? 1 : -1];
typedef char dg_rec_hand_size_assert[(sizeof(DG_REC_HAND) == 200) ? 1 : -1];
typedef char dg_rec_pair_size_assert[(sizeof(DG_REC_PAIRSTATE) == 472)
                                     ? 1 : -1];
typedef char dg_rec_frame_size_assert[(sizeof(DG_REC_FRAME) == 952 + sizeof(DG_REC_AIM_REPLAY)) ? 1 : -1];
#define DG_REC_V5_BYTES ((size_t)952)

#define DG_REC_V1_PREFIX ((size_t)(8 + sizeof(double[7]) + \
                                   2 * sizeof(DG_REC_HAND)))
#define DG_REC_V1_BYTES  (DG_REC_V1_PREFIX + 4 * sizeof(unsigned int))

#define DG_REC_V2_PAIR_BYTES ((size_t)264)
#define DG_REC_V2_BYTES  (DG_REC_V1_PREFIX + DG_REC_V2_PAIR_BYTES + \
                          4 * sizeof(unsigned int))

#define DG_REC_V3_PAIR_BYTES ((size_t)272)
#define DG_REC_V3_BYTES  (DG_REC_V1_PREFIX + DG_REC_V3_PAIR_BYTES + \
                          4 * sizeof(unsigned int))

#define DG_REC_V4_PAIR_BYTES ((size_t)424)
#define DG_REC_V4_BYTES  (DG_REC_V1_PREFIX + DG_REC_V4_PAIR_BYTES + \
                          4 * sizeof(unsigned int))

typedef struct {
    volatile long seq;          /* 0 empty; -1 being written; else 1-based
                                   append index of the payload */
    long pad_;
    DG_REC_FRAME f;
} DG_REC_SLOT;

typedef struct {
    DG_REC_SLOT slot[DG_REC_CAP];
    volatile long appended;     /* total frames ever recorded (monotone) */
    volatile long seen;         /* capture calls, duplicates included */
    volatile long dup;          /* folded away as byte-identical input */
    DG_REC_FRAME last;          /* dedupe reference, input span only */
    int have_last;
} DG_REC_RING;

void dg_rec_reset(DG_REC_RING *r);

/* DG_XR_FRAME -> record and back. Unpack rebuilds a frame the pipeline can
   consume via dg_xr_test_publish; fields the recorder does not carry (eye
   poses, FOVs, reference_relative, haptic handles) come back zeroed, and the
   replay harness is the documentation of why that is enough. */
void dg_rec_pack(const DG_XR_FRAME *f, long long qpc, unsigned int stream_id,
                 unsigned int pair_id, unsigned int present_frame,
                 unsigned int eye, DG_REC_FRAME *out);
void dg_rec_unpack(const DG_REC_FRAME *r, DG_XR_FRAME *out);

void dg_rec_pair_camera(DG_REC_PAIRSTATE *pair, const MAT *camera_world,
                        const MAT *projection);

/* Returns 1 if the frame was recorded, 0 if it was a byte-identical duplicate
   of the previous recorded input (time and identity fields excluded). */
int dg_rec_capture(DG_REC_RING *r, const DG_REC_FRAME *f);

/* Writes header + records (oldest surviving first). Returns records written,
   -1 on I/O error. torn_out (optional) counts slots skipped because the
   writer overwrote them mid-copy - nonzero only for a mid-session dump. */
long dg_rec_write(DG_REC_RING *r, long long qpf, FILE *out, long *torn_out);

int dg_rec_read_open(FILE *in, long *count_out, long long *qpf_out,
                     int *v1_out);
int dg_rec_read_next(FILE *in, int v1, DG_REC_FRAME *out);

#endif
