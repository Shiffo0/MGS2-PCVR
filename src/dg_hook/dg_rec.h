#include "dg_build_profile.h"
/* dg_rec.h - the flight recorder: raw XR input, kept, dumped, replayed.
 *
 * Every arm defect so far has been diagnosed from a phone video of a mirror
 * window plus a once-per-30-seconds heartbeat - which is why single bugs have
 * cost multiple headset runs. This module records the PUBLISHED XR frame (the
 * exact struct the whole pipeline consumes) so that any session can be
 * replayed at a desk through the real code, frame by frame, with every
 * intermediate value printed. The recording is the pipeline's complete input
 * surface: if replay ever fails to reproduce a live defect, a field is
 * missing HERE, and that is a finding rather than a mystery.
 *
 * Capacity: DG_REC_CAP = 16384 frames. The runtime publishes at headset rate
 * (~72-90 Hz measured), and duplicates seen by the faster camera seam are
 * folded away, so the ring holds the LAST ~3 minutes of input in ~15 MiB of
 * .bss. Three minutes is chosen deliberately: it covers the standard test
 * choreography plus the wandering that usually precedes "and THEN it broke",
 * and a mid-session dump knob exists for anything longer.
 *
 * File format: compact little-endian binary with a self-describing text
 * header (magic, version, field list, record size, QPC frequency, count).
 * Binary rather than CSV because the acceptance bar is BIT-identical replay -
 * doubles must survive the file untouched - and because the desk harness can
 * always export CSV afterwards. The header's record_bytes is validated on
 * read, so a struct that drifted refuses the old files instead of misreading
 * them.
 *
 * Threading: exactly one writer (the camera seam, inside the VEH - so the
 * append path is plain volatile stores into preallocated memory: no
 * allocation, no locking, no I/O, nothing that can fault). Each slot carries
 * a sequence stamp written before and after the payload; a reader that sees
 * the stamps disagree, or a stamp from a different lap of the ring, drops
 * that slot and counts it. This relies on MSVC's /volatile:ms ordering (the
 * default on x64, and what every build script here uses), the same reliance
 * dg_xr's own seqlock already carries.
 */

#ifndef DG_REC_H
#define DG_REC_H

#include <stdio.h>
#include "dg_xr.h"
#include "dg_aim_replay.h"

#define DG_REC_CAP DG_DIAGNOSTIC_CAPACITY(16384)
#define DG_REC_MAGIC  "DGREC6"
#define DG_REC_MAGIC_V5 "DGREC5"
/* Older files are still readable, fail-open by layout: v1 (no game-side pair
   state) comes back with the pair block zero-filled; v2 (pair block without
   rest_drift_deg) comes back with the drift at its absence value; v3 (pair
   block without the solved-arm tail) comes back with that tail zeroed and
   its flags unset. v4 lacks the camera tail, which is zeroed; CAMERA is
   cleared for every older layout. The reader reports the layout through its
   compat flag. */
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

/* The game-side half of the pair: what the character's hierarchy and the
   bridge's captured rests held when this frame was taken. This is the half
   whose absence made every tumble cost a live headset run - the XR input
   alone replays the map and the IK, but the HAND demand is built against
   the live animation base, and that base is the game's. 472 bytes, no
   holes. Filled by the bridge one camera-seam pass behind the XR frame
   (same thread, see the tap); all-zero when the bridge had nothing, with
   flags saying which fields stand. */
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
    /* REST-freeze reference drift at this pair, degrees: the larger of the
       upper/fore reference-vs-recovered angles the heartbeat's "REST ref"
       line prints. -1.0 when vr_arm_freeze=rest was not active or the
       bridge computed no drift this pass - 0.0 is a real measurement
       (perfect agreement), so absence needs its own value. v2 and v1
       files come back at -1.0. */
    float rest_drift_deg;

    unsigned int frame_word;
    /* DGREC4 (2026-09-03, run 14): the arm as this pass solved it, so the
       desk can see the joints the hand demand was built on. Four runs of
       wrist questions (a 150-degree swing demand the standard-skeleton
       model could not reproduce) ended at the headset because none of
       this was in the file. Floats: a millimetre or a degree to three
       decimals is all the analysis reads. All zero, with the DG_REC_PAIR_F_
       ARM / ENVELOPE / FACING flags clear, when the pass never got that
       far; v3 and older files come back the same way. */
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
    /* DGREC5: raw row-vector camera-to-world basis and projection samples
       captured at the same camera seam. These are evidence of the matrices
       used by that seam; CAMERA does not assert a gun axis or input route. */
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
#define DG_REC_PAIR_F_FRAME_LIVE 128u /* vr_adjust_frame was live this pair */
#define DG_REC_PAIR_F_ARM      256u /* joint_world / ik_* / q4_world / q5_world stand */
#define DG_REC_PAIR_F_ENVELOPE 512u /* fore_axis / raw_twist_deg / raw_swing_deg / fit_frac stand */
#define DG_REC_PAIR_F_FACING  1024u /* head_yaw_deg / stick_yaw_deg / aim_yaw_deg stand */
#define DG_REC_PAIR_F_CAMERA  2048u /* camera tail is valid at this seam */
#define DG_REC_PAIR_F_CHANNEL_PROJECT 4096u /* fit_frac is angular retention,
                                             not a radial scale factor */
#define DG_REC_FRAME_WORD_VALID 0x80000000u

/* One recorded frame. qpc first and the u32 tail last so the struct packs
   with no holes; the dedupe span is [head .. pair] - identity fields
   (time, frame number, eye, stream, pair id) never make two identical
   inputs count as different ones, but a game-side change under identical
   controller input DOES record: an animation moving beneath a still hand
   is exactly the evidence the recorder exists to keep. */
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

/* The v1 layout, needed to read old recordings: everything up to and
   including hand[1], then the u32 tail - no pair block. */
#define DG_REC_V1_PREFIX ((size_t)(8 + sizeof(double[7]) + \
                                   2 * sizeof(DG_REC_HAND)))
#define DG_REC_V1_BYTES  (DG_REC_V1_PREFIX + 4 * sizeof(unsigned int))

/* The v2 layout: the v1 prefix, then the pair block as it stood before
   rest_drift_deg - everything up to and including `flags` - then the u32
   tail. The 264 is deliberately a literal: it is the OLD sizeof, and tying
   it to the current struct would defeat the point of remembering it. */
#define DG_REC_V2_PAIR_BYTES ((size_t)264)
#define DG_REC_V2_BYTES  (DG_REC_V1_PREFIX + DG_REC_V2_PAIR_BYTES + \
                          4 * sizeof(unsigned int))

/* The v3 layout: the v1 prefix, the pair block up to and including
   frame_word - 272, the old sizeof, again a literal on purpose - then the
   u32 tail. */
#define DG_REC_V3_PAIR_BYTES ((size_t)272)
#define DG_REC_V3_BYTES  (DG_REC_V1_PREFIX + DG_REC_V3_PAIR_BYTES + \
                          4 * sizeof(unsigned int))

/* The v4 layout is the old current pair block before the DGREC5 camera tail.
   Keep the value literal: tying it to the current struct would defeat the
   compatibility boundary. */
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

/* Capture the camera seam's raw matrices into the pair. The camera tail and
   CAMERA flag are cleared before validation, so every rejected sample fails
   closed and cannot leave stale evidence behind. */
void dg_rec_pair_camera(DG_REC_PAIRSTATE *pair, const MAT *camera_world,
                        const MAT *projection);

/* Returns 1 if the frame was recorded, 0 if it was a byte-identical duplicate
   of the previous recorded input (time and identity fields excluded). */
int dg_rec_capture(DG_REC_RING *r, const DG_REC_FRAME *f);

/* Writes header + records (oldest surviving first). Returns records written,
   -1 on I/O error. torn_out (optional) counts slots skipped because the
   writer overwrote them mid-copy - nonzero only for a mid-session dump. */
long dg_rec_write(DG_REC_RING *r, long long qpf, FILE *out, long *torn_out);

/* Header validation. Returns 1 and fills count/qpf on success; 0 on any
   mismatch (magic, version, record size), which is the fail-closed refusal
   of files written by a different layout. Older files are accepted with
   *v1_out set to their DG_REC_COMPAT_* code (v1: no pair block; v2: pair
   block without rest_drift_deg); pass that value to read_next unchanged so
   it fills the missing fields with their absence values. The parameter
   keeps its historical name - callers shuttle it blindly, and a caller that
   still tests it as a boolean reads every old layout as "old", which is
   the safe direction. v1_out may be NULL when the caller only probes. */
int dg_rec_read_open(FILE *in, long *count_out, long long *qpf_out,
                     int *v1_out);
int dg_rec_read_next(FILE *in, int v1, DG_REC_FRAME *out);

#endif
