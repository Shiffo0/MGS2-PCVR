#ifndef DG_AIM_PROBE_H
#define DG_AIM_PROBE_H
#include <stdint.h>

/* Pure observation diagnostic. No allocation, game access, I/O or publication.
   No assertion that an observed root is the final drawn barrel transform. */
#define DG_AIM_PROBE_VERSION 1u
#define DG_AIM_PROBE_LABEL "OBSERVATION_ROOT_MINUS_Y_PROXY"
#define DG_AIM_PROBE_MAX_AGE_MS 100u
#define DG_AIM_PROBE_QUAT_NORM2_TOL 0.0001
#define DG_AIM_PROBE_MATRIX_TOL 0.002
#define DG_AIM_PROBE_HAVE_CAMERA 1u
#define DG_AIM_PROBE_HAVE_AIM 2u
#define DG_AIM_PROBE_HAVE_EYE 4u
#define DG_AIM_PROBE_HAVE_HAND 8u
#define DG_AIM_PROBE_HAVE_ROOT 16u
#define DG_AIM_PROBE_REQUIRED 31u
#define DG_AIM_PROBE_CONSUMED_KNOWN 32u

typedef enum {
    DG_AIM_PROBE_OK = 0, DG_AIM_PROBE_NULL, DG_AIM_PROBE_MISSING,
    DG_AIM_PROBE_IDENTITY, DG_AIM_PROBE_SAMPLE_MISMATCH,
    DG_AIM_PROBE_STALE, DG_AIM_PROBE_NONFINITE,
    DG_AIM_PROBE_QUATERNION, DG_AIM_PROBE_MATRIX
} DG_AIM_PROBE_RESULT;

typedef struct {
    uint64_t observation_id, stream_id, publication_aim_seq;
    /* All tags identify the aim sample in one captured DG_XR_FRAME, NOT independently
       meaningful DG_XR_HAND_POSE counters. Observation is not consumption. */
    uint64_t camera_publication_aim_seq, aim_publication_aim_seq, eye_publication_aim_seq;
    uint64_t hierarchy_observation_id;
    uint64_t consumed_command_sample_seq; /* zero unless CONSUMED_KNOWN */
    uint64_t selected_body, selected_unit, selected_root;
    uint64_t observed_body, observed_unit, observed_root;
    uint64_t expected_arm_body, observed_arm_body;
    uint32_t flags, aim_age_ms, eye_age_ms, weapon_type;
    /* Rows are world directions of local axes. No translation is required. */
    float camera_world[3][3], weapon_root_world[3][3];
    double aim_raw_q[4], eye_raw_q[4], hand_world_q[4]; /* x,y,z,w */
} DG_AIM_PROBE_INPUT;

typedef struct {
    uint32_t version, valid;
    DG_AIM_PROBE_INPUT observation;
    double root_minus_y_world[3], root_minus_y_camera[3];
    double hand_minus_y_world[3], hand_minus_y_camera[3];
    double raw_eye_relative_aim_xr[3];
    double root_hand_basis_angle_deg, root_hand_minus_y_angle_deg;
} DG_AIM_PROBE_SAMPLE;

/* out is all-zero on ANY rejection; result is separate rejection evidence.
   Successful output remains an observation proxy, never draw/aim-error proof.
   Accepted near-unit quaternions are used as supplied, never silently fixed.
   Directions are explicitly normalized for vector geometry after validation. */
DG_AIM_PROBE_RESULT dg_aim_probe_build(const DG_AIM_PROBE_INPUT *in,
                                      DG_AIM_PROBE_SAMPLE *out);
const char *dg_aim_probe_result_name(DG_AIM_PROBE_RESULT result);
#endif
