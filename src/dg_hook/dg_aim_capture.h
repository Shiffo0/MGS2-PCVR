#ifndef DG_AIM_CAPTURE_H
#define DG_AIM_CAPTURE_H
#include "dg_xr.h"
#include <stdint.h>

/* Bounded, revision-checked hand-root aim for M9, USP and run-1 weapons:
   SOCOM, AKS-74U, M4, RGB6, Coolant. Every actor must still resolve to the
   player's selected hand; model 0 must be parentless with the root's basis.
   Zero on refusal. No recorder state is touched. */
typedef struct {
    uint64_t arm, subobject, subobjs, hand, model;
    uint64_t weapon_id; /* explicit ID, also distinguishes reused object storage */
} DG_AIM_SELECTION;
int dg_aim_capture_hand_selection(uintptr_t image_base, uint64_t expected_arm,
                                DG_AIM_SELECTION *selection);

/* Nikita keeps native player-body ownership outside FPS aim. This separate
   witness admits only the measured one-piece launcher and its two objects.
   No trigger, camera, inventory or game-memory writes occur here. */
typedef struct {
    uint64_t player, actor, arm, body, body_objs, arm_objs;
    uint64_t native_root, hand, normal_object, normal_objs, normal_model;
    uint64_t subject_object, subject_objs, subject_model, normal_root;
} DG_NIKITA_BINDING;
int dg_aim_capture_nikita_binding(uintptr_t image_base, uint64_t expected_arm,
                                 DG_NIKITA_BINDING *out);

/* Opt-in observation only. No game writes, callbacks or new detours.
   Caller passes the EXACT XR frame and raw view used to build camera_world.
   raw_view_kind: -1 mono/head, 0/1 actual left/right eye, 2 head stereo-test.
   This is a camera-seam observation, not a final-draw witness. */
void dg_aim_capture_configure(int enabled);
void dg_aim_capture_native_configure(int enabled);
int dg_aim_capture_native_enabled(void);
void dg_aim_capture_native_observe(uintptr_t base,const MAT *camera,const MAT *projection);
int dg_aim_capture_enabled(void);
void dg_aim_capture_observe(uintptr_t image_base, uint64_t stream_id,
                            const MAT *camera_world, const MAT *projection,
                            const DG_XR_FRAME *frame, int frame_flags,
                            const DG_XR_RAW_POSE *raw_view, int raw_view_kind);
/* Worker only. Nonblocking with respect to camera thread: camera drops and
   counts an observation if a dump is copying the bounded ring. Returns rows,
   0 for empty, -1 for I/O failure. Ring survives failed/repeated dumps. */
long dg_aim_capture_dump(const char *path);
#endif
