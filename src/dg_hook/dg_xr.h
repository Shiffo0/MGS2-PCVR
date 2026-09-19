

#ifndef DG_XR_H
#define DG_XR_H
void dg_xr_zoom_changed(void);

#include <stdint.h>
#include "dg_radial_view.h"

/* Nonblocking copied-view mailbox. Generation is zero unless XR recently
 * produced a focused, renderable projection with capacity for this overlay.
 * Caller owns coherent labels/eligibility/catalog and must invalidate on any
 * gameplay-context cancellation (even when it has no visible view to publish).
 * A publication expires after 100ms; refresh every producer frame. */
uint64_t dg_xr_radial_generation(void);
int dg_xr_radial_publish(const dg_radial_view *view, uint64_t generation,
                         uint64_t context, uint64_t catalog);
void dg_xr_radial_invalidate(void);
/* Hide published views without withdrawing the session/reference lease. */
void dg_xr_radial_hide(void);

/* Physical gameplay hotkeys have explicit ownership. A held button must be
   released after a context/focus boundary before it can start a new gesture. */
typedef struct {
    unsigned context;
    unsigned a_armed, y_armed, a_down, y_down, y_fired;
    uint32_t y_since, last_ms;
    unsigned have_sample;
} DG_XR_BUTTON_GATE;
#define DG_XR_BUTTON_TOGGLE 1u
#define DG_XR_BUTTON_RECENTER 2u
unsigned dg_xr_button_gate_step(DG_XR_BUTTON_GATE *gate, unsigned context,
    int allowed, unsigned a, unsigned y, uint32_t now_ms);
/* Present is the sole context publisher; the XR thread owns button state.
   A context older than 100 ms refuses gameplay hotkeys. */
void dg_xr_controller_context(int gameplay);
int dg_xr_controller_toggle_take(void);

typedef void (*DG_XR_CAPTURE_OBSERVER)(int eye, uint64_t capture_id);
void dg_xr_set_capture_observer(DG_XR_CAPTURE_OBSERVER observer);

#ifdef DG_HOOK_TEST
unsigned dg_xr_test_controller_buttons(int active, unsigned a, unsigned y, uint32_t now_ms);
/* Runtime action-valid mask: bit 0 primary, bit 1 recenter action. */
unsigned dg_xr_test_controller_actions(unsigned valid, unsigned a, unsigned y, uint32_t now_ms);
void dg_xr_test_controller_context(int gameplay, uint32_t now_ms);
int dg_xr_test_controller_toggle_take(uint32_t now_ms);
int dg_xr_test_controller_recenter_current(uint32_t now_ms);
#endif

/* Shared with dg_hook.c so there is exactly one definition of a matrix. */
typedef struct { float m[4][4]; } MAT;

/* The pose handed to dg_hook, already in MGS2 terms: radians in view space,
   millimetres, and - importantly - tx/ty/tz are D's translation ROW, not the
   head's displacement. See dg_xr_head_to_pose() for why those differ. */
typedef struct {
    double yaw, pitch, roll;    /* radians, view space, YXZ order */
    double tx, ty, tz;          /* millimetres, D's translation row */
} DG_XR_POSE;

/* Defined HERE, next to MAT, rather than in dg_proj.h. dg_proj.h needs MAT, so
   if this header also needed dg_proj.h the two would form an include cycle -
   and an include guard makes a cycle terminate, not work: whichever header is
   reached first has the other's typedefs stripped out of it. Both shared types
   living at the root of the dependency breaks it properly.
   Radians, OpenXR sign convention: left and down are negative. */
typedef struct {
    double left, right, up, down;
} DG_PROJ_FOV;

/* Per-axis correction, all defaulting to +1. These exist because the sign
   conventions are the one part of the mapping that argument cannot settle:
   they are cheap to determine by moving your head once, and expensive to
   derive from first principles about a 2001 engine. Config keys are
   xr_yaw_sign, xr_pitch_sign, ... in the marker file. */
typedef struct {
    double yaw_sign, pitch_sign, roll_sign;
    double x_sign, y_sign, z_sign;
    double scale;               /* metres -> game units; 1000 = mm (plan 2.9) */
    int    positional;          /* 0 = rotation only, for isolating the gate */
    int    recenter_token;      /* any change to this re-captures the origin */
    int    stereo;              /* S4d: 1 = alternate the eye per game frame */
    int    stereo_test;         /* 1 = projection only, 2 = position only */
    int    stereo_proj_x_sign;  /* retail default -1; +1 restores unmirrored probe */
    double trigger_deadzone;    /* release threshold; default 0.10 */
    double trigger_fire;        /* press threshold; default 0.55 */
    /* The theater screen (vr_theater_dist / vr_theater_width, metres). The
       distance is baked into the anchor pose at capture; the width is read
       per frame, so it tunes live. Both clamped to 0.5..10 by the reader. */
    double theater_dist;        /* eye to screen centre; default 2.0 */
    double theater_width;       /* physical quad width; default 2.0 */
    int life_disabled;
    int grip_debug; /* read-only M9 hit-test visualization */       /* vr_life=off; default wrist + event LIFE */
    int    scene_blur_fix;      /* hook-only: suppress scene-history alpha in VR */
    int    stereo_phase_fix;   /* hook-only: bind capture to consumed render buffer */
    int    stereo_eye_x_sign;  /* hook-only: -1 = right eye at camera -x (default, headset-proven 2026-09-09); +1 swaps */
    /* hook-only, 2026-09-18: per-eye placement of the flat 2D sprites
       (camera overlay, HUD). See dg_ui2d.inl. */
    int    ui2d;               /* 0 = off (default), 1 = on */
    int    ui2d_scale_mils;    /* 300..1000, default 750 */
    int    ui2d_conv_e5;       /* convergence, 1e-5 NDC per eye, default 1200 */
    int    ui2d_sign;          /* +1 default, -1 mirrors the shift */
    int    ui2d_hold;          /* hook-only: camera-less frames 0 leave, 1 (default) hold, 2 opposite */
    int    ui2d_vs_count;
    int    feedback_skip;      /* hook-only: 1 (default) = do not forward the previous-frame feedback sprite in stereo */
    int    draw_skip;          /* hook-only: 0 observe, 1 hold skipped Presents, 2 also label drawn frames by their rendered eye */
    int    eye_truth;          /* hook-only: 0 observe, 1 drop mislabelled frames, 2 relabel them */
    /* 2026-09-18: the Soliton radar on the inside of the left wrist (dg_radar.inl, dg_xr_radar.inl). */
    int    radar_mode;         /* vr_radar_wrist: 0 off (default), 1 fixed (head-locked test quad), 2 wrist */
    int    radar_hud;          /* hook-only, vr_radar_hud: 1 (default) = leave the HUD radar, 0 = withhold its composite draw while the wrist radar is up */
    int    radar_space_local;  /* vr_radar_wrist_space: 0 (default) = the grip action space is the layer's space, 1 = pose composed in LOCAL */
    double radar_size;         /* vr_radar_wrist_size: quad width in metres, default 0.09; height follows the texture */
    double radar_offset[3];    /* vr_radar_wrist_offset: metres in the left grip space */
    double radar_rot[3];       /* vr_radar_wrist_rot: degrees, q = Ry*Rx*Rz (dg_radar_gaze.h) */
    double radar_gaze_deg;     /* vr_radar_gaze_deg: default 20; 0 = no gate, always shown */
    double radar_gaze_pitch;   /* vr_radar_gaze_pitch: head forward pitched down by this, default 15 */
    unsigned long long ui2d_vs[8];  /* sprite vertex-shader bytecode hashes */
} DG_XR_CONFIG;

/* ------------------------------------------------- S4d: stereo (plan 2.25) --
 *
 * Frame-sequential: game frame N is rendered for one eye, N+1 for the other,
 * and BOTH are submitted every runtime frame from a per-eye store. The stale
 * eye is submitted with the pose it was actually drawn at, which is what lets
 * the runtime reproject it correctly instead of smearing it onto the current
 * head position.
 */

/* Eye identifiers are the OpenXR view indices, so they index view[] and the
   per-eye stores directly rather than needing a mapping nobody would check. */
#define DG_EYE_LEFT   0
#define DG_EYE_RIGHT  1
#define DG_EYE_MONO  (-1)       /* stereo off: one image, shown to both eyes */

/* The eye's pose exactly as OpenXR reported it: the runtime's LOCAL space,
   NOT relative to the seated origin.
   This is emphatically not interchangeable with DG_XR_POSE. The composition
   layer is submitted in LOCAL space and needs this one; the camera is built
   relative to the seated origin and needs the other. Swapping them produces
   the exact smear frame-sequential rendering exists to avoid, and it looks
   like a tracking bug rather than a units bug. */
typedef struct {
    double qx, qy, qz, qw;
    double px, py, pz;
} DG_XR_RAW_POSE;

/* Reference-relative but still in OpenXR axes/metres. Its distinct type keeps
   later game-axis conversion from ever accepting a raw LOCAL pose by mistake. */
typedef struct {
    double qx, qy, qz, qw;
    double px, py, pz;
} DG_XR_REL_POSE;

enum {
    DG_XR_HAND_LEFT = 1,
    DG_XR_HAND_RIGHT = 2,
    DG_XR_POSE_GRIP = 1,
    DG_XR_POSE_AIM = 2
};

typedef struct {
    unsigned int hand;
    unsigned int kind;
    DG_XR_RAW_POSE raw_local;
    DG_XR_REL_POSE reference_relative;
    unsigned int position_valid;
    unsigned int orientation_valid;
    unsigned int active;
    unsigned int tracked;
    unsigned int pose_age_ms;
    uint64_t sample_seq;
    int64_t xr_time;
} DG_XR_HAND_POSE;

typedef struct {
    unsigned int hand;
    DG_XR_HAND_POSE grip;
    DG_XR_HAND_POSE aim;
    float trigger_value;
    unsigned int trigger_click;
    uint64_t trigger_press_seq;
    uint64_t trigger_release_seq;
    float squeeze_value;
    unsigned int squeeze_click;
    float thumbstick_x;
    float thumbstick_y;
    unsigned int thumbstick_active;
    unsigned int thumbstick_click;
    unsigned int primary_button;
    unsigned int secondary_button;
    unsigned int menu_button;
    uintptr_t haptic_output;
} DG_XR_HAND;

/* Everything about one eye for one runtime frame. */
typedef struct {
    DG_XR_POSE     pose;        /* for the camera: relative to the origin */
    DG_PROJ_FOV    fov;         /* that eye's own asymmetric frustum, radians */
    DG_XR_RAW_POSE raw;         /* for the layer: LOCAL space, as reported */
} DG_XR_EYE;

/* One seqlock record covering both eyes and the mono fallback, so that a
   single read cannot mix values from different frames. S4c published the pose
   and the FOV in one record but read them through two calls, which could in
   principle disagree; this closes that. */
typedef struct {
    DG_XR_EYE      eye[2];      /* indexed by DG_EYE_LEFT / DG_EYE_RIGHT */
    DG_XR_POSE     head;        /* stereo off: the head pose, as S4b/S4c */
    DG_XR_RAW_POSE head_raw;    /* the same head pose, LOCAL, for the layer */
    DG_PROJ_FOV    union_fov;   /* stereo off: S4c's union of both eyes */
    DG_XR_HAND     left_hand;
    DG_XR_HAND     right_hand;
} DG_XR_FRAME;

/* Bit 1 = poses usable, bit 2 = FOVs usable; 0 means leave the camera alone.
   Bounded, lock-free seqlock read - VEH-safe, exactly like dg_xr_get. */
int  dg_xr_get_stereo(DG_XR_FRAME *out);

/* Start the XR thread. Idempotent; returns 0 if OpenXR is unavailable, in
   which case the caller must carry on flat. */

int  dg_xr_adopt_device(void *d3d11_device);

/* Wrist radar. Called by the draw hook with the ID3D11Texture2D the game's
   radar composite draw samples, and with NO lock of the caller's held: it
   takes the capture critical section itself and only then calls D3D, the
   order the XR thread uses. -1 = not stored, 0 = stored, 1 = stored and the
   wrist layer is up, so the HUD radar may be withheld. */
int  dg_xr_radar_capture(void *d3d11_texture2d);
void dg_xr_radar_stats(char *out, size_t n);

/* Called from the game's Present. Copies the back buffer into that eye's store
   together with the pose and FOV the frame was ACTUALLY drawn with, so the
   image can still be submitted honestly once it is a frame old.
 *
 * eye   DG_EYE_LEFT / DG_EYE_RIGHT, or DG_EYE_MONO for the stereo=0 path.
 * raw   the layer pose to submit this image with. NULL means "use the live
 *       pose", which is correct only for MONO, where the image is current.
 * fov   the frustum the frame was rendered with. NULL means unknown, and the
 *       image must then not be submitted at all rather than guessed at.
 *
 * Passing fov explicitly replaces S4b's habit of recovering it from m00/m11:
 * the camera hook knows exactly what it retargeted to, so asking the pixels
 * afterwards was always the long way round. A frame the camera hook rejected
 * has no eye and no honest pose - pass fov = NULL for it and the store keeps
 * whatever it held, rather than taking a frame drawn from the game's own
 * camera and labelling it as an eye. */
void dg_xr_capture(void *swapchain, int eye,
                   const DG_XR_RAW_POSE *raw, const DG_PROJ_FOV *fov);

typedef struct {
    MAT camera_world, projection;
    DG_XR_RAW_POSE head_raw;
    DG_XR_HAND_POSE grip, aim;
    uint64_t camera_qpc;
    unsigned present_id;
    int valid;
} DG_NEAR_META;
void dg_xr_capture_measured(void *sc, int eye, const DG_XR_RAW_POSE *raw,
                           const DG_PROJ_FOV *fov, const DG_NEAR_META *meta);
int  dg_xr_submitting(void);        /* 1 once real layers are going out */

/* The theater/panel screen switch, called from Present with the bridge's
   verdict for this frame. 1 asks the frame loop to submit the mono store on
   a yaw-anchored LOCAL-space quad instead of the projection layer; 0 restores
   the projection. The frame loop owns every edge: it drops both capture
   stores on any change of this flag (frames captured under one submission
   rule must never be paired with the other's poses - the raw_pose_usable
   lesson) and re-anchors the quad on recentre and on a distance change.
   Fail-closed at every step: no anchor or no valid mono store means the
   plain mono PROJECTION goes out instead (head-coupled, today's fallback),
   and with nothing captured, zero layers - exactly as today. */
void dg_xr_screen(int active);

/* How many runtime frames have gone out as the theater quad. For the log. */
long dg_xr_screen_frames(void);

/* The quad's pose: yaw-only head orientation (pitch and roll stripped - the
   screen must be world-upright however the head was tilted at entry), centre
   placed dist metres straight ahead of the head ALONG THAT YAW, at eye
   height. LOCAL-space in, LOCAL-space out; after this the pose never moves
   with the head again - free look is the compositor's job. Pure, shared with
   the desk build, and the desk test pins the yaw-stripping so a head-coupled
   mutant cannot pass. */
void dg_xr_screen_pose(const DG_XR_RAW_POSE *head, double dist_m,
                       DG_XR_RAW_POSE *out);

int  dg_xr_start(void (*log)(const char *fmt, ...));
void dg_xr_stop(void);
/* Optional manual M9 interaction feedback. */
void dg_xr_m9_feedback(unsigned events);

/* Live config update, called from the worker as the marker file is re-read. */
void dg_xr_configure(const DG_XR_CONFIG *cfg);

/* Safe from any thread; the request stays pending until a valid pose arrives. */
void dg_xr_recenter(void);

/* -------- software turn: the stick rotates the tracking space ------------
   The recentre reference is rotated in place, pivoting about the LIVE head,
   so the view, the walk frame and the controllers all turn together - a
   stick turn becomes indistinguishable from the player turning on their
   feet, which is exactly the property the arm needs (measured 2026-08-23:
   with the body turned by pad bytes instead, the arm stayed pinned to the
   old facing and the player recalibrated eleven times in one session).
   Positive rate = turn right. The offset since the last reference capture
   is exposed so the arm's frozen frame can ride along; a recentre absorbs
   the offset back to zero by construction. */
void   dg_xr_turn_rate(double deg_per_s);   /* refresh <=100ms; 0 stops; focused XR only */
double dg_xr_turn_offset_rad(void);         /* accumulated since last capture */
/* The pure step, exposed for the desk suite: advances a yaw-only reference
   quaternion {qy,qw} by delta and re-pivots its position {px,pz} about the
   head so the mapped head pose is invariant. */
void   dg_xr_turn_step(double ref_q[2], double ref_p[2],
                       double head_px, double head_pz, double delta_rad);
void   dg_xr_test_set_turn_offset(double rad);

/* The explicit request is deliberately allowed to override the worn gate:
   a person can use the backstop even when a runtime reports presence badly. */
int dg_xr_should_capture(int have_ref, int explicit_req, int worn,
                         unsigned tracked_ms);

/* Latest pose. Returns 0 if there is no usable tracking right now, in which
   case the caller must leave the camera alone rather than apply a stale pose.
   Safe to call from the VEH: it is a bounded, lock-free seqlock read. */
int  dg_xr_get(DG_XR_POSE *out);
int  dg_xr_get_target_fov(DG_PROJ_FOV *out); /* 0 if not ready */
int  dg_xr_get_frame(DG_XR_POSE *pose, DG_PROJ_FOV *fov); /* bit 1 pose, bit 2 FOV */

/* Counters for the log, so a bad session can be diagnosed without a debugger. */
void dg_xr_stats(long *frames, long *valid, long *invalid, long *errors);

/* ---- exposed only so the self-test can reach them ---- */

/* Standard column-vector rotation matrix from a quaternion, stored as-is into
   a row-vector MAT. The caller is responsible for having already conjugated
   and mirrored the quaternion; see dg_xr_head_to_pose(). */
void dg_xr_quat_to_mat(double x, double y, double z, double w, MAT *out);

/* Decompose a row-vector rotation into the YXZ order that build_delta()
   composes, i.e. R = Ry(yaw) * Rx(pitch) * Rz(roll). Any other order would
   silently disagree with the transform dg_hook actually applies. */
void dg_xr_mat_to_ypr(const MAT *r, double *yaw, double *pitch, double *roll);

/* The whole conversion, factored out so it can be tested without a headset:
   an OpenXR head pose relative to the recentre reference (quaternion xyzw and
   position in metres, both already expressed in the reference frame) becomes
   a POSE for build_delta(). */
void dg_xr_head_to_pose(double qx, double qy, double qz, double qw,
                        double px, double py, double pz,
                        const DG_XR_CONFIG *cfg, DG_XR_POSE *out);

void dg_xr_pose_relative(const DG_XR_RAW_POSE *reference,
                         const DG_XR_RAW_POSE *pose,
                         DG_XR_REL_POSE *out);
void dg_xr_pose_from_relative(const DG_XR_RAW_POSE *reference,
                              const DG_XR_REL_POSE *relative,
                              DG_XR_RAW_POSE *out);
/* How many times B (right) or Y (left) has gone down on that hand since the
   process started, where hand is DG_XR_HAND_LEFT or DG_XR_HAND_RIGHT. A COUNT rather than a level, because the consumer is a
   once-a-second poll on another thread: a level can be pressed and released
   entirely between two polls and never be seen, and a count cannot. Never
   decreases, so a consumer only has to remember the last value it acted on. */
unsigned long dg_xr_secondary_presses(unsigned int hand);

/* Rising edges of B/Y that arrived while the trigger was NOT held, and were
   therefore ignored. A re-zero request is only meaningful mid-aim - the whole
   ritual is "hold the controller as you want the weapon held" - and the thumb
   rests on exactly that button while walking. One session measured 18 presses
   and 20 frame re-freezes in about a minute of walking, which the player saw
   as the arm tumbling; the trigger gate is what ended it. */
unsigned long dg_xr_secondary_ignored(unsigned int hand);

/* Rising-edge sequence for the LEFT Quest menu/hamburger button.  A sequence
   rather than a level is required because the game-side consumer runs on a
   different thread and the button may be tapped between two camera seams. */
uint64_t dg_xr_menu_press_seq(void);

/* The gate itself, pure so the desk build can pin its truth table: a press
   counts only on a rising edge with the same hand's trigger held. */
/* Menu edge ownership: 0 none, 1 native START, 2 theater recenter. */
unsigned int dg_xr_menu_route(unsigned int was_down, unsigned int is_down,
                              int theater);
unsigned int dg_xr_secondary_counts(unsigned int was_down, unsigned int is_down,
                                    unsigned int trigger_click);

/* Which part of the reference pose's ROTATION dg_xr_pose_relative_frame
   removes. The translation is always removed, in every mode: subtracting the
   head's position is what stops a lean reading as a hand movement, and that
   was never the part in question.

   HEAD removes all of it, so the result is what the hand is doing relative to
   where you are LOOKING. That is wrong for an arm: your arm hangs off your
   torso, and turning your head does not carry it with you. Kept because it is
   what every build up to 2026-08-19 used.

   YAW removes only the rotation about the up axis. Looking up, down or tilting
   your head no longer moves the arm at all; turning left and right still
   carries it, which is not anatomy but is self-correcting - the arm can never
   end up somewhere you cannot turn to find.

   ROOM removes the RECENTRE heading instead of anything the head is doing
   now, so no part of the live head reaches the arm at all. That is the honest
   model: the character's arm root turns with the character's body and not with
   our head, so body-frame in maps to body-frame out. It is the seated origin
   and not the runtime's raw LOCAL yaw for one reason - a player who turns
   their chair can put it right with the recentre they already have, where a
   raw tracking-space frame would have no correction at all. Before a seated
   origin exists it behaves as YAW rather than as an unreferenced frame. */
enum {
    DG_XR_REL_FRAME_HEAD = 0,
    DG_XR_REL_FRAME_YAW  = 1,
    DG_XR_REL_FRAME_ROOM = 2
};

/* The rotation about +Y contained in q, and nothing else - the twist half of a
   swing/twist split about the up axis. Degenerate input (a head rotated so far
   that the twist is undefined) yields the identity rather than a guess. */
void dg_xr_quat_yaw_only(const double q[4], double out[4]);

/* How many times the seated origin has been captured this process - the
   initial automatic capture included. The arm watches this to restart its own
   calibration on a recentre, so one held button re-nulls the camera and the
   arm together instead of leaving them pointing at two different forwards. */
long dg_xr_recenter_count(void);

unsigned int dg_xr_trigger_hysteresis(float value, float deadzone,
                                      float fire, unsigned int was_pressed);
void dg_xr_hand_pose_sample(DG_XR_HAND_POSE *out, unsigned int hand,
                            unsigned int kind,
                            const DG_XR_RAW_POSE *raw_local,
                            const DG_XR_REL_POSE *reference_relative,
                            unsigned int position_valid,
                            unsigned int orientation_valid,
                            unsigned int position_tracked,
                            unsigned int orientation_tracked,
                            unsigned int active, uint64_t sample_seq,
                            int64_t xr_time);

#ifdef DG_HOOK_TEST
void dg_xr_test_publish(const DG_XR_FRAME *frame);
void dg_xr_test_press_secondary(unsigned int hand);
void dg_xr_test_recenter(void);
#endif

#endif
