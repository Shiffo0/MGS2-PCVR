/* dg_hook.asi - the in-process camera hook.  Closes S2.
 *
 * S2 (plan 2.12) proved an external write reaches the renderer but cannot be
 * *ordered* against it: we won most frames and flickered on the rest. 2.13
 * located the writer - DG_Chanls[0]'s camera is produced by the function at
 * RVA 0x85A80, whose caller resumes at 0x85FA4, and every one of the thirteen
 * camera matrices is final by the time control reaches that address.
 *
 * The camera uses a hardware *execution* breakpoint at 0x85FA4. It runs
 * on the game's own thread, synchronously, inside the frame, after the camera
 * is written and before any consumer in 2.7's list reads it - the PHASE_FIRST
 * semantics of 2.7 without needing to locate DG_AddPlugin.
 *
 * A second, signature-gated breakpoint suppresses only the fullscreen
 * previous-frame blur alpha during VR stereo (see dg_scene_blur.h).
 * These camera/blur hooks patch no .text and allocate no trampoline. The
 * separately implemented game-thread bridge does use its own detours.
 * No game file is changed by these hooks. The retail .text lives under a
 * Steam DRM wrapper (the .bind section), so the two narrow synchronous
 * interventions use execution breakpoints instead of code patches.
 *
 * Disarm restores the debug registers and the game's own camera returns on the
 * next frame. Deleting the marker file disarms within a second.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tlhelp32.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ------------------------------------------------------------- offsets --- */

#define DG_CHANLS_RVA   0x15522A0ull
#define DG_CHANL_STRIDE 0x530ull
#define HOOK_RVA        0x85FA4ull      /* return site of the camera update */

#define O_EYE_PERS           0x000
#define O_EYE_INV            0x040
#define O_EYE                0x080
#define O_PERS               0x0C0
#define O_EYE_PERS2          0x100
#define O_PERS2              0x140
#define O_RAISE_PERS         0x180
#define O_RAISE_PERS2        0x1C0
#define O_RAISE_EYE_PERS     0x200
#define O_RAISE_EYE_PERS2    0x240
#define O_PERS_NO_OFFSET     0x280
#define O_EYE_PERS_NO_OFFSET 0x2C0

#define MARKER   "dg_hook.on"           /* absent => inert; contents = config */
#define PRESENT_MARKER "dg_present.on"  /* absent => no D3D11 hook at all */
#define XR_MARKER      "dg_xr.on"       /* absent => nothing submitted to XR */
#define LOG_NAME "logs\\dg_hook.log"
#define HOST_EXE "METAL GEAR SOLID2.exe"

#define WATCHDOG_TRAPS_PER_SEC 20000    /* runaway guard - see arm_loop() */

#include "dg_xr.h"                      /* also supplies MAT and DG_PROJ_FOV */
#include "dg_proj.h"                    /* S4c: per-eye projection retarget */
#include "dg_present.h"
#include "dg_draw_trial.h"                 /* S4a: D3D11 capture, no submission */
#include "dg_bridge.h"                  /* F2: game-thread bridge, FPS toggle */
#include "dg_pose.h"                    /* F5: stereo-latched controller pose */
#include "dg_fire.h"                    /* F7: trigger to the weapon pad contract */
#include "dg_rec.h"                     /* F8: flight recorder, replayed at a desk */
#include "dg_weapon_aim.h"
#include "dg_aim_capture.h"             /* optional coherent camera/weapon observation */
#include "dg_aim_target.h"
#include "dg_xr_script.h"
#include "dg_script_gate.h"
#include "dg_radial_owner.h"
#include "dg_radial_view.h"
#include "dg_menu.h"                    /* U2: controller to menu-pad contract */
#include "dg_codec_menu.h"
#include "dg_scene_blur.h"
/* Last, and with the redirect on: pose/IK/arm-map calls made in this file
   dispatch through the hot-reload policy table (dg_policy.c). With no DLL
   staged the table IS the statically linked modules - identical behaviour,
   one indirection. */
#define DG_POLICY_REDIRECT
#include "dg_policy.h"

#define SRC_SYNTHETIC 0
#define SRC_XR        1
#define SRC_SCRIPT    2

/* ------------------------------------------------------------- globals --- */

static ULONG64 g_base, g_hook_addr, g_chan0;
static ULONG64 g_scene_blur_addr; /* zero on unknown retail signature */
static volatile LONG g_scene_blur_enabled, g_scene_blur_hits, g_scene_blur_zeroed;
static unsigned long g_arm_pose_stream_id = 1;
static DWORD   g_self_tid;
static PVOID   g_veh;
static char    g_logpath[MAX_PATH];

typedef struct {
    double yaw, pitch, roll;            /* radians */
    double tx, ty, tz;                  /* millimetres */
    double sweep_rad, sweep_hz;         /* optional oscillation */
    int    sweep_axis;                  /* 0 yaw, 1 pitch, 2 roll */
} POSE;

static volatile LONG  g_armed;
static POSE           g_pose;           /* written by worker, read by handler */
static volatile LONG  g_source;         /* SRC_SYNTHETIC | SRC_XR | SRC_SCRIPT */
static int g_present_available;         /* set by worker before any session */
static int g_script_boot;
static DG_SCRIPT_GATE g_script_gate;
static int g_script_token_active;
static uint64_t g_script_primary_seen;  /* consumed only by Present */
static LONG g_script_camera_seen;       /* script has no XR/FOV handoff */
static LONG g_buttons_camera_seen;
static DWORD g_buttons_camera_ms, g_buttons_start_ms;
static int g_buttons_have_camera, g_buttons_start_block;
static int g_buttons_gameplay;
static unsigned g_buttons_epoch;
static DG_XR_BUTTON_GATE g_script_button_gate;
static int g_script_menu_cfg;           /* parse result, never a live gate */
static volatile LONG g_script_menu_active;
static volatile LONG g_script_menu_context_lost; /* permanent for this session */
static SRWLOCK g_script_present_lock = SRWLOCK_INIT;
static int g_script_present_stopped; /* protected by g_script_present_lock */
static int script_menu_blocked(void) {
    return InterlockedCompareExchange(&g_script_menu_active, 0, 0) &&
           InterlockedCompareExchange(&g_script_menu_context_lost, 0, 0);
}
static void script_menu_sample_context(void) {
    if (InterlockedCompareExchange(&g_script_menu_active, 0, 0) &&
        !dg_bridge_menu_context_ready())
        InterlockedExchange(&g_script_menu_context_lost, 1);
}
static void on_present(IDXGISwapChain *sc);
static void on_present_body(IDXGISwapChain *sc);
static void script_menu_callbacks_stop(void) {
    /* Drain callbacks already inside the body, and refuse a pointer copied
       by dg_present before its callback slot is cleared. */
    AcquireSRWLockExclusive(&g_script_present_lock);
    g_script_present_stopped = 1;
    ReleaseSRWLockExclusive(&g_script_present_lock);
    dg_present_set_callback(NULL);
}

/* Select once per call, never advance a scenario in a consumer. The script
   owns a separate publication; it cannot race the real XR writer or fall
   back to physical controllers on EOF/error. All input routes use these. */
#ifdef DG_HOOK_TEST
static unsigned g_controls_input_reads;
#endif
static int input_get_frame(DG_XR_FRAME *out) {
#ifdef DG_HOOK_TEST
    g_controls_input_reads++;
#endif
    return g_source == SRC_SCRIPT ? dg_xr_script_get_frame(out)
                                  : dg_xr_get_stereo(out);
}
static double input_turn_offset_rad(void) {
    return g_source == SRC_SCRIPT ? dg_xr_script_turn_offset_rad()
                                  : dg_xr_turn_offset_rad();
}
static void input_turn_rate(double rate) {
    if (g_source == SRC_SCRIPT) dg_xr_script_turn_rate(rate);
    else dg_xr_turn_rate(rate);
}
static long input_recenter_count(void) {
    return g_source == SRC_SCRIPT ? dg_xr_script_recenter_count()
                                  : dg_xr_recenter_count();
}
static void input_recenter(void) {
    if (g_source == SRC_SCRIPT) dg_xr_script_recenter();
    else dg_xr_recenter();
}
static unsigned long input_secondary_presses(unsigned int hand) {
    return g_source == SRC_SCRIPT ? dg_xr_script_secondary_presses(hand)
                                  : dg_xr_secondary_presses(hand);
}
static unsigned long input_secondary_ignored(unsigned int hand) {
    return g_source == SRC_SCRIPT ? dg_xr_script_secondary_ignored(hand)
                                  : dg_xr_secondary_ignored(hand);
}
static uint64_t input_menu_press_seq(void) {
    return g_source == SRC_SCRIPT ? dg_xr_script_menu_press_seq()
                                  : dg_xr_menu_press_seq();
}
static DG_XR_CONFIG   g_xrcfg;
static LARGE_INTEGER  g_qpf, g_qpc0;
static volatile LONG  g_applied, g_fresh, g_traps;
/* Rejections split by cause: "rejected" as one bucket conflates a menu, a load
   screen and something actually wrong, which is useless for diagnosis. */
static volatile LONG  g_rej_nonfinite, g_rej_not_ortho, g_rej_not_proj;
static volatile LONG  g_rej_no_pose;
/* Theater consumers, counted so the heartbeat can prove all three switched
   together: camera frames handed back to the director, and Presents captured
   mono under the verdict. The quad frames are the XR side's own counter. */
static volatile LONG  g_thea_cam_released;
static volatile LONG  g_thea_mono_frames;

/* The flat-screen fallback. The title screen, the load screens and the boot
   logos never reach the camera daemon at all, so the camera hook never hands
   over a pose for them - and dg_xr_capture deliberately refuses a frame with
   no FOV ("if (!fov) return", dg_xr.c), which is right for a rejected GAME
   camera frame and wrong for an image that never had a camera in the first
   place. The eye stores therefore stay empty and the runtime is handed zero
   layers: the headset shows NOTHING at all on the main menu, which is what a
   player experiences as "I cannot see the menu". Those frames are flat 2D by
   nature, so the theater quad is exactly where they belong. Counted in
   frames of camera silence rather than switched on a state word, because the
   states that behave this way are precisely the ones no status word of ours
   is resolved for yet. */
#define DG_FLAT_SILENT_FRAMES 30
static volatile LONG  g_flat_mode = 1;      /* vr_flat, on by default */
static volatile LONG  g_flat_silent;        /* Presents since a live pose */
static volatile LONG  g_flat_frames;        /* Presents shown on the quad */

#define DG_MENU_MODE_OFF     0
#define DG_MENU_MODE_MEASURE 1
#define DG_MENU_MODE_WRITE   2
/* vr_arm_freeze, mirrored out of the config struct purely so the heartbeat
   can print what the bridge is really holding. See the parse site. */
static volatile LONG g_arm_freeze_cfg;
static volatile LONG g_menu_mode;
static volatile LONG g_menu_confirm = (LONG)DG_MENU_DEFAULT_CONFIRM;
static volatile LONG g_menu_cancel  = (LONG)DG_MENU_DEFAULT_CANCEL;
static volatile LONG g_menu_delay_ms  = DG_MENU_DEFAULT_INITIAL_MS;
static volatile LONG g_menu_repeat_ms = DG_MENU_DEFAULT_REPEAT_MS;
static volatile LONG g_menu_deadzone_mils =
    (LONG)(DG_MENU_DEFAULT_DEADZONE * 1000.0 + 0.5);
static volatile LONG g_menu_hand_right;     /* vr_menu_hand, left by default */
static volatile LONG g_start_mode;           /* vr_start=on */
static uint64_t g_start_seen;
/* Present-thread only, so no interlock: the state machine is touched from
   exactly one call site. The counters are interlocked because the heartbeat
   reads them from the logger thread. */
static DG_MENU_STATE g_menu_state;
static volatile LONG g_menu_open_frames;    /* Presents with a front-end screen */
static volatile LONG g_menu_steps;
static volatile LONG g_menu_repeats;
static volatile LONG g_menu_confirms;
static volatile LONG g_menu_cancels;
static volatile LONG g_menu_yields;
static volatile LONG g_menu_no_input;
static volatile LONG g_menu_bad_config;
static volatile LONG g_menu_unwritable;     /* decided, and had nowhere to go */
static volatile LONG g_menu_writes_asked;   /* decided, and handed to the seam */

static const unsigned int g_menu_sweep[] = {
    /* Round three: PAIRS. Rounds one and two swept twenty single bits with
       a proven-good input path (26 confirms sent, no-input 0, bad-config 0)
       and not one of them confirmed - while the measured CANCEL is itself a
       pair, 0x00040040 = cross | EX3. So an EX bit reads as "this is a real
       button event" and the face bit chooses which button. Confirm is
       therefore a face bit beside an EX bit, and this is that cross
       product, most likely first: EX3 is the one the live record actually
       carries. */
    0x00040020u, /* circle | EX3 - circle confirms in most MGS builds */
    0x00040010u, /* triangle | EX3 */
    0x00040080u, /* square | EX3 */
    0x00040800u, /* START | EX3 */
    0x00040100u, /* SELECT | EX3 */
    0x00020020u, /* circle | EX2 */
    0x00020040u, /* cross | EX2 */
    0x00010020u, /* circle | EX1 */
    0x00010040u  /* cross | EX1 */
};
#define DG_MENU_SWEEP_N \
    ((int)(sizeof(g_menu_sweep) / sizeof(g_menu_sweep[0])))
static volatile LONG g_menu_clear_bits;  /* vr_menu_clear */

static const DWORD g_pad_asgn_rva[] = {
    0x00AA5A80u, 0x00AA5A88u,
    0x0153FC30u, 0x0153FC34u,
    0x016C9E2Cu, 0x016C9E28u,
    0x016E7FB4u, 0x016E7FB0u,
    0x0098A4F4u, 0x0098A4F0u
};
#define DG_PAD_ASGN_N \
    ((int)(sizeof(g_pad_asgn_rva) / sizeof(g_pad_asgn_rva[0])))
static volatile LONG g_menu_sweep_on;
static volatile LONG g_menu_sweep_idx;
static volatile LONG g_menu_sweep_last;
/* What the module was actually handed, last frame it had a sample at all.
   "The trigger does nothing" has three different causes - the controller is
   not vouched for, the trigger value never reaches the click threshold, or
   the level is stuck high so no rising edge ever occurs - and they need
   opposite fixes. Guessing between them has already cost several runs. */
static volatile LONG g_menu_dbg_trigger;    /* f2l float, raw travel */
static volatile LONG g_menu_dbg_click;      /* f2l float, threshold */
static volatile LONG g_menu_dbg_buttons;    /* bit0 A, bit1 B, bit2 input_ok */
static volatile LONG g_menu_dbg_samples;

static MAT     g_mine;                  /* last eye_inv we wrote */
static volatile LONG g_have_mine;
/* The game's OWN camera for the current frame, kept because the transform must
   be rebuilt from it rather than from our own output. See apply_transform. */
/* Set when the worker, not a camera session, owns the XR session's lifetime.
   S4b runs XR submission with the camera hook completely inert, so a session
   ending must not tear down the picture in the headset. */
static volatile LONG g_xr_worker_owned;

/* Present and the camera breakpoint are not required to run on the same
   thread.  The eye is therefore an interlocked frame counter rather than a
   plain flag, and the camera's handoff crosses the thread boundary through a
   small seqlock.  The writer is the VEH; Present is the bounded reader. */
#include "dg_capture_handoff.h"
static DG_NEAR_META g_near_pending;

static volatile LONG g_current_eye;
static volatile LONG g_present_frame;    /* increments exactly once per Present */
static volatile LONG g_stereo;
static volatile LONG g_stereo_test;   /* 0 full, 1 projection-only, 2 position-only */
static volatile LONG g_stereo_proj_x_sign; /* retail -1, +1 restores probe */
static volatile LONG g_stereo_eye_x_sign;  /* -1 (default): right eye at camera -x; +1 swaps */
/* Eye truth (2026-09-18). Present labels a frame with the eye of the NEWEST
   camera handoff, but the GPU renders with a camera two updates old (measured
   with dg_cb_probe). Usually the two agree; when the game skips or doubles a
   frame they do not, and the frame is submitted under the other eye's pose and
   frustum. Invisible while the HUD sat on the same pixels in both eyes; a
   visible flash now that ui2d places it per eye. The constant-buffer upload
   hook knows which frustum every frame was REALLY rendered with
   (dg_ui2d_last_frame_sign). This learns which sign belongs to which eye
   from the agreeing majority and counts the exceptions; vr_eye_truth=1 drops
   a mislabelled frame (the store keeps that eye's last honest image), 2
   relabels it with the rendered eye's own last handoff. 0 only observes. */
static volatile LONG g_eye_truth_mode;
static long g_eye_truth_n[2][2];            /* [label eye][sign < 0] */
static volatile LONG g_eye_truth_none, g_eye_truth_mismatch, g_eye_truth_dropped, g_eye_truth_relabelled;
/* Camera-less Presents (2026-09-18 evening). About one stereo Present in
   fifteen carries no perspective uploads: the game drew no scene of its own,
   yet the frame was captured under the STALE handoff's eye. If the back buffer
   then holds an older image (the other eye's, with its HUD baked in) that is a
   one-frame flash no sprite placement can repair. vr_eye_truth=3 does not
   submit such a frame: the store keeps that eye's last honest image. Bounded
   to DG_NOCAM_DROP_MAX in a row so a pause menu still reaches the headset. */
#define DG_NOCAM_DROP_MAX 6
static volatile LONG g_nocam_dropped, g_nocam_passed;
/* Probe (2DF4AF87 live: camera-less frames average ~1000 draws, so they DO
   hold a scene, and dropping them made play worse). What are they? The first
   DG_NOCAM_LOG_MAX such stereo Presents are logged in full, then every 100th. */
#define DG_NOCAM_LOG_MAX 40
static long g_nocam_seen, g_nocam_small_seen, g_nocam_traps_last, g_nocam_seq_last;
/* Object-eye probe: per stereo Present, the eye the scene's object shaders were
   really drawn with (dg_ui2d_last_frame_obj), against the label and against the
   PREVIOUS Present. Healthy alternate-eye rendering flips every Present; "same"
   means two Presents in a row showed the same eye = the mono the desktop mirror
   shows from the bad angles. Ticks per Present and Present spacing go with it. */
static long g_obj_n[2][4];                 /* [label eye][+, -, mixed, none] */
static long g_obj_same, g_obj_alt, g_obj_events, g_obj_run, g_obj_run_max;
static int  g_obj_prev;
/* Draw skip (2026-09-18, found with vr_eye_dump). From view angles with large
   objects close by, the engine draws only every SECOND Present: the pictures
   of Presents N (label L) and N+1 (label R) are byte-identical, consecutive
   distinct pictures show no eye shift at all. The constant uploads still
   alternate, which is why every matrix-level measurement looked healthy. With
   the eye flipping on every Present, the drawn frames are then always the SAME
   eye and the undrawn Present re-submits that picture under the other eye:
   mono on the desktop mirror, double in the headset.
   vr_draw_skip=1: a Present whose frame issued (almost) no draw calls is not
   submitted and does not flip the eye, so drawn frames alternate eyes again.
   vr_draw_skip=2: additionally a drawn frame is submitted under the eye it was
   really rendered with (object-shader sign, mapping learned while no skipping
   is going on), using that eye's own last handoff. 0 only counts. */
static volatile LONG g_draw_skip_mode, g_feedback_skip = 1;
static long g_draw_ema, g_draw_hist[4], g_draw_skipped, g_draw_held, g_draw_relabelled, g_draw_recent;
static long g_draw_map[2][2];              /* [label eye][sign < 0], learned only while nothing is being skipped */
static DG_HOOK_HANDOFF g_draw_last[2]; static int g_draw_have[2];
/* Back-buffer probe: frames that drew nothing into the swap chain's back buffer
   cannot have produced a new picture, whatever else they drew. */
static long g_bb_hist[3], g_bb_seen, g_bb_empty, g_bb_src_same, g_bb_src_alt, g_bb_events;
static void *g_bb_src_prev, *g_bb_src_prev2;
/* 1 = this frame issued so few draws that the picture cannot be new. */
static int draw_skip_detect(long draws, long *ema)
{
    if (draws >= 300) { *ema = *ema ? (*ema * 7 + draws) / 8 : draws; return 0; }
    return *ema > 0 && draws * 5 < *ema;
}
/* The label eye that a rendered sign belongs to, or -1 while unknown. */
static int draw_skip_label(long map[2][2], int sign)
{
    int s = sign < 0 ? 1 : 0; long l = map[0][s], r = map[1][s];
    if (!sign || l + r < 50) return -1;
    return l >= 4 * r ? 0 : r >= 4 * l ? 1 : -1;
}
static long g_obj_ticks[4], g_obj_dt[3];   /* camera updates per Present 0,1,2,3+; dt <20, <40, 40+ ms */
static DWORD g_nocam_tick_last;
static int g_nocam_run;
static int nocam_drop_step(int sign, LONG mode, int *run)
{
    if (sign) { *run = 0; return 0; }
    if (mode != 3) return 0;
    if (*run >= DG_NOCAM_DROP_MAX) return 0;
    (*run)++;
    return 1;
}
static DG_HOOK_HANDOFF g_eye_truth_last[2];
static int g_eye_truth_have[2];
/* 0 = no opinion, 1 = the label matches the render, -1 = it does not. */
static int eye_truth_step(int label_eye, int sign)
{
    int e = label_eye == DG_EYE_RIGHT ? 1 : 0, s = sign < 0 ? 1 : 0, verdict = 0;
    long a, b;
    if (!sign) { InterlockedIncrement(&g_eye_truth_none); return 0; }
    a = g_eye_truth_n[e][0]; b = g_eye_truth_n[e][1];
    if (a + b >= 50 && (a >= 4 * b || b >= 4 * a))
        verdict = ((a > b) ? 0 : 1) == s ? 1 : -1;
    if (g_eye_truth_n[e][s] < 1000000) g_eye_truth_n[e][s]++;
    if (verdict < 0) InterlockedIncrement(&g_eye_truth_mismatch);
    return verdict;
}
static volatile LONG g_eye_shift_applied, g_eye_shift_fallback;
static volatile LONG g_eye_shift_half_x100; /* last half separation, 0.01 mm */
static volatile LONG g_handoff_seq;
static DG_HOOK_HANDOFF g_handoff;

static MAT     g_src_eye_inv, g_src_eye;
static volatile LONG g_reapplied;       /* rebuilt over a camera the game left alone */
/* The projection family has the same rebase hazard as the camera.  These are
   the game's last untouched values; g_proj_mine holds our previous output. */
static MAT g_src_pers, g_src_pers2, g_src_raise_pers, g_src_raise_pers2;
static MAT g_src_pers_no_offset;
static MAT g_proj_mine, g_proj2_mine, g_raise_proj_mine, g_raise_proj2_mine;
static MAT g_proj_no_offset_mine;
static volatile LONG g_have_proj_mine;
/* One camera-seam telemetry sample.  The sample is published only after a
   successful transform; the VEH clears the valid bit before every pass so a
   rejected, theatrical, or disarmed seam cannot reuse an older camera. */
static MAT g_camera_telemetry_eye_world;
static MAT g_camera_telemetry_proj;
static volatile LONG g_camera_telemetry_valid;
static volatile LONG g_arm_absolute_aim;
/* Camera thread only: exact publication used to construct the rendered view. */
static struct {
    int valid;
    DG_XR_FRAME frame;
    DG_XR_RAW_POSE view;
    MAT camera;
    DG_CAMERA_GATE gate;
} g_aim_camera;

/* Camera-base heading anchor. State is camera-thread only; the bridge gate is
   read at the same seam and its identity starts a new epoch. */
static int g_camera_yaw_anchor_have;
static volatile LONG g_camera_yaw_anchor_mode;
static volatile LONG g_camera_yaw_anchor_generation;
static LONG g_camera_yaw_anchor_seen_generation = -1;
static double g_camera_yaw_anchor_heading;
static ULONGLONG g_camera_yaw_anchor_arm;
static ULONGLONG g_camera_yaw_anchor_camera;
static long g_camera_yaw_anchor_recenter;

static void camera_yaw_anchor_reset(void)
{
    g_camera_yaw_anchor_have = 0;
    g_camera_yaw_anchor_heading = 0.0;
    g_camera_yaw_anchor_arm = 0;
    g_camera_yaw_anchor_camera = 0;
}

#ifdef DG_HOOK_TEST
static int g_test_camera_gate_override = -1;
static DG_CAMERA_GATE g_test_camera_gate;
#endif

static int camera_gate_now(DG_CAMERA_GATE *out)
{
#ifdef DG_HOOK_TEST
    if (g_test_camera_gate_override >= 0) {
        if (out) {
            if (g_test_camera_gate_override) *out = g_test_camera_gate;
            else memset(out, 0, sizeof(*out));
        }
        return g_test_camera_gate_override;
    }
#endif
    return dg_bridge_camera_gate_now(out);
}

static void camera_pass_begin(void)
{
    InterlockedExchange(&g_camera_telemetry_valid, 0);
    g_aim_camera.valid = 0;
}

/* ------------------------------------------------------------- logging --- */

/* The line ending every log call uses, named once: an escaped pair is
   easy to mangle when these lines get edited by script. */
#define DG_EOL "\r\n"

static void logf_(const char *fmt, ...) {
    char buf[1024];
    va_list ap;
    int n, cut = 0;
    HANDLE h;
    DWORD w;
    va_start(ap, fmt);
    n = _vsnprintf_s(buf, sizeof(buf), _TRUNCATE, fmt, ap);
    va_end(ap);
    /* _TRUNCATE reports -1 rather than the length it wanted, so an overlong
       line used to end mid-word with nothing saying so. That is exactly what
       happened to the bridge arm block once the menu-status line was added:
       522 bytes into a 512-byte buffer, cut inside "(unsafe mask ", and the
       half that fell off was the value the live gate existed to read. A
       silent truncation in the log is worse than a missing line, because it
       looks like data. Say so instead. */
    if (n < 0) {
        n = (int)strlen(buf);
        cut = 1;
    }
    h = CreateFileA(g_logpath, FILE_APPEND_DATA, FILE_SHARE_READ, NULL,
                    OPEN_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return;
    SetFilePointer(h, 0, NULL, FILE_END);
    WriteFile(h, buf, (DWORD)n, &w, NULL);
    if (cut) WriteFile(h, " <LOG LINE TRUNCATED>\r\n", 23, &w, NULL);
    CloseHandle(h);
}

/* ---------------------------------------------------------------- math --- */

static void mat_mul(MAT *o, const MAT *a, const MAT *b) {
    int i, j, k;
    MAT t;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) {
        double s = 0.0;
        for (k = 0; k < 4; k++) s += (double)a->m[i][k] * (double)b->m[k][j];
        t.m[i][j] = (float)s;
    }
    *o = t;
}

/* Row-vector basis rotations: v' = v * R. */
static void rot_y(MAT *o, double rad) {
    double c = cos(rad), s = sin(rad);
    memset(o, 0, sizeof(*o));
    o->m[0][0] = (float)c;  o->m[0][2] = (float)(-s);
    o->m[1][1] = 1.0f;
    o->m[2][0] = (float)s;  o->m[2][2] = (float)c;
    o->m[3][3] = 1.0f;
}

static void rot_x(MAT *o, double rad) {
    double c = cos(rad), s = sin(rad);
    memset(o, 0, sizeof(*o));
    o->m[0][0] = 1.0f;
    o->m[1][1] = (float)c;  o->m[1][2] = (float)s;
    o->m[2][1] = (float)(-s); o->m[2][2] = (float)c;
    o->m[3][3] = 1.0f;
}

static void rot_z(MAT *o, double rad) {
    double c = cos(rad), s = sin(rad);
    memset(o, 0, sizeof(*o));
    o->m[0][0] = (float)c;  o->m[0][1] = (float)s;
    o->m[1][0] = (float)(-s); o->m[1][1] = (float)c;
    o->m[2][2] = 1.0f;
    o->m[3][3] = 1.0f;
}

/* Build the view-space rigid delta D and its exact inverse.
     D     = R * T(t)     so  v * D     = (v * R) + t
     D^-1  = T(-t) * R^T  so  v * D^-1  = (v - t) * R^T
   The inverse is constructed, not solved for: R is orthonormal by construction,
   so transposing is both exact and free, and a numerically drifting inverse
   here would show up as the camera and its projection disagreeing. */
static void build_delta(MAT *d, MAT *d_inv, const POSE *p) {
    MAT ry, rx, rz, r, tmp;
    int i, j;
    double t[3], ti[3];

    rot_y(&ry, p->yaw);
    rot_x(&rx, p->pitch);
    rot_z(&rz, p->roll);
    mat_mul(&tmp, &ry, &rx);
    mat_mul(&r, &tmp, &rz);

    t[0] = p->tx; t[1] = p->ty; t[2] = p->tz;

    *d = r;
    d->m[3][0] = (float)t[0];
    d->m[3][1] = (float)t[1];
    d->m[3][2] = (float)t[2];

    memset(d_inv, 0, sizeof(*d_inv));
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
        d_inv->m[i][j] = r.m[j][i];                 /* R^T */
    /* ti = -t * R^T, so ti[j] contracts t against ROW j of R. Indexing R the
       other way round transposes it, which is invisible for pure rotation and
       for pure translation, and wrong only when both are present. */
    for (j = 0; j < 3; j++)
        ti[j] = -(t[0] * r.m[j][0] + t[1] * r.m[j][1] + t[2] * r.m[j][2]);
    d_inv->m[3][0] = (float)ti[0];
    d_inv->m[3][1] = (float)ti[1];
    d_inv->m[3][2] = (float)ti[2];
    d_inv->m[3][3] = 1.0f;
}

static int finite_mat(const MAT *m) {
    int i, j;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) {
        float v = m->m[i][j];
        if (v != v || v > 3.0e38f || v < -3.0e38f) return 0;
    }
    return 1;
}

static int rows_orthonormal(const MAT *m) {
    int r;
    for (r = 0; r < 3; r++) {
        double a = m->m[r][0], b = m->m[r][1], c = m->m[r][2];
        double l = sqrt(a*a + b*b + c*c);
        if (l < 0.998 || l > 1.002) return 0;
    }
    return 1;
}

static int camera_rigid_valid(const MAT *m)
{
    double d, dot01, dot02, dot12;
    if (!finite_mat(m) || !rows_orthonormal(m)) return 0;
    if (fabs(m->m[0][3]) > 1e-4 || fabs(m->m[1][3]) > 1e-4 ||
        fabs(m->m[2][3]) > 1e-4 || fabs(m->m[3][3] - 1.0) > 1e-4)
        return 0;
    dot01 = m->m[0][0]*m->m[1][0] + m->m[0][1]*m->m[1][1] + m->m[0][2]*m->m[1][2];
    dot02 = m->m[0][0]*m->m[2][0] + m->m[0][1]*m->m[2][1] + m->m[0][2]*m->m[2][2];
    dot12 = m->m[1][0]*m->m[2][0] + m->m[1][1]*m->m[2][1] + m->m[1][2]*m->m[2][2];
    if (fabs(dot01) > 1e-3 || fabs(dot02) > 1e-3 || fabs(dot12) > 1e-3)
        return 0;
    d = m->m[0][0] * (m->m[1][1] * m->m[2][2] - m->m[1][2] * m->m[2][1])
      - m->m[0][1] * (m->m[1][0] * m->m[2][2] - m->m[1][2] * m->m[2][0])
      + m->m[0][2] * (m->m[1][0] * m->m[2][1] - m->m[1][1] * m->m[2][0]);
    return d > 0.998 && d < 1.002;
}

static int camera_heading(double x, double z, double *out)
{
    double h;
    if (!out || x != x || z != z || fabs(x) + fabs(z) < 1e-5) return 0;
    h = atan2(x, z);
    if (h != h || h < -3.141592653589793 - 1e-9 ||
        h > 3.141592653589793 + 1e-9) return 0;
    *out = h;
    return 1;
}

static void camera_inverse_from_world(MAT *out, const MAT *world)
{
    int i, j;
    double t[3];
    memset(out, 0, sizeof(*out));
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
        out->m[i][j] = world->m[j][i];
    t[0] = world->m[3][0]; t[1] = world->m[3][1]; t[2] = world->m[3][2];
    for (j = 0; j < 3; j++)
        out->m[3][j] = (float)-(t[0] * world->m[j][0] +
                                 t[1] * world->m[j][1] +
                                 t[2] * world->m[j][2]);
    out->m[3][3] = 1.0f;
}

static int camera_step_height_apply(MAT *world, MAT *inverse, int real_xr,
                                     int have_height, float height)
{
    if (!real_xr || !have_height || !isfinite(height) ||
        !camera_rigid_valid(world) || fabs(world->m[3][1]-height) > 250.0f)
        return 0;
    world->m[3][1] = height;
    camera_inverse_from_world(inverse, world);
    return 1;
}

/* Stereo eye placement (2026-09-09). Until now each eye's OpenXR position
   went through dg_xr_head_to_pose as if it were the head, and the camera was
   rebuilt per eye from that. The position there is mapped by the mirror
   (x, y, -z) while the orientation is mapped by the relabel (qx, -qy, -qz,
   qw); those differ by a mirror in y, so the vector between the eyes picked
   up a spurious rotation of twice the head roll plus pitch cross terms.
   Measured on the 14:17 aim dump: the eyes sat (56, 23, 13) mm apart in the
   camera's own basis instead of (62, 0, 0) - 23 mm of vertical disparity,
   which no one can fuse at pistol distance. The desk replica of that math
   reproduced the measurement to 1.3 mm.

   So: build the camera from the HEAD for both eyes (the path stereo_test=1
   already proved single) and then move it half the eye separation along the
   finished camera's own x-axis. Row 0 of the camera->world matrix is that
   axis (row vectors; the aim dump reads it the same way). The inverse is
   rebuilt from the shifted matrix so eye and eye_inv stay one camera. Which
   eye goes to +x is a marker sign (stereo_eye_x_sign). The headset settled
   it on 2026-09-09: with the right eye at +x (the side the old per-eye path
   produced) the pistol stayed double; at -x it fused with depth. So camera
   +x is screen-left, the default is -1, and +1 remains as the A/B. */
static void stereo_eye_shift(MAT *eye, MAT *eye_inv, int eye_idx, double half,
                             int sign)
{
    double ax[3], n, s;
    int j;
    ax[0] = eye->m[0][0]; ax[1] = eye->m[0][1]; ax[2] = eye->m[0][2];
    n = sqrt(ax[0]*ax[0] + ax[1]*ax[1] + ax[2]*ax[2]);
    if (!(n > 1e-6) || !(half > 0.0)) return;
    s = (eye_idx == DG_EYE_RIGHT ? 1.0 : -1.0) * (sign < 0 ? -1.0 : 1.0) * half / n;
    for (j = 0; j < 3; j++) eye->m[3][j] = (float)(eye->m[3][j] + s * ax[j]);
    camera_inverse_from_world(eye_inv, eye);
}

/* Half the eye separation from the raw LOCAL eye poses, in game units.
   0 when the runtime's views are not a plausible pair (40..90 mm), in which
   case the caller keeps the old per-eye path and counts it. */
static double stereo_half_separation(const DG_XR_FRAME *f, double scale)
{
    double dx = f->eye[1].raw.px - f->eye[0].raw.px;
    double dy = f->eye[1].raw.py - f->eye[0].raw.py;
    double dz = f->eye[1].raw.pz - f->eye[0].raw.pz;
    double sep = sqrt(dx*dx + dy*dy + dz*dz) * scale;
    if (!(sep >= 40.0 && sep <= 90.0)) return 0.0;
    return sep * 0.5;
}

static int camera_yaw_anchor_apply(MAT *world, double *heading_state,
                                   int *have_state)
{
    MAT ry, corrected;
    double current, delta;
    if (!camera_rigid_valid(world) || !heading_state || !have_state) return 0;
    if (!camera_heading(world->m[2][0], world->m[2][2], &current)) return 0;
    if (!*have_state) { *heading_state = current; *have_state = 1; return 1; }
    delta = *heading_state - current;
    rot_y(&ry, delta);
    /* Row vectors: rotating each basis row in world space is a right
       multiplication. A pre-multiply only appears correct for level cameras
       and corrupts tilted bases. */
    mat_mul(&corrected, world, &ry);
    corrected.m[3][0] = world->m[3][0];
    corrected.m[3][1] = world->m[3][1];
    corrected.m[3][2] = world->m[3][2];
    corrected.m[0][3] = world->m[0][3];
    corrected.m[1][3] = world->m[1][3];
    corrected.m[2][3] = world->m[2][3];
    corrected.m[3][3] = world->m[3][3];
    if (!camera_rigid_valid(&corrected)) return 0;
    *world = corrected;
    return 1;
}

#include "dg_hanging_camera.inl"
static int projlike(const MAT *p) {
    return (float)fabs(p->m[2][3] - 1.0f) < 1e-4f && (float)fabs(p->m[3][3]) < 1e-4f
        && (float)fabs(p->m[0][0]) > 1e-4f && (float)fabs(p->m[1][1]) > 1e-4f;
}

static int next_eye(int eye) {
    /* Written as a test against RIGHT rather than an XOR so that any malformed
       value collapses to a valid index instead of propagating: the result is
       always LEFT or RIGHT, and both are array indices into eye[2] by
       contract. */
    return (eye == DG_EYE_RIGHT) ? DG_EYE_LEFT : DG_EYE_RIGHT;
}

static DG_PROJ_FOV mirror_fov_x(DG_PROJ_FOV fov) {
    double old_left = fov.left;
    fov.left = -fov.right;
    fov.right = -old_left;
    return fov;
}

static void phase_record(int kind,int buffer);
static void handoff_write(int valid, int eye, const DG_XR_RAW_POSE *raw,
                          const DG_PROJ_FOV *fov) {
    InterlockedIncrement(&g_handoff_seq);       /* odd: reader must retry */
    g_handoff.near_meta = g_near_pending;
    if (!valid) g_handoff.near_meta.valid = 0;
    g_handoff.valid = valid;
    g_handoff.eye = eye;
    if (raw) g_handoff.raw = *raw;
    else memset(&g_handoff.raw, 0, sizeof(g_handoff.raw));
    if (fov) g_handoff.fov = *fov;
    else memset(&g_handoff.fov, 0, sizeof(g_handoff.fov));
    InterlockedIncrement(&g_handoff_seq);       /* even: publish the record */
    if (valid) { phase_record(1,-1); dg_ui2d_eye(eye); }
}

static int handoff_read(DG_HOOK_HANDOFF *out) {
    LONG before, after;
    int tries;
    for (tries = 0; tries < 8; tries++) {
        before = InterlockedCompareExchange(&g_handoff_seq, 0, 0);
        if (before & 1) continue;
        *out = g_handoff;
        after = InterlockedCompareExchange(&g_handoff_seq, 0, 0);
        if (before == after && !(after & 1)) return 1;
    }
    return 0;
}

#include "dg_pair_probe.h"
#include "dg_capture_identity.inl"
#include "dg_render_link.inl"
#include "dg_stereo_phase_probe.inl"
#include "dg_zoom_projection.h"
static DG_ZOOM_PROJECTION g_zoom_projection;
static double g_zoom_last_gain=1;
static uint64_t g_zoom_last_identity;
/* ------------------------------------------------------------ the hook --- */

#define M(off) ((MAT *)(g_chan0 + (off)))

/* Runs on the game's thread with the camera complete and nothing having read
   it yet. Convention is settled bitwise (2.12): eye_pers = eye_inv * pers. */
static void apply_transform(void) {
    MAT d, d_inv, n_eye_inv, n_eye, base_eye_inv, base_eye;
    MAT *pers2 = M(O_PERS2), *raise_pers = M(O_RAISE_PERS);
    MAT *raise_pers2 = M(O_RAISE_PERS2), *pers_no_offset = M(O_PERS_NO_OFFSET);
    MAT base_pers, base_pers2, base_raise_pers, base_raise_pers2;
    MAT base_pers_no_offset;
    DG_PROJ_FOV old_fov, new_fov, submit_fov;
    int have_target = 0, eye_idx = DG_EYE_LEFT, probe_frame_flags = 0;
    int probe_view_kind = -1;
    int eye_shift = 0;
    double eye_half = 0.0;
    DG_XR_FRAME frame;
    const DG_XR_RAW_POSE *use_raw = NULL;
    POSE p = g_pose;
    /* Every invocation owns its validity window, including direct test and
       replay calls that do not pass through VEH's camera_pass_begin(). */
    InterlockedExchange(&g_camera_telemetry_valid, 0);
    DG_CAMERA_GATE camera_gate;
    g_aim_camera.valid = 0;
    int camera_anchor = 0;

    MAT *eye_pers = M(O_EYE_PERS);
    MAT *eye_inv  = M(O_EYE_INV);
    MAT *eye      = M(O_EYE);
    MAT *pers     = M(O_PERS);

    /* The breakpoint fires for every call of the camera update, whichever
       channel it was for. If channel 0 still holds the value we wrote, the game
       has not refreshed it - either the update was for another channel, or the
       in-game camera is simply static.
       Re-transforming the value in memory would compound the pose, but SKIPPING
       is just as wrong: it pins the view to whatever pose was current when the
       camera last moved, and MGS2 holds a fixed camera for most of a room. The
       head would stop moving the view the moment Snake stood still.
       So the game's own camera is cached and the transform rebuilt from that. */
    if (g_have_mine && memcmp(eye_inv, &g_mine, sizeof(MAT)) == 0) {
        base_eye_inv = g_src_eye_inv;
        base_eye     = g_src_eye;
        InterlockedIncrement(&g_reapplied);
    } else {
        base_eye_inv = *eye_inv;
        base_eye     = *eye;
        g_src_eye_inv = base_eye_inv;
        g_src_eye     = base_eye;
        InterlockedIncrement(&g_fresh);
    }
    if(dg_aim_capture_native_enabled())
        dg_aim_capture_native_observe((uintptr_t)g_base,&base_eye,pers);
    {

        double h;
        if (finite_mat(&base_eye) &&
            camera_heading(base_eye.m[2][0], base_eye.m[2][2], &h))
            dg_bridge_pad_origin_now((LONG)(((long)floor(h * 2048.0 /
                                      3.141592653589793 + 0.5)) & 4095));
        else
            dg_bridge_pad_origin_now(-1);
    }

    /* Invalidate before every attempt, so a rejected camera update cannot
       cause Present to label an older image with this frame's eye. */
    handoff_write(0, DG_EYE_MONO, NULL, NULL);

    {
        DG_CAMERA_GATE radar_gate;
        dg_radar_view(g_source==SRC_XR && camera_gate_now(&radar_gate));
    }
    if (dg_bridge_theater_verdict()) {
        dg_radar_view(0);
        camera_yaw_anchor_reset();
        InterlockedIncrement(&g_thea_cam_released);
        return;
    }

    camera_anchor = InterlockedCompareExchange(&g_camera_yaw_anchor_mode, 0, 0) == 1;
    {
        LONG generation = InterlockedCompareExchange(&g_camera_yaw_anchor_generation, 0, 0);
        if (g_camera_yaw_anchor_seen_generation != generation) {
            camera_yaw_anchor_reset();
            g_camera_yaw_anchor_seen_generation = generation;
        }
    }
    if (!camera_anchor) {
        camera_yaw_anchor_reset();
    } else if (!camera_gate_now(&camera_gate)) {
        camera_yaw_anchor_reset();
    } else {
        long recenter_now = input_recenter_count();
        if (g_camera_yaw_anchor_recenter != recenter_now ||
            camera_gate.arm_body != g_camera_yaw_anchor_arm ||
            camera_gate.camera != g_camera_yaw_anchor_camera)
            camera_yaw_anchor_reset();
        g_camera_yaw_anchor_arm = camera_gate.arm_body;
        g_camera_yaw_anchor_camera = camera_gate.camera;
        g_camera_yaw_anchor_recenter = recenter_now;
    }
    if (!finite_mat(&base_eye_inv))       { if (camera_anchor) camera_yaw_anchor_reset(); InterlockedIncrement(&g_rej_nonfinite); return; }
    if (!rows_orthonormal(&base_eye_inv)) { if (camera_anchor) camera_yaw_anchor_reset(); InterlockedIncrement(&g_rej_not_ortho); return; }
    if (!projlike(pers))                  { if (camera_anchor) camera_yaw_anchor_reset(); InterlockedIncrement(&g_rej_not_proj);  return; }

    /* Sampled here rather than on the worker thread: a pose is only meaningful
       at the instant the frame's camera is built. Reading it a whole tick early
       is exactly the latency error the in-process hook exists to eliminate. */
    if (g_source != SRC_SYNTHETIC) {
        DG_XR_POSE xp;
        /* No tracking means leave the camera alone. Holding the last pose would
           freeze the view wherever the head was when tracking died, which reads
           as the mod having crashed rather than as tracking having dropped. */
        {
            int flags = input_get_frame(&frame);
            probe_frame_flags = flags;
            if (!(flags & 1)) {
            camera_yaw_anchor_reset();
            InterlockedIncrement(&g_rej_no_pose); return;
            }
            eye_idx = (InterlockedCompareExchange(&g_current_eye, 0, 0) == DG_EYE_RIGHT)
                ? DG_EYE_RIGHT : DG_EYE_LEFT;
            if (g_stereo) {
                /* Stereo changes two things at once - where each eye is, and
                   what frustum it renders - and double vision can come from
                   either. stereo_test isolates them so one launch answers it
                   instead of an argument:
                     1 = per-eye PROJECTION only, both eyes drawn from the head
                     2 = per-eye POSITION only, both at S4c's union FOV
                   The layer pose must follow whichever was used, or the image
                   is submitted from somewhere it was not drawn. */
                LONG mode = g_stereo_test;
                if (mode == 1) {
                    xp = frame.head;
                    use_raw = &frame.head_raw;
                    probe_view_kind = 2;
                    new_fov = frame.eye[eye_idx].fov;
                } else {
                    /* Modes 0 and 2: the camera is built from the head and
                       moved along its own x-axis afterwards (see
                       stereo_eye_shift). The old per-eye pose stays only as
                       a counted fallback for an implausible view pair. */
                    eye_half = stereo_half_separation(&frame, g_xrcfg.scale);
                    if (eye_half > 0.0) {
                        xp = frame.head;
                        eye_shift = 1;
                    } else {
                        xp = frame.eye[eye_idx].pose;
                        InterlockedIncrement(&g_eye_shift_fallback);
                    }
                    use_raw = &frame.eye[eye_idx].raw;
                    probe_view_kind = eye_idx;
                    new_fov = mode == 2 ? frame.union_fov : frame.eye[eye_idx].fov;
                }
                /* S4c's union FOV was horizontally symmetric, so it could not
                   validate which screen direction m[2][0] moves content. Keep
                   the OpenXR layer's original FOV, but permit the MGS2 render
                   frustum to be mirrored independently while settling that one
                   live sign. A mode change drops both stores in dg_xr_configure,
                   so old and new projection rules can never share a pair. */
                submit_fov = new_fov;
                dg_ui2d_display(submit_fov.left, submit_fov.right);
                if (mode != 2 &&
                    InterlockedCompareExchange(&g_stereo_proj_x_sign, 0, 0) < 0)
                    new_fov = mirror_fov_x(new_fov);
            } else {
                xp = frame.head;
                new_fov = frame.union_fov;
            }
            have_target = (flags & 2) != 0;
        }
        /* The marker's own yaw/pitch/roll/x/y/z stay live as a fixed offset,
           so the seated origin can be nudged without a recentre. */
        p.yaw += xp.yaw;  p.pitch += xp.pitch;  p.roll += xp.roll;
        p.tx  += xp.tx;   p.ty    += xp.ty;     p.tz   += xp.tz;
    } else if (p.sweep_rad != 0.0 && p.sweep_hz != 0.0) {
        LARGE_INTEGER now;
        double t, osc;
        QueryPerformanceCounter(&now);
        t = (double)(now.QuadPart - g_qpc0.QuadPart) / (double)g_qpf.QuadPart;
        osc = p.sweep_rad * sin(6.283185307179586 * p.sweep_hz * t);
        if      (p.sweep_axis == 1) p.pitch += osc;
        else if (p.sweep_axis == 2) p.roll  += osc;
        else                        p.yaw   += osc;
    }

    if (g_source == SRC_XR) {
        float standing_y;
        int have_height = dg_bridge_camera_standing_height_now(&standing_y);
        if (have_height)
            camera_step_height_apply(&base_eye, &base_eye_inv, 1, 1, standing_y);
    }

    if (camera_anchor && camera_gate.valid) {
        /* The cache remains untouched: anchor only the local copy just read
           above, then reconstruct its inverse from the corrected rigid camera. */
        if (!camera_yaw_anchor_apply(&base_eye, &g_camera_yaw_anchor_heading,
                                     &g_camera_yaw_anchor_have))
        {
            camera_yaw_anchor_reset();
            return;
        }
        camera_inverse_from_world(&base_eye_inv, &base_eye);
        if (!finite_mat(&base_eye_inv) || !camera_rigid_valid(&base_eye_inv))
        {
            camera_yaw_anchor_reset();
            return;
        }
    }

    /* Moving FPS skips native hanging SubjectTurn. Align only the local
       render base with the actual ledge-facing actor; tracked head motion
       is still applied below and the native camera/cache remain untouched. */
    if (g_source == SRC_XR) {
        double heading=0;
        int hanging=dg_bridge_hanging_heading_now(&heading);
        hanging_camera_apply(&base_eye,&base_eye_inv,hanging,heading);
    }
    build_delta(&d, &d_inv, &p);

    /* Retarget only from the game's own projection values.  A static camera
       update can leave our prior projection in memory, just as it leaves our
       prior eye_inv there; feeding that back would compound the FOV every
       frame. */
    if (have_target) {
#define REBASE_PROJ(cur, mine, src, base) \
        do { \
            if (g_have_proj_mine && memcmp((cur), &(mine), sizeof(MAT)) == 0) \
                (base) = (src); \
            else { (base) = *(cur); (src) = (base); } \
        } while (0)
        REBASE_PROJ(pers,            g_proj_mine,       g_src_pers,            base_pers);
        REBASE_PROJ(pers2,           g_proj2_mine,      g_src_pers2,           base_pers2);
        REBASE_PROJ(raise_pers,      g_raise_proj_mine, g_src_raise_pers,      base_raise_pers);
        REBASE_PROJ(raise_pers2,     g_raise_proj2_mine,g_src_raise_pers2,     base_raise_pers2);
        REBASE_PROJ(pers_no_offset,  g_proj_no_offset_mine, g_src_pers_no_offset,
                    base_pers_no_offset);
#undef REBASE_PROJ

        old_fov.left = -atan(1.0 / (double)base_pers.m[0][0]);
        old_fov.right = -old_fov.left;
        old_fov.up = atan(1.0 / fabs((double)base_pers.m[1][1]));
        old_fov.down = -old_fov.up;
        *pers = base_pers;
        *pers2 = base_pers2;
        *raise_pers = base_raise_pers;
        *raise_pers2 = base_raise_pers2;
        *pers_no_offset = base_pers_no_offset;

        dg_proj_retarget(pers,           &old_fov, &new_fov);
        dg_proj_retarget(pers2,          &old_fov, &new_fov);
        dg_proj_retarget(raise_pers,     &old_fov, &new_fov);
        dg_proj_retarget(raise_pers2,    &old_fov, &new_fov);
        dg_proj_retarget(pers_no_offset, &old_fov, &new_fov);
        {
            uint64_t identity=0;float angle=0;
            int valid=g_source==SRC_XR && dg_bridge_zoom_now(&identity,&angle);
            double gain=dg_zoom_projection_step(&g_zoom_projection,identity,valid,angle,
                fabs((double)base_pers.m[0][0]),fabs((double)base_pers.m[1][1]),g_stereo,eye_idx);
            dg_zoom_projection_apply(pers,gain);dg_zoom_projection_apply(pers2,gain);
            dg_zoom_projection_apply(raise_pers,gain);dg_zoom_projection_apply(raise_pers2,gain);
            dg_zoom_projection_apply(pers_no_offset,gain);
            if(gain!=g_zoom_last_gain || identity!=g_zoom_last_identity) {
                dg_xr_zoom_changed();
                logf_("  camera zoom projection: manager=%llx angle=%.4f native=%.6f gain=%.6f eye=%d reference=%d\r\n",
                    (unsigned long long)identity,angle,(double)base_pers.m[0][0],gain,eye_idx,
                    g_zoom_projection.reference_x>0);
                g_zoom_last_gain=gain;g_zoom_last_identity=identity;
            }
        }
        g_proj_mine = *pers;
        g_proj2_mine = *pers2;
        g_raise_proj_mine = *raise_pers;
        g_raise_proj2_mine = *raise_pers2;
        g_proj_no_offset_mine = *pers_no_offset;
        InterlockedExchange(&g_have_proj_mine, 1);
    }

    mat_mul(&n_eye_inv, &base_eye_inv, &d);     /* eye_inv' = eye_inv * D    */
    mat_mul(&n_eye, &d_inv, &base_eye);         /* eye'     = D^-1  * eye    */
    if (eye_shift) {
        stereo_eye_shift(&n_eye, &n_eye_inv, eye_idx, eye_half,
                         (int)InterlockedCompareExchange(&g_stereo_eye_x_sign, 0, 0));
        InterlockedIncrement(&g_eye_shift_applied);
        InterlockedExchange(&g_eye_shift_half_x100, (LONG)(eye_half * 100.0));
    }

    /* Re-derive every camera-dependent matrix from its own projection, so all
       consumers in 2.7's list see one coherent camera rather than a mix. The
       five pers* matrices are camera-independent and are left alone. */
    mat_mul(eye_pers,               &n_eye_inv, pers);
    mat_mul(M(O_EYE_PERS2),         &n_eye_inv, M(O_PERS2));
    mat_mul(M(O_RAISE_EYE_PERS),    &n_eye_inv, M(O_RAISE_PERS));
    mat_mul(M(O_RAISE_EYE_PERS2),   &n_eye_inv, M(O_RAISE_PERS2));
    mat_mul(M(O_EYE_PERS_NO_OFFSET),&n_eye_inv, M(O_PERS_NO_OFFSET));

    *eye_inv = n_eye_inv;
    *eye     = n_eye;
    /* Constant-buffer capture (2026-09-18): the matrices exactly as the game
       will read them from here on, so the desk analysis can search the
       vertex-shader constant buffers for them. Read-only side channel. */
    dg_cb_probe_camera(g_stereo ? eye_idx : -1, (const float *)eye_pers->m,
                       (const float *)pers->m, (const float *)n_eye_inv.m,
                       (const float *)n_eye.m);

    /* Keep the final transformed camera->world basis and the final projection
       beside the arm pair.  This is raw camera telemetry for the preceding
       seam's recorder record; it is not a claim about the displayed gun. */
    g_camera_telemetry_eye_world = n_eye;
    g_camera_telemetry_proj = *pers;
    InterlockedExchange(&g_camera_telemetry_valid, 1);
    if (InterlockedCompareExchange(&g_arm_absolute_aim, 0, 0) &&
        g_source != SRC_SYNTHETIC && (probe_frame_flags & 1) &&
        g_xrcfg.yaw_sign == 1.0 && g_xrcfg.pitch_sign == 1.0 &&
        g_xrcfg.roll_sign == 1.0 && g_pose.yaw == 0.0 &&
        g_pose.pitch == 0.0 && g_pose.roll == 0.0 &&
        camera_gate_now(&g_aim_camera.gate)) {
        g_aim_camera.frame = frame;
        g_aim_camera.view = use_raw ? *use_raw : frame.head_raw;
        g_aim_camera.camera = n_eye;
        g_aim_camera.valid = 1;
    }

    /* The optional observation uses THIS camera's exact publication and raw
       view, before later consumers can read a newer XR frame. It observes
       existing hierarchy/model matrices; it does not issue a solve or claim
       that this camera seam is the final D3D draw. */
    if (g_source != SRC_SYNTHETIC && (probe_frame_flags & 1) &&
        dg_aim_capture_enabled() && !dg_aim_capture_native_enabled()) {
        dg_aim_capture_observe((uintptr_t)g_base, (uint64_t)g_arm_pose_stream_id,
                               &n_eye, pers, &frame, probe_frame_flags,
                               use_raw ? use_raw : &frame.head_raw, probe_view_kind);
    }

    g_mine = n_eye_inv;
    InterlockedExchange(&g_have_mine, 1);
    InterlockedIncrement(&g_applied);

    memset(&g_near_pending, 0, sizeof(g_near_pending));
    if (g_source == SRC_XR && have_target && (probe_frame_flags & 3) == 3) {
        LARGE_INTEGER q;
        QueryPerformanceCounter(&q);
        g_near_pending.camera_qpc = (uint64_t)q.QuadPart;
        g_near_pending.camera_world = n_eye;
        g_near_pending.projection = *pers;
        g_near_pending.head_raw = frame.head_raw;
        g_near_pending.grip = frame.right_hand.grip;
        g_near_pending.aim = frame.right_hand.aim;
        g_near_pending.valid = 1;
    }
    if (g_source == SRC_XR && have_target) {
        if (g_stereo)
            handoff_write(1, eye_idx, use_raw, &submit_fov);
        else
            handoff_write(1, DG_EYE_MONO, NULL, &new_fov);
    }
}

/* ------------------------------------------------------ F5 hand pose --- */

/* Which controller pose drives the arm joint. */
enum { ARM_TRACK_OFF = 0, ARM_TRACK_R_GRIP, ARM_TRACK_R_AIM,
       ARM_TRACK_L_GRIP, ARM_TRACK_L_AIM };

static int g_arm_track = ARM_TRACK_OFF;

static int g_arm_qmap[4] = { -1, 2, -3, 4 };
static long g_arm_pose_stale;
static long g_arm_pose_untracked;
static DG_POSE_STATE g_arm_pose_state;
static unsigned long g_arm_pose_pair_id;
/* The arm-root basis is NOT the camera's view basis, and nothing forces the
   two to agree. dg_xr.c's signs describe the camera; these describe the arm,
   and they were measured on 2026-08-19 from the game's own skeleton rather
   than argued from a convention:

     the tracked RIGHT wrist's animated rest position sat at view X = -270 to
     -290 mm across four independent calibration streams, stable to a few
     millimetres, while the arm root was at the character's centre. A right arm
     hangs outboard, so view -X is the character's right and view +X is its
     left. An unnegated controller X therefore sent the hand the wrong way,
     which is exactly what the headset reported: move right, arm goes left.

   Y and Z came out of the same session with the sign they already had - the
   mapped view Y and the resulting world Y moved together, and world +Y is up
   by the measured torso chain - so only X changes here. Z has never been
   directly witnessed and is the reason this is a live knob and not a
   constant. */
static double g_arm_pos_sign[3] = { -1.0, 1.0, 1.0 };

/* Where the player's own shoulder sits relative to their headset, in
   millimetres: outboard toward the tracked arm, below, and ahead. Purely
   anthropometric - the game cannot know it and OpenXR does not report it - and
   the shoulder anchor is the only thing that reads it. Defaults are ordinary
   adult proportions. */
static double g_arm_shoulder_mm[3] = { 180.0, 200.0, 40.0 };

/* Which part of the head's rotation is taken out of the controller offset
   before it becomes an arm target. This is the single decision that says
   whether the character's arm hangs off its BODY or off your GAZE.

   Every build up to 2026-08-19 removed all of it, which made the offset "where
   your hand is relative to where you are looking". Look up 40 degrees without
   moving your hand and that vector swings 40 degrees: the arm follows your
   gaze out of its own body, so a session spent looking around for the arm is a
   session spent chasing it away. Measured that evening - a controller held in
   front of the chest reported at head-frame X = -0.24 m, a quarter of a metre
   to the LEFT of a right hand, and the target landed across the chest at head
   height, which from inside the head reads as an arm behind the camera.
   DG_XR_REL_FRAME_* in dg_xr.h says what each mode costs. */
static int g_arm_frame = DG_XR_REL_FRAME_ROOM;

/* ROOM's forward, frozen from the live head's heading at each arm stream
   start - NOT taken from the seated origin. It was, for exactly one build, and
   the log showed why that cannot work: the origin is captured automatically at
   every session start from wherever the headset happens to face in its first
   tracked half-second, and five consecutive sessions measured +1.2, -39.9,
   -44.1, -73.8 and -47.2 degrees. An arm hung off that heading lands somewhere
   new every launch, which the player experiences as "the arms moved AGAIN".
   The stream start is the one moment that is both player-chosen (B is a stream
   start, and so is a recentre now) and player-facing-forward (you press B
   while aiming), so it is the only heading here that is worth freezing. */
static double        g_arm_frame_q[4] = { 0.0, 0.0, 0.0, 1.0 };
static int           g_arm_frame_have = 0;
static unsigned long g_arm_frame_stream = 0;
static long          g_arm_frame_freezes = 0;
/* The software-turn offset at the moment of the freeze: the frozen heading
   rides the stick's turn from here, so a stick turn reaches the arm exactly
   the way the player turning on their feet does. */
static double        g_arm_frame_omega0 = 0.0;

static void arm_frame_freeze(const DG_XR_RAW_POSE *head)
{
    double q[4];
    q[0] = head->qx; q[1] = head->qy; q[2] = head->qz; q[3] = head->qw;
    dg_xr_quat_yaw_only(q, g_arm_frame_q);
    g_arm_frame_omega0 = input_turn_offset_rad();
    g_arm_frame_have = 1;
    g_arm_frame_freezes++;
}

static const char *arm_frame_name(int frame)
{
    return frame == DG_XR_REL_FRAME_HEAD ? "head"
         : frame == DG_XR_REL_FRAME_ROOM ? "room" : "yaw";
}

/* What the arm is ACTUALLY doing, which is not always what was asked for: room
   without a seated origin is yaw. A silent fallback is the failure mode where
   a knob is set, believed, and quietly not in effect, so the log says which
   one is running rather than which one was requested. */
static const char *arm_frame_running(int frame)
{
    if (frame == DG_XR_REL_FRAME_ROOM && !g_arm_frame_have)
        return "room (nothing frozen yet - running as yaw)";
    return arm_frame_name(frame);
}

/* The player's own shoulder-to-wrist length at full extension, millimetres.
   Supplied rather than measured: see the header of dg_arm_map.h for why one
   instant of elbow bend must not be allowed to set the session's scale. */
static double g_arm_reach_mm = 600.0;

/* The arm's half of the marker, carried as one struct rather than three more
   out-parameters on a function that already has nine. Everything in it changes
   where the hand is put, so everything in it restarts the arm stream. */
typedef struct {
    double sign[3];
    double shoulder_mm[3];
    double reach_mm;
    int    frame;               /* DG_XR_REL_FRAME_* */
    int    anchor;              /* 1 = shoulder anchor, 0 = offset anchor */
    /* Any change re-captures the controller-to-hand offset, exactly the way
       xr_recenter re-captures the seated origin. The value itself means
       nothing; only that it differs from the last one. */
    int    hand_zero;
    int    absolute_aim;
    int    left_arm, twohand;
} ARM_POS_CFG;

static LONG g_left_arm, g_twohand;
static DG_POSE_STATE g_left_pose;
static unsigned long g_left_stream;
static struct {
    unsigned long long seq;
    long long time;
    double distance;
    int coherent;
    DG_POSITION_INPUT position;
    DG_FREE_WRIST_INPUT wrist;
} g_left_sample;
static DG_FREE_WRIST_INPUT g_free_right;

/* The marker's number and the controller button are two routes to the same
   request, so they are ADDED: a change to either changes the total, and the
   total is what restarts the stream. Addition rather than a pair of fields
   because the downstream machinery already treats "different from last time"
   as the whole meaning of this value, and one number keeps it that way.

   The presses only ever increase, so the only way the two can cancel is a
   marker edit that decreases by exactly the number of presses seen in the same
   second. That would cost one missed re-zero and is fixed by pressing again;
   guarding it would mean carrying two tokens to defend against an edit nobody
   would make. */
static int arm_hand_zero_total(int marker_value, unsigned long presses)
{
    return marker_value + (int)presses;
}

/* Which controller carries the re-zero button: the one driving the arm, so
   the gesture is always on the hand you are aiming with. On the left that is
   Y, which is also the recenter's press-and-hold - a short tap re-zeroes, a
   long hold does both. Right-hand tracking, which is the configuration this
   has ever run in, has no such overlap. */
static unsigned int arm_tracked_xr_hand(int track)
{
    return (track == ARM_TRACK_R_GRIP || track == ARM_TRACK_R_AIM)
        ? DG_XR_HAND_RIGHT : DG_XR_HAND_LEFT;
}

/* What the last accepted configuration's re-zero total was, purely so the
   heartbeat can show that a button press was seen. */
static LONG g_arm_hand_zero_now;

static int arm_pos_cfg_same(const ARM_POS_CFG *a, const ARM_POS_CFG *b)
{
    int i;
    if (a->anchor != b->anchor || a->reach_mm != b->reach_mm ||
        a->frame != b->frame || a->hand_zero != b->hand_zero ||
        a->absolute_aim != b->absolute_aim || a->left_arm != b->left_arm ||
        a->twohand != b->twohand) return 0;
    for (i = 0; i < 3; i++)
        if (a->sign[i] != b->sign[i] ||
            a->shoulder_mm[i] != b->shoulder_mm[i]) return 0;
    return 1;
}

/* The arm's defaults, in one place and reachable from a test. They used to
   live inline in read_config, where the only way to check them was to read
   them - and a default is exactly the kind of value that gets edited during a
   sweep and never edited back. Every one of these decides where the hand goes
   for a player who has not touched the marker at all. */
static void arm_pos_cfg_defaults(ARM_POS_CFG *a)
{
    a->sign[0] = -1.0; a->sign[1] = 1.0; a->sign[2] = 1.0;
    a->shoulder_mm[0] = 180.0;
    a->shoulder_mm[1] = 200.0;
    a->shoulder_mm[2] = 40.0;
    a->reach_mm = 600.0;
    /* Room, on the evidence of a headset: with yaw, moving your head while
       aiming moved the laser, because yaw still hands the head's turn to the
       arm. Aiming is the case where any head-driven rotation of the weapon is
       wrong, and room is the only mode where none of it gets through. Yaw
       remains the fallback for the failure it was chosen against - an arm that
       ends up somewhere you cannot turn to find - and a recentre fixes the one
       thing room cannot see, which is you turning your chair. */
    a->frame = DG_XR_REL_FRAME_ROOM;
    a->anchor = 1;
    a->hand_zero = 0;
    a->absolute_aim = 0;
    a->left_arm = a->twohand = 0;
}

static void arm_pos_cfg_apply(const ARM_POS_CFG *a)
{
    int i;
    for (i = 0; i < 3; i++) {
        g_arm_pos_sign[i] = a->sign[i];
        g_arm_shoulder_mm[i] = a->shoulder_mm[i];
    }
    g_arm_reach_mm = a->reach_mm;
    g_arm_frame = a->frame;
    InterlockedExchange(&g_arm_absolute_aim, a->absolute_aim);
    InterlockedExchange(&g_left_arm, a->left_arm);
    InterlockedExchange(&g_twohand, a->twohand);
}

/* vr_arm_hand. A change starts a new stream, because the hand's rest pose is
   captured on a calibration pair and turning this on mid-stream would leave
   the bridge with nothing to measure the controller against. */
static LONG g_arm_hand = 0;
/* Latched from the marker so the camera seam can skip the trigger entirely
   when it is off. With this clear the seam does exactly what it did before the
   trigger existed - which is what makes an A/B run against it worth anything. */
static LONG g_fire_mode = 0;
static LONG g_move_mode = 0;
static LONG g_turn_mode = 0;
/* The software turn's full-stick rate. The right stick no longer writes the
   game's turn bytes - it rotates the tracking space in dg_xr, so view, walk
   frame and arm turn together (see dg_xr.h). The marker's turn gain scales
   this; the marker's move deadzone gates it. Mirrored here because
   move_command runs where the parsed config is not in scope. */
#define DG_TURN_RATE_DEG_S 120.0
static LONG g_turn_gain_mils_live = 1000;
static LONG g_turn_deadzone_mils_live = 150;
/* vr_move_third, for the producer: a TURN_ONLY context (no first person
   held) still publishes the walk when the third-person walk is on; the
   bridge's move_tick keeps every other gate. */
static volatile LONG g_move_third_live;
static LARGE_INTEGER g_arm_pose_last_qpc;
static int g_arm_pose_have_qpc;

static int parse_qmap(const char *s, int *out)
{
    int i;
    for (i = 0; i < 4; i++) {
        int sign = 1;
        while (*s == ' ' || *s == ',') s++;
        if (*s == '-') { sign = -1; s++; }
        else if (*s == '+') s++;
        switch (*s) {
        case 'x': case 'X': out[i] = sign * 1; break;
        case 'y': case 'Y': out[i] = sign * 2; break;
        case 'z': case 'Z': out[i] = sign * 3; break;
        case 'w': case 'W': out[i] = sign * 4; break;
        default: return 0;
        }
        s++;
    }
    return 1;
}

/* Convert a recentered OpenXR controller position to a camera-relative MGS2
   vector. Translation into the arm's matrix space belongs to the bridge: the
   live video proved that the native game camera and the subjective arm root
   can differ by more than a metre even though both call their matrices world.
   Anchoring here produced an unreachable target and a fully extended arm that
   swept circles around its shoulder. */
static void arm_position_to_view(const DG_XR_REL_POSE *p,
                                 const DG_XR_CONFIG *cfg, double out[3])
{
    /* Mirror and scale only. The per-axis signs are applied by the caller
       AFTER the frame correction below, because that correction is a rotation
       in the mirrored MGS2 frame and a sign flip is a further mirror: doing
       them in the other order would rotate about the wrong axes. With every
       sign at +1 - which is every configuration this has ever run in - the two
       orders are identical, so nothing moves. */
    out[0] = p->px * cfg->scale;
    out[1] = p->py * cfg->scale;
    out[2] = -p->pz * cfg->scale;
}

/* Ry * Rx * Rz for row vectors - the same product build_delta composes and
   the same expansion dg_xr.c already writes out for its translation row.
   Carried in double rather than in the game's float MAT because the result is
   a correction that is very often the identity, and float noise around the
   identity is noise added to every frame of hand position for nothing. */
static void ypr_to_rows(double m[3][3], double yaw, double pitch, double roll)
{
    double ca = cos(yaw),   sa = sin(yaw);
    double cb = cos(pitch), sb = sin(pitch);
    double cc = cos(roll),  sc = sin(roll);
    m[0][0] = ca*cc - sa*sb*sc;  m[0][1] = ca*sc + sa*sb*cc;  m[0][2] = -sa*cb;
    m[1][0] = -cb*sc;            m[1][1] = cb*cc;             m[1][2] = sb;
    m[2][0] = sa*cc + ca*sb*sc;  m[2][1] = sa*sc - ca*sb*cc;  m[2][2] = ca*cb;
}

/* The marker's rotation signs are applied to the CAMERA and to nothing else.
   A controller vector taken relative to the live head is therefore expressed
   in the head's own frame, not in the frame the camera is actually using, and
   those two differ by exactly those signs. It cost nothing while every sign
   was +1. With xr_pitch_sign=-1 it costs twice the pitch angle: look down 30
   degrees and the hand is placed 60 degrees away from where you are looking,
   which is how an arm ends up behind the head and cannot be found by looking
   for it.

   C = R_true^T * R_applied, so v_camera = v_head * C, and the same change of
   basis conjugates the hand's orientation. Identity whenever no rotation sign
   is negative, which is the early return. */
static int arm_frame_correction(const DG_XR_RAW_POSE *head,
                                const DG_XR_CONFIG *cfg, double c[3][3])
{
    DG_XR_CONFIG plain, real;
    DG_XR_POSE truth, applied;
    double rt[3][3], ra[3][3];
    int i, j, k;

    /* No negative rotation sign, no disagreement, and - this is the point of
       returning rather than computing an identity - not one arithmetic
       operation added to the path that every configuration so far has used. */
    if (cfg->yaw_sign >= 0.0 && cfg->pitch_sign >= 0.0 &&
        cfg->roll_sign >= 0.0)
        return 0;

    plain = *cfg;
    plain.yaw_sign = plain.pitch_sign = plain.roll_sign = 1.0;
    plain.positional = 0;
    real = *cfg;
    real.positional = 0;
    dg_xr_head_to_pose(head->qx, head->qy, head->qz, head->qw,
                       0.0, 0.0, 0.0, &plain, &truth);
    dg_xr_head_to_pose(head->qx, head->qy, head->qz, head->qw,
                       0.0, 0.0, 0.0, &real, &applied);

    ypr_to_rows(rt, truth.yaw, truth.pitch, truth.roll);
    ypr_to_rows(ra, applied.yaw, applied.pitch, applied.roll);
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) {
        double s = 0.0;
        for (k = 0; k < 3; k++) s += rt[k][i] * ra[k][j];   /* rt^T * ra */
        c[i][j] = s;
    }
    return 1;
}

/* The arm root follows the live subjective camera, so its controller vector
   must follow the live head as well. reference_relative is relative to the
   RECENTRE pose and therefore includes a common head/hand room translation;
   treating it as camera-relative makes leaning look like an arm movement.
   Work from the preserved LOCAL poses instead and remove the current head
   transform before applying the measured OpenXR -> game axis map. */
/* Everything the arm does to a head-relative offset, with no pose sampling in
   it, so the player's shoulder constant can be carried through the identical
   map as the controller. If the two ever went through different code the
   shoulder anchor would be subtracting a vector from a different space, and
   the failure would look like a mysterious constant arm displacement. */
static void arm_rel_to_view(const DG_XR_RAW_POSE *head,
                            const DG_XR_CONFIG *cfg,
                            const DG_XR_REL_POSE *relative, double out[3])
{
    double c[3][3];

    arm_position_to_view(relative, cfg, out);
    if (arm_frame_correction(head, cfg, c)) {
        double v[3];
        int j;
        for (j = 0; j < 3; j++) v[j] = out[j];
        for (j = 0; j < 3; j++)
            out[j] = v[0] * c[0][j] + v[1] * c[1][j] + v[2] * c[2][j];
    }
    /* Signs last, for the reason given in arm_position_to_view. The camera's
       first, then the arm's own: they describe two different disagreements
       between two different pairs of spaces, and the arm's is the outer one
       because the arm root is what the result finally lands in. */
    out[0] *= cfg->x_sign * g_arm_pos_sign[0];
    out[1] *= cfg->y_sign * g_arm_pos_sign[1];
    out[2] *= cfg->z_sign * g_arm_pos_sign[2];
}

/* The frame the controller offset is taken in, written as a POSE so that the
   whole choice is one reference and the transform stays a single subtraction.
   The position is always the live head's - removing it is what keeps a lean
   from reading as an arm movement, and that was never in question. Only the
   rotation differs, and it is the entire argument:

     HEAD  the live head. The offset means "relative to where you are looking",
           which is wrong for a limb that hangs off a torso.
     YAW   the live head's heading. Pitch and roll stop reaching the arm.
     ROOM  the heading FROZEN at the last arm stream start, so nothing the
           head does now reaches the arm at all. Falls back to YAW until the
           first freeze, because an unreferenced frame would put the arm at an
           angle nothing can correct. */
static void arm_frame_reference(const DG_XR_RAW_POSE *head,
                                DG_XR_RAW_POSE *ref)
{
    double q[4];

    *ref = *head;
    if (g_arm_frame == DG_XR_REL_FRAME_HEAD) return;
    if (g_arm_frame == DG_XR_REL_FRAME_ROOM && g_arm_frame_have) {
        /* The legacy relative route advances the frozen heading by the
           stick's software turn since
           the freeze. A right turn of delta must rotate the mapped
           controller offset exactly as the player physically turning
           right by delta would (hand-head swings -delta in XR yaw), which
           takes the reference yaw UP by delta - the same direction the
           recentre reference itself moved. Without this the arm ignores
           stick turns entirely, which is the room-pinning the 2026-08-23
           session measured. */
        /* Absolute aim uses the camera-anchored room stand. Its downstream
           organic body compensation already carries the software turn into
           world space (root * organic = software for a pure yaw). Advancing
           this LOCAL reference too rotates the shoulder-to-hand vector a
           second time relative to the camera: the wrist orbits out of view.
           Keep the captured physical heading and its stream/recenter reset;
           omit only this extra software delta. The legacy route is unchanged. */
        double d = InterlockedCompareExchange(&g_arm_absolute_aim, 0, 0)
            ? 0.0 : input_turn_offset_rad() - g_arm_frame_omega0;
        double s = sin(d * 0.5), c = cos(d * 0.5);
        ref->qx = 0.0;
        ref->qz = 0.0;
        ref->qy = c * g_arm_frame_q[1] + s * g_arm_frame_q[3];
        ref->qw = c * g_arm_frame_q[3] - s * g_arm_frame_q[1];
        return;
    }
    q[0] = head->qx; q[1] = head->qy; q[2] = head->qz; q[3] = head->qw;
    dg_xr_quat_yaw_only(q, q);
    ref->qx = q[0]; ref->qy = q[1]; ref->qz = q[2]; ref->qw = q[3];
}

static void arm_hand_to_view(const DG_XR_RAW_POSE *head,
                             const DG_XR_RAW_POSE *hand,
                             const DG_XR_CONFIG *cfg,
                             DG_XR_REL_POSE *relative, double out[3])
{
    DG_XR_RAW_POSE ref;
    arm_frame_reference(head, &ref);
    dg_xr_pose_relative(&ref, hand, relative);
    arm_rel_to_view(head, cfg, relative, out);
}

/* The player's shoulder as a head-relative offset, mapped exactly like the
   controller. `side` is +1 for a right arm and -1 for a left one, so the
   outboard component follows whichever arm is being tracked. */
static void arm_player_shoulder_view(const DG_XR_RAW_POSE *head,
                                     const DG_XR_CONFIG *cfg, double side,
                                     double out[3])
{
    /* The player's shoulder is rigid to the player's HEADING, not to the
       frozen room frame: place the head-relative constant in the room off
       the live head's yaw, then take it through the SAME frame reference
       as the controller. The old shortcut fed the constant straight to
       the axis map, which silently assumed the head still faced the
       frozen heading - false after any turn, stick or physical. The
       controller offset (the aim ray's TIP) rotated with the turn while
       this constant (the ray's TAIL) stayed behind, so on screen the
       muzzle swept the WRONG way by up to the shoulder offset - reported
       2026-08-23 as "my arm aims the other way when I stick-turn". At
       the frozen heading with no software turn this reduces exactly to
       the old constant, which is why every calibration-time measurement
       stays valid. */
    DG_XR_RAW_POSE sh, ref;
    DG_XR_REL_POSE rel;
    double q[4], yaw_q[4], off[3];
    double a, c, s;

    q[0] = head->qx; q[1] = head->qy; q[2] = head->qz; q[3] = head->qw;
    dg_xr_quat_yaw_only(q, yaw_q);
    off[0] =  side * g_arm_shoulder_mm[0] / 1000.0;
    off[1] = -g_arm_shoulder_mm[1] / 1000.0;
    off[2] = -g_arm_shoulder_mm[2] / 1000.0;   /* OpenXR forward is -Z */
    /* Rotate the offset by the head's heading (yaw-only, so a = the full
       yaw angle; same rotation convention as qrot on a yaw quat). */
    a = 2.0 * atan2(yaw_q[1], yaw_q[3]);
    c = cos(a); s = sin(a);
    memset(&sh, 0, sizeof sh);
    sh.qx = yaw_q[0]; sh.qy = yaw_q[1]; sh.qz = yaw_q[2]; sh.qw = yaw_q[3];
    sh.px = head->px + c * off[0] + s * off[2];
    sh.py = head->py + off[1];
    sh.pz = head->pz + c * off[2] - s * off[0];
    arm_frame_reference(head, &ref);
    dg_xr_pose_relative(&ref, &sh, &rel);
    arm_rel_to_view(head, cfg, &rel, out);
}

/* The controller exactly as OpenXR reported it, relative to the live head,
   in metres and in OpenXR axes: +X right, +Y up, -Z forward. Kept only so the
   heartbeat can print it beside the mapped result. Without both halves the log
   says where the arm was put but never where the hand actually was, and every
   question about the axis mapping turns into an argument instead of a
   reading. */
static double g_arm_raw_rel[3];
static int    g_arm_raw_have;

/* --------------------------------------------------- flight recorder ----
   Every arm defect so far has cost headset runs because the input that
   provoked it was gone by the time anyone could look. The recorder keeps the
   published XR frame - the pipeline's complete input surface - in a ring at
   the camera seam, and dumps it to a file the desk harness can replay
   through this same code (dg_hook_test.exe replay <file> <marker>).

   Default ON, which is only defensible because the append path is provably
   passive: one bounded seqlock read the seam performs anyway, one memcmp
   and one struct copy into preallocated .bss. No allocation, no lock, no
   I/O, no write to anything the game reads. All file I/O happens on the
   worker thread, at dump time. */
static DG_REC_RING   g_rec;
static volatile LONG g_rec_on = 1;
static volatile LONG g_rec_cfg_on = 1;   /* parsed; the worker applies it */
static volatile LONG g_rec_cfg_dump;     /* vr_rec_dump token; change dumps */
/* vr_eye_dump=<n>: a token like vr_rec_dump. A CHANGE writes the next
   DG_EYE_DUMP_FRAMES stereo Presents - the exact back buffer that is about to
   be submitted under an eye label - to logs\dg_eye_<tick>_<k>_<L|R>.raw, quarter
   size (16-byte header: width, height, eye, present; then BGRA rows). Numbers
   could not explain "stereo breaks from some view angles, the desktop mirror
   goes mono there" (camera seam, object eye and pass structure all check
   out), so this looks at the pictures. Read-only: one staging copy per frame. */
#define DG_EYE_DUMP_FRAMES 8
static volatile LONG g_eye_dump_cfg, g_eye_dump_written, g_eye_dump_failed;
static LONG g_eye_dump_seen; static int g_eye_dump_have_seen, g_eye_dump_left;
static DWORD g_eye_dump_tick; static int g_eye_dump_pred;
static volatile LONG g_rec_dumps, g_rec_dump_fail;
static volatile LONG g_rec_snapshots, g_rec_snapshot_fail;
static long          g_rec_last_torn;

/* vr_policy: the hot-reload DLL's path in the DEV TREE, never the game
   directory (dg_policy.c refuses the latter outright). Written by
   read_config and read by the worker's poll - both on the worker thread, so
   a plain buffer, by the same argument as the other worker-only state. */
static char g_policy_path[MAX_PATH];

#ifdef DG_HOOK_TEST
/* Replay must feed the pose blender the dt the session really had; live QPC
   would make "bit-identical" unachievable by construction. Negative means
   live. Set only by the desk harness - the shipping DLL never touches it. */
static double g_test_dt_override = -1.0;
#endif

static double arm_pose_dt(void)
{
    LARGE_INTEGER now;
    double dt = 1.0 / 90.0;
#ifdef DG_HOOK_TEST
    if (g_test_dt_override >= 0.0) return g_test_dt_override;
#endif
    QueryPerformanceCounter(&now);
    if (g_arm_pose_have_qpc && g_qpf.QuadPart > 0)
        dt = (double)(now.QuadPart - g_arm_pose_last_qpc.QuadPart) /
             (double)g_qpf.QuadPart;
    g_arm_pose_last_qpc = now;
    g_arm_pose_have_qpc = 1;
    return dt;
}

/* The ring becomes a file: <log dir>\dg_rec_YYYYMMDD-HHMMSS.dgrec. Worker
   thread only. The per-slot stamps inside dg_rec_write make the one racy
   case - a mid-session dump while the VEH is appending - lose at most the
   slots being overwritten at that instant, counted as torn. */
/* The log directory with its trailing backslash; returns its length. */
static size_t rec_log_dir(char *path, size_t cap)
{
    char *cut;
    strcpy_s(path, cap, g_logpath);
    cut = strrchr(path, '\\');
    if (cut) cut[1] = 0; else path[0] = 0;
    return strlen(path);
}

static void aim_observation_dump(void)
{
    char path[MAX_PATH];
    SYSTEMTIME st;
    size_t dir = rec_log_dir(path, sizeof path);
    long count;
    GetLocalTime(&st);
    _snprintf_s(path + dir, sizeof path - dir, _TRUNCATE,
                "dg_aim_%04u%02u%02u-%02u%02u%02u-%llu.jsonl",
                st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond,
                (unsigned long long)GetTickCount64());
    count = dg_aim_capture_dump(path);
    if (count < 0) logf_("  aim observation: FAILED dump -> %s (I/O or busy capture)\r\n", path);
    else if (count) logf_("  aim observation: %ld rows -> %s (camera seam; final draw unproven)\r\n",
                         count, path);
}

static int rec_dump(const char *why)
{
    char path[MAX_PATH];
    SYSTEMTIME st;
    FILE *f = NULL;
    long written, torn = 0;
    size_t dir;

    aim_observation_dump();

    if (!g_rec.appended) {
        logf_("  rec: nothing to dump (%s) - no XR frames were captured\r\n",
              why);
        return 0;
    }
    dir = rec_log_dir(path, sizeof path);
    GetLocalTime(&st);
    _snprintf_s(path + dir, sizeof path - dir, _TRUNCATE,
                "dg_rec_%04u%02u%02u-%02u%02u%02u.dgrec",
                st.wYear, st.wMonth, st.wDay,
                st.wHour, st.wMinute, st.wSecond);
    if (fopen_s(&f, path, "wb") != 0 || !f) {
        InterlockedIncrement(&g_rec_dump_fail);
        logf_("  rec: dump FAILED to open %s (%s)\r\n", path, why);
        return 0;
    }
    written = dg_rec_write(&g_rec, (long long)g_qpf.QuadPart, f, &torn);
    fclose(f);
    if (written < 0) {
        InterlockedIncrement(&g_rec_dump_fail);
        logf_("  rec: dump FAILED writing %s (%s)\r\n", path, why);
        return 0;
    }
    InterlockedIncrement(&g_rec_dumps);
    g_rec_last_torn = torn;
    logf_("  rec: dumped %ld frames to %s (%s; seen %ld, dup %ld,"
          " torn %ld)\r\n",
          written, path, why, g_rec.seen, g_rec.dup, torn);
    return 1;
}

#define DG_REC_SNAPSHOT_S 60
#define DG_REC_LIVE_NAME "dg_rec_live.dgrec"

/* dg_rec_write into <path>.tmp, replacing <path> only after a complete
   successful write. A kill mid-write can then only ever cost the newest
   snapshot, never the previous one - the property the live snapshot's whole
   existence stands on, because it is written FOR the sessions that die
   without running any exit path. On failure <path> is untouched and the
   .tmp does not survive. */
static long rec_write_atomic(const char *path, long *torn)
{
    char tmp[MAX_PATH];
    FILE *f = NULL;
    long written;
    size_t n = strlen(path);

    if (n == 0 || n + 5 >= sizeof tmp) return -1;
    memcpy(tmp, path, n);
    memcpy(tmp + n, ".tmp", 5);
    if (fopen_s(&f, tmp, "wb") != 0 || !f) return -1;
    written = dg_rec_write(&g_rec, (long long)g_qpf.QuadPart, f, torn);
    if (fclose(f) != 0) written = -1;
    if (written >= 0 && !MoveFileExA(tmp, path, MOVEFILE_REPLACE_EXISTING))
        written = -1;
    if (written < 0) remove(tmp);
    return written;
}

/* When is a live snapshot due? Every DG_REC_SNAPSHOT_S seconds, only while
   the recorder is on, and only when the ring moved since the last one - an
   idle headset must not rewrite the same file forever. Refusals advance
   nothing. Pure, so the desk holds the truth table. */
static int rec_snapshot_due(int on, long appended, int elapsed,
                            long *last_appended, int *last_elapsed)
{
    if (!on || appended == *last_appended) return 0;
    if (elapsed - *last_elapsed < DG_REC_SNAPSHOT_S) return 0;
    *last_appended = appended;
    *last_elapsed = elapsed;
    return 1;
}

/* The periodic half of the recorder's crash story. The end-of-session dump
   only runs when the worker loop is allowed to finish; the session that
   found this gap ended with a killed process - heartbeats still counting,
   5760 frames in the ring, not a byte on disk. Once a minute the ring goes
   to one rolling file, atomically, so a dead process costs at most the last
   minute of input. */
static void rec_snapshot(void)
{
    char path[MAX_PATH];
    long written, torn = 0;
    size_t dir = rec_log_dir(path, sizeof path);

    _snprintf_s(path + dir, sizeof path - dir, _TRUNCATE, DG_REC_LIVE_NAME);
    written = rec_write_atomic(path, &torn);
    if (written < 0) {
        InterlockedIncrement(&g_rec_snapshot_fail);
        logf_("  rec: live snapshot FAILED (%s)\r\n", path);
        return;
    }
    InterlockedIncrement(&g_rec_snapshots);
    logf_("  rec: live snapshot %ld frames (torn %ld)\r\n", written, torn);
}

/* Produces one policy-owned target whenever controller driving is configured.
   Invalid or stale input is still passed through dg_pose_step so it blends to
   release; only ARM_TRACK_OFF returns 0 and permits the fixed bend probe. */
static DG_AIM_REPLAY_STATE g_absolute_state;
static DG_REC_AIM_REPLAY g_absolute_record;
static int g_absolute_reset;
static unsigned long g_absolute_stream;
static DG_AIM_SELECTION g_absolute_selection;
static unsigned long long g_absolute_camera;
/* Equipment resets the right native pose, but is not a new left neutral.
   Physical refreezes and actual camera/arm replacement remain hard epochs. */
static struct {
    unsigned long epoch;
    long freeze;
    ULONGLONG arm, camera;
    int have;
} g_left_calibration;
static unsigned long arm_left_calibration_stream(void)
{
    if (!g_aim_camera.valid || !g_arm_frame_have) {
        g_left_calibration.have=0;
        return 0;
    }
    if (!g_left_calibration.have ||
        g_left_calibration.freeze!=g_arm_frame_freezes ||
        g_left_calibration.arm!=g_aim_camera.gate.arm_body ||
        g_left_calibration.camera!=g_aim_camera.gate.camera) {
        if (!++g_left_calibration.epoch) ++g_left_calibration.epoch;
        g_left_calibration.have=1;
        g_left_calibration.freeze=g_arm_frame_freezes;
        g_left_calibration.arm=g_aim_camera.gate.arm_body;
        g_left_calibration.camera=g_aim_camera.gate.camera;
    }
    return g_left_calibration.epoch;
}
#ifdef DG_HOOK_TEST
static int g_test_aim_selection = -1;
#endif

static int arm_absolute_prepare(DG_AIM_SELECTION *selection)
{
    if (!g_aim_camera.valid ||
        (g_arm_track != ARM_TRACK_R_GRIP && g_arm_track != ARM_TRACK_R_AIM)) return 0;
#ifdef DG_HOOK_TEST
    if (g_test_aim_selection >= 0) {
        if (!dg_weapon_hand_aim(g_test_aim_selection)) return 0;
        memset(selection, 0, sizeof *selection);
        selection->arm = g_aim_camera.gate.arm_body;
        selection->subobject = selection->subobjs = selection->hand = selection->model = 100;
        selection->weapon_id = (uint64_t)g_test_aim_selection;
    } else
#endif
    if (!dg_aim_capture_hand_selection((uintptr_t)g_base,
                                     g_aim_camera.gate.arm_body, selection)) return 0;
    if(selection->weapon_id==13 && !dg_bridge_blade_enabled())return 0;
    if(selection->weapon_id==7 && !dg_bridge_stinger_enabled())return 0;
    if (g_absolute_stream != g_arm_pose_stream_id ||
        g_absolute_camera != g_aim_camera.gate.camera ||
        memcmp(selection, &g_absolute_selection, sizeof *selection)) {
        /* Weapon/camera replacement cannot inherit the old pose latch. */
        if (g_absolute_stream == g_arm_pose_stream_id) {
            /* Rig identity is not a new physical forward. Rebuild the pose
               latch and native calibration, retaining the user's room stand.
               An explicit recenter/config stream has already advanced before
               this branch and still captures a fresh physical reference. */
            int keep_room = g_arm_frame_have &&
                g_arm_frame_stream == g_arm_pose_stream_id;
            g_arm_pose_stream_id++;
            if (keep_room) g_arm_frame_stream = g_arm_pose_stream_id;
        }
        g_absolute_stream = g_arm_pose_stream_id;
        g_absolute_selection = *selection;
        g_absolute_camera = g_aim_camera.gate.camera;
        g_absolute_reset = 1;
        dg_pose_init(&g_arm_pose_state, NULL);
    }
    return 1;
}

/* The one absolute input->target implementation used live AND in replay.
   Recorded outputs never enter this function. No game pointers are read. */
static void absolute_aim_step(DG_AIM_REPLAY_STATE *state,
                              const DG_AIM_REPLAY_IN *input,
                              DG_AIM_REPLAY_OUT *result)
{
    DG_POSE_IN pose;
    memset(result, 0, sizeof *result);
    memset(&pose, 0, sizeof pose);
    pose.quat[3] = 1.0;
    if (input->reset) {
        memset(state, 0, sizeof *state);
        dg_pose_init(&state->pose, NULL);
    }
    pose.frame = input->frame; pose.eye = (int)input->eye;
    pose.dt = input->dt; pose.age_ms = input->age_ms;
    pose.valid = input->source_valid && input->hand_tag == DG_XR_HAND_RIGHT &&
        input->kind_tag == DG_XR_POSE_AIM && (input->pose_flags & 14u) == 14u &&
        input->age_ms <= 100 && input->sample_seq && input->sample_time > 0 &&
        input->sample_seq >= state->sample_seq && input->sample_time >= state->sample_time &&
        dg_aim_target_build(input->camera, input->view, input->aim, pose.quat);
    if(pose.valid && (input->pose_flags & DG_AIM_REPLAY_BLADE))
        pose.valid=dg_aim_target_blade(pose.quat);
    result->input_valid = pose.valid;
    if (!pose.valid) {
        dg_pose_init(&state->pose, NULL);
        return;
    }
    dg_pose_step(&state->pose, &pose, &result->pose);
    if (!(result->pose.flags & DG_POSE_F_LATCHED)) {
        state->sample_seq = input->sample_seq;
        state->sample_time = input->sample_time;
    }
    result->sample_seq = state->sample_seq;
    result->sample_time = state->sample_time;
    result->write = result->pose.write &&
        ((result->pose.flags ^ input->main_pose_flags) & DG_POSE_F_LATCHED) == 0;
}

typedef struct {
    long entry;
    ULONGLONG arm;
    int mode, have, settling;
    DWORD since;
} DG_AUTO_RECENTER;
static int auto_recenter_step(DG_AUTO_RECENTER *s,int mode,long entry,
                              ULONGLONG arm,DWORD now)
{
    if(!mode || !arm) { s->settling=0; return 0; }
    if(s->mode!=mode || s->entry!=entry || s->arm!=arm) {
        s->mode=mode; s->entry=entry; s->arm=arm;
        s->have=0; s->settling=0;
    }
    if(s->have)return 0;
    /* Calibrate the settled gameplay view, after the native transition.
       Repeated camera callbacks do not substitute for elapsed game time. */
    if(!s->settling) { s->since=now; s->settling=1; return 0; }
    if((DWORD)(now-s->since)<100u)return 0;
    s->have=1;
    return 1;
}
static DG_AUTO_RECENTER g_auto_recenter;
static volatile LONG g_auto_recenter_session;

static void auto_recenter_present(int allowed)
{
    static LONG seen_session;
    DG_XR_FRAME frame;
    LONG session=InterlockedCompareExchange(&g_auto_recenter_session,0,0);
    long generation=0;
    unsigned long long identity=0;
    int mode=0;
    /* Session setup runs on the worker; keep the state reset on Present's
       owning thread. The same actor/generation can recur after VR restart. */
    if(session!=seen_session) {
        memset(&g_auto_recenter,0,sizeof g_auto_recenter);
        seen_session=session;
    }
    memset(&frame,0,sizeof frame);
    if(allowed && g_source==SRC_XR && (input_get_frame(&frame)&1)) {
        mode=dg_bridge_view_calibration_now(&generation,&identity);
        /* Both unarmed arms need a tracked calibration pose. This check is
           independent of weapon selection and the absolute-aim solver. */
        if(mode==1 && (!frame.left_hand.grip.active || !frame.left_hand.grip.tracked ||
            !frame.left_hand.grip.position_valid || !frame.left_hand.grip.orientation_valid ||
            !frame.left_hand.grip.sample_seq || frame.left_hand.grip.xr_time<=0 ||
            frame.left_hand.grip.pose_age_ms>100 || !frame.right_hand.grip.active ||
            !frame.right_hand.grip.tracked || !frame.right_hand.grip.position_valid ||
            !frame.right_hand.grip.orientation_valid || !frame.right_hand.grip.sample_seq ||
            frame.right_hand.grip.xr_time<=0 || frame.right_hand.grip.pose_age_ms>100))
            mode=0;
    }
    if(auto_recenter_step(&g_auto_recenter,mode,generation,identity,GetTickCount())) {
        input_recenter();
        logf_("  calibration: automatic %s recenter requested, transition %ld identity %llX\r\n",
              mode==1 ? "first-person arms":"third-person view",generation,identity);
    }
}

static DG_FREE_WRIST_INPUT free_wrist_input(const DG_XR_HAND_POSE *p,
                                             const DG_XR_CONFIG *cfg,int enabled)
{
    DG_FREE_WRIST_INPUT out={0};DG_POSITION_INPUT f={0};double q[4],g[4];int i;
    out.enabled=enabled;
    if(!enabled || !g_aim_camera.valid || !p || !p->active || !p->tracked ||
       !p->position_valid || !p->orientation_valid || !p->sample_seq ||
       p->xr_time<=0 || p->pose_age_ms>100)return out;
    f.valid=1;f.units=cfg->scale;
    for(i=0;i<16;i++) f.camera[i]=g_aim_camera.camera.m[i/4][i%4];
    f.view[0]=g_aim_camera.view.qx;f.view[1]=g_aim_camera.view.qy;
    f.view[2]=g_aim_camera.view.qz;f.view[3]=g_aim_camera.view.qw;
    f.view[4]=g_aim_camera.view.px;f.view[5]=g_aim_camera.view.py;f.view[6]=g_aim_camera.view.pz;
    g[0]=p->raw_local.qx;g[1]=p->raw_local.qy;g[2]=p->raw_local.qz;g[3]=p->raw_local.qw;
    if(!dg_position_frame(&f,q) || !dg_ik_quat_normalize(g))return out;
    dg_ik_quat_mul(q,g,out.world);out.valid=dg_ik_quat_normalize(out.world);return out;
}

static void m9_sample_from_frame(const DG_XR_FRAME *,int,DG_M9_SAMPLE *);
static int arm_pose_target(DG_BRIDGE_ARM_TARGET *target)
{
    DG_XR_FRAME f;
    const DG_XR_HAND *h;
    const DG_XR_HAND_POSE *p = NULL;
    DG_POSE_IN in;
    DG_POSE_OUT out;
    DG_XR_CONFIG cfg;
    DG_XR_REL_POSE head_relative;
    double src[4];
    /* Zero until a frame with a head in it arrives. The bridge only reads it
       under the shoulder anchor, and a zero there is a target at the
       character's shoulder rather than a wild one. */
    double player_shoulder[3] = { 0.0, 0.0, 0.0 };
    double player_reach = 0.0;
    double hand_yaw_rad = 0.0;
    int hand_yaw_valid = 0;
    int frame_flags, absolute, aim_ok = 0;
    DG_AIM_SELECTION selection;
    int i;

    memset(&g_absolute_record, 0, sizeof g_absolute_record);
    g_absolute_reset = 0;
    if (g_arm_track == ARM_TRACK_OFF) return 0;
    memset(target, 0, sizeof *target);
    memset(&f, 0, sizeof f);
    memset(&selection, 0, sizeof selection);
    absolute = InterlockedCompareExchange(&g_arm_absolute_aim, 0, 0) != 0;
    memset(&in, 0, sizeof in);
    in.quat[3] = 1.0;
    in.frame = (unsigned long)InterlockedCompareExchange(&g_present_frame, 0, 0);
    in.eye = InterlockedCompareExchange(&g_stereo, 0, 0)
        ? ((InterlockedCompareExchange(&g_current_eye, 0, 0) == DG_EYE_RIGHT)
            ? DG_POSE_EYE_RIGHT : DG_POSE_EYE_LEFT)
        : DG_POSE_EYE_MONO;
    in.dt = arm_pose_dt();
    cfg = g_xrcfg;

    /* A recentre is a whole-body statement - "THIS is forward" - so it
       restarts the arm stream: new calibration, new grip rest, new frozen
       heading. Without this the camera and the arm answer to two different
       forwards and only one of them has a button. */
    {
        static long s_rc_last;
        long rc = input_recenter_count();
        if (rc != s_rc_last) {
            s_rc_last = rc;
            g_arm_pose_stream_id++;
            /* None has no absolute weapon selection to reset this latch.
               Recenter must reset its pose history just as it does armed. */
            dg_pose_init(&g_arm_pose_state,NULL);
        }
    }

    if (absolute) {
        aim_ok = arm_absolute_prepare(&selection);
        frame_flags = g_aim_camera.valid ? 1 : 0;
        if (frame_flags) f = g_aim_camera.frame;
    } else frame_flags = input_get_frame(&f);
    if (frame_flags & 1) {
        /* The freeze rides the stream id rather than any event handler, so
           EVERY route into a new stream - marker edit, B press, recentre -
           lands here, on the first frame that has a head to freeze. */
        if (!g_arm_frame_have || g_arm_frame_stream != g_arm_pose_stream_id) {
            arm_frame_freeze(&f.head_raw);
            g_arm_frame_stream = g_arm_pose_stream_id;
        }
        h = (g_arm_track == ARM_TRACK_R_GRIP || g_arm_track == ARM_TRACK_R_AIM)
                ? &f.right_hand : &f.left_hand;
        /* Outside the tracked-pose test on purpose: this is a constant offset
           from the head, so it is available on every frame that has a head at
           all, including the ones where the controller is not. Deliberately
           not run through dg_pose_step either - a constant has no staleness,
           no blend and no stereo pair to latch. */
        arm_player_shoulder_view(&f.head_raw, &cfg,
                                 (g_arm_track == ARM_TRACK_R_GRIP ||
                                  g_arm_track == ARM_TRACK_R_AIM) ? 1.0 : -1.0,
                                 player_shoulder);
        /* A length, so it takes the metres-to-game-units scale and none of the
           axis mapping. Written the long way round rather than as a bare
           millimetre count so that xr_scale stays the single place the unit is
           decided. */
        player_reach = g_arm_reach_mm / 1000.0 * cfg.scale;
        p = (g_arm_track == ARM_TRACK_R_AIM || g_arm_track == ARM_TRACK_L_AIM)
                ? &h->aim : &h->grip;
        in.age_ms = (double)p->pose_age_ms;
        /* The gun's heading for the facing-gap publisher (vr_turn_follow_src
           =hand): the AIM pose always, whichever pose drives the arm - its
           -Z is the ray the player points, so its yaw is well defined while
           aiming, where a grip handle's can stand near vertical. Through the
           same reference-relative quaternion (turned by the stick like the
           head's) and the same dg_xr_head_to_pose mapping as head_yaw_rad,
           so the two facings share one frame and one sign convention. */
        if (h->aim.active && h->aim.tracked && h->aim.orientation_valid &&
            h->aim.pose_age_ms <= 100) {
            DG_XR_POSE hp;
            dg_xr_head_to_pose(h->aim.reference_relative.qx,
                               h->aim.reference_relative.qy,
                               h->aim.reference_relative.qz,
                               h->aim.reference_relative.qw,
                               0.0, 0.0, 0.0, &cfg, &hp);
            hand_yaw_rad = hp.yaw;
            hand_yaw_valid = 1;
        }
        if (p->active && p->tracked && p->position_valid &&
            p->orientation_valid) {
            arm_hand_to_view(&f.head_raw, &p->raw_local, &cfg,
                             &head_relative, in.pos);
            g_arm_raw_rel[0] = head_relative.px;
            g_arm_raw_rel[1] = head_relative.py;
            g_arm_raw_rel[2] = head_relative.pz;
            g_arm_raw_have = 1;
            src[0] = head_relative.qx;
            src[1] = head_relative.qy;
            src[2] = head_relative.qz;
            src[3] = head_relative.qw;
            for (i = 0; i < 4; i++) {
                int m = g_arm_qmap[i];
                double v = src[(m < 0 ? -m : m) - 1];
                in.quat[i] = (m < 0) ? -v : v;
            }
            in.valid = 1;
        } else {
            InterlockedIncrement(&g_arm_pose_untracked);
        }
        if (p->pose_age_ms > 100)
            InterlockedIncrement(&g_arm_pose_stale);
    } else {
        InterlockedIncrement(&g_arm_pose_untracked);
    }

    dg_pose_step(&g_arm_pose_state, &in, &out);
    if (!(out.flags & DG_POSE_F_LATCHED)) g_arm_pose_pair_id++;
    target->write = out.write;
    target->hand_write = InterlockedCompareExchange(&g_arm_hand, 0, 0) ? 1 : 0;
    target->pair_id = g_arm_pose_pair_id;
    target->stream_id = g_arm_pose_stream_id;
    target->left_stream_id = arm_left_calibration_stream();
    for (i = 0; i < 3; i++) target->wrist_view[i] = out.pos[i];
    for (i = 0; i < 3; i++)
        target->player_shoulder_view[i] = player_shoulder[i];
    target->player_reach_view = player_reach;
    /* The mapped head yaw, for the facing-gap publisher: it carries the
       software turn (the stick) and the player's physical turn alike, and
       nothing the body or the controller does can move it - the property
       both previous gap publishers lacked. */
    if (g_arm_frame_have && g_camera_yaw_anchor_have &&
        g_aim_camera.valid && (frame_flags&1)) {
        MAT base; double basis[3][3]; int r,k;
        rot_y(&base,g_camera_yaw_anchor_heading);
        for(r=0;r<3;r++)for(k=0;k<3;k++)basis[r][k]=base.m[r][k];
        if(dg_ik_basis_quat(basis,target->calibration_camera))
            target->calibration_id=(unsigned long)g_arm_frame_freezes+1;
    }
    target->head_yaw_rad = f.head.yaw;
    target->head_yaw_valid = (frame_flags & 1) != 0;
    target->hand_yaw_rad = hand_yaw_rad;
    target->hand_yaw_valid = hand_yaw_valid;
    /* The stick's software turn alone (vr_turn_follow_src=stick): the mapped
       head yaw carries the turned reference AND the physical head; the
       head's LOCAL pose through the same mapping carries only the physical
       head. Their difference is the reference's yaw - the stick's turn plus
       the recentre constant the calibration zero absorbs - with the sign
       convention of the head term, because it IS the head term minus its
       physical part. */
    {
        DG_XR_POSE lp;
        double d;
        dg_xr_head_to_pose(f.head_raw.qx, f.head_raw.qy, f.head_raw.qz,
                           f.head_raw.qw, 0.0, 0.0, 0.0, &cfg, &lp);
        d = f.head.yaw - lp.yaw;
        while (d > 3.14159265358979323846) d -= 2.0 * 3.14159265358979323846;
        while (d < -3.14159265358979323846) d += 2.0 * 3.14159265358979323846;
        target->stick_yaw_rad = d;
        target->stick_yaw_valid = (frame_flags & 1) != 0;
    }
    for (i = 0; i < 4; i++) target->hand_quat[i] = out.quat[i];
    target->weight = out.weight;
    target->pose_flags = out.flags;
    {
        DG_FREE_WRIST_INPUT fresh=free_wrist_input(&f.right_hand.grip,&cfg,absolute);
        if(!(out.flags&DG_POSE_F_LATCHED))g_free_right=fresh;
        target->free_right=g_free_right;
        if(!fresh.valid)target->free_right.valid=0;
    }
    target->unarmed_hand_write = target->hand_write;
    target->observed_right_trigger = f.right_hand.trigger_value;
    target->observed_right_stick_x = f.right_hand.thumbstick_x;
    target->absolute_aim = absolute;
    if (absolute) {
        DG_AIM_REPLAY_IN *ai = &g_absolute_record.input;
        DG_AIM_REPLAY_OUT *ao = &g_absolute_record.observed;
        const DG_XR_HAND_POSE *aim = &f.right_hand.aim;
        int j;
        ai->frame = in.frame; ai->eye = (unsigned int)in.eye; ai->dt = in.dt;
        ai->age_ms = aim->pose_age_ms;
        for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
            ai->camera[i][j] = g_aim_camera.camera.m[i][j];
        ai->view[0] = g_aim_camera.view.qx; ai->view[1] = g_aim_camera.view.qy;
        ai->view[2] = g_aim_camera.view.qz; ai->view[3] = g_aim_camera.view.qw;
        ai->view[4] = g_aim_camera.view.px; ai->view[5] = g_aim_camera.view.py;
        ai->view[6] = g_aim_camera.view.pz;
        ai->aim[0] = aim->raw_local.qx; ai->aim[1] = aim->raw_local.qy;
        ai->aim[2] = aim->raw_local.qz; ai->aim[3] = aim->raw_local.qw;
        ai->aim[4] = aim->raw_local.px; ai->aim[5] = aim->raw_local.py;
        ai->aim[6] = aim->raw_local.pz;
        ai->hand_tag = aim->hand; ai->kind_tag = aim->kind;
        ai->pose_flags = (aim->position_valid ? 1u : 0u) | (aim->orientation_valid ? 2u : 0u) |
                         (aim->active ? 4u : 0u) | (aim->tracked ? 8u : 0u);
        if(selection.weapon_id==13)ai->pose_flags|=DG_AIM_REPLAY_BLADE;
        ai->source_valid = aim_ok; ai->reset = g_absolute_reset;
        ai->sample_seq = aim->sample_seq; ai->sample_time = aim->xr_time;
        ai->camera_id = g_aim_camera.gate.camera; ai->stream = target->stream_id;
        ai->arm = selection.arm; ai->subobject = selection.subobject;
        ai->subobjs = selection.subobjs; ai->hand = selection.hand; ai->model = selection.model;
        ai->main_pose_flags = out.flags;
        g_absolute_record.before = g_absolute_state;
        absolute_aim_step(&g_absolute_state, ai, ao);
        g_absolute_record.present = 1;
        target->aim_write = ao->write;
        if (!target->aim_write) target->hand_write = 0;
        memcpy(target->aim_world, ao->pose.quat, sizeof target->aim_world);
        target->aim_weight = ao->pose.weight;
        target->aim_pair_id = target->pair_id;
        target->aim_stream_id = target->stream_id;
        target->aim_sample_seq = ao->sample_seq;
        target->aim_sample_time = ao->sample_time;
        target->aim_arm = selection.arm; target->aim_subobject = selection.subobject;
        target->aim_subobjs = selection.subobjs; target->aim_hand = selection.hand;
        target->aim_model = selection.model;
        target->aim_weapon_id = selection.weapon_id;
        /* Exact camera/raw eye publication used by absolute orientation.
           Position uses the selected grip/aim pose, not a head-locked offset.
           Eye translation is unscaled; only controller displacement takes
           the character/player arm-length ratio in the bridge. */
        target->position.enabled=1;
        /* Raw grip position is also valid for None. The bridge separately
           checks native weapon ownership before using it for armed AIM. */
        target->position.valid=g_aim_camera.valid && out.write && cfg.positional &&
            cfg.x_sign==1 && cfg.y_sign==1 && cfg.z_sign==1 && p &&
            p->active && p->tracked && p->position_valid &&
            p->pose_age_ms<=100 && p->sample_seq && p->xr_time>0;
        target->position.units=cfg.scale;
        for(i=0;i<16;i++) target->position.camera[i]=
            g_aim_camera.camera.m[i/4][i%4];
        for(i=0;i<7;i++) target->position.view[i]=ai->view[i];
        if(p) {
            target->position.grip[0]=p->raw_local.px;
            target->position.grip[1]=p->raw_local.py;
            target->position.grip[2]=p->raw_local.pz;
        }
    }
    target->left_enabled = g_left_arm &&
        (g_arm_track == ARM_TRACK_R_GRIP || g_arm_track == ARM_TRACK_R_AIM);
    target->twohand_enabled = g_twohand &&
        dg_weapon_pistol_support((int)target->aim_weapon_id);
    if (target->left_enabled) {
        DG_POSE_IN li;
        DG_POSE_OUT lo;
        DG_XR_REL_POSE lr;
        const DG_XR_HAND_POSE *lp = &f.left_hand.grip;
        const DG_XR_HAND_POSE *rp = &f.right_hand.grip;
        double dx, dy, dz;
        int coherent_now;
        unsigned long left_stream=target->left_stream_id ? target->left_stream_id : target->stream_id;
        if (g_left_stream != left_stream) {
            dg_pose_init(&g_left_pose, NULL);
            memset(&g_left_sample,0,sizeof g_left_sample);
            g_left_stream = left_stream;
        }
        memset(&li, 0, sizeof li);
        li.quat[3] = 1;
        li.frame = in.frame; li.eye = in.eye; li.dt = in.dt;
        li.age_ms = lp->pose_age_ms;
        li.valid = (frame_flags & 1) && lp->active && lp->tracked &&
            lp->position_valid && lp->orientation_valid && lp->pose_age_ms <= 100 &&
            lp->hand == DG_XR_HAND_LEFT && lp->kind == DG_XR_POSE_GRIP &&
            lp->sample_seq && lp->xr_time > 0;
        if (li.valid) {
            arm_hand_to_view(&f.head_raw, &lp->raw_local, &cfg, &lr, li.pos);
            arm_player_shoulder_view(&f.head_raw, &cfg, -1,
                                     target->left_shoulder_view);
        }
        dg_pose_step(&g_left_pose, &li, &lo);
        /* Loss revokes ownership before any stereo/cache replay. */
        target->left_valid = li.valid && lo.write;
        memcpy(target->left_wrist_view, lo.pos, sizeof lo.pos);
        target->left_weight = lo.weight;
        coherent_now = li.valid && rp->active && rp->tracked &&
            rp->position_valid && rp->orientation_valid && rp->pose_age_ms <= 100 &&
            lp->sample_seq && lp->sample_seq == rp->sample_seq &&
            lp->xr_time > 0 && lp->xr_time == rp->xr_time &&
            lp->sample_seq == f.right_hand.aim.sample_seq &&
            lp->xr_time == f.right_hand.aim.xr_time;
        dx = lp->raw_local.px - rp->raw_local.px;
        dy = lp->raw_local.py - rp->raw_local.py;
        dz = lp->raw_local.pz - rp->raw_local.pz;
        /* Position, sample identity and proximity describe one observation.
           The closing eye must not pair yesterday's latched AIM with today's
           grip timestamp: that falsely revoked the latch every other eye. */
        if (!(lo.flags & DG_POSE_F_LATCHED)) {
            g_left_sample.seq=lp->sample_seq;
            g_left_sample.time=lp->xr_time;
            g_left_sample.distance=sqrt(dx*dx + dy*dy + dz*dz);
            g_left_sample.coherent=coherent_now;
            g_left_sample.wrist=free_wrist_input(lp,&cfg,absolute && target->unarmed_hand_write);
            /* Keep camera, eye and raw grip from one observation together.
               Left tracking does not depend on the right pistol aim gate. */
            memset(&g_left_sample.position,0,sizeof g_left_sample.position);
            g_left_sample.position.enabled=absolute;
            g_left_sample.position.valid=absolute && li.valid && g_aim_camera.valid &&
                cfg.positional && cfg.x_sign==1 && cfg.y_sign==1 && cfg.z_sign==1;
            g_left_sample.position.units=cfg.scale;
            for(i=0;i<16;i++) g_left_sample.position.camera[i]=
                g_aim_camera.camera.m[i/4][i%4];
            g_left_sample.position.view[0]=g_aim_camera.view.qx;
            g_left_sample.position.view[1]=g_aim_camera.view.qy;
            g_left_sample.position.view[2]=g_aim_camera.view.qz;
            g_left_sample.position.view[3]=g_aim_camera.view.qw;
            g_left_sample.position.view[4]=g_aim_camera.view.px;
            g_left_sample.position.view[5]=g_aim_camera.view.py;
            g_left_sample.position.view[6]=g_aim_camera.view.pz;
            g_left_sample.position.grip[0]=lp->raw_local.px;
            g_left_sample.position.grip[1]=lp->raw_local.py;
            g_left_sample.position.grip[2]=lp->raw_local.pz;
        }
        target->free_left=g_left_sample.wrist;
        if(!li.valid || !g_aim_camera.valid || !target->unarmed_hand_write)
            target->free_left.valid=0;
        target->left_position=g_left_sample.position;
        if(!li.valid || !g_aim_camera.valid) target->left_position.valid=0;
        target->left_sample_seq=g_left_sample.seq;
        target->left_sample_time=g_left_sample.time;
        target->hands_distance_m=g_left_sample.distance;
        /* Fresh invalid input still revokes ownership before any replay. */
        target->hands_coherent=coherent_now && g_left_sample.coherent;
        m9_sample_from_frame(&f,1,&target->m9_pose);
        if(!target->hands_coherent || target->m9_pose.sequence!=target->left_sample_seq ||
           target->m9_pose.sequence!=target->aim_sample_seq)target->m9_pose.valid=0;
    }
    return 1;
}

/* Game-tick producer builds all commands from ONE selected snapshot. */
static int fire_from_frame(const DG_XR_FRAME *frame, DG_BRIDGE_FIRE *cmd)
{
    const DG_XR_HAND *h;
    const DG_XR_HAND_POSE *p;

    memset(cmd, 0, sizeof *cmd);
    if (!InterlockedCompareExchange(&g_fire_mode, 0, 0)) return 0;
    cmd->click = g_xrcfg.trigger_fire;
    cmd->stream_id = g_arm_pose_stream_id;
    if (!frame) return 0;

    if (g_arm_track == ARM_TRACK_OFF) {
        /* No tracked arm does not mean no trigger. The vanilla-arm decision
           run of 2026-09-01 (head aim, vr_arm_track=off) fired nothing
           because this used to return 0 here - 2248 ticks of "stale" on the
           fire gate against 0 published. Without an arm to name the hand,
           the trigger belongs to the RIGHT hand by the same fixed convention
           that gives move_command the left stick. */
        h = &frame->right_hand;
        p = &h->grip;
    } else {
        h = (g_arm_track == ARM_TRACK_R_GRIP || g_arm_track == ARM_TRACK_R_AIM)
                ? &frame->right_hand : &frame->left_hand;
        p = (g_arm_track == ARM_TRACK_R_AIM || g_arm_track == ARM_TRACK_L_AIM)
                ? &h->aim : &h->grip;
    }

    cmd->press_seq = h->trigger_press_seq;
    cmd->release_seq = h->trigger_release_seq;
    cmd->value = (double)h->trigger_value;
    /* Same freshness bar as the pose. A trigger that is trusted while the hand
       it belongs to is not would aim with an animated arm and fire with a
       tracked finger. */
    cmd->valid = (p->active && p->tracked && p->orientation_valid &&
                  p->pose_age_ms <= 100) ? 1 : 0;
    return 1;
}

static int move_from_frame(const DG_XR_FRAME *frame, DG_BRIDGE_MOVE *cmd)
{

    memset(cmd, 0, sizeof *cmd);
    if (!InterlockedCompareExchange(&g_move_mode, 0, 0) &&
        !InterlockedCompareExchange(&g_turn_mode, 0, 0)) {
        input_turn_rate(0.0);
        return 0;
    }
    if (!frame) { input_turn_rate(0.0); return 0; }
    cmd->stream_id = g_arm_pose_stream_id;
    cmd->x = (double)frame->left_hand.thumbstick_x;
    cmd->y = (double)frame->left_hand.thumbstick_y;
    cmd->valid = (frame->left_hand.grip.active &&
                  frame->left_hand.grip.pose_age_ms <= 100) ? 1 : 0;
    /* The right stick no longer writes the game's turn bytes: it rotates
       the TRACKING SPACE (dg_xr_turn_rate), so the view, the walk frame
       and the arm all turn together and a stick turn is indistinguishable
       from the player turning on their feet. Measured before this existed
       (2026-08-23): the pad-byte stick turn rotated the body under a
       room-pinned arm, and the player recalibrated after every turn. The
       pad turn path stays alive in the bridge as the body-follow's
       actuator only, so turn_x is deliberately 0 here. */
    {
        double x = (frame->right_hand.grip.active &&
                    frame->right_hand.grip.pose_age_ms <= 100)
                       ? (double)frame->right_hand.thumbstick_x : 0.0;
        double dz = (double)InterlockedCompareExchange(
                        &g_turn_deadzone_mils_live, 0, 0) / 1000.0;
        double gain = (double)InterlockedCompareExchange(
                          &g_turn_gain_mils_live, 0, 0) / 1000.0;
        double m = x < 0.0 ? -x : x;
        if (!InterlockedCompareExchange(&g_turn_mode, 0, 0) ||
            m <= dz || dz >= 1.0)
            input_turn_rate(0.0);
        else
            input_turn_rate((x > 0.0 ? 1.0 : -1.0) * (m - dz) / (1.0 - dz) *
                            DG_TURN_RATE_DEG_S * gain);
    }
    cmd->turn_x = 0.0;
    cmd->turn_valid = 0;
    return 1;
}

#ifdef DG_HOOK_TEST
/* Compatibility checks exercise the same shaping without a live bridge. */
static int fire_command(DG_BRIDGE_FIRE *cmd) {
    DG_XR_FRAME f;
    return fire_from_frame((input_get_frame(&f)&1) ? &f : NULL,cmd);
}
static int move_command(DG_BRIDGE_MOVE *cmd) {
    DG_XR_FRAME f;
    return move_from_frame((input_get_frame(&f)&1) ? &f : NULL,cmd);
}
#endif
static volatile LONG g_action_route_denied;
static volatile LONG g_camera_route_denied=1;
static void action_from_frame(DG_ACTION_SAMPLE *out) {
    DG_XR_FRAME f; const DG_XR_HAND *l,*r;
    memset(out,0,sizeof *out);
    if(!InterlockedCompareExchange(&g_armed,0,0) || g_source!=SRC_XR ||
       InterlockedCompareExchange(&g_script_menu_active,0,0) || !(input_get_frame(&f)&1)) return;
    l=&f.left_hand;r=&f.right_hand;
    out->source=((uint64_t)g_arm_pose_stream_id<<8)|(unsigned)(g_source+1);
    out->sample=l->grip.sample_seq;out->down=l->primary_button!=0;
    out->shutter_down=r->trigger_click!=0;
    out->zoom_y=r->thumbstick_y;out->zoom_active=r->thumbstick_active!=0;
    out->shutter_denied=InterlockedCompareExchange(&g_camera_route_denied,0,0)!=0;
    out->age_ms=l->grip.pose_age_ms>r->grip.pose_age_ms?l->grip.pose_age_ms:r->grip.pose_age_ms;
    out->valid=out->sample && out->sample==r->grip.sample_seq && l->grip.active && r->grip.active &&
        l->grip.tracked && r->grip.tracked && l->grip.orientation_valid && r->grip.orientation_valid;
    out->denied=InterlockedCompareExchange(&g_action_route_denied,0,0) ||
        l->thumbstick_click || r->thumbstick_click;
}
#include "dg_m9_input.inl"
#include "dg_blade_input.inl"
#include "dg_mod_menu_input.inl"
#include "dg_controls_producer.inl"
#include "dg_controls_cleanup.inl"

static void start_command(void)
{
    uint64_t seq;
    if (script_menu_blocked()) return;
    if (!InterlockedCompareExchange(&g_start_mode, 0, 0)) return;
    seq = input_menu_press_seq();
    if (!seq || seq == g_start_seen) return;
    g_start_seen = seq;
    if(InterlockedCompareExchange(&g_mod_input_capture,0,0))return;
    if (g_source == SRC_SCRIPT) {
        DG_XR_FRAME frame;
        if (!g_armed || !(input_get_frame(&frame) & 1)) return;
    }
    if (!script_menu_blocked()) {
        /* START may pause before the next camera/status sample. Close hotkey
           ownership immediately, including a quick pause/resume cycle. */
        g_buttons_start_ms=GetTickCount(); g_buttons_start_block=1;
        dg_bridge_controls_context(0);
        dg_bridge_start_now();
    }
}

static int controller_camera_context(LONG camera_seq, int raw_gameplay, DWORD now_ms)
{
    if (camera_seq!=g_buttons_camera_seen) {
        g_buttons_camera_seen=camera_seq;
        g_buttons_camera_ms=now_ms; g_buttons_have_camera=1;
    }
    if (g_buttons_start_block) {
        if ((DWORD)(now_ms-g_buttons_start_ms)<=100u) return 0;
        g_buttons_start_block=0;
    }
    /* A camera sample earns a bounded lease, not a 1:1 camera/Present
       assumption. Extra Presents must not break a continuously held Y. */
    return raw_gameplay && g_buttons_have_camera &&
           (DWORD)(now_ms-g_buttons_camera_ms)<=100u;
}

static int action_present_allowed(int armed,int flat,int script_menu,int gameplay,int hatch)
{
    /* Hatch theater deliberately stops camera handoff. Its validated native
       context must survive the resulting flat fallback for the entire hold. */
    return armed && !script_menu && ((!flat && gameplay) || hatch==2);
}

static void controller_context_publish_all(int allowed,int special,int radial)
{
    if (g_buttons_gameplay != (allowed ? 1 : 0)) g_buttons_epoch++;
    g_buttons_gameplay=allowed ? 1 : 0;
    dg_xr_controller_context(allowed && !InterlockedCompareExchange(&g_mod_input_capture,0,0));
    dg_bridge_controls_context_all(allowed,special,radial);
    if (!allowed) {
        dg_bridge_cancel_toggle();
        dg_xr_button_gate_step(&g_script_button_gate,g_buttons_epoch,0,0,0,GetTickCount());
    }
}
static void controller_context_publish(int allowed)
{
    controller_context_publish_all(allowed,0,allowed);
}

static void script_primary_command(void)
{
    DG_XR_FRAME frame;
    unsigned events;
    if (g_source != SRC_SCRIPT || !g_armed ||
        InterlockedCompareExchange(&g_script_menu_active, 0, 0)) return;
    g_script_primary_seen=dg_xr_script_primary_press_seq(DG_XR_HAND_RIGHT);
    if (!(input_get_frame(&frame)&1)) {
        dg_xr_button_gate_step(&g_script_button_gate,g_buttons_epoch,0,0,0,GetTickCount());
        return;
    }
    events=dg_xr_button_gate_step(&g_script_button_gate,g_buttons_epoch,
        g_buttons_gameplay && frame.right_hand.grip.active && frame.right_hand.grip.pose_age_ms<=100,
        frame.right_hand.primary_button,0,GetTickCount());
    if (events&DG_XR_BUTTON_TOGGLE) dg_bridge_request_toggle();
}

#include "dg_probe_health.inl"
#include "dg_hud_watch.inl"
static void capture_observed(int eye,uint64_t id) {hud_capture(eye,id);ci_published(eye,id);}

static LONG CALLBACK veh(EXCEPTION_POINTERS *ep) {
    CONTEXT *c;

    if (ep->ExceptionRecord->ExceptionCode != EXCEPTION_SINGLE_STEP)
        return EXCEPTION_CONTINUE_SEARCH;

    c = ep->ContextRecord;
    pd_raw(c,g_hook_addr,g_scene_blur_addr);
    if (hud_context(c)) {pd_result(1);return EXCEPTION_CONTINUE_EXECUTION;}
    if (phase_context(c)) {pd_result(2);return EXCEPTION_CONTINUE_EXECUTION;}
    if(c->Dr6&12u)pd_result(0);
    if (dg_scene_blur_owns(c, g_scene_blur_addr)) {
        DG_XR_FRAME frame;
        int flags = 0, suppress;
        int armed = InterlockedCompareExchange(&g_armed,0,0) != 0;
        int real_xr = InterlockedCompareExchange(&g_source,0,0) == SRC_XR;
        int stereo = InterlockedCompareExchange(&g_stereo,0,0) != 0;
        int enabled = InterlockedCompareExchange(&g_scene_blur_enabled,0,0) != 0;
        int menu = InterlockedCompareExchange(&g_script_menu_active,0,0) != 0;
        int theater = dg_bridge_theater_verdict() != 0;
        if (armed && real_xr && stereo && enabled && !menu && !theater)
            flags = dg_xr_get_stereo(&frame); /* bounded VEH-safe publication */
        suppress = dg_scene_blur_allowed(armed,real_xr,stereo,enabled,
                                         theater,menu,flags);
        dg_scene_blur_context(c,g_scene_blur_addr,suppress);
        InterlockedIncrement(&g_traps); /* same resume-flag watchdog */
        InterlockedIncrement(&g_scene_blur_hits);
        if (suppress) InterlockedIncrement(&g_scene_blur_zeroed);
        return EXCEPTION_CONTINUE_EXECUTION;
    }
    if (!(c->Dr6 & 0x1)) return EXCEPTION_CONTINUE_SEARCH;   /* not ours */

    render_link_camera(c,g_phase_stage_addr,g_phase_consume_addr);
    phase_camera_context(c);

    InterlockedIncrement(&g_traps);
    if (InterlockedCompareExchange(&g_script_menu_active, 0, 0)) {
        /* Detector only: never enter camera/policy/gameplay callbacks. */
        script_menu_sample_context();
        c->EFlags |= 0x10000;
        c->Dr6 = 0;
        return EXCEPTION_CONTINUE_EXECUTION;
    }
    {
    /* One camera-seam visit is one policy pass: arm_pose_target's
       dg_pose_step and everything dg_bridge runs from the seam calls below
       dispatch through the table snapshotted here, so a policy swap adopted
       on the tick seam of ANOTHER thread can never split this visit into
       half-old, half-new maths. */
    void *pol_pass = dg_policy_pass_begin();
    camera_pass_begin();
    if (g_armed) apply_transform();
    /* F4, and later F5. This fires during render setup, so it is after every
       actor has run - which the Action() seam is not: the first probe wrote
       the arm's visibility 1329 times and read it back set on every following
       tick, because the arm actor runs later and puts it straight back.
       Ordering is the whole problem here, and this is the only seam we already
       hold that is on the right side of it. */
    {
        DG_BRIDGE_ARM_TARGET target;
        DG_REC_FRAME r;
        int record_on = InterlockedCompareExchange(&g_rec_on, 0, 0) != 0;
        int record_frame = 0, have_target;
        /* The flight recorder taps the same published frame the consumers
           below are about to read, before arm_pose_target advances the pair
           counter - so the record carries the identity the PREVIOUS pair
           completed under, and a diff of consecutive records shows exactly
           which frames advanced it. Duplicate frames (this seam runs several
           times per published XR frame) fold away inside dg_rec_capture. */
        if (record_on) {
            DG_XR_FRAME rf;
            memset(&rf, 0, sizeof rf);
            if (g_arm_absolute_aim && g_aim_camera.valid) {
                rf = g_aim_camera.frame;
                record_frame = 1;
            } else record_frame = (input_get_frame(&rf) & 1) != 0;
            {
                LARGE_INTEGER q;
                QueryPerformanceCounter(&q);
                dg_rec_pack(&rf, q.QuadPart,
                            (unsigned int)g_arm_pose_stream_id,
                            (unsigned int)g_arm_pose_pair_id,
                            (unsigned int)InterlockedCompareExchange(
                                &g_present_frame, 0, 0),
                            (unsigned int)InterlockedCompareExchange(
                                &g_current_eye, 0, 0),
                            &r);
                /* The game-side half, taken here for the same reason the
                   identity fields are: what the bridge holds right now is
                   what the PREVIOUS pair solved with, so one record pairs
                   an input with the state that consumed it rather than
                   with a solve that has not happened yet. */
                dg_bridge_rec_pair(&r.pair);
            }
        }
        have_target = arm_pose_target(&target);
        if (record_on) {
            r.absolute = g_absolute_record;
            if (record_frame || r.absolute.present) dg_rec_capture(&g_rec, &r);
        }
        dg_bridge_arm_seam_now(have_target ? &target : NULL);
        if (InterlockedCompareExchange(&g_camera_telemetry_valid, 0, 0))
            dg_bridge_rec_camera(&g_camera_telemetry_eye_world,
                                 &g_camera_telemetry_proj);
        else
            dg_bridge_rec_camera(NULL, NULL);
        dg_bridge_move_probe_now((ULONGLONG)(ULONG_PTR)g_base);
        /* A bridge-off session retains only its bounded tracking-turn route. */
        controls_fallback_turn();
    }
    dg_policy_pass_end(pol_pass);
    }

    /* An execution breakpoint is a fault, delivered before the instruction
       runs. Without the resume flag the same instruction would trap again
       immediately and the game would hang; RF is cleared by the CPU after the
       next instruction retires. */
    c->EFlags |= 0x10000;
    c->Dr6 = 0;
    return EXCEPTION_CONTINUE_EXECUTION;
}

/* ------------------------------------------------------- arm / disarm --- */

/* DR0 camera; optional DR1 scene blur: L1 (bit 2), execute, length zero.
   R/W0 = 00 (execute), LEN0 = 00 (must be 0 for exec),
   plus the reserved bit 10 that some CPUs require set. */
#define DR7_ARM 0x00000401ull

#include "dg_debug_registers.inl"

static int arm_all(int arm) {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    THREADENTRY32 te;
    DWORD self_pid = GetCurrentProcessId();
    int n = 0;
    int diagnostic_mode=arm?(hud_active?1:(pp_status==PP_RECORDING?2:(g_phase_enabled?4:0))):0;
    if(diagnostic_mode)pd_begin(diagnostic_mode,
        hud_active?g_base+HUD_ENTRY_RVA:g_phase_stage_addr,
        hud_active?0:g_phase_consume_addr,hud_active?0x10:0x50);
    if (snap == INVALID_HANDLE_VALUE) {if(pd_mode)InterlockedIncrement(&pd_snapshot_fail);return 0;}
    te.dwSize = sizeof(te);
    if (Thread32First(snap, &te)) {
        do {
            if (te.th32OwnerProcessID != self_pid) continue;
            if (te.th32ThreadID == g_self_tid) continue;
            {
                HANDLE t = OpenThread(THREAD_GET_CONTEXT | THREAD_SET_CONTEXT |
                                      THREAD_SUSPEND_RESUME, FALSE, te.th32ThreadID);
                if (!t) {if(pd_mode)InterlockedIncrement(&pd_open_fail);continue;}
                if (SuspendThread(t) != (DWORD)-1) {
                    int set_ok=set_dr(t,arm);
                    if(set_ok)n++;
                    phase_registration(t,te.th32ThreadID,set_ok);
                    if(diagnostic_mode)pd_registration(t,te.th32ThreadID,set_ok);
                    if(ResumeThread(t)==(DWORD)-1 && pd_mode)InterlockedIncrement(&pd_resume_fail);
                } else if(pd_mode)InterlockedIncrement(&pd_suspend_fail);
                CloseHandle(t);
            }
        } while (Thread32Next(snap, &te));
    }
    CloseHandle(snap);
    return n;
}

/* --------------------------------------------------------------- config --- */

#define DEG2RAD (3.14159265358979323846 / 180.0)

static int parse_script_menu(const char *buf, int *enabled)
{
    const char *s = buf;
    int seen = 0;
    *enabled = 0;
    while (*s) {
        size_t n;
        s += strspn(s, " \t\r\n");
        n = strcspn(s, " \t\r\n");
        if (n >= 14 && !_strnicmp(s, "vr_script_menu", 14) &&
            (n == 14 || s[14] == '=')) {
            if (seen++) return 0;
            if (n == 17 && !memcmp(s + 14, "=on", 3)) *enabled = 1;
            else if (n == 18 && !memcmp(s + 14, "=off", 4)) *enabled = 0;
            else return 0;
        }
        s += n;
    }
    return 1;
}

static void script_menu_sanitize(POSE *p, DG_BRIDGE_CONFIG *bc,
                                 int *track, int *hand)
{
    int menu = bc->menu_mode;
    memset(bc, 0, sizeof *bc);
    bc->menu_mode = menu;
    bc->script_menu_only = 1;
    memset(p, 0, sizeof *p);
    *track = ARM_TRACK_OFF;
    *hand = 0;
}

static int parse_config(char *buf, POSE *p, double *seconds,
                       int *source, DG_XR_CONFIG *xc, DG_BRIDGE_CONFIG *bc,
                       int *arm_track_out, int arm_qmap_out[4],
                       int *arm_hand_out, ARM_POS_CFG *arm_pos_out) {
    char *s;
    int arm_track_cfg = ARM_TRACK_OFF;
    int arm_qmap_cfg[4] = { -1, 2, -3, 4 };
    int arm_hand_cfg = 0;
    int aim_probe_cfg = 0, geometry_probe_cfg = 0;
    ARM_POS_CFG ap;

    /* Diagnostic opt-in never survives a missing/malformed configuration. */
    dg_aim_capture_configure(0);dg_aim_capture_native_configure(0);
    if (!parse_script_menu(buf, &g_script_menu_cfg)) return 0;
    arm_pos_cfg_defaults(&ap);

    memset(p, 0, sizeof(*p));
    *source = SRC_SYNTHETIC;
    memset(xc, 0, sizeof(*xc));
    xc->yaw_sign = xc->pitch_sign = xc->roll_sign = 1.0;
    xc->x_sign = xc->y_sign = xc->z_sign = 1.0;
    xc->scale = 1000.0;
    xc->positional = 1;
    xc->stereo = 0;
    xc->stereo_test = 0;
    xc->scene_blur_fix = 1;
    xc->stereo_phase_fix = 1;
    xc->stereo_eye_x_sign = -1;
    xc->ui2d = 0; xc->ui2d_scale_mils = 750; xc->ui2d_conv_e5 = 1200; xc->ui2d_sign = 1; xc->ui2d_hold = 0; xc->feedback_skip = 1;
    /* Wrist radar: off, HUD radar left alone. The offset is a guess for a
       Touch Plus left controller (grip +Y runs back toward the forearm, +X
       out of the palm), meant to be tuned live. */
    xc->radar_mode = 0; xc->radar_hud = 1; xc->radar_space_local = 0; xc->radar_size = 0.09;
    xc->radar_offset[0] = 0.02; xc->radar_offset[1] = 0.08; xc->radar_offset[2] = 0.03;
    xc->radar_rot[0] = 0.0; xc->radar_rot[1] = 90.0; xc->radar_rot[2] = -90.0;
    xc->radar_gaze_deg = 35.0; xc->radar_gaze_pitch = 15.0;

    xc->stereo_proj_x_sign = -1;
    xc->trigger_deadzone = 0.10;
    xc->trigger_fire = 0.55;
    /* Theater screen geometry, comfort defaults per the research doc: 2 m
       away, 2 m wide (~50 degrees). Validated below with the triggers. */
    xc->theater_dist = 2.0;
    xc->theater_width = 2.0;
    memset(bc, 0, sizeof(*bc));
    /* Rebuilt from the file on every read: deleting the vr_policy line is
       how the builtin policy is asked back, so it must not stick. */
    g_policy_path[0] = 0;
    bc->fps_mode = DG_FPS_MODE_OFF;             /* fail-closed default */
    bc->move_deadzone_mils = 150;               /* mode stays 0: no walking */
    bc->move_prone_max = 120;                   /* third/prone stay 0 */
    bc->move_dir_sign = 1;
    bc->turn_gain_mils = 1000;                  /* mode stays 0: no turning */
    bc->turn_follow_thresh_mdeg = 0;            /* body-follows-aim off */
    InterlockedExchange(&g_arm_freeze_cfg, 0);  /* deleting the line = off */
    bc->turn_follow_full_mdeg = 0;
    /* The ROLL V5.1 outcome (2026-09-02, runs 2-9) is the default: the live
       actor-yaw frame, the organic F10 compensation, the stick as the
       follow's facing and no aim-hold. Each has a rollback WORD (legacy,
       full, head, hold) rather than a rollback by deletion: a line that
       goes missing must not quietly bring the 45-degree strip residue or
       the glance-turns back. test_config_defaults_are_the_proven_stand. */
    bc->adjust_frame = 1;
    bc->arm_comp = 1;
    bc->follow_src = 2;
    bc->follow_aim = 1;
    bc->arm_hand_basis = 1;                     /* world; `root` rolls back */

    if (!strchr(buf, '=')) {                    /* legacy: "<yaw> [seconds]" */
        char *end;
        double v = strtod(buf, &end);
        if (end != buf) {
            p->yaw = v * DEG2RAD;
            v = strtod(end, &end);
            if (v > 0.0) *seconds = v;
        }
        *arm_track_out = arm_track_cfg;
        memcpy(arm_qmap_out, arm_qmap_cfg, sizeof arm_qmap_cfg);
        *arm_hand_out = arm_hand_cfg;
        *arm_pos_out = ap;
        bc->arm_anchor = ap.anchor;
        return 1;
    }

    s = buf;
    while (*s) {
        char key[32], *k = key, *end;
        double v;
        while (*s && (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')) s++;
        if (!*s) break;
        while (*s && *s != '=' && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n'
               && (size_t)(k - key) < sizeof(key) - 1)
            *k++ = *s++;
        *k = 0;
        if (*s != '=') continue;                /* malformed token: skip it */
        s++;

        if (_stricmp(key, "sweep_axis") == 0) {  /* non-numeric values */
            if      (_strnicmp(s, "pitch", 5) == 0) p->sweep_axis = 1;
            else if (_strnicmp(s, "roll", 4) == 0)  p->sweep_axis = 2;
            else                                    p->sweep_axis = 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_pistol_reload") == 0) {
            char *e=s;
            while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            bc->pistol_reload=(e-s==2 && !_strnicmp(s,"on",2));
            s=e;continue;
        }
        if (_stricmp(key, "vr_hf_blade") == 0) {
            char *e=s;
            while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s==2 && !_strnicmp(s,"on",2))bc->hf_blade=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))bc->hf_blade=0;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_m9_slide") == 0) {
            char *e=s;
            while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            bc->m9_slide=(e-s==2 && !_strnicmp(s,"on",2));
            s=e;continue;
        }
        if (_stricmp(key, "vr_arm_show") == 0) {
            /* F4 probe. Off unless the word is `on`, like every other switch
               here: the default is that we observe and change nothing. */
            bc->arm_show = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_grip_debug") == 0) {
            char *e=s;while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s==2 && !_strnicmp(s,"on",2)) xc->grip_debug=1;
            else if(e-s==3 && !_strnicmp(s,"off",3)) xc->grip_debug=0;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_geometry_probe") == 0) {
            char *e=s;while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s==2 && !_strnicmp(s,"on",2)) geometry_probe_cfg=1;
            else if(e-s==3 && !_strnicmp(s,"off",3)) geometry_probe_cfg=0;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_aim_probe") == 0) {
            char *e = s;
            while (*e && *e != ' ' && *e != '\t' && *e != '\r' && *e != '\n') e++;
            if (e-s == 2 && _strnicmp(s, "on", 2) == 0) aim_probe_cfg = 1;
            else if (e-s == 3 && _strnicmp(s, "off", 3) == 0) aim_probe_cfg = 0;
            else return 0;
            s = e;
            continue;
        }
        if (_stricmp(key,"vr_ui2d_vs")==0) {
            /* Comma-separated 64-bit hex hashes of the sprite vertex shaders. */
            xc->ui2d_vs_count = 0;
            if (_strnicmp(s, "any", 3) == 0) { xc->ui2d_vs[xc->ui2d_vs_count++] = ~0ull; s += 3; }
            while (*s && *s!=' ' && *s!='\t' && *s!='\r' && *s!='\n') {
                char *e2 = s; unsigned long long h = _strtoui64(s, &e2, 16);
                if (e2 == s) return 0;
                if (h && xc->ui2d_vs_count < 8) xc->ui2d_vs[xc->ui2d_vs_count++] = h;
                s = e2; if (*s == ',') s++;
            }
            continue;
        }
        if (_stricmp(key,"vr_stereo_phase_fix")==0) {
            char *e=s;
            while(*e&&*e!=' '&&*e!='\t'&&*e!='\r'&&*e!='\n')e++;
            if(e-s!=1||(*s!='0'&&*s!='1'))return 0;
            xc->stereo_phase_fix=*s=='1';s=e;continue;
        }
        if (_stricmp(key, "vr_rec") == 0) {
            /* Anything that is not exactly on or off leaves the setting as
               it was: a typo must not silently stop the evidence. */
            if (_strnicmp(s, "on", 2) == 0)
                InterlockedExchange(&g_rec_cfg_on, 1);
            else if (_strnicmp(s, "off", 3) == 0)
                InterlockedExchange(&g_rec_cfg_on, 0);
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_eye_dump") == 0) {
            InterlockedExchange(&g_eye_dump_cfg, strtol(s, &s, 10));
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_rec_dump") == 0) {
            InterlockedExchange(&g_rec_cfg_dump, strtol(s, &s, 10));
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_policy") == 0) {
            /* A filesystem path, so the value runs to the END OF THE LINE
               (paths may contain spaces) with trailing whitespace trimmed -
               put nothing after it on its line. The worker's 1 Hz poll does
               the rest: watch <path>.ver, stage a copy, validate, offer.
               Everything about the value is validated in dg_policy.c; a
               nonsense path is a counted, logged refusal, never a crash. */
            char *e = s;
            size_t len;
            while (*e && *e != '\r' && *e != '\n') e++;
            len = (size_t)(e - s);
            while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t')) len--;
            if (len >= sizeof(g_policy_path)) len = sizeof(g_policy_path) - 1;
            memcpy(g_policy_path, s, len);
            g_policy_path[len] = 0;
            s = e;
            continue;
        }
        if (_stricmp(key, "vr_recoil") == 0) {
            /* Degrees and millimetres, either of them fractional, and the
               millimetres optional. Anything unparsable leaves the kick off
               rather than half configured - the same rule as the probe above,
               and it matters more here, because this is the one setting that
               moves the player's hand for a reason the player did not give. */
            char *end = s, *before;
            double v[2];
            int i, good = 1;
            for (i = 0; i < 2; i++) {
                while (*end == ' ' || *end == ',' || *end == '\t') end++;
                before = end;
                v[i] = strtod(end, &end);
                if (end == before) { if (i == 0) good = 0; v[i] = 0.0; break; }
            }
            if (good) {
                bc->recoil_climb_mdeg = (int)(v[0] * 1000.0 + 0.5);
                bc->recoil_push_um = (int)(v[1] * 1000.0 + 0.5);
            }
            s = end;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_hand_probe") == 0) {
            /* Three signed PS2 angles, 4096 to a turn. Anything unparsable
               leaves the probe off rather than half configured: a probe that
               writes a number nobody chose is worse than one that does
               nothing. */
            char *end = s, *before;
            long v[3];
            int i, good = 1;
            for (i = 0; i < 3; i++) {
                while (*end == ' ' || *end == ',' || *end == '\t') end++;
                before = end;
                v[i] = strtol(end, &end, 10);
                if (end == before) { good = 0; break; }
            }
            if (good) {
                for (i = 0; i < 3; i++) bc->hand_probe_rot[i] = (int)v[i];
                bc->hand_probe = 1;
            }
            s = end;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_turn") == 0) {
            /* smooth[,gain]. Only `smooth` turns it on: snap is deliberately
               not a mode (dg_move.h says why), and refusing the word beats
               silently running smooth under a snap label. */
            if (_strnicmp(s, "smooth", 6) == 0) {
                bc->turn_mode = 1;
                s += 6;
                if (*s == ',') {
                    double v3 = strtod(s + 1, &end);
                    if (end != s + 1 && v3 >= 0.1 && v3 <= 3.0) {
                        bc->turn_gain_mils = (int)(v3 * 1000.0 + 0.5);
                        s = end;
                    }
                }
            } else {
                bc->turn_mode = 0;
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_twist") == 0) {
            /* fore[,wrist[,swing]] in degrees: the arm's whole roll and
               swing authority. Out-of-range or missing values keep the
               built-in defaults (90/55/70) rather than half-applying. */
            static const int lo[3] = { 10, 0, 10 };
            int *dst[3];
            int part;
            dst[0] = &bc->hand_fore_twist_mdeg;
            dst[1] = &bc->hand_wrist_twist_mdeg;
            dst[2] = &bc->hand_wrist_swing_mdeg;
            for (part = 0; part < 3; part++) {
                double v4 = strtod(s, &end);
                if (end == s) break;
                if (v4 >= (double)lo[part] && v4 <= 180.0)
                    *dst[part] = (int)(v4 * 1000.0 + 0.5);
                s = end;
                if (*s != ',') break;
                s++;
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_turn_follow") == 0) {
            /* deg[,full]: the aim-gap threshold past which the body turns
               toward the aim, and the gap at which it turns at full stick
               rate (defaults to deg+50). Any non-number, `off` included,
               leaves it off - fail-closed like every switch here. */
            double v1 = strtod(s, &end);
            if (end != s && v1 >= 10.0 && v1 <= 120.0) {
                bc->turn_follow_thresh_mdeg = (int)(v1 * 1000.0 + 0.5);
                bc->turn_follow_full_mdeg =
                    (int)((v1 + 50.0) * 1000.0 + 0.5);
                s = end;
                if (*s == ',') {
                    double v2 = strtod(s + 1, &end);
                    if (end != s + 1 && v2 > v1 && v2 <= 180.0) {
                        bc->turn_follow_full_mdeg =
                            (int)(v2 * 1000.0 + 0.5);
                        s = end;
                    }
                }
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_follow_head_sign") == 0) {
            /* +1 or -1: flips the head-yaw term of the facing gap, in case
               the camera's yaw convention turns out mirrored against the
               character root's. One marker edit instead of a redeploy;
               anything else keeps +1. */
            long hv = strtol(s, &end, 10);
            if (end != s && (hv == 1 || hv == -1))
                bc->follow_head_sign = (int)hv;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_turn_follow_src") == 0) {
            /* stick (default), hand or head: which facing the body follows.
               Head turned the body on every glance (run 3, 2026-09-01). */
            bc->follow_src = (_strnicmp(s, "head", 4) == 0) ? 0 :
                             (_strnicmp(s, "hand", 4) == 0) ? 1 : 2;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_adjust_frame") == 0) {
            /* live (default) or legacy: which heading the adjust<->world
               conversions conjugate with. Only the rollback word selects
               the frozen frame; a typo keeps the proven one. */
            bc->adjust_frame = (_strnicmp(s, "legacy", 6) == 0) ? 0 : 1;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_camera_yaw") == 0) {
            /* Experimental and fail-closed: only the exact anchor token opts
               in; off and every unknown value select legacy behavior. */
            bc->camera_yaw_anchor =
                (_strnicmp(s, "anchor", 6) == 0 &&
                 (s[6] == 0 || s[6] == ' ' || s[6] == '\t' ||
                  s[6] == '\r' || s[6] == '\n')) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_comp") == 0) {
            /* organic (default) or full: does the F10 compensation remove
               only the uncommanded body yaw, or the stick-driven turn too
               (full reversed the arm after a 180, run 3 2026-09-02). */
            bc->arm_comp = (_strnicmp(s, "full", 4) == 0) ? 0 : 1;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_turn_follow_aim") == 0) {
            /* on (default) or hold: does the follow write while aiming. */
            bc->follow_aim = (_strnicmp(s, "hold", 4) == 0) ? 0 : 1;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_turn_dir") == 0) {
            /* +1 or -1: declare the follow's byte sign instead of making
               every session re-learn it from stick turns (measured -1 on
               this install, 2026-08-23). Anything else - `learn`
               included - keeps the per-session vote learning, fail-open
               to the mechanism that needs no configuration. */
            long dv = strtol(s, &end, 10);
            if (end != s && (dv == 1 || dv == -1))
                bc->turn_dir_override = (int)dv;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_move") == 0) {
            /* Off unless the word is `on`, like every switch here. */
            bc->move_mode = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_move_third") == 0) {
            bc->move_third = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_move_prone") == 0) {
            bc->move_prone = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_move_prone_max") == 0) {
            /* Out of range keeps the default - the vr_move_deadzone rule. */
            long v3 = strtol(s, &end, 10);
            if (end != s && v3 >= 34 && v3 <= 127) bc->move_prone_max = (int)v3;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_move_dir_offset") == 0) {
            long v3 = strtol(s, &end, 10);
            if (end != s && v3 >= -4095 && v3 <= 4095) bc->move_dir_offset = (int)(v3 & 4095);
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_tick_seam") == 0) {
            /* publish (default) or copy: see DG_BRIDGE_CONFIG.tick_seam. */
            bc->tick_seam = (_strnicmp(s, "copy", 4) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_move_dir_org") == 0) {
            /* camera (default) or body: which yaw pad->dir is built on. */
            bc->move_dir_org = (_strnicmp(s, "body", 4) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_move_dir_sign") == 0) {
            long v3 = strtol(s, &end, 10);
            if (end != s && (v3 == 1 || v3 == -1)) bc->move_dir_sign = (int)v3;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_move_deadzone") == 0) {
            /* Parsed as a float once, carried as thousandths everywhere
               else. Out-of-range keeps the default rather than clamping
               here - the bridge clamps too, but a wrong marker should read
               back as the default it got, not as a silent nearest-fit. */
            double v3 = strtod(s, &end);
            if (end != s && v3 >= 0.0 && v3 <= 0.9)
                bc->move_deadzone_mils = (int)(v3 * 1000.0 + 0.5);
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_menu_clear") == 0) {

            if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
                char *cend = NULL;
                unsigned long raw = strtoul(s + 2, &cend, 16);
                if (cend != s + 2 && raw <= 0x00FFFFFFUL)
                    InterlockedExchange(&g_menu_clear_bits, (LONG)raw);
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_menu_deadzone") == 0 ||
            _stricmp(key, "vr_menu_delay") == 0 ||
            _stricmp(key, "vr_menu_repeat") == 0 ||
            _stricmp(key, "vr_menu_hand") == 0) {
            /* Out-of-range keeps the default rather than clamping here, the
               vr_move_deadzone rule: a wrong marker should read back as the
               default it got, not as a silent nearest-fit. The module clamps
               again on its own account, so the two can never disagree about
               what is safe - only about what was asked for. */
            if (_stricmp(key, "vr_menu_hand") == 0) {
                InterlockedExchange(&g_menu_hand_right,
                                    (_strnicmp(s, "right", 5) == 0) ? 1 : 0);
                while (*s && *s != ' ' && *s != '\t' && *s != '\r' &&
                       *s != '\n') s++;
                continue;
            }
            {
                double v4 = strtod(s, &end);
                if (end != s) {
                    if (_stricmp(key, "vr_menu_deadzone") == 0) {
                        if (v4 >= 0.05 && v4 <= 0.9)
                            InterlockedExchange(&g_menu_deadzone_mils,
                                                (LONG)(v4 * 1000.0 + 0.5));
                    } else if (_stricmp(key, "vr_menu_delay") == 0) {
                        if (v4 >= DG_MENU_MIN_INITIAL_MS &&
                            v4 <= DG_MENU_MAX_INITIAL_MS)
                            InterlockedExchange(&g_menu_delay_ms, (LONG)v4);
                    } else {
                        if (v4 >= DG_MENU_MIN_REPEAT_MS &&
                            v4 <= DG_MENU_MAX_REPEAT_MS)
                            InterlockedExchange(&g_menu_repeat_ms, (LONG)v4);
                    }
                }
                s = end;
            }
            continue;
        }
        if (_stricmp(key, "vr_fire") == 0) {
            /* F7. `dry` evaluates the whole trigger contract every tick and
               writes nothing; `on` adds the pad write. Anything else is off -
               the one setting that can put rounds downrange is not something a
               typo should be able to reach, so both live words are matched and
               everything else falls through to off. */
            bc->fire_mode = (_strnicmp(s, "dry", 3) == 0) ? DG_FIRE_MODE_DRY
                          : (_strnicmp(s, "on", 2) == 0)  ? DG_FIRE_MODE_ON
                                                          : DG_FIRE_MODE_OFF;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_hand") == 0) {
            /* Only meaningful alongside vr_arm_track; off unless the word is
               `on`, like every other switch here. */
            arm_hand_cfg = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_aim") == 0) {
            size_t n = strcspn(s, " \t\r\n");
            if (n == 8 && !memcmp(s, "absolute", 8)) ap.absolute_aim = 1;
            else if (n == 8 && !memcmp(s, "relative", 8)) ap.absolute_aim = 0;
            else return 0;
            s += n;
            continue;
        }
        if (_stricmp(key, "vr_left_arm") == 0 ||
            _stricmp(key, "vr_twohand") == 0) {
            size_t n = strcspn(s, " \t\r\n");
            int enabled;
            if (n == 2 && !memcmp(s, "on", 2)) enabled = 1;
            else if (n == 3 && !memcmp(s, "off", 3)) enabled = 0;
            else return 0;
            if (_stricmp(key, "vr_left_arm") == 0) ap.left_arm = enabled;
            else ap.twohand = enabled;
            s += n;
            continue;
        }
        if (_stricmp(key, "vr_arm_freeze") == 0) {
            /* CONSTANT-REPLAY, a diagnostic. Off unless the word is `on`.
               It needs vr_arm_track to be something, because it freezes the
               first pair the tracker solves; with tracking off there is
               nothing to freeze and the key does nothing at all. The
               heartbeat prints the value the bridge actually holds - a
               marker key that silently failed to parse cost five live runs
               once already. */
            /* `on` = constant-replay (step 1, RAN 2026-08-22: 831 pairs,
               no tumble, so the write path is innocent). `rest` = step 2:
               only the rest reference is frozen, the target keeps
               following the controller, and the drift between frozen and
               recovered reference is measured - the strip feedback read
               directly. Anything else is off. */
            bc->arm_freeze = (_strnicmp(s, "rest", 4) == 0) ? 2
                           : (_strnicmp(s, "on", 2) == 0)   ? 1 : 0;
            InterlockedExchange(&g_arm_freeze_cfg, (LONG)bc->arm_freeze);
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_track") == 0) {
            /* Named poses only; anything unrecognised is off, like every other
               switch here. grip drives the wrist, aim is the weapon axis. */
            if      (_strnicmp(s, "right_aim",  9) == 0) arm_track_cfg = ARM_TRACK_R_AIM;
            else if (_strnicmp(s, "right_grip",10) == 0) arm_track_cfg = ARM_TRACK_R_GRIP;
            else if (_strnicmp(s, "right",      5) == 0) arm_track_cfg = ARM_TRACK_R_GRIP;
            else if (_strnicmp(s, "left_aim",   8) == 0) arm_track_cfg = ARM_TRACK_L_AIM;
            else if (_strnicmp(s, "left_grip",  9) == 0) arm_track_cfg = ARM_TRACK_L_GRIP;
            else if (_strnicmp(s, "left",       4) == 0) arm_track_cfg = ARM_TRACK_L_GRIP;
            else                                         arm_track_cfg = ARM_TRACK_OFF;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_quat_map") == 0) {
            /* Four signed axis letters, e.g. x,-z,y,w. A malformed map leaves
               the identity rather than a half-applied permutation. */
            int m[4];
            if (parse_qmap(s, m)) {
                int i;
                for (i = 0; i < 4; i++) arm_qmap_cfg[i] = m[i];
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_pos_sign") == 0) {
            /* Three signed numbers; only the sign of each is read. A short or
               malformed list leaves the measured default alone rather than
               applying half a mapping - the same rule vr_arm_quat_map uses. */
            double v3[3];
            char *e3 = s;
            int n;
            for (n = 0; n < 3; n++) {
                while (*e3 == ' ' || *e3 == ',') e3++;
                v3[n] = strtod(e3, &end);
                if (end == e3) break;
                e3 = end;
            }
            if (n == 3)
                for (n = 0; n < 3; n++)
                    ap.sign[n] = (v3[n] < 0.0) ? -1.0 : 1.0;
            s = e3;
            continue;
        }
        if (_stricmp(key, "vr_arm_shoulder") == 0) {
            /* Outboard, down, forward from the headset in millimetres. Clamped
               to half a metre: past that it stops being a shoulder and starts
               being an undeclared offset knob that would hide whatever is
               really wrong with the mapping. */
            double v3[3];
            char *e3 = s;
            int n;
            for (n = 0; n < 3; n++) {
                while (*e3 == ' ' || *e3 == ',') e3++;
                v3[n] = strtod(e3, &end);
                if (end == e3) break;
                e3 = end;
            }
            if (n == 3)
                for (n = 0; n < 3; n++) {
                    if (v3[n] < -500.0) v3[n] = -500.0;
                    if (v3[n] > 500.0) v3[n] = 500.0;
                    ap.shoulder_mm[n] = v3[n];
                }
            s = e3;
            continue;
        }
        if (_stricmp(key, "vr_arm_frame") == 0) {
            /* Named, and an unrecognised word gets the default rather than
               mode zero - mode zero is the gaze-following one this knob exists
               to stop being the default. */
            if      (_strnicmp(s, "head", 4) == 0) ap.frame = DG_XR_REL_FRAME_HEAD;
            else if (_strnicmp(s, "room", 4) == 0) ap.frame = DG_XR_REL_FRAME_ROOM;
            else if (_strnicmp(s, "body", 4) == 0) ap.frame = DG_XR_REL_FRAME_ROOM;
            else if (_strnicmp(s, "yaw", 3) == 0)  ap.frame = DG_XR_REL_FRAME_YAW;
            else                                   ap.frame = DG_XR_REL_FRAME_ROOM;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_hand_zero") == 0) {
            ap.hand_zero = (int)strtod(s, &end);
            if (end == s) ap.hand_zero = 0;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_arm_reach") == 0) {
            /* Millimetres, and refused rather than clamped outside human
               range: a reach of zero divides the scale by nothing and a reach
               of two metres silently halves every gesture. Both are better
               reported by keeping the default than by being obeyed. */
            double v3 = strtod(s, &end);
            if (end != s && v3 >= 300.0 && v3 <= 1000.0) ap.reach_mm = v3;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_arm_anchor") == 0) {
            /* Only `wrist` selects the old offset anchor; anything else,
               including a typo, gets the shoulder anchor. That is the
               deliberate direction to fail in - the offset anchor is the one
               that was measured to park the hand at the character's hip. */
            ap.anchor = (_strnicmp(s, "wrist", 5) == 0) ? 0 : 1;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_arm_bend") == 0) {
            /* Degrees, signed. 0 - and anything unparsable - means no write. */
            bc->arm_bend_deg = (int)strtod(s, &end);
            if (end == s) bc->arm_bend_deg = 0;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_arm_bend_joint") == 0) {
            bc->arm_bend_joint = (int)strtod(s, &end);
            if (end == s) bc->arm_bend_joint = 5;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_arm_uproll") == 0) {
            bc->arm_uproll = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            continue;
        }
        if (_stricmp(key, "vr_arm_hand_basis") == 0) {
            /* world (default) or root: how the hand drive turns the
               calibration-time hand by the controller's turn. Only the
               rollback word selects the old conjugation by the live root. */
            bc->arm_hand_basis = (_strnicmp(s, "root", 4) == 0) ? 0 : 1;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_turn_probe") == 0) {
            bc->turn_probe = (_strnicmp(s, "all", 3) == 0) ? 3 :
                             (_strnicmp(s, "dense", 5) == 0) ? 2 :
                             (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            continue;
        }
        if (_stricmp(key, "vr_work_probe") == 0) {
            /* Read-only, and off unless the word is `on`. */
            bc->work_probe = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_skel_probe") == 0) {
            bc->skel_probe = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_skel_base") == 0) {
            bc->skel_base = (int)strtod(s, &end);
            if (end == s) bc->skel_base = 0;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_adjust_probe") == 0) {
            bc->adjust_probe = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_adjust_joint") == 0) {
            bc->adjust_probe_joint = (int)strtod(s, &end);
            if (end == s) bc->adjust_probe_joint = 5;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_adjust_deg") == 0) {
            bc->adjust_probe_deg = (int)strtod(s, &end);
            if (end == s) bc->adjust_probe_deg = 30;
            s = end;
            continue;
        }
        if (_stricmp(key, "vr_adjust_axis") == 0) {
            /* Named, so a typo cannot quietly become axis 0 and hand back a
               measurement of the wrong axis that looks perfectly valid. */
            if (_strnicmp(s, "x", 1) == 0) bc->adjust_probe_axis = 0;
            else if (_strnicmp(s, "y", 1) == 0) bc->adjust_probe_axis = 1;
            else if (_strnicmp(s, "z", 1) == 0) bc->adjust_probe_axis = 2;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_fps_move") == 0) {
            /* Named values only, and `off` is tested before `on` so the two
               cannot be confused by a prefix match. Same rule as the mode
               above: an unknown word never becomes a guess, and here the safe
               guess is leaving the game's own value alone. */
            if      (_strnicmp(s, "off", 3) == 0) bc->fps_move = DG_FPS_MOVE_OFF;
            else if (_strnicmp(s, "on", 2) == 0)  bc->fps_move = DG_FPS_MOVE_ON;
            else                                  bc->fps_move = DG_FPS_MOVE_NATIVE;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_fps_mode") == 0) {
            /* Named modes only. An unknown word is `off`, never a guess. */
            if      (_strnicmp(s, "always", 6) == 0) bc->fps_mode = DG_FPS_MODE_ALWAYS;
            else if (_strnicmp(s, "toggle", 6) == 0) bc->fps_mode = DG_FPS_MODE_TOGGLE;
            else                                     bc->fps_mode = DG_FPS_MODE_OFF;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_start") == 0) {
            /* START is a write-capable mapping, so only the exact opt-in word
               enables it. Unknown values remain off. */
            InterlockedExchange(&g_start_mode,
                                (_strnicmp(s, "on", 2) == 0) ? 1 : 0);
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_theater") == 0) {
            /* Named modes only, and `measure` is tested before `on` cannot
               shadow it - they share no prefix, but the rule is the same as
               vr_fps_mode: an unknown word is `off`, never a guess, because
               `on` is the value that hands the camera to the director. */
            if      (_strnicmp(s, "measure", 7) == 0)
                bc->theater_mode = DG_THEATER_MEASURE;
            else if (_strnicmp(s, "on", 2) == 0)
                bc->theater_mode = DG_THEATER_ON;
            else
                bc->theater_mode = DG_THEATER_OFF;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_flat") == 0) {
            /* On unless the word is `off` - the reversed default of every
               other switch here, and deliberately so: OFF is a BLACK
               headset on the title and load screens, because those frames
               never have a camera pose and dg_xr_capture refuses them.
               Failing closed would mean failing to a blank screen, which
               is the defect rather than the safe side of it. */
            InterlockedExchange(&g_flat_mode,
                                (_strnicmp(s, "off", 3) == 0) ? 0 : 1);
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_menu") == 0) {
            /* Off unless the word is `measure`. `on` is accepted and means
               exactly `measure`, and the heartbeat says so out loud - there is
               no write route for these screens yet (see the vr_menu block near
               g_flat_mode), and a marker word that quietly did nothing while
               reading as "on" is the kind of thing that gets debugged for an
               evening. Anything else is off. */
            /* `measure` decides and counts but writes nothing - the mode
               that existed while there was no route to these screens.
               `on` now writes, through the pad detour in dg_bridge, because
               that route exists as of this build. Anything else is off. */
            {
                LONG m = DG_MENU_MODE_OFF;
                if (_strnicmp(s, "measure", 7) == 0) m = DG_MENU_MODE_MEASURE;
                else if (_strnicmp(s, "on", 2) == 0) m = DG_MENU_MODE_WRITE;
                InterlockedExchange(&g_menu_mode, m);
                bc->menu_mode = (int)m;
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_menu_confirm") == 0 ||
            _stricmp(key, "vr_menu_cancel") == 0) {

            unsigned int bit = 0;
            if      (_strnicmp(s, "cross",    5) == 0) bit = DG_MENU_PAD_B;
            else if (_strnicmp(s, "circle",   6) == 0) bit = DG_MENU_PAD_A;
            else if (_strnicmp(s, "triangle", 8) == 0) bit = DG_MENU_PAD_X;
            else if (_strnicmp(s, "square",   6) == 0) bit = DG_MENU_PAD_Y;
            else if (_strnicmp(s, "start",    5) == 0) bit = DG_MENU_PAD_STA;
            else if (_strnicmp(s, "select",   6) == 0) bit = DG_MENU_PAD_SEL;

            else if (_strnicmp(s, "sweep", 5) == 0 &&
                     _stricmp(key, "vr_menu_confirm") == 0) {
                /* Not a bit at all: walk the candidate table, one per
                   press. See g_menu_sweep. */
                /* Arm it, but only RESET the walk on the transition into
                   sweep. The marker is re-read about once a second, so
                   resetting here unconditionally restarted the table on
                   every poll: 34 confirms went out and only ever carried
                   the first two candidates, which reads exactly like "none
                   of them work". */
                if (!InterlockedCompareExchange(&g_menu_sweep_on, 0, 0)) {
                    InterlockedExchange(&g_menu_sweep_idx, 0);
                    InterlockedExchange(&g_menu_sweep_on, 1);
                }
                /* And a real bit, not the default. Leaving g_menu_confirm
                   on its default cross made it overlap the measured cancel
                   word (cross+EX3), config_ok refused the PAIR, and the
                   whole feature - navigation included - went silent with
                   bad-config on every frame. The table's own first entry
                   is valid by construction and overlaps nothing. */
                InterlockedExchange(&g_menu_confirm, (LONG)g_menu_sweep[0]);
            }
            else if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
                char *hend = NULL;
                unsigned long raw = strtoul(s + 2, &hend, 16);
                if (hend != s + 2 && raw && raw <= 0x00FFFFFFUL)
                    bit = (unsigned int)raw;
            }
            if (bit) {
                if (_stricmp(key, "vr_menu_confirm") == 0) {
                    /* Naming a button ENDS the sweep. Nothing else cleared
                       g_menu_sweep_on, so a sweep left running would have
                       gone on overwriting the confirm word once a second
                       and the named button would never have been sent -
                       silently, with the log still reading "sweep". That
                       is the third time this table has quietly eaten a
                       live run; it does not get a fourth. */
                    InterlockedExchange(&g_menu_sweep_on, 0);
                    InterlockedExchange(&g_menu_confirm, (LONG)bit);
                } else {
                    InterlockedExchange(&g_menu_cancel, (LONG)bit);
                }
            }
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_ui") == 0) {
            /* Off unless the word is `on`, like every other switch here. */
            bc->theater_ui = (_strnicmp(s, "on", 2) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_radar_wrist") == 0 || _stricmp(key, "vr_radar_hud") == 0 ||
            _stricmp(key, "vr_radar_wrist_space") == 0) {
            /* Words, matched whole: a typo is the default (off / on / grip),
               never a prefix of something else. */
            size_t len = strcspn(s, " \t\r\n");
            if (_stricmp(key, "vr_radar_wrist") == 0)
                xc->radar_mode = (len == 5 && _strnicmp(s, "fixed", 5) == 0) ? 1
                               : (len == 5 && _strnicmp(s, "wrist", 5) == 0) ? 2 : 0;
            else if (_stricmp(key, "vr_radar_hud") == 0)
                xc->radar_hud = (len == 3 && _strnicmp(s, "off", 3) == 0) ? 0 : 1;
            else
                xc->radar_space_local = (len == 5 && _strnicmp(s, "local", 5) == 0) ? 1 : 0;
            s += len;
            continue;
        }
        if (_stricmp(key, "vr_radar_wrist_offset") == 0 || _stricmp(key, "vr_radar_wrist_rot") == 0) {
            /* x,y,z - all three or the default stays: a half-read pose puts
               the quad somewhere nobody chose. Metres within +-0.30 of the
               grip, degrees within +-360. */
            int is_rot = _stricmp(key, "vr_radar_wrist_rot") == 0;
            double lim = is_rot ? 360.0 : 0.30, v3[3];
            char *e3 = s, *before;
            int i, good = 1;
            for (i = 0; i < 3 && good; i++) {
                if (i && *e3++ != ',') { good = 0; break; }
                before = e3;
                v3[i] = strtod(e3, &e3);
                if (e3 == before || !(v3[i] >= -lim && v3[i] <= lim)) good = 0;
            }
            if (good && (*e3 == 0 || *e3 == ' ' || *e3 == '\t' || *e3 == '\r' || *e3 == '\n'))
                for (i = 0; i < 3; i++) (is_rot ? xc->radar_rot : xc->radar_offset)[i] = v3[i];
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_life") == 0) {
            size_t len=strcspn(s," \t\r\n");
            if(len==2 && _strnicmp(s,"on",2)==0) xc->life_disabled=0;
            else if(len==3 && _strnicmp(s,"off",3)==0) xc->life_disabled=1;
            else return 0;
            s+=len; continue;
        }
        if (_stricmp(key, "vr_hud") == 0) {
            /* Reversed sense on purpose: the KNOB is "vr_hud", so `on` means
               the HUD as the game draws it (default, write nothing) and only
               the exact word `off` hides it. A typo therefore leaves the
               game's HUD alone rather than blanking it. */
            bc->hud_mode = (_strnicmp(s, "off", 3) == 0) ? 1 : 0;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "source") == 0) {
            size_t len = strcspn(s, " \t\r\n");
            if (len == 2 && _strnicmp(s, "xr", 2) == 0) *source = SRC_XR;
            else if (len == 6 && _strnicmp(s, "script", 6) == 0) *source = SRC_SCRIPT;
            else if (len == 9 && _strnicmp(s, "synthetic", 9) == 0) *source = SRC_SYNTHETIC;
            else return 0; /* Never silently accept script/xr suffix typos. */
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }

        v = strtod(s, &end);
        if (end == s) continue;
        s = end;

        if      (_stricmp(key, "yaw")      == 0) p->yaw   = v * DEG2RAD;
        else if (_stricmp(key, "pitch")    == 0) p->pitch = v * DEG2RAD;
        else if (_stricmp(key, "roll")     == 0) p->roll  = v * DEG2RAD;
        else if (_stricmp(key, "x")        == 0) p->tx    = v;
        else if (_stricmp(key, "y")        == 0) p->ty    = v;
        else if (_stricmp(key, "z")        == 0) p->tz    = v;
        else if (_stricmp(key, "sweep")    == 0) p->sweep_rad = v * DEG2RAD;
        else if (_stricmp(key, "sweep_hz") == 0) p->sweep_hz  = v;
        else if (_stricmp(key, "seconds")  == 0) { if (v > 0.0) *seconds = v; }
        else if (_stricmp(key, "xr_yaw_sign")   == 0) xc->yaw_sign   = v;
        else if (_stricmp(key, "xr_pitch_sign") == 0) xc->pitch_sign = v;
        else if (_stricmp(key, "xr_roll_sign")  == 0) xc->roll_sign  = v;
        else if (_stricmp(key, "xr_x_sign")     == 0) xc->x_sign     = v;
        else if (_stricmp(key, "xr_y_sign")     == 0) xc->y_sign     = v;
        else if (_stricmp(key, "xr_z_sign")     == 0) xc->z_sign     = v;
        else if (_stricmp(key, "xr_scale")      == 0) xc->scale      = v;
        else if (_stricmp(key, "xr_pos")        == 0) xc->positional = (v != 0.0);
        else if (_stricmp(key, "xr_recenter")   == 0) xc->recenter_token = (int)v;
        else if (_stricmp(key, "stereo")        == 0) xc->stereo = (v != 0.0);
        else if (_stricmp(key, "stereo_test")   == 0) xc->stereo_test = (int)v;
        else if (_stricmp(key, "vr_scene_blur_fix") == 0) xc->scene_blur_fix = v == 1.0;
        else if (_stricmp(key, "stereo_proj_x_sign") == 0)
            xc->stereo_proj_x_sign = (v < 0.0) ? -1 : 1;
        else if (_stricmp(key, "stereo_eye_x_sign") == 0)
            xc->stereo_eye_x_sign = (v < 0.0) ? -1 : 1;
        else if (_stricmp(key, "vr_ui2d") == 0) xc->ui2d = (v != 0.0);
        else if (_stricmp(key, "vr_eye_truth") == 0) { if (v == 0.0 || v == 1.0 || v == 2.0 || v == 3.0) xc->eye_truth = (int)v; }
        else if (_stricmp(key, "vr_feedback_skip") == 0) xc->feedback_skip = (v != 0.0);
        else if (_stricmp(key, "vr_draw_skip") == 0) { if (v == 0.0 || v == 1.0 || v == 2.0) xc->draw_skip = (int)v; }
        else if (_stricmp(key, "vr_ui2d_scale") == 0) { if (v >= 0.3 && v <= 1.0) xc->ui2d_scale_mils = (int)(v * 1000.0 + 0.5); }
        else if (_stricmp(key, "vr_ui2d_conv") == 0) { if (v >= 0.0 && v <= 0.1) xc->ui2d_conv_e5 = (int)(v * 100000.0 + 0.5); }
        else if (_stricmp(key, "vr_ui2d_sign") == 0) xc->ui2d_sign = (v < 0.0) ? -1 : 1;
        else if (_stricmp(key, "vr_ui2d_hold") == 0) { if (v == 0.0 || v == 1.0 || v == 2.0) xc->ui2d_hold = (int)v; }
        else if (_stricmp(key, "vr_trigger_deadzone") == 0)
            xc->trigger_deadzone = v;
        else if (_stricmp(key, "vr_trigger_fire") == 0)
            xc->trigger_fire = v;
        else if (_stricmp(key, "vr_theater_dist") == 0)
            xc->theater_dist = v;
        else if (_stricmp(key, "vr_theater_width") == 0)
            xc->theater_width = v;
        /* Out of range keeps the default, the theater's rule. */
        else if (_stricmp(key, "vr_radar_wrist_size") == 0) { if (v >= 0.03 && v <= 0.50) xc->radar_size = v; }
        else if (_stricmp(key, "vr_radar_gaze_deg") == 0) { if (v >= 0.0 && v <= 90.0) xc->radar_gaze_deg = v; }
        else if (_stricmp(key, "vr_radar_gaze_pitch") == 0) { if (v >= -45.0 && v <= 60.0) xc->radar_gaze_pitch = v; }
    }
    if (!(xc->trigger_deadzone >= 0.0 && xc->trigger_deadzone < 1.0 &&
          xc->trigger_fire > xc->trigger_deadzone &&
          xc->trigger_fire <= 1.0)) {
        xc->trigger_deadzone = 0.10;
        xc->trigger_fire = 0.55;
    }
    /* Out-of-range keeps the default rather than the nearest fit, so a wrong
       marker reads back as the default it got - the vr_move_deadzone rule.
       The XR side clamps identically, so the two can never disagree. */
    if (!(xc->theater_dist >= 0.5 && xc->theater_dist <= 10.0))
        xc->theater_dist = 2.0;
    if (!(xc->theater_width >= 0.5 && xc->theater_width <= 10.0))
        xc->theater_width = 2.0;
    if (g_script_menu_cfg && *source != SRC_SCRIPT) return 0;
    *arm_track_out = arm_track_cfg;
    memcpy(arm_qmap_out, arm_qmap_cfg, sizeof arm_qmap_cfg);
    *arm_hand_out = arm_hand_cfg;
    *arm_pos_out = ap;
    /* The anchor is the bridge's to honour, so it rides in the bridge config
       and is reported back by the heartbeat like every other setting the
       bridge can refuse. */
    bc->arm_anchor = ap.anchor;
    dg_aim_capture_configure(aim_probe_cfg || geometry_probe_cfg);
    dg_aim_capture_native_configure(geometry_probe_cfg);
    return 1;
}

/* Preserve the legacy file-loading contract; script requests use the stricter
   complete snapshot below and pass its identical bytes to parse_config. */
static int read_config(const char *marker, POSE *p, double *seconds,
                       int *source, DG_XR_CONFIG *xc, DG_BRIDGE_CONFIG *bc,
                       int *arm_track_out, int arm_qmap_out[4],
                       int *arm_hand_out, ARM_POS_CFG *arm_pos_out)
{
    HANDLE h;
    /* 4096, was 1024: on 2026-09-11 the marker grew past 1023 bytes and the
       last two lines (vr_left_arm, vr_twohand) silently fell off the end -
       the left hand "stopped working" for a whole evening. A file that fills
       the buffer is now called out in the log instead of trimmed quietly. */
    char buf[4096];
    DWORD got = 0;
    dg_aim_capture_configure(0);dg_aim_capture_native_configure(0);
    h = CreateFileA(marker, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL,
                    OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return 0;
    if (!ReadFile(h, buf, sizeof(buf) - 1, &got, NULL)) {
        CloseHandle(h);
        return 0;
    }
    CloseHandle(h);
    buf[got] = 0;
    if (got == sizeof(buf) - 1) {
        static LONG warned;
        if (!InterlockedExchange(&warned, 1))
            logf_("  marker: file is %u bytes or longer - the tail is IGNORED, "
                  "shorten it\r\n", (unsigned)sizeof(buf) - 1);
    }
    return parse_config(buf, p, seconds, source, xc, bc, arm_track_out,
                        arm_qmap_out, arm_hand_out, arm_pos_out);
}

static int camera_live(void) {
    return finite_mat(M(O_EYE_INV)) && rows_orthonormal(M(O_EYE_INV))
        && finite_mat(M(O_PERS))    && projlike(M(O_PERS));
}

/* ---------------------------------------------------------------- main --- */

static int marker_present(const char *marker) {
    return GetFileAttributesA(marker) != INVALID_FILE_ATTRIBUTES;
}

typedef struct {
    DG_SCRIPT_MARKER_INFO info;
    DWORD size;
    char text[DG_SCRIPT_MARKER_MAX_BYTES + 1];
} SCRIPT_MARKER_SNAPSHOT;

/* Deny concurrent writes while reading; never accept a prefix of a marker.
   Parsing the control surface has no runtime/configuration side effects. */
static int read_script_snapshot(const char *path, SCRIPT_MARKER_SNAPSHOT *out)
{
    HANDLE h;
    LARGE_INTEGER size;
    DWORD got = 0;
    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    if (!path) return 0;
    h = CreateFileA(path, GENERIC_READ, FILE_SHARE_READ,
                    NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return 0;
    if (!GetFileSizeEx(h, &size) || size.QuadPart <= 0 ||
        size.QuadPart > DG_SCRIPT_MARKER_MAX_BYTES ||
        !ReadFile(h, out->text, (DWORD)size.QuadPart, &got, NULL) ||
        got != (DWORD)size.QuadPart) {
        CloseHandle(h);
        return 0;
    }
    CloseHandle(h);
    out->size = got;
    return dg_script_marker_parse(out->text, got, &out->info);
}

static int read_script_marker(const char *path, DG_SCRIPT_MARKER_INFO *info)
{
    SCRIPT_MARKER_SNAPSHOT snap;
    int ok = read_script_snapshot(path, &snap);
    *info = snap.info;
    return ok;
}

static int script_snapshot_equal(const SCRIPT_MARKER_SNAPSHOT *a,
                                 const SCRIPT_MARKER_SNAPSHOT *b)
{
    return a->size == b->size && !memcmp(a->text, b->text, a->size);
}

/* Observe before comparing the full config so a superseding busy command is
   consumed even when it also changes settings. No scenario actions are queued. */
static int script_request_current(const char *marker,
                                   const SCRIPT_MARKER_SNAPSHOT *config)
{
    SCRIPT_MARKER_SNAPSHOT now;
    int ok = read_script_snapshot(marker, &now);
    int same = dg_script_gate_observe(&g_script_gate,
                    ok && now.info.source_script, ok && now.info.token_valid,
                    now.info.token);
    return same && (!config || script_snapshot_equal(config, &now));
}

/* The bridge counters used to reach the log only on a clean disarm. That is the
   one exit nobody takes: the natural way to end a session is to quit the game,
   which kills the process with the counters still in it. It cost the F2 live
   gate its edge and restore totals twice - the transitions were all there, but
   only as a list to count by hand. So it is emitted on a heartbeat as well, and
   labelled with which one it is. */
/* Just enough of the VK table to name a key in the log. Anything unnamed still
   prints its hex, which is enough to look up. The point is to stop asking the
   player to press a button nobody in this conversation can name. */
static const char *vk_name(int vk)
{
    static char buf[8];
    switch (vk) {
    case VK_LBUTTON:  return "Mouse Left";
    case VK_RBUTTON:  return "Mouse Right";
    case VK_MBUTTON:  return "Mouse Middle";
    case VK_XBUTTON1: return "Mouse 4";
    case VK_XBUTTON2: return "Mouse 5";
    case VK_SPACE:    return "Space";
    case VK_RETURN:   return "Enter";
    case VK_TAB:      return "Tab";
    case VK_ESCAPE:   return "Esc";
    case VK_LSHIFT:   return "Left Shift";
    case VK_RSHIFT:   return "Right Shift";
    case VK_LCONTROL: return "Left Ctrl";
    case VK_RCONTROL: return "Right Ctrl";
    case VK_LMENU:    return "Left Alt";
    case VK_RMENU:    return "Right Alt";
    default: break;
    }
    if ((vk >= '0' && vk <= '9') || (vk >= 'A' && vk <= 'Z')) {
        buf[0] = (char)vk; buf[1] = 0;
        return buf;
    }
    return "?";
}

/* The actuator probe's drain: one line per captured sample, the six deciders
   of FPS_AIM_ACTUATOR_ONDERZOEK.md par. 6. flag3 set = H-A's windowed
   regime; turn.vy moving while rot.vy stays = H-C's re-stomper; neither
   moving = the write died at intake. Under vr_turn_probe=dense the ring also
   holds one sample per aim tick without a write ("idle"), so rot.vy chasing
   turn.vy is a series rather than a point. Called from the 1 Hz worker pass
   as well as the heartbeat: a 256-slot ring at ~30 ticks/s would overflow
   long before a 30 s heartbeat came round. */
static void drain_turn_probe(void)
{
    DG_TURN_PROBE_SAMPLE ps;
    while (dg_bridge_turn_probe_take(&ps)) {
        if (ps.pw_ok)
            logf_("  turn probe: tick %ld byte %u%s fire %ld"
                  "  flag3 %d homing %d  turn.vy %+d rot.vy %+d"
                  "  camdir.vy %+d pad %+d  act %llX/%llX"
                  "  gap %+.1f drift %+.1f head %+.1f tilt %.1f age %ld\r\n",
                  ps.tick, (unsigned)ps.byte, ps.wrote ? "" : " idle",
                  ps.fire_state,
                  (int)((ps.flags >> 3) & 1),
                  (int)((ps.flags >> 45) & 1),
                  (int)ps.turn_vy, (int)ps.rot_vy,
                  (int)ps.cam_vy, (int)ps.cam_pad,
                  ps.action, ps.action2,
                  (double)ps.gap_deg, (double)ps.drift_deg,
                  (double)ps.head_deg, (double)ps.root_tilt_deg,
                  ps.gap_age);
        else
            logf_("  turn probe: tick %ld byte %u%s fire %ld"
                  "  player work UNREADABLE\r\n",
                  ps.tick, (unsigned)ps.byte, ps.wrote ? "" : " idle",
                  ps.fire_state);
    }
}

/* The adjust probe's per-cycle drain (Meetplan A', ROLL V5.1 par. 3): one
   line per closed identity/X/Y/Z cycle, the world heading of adjust-X beside
   rot.vy, with the cycle's own quality stamp so the desk can drop cycles
   read while the body moved instead of averaging them in. 1 Hz like the
   turn probe: a cycle every 4 x DG_ADJ_SETTLE_TICKS would fill a 128-slot
   ring long before a 30 s heartbeat. */
static void drain_adj_cycles(void)
{
    DG_ADJ_CYCLE cy;
    while (dg_bridge_adj_cycle_take(&cy))
        logf_("  adj cycle: tick %ld ok %d rot.vy %+d/%+d  yaw %.2f"
              "  ang %.2f/%.2f/%.2f  ydot %.4f\r\n",
              cy.tick, (int)cy.ok, (int)cy.rot0, (int)cy.rot3,
              (double)cy.yaw_deg, (double)cy.ang_deg[0],
              (double)cy.ang_deg[1], (double)cy.ang_deg[2],
              (double)cy.ydot);
}

static void log_bridge_summary(const char *when) {
    DG_BRIDGE_STATS bs;
    dg_bridge_stats(&bs);
    /* Wall clock and tick on every heartbeat. Run 15 (2026-09-05) had to be
       timed by counting stale ticks backwards from a file's mtime; the
       tick matches the stamps the XR session-state lines print. */
    {   SYSTEMTIME st;
        GetLocalTime(&st);
        logf_("  --- heartbeat %02d:%02d:%02d (tick %llu ms) ---\r\n",
              (int)st.wHour, (int)st.wMinute, (int)st.wSecond,
              (unsigned long long)GetTickCount64());
    }
    /* Split, for the reason the arm block was split: one call per group means a
       line added later cannot silently push the tail of the block off the end
       of the logger's buffer. */
    logf_("  bridge (%s) ticks %ld  requested %ld  entered %ld  left %ld"
          "  transitions %ld  edges %ld  writes %ld\r\n"
          "  bridge suspends: mask-zero %ld  level-load %ld  unsafe %ld"
          "  timeouts %ld  dropped %ld\r\n"
          "  bridge state %s(%s)  native active %d  mgshdfix owner %d"
          "  restored %d\r\n",
          when, bs.ticks, bs.fps_requested, bs.fps_entered, bs.fps_left,
          bs.fps_transitions, bs.fps_edges_injected, bs.fps_writes_applied,
          bs.fps_refused_mask_zero, bs.fps_suspend_level_load,
          bs.fps_suspend_unsafe, bs.fps_timeouts, bs.fps_events_dropped,
          dg_fps_state_name(bs.fps_state),
          dg_fps_reason_name(bs.fps_suspend_reason),
          bs.fps_native_active, bs.mgshdfix_fps_owner, bs.restored);
    logf_("  bridge values: Override %d  Toggle %d  Move %d"
          "  PL_SubjectMove %d (set on %ld ticks)  PL_SubjectToggle %d\r\n"
          "  bridge masks: PL_PAD_SUBJECT 0x%08X  PL_PAD_STOP_AIM 0x%08X\r\n"
          "  bridge status: player 0x%016llX  game 0x%08X  menu 0x%08X\r\n",
          bs.override_value, bs.toggle_value, bs.move_value,
          bs.subject_move_value, bs.subject_move_ticks, bs.subject_toggle_value,
          bs.pad_subject_mask, bs.pad_stop_aim_mask,
          bs.player_status, bs.game_status, bs.menu_status);
    /* F4. The flag word is decoded rather than only printed, because "0x3000"
       is not something anyone should have to decode at 2am: 0x1000 is
       DG_FLAG_INVISIBLE0, and channel n shifts it left by n. Two channels are
       named because that is what the retail hide does (or 0x3000), and the
       stereo question - whether both eyes agree - is exactly whether these two
       ever differ. */
    /* Observed, never written. Degrees are printed beside the raw units
       because 4096-to-a-turn is not a number anyone reads at a glance, and
       whether these are small angles is the whole question. */
    /* Printed unconditionally, all-zero included, because a gate that never
       fired and a gate that never ran must not produce the same line. The
       camera-seam refusals are the interesting half: the tick seam runs at a
       fifth of its normal rate during a codec or a cutscene, so the suspends
       counted on the line above can be zero while the writer stood down
       hundreds of times on this seam's own reading. */
    logf_("  bridge gate: camera-seam status samples %ld (latch stale %ld)"
          "  writer stood down on its own reading %ld times" DG_EOL,
          bs.late_status_samples, bs.late_status_stale,
          bs.seam_refused_unsafe);
    /* The theater, phase 0's instrument: the per-bit occupancy plus the flank
       tallies are what one headset evening needs to prove the mask's
       semantics per moment. Printed unconditionally and zeros included -
       samples is the denominator that separates "never happened" from "never
       ran", the same contract as the gate line above. */
    logf_("  theater: mode %s%s  verdict %d  samples %ld (theater-masked %ld,"
          "  ui-masked %ld)" DG_EOL,
          bs.thea_mode == DG_THEATER_ON ? "on"
              : bs.thea_mode == DG_THEATER_MEASURE ? "measure" : "off",
          bs.thea_ui_on ? " +ui" : "", bs.thea_verdict, bs.thea_samples,
          bs.thea_masked, bs.thea_ui_masked);
    logf_("  theater flanks: enter %ld exit %ld (ui %ld/%ld)  hysteresis held"
          " enter %ld exit %ld" DG_EOL,
          bs.thea_enter, bs.thea_exit, bs.thea_ui_enter, bs.thea_ui_exit,
          bs.thea_enter_held, bs.thea_exit_held);
    logf_("  theater bits: DEMO %ld  SCN_DEMO %ld  PAD_DEMO %ld  RADIO %ld"
          "  WEAPON_OPEN %ld  ITEM_OPEN %ld" DG_EOL,
          bs.thea_bit_demo, bs.thea_bit_scn, bs.thea_bit_pad,
          bs.thea_bit_radio, bs.thea_bit_weapon, bs.thea_bit_item);
    /* The flat fallback's own line: how long the camera hook has been
       silent, and how many Presents that put on the quad. A title screen
       that stays black now says which of the two halves failed - never
       silent (the camera IS handing over) or silent but never shown (the
       mode is off, or stereo/armed is not what it looks like). */
    logf_("  screen fallback: mode %s  camera silent %ld frames"
          "  shown %ld" DG_EOL,
          InterlockedCompareExchange(&g_flat_mode, 0, 0) ? "on" : "off",
          InterlockedCompareExchange(&g_flat_silent, 0, 0),
          InterlockedCompareExchange(&g_flat_frames, 0, 0));
    /* U2's own line, printed only when the feature was switched on so the
       heartbeat does not grow a row of zeros for every off feature.
       What it has to show before anyone writes a byte to the game: `open`
       climbing on the title screen (the front-end detector works at all), then
       `steps` and `confirm` climbing while the stick and trigger move (the
       whole controller-to-pad-word chain works), with `no-input` flat (the XR
       frame is being believed) and `bad-config` at zero. `nowhere` equals the
       number of decisions that had no route into the game, which in this build
       is all of them - see the vr_menu block near g_flat_mode. A run where
       `open` climbs and `steps` never does means the stick is not being read;
       one where neither climbs means the screen was never judged flat, which
       is a vr_flat problem and not a vr_menu one. */
    if (InterlockedCompareExchange(&g_menu_mode, 0, 0)) {
        /* The anchor's own state is NOT on this line, and that is a gap with a
           name: DG_ANCHORS.pad_press_ok lives inside dg_bridge's private
           resolve and nothing publishes it. One field on DG_BRIDGE_STATS and
           three lines in dg_bridge_stats() would put it here; until then the
           offline scanner is where to read it (dg_f1b --image, the
           "[GV_PadPress scenario-press array]" block). Saying so beats a line
           that quietly prints nothing. */
        /* The write route exists now (dg_bridge's pad detour), so this line
           reports the whole chain instead of explaining its absence: the
           OPTIONAL anchor, whether the detour actually installed, and what
           the seam did with what it was handed. A live detour with zero
           writes and a rising gate count is a different story from a
           detour that never installed, and both are different from a
           marker that was never switched on. */
        logf_("  menu: %s  anchor %s  detour %s  confirm 0x%04X"
              "  cancel 0x%04X  %s stick" DG_EOL,
              (InterlockedCompareExchange(&g_menu_mode, 0, 0) ==
                   DG_MENU_MODE_WRITE) ? "on (writing)" : "measure",
              bs.menu_anchor_ok ? "ok" : "NOT FOUND",
              bs.menu_detour_live ? "live" : "not installed",
              (unsigned int)InterlockedCompareExchange(&g_menu_confirm, 0, 0),
              (unsigned int)InterlockedCompareExchange(&g_menu_cancel, 0, 0),
              InterlockedCompareExchange(&g_menu_hand_right, 0, 0)
                  ? "right" : "left");
        if (InterlockedCompareExchange(&g_menu_sweep_on, 0, 0))
            logf_("  menu SWEEP: %ld confirms sent, last bit 0x%08X"
                  "  (next 0x%08X of %d)" DG_EOL,
                  InterlockedCompareExchange(&g_menu_sweep_idx, 0, 0),
                  (unsigned int)InterlockedCompareExchange(
                      &g_menu_sweep_last, 0, 0),
                  g_menu_sweep[InterlockedCompareExchange(
                      &g_menu_sweep_idx, 0, 0) % DG_MENU_SWEEP_N],
                  DG_MENU_SWEEP_N);
        {
            union { float f; LONG l; } tv, cv;
            LONG b = InterlockedCompareExchange(&g_menu_dbg_buttons, 0, 0);
            tv.l = InterlockedCompareExchange(&g_menu_dbg_trigger, 0, 0);
            cv.l = InterlockedCompareExchange(&g_menu_dbg_click, 0, 0);
            logf_("  menu input seen: trigger %.3f (click %.3f)  A %d  B %d"
                  "  vouched %d  over %ld samples" DG_EOL,
                  (double)tv.f, (double)cv.f, (b & 1) ? 1 : 0,
                  (b & 2) ? 1 : 0, (b & 4) ? 1 : 0,
                  InterlockedCompareExchange(&g_menu_dbg_samples, 0, 0));
        }
        {
            /* The ten candidate assignment globals, live. Whichever holds
               0x00040040 is the CANCEL slot; the other half of that pair
               is the confirm word. See g_pad_asgn_rva. */
            unsigned char *mod = (unsigned char *)GetModuleHandleA(NULL);
            MEMORY_BASIC_INFORMATION mbi;
            int gi;
            for (gi = 0; gi < DG_PAD_ASGN_N; gi += 2) {
                DWORD a = 0, b = 0;
                unsigned char *pa = mod + g_pad_asgn_rva[gi];
                unsigned char *pb = mod + g_pad_asgn_rva[gi + 1];
                if (VirtualQuery(pa, &mbi, sizeof mbi) &&
                    mbi.State == MEM_COMMIT)
                    a = *(volatile DWORD *)pa;
                if (VirtualQuery(pb, &mbi, sizeof mbi) &&
                    mbi.State == MEM_COMMIT)
                    b = *(volatile DWORD *)pb;
                logf_("  pad assign candidate %d: [+0x%08X]=0x%08X"
                      "  [+0x%08X]=0x%08X%s" DG_EOL,
                      gi / 2, g_pad_asgn_rva[gi], a,
                      g_pad_asgn_rva[gi + 1], b,
                      (a == 0x00040040u || b == 0x00040040u)
                          ? "   <- THIS PAIR (holds the measured cancel)"
                          : "");
            }
        }
        logf_("  menu clear mask in force: 0x%08X%s" DG_EOL,
              (unsigned int)InterlockedCompareExchange(&g_menu_clear_bits,
                                                       0, 0),
              InterlockedCompareExchange(&g_menu_clear_bits, 0, 0)
                  ? "" : "   <- nothing is being silenced");
        logf_("  menu seam: writes %ld  refused stale %ld  refused gate %ld"
              "  asked %ld" DG_EOL,
              bs.menu_writes, bs.menu_refused_stale, bs.menu_refused_gate,
              InterlockedCompareExchange(&g_menu_writes_asked, 0, 0));

        logf_("  menu observed (game's own record): status 0x%08X"
              "  press 0x%08X  [we send confirm 0x%04X]" DG_EOL,
              bs.menu_seen_status, bs.menu_seen_press,
              (unsigned int)InterlockedCompareExchange(&g_menu_confirm, 0, 0));
        {

            int pk;
            for (pk = 0; pk < DG_PAD_PRESS_RING; pk++)
                if (bs.menu_press_word[pk])
                    logf_("  menu press seen: 0x%08X  x%ld" DG_EOL,
                          bs.menu_press_word[pk], bs.menu_press_count[pk]);
            /* The same presses in ORDER, which is the line that can actually
               name a button. A histogram over a thirty-second window only
               names one if the window can be lined up with the presses by
               hand, and that alignment is what went wrong last time: the
               words were read, a guess was made about which was confirm, and
               it was the wrong half of the pair. An ordered list needs no
               alignment - "the last three were the confirm key" reads
               straight off it. The oldest are dropped, not the newest. */
            if (bs.menu_press_seq_n > 0) {
                long total = bs.menu_press_seq_n;
                long kept = total < DG_PAD_PRESS_SEQ
                                ? total : DG_PAD_PRESS_SEQ;
                long first = total - kept;
                long i;
                logf_("  menu press ORDER (%ld press%s, showing the last %ld,"
                      " oldest first):" DG_EOL,
                      total, (total == 1) ? "" : "es", kept);
                for (i = first; i < total; i++) {
                    long slot = i % DG_PAD_PRESS_SEQ;
                    logf_("    #%ld  0x%08X  at %ld ms" DG_EOL,
                          i + 1, bs.menu_press_seq[slot],
                          bs.menu_press_seq_ms[slot]);
                }
            }
        }
        logf_("  menu counts: open %ld  steps %ld (repeat %ld)  confirm %ld"
              "  cancel %ld  yielded %ld  no-input %ld  bad-config %ld"
              "  nowhere-to-write %ld%s" DG_EOL,
              InterlockedCompareExchange(&g_menu_open_frames, 0, 0),
              InterlockedCompareExchange(&g_menu_steps, 0, 0),
              InterlockedCompareExchange(&g_menu_repeats, 0, 0),
              InterlockedCompareExchange(&g_menu_confirms, 0, 0),
              InterlockedCompareExchange(&g_menu_cancels, 0, 0),
              InterlockedCompareExchange(&g_menu_yields, 0, 0),
              InterlockedCompareExchange(&g_menu_no_input, 0, 0),
              InterlockedCompareExchange(&g_menu_bad_config, 0, 0),
              InterlockedCompareExchange(&g_menu_unwritable, 0, 0),
              (InterlockedCompareExchange(&g_menu_open_frames, 0, 0) &&
               !InterlockedCompareExchange(&g_menu_steps, 0, 0))
                  ? "   <- a screen was up and the stick moved nothing"
                  : "");
    }
    logf_("  pad seam: entries %ld  START queued %ld dequeued %ld"
          "  context refused %ld" DG_EOL,
          bs.pad_seam_entries, bs.start_queued, bs.start_consumed,
          bs.pad_context_refused);
    logf_("  theater consumers: camera released %ld frames  mono captures %ld"
          "  quad frames %ld;  hud %s (writes %ld, cleared %ld)" DG_EOL,
          InterlockedCompareExchange(&g_thea_cam_released, 0, 0),
          InterlockedCompareExchange(&g_thea_mono_frames, 0, 0),
          dg_xr_screen_frames(),
          bs.hud_hide ? "HIDDEN" : "native", bs.hud_writes, bs.hud_cleared);
    logf_("  bridge hand: ArmCamRotateShift 0x%016llX  = [%d %d %d]"
          " = [%.2f %.2f %.2f] deg%s" DG_EOL,
          bs.arm_cam_rotate_shift,
          bs.arm_cam_rot[0], bs.arm_cam_rot[1], bs.arm_cam_rot[2],
          bs.arm_cam_rot[0] * 360.0 / 4096.0,
          bs.arm_cam_rot[1] * 360.0 / 4096.0,
          bs.arm_cam_rot[2] * 360.0 / 4096.0,
          bs.arm_cam_rot[1] ? "" : "  (vy 0: SetPos takes the 2-DOF XAfterY"
                                   " branch)");
    /* Printed whenever the probe wrote anything, including the case where it
       wrote and never got to read - "writes 240, reads 0" says the camera seam
       never ran, which is a completely different finding from a bad match and
       must not look like one. */
    if (bs.hand_probe_writes || bs.hand_probe_reads) {
        logf_("  bridge hand probe: wrote [%d %d %d]  read back [%d %d %d]"
              "  (writes %ld, reads %ld)" DG_EOL,
              bs.hand_probe_wrote[0], bs.hand_probe_wrote[1],
              bs.hand_probe_wrote[2], bs.hand_probe_read[0],
              bs.hand_probe_read[1], bs.hand_probe_read[2],
              bs.hand_probe_writes, bs.hand_probe_reads);
        logf_("  bridge hand probe: adjust[6] %.6f %.6f %.6f %.6f  predicted"
              " %.6f %.6f %.6f %.6f  worst |diff| %.6f  no-SetPos %ld" DG_EOL,
              bs.hand_probe_adjust6[0], bs.hand_probe_adjust6[1],
              bs.hand_probe_adjust6[2], bs.hand_probe_adjust6[3],
              bs.hand_probe_predicted[0], bs.hand_probe_predicted[1],
              bs.hand_probe_predicted[2], bs.hand_probe_predicted[3],
              bs.hand_probe_worst_diff, bs.hand_probe_no_setpos);
    }
    logf_("  bridge arm: GM_PlayerArmBody 0x%016llX  objs 0x%016llX"
          "  flag 0x%08X [ch0 %s, ch1 %s, ch2 %s, ch3 %s]"
          "  created %ld  destroyed %ld\r\n",
          bs.arm_body, bs.arm_objs, bs.arm_flag,
          (bs.arm_flag & 0x1000) ? "hidden" : "SHOWN",
          (bs.arm_flag & 0x2000) ? "hidden" : "SHOWN",
          (bs.arm_flag & 0x4000) ? "hidden" : "SHOWN",
          (bs.arm_flag & 0x8000) ? "hidden" : "SHOWN",
          bs.arm_created, bs.arm_destroyed);
    /* The line the previous run needed and did not have. "seen" alone cannot
       distinguish "hidden the whole time" from "shown only while a button was
       held", because an OR accumulator has no way to record a bit going clear.
       visible-ticks can, and it is the number to read first. */
    logf_("  bridge arm: visible on %ld of %ld ticks at the TICK seam"
          " (or 0x%08X, and 0x%08X)\r\n"
          "  bridge arm: visible on %ld ticks at the CAMERA seam, flag 0x%08X"
          " (or 0x%08X, and 0x%08X)  <- the one the frame is drawn with\r\n",
          bs.arm_visible_ticks, bs.ticks, bs.seen_arm_flag, bs.held_arm_flag,
          bs.arm_visible_late_ticks, bs.arm_flag_late, bs.seen_arm_flag_late,
          bs.held_arm_flag_late);
    /* Printed next to the arm on purpose. One object hidden and the other shown
       in the same frame is the whole finding; either flag alone says nothing
       about what is on screen.
       No verdict without a reading, either: the probe this replaced printed
       "ch0 SHOWN" off a flag word it had never read, because objs was null and
       0 & 0x1000 is 0. Nothing here prints unless the Work resolved. */
    if (bs.arm_work) {
        logf_("  bridge armwork: Work 0x%016llX  arm camera on %d"
              " (set on %ld of %ld ticks)  arm_trigger 0x%08X"
              " (invisible %s, seen 0x%08X)\r\n",
              bs.arm_work, bs.arm_camera_on, bs.arm_camera_on_ticks, bs.ticks,
              bs.arm_trigger, (bs.arm_trigger & 1) ? "SET" : "clear",
              bs.seen_arm_trigger);
        logf_("  bridge armwork: PlayerWork 0x%016llX (derived, trigger-0xCF4)"
              "  pbody 0x%016llX  objs 0x%016llX\r\n",
              bs.player_work, bs.body_cand, bs.body_objs);
        if (bs.body_objs)
            logf_("  bridge armwork: player body flag 0x%08X [ch0 %s]"
                  "  visible on %ld ticks (or 0x%08X, and 0x%08X)\r\n",
                  bs.body_flag, (bs.body_flag & 0x1000) ? "hidden" : "SHOWN",
                  bs.body_visible_ticks, bs.seen_body_flag, bs.held_body_flag);
    }
    /* F5. n_joints is printed even with the bend off, because 21 is what says
       the struct reading is right, and it is the gate every write is behind. */
    /* Printed whenever a bend is configured, not only when it got far enough
       to read a joint count: "writes 0" is the finding when a gate refuses. */
    if (bs.arm_bend_deg || bs.arm_joints || bs.arm_adjust)
        logf_("  bridge bend: n_joints %d (55 on the arm rig)  adjust 0x%016llX"
              "  joint %d  angle %d deg  writes %ld\r\n",
              bs.arm_joints, bs.arm_adjust, bs.arm_bend_joint,
              bs.arm_bend_deg, bs.arm_bend_writes);
    logf_("  bridge IK: writes %ld  refused %ld (implausible %ld)  clamped %ld"
          "  weight %.3f  pose-flags 0x%02X;"
          " pose lost %ld, stale %ld" DG_EOL,
          bs.arm_track_writes, bs.arm_ik_refused, bs.arm_ik_implausible,
          bs.arm_ik_clamped, bs.arm_ik_weight, bs.arm_pose_flags,
          InterlockedCompareExchange(&g_arm_pose_untracked, 0, 0),
          InterlockedCompareExchange(&g_arm_pose_stale, 0, 0));
    if (g_arm_raw_have)
        logf_("  bridge IK: controller as OpenXR gave it, relative to the live "
              "head: [%.3f %.3f %.3f] m  (+X right, +Y up, -Z forward)" DG_EOL,
              g_arm_raw_rel[0], g_arm_raw_rel[1], g_arm_raw_rel[2]);
    logf_("  bridge IK: view [%.2f %.2f %.2f]  root [%.2f %.2f %.2f]"
          "  target [%.2f %.2f %.2f]  root-distance %.2f / limit %.2f mm"
          DG_EOL,
          bs.arm_ik_view[0], bs.arm_ik_view[1], bs.arm_ik_view[2],
          bs.arm_ik_root[0], bs.arm_ik_root[1], bs.arm_ik_root[2],
          bs.arm_ik_target[0], bs.arm_ik_target[1], bs.arm_ik_target[2],
          bs.arm_ik_root_distance, bs.arm_ik_root_limit);
    logf_("  bridge arm-map pairs: seen %ld  eligible %ld  accepted %ld"
          "  refused %ld  calibration/settle %ld  replays %ld" DG_EOL,
          bs.arm_pairs_seen, bs.arm_pairs_eligible, bs.arm_pairs_accepted,
          bs.arm_pairs_refused, bs.arm_pairs_calibration,
          bs.arm_pair_replays);
    logf_("  bridge left: mode %d accepted %ld refused %ld support-pairs %ld reason %d"
          " blend %.3f target [%.2f %.2f %.2f]" DG_EOL,
          bs.left_status,bs.left_pairs_accepted,bs.left_pairs_refused,
          bs.left_support_pairs,bs.left_refuse_reason,bs.left_blend,
          bs.left_target[0],bs.left_target[1],bs.left_target[2]);
    logf_("  bridge support gates: aim/input %ld right-commit %ld anchor %ld"
          " unreachable %ld distance %ld dwell %ld attached %ld" DG_EOL,
          bs.left_support_gate[0],bs.left_support_gate[1],bs.left_support_gate[2],
          bs.left_support_gate[3],bs.left_support_gate[4],bs.left_support_gate[5],
          bs.left_support_gate[6]);
    logf_("  bridge arm-map: stream %lu pair %lu flags 0x%02X"
          "  scale %.4f source-span %.2f reach %.2f mm  map-clamped %ld"
          "  soft-zone %ld  owner-mismatch %ld" DG_EOL,
          bs.arm_map_stream, bs.arm_map_pair, bs.arm_map_flags,
          bs.arm_map_scale, bs.arm_map_source_span, bs.arm_map_reach,
          bs.arm_pairs_map_clamped, bs.arm_pairs_map_soft,
          bs.arm_release_owner_mismatch);
    logf_("  bridge arm-map: delta [%.2f %.2f %.2f]  target-view"
          " [%.2f %.2f %.2f]" DG_EOL,
          bs.arm_map_delta[0], bs.arm_map_delta[1], bs.arm_map_delta[2],
          bs.arm_map_target[0], bs.arm_map_target[1], bs.arm_map_target[2]);
    /* The arm-root basis, read off the character instead of assumed. The
       shoulder and the animated wrist are two points whose real-world relation
       everyone already knows - a hanging arm puts its wrist below its shoulder
       and outboard of it - so their difference names the axes. This line is
       what settled vr_arm_pos_sign, and it is the first place to look whenever
       the arm goes somewhere unexpected. */
    logf_("  bridge arm-basis: frame %s @ %+.1f deg (freeze %ld,"
          " recentres %ld)  anchor %s  hand-zero %ld (B pressed %lu, ignored %lu: no trigger)"
          "  character shoulder [%.1f %.1f %.1f]"
          "  animated wrist [%.1f %.1f %.1f] mm"
          "  body-drift %+.1f deg (uncompensated %ld)  comp %s (no-stick %ld)" DG_EOL,
          arm_frame_running(g_arm_frame),
          atan2(g_arm_frame_q[1], g_arm_frame_q[3]) * 2.0 / DEG2RAD,
          g_arm_frame_freezes, input_recenter_count(),
          bs.arm_anchor ? "shoulder" : "wrist",
          InterlockedCompareExchange(&g_arm_hand_zero_now, 0, 0),
          input_secondary_presses(arm_tracked_xr_hand(g_arm_track)),
          input_secondary_ignored(arm_tracked_xr_hand(g_arm_track)),
          bs.arm_shoulder_view[0], bs.arm_shoulder_view[1],
          bs.arm_shoulder_view[2], bs.arm_native_wrist_view[0],
          bs.arm_native_wrist_view[1], bs.arm_native_wrist_view[2],
          bs.arm_body_drift_deg, bs.arm_body_uncompensated,
          bs.arm_comp ? "organic" : "full", bs.comp_no_stick);
    /* One line answers "will there be a recording": frames only move when
       capture is on and XR frames are arriving, dup says the seam is being
       read faster than XR publishes (expected), and a failed dump is the
       only way this feature can silently not exist at diagnosis time. */
    logf_("  rec: frames %ld (seen %ld, dup %ld)  dumps %ld"
          " (failed %ld, torn %ld)  live snapshots %ld (failed %ld)" DG_EOL,
          g_rec.appended, g_rec.seen, g_rec.dup,
          InterlockedCompareExchange(&g_rec_dumps, 0, 0),
          InterlockedCompareExchange(&g_rec_dump_fail, 0, 0),
          g_rec_last_torn,
          InterlockedCompareExchange(&g_rec_snapshots, 0, 0),
          InterlockedCompareExchange(&g_rec_snapshot_fail, 0, 0));
    /* The residual is the whole point of this line. It is the angle between
       what the previous pair asked joint 6 to become and what the next
       hierarchy pass actually produced, measured rather than assumed. A few
       degrees is the frame of lag every write here has; tens of degrees means
       the composition model is wrong, and no amount of smoothing will fix
       that. Zero samples with writes > 0 means nothing was ever checked. */
    logf_("  bridge hand: %s  basis %s  published %ld  refused %ld  tick-writes %ld"
          "  residual %.2f deg (worst %.2f over %ld samples)" DG_EOL,
          InterlockedCompareExchange(&g_arm_hand, 0, 0) ? "on" : "off",
          bs.arm_hand_basis_world ? "world" : "root",
          bs.arm_hand_written, bs.arm_hand_refused, bs.arm_hand_tick_writes,
          bs.arm_hand_residual_deg, bs.arm_hand_worst_deg,
          bs.arm_hand_measured);
    /* The residual cut at the SetPos slot instead of the hierarchy. The
       matrix a bad residual leaves open: dirty echo = the game is not
       reconstructing our published angles (conversion/pull/another writer);
       clean echo + bad residual = the slot is ours but the hierarchy does
       something else with it (adjust application, hand part frame). */
    logf_("  bridge hand slot-echo: %.2f deg (worst %.2f, dirty %ld)"
          "  offset-drift %.2f deg (worst %.2f)  adjust-bits lost %ld" DG_EOL,
          bs.arm_hand_slot_echo_deg, bs.arm_hand_slot_echo_worst_deg,
          bs.arm_hand_slot_dirty,
          bs.arm_hand_off_drift_deg, bs.arm_hand_off_drift_worst_deg,
          bs.arm_adjust_bits_lost);
    /* The SVECTOR channel (run 13): past `reach` units per component the
       engine folds the precompensated short and the hand flips 90 degrees.
       alt-named = renamed to the second ZYX solution (same rotation);
       scaled = shortened along its axis, with the fraction kept. Scaled
       pairs are a shortfall the player can see; a growing count says the
       hand is being asked further from its animation than the channel
       carries in one pull. */
    logf_("  bridge hand channel: reach %ld units  alt-named %ld  scaled %ld"
          "  fraction %.3f (worst shortfall %.3f)" DG_EOL,
          bs.arm_hand_reach_units, bs.arm_hand_alt_named, bs.arm_hand_scaled,
          bs.arm_hand_fit_frac, bs.arm_hand_shortfall_worst);
    logf_("  bridge hand rest: auto-recaptured %ld (after long pair gaps)"
          "  base-drift %.2f deg (worst %.2f)  cmd-drift %.2f deg"
          " (worst %.2f)" DG_EOL,
          bs.arm_rest_recaptured,
          bs.arm_base_drift_deg, bs.arm_base_drift_worst_deg,
          bs.arm_cmd_drift_deg, bs.arm_cmd_drift_worst_deg);
    /* The caps ride on this line because "limited" only means something
       against them: a limited count near the pair count says the arm has
       run out of roll, which is the hand no longer following the player. */
    logf_("  bridge hand envelope: fore-twist pairs %ld  limited %ld  "
          "raw-twist %.2f deg -> fore %.2f + wrist %.2f deg; wrist-swing "
          "%.2f deg  caps %.0f/%.0f/%.0f" DG_EOL,
          bs.arm_hand_fore_twist, bs.arm_hand_limited,
          bs.arm_hand_raw_twist_deg, bs.arm_hand_fore_twist_deg,
          bs.arm_hand_wrist_twist_deg, bs.arm_hand_wrist_swing_deg,
          bs.arm_cap_fore_deg, bs.arm_cap_wrist_deg, bs.arm_cap_swing_deg);
    logf_("  bridge hand gate: no-command %ld stale %ld no-player %ld"
          " owner-mismatch %ld bad-weapon %ld microphone %ld no-SetPos %ld"
          "  release-zeros %ld" DG_EOL,
          bs.arm_hand_tick_no_command, bs.arm_hand_tick_stale,
          bs.arm_hand_tick_no_player, bs.arm_hand_tick_owner_mismatch,
          bs.arm_hand_tick_bad_weapon, bs.arm_hand_tick_mic,
          bs.arm_hand_no_setpos, bs.arm_hand_zeroed);
    /* F5 step 2. Printed whenever the probe found anything at all, including
       the case where it found a joint count but no stride - "n_models 55,
       stride none after 100 candidates" is a finding, and a silent probe is
       the failure mode that has cost this project the most runs. */
    if (bs.skel_n_models || bs.skel_stride) {
        int i;
        logf_("  bridge skel: objs 0x%016llX  region +0x%X  n_models %d"
              "  stride 0x%X (score %d, %d rejected)" DG_EOL,
              bs.skel_objs, bs.skel_region_end, bs.skel_n_models,
              bs.skel_stride, bs.skel_stride_score, bs.skel_stride_tried);
        logf_("  bridge skel: m_ctrl trans 0x%016llX [%s]" DG_EOL,
              bs.skel_mctrl_trans,
              bs.skel_mctrl_trans ? "LIVE - a direct route to joint position"
                                  : "null - position must be solved for");
        if (bs.skel_chain_len) {
            /* Hand first, then up. Consecutive distances are the bone lengths,
               and the indices are what the model says its arm is - not what
               HUMAN21 says a body's arm would be. */
            logf_("  bridge skel: chain from joint 6 (the proven hand), %d deep:"
                  DG_EOL, bs.skel_chain_len);
            for (i = 0; i < bs.skel_chain_len && i < DG_SKEL_CHAIN; i++) {
                double d = -1.0;
                if (i > 0) {
                    double dx = bs.skel_chain_pos[i][0] - bs.skel_chain_pos[i-1][0];
                    double dy = bs.skel_chain_pos[i][1] - bs.skel_chain_pos[i-1][1];
                    double dz = bs.skel_chain_pos[i][2] - bs.skel_chain_pos[i-1][2];
                    d = sqrt(dx * dx + dy * dy + dz * dz);
                }
                logf_("  bridge skel:   [%d] joint %-3d world %10.2f %10.2f "
                      "%10.2f   bone %.2f" DG_EOL,
                      i, (int)bs.skel_chain[i], bs.skel_chain_pos[i][0],
                      bs.skel_chain_pos[i][1], bs.skel_chain_pos[i][2], d);
            }
        }
        /* Sixteen to a line, fixed width, so the whole table can be read as a
           grid and any joint's parent found by counting. Fixed width also means
           the buffer size is arithmetic rather than a truncation question. */
        {
            int total = bs.skel_parents_read;
            if (total > DG_SKEL_MAX) total = DG_SKEL_MAX;
            for (i = 0; i < total; i += 16) {
                char line[16 * 5 + 1];
                int k, n = 0;
                for (k = i; k < i + 16 && k < total; k++) {
                    /* A root joint usually reports -1, so the sign has to
                       survive - it is the terminator the chain walk relies
                       on, not a formatting detail. */
                    int v = bs.skel_parents[k];
                    int a = v < 0 ? -v : v;
                    line[n++] = v < 0 ? '-' : ' ';
                    line[n++] = (char)('0' + (a / 100) % 10);
                    line[n++] = (char)('0' + (a / 10) % 10);
                    line[n++] = (char)('0' + a % 10);
                }
                line[n] = 0;
                logf_("  bridge skel: parent[%2d..]:%s" DG_EOL, i, line);
            }
        }
        for (i = 0; i < DG_SKEL_WINDOW; i++)
            if (bs.skel_window_pos[i][0] || bs.skel_window_pos[i][1] ||
                bs.skel_window_pos[i][2])
                logf_("  bridge skel: window joint %-3d world %10.2f %10.2f "
                      "%10.2f" DG_EOL, bs.skel_base + i,
                      bs.skel_window_pos[i][0], bs.skel_window_pos[i][1],
                      bs.skel_window_pos[i][2]);
    }
    /* A probe that measured nothing has to say which of the two nothings it
       was. The run of 2026-08-17 23:14 came back completely silent because
       first person was never entered, and silence is indistinguishable from a
       broken probe - which cost the run twice over, once to make and once to
       diagnose. */
    if (!bs.seam_active_samples)
        logf_("  bridge seam: the camera seam never reached the probes this"
              " session - first person was not held (entered %ld). The skeleton"
              " and adjust probes are behind that gate by design." DG_EOL,
              bs.fps_entered);
    /* F5 step 3. Printed as raw matrices on purpose: the convention gets solved
       offline from exactly these numbers, and rounding them for readability
       here would throw away the precision that solving needs. */
    if (bs.adj_probe_samples[0] || bs.adj_probe_samples[1]) {
        static const char *WHO[DG_ADJ_JOINTS] =
            { "root  ", "parent", "joint ", "child " };
        int c, k, r;
        logf_("  bridge adj: joint %d  held %ld  (indices root %d parent %d"
              " joint %d child %d)" DG_EOL,
              bs.adj_probe_joint, bs.adj_probe_held,
              bs.adj_probe_indices[0], bs.adj_probe_indices[1],
              bs.adj_probe_indices[2], bs.adj_probe_indices[3]);
        for (c = 0; c < DG_ADJ_CASES; c++)
            logf_("  bridge adj: case %d wrote quat %.6f %.6f %.6f %.6f"
                  "  samples %ld" DG_EOL, c,
                  bs.adj_probe_quat[c][0], bs.adj_probe_quat[c][1],
                  bs.adj_probe_quat[c][2], bs.adj_probe_quat[c][3],
                  bs.adj_probe_samples[c]);
        /* Meetplan A' (ROLL V5.1 par. 3): the actor's heading words that
           bracket each case - at the write and at the read, raw int16 on the
           4096 grid, so the mapping yaw in the matrices below can be laid
           against rot.vy in DIFFERENCES (sign) and in the absolute
           (DG_FRAME_ZERO) with the skew between the two reads on record. */
        for (c = 0; c < DG_ADJ_CASES; c++) {
            const long *w0 = bs.adj_probe_words[c][0];
            const long *w1 = bs.adj_probe_words[c][1];
            if (!w0[1] && !w1[1]) continue;
            logf_("  bridge adj: case %d src write tick %ld ok %ld rot.vy %+ld"
                  " turn.vy %+ld camdir.vy %+ld | read tick %ld ok %ld"
                  " rot.vy %+ld turn.vy %+ld camdir.vy %+ld" DG_EOL, c,
                  w0[0], w0[1], w0[2], w0[3], w0[4],
                  w1[0], w1[1], w1[2], w1[3], w1[4]);
        }
        /* The rate that broke the first attempt at this measurement, printed
           so it never has to be reconstructed from two counters again. The
           camera seam runs once per tick in third person and several times per
           tick in first person, while the hierarchy pass runs once - so a case
           held for less than a tick is read back before it was ever applied. */
        logf_("  bridge adj: %ld active seams over %ld active ticks = %.2f"
              " per active tick, case held %d ticks" DG_EOL,
              bs.seam_active_samples, bs.seam_active_ticks,
              bs.seam_active_ticks
                  ? (double)bs.seam_active_samples /
                    (double)bs.seam_active_ticks
                  : 0.0,
              DG_ADJ_SETTLE_TICKS);
        for (c = 0; c < DG_ADJ_CASES; c++) {
            if (!bs.adj_probe_samples[c]) continue;
            for (k = 0; k < DG_ADJ_JOINTS; k++) {
                for (r = 0; r < 4; r++)
                    logf_("  bridge adj: case %d %s row %d  %14.6f %14.6f"
                          " %14.6f %14.6f" DG_EOL, c, WHO[k], r,
                          bs.adj_probe_world[c][k][r * 4 + 0],
                          bs.adj_probe_world[c][k][r * 4 + 1],
                          bs.adj_probe_world[c][k][r * 4 + 2],
                          bs.adj_probe_world[c][k][r * 4 + 3]);
            }
        }
    }
    if (bs.arm_obj_head[0])
        logf_("  bridge bend: OBJECT +0x00: %016llX %016llX %016llX %016llX"
              DG_EOL
              "  bridge bend: OBJECT +0x20: %016llX %016llX %016llX %016llX"
              "   (+0x00 objs, +0x08 m_ctrl, +0x30 evmobj)" DG_EOL,
              bs.arm_obj_head[0], bs.arm_obj_head[1], bs.arm_obj_head[2],
              bs.arm_obj_head[3], bs.arm_obj_head[4], bs.arm_obj_head[5],
              bs.arm_obj_head[6], bs.arm_obj_head[7]);
    if (bs.arm_mctrl) {
        int i;
        for (i = 0; i < 10; i += 5)
            logf_("  bridge bend: m_ctrl 0x%016llX +0x%02X: "
                  "%016llX %016llX %016llX %016llX %016llX" DG_EOL,
                  bs.arm_mctrl, i * 8,
                  bs.arm_mctrl_head[i], bs.arm_mctrl_head[i + 1],
                  bs.arm_mctrl_head[i + 2], bs.arm_mctrl_head[i + 3],
                  bs.arm_mctrl_head[i + 4]);
    }
    if (bs.arm_forced || bs.arm_forced_late)
        logf_("  bridge arm: visibility forced %ld at the tick seam, "
              "%ld at the camera seam\r\n",
              bs.arm_forced, bs.arm_forced_late);
    /* The line that separates "we saw nothing" from "nothing happened". Every
       bit each word has held at any point, not the one tick the heartbeat
       happened to land on. */
    logf_("  bridge seen (whole session): player 0x%016llX  game 0x%08X"
          "  menu 0x%08X  arm flag 0x%08X\r\n",
          bs.seen_player, bs.seen_game, bs.seen_menu, bs.seen_arm_flag);
    /* And the same words at the camera seam, which unlike the tick seam is not
       gated on holding first person and so keeps sampling through a codec or a
       cutscene. The sample count is the half that makes a zero readable: zero
       bits over zero samples says nothing at all, zero bits over thousands of
       samples with a cutscene played says the anchor is wrong. */
    logf_("  bridge seen (camera seam, %ld samples): player 0x%016llX"
          "  game 0x%08X  menu 0x%08X%s" DG_EOL,
          bs.late_status_samples, bs.seen_player_late, bs.seen_game_late,
          bs.seen_menu_late,
          (bs.late_status_samples && !bs.seen_game_late)
              ? "   <- game word never moved here either" : "");
    /* And the same words at the PRESENT seam, the only one of the three that
       keeps running while the game is paused - which is to say the only one
       that can ever see a menu open (UI-U1). Bits here that are missing from
       the two lines above are the expected reading, not a contradiction, and
       the sample count is again what separates "never looked" from "never
       happened". 0x300 is MENU_WEAPON_OPEN|MENU_ITEM_OPEN. */
    /* The roll branch, live. The pose anchor is proved on the desk; this
       says whether the live rig ever offers it a reference, which is a
       different question and the one that decides what a persisting
       tumble means. */
    /* Totals, because the defect is smooth: fifteen revolutions at under a
       degree a frame read as perfectly calm on every per-pair instrument
       above. "hand" is the hierarchy measured passively (it keeps counting
       with vr_arm_track=off, which is how the 2026-08-21 run proved the
       spin is ours), "we asked" is the rotation our own two joints turned
       over the same stretch. Hand far above ours means the hierarchy is
       compounding what we write; the two rising together means the
       solution itself is winding. */
    /* The assumption the whole strip rests on, finally measured. Anything
       but ~0 here means the hierarchy did not keep what we wrote into
       joints 4 and 5, so removing the CACHED rotation leaves a residue of
       our own last write inside the "animation" we solve against - and the
       roll anchor references exactly those recovered points. */
    /* The adjust frame's health (V5.1 par. 4.6): which heading the
       conversions used, how many live pairs had none, and the largest
       heading change measured inside one pair's processing. */
    logf_("  bridge adjust frame: %s  missing %ld  skew-worst %.2f deg"
          "  wrist-miss worst %.1f mm  over-20mm %ld" DG_EOL,
          bs.adjust_frame_live ? "live" : "legacy", bs.frame_missing,
          (double)bs.frame_skew_worst, (double)bs.wrist_miss_worst,
          bs.wrist_miss_over);
    logf_("  bridge adjust echo (j4/j5): %.2f deg (worst %.2f, dirty %ld)"
          DG_EOL,
          (double)bs.arm_adj_echo_deg, (double)bs.arm_adj_echo_worst_deg,
          bs.arm_adj_echo_dirty);
    logf_("  bridge turned totals: hand %.0f deg over %ld samples;"
          "  we asked j4 %.0f  j5 %.0f" DG_EOL,
          (double)bs.skel_hand_turned_deg, bs.skel_hand_samples,
          (double)bs.adj_turned4_deg, (double)bs.adj_turned5_deg);
    /* The bar the totals above cannot state. They sum absolute steps, so a
       hand that shakes and a hand that winds read alike; this is the angle
       against a reference captured once and never moved. Stand still and a
       healthy arm keeps `max` in single digits however long it runs. */
    logf_("  bridge hand NET (vs fixed reference): now %.1f deg  max %.1f deg"
          DG_EOL,
          (double)bs.skel_hand_net_deg, (double)bs.skel_hand_net_max_deg);
    {
        int fz = (int)InterlockedCompareExchange(&g_arm_freeze_cfg, 0, 0);
        logf_("  bridge arm FREEZE: %s  pairs replayed unsolved %ld%s" DG_EOL,
              (fz == 2) ? "rest (reference frozen, target live)"
            : (fz == 1) ? "on (constant-replay, solver bypassed)" : "off",
              bs.arm_pairs_frozen,
              (fz == 1 && !bs.arm_pairs_frozen)
                  ? "   <- nothing frozen yet: no pair has solved" : "");
        /* The number the rest mode exists to produce. Read it against the
           hand NET meter: reference drifting while the hand is parked is
           the strip residue accumulating - the feedback caught directly.
           Reference at ~0 while the hand still winds acquits the strip
           and convicts the solver's own maths. */
        if (fz == 2)
            logf_("  bridge REST ref: %s  drift vs recovered now %.2f deg"
                  "  max %.2f deg" DG_EOL,
                  bs.rest_ref_captured ? "captured (root frame)"
                                       : "NOT CAPTURED - no calibration yet",
                  (double)bs.rest_ref_drift_deg,
                  (double)bs.rest_ref_drift_max_deg);
    }
    logf_("  bridge orient roll: anchored %ld  fell back %ld"
          "  up-rolled %ld  skipped %ld%s%s" DG_EOL,
          bs.orient_anchored, bs.orient_fallback,
          bs.orient_uprolled, bs.uproll_skipped,
          (bs.orient_fallback > bs.orient_anchored)
              ? "   <- the anchor is NOT reaching these frames" : "",
          (bs.arm_uproll_on && bs.orient_anchored > 0 &&
           bs.orient_uprolled == 0)
              ? "   <- the grip roll is NOT reaching these frames" : "");
    logf_("  bridge seen (present seam, %ld samples): game 0x%08X"
          "  menu 0x%08X%s" DG_EOL,
          bs.screen_status_samples, bs.seen_game_screen, bs.seen_menu_screen,
          (bs.screen_status_samples && !(bs.seen_menu_screen & 0x300))
              ? "   <- no weapon or item menu was opened this session" : "");
    /* Which pad bits this session has actually seen, and how often the weapon
       button among them. The game is on keyboard and mouse here and the binding
       is not something to guess at - press keys, read the count. */
    logf_("  bridge pad: seen 0x%08X  PL_PAD_WEAPON 0x%08X  weapon presses %ld"
          " (%ld in first person)\r\n",
          bs.seen_pad_status, bs.pad_weapon_mask, bs.weapon_presses,
          bs.weapon_presses_in_fps);
    if (bs.weapon_presses) {
        char keys[96];
        int i, n = 0;
        keys[0] = 0;
        for (i = 0; i < 4 && bs.weapon_vk[i]; i++)
            n += _snprintf_s(keys + n, sizeof(keys) - n, _TRUNCATE,
                             "%s%s (VK 0x%02X)", n ? ", " : "",
                             vk_name(bs.weapon_vk[i]), bs.weapon_vk[i]);
        logf_("  bridge pad: weapon button is %s\r\n",
              n ? keys : "<nothing was down - a pad, or released too fast>");
    }
    if (bs.fire_mode != InterlockedCompareExchange(&g_fire_mode, 0, 0)) {
        /* The marker asked for one thing and the bridge is doing another. That
           has happened - a stale second gate downgraded ON to OFF, and because
           the block below is keyed on the bridge's mode the whole session went
           quiet instead of complaining. Never again silently. */
        logf_("  bridge fire: MARKER ASKED FOR MODE %ld, THE WRITER IS RUNNING"
              " MODE %d - the run below is not the run that was asked for\r\n",
              InterlockedCompareExchange(&g_fire_mode, 0, 0), bs.fire_mode);
    }
    if (bs.move_mode) {
        /* One line: what we wrote and every reason we did not. yielded is the
           write contract made visible - the player's own pad silencing us -
           and a run that walks fine shows idle counting whenever the stick is
           at rest, which is the cheapest proof the deadzone gate is alive. */
        logf_("  bridge move: writes %ld  yielded %ld  idle %ld  stale %ld"
              "  no-command %ld  gate %ld  not-subject %ld  published %ld"
              "  deadzone %.2f\r\n",
              bs.move_writes, bs.move_yielded, bs.move_idle, bs.move_stale,
              bs.move_no_command, bs.move_blocked_gate, bs.move_not_subject,
              bs.move_published, bs.move_deadzone);
        if (bs.move_third || bs.move_prone)
            logf_("  bridge move ext: third %d (writes %ld, refused %ld)"
                  "  prone %d (capped writes %ld)  dir writes %ld  padTo writes %ld (no-workL %ld)"
                  "  cam-dir %ld  last org %ld dir %ld  seam %s\r\n",
                  bs.move_third, bs.move_third_writes, bs.move_not_third,
                  bs.move_prone, bs.move_prone_writes, bs.move_dir_writes,
                  bs.move_padto_writes, bs.move_no_workl,
                  bs.move_cam_dir, bs.move_last_org, bs.move_last_dir,
                  bs.tick_seam_copy ? "copy" : "publish");
        if (bs.mp_w_tick)
            logf_("  bridge move probe: wrote tick %ld dir %ld status 0x%08lX bytes %02lX/%02lX"
                  " | camera seam tick %ld (seen %ld) dir %ld status 0x%08lX analog 0x%04lX"
                  " bytes %02lX/%02lX act 0x%llX act2 0x%llX rot %ld turn %ld org %s\r\n",
                  bs.mp_w_tick, bs.mp_w_dir, bs.mp_w_status,
                  (bs.mp_w_bytes >> 8) & 0xFF, bs.mp_w_bytes & 0xFF,
                  bs.mp_c_tick, bs.mp_c_seen, bs.mp_c_dir, bs.mp_c_status, bs.mp_c_analog,
                  (bs.mp_c_bytes >> 8) & 0xFF, bs.mp_c_bytes & 0xFF,
                  bs.mp_c_act, bs.mp_c_act2, bs.mp_c_rot, bs.mp_c_turn,
                  bs.move_dir_org ? "body" : "camera");
        if (bs.mp_w_tick)
            logf_("  bridge move probe: work->pad 0x%llX  our record 0x%llX (%s)  GV_PadData.flag 0x%08lX%s"
                  "  through work->pad: status 0x%08lX dir %ld bytes %02lX/%02lX\r\n",
                  bs.mp_c_work_pad, bs.mp_c_our_pad,
                  bs.mp_c_work_pad == bs.mp_c_our_pad ? "SAME" : "DIFFERENT",
                  bs.mp_c_gv_flag, (bs.mp_c_gv_flag & 0x103) ? " <- RELEASE set" : "",
                  bs.mp_c_wp_status, bs.mp_c_wp_dir,
                  (bs.mp_c_wp_bytes >> 8) & 0xFF, bs.mp_c_wp_bytes & 0xFF);
        if (bs.mp_w_tick)
            logf_("  bridge move probe: workL 0x%llX  PadTo %ld  WallTo %ld  Liable %ld  PadForce %ld  |  work->data %ld  data2 %ld\r\n",
                  bs.mp_c_workl, bs.mp_c_padto, bs.mp_c_wallto, bs.mp_c_liable,
                  bs.mp_c_padforce, bs.mp_c_data, bs.mp_c_data2);
    }
    if (bs.turn_mode) {
        logf_("  bridge turn: writes %ld  yielded %ld  idle %ld  gain %.2f"
              "  soft-turn %+.1f deg since recentre\r\n",
              bs.turn_writes, bs.turn_yielded, bs.turn_idle, bs.turn_gain,
              input_turn_offset_rad() / DEG2RAD);
        /* The body-follow's whole story in one line: how often it spoke,
           the gap it steers on, whether the byte sign has committed - dir
           0 with a standing gap means it is still learning from the
           player's own stick - and every refusal by name. A session
           ending "writes 0" must be explainable from this line alone: the
           2026-08-23 run needed exactly that and the line could not say
           no-sign. */
        logf_("  bridge turn follow: writes %ld  src %s  aim %s  aim-gap %+.1f deg  "
              "dir %+d (votes %ld)  refused: no-sign %ld  stale %ld"
              "  under-thresh %ld  aim-hold %ld\r\n",
              bs.turn_follow_writes,
              bs.follow_src == 2 ? "stick" : bs.follow_src ? "hand" : "head",
              bs.follow_aim ? "on" : "hold",
              (double)bs.turn_aim_gap,
              bs.turn_dir, bs.turn_dir_votes, bs.follow_gate_no_sign,
              bs.follow_gate_stale, bs.follow_gate_under,
              bs.follow_gate_aim_hold);
        /* The one-byte intake gap, forever: nonzero means some path still
           writes 176/80, which the game's strict gate ignores. */
        if (bs.turn_write_dead)
            logf_("  bridge turn DEAD-BYTES: %ld writes at exactly 176/80"
                  " - the intake gap is back\r\n", bs.turn_write_dead);
        drain_turn_probe();
    }
    drain_adj_cycles();
    if (bs.move_mode || bs.turn_mode)
        /* The driver's own bytes, before anything of ours. If a phantom turn
           or walk ever appears, this line says in one run whether a second
           input path (a runtime emulating a gamepad) is feeding the pad or
           whether it is us. 128 everywhere = nobody but us is speaking. */
        logf_("  bridge pad analog (driver's own): right [%u %u]  left"
              " [%u %u]\r\n",
              bs.pad_right_dx, bs.pad_right_dy,
              bs.pad_left_dx, bs.pad_left_dy);
    if (bs.fire_mode != DG_FIRE_MODE_OFF) {
        /* Two lines, split the way every block here is split: what the
           contract decided, then why it was refused. In DRY the whole thing is
           arithmetic - nothing below has touched the game. */
        logf_("  bridge fire (%s): published %ld  drawn %ld  released %ld"
              "  aborted %ld  auto ticks %ld\r\n",
              bs.fire_mode == DG_FIRE_MODE_DRY ? "dry, writes nothing"
                                               : "writing",
              bs.fire_published, bs.fire_drawn, bs.fire_released,
              bs.fire_aborted, bs.fire_auto_ticks);
        logf_("  bridge fire gate: no command %ld  stale %ld  game refused %ld"
              "  player's own button %ld  repeats %ld  forced %ld\r\n"
              "    coasted %ld aim ticks across sample gaps (grace %d)\r\n"
              "    state %d  last pressure %d  weapon type 0x%08X (%s)\r\n",
              bs.fire_no_command, bs.fire_stale, bs.fire_blocked_gate,
              bs.fire_blocked_phys, bs.fire_repeats, bs.fire_forced,
              bs.fire_coasting, DG_FIRE_GRACE_TICKS,
              bs.fire_state, bs.fire_pressure, bs.fire_wtype,
              (bs.fire_wtype & DG_FIRE_WP_CONSECUTIVE) ? "automatic"
                  : (bs.fire_wtype & DG_FIRE_WP_PRESSURE) ? "release to fire"
                  : "neither");
        /* Third line, and it means something only in ON: what actually left
           this process, per pad field. In DRY every one of these is zero by
           construction, which is the cheapest possible check that DRY is
           still dry. */
        logf_("  bridge fire wrote: press %ld  status %ld  release %ld"
              "  pressure %ld (game byte kept %ld)  yielded %ld\r\n"
              "    PL_PAD_PRESS_WEAPON is index %d into pressure[12]"
              "  (outside it %ld times)\r\n",
              bs.fire_wrote_press, bs.fire_wrote_status,
              bs.fire_wrote_release, bs.fire_wrote_pressure,
              bs.fire_pressure_kept, bs.fire_yielded,
              bs.fire_press_index, bs.fire_no_index);
        /* And the witness. These two are the Bluepoint layer own answer, not
           ours: WS_Draw (2) means our press was taken, WS_HolsterQuick (1)
           means it was refused, BS_SoftRelease (3) is the cancel path. A run
           where we wrote and this never moved is a run where we wrote into a
           record nobody reads. */
        logf_("  bridge fire witness: weaponState %d (seen 0x%04X)"
              "  buttonState %d (seen 0x%04X)\r\n",
              bs.weapon_state, bs.weapon_state_seen,
              bs.button_state, bs.button_state_seen);
        if (bs.recoil_climb_mdeg || bs.recoil_push_um) {
            /* Motion we invented, so it gets its own line and its own
               accounting. kicks should equal released - every shot is one
               impulse - and amplitude should read 0.000 at the end of any
               session that is not mid-burst, because a spring that does not
               come home is a standing aim error. */
            logf_("  bridge recoil: %.2f deg climb + %.1f mm push per round"
                  "  kicks %ld (shots %ld)  climb frames %ld  push frames %ld"
                  "\r\n"
                  "    no axis %ld  push refused %ld  amplitude now %.3f"
                  "  worst %.3f shots\r\n",
                  bs.recoil_climb_mdeg / 1000.0, bs.recoil_push_um / 1000.0,
                  bs.recoil_kicks, bs.fire_released,
                  bs.recoil_climb_writes, bs.recoil_push_writes,
                  bs.recoil_no_axis, bs.recoil_push_refused,
                  bs.recoil_amplitude, bs.recoil_worst);
        }
    }
}

/* One armed session: wait for a live camera, arm, run, disarm. Returns the
   reason it ended. */
static const char *run_once(const char *marker) {
    SCRIPT_MARKER_SNAPSHOT script_config;
    POSE p;
    DG_XR_CONFIG xc;
    DG_BRIDGE_CONFIG bc;
    int source = SRC_SYNTHETIC;
    double seconds = 600.0;
    DWORD waited = 0;
    int elapsed = 0, armed_threads;
    const char *reason = "duration elapsed";
    LONG last_traps;
    int bridge_on = 0;
    int menu_session = 0;
    int last_summary = 0;
    long last_transitions = -1;
    /* Not 0: an arm that is legitimately 0x0000 for the whole session still has
       to produce its first line, or "we saw nothing" and "we never looked" read
       identically in the log. */
    unsigned int last_arm_flag = 0xFFFFFFFFu;
    unsigned int last_pad_seen = 0xFFFFFFFFu;
    unsigned int last_body_flag = 0xFFFFFFFFu;
    long last_arm_visible = -1;
    int last_cam_on = -1;
    int active_arm_track;
    int active_arm_qmap[4];
    int active_arm_hand;
    ARM_POS_CFG active_arm_pos;
    LONG rec_dump_token_seen;
    long rec_snap_appended = 0;
    int rec_snap_elapsed = 0;

    if (g_script_boot) {
        char config_text[DG_SCRIPT_MARKER_MAX_BYTES + 1];
        if (!g_script_gate.busy || g_script_token_active <= 0 ||
            !read_script_snapshot(marker, &script_config) ||
            !dg_script_gate_observe(&g_script_gate,
                script_config.info.source_script, script_config.info.token_valid,
                script_config.info.token))
            return "script request changed before config";
        memcpy(config_text, script_config.text, script_config.size + 1);
        if (!parse_config(config_text, &p, &seconds, &source, &xc, &bc,
                          &active_arm_track, active_arm_qmap, &active_arm_hand,
                          &active_arm_pos) || source != SRC_SCRIPT)
            return "script configuration invalid";
    } else if (!read_config(marker, &p, &seconds, &source, &xc, &bc,
                            &active_arm_track, active_arm_qmap, &active_arm_hand,
                            &active_arm_pos)) {
        return "marker invalid or unavailable";
    }
    menu_session = g_script_menu_cfg;
    if (menu_session) {
        script_menu_sanitize(&p, &bc, &active_arm_track, &active_arm_hand);
        InterlockedExchange(&g_arm_freeze_cfg, 0);
        g_policy_path[0] = 0;
    }
    if (source == SRC_SCRIPT) {
        if (!g_script_boot) return "source=script requires process restart";
        if (!g_present_available || g_xr_worker_owned) {
            logf_("  script: needs Present and exclusive desktop ownership; "
                  "select source=script before launch\r\n");
            return "script source unavailable";
        }
        /* A desktop input experiment uses the native mono projection. */
        xc.stereo = 0;
        xc.stereo_test = 0;
        AcquireSRWLockExclusive(&g_script_present_lock);
        g_script_present_stopped = 0;
        ReleaseSRWLockExclusive(&g_script_present_lock);
        dg_present_set_callback(on_present);
    }
    InterlockedExchange(&g_source, source);

    InterlockedIncrement(&g_camera_yaw_anchor_generation);
    /* The recorder starts fresh per session and honours the marker from the
       first frame. The token present at startup is the baseline, not a
       request: only a CHANGE dumps. */
    dg_rec_reset(&g_rec);
    g_rec_last_torn = 0;
    rec_dump_token_seen = InterlockedCompareExchange(&g_rec_cfg_dump, 0, 0);
    InterlockedExchange(&g_rec_on,
                        menu_session ? 0 : InterlockedCompareExchange(&g_rec_cfg_on, 0, 0));
    active_arm_pos.hand_zero =
        arm_hand_zero_total(active_arm_pos.hand_zero,
                            input_secondary_presses(
                                arm_tracked_xr_hand(active_arm_track)));
    InterlockedExchange(&g_arm_hand_zero_now, active_arm_pos.hand_zero);
    arm_pos_cfg_apply(&active_arm_pos);
    memcpy(g_arm_qmap, active_arm_qmap, sizeof active_arm_qmap);
    g_arm_track = active_arm_track;
    InterlockedExchange(&g_arm_hand, active_arm_hand);
    InterlockedExchange(&g_fire_mode, bc.fire_mode);
    InterlockedExchange(&g_move_mode, bc.move_mode);
    InterlockedExchange(&g_turn_mode, bc.turn_mode);
    InterlockedExchange(&g_turn_gain_mils_live, bc.turn_gain_mils);
    InterlockedExchange(&g_turn_deadzone_mils_live, bc.move_deadzone_mils);
    InterlockedExchange(&g_move_third_live, bc.move_third);
    InterlockedExchange(&g_camera_yaw_anchor_mode,
                        bc.camera_yaw_anchor == 1 ? 1 : 0);
    logf_("  camera yaw: %s\r\n", bc.camera_yaw_anchor == 1 ? "anchor" : "off");
    g_pose = p;
    g_xrcfg = xc;
    InterlockedExchange(&g_stereo, xc.stereo != 0);
    InterlockedExchange(&g_scene_blur_enabled, xc.scene_blur_fix);
    InterlockedExchange(&g_link_requested, xc.stereo_phase_fix);
    render_link_reset();
    InterlockedExchange(&g_stereo_test, xc.stereo_test);
    InterlockedExchange(&g_stereo_proj_x_sign, xc.stereo_proj_x_sign);
    InterlockedExchange(&g_stereo_eye_x_sign, xc.stereo_eye_x_sign);
    dg_ui2d_configure(xc.ui2d, xc.ui2d_scale_mils, xc.ui2d_conv_e5, xc.ui2d_sign, xc.ui2d_vs, (unsigned)xc.ui2d_vs_count);
    InterlockedExchange(&g_eye_truth_mode, xc.eye_truth);
    InterlockedExchange(&g_draw_skip_mode, xc.draw_skip);
    InterlockedExchange(&g_feedback_skip, xc.feedback_skip);
    dg_ui2d_hold(xc.ui2d_hold);
    dg_radar_configure(xc.radar_mode != 0, xc.radar_hud == 0);
    dg_bridge_native_hud_configure(xc.radar_mode != 0 && xc.radar_hud == 0);
    InterlockedExchange(&g_current_eye, DG_EYE_LEFT);
    handoff_write(0, DG_EYE_MONO, NULL, NULL);
    InterlockedExchange(&g_source, source);

    InterlockedExchange(&g_applied, 0);
    InterlockedExchange(&g_fresh, 0);
    InterlockedExchange(&g_reapplied, 0);
    InterlockedExchange(&g_rej_nonfinite, 0);
    InterlockedExchange(&g_rej_not_ortho, 0);
    InterlockedExchange(&g_rej_not_proj, 0);
    InterlockedExchange(&g_rej_no_pose, 0);
    InterlockedExchange(&g_traps, 0);
    g_script_camera_seen = 0;
    InterlockedExchange(&g_have_mine, 0);
    last_traps = 0;

    QueryPerformanceFrequency(&g_qpf);
    QueryPerformanceCounter(&g_qpc0);
    dg_pose_init(&g_arm_pose_state, NULL);
    g_arm_pose_have_qpc = 0;
    InterlockedExchange(&g_present_frame, 0);
    InterlockedExchange(&g_arm_pose_stale, 0);
    InterlockedExchange(&g_arm_pose_untracked, 0);

    logf_("\r\n--- session ---\r\n"
          "  source %s\r\n"
          "  rot   yaw %.2f  pitch %.2f  roll %.2f deg\r\n"
          "  trans x %.1f  y %.1f  z %.1f mm\r\n"
          "  sweep %.2f deg @ %.3f Hz on axis %d\r\n"
          "  max   %.0f s\r\n",
          source == SRC_XR ? "xr" : source == SRC_SCRIPT ? "script" : "synthetic",
          p.yaw / DEG2RAD, p.pitch / DEG2RAD, p.roll / DEG2RAD,
          p.tx, p.ty, p.tz, p.sweep_rad / DEG2RAD, p.sweep_hz, p.sweep_axis,
          seconds);
    logf_("  camera-yaw %s (base heading anchor; opt-in)\r\n",
          bc.camera_yaw_anchor == 1 ? "anchor" : "off");
    /* Said once, at the top, where it cannot be missed when a log is read back
       weeks later: this is the only setting in the file that writes input into
       the game. It says `asked for` because at this point that is all it is -
       the bridge has not been configured yet, and what it honours is reported
       by the heartbeat instead. */
    logf_("  arm   frame %s - %s\r\n",
          arm_frame_name(active_arm_pos.frame),
          active_arm_pos.frame == DG_XR_REL_FRAME_HEAD
              ? "the arm hangs off your gaze, and looking away carries it"
              : active_arm_pos.frame == DG_XR_REL_FRAME_ROOM
              ? "the arm hangs off the character body, in tracking-space axes"
              : "looking up, down or tilting no longer moves the arm");
    logf_("  arm   anchor %s  pos-sign [%+.0f %+.0f %+.0f]"
          "  player shoulder out %.0f down %.0f fwd %.0f mm  reach %.0f mm\r\n",
          active_arm_pos.anchor ? "shoulder - the hand goes where YOUR hand is"
                                : "wrist - the hand starts at the animation's",
          active_arm_pos.sign[0], active_arm_pos.sign[1], active_arm_pos.sign[2],
          active_arm_pos.shoulder_mm[0], active_arm_pos.shoulder_mm[1],
          active_arm_pos.shoulder_mm[2], active_arm_pos.reach_mm);
    logf_("  rec   %s - raw XR input, last ~3 min, dumped to the log dir on"
          " session end + a rolling dg_rec_live.dgrec every %ds"
          " (vr_rec_dump=<n>: change dumps now)\r\n",
          InterlockedCompareExchange(&g_rec_on, 0, 0) ? "ON" : "off",
          DG_REC_SNAPSHOT_S);
    /* Policy pickup runs on the same 1 Hz poll as everything else; this
       first call catches a DLL that was already built before the session
       started. Said in the header either way, so a log read back later
       states which maths the session began on. */
    if (!menu_session) dg_policy_poll(g_policy_path);
    {
        DG_POLICY_STATUS pst;
        dg_policy_status(&pst);
        if (g_policy_path[0])
            logf_("  policy vr_policy=%s - hot reload armed (builds adopt"
                  " on the tick seam; staged %ld refused %ld)\r\n",
                  g_policy_path, pst.staged, pst.refused);
        else
            logf_("  policy builtin (no vr_policy in the marker)\r\n");
    }
    if (bc.move_mode)
        logf_("  move  asked for ON - the left stick writes the walk pad"
              " (deadzone %.2f)\r\n",
              (double)bc.move_deadzone_mils / 1000.0);
    if (bc.turn_mode)
        logf_("  turn  asked for SMOOTH - the right stick rotates the"
              " tracking space (view, walk and arm together, %.0f deg/s at"
              " full stick); the pad turn path serves the body-follow"
              " only\r\n",
              DG_TURN_RATE_DEG_S * (double)bc.turn_gain_mils / 1000.0);
    if (bc.fire_mode != DG_FIRE_MODE_OFF)
        logf_("  fire  asked for %s\r\n",
              bc.fire_mode == DG_FIRE_MODE_DRY
                  ? "dry - the whole contract is evaluated and nothing is"
                    " written"
                  : "ON - the trigger writes the weapon pad and rounds go"
                    " downrange");

    if (source == SRC_XR) {
        dg_xr_configure(&xc);
        logf_("  xr signs y %+.0f p %+.0f r %+.0f  x %+.0f y %+.0f z %+.0f"
              "  scale %.0f  positional %d  stereo-proj-x %+d  stereo-eye-x %+d"
              "  trigger %.2f/%.2f\r\n",
              xc.yaw_sign, xc.pitch_sign, xc.roll_sign,
              xc.x_sign, xc.y_sign, xc.z_sign, xc.scale, xc.positional,
              xc.stereo_proj_x_sign, xc.stereo_eye_x_sign,
              xc.trigger_deadzone, xc.trigger_fire);
        if (!g_xr_worker_owned && !dg_xr_start(logf_)) return "OpenXR unavailable";
    }

    if (source == SRC_SCRIPT)
        logf_(menu_session
            ? "  script menu: camera wait bypassed; detector-only session\r\n"
            : "  script: camera wait started; camera validity is not proof of gameplay\r\n");
    while (!menu_session && waited < 60000 && !camera_live()) {
        if (source == SRC_SCRIPT) {
            if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
                return "script aborted by Escape while waiting for camera";
            if (!script_request_current(marker, &script_config))
                return "script marker/config changed while waiting for camera";
        }
        Sleep(250); waited += 250;
    }
    if (!menu_session && !camera_live()) { if (source == SRC_XR) dg_xr_stop(); return "camera never went live"; }

    if (source == SRC_SCRIPT) {
        char script[MAX_PATH], *slash;
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000)
            return "script aborted by Escape before publisher start";
        if (!script_request_current(marker, &script_config))
            return "script marker/config changed before publisher start";
        strcpy_s(script, sizeof script, marker);
        slash = strrchr(script, '\\');
        if (!slash) return "script directory unavailable";
        slash[1] = 0;
        if (strcat_s(script, sizeof script, "dg_controller.script") != 0)
            return "script path too long";
        if (!dg_xr_script_start(script, &xc, logf_))
            return "script rejected";
        g_script_primary_seen = dg_xr_script_primary_press_seq(DG_XR_HAND_RIGHT);
        g_start_seen = input_menu_press_seq();
    }

    /* F2 normally waits for a live camera before resolving decrypted anchors.
       Explicit script-menu sessions try the same complete bridge validation
       earlier, with every gameplay setting OFF; failure refuses script go. */
    if (menu_session) {
        InterlockedExchange(&g_script_menu_context_lost, 0);
    }
    if (menu_session || bc.fps_mode != DG_FPS_MODE_OFF) {
        bc.unarmed_prone_enabled=source==SRC_XR && g_arm_track!=ARM_TRACK_OFF;
        bridge_on = dg_bridge_start(logf_, &bc);
        if (menu_session && !bridge_on) {
            dg_xr_script_stop();
            script_menu_callbacks_stop();
            InterlockedExchange(&g_script_menu_active, 0);
            InterlockedExchange(&g_script_menu_context_lost, 0);
            return "script menu bridge validation failed";
        }
        if (menu_session) InterlockedExchange(&g_script_menu_active, 1);
        if (!bridge_on)
            logf_("  bridge: inert (vr_fps_mode=%d requested)\r\n", bc.fps_mode);
    } else {
        logf_("  bridge off (vr_fps_mode=off)\r\n");
    }
    /* Named at startup like the bridge, because a theater that quietly is not
       running (bridge off, anchors unresolved) must read as such in the log
       rather than be discovered in the headset. */
    logf_("  theater %s  ui %s  hud %s  dist %.2f m  width %.2f m%s\r\n",
          bc.theater_mode == DG_THEATER_ON ? "on"
              : bc.theater_mode == DG_THEATER_MEASURE ? "measure" : "off",
          bc.theater_ui ? "on" : "off",
          bc.hud_mode ? "HIDE" : "on",
          xc.theater_dist, xc.theater_width,
          (bc.theater_mode != DG_THEATER_OFF || bc.theater_ui || bc.hud_mode)
              && !bridge_on
              ? "  (INERT: the bridge is not armed, nothing will measure or"
                " switch)" : "");

    logf_("  radar wrist: %s  hud radar %s  size %.3f m  offset %.3f,%.3f,%.3f m  rot %.1f,%.1f,%.1f deg  space %s  gaze %.1f deg (pitch %.1f)%s\r\n",
          xc.radar_mode == 1 ? "fixed" : xc.radar_mode == 2 ? "wrist" : "off",
          xc.radar_hud ? "on" : "OFF while the wrist radar is up", xc.radar_size,
          xc.radar_offset[0], xc.radar_offset[1], xc.radar_offset[2],
          xc.radar_rot[0], xc.radar_rot[1], xc.radar_rot[2],
          xc.radar_space_local ? "local" : "grip", xc.radar_gaze_deg, xc.radar_gaze_pitch,
          xc.radar_mode && bc.hud_mode ? "  (vr_hud=off stops the game rendering its radar: nothing to show)" : "");

    controls_init();
    g_controls_turn_fallback=!bridge_on && !menu_session;
    if (bridge_on && !menu_session) {
        dg_bridge_controls_register(controls_provide, controls_stop, NULL);
        dg_bridge_action_register(g_source==SRC_XR?action_from_frame:NULL);
        logf_("  controls: central tick routing; radial catalog/equip gate closed\r\n");
    }
    /* Optional retail-anchored DR1 seam. Never patch unknown code or touch
       actor storage. Resolve once per session before debug registers arm. */
    g_scene_blur_addr = 0;
    if (source == SRC_XR && !menu_session) {
        __try {
            if (dg_scene_blur_signature((const unsigned char *)(g_base+0x2BB6B8),
                    (const unsigned char *)(g_base+DG_SCENE_BLUR_RVA),
                    (const unsigned char *)(g_base+0x2BB6F3),
                    (const unsigned char *)(g_base+0x2BB725)))
                g_scene_blur_addr = g_base + DG_SCENE_BLUR_RVA;
        } __except(EXCEPTION_EXECUTE_HANDLER) { g_scene_blur_addr = 0; }
    }
    InterlockedExchange(&g_scene_blur_hits,0);
    InterlockedExchange(&g_scene_blur_zeroed,0);
    logf_("  scene blur: seam %s, fix %s (VR stereo only; original actor untouched)\r\n",
          g_scene_blur_addr ? "verified" : "unavailable",
          xc.scene_blur_fix ? "on" : "off");
    g_veh = AddVectoredExceptionHandler(1, veh);
    if (!g_veh) {
        if (menu_session) script_menu_callbacks_stop();
        if (bridge_on) {
            dg_bridge_action_register(NULL);
            dg_bridge_controls_register(NULL, NULL, NULL);
            dg_bridge_stop();
        }
        if (source == SRC_SCRIPT) dg_xr_script_stop();
        if (menu_session) {
            InterlockedExchange(&g_script_menu_active, 0);
            InterlockedExchange(&g_script_menu_context_lost, 0);
        }
        return "AddVectoredExceptionHandler failed";
    }

    phase_init();
    {
        char probe_marker[MAX_PATH], probe_output[MAX_PATH], *leaf;
        SYSTEMTIME stamp;
        strcpy_s(probe_marker,sizeof probe_marker,marker);
        leaf=strrchr(probe_marker,'\\');
        if (leaf) strcpy_s(leaf+1,sizeof(probe_marker)-(leaf+1-probe_marker),"dg_pair_probe.on");
        else strcpy_s(probe_marker,sizeof probe_marker,"dg_pair_probe.on");
        GetSystemTime(&stamp);
        strcpy_s(probe_output,sizeof probe_output,g_logpath);
        leaf=strrchr(probe_output,'\\');
        if (leaf) *leaf=0;
        else strcpy_s(probe_output,sizeof probe_output,".");
        {
            size_t len=strlen(probe_output);
            _snprintf_s(probe_output+len,sizeof(probe_output)-len,_TRUNCATE,
                "\\pair_probe_%04u%02u%02u_%02u%02u%02u_%lu_%lu.csv",
                stamp.wYear,stamp.wMonth,stamp.wDay,stamp.wHour,stamp.wMinute,
                stamp.wSecond,GetCurrentProcessId(),GetTickCount());
        }
        pp_prepare((menu_session||!g_phase_stage_addr||!g_phase_consume_addr)?0:g_base,probe_marker,probe_output);
        pp_poll(logf_);
    }

    InterlockedExchange(&g_render_link_active,g_link_requested&&source==SRC_XR&&
        g_phase_stage_addr&&g_phase_consume_addr);
    logf_("  render link: requested %ld anchors %s (buffer-linked pose/FOV; no guessed fallback)\r\n",
        g_link_requested,g_phase_stage_addr?"verified":"UNAVAILABLE");
    InterlockedIncrement(&g_auto_recenter_session);
    InterlockedExchange(&g_armed, 1);
    dg_xr_set_capture_observer(capture_observed);
    armed_threads = arm_all(1);
    if (menu_session && armed_threads <= 0) {
        reason = "script menu camera detector not armed";
        goto session_cleanup;
    }
    if (menu_session)
        logf_("  script menu: detector-only camera breakpoint armed on %d threads\r\n", armed_threads);
    else
        logf_("  camera live after %u ms - armed %d threads\r\n", waited, armed_threads);
    if (source == SRC_SCRIPT) {
        if (menu_session) {
            DG_BRIDGE_STATS menu_stats;
            dg_bridge_stats(&menu_stats);
            if (!menu_stats.menu_anchor_ok || !menu_stats.menu_detour_live) {
                reason = "script menu input route unavailable";
                goto session_cleanup;
            }
            script_menu_sample_context();
        }
        if (script_menu_blocked() || (GetAsyncKeyState(VK_ESCAPE) & 0x8000) ||
            !script_request_current(marker, &script_config)) {
            reason = "script cancelled before go (Escape or marker/config changed)";
            goto session_cleanup;
        }
        logf_("  script: ready; seams armed, requesting tick 0 (focus gate follows)\r\n");
        dg_xr_script_go();
    }

    for (;;) {
        LONG traps_now;
        Sleep(1000);
        elapsed++;
        phase_poll();
        dg_present_poll_state_probe(marker);
        dg_draw_trial_poll(marker);
        if (!hud_active && pp_poll(logf_)) arm_all(1);
        pp_flush(0,logf_);
        hud_poll();
        ci_drain();
        pd_drain(hud_active?1:(pp_status==PP_RECORDING?2:(g_phase_enabled?4:0)));
        if(g_render_link_active && (ULONG)(g_link_hits-g_link_last_hits)>100000u) {
            InterlockedExchange(&g_render_link_active,0);render_link_reset();arm_all(1);
            logf_("  render link: disabled by trap-rate guard; stereo captures refused\r\n");
        }
        g_link_last_hits=g_link_hits;
        if(elapsed%10==0&&g_link_requested)
            logf_("  render link: active %ld stage %ld consumer %ld linked-attempts %ld refused %ld contention %ld register-mismatch %ld\r\n",
                g_render_link_active,g_link_stage_hits,g_link_consumer_hits,g_link_accepted,
                g_link_refused,g_link_contention,g_link_register_mismatch);
        if (menu_session) script_menu_sample_context();
        if (script_menu_blocked()) {
            reason = "script menu ended: player context appeared or became unavailable";
            break;
        }
        if (source == SRC_SCRIPT) {
            if (!script_request_current(marker, menu_session ? &script_config : NULL)) {
                reason = "script request cancelled (marker/source/token changed)";
                break;
            }
        }
        if (source == SRC_SCRIPT && dg_xr_script_finished()) {
            reason = "script completed or aborted";
            break;
        }
        if (menu_session) {
            if (elapsed >= (int)seconds) break;
            continue; /* No gameplay shortcuts, probes, policy or hot config. */
        }

        traps_now = g_traps;
        /* If the resume flag were not honoured the breakpoint would re-fire on
           the same instruction forever. This catches that in one second rather
           than with a hung game. */
        if (traps_now - last_traps > WATCHDOG_TRAPS_PER_SEC) {
            reason = "WATCHDOG: trap rate implausible, resume flag suspect";
            break;
        }
        last_traps = traps_now;
        if (elapsed % 10 == 0 && g_scene_blur_addr)
            logf_("  scene blur: visits %ld suppressed %ld enabled %ld\r\n",
                  g_scene_blur_hits,g_scene_blur_zeroed,g_scene_blur_enabled);
        if (elapsed % 10 == 0 && g_stereo)
            logf_("  stereo eye: shifted %ld  legacy-fallback %ld  half-sep %.2f mm"
                  "  sign %+ld (right eye at camera %s)\r\n",
                  g_eye_shift_applied, g_eye_shift_fallback,
                  (double)g_eye_shift_half_x100 / 100.0,
                  (long)g_stereo_eye_x_sign, g_stereo_eye_x_sign < 0 ? "-x" : "+x");
        if (elapsed % 10 == 0 && g_stereo) {
            char ui[1100]; dg_ui2d_stats(ui, sizeof ui); logf_("  %s\r\n", ui);
            logf_("  eye truth: mode %ld  label L rendered +%ld/-%ld  label R rendered +%ld/-%ld  no-camera %ld"
                  "  MISLABELLED %ld  dropped %ld  relabelled %ld  no-camera dropped %ld passed %ld\r\n",
                  (long)g_eye_truth_mode, g_eye_truth_n[0][0], g_eye_truth_n[0][1],
                  g_eye_truth_n[1][0], g_eye_truth_n[1][1], (long)g_eye_truth_none,
                  (long)g_eye_truth_mismatch, (long)g_eye_truth_dropped, (long)g_eye_truth_relabelled,
                  (long)g_nocam_dropped, (long)g_nocam_passed);
            logf_("  draw skip: mode %ld  draws/present <50:%ld <300:%ld <1000:%ld 1000+:%ld  usual %ld  SKIPPED %ld held %ld relabelled %ld  map L +%ld/-%ld R +%ld/-%ld\r\n",
                  (long)g_draw_skip_mode, g_draw_hist[0], g_draw_hist[1], g_draw_hist[2], g_draw_hist[3], g_draw_ema,
                  g_draw_skipped, g_draw_held, g_draw_relabelled, g_draw_map[0][0], g_draw_map[0][1], g_draw_map[1][0], g_draw_map[1][1]);
            { long fs = 0, fc = 0; dg_ui2d_feedback_stats(&fs, &fc);
              logf_("  feedback skip: mode %ld  previous-frame sprites not forwarded %ld  (4-vertex draws into a final texture checked %ld)\r\n", (long)g_feedback_skip, fs, fc); }
            logf_("  bb probe: back-buffer draws/present 0:%ld 1-7:%ld 8+:%ld  EMPTY (held as skipped) %ld  first source same-as-previous %ld changed %ld\r\n",
                  g_bb_hist[0], g_bb_hist[1], g_bb_hist[2], g_bb_empty, g_bb_src_same, g_bb_src_alt);
            logf_("  obj eye: label L drawn +%ld/-%ld mixed %ld none %ld  label R drawn +%ld/-%ld mixed %ld none %ld"
                  "  alternated %ld SAME-AS-PREVIOUS %ld (longest run %ld)  ticks/present 0:%ld 1:%ld 2:%ld 3+:%ld  dt <20:%ld <40:%ld 40+:%ld\r\n",
                  g_obj_n[0][0], g_obj_n[0][1], g_obj_n[0][2], g_obj_n[0][3], g_obj_n[1][0], g_obj_n[1][1], g_obj_n[1][2], g_obj_n[1][3],
                  g_obj_alt, g_obj_same, g_obj_run_max, g_obj_ticks[0], g_obj_ticks[1], g_obj_ticks[2], g_obj_ticks[3],
                  g_obj_dt[0], g_obj_dt[1], g_obj_dt[2]);
        }
        if (elapsed % 10 == 0 && g_xrcfg.radar_mode) {
            char rx[420], rd[700]; dg_xr_radar_stats(rx, sizeof rx); dg_radar_stats(rd, sizeof rd);
            logf_("  radar wrist: %s  hud radar %s  |  %s\r\n", rx, g_xrcfg.radar_hud ? "on" : "off", rd);
        }

        {   /* The low bit latches "pressed since the last call", which is the
               only reason a 1 Hz poll can catch a quick tap at all. But it must
               be CONSUMED every poll, unconditionally: read it only when the
               modifiers happen to be down and an unrelated R from minutes ago
               stays latched, then fires the next time Ctrl+Alt is held for some
               other reason - a recentre nobody asked for, at random.
               The modifiers test 0x8000 (down now) rather than the latch, which
               is what "modifier" means. */
            int tapped = (GetAsyncKeyState('R') & 0x0001) != 0;
            if (tapped && g_source != SRC_SYNTHETIC &&
                (GetAsyncKeyState(VK_CONTROL) & 0x8000) &&
                (GetAsyncKeyState(VK_MENU) & 0x8000)) {
                input_recenter();
                logf_("  xr: recenter requested by hotkey\r\n");
            }
        }

        if (bridge_on) {
            dg_bridge_drain_log();
            /* The probe rings empty every second, not every heartbeat: the
               dense mode fills its ring in under nine seconds of aiming. */
            drain_turn_probe();
            drain_adj_cycles();
            /* Only when something actually moved, so an idle session does not
               fill the log with identical blocks - and at most every 30 s, so a
               busy one does not either. */
            if (elapsed - last_summary >= 30) {
                DG_BRIDGE_STATS bs;
                dg_bridge_stats(&bs);
                /* The arm's visibility counts as movement too. Without this the
                   F4 observation would only ever be sampled on ticks where the
                   FPS machine happened to change state, which is precisely the
                   correlation the run is meant to establish rather than assume. */
                /* A newly seen pad bit counts as movement too, or the pad
                   instrument logs nothing at all: pressing keys changes
                   neither the FPS state nor the arm, so a session spent
                   hunting for the fire key would produce no summary to read
                   it in. */
                if (bs.fps_transitions != last_transitions ||
                    bs.arm_flag != last_arm_flag ||
                    bs.body_flag != last_body_flag ||
                    bs.arm_camera_on != last_cam_on ||
                    /* A visible tick is the event this instrument exists for,
                       so it wakes the heartbeat itself rather than waiting for
                       the next scheduled one to happen to land on it. */
                    (bs.arm_visible_ticks != 0) != (last_arm_visible != 0) ||
                    bs.seen_pad_status != last_pad_seen) {
                    log_bridge_summary("running");
                    last_transitions = bs.fps_transitions;
                    last_arm_flag = bs.arm_flag;
                    last_pad_seen = bs.seen_pad_status;
                    last_body_flag = bs.body_flag;
                    last_arm_visible = bs.arm_visible_ticks;
                    last_cam_on = bs.arm_camera_on;
                }
                last_summary = elapsed;
            }
        }

        if (!menu_session) { int src2, arm_track2, arm_qmap2[4], arm_hand2;
            DG_XR_CONFIG xc2; DG_BRIDGE_CONFIG bc2;
            ARM_POS_CFG ap2;
            if (!read_config(marker, &p, &seconds, &src2, &xc2, &bc2,
                             &arm_track2, arm_qmap2, &arm_hand2, &ap2)) {
                reason = "marker removed"; break;
            }
            if (src2 != source) {
                reason = "source changed - rearm required";
                break;
            }
            if (source == SRC_SCRIPT) {
                xc2.stereo = 0;
                xc2.stereo_test = 0;
                if (memcmp(&xc2, &xc, sizeof xc) != 0) {
                    reason = "script XR config changed - rearm required";
                    break;
                }
            }
            /* Folded in before the comparison below, so a button press
               restarts the stream through exactly the same path a marker edit
               does. Read once per pass; the count cannot go backwards, so a
               press between two passes is never lost. */
            ap2.hand_zero =
                arm_hand_zero_total(ap2.hand_zero,
                                    input_secondary_presses(
                                        arm_tracked_xr_hand(arm_track2)));
            if (bridge_on) dg_bridge_configure(&bc2);
            if(source==SRC_XR) {
                LONG choice=InterlockedCompareExchange(&g_mod_stereo_override,0,0);
                if(choice>=0)xc2.stereo=choice;
            }
            /* Unconditional: the trigger is not part of the pose stream, so a
               change to it must not have to wait for the arm config to move. */
            InterlockedExchange(&g_fire_mode, bc2.fire_mode);
            InterlockedExchange(&g_move_mode, bc2.move_mode);
            InterlockedExchange(&g_turn_mode, bc2.turn_mode);
            InterlockedExchange(&g_turn_gain_mils_live, bc2.turn_gain_mils);
            InterlockedExchange(&g_turn_deadzone_mils_live,
                                bc2.move_deadzone_mils);
            InterlockedExchange(&g_move_third_live, bc2.move_third);
            {
                LONG want_camera_yaw = bc2.camera_yaw_anchor == 1 ? 1 : 0;
                if (InterlockedExchange(&g_camera_yaw_anchor_mode,
                                        want_camera_yaw) != want_camera_yaw) {
                    InterlockedIncrement(&g_camera_yaw_anchor_generation);
                    logf_("  camera yaw: %s\r\n", want_camera_yaw ? "anchor" : "off");
                }
            }
            /* Recorder knobs, livest of all: they must work while everything
               else is mid-defect. A changed vr_rec_dump token dumps without
               touching anything; turning the recorder off dumps FIRST, so
               the knob can pause evidence but never destroy it. */
            {
                LONG tok = InterlockedCompareExchange(&g_rec_cfg_dump, 0, 0);
                LONG want = InterlockedCompareExchange(&g_rec_cfg_on, 0, 0);
                if (tok != rec_dump_token_seen) {
                    rec_dump_token_seen = tok;
                    rec_dump("vr_rec_dump token changed");
                }
                if (!want && InterlockedCompareExchange(&g_rec_on, 0, 0))
                    rec_dump("vr_rec turned off");
                InterlockedExchange(&g_rec_on, want);
            }
            /* The rolling snapshot rides the same 1 Hz pass as the knobs. */
            if (rec_snapshot_due((int)InterlockedCompareExchange(&g_rec_on,
                                                                 0, 0),
                                 g_rec.appended, elapsed,
                                 &rec_snap_appended, &rec_snap_elapsed))
                rec_snapshot();
            /* Hot-reload pickup, same cadence as every other knob: watches
               the version file next to the vr_policy DLL, stages and offers
               on a change, and logs adoptions the tick seam performed. All
               of it on this worker thread; the game threads only ever see
               the interlocked pending/active pointers. */
            dg_policy_poll(g_policy_path);
            if (arm_track2 != active_arm_track || arm_hand2 != active_arm_hand ||
                memcmp(arm_qmap2, active_arm_qmap,
                       sizeof active_arm_qmap) != 0 ||
                /* An axis sign is part of the mapping, so changing one
                   invalidates the anchor that was captured under the old one.
                   Without this a live sign flip leaves the arm calibrated to a
                   basis that no longer exists, and the knob reads as broken
                   when it is only stale. */
                xc2.x_sign != g_xrcfg.x_sign ||
                xc2.y_sign != g_xrcfg.y_sign ||
                xc2.z_sign != g_xrcfg.z_sign ||
                xc2.yaw_sign != g_xrcfg.yaw_sign ||
                xc2.pitch_sign != g_xrcfg.pitch_sign ||
                xc2.roll_sign != g_xrcfg.roll_sign ||
                /* Same argument one level out: the arm's own signs, its
                   anchor and the shoulder the anchor measures from all change
                   where a controller position lands, so a calibration taken
                   under the old ones describes a mapping that no longer
                   exists. */
                !arm_pos_cfg_same(&ap2, &active_arm_pos)) {

                g_arm_track = ARM_TRACK_OFF;
                dg_pose_init(&g_arm_pose_state, NULL);
                g_arm_pose_have_qpc = 0;
                g_arm_pose_stream_id++;
                memcpy(g_arm_qmap, arm_qmap2, sizeof arm_qmap2);
                memcpy(active_arm_qmap, arm_qmap2, sizeof active_arm_qmap);
                InterlockedExchange(&g_arm_hand, arm_hand2);
                active_arm_hand = arm_hand2;
                active_arm_track = arm_track2;
                g_arm_track = arm_track2;
                active_arm_pos = ap2;
                InterlockedExchange(&g_arm_hand_zero_now, ap2.hand_zero);
                arm_pos_cfg_apply(&ap2);
            }
            g_pose = p;
            /* Signs and recentre are live: pinning the axis mapping down means
               changing one number and moving your head, not restarting. */
            if (source == SRC_XR) {
                if(memcmp(&g_xrcfg,&xc2,sizeof xc2)!=0)render_link_reset();
                g_xrcfg = xc2;
                InterlockedExchange(&g_stereo, xc2.stereo != 0);
                InterlockedExchange(&g_scene_blur_enabled, xc2.scene_blur_fix);
                if(InterlockedExchange(&g_link_requested,xc2.stereo_phase_fix)!=xc2.stereo_phase_fix) {
                    InterlockedExchange(&g_render_link_active,g_link_requested&&g_source==SRC_XR&&
                        g_phase_stage_addr&&g_phase_consume_addr);
                    arm_all(1);
                }
                InterlockedExchange(&g_stereo_test, xc2.stereo_test);
                InterlockedExchange(&g_stereo_proj_x_sign,
                                    xc2.stereo_proj_x_sign);
                InterlockedExchange(&g_stereo_eye_x_sign, xc2.stereo_eye_x_sign);
                dg_ui2d_configure(xc2.ui2d, xc2.ui2d_scale_mils, xc2.ui2d_conv_e5, xc2.ui2d_sign, xc2.ui2d_vs, (unsigned)xc2.ui2d_vs_count);
                InterlockedExchange(&g_eye_truth_mode, xc2.eye_truth);
                InterlockedExchange(&g_draw_skip_mode, xc2.draw_skip);
                InterlockedExchange(&g_feedback_skip, xc2.feedback_skip);
                dg_ui2d_hold(xc2.ui2d_hold);
                dg_radar_configure(xc2.radar_mode != 0, xc2.radar_hud == 0);
                dg_bridge_native_hud_configure(xc2.radar_mode != 0 && xc2.radar_hud == 0);
                dg_xr_configure(&xc2);
            }
        }
        if (elapsed >= (int)seconds) break;
    }

session_cleanup:
    hud_stop();
    pp_flush(1,logf_);
    phase_finish("session_end");
    InterlockedExchange(&g_render_link_active,0);render_link_reset();
    controls_session_end(bridge_on);
    if (source == SRC_SCRIPT) dg_xr_script_stop();
    if (menu_session) script_menu_callbacks_stop();
    arm_all(0);
    Sleep(50);
    RemoveVectoredExceptionHandler(g_veh);
    g_veh = NULL;

    /* The session's last act: the ring becomes a file, so whatever just
       happened in the headset is replayable at a desk. After the VEH is
       gone, so nothing is appending. When it succeeds, the live snapshot
       THIS session wrote is deleted: with a snapshot every minute and a
       three-minute ring, everything the snapshot held is inside the final
       dump. A live file this session never wrote is left alone - if a
       previous run died hard, that file is its only evidence, and this
       session's clean ending is no reason to destroy it. */
    if (rec_dump(reason) && rec_snap_appended != 0) {
        char live[MAX_PATH];
        size_t dir = rec_log_dir(live, sizeof live);
        _snprintf_s(live + dir, sizeof live - dir, _TRUNCATE,
                    DG_REC_LIVE_NAME);
        if (DeleteFileA(live))
            logf_("  rec: live snapshot removed -"
                  " the final dump supersedes it\r\n");
    }

    logf_("  disarmed after %d s (%s)\r\n"
          "  traps %ld  applied %ld  (from fresh camera %ld, rebuilt %ld)\r\n"
          "  rejected: nonfinite %ld  not-orthonormal %ld  not-projection %ld"
          "  no-pose %ld\r\n",
          elapsed, reason, g_traps, g_applied, g_fresh, g_reapplied,
          g_rej_nonfinite, g_rej_not_ortho, g_rej_not_proj, g_rej_no_pose);

    if (bridge_on) {
        dg_bridge_drain_log();
        dg_bridge_stop();
        log_bridge_summary("final");
    }
    if (menu_session) {
        InterlockedExchange(&g_script_menu_active, 0);
        InterlockedExchange(&g_script_menu_context_lost, 0);
    }

    if (source == SRC_XR) {
        long f, v, iv, e;
        if (!g_xr_worker_owned) dg_xr_stop();
        dg_xr_stats(&f, &v, &iv, &e);
        logf_("  xr frames %ld  valid %ld  invalid %ld  errors %ld\r\n", f, v, iv, e);
    }
    return reason;
}

static int script_menu_preserve_neutral(int front_end, const DG_MENU_IN *in,
                                         const DG_MENU_OUT *out)
{
    return (g_source==SRC_XR || InterlockedCompareExchange(&g_script_menu_active, 0, 0)) &&
           !script_menu_blocked() && front_end && !out->write &&
           in->have_sample && in->input_ok &&
           !(out->flags & (DG_MENU_F_BAD_INPUT | DG_MENU_F_BAD_CONFIG | DG_MENU_F_YIELDED));
}

static int menu_frontend_allowed(int flat,int codec,int gameover,int capture)
{
    return !capture && (flat || codec || gameover);
}
static void menu_input_from_frame(DG_MENU_IN *in,const DG_XR_FRAME *f)
{
    const DG_XR_HAND *h;
    if(!f)return;
    h=InterlockedCompareExchange(&g_menu_hand_right,0,0)?&f->right_hand:&f->left_hand;
    in->have_sample=1;in->x=h->thumbstick_x;in->y=h->thumbstick_y;
    in->trigger=h->trigger_value;in->button_a=h->primary_button!=0;
    in->button_b=h->secondary_button!=0;
    in->input_ok=h->grip.active && h->grip.pose_age_ms<=100;
}
static void menu_seam(int front_end)
{
    DG_MENU_IN in;
    DG_MENU_OUT out;
    DG_XR_FRAME f;
    int mode = (int)InterlockedCompareExchange(&g_menu_mode, 0, 0);
    if (InterlockedCompareExchange(&g_mod_input_capture,0,0)) front_end=0;

    if (script_menu_blocked()) {
        DG_BRIDGE_MENU empty;
        memset(&empty, 0, sizeof empty);
        dg_bridge_menu_now(&empty);
        return;
    }
    if (mode == DG_MENU_MODE_OFF) return;

    memset(&in, 0, sizeof in);
    in.front_end = front_end ? 1 : 0;
    in.now_ms = GetTickCount();
    in.deadzone =
        (double)InterlockedCompareExchange(&g_menu_deadzone_mils, 0, 0) / 1000.0;
    in.initial_ms = (int)InterlockedCompareExchange(&g_menu_delay_ms, 0, 0);
    in.repeat_ms = (int)InterlockedCompareExchange(&g_menu_repeat_ms, 0, 0);
    in.confirm_bit =
        (unsigned int)InterlockedCompareExchange(&g_menu_confirm, 0, 0);
    /* In sweep the confirm bit IS the candidate, chosen before the module
       decides, so config_ok validates the same word that will be sent. */
    if (InterlockedCompareExchange(&g_menu_sweep_on, 0, 0)) {
        /* Skip any candidate that collides with the cancel bit, and skip it
           HERE rather than leaving it to config_ok. The module refuses an
           overlapping pair outright - correctly, since half a control
           scheme in a menu is worse than none - but the sweep only advances
           on an emitted confirm, so a colliding candidate refuses every
           frame, never advances, and locks the whole feature including
           navigation until the marker is edited. That deadlock cost a live
           run: 1275 bad-config frames on candidate 0x00000100 while cancel
           sat on the same bit. Bounded walk, so a cancel value that somehow
           matched every entry still leaves the loop. */
        LONG i = InterlockedCompareExchange(&g_menu_sweep_idx, 0, 0);
        unsigned int cancel =
            (unsigned int)InterlockedCompareExchange(&g_menu_cancel, 0, 0);
        int guard;
        for (guard = 0; guard < DG_MENU_SWEEP_N; guard++) {
            if (!(g_menu_sweep[(i + guard) % DG_MENU_SWEEP_N] & cancel))
                break;
        }
        if (guard < DG_MENU_SWEEP_N) {
            if (guard) InterlockedExchange(&g_menu_sweep_idx, i + guard);
            in.confirm_bit =
                g_menu_sweep[(i + guard) % DG_MENU_SWEEP_N];
        }
    }
    in.cancel_bit =
        (unsigned int)InterlockedCompareExchange(&g_menu_cancel, 0, 0);
    /* The same click threshold the trigger already uses everywhere else, and
       the same clamp: g_xrcfg.trigger_fire is validated at marker-parse time
       into (deadzone, 1.0], so it can never be the 0.0 that would make an
       untouched trigger read as held. */
    in.trigger_click = g_xrcfg.trigger_fire;

    if (input_get_frame(&f) & 1) menu_input_from_frame(&in,&f);

    /* Recorded before the decision, so it shows what the decision SAW. */
    if (in.have_sample) {
        union { float f; LONG l; } cv;
        cv.f = (float)in.trigger;
        InterlockedExchange(&g_menu_dbg_trigger, cv.l);
        cv.f = (float)in.trigger_click;
        InterlockedExchange(&g_menu_dbg_click, cv.l);
        InterlockedExchange(&g_menu_dbg_buttons,
                            (in.button_a ? 1 : 0) | (in.button_b ? 2 : 0) |
                            (in.input_ok ? 4 : 0));
        InterlockedIncrement(&g_menu_dbg_samples);
    }

    dg_menu_step(&g_menu_state, &in, &out);

    if (front_end) InterlockedIncrement(&g_menu_open_frames);
    if (out.flags & DG_MENU_F_YIELDED)    InterlockedIncrement(&g_menu_yields);
    if (out.flags & DG_MENU_F_NO_INPUT)   InterlockedIncrement(&g_menu_no_input);
    if (out.flags & DG_MENU_F_BAD_INPUT)  InterlockedIncrement(&g_menu_no_input);
    if (out.flags & DG_MENU_F_BAD_CONFIG) InterlockedIncrement(&g_menu_bad_config);
    if (out.flags & DG_MENU_F_STEP)       InterlockedIncrement(&g_menu_steps);
    if (out.flags & DG_MENU_F_REPEAT)     InterlockedIncrement(&g_menu_repeats);
    if (out.flags & DG_MENU_F_CONFIRM)    InterlockedIncrement(&g_menu_confirms);
    if (out.flags & DG_MENU_F_CANCEL)     InterlockedIncrement(&g_menu_cancels);
    /* MEASURE still ends here, and the counter still separates "the
       controller was never read" from "read, decided, and had nowhere to
       put it". In WRITE the decision is handed to the bridge, which owns
       the only seam these screens can be reached from - and which applies
       its own gate on the game thread, because `front_end` is also true
       while a weapon or item menu is open and those are out of scope. */
    if (mode >= DG_MENU_MODE_WRITE) {
        DG_BRIDGE_MENU cmd;
        cmd.status = out.write ? out.status : 0u;
        /* Only silence the competition on a CONFIRM. A cancel of ours has
           no rival to clear, and clearing on every frame would be us
           holding a button down for the player. */
        cmd.clear = (out.flags & DG_MENU_F_CONFIRM)
                        ? (unsigned int)InterlockedCompareExchange(
                              &g_menu_clear_bits, 0, 0)
                        : 0u;
        /* One candidate per confirm the player asks for. The bit is
           already in out.status - it was in.confirm_bit - so this only
           records what went and steps to the next. */
        if ((out.flags & DG_MENU_F_CONFIRM) &&
            InterlockedCompareExchange(&g_menu_sweep_on, 0, 0)) {
            LONG i = InterlockedCompareExchange(&g_menu_sweep_idx, 0, 0);
            InterlockedExchange(&g_menu_sweep_last,
                                (LONG)g_menu_sweep[i % DG_MENU_SWEEP_N]);
            InterlockedExchange(&g_menu_sweep_idx, i + 1);
        }
        cmd.allow = front_end ? 1 : 0;
        if (front_end && g_source==SRC_XR)
            cmd.allow=(out.flags&DG_MENU_F_CONFIRM)?DG_MENU_ALLOW_XR_CONFIRM:DG_MENU_ALLOW_XR;
        if (front_end && (out.flags & DG_MENU_F_CANCEL) &&
            dg_bridge_codec_input_now())
            cmd.allow = DG_CODEC_MENU_ALLOW_EXIT;
        if (script_menu_blocked()) memset(&cmd, 0, sizeof cmd);
        /* A valid neutral Present is not cancellation of a pending pulse.
           XR and the desktop menu session use the bounded bridge mailbox. */
        if (script_menu_preserve_neutral(front_end, &in, &out))
            return;
        dg_bridge_menu_now(&cmd);
        if (out.write) InterlockedIncrement(&g_menu_writes_asked);
    } else if (out.write) {
        InterlockedIncrement(&g_menu_unwritable);
    }
}

/* Runs on the game's render thread, inside Present. It hands over only the
   pose and FOV the camera hook recorded for this image; an invalid handoff
   leaves the eye store untouched rather than attaching an invented pose. */
static void on_present(IDXGISwapChain *sc) {
    AcquireSRWLockShared(&g_script_present_lock);
    if (!g_script_present_stopped) {
        pp_present();
        on_present_body(sc);
    }
    ReleaseSRWLockShared(&g_script_present_lock);
}

static void eye_dump_step(IDXGISwapChain *sc, int eye)
{
    LONG cfg = InterlockedCompareExchange(&g_eye_dump_cfg, 0, 0);
    ID3D11Texture2D *bb = NULL, *st = NULL; ID3D11Device *dev = NULL; ID3D11DeviceContext *ctx = NULL;
    D3D11_TEXTURE2D_DESC d; D3D11_MAPPED_SUBRESOURCE m; int ok = 0;
    if (!g_eye_dump_have_seen) { g_eye_dump_seen = cfg; g_eye_dump_have_seen = 1; }
    if (cfg != g_eye_dump_seen) {
        g_eye_dump_seen = cfg; g_eye_dump_left = DG_EYE_DUMP_FRAMES; g_eye_dump_tick = GetTickCount();
        dg_ui2d_trace(DG_EYE_DUMP_FRAMES - 1, g_bb_src_prev, g_bb_src_prev2);
    }
    if (g_eye_dump_left <= 0 || !sc) return;
    g_eye_dump_left--;
    if (FAILED(sc->lpVtbl->GetBuffer(sc, 0, &IID_ID3D11Texture2D, (void **)&bb)) || !bb) goto done;
    bb->lpVtbl->GetDesc(bb, &d);
    if (d.SampleDesc.Count != 1 || (d.Format != DXGI_FORMAT_B8G8R8A8_UNORM && d.Format != DXGI_FORMAT_R8G8B8A8_UNORM &&
        d.Format != DXGI_FORMAT_B8G8R8A8_UNORM_SRGB && d.Format != DXGI_FORMAT_R8G8B8A8_UNORM_SRGB)) goto done;
    bb->lpVtbl->GetDevice(bb, &dev); if (!dev) goto done;
    d.Usage = D3D11_USAGE_STAGING; d.BindFlags = 0; d.CPUAccessFlags = D3D11_CPU_ACCESS_READ | D3D11_CPU_ACCESS_WRITE; d.MiscFlags = 0;
    if (FAILED(dev->lpVtbl->CreateTexture2D(dev, &d, NULL, &st)) || !st) goto done;
    dev->lpVtbl->GetImmediateContext(dev, &ctx); if (!ctx) goto done;
    {   /* Validity of the dump itself: a fresh staging texture may reuse the previous dump's memory, so a copy the GPU
           SKIPPED (predicated rendering) would look like a repeated picture. Poison first; and read the predicate. */
        ID3D11Predicate *pred = NULL; BOOL pval = FALSE; unsigned yy;
        if (SUCCEEDED(ctx->lpVtbl->Map(ctx, (ID3D11Resource *)st, 0, D3D11_MAP_WRITE, 0, &m))) {
            for (yy = 0; yy < d.Height; yy++) memset((unsigned char *)m.pData + (size_t)yy * m.RowPitch, 0xAB, (size_t)d.Width * 4);
            ctx->lpVtbl->Unmap(ctx, (ID3D11Resource *)st, 0);
        }
        ctx->lpVtbl->GetPredication(ctx, &pred, &pval);
        g_eye_dump_pred = pred ? 1 + (pval ? 1 : 0) : 0;
        if (pred) pred->lpVtbl->Release(pred);
    }
    ctx->lpVtbl->CopyResource(ctx, (ID3D11Resource *)st, (ID3D11Resource *)bb);
    if (SUCCEEDED(ctx->lpVtbl->Map(ctx, (ID3D11Resource *)st, 0, D3D11_MAP_READ, 0, &m))) {
        char path[MAX_PATH]; FILE *f = NULL; size_t dir = rec_log_dir(path, sizeof path);
        unsigned w = d.Width / 4, h = d.Height / 4, x, y, head[4];
        _snprintf_s(path + dir, sizeof path - dir, _TRUNCATE, "dg_eye_%lu_%d_%s.raw", (unsigned long)g_eye_dump_tick,
                    DG_EYE_DUMP_FRAMES - 1 - g_eye_dump_left, eye == DG_EYE_RIGHT ? "R" : "L");
        if (w && h && fopen_s(&f, path, "wb") == 0 && f) {
            head[0] = w; head[1] = h; head[2] = (unsigned)(eye == DG_EYE_RIGHT); head[3] = (unsigned)g_present_frame;
            if (d.Format == DXGI_FORMAT_R8G8B8A8_UNORM || d.Format == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) head[2] |= 0x100;   /* RGBA order */
            fwrite(head, sizeof head, 1, f);
            for (y = 0; y < h; y++) {
                const unsigned char *row = (const unsigned char *)m.pData + (size_t)(y * 4) * m.RowPitch;
                for (x = 0; x < w; x++) fwrite(row + (size_t)x * 16, 4, 1, f);
            }
            fclose(f); ok = 1;
            {   /* picture fingerprint, so identical Presents show up in the log itself */
                unsigned long long fp = 1469598103934665603ull; long bbi = 0; void *bbsrc = NULL; long bbd = dg_ui2d_last_frame_bb(&bbi, &bbsrc);
                for (y = 0; y < h; y += 4) {
                    const unsigned char *row = (const unsigned char *)m.pData + (size_t)(y * 4) * m.RowPitch;
                    for (x = 0; x < w; x += 4) { fp ^= row[(size_t)x * 16]; fp *= 1099511628211ull; fp ^= row[(size_t)x * 16 + 1]; fp *= 1099511628211ull; fp ^= row[(size_t)x * 16 + 2]; fp *= 1099511628211ull; }
                }
                {   unsigned long poisoned = 0, total = 0;
                    for (y = 0; y < h; y += 4) { const unsigned char *row = (const unsigned char *)m.pData + (size_t)(y * 4) * m.RowPitch;
                        for (x = 0; x < w; x += 4) { const unsigned char *px = row + (size_t)x * 16; total++; if (px[0] == 0xAB && px[1] == 0xAB && px[2] == 0xAB && px[3] == 0xAB) poisoned++; } }
                    logf_("  eye dump %d: present %ld label %s  picture %016llX  blit source %p (bb draws %ld)  all draws %ld  POISON left %lu of %lu  predicate %s\r\n",
                          DG_EYE_DUMP_FRAMES - 1 - g_eye_dump_left, (long)g_present_frame, eye == DG_EYE_RIGHT ? "R" : "L", fp, bbsrc, bbd, dg_ui2d_last_frame_draws(),
                          poisoned, total, g_eye_dump_pred == 0 ? "none" : g_eye_dump_pred == 1 ? "SET (value FALSE)" : "SET (value TRUE)");
                }
            }
        }
        ctx->lpVtbl->Unmap(ctx, (ID3D11Resource *)st, 0);
    }
done:
    InterlockedIncrement(ok ? &g_eye_dump_written : &g_eye_dump_failed);
    if (ctx) ctx->lpVtbl->Release(ctx);
    if (st) st->lpVtbl->Release(st);
    if (dev) dev->lpVtbl->Release(dev);
    if (bb) bb->lpVtbl->Release(bb);
    if (!g_eye_dump_left) logf_("  eye dump: done, written %ld failed %ld (logs\\dg_eye_%lu_*.raw)\r\n",
                               (long)g_eye_dump_written, (long)g_eye_dump_failed, (unsigned long)g_eye_dump_tick);
}

static void on_present_body(IDXGISwapChain *sc) {
    DG_HOOK_HANDOFF handoff;
    int keep_eye = 0;
    DG_HOOK_HANDOFF linked_handoff;
    int linked_seq=0,linked_have=render_link_take(&linked_handoff,&linked_seq);
    int stereo = InterlockedCompareExchange(&g_stereo, 0, 0) != 0;
    if (sc) {   /* identity only: buffer 0 is the current back buffer of every swap effect */
        ID3D11Texture2D *bbt = NULL;
        if (SUCCEEDED(sc->lpVtbl->GetBuffer(sc, 0, &IID_ID3D11Texture2D, (void **)&bbt)) && bbt) {
            dg_ui2d_backbuffer_ptr((void *)bbt); bbt->lpVtbl->Release(bbt);
        }
    }
    int armed  = g_armed;
    int eye = (InterlockedCompareExchange(&g_current_eye, 0, 0) == DG_EYE_RIGHT)
        ? DG_EYE_RIGHT : DG_EYE_LEFT;
    int got = handoff_read(&handoff);
    int have = got && handoff.valid;
    int screen;
    int flat = 0;
    phase_record(4,-1);

    script_menu_sample_context();
    if (script_menu_blocked()) {
        DG_BRIDGE_MENU empty;
        controller_context_publish(0);
        memset(&empty, 0, sizeof empty);
        dg_bridge_menu_now(&empty);
        InterlockedIncrement(&g_present_frame);
        return;
    }

    dg_bridge_screen_seam_now();
    /* The title/start screens keep Present running but pause the camera and
       gameplay seams.  Consume the XR menu rising edge here so the pad seam
       can publish START on the next UpdatePad pass. */
    start_command();
    screen = dg_bridge_theater_verdict();

    /* Camera silence, counted. `have` is false for every frame the camera
       hook did not vouch for, and on the title and load screens that is
       EVERY frame forever - there is no game camera to hook. See the
       g_flat_* block: without this the runtime gets zero layers and the
       headset is black exactly where the player is trying to read a menu. */
    /* Script frames intentionally carry no headset FOV and therefore cannot
       create an XR capture handoff. Detect real camera-seam activity instead
       of misclassifying every mono gameplay frame as a front-end screen. */
    if (g_source == SRC_SCRIPT) {
        LONG camera_now = InterlockedCompareExchange(&g_traps, 0, 0);
        have = camera_now != g_script_camera_seen;
        g_script_camera_seen = camera_now;
        if (InterlockedCompareExchange(&g_script_menu_active, 0, 0))
            have = 0; /* Incidental title/intro camera visits are not gameplay. */
    }
    if (have) {
        InterlockedExchange(&g_flat_silent, 0);
    } else if (armed && (stereo || g_source == SRC_SCRIPT)) {
        if (InterlockedCompareExchange(&g_flat_silent, 0, 0) <
            DG_FLAT_SILENT_FRAMES * 4)
            InterlockedIncrement(&g_flat_silent);
    }
    if (InterlockedCompareExchange(&g_flat_mode, 0, 0) &&
        (stereo || g_source == SRC_SCRIPT) && armed &&
        InterlockedCompareExchange(&g_flat_silent, 0, 0) >=
            DG_FLAT_SILENT_FRAMES) {
        flat = 1;
        screen = 1;
        InterlockedIncrement(&g_flat_frames);
    }

    /* U2. `flat` IS the front-end condition - a title or load screen is an
       image with no game camera behind it, which is exactly what the block
       above just established. Called every Present, including the ones where
       it is false, so the module sees the window close. Reads only. */
    {
        LONG camera_now=InterlockedCompareExchange(&g_traps,0,0);
        int raw_gameplay=armed && !flat && !screen &&
            !InterlockedCompareExchange(&g_script_menu_active,0,0) &&
            dg_bridge_controller_gameplay_now();
        int raw_special=(armed && !flat && !screen &&
            !InterlockedCompareExchange(&g_script_menu_active,0,0)) ?
            dg_bridge_controller_special_now():0;
        int raw_radial=armed && !flat && !screen &&
            !InterlockedCompareExchange(&g_script_menu_active,0,0) &&
            dg_bridge_controller_radial_now();
        dg_bridge_action_context(action_present_allowed(armed,flat,
            InterlockedCompareExchange(&g_script_menu_active,0,0),
            controller_camera_context(camera_now,raw_gameplay,GetTickCount()),
            dg_bridge_hatch_now()) ||
            (armed && !flat && !screen && !InterlockedCompareExchange(&g_script_menu_active,0,0) &&
             dg_bridge_action_ladder_now()));
        controller_context_publish_all(
            controller_camera_context(camera_now,raw_gameplay,GetTickCount()),
            controller_camera_context(camera_now,raw_special,GetTickCount())?raw_special:0,
            controller_camera_context(camera_now,raw_radial,GetTickCount()));
        if (g_source==SRC_XR) {
            int toggle=dg_xr_controller_toggle_take();
            if (g_buttons_gameplay && toggle && !InterlockedCompareExchange(&g_mod_input_capture,0,0)) dg_bridge_request_toggle();
        } else script_primary_command();
    }
    menu_seam(menu_frontend_allowed(flat,dg_bridge_codec_input_now(),
        dg_bridge_menu_gameover_now(),InterlockedCompareExchange(&g_mod_input_capture,0,0)));

    /* Desktop scripts have no XR device, swapchains or layer submission.
       Keep Present's game-frame identity while avoiding every capture path. */
    if (g_source == SRC_SCRIPT) {
        InterlockedIncrement(&g_present_frame);
        InterlockedExchange(&g_current_eye, DG_EYE_LEFT);
        return;
    }

    /* The same single verdict every consumer reads. Told to the XR side
       every frame - the frame loop owns the edges (store drops, anchoring) -
       and the capture below takes the MONO path with the live game FOV,
       overriding stereo: the director's frame is one flat image, read per
       frame because cutscenes animate the projection. */
    auto_recenter_present(armed && !flat && !screen);
    if(!armed || flat || screen)dg_radar_view(0);
    dg_xr_screen(screen);
    /* Only while alternate-eye gameplay is what reaches the headset; cutscenes (theater), menus and mono keep their blur. */
    dg_ui2d_feedback(InterlockedCompareExchange(&g_feedback_skip, 0, 0) && stereo && armed && have && !screen && g_source == SRC_XR);
    if(screen||!armed||!stereo||g_source!=SRC_XR)render_link_reset();
    if (screen) {
        const MAT *pers = (const MAT *)(g_chan0 + O_PERS);
        DG_PROJ_FOV live;
        if (!g_chan0 || !projlike(pers)) {
            if (flat) {
                /* A title or load screen has no game camera and therefore
                   no projection to recover - which is exactly why nothing
                   ever reached the headset here. The quad is a flat panel,
                   so the FOV only decides how large it hangs; a plain 45
                   degrees is an honest default rather than a guess at
                   something the frame never had. Without this the fallback
                   would refuse its own frames through dg_xr_capture's
                   no-FOV rule and change nothing at all. */
                DG_PROJ_FOV flat_fov;
                flat_fov.right = 45.0 * 3.14159265358979323846 / 360.0;
                flat_fov.left = -flat_fov.right;
                flat_fov.up = flat_fov.right * 9.0 / 16.0;
                flat_fov.down = -flat_fov.up;
                dg_xr_capture(sc, DG_EYE_MONO, NULL, &flat_fov);
                InterlockedIncrement(&g_thea_mono_frames);
            } else {
                /* No honest FOV: keep the store as it is rather than label
                   a frame with a guess. The quad shows the last good
                   frame. */
                dg_xr_capture(sc, DG_EYE_MONO, NULL, NULL);
            }
        } else {
            live.right = atan(1.0 / fabs((double)pers->m[0][0]));
            live.left  = -live.right;
            live.up    = atan(1.0 / fabs((double)pers->m[1][1]));
            live.down  = -live.up;
            dg_xr_capture(sc, DG_EYE_MONO, NULL, &live);
            InterlockedIncrement(&g_thea_mono_frames);
        }
    } else if (have && stereo && armed) {
        if(g_source==SRC_XR&&g_link_requested) {
            if(!linked_have) {
                InterlockedIncrement(&g_link_refused);
                phase_link_record(NULL,0);
                dg_xr_capture(sc,eye,NULL,NULL);
                goto present_done;
            }
            handoff=linked_handoff;
            InterlockedIncrement(&g_link_accepted);
            phase_link_record(&handoff,linked_seq);
        }
        if (g_source == SRC_XR) {
            int label = handoff.eye == DG_EYE_RIGHT ? 1 : 0;
            int frame_sign = dg_ui2d_last_frame_sign();
            {
                long traps = (long)InterlockedCompareExchange(&g_traps, 0, 0);
                long seq = (long)InterlockedCompareExchange(&g_handoff_seq, 0, 0);
                DWORD now = GetTickCount();
                char det[400]; long scene = frame_sign ? 0 : dg_ui2d_last_frame_detail(det, sizeof det);
                /* a paused game (a handful of draws) must not use up the budget */
                if (!frame_sign && (scene >= 300 ? (++g_nocam_seen <= DG_NOCAM_LOG_MAX || g_nocam_seen % 100 == 0)
                                                 : ++g_nocam_small_seen <= 10)) {
                    logf_("  nocam frame #%ld: present %ld label %s  camera-updates since last Present %ld  handoff writes %ld  dt %lu ms  %s\r\n",
                          g_nocam_seen, (long)g_present_frame, handoff.eye == DG_EYE_RIGHT ? "R" : "L",
                          traps - g_nocam_traps_last, (seq - g_nocam_seq_last) / 2, (unsigned long)(now - g_nocam_tick_last), det);
                }
                {
                    long op = 0, on = 0, ticks = traps - g_nocam_traps_last; DWORD dtm = now - g_nocam_tick_last;
                    int os = dg_ui2d_last_frame_obj(&op, &on), lab = handoff.eye == DG_EYE_RIGHT ? 1 : 0;
                    g_obj_n[lab][os == 1 ? 0 : os == -1 ? 1 : os == 2 ? 2 : 3]++;
                    g_obj_ticks[ticks < 0 ? 0 : ticks > 3 ? 3 : ticks]++;
                    g_obj_dt[dtm < 20 ? 0 : dtm < 40 ? 1 : 2]++;
                    if (os == 1 || os == -1) {
                        if (g_obj_prev == os) {
                            g_obj_same++; g_obj_run++; if (g_obj_run > g_obj_run_max) g_obj_run_max = g_obj_run;
                            if (++g_obj_events <= 60 || g_obj_events % 50 == 0)
                                logf_("  obj-eye SAME #%ld: present %ld label %s  objects +%ld/-%ld  c29 sign %+d  ticks %ld  dt %lu ms  run %ld\r\n",
                                      g_obj_events, (long)g_present_frame, lab ? "R" : "L", op, on, frame_sign, ticks, (unsigned long)dtm, g_obj_run);
                        } else { if (g_obj_prev) g_obj_alt++; g_obj_run = 0; }
                        g_obj_prev = os;
                    }
                }
                g_nocam_traps_last = traps; g_nocam_seq_last = seq; g_nocam_tick_last = now;
            }
            {
                long fd = dg_ui2d_last_frame_draws(), dop = 0, don = 0;
                int dos = dg_ui2d_last_frame_obj(&dop, &don);
                LONG dmode = InterlockedCompareExchange(&g_draw_skip_mode, 0, 0);
                int skipped = draw_skip_detect(fd, &g_draw_ema);
                {
                    long bbi = 0; void *bbsrc = NULL; long bbd = dg_ui2d_last_frame_bb(&bbi, &bbsrc);
                    if (bbd >= 0) {
                        g_bb_hist[bbd + bbi == 0 ? 0 : bbd + bbi < 8 ? 1 : 2]++;
                        if (bbd + bbi > 0) { if (g_bb_seen < 1000000) g_bb_seen++; }
                        if (bbsrc) { if (bbsrc == g_bb_src_prev) g_bb_src_same++; else { g_bb_src_alt++; g_bb_src_prev2 = g_bb_src_prev; } g_bb_src_prev = bbsrc; }
                        if (bbd + bbi == 0 && g_bb_seen >= 100) {
                            skipped = 1; g_bb_empty++;
                        }
                        if ((bbd + bbi == 0 || g_bb_events < 12) && (++g_bb_events <= 40 || g_bb_events % 500 == 0))
                            logf_("  bb probe #%ld: present %ld label %s  back-buffer draws %ld (+%ld sampled indexed)  source %p  all draws %ld  objects +%ld/-%ld\r\n",
                                  g_bb_events, (long)g_present_frame, label ? "R" : "L", bbd, bbi, bbsrc, fd, dop, don);
                    }
                }
                g_draw_hist[fd < 50 ? 0 : fd < 300 ? 1 : fd < 1000 ? 2 : 3]++;
                if (skipped) {
                    g_draw_recent = 120;
                    if (++g_draw_skipped <= 20 || g_draw_skipped % 500 == 0)
                        logf_("  draw skip #%ld: present %ld label %s  draws %ld (usual %ld)  objects +%ld/-%ld  mode %ld\r\n",
                              g_draw_skipped, (long)g_present_frame, label ? "R" : "L", fd, g_draw_ema, dop, don, (long)dmode);
                    if (dmode >= 1) {
                        g_draw_held++;
                        dg_xr_capture(sc, eye, NULL, NULL);
                        keep_eye = 1;
                        goto present_done;
                    }
                } else {
                    if (g_draw_recent > 0) g_draw_recent--;
                    if (!g_draw_recent && (dos == 1 || dos == -1) && g_draw_map[label][dos < 0] < 1000000) g_draw_map[label][dos < 0]++;
                    if (dmode == 2 && (dos == 1 || dos == -1)) {
                        int want = draw_skip_label(g_draw_map, dos);
                        if (want >= 0 && want != label && g_draw_have[want]) {
                            handoff = g_draw_last[want]; label = want; g_draw_relabelled++;
                        } else if (want < 0 || want == label) { g_draw_last[label] = handoff; g_draw_have[label] = 1; }
                    } else { g_draw_last[label] = handoff; g_draw_have[label] = 1; }
                }
            }
            int verdict = eye_truth_step(handoff.eye, frame_sign);
            LONG mode = InterlockedCompareExchange(&g_eye_truth_mode, 0, 0);
            if (nocam_drop_step(frame_sign, mode, &g_nocam_run)) {
                InterlockedIncrement(&g_nocam_dropped);
                dg_xr_capture(sc, eye, NULL, NULL);
                goto present_done;
            }
            if (!frame_sign) InterlockedIncrement(&g_nocam_passed);
            if (mode == 3) mode = 1;
            if (verdict >= 0) { g_eye_truth_last[label] = handoff; g_eye_truth_have[label] = 1; }
            else if (mode == 1) {
                InterlockedIncrement(&g_eye_truth_dropped);
                dg_xr_capture(sc, eye, NULL, NULL);
                goto present_done;
            } else if (mode == 2 && g_eye_truth_have[!label]) {
                handoff = g_eye_truth_last[!label];
                InterlockedIncrement(&g_eye_truth_relabelled);
            }
        }
        handoff.near_meta.present_id = (unsigned)g_present_frame;
        ci_record(3,-1,-1,&handoff,0); /* exact arguments, not latest mailbox */
        eye_dump_step(sc, handoff.eye);
        dg_xr_capture_measured(sc, handoff.eye, &handoff.raw, &handoff.fov, &handoff.near_meta);
    }
    else if (have && !stereo)
        dg_xr_capture(sc, DG_EYE_MONO, NULL, &handoff.fov);
    else if (armed && stereo)
        /* The ONE case that must not fall back. A frame the camera hook
           rejected was drawn from the game's own camera; storing it would
           label non-eye pixels with an eye and submit them under that eye's
           pose. Leave the store holding the last honest image instead. */
        dg_xr_capture(sc, eye, NULL, NULL);
    else {
        /* Everything else falls back to what S4b did, and that path has to keep
           working: the present hook running with the camera hook INERT is how
           the picture first reached the headset (2.23), and it is still the way
           to see whether submission is healthy independently of the camera.
           Nobody records a FOV there, so recover the game's own from the live
           projection - read per frame, never cached, because cutscenes animate
           it (2.2). */
        const MAT *pers = (const MAT *)(g_chan0 + O_PERS);
        DG_PROJ_FOV live;
        if (!g_chan0 || !projlike(pers)) {
            dg_xr_capture(sc, DG_EYE_MONO, NULL, NULL);
        } else {
            live.right = atan(1.0 / fabs((double)pers->m[0][0]));
            live.left  = -live.right;
            live.up    = atan(1.0 / fabs((double)pers->m[1][1]));
            live.down  = -live.up;
            dg_xr_capture(sc, DG_EYE_MONO, NULL, &live);
        }
    }

    /* Present is definitionally one game frame. Rejected camera frames still
       advance, otherwise one eye would starve until the hook recovers. Mono
       ignores the counter, but advancing it there keeps a live stereo toggle
       aligned to the real frame sequence too. */
present_done:
    InterlockedIncrement(&g_present_frame);
    if (!keep_eye) InterlockedExchange(&g_current_eye, next_eye(eye));
}

/* The game builds its device some time after our hooks go in, so this waits
   rather than racing it. */
static int adopt_game_device(void) {
    DWORD waited = 0;
    while (waited < 60000) {
        ID3D11Device *dev = NULL;
        IDXGISwapChain *sc = NULL;
        if (dg_present_get_device(&dev, &sc) && dev) {
            int ok = dg_xr_adopt_device(dev);
            dev->lpVtbl->Release(dev);
            if (sc) sc->lpVtbl->Release(sc);
            return ok;
        }
        if (dev) dev->lpVtbl->Release(dev);
        if (sc) sc->lpVtbl->Release(sc);
        Sleep(250);
        waited += 250;
    }
    return 0;
}

static DWORD WINAPI worker(LPVOID unused) {
    char exe[MAX_PATH], dir[MAX_PATH], marker[MAX_PATH], pmarker[MAX_PATH];
    char xmarker[MAX_PATH], *leaf;
    int present_on = 0;
    int script_boot = 0;
    int idle = 0;

    (void)unused;
    g_self_tid = GetCurrentThreadId();

    GetModuleFileNameA(NULL, exe, sizeof(exe));
    leaf = strrchr(exe, '\\'); leaf = leaf ? leaf + 1 : exe;
    if (_stricmp(leaf, HOST_EXE) != 0) return 0;

    strcpy_s(dir, sizeof(dir), exe);
    { char *p = strrchr(dir, '\\'); if (p) *p = 0; }
    _snprintf_s(marker, sizeof(marker), _TRUNCATE, "%s\\%s", dir, MARKER);
    _snprintf_s(pmarker, sizeof(pmarker), _TRUNCATE, "%s\\%s", dir, PRESENT_MARKER);
    _snprintf_s(xmarker, sizeof(xmarker), _TRUNCATE, "%s\\%s", dir, XR_MARKER);
    _snprintf_s(g_logpath, sizeof(g_logpath), _TRUNCATE, "%s\\%s", dir, LOG_NAME);

    /* Hot-reload policy: the game's own directory is handed over as the one
       place a policy DLL may never be loaded from - the code-level backstop
       of "never touch game files while the game runs". */
    dg_policy_setup(dir, logf_);

    g_base      = (ULONG64)GetModuleHandleW(NULL);
    g_hook_addr = g_base + HOOK_RVA;
    g_chan0     = g_base + DG_CHANLS_RVA;

    logf_("\r\n=== dg_hook ready ===\r\n"
          "  module base   %016llX\r\n"
          "  hook          %016llX (rva 0x%llX)\r\n"
          "  DG_Chanls[0]  %016llX\r\n"
          "  waiting for %s\r\n",
          g_base, g_hook_addr, HOOK_RVA, g_chan0, MARKER);

    /* S4a. This has to go in now rather than when a session arms: the game
       builds its D3D11 device during startup, and an IAT hook that arrives
       after that has already missed it. Gated on its own file so that a launch
       without one is byte-for-byte the S3 behaviour - the D3D11 hook and the
       camera hook are independently switchable, which is what makes a bad
       interaction between them diagnosable. */
    if (marker_present(pmarker)) {
        present_on = dg_present_start(logf_);
        logf_("  present hook %s\r\n",
              present_on ? "installed" : "FAILED - continuing without it");
    } else {
        logf_("  present hook off (%s absent)\r\n", PRESENT_MARKER);
    }
    g_present_available = present_on;
    {
        DG_SCRIPT_MARKER_INFO si;
        POSE p;
        double seconds = 600.0;
        int source, track, qmap[4], hand;
        DG_XR_CONFIG xc;
        DG_BRIDGE_CONFIG bc;
        ARM_POS_CFG ap;
        if (read_config(marker, &p, &seconds, &source, &xc, &bc,
                        &track, qmap, &hand, &ap))
            script_boot = source == SRC_SCRIPT;
        dg_script_gate_init(&g_script_gate, -1);
        {
            int ok = read_script_marker(marker, &si);
            dg_script_gate_poll(&g_script_gate, ok, &si);
        }
        g_script_boot = script_boot;
        if (script_boot)
            logf_("  script: waiting for a new positive vr_script_start token\r\n");
    }
    if (present_on && script_boot) {
        InterlockedExchange(&g_source, SRC_SCRIPT);
        dg_present_set_callback(on_present);
        logf_("  script: desktop Present wired; OpenXR startup suppressed\r\n");
    }

    /* S4b. Submission needs the Present hook, so it is gated on that first;
       the two markers together are what puts a picture in the headset, and
       either one alone leaves the game exactly as it was. */
    if (present_on && !script_boot && marker_present(xmarker)) {
        DG_XR_CONFIG xc;
        memset(&xc, 0, sizeof(xc));
        xc.yaw_sign = xc.pitch_sign = xc.roll_sign = 1.0;
        xc.x_sign = xc.y_sign = xc.z_sign = 1.0;
        xc.scale = 1000.0;              /* plan 2.9: 1 unit = 1 mm */
        xc.positional = 1;
        xc.stereo = 0;
        xc.stereo_test = 0;
        if (!adopt_game_device()) {
            logf_("  xr submission: game device never appeared\r\n");
        } else {
            dg_xr_configure(&xc);
            if (dg_xr_start(logf_)) {
                InterlockedExchange(&g_xr_worker_owned, 1);
                dg_present_set_callback(on_present);
                logf_("  xr submission: started, capture wired to Present\r\n");
            } else {
                logf_("  xr submission: dg_xr_start failed\r\n");
            }
        }
    } else if (present_on && !script_boot) {
        logf_("  xr submission off (%s absent)\r\n", XR_MARKER);
    }

    /* Arm and disarm as often as the marker appears and disappears. A run that
       ends on its own timer must not immediately restart, so wait for the file
       to go away first - otherwise "10 minutes" would mean "forever". */
    for (;;) {
        if (script_boot) {
            DG_SCRIPT_MARKER_INFO si;
            int ok = read_script_marker(marker, &si);
            int decision = dg_script_gate_poll(&g_script_gate, ok, &si);
            if (decision <= 0) {
                if (decision < 0)
                    logf_("  script: baseline %d; waiting for a higher vr_script_start\r\n",
                          g_script_gate.high_water);
                Sleep(500);
                if (present_on && ++idle >= 60) {
                    long pr, er;
                    dg_present_stats(&pr, &er);
                    logf_("  script: waiting; baseline %d, present %ld, errors %ld\r\n",
                          g_script_gate.high_water, pr, er);
                    idle = 0;
                }
                continue;
            }
            g_script_token_active = g_script_gate.active;
            logf_("  script: start requested, token %d\r\n", g_script_token_active);
        } else {
            while (!marker_present(marker)) {
                Sleep(500);
                if (present_on && ++idle % 60 == 0) {
                    long pr, er;
                    dg_present_stats(&pr, &er);
                    logf_("  present %ld  errors %ld\r\n", pr, er);
                }
            }
        }
        {
            const char *reason;
            idle = 0;
            reason = run_once(marker);
            logf_("  worker: run_once returned: %s\r\n", reason);
            if (script_boot) {
                /* Any positive token observed during the run is already in
                   the gate high-water mark. Finish discards a queued retry;
                   a later offer therefore requires a genuinely newer token. */
                DG_SCRIPT_MARKER_INFO si;
                int ok = read_script_marker(marker, &si);
                dg_script_gate_observe(&g_script_gate, ok && si.source_script,
                                        ok && si.token_valid, si.token);
                dg_script_gate_finish(&g_script_gate);
                g_script_token_active = 0;
                logf_("  script: waiting for vr_script_start greater than %d\r\n",
                      g_script_gate.high_water);
            } else if (strcmp(reason, "marker removed") != 0) {
                while (marker_present(marker)) Sleep(500);
            }
        }
    }
}

#ifdef DG_HOOK_TEST
#include "dg_ik.h"                  /* the desk replay runs the real solver */
#include "dg_arm_map.h"             /* and the real map, on a stated skeleton */
/* The anchor resolver, for its own self-test only. Test-side because the
   shipping half of this file never touches an image - dg_bridge.c owns the
   scan - and pulling the whole scanner into the .asi to run a test would be
   the wrong trade. The header is include-guarded, so the bridge's own copy of
   its statics is unaffected. */
#include "../shared/dg_anchors.h"
#include "camera_yaw_measured.h"
/* dg_menu_test.c is compiled into this binary with DG_MENU_TEST_EMBED, so the
   menu module's assertions are the SAME ones menu_test.bat runs. */
int dg_menu_self_test(void);

static void compose_ypr(MAT *o, double y, double p, double r) {
    MAT ry, rx, rz, t;
    rot_y(&ry, y); rot_x(&rx, p); rot_z(&rz, r);
    mat_mul(&t, &ry, &rx);
    mat_mul(o, &t, &rz);
}

/* dg_xr_mat_to_ypr must invert exactly the composition build_delta performs.
   If it inverts some other Euler order the error is zero for single-axis
   motion and grows with combined motion - i.e. invisible until it is being
   worn on a head. */
static int test_ypr_roundtrip(void) {
    static const double cases[][3] = {
        {   0,   0,   0 }, {  30,   0,   0 }, {   0,  25,   0 },
        {   0,   0, -15 }, {  40, -20,  10 }, { -75,  35, -50 },
        { 179,   0,   0 }, {  10,  89.9,  0 },
    };
    int n = (int)(sizeof(cases) / sizeof(cases[0])), i, bad = 0;
    for (i = 0; i < n; i++) {
        MAT r;
        double y, p, rr, worst;
        compose_ypr(&r, cases[i][0]*DEG2RAD, cases[i][1]*DEG2RAD, cases[i][2]*DEG2RAD);
        dg_xr_mat_to_ypr(&r, &y, &p, &rr);
        worst = fabs(y - cases[i][0]*DEG2RAD);
        if (fabs(p  - cases[i][1]*DEG2RAD) > worst) worst = fabs(p  - cases[i][1]*DEG2RAD);
        if (fabs(rr - cases[i][2]*DEG2RAD) > worst) worst = fabs(rr - cases[i][2]*DEG2RAD);
        if (worst > 1e-4) bad++;
        printf("  %-6s ypr(%7.1f,%6.1f,%7.1f) -> err %.2e\n",
               worst <= 1e-4 ? "ok" : "FAIL",
               cases[i][0], cases[i][1], cases[i][2], worst);
    }
    return bad;
}

/* The handedness fix, checked two ways.
   PURITY: a rotation about exactly one OpenXR axis must stay a rotation about
   exactly one MGS2 axis. Leakage means the mirror or the transpose is wrong,
   and no amount of per-axis sign flipping will rescue it.
   DIRECTION: the sign must make the camera follow the head, not oppose it.
   Expected signs, derived in dg_xr.c's header and each checkable by hand:
     XR +Y is a head turning LEFT (right-handed about +Y sends -Z forward to
       -X), which must give MGS2 yaw +30: rot_y(+30) sends a point 90 deg
       right to 60 deg right, i.e. the view swung left with the head.
     XR +X pitches the head UP, and MGS2 pitch -30 is what follows it. This
       one is a MEASUREMENT, not a derivation: on 2026-08-19 a headset reported
       that with the opposite sign here, looking up sent the view down. The
       argument that used to justify +30 rested on assuming which way MGS2's
       view X and Y point, which nothing had ever established - see the header
       of dg_xr.c.
     XR +Z rolls, and it comes out with the same sign as the head for the same
       reason and from the same measurement: the relabelling that inverts pitch
       inverts roll with it. */
static int test_axis_purity(void) {
    static const struct { const char *name; int axis; int want; double sign; } t[] = {
        { "XR +X (pitch)", 0, 1, -1.0 },
        { "XR +Y (yaw)",   1, 0, +1.0 },
        { "XR +Z (roll)",  2, 2, +1.0 },
    };
    DG_XR_CONFIG cfg;
    int i, bad = 0;
    const double ang = 30.0 * DEG2RAD;

    memset(&cfg, 0, sizeof(cfg));
    cfg.yaw_sign = cfg.pitch_sign = cfg.roll_sign = 1.0;
    cfg.x_sign = cfg.y_sign = cfg.z_sign = 1.0;
    cfg.scale = 1000.0;
    cfg.positional = 0;

    for (i = 0; i < 3; i++) {
        double q[4] = {0,0,0,0}, out[3], on, off = 0.0;
        DG_XR_POSE pose;
        int k;
        q[t[i].axis] = sin(ang * 0.5);
        q[3] = cos(ang * 0.5);
        dg_xr_head_to_pose(q[0], q[1], q[2], q[3], 0,0,0, &cfg, &pose);
        out[0] = pose.yaw; out[1] = pose.pitch; out[2] = pose.roll;
        on = out[t[i].want];
        for (k = 0; k < 3; k++) if (k != t[i].want && fabs(out[k]) > off) off = fabs(out[k]);
        {
            int ok = fabs(on - t[i].sign * ang) <= 1e-4 && off <= 1e-4;
            if (!ok) bad++;
            printf("  %-6s %-14s -> yaw %+7.2f  pitch %+7.2f  roll %+7.2f deg"
                   "   (want %+.0f, leak %.2e)\n",
                   ok ? "ok" : "FAIL",
                   t[i].name, out[0]/DEG2RAD, out[1]/DEG2RAD, out[2]/DEG2RAD,
                   t[i].sign * ang / DEG2RAD, off);
        }
    }
    return bad;
}

/* With the head both rotating and displaced, D's translation row is -t_c * R,
   never t_c. Check the round trip: a point at the head's new position must
   land at the view-space origin. */
static int test_translation_composition(void) {
    DG_XR_CONFIG cfg;
    DG_XR_POSE xp;
    POSE p;
    MAT d, di;
    double ang = 40.0 * DEG2RAD;
    double q[4], v[4], worst = 0.0;
    int j;

    memset(&cfg, 0, sizeof(cfg));
    cfg.yaw_sign = cfg.pitch_sign = cfg.roll_sign = 1.0;
    cfg.x_sign = cfg.y_sign = cfg.z_sign = 1.0;
    cfg.scale = 1000.0;
    cfg.positional = 1;

    q[0] = 0; q[1] = sin(ang*0.5); q[2] = 0; q[3] = cos(ang*0.5);
    /* head 0.25 m right, 0.1 m up, 0.3 m forward (-Z in OpenXR) */
    dg_xr_head_to_pose(q[0], q[1], q[2], q[3], 0.25, 0.10, -0.30, &cfg, &xp);

    memset(&p, 0, sizeof(p));
    p.yaw = xp.yaw; p.pitch = xp.pitch; p.roll = xp.roll;
    p.tx = xp.tx;  p.ty = xp.ty;  p.tz = xp.tz;
    build_delta(&d, &di, &p);

    /* The head's position in OLD view coords, mirrored to MGS2 mm. */
    v[0] = 0.25 * 1000.0; v[1] = 0.10 * 1000.0; v[2] = 0.30 * 1000.0; v[3] = 1.0;
    for (j = 0; j < 3; j++) {
        double s = v[0]*d.m[0][j] + v[1]*d.m[1][j] + v[2]*d.m[2][j] + d.m[3][j];
        if (fabs(s) > worst) worst = fabs(s);
    }
    printf("  %-6s head position maps to new-view origin, residual %.3e mm\n",
           worst < 1e-2 ? "ok" : "FAIL", worst);
    return worst < 1e-2 ? 0 : 1;
}

/* F3 camera telemetry: run the production transform against a local channel
   fixture, then verify the published sample is the final camera->world matrix
   and final projection.  The asymmetric basis and literal result keep this
   independent of the implementation's matrix products. */
static int test_camera_telemetry_transform(void)
{
    MAT fake[64], base_eye, base_eye_inv;
    MAT saved_mine, saved_src_eye_inv, saved_src_eye;
    MAT saved_cam_eye, saved_cam_proj;
    POSE saved_pose;
    ULONG64 saved_chan0 = g_chan0;
    LONG saved_source = g_source;
    LONG saved_have_mine = g_have_mine;
    LONG saved_have_proj = g_have_proj_mine;
    LONG saved_applied = g_applied, saved_fresh = g_fresh;
    LONG saved_rej_ortho = g_rej_not_ortho;
    LONG saved_cam_valid = g_camera_telemetry_valid;
    int i, j, bad = 0;
    static const float want[3][3] = {
        { 0.0f,  0.0f,  1.0f },
        {-1.0f,  0.0f,  0.0f },
        { 0.0f, -1.0f,  0.0f }
    };

    memset(fake, 0, sizeof fake);
    for (i = 0; i < (int)(sizeof fake / sizeof fake[0]); i++) {
        fake[i].m[0][0] = 1.0f;
        fake[i].m[1][1] = 1.0f;
        fake[i].m[2][2] = 1.0f;
        fake[i].m[3][3] = 1.0f;
    }
    memset(&base_eye, 0, sizeof base_eye);
    base_eye.m[0][1] = 1.0f;
    base_eye.m[1][0] = -1.0f;
    base_eye.m[2][2] = 1.0f;
    base_eye.m[3][3] = 1.0f;
    memset(&base_eye_inv, 0, sizeof base_eye_inv);
    base_eye_inv.m[0][1] = -1.0f;
    base_eye_inv.m[1][0] = 1.0f;
    base_eye_inv.m[2][2] = 1.0f;
    base_eye_inv.m[3][3] = 1.0f;
    g_chan0 = (ULONG64)(ULONG_PTR)fake;
    *M(O_EYE) = base_eye;
    *M(O_EYE_INV) = base_eye_inv;
    M(O_PERS)->m[0][0] = 1.7f;
    M(O_PERS)->m[1][1] = 2.3f;
    M(O_PERS)->m[2][3] = 1.0f;
    M(O_PERS)->m[3][3] = 0.0f;

    saved_mine = g_mine;
    saved_src_eye_inv = g_src_eye_inv;
    saved_src_eye = g_src_eye;
    saved_cam_eye = g_camera_telemetry_eye_world;
    saved_cam_proj = g_camera_telemetry_proj;
    saved_pose = g_pose;
    InterlockedExchange(&g_source, SRC_SYNTHETIC);
    memset(&g_pose, 0, sizeof g_pose);
    g_pose.yaw = 90.0 * DEG2RAD;
    InterlockedExchange(&g_have_mine, 0);
    InterlockedExchange(&g_have_proj_mine, 0);
    camera_pass_begin();
    apply_transform();
    if (!InterlockedCompareExchange(&g_camera_telemetry_valid, 0, 0))
        bad++;
    for (i = 0; i < 3; i++) for (j = 0; j < 3; j++)
        if (fabs((double)g_camera_telemetry_eye_world.m[i][j] - want[i][j]) > 1e-5)
            bad++;
    if (memcmp(&g_camera_telemetry_proj, M(O_PERS), sizeof(MAT)) != 0)
        bad++;
    if (memcmp(&g_camera_telemetry_eye_world, &base_eye, sizeof(MAT)) == 0)
        bad++;
    if (memcmp(&g_camera_telemetry_eye_world,
               &fake[O_EYE_INV / sizeof(MAT)], sizeof(MAT)) == 0)
        bad++;

    /* A later rejected seam starts with the same production helper and must
       leave the previous successful telemetry invalid. */
    M(O_EYE_INV)->m[0][0] = 2.0f;
    camera_pass_begin();
    apply_transform();
    if (InterlockedCompareExchange(&g_camera_telemetry_valid, 0, 0)) bad++;

    g_chan0 = saved_chan0;
    InterlockedExchange(&g_source, saved_source);
    g_pose = saved_pose;
    g_mine = saved_mine;
    g_src_eye_inv = saved_src_eye_inv;
    g_src_eye = saved_src_eye;
    g_camera_telemetry_eye_world = saved_cam_eye;
    g_camera_telemetry_proj = saved_cam_proj;
    InterlockedExchange(&g_have_mine, saved_have_mine);
    InterlockedExchange(&g_have_proj_mine, saved_have_proj);
    InterlockedExchange(&g_applied, saved_applied);
    InterlockedExchange(&g_fresh, saved_fresh);
    InterlockedExchange(&g_rej_not_ortho, saved_rej_ortho);
    InterlockedExchange(&g_camera_telemetry_valid, saved_cam_valid);
    printf("  %-6s camera telemetry: final n_eye basis captured, reject clears, "
           "inverse is not reported\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int test_camera_step_height(void)
{
    MAT raw, world, inv, d, di, eye, product;
    POSE p;
    int i,j,k,bad=0;
    memset(&p,0,sizeof p);
    for (i=0;i<120;i++) {
        float ground=(float)i*2.0f;
        float head_y=(float)(30.0*cos(i*0.1));
        memset(&raw,0,sizeof raw);
        raw.m[0][0]=raw.m[1][1]=raw.m[2][2]=raw.m[3][3]=1;
        raw.m[3][0]=(float)i*7; raw.m[3][2]=50;
        raw.m[3][1]=ground+1500+(float)(50.0*sin(i*0.4));
        world=raw; camera_inverse_from_world(&inv,&world);
        if (!camera_step_height_apply(&world,&inv,1,1,ground+1500)) bad++;
        for (j=0;j<4;j++) for (k=0;k<4;k++)
            if (!(j==3 && k==1) && world.m[j][k]!=raw.m[j][k]) bad++;
        /* The same production composition follows stabilization: real head Y
           and left/right eye offsets must survive the removed 50 mm bob. */
        p.ty=head_y; p.tx=(i&1)?32:-32;
        build_delta(&d,&di,&p); mat_mul(&eye,&di,&world);
        if (fabs(eye.m[3][1]-(ground+1500-head_y))>0.001 ||
            fabs(eye.m[3][0]-(raw.m[3][0]-p.tx))>0.001) bad++;
        mat_mul(&product,&inv,&world);
        for(j=0;j<4;j++) for(k=0;k<4;k++)
            if(fabs(product.m[j][k]-(j==k?1.0:0.0))>0.001) bad++;
        eye=world;
        if (!camera_step_height_apply(&world,&inv,1,1,ground+1500) ||
            memcmp(&eye,&world,sizeof eye)) bad++;
    }
    for(i=0;i<4;i++) {
        MAT saved_inv;
        world=raw; camera_inverse_from_world(&inv,&world); saved_inv=inv;
        if(camera_step_height_apply(&world,&inv,i!=0,i!=1,
             i==2?raw.m[3][1]+251:i==3?NAN:1500) ||
             memcmp(&world,&raw,sizeof raw) || memcmp(&inv,&saved_inv,sizeof inv)) bad++;
    }
    printf("  %-6s camera step transform: 120 walking/terrain/head/stereo frames, rigid inverse, repeat and bypass checks\n",bad?"FAIL":"ok");
    return bad;
}

#include "dg_hanging_camera_test.inl"
static int test_camera_yaw_anchor(void)
{
    int i, j, k, have, bad = 0;
    double h;
    for (i = 0; i < (int)(sizeof(camera_yaw_measured) /
                          sizeof(camera_yaw_measured[0])); i++) {
        MAT first, cur, expected, di;
        POSE test_pose;
        h = 0.0;
        have = 0;
        memset(&first, 0, sizeof first);
        memset(&cur, 0, sizeof cur);
        for (j = 0; j < 3; j++) {
            memcpy(first.m[j], camera_yaw_measured[i].initial_base[j],
                   3 * sizeof(float));
            memcpy(cur.m[j], camera_yaw_measured[i].current_base[j],
                   3 * sizeof(float));
        }
        first.m[3][3] = cur.m[3][3] = 1.0f;
        if (!camera_yaw_anchor_apply(&first, &h, &have) ||
            !camera_yaw_anchor_apply(&cur, &h, &have)) bad++;
        memset(&test_pose, 0, sizeof test_pose);
        test_pose.yaw = camera_yaw_measured[i].mapped_yaw_deg * DEG2RAD;
        build_delta(&expected, &di, &test_pose);
        mat_mul(&expected, &di, &cur);
        for (j = 0; j < 3; j++) for (k = 0; k < 3; k++)
            if (fabs(expected.m[j][k] - camera_yaw_measured[i].expected_eye[j][k]) > 2e-4)
                bad++;
    }
    {
        MAT tilted, want, ry;
        double q = 0.04, n = sqrt(1.0 - q*q), curh;
        memset(&tilted, 0, sizeof tilted);
        tilted.m[0][0] = 1.0f; tilted.m[1][0] = (float)q;
        tilted.m[1][1] = (float)n; tilted.m[2][2] = 1.0f; tilted.m[3][3] = 1.0f;
        if (camera_rigid_valid(&tilted)) bad++;
        memset(&tilted, 0, sizeof tilted);
        tilted.m[0][0] = 1.0f;
        tilted.m[1][1] = 0.8660254f; tilted.m[1][2] = 0.5f;
        tilted.m[2][1] = -0.5f; tilted.m[2][2] = 0.8660254f;
        tilted.m[3][0] = 100.0f; tilted.m[3][1] = 200.0f;
        tilted.m[3][2] = 300.0f; tilted.m[3][3] = 1.0f;
        curh = atan2(tilted.m[2][0], tilted.m[2][2]);
        h = curh + 90.0 * DEG2RAD; have = 1;
        want = tilted; rot_y(&ry, 90.0 * DEG2RAD); mat_mul(&want, &tilted, &ry);
        want.m[3][0] = tilted.m[3][0]; want.m[3][1] = tilted.m[3][1];
        want.m[3][2] = tilted.m[3][2];
        if (!camera_yaw_anchor_apply(&tilted, &h, &have)) bad++;
        for (j = 0; j < 3; j++) for (i = 0; i < 3; i++)
            if (fabs(tilted.m[j][i] - want.m[j][i]) > 2e-4) bad++;
    }
    {
        static const float turns[4][2] = {
            {0,1}, {0,-1}, {-1,0}, {-0.9998476952f,-0.0174524064f}
        };
        MAT m;
        for (i = 0; i < 4; i++) {
            memset(&m, 0, sizeof m);
            m.m[0][0] = m.m[2][2] = turns[i][0];
            m.m[2][0] = turns[i][1]; m.m[0][2] = -turns[i][1];
            m.m[1][1] = m.m[3][3] = 1;
            have = 1;
            h = i == 3 ? 179.0 * DEG2RAD : 0.0;
            if (!camera_yaw_anchor_apply(&m, &h, &have)) bad++;
            if (i == 3) {
                if (fabs(m.m[2][0] - 0.0174524064) > 2e-5 ||
                    fabs(m.m[2][2] + 0.9998476952) > 2e-5) bad++;
            } else if (fabs(m.m[0][0] - 1) > 2e-5 ||
                       fabs(m.m[2][2] - 1) > 2e-5 ||
                       fabs(m.m[2][0]) > 2e-5) bad++;
        }
    }
    printf("  %-6s camera yaw anchor: measured fixtures, row order, wrap and rigid reject\n",
           bad ? "FAIL" : "ok");
    return bad;
}

static int test_camera_yaw_apply_integration(void)
{
    MAT fake[64], raw, raw_inv, current, current_inv, d, di;
    POSE save_pose, p;
    ULONG64 save_chan = g_chan0;
    LONG save_src = g_source, save_mode = g_camera_yaw_anchor_mode;
    LONG save_gen = g_camera_yaw_anchor_generation;
    LONG save_mine = g_have_mine, save_proj = g_have_proj_mine;
    DG_CAMERA_GATE save_gate = g_test_camera_gate;
    MAT save_mine_mat = g_mine, save_src_inv = g_src_eye_inv, save_src_world = g_src_eye;
    MAT save_cam_eye = g_camera_telemetry_eye_world, save_cam_proj = g_camera_telemetry_proj;
    LONG save_cam_valid = g_camera_telemetry_valid;
    int save_override = g_test_camera_gate_override, i, j, bad = 0;
    double c = cos(30.0 * DEG2RAD), s = sin(30.0 * DEG2RAD);
    static const double want_pos[3] = {115.9807621135,177.0096189432,275.1794919243};
    static const float want_r[3][3] = {
        {0.0f,-0.5f,0.8660254038f},
        {0.5f,0.75f,0.4330127019f},
        {-0.8660254038f,0.4330127019f,0.25f}
    };
    memset(fake, 0, sizeof fake);
    for (i = 0; i < 64; i++) { fake[i].m[0][0]=fake[i].m[1][1]=fake[i].m[2][2]=fake[i].m[3][3]=1.0f; }
    memset(&raw, 0, sizeof raw);
    raw.m[0][0]=1.0f; raw.m[1][1]=(float)c; raw.m[1][2]=(float)s;
    raw.m[2][1]=(float)-s; raw.m[2][2]=(float)c;
    raw.m[3][0]=100.0f; raw.m[3][1]=200.0f; raw.m[3][2]=300.0f; raw.m[3][3]=1.0f;
    camera_inverse_from_world(&raw_inv, &raw);
    rot_y(&d, 90.0 * DEG2RAD); mat_mul(&current, &raw, &d);
    current.m[3][0]=raw.m[3][0]; current.m[3][1]=raw.m[3][1]; current.m[3][2]=raw.m[3][2];
    camera_inverse_from_world(&current_inv, &current);
    memset(&p, 0, sizeof p); p.yaw=90.0*DEG2RAD; p.pitch=30.0*DEG2RAD;
    p.tx=10.0; p.ty=20.0; p.tz=30.0;
    save_pose=g_pose; g_pose=p; g_chan0=(ULONG64)(ULONG_PTR)fake;
    InterlockedExchange(&g_source, SRC_SYNTHETIC); InterlockedExchange(&g_camera_yaw_anchor_mode,1);
    InterlockedIncrement(&g_camera_yaw_anchor_generation); InterlockedExchange(&g_have_mine,0);
    InterlockedExchange(&g_have_proj_mine,0); g_test_camera_gate_override=1;
    memset(&g_test_camera_gate,0,sizeof g_test_camera_gate); g_test_camera_gate.valid=1;
    g_test_camera_gate.arm_body=0x10000; g_test_camera_gate.camera=0x20000;
    *M(O_EYE)=raw; *M(O_EYE_INV)=raw_inv; M(O_PERS)->m[0][0]=1.7f; M(O_PERS)->m[1][1]=2.3f;
    M(O_PERS)->m[2][3]=1.0f; M(O_PERS)->m[3][3]=0.0f;
    apply_transform();
    *M(O_EYE)=current; *M(O_EYE_INV)=current_inv;
    apply_transform();
    for (i=0;i<3;i++) for (j=0;j<3;j++) if (fabs(M(O_EYE)->m[i][j]-want_r[i][j])>3e-4) bad++;
    for (i=0;i<3;i++) if (fabs(M(O_EYE)->m[3][i]-want_pos[i])>3e-3) bad++;
    {
        MAT product;
        mat_mul(&product, M(O_EYE_INV), M(O_EYE));
        for (i=0;i<4;i++) for (j=0;j<4;j++)
            if (fabs(product.m[i][j] - (i == j ? 1.0 : 0.0)) > 3e-3) bad++;
    }

    for (j = 0; j < 3; j++) {
        MAT previous = *M(O_EYE);
        apply_transform();
        if (memcmp(&previous, M(O_EYE), sizeof previous)) bad++;
        if (memcmp(&g_src_eye, &current, sizeof current)) bad++;
    }
    /* Each event starts from a proven anchor, then presents a native 90-degree
       turn. Reset means this frame equals the existing legacy composition;
       keeping the old heading would instead reproduce want_r above. */
    for (j = 0; j < 9; j++) {
        MAT legacy, legacy_inv;
        int a, b;
        /* Independent case setup: a preceding case must not accidentally
           leave precisely the heading expected AFTER this case's reset. */
        camera_yaw_anchor_reset();
        InterlockedExchange(&g_camera_yaw_anchor_mode, 1);
        g_test_camera_gate_override = 1;
        InterlockedIncrement(&g_camera_yaw_anchor_generation);
        InterlockedExchange(&g_have_mine, 0);
        *M(O_EYE) = raw; *M(O_EYE_INV) = raw_inv;
        M(O_PERS)->m[2][3] = 1.0f;
        apply_transform();
        if (!g_camera_yaw_anchor_have) bad++;
        for (a = 0; a < 3; a++) for (b = 0; b < 3; b++)
            if (fabs(M(O_EYE)->m[a][b] - want_r[a][b]) > 3e-4) bad++;
        *M(O_EYE) = current; *M(O_EYE_INV) = current_inv;
        InterlockedExchange(&g_have_mine, 0);
        switch (j) {
        case 0: InterlockedExchange(&g_camera_yaw_anchor_mode, 0); break;
        case 1: g_test_camera_gate_override = 0; break;
        case 2: g_test_camera_gate.camera++; break;
        case 3: g_test_camera_gate.arm_body++; break;
        case 4: g_camera_yaw_anchor_recenter = input_recenter_count() - 1; break;
        case 5: InterlockedIncrement(&g_camera_yaw_anchor_generation); break;
        case 6: M(O_EYE_INV)->m[0][0] = 2.0f; break;
        case 7: M(O_PERS)->m[2][3] = 0.0f; break;
        case 8: {
            unsigned int nan_bits = 0x7FC00000u;
            memcpy(&M(O_EYE_INV)->m[0][0], &nan_bits, sizeof nan_bits);
            break;
        }
        }
        apply_transform();
        if (j >= 6) {
            if (g_camera_yaw_anchor_have || g_camera_telemetry_valid) bad++;
            *M(O_EYE) = current; *M(O_EYE_INV) = current_inv;
            M(O_PERS)->m[2][3] = 1.0f;
            InterlockedExchange(&g_have_mine, 0);
            apply_transform();
        }
        build_delta(&d, &di, &p);
        mat_mul(&legacy, &di, &current);
        mat_mul(&legacy_inv, &current_inv, &d);
        for (a = 0; a < 4; a++) for (b = 0; b < 4; b++) {
            if (fabs(M(O_EYE)->m[a][b] - legacy.m[a][b]) > 3e-3 ||
                fabs(M(O_EYE_INV)->m[a][b] - legacy_inv.m[a][b]) > 3e-3) {
                printf("  FAIL camera epoch event %d element %d,%d\n", j, a, b);
                bad++;
            }
        }
        if (j == 0 && (memcmp(M(O_EYE), &legacy, sizeof legacy) ||
                       memcmp(M(O_EYE_INV), &legacy_inv, sizeof legacy_inv))) bad++;
    }
    /* Gate loss falls back to legacy and must clear anchor state for the next
       safe epoch; an identity change then captures a fresh first frame. */
    g_test_camera_gate_override=0; apply_transform();
    if (g_camera_yaw_anchor_have) bad++;
    g_test_camera_gate_override=save_override; g_test_camera_gate=save_gate;
    g_chan0=save_chan; g_pose=save_pose; InterlockedExchange(&g_source,save_src);
    InterlockedExchange(&g_camera_yaw_anchor_mode,save_mode); InterlockedExchange(&g_camera_yaw_anchor_generation,save_gen);
    InterlockedExchange(&g_have_mine,save_mine); InterlockedExchange(&g_have_proj_mine,save_proj);
    g_mine=save_mine_mat; g_src_eye_inv=save_src_inv; g_src_eye=save_src_world;
    g_camera_telemetry_eye_world=save_cam_eye; g_camera_telemetry_proj=save_cam_proj;
    InterlockedExchange(&g_camera_telemetry_valid,save_cam_valid);
    camera_yaw_anchor_reset();
    printf("  %-6s camera yaw apply: literal tilt/head/translation, inverse, cache, off, gate, identities, recenter, generation and invalid epochs\n", bad?"FAIL":"ok");
    return bad;
}

static int test_render_link_config(void) {
    const char *words[]={"source=xr","source=xr vr_stereo_phase_fix=0",
        "source=xr vr_stereo_phase_fix=1","source=xr vr_stereo_phase_fix=0oops",
        "source=xr vr_stereo_phase_fix=2"};
    int i,bad=0;
    for(i=0;i<5;i++) {
        char b[128];POSE pose;double seconds=600;int source,track,map[4],hand,ok;
        ARM_POS_CFG ap;DG_XR_CONFIG cfg;DG_BRIDGE_CONFIG bridge;
        strcpy_s(b,sizeof b,words[i]);
        ok=parse_config(b,&pose,&seconds,&source,&cfg,&bridge,&track,map,&hand,&ap);
        if(ok!=(i<3)||(ok&&cfg.stereo_phase_fix!=(i!=1)))bad++;
    }
    printf("  %s render-link config: default/on/off and malformed-token refusal\n",bad?"FAIL":"PASS");
    return bad;
}
/* vr_radar_*: defaults (everything off, HUD radar left alone), whole-word
   switches, all-or-nothing triples, out-of-range keeps the default. */
static int test_radar_config(void) {
    static const struct { const char *text; int mode, hud, local; double size, off[3], rot[3], gaze, pitch; } t[] = {
        { "source=xr",                                   0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist=fixed",              1, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist=wrist vr_radar_hud=off vr_radar_wrist_space=local", 2, 0, 1, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist=WRIST vr_radar_hud=on vr_radar_wrist_space=grip",   2, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist=wristy vr_radar_hud=offf vr_radar_wrist_space=loc", 0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist=on",                 0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist=wrist vr_radar_wrist=off", 0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist_size=0.12 vr_radar_wrist_offset=0.03,-0.01,0.09 vr_radar_wrist_rot=10,-80.5,180 vr_radar_gaze_deg=0 vr_radar_gaze_pitch=-5 vr_radar_wrist=wrist",
                                                         2, 1, 0, 0.12, { 0.03, -0.01, 0.09 }, { 10, -80.5, 180 }, 0, -5 },
        { "source=xr vr_radar_wrist_size=5 vr_radar_gaze_deg=91 vr_radar_gaze_pitch=61", 0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist_size=0.02 vr_radar_gaze_deg=-1",                     0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist_offset=0.03,0.01 vr_radar_wrist_rot=1,2",            0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist_offset=0.03,0.01,0.5 vr_radar_wrist_rot=1,2,400",    0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist_offset=0.03,0.01,0.05,7 vr_radar_wrist_rot=1,2,3x",  0, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
        { "source=xr vr_radar_wrist_offset=0.03,x,0.05 vr_radar_wrist=fixed stereo=1",   1, 1, 0, 0.09, { 0.02, 0.08, 0.03 }, { 0, 90, -90 }, 35, 15 },
    };
    int i, j, bad = 0, stereo_kept = 0;
    for (i = 0; i < (int)(sizeof t / sizeof t[0]); i++) {
        char b[320]; POSE pose; double seconds = 600; int source, track, map[4], hand, ok, row = 0;
        ARM_POS_CFG ap; DG_XR_CONFIG cfg; DG_BRIDGE_CONFIG bridge;
        strcpy_s(b, sizeof b, t[i].text);
        ok = parse_config(b, &pose, &seconds, &source, &cfg, &bridge, &track, map, &hand, &ap);
        if (!ok || cfg.radar_mode != t[i].mode || cfg.radar_hud != t[i].hud || cfg.radar_space_local != t[i].local ||
            cfg.radar_size != t[i].size || cfg.radar_gaze_deg != t[i].gaze || cfg.radar_gaze_pitch != t[i].pitch) row = 1;
        for (j = 0; j < 3; j++) if (cfg.radar_offset[j] != t[i].off[j] || cfg.radar_rot[j] != t[i].rot[j]) row = 1;
        if (ok && cfg.stereo) stereo_kept = 1;          /* a refused triple must not swallow the keys after it */
        if (row) { printf("  FAIL radar config row %d: %s\n", i, t[i].text); bad++; }
    }
    if (!stereo_kept) bad++;
    printf("  %s radar config: defaults off, whole words, all-or-nothing triples, ranges\n", bad ? "FAIL" : "PASS");
    return bad;
}
/* The eye shift: the pair must end up exactly the separation apart along
   the finished camera's x-axis, with zero vertical/depth component in that
   camera's own basis, on a rolled AND pitched camera - the case the old
   per-eye mapping got wrong by 23 mm - and eye/eye_inv must stay inverses. */
static int test_stereo_eye_shift(void) {
    MAT ry, rx, rz, t, world, left, right, left_inv, right_inv, id;
    double half = 30.85, d[3], c[3];
    int bad = 0, i, j, sign;
    const char *words[] = {"source=xr", "source=xr stereo_eye_x_sign=-1",
                           "source=xr stereo_eye_x_sign=+1", "source=xr stereo_eye_x_sign=0"};
    int want[] = {-1, -1, 1, 1};
    for (i = 0; i < 4; i++) {
        char b[128]; POSE pose; double seconds = 600; int source, track, map[4], hand, ok;
        ARM_POS_CFG ap; DG_XR_CONFIG cfg; DG_BRIDGE_CONFIG bridge;
        strcpy_s(b, sizeof b, words[i]);
        ok = parse_config(b, &pose, &seconds, &source, &cfg, &bridge, &track, map, &hand, &ap);
        if (!ok || cfg.stereo_eye_x_sign != want[i]) bad++;
    }
    rot_y(&ry, 0.6); rot_x(&rx, -0.5); rot_z(&rz, 0.35);
    mat_mul(&t, &ry, &rx); mat_mul(&world, &t, &rz);
    world.m[3][0] = 100.0f; world.m[3][1] = 200.0f; world.m[3][2] = 300.0f;
    for (sign = 1; sign >= -1; sign -= 2) {
        left = world; right = world;
        camera_inverse_from_world(&left_inv, &left);
        camera_inverse_from_world(&right_inv, &right);
        stereo_eye_shift(&left, &left_inv, DG_EYE_LEFT, half, sign);
        stereo_eye_shift(&right, &right_inv, DG_EYE_RIGHT, half, sign);
        /* Rotation rows untouched. */
        for (i = 0; i < 3; i++) for (j = 0; j < 4; j++)
            if (left.m[i][j] != world.m[i][j] || right.m[i][j] != world.m[i][j]) bad++;
        /* right - left, in the camera's own basis: (sign * 2 * half, 0, 0). */
        for (j = 0; j < 3; j++) d[j] = (double)right.m[3][j] - (double)left.m[3][j];
        for (i = 0; i < 3; i++)
            c[i] = d[0]*world.m[i][0] + d[1]*world.m[i][1] + d[2]*world.m[i][2];
        if (fabs(c[0] - sign * 2.0 * half) > 1e-3 || fabs(c[1]) > 1e-3 || fabs(c[2]) > 1e-3) bad++;
        /* Each eye sits half away from the head, symmetrically. */
        for (j = 0; j < 3; j++) d[j] = (double)right.m[3][j] + (double)left.m[3][j] - 2.0 * world.m[3][j];
        if (fabs(d[0]) > 1e-3 || fabs(d[1]) > 1e-3 || fabs(d[2]) > 1e-3) bad++;
        /* eye * eye_inv == identity for both. */
        mat_mul(&id, &left, &left_inv);
        for (i = 0; i < 4; i++) for (j = 0; j < 4; j++)
            if (fabs((double)id.m[i][j] - (i == j ? 1.0 : 0.0)) > 1e-4) bad++;
        mat_mul(&id, &right, &right_inv);
        for (i = 0; i < 4; i++) for (j = 0; j < 4; j++)
            if (fabs((double)id.m[i][j] - (i == j ? 1.0 : 0.0)) > 1e-4) bad++;
    }
    /* Implausible pairs are refused, plausible ones halved, in game units. */
    {
        DG_XR_FRAME f; memset(&f, 0, sizeof f);
        f.eye[1].raw.px = 0.0617;
        if (fabs(stereo_half_separation(&f, 1000.0) - 30.85) > 1e-6) bad++;
        f.eye[1].raw.px = 0.0; f.eye[1].raw.py = 0.0617;
        if (fabs(stereo_half_separation(&f, 1000.0) - 30.85) > 1e-6) bad++;
        f.eye[1].raw.py = 0.02;
        if (stereo_half_separation(&f, 1000.0) != 0.0) bad++;
        f.eye[1].raw.py = 0.2;
        if (stereo_half_separation(&f, 1000.0) != 0.0) bad++;
        left = world; camera_inverse_from_world(&left_inv, &left);
        stereo_eye_shift(&left, &left_inv, DG_EYE_LEFT, 0.0, 1);
        if (memcmp(&left, &world, sizeof world) != 0) bad++;
    }
    printf("  %s stereo eye shift: pair along camera x, symmetric, inverse kept, sign knob, plausibility\n", bad ? "FAIL" : "PASS");
    return bad;
}
static int test_eye_truth(void) {
    int bad = 0, i;
    memset(g_eye_truth_n, 0, sizeof g_eye_truth_n);
    g_eye_truth_none = g_eye_truth_mismatch = 0;
    /* no camera this frame: no opinion, counted apart */
    if (eye_truth_step(DG_EYE_LEFT, 0) != 0 || g_eye_truth_none != 1) bad++;
    /* still learning: no verdict before 50 samples, not even on a stray */
    for (i = 0; i < 49; i++) if (eye_truth_step(DG_EYE_LEFT, +1) != 0) bad++;
    if (eye_truth_step(DG_EYE_LEFT, -1) != 0) bad++;
    /* learned: left renders +, a - frame under the left label is a mismatch */
    if (eye_truth_step(DG_EYE_LEFT, +1) != 1) bad++;
    if (eye_truth_step(DG_EYE_LEFT, -1) != -1 || g_eye_truth_mismatch != 1) bad++;
    /* the right eye learns on its own and the other way round */
    for (i = 0; i < 50; i++) eye_truth_step(DG_EYE_RIGHT, -1);
    if (eye_truth_step(DG_EYE_RIGHT, -1) != 1 || eye_truth_step(DG_EYE_RIGHT, +1) != -1) bad++;
    /* no clear majority (labels are noise): never call a frame mislabelled */
    memset(g_eye_truth_n, 0, sizeof g_eye_truth_n);
    for (i = 0; i < 200; i++) eye_truth_step(DG_EYE_LEFT, (i & 1) ? 1 : -1);
    if (eye_truth_step(DG_EYE_LEFT, +1) != 0 || eye_truth_step(DG_EYE_LEFT, -1) != 0) bad++;
    {   /* draw skip: a frame with almost no draws against a learned usual count; never before a usual count exists */
        long ema = 0, map[2][2] = {{0,0},{0,0}}; int k;
        if (draw_skip_detect(3, &ema)) bad++;
        for (k = 0; k < 10; k++) if (draw_skip_detect(1700, &ema)) bad++;
        if (ema != 1700 || !draw_skip_detect(3, &ema) || !draw_skip_detect(120, &ema) || draw_skip_detect(400, &ema)) bad++;
        if (draw_skip_label(map, 1) != -1) bad++;
        map[0][0] = 60; map[1][1] = 70; map[1][0] = 2;
        if (draw_skip_label(map, 1) != 0 || draw_skip_label(map, -1) != 1 || draw_skip_label(map, 0) != -1) bad++;
        map[1][0] = 40; if (draw_skip_label(map, 1) != -1) bad++;
    }
    {   /* camera-less frames: dropped only in mode 3, at most DG_NOCAM_DROP_MAX in a row, a voted frame rearms */
        int run = 0, k, dropped = 0;
        if (nocam_drop_step(0, 0, &run) || nocam_drop_step(0, 1, &run) || nocam_drop_step(1, 3, &run)) bad++;
        for (k = 0; k < 10; k++) dropped += nocam_drop_step(0, 3, &run);
        if (dropped != DG_NOCAM_DROP_MAX) bad++;
        if (nocam_drop_step(-1, 3, &run) || run != 0 || !nocam_drop_step(0, 3, &run)) bad++;
    }
    memset(g_eye_truth_n, 0, sizeof g_eye_truth_n);
    g_eye_truth_none = g_eye_truth_mismatch = 0;
    printf("  %s eye truth: learns sign per label eye, flags the exception, silent without majority\n", bad ? "FAIL" : "PASS");
    return bad;
}
static int test_capture_gate(void) {
    int bad = 0;
    struct { const char *name; int have_ref, req, worn; unsigned dwell; int want; } t[] = {
        { "not worn",       0, 0, 0,   999, 0 },
        { "resume sticky",  1, 0, 1, 9999, 0 },
        { "explicit unworn",1, 1, 0,   0, 1 },
        { "explicit no dwell",0, 1, 1, 0, 1 },
        { "dwell below",     0, 0, 1, 499, 0 },
        { "dwell boundary",  0, 0, 1, 500, 1 },
    };
    int i, n = (int)(sizeof(t) / sizeof(t[0]));
    for (i = 0; i < n; i++) {
        int got = dg_xr_should_capture(t[i].have_ref, t[i].req,
                                       t[i].worn, t[i].dwell);
        if (got != t[i].want) bad++;
        printf("  %-6s capture gate %-18s -> %d (want %d)\n",
               got == t[i].want ? "ok" : "FAIL", t[i].name, got, t[i].want);
    }
    return bad;
}

static int test_stereo_alternation(void) {
    int bad = 0, eye = DG_EYE_LEFT, i, next;
    for (i = 0; i < 8; i++) {
        next = next_eye(eye);
        if (next < DG_EYE_LEFT || next > DG_EYE_RIGHT || next == eye) bad++;
        eye = next;
    }
    /* A rejected frame uses the same transition as an accepted frame. */
    eye = DG_EYE_LEFT;
    eye = next_eye(eye);                 /* rejected frame */
    if (eye != DG_EYE_RIGHT) bad++;
    eye = next_eye(eye);                 /* next accepted frame */
    if (eye != DG_EYE_LEFT) bad++;
    printf("  %-6s stereo alternation: strict frames, rejection advances, index in eye[2]\n",
           bad ? "FAIL" : "ok");
    return bad;
}

static int test_stereo_projection_mirror(void) {
    DG_PROJ_FOV original, mirrored, roundtrip;
    int bad = 0;
    original.left = -0.82;
    original.right = 0.96;
    original.up = 0.91;
    original.down = -0.74;
    mirrored = mirror_fov_x(original);
    roundtrip = mirror_fov_x(mirrored);
    if (mirrored.left != -original.right ||
        mirrored.right != -original.left ||
        mirrored.up != original.up ||
        mirrored.down != original.down ||
        memcmp(&roundtrip, &original, sizeof(original)) != 0)
        bad++;
    printf("  %-6s stereo projection mirror: horizontal only, exact round trip\n",
           bad ? "FAIL" : "ok");
    return bad;
}

/* The software turn: the stick rotates the tracking space by advancing
   the recentre reference, pivoting about the LIVE head. Three properties
   carry the whole design: the reference yaw moves by exactly the delta
   (so the mapped view turns, delta > 0 = rightward); the mapped HEAD
   position is invariant (the player turns where they stand instead of
   sliding through the world); and the arm's frozen ROOM frame rides the
   offset, so a stick turn reaches the arm exactly the way the player
   turning on their feet does - the missing property the 2026-08-23
   session measured as an arm pinned to the old facing after every stick
   turn. */
static int test_stick_turn_is_a_room_turn(void) {
    int bad = 0;
    double q[2] = { 0.0, 1.0 };            /* ref yaw 0 */
    double p[2] = { 0.0, 0.0 };            /* ref at the origin */
    double hx = 1.0, hz = 2.0;             /* head away from the origin */
    double d30 = 30.0 * DEG2RAD;

    dg_xr_turn_step(q, p, hx, hz, d30);

    /* The reference yaw took the whole delta. */
    if (fabs(2.0 * atan2(q[0], q[1]) - d30) > 1e-9) bad++;

    /* The mapped head position is invariant: rotate (head - ref) by the
       NEGATED new reference yaw with the same convention qrot applies
       (x' = c*x + s*z, z' = c*z - s*x) and land back on (1, 2). */
    {
        double vx = hx - p[0], vz = hz - p[1];
        double c = cos(-d30), s = sin(-d30);
        double mx = c * vx + s * vz;
        double mz = c * vz - s * vx;
        if (fabs(mx - hx) > 1e-9 || fabs(mz - hz) > 1e-9) bad++;
    }

    /* A point half a metre in front of the head swings RIGHT of it: the
       mapped forward offset (0, -0.5) gains a +X component. */
    {
        double ox = (hx + 0.0) - hx, oz = (hz - 0.5) - hz;
        double c = cos(-d30), s = sin(-d30);
        double mx = c * ox + s * oz;
        double mz = c * oz - s * ox;
        if (!(mx > 0.24 && mx < 0.26)) bad++;
        if (!(mz < -0.42 && mz > -0.44)) bad++;
    }

    /* The arm's frozen frame rides the published offset: freeze at yaw 0
       with the offset at 0, advance the offset 30 degrees, and the frame
       reference the arm actually uses reads 30. */
    {
        double save_q[4];
        double save_omega = g_arm_frame_omega0;
        int save_have = g_arm_frame_have;
        long save_freezes = g_arm_frame_freezes;
        int save_frame = g_arm_frame;
        DG_XR_RAW_POSE head, ref;
        int j;
        for (j = 0; j < 4; j++) save_q[j] = g_arm_frame_q[j];

        memset(&head, 0, sizeof head);
        head.qw = 1.0;
        g_arm_frame = DG_XR_REL_FRAME_ROOM;
        dg_xr_test_set_turn_offset(0.0);
        arm_frame_freeze(&head);
        dg_xr_test_set_turn_offset(d30);
        arm_frame_reference(&head, &ref);
        if (fabs(2.0 * atan2(ref.qy, ref.qw) - d30) > 1e-5) bad++;
        if (fabs(ref.qx) > 1e-12 || fabs(ref.qz) > 1e-12) bad++;

        /* The whole dg_hook half, stick versus feet: the same controller,
           mapped once under a 30-degree software turn with the player
           unmoved, and once with the player physically rotated 30 degrees
           (head yaw and hand swung about the head) - the two must land
           the SAME wrist target and the SAME player shoulder. The
           shoulder half is the leg that was missing on 2026-08-23: the
           controller rotated with the turn while the shoulder constant
           stayed behind, and the aim ray's tail dragged the muzzle the
           wrong way across the screen. */
        {
            DG_XR_CONFIG cfg;
            DG_XR_RAW_POSE h1, h2, hand1, hand2;
            DG_XR_REL_POSE rel;
            double outS[3], outP[3], shS[3], shP[3];
            double save_sign[3];
            double off0[3] = { 0.2, -0.1, -0.5 };
            double ca = cos(-d30), sa = sin(-d30);
            int i2;

            for (i2 = 0; i2 < 3; i2++) save_sign[i2] = g_arm_pos_sign[i2];
            g_arm_pos_sign[0] = -1.0;
            g_arm_pos_sign[1] = 1.0;
            g_arm_pos_sign[2] = 1.0;
            memset(&cfg, 0, sizeof cfg);
            cfg.yaw_sign = cfg.pitch_sign = cfg.roll_sign = 1.0;
            cfg.x_sign = cfg.y_sign = cfg.z_sign = 1.0;
            cfg.scale = 1000.0;

            memset(&h1, 0, sizeof h1);
            h1.qw = 1.0; h1.px = 0.3; h1.py = 1.6; h1.pz = -0.2;
            hand1 = h1;
            hand1.px += off0[0]; hand1.py += off0[1]; hand1.pz += off0[2];

            /* Physically turned right 30: head yaw -30 (XR positive yaw
               is left), hand swung about the head by the same. */
            h2 = h1;
            h2.qy = sin(-d30 * 0.5); h2.qw = cos(-d30 * 0.5);
            hand2 = h2;
            hand2.px += ca * off0[0] + sa * off0[2];
            hand2.py += off0[1];
            hand2.pz += ca * off0[2] - sa * off0[0];

            g_arm_frame = DG_XR_REL_FRAME_ROOM;
            dg_xr_test_set_turn_offset(0.0);
            arm_frame_freeze(&h1);

            dg_xr_test_set_turn_offset(d30);
            arm_hand_to_view(&h1, &hand1, &cfg, &rel, outS);
            arm_player_shoulder_view(&h1, &cfg, 1.0, shS);

            dg_xr_test_set_turn_offset(0.0);
            arm_hand_to_view(&h2, &hand2, &cfg, &rel, outP);
            arm_player_shoulder_view(&h2, &cfg, 1.0, shP);

            for (i2 = 0; i2 < 3; i2++) {
                if (fabs(outS[i2] - outP[i2]) > 1e-4) bad++;
                if (fabs(shS[i2] - shP[i2]) > 1e-4) bad++;
            }
            for (i2 = 0; i2 < 3; i2++) g_arm_pos_sign[i2] = save_sign[i2];
        }

        dg_xr_test_set_turn_offset(0.0);
        for (j = 0; j < 4; j++) g_arm_frame_q[j] = save_q[j];
        g_arm_frame_omega0 = save_omega;
        g_arm_frame_have = save_have;
        g_arm_frame_freezes = save_freezes;
        g_arm_frame = save_frame;
    }

    printf("  %-6s stick turn is a room turn: the reference takes the "
           "delta, the head maps to where it stood, a forward point "
           "swings right, the arm's frozen frame rides the offset, and "
           "the legacy hook segment - wrist and player shoulder alike - maps a "
           "stick turn and a physical turn identically\n",
           bad ? "FAIL" : "ok");
    return bad;
}

/* The marker's four V5.1 words default to the PROVEN stand (live frame,
   organic comp, stick facing, follow under aim) and roll back by WORD only.
   read_config is the shipping parser, run on real files: a minimal marker,
   the legacy "<yaw> <seconds>" form that never reaches the key loop, the
   four rollback words, and four typos that must read back as the default
   they got - the house rule every other marker key follows. */
static int cfg_words(const char *body, int want_src, int want_frame,
                     int want_comp, int want_aim, int want_basis,
                     int want_yaw, const char *label)
{
    char path[MAX_PATH];
    POSE p;
    double seconds = 0.0;
    int source = 0, track = 0, qmap[4], hand = 0, bad = 0;
    DG_XR_CONFIG xc;
    DG_BRIDGE_CONFIG bc;
    ARM_POS_CFG ap;
    HANDLE h;
    DWORD wrote = 0;

    GetTempPathA(sizeof path, path);
    strncat(path, "dg_hook_cfg_test.on", sizeof path - strlen(path) - 1);
    h = CreateFileA(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS,
                    FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        printf("  FAIL   config words: cannot write %s\n", path);
        return 1;
    }
    WriteFile(h, body, (DWORD)strlen(body), &wrote, NULL);
    CloseHandle(h);

    if (!read_config(path, &p, &seconds, &source, &xc, &bc,
                     &track, qmap, &hand, &ap)) bad++;
    if (bc.follow_src != want_src) bad++;
    if (bc.adjust_frame != want_frame) bad++;
    if (bc.arm_comp != want_comp) bad++;
    if (bc.follow_aim != want_aim) bad++;
    if (bc.arm_hand_basis != want_basis) bad++;
    if (bc.camera_yaw_anchor != want_yaw) bad++;
    DeleteFileA(path);
    if (bad)
        printf("  FAIL   config words (%s): src %d frame %d comp %d aim %d "
               "basis %d, want %d %d %d %d %d\n", label, bc.follow_src,
               bc.adjust_frame, bc.arm_comp, bc.follow_aim, bc.arm_hand_basis,
               want_src, want_frame, want_comp, want_aim, want_basis);
    return bad;
}

#include "dg_twohand.h"
static int test_absolute_aim_transport(void)
{
    DG_BRIDGE_ARM_TARGET t, first;
    DG_XR_CONFIG saved_cfg = g_xrcfg;
    int saved_track = g_arm_track, saved_stereo = g_stereo;
    LONG saved_hand = g_arm_hand, saved_absolute = g_arm_absolute_aim;
    LONG saved_left = g_left_arm, saved_twohand = g_twohand;
    LONG saved_eye = g_current_eye;
    double saved_dt = g_test_dt_override;
    unsigned long saved_stream = g_arm_pose_stream_id;
    int bad = 0, i;
#define AIM_CHECK(x) do { if (!(x)) { bad++; printf("  FAIL absolute aim line %d\n", __LINE__); } } while (0)
    memset(&g_aim_camera, 0, sizeof g_aim_camera);
    memset(&g_xrcfg, 0, sizeof g_xrcfg);
    g_xrcfg.yaw_sign = g_xrcfg.pitch_sign = g_xrcfg.roll_sign = 1;
    g_xrcfg.x_sign = g_xrcfg.y_sign = g_xrcfg.z_sign = 1;
    g_xrcfg.scale = 1000;
    g_arm_track = ARM_TRACK_R_GRIP; g_arm_hand = 1; g_arm_absolute_aim = 1;
    g_xrcfg.positional=1;
    g_stereo = 0; g_test_dt_override = 1.0/60.0;
    g_test_aim_selection = 1;
    g_arm_pose_stream_id++;
    g_aim_camera.valid = 1; g_aim_camera.gate.arm_body = 1;
    g_aim_camera.gate.camera = 2; g_aim_camera.view.qw = 1;
    for (i=0;i<4;i++) g_aim_camera.camera.m[i][i] = 1;
    g_aim_camera.frame.head_raw.qw = 1;
    {
        DG_XR_HAND_POSE *p = &g_aim_camera.frame.right_hand.grip;
        p->active = p->tracked = p->orientation_valid = p->position_valid = 1;
        p->raw_local.qw = 1;
        g_aim_camera.frame.right_hand.aim = *p;
        p = &g_aim_camera.frame.right_hand.aim;
        p->hand = DG_XR_HAND_RIGHT; p->kind = DG_XR_POSE_AIM;
        p->sample_seq = 10; p->xr_time = 100;
        /* Grip deliberately differs. Absolute orientation must still use AIM. */
        g_aim_camera.frame.right_hand.grip.raw_local.qw = 0;
        g_aim_camera.frame.right_hand.grip.raw_local.qx = 1;
    }
    for (i=0;i<15;i++) {
        InterlockedIncrement(&g_present_frame);
        AIM_CHECK(arm_pose_target(&t));
    }
    AIM_CHECK(t.hand_write && t.aim_write && t.aim_weight > .999);
    AIM_CHECK(t.position.enabled && !t.position.valid); /* missing grip sequence */
    AIM_CHECK(fabs(t.aim_world[0]) < 1e-9 && fabs(t.aim_world[3]) < 1e-9);
    AIM_CHECK(fabs(t.aim_world[1] + sqrt(.5)) < 1e-9 &&
              fabs(t.aim_world[2] - sqrt(.5)) < 1e-9);
    /* The left grip is a separate pose stream; neither controller's loss
       may invalidate the other controller's publication. */
    g_left_arm=g_twohand=1;
    g_aim_camera.frame.right_hand.grip.sample_seq=10;
    g_aim_camera.frame.right_hand.grip.xr_time=100;
    g_aim_camera.frame.left_hand.grip=g_aim_camera.frame.right_hand.grip;
    g_aim_camera.frame.left_hand.grip.hand=DG_XR_HAND_LEFT;
    g_aim_camera.frame.left_hand.grip.kind=DG_XR_POSE_GRIP;
    g_aim_camera.frame.left_hand.grip.raw_local.px=-.25;
    for(i=0;i<20;i++) {
        InterlockedIncrement(&g_present_frame);
        AIM_CHECK(arm_pose_target(&t));
    }
    AIM_CHECK(t.left_enabled && t.left_valid && t.hands_coherent);
    {int saved_source=g_source;g_source=SRC_XR;
     InterlockedIncrement(&g_present_frame);AIM_CHECK(arm_pose_target(&t));g_source=saved_source;}
    AIM_CHECK(t.m9_pose.valid && t.m9_pose.sequence==t.left_sample_seq &&
              t.m9_pose.source==t.stream_id && fabs(t.m9_pose.local[0]-.25)<1e-9);
    AIM_CHECK(t.position.valid && t.position.units==1000);
    AIM_CHECK(t.left_position.enabled && t.left_position.valid);
    AIM_CHECK(t.free_right.enabled && t.free_right.valid && t.free_left.enabled && t.free_left.valid);
    AIM_CHECK(t.left_position.grip[0]==-.25);
    AIM_CHECK(t.left_position.units==1000);
    for(i=0;i<16;i++) AIM_CHECK(t.left_position.camera[i]==g_aim_camera.camera.m[i/4][i%4]);
    AIM_CHECK(t.position.grip[0]==g_aim_camera.frame.right_hand.grip.raw_local.px);
    for(i=0;i<16;i++) AIM_CHECK(t.position.camera[i]==g_aim_camera.camera.m[i/4][i%4]);
    AIM_CHECK(fabs(t.hands_distance_m-.25)<1e-9);
    first=t;
    g_aim_camera.frame.left_hand.grip.raw_local.px=-.10;
    InterlockedIncrement(&g_present_frame);AIM_CHECK(arm_pose_target(&t));
    AIM_CHECK(memcmp(first.wrist_view,t.wrist_view,sizeof t.wrist_view)==0);
    AIM_CHECK(memcmp(first.aim_world,t.aim_world,sizeof t.aim_world)==0);
    AIM_CHECK(memcmp(first.left_wrist_view,t.left_wrist_view,sizeof t.left_wrist_view)!=0);
    g_aim_camera.frame.left_hand.grip.tracked=0;
    AIM_CHECK(arm_pose_target(&t) && !t.left_valid && t.aim_write);
    AIM_CHECK(!t.m9_pose.valid);
    g_aim_camera.frame.left_hand.grip.tracked=1;
    g_aim_camera.frame.right_hand.aim.tracked=0;
    InterlockedIncrement(&g_present_frame);
    AIM_CHECK(arm_pose_target(&t) && t.left_valid && !t.aim_write);
    AIM_CHECK(t.left_position.valid); /* right tracking loss is independent */
    g_aim_camera.frame.right_hand.aim.tracked=1;
    g_aim_camera.frame.left_hand.grip.sample_seq=9;
    InterlockedIncrement(&g_present_frame);
    AIM_CHECK(arm_pose_target(&t) && t.left_valid && !t.hands_coherent && t.aim_write);
    AIM_CHECK(!t.m9_pose.valid);
    {
        DG_TWOHAND latch;
        int n;
        dg_twohand_reset(&latch);
        g_stereo=1;g_arm_pose_stream_id++;
        for(n=0;n<30;n++) {
            unsigned long long seq=100+n;
            long long time=1000000000LL+n*16666667LL;
            g_current_eye=(n&1)?DG_EYE_RIGHT:DG_EYE_LEFT;
            g_aim_camera.frame.left_hand.grip.raw_local.px=(n&1)?-.15:-.10;
            g_aim_camera.frame.left_hand.grip.sample_seq=seq;
            g_aim_camera.frame.right_hand.grip.sample_seq=seq;
            g_aim_camera.frame.right_hand.aim.sample_seq=seq;
            g_aim_camera.frame.left_hand.grip.xr_time=time;
            g_aim_camera.frame.right_hand.grip.xr_time=time;
            g_aim_camera.frame.right_hand.aim.xr_time=time;
            InterlockedIncrement(&g_present_frame);
            AIM_CHECK(arm_pose_target(&t));
            AIM_CHECK(t.left_sample_seq==t.aim_sample_seq &&
                      t.left_sample_time==t.aim_sample_time);
            if(n&1) {
                AIM_CHECK(t.hands_distance_m==first.hands_distance_m);
                AIM_CHECK(memcmp(t.free_left.world,first.free_left.world,sizeof t.free_left.world)==0);
                AIM_CHECK(memcmp(t.free_right.world,first.free_right.world,sizeof t.free_right.world)==0);
            }
            else first=t;
            dg_twohand_step(&latch,t.left_valid,
                t.hands_coherent && t.aim_write &&
                t.left_sample_seq==t.aim_sample_seq &&
                t.left_sample_time==t.aim_sample_time,
                t.hands_distance_m,t.left_sample_seq,t.left_sample_time);
        }
        AIM_CHECK(latch.engaged && latch.blend==1);
        g_aim_camera.frame.left_hand.grip.tracked=0;
        AIM_CHECK(arm_pose_target(&t) && !t.left_valid && !t.hands_coherent);
        AIM_CHECK(dg_twohand_step(&latch,t.left_valid,t.hands_coherent,
                  t.hands_distance_m,t.left_sample_seq,t.left_sample_time)==0);
        g_aim_camera.frame.left_hand.grip.tracked=1;
        g_stereo=0;
        g_arm_pose_stream_id++;
        g_aim_camera.frame.right_hand.aim.sample_seq=10;
        g_aim_camera.frame.right_hand.aim.xr_time=100;
        InterlockedIncrement(&g_present_frame);
        AIM_CHECK(arm_pose_target(&t));
    }
    g_left_arm=g_twohand=0;
    first = t;
    g_arm_pose_stream_id++; /* B changes the calibration, never absolute forward. */
    InterlockedIncrement(&g_present_frame);
    AIM_CHECK(arm_pose_target(&t) && t.aim_write);
    for (i=0;i<4;i++) AIM_CHECK(fabs(t.aim_world[i]-first.aim_world[i])<1e-9);
    g_stereo = 1; g_current_eye = DG_EYE_LEFT;
    g_arm_pose_stream_id++;
    InterlockedIncrement(&g_present_frame);
    AIM_CHECK(arm_pose_target(&first) && first.aim_write);
    g_current_eye = DG_EYE_RIGHT;
    g_aim_camera.frame.right_hand.aim.raw_local.qy = sin(.2);
    g_aim_camera.frame.right_hand.aim.raw_local.qw = cos(.2);
    g_aim_camera.frame.right_hand.aim.sample_seq = 11;
    g_aim_camera.frame.right_hand.aim.xr_time = 110;
    InterlockedIncrement(&g_present_frame);
    AIM_CHECK(arm_pose_target(&t) && t.aim_write && t.pair_id == first.pair_id);
    AIM_CHECK(t.aim_sample_seq == first.aim_sample_seq && t.aim_sample_time == first.aim_sample_time);
    for (i=0;i<4;i++) AIM_CHECK(t.aim_world[i] == first.aim_world[i]);
    /* Same frame, eye and object addresses, but a different equipped pistol:
       the USP must start from its fresh AIM instead of the M9 eye latch. */
    g_test_aim_selection = 2;
    AIM_CHECK(arm_pose_target(&t) && t.hand_write && t.aim_write);
    AIM_CHECK(t.aim_weapon_id == 2 && t.stream_id != first.stream_id);
    AIM_CHECK(t.left_stream_id && t.left_stream_id==first.left_stream_id);
    AIM_CHECK(g_absolute_record.input.reset && t.aim_sample_seq == 11);
    AIM_CHECK(dg_ik_quat_angle(t.aim_world, first.aim_world) > .1);
    first = t;
    g_test_aim_selection = 1;
    AIM_CHECK(arm_pose_target(&t) && t.hand_write && t.aim_write);
    AIM_CHECK(t.aim_weapon_id == 1 && t.stream_id != first.stream_id);
    AIM_CHECK(t.left_stream_id==first.left_stream_id);
    AIM_CHECK(g_absolute_record.input.reset);
    for (i=0;i<4;i++) AIM_CHECK(fabs(t.aim_world[i]-first.aim_world[i]) < 1e-9);
    {
        const int weapons[]={3,15,18,5,14,12,7};
        int w;
        dg_bridge_test_stinger_available(1);
        g_twohand=1;
        for(w=0;w<7;w++) {
            first=t;g_test_aim_selection=weapons[w];
            AIM_CHECK(arm_pose_target(&t) && t.hand_write && t.aim_write);
            AIM_CHECK(t.aim_weapon_id==(uint64_t)weapons[w] && t.stream_id!=first.stream_id);
            AIM_CHECK(t.left_stream_id==first.left_stream_id);
            AIM_CHECK(g_absolute_record.input.reset);
            AIM_CHECK(t.twohand_enabled==(weapons[w]==3));
        }
        dg_bridge_test_stinger_available(0);
        g_test_aim_selection=7;
        AIM_CHECK(arm_pose_target(&t) && !t.hand_write && !t.aim_write);
        g_twohand=0;
        g_test_aim_selection=6;
        AIM_CHECK(arm_pose_target(&t) && !t.hand_write && !t.aim_write);
        g_test_aim_selection=1;
        AIM_CHECK(arm_pose_target(&t) && t.aim_write);
    }
    /* First equip after None has no previous absolute weapon selection.
       Exercise the actual producer path: left validity and its calibration
       survive both that first equip and subsequent weapon-stream changes. */
    {
        unsigned long left_epoch;
        g_stereo=0;g_test_aim_selection=0;g_absolute_stream=0;
        g_left_arm=1;
        for(i=0;i<20;i++) {
            InterlockedIncrement(&g_present_frame);
            AIM_CHECK(arm_pose_target(&t));
        }
        AIM_CHECK(t.left_valid && t.left_weight>.999 && t.left_stream_id);
        AIM_CHECK(t.position.enabled && t.position.valid && !t.aim_write);
        /* None publishes the same live camera-height input as armed hands. */
        g_aim_camera.camera.m[3][1]-=40;
        InterlockedIncrement(&g_present_frame);
        AIM_CHECK(arm_pose_target(&t) && t.position.valid);
        AIM_CHECK(t.position.camera[13]==g_aim_camera.camera.m[3][1]);
        g_aim_camera.camera.m[3][1]+=40;
        left_epoch=t.left_stream_id;
        g_test_aim_selection=1;
        InterlockedIncrement(&g_present_frame);
        AIM_CHECK(arm_pose_target(&t) && t.left_valid && t.left_weight>.999);
        AIM_CHECK(t.left_stream_id==left_epoch);
        first=t;g_test_aim_selection=2;
        InterlockedIncrement(&g_present_frame);
        AIM_CHECK(arm_pose_target(&t) && t.left_valid && t.left_weight>.999);
        AIM_CHECK(t.stream_id!=first.stream_id && t.left_stream_id==left_epoch);
    }
    /* FPS/level replacement while looking sideways must reset native latches
       without redefining physical forward. No old actor writes are retained. */
    {
        long freezes = g_arm_frame_freezes;
        unsigned long left_epoch=t.left_stream_id;
        double room[4];
        memcpy(room, g_arm_frame_q, sizeof room);
        g_aim_camera.frame.head_raw.qy = sin(.55);
        g_aim_camera.frame.head_raw.qw = cos(.55);
        g_aim_camera.valid = 0;
        AIM_CHECK(arm_pose_target(&t) && !t.aim_write);
        g_aim_camera.valid = 1;
        g_aim_camera.gate.camera++;
        g_aim_camera.gate.arm_body++;
        AIM_CHECK(arm_pose_target(&t) && t.aim_write);
        AIM_CHECK(g_absolute_record.input.reset);
        AIM_CHECK(t.left_stream_id && t.left_stream_id!=left_epoch);
        left_epoch=t.left_stream_id;
        AIM_CHECK(g_arm_frame_freezes == freezes);
        AIM_CHECK(!memcmp(room, g_arm_frame_q, sizeof room));
        /* Manual calibration retains precedence even during replacement. */
        g_arm_pose_stream_id++;
        g_aim_camera.gate.camera++;
        AIM_CHECK(arm_pose_target(&t) && t.aim_write);
        AIM_CHECK(g_arm_frame_freezes == freezes + 1);
        AIM_CHECK(t.left_stream_id!=left_epoch);
        AIM_CHECK(fabs(g_arm_frame_q[1] - sin(.55)) < 1e-9);
    }
    /* A repeated frame with invalid camera must refuse before pose latching. */
    g_aim_camera.valid = 0;
    AIM_CHECK(arm_pose_target(&t) && !t.hand_write && !t.aim_write);
    AIM_CHECK(!t.position.valid);
    g_aim_camera.valid = 1; g_test_aim_selection = 0;
    AIM_CHECK(arm_pose_target(&t) && !t.hand_write);
    g_test_aim_selection = 1;
    InterlockedIncrement(&g_present_frame);
    g_aim_camera.frame.right_hand.aim.pose_age_ms = 101;
    AIM_CHECK(arm_pose_target(&t) && !t.hand_write);
    g_aim_camera.frame.right_hand.aim.pose_age_ms = 0;
    g_aim_camera.frame.right_hand.aim.sample_seq = 0;
    AIM_CHECK(arm_pose_target(&t) && !t.hand_write);
    g_test_aim_selection = -1;
    g_arm_track = saved_track; g_arm_hand = saved_hand; g_stereo = saved_stereo;
    g_current_eye = saved_eye;
    g_arm_absolute_aim = saved_absolute; g_test_dt_override = saved_dt;
    g_left_arm=saved_left;g_twohand=saved_twohand;
    g_arm_pose_stream_id = saved_stream; g_xrcfg = saved_cfg;
    g_absolute_stream = 0; memset(&g_aim_camera, 0, sizeof g_aim_camera);
    dg_pose_init(&g_arm_pose_state, NULL);
    printf("  %s absolute aim: exact camera AIM, calibration invariant, invalid duplicate/selection/stale release\n",
           bad ? "FAIL" : "ok");
#undef AIM_CHECK
    return bad != 0;
}

static int test_absolute_record_replay(void)
{
    static DG_REC_RING ring;
    DG_AIM_REPLAY_STATE live, replay;
    DG_REC_FRAME record, back;
    DG_XR_FRAME xr;
    FILE *file;
    char path[MAX_PATH];
    int i,bad=0,compat=0,seeded=0;
    long count=0;
    long long frequency=0;
    memset(&live,0,sizeof live);
    memset(&xr,0,sizeof xr); xr.head_raw.qw=1;
    dg_rec_reset(&ring);
    for(i=0;i<300;i++) {
        DG_AIM_REPLAY_IN *input;
        double a=.3*sin(i*.07);
        double body=i<150?0:.6*sin((i-150)*.07);
        double camera=a+body;
        dg_rec_pack(&xr,i*166667,7,(unsigned int)i,(unsigned int)i,0,&record);
        input=&record.absolute.input;
        input->camera[0][0]=input->camera[2][2]=cos(camera);
        input->camera[0][2]=sin(camera); input->camera[2][0]=-sin(camera);
        input->camera[1][1]=1;
        input->view[1]=sin(a*.5); input->view[3]=cos(a*.5);
        /* Independent countersteer: body yaw +b, raw AIM yaw -b keeps
           the same world ray while the head separately moves by a. */
        input->aim[1]=-sin(body*.5); input->aim[3]=cos(body*.5);
        input->frame=i; input->eye=(unsigned int)DG_POSE_EYE_MONO;
        input->dt=1.0/60; input->sample_seq=i+1; input->sample_time=(i+1)*1000;
        input->source_valid=1; input->pose_flags=15;
        if(i>=120)input->pose_flags|=DG_AIM_REPLAY_BLADE;
        input->hand_tag=DG_XR_HAND_RIGHT; input->kind_tag=DG_XR_POSE_AIM;
        input->reset=(i==0 || i==75 || i==120); input->stream=i<75?7:(i<120?8:9);
        input->camera_id=1; input->arm=2; input->subobject=3;
        input->subobjs=4; input->hand=5; input->model=6;
        if(i==100) input->source_valid=0;
        if(i==110) input->age_ms=101;
        record.absolute.before=live;
        absolute_aim_step(&live,input,&record.absolute.observed);
        record.absolute.present=1;
        /* Independent expected fixed world orientation while head/camera
           move together; no production target builder used as the oracle. */
        if(record.absolute.observed.write) {
            const double *q=record.absolute.observed.pose.quat;
            if(i<120) {
                if(fabs(q[0])>1e-8 || fabs(q[3])>1e-8 ||
                   fabs(q[1]+sqrt(.5))>1e-8 || fabs(q[2]-sqrt(.5))>1e-8) bad++;
            } else {

                double v[4]={3.454840660095215,-383.86553955078125,759.2001342773438,0};
                double inverse[4],world[4],n=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
                dg_ik_quat_conj(q,inverse);dg_ik_quat_mul(q,v,world);
                dg_ik_quat_mul(world,inverse,world);
                if(fabs(world[0]/n)>1e-8 || fabs(world[1]/n)>1e-8 || fabs(world[2]/n-1)>1e-8)bad++;
            }
        }
        if((i==100 || i==110) && record.absolute.observed.write) bad++;
        /* Start mid-stream, as an overwritten ring does. The checkpoint must
           reproduce the filter without replaying the unavailable prefix. */
        if(i>=37 && dg_rec_capture(&ring,&record)!=1) bad++;
    }
    GetTempPathA(sizeof path,path);
    strcat_s(path,sizeof path,"mgs2_absolute_replay_fixture.dgrec");
    file=fopen(path,"wb");
    if(!file) return 1;
    if(dg_rec_write(&ring,10000000,file,NULL)!=263) bad++;
    fclose(file); file=fopen(path,"rb");
    if(!file) return 1;
    if(!dg_rec_read_open(file,&count,&frequency,&compat) || compat || count!=263) bad++;
    for(i=0;dg_rec_read_next(file,compat,&back);i++) {
        DG_AIM_REPLAY_OUT out;
        if(!seeded) { replay=back.absolute.before; seeded=1; }
        if(memcmp(&replay,&back.absolute.before,sizeof replay)) bad++;
        absolute_aim_step(&replay,&back.absolute.input,&out);
        if(memcmp(&out,&back.absolute.observed,sizeof out)) bad++;
        if(i==0) {
            DG_AIM_REPLAY_STATE mutated=back.absolute.before;
            DG_AIM_REPLAY_IN changed=back.absolute.input;
            changed.source_valid=0;
            absolute_aim_step(&mutated,&changed,&out);
            if(!memcmp(&out,&back.absolute.observed,sizeof out)) bad++;
        }
    }
    if(i!=263) bad++;
    fclose(file);
    printf("  %s absolute replay: serialized exact raw inputs, mid-stream seed, head/body countersteer cancellation, resets, refusals, sequential bit identity (%d failures)\n",
           bad?"FAIL":"ok",bad);
    return bad!=0;
}

static int test_aim_probe_config(void) {
    static const struct { const char *words; int ok, on; } cases[] = {
        {"source=xr\nvr_aim_probe=on\n",1,1},
        {"source=xr\n",1,0},
        {"source=xr\nvr_aim_probe=ON\n",1,1},
        {"source=synthetic\nvr_geometry_probe=on\n",1,1},
        {"source=xr\nvr_grip_debug=on\n",1,0},
        {"source=xr\nvr_grip_debug=onjunk\n",0,0},
        {"source=xr\nvr_geometry_probe=offjunk\n",0,0},
        {"source=xr\nvr_aim_probe=off\n",1,0},
        {"source=xr\nvr_aim_probe=onjunk\n",0,0},
        {"source=xr\nvr_aim_probe=offjunk\n",0,0},
        {"source=xr\nvr_aim_probe=\n",0,0},
        {"source=xr\nvr_aim_probe=on\nvr_aim_probe=offjunk\n",0,0},
        {"0 3600\n",1,0}
    };
    int i,bad=0,saved=dg_aim_capture_enabled();
    for(i=0;i<(int)(sizeof cases/sizeof cases[0]);i++) {
        char text[160]; POSE p; double seconds=3600;
        int source,track,map[4],hand,ok;
        DG_XR_CONFIG xc; DG_BRIDGE_CONFIG bc; ARM_POS_CFG ap;
        strcpy_s(text,sizeof text,cases[i].words);
        ok=parse_config(text,&p,&seconds,&source,&xc,&bc,&track,map,&hand,&ap);
        if(ok!=cases[i].ok || dg_aim_capture_enabled()!=cases[i].on) bad++;
    }
    dg_aim_capture_configure(saved);
    printf("  %-6s aim observation probe: opt-in, deletion and malformed tokens fail closed\n",
           bad?"FAIL":"ok");
    return bad;
}

static int test_config_defaults_are_the_proven_stand(void) {
    int bad = 0;
    bad += cfg_words("source=xr\nseconds=3600\n", 2, 1, 1, 1, 1, 0, "minimal");
    bad += cfg_words("0 3600\n", 2, 1, 1, 1, 1, 0, "legacy form");
    bad += cfg_words("source=xr\n"
                     "vr_turn_follow_src=head\n"
                     "vr_adjust_frame=legacy\n"
                     "vr_arm_comp=full\n"
                     "vr_turn_follow_aim=hold\n"
                     "vr_arm_hand_basis=root\n"
                     "vr_move=on\n", 0, 0, 0, 0, 0, 0, "rollback words");
    bad += cfg_words("source=xr\n"
                     "vr_turn_follow_src=hand\n"
                     "vr_adjust_frame=live\n"
                     "vr_arm_comp=organic\n"
                     "vr_turn_follow_aim=on\n"
                     "vr_arm_hand_basis=world\n", 1, 1, 1, 1, 1, 0, "explicit words");
    bad += cfg_words("source=xr\n"
                     "vr_turn_follow_src=bogus\n"
                     "vr_adjust_frame=bogus\n"
                     "vr_arm_comp=bogus\n"
                     "vr_turn_follow_aim=bogus\n"
                     "vr_arm_hand_basis=bogus\n", 2, 1, 1, 1, 1, 0, "typos");
    bad += cfg_words("source=xr\nvr_camera_yaw=anchor\n", 2, 1, 1, 1, 1, 1,
                     "camera anchor opt-in");
    bad += cfg_words("source=xr\nvr_camera_yaw=bogus\n", 2, 1, 1, 1, 1, 0,
                     "camera anchor typo");
    bad += cfg_words("source=xr\nvr_camera_yaw=anchorXYZ\n", 2, 1, 1, 1, 1, 0,
                     "camera anchor suffix");
    bad += cfg_words("source=xr\nvr_camera_yaw=anchor\nvr_camera_yaw=off\n",
                     2, 1, 1, 1, 1, 0, "camera anchor rollback");
    printf("  %-6s config defaults: a missing or misspelt V5.1 word reads "
           "back as the proven stand (live frame, organic comp, stick "
           "facing, follow under aim); legacy/full/head/hold roll back\n",
           bad ? "FAIL" : "ok");
    return bad;
}

/* The theater quad's pose. Three properties are load-bearing, and each is
   exactly one of the mutations the design forbids:
     - yaw only: a head-coupled or full-orientation anchor tilts the screen
       with whatever glance the player entered the cutscene in;
     - fixed by value: the function is pure in the head pose it was given, so
       once captured the anchor cannot follow the head - LOCAL-space free
       look belongs to the compositor;
     - fail-closed on degenerate input: an impossible head yields the
       identity heading, never an invented rotation. */
static int test_screen_pose_is_yaw_anchored(void) {
    DG_XR_RAW_POSE head, out, out2;
    double half = 35.0 * DEG2RAD * 0.5;
    int bad = 0;

    /* A head yawed 35 degrees left, pitched down and rolled - only the yaw
       may survive into the anchor. */
    {
        /* q = yaw(35) * pitch(-40): build via two known quats. */
        double qy[4] = { 0.0, sin(half), 0.0, cos(half) };
        double hp = -40.0 * DEG2RAD * 0.5;
        double qp[4] = { sin(hp), 0.0, 0.0, cos(hp) };
        /* Hamilton product qy * qp, written out. */
        head.qx = qy[3]*qp[0] + qy[0]*qp[3] + qy[1]*qp[2] - qy[2]*qp[1];
        head.qy = qy[3]*qp[1] - qy[0]*qp[2] + qy[1]*qp[3] + qy[2]*qp[0];
        head.qz = qy[3]*qp[2] + qy[0]*qp[1] - qy[1]*qp[0] + qy[2]*qp[3];
        head.qw = qy[3]*qp[3] - qy[0]*qp[0] - qy[1]*qp[1] - qy[2]*qp[2];
    }
    head.px = 0.30; head.py = 1.65; head.pz = -0.20;
    dg_xr_screen_pose(&head, 2.0, &out);

    /* Yaw only: no x or z in the anchor's quaternion, and the yaw matches. */
    if (fabs(out.qx) > 1e-9 || fabs(out.qz) > 1e-9) bad++;
    if (fabs(out.qy - sin(half)) > 1e-6 || fabs(out.qw - cos(half)) > 1e-6)
        bad++;
    /* Centre 2 m ahead ALONG THE YAW, at the head's own eye height: forward
       for yaw a is (-sin a, 0, -cos a) from -Z. */
    if (fabs(out.px - (head.px + 2.0 * -sin(35.0 * DEG2RAD))) > 1e-6) bad++;
    if (fabs(out.py - head.py) > 1e-9) bad++;
    if (fabs(out.pz - (head.pz + 2.0 * -cos(35.0 * DEG2RAD))) > 1e-6) bad++;
    /* The distance knob is honoured, direction unchanged. */
    dg_xr_screen_pose(&head, 3.5, &out2);
    if (fabs((out2.px - head.px) / (out.px - head.px) - 1.75) > 1e-6) bad++;

    /* Pure in its input: the SAME yaw under a different glance - here a
       25-degree roll instead of the pitch - gives the SAME anchor. The
       head-coupled mutant fails here, because its output tracks the glance. */
    {
        DG_XR_RAW_POSE head2 = head, outb;
        double hr = 25.0 * DEG2RAD * 0.5;
        double qy2[4] = { 0.0, sin(half), 0.0, cos(half) };
        double qr[4] = { 0.0, 0.0, sin(hr), cos(hr) };
        head2.qx = qy2[3]*qr[0] + qy2[0]*qr[3] + qy2[1]*qr[2] - qy2[2]*qr[1];
        head2.qy = qy2[3]*qr[1] - qy2[0]*qr[2] + qy2[1]*qr[3] + qy2[2]*qr[0];
        head2.qz = qy2[3]*qr[2] + qy2[0]*qr[1] - qy2[1]*qr[0] + qy2[2]*qr[3];
        head2.qw = qy2[3]*qr[3] - qy2[0]*qr[0] - qy2[1]*qr[1] - qy2[2]*qr[2];
        dg_xr_screen_pose(&head2, 2.0, &outb);
        if (fabs(outb.qy - out.qy) > 1e-6 || fabs(outb.qw - out.qw) > 1e-6 ||
            fabs(outb.px - out.px) > 1e-6 || fabs(outb.pz - out.pz) > 1e-6)
            bad++;
    }

    /* Degenerate head (half-turn about a horizontal axis: yaw undefined):
       identity heading, screen straight down LOCAL -Z. Fail closed. */
    head.qx = 1.0; head.qy = 0.0; head.qz = 0.0; head.qw = 0.0;
    head.px = head.py = head.pz = 0.0;
    dg_xr_screen_pose(&head, 2.0, &out);
    if (fabs(out.qx) > 1e-9 || fabs(out.qy) > 1e-9 || fabs(out.qz) > 1e-9 ||
        fabs(out.qw - 1.0) > 1e-9) bad++;
    if (fabs(out.px) > 1e-9 || fabs(out.pz + 2.0) > 1e-9) bad++;

    /* Menu ownership is exclusive, edge-triggered, and consumes held presses
       across a theater transition instead of forwarding them to START. */
    if (dg_xr_menu_route(0,1,1) != 2 ||
        dg_xr_menu_route(0,1,0) != 1 ||
        dg_xr_menu_route(1,1,1) || dg_xr_menu_route(1,1,0) ||
        dg_xr_menu_route(1,0,1) || dg_xr_menu_route(0,0,1)) bad++;
    printf("  %-6s theater screen pose: yaw-only anchor at eye height, dist "
           "along the heading, same anchor whatever the entry glance, "
           "identity on a degenerate head\n", bad ? "FAIL" : "ok");
    return bad;
}

static int test_hand_tags(void) {
    DG_XR_RAW_POSE raw;
    DG_XR_REL_POSE relative;
    DG_XR_HAND_POSE left_grip, right_aim;
    int bad = 0;
    raw.qx = 0.11; raw.qy = -0.22; raw.qz = 0.33; raw.qw = 0.88;
    raw.px = 1.25; raw.py = -2.5; raw.pz = 3.75;
    relative.qx = -0.1; relative.qy = 0.2; relative.qz = -0.3; relative.qw = 0.9;
    relative.px = -4.0; relative.py = 5.0; relative.pz = -6.0;
    dg_xr_hand_pose_sample(&left_grip, DG_XR_HAND_LEFT, DG_XR_POSE_GRIP,
                           &raw, &relative, 1, 1, 1, 1, 1, 41, 123456);
    dg_xr_hand_pose_sample(&right_aim, DG_XR_HAND_RIGHT, DG_XR_POSE_AIM,
                           &raw, &relative, 1, 1, 1, 1, 1, 42, 123457);
    if (left_grip.hand != DG_XR_HAND_LEFT ||
        left_grip.kind != DG_XR_POSE_GRIP ||
        right_aim.hand != DG_XR_HAND_RIGHT ||
        right_aim.kind != DG_XR_POSE_AIM)
        bad++;
    printf("  %-6s named hand/pose fields retain left/right and grip/aim tags\n",
           bad ? "FAIL" : "ok");
    return bad;
}

static int test_raw_local_verbatim(void) {
    DG_XR_RAW_POSE raw, before;
    DG_XR_REL_POSE relative;
    DG_XR_HAND_POSE pose;
    int bad = 0;
    raw.qx = 0.11; raw.qy = -0.22; raw.qz = 0.33; raw.qw = 0.88;
    raw.px = 1.25; raw.py = -2.5; raw.pz = 3.75;
    relative.qx = -0.1; relative.qy = 0.2; relative.qz = -0.3; relative.qw = 0.9;
    relative.px = -4.0; relative.py = 5.0; relative.pz = -6.0;
    before = raw;
    dg_xr_hand_pose_sample(&pose, DG_XR_HAND_LEFT, DG_XR_POSE_GRIP,
                           &raw, &relative, 1, 1, 1, 1, 1, 43, 123458);
    if (memcmp(&raw, &before, sizeof(raw)) != 0 ||
        memcmp(&pose.raw_local, &raw, sizeof(raw)) != 0)
        bad++;
    printf("  %-6s raw LOCAL pose is copied verbatim and input is unchanged\n",
           bad ? "FAIL" : "ok");
    return bad;
}

static int test_reference_pose_roundtrip(void) {
    DG_XR_RAW_POSE reference, pose, roundtrip;
    DG_XR_REL_POSE relative;
    double a = 37.0 * DEG2RAD, b = -22.0 * DEG2RAD;
    double worst = 0.0;
    const double *want, *got;
    int i;
    memset(&reference, 0, sizeof(reference));
    reference.qy = sin(a * 0.5); reference.qw = cos(a * 0.5);
    reference.px = 1.2; reference.py = -0.4; reference.pz = 2.1;
    memset(&pose, 0, sizeof(pose));
    pose.qx = sin(b * 0.5); pose.qw = cos(b * 0.5);
    pose.px = -0.8; pose.py = 1.7; pose.pz = -3.2;
    dg_xr_pose_relative(&reference, &pose, &relative);
    dg_xr_pose_from_relative(&reference, &relative, &roundtrip);
    want = (const double *)&pose;
    got = (const double *)&roundtrip;
    for (i = 0; i < 7; i++) {
        double e = fabs(want[i] - got[i]);
        if (e > worst) worst = e;
    }
    printf("  %-6s reference-relative pose round trip, worst %.3e\n",
           worst <= 1e-12 ? "ok" : "FAIL", worst);
    return worst <= 1e-12 ? 0 : 1;
}

static int test_trigger_hysteresis(void) {
    static const float values[] = {
        0.00f, 0.52f, 0.55f, 0.54f, 0.56f, 0.20f,
        0.11f, 0.10f, 0.12f, 0.09f
    };
    unsigned int pressed = 0;
    int presses = 0, releases = 0;
    size_t i;
    for (i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
        unsigned int next = dg_xr_trigger_hysteresis(
            values[i], 0.10f, 0.55f, pressed);
        if (!pressed && next) presses++;
        if (pressed && !next) releases++;
        pressed = next;
    }
    printf("  %-6s trigger hysteresis noisy crossing: %d press, %d release\n",
           presses == 1 && releases == 1 ? "ok" : "FAIL",
           presses, releases);
    return presses == 1 && releases == 1 ? 0 : 1;
}

static int raw_pose_is_zero(const DG_XR_RAW_POSE *pose) {
    DG_XR_RAW_POSE zero;
    memset(&zero, 0, sizeof(zero));
    return memcmp(pose, &zero, sizeof(zero)) == 0;
}

static int rel_pose_is_zero(const DG_XR_REL_POSE *pose) {
    DG_XR_REL_POSE zero;
    memset(&zero, 0, sizeof(zero));
    return memcmp(pose, &zero, sizeof(zero)) == 0;
}

static int test_tracking_loss_clears_pose(void) {
    DG_XR_RAW_POSE raw;
    DG_XR_REL_POSE relative;
    DG_XR_HAND_POSE pose;
    DG_XR_FRAME frame, readback;
    int bad = 0;
    memset(&raw, 0, sizeof(raw));
    raw.qw = 1.0; raw.px = 2.0;
    memcpy(&relative, &raw, sizeof(relative));
    dg_xr_hand_pose_sample(&pose, DG_XR_HAND_LEFT, DG_XR_POSE_AIM,
                           &raw, &relative, 1, 1, 1, 1, 1, 100, 200);
    if (!pose.position_valid || !pose.orientation_valid || !pose.tracked)
        bad++;
    dg_xr_hand_pose_sample(&pose, DG_XR_HAND_LEFT, DG_XR_POSE_AIM,
                           &raw, &relative, 1, 1, 0, 0, 1, 101, 201);
    if (pose.position_valid || pose.orientation_valid || pose.tracked ||
        !raw_pose_is_zero(&pose.raw_local) ||
        !rel_pose_is_zero(&pose.reference_relative))
        bad++;
    memset(&frame, 0, sizeof(frame));
    dg_xr_hand_pose_sample(&frame.left_hand.aim, DG_XR_HAND_LEFT,
                           DG_XR_POSE_AIM, &raw, &relative,
                           1, 1, 1, 1, 1, 102, 202);
    dg_xr_test_publish(&frame);
    Sleep(110);
    if (!dg_xr_get_stereo(&readback) ||
        readback.left_hand.aim.position_valid ||
        readback.left_hand.aim.orientation_valid ||
        !raw_pose_is_zero(&readback.left_hand.aim.raw_local) ||
        readback.left_hand.aim.pose_age_ms <= 100)
        bad++;
    printf("  %-6s tracking loss and bounded age clear validity and pose payloads\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static volatile LONG g_seqlock_test_go;
static volatile LONG g_seqlock_test_done;

static void fill_test_generation(DG_XR_FRAME *frame, uint64_t generation) {
    memset(frame, 0, sizeof(*frame));
    frame->head.yaw = (double)generation;
    frame->left_hand.hand = DG_XR_HAND_LEFT;
    frame->right_hand.hand = DG_XR_HAND_RIGHT;
    frame->left_hand.grip.sample_seq = generation;
    frame->left_hand.aim.sample_seq = generation;
    frame->right_hand.grip.sample_seq = generation;
    frame->right_hand.aim.sample_seq = generation;
    frame->left_hand.trigger_press_seq = generation;
    frame->left_hand.trigger_release_seq = generation;
    frame->right_hand.trigger_press_seq = generation;
    frame->right_hand.trigger_release_seq = generation;
    frame->left_hand.grip.raw_local.px = (double)generation;
    frame->left_hand.aim.raw_local.px = (double)generation;
    frame->right_hand.grip.raw_local.px = (double)generation;
    frame->right_hand.aim.raw_local.px = (double)generation;
}

static DWORD WINAPI seqlock_test_writer(LPVOID unused) {
    uint64_t i;
    DG_XR_FRAME frame;
    (void)unused;
    while (!InterlockedCompareExchange(&g_seqlock_test_go, 0, 0)) Sleep(0);
    for (i = 1; i <= 50000; i++) {
        fill_test_generation(&frame, i);
        dg_xr_test_publish(&frame);
        if ((i & 255) == 0) Sleep(0);
    }
    InterlockedExchange(&g_seqlock_test_done, 1);
    return 0;
}

static int frame_is_one_generation(const DG_XR_FRAME *frame) {
    uint64_t g = (uint64_t)frame->head.yaw;
    return g != 0 &&
        frame->left_hand.hand == DG_XR_HAND_LEFT &&
        frame->right_hand.hand == DG_XR_HAND_RIGHT &&
        frame->left_hand.grip.sample_seq == g &&
        frame->left_hand.aim.sample_seq == g &&
        frame->right_hand.grip.sample_seq == g &&
        frame->right_hand.aim.sample_seq == g &&
        frame->left_hand.trigger_press_seq == g &&
        frame->left_hand.trigger_release_seq == g &&
        frame->right_hand.trigger_press_seq == g &&
        frame->right_hand.trigger_release_seq == g &&
        frame->left_hand.grip.raw_local.px == (double)g &&
        frame->left_hand.aim.raw_local.px == (double)g &&
        frame->right_hand.grip.raw_local.px == (double)g &&
        frame->right_hand.aim.raw_local.px == (double)g;
}

static int test_hand_seqlock(void) {
    HANDLE thread;
    int bad = 0;
    unsigned int reads = 0;
    InterlockedExchange(&g_seqlock_test_go, 0);
    InterlockedExchange(&g_seqlock_test_done, 0);
    thread = CreateThread(NULL, 0, seqlock_test_writer, NULL, 0, NULL);
    if (!thread) {
        printf("  FAIL   seqlock writer thread could not start\n");
        return 1;
    }
    InterlockedExchange(&g_seqlock_test_go, 1);
    while (!InterlockedCompareExchange(&g_seqlock_test_done, 0, 0) ||
           reads < 10000) {
        DG_XR_FRAME frame;
        if (dg_xr_get_stereo(&frame)) {
            reads++;
            if (!frame_is_one_generation(&frame)) {
                bad++;
                break;
            }
        }
    }
    WaitForSingleObject(thread, INFINITE);
    CloseHandle(thread);
    printf("  %-6s seqlock %u concurrent snapshots, no mixed generation\n",
           bad ? "FAIL" : "ok", reads);
    return bad;
}

/* The axis map is a few lines of parser and one array, and it is also the
   entire conversion between OpenXR's frame and the game's. It was found by
   watching an arm move the wrong way, so it deserves a test that records what
   was found - otherwise the next person here has no way to tell a measured
   default from an assumed one. */
static int test_arm_quat_map(void) {
    int m[4], bad = 0, i;
    static const int measured[4] = { -1, 2, -3, 4 };
    double src[4] = { 0.1, 0.2, 0.3, 0.9 }, q[4];

    for (i = 0; i < 4; i++)
        if (g_arm_qmap[i] != measured[i]) bad++;

    if (!parse_qmap("x,y,z,w", m)) bad++;
    if (m[0] != 1 || m[1] != 2 || m[2] != 3 || m[3] != 4) bad++;
    if (!parse_qmap("-x, y, -z, w", m)) bad++;
    if (m[0] != -1 || m[1] != 2 || m[2] != -3 || m[3] != 4) bad++;
    if (!parse_qmap("+w-z+y-x", m)) bad++;
    if (m[0] != 4 || m[1] != -3 || m[2] != 2 || m[3] != -1) bad++;
    /* Garbage is refused whole: a half-applied permutation is a pose that is
       wrong in a way nobody can read off the screen. */
    if (parse_qmap("x,y,q,w", m)) bad++;
    if (parse_qmap("x,y", m)) bad++;

    /* And the mapping itself, applied the way the seam applies it. */
    for (i = 0; i < 4; i++) {
        int k = measured[i];
        double v = src[(k < 0 ? -k : k) - 1];
        q[i] = (k < 0) ? -v : v;
    }
    if (q[0] != -0.1 || q[1] != 0.2 || q[2] != -0.3 || q[3] != 0.9) bad++;

    printf("  %-6s arm quaternion map: default is the measured (-x,y,-z,w), "
           "signs parse, malformed maps are refused whole\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* Every field of the arm's configuration changes where a controller position
   or rotation lands, so every field has to restart the stream - that is what
   throws away a calibration taken under a mapping that no longer exists.
   Written as a sweep over the struct rather than as one comparison per field,
   because the failure mode being guarded against is somebody adding a tenth
   field and updating only the parser. */
static int test_arm_cfg_change_restarts_stream(void) {
    ARM_POS_CFG a, b;
    int bad = 0, i;

    memset(&a, 0, sizeof a);
    a.sign[0] = -1.0; a.sign[1] = 1.0; a.sign[2] = 1.0;
    a.shoulder_mm[0] = 180.0; a.shoulder_mm[1] = 200.0; a.shoulder_mm[2] = 40.0;
    a.reach_mm = 600.0;
    a.frame = DG_XR_REL_FRAME_YAW;
    a.anchor = 1;
    a.hand_zero = 3;

    b = a;
    if (!arm_pos_cfg_same(&a, &b)) bad++;

    for (i = 0; i < 3; i++) {
        b = a; b.sign[i] = -a.sign[i];
        if (arm_pos_cfg_same(&a, &b)) bad++;
        b = a; b.shoulder_mm[i] = a.shoulder_mm[i] + 1.0;
        if (arm_pos_cfg_same(&a, &b)) bad++;
    }
    b = a; b.reach_mm += 1.0;
    if (arm_pos_cfg_same(&a, &b)) bad++;
    b = a; b.anchor = 0;
    if (arm_pos_cfg_same(&a, &b)) bad++;
    b = a; b.frame = DG_XR_REL_FRAME_HEAD;
    if (arm_pos_cfg_same(&a, &b)) bad++;
    b = a; b.frame = DG_XR_REL_FRAME_ROOM;
    if (arm_pos_cfg_same(&a, &b)) bad++;
    /* The re-zero knob carries no meaning beyond differing, so a change in
       either direction has to count - including back to a value used before. */
    b = a; b.hand_zero = 4;
    if (arm_pos_cfg_same(&a, &b)) bad++;
    b = a; b.hand_zero = 2;
    if (arm_pos_cfg_same(&a, &b)) bad++;
    b=a;b.left_arm=1;
    if(arm_pos_cfg_same(&a,&b)) bad++;
    b=a;b.twohand=1;
    if(arm_pos_cfg_same(&a,&b)) bad++;

    /* The defaults a player who never opens the marker actually runs on.
       Spelled out rather than compared against the same constants the code
       uses, because the point is to make a change to any of them a decision
       somebody had to make twice. */
    {
        ARM_POS_CFG d;
        arm_pos_cfg_defaults(&d);
        if (d.sign[0] != -1.0 || d.sign[1] != 1.0 || d.sign[2] != 1.0) bad++;
        if (d.shoulder_mm[0] != 180.0 || d.shoulder_mm[1] != 200.0 ||
            d.shoulder_mm[2] != 40.0) bad++;
        if (d.reach_mm != 600.0) bad++;
        if (d.anchor != 1) bad++;
        if (d.hand_zero != 0) bad++;
        if (d.left_arm || d.twohand) bad++;
        /* Not head, and not yaw either: both let some part of a head turn
           reach the weapon, which a headset caught as a laser that moved when
           the player did. */
        if (d.frame != DG_XR_REL_FRAME_ROOM) bad++;
    }

    /* The two routes to a re-zero are one number, and each has to move it on
       its own. This is the whole contract the controller button rests on. */
    if (arm_hand_zero_total(0, 0) != arm_hand_zero_total(0, 0)) bad++;
    if (arm_hand_zero_total(2, 5) == arm_hand_zero_total(2, 6)) bad++;
    if (arm_hand_zero_total(2, 5) == arm_hand_zero_total(3, 5)) bad++;
    /* A press seen once must keep counting: the total may not fall back when
       the marker is re-read unchanged, or every poll would restart the
       stream. */
    if (arm_hand_zero_total(2, 5) != arm_hand_zero_total(2, 5)) bad++;

    /* The gate on the button: only a rising edge with the trigger held is a
       re-zero. Every other combination is the resting thumb. */
    if (dg_xr_secondary_counts(0, 1, 1) != 1) bad++;
    if (dg_xr_secondary_counts(0, 1, 0) != 0) bad++;   /* walking bump */
    if (dg_xr_secondary_counts(1, 1, 1) != 0) bad++;   /* held, not an edge */
    if (dg_xr_secondary_counts(0, 0, 1) != 0) bad++;   /* trigger alone */
    if (dg_xr_secondary_counts(1, 0, 1) != 0) bad++;   /* release edge */

    /* The button belongs to whichever arm is being driven. */
    if (arm_tracked_xr_hand(ARM_TRACK_R_GRIP) != DG_XR_HAND_RIGHT) bad++;
    if (arm_tracked_xr_hand(ARM_TRACK_R_AIM) != DG_XR_HAND_RIGHT) bad++;
    if (arm_tracked_xr_hand(ARM_TRACK_L_GRIP) != DG_XR_HAND_LEFT) bad++;
    if (arm_tracked_xr_hand(ARM_TRACK_L_AIM) != DG_XR_HAND_LEFT) bad++;

    /* And the count has to come back from the hand it was pressed on. Both
       hands are exercised because the two enums involved - the public
       DG_XR_HAND_* and the runtime's own 0/1 - differ by one, which is the
       kind of mismatch that reads correctly for one hand and silently zero for
       the other. */
    {
        unsigned long l0 = dg_xr_secondary_presses(DG_XR_HAND_LEFT);
        unsigned long r0 = dg_xr_secondary_presses(DG_XR_HAND_RIGHT);
        dg_xr_test_press_secondary(DG_XR_HAND_RIGHT);
        if (dg_xr_secondary_presses(DG_XR_HAND_RIGHT) != r0 + 1) bad++;
        if (dg_xr_secondary_presses(DG_XR_HAND_LEFT) != l0) bad++;
        dg_xr_test_press_secondary(DG_XR_HAND_LEFT);
        dg_xr_test_press_secondary(DG_XR_HAND_LEFT);
        if (dg_xr_secondary_presses(DG_XR_HAND_LEFT) != l0 + 2) bad++;
        if (dg_xr_secondary_presses(DG_XR_HAND_RIGHT) != r0 + 1) bad++;
        /* A hand value that is neither reads zero rather than indexing off
           the end of the array. */
        if (dg_xr_secondary_presses(0) != 0) bad++;
        if (dg_xr_secondary_presses(99) != 0) bad++;
    }

    /* And applying a config has to leave the globals the mapping reads. */
    {
        double saved_sign[3], saved_shoulder[3], saved_reach = g_arm_reach_mm;
        for (i = 0; i < 3; i++) {
            saved_sign[i] = g_arm_pos_sign[i];
            saved_shoulder[i] = g_arm_shoulder_mm[i];
        }
        int saved_frame2 = g_arm_frame;
        b = a;
        b.sign[2] = -1.0;
        b.shoulder_mm[1] = 215.0;
        b.reach_mm = 640.0;
        b.frame = DG_XR_REL_FRAME_ROOM;
        arm_pos_cfg_apply(&b);
        if (g_arm_frame != DG_XR_REL_FRAME_ROOM) bad++;
        g_arm_frame = saved_frame2;
        for (i = 0; i < 3; i++) {
            if (g_arm_pos_sign[i] != b.sign[i]) bad++;
            if (g_arm_shoulder_mm[i] != b.shoulder_mm[i]) bad++;
        }
        if (g_arm_reach_mm != b.reach_mm) bad++;
        for (i = 0; i < 3; i++) {
            g_arm_pos_sign[i] = saved_sign[i];
            g_arm_shoulder_mm[i] = saved_shoulder[i];
        }
        g_arm_reach_mm = saved_reach;
    }

    printf("  %-6s arm config: every field that moves the hand restarts the "
           "stream, marker and controller button are one number, and applying "
           "a config reaches the globals the map reads\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* Which part of the head's rotation survives into the arm target. Every claim
   here is about a hand that does NOT move in the room while the head does,
   because that is the situation the whole knob exists for: you keep your hand
   still and look around for the arm. */

/* The freeze must ride the stream id and the stream id must ride a recentre.
   Both live in arm_pose_target, which needs a published XR frame - which the
   desk build can supply. Both claims survived a mutation sweep purely because
   nothing exercised this function, which is exactly the situation this test
   exists to end. */
static int test_auto_recenter(void) {
    DG_AUTO_RECENTER s={0};int i,bad=0;
    if(auto_recenter_step(&s,0,1,100,0))bad++;
    if(auto_recenter_step(&s,1,1,100,0))bad++;
    for(i=0;i<100;i++)if(auto_recenter_step(&s,1,1,100,(DWORD)i))bad++;
    if(!auto_recenter_step(&s,1,1,100,100))bad++;
    for(i=0;i<20;i++)if(auto_recenter_step(&s,1,1,100,101+i))bad++;
    if(auto_recenter_step(&s,0,1,100,200) ||
       auto_recenter_step(&s,1,1,100,300))bad++; /* tracking loss is not entry */
    if(auto_recenter_step(&s,2,1,900,400))bad++; /* native third-person leave */
    if(!auto_recenter_step(&s,2,1,900,500))bad++;
    if(auto_recenter_step(&s,1,2,100,600))bad++; /* same actor, new entry */
    if(auto_recenter_step(&s,0,2,100,650))bad++; /* interrupt settling */
    if(auto_recenter_step(&s,1,2,100,700) || auto_recenter_step(&s,1,2,100,799))bad++;
    if(!auto_recenter_step(&s,1,2,100,800))bad++;
    if(auto_recenter_step(&s,1,2,200,900))bad++; /* new level */
    if(!auto_recenter_step(&s,1,2,200,1000))bad++;
    memset(&s,0,sizeof s);
    if(auto_recenter_step(&s,1,1,100,0xfffffff0u))bad++;
    if(!auto_recenter_step(&s,1,1,100,84))bad++; /* DWORD wrap */
    printf("  %s auto calibration: settled FPS/third transitions once, tracking interruption, level and clock wrap\n",bad?"FAIL":"ok");
    return bad;
}

static int test_arm_frame_freeze_rides_the_stream(void) {
    DG_XR_FRAME f;
    DG_BRIDGE_ARM_TARGET t;
    double saved_q[4];
    long saved_freezes = g_arm_frame_freezes;
    int saved_have = g_arm_frame_have;
    int saved_track = g_arm_track;
    unsigned long stream_before;
    long f0;
    int bad = 0, k;

    for (k = 0; k < 4; k++) saved_q[k] = g_arm_frame_q[k];

    memset(&f, 0, sizeof f);
    f.head_raw.qy = sin(0.2); f.head_raw.qw = cos(0.2);
    f.head_raw.px = 0.5; f.head_raw.py = 1.6; f.head_raw.pz = -1.0;
    f.right_hand.hand = DG_XR_HAND_RIGHT;
    f.right_hand.grip.hand = DG_XR_HAND_RIGHT;
    f.right_hand.grip.kind = DG_XR_POSE_GRIP;
    f.right_hand.grip.active = 1;
    f.right_hand.grip.tracked = 1;
    f.right_hand.grip.position_valid = 1;
    f.right_hand.grip.orientation_valid = 1;
    f.right_hand.grip.raw_local.qw = 1.0;
    f.right_hand.grip.raw_local.px = 0.7;
    f.right_hand.grip.raw_local.py = 1.2;
    f.right_hand.grip.raw_local.pz = -1.4;
    dg_xr_test_publish(&f);

    g_arm_track = ARM_TRACK_R_GRIP;
    g_arm_frame_have = 0;

    /* First frame of a stream freezes. The SAME stream never re-freezes, no
       matter how many frames pass - that is what makes it a frame and not a
       smoothed follow. */
    f0 = g_arm_frame_freezes;
    if (!arm_pose_target(&t)) bad++;
    if (g_arm_frame_freezes != f0 + 1) bad++;
    if (!arm_pose_target(&t) || !arm_pose_target(&t)) bad++;
    if (g_arm_frame_freezes != f0 + 1) bad++;

    /* A new stream re-freezes: this is every route into a restart - marker
       edit, B press - seen from here. */
    g_arm_pose_stream_id++;
    if (!arm_pose_target(&t)) bad++;
    if (g_arm_frame_freezes != f0 + 2) bad++;

    /* And a recentre IS a restart: the count moves, the stream follows, the
       freeze follows the stream. */
    stream_before = g_arm_pose_stream_id;
    dg_xr_test_recenter();
    if (!arm_pose_target(&t)) bad++;
    if (g_arm_pose_stream_id == stream_before) bad++;
    if (g_arm_frame_freezes != f0 + 3) bad++;

    g_arm_track = saved_track;
    dg_pose_init(&g_arm_pose_state, NULL);
    for (k = 0; k < 4; k++) g_arm_frame_q[k] = saved_q[k];
    g_arm_frame_have = saved_have;
    g_arm_frame_freezes = saved_freezes;

    printf("  %-6s arm frame freeze: one freeze per stream, a new stream "
           "re-freezes, and a recentre starts a new stream\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* The flight recorder's acceptance bar, stated as a test: a session captured
   through the SAME call the camera seam uses, written to a file, read back
   and fed through arm_pose_target again produces BIT-identical targets. That
   is what makes a .dgrec file evidence rather than an approximation. It is
   also the completeness check for the record layout: any input
   arm_pose_target consumes that the record does not carry - a flag, a seq, a
   dt, the stream identity - shows up here as a mismatch, not as a desk
   replay that quietly disagrees with the headset. */
static int test_flight_recorder_replays_bit_identical(void) {
    enum { N = 24 };
    static DG_BRIDGE_ARM_TARGET live[N];
    static int live_got[N];
    static DG_REC_RING ring;        /* 7.6 MB - static, like the hook's own */
    DG_XR_FRAME f;
    DG_REC_FRAME r;
    FILE *file = NULL;
    long count = 0, freezes_live, freezes_replay;
    long long qpf = 0;
    int saved_track = g_arm_track;
    double saved_dt = g_test_dt_override;
    unsigned long saved_stream = g_arm_pose_stream_id;
    unsigned long saved_pair = g_arm_pose_pair_id;
    long saved_freezes = g_arm_frame_freezes;
    int saved_have = g_arm_frame_have;
    LONG saved_stereo = InterlockedCompareExchange(&g_stereo, 0, 0);
    LONG saved_present = InterlockedCompareExchange(&g_present_frame, 0, 0);
    double saved_q[4];
    int i, k, bad = 0;

    for (k = 0; k < 4; k++) saved_q[k] = g_arm_frame_q[k];

    g_arm_track = ARM_TRACK_R_GRIP;
    g_test_dt_override = 1.0 / 90.0;
    InterlockedExchange(&g_stereo, 0);
    dg_rec_reset(&ring);
    dg_pose_init(&g_arm_pose_state, NULL);
    g_arm_frame_have = 0;
    g_arm_pose_stream_id = 100;
    g_arm_pose_pair_id = 0;
    freezes_live = g_arm_frame_freezes;

    /* Live pass: a head that turns, a hand that moves and rotates, and one
       "B press" (stream bump) half-way - the restart every replay must land
       on the same frame. Every frame is distinct, so the dedupe must record
       all of them. */
    for (i = 0; i < N; i++) {
        double a = 0.05 * (double)i;
        memset(&f, 0, sizeof f);
        f.head_raw.qy = sin(a * 0.5); f.head_raw.qw = cos(a * 0.5);
        f.head_raw.px = 0.02 * i; f.head_raw.py = 1.60; f.head_raw.pz = -1.0;
        f.right_hand.hand = DG_XR_HAND_RIGHT;
        f.right_hand.grip.hand = DG_XR_HAND_RIGHT;
        f.right_hand.grip.kind = DG_XR_POSE_GRIP;
        f.right_hand.grip.active = 1;
        f.right_hand.grip.tracked = 1;
        f.right_hand.grip.position_valid = 1;
        f.right_hand.grip.orientation_valid = 1;
        f.right_hand.grip.raw_local.qx = sin(a * 0.3);
        f.right_hand.grip.raw_local.qw = cos(a * 0.3);
        f.right_hand.grip.raw_local.px = 0.30 + 0.01 * i;
        f.right_hand.grip.raw_local.py = 1.20 - 0.005 * i;
        f.right_hand.grip.raw_local.pz = -0.40 - 0.01 * i;
        f.right_hand.grip.sample_seq = 1000 + (unsigned long long)i;
        if (i == N / 2) g_arm_pose_stream_id++;
        dg_xr_test_publish(&f);
        InterlockedExchange(&g_present_frame, (LONG)i);
        dg_rec_pack(&f, (long long)i * 111111,
                    (unsigned int)g_arm_pose_stream_id,
                    (unsigned int)g_arm_pose_pair_id, (unsigned int)i, 0, &r);
        if (dg_rec_capture(&ring, &r) != 1) bad++;
        memset(&live[i], 0, sizeof live[i]);
        live_got[i] = arm_pose_target(&live[i]);
    }
    freezes_live = g_arm_frame_freezes - freezes_live;

    /* The game-side half is INPUT, not identity: a byte-identical
       controller over a moved animation base must record, because an
       animation moving beneath a still hand is precisely the evidence
       this recorder exists to keep. Captured and then rolled back so the
       replay below still faces exactly N frames. */
    {
        DG_REC_FRAME q = r;
        long appended_was = ring.appended, dup_was = ring.dup;
        if (dg_rec_capture(&ring, &q) != 0) bad++;      /* pure duplicate */
        if (ring.dup != dup_was + 1) bad++;
        q.pair.base_now[0] = 0.5;
        q.pair.flags = DG_REC_PAIR_F_BASE;
        if (dg_rec_capture(&ring, &q) != 1) bad++;      /* base moved */
        if (ring.appended != appended_was + 1) bad++;
        ring.appended = appended_was;
        ring.last = r;
    }

    if (fopen_s(&file, "dg_rec_f3.tmp", "wb") != 0 || !file) bad++;
    else {
        if (dg_rec_write(&ring, 9999937, file, NULL) != N) bad++;
        fclose(file);
    }

    /* Layout compatibility (DGREC3): a v2 file (pair block without
       rest_drift_deg) and a v1 file (no pair block) both come back through
       the same door with the drift at its absence value and every other
       byte intact, and a current file carries a real drift through
       untouched. The old-flavour files are fabricated from this run's own
       records, byte for byte, so the comparison is against ground truth. */
    {
        FILE *cf;
        static DG_REC_FRAME orig[4];
        DG_REC_FRAME got;
        long cnt;
        long long qq;
        int compat, j, nkeep = 0;
        if (fopen_s(&cf, "dg_rec_f3.tmp", "rb") == 0 && cf) {
            if (!dg_rec_read_open(cf, &cnt, &qq, &compat) ||
                compat != DG_REC_COMPAT_NONE || cnt != N) bad++;
            while (nkeep < 4 && dg_rec_read_next(cf, compat, &orig[nkeep]))
                nkeep++;
            fclose(cf);
        } else bad++;
        if (nkeep < 2) bad++;
        if (fopen_s(&cf, "dg_rec_f3v2.tmp", "wb") == 0 && cf) {
            fprintf(cf, "%s\nrecord_bytes=%u\nqpf=%lld\ncount_hint=%d\nend\n",
                    DG_REC_MAGIC_V2, (unsigned int)DG_REC_V2_BYTES,
                    (long long)9999937, nkeep);
            for (j = 0; j < nkeep; j++) {
                fwrite(&orig[j], DG_REC_V1_PREFIX, 1, cf);
                fwrite(&orig[j].pair, DG_REC_V2_PAIR_BYTES, 1, cf);
                fwrite(&orig[j].present_frame, 4 * sizeof(unsigned int), 1,
                       cf);
            }
            fclose(cf);
        } else bad++;
        if (fopen_s(&cf, "dg_rec_f3v2.tmp", "rb") == 0 && cf) {
            if (!dg_rec_read_open(cf, &cnt, &qq, &compat) ||
                compat != DG_REC_COMPAT_V2 || cnt != nkeep) bad++;
            for (j = 0; j < nkeep; j++) {
                memset(&got, 0xAA, sizeof got);
                if (!dg_rec_read_next(cf, compat, &got)) { bad++; break; }
                if (got.pair.rest_drift_deg != -1.0f ||
                    got.pair.frame_word != 0) bad++;
                if (memcmp(&got, &orig[j], DG_REC_V1_PREFIX) ||
                    memcmp(&got.pair, &orig[j].pair,
                           DG_REC_V2_PAIR_BYTES) ||
                    got.present_frame != orig[j].present_frame ||
                    got.pair_id != orig[j].pair_id) bad++;
            }
            fclose(cf);
        } else bad++;
        remove("dg_rec_f3v2.tmp");
        if (fopen_s(&cf, "dg_rec_f3v1.tmp", "wb") == 0 && cf) {
            fprintf(cf, "%s\nrecord_bytes=%u\nqpf=%lld\ncount_hint=%d\nend\n",
                    DG_REC_MAGIC_V1, (unsigned int)DG_REC_V1_BYTES,
                    (long long)9999937, 1);
            fwrite(&orig[0], DG_REC_V1_PREFIX, 1, cf);
            fwrite(&orig[0].present_frame, 4 * sizeof(unsigned int), 1, cf);
            fclose(cf);
        } else bad++;
        if (fopen_s(&cf, "dg_rec_f3v1.tmp", "rb") == 0 && cf) {
            if (!dg_rec_read_open(cf, &cnt, &qq, &compat) ||
                compat != DG_REC_COMPAT_V1 || cnt != 1) bad++;
            memset(&got, 0xAA, sizeof got);
            if (!dg_rec_read_next(cf, compat, &got)) bad++;
            else {
                const unsigned char *pb =
                    (const unsigned char *)&got.pair;
                int allz = 1;
                for (j = 0; j < (int)DG_REC_V2_PAIR_BYTES; j++)
                    if (pb[j]) { allz = 0; break; }
                if (!allz || got.pair.rest_drift_deg != -1.0f) bad++;
                if (memcmp(&got, &orig[0], DG_REC_V1_PREFIX)) bad++;
            }
            fclose(cf);
        } else bad++;
        remove("dg_rec_f3v1.tmp");
        {
            DG_REC_FRAME q = r;
            long appended_was = ring.appended;
            DG_REC_FRAME keep_last = ring.last;
            q.pair.rest_drift_deg = 12.5f;
            if (dg_rec_capture(&ring, &q) != 1) bad++;
            if (fopen_s(&cf, "dg_rec_f3v3.tmp", "wb") == 0 && cf) {
                if (dg_rec_write(&ring, 9999937, cf, NULL) < 1) bad++;
                fclose(cf);
            } else bad++;
            if (fopen_s(&cf, "dg_rec_f3v3.tmp", "rb") == 0 && cf) {
                long c2;
                int cp2;
                if (!dg_rec_read_open(cf, &c2, &qq, &cp2) ||
                    cp2 != DG_REC_COMPAT_NONE) bad++;
                got.pair.rest_drift_deg = 0.0f;
                for (j = 0; j < c2; j++)
                    if (!dg_rec_read_next(cf, cp2, &got)) { bad++; break; }
                if (got.pair.rest_drift_deg != 12.5f) bad++;
                fclose(cf);
            } else bad++;
            remove("dg_rec_f3v3.tmp");
            ring.appended = appended_was;
            ring.last = keep_last;
        }
    }

    /* Replay pass: fresh pipeline state, identical config, every input from
       the FILE. Stream identity comes from the record, which is what makes
       the mid-session restart land on the same frame it did live. */
    dg_pose_init(&g_arm_pose_state, NULL);
    g_arm_frame_have = 0;
    g_arm_pose_pair_id = 0;
    freezes_replay = g_arm_frame_freezes;

    file = NULL;
    if (fopen_s(&file, "dg_rec_f3.tmp", "rb") != 0 || !file) bad++;
    else {
        int v1 = 1;
        if (!dg_rec_read_open(file, &count, &qpf, &v1) || count != N ||
            qpf != 9999937 || v1 != 0) bad++;
        for (i = 0; i < N && dg_rec_read_next(file, 0, &r); i++) {
            DG_XR_FRAME rf;
            DG_BRIDGE_ARM_TARGET t;
            int got;
            dg_rec_unpack(&r, &rf);
            dg_xr_test_publish(&rf);
            g_arm_pose_stream_id = r.stream_id;
            InterlockedExchange(&g_present_frame, (LONG)r.present_frame);
            memset(&t, 0, sizeof t);
            got = arm_pose_target(&t);
            if (got != live_got[i]) bad++;
            else if (memcmp(&t, &live[i], sizeof t) != 0) bad++;
        }
        if (i != N) bad++;
        fclose(file);
    }
    remove("dg_rec_f3.tmp");
    freezes_replay = g_arm_frame_freezes - freezes_replay;
    if (freezes_live != 2 || freezes_replay != 2) bad++;

    g_arm_track = saved_track;
    g_test_dt_override = saved_dt;
    g_arm_pose_stream_id = saved_stream;
    g_arm_pose_pair_id = saved_pair;
    g_arm_frame_freezes = saved_freezes;
    g_arm_frame_have = saved_have;
    for (k = 0; k < 4; k++) g_arm_frame_q[k] = saved_q[k];
    InterlockedExchange(&g_stereo, saved_stereo);
    InterlockedExchange(&g_present_frame, saved_present);
    dg_pose_init(&g_arm_pose_state, NULL);

    printf("  %-6s flight recorder: %d distinct frames captured at the seam's "
           "own call, dumped, reloaded, and replayed through arm_pose_target "
           "bit-identically, restart and all\n", bad ? "FAIL" : "ok", N);
    return bad ? 1 : 0;
}

/* The live snapshot's two halves, held at the desk. The cadence gate: a
   snapshot only when the recorder is on, the ring moved, and a minute
   passed - and only a taken snapshot advances the baseline. The atomic
   writer: success replaces the previous file whole, failure leaves it
   byte-for-byte alone - the property that lets a killed game process cost
   at most the last minute of input rather than the recording. */
static int test_rec_live_snapshot_machinery(void)
{
    static const char *live = "dg_rec_snaptest.dgrec";
    static const char *tmpdir = "dg_rec_snaptest.dgrec.tmp";
    DG_REC_FRAME fr;
    long snap_app = 0;
    int snap_el = 0;
    long count = 0;
    long long qpf = 0;
    FILE *f = NULL;
    int i, bad = 0;

    /* A previous run - typically a mutation sweep's - may have left either
       scratch name behind in either form; the test owns these names, so it
       clears them before measuring anything. */
    remove(live);
    RemoveDirectoryA(live);
    remove(tmpdir);
    RemoveDirectoryA(tmpdir);

    /* The gate's truth table. */
    if (rec_snapshot_due(1, 10, 60, &snap_app, &snap_el) != 1) bad++;
    if (snap_app != 10 || snap_el != 60) bad++;
    if (rec_snapshot_due(1, 20, 61, &snap_app, &snap_el) != 0) bad++;
    if (rec_snapshot_due(0, 30, 200, &snap_app, &snap_el) != 0) bad++;
    if (rec_snapshot_due(1, 20, 200, &snap_app, &snap_el) != 1) bad++;
    if (rec_snapshot_due(1, 20, 400, &snap_app, &snap_el) != 0) bad++;
    if (snap_app != 20 || snap_el != 200) bad++;

    /* Two successive snapshots each replace the file whole; the .tmp never
       survives a success. */
    dg_rec_reset(&g_rec);
    for (i = 0; i < 3; i++) {
        memset(&fr, 0, sizeof fr);
        fr.head[0] = 0.25 * (double)(i + 1);
        fr.pair_id = (unsigned int)i;
        if (dg_rec_capture(&g_rec, &fr) != 1) bad++;
    }
    if (rec_write_atomic(live, NULL) != 3) bad++;
    memset(&fr, 0, sizeof fr);
    fr.head[0] = 9.0;
    if (dg_rec_capture(&g_rec, &fr) != 1) bad++;
    if (rec_write_atomic(live, NULL) != 4) bad++;
    if (fopen_s(&f, tmpdir, "rb") == 0) { fclose(f); bad++; }
    if (fopen_s(&f, live, "rb") != 0 || !f) {
        bad++;
    } else {
        if (!dg_rec_read_open(f, &count, &qpf, NULL) || count != 4) bad++;
        fclose(f);
    }

    /* Failure leaves the previous file alone. The .tmp path is made
       impossible (a directory bearing that name), so the write cannot even
       start - and the file from the previous snapshot must still read back
       exactly as it was. */
    if (!CreateDirectoryA(tmpdir, NULL)) bad++;
    memset(&fr, 0, sizeof fr);
    fr.head[0] = 17.0;
    if (dg_rec_capture(&g_rec, &fr) != 1) bad++;
    if (rec_write_atomic(live, NULL) != -1) bad++;
    if (fopen_s(&f, live, "rb") != 0 || !f) {
        bad++;
    } else {
        if (!dg_rec_read_open(f, &count, &qpf, NULL) || count != 4) bad++;
        fclose(f);
    }
    RemoveDirectoryA(tmpdir);
    remove(live);
    dg_rec_reset(&g_rec);

    printf("  %-6s rec live snapshot: the gate fires once a minute only on a"
           " moving ring, a snapshot atomically replaces the previous file,"
           " and a failed one leaves it byte-for-byte alone\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int test_arm_frame_modes(void) {
    DG_XR_RAW_POSE head, pitched, yawed, rolled, hand;
    DG_XR_REL_POSE rel;
    DG_XR_CONFIG cfg;
    double base[3], moved[3];
    double saved_q[4];
    int saved_have, k;
    long saved_freezes;
    int bad = 0, saved_frame = g_arm_frame;

    for (k = 0; k < 4; k++) saved_q[k] = g_arm_frame_q[k];
    saved_have = g_arm_frame_have;
    saved_freezes = g_arm_frame_freezes;

    memset(&cfg, 0, sizeof cfg);
    cfg.scale = 1000.0;
    cfg.x_sign = cfg.y_sign = cfg.z_sign = 1.0;
    cfg.yaw_sign = cfg.pitch_sign = cfg.roll_sign = 1.0;

    /* One head position, four orientations, and a hand nailed to a fixed point
       in the room the whole time. */
    memset(&head, 0, sizeof head);
    head.qw = 1.0;
    head.px = 0.4; head.py = 1.55; head.pz = -1.2;
    pitched = yawed = rolled = head;
    pitched.qx = sin(0.35); pitched.qw = cos(0.35);   /* 40 deg up   */
    yawed.qy   = sin(0.30); yawed.qw   = cos(0.30);   /* 34 deg left */
    rolled.qz  = sin(0.20); rolled.qw  = cos(0.20);
    memset(&hand, 0, sizeof hand);
    hand.qw = 1.0;
    hand.px = 0.58; hand.py = 1.25; hand.pz = -1.55;

    /* ROOM before anything has been frozen is deliberately not an
       unreferenced frame: it behaves as YAW, so a head yaw still carries the
       arm. Asserted before the first freeze, because "falls back" is a claim
       and not a comment. */
    g_arm_frame = DG_XR_REL_FRAME_ROOM;
    g_arm_frame_have = 0;
    arm_hand_to_view(&head, &hand, &cfg, &rel, base);
    arm_hand_to_view(&yawed, &hand, &cfg, &rel, moved);
    {
        double d = 0.0;
        for (k = 0; k < 3; k++) d += (moved[k] - base[k]) * (moved[k] - base[k]);
        if (sqrt(d) < 100.0) bad++;
    }

    /* ROOM with a frozen heading: no part of the head's rotation reaches the
       arm, yaw included. That is the whole point of the mode. The freeze is
       taken from a PITCHED-AND-YAWED head on purpose: only its heading may
       survive into the frame, or looking down while pressing B would bake the
       downward look into every arm target of the stream. */
    {
        DG_XR_RAW_POSE fh;
        memset(&fh, 0, sizeof fh);
        /* yaw 0.30 composed with pitch 0.50 - built by hand so the only thing
           this head is clean about is nothing at all. */
        fh.qx = cos(0.15) * sin(0.25);
        fh.qy = sin(0.15) * cos(0.25);
        fh.qz = -sin(0.15) * sin(0.25);
        fh.qw = cos(0.15) * cos(0.25);
        fh.px = 9.0; fh.py = 1.2; fh.pz = -3.3;
        arm_frame_freeze(&fh);
        if (!g_arm_frame_have) bad++;
        if (fabs(g_arm_frame_q[0]) > 1e-12 ||
            fabs(g_arm_frame_q[2]) > 1e-12) bad++;
    }
    arm_hand_to_view(&head, &hand, &cfg, &rel, base);
    {
        const DG_XR_RAW_POSE *turned[3];
        int t;
        turned[0] = &pitched; turned[1] = &yawed; turned[2] = &rolled;
        for (t = 0; t < 3; t++) {
            arm_hand_to_view(turned[t], &hand, &cfg, &rel, moved);
            for (k = 0; k < 3; k++)
                if (fabs(moved[k] - base[k]) > 1e-9) bad++;
        }
    }
    /* And the frozen heading is what it uses, not the identity: freezing a
       different forward puts the same hand somewhere else in the character's
       body. This is the property that makes B-while-facing-forward the fix for
       an arm that came up rotated. */
    {
        DG_XR_RAW_POSE fh2;
        double other[3];
        double d = 0.0;
        double q_keep[4];
        int j;
        for (j = 0; j < 4; j++) q_keep[j] = g_arm_frame_q[j];
        memset(&fh2, 0, sizeof fh2);
        fh2.qy = sin(-0.225); fh2.qw = cos(-0.225);   /* yaw -0.45 rad */
        arm_frame_freeze(&fh2);
        arm_hand_to_view(&head, &hand, &cfg, &rel, other);
        for (k = 0; k < 3; k++) d += (other[k] - base[k]) * (other[k] - base[k]);
        if (sqrt(d) < 100.0) bad++;
        for (j = 0; j < 4; j++) g_arm_frame_q[j] = q_keep[j];
    }

    /* And a hand that DOES move still moves the target, or the mode would be
       passing by ignoring its input. */
    {
        DG_XR_RAW_POSE shifted = hand;
        shifted.px += 0.12;
        arm_hand_to_view(&head, &shifted, &cfg, &rel, moved);
        if (fabs(moved[0] - base[0]) < 100.0) bad++;
    }

    /* YAW: pitch and roll are gone, yaw is not. */
    g_arm_frame = DG_XR_REL_FRAME_YAW;
    arm_hand_to_view(&head, &hand, &cfg, &rel, base);
    arm_hand_to_view(&pitched, &hand, &cfg, &rel, moved);
    for (k = 0; k < 3; k++)
        if (fabs(moved[k] - base[k]) > 1e-9) bad++;
    arm_hand_to_view(&rolled, &hand, &cfg, &rel, moved);
    for (k = 0; k < 3; k++)
        if (fabs(moved[k] - base[k]) > 1e-9) bad++;
    arm_hand_to_view(&yawed, &hand, &cfg, &rel, moved);
    {
        double d = 0.0;
        for (k = 0; k < 3; k++) d += (moved[k] - base[k]) * (moved[k] - base[k]);
        if (sqrt(d) < 100.0) bad++;             /* tens of millimetres is not it */
    }

    /* HEAD: every part of it reaches the arm, which is the defect this knob
       was added to name. The hand has not moved in any of these. */
    g_arm_frame = DG_XR_REL_FRAME_HEAD;
    arm_hand_to_view(&head, &hand, &cfg, &rel, base);
    {
        const DG_XR_RAW_POSE *turned[3];
        int t;
        turned[0] = &pitched; turned[1] = &yawed; turned[2] = &rolled;
        for (t = 0; t < 3; t++) {
            double d = 0.0;
            arm_hand_to_view(turned[t], &hand, &cfg, &rel, moved);
            for (k = 0; k < 3; k++)
                d += (moved[k] - base[k]) * (moved[k] - base[k]);
            if (sqrt(d) < 100.0) bad++;
        }
    }

    /* All three keep the head's TRANSLATION out, in every mode: a lean that
       carries head and hand together may never look like an arm movement. */
    {
        int m;
        for (m = 0; m < 3; m++) {
            DG_XR_RAW_POSE lh = head, lhand = hand;
            g_arm_frame = m;
            arm_hand_to_view(&head, &hand, &cfg, &rel, base);
            lh.px += 0.3; lh.py -= 0.15; lh.pz += 0.45;
            lhand.px += 0.3; lhand.py -= 0.15; lhand.pz += 0.45;
            arm_hand_to_view(&lh, &lhand, &cfg, &rel, moved);
            for (k = 0; k < 3; k++)
                if (fabs(moved[k] - base[k]) > 1e-9) bad++;
        }
    }

    /* The yaw extraction itself. A pure yaw survives whole, a pure pitch or
       roll leaves nothing, and an undefined twist is the identity rather than
       an invented rotation. */
    {
        double q[4], y[4];
        q[0] = 0.0; q[1] = sin(0.3); q[2] = 0.0; q[3] = cos(0.3);
        dg_xr_quat_yaw_only(q, y);
        for (k = 0; k < 4; k++) if (fabs(y[k] - q[k]) > 1e-12) bad++;

        q[0] = sin(0.3); q[1] = 0.0; q[2] = 0.0; q[3] = cos(0.3);
        dg_xr_quat_yaw_only(q, y);
        if (fabs(y[0]) > 1e-12 || fabs(y[1]) > 1e-12 ||
            fabs(y[2]) > 1e-12 || fabs(y[3] - 1.0) > 1e-12) bad++;

        q[0] = 0.0; q[1] = 0.0; q[2] = sin(0.3); q[3] = cos(0.3);
        dg_xr_quat_yaw_only(q, y);
        if (fabs(y[3] - 1.0) > 1e-12) bad++;

        /* Half a turn about X: the twist about Y has no value at all. */
        q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
        dg_xr_quat_yaw_only(q, y);
        if (fabs(y[0]) > 1e-12 || fabs(y[1]) > 1e-12 ||
            fabs(y[2]) > 1e-12 || fabs(y[3] - 1.0) > 1e-12) bad++;

        /* Whatever comes out is a unit quaternion about +Y and nothing else. */
        q[0] = 0.11; q[1] = 0.42; q[2] = -0.23; q[3] = 0.87;
        dg_xr_quat_yaw_only(q, y);
        if (fabs(y[0]) > 1e-12 || fabs(y[2]) > 1e-12) bad++;
        if (fabs(y[1]*y[1] + y[3]*y[3] - 1.0) > 1e-12) bad++;
    }

    g_arm_frame = saved_frame;
    for (k = 0; k < 4; k++) g_arm_frame_q[k] = saved_q[k];
    g_arm_frame_have = saved_have;
    g_arm_frame_freezes = saved_freezes;
    printf("  %-6s arm frame: a hand held still while the head turns moves the "
           "target in head mode, only on yaw in yaw mode, and never in room "
           "mode; a lean moves it in none of them; the freeze keeps only the "
           "heading\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int test_arm_position_view_map(void) {
    DG_XR_RAW_POSE head, hand, moved_head, moved_hand, turned_head, turned_hand;
    DG_XR_REL_POSE wanted, recovered, moved_relative;
    DG_XR_CONFIG cfg;
    double out[3], moved[3];
    double saved_sign[3], saved_shoulder[3];
    int saved_frame = g_arm_frame;
    int bad = 0, i;

    /* Everything below is a claim about the HEAD frame - it is where the
       correction, the signs and the shoulder constant were all worked out, and
       "invariant under a common room rotation" is only true there. The default
       is no longer head, so the mode is stated rather than inherited. */
    g_arm_frame = DG_XR_REL_FRAME_HEAD;

    /* The arm's own signs are a global that the marker steers, so this test
       states the ones it is asserting against instead of inheriting today's
       measured default. Restored at the end: a desk test that leaves a global
       changed is a bug in whichever test happens to run next. */
    for (i = 0; i < 3; i++) {
        saved_sign[i] = g_arm_pos_sign[i];
        saved_shoulder[i] = g_arm_shoulder_mm[i];
        g_arm_pos_sign[i] = 1.0;
    }

    memset(&head, 0, sizeof head);
    memset(&hand, 0, sizeof hand);
    memset(&wanted, 0, sizeof wanted);
    memset(&cfg, 0, sizeof cfg);
    head.qy = sin(0.25); head.qw = cos(0.25);
    head.px = 1.0; head.py = 1.6; head.pz = -2.0;
    wanted.qy = sin(0.15); wanted.qw = cos(0.15);
    wanted.px = 0.1; wanted.py = -0.2; wanted.pz = 0.3;
    dg_xr_pose_from_relative(&head, &wanted, &hand);
    cfg.scale = 1000.0;
    cfg.x_sign = -1.0; cfg.y_sign = 1.0; cfg.z_sign = -1.0;
    /* Explicit now that they are read: everything below this point is the
       no-correction case, and it has to stay bit-identical to what it was
       before the correction existed. */
    cfg.yaw_sign = cfg.pitch_sign = cfg.roll_sign = 1.0;
    arm_hand_to_view(&head, &hand, &cfg, &recovered, out);
    if (fabs(out[0] + 100.0) > 1e-9 ||
        fabs(out[1] + 200.0) > 1e-9 ||
        fabs(out[2] - 300.0) > 1e-9) bad++;

    /* A room-space translation common to head and hand must disappear. This
       is the regression the recenter-relative path could not satisfy. */
    moved_head = head;
    moved_hand = hand;
    moved_head.px += 7.0; moved_head.py -= 3.0; moved_head.pz += 11.0;
    moved_hand.px += 7.0; moved_hand.py -= 3.0; moved_hand.pz += 11.0;
    arm_hand_to_view(&moved_head, &moved_hand, &cfg, &moved_relative, moved);
    for (i = 0; i < 3; i++)
        if (fabs(moved[i] - out[i]) > 1e-9) bad++;
    if (fabs(moved_relative.qy - recovered.qy) > 1e-12 ||
        fabs(moved_relative.qw - recovered.qw) > 1e-12) bad++;

    /* A different common head orientation is removed too: position and hand
       orientation arrive from the same live-head-relative record. */
    memset(&turned_head, 0, sizeof turned_head);
    turned_head.qx = sin(-0.35); turned_head.qw = cos(-0.35);
    turned_head.px = -4.0; turned_head.py = 2.1; turned_head.pz = 8.0;
    dg_xr_pose_from_relative(&turned_head, &wanted, &turned_hand);
    arm_hand_to_view(&turned_head, &turned_hand, &cfg, &moved_relative, moved);
    for (i = 0; i < 3; i++)
        if (fabs(moved[i] - out[i]) > 1e-9) bad++;
    if (fabs(moved_relative.qy - wanted.qy) > 1e-12 ||
        fabs(moved_relative.qw - wanted.qw) > 1e-12) bad++;

    /* And the frame correction, which exists because the marker's rotation
       signs are applied to the camera and to nothing else. Without it a
       controller vector is expressed in the head's frame while the arm that
       consumes it hangs off the camera's, and with xr_pitch_sign=-1 those two
       are twice the pitch angle apart - which is a hand behind the head that
       cannot be found by looking for it. */
    {
        DG_XR_CONFIG flipped;
        DG_XR_RAW_POSE ph, phh;
        DG_XR_REL_POSE rel;
        DG_XR_POSE plain_pose;
        double plain_out[3], flip_out[3], want[3];
        double ang = 28.0 * DEG2RAD, p2;
        double rx[3][3];
        int k;

        flipped = cfg;
        flipped.pitch_sign = -1.0;

        /* A level head first: with no pitch there is nothing to disagree
           about, so the correction must come out as the identity. It is
           computed rather than special-cased, so this is a rounding tolerance
           and not bit-equality - a micron on a vector of hundreds of
           millimetres. The bit-identical claim belongs to the no-negative-sign
           path, which returns before computing anything at all, and which the
           checks above this block exercise. */
        arm_hand_to_view(&head, &hand, &cfg, &rel, plain_out);
        arm_hand_to_view(&head, &hand, &flipped, &rel, flip_out);
        for (k = 0; k < 3; k++)
            if (fabs(plain_out[k] - flip_out[k]) > 1e-9) bad++;

        /* Now pitch the head, with the hand carried rigidly with it so the
           head-frame offset is unchanged. The correction must move the result
           by exactly twice the pitch the camera was given, about X - that is
           the whole claim, stated as an angle rather than as a formula. */
        memset(&ph, 0, sizeof ph);
        ph.qx = sin(ang * 0.5); ph.qw = cos(ang * 0.5);
        ph.px = 0.3; ph.py = 1.7; ph.pz = -1.1;
        dg_xr_pose_from_relative(&ph, &wanted, &phh);

        arm_hand_to_view(&ph, &phh, &cfg, &rel, plain_out);
        arm_hand_to_view(&ph, &phh, &flipped, &rel, flip_out);

        /* The pitch the conversion actually reports, so this checks the
           correction and not the extraction. */
        {
            DG_XR_CONFIG probe = cfg;
            probe.positional = 0;
            dg_xr_head_to_pose(ph.qx, ph.qy, ph.qz, ph.qw, 0.0, 0.0, 0.0,
                               &probe, &plain_pose);
            p2 = -2.0 * plain_pose.pitch;
        }
        /* Undo the per-axis signs, rotate, put them back: the correction lives
           in the mirrored frame, ahead of the signs. */
        ypr_to_rows(rx, 0.0, p2, 0.0);
        {
            double v[3];
            v[0] = plain_out[0] / cfg.x_sign;
            v[1] = plain_out[1] / cfg.y_sign;
            v[2] = plain_out[2] / cfg.z_sign;
            for (k = 0; k < 3; k++)
                want[k] = v[0] * rx[0][k] + v[1] * rx[1][k] + v[2] * rx[2][k];
            want[0] *= cfg.x_sign; want[1] *= cfg.y_sign; want[2] *= cfg.z_sign;
        }
        for (k = 0; k < 3; k++)
            if (fabs(flip_out[k] - want[k]) > 1e-9) bad++;

        /* And it is a rotation, so it cannot change how far away the hand is. */
        {
            double a = 0.0, b = 0.0;
            for (k = 0; k < 3; k++) {
                a += plain_out[k] * plain_out[k];
                b += flip_out[k] * flip_out[k];
            }
            if (fabs(sqrt(a) - sqrt(b)) > 1e-9) bad++;
        }
    }

    /* The arm's own axis signs. They exist because the arm-root basis is not
       the camera's view basis, so they have to compose with the camera's
       rather than replace them, and they have to touch nothing else. */
    {
        double signed_out[3];
        int axis, k;

        /* One axis at a time, all three of them. Flipping only the axis this
           file happens to have measured would let a permutation - Y's sign
           applied to Z - pass unnoticed, which is exactly the mutation that
           survived the first sweep of this code. */
        for (axis = 0; axis < 3; axis++) {
            for (k = 0; k < 3; k++) g_arm_pos_sign[k] = 1.0;
            arm_hand_to_view(&head, &hand, &cfg, &recovered, out);
            g_arm_pos_sign[axis] = -1.0;
            arm_hand_to_view(&head, &hand, &cfg, &recovered, signed_out);
            for (k = 0; k < 3; k++) {
                double want_k = (k == axis) ? -out[k] : out[k];
                if (fabs(signed_out[k] - want_k) > 1e-12) bad++;
            }
        }
        for (k = 0; k < 3; k++) g_arm_pos_sign[k] = 1.0;
    }

    /* The player's shoulder is a constant hung off the player's HEADING,
       and the only thing that matters about it is that it travels the
       SAME map as the controller. Asserted by construction rather than
       by repeating the arithmetic: a controller physically held exactly
       where the shoulder is - the head-relative offset placed off the
       head's yaw-only heading, because a shoulder does not tilt when the
       player looks down - must map to exactly the shoulder vector,
       whatever the signs and whatever else the head is doing. */
    {
        DG_XR_RAW_POSE sh_head, sh_head_yaw, sh_hand;
        DG_XR_REL_POSE sh_rel, sh_want;
        double shoulder[3], via_hand[3];
        double hq[4], yq[4];
        int k;

        g_arm_shoulder_mm[0] = 175.0;
        g_arm_shoulder_mm[1] = 210.0;
        g_arm_shoulder_mm[2] = 35.0;
        g_arm_pos_sign[0] = -1.0;

        memset(&sh_head, 0, sizeof sh_head);
        sh_head.qx = sin(-0.21); sh_head.qw = cos(-0.21);
        sh_head.px = 2.5; sh_head.py = 1.4; sh_head.pz = -6.0;

        hq[0] = sh_head.qx; hq[1] = sh_head.qy;
        hq[2] = sh_head.qz; hq[3] = sh_head.qw;
        dg_xr_quat_yaw_only(hq, yq);
        sh_head_yaw = sh_head;
        sh_head_yaw.qx = yq[0]; sh_head_yaw.qy = yq[1];
        sh_head_yaw.qz = yq[2]; sh_head_yaw.qw = yq[3];

        memset(&sh_want, 0, sizeof sh_want);
        sh_want.qw = 1.0;
        sh_want.px =  0.175;
        sh_want.py = -0.210;
        sh_want.pz = -0.035;              /* OpenXR forward is -Z */
        dg_xr_pose_from_relative(&sh_head_yaw, &sh_want, &sh_hand);

        arm_player_shoulder_view(&sh_head, &cfg, 1.0, shoulder);
        arm_hand_to_view(&sh_head, &sh_hand, &cfg, &sh_rel, via_hand);
        for (k = 0; k < 3; k++)
            if (fabs(shoulder[k] - via_hand[k]) > 1e-9) bad++;

        /* A left arm mirrors the outboard component and nothing else, so the
           two shoulders differ only where the map sends OpenXR X. */
        {
            double left[3], mirror[3];
            DG_XR_REL_POSE m_rel;
            DG_XR_RAW_POSE m_hand;
            sh_want.px = -0.175;
            dg_xr_pose_from_relative(&sh_head_yaw, &sh_want, &m_hand);
            arm_player_shoulder_view(&sh_head, &cfg, -1.0, left);
            arm_hand_to_view(&sh_head, &m_hand, &cfg, &m_rel, mirror);
            for (k = 0; k < 3; k++)
                if (fabs(left[k] - mirror[k]) > 1e-9) bad++;
        }
    }

    for (i = 0; i < 3; i++) {
        g_arm_pos_sign[i] = saved_sign[i];
        g_arm_shoulder_mm[i] = saved_shoulder[i];
    }
    g_arm_frame = saved_frame;

    printf("  %-6s controller pose: LOCAL hand relative to the live head is "
           "invariant under common room translation/rotation, mapped once, and "
           "re-expressed in the frame the camera actually uses - untouched "
           "while no rotation sign is negative, exactly twice the pitch when "
           "one is; the arm's own signs compose on top and the player's "
           "shoulder travels the identical map\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

/* ------------------------------------------------------- desk replay ----
   dg_hook_test.exe replay <file.dgrec> <marker> [--raw]

   Plays a recording through the REAL pipeline - the same arm_pose_target,
   dg_pose, dg_arm_map and dg_ik this binary was built from - configured by
   the same marker file the session ran with, and prints every intermediate
   quantity per frame as CSV on stdout. --raw instead exports the raw record
   fields (the CSV face of the binary format).

   What replays exactly: everything arm_pose_target computes - the raw
   controller offset, the frame heading and freezes, the mapped view vector,
   the pose blend with the session's own dt (from the recorded QPC deltas),
   pair/stream identity, the hand quaternion. The F3 category above proves
   this half is bit-identical to live.

   What is standardized instead, because the desk has no game: the skeleton.
   The character's shoulder, native wrist and bone lengths are the values
   MEASURED in session logs (bones 295 / 233.45 mm, shoulder [-139 377 171],
   wrist [-287.9 -71.8 158.7] view mm), and the IK parameters mirror
   dg_bridge's literals (reach 0.99, elbow 15..175, down-and-outboard pole).
   NOT replayable at all: the live skeleton's per-tick pose, the bridge's
   adjust-space conversion and ArmCamRotateShift precompensation, and the
   game's own hierarchy pass - those stay headset-only, and the residual
   heartbeat line is their instrument. */
static void replay_apply_marker(const char *marker, ARM_POS_CFG *ap_out,
                                DG_BRIDGE_CONFIG *bc_out)
{
    POSE p;
    DG_XR_CONFIG xc;
    DG_BRIDGE_CONFIG bc;
    double seconds;
    int source, track, qmap[4], hand;

    memset(bc_out, 0, sizeof *bc_out);
    if (!read_config(marker, &p, &seconds, &source, &xc, &bc,
                     &track, qmap, &hand, ap_out)) {
        fprintf(stderr, "replay: cannot read marker %s - using defaults\n",
                marker);
        arm_pos_cfg_defaults(ap_out);
        return;
    }
    *bc_out = bc;
    arm_pos_cfg_apply(ap_out);
    memcpy(g_arm_qmap, qmap, sizeof qmap);
    InterlockedExchange(&g_arm_hand, hand);
    g_arm_track = track;
    g_xrcfg = xc;
    InterlockedExchange(&g_stereo, xc.stereo != 0);
    if (track == ARM_TRACK_OFF)
        fprintf(stderr, "replay: marker has vr_arm=off - arm_pose_target "
                        "will refuse every frame\n");
}

/* Rotate a view-space vector by a quaternion - the operation the bridge's
   F10 block performs with its own static helper, rebuilt here on the
   solver's product so the replay depends on nothing bridge-internal. */
static void replay_quat_rotate(const double q[4], const double v[3],
                               double out[3])
{
    double vq[4], t1[4], qc[4];
    vq[0] = v[0]; vq[1] = v[1]; vq[2] = v[2]; vq[3] = 0.0;
    dg_ik_quat_conj(q, qc);
    dg_ik_quat_mul(q, vq, t1);
    dg_ik_quat_mul(t1, qc, vq);
    out[0] = vq[0]; out[1] = vq[1]; out[2] = vq[2];
}

/* The bridge's F10 yaw twist, faithfully: the compensation quaternion it
   would apply to the incoming view vectors for this root pair, and the
   drift it would publish. Returns 0 when either root is absent (all-zero
   in the record). */
static int replay_comp_from_roots(const double root_now[4],
                                  const double root0[4],
                                  double comp[4], double *drift_deg)
{
    double inv[4], rel[4], norm, drift;
    if (root0[0] == 0.0 && root0[1] == 0.0 &&
        root0[2] == 0.0 && root0[3] == 0.0) return 0;
    if (root_now[0] == 0.0 && root_now[1] == 0.0 &&
        root_now[2] == 0.0 && root_now[3] == 0.0) return 0;
    dg_ik_quat_conj(root_now, inv);
    dg_ik_quat_mul(inv, root0, rel);
    norm = sqrt(rel[1] * rel[1] + rel[3] * rel[3]);
    comp[0] = 0.0; comp[2] = 0.0;
    if (norm <= 1.0e-6) {
        comp[1] = 0.0; comp[3] = 1.0;
        *drift_deg = 0.0;
        return 1;
    }
    comp[1] = rel[1] / norm;
    comp[3] = rel[3] / norm;
    drift = -2.0 * atan2(comp[1], comp[3]) / DEG2RAD;
    if (drift > 180.0) drift -= 360.0;
    if (drift < -180.0) drift += 360.0;
    *drift_deg = drift;
    return 1;
}

/* A game-space yaw quaternion about +Y. The sign convention is deliberately
   NOT assumed: the caller exports both signs and the analysis keeps the one
   whose synthetic drift matches the live log's. */
static void replay_yaw_quat(double deg, double q[4])
{
    double a = deg * DEG2RAD * 0.5;
    q[0] = 0.0; q[1] = sin(a); q[2] = 0.0; q[3] = cos(a);
}

/* One composition variant: the F10 comp rotation applied to the same map
   inputs the plain replay used, through this variant's own persistent map
   state, exactly as the bridge rotates ctrl_view_c/player_shoulder_c before
   its map. Returns 1 with the map's target when the map produced one. */
static int replay_variant_map(DG_ARM_MAP_STATE *st,
                              const DG_BRIDGE_ARM_TARGET *t,
                              const double comp[4], const ARM_POS_CFG *ap,
                              const double sk_shoulder[3],
                              const double sk_wrist[3],
                              double upper, double fore, double target[3])
{
    DG_ARM_MAP_IN mi;
    DG_ARM_MAP_OUT mo;
    double wrist_c[3], shoulder_c[3];
    int mk;

    replay_quat_rotate(comp, t->wrist_view, wrist_c);
    replay_quat_rotate(comp, t->player_shoulder_view, shoulder_c);
    memset(&mi, 0, sizeof mi);
    for (mk = 0; mk < 3; mk++) {
        mi.controller_view[mk] = wrist_c[mk];
        mi.shoulder_view[mk] = sk_shoulder[mk];
        mi.native_wrist_view[mk] = sk_wrist[mk];
        mi.player_shoulder[mk] = shoulder_c[mk];
    }
    mi.player_reach = t->player_reach_view;
    mi.upper = upper;
    mi.fore = fore;
    mi.anchor_shoulder = ap->anchor;
    if (!dg_arm_map_step(st, &mi, &mo)) return 0;
    for (mk = 0; mk < 3; mk++) target[mk] = mo.target_view[mk];
    return 1;
}

static int replay_absolute(FILE *file, int compat, long count)
{
    DG_AIM_REPLAY_STATE state;
    DG_REC_FRAME record;
    long rows=0, checked=0, written=0, failed=0, missing=0;
    long start=ftell(file);
    if (compat != DG_REC_COMPAT_NONE) {
        fprintf(stderr,"absolute replay: this older recording has no exact camera/AIM input; refused\n");
        return 2;
    }
    memset(&state,0,sizeof state);
    printf("i,frame,eye,stream,source_valid,input_valid,write,weight,flags,sample_seq,sample_time,qx,qy,qz,qw,state_match,output_match\n");
    while (dg_rec_read_next(file,compat,&record)) {
        const DG_REC_AIM_REPLAY *a=&record.absolute;
        DG_AIM_REPLAY_OUT result;
        int state_match, output_match;
        rows++;
        if (!a->present) { missing++; continue; }
        /* Only the first checkpoint seeds replay. All later checkpoints are
           assertions against sequentially recomputed state. */
        if (!checked) state=a->before;
        state_match=memcmp(&state,&a->before,sizeof state)==0;
        absolute_aim_step(&state,&a->input,&result);
        output_match=memcmp(&result,&a->observed,sizeof result)==0;
        if (!state_match || !output_match) {
            if(failed<8) fprintf(stderr,"absolute replay: mismatch at row %ld (state=%d output=%d)\n",
                                 rows-1,state_match,output_match);
            failed++;
        }
        checked++; written+=result.write!=0;
        printf("%ld,%u,%u,%u,%u,%u,%u,%.17g,%u,%llu,%lld,%.17g,%.17g,%.17g,%.17g,%d,%d\n",
               rows-1,a->input.frame,a->input.eye,a->input.stream,a->input.source_valid,
               result.input_valid,result.write,result.pose.weight,result.pose.flags,
               result.sample_seq,result.sample_time,result.pose.quat[0],result.pose.quat[1],
               result.pose.quat[2],result.pose.quat[3],state_match,output_match);
    }
    if (ferror(file) || (ftell(file)-start)%(long)sizeof record) failed++;
    fprintf(stderr,"absolute replay: %ld records/%ld hinted, %ld checked, %ld writes, %ld without absolute input, %ld mismatches\n",
            rows,count,checked,written,missing,failed);
    return (!checked || failed || rows!=count) ? 1 : 0;
}

static int replay_main(int nargs, char **args)
{
    FILE *in = NULL;
    ARM_POS_CFG ap;
    DG_BRIDGE_CONFIG bc;
    DG_ARM_MAP_STATE map_state;
    /* One persistent map state per composition variant: A = comp against
       the RECORDED root, P/N = comp against a synthetic root that follows
       the software turn at each sign. A shared state would let one
       variant's calibration bleed into another's. */
    DG_ARM_MAP_STATE vstate[3];
    double turn_off_rad = 0.0;
    DG_REC_FRAME r;
    long count = 0, i, writes = 0, freezes0;
    long long qpf = 0, qpc0 = 0, qpc_prev = 0;
    unsigned int stream_prev = 0;
    int raw = (nargs >= 3 && _stricmp(args[2], "--raw") == 0);
    int v1 = 0;
    int have_rest = 0;
    double rest[4] = { 0, 0, 0, 1 };
    /* The standardized skeleton, measured not invented - see the header. */
    static const double sk_shoulder[3] = { -139.0, 377.0, 171.0 };
    static const double sk_wrist[3] = { -287.9, -71.8, 158.7 };
    static const double sk_upper = 295.0, sk_fore = 233.45;

    if (nargs < 2) {
        fprintf(stderr, "usage: dg_hook_test.exe replay <file.dgrec> "
                        "<marker> [--raw]\n");
        return 2;
    }
    if (fopen_s(&in, args[0], "rb") != 0 || !in) {
        fprintf(stderr, "replay: cannot open %s\n", args[0]);
        return 2;
    }
    if (!dg_rec_read_open(in, &count, &qpf, &v1)) {
        fprintf(stderr, "replay: %s is not a DGREC file with a record layout "
                        "this build knows (current %u bytes, v4 %u, v3 %u, "
                        "v2 %u, v1 %u) - "
                        "refusing to misread it\n",
                args[0], (unsigned int)sizeof(DG_REC_FRAME),
                (unsigned int)DG_REC_V4_BYTES,
                (unsigned int)DG_REC_V3_BYTES,
                (unsigned int)DG_REC_V2_BYTES,
                (unsigned int)DG_REC_V1_BYTES);
        fclose(in);
        return 2;
    }
    if (nargs >= 3 && _stricmp(args[2], "--absolute") == 0) {
        int rc = replay_absolute(in,v1,count);
        fclose(in);
        return rc;
    }
    if (v1 == DG_REC_COMPAT_V1)
        fprintf(stderr, "replay: v1 recording - no game-side pair state, so "
                        "the hand columns replay against an empty base\n");
    else if (v1 == DG_REC_COMPAT_V2)
        fprintf(stderr, "replay: v2 recording - pair state present, "
                        "rest_drift replays as -1 (not recorded)\n");
    else if (v1 == DG_REC_COMPAT_V4)
        fprintf(stderr, "replay: v4 recording - camera telemetry absent; "
                        "normal CSV camera columns replay as zeros "
                        "(CAMERA clear)\n");

    if (raw) {
        /* The CSV exporter: the binary format, spelled out. Raw CSV remains
           the XR input surface; camera telemetry belongs only to normal CSV. */
        printf("i,qpc,stream,pair,present,eye,"
               "head_qx,head_qy,head_qz,head_qw,head_px,head_py,head_pz");
        {
            const char *hn[2] = { "l", "r" };
            const char *pn[2] = { "grip", "aim" };
            int h, k;
            for (h = 0; h < 2; h++) {
                for (k = 0; k < 2; k++)
                    printf(",%s_%s_qx,%s_%s_qy,%s_%s_qz,%s_%s_qw"
                           ",%s_%s_px,%s_%s_py,%s_%s_pz"
                           ",%s_%s_flags,%s_%s_age,%s_%s_seq,%s_%s_time",
                           hn[h], pn[k], hn[h], pn[k], hn[h], pn[k],
                           hn[h], pn[k], hn[h], pn[k], hn[h], pn[k],
                           hn[h], pn[k], hn[h], pn[k], hn[h], pn[k],
                           hn[h], pn[k], hn[h], pn[k]);
                printf(",%s_trigger,%s_squeeze,%s_stick_x,%s_stick_y"
                       ",%s_buttons,%s_press_seq,%s_release_seq",
                       hn[h], hn[h], hn[h], hn[h], hn[h], hn[h], hn[h]);
            }
        }
        printf("\n");
        for (i = 0; dg_rec_read_next(in, v1, &r); i++) {
            int h, k;
            printf("%ld,%lld,%u,%u,%u,%u", i, r.qpc, r.stream_id, r.pair_id,
                   r.present_frame, r.eye);
            for (k = 0; k < 7; k++) printf(",%.17g", r.head[k]);
            for (h = 0; h < 2; h++) {
                const DG_REC_HAND *hd = &r.hand[h];
                const DG_REC_POSE *ps[2];
                ps[0] = &hd->grip; ps[1] = &hd->aim;
                for (k = 0; k < 2; k++)
                    printf(",%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g"
                           ",%u,%u,%llu,%lld",
                           ps[k]->q[0], ps[k]->q[1], ps[k]->q[2], ps[k]->q[3],
                           ps[k]->p[0], ps[k]->p[1], ps[k]->p[2],
                           ps[k]->flags, ps[k]->pose_age_ms,
                           ps[k]->sample_seq, ps[k]->xr_time);
                printf(",%.9g,%.9g,%.9g,%.9g,%u,%llu,%llu",
                       hd->trigger_value, hd->squeeze_value,
                       hd->thumbstick_x, hd->thumbstick_y, hd->buttons,
                       hd->trigger_press_seq, hd->trigger_release_seq);
            }
            printf("\n");
        }
        fclose(in);
        fprintf(stderr, "replay: exported %ld raw records (header said "
                        "%ld)\n", i, count);
        return 0;
    }

    replay_apply_marker(args[1], &ap, &bc);
    dg_pose_init(&g_arm_pose_state, NULL);
    g_arm_frame_have = 0;
    g_arm_pose_pair_id = 0;
    dg_arm_map_reset(&map_state);
    dg_arm_map_reset(&vstate[0]);
    dg_arm_map_reset(&vstate[1]);
    dg_arm_map_reset(&vstate[2]);
    dg_xr_test_set_turn_offset(0.0);
    freezes0 = g_arm_frame_freezes;

    printf("i,t_s,dt_ms,stream,pair,eye,write,weight,pose_flags,"
           "rel_x,rel_y,rel_z,view_x,view_y,view_z,heading_deg,freezes,"
           "shoulder_x,shoulder_y,shoulder_z,reach,"
           "map_flags,target_x,target_y,target_z,"
           "ik_clamped,elbow_deg,wrist_x,wrist_y,wrist_z,"
           "hand_qx,hand_qy,hand_qz,hand_qw,delta_deg,"
           "pflags,wtype,"
           "base_x,base_y,base_z,base_w,"
           "live_x,live_y,live_z,live_w,"
           "root_x,root_y,root_z,root_w,"
           "root0_x,root0_y,root0_z,root0_w,"
           "rest_x,rest_y,rest_z,rest_w,"
           "ctrlrest_x,ctrlrest_y,ctrlrest_z,ctrlrest_w,"
           "demand_x,demand_y,demand_z,demand_w,"
           "desired_x,desired_y,desired_z,desired_w,"
           "rest_drift,frame_word,"
           "turn_deg,"
           "driftA_deg,tA_x,tA_y,tA_z,"
           "driftP_deg,tP_x,tP_y,tP_z,"
           "driftN_deg,tN_x,tN_y,tN_z,"
           "j3_x,j3_y,j3_z,j4_x,j4_y,j4_z,j5_x,j5_y,j5_z,j6_x,j6_y,j6_z,"
           "ike_x,ike_y,ike_z,ikw_x,ikw_y,ikw_z,ikt_x,ikt_y,ikt_z,"
           "q4w_x,q4w_y,q4w_z,q4w_w,q5w_x,q5w_y,q5w_z,q5w_w,"
           "fore_x,fore_y,fore_z,raw_twist_deg,raw_swing_deg,fit_frac,"
           "head_yaw_deg,stick_yaw_deg,aim_yaw_deg,"
           "cam_r0x,cam_r0y,cam_r0z,cam_r1x,cam_r1y,cam_r1z,"
           "cam_r2x,cam_r2y,cam_r2z,cam_proj_x,cam_proj_y,cam_proj_w\n");

    for (i = 0; dg_rec_read_next(in, v1, &r); i++) {
        DG_XR_FRAME rf;
        DG_BRIDGE_ARM_TARGET t;
        int got;
        double t_s, dt;

        if (i == 0) { qpc0 = r.qpc; qpc_prev = r.qpc; }
        t_s = (qpf > 0) ? (double)(r.qpc - qpc0) / (double)qpf : 0.0;
        dt = (qpf > 0 && r.qpc > qpc_prev)
                 ? (double)(r.qpc - qpc_prev) / (double)qpf : 1.0 / 90.0;
        qpc_prev = r.qpc;
        g_test_dt_override = dt;

        /* A stream change in the record IS the session's restart - B press,
           recentre, marker edit - landing on the frame it landed on live.
           The map recalibrates there for the same reason the bridge does. */
        if (i == 0 || r.stream_id != stream_prev) {
            dg_arm_map_reset(&map_state);
            dg_arm_map_reset(&vstate[0]);
            dg_arm_map_reset(&vstate[1]);
            dg_arm_map_reset(&vstate[2]);
            /* Live, the restart's recentre zeroed the turn offset. */
            turn_off_rad = 0.0;
            have_rest = 0;
        }
        stream_prev = r.stream_id;
        /* The session's software turn, re-derived from the recorded right
           stick through the exact live shaping (move_command) and clamped
           integration (apply_turn), published through the same float path
           the runtime uses, BEFORE the target is computed - the live
           worker samples in that order too. */
        {
            const DG_REC_HAND *rh = &r.hand[1];
            double x = ((rh->grip.flags & 4u) && rh->grip.pose_age_ms <= 100)
                           ? (double)rh->thumbstick_x : 0.0;
            double dz = (double)bc.move_deadzone_mils / 1000.0;
            double gain = (double)bc.turn_gain_mils / 1000.0;
            double m = x < 0.0 ? -x : x;
            double dtt = dt > 0.25 ? 0.25 : dt;
            if (bc.turn_mode && m > dz && dz < 1.0)
                turn_off_rad += (x > 0.0 ? 1.0 : -1.0) * (m - dz) /
                                (1.0 - dz) * DG_TURN_RATE_DEG_S * gain *
                                dtt * DEG2RAD;
            dg_xr_test_set_turn_offset(turn_off_rad);
        }
        g_arm_pose_stream_id = r.stream_id;
        InterlockedExchange(&g_present_frame, (LONG)r.present_frame);
        InterlockedExchange(&g_current_eye, (LONG)r.eye);

        dg_rec_unpack(&r, &rf);
        dg_xr_test_publish(&rf);
        memset(&t, 0, sizeof t);
        got = arm_pose_target(&t);

        printf("%ld,%.4f,%.2f,%u,%lu,%u,%d,%.3f,%u",
               i, t_s, dt * 1000.0, r.stream_id, t.pair_id, r.eye,
               got ? t.write : 0, got ? t.weight : 0.0,
               got ? t.pose_flags : 0u);
        printf(",%.4f,%.4f,%.4f", g_arm_raw_rel[0], g_arm_raw_rel[1],
               g_arm_raw_rel[2]);
        printf(",%.2f,%.2f,%.2f",
               t.wrist_view[0], t.wrist_view[1], t.wrist_view[2]);
        printf(",%.2f,%ld",
               atan2(g_arm_frame_q[1], g_arm_frame_q[3]) * 2.0 / DEG2RAD,
               g_arm_frame_freezes - freezes0);
        printf(",%.1f,%.1f,%.1f,%.1f",
               t.player_shoulder_view[0], t.player_shoulder_view[1],
               t.player_shoulder_view[2], t.player_reach_view);

        if (got && t.write) {
            DG_ARM_MAP_IN mi;
            DG_ARM_MAP_OUT mo;
            int mk, ok;
            writes++;
            memset(&mi, 0, sizeof mi);
            for (mk = 0; mk < 3; mk++) {
                mi.controller_view[mk] = t.wrist_view[mk];
                mi.shoulder_view[mk] = sk_shoulder[mk];
                mi.native_wrist_view[mk] = sk_wrist[mk];
                mi.player_shoulder[mk] = t.player_shoulder_view[mk];
            }
            mi.player_reach = t.player_reach_view;
            mi.upper = sk_upper;
            mi.fore = sk_fore;
            mi.anchor_shoulder = ap.anchor;
            ok = dg_arm_map_step(&map_state, &mi, &mo);
            printf(",%u", mo.flags);
            if (ok) {
                DG_IK_IN ii;
                DG_IK_OUT io;
                double span = sk_upper + sk_fore, out_x, out_z, n;
                printf(",%.1f,%.1f,%.1f", mo.target_view[0],
                       mo.target_view[1], mo.target_view[2]);
                memset(&ii, 0, sizeof ii);
                for (mk = 0; mk < 3; mk++) {
                    ii.shoulder[mk] = sk_shoulder[mk];
                    ii.target[mk] = mo.target_view[mk];
                }
                /* Down and outboard, dg_bridge's own pole shape on the
                   standardized skeleton. */
                out_x = sk_wrist[0] - sk_shoulder[0];
                out_z = sk_wrist[2] - sk_shoulder[2];
                n = sqrt(out_x * out_x + out_z * out_z);
                if (n > 1e-9) { out_x /= n; out_z /= n; }
                ii.pole[0] = sk_shoulder[0] + span * out_x;
                ii.pole[1] = sk_shoulder[1] - span;
                ii.pole[2] = sk_shoulder[2] + span * out_z;
                ii.upper = sk_upper;
                ii.fore = sk_fore;
                ii.max_reach_frac = 0.99;
                ii.min_elbow_deg = 15.0;
                ii.max_elbow_deg = 175.0;
                if (dg_ik_solve(&ii, &io))
                    printf(",%u,%.1f,%.1f,%.1f,%.1f", io.clamped,
                           io.elbow_deg, io.wrist[0], io.wrist[1],
                           io.wrist[2]);
                else
                    printf(",%u,,,,", io.clamped);
            } else {
                printf(",,,,,,,,");
            }
        } else {
            printf(",,,,,,,,,");
        }

        /* The hand chain the way the bridge sees it: the first written pair
           of a stream is the grip rest, and every later frame is an angle
           from it. Convention-free (dg_ik_quat_angle), so it reads the same
           whatever the map signs did. */
        if (got && t.write) {
            if (!have_rest) {
                int mk;
                for (mk = 0; mk < 4; mk++) rest[mk] = t.hand_quat[mk];
                have_rest = 1;
            }
            printf(",%.4f,%.4f,%.4f,%.4f,%.2f",
                   t.hand_quat[0], t.hand_quat[1], t.hand_quat[2],
                   t.hand_quat[3],
                   dg_ik_quat_angle(t.hand_quat, rest) / DEG2RAD);
        } else {
            printf(",,,,,");
        }

        {
            const double *pq[8];
            int pk, pj;
            pq[0] = r.pair.base_now;   pq[1] = r.pair.live_hand;
            pq[2] = r.pair.root_q;     pq[3] = r.pair.root_q0;
            pq[4] = r.pair.rest_view;  pq[5] = r.pair.ctrl_rest;
            pq[6] = r.pair.demand;     pq[7] = r.pair.desired;
            printf(",%u,%u", r.pair.flags, r.pair.wtype);
            for (pj = 0; pj < 8; pj++)
                for (pk = 0; pk < 4; pk++) printf(",%.17g", pq[pj][pk]);
            printf(",%.9g", (double)r.pair.rest_drift_deg);
            printf(",%u", r.pair.frame_word);
        }
        /* The composition variants the bridge would have produced: the F10
           comp against the RECORDED root (A - what the session actually
           composed) and against a synthetic root that follows the software
           turn at each yaw sign (P/N - the body-follow-converged case the
           17:35 session failed in). Targets are the map output the IK
           would chase; the analysis composes each with its root to read
           the world-space aim. */
        {
            double turn_deg = turn_off_rad / DEG2RAD;
            int vi;
            printf(",%.2f", turn_deg);
            for (vi = 0; vi < 3; vi++) {
                double root_used[4], comp[4], drift = 0.0, tgt[3];
                int vk, okc, okm = 0;
                if (vi == 0) {
                    for (vk = 0; vk < 4; vk++)
                        root_used[vk] = r.pair.root_q[vk];
                } else {
                    double yq[4];
                    replay_yaw_quat(vi == 1 ? turn_deg : -turn_deg, yq);
                    dg_ik_quat_mul(yq, r.pair.root_q0, root_used);
                }
                okc = replay_comp_from_roots(root_used, r.pair.root_q0,
                                             comp, &drift);
                if (okc && got && t.write)
                    okm = replay_variant_map(&vstate[vi], &t, comp, &ap,
                                             sk_shoulder, sk_wrist,
                                             sk_upper, sk_fore, tgt);
                if (okc && okm)
                    printf(",%.2f,%.1f,%.1f,%.1f", drift,
                           tgt[0], tgt[1], tgt[2]);
                else if (okc)
                    printf(",%.2f,,,", drift);
                else
                    printf(",,,,");
            }
        }

        {
            const DG_REC_PAIRSTATE *pp = &r.pair;
            int pk, pj;
            for (pj = 0; pj < 4; pj++)
                for (pk = 0; pk < 3; pk++)
                    printf(",%.3f", (double)pp->joint_world[pj][pk]);
            for (pk = 0; pk < 3; pk++) printf(",%.3f", (double)pp->ik_elbow[pk]);
            for (pk = 0; pk < 3; pk++) printf(",%.3f", (double)pp->ik_wrist[pk]);
            for (pk = 0; pk < 3; pk++) printf(",%.3f", (double)pp->ik_target[pk]);
            for (pk = 0; pk < 4; pk++) printf(",%.7g", (double)pp->q4_world[pk]);
            for (pk = 0; pk < 4; pk++) printf(",%.7g", (double)pp->q5_world[pk]);
            for (pk = 0; pk < 3; pk++) printf(",%.6f", (double)pp->fore_axis[pk]);
            printf(",%.3f,%.3f,%.4f,%.3f,%.3f,%.3f",
                   (double)pp->raw_twist_deg, (double)pp->raw_swing_deg,
                   (double)pp->fit_frac, (double)pp->head_yaw_deg,
                   (double)pp->stick_yaw_deg, (double)pp->aim_yaw_deg);
            /* DGREC5 camera telemetry: raw row-vector camera->world basis and
               projection m00,m11,m23 from the preceding successful camera
               pass. Older files leave the tail zeroed and CAMERA clear. */
            for (pj = 0; pj < 3; pj++)
                for (pk = 0; pk < 3; pk++)
                    printf(",%.9g", (double)pp->camera_world[pj][pk]);
            for (pk = 0; pk < 3; pk++)
                printf(",%.9g", (double)pp->camera_proj[pk]);
        }
        printf("\n");
    }
    fclose(in);
    fprintf(stderr,
            "replay: %ld frames (header said %ld), %ld written pairs, "
            "%ld freezes, dt from recorded QPC (qpf %lld)\n",
            i, count, writes, g_arm_frame_freezes - freezes0, qpf);
    return 0;
}

static int test_script_source_parser(void)
{
    static const struct { const char *text; int valid, source; } cases[] = {
        { "seconds=1\n", 1, SRC_SYNTHETIC },
        { "source=synthetic\n", 1, SRC_SYNTHETIC },
        { "source=xr\n", 1, SRC_XR },
        { "source=script\n", 1, SRC_SCRIPT },
        { "source=SCRIPT\n", 1, SRC_SCRIPT },
        { "source=script_suffix\n", 0, 0 },
        { "source=xr_suffix\n", 0, 0 },
        { "source=\n", 0, 0 },
    };
    char dir[MAX_PATH], path[MAX_PATH];
    unsigned int i;
    int bad = 0;
    if (!GetTempPathA(sizeof dir, dir) ||
        !GetTempFileNameA(dir, "dgs", 0, path)) return 1;
    for (i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        FILE *file = NULL;
        POSE pose;
        double seconds = 1;
        int source, track, qmap[4], hand, valid;
        DG_XR_CONFIG cfg;
        DG_BRIDGE_CONFIG bridge;
        ARM_POS_CFG arm;
        if (fopen_s(&file, path, "wb") || !file) { bad++; break; }
        fputs(cases[i].text, file);
        fclose(file);
        valid = read_config(path, &pose, &seconds, &source, &cfg, &bridge,
                            &track, qmap, &hand, &arm);
        if (valid != cases[i].valid || (valid && source != cases[i].source)) bad++;
    }
    DeleteFileA(path);
    printf("  %-6s script selection: absent stays native, exact source tokens "
           "accepted, suffix/empty values refused\n", bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

static int script_snapshot_test_write(const char *path, const void *data, size_t n)
{
    FILE *f = NULL;
    int ok;
    if (fopen_s(&f, path, "wb") || !f) return 0;
    ok = fwrite(data, 1, n, f) == n;
    return fclose(f) == 0 && ok;
}

#include "dg_menu_producer_test.inl"
static int test_script_menu_session(void)
{
    static const struct { const char *text; int valid, enabled; } cases[] = {
        { "source=script\n", 1, 0 },
        { "source=script\nvr_script_menu=on\n", 1, 1 },
        { "source=script\nvr_script_menu=off\n", 1, 0 },
        { "source=script\nvr_script_menu=on\nvr_script_menu=off\n", 0, 0 },
        { "source=script\nvr_script_menu=onward\n", 0, 0 },
        { "source=script\nvr_script_menu=\n", 0, 0 },
        { "source=script\nvr_script_menu on\n", 0, 0 },
        { "source=xr\nvr_script_menu=on\n", 0, 0 }
    };
    int i, bad = 0, saved_cfg = g_script_menu_cfg;
    LONG saved_active = g_script_menu_active, saved_seen = g_script_menu_context_lost;
    LONG saved_traps = g_traps, saved_armed = g_armed, saved_source = g_source;
    LONG saved_silent = g_flat_silent;
    LONG saved_freeze = g_arm_freeze_cfg;
    char saved_policy[MAX_PATH];
    LONG applied = g_applied, fresh = g_fresh, reapplied = g_reapplied;
    LONG menu_steps = g_menu_steps, menu_confirms = g_menu_confirms;
    uint64_t start_seen = g_start_seen, primary_seen = g_script_primary_seen;
    POSE pose, zero_pose;
    DG_BRIDGE_CONFIG bc, expected;
    DG_XR_CONFIG xc;
    ARM_POS_CFG ap;
    int track, hand, source, qmap[4];
    double seconds = 10;
    EXCEPTION_RECORD er;
    CONTEXT context;
    EXCEPTION_POINTERS ep;
#define MENU_CHECK(x) do { if (!(x)) { bad++; printf("  FAIL   script menu at dg_hook.c:%d\n", __LINE__); } } while (0)
    memcpy(saved_policy, g_policy_path, sizeof saved_policy);
    for (i = 0; i < (int)(sizeof cases / sizeof cases[0]); i++) {
        char text[192];
        int ok;
        strcpy_s(text, sizeof text, cases[i].text);
        ok = parse_config(text, &pose, &seconds, &source, &xc, &bc,
                          &track, qmap, &hand, &ap);
        MENU_CHECK(ok == cases[i].valid);
        if (ok) MENU_CHECK(g_script_menu_cfg == cases[i].enabled);
    }
    memset(&bc, 0x5a, sizeof bc);
    memset(&pose, 0x5a, sizeof pose);
    memset(&expected, 0, sizeof expected);
    memset(&zero_pose, 0, sizeof zero_pose);
    expected.menu_mode = bc.menu_mode = DG_MENU_MODE_WRITE;
    expected.script_menu_only = 1;
    track = hand = 9;
    script_menu_sanitize(&pose, &bc, &track, &hand);
    MENU_CHECK(!memcmp(&bc, &expected, sizeof bc));
    MENU_CHECK(!memcmp(&pose, &zero_pose, sizeof pose) && track == ARM_TRACK_OFF && hand == 0);
    g_source = SRC_SCRIPT;
    g_armed = 1;
    g_script_menu_active = 1;
    g_script_menu_context_lost = 0;
    {
        DG_MENU_IN in;
        DG_MENU_OUT out;
        memset(&in, 0, sizeof in); memset(&out, 0, sizeof out);
        in.have_sample = in.input_ok = 1;
        MENU_CHECK(script_menu_preserve_neutral(1, &in, &out));
        MENU_CHECK(!script_menu_preserve_neutral(0, &in, &out));
        in.have_sample = 0;
        MENU_CHECK(!script_menu_preserve_neutral(1, &in, &out));
        in.have_sample = 1; in.input_ok = 0;
        MENU_CHECK(!script_menu_preserve_neutral(1, &in, &out));
        in.input_ok = 1; out.flags = DG_MENU_F_BAD_INPUT;
        MENU_CHECK(!script_menu_preserve_neutral(1, &in, &out));
        out.flags = DG_MENU_F_BAD_CONFIG;
        MENU_CHECK(!script_menu_preserve_neutral(1, &in, &out));
        out.flags = DG_MENU_F_YIELDED;
        MENU_CHECK(!script_menu_preserve_neutral(1, &in, &out));
    }
    MENU_CHECK(!dg_bridge_menu_context_ready()); /* desk bridge has no live context */
    MENU_CHECK(!script_menu_blocked());
    script_primary_command(); /* disabled even before the camera latch */
    MENU_CHECK(g_script_primary_seen == primary_seen);
    memset(&er, 0, sizeof er);
    memset(&context, 0, sizeof context);
    er.ExceptionCode = EXCEPTION_SINGLE_STEP;
    context.Dr6 = 1;
    ep.ExceptionRecord = &er;
    ep.ContextRecord = &context;
    MENU_CHECK(veh(&ep) == EXCEPTION_CONTINUE_EXECUTION);
    MENU_CHECK(context.Dr6 == 0 && (context.EFlags & 0x10000));
    MENU_CHECK(script_menu_blocked() && g_traps == saved_traps + 1);
    MENU_CHECK(g_applied == applied && g_fresh == fresh && g_reapplied == reapplied);
    g_flat_silent = DG_FLAT_SILENT_FRAMES * 4;
    start_command();
    script_primary_command();
    menu_seam(1);
    MENU_CHECK(script_menu_blocked()); /* silence cannot reopen the session */
    MENU_CHECK(g_start_seen == start_seen && g_script_primary_seen == primary_seen);
    MENU_CHECK(g_menu_steps == menu_steps && g_menu_confirms == menu_confirms);
    g_script_menu_cfg = saved_cfg;
    g_script_menu_active = saved_active;
    g_script_menu_context_lost = saved_seen;
    g_traps = saved_traps; g_armed = saved_armed; g_source = saved_source;
    g_flat_silent = saved_silent;
    g_arm_freeze_cfg = saved_freeze;
    memcpy(g_policy_path, saved_policy, sizeof saved_policy);
    g_applied = applied; g_fresh = fresh; g_reapplied = reapplied;
    g_menu_steps = menu_steps; g_menu_confirms = menu_confirms;
    g_start_seen = start_seen; g_script_primary_seen = primary_seen;
    printf("  %-6s script menu: exact opt-in, full sanitizer, detector-only VEH and permanent latch\n", bad ? "FAIL" : "ok");
#undef MENU_CHECK
    return bad ? 1 : 0;
}

static int test_script_snapshot_dispatch(void)
{
    static const char first[] = "source=script\nvr_script_start=1\nvr_fire=off\n";
    static const char edited[] = "source=script\nvr_script_start=1\nvr_fire=on\n";
    static const char next[] = "source=script\nvr_script_start=2\nvr_fire=on\n";
    static const char other[] = "source=xr\nvr_script_start=1\n";
    char dir[MAX_PATH], path[MAX_PATH], oversized[1100];
    SCRIPT_MARKER_SNAPSHOT a, b;
    DG_SCRIPT_GATE saved_gate = g_script_gate;
    int saved_boot = g_script_boot, saved_token = g_script_token_active;
    LONG saved_source = g_source, saved_armed = g_armed;
    int bad = 0;
    HANDLE writer;
#define SNAP_CHECK(x) do { if (!(x)) { bad++; printf("  FAIL   script snapshot at dg_hook.c:%d\n", __LINE__); } } while (0)
    if (!GetTempPathA(sizeof dir, dir) ||
        !GetTempFileNameA(dir, "dgg", 0, path)) return 1;
    SNAP_CHECK(script_snapshot_test_write(path, first, sizeof first - 1));
    SNAP_CHECK(read_script_snapshot(path, &a));
    SNAP_CHECK(read_script_snapshot(path, &b) && script_snapshot_equal(&a, &b));
    dg_script_gate_init(&g_script_gate, 0);
    SNAP_CHECK(dg_script_gate_poll(&g_script_gate, 1, &a.info) == 1);
    SNAP_CHECK(script_request_current(path, &a));
    SNAP_CHECK(script_snapshot_test_write(path, edited, sizeof edited - 1));
    SNAP_CHECK(!script_request_current(path, &a));
    SNAP_CHECK(script_request_current(path, NULL)); /* Live settings remain supported after go. */
    SNAP_CHECK(script_snapshot_test_write(path, next, sizeof next - 1));
    SNAP_CHECK(!script_request_current(path, &a) && g_script_gate.high_water == 2);
    dg_script_gate_finish(&g_script_gate);
    SNAP_CHECK(read_script_snapshot(path, &b));
    SNAP_CHECK(dg_script_gate_poll(&g_script_gate, 1, &b.info) == 0);

    g_script_boot = 1;
    g_script_token_active = 1;
    dg_script_gate_init(&g_script_gate, 0);
    SNAP_CHECK(dg_script_gate_poll(&g_script_gate, 1, &a.info) == 1);
    SNAP_CHECK(script_snapshot_test_write(path, other, sizeof other - 1));
    SNAP_CHECK(!strcmp(run_once(path), "script request changed before config"));
    SNAP_CHECK(g_source == saved_source && g_armed == saved_armed);
    g_script_boot = 0;
    SNAP_CHECK(script_snapshot_test_write(path, first, sizeof first - 1));
    SNAP_CHECK(!strcmp(run_once(path), "source=script requires process restart"));
    SNAP_CHECK(g_source == saved_source && g_armed == saved_armed);

    writer = CreateFileA(path, GENERIC_WRITE, FILE_SHARE_READ, NULL,
                         OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    SNAP_CHECK(writer != INVALID_HANDLE_VALUE);
    if (writer != INVALID_HANDLE_VALUE) {
        SNAP_CHECK(!read_script_snapshot(path, &b));
        CloseHandle(writer);
    }
    memset(oversized, '#', sizeof oversized);
    memcpy(oversized, first, sizeof first - 1);
    SNAP_CHECK(script_snapshot_test_write(path, oversized, sizeof oversized));
    SNAP_CHECK(!read_script_snapshot(path, &b));
    SNAP_CHECK(script_snapshot_test_write(path, first, sizeof first)); /* NUL is not marker text. */
    SNAP_CHECK(!read_script_snapshot(path, &b));
    DeleteFileA(path);
    SNAP_CHECK(!read_script_snapshot(path, &b));
    g_script_gate = saved_gate;
    g_script_boot = saved_boot;
    g_script_token_active = saved_token;
    g_source = saved_source;
    g_armed = saved_armed;
    printf("  %-6s script snapshot/dispatch: stable complete read, config freshness, "
           "source-race and late-source refusal\n", bad ? "FAIL" : "ok");
#undef SNAP_CHECK
    return bad;
}

static int script_route_failure(int line) {
    printf("  FAIL   script route assertion at dg_hook.c:%d\n", line);
    return 1;
}

static int test_script_input_routes(void)
{
    static const char script[] =
        "DGXR_SCRIPT 1 10\n"
        "pose left grip 1 -0.2 -0.3 -0.4 0 0 0 1\n"
        "pose right grip 1 0.2 -0.3 -0.4 0 0 0 1\n"
        "pose right aim 1 0.25 -0.3 -0.45 0 0 0 1\n"
        "input left 0 0 0.25 -0.5 0\n"
        "input right 1 0 0.5 0 0x0c\n"
        "hold 2\n"
        "input right 0 0 0 0 0\n"
        "hold 1\n";
    DG_XR_SCRIPT_PROGRAM program;
    DG_XR_CONFIG cfg, saved_cfg = g_xrcfg;
    DG_XR_FRAME frame, native, readback, released;
    DG_BRIDGE_FIRE fire;
    DG_BRIDGE_MOVE move;
    DG_REC_FRAME record;
    char error[160];
    LONG saved_source = g_source, saved_fire = g_fire_mode;
    LONG saved_move = g_move_mode, saved_turn = g_turn_mode;
    LONG saved_gain = g_turn_gain_mils_live, saved_dz = g_turn_deadzone_mils_live;
    int saved_track = g_arm_track, bad = 0;
    uint64_t press = 0;
    unsigned long secondary;
    double old_real_turn = dg_xr_turn_offset_rad();

    memset(&cfg, 0, sizeof cfg);
    cfg.yaw_sign = cfg.pitch_sign = cfg.roll_sign = 1;
    cfg.x_sign = cfg.y_sign = cfg.z_sign = 1;
    cfg.scale = 1000;
    cfg.trigger_deadzone = 0.1;
    cfg.trigger_fire = 0.55;
    memset(&native, 0, sizeof native);
    native.head_raw.qw = 1;
    native.right_hand.trigger_value = 0.125f;
    native.left_hand.thumbstick_x = -0.75f;
    dg_xr_test_publish(&native);
    if (!dg_xr_script_program_parse_text(&program, script, sizeof script - 1,
                                        error, sizeof error)) return 1;
    if (!dg_xr_script_test_begin(&program, &cfg)) {
        dg_xr_script_program_free(&program);
        return 1;
    }
    g_source = SRC_SCRIPT;
    g_xrcfg = cfg;
    g_arm_track = ARM_TRACK_OFF; /* vanilla arm still uses RIGHT trigger */
    g_fire_mode = g_move_mode = g_turn_mode = 1;
    g_turn_gain_mils_live = 1000;
    g_turn_deadzone_mils_live = 200;
    secondary = input_secondary_presses(DG_XR_HAND_RIGHT);
    if (!dg_xr_script_test_step(&frame) || !(input_get_frame(&readback) & 1)) bad += script_route_failure(__LINE__);
    if (readback.right_hand.grip.raw_local.px != 0.2 ||
        readback.right_hand.aim.raw_local.pz != -0.45) bad += script_route_failure(__LINE__);
    if (!fire_command(&fire) || !fire.valid || fire.value != 1.0 ||
        !fire.press_seq) bad += script_route_failure(__LINE__);
    press = fire.press_seq;
    if (!move_command(&move) || !move.valid || move.x != 0.25 || move.y != -0.5) bad += script_route_failure(__LINE__);
    if (input_secondary_presses(DG_XR_HAND_RIGHT) != secondary + 1) bad += script_route_failure(__LINE__);
    dg_rec_pack(&readback, 1, 1, 1, 1, 0, &record);
    dg_rec_unpack(&record, &released);
    if (released.right_hand.trigger_value != 1.0f ||
        released.right_hand.grip.raw_local.px != 0.2) bad += script_route_failure(__LINE__);
    if (!dg_xr_script_test_step(&frame) || !fire_command(&fire) ||
        fire.press_seq != press || input_turn_offset_rad() <= 0.0) bad += script_route_failure(__LINE__);
    if (dg_xr_turn_offset_rad() != old_real_turn) bad += script_route_failure(__LINE__); /* no XR writer leak */
    if (!dg_xr_script_test_step(&frame) || !fire_command(&fire) ||
        fire.value != 0.0 || fire.release_seq <= press) bad += script_route_failure(__LINE__);
    if (!dg_xr_script_test_abort(&released)) bad += script_route_failure(__LINE__);
    if (input_get_frame(&readback) || fire_command(&fire) || move_command(&move)) bad += script_route_failure(__LINE__);
    /* A fresh real-XR sentinel is deliberately waiting after script EOF. */
    dg_xr_test_publish(&native);
    if (input_get_frame(&readback)) bad += script_route_failure(__LINE__);
    g_source = SRC_XR;
    if (!(input_get_frame(&readback) & 1) ||
        readback.right_hand.trigger_value != 0.125f) bad += script_route_failure(__LINE__);
    dg_xr_script_program_free(&program);
    g_source = saved_source; g_xrcfg = saved_cfg; g_arm_track = saved_track;
    g_fire_mode = saved_fire; g_move_mode = saved_move; g_turn_mode = saved_turn;
    g_turn_gain_mils_live = saved_gain; g_turn_deadzone_mils_live = saved_dz;
    printf("  %-6s script routes: selected snapshot, right fire, left move, "
           "turn provider, secondary mailbox, recorder, release and no XR fallback\n",
           bad ? "FAIL" : "ok");
    return bad ? 1 : 0;
}

#include "dg_controls_producer_test.inl"
#include "dg_m9_input_test.inl"
#include "dg_blade_input_test.inl"
#include "dg_reload_config_test.inl"
#include "dg_position_turn_test.h"
#include "dg_controller_buttons_test.h"
int dg_bridge_test_position_turn(
    void (*make_target)(double software_rad, DG_BRIDGE_ARM_TARGET *out));

#include "dg_scene_blur_test.inl"

int main(int argc, char **argv) {
    static const double cases[][6] = {
        {   0,   0,   0,     0,    0,    0 },
        {  30,   0,   0,     0,    0,    0 },
        {   0,  25,   0,     0,    0,    0 },
        {   0,   0, -15,     0,    0,    0 },
        {  40, -20,  10,     0,    0,    0 },
        {   0,   0,   0,   65,  -32,  120 },
        {  40, -20,  10,   65,  -32,  120 },
        { 179,  89, -179, -900, 1775, -450 },
    };
    int n = (int)(sizeof(cases) / sizeof(cases[0])), i, r, c, bad = 0;
    int f3_bad = 0;

    /* Preserve the last completed category if a native self-test crashes. */
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc == 2 && _stricmp(argv[1], "controls-self-test") == 0)
        return test_controls_producer();
    if (argc == 2 && _stricmp(argv[1], "script-self-test") == 0)
        return test_script_input_routes();

    /* Not a test run: play a flight recording through the pipeline this
       binary was built from. Everything else in main stays the default so
       `test.bat` remains exactly what it was. */
    if (argc >= 2 && _stricmp(argv[1], "replay") == 0)
        return replay_main(argc - 2, argv + 2);

    printf("D * D^-1 = I\n");
    for (i = 0; i < n; i++) {
        POSE p;
        MAT d, di, prod;
        double worst = 0.0;
        memset(&p, 0, sizeof(p));
        p.yaw = cases[i][0] * DEG2RAD; p.pitch = cases[i][1] * DEG2RAD;
        p.roll = cases[i][2] * DEG2RAD;
        p.tx = cases[i][3]; p.ty = cases[i][4]; p.tz = cases[i][5];

        build_delta(&d, &di, &p);
        mat_mul(&prod, &d, &di);
        for (r = 0; r < 4; r++) for (c = 0; c < 4; c++) {
            double want = (r == c) ? 1.0 : 0.0;
            double e = fabs((double)prod.m[r][c] - want);
            if (e > worst) worst = e;
        }
        /* Tolerance is scaled by the translation because the inverse's
           translation row is a product of millimetre-scale terms in float32. */
        {
            double scale = 1.0 + fabs(p.tx) + fabs(p.ty) + fabs(p.tz);
            double tol = 1e-4 * scale;
            int ok = worst <= tol;
            if (!ok) bad++;
            printf("  %-28s worst |D*D^-1 - I| = %.3e  %s\n",
                   ok ? "ok" : "FAIL", worst, ok ? "" : "<-- inverse is wrong");
        }
    }
    printf("  -> inverse: %s, %d/%d cases\n\n", bad ? "FAILED" : "passed", n - bad, n);

    printf("YXZ decomposition (must invert build_delta's composition)\n");
    bad += test_ypr_roundtrip();
    printf("\nOpenXR -> MGS2 axis mapping (leakage must be zero)\n");
    bad += test_axis_purity();
    printf("\nrotation + translation composition\n");
    bad += test_translation_composition();
    printf("\nreference capture gate\n");
    bad += test_capture_gate();
    printf("\nframe-sequential stereo eye\n");
    bad += test_stereo_alternation();
    bad += test_stereo_projection_mirror();
    bad += test_scene_blur();
    bad += test_render_link_config();
    bad += test_radar_config();
    bad += test_stereo_eye_shift();
    bad += test_eye_truth();

    printf("\nF3 OpenXR motion publication\n");
    bad += dg_script_gate_self_test();
    bad += test_script_snapshot_dispatch();
    bad += test_script_menu_session();
    bad += test_menu_gameover_producer();
    f3_bad += test_hand_seqlock();
    f3_bad += test_hand_tags();
    f3_bad += test_raw_local_verbatim();
    f3_bad += test_reference_pose_roundtrip();
    f3_bad += test_trigger_hysteresis();
    f3_bad += test_tracking_loss_clears_pose();
    f3_bad += test_arm_quat_map();
    f3_bad += test_arm_position_view_map();
    f3_bad += test_arm_cfg_change_restarts_stream();
    f3_bad += test_arm_frame_modes();
    f3_bad += test_arm_frame_freeze_rides_the_stream();
    f3_bad += test_auto_recenter();
    f3_bad += test_flight_recorder_replays_bit_identical();
    f3_bad += test_rec_live_snapshot_machinery();
    f3_bad += test_camera_telemetry_transform();
    f3_bad += test_camera_yaw_anchor();
    f3_bad += test_hanging_camera();
    f3_bad += test_camera_step_height();
    f3_bad += test_camera_yaw_apply_integration();
    f3_bad += test_screen_pose_is_yaw_anchored();
    f3_bad += test_stick_turn_is_a_room_turn();
    f3_bad += test_config_defaults_are_the_proven_stand();
    f3_bad += test_aim_probe_config();
    bad += test_absolute_aim_transport();
    bad += test_absolute_record_replay();
    bad += test_position_turn_covariance();
    bad += test_controller_button_context();
    bad += dg_bridge_test_position_turn(position_turn_make_bridge_target);
    printf("  -> F3 desk tests: %s, %d/20 categories\n",
           f3_bad ? "FAILED" : "passed", 20 - f3_bad);
    bad += f3_bad;

    printf("\nU2 menu navigation (pure decision module)\n");
    bad += dg_menu_self_test();

    printf("\nU2 GV_PadPress anchor (shared resolver, synthetic image)\n");
    {
        /* The same finder the bridge and the offline scanner call, run against
           a hand-built image whose decoys are the two GV_PadMask words that sit
           four and eight bytes from the press read in the real function. One
           category, because it is one claim: this resolves GV_PadPress or it
           resolves nothing. */
        int anchor_bad = dg_anchors_pad_press_self_test() ? 0 : 1;
        printf("  %-6s anchor: two 0x20-guarded reads name GV_PadPress, the "
               "masks beside it do not fool the finder, and every mutation "
               "fails closed\n", anchor_bad ? "FAIL" : "PASS");
        printf("  -> U2 anchor desk tests: %s, %d/1 categories\n",
               anchor_bad ? "FAILED" : "passed", 1 - anchor_bad);
        bad += anchor_bad;
    }

    printf("\nF2 game-thread bridge (state machine, detour, anchor gate)\n");
    {
        int f2_bad = dg_bridge_self_test();
        printf("  -> F2 desk tests: %s, %d/55 categories\n",
               f2_bad ? "FAILED" : "passed", 55 - f2_bad);
        bad += f2_bad;
    }

    printf("\npolicy hot reload (dispatcher, staged loader, swap gate)\n");
    {
        /* The loader tests need the two DLLs test.bat builds next to this
           binary - the good one and the wrong-ABI-version one. Derived from
           the binary's own path so the test runs from any cwd. */
        char exe_dir[MAX_PATH];
        char *slash;
        int pol_bad;
        GetModuleFileNameA(NULL, exe_dir, sizeof(exe_dir));
        slash = strrchr(exe_dir, '\\');
        if (slash) *slash = 0;
        pol_bad = dg_policy_self_test(exe_dir);
        printf("  -> policy desk tests: %s, %d/9 categories\n",
               pol_bad ? "FAILED" : "passed", 9 - pol_bad);
        bad += pol_bad;
    }

    printf("\nscript input integration\n");
    bad += test_script_source_parser();
    bad += test_controls_producer();
    bad += test_m9_input();
    bad += test_blade_input();
    bad += test_reload_config();
    bad += test_script_input_routes();
    printf("\n%s\n", bad ? "FAILED" : "PASSED");
    return bad ? 1 : 0;
}
#endif

BOOL WINAPI DllMain(HINSTANCE inst, DWORD reason, LPVOID reserved) {
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(inst);
        CloseHandle(CreateThread(NULL, 0, worker, NULL, 0, NULL));
    }
    /* The recorder's last chance (ROLL V5.1 par. 4.5, R-7): the game closing
       kills the worker before its end-of-session dump, and every run since
       2026-08-30 lost its ring that way unless a marker token was flipped
       in time. On process teardown the other threads are already gone, so
       reading the ring here races nothing; the CRT is still up for the
       file write. FreeLibrary (reserved NULL) takes the same path. */
    if (reason == DLL_PROCESS_DETACH) {
        static LONG once;
        if (InterlockedExchange(&once, 1) == 0 &&
            InterlockedCompareExchange(&g_rec_on, 0, 0))
            rec_dump(reserved ? "process exit" : "library unload");
    }
    return TRUE;
}
