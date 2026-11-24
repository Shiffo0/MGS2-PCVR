#include "dg_build_profile.h"

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
#include "dg_grip_blend.h"              /* shared hand-to-grip transition */
#include "dg_pose.h"                    /* F5: stereo-latched controller pose */
#include "dg_fire.h"                    /* F7: trigger to the weapon pad contract */
#include "dg_rec.h"
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
static LONG g_script_camera_seen;
static LONG g_buttons_camera_seen;
static DWORD g_buttons_camera_ms, g_buttons_start_ms;
static int g_buttons_have_camera, g_buttons_start_block;
static int g_buttons_gameplay;
static unsigned g_buttons_epoch;
static DG_XR_BUTTON_GATE g_script_button_gate;
static int g_script_menu_cfg;
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
static int input_get_frame(DG_XR_FRAME *out) {
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

#define DG_FLAT_SILENT_FRAMES 30
static volatile LONG  g_flat_mode = 1;      /* vr_flat, on by default */
static volatile LONG  g_flat_silent;
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

static volatile LONG g_menu_dbg_trigger;    /* f2l float, raw travel */
static volatile LONG g_menu_dbg_click;      /* f2l float, threshold */
static volatile LONG g_menu_dbg_buttons;    /* bit0 A, bit1 B, bit2 input_ok */
static volatile LONG g_menu_dbg_samples;

static MAT     g_mine;                  /* last eye_inv we wrote */
static volatile LONG g_have_mine;
/* The game's OWN camera for the current frame, kept because the transform must
   be rebuilt from it rather than from our own output. See apply_transform. */

static volatile LONG g_xr_worker_owned;

#include "dg_capture_handoff.h"
static DG_NEAR_META g_near_pending;

static volatile LONG g_current_eye;
static volatile LONG g_present_frame;    /* increments exactly once per Present */
static volatile LONG g_stereo;
static volatile LONG g_stereo_test;   /* 0 full, 1 projection-only, 2 position-only */
static volatile LONG g_stereo_proj_x_sign; /* retail -1, +1 restores probe */
static volatile LONG g_stereo_eye_x_sign;  /* -1 (default): right eye at camera -x; +1 swaps */

static volatile LONG g_eye_truth_mode;
static long g_eye_truth_n[2][2];            /* [label eye][sign < 0] */
static volatile LONG g_eye_truth_none, g_eye_truth_mismatch, g_eye_truth_dropped, g_eye_truth_relabelled;

#define DG_NOCAM_DROP_MAX 6
static volatile LONG g_nocam_dropped, g_nocam_passed;

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

static volatile LONG g_draw_skip_mode, g_feedback_skip = 1;
static int g_left_snap_logged;   /* blade brace: left hand drawn on the blade */
/* Blade block: the left hand's own grip blend (dg_grip_blend.h), stepped on
   the left sample's OpenXR time, reset with the left stream. Counters for the
   heartbeat: samples part-way (0 < w < 1) and fully on the blade. */
static DG_GRIP_BLEND g_left_brace_blend;
static DG_BLADE_SNAP_MEMO g_left_brace_memo;
static volatile LONG g_left_brace_partial, g_left_brace_full;

static volatile LONG g_observer_cfg = 1, g_mod_observer_override = -1;
static volatile LONG g_observer_mode = 2, g_mod_observer_mode_override = -1;
static int observer_mode(void) {
    LONG o=InterlockedCompareExchange(&g_mod_observer_mode_override,0,0);
    return o>=0?(int)o:(int)InterlockedCompareExchange(&g_observer_mode,0,0);
}
static volatile LONG g_second_eye_frame = -100; /* last Present that stored B */
static volatile LONG g_second_eye_swapped;       /* Presents stored with the eye held */
static int observer_on(void) {
    LONG o = InterlockedCompareExchange(&g_mod_observer_override, 0, 0);
    return o >= 0 ? o != 0 : InterlockedCompareExchange(&g_observer_cfg, 0, 0) != 0;
}
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
static MAT g_pair_ep_pending[2];
static int g_pair_pending_valid;

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
    int frame_flags;
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

static int camera_gate_now(DG_CAMERA_GATE *out)
{
    return dg_bridge_camera_gate_now(out);
}

static void camera_pass_begin(void)
{
    dg_bridge_psg_camera(NULL);
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
                          const DG_PROJ_FOV *fov, const DG_XR_RAW_POSE *other_raw,
                          const DG_PROJ_FOV *other_fov) {
    InterlockedIncrement(&g_handoff_seq);       /* odd: reader must retry */
    g_handoff.near_meta = g_near_pending;
    if (!valid) g_handoff.near_meta.valid = 0;
    g_handoff.valid = valid;
    g_handoff.eye = eye;
    g_handoff.pair_valid = valid && g_pair_pending_valid;
    if(g_handoff.pair_valid)memcpy(g_handoff.pair_ep,g_pair_ep_pending,sizeof g_pair_ep_pending);
    else memset(g_handoff.pair_ep,0,sizeof g_handoff.pair_ep);
    if (raw) g_handoff.raw = *raw;
    else memset(&g_handoff.raw, 0, sizeof(g_handoff.raw));
    if (fov) g_handoff.fov = *fov;
    else memset(&g_handoff.fov, 0, sizeof(g_handoff.fov));
    g_handoff.other_valid = valid && other_raw && other_fov;
    if (g_handoff.other_valid) { g_handoff.other_raw = *other_raw; g_handoff.other_fov = *other_fov; }
    else { memset(&g_handoff.other_raw, 0, sizeof(g_handoff.other_raw)); memset(&g_handoff.other_fov, 0, sizeof(g_handoff.other_fov)); }
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
#include "dg_crash_trace.inl"
#include "dg_stereo_phase_probe.inl"
#include "dg_zoom_projection.h"
static DG_ZOOM_PROJECTION g_zoom_projection,g_psg_projection;
static double g_zoom_last_gain=1;
static uint64_t g_zoom_last_identity;

static int psg_head_camera(const MAT *base, const POSE *offset,
                           const DG_XR_FRAME *frame, MAT *out) {
    POSE hp=*offset; MAT d,di;
    hp.yaw+=frame->head.yaw;hp.pitch+=frame->head.pitch;hp.roll+=frame->head.roll;
    hp.tx+=frame->head.tx;hp.ty+=frame->head.ty;hp.tz+=frame->head.tz;
    build_delta(&d,&di,&hp);mat_mul(out,&di,base);
    return camera_rigid_valid(out);
}
/* ------------------------------------------------------------ the hook --- */

#define M(off) ((MAT *)(g_chan0 + (off)))

static void apply_transform(void) {
    /* Union FOV (stereo_test=2) only while Two Eye rendering is wanted; AER keeps per-eye FOVs. */
    LONG stereo_mode = g_stereo_test;
    if (stereo_mode == 2 && !second_eye_wanted()) stereo_mode = 0;
    dg_bridge_psg_camera(NULL);
    /* Raw gameplay context closes this before theater's display hysteresis.
       No D3D calls here: restoration belongs to the next native draw. */
    dg_ui2d_gameplay(g_armed && g_source==SRC_XR && g_stereo &&
        !script_menu_blocked() && !dg_bridge_theater_verdict() &&
        dg_bridge_controller_gameplay_now());
    MAT d, d_inv, n_eye_inv, n_eye, base_eye_inv, base_eye;
    MAT *pers2 = M(O_PERS2), *raise_pers = M(O_RAISE_PERS);
    MAT *raise_pers2 = M(O_RAISE_PERS2), *pers_no_offset = M(O_PERS_NO_OFFSET);
    MAT base_pers, base_pers2, base_raise_pers, base_raise_pers2;
    MAT base_pers_no_offset;
    DG_PROJ_FOV old_fov, new_fov, submit_fov;
    int have_target = 0, eye_idx = DG_EYE_LEFT, probe_frame_flags = 0;
    int probe_view_kind = -1;
    int eye_shift = 0;
    MAT pair_head_eye,pair_head_inv;
    double pair_zoom_gain=1.0;
    int scope_optics=0;
    uint64_t scope_id=0;
    float scope_angle=0;
    int stereo_now = g_stereo;
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
    g_pair_pending_valid=0;
    handoff_write(0, DG_EYE_MONO, NULL, NULL, NULL, NULL);
    dg_bridge_blade_camera(NULL);

    {
        DG_CAMERA_GATE radar_gate;
        dg_radar_view(g_source==SRC_XR && camera_gate_now(&radar_gate));
    }
    {
        int verdict = dg_bridge_theater_verdict();
        /* Fission Mailed keeps the head camera (the game still plays), but
           mono: the theater panel shows one image, not alternating eyes. */
        stereo_now = g_stereo && !(verdict & DG_THEATER_V_FAKE);
        if (verdict & ~DG_THEATER_V_FAKE) {
            dg_radar_view(0);
            camera_yaw_anchor_reset();
            InterlockedIncrement(&g_thea_cam_released);
            return;
        }
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
            {
                scope_optics=g_source==SRC_XR && (flags & 3)==3 && !dg_bridge_psg_hip_now() &&
                    dg_bridge_psg_render_zoom(&scope_id,&scope_angle);
            }
            if (stereo_now && scope_optics) {
                /* One optical image, with the displayed centre on the shot axis. */
                xp=frame.head;
                use_raw=&frame.head_raw;
                probe_view_kind=2;
                new_fov=frame.union_fov;
                new_fov.right=fmax(new_fov.right,-new_fov.left);
                new_fov.left=-new_fov.right;
                new_fov.up=fmax(new_fov.up,-new_fov.down);
                new_fov.down=-new_fov.up;
                submit_fov=new_fov;
                dg_ui2d_display(new_fov.left,new_fov.right);
            } else if (stereo_now) {
                /* Stereo changes two things at once - where each eye is, and
                   what frustum it renders - and double vision can come from
                   either. stereo_test isolates them so one launch answers it
                   instead of an argument:
                     1 = per-eye PROJECTION only, both eyes drawn from the head
                     2 = per-eye POSITION only, both at S4c's union FOV
                   The layer pose must follow whichever was used, or the image
                   is submitted from somewhere it was not drawn. */
                LONG mode = stereo_mode;
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

                submit_fov = new_fov;
                dg_ui2d_display(submit_fov.left, submit_fov.right);
                if (mode != 2 &&
                    InterlockedCompareExchange(&g_stereo_proj_x_sign, 0, 0) < 0)
                    new_fov = mirror_fov_x(new_fov);
            } else {
                xp = frame.head;
                new_fov = frame.union_fov;
                if(scope_optics){
                    new_fov.right=fmax(new_fov.right,-new_fov.left);new_fov.left=-new_fov.right;
                    new_fov.up=fmax(new_fov.up,-new_fov.down);new_fov.down=-new_fov.up;
                }
            }
            have_target = (flags & 2) != 0;
        }

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
            /* vr_psg_rifle HIP (scope closed): no PSG zoom on the view. The
               photo-camera route below never owns the PSG zoom actor (it
               requires a camera item), so it stays as it was. */
            int psg=scope_optics;
            if(psg){identity=scope_id;angle=scope_angle;}
            int valid=!psg && g_source==SRC_XR && dg_bridge_zoom_now(&identity,&angle);
            double gain=dg_zoom_projection_step(&g_zoom_projection,identity,valid,angle,
                fabs((double)base_pers.m[0][0]),fabs((double)base_pers.m[1][1]),stereo_now,eye_idx);
            double psg_gain=dg_psg_projection_step(&g_psg_projection,identity,psg,angle,stereo_now && !second_eye_wanted(),eye_idx);
            if(psg)gain=psg_gain;
            {
                static DG_ZOOM_PROJECTION optics_projection;
                int optic=!psg && g_source==SRC_XR && dg_bridge_optics_zoom_now(&identity,&angle);
                double optic_gain=dg_optics_projection_step(&optics_projection,identity,optic,angle,
                    stereo_now && !second_eye_wanted(),eye_idx);
                if(optic){gain=optic_gain;psg=2;}
            }
            pair_zoom_gain=gain;
            dg_zoom_projection_apply(pers,gain);dg_zoom_projection_apply(pers2,gain);
            dg_zoom_projection_apply(raise_pers,gain);dg_zoom_projection_apply(raise_pers2,gain);
            dg_zoom_projection_apply(pers_no_offset,gain);
            if(gain!=g_zoom_last_gain || identity!=g_zoom_last_identity) {
                /* A complete scoped pair can stay on screen until the next one
                   arrives. Flushing it for each focal step starves XR submission. */
                if(dg_zoom_drop_stores(psg,identity,g_zoom_last_identity))dg_xr_zoom_changed();
                if(psg==2)logf_("  optics XR projection: optic=%llx angle=%.4f gain=%.6f world_x=%.6f world_y=%.6f eye=%d\r\n",
                    (unsigned long long)identity,angle,gain,pers->m[0][0],pers->m[1][1],eye_idx);
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
    /* Rebuild the central head camera even when rendering falls back to a
       per-eye pose. Never publish eye separation or controller orientation. */
    if (g_source == SRC_XR && (probe_frame_flags & 1)) {
        MAT central;
        if (scope_optics) dg_bridge_psg_camera(n_eye.m);
        else if (psg_head_camera(&base_eye, &g_pose, &frame, &central))
            dg_bridge_psg_camera(central.m);
    }

    pair_head_eye=n_eye;pair_head_inv=n_eye_inv;
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
    if(g_source==SRC_XR && (probe_frame_flags&1))
        dg_bridge_blade_camera(n_eye.m);
    /* Derive the companion from this exact head sample and native projection.
       Fallback per-eye poses are deliberately excluded until their ownership
       and asymmetric orientation are supported. No second simulation update. */
    if(g_source==SRC_XR && stereo_now && have_target && (probe_frame_flags&3)==3 &&
       (eye_shift || stereo_mode==1 || scope_optics)) {
        int e,ok=1;
        int sign=(int)InterlockedCompareExchange(&g_stereo_eye_x_sign,0,0);
        int mirror=(int)InterlockedCompareExchange(&g_stereo_proj_x_sign,0,0)<0;
        for(e=0;e<2;e++) {
            MAT ew=pair_head_eye,ei=pair_head_inv,pr=base_pers;
            DG_PROJ_FOV f=stereo_mode==2?frame.union_fov:frame.eye[e].fov;
            if(scope_optics)f=new_fov;
            else if(stereo_mode!=2 && mirror)f=mirror_fov_x(f);
            dg_proj_retarget(&pr,&old_fov,&f);
            dg_zoom_projection_apply(&pr,pair_zoom_gain);
            if(eye_shift)stereo_eye_shift(&ew,&ei,e,eye_half,sign);
            mat_mul(&g_pair_ep_pending[e],&ei,&pr);
            if(!finite_mat(&g_pair_ep_pending[e]))ok=0;
        }
        /* Native source camera must be bit-identical to the labelled member. */
        if(memcmp(&g_pair_ep_pending[eye_idx],eye_pers,sizeof(MAT)))ok=0;
        g_pair_pending_valid=ok;
    }

    dg_cb_probe_camera(stereo_now ? eye_idx : -1, (const float *)eye_pers->m,
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
        g_aim_camera.frame_flags = probe_frame_flags;
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
        if (stereo_now) {
            /* The other eye of this same head sample, as the pair's target was built. */
            int other = eye_idx == DG_EYE_RIGHT ? DG_EYE_LEFT : DG_EYE_RIGHT;
            DG_PROJ_FOV other_fov = stereo_mode == 2 ? frame.union_fov : frame.eye[other].fov;
            handoff_write(1, eye_idx, use_raw, &submit_fov,
                          scope_optics ? &frame.head_raw : stereo_mode == 1 ? NULL : &frame.eye[other].raw,
                          scope_optics ? &submit_fov : &other_fov);
        } else
            handoff_write(1, DG_EYE_MONO, NULL, &new_fov, NULL, NULL);
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

static double g_arm_pos_sign[3] = { -1.0, 1.0, 1.0 };

static double g_arm_shoulder_mm[3] = { 180.0, 200.0, 40.0 };

static int g_arm_frame = DG_XR_REL_FRAME_ROOM;

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

static void arm_pos_cfg_defaults(ARM_POS_CFG *a)
{
    a->sign[0] = -1.0; a->sign[1] = 1.0; a->sign[2] = 1.0;
    a->shoulder_mm[0] = 180.0;
    a->shoulder_mm[1] = 200.0;
    a->shoulder_mm[2] = 40.0;
    a->reach_mm = 600.0;

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

static LONG g_fire_mode = 0;
static LONG g_move_mode = 0;
static LONG g_turn_mode = 0;

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

static void arm_position_to_view(const DG_XR_REL_POSE *p,
                                 const DG_XR_CONFIG *cfg, double out[3])
{

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

static void arm_frame_reference(const DG_XR_RAW_POSE *head,
                                DG_XR_RAW_POSE *ref)
{
    double q[4];

    *ref = *head;
    if (g_arm_frame == DG_XR_REL_FRAME_HEAD) return;
    if (g_arm_frame == DG_XR_REL_FRAME_ROOM && g_arm_frame_have) {

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

static double g_arm_raw_rel[3];
static int    g_arm_raw_have;

static DG_REC_RING   g_rec;
static volatile LONG g_rec_on = 1;
static volatile LONG g_rec_cfg_on = 0;
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

static double arm_pose_dt(void)
{
    LARGE_INTEGER now;
    double dt = 1.0 / 90.0;
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

static int rec_snapshot_due(int on, long appended, int elapsed,
                            long *last_appended, int *last_elapsed)
{
    if (!on || appended == *last_appended) return 0;
    if (elapsed - *last_elapsed < DG_REC_SNAPSHOT_S) return 0;
    *last_appended = appended;
    *last_elapsed = elapsed;
    return 1;
}

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

static int arm_absolute_prepare(DG_AIM_SELECTION *selection)
{
    if ((!g_aim_camera.valid) ||
        ((g_arm_track != ARM_TRACK_R_GRIP && g_arm_track != ARM_TRACK_R_AIM))) return 0;
    if ((!dg_aim_capture_hand_selection((uintptr_t)g_base,
                                     g_aim_camera.gate.arm_body, selection))) return 0;
    if((selection->weapon_id==13 && !dg_bridge_blade_enabled()))return 0;
    if((selection->weapon_id==7 && !dg_bridge_stinger_enabled()))return 0;
    if(((selection->weapon_id==4 || selection->weapon_id==19) && !dg_bridge_psg_rifle_enabled()))return 0;
    if (g_absolute_stream != g_arm_pose_stream_id ||
        g_absolute_camera != g_aim_camera.gate.camera ||
        memcmp(selection, &g_absolute_selection, sizeof *selection)) {
        /* Weapon/camera replacement cannot inherit the old pose latch. */
        if (g_absolute_stream == g_arm_pose_stream_id) {

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

/* Camera-seam ownership. Manual requests are bounded to the current safe,
   unarmed pose stream; a held gesture or stale request cannot recalibrate a
   later level. The profile itself is pair-latched below. */
static void persistent_hand_input(DG_BRIDGE_ARM_TARGET *t,int absolute) {
    static DG_HAND_REQUEST request;
    static DG_HAND_PROFILE pair_profile;
    unsigned long press=input_secondary_presses(DG_XR_HAND_RIGHT);
    unsigned long previous=request.request;
    int eligible=absolute && g_source==SRC_XR && t->hands_coherent &&
        t->free_left.valid && t->free_right.valid && t->aim_weapon_id==0 &&
        t->position.valid && t->left_position.valid;
    t->persistent_hands=absolute && g_source==SRC_XR;
    t->hand_calibration_request=dg_hand_request_step(&request,press,eligible,t->stream_id,GetTickCount());
    if(t->hand_calibration_request && t->hand_calibration_request!=previous)
        logf_("  hands: manual unarmed alignment requested\r\n");
    if(!t->persistent_hands)return;
    if(!(t->pose_flags&DG_POSE_F_LATCHED) || !dg_hand_profile_valid(&pair_profile))
        dg_bridge_hand_profile_get(&pair_profile);
    t->free_left.persistent=t->free_right.persistent=1;
    memcpy(t->free_left.alignment,pair_profile.rotation[0],sizeof t->free_left.alignment);
    memcpy(t->free_right.alignment,pair_profile.rotation[1],sizeof t->free_right.alignment);
}
static void hand_profile_worker(void) {
    int status=dg_bridge_hand_profile_worker();
    if(status==1)logf_("  hands: manual alignment saved to dg_hand_calibration.bin\r\n");
    if(status==2)logf_("  hands: saved alignment loaded\r\n");
    if(status==3)logf_("  hands: automatic controller-grip defaults active\r\n");
    if(status<0)logf_("  hands: calibration file invalid or I/O failed; keeping current alignment\r\n");
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
        (dg_weapon_pistol_support((int)target->aim_weapon_id) ||
         ((dg_bridge_twohand_flags() & 1) && dg_weapon_rifle_support((int)target->aim_weapon_id)));
    if (target->left_enabled) {
        DG_POSE_IN li;
        DG_POSE_OUT lo;
        DG_XR_REL_POSE lr;
        const DG_XR_HAND_POSE *lp = &f.left_hand.grip;
        const DG_XR_HAND_POSE *rp = &f.right_hand.grip;
        DG_XR_RAW_POSE left_raw;
        int snapped = 0;
        double dx, dy, dz;
        int coherent_now;
        unsigned long left_stream=target->left_stream_id ? target->left_stream_id : target->stream_id;
        if (g_left_stream != left_stream) {
            dg_pose_init(&g_left_pose, NULL);
            memset(&g_left_sample,0,sizeof g_left_sample);
            dg_grip_blend_reset(&g_left_brace_blend);
            g_left_brace_memo.have = 0;
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
        /* Blocking: the left hand rests on the blade. One adjusted position
           feeds both the arm mapping and the absolute hand target. */
        left_raw = lp->raw_local;
        target->brace_weight = 0;
        if (li.valid) {
            const DG_XR_RAW_POSE *q = &f.right_hand.aim.raw_local;
            DG_GRIP_TIMES brace_times;
            double grip[3], fwd[3], left[3], w;
            int have_blade = f.right_hand.grip.position_valid && f.right_hand.aim.orientation_valid;
            grip[0]=f.right_hand.grip.raw_local.px; grip[1]=f.right_hand.grip.raw_local.py;
            grip[2]=f.right_hand.grip.raw_local.pz;
            fwd[0]=-2*(q->qx*q->qz+q->qw*q->qy);
            fwd[1]=-2*(q->qy*q->qz-q->qw*q->qx);
            fwd[2]=-(1-2*(q->qx*q->qx+q->qy*q->qy));
            left[0]=left_raw.px; left[1]=left_raw.py; left[2]=left_raw.pz;
            dg_grip_unpack(dg_bridge_grip_packed(), DG_GRIP_BRACE, &brace_times);
            w = dg_blade_brace_visible(&g_left_brace_blend, &g_left_brace_memo, dg_bridge_blade_braced(),
                    lp->xr_time, &brace_times, have_blade, grip, fwd, left);
            if (w > 0) {
                left_raw.px=left[0]; left_raw.py=left[1]; left_raw.pz=left[2];
                target->brace_weight = w;
                snapped = 1;
                InterlockedIncrement(w < 1 ? &g_left_brace_partial : &g_left_brace_full);
            }
        }
        if (snapped != g_left_snap_logged) {
            g_left_snap_logged = snapped;
            logf_("  blade brace: left hand %s\r\n", snapped ? "snap input active" : "follows the controller");
        }
        if (li.valid) {
            arm_hand_to_view(&f.head_raw, &left_raw, &cfg, &lr, li.pos);
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
            g_left_sample.position.grip[0]=left_raw.px;
            g_left_sample.position.grip[1]=left_raw.py;
            g_left_sample.position.grip[2]=left_raw.pz;
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
        if (((absolute ? g_aim_camera.frame_flags : frame_flags) & 3)==3 &&
            li.valid && target->left_position.valid &&
            f.left_hand.aim.active && f.left_hand.aim.tracked && f.left_hand.aim.orientation_valid &&
            f.left_hand.aim.pose_age_ms<=100 && f.left_hand.aim.sample_seq==lp->sample_seq &&
            f.left_hand.aim.xr_time==lp->xr_time &&
            f.right_hand.grip.active && f.right_hand.grip.tracked &&
            f.right_hand.grip.sample_seq==lp->sample_seq && f.right_hand.grip.xr_time==lp->xr_time) {
            const DG_XR_RAW_POSE *grip=&lp->raw_local,*aim=&f.left_hand.aim.raw_local,*head=&f.head_raw;
            for (i=0;i<2;i++) {
                target->optics.eyes[i][0]=f.eye[i].raw.px;
                target->optics.eyes[i][1]=f.eye[i].raw.py;
                target->optics.eyes[i][2]=f.eye[i].raw.pz;
            }
            target->optics.head_q[0]=head->qx;target->optics.head_q[1]=head->qy;
            target->optics.head_q[2]=head->qz;target->optics.head_q[3]=head->qw;
            target->optics.grip_q[0]=aim->qx;target->optics.grip_q[1]=aim->qy;
            target->optics.grip_q[2]=aim->qz;target->optics.grip_q[3]=aim->qw;
            target->optics.grip_m[0]=grip->px;target->optics.grip_m[1]=grip->py;
            target->optics.grip_m[2]=grip->pz;
            target->optics.left=!!f.left_hand.squeeze_click;
            target->optics.right=!!f.right_hand.squeeze_click;
            target->optics.valid=1;
        }
        m9_sample_from_frame(&f,1,&target->m9_pose);
        if(!target->hands_coherent || target->m9_pose.sequence!=target->left_sample_seq ||
           target->m9_pose.sequence!=target->aim_sample_seq)target->m9_pose.valid=0;
    }
    persistent_hand_input(target,absolute);
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

static volatile LONG g_action_route_denied;
static volatile LONG g_camera_route_denied=1;
#include "dg_psg_gesture.h"
/* vr_psg_rifle gesture state. Only the action provider touches it; every
   caller holds the bridge's exclusive action lock. */
static DG_PSG_GESTURE g_psg_gesture;
static void psg_gesture_from_frame(const DG_XR_FRAME *f,DG_ACTION_SAMPLE *out) {
    const DG_XR_HAND *w;DG_PSG_GESTURE_CFG cfg;int valid,g;
    double hp[3],hq[4],gp[3],aq[4];
    if(!dg_bridge_psg_rifle_enabled())return;   /* key off: no state, no field */
    w=(g_arm_track==ARM_TRACK_L_GRIP || g_arm_track==ARM_TRACK_L_AIM) ? &f->left_hand : &f->right_hand;
    valid=w->grip.active && w->grip.tracked && w->grip.position_valid && w->grip.sample_seq &&
        w->grip.pose_age_ms<=100 && w->aim.active && w->aim.tracked && w->aim.orientation_valid &&
        w->aim.pose_age_ms<=100;
    hp[0]=f->head_raw.px;hp[1]=f->head_raw.py;hp[2]=f->head_raw.pz;
    hq[0]=f->head_raw.qx;hq[1]=f->head_raw.qy;hq[2]=f->head_raw.qz;hq[3]=f->head_raw.qw;
    gp[0]=w->grip.raw_local.px;gp[1]=w->grip.raw_local.py;gp[2]=w->grip.raw_local.pz;
    aq[0]=w->aim.raw_local.qx;aq[1]=w->aim.raw_local.qy;aq[2]=w->aim.raw_local.qz;aq[3]=w->aim.raw_local.qw;
    dg_bridge_psg_gesture_cfg(&cfg);
    g=dg_psg_gesture_step(&g_psg_gesture,&cfg,valid,hp,hq,gp,aq);
    out->psg_gesture=g;
    if(g){out->psg_gesture_mm=(int)(g_psg_gesture.last_m*1000.0+0.5);
          out->psg_gesture_ddeg=(int)(g_psg_gesture.last_deg*10.0+0.5);}
}
static void action_from_frame(DG_ACTION_SAMPLE *out) {
    DG_XR_FRAME f; const DG_XR_HAND *l,*r;
    memset(out,0,sizeof *out);
    if(!InterlockedCompareExchange(&g_armed,0,0) || g_source!=SRC_XR ||
       InterlockedCompareExchange(&g_script_menu_active,0,0) || !(input_get_frame(&f)&1)) return;
    psg_gesture_from_frame(&f,out);
    l=&f.left_hand;r=&f.right_hand;
    out->source=((uint64_t)g_arm_pose_stream_id<<8)|(unsigned)(g_source+1);
    out->sample=l->grip.sample_seq;out->down=l->primary_button!=0;
    out->shutter_down=r->trigger_click!=0;
    out->zoom_y=r->thumbstick_y;out->zoom_active=r->thumbstick_active!=0;
    out->psg_grip_zoom=(r->squeeze_click!=0)-(l->squeeze_click!=0);
    out->shutter_denied=InterlockedCompareExchange(&g_camera_route_denied,0,0)!=0;
    out->age_ms=l->grip.pose_age_ms>r->grip.pose_age_ms?l->grip.pose_age_ms:r->grip.pose_age_ms;
    out->valid=out->sample && out->sample==r->grip.sample_seq && l->grip.active && r->grip.active &&
        l->grip.tracked && r->grip.tracked && l->grip.orientation_valid && r->grip.orientation_valid;
    out->denied=InterlockedCompareExchange(&g_action_route_denied,0,0) ||
        l->thumbstick_click || r->thumbstick_click;
}
#include "dg_m9_input.inl"
#include "dg_blade_input.inl"
#include "dg_stinger_input.inl"
#include "dg_nikita_input.inl"
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

static void chanl1_step(void);   /* dg_chanl_probe.inl */
static void nocull_step(void);   /* dg_chanl_probe.inl */
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
        dg_ui2d_gameplay(0);
        script_menu_sample_context();
        c->EFlags |= 0x10000;
        c->Dr6 = 0;
        return EXCEPTION_CONTINUE_EXECUTION;
    }
    {

    void *pol_pass = dg_policy_pass_begin();
    camera_pass_begin();
    if (g_armed) apply_transform();
    if (g_armed) chanl1_step();   /* vr_chanl1 (diagnostic): engine channel 1 */
    if (g_armed) nocull_step();   /* vr_nocull (diagnostic): engine bound check off */

    {
        DG_BRIDGE_ARM_TARGET target;
        DG_REC_FRAME r;
        int record_on = 1 && InterlockedCompareExchange(&g_rec_on, 0, 0) != 0;
        int record_frame = 0, have_target;

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
            if (1 && (record_frame || r.absolute.present)) dg_rec_capture(&g_rec, &r);
        }
        dg_bridge_arm_seam_now(have_target ? &target : NULL);
        if (1 && InterlockedCompareExchange(&g_camera_telemetry_valid, 0, 0))
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

/* Zero explicitly disables the developer session timer. */
static int session_timer_expired(double seconds, int elapsed) {
    return seconds > 0.0 && (double)elapsed >= seconds;
}

#include "dg_devstage.h"                /* testlevel: vr_dev_pool / spawn_n / spawn_kind values */
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
    xc->stereo = 1;
    xc->stereo_test = 2;
    xc->second_eye = 1;
    xc->scene_blur_fix = 1;
    xc->stereo_phase_fix = 0;
    xc->stereo_eye_x_sign = -1;
    xc->ui2d = 0; xc->ui2d_scale_mils = 750; xc->ui2d_conv_e5 = 1200; xc->ui2d_sign = 1; xc->ui2d_hold = 0; xc->feedback_skip = 1; xc->observer = 1; xc->observer_mode = 1;

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
            if (_finite(v) && v >= 0.0) *seconds = v;
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

        { char *lower=key; for (; *lower; ++lower)
            if (*lower>='A' && *lower<='Z') *lower=(char)(*lower+('a'-'A')); }
        if (strstr(key, "probe") || strstr(key, "dump") || strstr(key, "debug") ||
            !_stricmp(key,"vr_rec") || !_stricmp(key,"vr_policy") ||
            !_stricmp(key,"vr_eye_truth") || !_strnicmp(key,"vr_dev_",7) ||
            !_stricmp(key,"vr_se_mode") || !_stricmp(key,"vr_se_near") ||
            !_stricmp(key,"vr_chanl1") || !_stricmp(key,"vr_nocull") || !_stricmp(key,"vr_xr_sim")) {
            while (*s && *s!='\n' && *s!='\r') ++s;
            continue;
        }
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
        if (_stricmp(key, "vr_pistol_reload_v2") == 0) {
            char *e=s;
            while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            bc->pistol_reload_v2=(e-s==2 && !_strnicmp(s,"on",2)) ? 1 : (e-s==5 && !_strnicmp(s,"touch",5)) ? 2 : 0;
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
        if (_stricmp(key, "vr_blade_armplace") == 0) {

            char *e=s;
            while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s==3 && !_strnicmp(s,"off",3))bc->blade_armplace=0;
            else if(e-s==2 && !_strnicmp(s,"m9",2))bc->blade_armplace=1;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_blade_live") == 0) {

            char *e=s;
            while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s==1 && *s>='0' && *s<='2')bc->blade_live=*s-'0';
            else return 0;
            s=e;continue;
        }
        if (!_stricmp(key, "vr_blade_haptic") || !_stricmp(key, "vr_fp_rehit")) {

            char *e=s;int v;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))v=0;
            else if(e-s==3 && !_strnicmp(s,"off",3))v=1;
            else return 0;
            if(!_stricmp(key,"vr_blade_haptic"))bc->blade_haptic_off=v;else bc->fp_rehit_off=v;
            s=e;continue;
        }
        if (_stricmp(key, "vr_blade_hit_speed") == 0) {

            char *e=NULL;double v=strtod(s,&e);
            if(!e || e==s || (*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a) || !_finite(v) ||
               !(v==0 || (v>=0.5 && v<=20)))return 0;
            bc->blade_hit_speed=v;bc->blade_hit_speed_set=1;
            s=e;continue;
        }
        if (_stricmp(key, "vr_twohand_rifles") == 0 || _stricmp(key, "vr_twohand_steer") == 0) {

            char *e=s;int v;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))v=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))v=0;
            else return 0;
            if(!_stricmp(key,"vr_twohand_rifles"))bc->twohand_rifles=v;else bc->twohand_steer=v?1:2;
            s=e;continue;
        }
        if (_stricmp(key, "vr_twohand_reach") == 0 || _stricmp(key, "vr_twohand_anchor") == 0) {

            char *e=s;int v,isreach=!_stricmp(key,"vr_twohand_reach");
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(isreach && e-s==5 && !_strnicmp(s,"clamp",5))v=1;
            else if(isreach && e-s==7 && !_strnicmp(s,"release",7))v=2;
            else if(!isreach && e-s==4 && !_strnicmp(s,"keep",4))v=1;
            else if(!isreach && e-s==4 && !_strnicmp(s,"live",4))v=2;
            else return 0;
            if(isreach)bc->twohand_reach=v;else bc->twohand_anchor=v;
            s=e;continue;
        }
        if (_stricmp(key, "vr_left_reach_centre") == 0) {

            char *e=s;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))bc->left_reach_centre=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))bc->left_reach_centre=2;
            else if(e-s==4 && !_strnicmp(s,"root",4))bc->left_reach_centre=3;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_blade_follow") == 0) {

            char *e=s;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))bc->blade_follow=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))bc->blade_follow=2;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_twohand_latch") == 0) {

            char *e=NULL;long a=strtol(s,&e,10),b;
            if(!e || e==s || *e!=0x2c)return 0;
            s=e+1;b=strtol(s,&e,10);
            if(!e || e==s || (*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a))return 0;
            if(a<50 || a>200 || b<a+20 || b>400)return 0;
            bc->twohand_latch_enter_mm=(int)a;bc->twohand_latch_exit_mm=(int)b;
            s=e;continue;
        }
        if (_stricmp(key, "vr_psg_rifle") == 0) {

            char *e=s;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))bc->psg_rifle=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))bc->psg_rifle=0;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_optics") == 0) {
            char *e=s;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))bc->optics=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))bc->optics=0;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_psg_move") == 0) {

            char *e=s;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(e-s==2 && !_strnicmp(s,"on",2))bc->psg_move=1;
            else if(e-s==3 && !_strnicmp(s,"off",3))bc->psg_move=0;
            else return 0;
            s=e;continue;
        }
        if (_stricmp(key, "vr_psg_gesture") == 0) {

            int v[4],n=0;DG_PSG_GESTURE_CFG probe;
            while(n<4) {
                char *e=NULL;long x;
                if(*s<'0' || *s>'9')return 0;
                x=strtol(s,&e,10);
                if(!e || e==s || e-s>3)return 0;
                v[n++]=(int)x;s=e;
                if(n<4){if(*s!=',')return 0;s++;}
            }
            if(*s && *s!=0x20 && *s!=0x09 && *s!=0x0d && *s!=0x0a)return 0;
            if(!dg_psg_gesture_cfg_set(&probe,v[0],v[1],v[2],v[3]))return 0;
            bc->psg_gesture_set=1;bc->psg_open_cm=v[0];bc->psg_close_cm=v[1];
            bc->psg_open_deg=v[2];bc->psg_close_deg=v[3];
            continue;
        }
        if (_stricmp(key, "vr_grip_ms") == 0) {

            char *e=s;
            while(*e && *e!=0x20 && *e!=0x09 && *e!=0x0d && *e!=0x0a)e++;
            if(!dg_grip_parse(s,(size_t)(e-s),&bc->grip_mode,&bc->grip_grab_ms,&bc->grip_release_ms))return 0;
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
        if (_stricmp(key,"vr_second_camera_probe")==0) {
            char *e=s;while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s!=1 || (*s!='0' && *s!='1'))return 0;
            xc->second_camera_probe=*s=='1';
            s=e;continue;
        }
        if (_stricmp(key,"vr_second_eye")==0) {
            char *e=s;while(*e && *e!=' ' && *e!='\t' && *e!='\r' && *e!='\n')e++;
            if(e-s!=1 || (*s!='0' && *s!='1'))return 0;
            xc->second_eye=*s=='1';
            s=e;continue;
        }
        if (_stricmp(key,"vr_stereo_phase_fix")==0) {
            char *e=s;
            while(*e&&*e!=' '&&*e!='\t'&&*e!='\r'&&*e!='\n')e++;
            if(e-s!=1||(*s!='0'&&*s!='1'))return 0;
            xc->stereo_phase_fix=*s=='1';s=e;continue;
        }
        if (_stricmp(key, "vr_rec") == 0) {

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

            bc->follow_src = (_strnicmp(s, "head", 4) == 0) ? 0 :
                             (_strnicmp(s, "hand", 4) == 0) ? 1 : 2;
            while (*s && *s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') s++;
            continue;
        }
        if (_stricmp(key, "vr_adjust_frame") == 0) {

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
        else if (_stricmp(key, "seconds")  == 0) { if (_finite(v) && v >= 0.0) *seconds = v; }
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
        else if (_stricmp(key, "vr_observer_mode") == 0) { if (v == 0.0 || v == 1.0 || v == 2.0) xc->observer_mode = (int)v; }
        else if (_stricmp(key, "vr_observer") == 0) xc->observer = (v != 0.0);
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
#include "dg_laser_ui.h"
#include "dg_laser_settings.inl"
static int read_config(const char *marker, POSE *p, double *seconds,
                       int *source, DG_XR_CONFIG *xc, DG_BRIDGE_CONFIG *bc,
                       int *arm_track_out, int arm_qmap_out[4],
                       int *arm_hand_out, ARM_POS_CFG *arm_pos_out)
{
    HANDLE h;

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
    if(!parse_config(buf,p,seconds,source,xc,bc,arm_track_out,arm_qmap_out,arm_hand_out,arm_pos_out))return 0;
    if(*source==SRC_XR && !_stricmp(strrchr(marker,'\\')?strrchr(marker,'\\')+1:marker,MARKER))
        laser_read_settings(xc,bc);
    return 1;
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
          "  fps move flag %d (set on %ld ticks)  fps toggle flag %d\r\n"
          "  bridge masks: subject pad bit 0x%08X  stop-aim pad bit 0x%08X\r\n"
          "  bridge status: player 0x%016llX  game 0x%08X  menu 0x%08X\r\n",
          bs.override_value, bs.toggle_value, bs.move_value,
          bs.subject_move_value, bs.subject_move_ticks, bs.subject_toggle_value,
          bs.pad_subject_mask, bs.pad_stop_aim_mask,
          bs.player_status, bs.game_status, bs.menu_status);

    /* Observed, never written. Degrees are printed beside the raw units
       because 4096-to-a-turn is not a number anyone reads at a glance, and
       whether these are small angles is the whole question. */

    logf_("  bridge gate: camera-seam status samples %ld (latch stale %ld)"
          "  writer stood down on its own reading %ld times" DG_EOL,
          bs.late_status_samples, bs.late_status_stale,
          bs.seam_refused_unsafe);

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
    logf_("  theater bits: DEMO %ld  scene demo %ld  PAD_DEMO %ld  RADIO %ld"
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

    if (InterlockedCompareExchange(&g_menu_mode, 0, 0)) {

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
    logf_("  bridge arm: player arm body 0x%016llX  objs 0x%016llX"
          "  flag 0x%08X [ch0 %s, ch1 %s, ch2 %s, ch3 %s]"
          "  created %ld  destroyed %ld\r\n",
          bs.arm_body, bs.arm_objs, bs.arm_flag,
          (bs.arm_flag & 0x1000) ? "hidden" : "SHOWN",
          (bs.arm_flag & 0x2000) ? "hidden" : "SHOWN",
          (bs.arm_flag & 0x4000) ? "hidden" : "SHOWN",
          (bs.arm_flag & 0x8000) ? "hidden" : "SHOWN",
          bs.arm_created, bs.arm_destroyed);

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
    {   DG_GRIP_TIMES gs, gb;
        long gp = dg_bridge_grip_packed();
        dg_grip_unpack(gp, DG_GRIP_SUPPORT, &gs);
        dg_grip_unpack(gp, DG_GRIP_BRACE, &gb);
        logf_("  bridge grip blend: mode %ld support %d/%d %s block %d/%d %s"
              "  block samples part-way %ld on-blade %ld" DG_EOL,
              gp & 15, gs.grab_ms, gs.release_ms, gs.linear ? "linear" : "smooth",
              gb.grab_ms, gb.release_ms, gb.linear ? "linear" : "smooth",
              (long)g_left_brace_partial, (long)g_left_brace_full);
    }
    logf_("  bridge twohand: rifles %s steer %s  swing computed %ld refused %ld applied %ld  last lever %ld mm angle %.1f deg" DG_EOL,
          (dg_bridge_twohand_flags() & 1) ? "on" : "off", (dg_bridge_twohand_flags() & 2) ? "on" : "off",
          bs.left_swing_frames, bs.left_swing_refused, bs.left_swing_applied,
          bs.left_swing_lever_mm, bs.left_swing_deci_deg / 10.0);
    logf_("  bridge support gates: aim/input %ld right-commit %ld anchor %ld"
          " unreachable %ld distance %ld dwell %ld attached %ld (beyond the left arm %ld)  anchor kept %ld"
          "  left reach centre: used %ld (beyond reach %ld) resets %ld" DG_EOL,
          bs.left_support_gate[0],bs.left_support_gate[1],bs.left_support_gate[2],
          bs.left_support_gate[3],bs.left_support_gate[4],bs.left_support_gate[5],
          bs.left_support_gate[6],bs.left_support_overreach,bs.left_anchor_kept,
          bs.left_centre_used,bs.left_centre_clamped,bs.left_centre_resets);
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

    ;
    dg_present_gpu_report(logf_);

    if (bs.skel_n_models || bs.skel_stride) {
        int i;
        logf_("  bridge skel: objs 0x%016llX  region +0x%X  n_models %d"
              "  stride 0x%X (score %d, %d rejected)" DG_EOL,
              bs.skel_objs, bs.skel_region_end, bs.skel_n_models,
              bs.skel_stride, bs.skel_stride_score, bs.skel_stride_tried);
        logf_("  bridge skel: motion control trans 0x%016llX [%s]" DG_EOL,
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
              "   (+0x00 objs, +0x08 motion control, +0x30 evmobj)" DG_EOL,
              bs.arm_obj_head[0], bs.arm_obj_head[1], bs.arm_obj_head[2],
              bs.arm_obj_head[3], bs.arm_obj_head[4], bs.arm_obj_head[5],
              bs.arm_obj_head[6], bs.arm_obj_head[7]);
    if (bs.arm_mctrl) {
        int i;
        for (i = 0; i < 10; i += 5)
            logf_("  bridge bend: motion control 0x%016llX +0x%02X: "
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

    /* The assumption the whole strip rests on, finally measured. Anything
       but ~0 here means the hierarchy did not keep what we wrote into
       joints 4 and 5, so removing the CACHED rotation leaves a residue of
       our own last write inside the "animation" we solve against - and the
       roll anchor references exactly those recovered points. */

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
    logf_("  bridge pad: seen 0x%08X  weapon pad bit 0x%08X  weapon presses %ld"
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
            logf_("  bridge move probe: work->pad 0x%llX  our record 0x%llX (%s)  pad data.flag 0x%08lX%s"
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
              "    weapon press is index %d into pressure[12]"
              "  (outside it %ld times)\r\n",
              bs.fire_wrote_press, bs.fire_wrote_status,
              bs.fire_wrote_release, bs.fire_wrote_pressure,
              bs.fire_pressure_kept, bs.fire_yielded,
              bs.fire_press_index, bs.fire_no_index);

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
    long last_activity = -1;
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
    if(source!=SRC_XR || !active_arm_pos.absolute_aim)
        active_arm_pos.hand_zero=arm_hand_zero_total(active_arm_pos.hand_zero,
            input_secondary_presses(arm_tracked_xr_hand(active_arm_track)));
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
    InterlockedExchange(&g_link_probe_requested,xc.second_camera_probe||xc.second_eye);
    InterlockedExchange(&g_second_eye,xc.second_eye);
    render_link_reset();
    InterlockedExchange(&g_stereo_test, xc.stereo_test);
    InterlockedExchange(&g_stereo_proj_x_sign, xc.stereo_proj_x_sign);
    InterlockedExchange(&g_stereo_eye_x_sign, xc.stereo_eye_x_sign);
    dg_ui2d_configure(xc.ui2d, xc.ui2d_scale_mils, xc.ui2d_conv_e5, xc.ui2d_sign, xc.ui2d_vs, (unsigned)xc.ui2d_vs_count);
    InterlockedExchange(&g_eye_truth_mode, xc.eye_truth);
    InterlockedExchange(&g_draw_skip_mode, xc.draw_skip);
    InterlockedExchange(&g_feedback_skip, xc.feedback_skip);
    InterlockedExchange(&g_observer_cfg, xc.observer);
    InterlockedExchange(&g_observer_mode, xc.observer_mode);
    dg_ui2d_hold(xc.ui2d_hold);
    dg_radar_configure(xc.radar_mode != 0, xc.radar_hud == 0);
    dg_bridge_native_hud_configure(xc.radar_mode != 0 && xc.radar_hud == 0);
    InterlockedExchange(&g_current_eye, DG_EYE_LEFT);
    handoff_write(0, DG_EYE_MONO, NULL, NULL, NULL, NULL);
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

    InterlockedExchange(&g_render_link_active,(g_link_requested||g_link_probe_requested)&&source==SRC_XR&&
        g_phase_stage_addr&&g_phase_consume_addr);
    logf_("  render link: requested %ld anchors %s (buffer-linked pose/FOV; no guessed fallback)\r\n",
        g_link_requested,g_phase_stage_addr?"verified":"UNAVAILABLE");
    logf_("  second camera observer: %ld capture_phase_fix %ld active %ld\r\n",g_link_probe_requested,g_link_requested,g_render_link_active);
    if(source==SRC_XR)hand_profile_worker();
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
        if(source==SRC_XR) {   /* Home menu SECOND EYE switched: (dis)arm the link */
            LONG want=g_xrcfg.second_camera_probe||second_eye_wanted();
            if(want!=InterlockedCompareExchange(&g_link_probe_requested,0,0)) {
                InterlockedExchange(&g_link_probe_requested,want);
                InterlockedExchange(&g_render_link_active,(g_link_requested||want)&&
                    g_phase_stage_addr&&g_phase_consume_addr);
                render_link_reset();arm_all(1);
                logf_("  second eye: %s by Home menu, render link active %ld\r\n",
                    second_eye_wanted()?"on":"off",g_render_link_active);
            }
        }
        if(elapsed%10==0&&(g_link_requested||g_link_probe_requested))
            logf_("  render link: active %ld stage %ld consumer %ld linked-attempts %ld refused %ld contention %ld register-mismatch %ld rearmed %ld\r\n",
                g_render_link_active,g_link_stage_hits,g_link_consumer_hits,g_link_accepted,
                g_link_refused,g_link_contention,g_link_register_mismatch,g_link_rearmed);
        if(elapsed%10==0&&second_eye_wanted())
            logf_("  second eye: held-eye presents %ld (native to the slot opposite its camera eye)\r\n",
                g_second_eye_swapped);
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
            if (session_timer_expired(seconds, elapsed)) break;
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
            char ui[1800]; dg_ui2d_stats(ui, sizeof ui); logf_("  %s\r\n", ui);
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
                long activity;
                dg_bridge_stats(&bs);

                activity = dg_dev_activity(bs.fire_drawn, bs.fire_released, bs.fire_aborted,
                                           bs.left_support_gate[6], bs.left_swing_applied);

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
                    activity != last_activity ||
                    bs.seen_pad_status != last_pad_seen) {
                    log_bridge_summary("running");
                    last_transitions = bs.fps_transitions;
                    last_arm_flag = bs.arm_flag;
                    last_pad_seen = bs.seen_pad_status;
                    last_body_flag = bs.body_flag;
                    last_arm_visible = bs.arm_visible_ticks;
                    last_cam_on = bs.arm_camera_on;
                    last_activity = activity;
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
            if(source!=SRC_XR || !g_arm_absolute_aim)
                ap2.hand_zero=arm_hand_zero_total(ap2.hand_zero,
                    input_secondary_presses(arm_tracked_xr_hand(arm_track2)));
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
            if(source==SRC_XR)hand_profile_worker();
            if (arm_track2 != active_arm_track || arm_hand2 != active_arm_hand ||
                memcmp(arm_qmap2, active_arm_qmap,
                       sizeof active_arm_qmap) != 0 ||

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

            if (source == SRC_XR) {
                if(memcmp(&g_xrcfg,&xc2,sizeof xc2)!=0)render_link_reset();
                g_xrcfg = xc2;
                InterlockedExchange(&g_stereo, xc2.stereo != 0);
                InterlockedExchange(&g_scene_blur_enabled, xc2.scene_blur_fix);
                LONG prior_link=InterlockedExchange(&g_link_requested,xc2.stereo_phase_fix);
                InterlockedExchange(&g_second_eye,xc2.second_eye);
                LONG want_probe=xc2.second_camera_probe||second_eye_wanted();
                LONG prior_probe=InterlockedExchange(&g_link_probe_requested,want_probe);
                if(prior_link!=xc2.stereo_phase_fix || prior_probe!=want_probe) {
                    InterlockedExchange(&g_render_link_active,(g_link_requested||g_link_probe_requested)&&g_source==SRC_XR&&
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
                InterlockedExchange(&g_observer_cfg, xc2.observer);
                InterlockedExchange(&g_observer_mode, xc2.observer_mode);
                dg_ui2d_hold(xc2.ui2d_hold);
                dg_radar_configure(xc2.radar_mode != 0, xc2.radar_hud == 0);
                dg_bridge_native_hud_configure(xc2.radar_mode != 0 && xc2.radar_hud == 0);
                dg_xr_configure(&xc2);
            }
        }
        if (session_timer_expired(seconds, elapsed)) break;
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

    if(source==SRC_XR)hand_profile_worker(); /* flush a just-accepted override */

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
    dg_xr_menu_pointer_hand(front_end && mode == DG_MENU_MODE_WRITE &&
        g_source == SRC_XR && !script_menu_blocked() ?
        (InterlockedCompareExchange(&g_menu_hand_right,0,0) ? 2 : 1) : 0);

    if (script_menu_blocked()) {
        DG_BRIDGE_MENU empty;
        dg_ui2d_gameplay(0);
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

    if (InterlockedCompareExchange(&g_menu_sweep_on, 0, 0)) {

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

    {
        static int held,owner,b_swallow;
        int use=front_end && mode==DG_MENU_MODE_WRITE && g_source==SRC_XR;
        int options=use && native_options_open(), hand=g_menu_hand_right?2:1;
        int target,down,i,listopen; unsigned values=0;
        DG_NATIVE_LIST nl;
        listopen=use && !options && native_list_open(&nl);   /* main menu or LOAD list: no bar, point at the rows */
        if(!options)laser_page=0;
        if(!options || laser_page){InterlockedExchange(&g_vr_row_show,0);InterlockedExchange(&g_vr_row_sel,0);}
        for(i=0;i<LASER_ROWS;i++)values|=(unsigned)(laser_values[i]&3)<<(2*i);
        dg_xr_laser_panel(options?1:listopen?2:0,laser_page,values);
        target=use?dg_xr_laser_target(options?1:listopen?2:0,laser_page,hand):-1;
        down=in.input_ok && in.trigger>=in.trigger_click;
        if(!down)owner=0;
        if(down && !held && target>=0)owner=target+1;

        if(b_swallow){ if(in.button_b)in.button_b=0; else b_swallow=0; }
        if(laser_page || owner){
            in.trigger=0;
            if(laser_page){
                in.x=in.y=0;in.button_a=0;
                if(in.button_b){laser_page=0;b_swallow=1;}
                in.button_b=0;
            }
            if(down && !held && owner){
                switch(owner-1){
                case LASER_UP:in.y=1;break;
                case LASER_DOWN:in.y=-1;break;
                case LASER_LEFT:in.x=-1;break;
                case LASER_RIGHT:in.x=1;break;
                case LASER_OK:in.button_a=1;break;
                case LASER_BACK:in.button_b=1;break;
                case LASER_SETTINGS:laser_page=options;break;
                case LASER_DONE:laser_page=0;break;
                default:if(laser_page)laser_change(owner-1-LASER_FIRST_ROW);break;
                }
            }
        }

        if(options && !laser_page) {
            static LASER_NATIVE_NAV nav={-1,-1,-1,-1,0,0};
            static int vr_sel,prev_y,prev_a;
            int cur=-1,ready=0,want=-1,ok=0,st,click=down && !held,open=0;
            if(target>=LASER_NATIVE_ROW0 && target<LASER_NATIVE_ROW0+LASER_NATIVE_ROWS)want=target-LASER_NATIVE_ROW0;
            native_options_cursor(&cur,&ready);
            if(target==LASER_NATIVE_VR && !InterlockedCompareExchange(&g_vr_row_drawn,0,0))target=-1;
            /* stick/A: EXIT -> DOWN selects VR SETTINGS, OK there opens the VR page (laser_vr_row_stick) */
            {

                int y0=laser_stick_dir(in.x,in.y,in.deadzone),yy=y0,aa=in.button_a?1:0;
                int drawn=InterlockedCompareExchange(&g_vr_row_drawn,0,0)!=0;
                vr_sel=laser_vr_row_stick(vr_sel,cur,ready && drawn,&yy,&aa,&prev_y,&prev_a,&open);
                if(y0 && !yy){in.x=0;in.y=0;}
                in.button_a=aa; }
            if(open){laser_page=1;InterlockedIncrement(&g_laser_native_vr);}
            InterlockedExchange(&g_vr_row_show,1);
            InterlockedExchange(&g_vr_row_sel,vr_sel || target==LASER_NATIVE_VR);
            if(target>=0)in.trigger=0;   /* the laser is on the menu: the trigger is ours */
            if(click && target==LASER_NATIVE_VR){laser_page=1;vr_sel=0;InterlockedIncrement(&g_laser_native_vr);}
            st=laser_native_nav(&nav,want,cur,ready,click && want>=0,GetTickCount64(),&ok);
            if(st){in.y=st;InterlockedIncrement(&g_laser_native_pulses);}
            if(ok){in.button_a=1;InterlockedIncrement(&g_laser_native_ok);}
        }

        {
            static LASER_NATIVE_NAV lnav={-1,-1,-1,-1,0,0,0,0};
            static int lkind;
            if(!listopen)lkind=NS_NONE;
            else {
                double u,v;
                int want=-1,ok=0,st,click=down && !held;
                if(nl.kind!=lkind){laser_native_nav_reset(&lnav);lkind=nl.kind;}
                lnav.rows=nl.n;lnav.wrap=nl.wrap;
                if(dg_xr_laser_uv(&u,&v))want=laser_native_list_hit(&nl,u,v);
                if(want>=0)in.trigger=0;   /* the laser is on a row: the trigger is ours */
                st=laser_native_nav(&lnav,want,nl.cur,nl.ready,click && want>=0,GetTickCount64(),&ok);
                if(st){in.y=st;InterlockedIncrement(&g_laser_native_pulses);}
                if(ok){in.button_a=1;InterlockedIncrement(&g_laser_native_ok);}
            }
        }
        if(use)menu_native_probe();
        held=down;
    }
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

#include "dg_chanl_probe.inl"
#include "dg_xr_sim.inl"
static void on_present_body(IDXGISwapChain *sc) {
    chanl_probe_present();
    chanl1_present();        /* vr_chanl1 marker key and its log lines */
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
        dg_ui2d_gameplay(0);
        memset(&empty, 0, sizeof empty);
        dg_bridge_menu_now(&empty);
        InterlockedIncrement(&g_present_frame);
        return;
    }

    dg_bridge_screen_seam_now();

    start_command();
    screen = dg_bridge_theater_verdict();
    {   /* Fission Mailed flanks, for the log. */
        static int fake_logged;
        int fake=(screen & DG_THEATER_V_FAKE)!=0;
        if (fake != fake_logged) {
            fake_logged = fake;
            logf_("  fission mailed: %s (actor frames %ld)\r\n", fake ? "ENTER theater" : "EXIT theater",
                  dg_bridge_fake_gameover_frames());
        }
    }

    if (g_source == SRC_SCRIPT) {
        LONG camera_now = InterlockedCompareExchange(&g_traps, 0, 0);
        have = camera_now != g_script_camera_seen;
        g_script_camera_seen = camera_now;
        if (InterlockedCompareExchange(&g_script_menu_active, 0, 0))
            have = 0; /* Incidental title/intro camera visits are not gameplay. */
    }
    if (have || dg_bridge_radial_context_held() || (screen & DG_THEATER_V_FAKE)) {
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

    {
        DG_STEREO_PRESENT p;
        memset(&p,0,sizeof(p));
        p.present=(unsigned)g_present_frame;p.camera_updates=(unsigned)g_traps;
        p.camera_silent=(unsigned)g_flat_silent;p.armed=armed;p.stereo=stereo;
        p.screen=screen;p.flat=flat;p.handoff_valid=have;p.linked_valid=linked_have;
        p.eye=eye;p.eye_truth_mode=(int)g_eye_truth_mode;p.draw_skip_mode=(int)g_draw_skip_mode;
        p.frame_sign=dg_ui2d_last_frame_sign();
        p.final_source=dg_ui2d_measure_source();
        dg_xr_measure_present(&p);
        dg_xr_observer_present(p.present);
        {
            char dir[MAX_PATH]={0};unsigned mark=0;
            int recording=dg_xr_measure_control(dir,sizeof(dir),&mark);
            dg_blit_boundary_control(dir,recording,mark,p.present,p.eye);
        }
        {
            static unsigned last_request;
            unsigned request=dg_xr_measure_trace_request();
            if(request && request!=last_request) {
                last_request=request;
                dg_ui2d_trace(12,g_bb_src_prev,g_bb_src_prev2);
                logf_("  stereo final path: trace request=%u armed after Present=%u tick=%llu (next 12 frames)\r\n",request,p.present,GetTickCount64());
            }
        }
    }
    /* U2. `flat` IS the front-end condition - a title or load screen is an
       image with no game camera behind it, which is exactly what the block
       above just established. Called every Present, including the ones where
       it is false, so the module sees the window close. Reads only. */
    {
        LONG camera_now=InterlockedCompareExchange(&g_traps,0,0);
        /* Fission Mailed is shown like a cutscene, but the game still plays. */
        int ctl_screen=screen & ~DG_THEATER_V_FAKE;
        int raw_gameplay=armed && !flat && !ctl_screen &&
            !InterlockedCompareExchange(&g_script_menu_active,0,0) &&
            dg_bridge_controller_gameplay_now();
        dg_ui2d_gameplay(raw_gameplay && have && stereo && g_source==SRC_XR);
        int raw_special=(armed && !flat && !ctl_screen &&
            !InterlockedCompareExchange(&g_script_menu_active,0,0)) ?
            dg_bridge_controller_special_now():0;
        int raw_radial=armed && (dg_bridge_radial_context_held() || (!flat && !ctl_screen)) &&
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
            (dg_bridge_radial_context_held() && raw_radial && !g_buttons_start_block) ||
            controller_camera_context(camera_now,raw_radial,GetTickCount()));
        if (g_source==SRC_XR) {
            int toggle=dg_xr_controller_toggle_take();
            if (g_buttons_gameplay && toggle && !InterlockedCompareExchange(&g_mod_input_capture,0,0)) dg_bridge_request_toggle();
        } else script_primary_command();
    }
    menu_seam(menu_frontend_allowed(flat,dg_bridge_codec_input_now(),
        dg_bridge_menu_gameover_now(),InterlockedCompareExchange(&g_mod_input_capture,0,0)));
    laser_vr_row_present(sc);

    /* Desktop scripts have no XR device, swapchains or layer submission.
       Keep Present's game-frame identity while avoiding every capture path. */
    if (g_source == SRC_SCRIPT) {
        InterlockedIncrement(&g_present_frame);
        InterlockedExchange(&g_current_eye, DG_EYE_LEFT);
        return;
    }

    auto_recenter_present(armed && !flat && !screen);
    if(!armed || flat || screen)dg_radar_view(0);
    dg_xr_screen(screen);

    dg_ui2d_feedback(InterlockedCompareExchange(&g_feedback_skip, 0, 0) && stereo && armed && have && !screen && g_source == SRC_XR);
    if(screen||!armed||!stereo||g_source!=SRC_XR)render_link_reset();
    if (screen) {
        const MAT *pers = (const MAT *)(g_chan0 + O_PERS);
        DG_PROJ_FOV live;
        if (!g_chan0 || !projlike(pers)) {
            if (flat) {

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
        {

            int native_slot = handoff.eye, b_slot = -1;
            const DG_XR_RAW_POSE *native_raw = &handoff.raw, *b_raw = NULL;
            const DG_PROJ_FOV *native_fov = &handoff.fov, *b_fov = NULL;
            if (dg_second_eye_enabled() && handoff.other_valid &&
                (handoff.eye == DG_EYE_LEFT || handoff.eye == DG_EYE_RIGHT)) {
                int other = handoff.eye == DG_EYE_RIGHT ? DG_EYE_LEFT : DG_EYE_RIGHT;
                if (dg_render_second_camera_eye() == handoff.eye) {   /* eye held */
                    native_slot = other; native_raw = &handoff.other_raw; native_fov = &handoff.other_fov;
                    b_slot = handoff.eye; b_raw = &handoff.raw; b_fov = &handoff.fov;
                    InterlockedIncrement(&g_second_eye_swapped);
                } else {
                    b_slot = other; b_raw = &handoff.other_raw; b_fov = &handoff.other_fov;
                }
            }
            dg_xr_capture_measured(sc, native_slot, native_raw, native_fov, &handoff.near_meta);
            if (b_slot >= 0 && dg_second_eye_publish(b_slot, b_raw, b_fov)) {
                InterlockedExchange(&g_second_eye_frame, InterlockedCompareExchange(&g_present_frame, 0, 0));
                extern LONG64 dg_bridge_psg_capture_request(void);
                extern void dg_psg_dump_pair(unsigned,unsigned,unsigned,unsigned);
                static LONG64 previous;
                static unsigned phase;
                LONG64 request = dg_bridge_psg_capture_request();
                unsigned age = GetTickCount() - (DWORD)((ULONG64)request >> 32);
                if (request != previous) { previous = request; phase = 0; }
                if (request && age < 750 && phase < 2 && (!phase || age >= 100)) {
                    dg_psg_dump_pair((unsigned)request, g_present_frame, age, phase);
                    ++phase;
                }
            }

        }
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

present_done:
    {   /* Correct aspect on every desktop gameplay frame, after XR capture. */
        static int observer_logged = -1, observer_mode_logged = -1;
        int on = observer_on();
        if (on != observer_logged || observer_mode_logged != observer_mode()) {
            observer_logged = on; observer_mode_logged = observer_mode();
            logf_("  desktop observer: %s\r\n", !on ? "off" : observer_mode()==2 ? "both eyes (16:9, right dominant)" : observer_mode()==1 ? "right eye (16:9)" : "left eye (16:9)");
        }
        /* Every eye needs aspect correction, including native right frames. */
        if (on && armed && stereo && !screen)
            dg_xr_observer_fill(sc, observer_mode());
    }
    InterlockedIncrement(&g_present_frame);
    /* Second eye: the native eye stays put and B fills the other one, so each
       eye always shows the same kind of image (no native/replay flicker per
       eye). Only while B was stored in the last two Presents - if the replay
       stops, the eyes alternate again instead of one freezing. */
    if (dg_second_eye_enabled() &&
        (LONG)(InterlockedCompareExchange(&g_present_frame, 0, 0) -
               InterlockedCompareExchange(&g_second_eye_frame, 0, 0)) <= 2)
        keep_eye = 1;
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

    dg_policy_setup(dir, logf_);

    g_base      = (ULONG64)GetModuleHandleW(NULL);
    g_hook_addr = g_base + HOOK_RVA;
    g_chan0     = g_base + DG_CHANLS_RVA;

    crash_trace_start();
    logf_("\r\n=== dg_hook ready ===\r\n"
          "  module base   %016llX\r\n"
          "  hook          %016llX (rva 0x%llX)\r\n"
          "  render channels[0]  %016llX\r\n"
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

    if (present_on && !script_boot && marker_present(xmarker)) {
        DG_XR_CONFIG xc;
        memset(&xc, 0, sizeof(xc));
        xc.yaw_sign = xc.pitch_sign = xc.roll_sign = 1.0;
        xc.x_sign = xc.y_sign = xc.z_sign = 1.0;
        xc.scale = 1000.0;              /* plan 2.9: 1 unit = 1 mm */
        xc.positional = 1;
        xc.stereo = 0;
        xc.stereo_test = 0;
        dg_xr_set_sim_mode(xr_sim_env(marker, logf_));
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

BOOL WINAPI DllMain(HINSTANCE inst, DWORD reason, LPVOID reserved) {
    if (reason == DLL_PROCESS_ATTACH) {
        DisableThreadLibraryCalls(inst);
        CloseHandle(CreateThread(NULL, 0, worker, NULL, 0, NULL));
    }

    if (reason == DLL_PROCESS_DETACH) {
        static LONG once;

        dg_draw_trial_detach();
        /* Tells a normal process exit apart from a kill: a kill never gets here. */
        if (g_logpath[0]) logf_("  process %s: present %ld\r\n", reserved ? "exit" : "library unload",
                                (long)g_present_frame);
        if (InterlockedExchange(&once, 1) == 0 &&
            InterlockedCompareExchange(&g_rec_on, 0, 0))
            rec_dump(reserved ? "process exit" : "library unload");
    }
    return TRUE;
}
