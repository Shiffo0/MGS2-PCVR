#include "dg_build_profile.h"


#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>

#include "dg_xr.h"
static PVOID volatile g_capture_observer;
void dg_xr_set_capture_observer(DG_XR_CAPTURE_OBSERVER observer) {
    InterlockedExchangePointer(&g_capture_observer,(PVOID)observer);
}
#ifdef DG_XR_NO_RUNTIME
void dg_xr_m9_feedback(unsigned events) {(void)events;}
#endif
#include "dg_proj.h"
/* Shared bridge types only. Physical button events go through the bounded
   context mailbox above; this runtime thread does not enqueue FPS directly. */
#include "dg_bridge.h"
#include "dg_model_arm.h"

#ifndef DG_XR_NO_RUNTIME
#include <d3d11.h>
#include <d3d11_4.h>                    /* ID3D11Multithread - see g_mt */
#include <dxgi.h>
#define XR_USE_PLATFORM_WIN32
#define XR_USE_GRAPHICS_API_D3D11
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#endif

#define PI 3.14159265358979323846

/* =========================================================== the math === */

void dg_xr_quat_to_mat(double x, double y, double z, double w, MAT *o) {
    double n = sqrt(x*x + y*y + z*z + w*w);
    double xx, yy, zz, xy, xz, yz, wx, wy, wz;
    if (n < 1e-12) { n = 1.0; x = y = z = 0.0; w = 1.0; }
    x /= n; y /= n; z /= n; w /= n;

    xx = x*x; yy = y*y; zz = z*z;
    xy = x*y; xz = x*z; yz = y*z;
    wx = w*x; wy = w*y; wz = w*z;

    memset(o, 0, sizeof(*o));
    o->m[0][0] = (float)(1.0 - 2.0*(yy + zz));
    o->m[0][1] = (float)(2.0*(xy - wz));
    o->m[0][2] = (float)(2.0*(xz + wy));
    o->m[1][0] = (float)(2.0*(xy + wz));
    o->m[1][1] = (float)(1.0 - 2.0*(xx + zz));
    o->m[1][2] = (float)(2.0*(yz - wx));
    o->m[2][0] = (float)(2.0*(xz - wy));
    o->m[2][1] = (float)(2.0*(yz + wx));
    o->m[2][2] = (float)(1.0 - 2.0*(xx + yy));
    o->m[3][3] = 1.0f;
}

/* Inverse of build_delta()'s composition, which is R = Ry * Rx * Rz for row
   vectors. Working that product out by hand gives

       R[1][2] =  sin(pitch)
       R[0][2] = -sin(yaw)  * cos(pitch)      R[2][2] = cos(yaw) * cos(pitch)
       R[1][0] = -cos(pitch)* sin(roll)       R[1][1] = cos(pitch)* cos(roll)

   so each angle comes out of a single atan2 with no ambiguity, except at
   pitch = +-90 where cos(pitch) vanishes and yaw and roll stop being
   separable. There the convention is to give everything to yaw. */
void dg_xr_mat_to_ypr(const MAT *r, double *yaw, double *pitch, double *roll) {
    double sp = (double)r->m[1][2];
    if (sp >  1.0) sp =  1.0;
    if (sp < -1.0) sp = -1.0;
    *pitch = asin(sp);

    if (fabs(sp) < 0.999999) {
        *yaw  = atan2(-(double)r->m[0][2], (double)r->m[2][2]);
        *roll = atan2(-(double)r->m[1][0], (double)r->m[1][1]);
    } else {
        *yaw  = atan2((double)r->m[2][0], (double)r->m[0][0]);
        *roll = 0.0;
    }
}

void dg_xr_head_to_pose(double qx, double qy, double qz, double qw,
                        double px, double py, double pz,
                        const DG_XR_CONFIG *cfg, DG_XR_POSE *out) {
    MAT r;
    double yaw, pitch, roll;
    double tc[3];

    /* The relabelling, measured rather than derived - see the file header for
       why that sentence is the important one. Both the handedness fix and the
       coordinate/camera inversion are in this one line. */
    dg_xr_quat_to_mat(qx, -qy, -qz, qw, &r);
    dg_xr_mat_to_ypr(&r, &yaw, &pitch, &roll);

    out->yaw   = yaw   * cfg->yaw_sign;
    out->pitch = pitch * cfg->pitch_sign;
    out->roll  = roll  * cfg->roll_sign;

    if (!cfg->positional) {
        out->tx = out->ty = out->tz = 0.0;
        return;
    }

    /* Head displacement in MGS2 view axes: same Z mirror, metres to game units. */
    tc[0] = px * cfg->scale * cfg->x_sign;
    tc[1] = py * cfg->scale * cfg->y_sign;
    tc[2] = -pz * cfg->scale * cfg->z_sign;

    /* D maps OLD view coords to NEW view coords: v' = v * D. If the camera
       itself moves by t_c and rotates by R (both in the old view frame), a
       fixed point's new coords are (v - t_c) * R, so
             D = T(-t_c) * R,  whose translation row is  -t_c * R.
       build_delta() constructs D = R * T(t) with translation row t, so what it
       must be handed is -t_c * R and NOT the head displacement itself. The two
       agree only when the head is not also rotating, which during head
       tracking is never. */
    {
        /* Rebuilt from the SIGN-CORRECTED angles rather than reusing r above:
           if a sign flag flips the rotation, the translation has to be
           contracted against the rotation that will actually be applied. */
        MAT rot;
        double ca = cos(out->yaw),   sa = sin(out->yaw);
        double cb = cos(out->pitch), sb = sin(out->pitch);
        double cc = cos(out->roll),  sc = sin(out->roll);

        memset(&rot, 0, sizeof(rot));
        rot.m[0][0] = (float)(ca*cc - sa*sb*sc);
        rot.m[0][1] = (float)(ca*sc + sa*sb*cc);
        rot.m[0][2] = (float)(-sa*cb);
        rot.m[1][0] = (float)(-cb*sc);
        rot.m[1][1] = (float)(cb*cc);
        rot.m[1][2] = (float)(sb);
        rot.m[2][0] = (float)(sa*cc + ca*sb*sc);
        rot.m[2][1] = (float)(sa*sc - ca*sb*cc);
        rot.m[2][2] = (float)(ca*cb);
        rot.m[3][3] = 1.0f;

        out->tx = -(tc[0]*rot.m[0][0] + tc[1]*rot.m[1][0] + tc[2]*rot.m[2][0]);
        out->ty = -(tc[0]*rot.m[0][1] + tc[1]*rot.m[1][1] + tc[2]*rot.m[2][1]);
        out->tz = -(tc[0]*rot.m[0][2] + tc[1]*rot.m[1][2] + tc[2]*rot.m[2][2]);
    }
}

/* q1 * q2, Hamilton convention, matching OpenXR's. */
static void qmul(const double *a, const double *b, double *o) {
    double x = a[3]*b[0] + a[0]*b[3] + a[1]*b[2] - a[2]*b[1];
    double y = a[3]*b[1] - a[0]*b[2] + a[1]*b[3] + a[2]*b[0];
    double z = a[3]*b[2] + a[0]*b[1] - a[1]*b[0] + a[2]*b[3];
    double w = a[3]*b[3] - a[0]*b[0] - a[1]*b[1] - a[2]*b[2];
    o[0] = x; o[1] = y; o[2] = z; o[3] = w;
}

/* v rotated by unit q, via the usual two cross products. */
static void qrot(const double *q, const double *v, double *o) {
    double t[3], c[3];
    t[0] = 2.0 * (q[1]*v[2] - q[2]*v[1]);
    t[1] = 2.0 * (q[2]*v[0] - q[0]*v[2]);
    t[2] = 2.0 * (q[0]*v[1] - q[1]*v[0]);
    c[0] = q[1]*t[2] - q[2]*t[1];
    c[1] = q[2]*t[0] - q[0]*t[2];
    c[2] = q[0]*t[1] - q[1]*t[0];
    o[0] = v[0] + q[3]*t[0] + c[0];
    o[1] = v[1] + q[3]*t[1] + c[1];
    o[2] = v[2] + q[3]*t[2] + c[2];
}

/* Counted in capture_reference, the one place a seated origin is ever
   decided. The runtime half owns the increment, so like every line in that
   half it cannot be desk-tested; what makes it diagnosable anyway is that the
   heartbeat prints both this count and the arm's own freeze count, and the two
   are supposed to track each other. */
static volatile LONG g_recenter_count;

long dg_xr_recenter_count(void) {
    return (long)InterlockedCompareExchange(&g_recenter_count, 0, 0);
}

unsigned dg_xr_button_gate_step(DG_XR_BUTTON_GATE *g, unsigned context,
    int allowed, unsigned a, unsigned y, uint32_t now_ms)
{
    unsigned out = 0;
    if (!g) return 0;
    if (!allowed || g->context != context ||
        (g->have_sample && (uint32_t)(now_ms-g->last_ms)>100u)) {
        memset(g, 0, sizeof(*g));
        g->context = context;
    }
    if (!allowed) return 0;
    g->last_ms=now_ms; g->have_sample=1;
    if (!a) g->a_armed = 1;
    if (a && !g->a_down && g->a_armed) out |= DG_XR_BUTTON_TOGGLE;
    g->a_down = a ? 1 : 0;
    if (!y) {
        g->y_armed = 1;
        g->y_down = g->y_fired = 0;
    } else if (g->y_armed) {
        if (!g->y_down) { g->y_down = 1; g->y_since = now_ms; }
        if (!g->y_fired && (uint32_t)(now_ms-g->y_since) >= 1000u) {
            g->y_fired = 1;
            out |= DG_XR_BUTTON_RECENTER;
        }
    }
    return out;
}

/* One atomic publication: timestamp in the high word, epoch/allow in low.
   Epoch changes even if XR misses the entire intervening menu interval. */
static volatile LONG64 g_controller_context;
static volatile LONG64 g_controller_toggle;
static unsigned g_controller_recenter_context;
static uint32_t g_controller_recenter_ms;
static DG_XR_BUTTON_GATE g_controller_a_gate, g_controller_y_gate;

static void controller_context_at(int gameplay, uint32_t now_ms)
{
    uint64_t old = (uint64_t)InterlockedCompareExchange64(&g_controller_context,0,0);
    uint32_t tag = (uint32_t)old;
    if ((tag & 1u) != (gameplay ? 1u : 0u)) tag = (tag + 2u) & ~1u;
    tag = (tag & ~1u) | (gameplay ? 1u : 0u);
    InterlockedExchange64(&g_controller_context,(LONG64)(((uint64_t)now_ms<<32)|tag));
    if (!gameplay) InterlockedExchange64(&g_controller_toggle,0);
}

void dg_xr_controller_context(int gameplay)
{
    controller_context_at(gameplay,(uint32_t)GetTickCount());
}

static unsigned controller_context_fresh(uint32_t now_ms)
{
    uint64_t value = (uint64_t)InterlockedCompareExchange64(&g_controller_context,0,0);
    uint32_t tag=(uint32_t)value;
    return (tag&1u) && (uint32_t)(now_ms-(uint32_t)(value>>32)) <= 100u ? tag : 0;
}

static unsigned controller_buttons_at(int active, unsigned a, unsigned y, uint32_t now_ms)
{
    unsigned tag=controller_context_fresh(now_ms);
    unsigned events=dg_xr_button_gate_step(&g_controller_a_gate,tag,(active&1)&&tag,a,0,now_ms);
    events |= dg_xr_button_gate_step(&g_controller_y_gate,tag,(active&2)&&tag,0,y,now_ms);
    if (!(active&1) || !tag) {
        InterlockedExchange64(&g_controller_toggle,0);
    }
    if (!(active&2) || !tag) {
        g_controller_recenter_context=0;
    }
    if (events & DG_XR_BUTTON_TOGGLE)
        InterlockedExchange64(&g_controller_toggle,(LONG64)(((uint64_t)now_ms<<32)|tag));
    if (events & DG_XR_BUTTON_RECENTER) {
        g_controller_recenter_context=tag; g_controller_recenter_ms=now_ms;
    }
    return events;
}

static int controller_toggle_take_at(uint32_t now_ms)
{
    uint64_t pending=(uint64_t)InterlockedExchange64(&g_controller_toggle,0);
    return pending && (uint32_t)(now_ms-(uint32_t)(pending>>32))<=100u &&
           (uint32_t)pending==controller_context_fresh(now_ms);
}

int dg_xr_controller_toggle_take(void)
{ return controller_toggle_take_at((uint32_t)GetTickCount()); }

static int controller_recenter_current(uint32_t now_ms)
{
    return g_controller_recenter_context &&
           (uint32_t)(now_ms-g_controller_recenter_ms)<=100u &&
           g_controller_recenter_context==controller_context_fresh(now_ms);
}

#ifdef DG_HOOK_TEST
unsigned dg_xr_test_controller_buttons(int active, unsigned a, unsigned y, uint32_t now_ms)
{ return controller_buttons_at(active?3:0,a,y,now_ms); }
unsigned dg_xr_test_controller_actions(unsigned valid, unsigned a, unsigned y, uint32_t now_ms)
{ return controller_buttons_at((int)valid,a,y,now_ms); }
void dg_xr_test_controller_context(int gameplay, uint32_t now_ms)
{ controller_context_at(gameplay,now_ms); }
int dg_xr_test_controller_toggle_take(uint32_t now_ms)
{ return controller_toggle_take_at(now_ms); }
int dg_xr_test_controller_recenter_current(uint32_t now_ms)
{ return controller_recenter_current(now_ms); }
#endif

#ifdef DG_HOOK_TEST
/* capture_reference lives in the runtime half and the desk build has none, so
   without this the arm's recentre-restarts-the-stream contract could only be
   argued about. Same precedent as dg_xr_test_publish. */
void dg_xr_test_recenter(void) {
    InterlockedIncrement(&g_recenter_count);
}
#endif

void dg_xr_quat_yaw_only(const double q[4], double out[4]) {
    /* The twist of a swing/twist split about +Y is the Y and W components on
       their own, renormalised. When both are ~0 the head is rotated a half
       turn about some horizontal axis and the twist genuinely has no value;
       the identity is the only answer that cannot invent a rotation. */
    double n = sqrt(q[1] * q[1] + q[3] * q[3]);
    if (!(n > 1.0e-9)) {
        out[0] = out[1] = out[2] = 0.0; out[3] = 1.0;
        return;
    }
    out[0] = 0.0;
    out[1] = q[1] / n;
    out[2] = 0.0;
    out[3] = q[3] / n;
}

/* The theater quad's fixed pose. Yaw-only on purpose and by contract: the
   current mono layer is HEAD-COUPLED (its pose is the live view pose) and the
   theater needs the exact opposite - a screen that stands still in the world
   while the compositor gives the head its freedom. So the head pose is used
   once, at anchor time: its yaw says which way "ahead" is, its position says
   where the eyes are, and pitch/roll are stripped so a player who entered a
   cutscene mid-glance still gets an upright screen at eye height.

   For a pure +Y yaw quaternion (0, s, 0, c): sin(a) = 2sc, cos(a) = 1-2s^2,
   and rotating OpenXR's forward (0,0,-1) by it lands on (-sin a, 0, -cos a).
   The quad's orientation IS the yaw quaternion, which leaves the quad's +Z
   facing back along that heading - toward the viewer, which is the side
   XrCompositionLayerQuad displays. Degenerate input follows
   dg_xr_quat_yaw_only's contract: identity heading, screen at LOCAL -Z,
   never an invented rotation. */
void dg_xr_screen_pose(const DG_XR_RAW_POSE *head, double dist_m,
                       DG_XR_RAW_POSE *out) {
    double q[4], yaw[4], sin_a, cos_a;
    q[0] = head->qx; q[1] = head->qy; q[2] = head->qz; q[3] = head->qw;
    dg_xr_quat_yaw_only(q, yaw);
    sin_a = 2.0 * yaw[1] * yaw[3];
    cos_a = 1.0 - 2.0 * yaw[1] * yaw[1];
    out->qx = yaw[0]; out->qy = yaw[1]; out->qz = yaw[2]; out->qw = yaw[3];
    out->px = head->px + dist_m * -sin_a;
    out->py = head->py;                 /* eye height rides the head's */
    out->pz = head->pz + dist_m * -cos_a;
}

/* One reference pose in, one relative pose out, and no modes: which FRAME the
   arm wants is expressed by which reference it hands in - the live head, the
   head's heading, or the seated origin's heading, all with the live head's
   position. That kept the mode selection where the policy is and left this a
   single transform. */
void dg_xr_pose_relative(const DG_XR_RAW_POSE *reference,
                         const DG_XR_RAW_POSE *pose,
                         DG_XR_REL_POSE *out) {
    double qi[4], qp[4], qr[4], dp[3], pr[3];
    qi[0] = -reference->qx; qi[1] = -reference->qy;
    qi[2] = -reference->qz; qi[3] = reference->qw;
    qp[0] = pose->qx; qp[1] = pose->qy;
    qp[2] = pose->qz; qp[3] = pose->qw;
    qmul(qi, qp, qr);
    dp[0] = pose->px - reference->px;
    dp[1] = pose->py - reference->py;
    dp[2] = pose->pz - reference->pz;
    qrot(qi, dp, pr);
    out->qx = qr[0]; out->qy = qr[1]; out->qz = qr[2]; out->qw = qr[3];
    out->px = pr[0]; out->py = pr[1]; out->pz = pr[2];
}

void dg_xr_pose_from_relative(const DG_XR_RAW_POSE *reference,
                              const DG_XR_REL_POSE *relative,
                              DG_XR_RAW_POSE *out) {
    double qref[4], qrel[4], qp[4], p[3], pr[3];
    qref[0] = reference->qx; qref[1] = reference->qy;
    qref[2] = reference->qz; qref[3] = reference->qw;
    qrel[0] = relative->qx; qrel[1] = relative->qy;
    qrel[2] = relative->qz; qrel[3] = relative->qw;
    qmul(qref, qrel, qp);
    p[0] = relative->px; p[1] = relative->py; p[2] = relative->pz;
    qrot(qref, p, pr);
    out->qx = qp[0]; out->qy = qp[1]; out->qz = qp[2]; out->qw = qp[3];
    out->px = reference->px + pr[0];
    out->py = reference->py + pr[1];
    out->pz = reference->pz + pr[2];
}

unsigned int dg_xr_trigger_hysteresis(float value, float deadzone,
                                      float fire, unsigned int was_pressed) {
    if (was_pressed) return value > deadzone;
    return value >= fire;
}

void dg_xr_hand_pose_sample(DG_XR_HAND_POSE *out, unsigned int hand,
                            unsigned int kind,
                            const DG_XR_RAW_POSE *raw_local,
                            const DG_XR_REL_POSE *reference_relative,
                            unsigned int position_valid,
                            unsigned int orientation_valid,
                            unsigned int position_tracked,
                            unsigned int orientation_tracked,
                            unsigned int active, uint64_t sample_seq,
                            int64_t xr_time) {
    int usable = active && position_valid && orientation_valid &&
                 position_tracked && orientation_tracked &&
                 raw_local && reference_relative;
    memset(out, 0, sizeof(*out));
    out->hand = hand;
    out->kind = kind;
    out->active = active != 0;
    out->sample_seq = sample_seq;
    out->xr_time = xr_time;
    if (!usable) return;
    out->raw_local = *raw_local;
    out->reference_relative = *reference_relative;
    out->position_valid = 1;
    out->orientation_valid = 1;
    out->tracked = 1;
}

/* ==================================================== pose publication === */

/* A seqlock. The reader is a VEH on the game's render thread, so it may not
   block, may not allocate and may not take a lock - but it must never see a
   half-written pose either, because a torn quaternion is a camera that jumps. */
static volatile LONG  g_seq;
static DG_XR_FRAME    g_pub;
static volatile LONG   g_pub_fov_valid;
static volatile LONG  g_pub_valid;
static volatile LONG64 g_pub_ms;

static void publish(const DG_XR_FRAME *frame, int valid, int fov_valid) {
    InterlockedIncrement(&g_seq);           /* odd: write in progress */
    MemoryBarrier();
    g_pub = *frame;
    g_pub_fov_valid = fov_valid;
    g_pub_valid = valid;
    g_pub_ms = (LONG64)GetTickCount64();
    MemoryBarrier();
    InterlockedIncrement(&g_seq);           /* even: settled */
}

#define POSE_STALE_MS 250                   /* ~15 frames at 60 Hz */
#define HAND_POSE_STALE_MS 100

static void age_hand_pose(DG_XR_HAND_POSE *pose, uint64_t age_ms) {
    pose->pose_age_ms = age_ms > UINT_MAX ? UINT_MAX : (unsigned int)age_ms;
    if (age_ms > HAND_POSE_STALE_MS) {
        memset(&pose->raw_local, 0, sizeof(pose->raw_local));
        memset(&pose->reference_relative, 0, sizeof(pose->reference_relative));
        pose->position_valid = 0;
        pose->orientation_valid = 0;
        pose->tracked = 0;
    }
}

static void age_hands(DG_XR_FRAME *frame, uint64_t age_ms) {
    age_hand_pose(&frame->left_hand.grip, age_ms);
    age_hand_pose(&frame->left_hand.aim, age_ms);
    age_hand_pose(&frame->right_hand.grip, age_ms);
    age_hand_pose(&frame->right_hand.aim, age_ms);
}

int dg_xr_get_frame(DG_XR_POSE *out, DG_PROJ_FOV *fov_out) {
    int attempt;
    for (attempt = 0; attempt < 8; attempt++) {
        LONG s0 = g_seq, s1;
        DG_XR_FRAME tmp;
        LONG valid, fov_valid;
        LONG64 ms;
        if (s0 & 1) continue;               /* writer mid-update */
        MemoryBarrier();
        tmp = g_pub;
        valid = g_pub_valid;
        fov_valid = g_pub_fov_valid;
        ms = g_pub_ms;
        MemoryBarrier();
        s1 = g_seq;
        if (s0 != s1) continue;             /* torn; try again */
        if (!valid) return 0;
        /* A pose that stopped arriving is worse than no pose: the camera would
           lock to wherever the head was when tracking died. */
        if ((LONG64)GetTickCount64() - ms > POSE_STALE_MS) return 0;
        *out = tmp.head;
        if (fov_out) {
            if (!fov_valid) return 1;
            *fov_out = tmp.union_fov;
            return 3;
        }
        return 1;
    }
    return 0;
}

int dg_xr_get(DG_XR_POSE *out) { return dg_xr_get_frame(out, NULL); }

int dg_xr_get_stereo(DG_XR_FRAME *out) {
    int attempt;
    for (attempt = 0; attempt < 8; attempt++) {
        LONG s0 = g_seq, s1;
        DG_XR_FRAME tmp;
        LONG valid, fov_valid;
        LONG64 ms;
        if (s0 & 1) continue;
        MemoryBarrier();
        tmp = g_pub;
        valid = g_pub_valid;
        fov_valid = g_pub_fov_valid;
        ms = g_pub_ms;
        MemoryBarrier();
        s1 = g_seq;
        if (s0 != s1) continue;
        if (!valid || (LONG64)GetTickCount64() - ms > POSE_STALE_MS) return 0;
        age_hands(&tmp, (uint64_t)((LONG64)GetTickCount64() - ms));
        if (out) *out = tmp;
        return (fov_valid ? 2 : 0) | 1;
    }
    return 0;
}

#ifdef DG_HOOK_TEST
void dg_xr_test_publish(const DG_XR_FRAME *frame) {
    publish(frame, 1, 1);
}
void dg_xr_test_publish_status(const DG_XR_FRAME *frame, int flags) {
    publish(frame, (flags & 1) != 0, (flags & 2) != 0);
}
#endif

int dg_xr_get_target_fov(DG_PROJ_FOV *out) {
    DG_XR_POSE pose;
    return (dg_xr_get_frame(&pose, out) & 2) != 0;
}

/* ========================================================= the runtime === */

static void (*g_log)(const char *fmt, ...);
static DG_XR_CONFIG   g_cfg;
static volatile LONG  g_trigger_deadzone_bits;
static volatile LONG  g_trigger_fire_bits;
static volatile LONG  g_run;
static volatile LONG  g_started;
static HANDLE         g_thread;
static volatile LONG  g_recenter_req;
static volatile LONG  g_recenter_cause;
static volatile LONG  g_st_frames, g_st_valid, g_st_invalid, g_st_errors;

enum { DG_XR_CAUSE_INITIAL = 0, DG_XR_CAUSE_MARKER = 1,
       DG_XR_CAUSE_HOTKEY = 2, DG_XR_CAUSE_CONTROLLER = 3 };

int dg_xr_should_capture(int have_ref, int explicit_req, int worn,
                         unsigned tracked_ms) {
    if (explicit_req) return 1;
    if (have_ref || !worn) return 0;
    return tracked_ms >= 500;
}

/* Indexed by DG_XR_HAND_LEFT/RIGHT. Written only by the input thread on a
   rising edge and read by anyone; a LONG so the read is a single instruction
   and the value can never be seen half-updated. */
static volatile LONG g_secondary_presses[2];
static volatile LONG g_secondary_ignored[2];
static volatile LONG64 g_menu_press_seq;

unsigned int dg_xr_menu_route(unsigned int was_down, unsigned int is_down,
                              int theater)
{
    return !was_down && is_down ? (theater ? 2u : 1u) : 0u;
}

unsigned int dg_xr_secondary_counts(unsigned int was_down, unsigned int is_down,
                                    unsigned int trigger_click)
{
    return !was_down && is_down && trigger_click;
}

unsigned long dg_xr_secondary_ignored(unsigned int hand) {
    int i = (hand == DG_XR_HAND_LEFT) ? 0
          : (hand == DG_XR_HAND_RIGHT) ? 1 : -1;
    if (i < 0) return 0;
    return (unsigned long)InterlockedCompareExchange(
        &g_secondary_ignored[i], 0, 0);
}

unsigned long dg_xr_secondary_presses(unsigned int hand) {
    /* The public DG_XR_HAND_* values are 1 and 2; the array is indexed by the
       runtime's own 0 and 1. Translated here rather than at the call site, so
       that a caller holding a DG_XR_HAND_* - which is the only thing it ever
       has - cannot get it wrong. Anything else reads zero. */
    int i = (hand == DG_XR_HAND_LEFT) ? 0
          : (hand == DG_XR_HAND_RIGHT) ? 1 : -1;
    if (i < 0) return 0;
    return (unsigned long)InterlockedCompareExchange(
        &g_secondary_presses[i], 0, 0);
}

uint64_t dg_xr_menu_press_seq(void) {
    return (uint64_t)InterlockedCompareExchange64(&g_menu_press_seq, 0, 0);
}

#ifdef DG_HOOK_TEST
/* The runtime half is what normally increments these, and the desk build has
   no runtime. Without this the LEFT/RIGHT to 0/1 translation above could only
   be argued about, not asserted: every accessor call would read zero and any
   wrong mapping would look exactly like a correct one. Same precedent as
   dg_xr_test_publish. */
void dg_xr_test_press_secondary(unsigned int hand) {
    if (hand == DG_XR_HAND_LEFT) InterlockedIncrement(&g_secondary_presses[0]);
    else if (hand == DG_XR_HAND_RIGHT)
        InterlockedIncrement(&g_secondary_presses[1]);
}
#endif

void dg_xr_recenter(void) {
    InterlockedExchange(&g_recenter_cause, DG_XR_CAUSE_HOTKEY);
    InterlockedExchange(&g_recenter_req, 1);
}

/* Set once the marker's recenter token has been adopted, so that only a CHANGE
   to it after startup counts as a request. */
static int g_cfg_seen;

void dg_xr_stats(long *frames, long *valid, long *invalid, long *errors) {
    if (frames)  *frames  = g_st_frames;
    if (valid)   *valid   = g_st_valid;
    if (invalid) *invalid = g_st_invalid;
    if (errors)  *errors  = g_st_errors;
}

/* The capture stores live in the runtime half of this file, and this function
   does not - it is shared, because the test build configures too. Declared here
   and defined once on each side of the split rather than reaching forward into
   state that only sometimes exists. */
static void drop_capture_stores(void);
void dg_xr_zoom_changed(void) {drop_capture_stores();}

void dg_xr_configure(const DG_XR_CONFIG *cfg) {
    DG_XR_CONFIG next = *cfg;
    float deadzone, fire;
    int prev = g_cfg.recenter_token;
    int first = !g_cfg_seen;
    int mode_changed = !first && (cfg->stereo != g_cfg.stereo ||
                                  cfg->stereo_test != g_cfg.stereo_test ||
                                  cfg->stereo_proj_x_sign != g_cfg.stereo_proj_x_sign ||
                                  cfg->stereo_eye_x_sign != g_cfg.stereo_eye_x_sign);
    if (!(next.trigger_deadzone >= 0.0 && next.trigger_deadzone < 1.0 &&
          next.trigger_fire > next.trigger_deadzone && next.trigger_fire <= 1.0)) {
        next.trigger_deadzone = 0.10;
        next.trigger_fire = 0.55;
    }
    deadzone = (float)next.trigger_deadzone;
    fire = (float)next.trigger_fire;
    g_cfg = next;
    {
        LONG bits;
        memcpy(&bits, &deadzone, sizeof(bits));
        InterlockedExchange(&g_trigger_deadzone_bits, bits);
        memcpy(&bits, &fire, sizeof(bits));
        InterlockedExchange(&g_trigger_fire_bits, bits);
    }
    g_cfg_seen = 1;
    /* Changing the stereo mode invalidates every stored image: they were drawn
       under the OLD rule, and a mono store carries a deliberately zeroed pose
       that is not a legal thing to submit as an eye. Dropping them costs one
       frame; keeping them cost the whole session (XR_ERROR_POSE_INVALID). */
    if (mode_changed) drop_capture_stores();
    /* The FIRST configure adopts the token instead of reacting to it. The
       marker keeps whatever number the last manual recentre left behind, so on
       the next launch a stale token is indistinguishable from a fresh request -
       and an explicit request deliberately overrides the worn gate. That handed
       the origin straight back to whichever way the headset happened to be
       facing at startup: exactly the defect the gate exists to prevent, coming
       in through the config file rather than through the first pose. */
    if (!first && cfg->recenter_token != prev) {
        InterlockedExchange(&g_recenter_cause, DG_XR_CAUSE_MARKER);
        InterlockedExchange(&g_recenter_req, 1);
    }
}

/* ------------------- software turn (shared half; see dg_xr.h) ------------ */

/* Single atomic record: sample tick in high32, signed millidegrees/s low32.
 * Keeping timestamp and value together prevents a new stamp reviving an old
 * nonzero demand while the game thread is cancelling it. */
static volatile LONG64 g_turn_demand;
static volatile LONG g_turn_offset_bits;    /* float degrees, for readers   */

static void turn_rate_publish_at(double deg_per_s, uint32_t tick)
{
    uint64_t packed;
    /* Fail to zero: a NaN or an absurd rate stops the turn rather than
       spinning the room - same fail-closed rule as every knob here. */
    if (!(deg_per_s > -720.0 && deg_per_s < 720.0)) deg_per_s = 0.0;
    packed=((uint64_t)tick<<32) | (uint32_t)(LONG)(deg_per_s*1000.0);
    InterlockedExchange64(&g_turn_demand,(LONG64)packed);
}
void dg_xr_turn_rate(double deg_per_s) { turn_rate_publish_at(deg_per_s,(uint32_t)GetTickCount()); }
static LONG turn_rate_at(uint32_t now) {
    uint64_t packed=(uint64_t)InterlockedCompareExchange64(&g_turn_demand,0,0);
    uint32_t age=now-(uint32_t)(packed>>32);
    return age<=100u ? (LONG)(uint32_t)packed : 0;
}

static void turn_offset_publish(double rad)
{
    float f = (float)(rad * 180.0 / PI);
    LONG b;
    memcpy(&b, &f, sizeof b);
    InterlockedExchange(&g_turn_offset_bits, b);
}

double dg_xr_turn_offset_rad(void)
{
    LONG b = InterlockedCompareExchange(&g_turn_offset_bits, 0, 0);
    float f;
    memcpy(&f, &b, sizeof f);
    return (double)f * PI / 180.0;
}

void dg_xr_test_set_turn_offset(double rad) { turn_offset_publish(rad); }

/* Advance the reference yaw by delta and re-pivot its position about the
   head, so the mapped HEAD pose is invariant: the player does not slide
   through the world when they stand away from the recentre origin, they
   turn where they stand. The rotation convention is exactly the one
   relative_pose's qrot applies, or the invariance would only hold at the
   origin. */
void dg_xr_turn_step(double ref_q[2], double ref_p[2],
                     double head_px, double head_pz, double delta_rad)
{
    double s = sin(delta_rad * 0.5), c = cos(delta_rad * 0.5);
    double s1 = 2.0 * s * c;              /* sin(delta) */
    double c1 = 1.0 - 2.0 * s * s;        /* cos(delta) */
    double vx = head_px - ref_p[0];
    double vz = head_pz - ref_p[1];
    double qy = ref_q[0], qw = ref_q[1];
    ref_q[0] = c * qy + s * qw;
    ref_q[1] = c * qw - s * qy;
    ref_p[0] = head_px - (c1 * vx + s1 * vz);
    ref_p[1] = head_pz - (c1 * vz - s1 * vx);
}

#ifdef DG_XR_NO_RUNTIME
uint64_t dg_xr_radial_generation(void) { return 0; }
int dg_xr_radial_publish(const dg_radial_view *v, uint64_t g, uint64_t c, uint64_t k) {
    (void)v; (void)g; (void)c; (void)k; return 0;
}
void dg_xr_radial_invalidate(void) {}
void dg_xr_radial_hide(void) {}
int  dg_xr_start(void (*log)(const char *fmt, ...)) { (void)log; return 0; }
void dg_xr_stop(void) {}
int  dg_xr_adopt_device(void *d) { (void)d; return 0; }
void dg_xr_capture_measured(void *sc, int eye, const DG_XR_RAW_POSE *raw,
                          const DG_PROJ_FOV *fov, const DG_NEAR_META *meta) {
    (void)sc; (void)eye; (void)raw; (void)fov; (void)meta;
}
void dg_xr_capture(void *sc, int eye, const DG_XR_RAW_POSE *raw,
                   const DG_PROJ_FOV *fov) {
    (void)sc; (void)eye; (void)raw; (void)fov;
}
int  dg_xr_submitting(void) { return 0; }
int  dg_xr_radar_capture(void *t) { (void)t; return -1; }
void dg_xr_radar_stats(char *out, size_t n) { if (out && n) out[0] = 0; }
/* No runtime, no quad: the desk build tests the POSE (shared above), the
   runtime half owns the submission - the same split as everything here. */
void dg_xr_screen(int active) { (void)active; }
long dg_xr_screen_frames(void) { return 0; }
/* Nothing ever captured, so there is nothing to drop. */
static void drop_capture_stores(void) {}
/* No stub for dg_xr_get_target_fov: it is a seqlock read like dg_xr_get, so it
   lives in the shared section above and is correct with or without a runtime -
   it simply never finds a valid record when nothing publishes one. */
#else

/* ---- dynamically resolved entry points ---------------------------------
   The loader is loaded by name rather than linked, so a machine with no
   OpenXR at all produces a clean "unavailable" log line instead of a game
   that will not start. That is half of S3's gate on its own. */

static HMODULE g_loader;
static PFN_xrGetInstanceProcAddr pfn_GetInstanceProcAddr;

static PFN_xrEnumerateInstanceExtensionProperties pfn_EnumExt;
static PFN_xrCreateInstance        pfn_CreateInstance;
static PFN_xrDestroyInstance       pfn_DestroyInstance;
static PFN_xrGetInstanceProperties pfn_GetInstanceProperties;
static PFN_xrGetSystem             pfn_GetSystem;
static PFN_xrGetSystemProperties   pfn_GetSystemProperties;
static PFN_xrCreateSession         pfn_CreateSession;
static PFN_xrDestroySession        pfn_DestroySession;
static PFN_xrCreateReferenceSpace  pfn_CreateReferenceSpace;
static PFN_xrDestroySpace          pfn_DestroySpace;
static PFN_xrPollEvent             pfn_PollEvent;
static PFN_xrBeginSession          pfn_BeginSession;
static PFN_xrEndSession            pfn_EndSession;
static PFN_xrWaitFrame             pfn_WaitFrame;
static PFN_xrBeginFrame            pfn_BeginFrame;
static PFN_xrEndFrame              pfn_EndFrame;
static PFN_xrLocateSpace           pfn_LocateSpace;
static PFN_xrEnumerateViewConfigurationViews pfn_EnumViews;
static PFN_xrLocateViews           pfn_LocateViews;
static PFN_xrEnumerateSwapchainFormats pfn_EnumScFormats;
static PFN_xrCreateSwapchain       pfn_CreateSwapchain;
static PFN_xrDestroySwapchain      pfn_DestroySwapchain;
static PFN_xrEnumerateSwapchainImages pfn_EnumScImages;
static PFN_xrAcquireSwapchainImage pfn_AcquireImage;
static PFN_xrWaitSwapchainImage    pfn_WaitImage;
static PFN_xrReleaseSwapchainImage pfn_ReleaseImage;
static PFN_xrCreateActionSet       pfn_CreateActionSet;
static PFN_xrDestroyActionSet      pfn_DestroyActionSet;
static PFN_xrCreateAction           pfn_CreateAction;
static PFN_xrCreateActionSpace      pfn_CreateActionSpace;
static PFN_xrSuggestInteractionProfileBindings pfn_SuggestBindings;
static PFN_xrAttachSessionActionSets pfn_AttachActionSets;
static PFN_xrSyncActions            pfn_SyncActions;
static PFN_xrGetActionStateBoolean  pfn_GetActionStateBoolean;
static PFN_xrGetActionStateFloat    pfn_GetActionStateFloat;
static PFN_xrGetActionStateVector2f pfn_GetActionStateVector2f;
static PFN_xrGetActionStatePose     pfn_GetActionStatePose;
static PFN_xrStringToPath           pfn_StringToPath;

static XrInstance g_inst = XR_NULL_HANDLE;
static XrSystemId g_sys  = XR_NULL_SYSTEM_ID;
static XrSession  g_sess = XR_NULL_HANDLE;
static XrSpace    g_view_space  = XR_NULL_HANDLE;
static XrSpace    g_local_space = XR_NULL_HANDLE;
static XrSessionState g_state = XR_SESSION_STATE_UNKNOWN;
static int        g_session_running;
static int        g_presence_enabled;
static int        g_user_present;
static ULONGLONG  g_tracking_since;
static int        g_tracking_was_valid;
static int        g_tracking_lost_logged;
static int        g_input_ready;
static int        g_input_attached;
static int        g_input_failed;
static XrActionSet g_action_set = XR_NULL_HANDLE;
static XrAction    g_recenter_action = XR_NULL_HANDLE;
static XrAction    g_grip_action = XR_NULL_HANDLE;
static XrAction    g_aim_action = XR_NULL_HANDLE;
static XrAction    g_trigger_action = XR_NULL_HANDLE;
static XrAction    g_squeeze_action = XR_NULL_HANDLE;
static XrAction    g_thumbstick_action = XR_NULL_HANDLE;
static XrAction    g_thumbstick_click_action = XR_NULL_HANDLE;
static XrAction    g_primary_action = XR_NULL_HANDLE;
static XrAction    g_secondary_action = XR_NULL_HANDLE;
static XrAction    g_menu_action = XR_NULL_HANDLE;
static XrAction    g_haptic_action = XR_NULL_HANDLE;
static uint64_t    g_hand_sample_seq;
static uint64_t    g_action_edge_seq;

typedef struct {
    XrPath path;
    XrSpace grip_space;
    XrSpace aim_space;
} DG_XR_HAND_RUNTIME;

/* Index into g_hand_runtime / g_hand_action, fixed where the subaction paths
   are resolved: /user/hand/left first, /user/hand/right second. */
enum { HAND_LEFT = 0, HAND_RIGHT = 1 };

typedef struct {
    int grip_active;
    int aim_active;
    int trigger_active;
    float trigger;
    float squeeze;
    float thumbstick_x;
    float thumbstick_y;
    unsigned int thumbstick_active;
    unsigned int trigger_click;
    unsigned int squeeze_click;
    unsigned int thumbstick_click;
    unsigned int primary_button;
    unsigned int primary_active;
    unsigned int secondary_button;
    unsigned int menu_button;
    uint64_t trigger_press_seq;
    uint64_t trigger_release_seq;
} DG_XR_ACTION_SAMPLE;

static DG_XR_HAND_RUNTIME g_hand_runtime[2];
static DG_XR_ACTION_SAMPLE g_hand_action[2];

static void clear_action_samples(void) {
    int i;
    controller_buttons_at(0,0,0,(uint32_t)GetTickCount());
    InterlockedExchange64(&g_controller_toggle,0);
    for (i = 0; i < 2; i++) {
        uint64_t press = g_hand_action[i].trigger_press_seq;
        uint64_t release = g_hand_action[i].trigger_release_seq;
        memset(&g_hand_action[i], 0, sizeof(g_hand_action[i]));
        g_hand_action[i].trigger_press_seq = press;
        g_hand_action[i].trigger_release_seq = release;
    }
}
static ID3D11Device        *g_dev;
static ID3D11DeviceContext *g_ctx;

/* ------------------------------------------------------------ S4b state --- */

/* The game's device, handed over before start. Owning it changes teardown:
   we borrow it, so we AddRef and Release but must never destroy it. */
static ID3D11Device *g_adopted;

/* Both swapchains exist for the session, even when stereo is off. Keeping
   their lifetime independent of the live setting avoids runtime churn. */
static XrSwapchain  g_swap[2] = { XR_NULL_HANDLE, XR_NULL_HANDLE };
static XrSwapchainImageD3D11KHR *g_images[2];
static uint32_t     g_image_count[2];
static uint32_t     g_sw, g_sh;         /* back-buffer sized, per plan 2.20 */

/* Captured at Present on the game thread, consumed by the XR thread. Both
   touch the immediate context, so the device is put into multithread-protected
   mode; without that this is a data race D3D11 will not diagnose for you. */
typedef struct {
    ID3D11Texture2D *texture;
    DG_XR_RAW_POSE raw;
    DG_PROJ_FOV fov;
    int valid;
    uint64_t capture_id, capture_ms;
    DG_MODEL_ARM model_arm;
} DG_CAPTURE_STORE;
static DG_CAPTURE_STORE g_store[2];
static ID3D11Multithread *g_mt;
static volatile LONG g_have_capture;
static volatile LONG g_submitting;
static CRITICAL_SECTION g_cap_cs;
static volatile LONG g_cap_cs_ready;

/* The theater screen. g_screen_req is Present's word (the bridge verdict,
   one interlocked flag); everything else is the frame loop's own and touched
   only on the XR thread. The anchor is captured ONCE per screen session -
   that is the entire point: after capture the quad's pose never reads the
   head again - and recaptured only on recentre ("this is forward" moves the
   screen too) or a live vr_theater_dist change. */
static volatile LONG g_screen_req;
static int g_screen_shown;              /* frame loop's latched copy */
static int g_screen_have_anchor;
static DG_XR_RAW_POSE g_screen_anchor;
static double g_screen_anchor_dist;
static long g_screen_recenter_seen;
static unsigned long g_screen_menu_seq, g_screen_menu_seen; /* XR thread only */
static volatile LONG g_st_screen_frames;
static volatile LONG g_screen_logged;   /* first quad up / first fallback */

void dg_xr_screen(int active) {
    LONG next=active ? 1 : 0;
    if (InterlockedExchange(&g_screen_req,next)!=next) dg_xr_radial_invalidate();
}

long dg_xr_screen_frames(void) {
    return (long)InterlockedCompareExchange(&g_st_screen_frames, 0, 0);
}

/* Clamp shared by anchor and per-frame width so a wild marker value cannot
   put the screen inside the viewer's head or a kilometre out. */
static double screen_clamp_m(double v, double dflt) {
    if (!(v >= 0.5 && v <= 10.0)) return dflt;
    return v;
}

/* The FOV the game actually rendered with, recovered from the live projection
   rather than assumed. Submitting the headset's FOV over an image drawn at the
   game's would stretch the world; this keeps the layer honest, and widening it
   to fill the headset is S4c's job, not S4b's. */
static volatile LONG g_fov_mismatch_logged;

/* Reference frame captured at recentre. Yaw only: the runtime's LOCAL space is
   already gravity-aligned, so cancelling the user's pitch and roll as well
   would lock the game's horizon to however their head happened to be tilted. */
static int    g_have_ref;
static int    g_waiting_logged;
/* Set when xrGetSystem reports no headset. Distinguishes "there is no runtime,
   stay flat" from "the runtime is fine, the headset is not on yet". */
static int    g_no_hmd;
static int    g_no_hmd_logged;
static int    g_sep_logged;
static volatile LONG g_bad_pose_logged;
#include "dg_capture_format.h"
#include "dg_capture_diag.inl"
#include "dg_pixel_probe.inl"
#include "dg_near_probe.inl"
#include "dg_xr_radial.inl"
static XrSwapchain health_radar_swap(void);
#include "dg_xr_health.inl"
#include "dg_xr_grip_debug.inl"
static void radar_teardown(void);      /* dg_xr_radar.inl, included ahead of the frame loop */
static double g_ref_qy, g_ref_qw;      /* yaw-only quaternion, (0,qy,0,qw) */
static double g_ref_px, g_ref_py, g_ref_pz;
/* Software-turn bookkeeping, sample thread only (the shared mirror in
   g_turn_offset_bits is what other threads read). */
static double    g_turn_offset_rad_rt;
static ULONGLONG g_turn_prev_ms;

/* Guarded by the readiness flag because dg_xr_configure can be called before
   the session exists at all - the worker configures once on startup. */
static void drop_capture_stores(void) {
    if (!g_cap_cs_ready) return;
    EnterCriticalSection(&g_cap_cs);
    g_store[0].valid = g_store[1].valid = 0;
    LeaveCriticalSection(&g_cap_cs);
    InterlockedExchange(&g_bad_pose_logged, 0);
}

static const char *capture_cause_name(int cause) {
    if (cause == DG_XR_CAUSE_MARKER) return "marker";
    if (cause == DG_XR_CAUSE_HOTKEY) return "hotkey";
    if (cause == DG_XR_CAUSE_CONTROLLER) return "controller";
    return "initial";
}

#define XRCHK(expr, what) \
    do { XrResult _r = (expr); \
         if (XR_FAILED(_r)) { InterlockedIncrement(&g_st_errors); \
             g_log("  xr: %s failed (%d)\r\n", (what), (int)_r); return 0; } } while (0)

static int resolve(const char *name, void *slot) {
    PFN_xrVoidFunction f = NULL;
    if (XR_FAILED(pfn_GetInstanceProcAddr(g_inst, name, &f)) || !f) {
        g_log("  xr: missing entry point %s\r\n", name);
        return 0;
    }
    *(PFN_xrVoidFunction *)slot = f;
    return 1;
}

static int have_extension(const char *want) {
    uint32_t n = 0, i;
    XrExtensionProperties *props;
    int found = 0;
    if (XR_FAILED(pfn_EnumExt(NULL, 0, &n, NULL)) || n == 0) return 0;
    props = (XrExtensionProperties *)calloc(n, sizeof(*props));
    if (!props) return 0;
    for (i = 0; i < n; i++) props[i].type = XR_TYPE_EXTENSION_PROPERTIES;
    if (XR_SUCCEEDED(pfn_EnumExt(NULL, n, &n, props)))
        for (i = 0; i < n; i++)
            if (strcmp(props[i].extensionName, want) == 0) { found = 1; break; }
    free(props);
    return found;
}

static int create_hand_action(const char *name, const char *localized,
                              XrActionType type, const XrPath *subactions,
                              XrAction *out) {
    XrActionCreateInfo aci;
    memset(&aci, 0, sizeof(aci));
    aci.type = XR_TYPE_ACTION_CREATE_INFO;
    strcpy_s(aci.actionName, sizeof(aci.actionName), name);
    strcpy_s(aci.localizedActionName, sizeof(aci.localizedActionName), localized);
    aci.actionType = type;
    aci.countSubactionPaths = 2;
    aci.subactionPaths = subactions;
    return XR_SUCCEEDED(pfn_CreateAction(g_action_set, &aci, out));
}

typedef struct {
    XrAction action;
    const char *path;
} DG_XR_BINDING_SPEC;

static int suggest_profile(const char *profile_name,
                           const DG_XR_BINDING_SPEC *spec, size_t count) {
    XrActionSuggestedBinding bindings[24];
    XrInteractionProfileSuggestedBinding sb;
    XrPath profile;
    size_t i;
    if (count > sizeof(bindings) / sizeof(bindings[0]) ||
        XR_FAILED(pfn_StringToPath(g_inst, profile_name, &profile)))
        return 0;
    for (i = 0; i < count; i++) {
        bindings[i].action = spec[i].action;
        if (XR_FAILED(pfn_StringToPath(g_inst, spec[i].path,
                                       &bindings[i].binding)))
            return 0;
    }
    memset(&sb, 0, sizeof(sb));
    sb.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
    sb.interactionProfile = profile;
    sb.countSuggestedBindings = (uint32_t)count;
    sb.suggestedBindings = bindings;
    return XR_SUCCEEDED(pfn_SuggestBindings(g_inst, &sb));
}

static void setup_controller_input(void) {
    XrActionSetCreateInfo asci;
    XrActionCreateInfo aci;
    XrPath subactions[2];
    DG_XR_BINDING_SPEC touch[20];
    DG_XR_BINDING_SPEC simple[11];
    int suggested = 0;

    if (!g_input_ready) return;
    if (XR_FAILED(pfn_StringToPath(g_inst, "/user/hand/left",
                                   &subactions[HAND_LEFT])) ||
        XR_FAILED(pfn_StringToPath(g_inst, "/user/hand/right",
                                   &subactions[HAND_RIGHT])))
        goto fail;
    g_hand_runtime[HAND_LEFT].path = subactions[HAND_LEFT];
    g_hand_runtime[HAND_RIGHT].path = subactions[HAND_RIGHT];

    memset(&asci, 0, sizeof(asci));
    asci.type = XR_TYPE_ACTION_SET_CREATE_INFO;
    strcpy_s(asci.actionSetName, sizeof(asci.actionSetName), "dg_recenter");
    strcpy_s(asci.localizedActionSetName, sizeof(asci.localizedActionSetName),
              "MGS2 PCVR Input");
    asci.priority = 0;
    if (XR_FAILED(pfn_CreateActionSet(g_inst, &asci, &g_action_set))) goto fail;

    memset(&aci, 0, sizeof(aci));
    aci.type = XR_TYPE_ACTION_CREATE_INFO;
    strcpy_s(aci.actionName, sizeof(aci.actionName), "recenter");
    strcpy_s(aci.localizedActionName, sizeof(aci.localizedActionName),
              "Recenter");
    aci.actionType = XR_ACTION_TYPE_BOOLEAN_INPUT;
    if (XR_FAILED(pfn_CreateAction(g_action_set, &aci, &g_recenter_action))) goto fail;

    if (!create_hand_action("grip_pose", "Grip Pose", XR_ACTION_TYPE_POSE_INPUT,
                            subactions, &g_grip_action) ||
        !create_hand_action("aim_pose", "Aim Pose", XR_ACTION_TYPE_POSE_INPUT,
                            subactions, &g_aim_action) ||
        !create_hand_action("trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT,
                            subactions, &g_trigger_action) ||
        !create_hand_action("squeeze", "Squeeze", XR_ACTION_TYPE_FLOAT_INPUT,
                            subactions, &g_squeeze_action) ||
        !create_hand_action("thumbstick", "Thumbstick",
                            XR_ACTION_TYPE_VECTOR2F_INPUT,
                            subactions, &g_thumbstick_action) ||
        !create_hand_action("thumbstick_click", "Thumbstick Click",
                            XR_ACTION_TYPE_BOOLEAN_INPUT,
                            subactions, &g_thumbstick_click_action) ||
        !create_hand_action("primary_button", "Primary Button",
                            XR_ACTION_TYPE_BOOLEAN_INPUT,
                            subactions, &g_primary_action) ||
        !create_hand_action("secondary_button", "Secondary Button",
                            XR_ACTION_TYPE_BOOLEAN_INPUT,
                            subactions, &g_secondary_action) ||
        !create_hand_action("menu_button", "Menu Button",
                            XR_ACTION_TYPE_BOOLEAN_INPUT,
                            subactions, &g_menu_action) ||
        !create_hand_action("haptic", "Haptic",
                            XR_ACTION_TYPE_VIBRATION_OUTPUT,
                            subactions, &g_haptic_action))
        goto fail;

#define BIND(list, index, act, component) \
    do { (list)[(index)].action = (act); (list)[(index)].path = (component); } while (0)
    BIND(touch, 0, g_recenter_action, "/user/hand/left/input/y/click");
    BIND(touch, 1, g_grip_action, "/user/hand/left/input/grip/pose");
    BIND(touch, 2, g_grip_action, "/user/hand/right/input/grip/pose");
    BIND(touch, 3, g_aim_action, "/user/hand/left/input/aim/pose");
    BIND(touch, 4, g_aim_action, "/user/hand/right/input/aim/pose");
    BIND(touch, 5, g_trigger_action, "/user/hand/left/input/trigger/value");
    BIND(touch, 6, g_trigger_action, "/user/hand/right/input/trigger/value");
    BIND(touch, 7, g_squeeze_action, "/user/hand/left/input/squeeze/value");
    BIND(touch, 8, g_squeeze_action, "/user/hand/right/input/squeeze/value");
    BIND(touch, 9, g_thumbstick_action, "/user/hand/left/input/thumbstick");
    BIND(touch, 10, g_thumbstick_action, "/user/hand/right/input/thumbstick");
    BIND(touch, 11, g_thumbstick_click_action, "/user/hand/left/input/thumbstick/click");
    BIND(touch, 12, g_thumbstick_click_action, "/user/hand/right/input/thumbstick/click");
    BIND(touch, 13, g_primary_action, "/user/hand/left/input/x/click");
    BIND(touch, 14, g_primary_action, "/user/hand/right/input/a/click");
    BIND(touch, 15, g_secondary_action, "/user/hand/left/input/y/click");
    BIND(touch, 16, g_secondary_action, "/user/hand/right/input/b/click");
    BIND(touch, 17, g_menu_action, "/user/hand/left/input/menu/click");
    BIND(touch, 18, g_haptic_action, "/user/hand/left/output/haptic");
    BIND(touch, 19, g_haptic_action, "/user/hand/right/output/haptic");

    BIND(simple, 0, g_recenter_action, "/user/hand/left/input/select/click");
    BIND(simple, 1, g_grip_action, "/user/hand/left/input/grip/pose");
    BIND(simple, 2, g_grip_action, "/user/hand/right/input/grip/pose");
    BIND(simple, 3, g_aim_action, "/user/hand/left/input/aim/pose");
    BIND(simple, 4, g_aim_action, "/user/hand/right/input/aim/pose");
    BIND(simple, 5, g_trigger_action, "/user/hand/left/input/select/click");
    BIND(simple, 6, g_trigger_action, "/user/hand/right/input/select/click");
    BIND(simple, 7, g_menu_action, "/user/hand/left/input/menu/click");
    BIND(simple, 8, g_menu_action, "/user/hand/right/input/menu/click");
    BIND(simple, 9, g_haptic_action, "/user/hand/left/output/haptic");
    BIND(simple, 10, g_haptic_action, "/user/hand/right/output/haptic");
#undef BIND

    if (suggest_profile("/interaction_profiles/oculus/touch_controller",
                        touch, sizeof(touch) / sizeof(touch[0])))
        suggested++;
    else
        g_log("  xr: no motion bindings for Oculus Touch\r\n");
    if (suggest_profile("/interaction_profiles/khr/simple_controller",
                        simple, sizeof(simple) / sizeof(simple[0])))
        suggested++;
    else
        g_log("  xr: no motion bindings for KHR simple controller\r\n");
    if (!suggested) goto fail;
    g_log("  xr: motion input bound on %d controller profile(s)\r\n", suggested);
    return;

fail:
    g_log("  xr: controller input setup failed; continuing without it\r\n");
    if (g_action_set && pfn_DestroyActionSet) pfn_DestroyActionSet(g_action_set);
    g_action_set = XR_NULL_HANDLE;
    g_recenter_action = XR_NULL_HANDLE;
    g_grip_action = g_aim_action = XR_NULL_HANDLE;
    g_trigger_action = g_squeeze_action = XR_NULL_HANDLE;
    g_thumbstick_action = g_thumbstick_click_action = XR_NULL_HANDLE;
    g_primary_action = g_secondary_action = g_menu_action = XR_NULL_HANDLE;
    g_haptic_action = XR_NULL_HANDLE;
    g_input_ready = 0;
}

static int create_action_spaces(void) {
    XrActionSpaceCreateInfo ci;
    int i;
    if (!g_input_ready) return 1;
    for (i = 0; i < 2; i++) {
        memset(&ci, 0, sizeof(ci));
        ci.type = XR_TYPE_ACTION_SPACE_CREATE_INFO;
        ci.poseInActionSpace.orientation.w = 1.0f;
        ci.subactionPath = g_hand_runtime[i].path;
        ci.action = g_grip_action;
        if (XR_FAILED(pfn_CreateActionSpace(g_sess, &ci,
                                            &g_hand_runtime[i].grip_space)))
            return 0;
        ci.action = g_aim_action;
        if (XR_FAILED(pfn_CreateActionSpace(g_sess, &ci,
                                            &g_hand_runtime[i].aim_space)))
            return 0;
    }
    return 1;
}

/* Our own device, deliberately. Borrowing the game's would mean hooking
   Present to get at it, and S3 has no reason to touch the game's rendering. */
static int make_device(LUID want) {
    IDXGIFactory1 *factory = NULL;
    IDXGIAdapter1 *adapter = NULL, *chosen = NULL;
    UINT i;
    D3D_FEATURE_LEVEL want_fl[] = { D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0 };
    D3D_FEATURE_LEVEL got_fl;
    HRESULT hr;

    if (FAILED(CreateDXGIFactory1(&IID_IDXGIFactory1, (void **)&factory)))
        return 0;
    for (i = 0; factory->lpVtbl->EnumAdapters1(factory, i, &adapter) == S_OK; i++) {
        DXGI_ADAPTER_DESC1 d;
        if (SUCCEEDED(adapter->lpVtbl->GetDesc1(adapter, &d)) &&
            d.AdapterLuid.LowPart == want.LowPart &&
            d.AdapterLuid.HighPart == want.HighPart) {
            chosen = adapter;
            break;
        }
        adapter->lpVtbl->Release(adapter);
    }
    if (!chosen) {
        factory->lpVtbl->Release(factory);
        g_log("  xr: no adapter matches the runtime's LUID\r\n");
        return 0;
    }
    hr = D3D11CreateDevice((IDXGIAdapter *)chosen, D3D_DRIVER_TYPE_UNKNOWN, NULL, 0,
                           want_fl, 2, D3D11_SDK_VERSION, &g_dev, &got_fl, &g_ctx);
    chosen->lpVtbl->Release(chosen);
    factory->lpVtbl->Release(factory);
    if (FAILED(hr)) { g_log("  xr: D3D11CreateDevice failed (0x%08lX)\r\n", (unsigned long)hr); return 0; }
    return 1;
}

static int xr_init(void) {
    XrInstanceCreateInfo ici;
    XrSystemGetInfo sgi;
    XrInstanceProperties ip;
    XrSystemProperties sp;
    XrSessionCreateInfo sci;
    XrGraphicsBindingD3D11KHR bind;
    XrReferenceSpaceCreateInfo rsci;
    XrGraphicsRequirementsD3D11KHR req;
    PFN_xrGetD3D11GraphicsRequirementsKHR get_req = NULL;
    const char *exts[2];

    g_no_hmd = 0;
    g_loader = LoadLibraryA("openxr_loader.dll");
    if (!g_loader) { g_log("  xr: openxr_loader.dll not found - staying flat\r\n"); return 0; }
    pfn_GetInstanceProcAddr =
        (PFN_xrGetInstanceProcAddr)GetProcAddress(g_loader, "xrGetInstanceProcAddr");
    if (!pfn_GetInstanceProcAddr) { g_log("  xr: loader has no xrGetInstanceProcAddr\r\n"); return 0; }

    if (!resolve("xrEnumerateInstanceExtensionProperties", &pfn_EnumExt)) return 0;
    if (!resolve("xrCreateInstance", &pfn_CreateInstance)) return 0;

    if (!have_extension(XR_KHR_D3D11_ENABLE_EXTENSION_NAME)) {
        g_log("  xr: runtime lacks %s\r\n", XR_KHR_D3D11_ENABLE_EXTENSION_NAME);
        return 0;
    }
    exts[0] = XR_KHR_D3D11_ENABLE_EXTENSION_NAME;
    g_presence_enabled = have_extension(XR_EXT_USER_PRESENCE_EXTENSION_NAME);
    if (g_presence_enabled) {
        exts[1] = XR_EXT_USER_PRESENCE_EXTENSION_NAME;
        g_log("  xr: user-presence gate enabled\r\n");
    } else {
        g_log("  xr: user-presence unavailable; using focused + 500 ms tracking gate\r\n");
    }

    memset(&ici, 0, sizeof(ici));
    ici.type = XR_TYPE_INSTANCE_CREATE_INFO;
    strcpy_s(ici.applicationInfo.applicationName,
             sizeof(ici.applicationInfo.applicationName), "MGS2 PCVR v3");
    ici.applicationInfo.applicationVersion = 3;
    strcpy_s(ici.applicationInfo.engineName,
             sizeof(ici.applicationInfo.engineName), "dg_hook");
    ici.applicationInfo.engineVersion = 1;
    ici.applicationInfo.apiVersion = XR_API_VERSION_1_0;
    ici.enabledExtensionCount = g_presence_enabled ? 2 : 1;
    ici.enabledExtensionNames = exts;
    XRCHK(pfn_CreateInstance(&ici, &g_inst), "xrCreateInstance");

    if (!resolve("xrDestroyInstance", &pfn_DestroyInstance)) return 0;
    if (!resolve("xrGetInstanceProperties", &pfn_GetInstanceProperties)) return 0;
    if (!resolve("xrGetSystem", &pfn_GetSystem)) return 0;
    if (!resolve("xrGetSystemProperties", &pfn_GetSystemProperties)) return 0;
    if (!resolve("xrCreateSession", &pfn_CreateSession)) return 0;
    if (!resolve("xrDestroySession", &pfn_DestroySession)) return 0;
    if (!resolve("xrCreateReferenceSpace", &pfn_CreateReferenceSpace)) return 0;
    if (!resolve("xrDestroySpace", &pfn_DestroySpace)) return 0;
    if (!resolve("xrPollEvent", &pfn_PollEvent)) return 0;
    if (!resolve("xrBeginSession", &pfn_BeginSession)) return 0;
    if (!resolve("xrEndSession", &pfn_EndSession)) return 0;
    if (!resolve("xrWaitFrame", &pfn_WaitFrame)) return 0;
    if (!resolve("xrBeginFrame", &pfn_BeginFrame)) return 0;
    if (!resolve("xrEndFrame", &pfn_EndFrame)) return 0;
    if (!resolve("xrLocateSpace", &pfn_LocateSpace)) return 0;
    if (!resolve("xrGetD3D11GraphicsRequirementsKHR", &get_req)) return 0;
    /* S4b additions. Resolved unconditionally: if the runtime cannot do these
       it cannot do stereo either, and finding out here beats finding out
       halfway through the first frame. */
    if (!resolve("xrEnumerateViewConfigurationViews", &pfn_EnumViews)) return 0;
    if (!resolve("xrLocateViews", &pfn_LocateViews)) return 0;
    if (!resolve("xrEnumerateSwapchainFormats", &pfn_EnumScFormats)) return 0;
    if (!resolve("xrCreateSwapchain", &pfn_CreateSwapchain)) return 0;
    if (!resolve("xrDestroySwapchain", &pfn_DestroySwapchain)) return 0;
    if (!resolve("xrEnumerateSwapchainImages", &pfn_EnumScImages)) return 0;
    if (!resolve("xrAcquireSwapchainImage", &pfn_AcquireImage)) return 0;
    if (!resolve("xrWaitSwapchainImage", &pfn_WaitImage)) return 0;
    if (!resolve("xrReleaseSwapchainImage", &pfn_ReleaseImage)) return 0;
    /* Controller input is optional: a broken input path must never take down
       the pose session, which remains useful with the marker or hotkey. */
    if (resolve("xrCreateActionSet", &pfn_CreateActionSet) &&
        resolve("xrDestroyActionSet", &pfn_DestroyActionSet) &&
        resolve("xrCreateAction", &pfn_CreateAction) &&
        resolve("xrCreateActionSpace", &pfn_CreateActionSpace) &&
        resolve("xrSuggestInteractionProfileBindings", &pfn_SuggestBindings) &&
        resolve("xrAttachSessionActionSets", &pfn_AttachActionSets) &&
        resolve("xrSyncActions", &pfn_SyncActions) &&
        resolve("xrGetActionStateBoolean", &pfn_GetActionStateBoolean) &&
        resolve("xrGetActionStateFloat", &pfn_GetActionStateFloat) &&
        resolve("xrGetActionStateVector2f", &pfn_GetActionStateVector2f) &&
        resolve("xrGetActionStatePose", &pfn_GetActionStatePose) &&
        resolve("xrStringToPath", &pfn_StringToPath))
        g_input_ready = 1;
    else
        g_log("  xr: controller input unavailable; continuing without it\r\n");

    setup_controller_input();

    memset(&ip, 0, sizeof(ip)); ip.type = XR_TYPE_INSTANCE_PROPERTIES;
    if (XR_SUCCEEDED(pfn_GetInstanceProperties(g_inst, &ip)))
        g_log("  xr: runtime %s %u.%u.%u\r\n", ip.runtimeName,
              (unsigned)XR_VERSION_MAJOR(ip.runtimeVersion),
              (unsigned)XR_VERSION_MINOR(ip.runtimeVersion),
              (unsigned)XR_VERSION_PATCH(ip.runtimeVersion));

    memset(&sgi, 0, sizeof(sgi));
    sgi.type = XR_TYPE_SYSTEM_GET_INFO;
    sgi.formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY;
    /* Not XRCHK, because one of this call's failures is not a failure of ours.
       XR_ERROR_FORM_FACTOR_UNAVAILABLE means the runtime is present and
       answering - it has just told us its name and version - but no headset is
       powered on or in Link yet. The spec describes exactly this as transient.
       Treating it like "no runtime" is what left the mod flat for an entire
       run when the game was launched before the headset was awake, with a
       restart of MGS2 as the only cure. */
    {
        XrResult r = pfn_GetSystem(g_inst, &sgi, &g_sys);
        if (r == XR_ERROR_FORM_FACTOR_UNAVAILABLE) { g_no_hmd = 1; return 0; }
        if (XR_FAILED(r)) {
            InterlockedIncrement(&g_st_errors);
            g_log("  xr: xrGetSystem failed (%d)\r\n", (int)r);
            return 0;
        }
    }

    memset(&sp, 0, sizeof(sp)); sp.type = XR_TYPE_SYSTEM_PROPERTIES;
    g_radial_max_layers=0;
    if (pfn_GetSystemProperties(g_inst, g_sys, &sp)==XR_SUCCESS) {
        g_radial_max_layers=sp.graphicsProperties.maxLayerCount;
        g_log("  xr: system %s\r\n", sp.systemName);
    }

    memset(&req, 0, sizeof(req));
    req.type = XR_TYPE_GRAPHICS_REQUIREMENTS_D3D11_KHR;
    XRCHK(get_req(g_inst, g_sys, &req), "xrGetD3D11GraphicsRequirementsKHR");

    if (g_adopted) {
        IDXGIDevice  *dxgi_dev = NULL;
        IDXGIAdapter *adapter  = NULL;
        DXGI_ADAPTER_DESC ad;
        int usable = 0;
        if (SUCCEEDED(g_adopted->lpVtbl->QueryInterface(g_adopted, &IID_IDXGIDevice,
                                                        (void **)&dxgi_dev)) && dxgi_dev) {
            if (SUCCEEDED(dxgi_dev->lpVtbl->GetAdapter(dxgi_dev, &adapter)) && adapter) {
                if (SUCCEEDED(adapter->lpVtbl->GetDesc(adapter, &ad)) &&
                    ad.AdapterLuid.LowPart  == req.adapterLuid.LowPart &&
                    ad.AdapterLuid.HighPart == req.adapterLuid.HighPart)
                    usable = 1;
                adapter->lpVtbl->Release(adapter);
            }
            dxgi_dev->lpVtbl->Release(dxgi_dev);
        }
        if (usable) {
            g_dev = g_adopted;
            g_dev->lpVtbl->AddRef(g_dev);
            g_dev->lpVtbl->GetImmediateContext(g_dev, &g_ctx);
            /* Two threads will drive this context. Say so explicitly. */
            if (SUCCEEDED(g_ctx->lpVtbl->QueryInterface(g_ctx, &IID_ID3D11Multithread,
                                                        (void **)&g_mt)) && g_mt)
                g_mt->lpVtbl->SetMultithreadProtected(g_mt, TRUE);
            else
                g_log("  xr: ID3D11Multithread unavailable - context sharing is unsafe\r\n");
            g_log("  xr: adopted the game's D3D11 device\r\n");
        } else {
            g_log("  xr: game device is on a different adapter - staying pose-only\r\n");
            g_adopted = NULL;
        }
    }
    if (!g_dev && !make_device(req.adapterLuid)) return 0;

    memset(&bind, 0, sizeof(bind));
    bind.type = XR_TYPE_GRAPHICS_BINDING_D3D11_KHR;
    bind.device = g_dev;
    memset(&sci, 0, sizeof(sci));
    sci.type = XR_TYPE_SESSION_CREATE_INFO;
    sci.next = &bind;
    sci.systemId = g_sys;
    XRCHK(pfn_CreateSession(g_inst, &sci, &g_sess), "xrCreateSession");
    radial_new_session();
    health_new_session();gripdbg_new_session();

    memset(&rsci, 0, sizeof(rsci));
    rsci.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO;
    rsci.poseInReferenceSpace.orientation.w = 1.0f;
    rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
    XRCHK(pfn_CreateReferenceSpace(g_sess, &rsci, &g_view_space), "refspace VIEW");
    rsci.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
    XRCHK(pfn_CreateReferenceSpace(g_sess, &rsci, &g_local_space), "refspace LOCAL");
    /* pump_events attaches only after this returns; OpenXR forbids adding
       actions or action spaces after xrAttachSessionActionSets. */
    if (!create_action_spaces()) {
        g_log("  xr: controller action-space setup failed; continuing without input\r\n");
        g_input_ready = 0;
    }

    g_log("  xr: session ready (zero layers until both eye stores are valid)\r\n");
    return 1;
}

static void xr_teardown(void) {
    int stopped=!g_session_running;
    pixel_probe_teardown();
    if (g_session_running && pfn_EndSession) stopped=pfn_EndSession(g_sess)==XR_SUCCESS;
    radial_stop(stopped);
    health_stop(stopped);gripdbg_stop(stopped);
    radar_teardown();
    g_session_running = 0;
    InterlockedExchange(&g_submitting, 0);
    InterlockedExchange(&g_have_capture, 0);
    if (g_cap_cs_ready) {
        EnterCriticalSection(&g_cap_cs);
        near_probe_free();
        if (g_store[0].texture) {
            g_store[0].texture->lpVtbl->Release(g_store[0].texture);
            g_store[0].texture = NULL;
        }
        if (g_store[1].texture) {
            g_store[1].texture->lpVtbl->Release(g_store[1].texture);
            g_store[1].texture = NULL;
        }
        g_store[0].valid = g_store[1].valid = 0;
        LeaveCriticalSection(&g_cap_cs);
    }
    if (pfn_DestroySwapchain) {
        if (g_swap[0]) pfn_DestroySwapchain(g_swap[0]);
        if (g_swap[1]) pfn_DestroySwapchain(g_swap[1]);
    }
    g_swap[0] = g_swap[1] = XR_NULL_HANDLE;
    if (g_images[0]) { free(g_images[0]); g_images[0] = NULL; }
    if (g_images[1]) { free(g_images[1]); g_images[1] = NULL; }
    g_image_count[0] = g_image_count[1] = 0;
    if (g_mt) { g_mt->lpVtbl->Release(g_mt); g_mt = NULL; }
    /* g_adopted deliberately SURVIVES teardown. It is the game's device, we
       hold a reference to it (see dg_xr_adopt_device), and a session restart
       must re-bind that same device - clearing it here made a restart fall
       back to a private device and silently become pose-only, which looks
       exactly like the headset simply staying dark. It is released in
       dg_xr_stop, not here. */
    if (pfn_DestroySpace) {
        int i;
        for (i = 0; i < 2; i++) {
            if (g_hand_runtime[i].grip_space)
                pfn_DestroySpace(g_hand_runtime[i].grip_space);
            if (g_hand_runtime[i].aim_space)
                pfn_DestroySpace(g_hand_runtime[i].aim_space);
            g_hand_runtime[i].grip_space = XR_NULL_HANDLE;
            g_hand_runtime[i].aim_space = XR_NULL_HANDLE;
        }
    }
    if (g_view_space  && pfn_DestroySpace) pfn_DestroySpace(g_view_space);
    if (g_local_space && pfn_DestroySpace) pfn_DestroySpace(g_local_space);
    g_view_space = g_local_space = XR_NULL_HANDLE;
    if (g_sess && pfn_DestroySession) {
        int destroyed=pfn_DestroySession(g_sess)==XR_SUCCESS;
        radial_parent_destroyed(destroyed);health_parent_destroyed(destroyed);gripdbg_parent_destroyed(destroyed);
    }
    g_sess = XR_NULL_HANDLE;
    if (g_action_set && pfn_DestroyActionSet) pfn_DestroyActionSet(g_action_set);
    g_action_set = XR_NULL_HANDLE;
    g_recenter_action = XR_NULL_HANDLE;
    g_grip_action = g_aim_action = XR_NULL_HANDLE;
    g_trigger_action = g_squeeze_action = XR_NULL_HANDLE;
    g_thumbstick_action = g_thumbstick_click_action = XR_NULL_HANDLE;
    g_primary_action = g_secondary_action = g_menu_action = XR_NULL_HANDLE;
    g_haptic_action = XR_NULL_HANDLE;
    clear_action_samples();
    g_input_attached = 0;
    if (g_ctx) { g_ctx->lpVtbl->Release(g_ctx); g_ctx = NULL; }
    if (g_dev) { g_dev->lpVtbl->Release(g_dev); g_dev = NULL; }
    if (g_inst && pfn_DestroyInstance) pfn_DestroyInstance(g_inst);
    g_inst = XR_NULL_HANDLE;
    if (g_loader) { FreeLibrary(g_loader); g_loader = NULL; }
    /* The seated origin is process-sticky. Session teardown must not turn a
       resume or headset re-wear into an implicit recentre. */
}

/* Created lazily: the swapchain is back-buffer sized (plan 2.20), and that size
   is not known until the first Present has been seen. */
static int ensure_swapchains(void) {
    XrSwapchainCreateInfo sci;
    int64_t *formats = NULL;
    uint32_t n = 0, cap = 0, i;
    int64_t want = 0;

    if (g_swap[0] != XR_NULL_HANDLE && g_swap[1] != XR_NULL_HANDLE) return 1;
    if (!g_adopted || !g_sw || !g_sh) return 0;

    if (XR_FAILED(pfn_EnumScFormats(g_sess, 0, &n, NULL)) || !n) return 0;
    formats = (int64_t *)calloc(n, sizeof(int64_t));
    if (!formats) return 0;
    if (XR_FAILED(pfn_EnumScFormats(g_sess, n, &n, formats))) { free(formats); return 0; }

    /* The back buffer is B8G8R8A8_UNORM (2.21). Matching it keeps CopyResource
       legal with no conversion blit. The _SRGB sibling is an acceptable second
       choice - same TYPELESS family, so the copy is still legal, and only the
       runtime's interpretation of the bits differs. */
    for (i = 0; i < n && !want; i++)
        if (formats[i] == DXGI_FORMAT_B8G8R8A8_UNORM) want = formats[i];
    for (i = 0; i < n && !want; i++)
        if (formats[i] == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB) want = formats[i];
    if (!want) want = formats[0];
    free(formats);

    memset(&sci, 0, sizeof(sci));
    sci.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
    sci.usageFlags = XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
                     XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
    sci.format = want;
    sci.sampleCount = 1;
    sci.width = g_sw;
    sci.height = g_sh;
    sci.faceCount = 1;
    sci.arraySize = 1;
    sci.mipCount = 1;
    for (i = 0; i < 2; i++) {
        uint32_t j;
        if (XR_FAILED(pfn_CreateSwapchain(g_sess, &sci, &g_swap[i]))) {
            g_swap[i] = XR_NULL_HANDLE;
            g_log("  xr: xrCreateSwapchain failed\r\n");
            return 0;
        }
        cap = 0;
        if (XR_FAILED(pfn_EnumScImages(g_swap[i], 0, &cap, NULL)) || !cap)
            return 0;
        g_images[i] = (XrSwapchainImageD3D11KHR *)calloc(cap, sizeof(*g_images[i]));
        if (!g_images[i]) return 0;
        for (j = 0; j < cap; j++)
            g_images[i][j].type = XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR;
        if (XR_FAILED(pfn_EnumScImages(g_swap[i], cap, &cap,
                    (XrSwapchainImageBaseHeader *)g_images[i])) ) return 0;
        g_image_count[i] = cap;
    }
    g_log("  xr: stereo swapchains %ux%u fmt %lld, %u images each\r\n",
          g_sw, g_sh, (long long)want, g_image_count[0]);
    return 1;
}

int dg_xr_adopt_device(void *d3d11_device) {
    ID3D11Device *dev = (ID3D11Device *)d3d11_device;
    if (!dev) return 0;
    /* A reference is held for the life of the module, not just the session.
       The caller releases its own as soon as this returns, and a session
       restart has to re-bind the SAME device - without a reference of our own
       that pointer would be a borrowed one surviving across a teardown, which
       is the kind of thing that works until the day it does not. */
    if (g_adopted == dev) return 1;
    if (g_adopted) g_adopted->lpVtbl->Release(g_adopted);
    g_adopted = dev;
    g_adopted->lpVtbl->AddRef(g_adopted);
    return 1;
}

int dg_xr_submitting(void) { return g_submitting; }

void dg_xr_capture_measured(void *swapchain, int eye, const DG_XR_RAW_POSE *raw,
                   const DG_PROJ_FOV *fov, const DG_NEAR_META *meta) {
    IDXGISwapChain *sc = (IDXGISwapChain *)swapchain;
    ID3D11Texture2D *back = NULL;
    D3D11_TEXTURE2D_DESC td;
    DG_CAPTURE_STORE *store;
    HRESULT hr;
    int mono = eye == DG_EYE_MONO;

    capture_diag_inc(CD_ATTEMPT);
    if (!sc || !g_ctx || !g_dev || !g_cap_cs_ready) {
        capture_diag_producer_fail(CD_INPUT, E_POINTER); return;
    }
    if (eye == DG_EYE_MONO) eye = DG_EYE_LEFT;
    if (eye != DG_EYE_LEFT && eye != DG_EYE_RIGHT) {
        capture_diag_producer_fail(CD_INPUT, E_INVALIDARG); return;
    }
    /* A rejected camera frame has no honest FOV or pose. Leave the previous
       pixels and metadata paired, rather than labelling game-camera pixels. */
    if (!fov) { capture_diag_producer_fail(CD_NO_FOV, S_OK); return; }
    store = &g_store[eye];

    hr = sc->lpVtbl->GetBuffer(sc, 0, &IID_ID3D11Texture2D, (void **)&back);
    if (FAILED(hr) || !back) {
        capture_diag_producer_fail(CD_GET_BUFFER, FAILED(hr) ? hr : E_POINTER);
        return;
    }
    back->lpVtbl->GetDesc(back, &td);

    EnterCriticalSection(&g_cap_cs);
    if (store->texture) {
        D3D11_TEXTURE2D_DESC cd;
        store->texture->lpVtbl->GetDesc(store->texture, &cd);
        /* A resolution change mid-session must not silently copy the wrong
           size; drop the texture and let it be rebuilt. */
        if (cd.Width != td.Width || cd.Height != td.Height || cd.Format != td.Format) {
            store->texture->lpVtbl->Release(store->texture);
            store->texture = NULL;
            store->valid = 0;
        }
    }
    if (!store->texture) {
        D3D11_TEXTURE2D_DESC cd = td;
        cd.Usage = D3D11_USAGE_DEFAULT;
        cd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        cd.CPUAccessFlags = 0;
        cd.MiscFlags = 0;
        cd.SampleDesc.Count = 1;
        cd.SampleDesc.Quality = 0;
        hr = g_dev->lpVtbl->CreateTexture2D(g_dev, &cd, NULL, &store->texture);
        if (SUCCEEDED(hr) && store->texture) {
            g_sw = td.Width;
            g_sh = td.Height;
        } else {
            capture_diag_producer_fail(CD_CREATE, FAILED(hr) ? hr : E_POINTER);
            store->texture = NULL;
        }
    }
    if (store->texture) {
        g_ctx->lpVtbl->CopyResource(g_ctx, (ID3D11Resource *)store->texture,
                                    (ID3D11Resource *)back);
        if (raw) store->raw = *raw;
        else memset(&store->raw, 0, sizeof(store->raw));
        store->fov = *fov;
        store->valid = 1;
        store->capture_id = (uint64_t)InterlockedIncrement64(&g_cd_count[CD_STORED]);
        store->capture_ms = GetTickCount64();
        dg_bridge_model_arm_snapshot(&store->model_arm);
        {
            DG_XR_CAPTURE_OBSERVER observer=(DG_XR_CAPTURE_OBSERVER)
                InterlockedCompareExchangePointer(&g_capture_observer,NULL,NULL);
            if(observer)observer(eye,store->capture_id);
        }
        pixel_probe_source(back, mono);
        near_probe_capture(back, eye, mono, raw, fov, meta, store->capture_id);
        InterlockedExchange(&g_have_capture, 1);
    }
    LeaveCriticalSection(&g_cap_cs);
    back->lpVtbl->Release(back);
}

void dg_xr_capture(void *sc, int eye, const DG_XR_RAW_POSE *raw, const DG_PROJ_FOV *fov) {
    dg_xr_capture_measured(sc, eye, raw, fov, NULL);
}

/* Session-state names for the log. Run 15 (2026-09-05) ended with the
   headset dark: the runtime sent STOPPING and never READY again, and the
   log could not say whether it had parked us in IDLE, whether a later
   state came that we mishandled, or whether this thread was even alive.
   Every transition is now named and stamped (GetTickCount64, the clock the
   heartbeat prints too). */
static const char *xr_state_name(XrSessionState st) {
    switch (st) {
    case XR_SESSION_STATE_UNKNOWN:      return "UNKNOWN";
    case XR_SESSION_STATE_IDLE:         return "IDLE";
    case XR_SESSION_STATE_READY:        return "READY";
    case XR_SESSION_STATE_SYNCHRONIZED: return "SYNCHRONIZED";
    case XR_SESSION_STATE_VISIBLE:      return "VISIBLE";
    case XR_SESSION_STATE_FOCUSED:      return "FOCUSED";
    case XR_SESSION_STATE_STOPPING:     return "STOPPING";
    case XR_SESSION_STATE_LOSS_PENDING: return "LOSS_PENDING";
    case XR_SESSION_STATE_EXITING:      return "EXITING";
    default: break;
    }
    return "?";
}

/* How long the session has not been running, for the idle line below. */
static ULONGLONG g_not_running_since;
static unsigned  g_not_running_bucket;

/* Returns 0 if the session is gone for good. */
static int pump_events(void) {
    XrEventDataBuffer ev;
    for (;;) {
        XrResult r;
        memset(&ev, 0, sizeof(ev));
        ev.type = XR_TYPE_EVENT_DATA_BUFFER;
        r = pfn_PollEvent(g_inst, &ev);
        if (r == XR_EVENT_UNAVAILABLE) return 1;
        if (XR_FAILED(r)) return 0;

        if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
            const XrEventDataSessionStateChanged *s =
                (const XrEventDataSessionStateChanged *)&ev;
            g_log("  xr: session state -> %s (tick %llu ms)\r\n",
                  xr_state_name(s->state),
                  (unsigned long long)GetTickCount64());
            if (s->state!=g_state) dg_xr_radial_invalidate();
            g_state = s->state;
            if (g_state == XR_SESSION_STATE_READY) {
                XrSessionBeginInfo bi;
                memset(&bi, 0, sizeof(bi));
                bi.type = XR_TYPE_SESSION_BEGIN_INFO;
                bi.primaryViewConfigurationType =
                    XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                if (g_input_ready && !g_input_attached) {
                    XrSessionActionSetsAttachInfo ai;
                    ai.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
                    ai.next = NULL; ai.countActionSets = 1;
                    ai.actionSets = &g_action_set;
                    if (XR_FAILED(pfn_AttachActionSets(g_sess, &ai))) {
                        g_log("  xr: controller action attach failed; continuing without it\r\n");
                        g_input_ready = 0;
                    } else g_input_attached = 1;
                }
                if (XR_FAILED(pfn_BeginSession(g_sess, &bi))) return 0;
                g_session_running = 1;
                g_log("  xr: session begun\r\n");
            } else if (g_state == XR_SESSION_STATE_STOPPING) {
                if (g_session_running && pfn_EndSession(g_sess)!=XR_SUCCESS) return 0;
                g_session_running = 0;
                g_log("  xr: session stopping\r\n");
            } else if (g_state == XR_SESSION_STATE_EXITING ||
                       g_state == XR_SESSION_STATE_LOSS_PENDING) {
                g_log("  xr: session exiting\r\n");
                return 0;
            }
        } else if (g_presence_enabled &&
                   ev.type == XR_TYPE_EVENT_DATA_USER_PRESENCE_CHANGED_EXT) {
            const XrEventDataUserPresenceChangedEXT *p =
                (const XrEventDataUserPresenceChangedEXT *)&ev;
            g_user_present = p->isUserPresent ? 1 : 0;
        } else if (ev.type == XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING) {
            return 0;
        }
    }
}

static int action_pose_active(XrAction action, XrPath path, int *active_out) {
    XrActionStateGetInfo gi;
    XrActionStatePose state;
    memset(&gi, 0, sizeof(gi));
    gi.type = XR_TYPE_ACTION_STATE_GET_INFO;
    gi.action = action;
    gi.subactionPath = path;
    memset(&state, 0, sizeof(state));
    state.type = XR_TYPE_ACTION_STATE_POSE;
    if (XR_FAILED(pfn_GetActionStatePose(g_sess, &gi, &state))) return 0;
    *active_out = state.isActive != 0;
    return 1;
}

static int action_float(XrAction action, XrPath path, float *value,
                        int *active_out) {
    XrActionStateGetInfo gi;
    XrActionStateFloat state;
    memset(&gi, 0, sizeof(gi));
    gi.type = XR_TYPE_ACTION_STATE_GET_INFO;
    gi.action = action;
    gi.subactionPath = path;
    memset(&state, 0, sizeof(state));
    state.type = XR_TYPE_ACTION_STATE_FLOAT;
    if (XR_FAILED(pfn_GetActionStateFloat(g_sess, &gi, &state))) return 0;
    *active_out = state.isActive != 0;
    *value = state.isActive ? state.currentState : 0.0f;
    return 1;
}

static int action_boolean_active(XrAction action, XrPath path,
                          unsigned int *value, unsigned int *is_active) {
    XrActionStateGetInfo gi;
    XrActionStateBoolean state;
    memset(&gi, 0, sizeof(gi));
    gi.type = XR_TYPE_ACTION_STATE_GET_INFO;
    gi.action = action;
    gi.subactionPath = path;
    memset(&state, 0, sizeof(state));
    state.type = XR_TYPE_ACTION_STATE_BOOLEAN;
    if (XR_FAILED(pfn_GetActionStateBoolean(g_sess, &gi, &state))) return 0;
    *value = state.isActive && state.currentState;
    if (is_active) *is_active = state.isActive ? 1 : 0;
    return 1;
}

static int action_boolean(XrAction action, XrPath path, unsigned int *value)
{ return action_boolean_active(action,path,value,NULL); }

static int action_vector(XrAction action, XrPath path, float *x, float *y, unsigned int *active) {
    XrActionStateGetInfo gi;
    XrActionStateVector2f state;
    memset(&gi, 0, sizeof(gi));
    gi.type = XR_TYPE_ACTION_STATE_GET_INFO;
    gi.action = action;
    gi.subactionPath = path;
    memset(&state, 0, sizeof(state));
    state.type = XR_TYPE_ACTION_STATE_VECTOR2F;
    if (XR_FAILED(pfn_GetActionStateVector2f(g_sess, &gi, &state))) return 0;
    *active=state.isActive!=0;
    *x = state.isActive ? state.currentState.x : 0.0f;
    *y = state.isActive ? state.currentState.y : 0.0f;
    return 1;
}

static void load_trigger_thresholds(float *deadzone, float *fire) {
    LONG db = InterlockedCompareExchange(&g_trigger_deadzone_bits, 0, 0);
    LONG fr = InterlockedCompareExchange(&g_trigger_fire_bits, 0, 0);
    memcpy(deadzone, &db, sizeof(*deadzone));
    memcpy(fire, &fr, sizeof(*fire));
    if (!(*deadzone >= 0.0f && *deadzone < 1.0f &&
          *fire > *deadzone && *fire <= 1.0f)) {
        *deadzone = 0.10f;
        *fire = 0.55f;
    }
}

#include "dg_xr_m9.inl"
static void poll_controller(void) {
    xr_m9_haptic_drain();
    XrActiveActionSet active;
    XrActionsSyncInfo si;
    XrActionStateGetInfo gi;
    XrActionStateBoolean state;
    XrResult r;
    XrTime now;
    float deadzone, fire;
    int i;

    if (!g_input_ready || !g_input_attached) {
        clear_action_samples();
        return;
    }
    memset(&active, 0, sizeof(active));
    active.actionSet = g_action_set;
    memset(&si, 0, sizeof(si));
    si.type = XR_TYPE_ACTIONS_SYNC_INFO;
    si.countActiveActionSets = 1; si.activeActionSets = &active;
    r = pfn_SyncActions(g_sess, &si);
    /* Losing focus is expected while the user is in another app. */
    if (r == XR_SESSION_NOT_FOCUSED) {
        clear_action_samples();
        return;
    }
    if (XR_FAILED(r)) goto fail;

    load_trigger_thresholds(&deadzone, &fire);
    for (i = 0; i < 2; i++) {
        DG_XR_ACTION_SAMPLE *sample = &g_hand_action[i];
        unsigned int old_trigger = sample->trigger_click;
        unsigned int old_secondary = sample->secondary_button;
        unsigned int old_menu = sample->menu_button;
        int was_active = sample->trigger_active;
        int trigger_active, squeeze_active;
        if (!action_pose_active(g_grip_action, g_hand_runtime[i].path,
                                &sample->grip_active) ||
            !action_pose_active(g_aim_action, g_hand_runtime[i].path,
                                &sample->aim_active) ||
            !action_float(g_trigger_action, g_hand_runtime[i].path,
                          &sample->trigger, &trigger_active) ||
            !action_float(g_squeeze_action, g_hand_runtime[i].path,
                          &sample->squeeze, &squeeze_active) ||
            !action_vector(g_thumbstick_action, g_hand_runtime[i].path,
                           &sample->thumbstick_x, &sample->thumbstick_y, &sample->thumbstick_active) ||
            !action_boolean(g_thumbstick_click_action, g_hand_runtime[i].path,
                            &sample->thumbstick_click) ||
            !action_boolean_active(g_primary_action, g_hand_runtime[i].path,
                            &sample->primary_button,&sample->primary_active) ||
            !action_boolean(g_secondary_action, g_hand_runtime[i].path,
                            &sample->secondary_button) ||
            !action_boolean(g_menu_action, g_hand_runtime[i].path,
                            &sample->menu_button))
            goto fail;

        sample->trigger_active = trigger_active;
        if (!trigger_active) {
            sample->trigger_click = 0;
        } else if (!was_active) {
            sample->trigger_click = sample->trigger >= fire;
        } else {
            sample->trigger_click = dg_xr_trigger_hysteresis(
                sample->trigger, deadzone, fire, old_trigger);
            if (!old_trigger && sample->trigger_click)
                sample->trigger_press_seq = ++g_action_edge_seq;
            else if (old_trigger && !sample->trigger_click)
                sample->trigger_release_seq = ++g_action_edge_seq;
        }
        sample->squeeze_click = squeeze_active &&
            dg_xr_trigger_hysteresis(sample->squeeze, deadzone, fire,
                                     sample->squeeze_click);

        /* Raw A remains available to menus. Gameplay ownership is decided
           below together with the recenter gesture, not by a second writer. */

        /* In theater, hamburger moves only the quad. It must not also send
           START or change the gameplay reference used by the arms. */
        if (i == HAND_LEFT) {
            unsigned route = dg_xr_menu_route(old_menu, sample->menu_button,
                InterlockedCompareExchange(&g_screen_req, 0, 0) != 0);
            if (route == 1) InterlockedIncrement64(&g_menu_press_seq);
            else if (route == 2) {
                g_screen_menu_seq++;
                g_log("  xr: theater recenter requested by left menu button\r\n");
            }
        }

        /* B on the right hand, Y on the left. Counted rather than acted on:
           this file owns the input and has no business deciding what a button
           means, and the arm's calibration lives two layers away on a thread
           that polls. The same inactive-reads-zero rule as above applies, so a
           button held across a focus loss comes back as a fresh press.
           Gated on the same hand's trigger: a re-zero is an aiming gesture,
           and the resting thumb bumps this button while walking - see
           dg_xr_secondary_ignored for the session that proved it. */
        if (dg_xr_secondary_counts(old_secondary, sample->secondary_button,
                                   sample->trigger_click))
            InterlockedIncrement(&g_secondary_presses[i]);
        else if (!old_secondary && sample->secondary_button)
            InterlockedIncrement(&g_secondary_ignored[i]);
    }

    memset(&gi, 0, sizeof(gi)); gi.type = XR_TYPE_ACTION_STATE_GET_INFO;
    gi.action = g_recenter_action;
    memset(&state, 0, sizeof(state)); state.type = XR_TYPE_ACTION_STATE_BOOLEAN;
    if (XR_FAILED(pfn_GetActionStateBoolean(g_sess, &gi, &state))) goto fail;
    now = (XrTime)GetTickCount();
    if (controller_buttons_at((g_hand_action[HAND_RIGHT].primary_active?1:0) |
                             (state.isActive?2:0),g_hand_action[HAND_RIGHT].primary_button,
                             state.isActive && state.currentState,(uint32_t)now)
        & DG_XR_BUTTON_RECENTER) {
            InterlockedExchange(&g_recenter_cause, DG_XR_CAUSE_CONTROLLER);
            InterlockedExchange(&g_recenter_req, 1);
    }
    return;
fail:
    if (!g_input_failed) {
        g_input_failed = 1;
        g_log("  xr: controller input failed; continuing without it\r\n");
    }
    g_input_ready = 0;
    clear_action_samples();
}

static void capture_reference(const XrPosef *p) {
    /* Heading about XR +Y in a gravity-aligned space. */
    double yaw = atan2(2.0 * ((double)p->orientation.w * p->orientation.y +
                              (double)p->orientation.x * p->orientation.z),
                       1.0 - 2.0 * ((double)p->orientation.y * p->orientation.y +
                                    (double)p->orientation.z * p->orientation.z));
    g_ref_qy = sin(yaw * 0.5);
    g_ref_qw = cos(yaw * 0.5);
    g_ref_px = p->position.x;
    g_ref_py = p->position.y;
    g_ref_pz = p->position.z;
    g_have_ref = 1;
    /* A fresh reference IS the current facing: the software turn's offset
       is absorbed into it and starts over at zero. */
    g_turn_offset_rad_rt = 0.0;
    turn_offset_publish(0.0);
    /* The arm watches this count and restarts its own calibration when it
       moves, so one held button re-nulls the camera and the arm together. */
    dg_xr_radial_invalidate();
    InterlockedIncrement(&g_recenter_count);
    g_log("  xr: recentred, heading %.1f deg (%s)\r\n",
          yaw * 180.0 / PI, capture_cause_name(g_recenter_cause));
}

/* Consume the stick's turn demand into the reference, per pose sample, with
   wall-clock dt so the rate is degrees per second regardless of frame rate.
   A stall clamps at 250 ms: a hitch must cost a beat, not become a spin. */
static void apply_turn(const XrPosef *head)
{
    ULONGLONG now = GetTickCount64();
    LONG r = g_state==XR_SESSION_STATE_FOCUSED ? turn_rate_at((uint32_t)now) : 0;
    double dt = g_turn_prev_ms ? (double)(now - g_turn_prev_ms) / 1000.0
                               : 0.0;
    g_turn_prev_ms = now;
    if (r == 0 || dt <= 0.0) return;
    if (dt > 0.25) dt = 0.25;
    {
        double delta = (double)r / 1000.0 * dt * PI / 180.0;
        double q[2], p[2];
        q[0] = g_ref_qy; q[1] = g_ref_qw;
        p[0] = g_ref_px; p[1] = g_ref_pz;
        dg_xr_turn_step(q, p, (double)head->position.x,
                        (double)head->position.z, delta);
        g_ref_qy = q[0]; g_ref_qw = q[1];
        g_ref_px = p[0]; g_ref_pz = p[1];
        g_turn_offset_rad_rt += delta;
        turn_offset_publish(g_turn_offset_rad_rt);
    }
}

static void relative_pose(const XrPosef *pose, DG_XR_POSE *out) {
    double qref_inv[4], qcur[4], qd[4], dp[3], rel[3];
    qcur[0] = pose->orientation.x; qcur[1] = pose->orientation.y;
    qcur[2] = pose->orientation.z; qcur[3] = pose->orientation.w;
    qref_inv[0] = 0.0; qref_inv[1] = -g_ref_qy;
    qref_inv[2] = 0.0; qref_inv[3] = g_ref_qw;
    qmul(qref_inv, qcur, qd);
    dp[0] = pose->position.x - g_ref_px;
    dp[1] = pose->position.y - g_ref_py;
    dp[2] = pose->position.z - g_ref_pz;
    qrot(qref_inv, dp, rel);
    dg_xr_head_to_pose(qd[0], qd[1], qd[2], qd[3], rel[0], rel[1], rel[2],
                       &g_cfg, out);
}

static void view_fov(const XrFovf *in, DG_PROJ_FOV *out) {
    out->left = (double)in->angleLeft;
    out->right = (double)in->angleRight;
    out->up = (double)in->angleUp;
    out->down = (double)in->angleDown;
}

static void raw_pose(const XrPosef *in, DG_XR_RAW_POSE *out) {
    out->qx = in->orientation.x; out->qy = in->orientation.y;
    out->qz = in->orientation.z; out->qw = in->orientation.w;
    out->px = in->position.x; out->py = in->position.y; out->pz = in->position.z;
}

static void current_reference(DG_XR_RAW_POSE *out) {
    memset(out, 0, sizeof(*out));
    out->qy = g_ref_qy;
    out->qw = g_ref_qw;
    out->px = g_ref_px;
    out->py = g_ref_py;
    out->pz = g_ref_pz;
}

static void sample_hand_pose(XrSpace space, int active, unsigned int hand,
                             unsigned int kind, XrTime when, uint64_t seq,
                             DG_XR_HAND_POSE *out) {
    XrSpaceLocation loc;
    DG_XR_RAW_POSE raw, reference;
    DG_XR_REL_POSE relative;
    XrSpaceLocationFlags flags = 0;
    int located = 0;
    memset(&loc, 0, sizeof(loc));
    loc.type = XR_TYPE_SPACE_LOCATION;
    if (space && active &&
        XR_SUCCEEDED(pfn_LocateSpace(space, g_local_space, when, &loc))) {
        flags = loc.locationFlags;
        raw_pose(&loc.pose, &raw);
        current_reference(&reference);
        dg_xr_pose_relative(&reference, &raw, &relative);
        located = 1;
    }
    dg_xr_hand_pose_sample(
        out, hand, kind, located ? &raw : NULL, located ? &relative : NULL,
        (flags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0,
        (flags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0,
        (flags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0,
        (flags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) != 0,
        active, seq, (int64_t)when);
}

static void sample_hand(unsigned int hand, int index, XrTime when,
                        uint64_t seq, DG_XR_HAND *out) {
    const DG_XR_ACTION_SAMPLE *action = &g_hand_action[index];
    memset(out, 0, sizeof(*out));
    out->hand = hand;
    sample_hand_pose(g_hand_runtime[index].grip_space, action->grip_active,
                     hand, DG_XR_POSE_GRIP, when, seq, &out->grip);
    sample_hand_pose(g_hand_runtime[index].aim_space, action->aim_active,
                     hand, DG_XR_POSE_AIM, when, seq, &out->aim);
    out->trigger_value = action->trigger;
    out->trigger_click = action->trigger_click;
    out->trigger_press_seq = action->trigger_press_seq;
    out->trigger_release_seq = action->trigger_release_seq;
    out->squeeze_value = action->squeeze;
    out->squeeze_click = action->squeeze_click;
    out->thumbstick_x = action->thumbstick_x;
    out->thumbstick_y = action->thumbstick_y;
    out->thumbstick_active = action->thumbstick_active;
    out->thumbstick_click = action->thumbstick_click;
    out->primary_button = action->primary_button;
    out->secondary_button = action->secondary_button;
    out->menu_button = action->menu_button;
    out->haptic_output = (g_input_ready && g_input_attached) ?
        (uintptr_t)g_haptic_action : (uintptr_t)0;
}

static void union_fov(const XrView *views, DG_PROJ_FOV *out);

static void sample_pose(XrTime when, const XrView *views, int views_valid) {
    XrSpaceLocation loc;
    const XrSpaceLocationFlags need =
        XR_SPACE_LOCATION_ORIENTATION_VALID_BIT |
        XR_SPACE_LOCATION_POSITION_VALID_BIT |
        XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT |
        XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
    DG_XR_FRAME frame;

    memset(&loc, 0, sizeof(loc));
    loc.type = XR_TYPE_SPACE_LOCATION;
    if (XR_FAILED(pfn_LocateSpace(g_view_space, g_local_space, when, &loc)) ||
        (loc.locationFlags & need) != need) {
        if (g_tracking_was_valid && !g_tracking_lost_logged) {
            g_log("  xr: tracking lost\r\n");
            g_tracking_lost_logged = 1;
        }
        g_tracking_was_valid = 0;
        g_tracking_since = 0;
        InterlockedIncrement(&g_st_invalid);
        memset(&frame, 0, sizeof(frame));
        publish(&frame, 0, 0);
        return;
    }

    if (!g_tracking_was_valid) {
        if (g_tracking_lost_logged) g_log("  xr: tracking regained\r\n");
        g_tracking_lost_logged = 0;
        g_tracking_was_valid = 1;
        g_tracking_since = GetTickCount64();
    }

    {
        LONG request = g_recenter_req;
        if (request && g_recenter_cause == DG_XR_CAUSE_CONTROLLER &&
            !controller_recenter_current((uint32_t)GetTickCount())) {
            InterlockedExchange(&g_recenter_req,0);
            request = 0;
        }
        int worn = g_presence_enabled ? g_user_present :
            (g_state == XR_SESSION_STATE_FOCUSED);
        unsigned tracked_ms = (unsigned)(GetTickCount64() - g_tracking_since);
        if (dg_xr_should_capture(g_have_ref, request, worn, tracked_ms)) {
            if (request) InterlockedCompareExchange(&g_recenter_req, 0, 1);
            if (!request) InterlockedExchange(&g_recenter_cause, DG_XR_CAUSE_INITIAL);
            capture_reference(&loc.pose);
        }
    }

    /* Gating the capture creates a state that could not exist before: valid
       tracking with no reference yet. Falling through would subtract a zero
       reference, i.e. hand the camera the headset's ABSOLUTE position in the
       runtime's LOCAL space - metres, scaled by 1000 - and rotate by a zero
       quaternion. The no-pose path is the well-worn one (2.19) and leaves the
       camera alone, so take it, and say why once so "head tracking is dead"
       is diagnosable from the log alone. */
    if (!g_have_ref) {
        if (!g_waiting_logged) {
            g_log("  xr: waiting to capture the seated origin - %s\r\n",
                  g_presence_enabled ? "headset not reported as worn"
                                     : "need focus plus 500 ms of tracking");
            g_waiting_logged = 1;
        }
        InterlockedIncrement(&g_st_invalid);
        memset(&frame, 0, sizeof(frame));
        publish(&frame, 0, 0);
        return;
    }

    /* The stick's turn, before any pose is mapped this sample: everything
       below - head, eyes, hands - sees one coherent, freshly turned
       reference. */
    apply_turn(&loc.pose);

    memset(&frame, 0, sizeof(frame));
    g_hand_sample_seq++;
    relative_pose(&loc.pose, &frame.head);
    /* The head's LOCAL pose as well as its origin-relative one: the stereo
       bisect can render both eyes from the head, and an image rendered from
       the head must be submitted with the head's pose or it smears. */
    raw_pose(&loc.pose, &frame.head_raw);
    if (views_valid) {
        relative_pose(&views[0].pose, &frame.eye[0].pose);
        relative_pose(&views[1].pose, &frame.eye[1].pose);
        view_fov(&views[0].fov, &frame.eye[0].fov);
        view_fov(&views[1].fov, &frame.eye[1].fov);
        raw_pose(&views[0].pose, &frame.eye[0].raw);
        raw_pose(&views[1].pose, &frame.eye[1].raw);
        union_fov(views, &frame.union_fov);
    }
    sample_hand(DG_XR_HAND_LEFT, HAND_LEFT, when, g_hand_sample_seq,
                &frame.left_hand);
    sample_hand(DG_XR_HAND_RIGHT, HAND_RIGHT, when, g_hand_sample_seq,
                &frame.right_hand);
    /* Logged once: if the two eyes are not about 55-75 apart in game units,
       the metres->units scale is wrong and no amount of projection work will
       make the pair fuse. Cheap to print, and it settles a whole class of
       "stereo looks wrong" before anyone opens a matrix. */
    if (views_valid && !g_sep_logged) {
        double dx = frame.eye[1].pose.tx - frame.eye[0].pose.tx;
        double dy = frame.eye[1].pose.ty - frame.eye[0].pose.ty;
        double dz = frame.eye[1].pose.tz - frame.eye[0].pose.tz;
        g_log("  xr: eye separation %.1f game units (1 unit = 1 mm)\r\n",
              sqrt(dx * dx + dy * dy + dz * dz));
        g_sep_logged = 1;
    }
    InterlockedIncrement(&g_st_valid);
    publish(&frame, 1, views_valid);
}

static void union_fov(const XrView *views, DG_PROJ_FOV *out) {
    double left = 0.0, right = 0.0, up = 0.0, down = 0.0;
    int i;
    for (i = 0; i < 2; i++) {
        double v;
        v = fabs((double)views[i].fov.angleLeft);  if (v > left)  left = v;
        v = fabs((double)views[i].fov.angleRight); if (v > right) right = v;
        v = fabs((double)views[i].fov.angleUp);    if (v > up)    up = v;
        v = fabs((double)views[i].fov.angleDown);  if (v > down)  down = v;
    }
    out->left = -left; out->right = right; out->up = up; out->down = -down;
}

/* Returns a short reason for leaving the frame loop, or NULL for a normal stop.
   Every exit names itself: a loop with five ways out that logged one
   indistinguishable "thread stopped" afterwards is exactly the position the
   first stereo session left us in - the picture was gone and the log could not
   say why. */
static void log_xr_fail(const char *call, XrResult r) {
    InterlockedIncrement(&g_st_errors);
    g_log("  xr: %s failed (%d)\r\n", call, (int)r);
}

/* A pose whose quaternion is not unit-length is rejected by xrEndFrame with
   XR_ERROR_POSE_INVALID, and that error kills the WHOLE frame, not the one bad
   view. Observed live: toggling stereo on reused a store captured in mono,
   where the pose is deliberately zeroed - and an all-zero quaternion is
   exactly this failure. Never hand the runtime a pose we have not checked. */
static int raw_pose_usable(const DG_XR_RAW_POSE *p) {
    double n = p->qx * p->qx + p->qy * p->qy + p->qz * p->qz + p->qw * p->qw;
    return n > 0.98 && n < 1.02;
}

#include "dg_capture_transaction.h"
typedef struct {
    uint32_t idx[2];
    int stereo;
    uint32_t width, height;
    DG_XR_RAW_POSE raw[2];
    DG_PROJ_FOV fov[2];
    DG_MODEL_ARM model_arm[2];
} DG_SUBMIT_CAPTURE;

static void submit_capture_lock(void *unused) {
    (void)unused; EnterCriticalSection(&g_cap_cs);
}
static void submit_capture_unlock(void *unused) {
    (void)unused; LeaveCriticalSection(&g_cap_cs);
}
static int submit_capture_valid(void *ctx, int eye) {
    DG_SUBMIT_CAPTURE *cap = (DG_SUBMIT_CAPTURE *)ctx;
    D3D11_TEXTURE2D_DESC src, dst;
    unsigned reason = 0;
    g_cd.checked[eye] = 1;
    g_cd.store_valid[eye] = g_store[eye].valid;
    g_cd.capture_id[eye] = g_store[eye].capture_id;
    g_cd.capture_ms[eye] = g_store[eye].capture_ms;
    g_cd.idx[eye] = cap->idx[eye];
    g_cd.images[eye] = g_image_count[eye];
    if (!g_store[eye].valid || !g_store[eye].texture) reason |= CR_STORE;
    if (!g_images[eye] || cap->idx[eye] >= g_image_count[eye] ||
        !g_images[eye][cap->idx[eye]].texture) reason |= CR_INDEX;
    if (reason) { g_cd.reason[eye] = reason; return 0; }
    if (cap->stereo && !raw_pose_usable(&g_store[eye].raw)) reason |= CR_POSE;
    /* A backbuffer resize must not send an incompatible CopyResource to an
       existing XR swapchain. Rebuilding swapchains is a separate lifecycle. */
    g_store[eye].texture->lpVtbl->GetDesc(g_store[eye].texture, &src);
    g_images[eye][cap->idx[eye]].texture->lpVtbl->GetDesc(
        g_images[eye][cap->idx[eye]].texture, &dst);
    g_cd.src[eye] = src; g_cd.dst[eye] = dst;
    if (src.Width != dst.Width || src.Height != dst.Height ||
        (eye && (dst.Width != cap->width || dst.Height != cap->height))) reason |= CR_SIZE;
    if (src.MipLevels != dst.MipLevels) reason |= CR_MIPS;
    if (src.ArraySize != dst.ArraySize) reason |= CR_ARRAY;
    if (src.SampleDesc.Count != dst.SampleDesc.Count ||
        src.SampleDesc.Quality != dst.SampleDesc.Quality) reason |= CR_SAMPLE;
    if (!dg_capture_format_compatible(src.Format, dst.Format)) reason |= CR_FORMAT;

    g_cd.legacy_format[eye] = !(src.Format == dst.Format ||
        (src.Format == DXGI_FORMAT_B8G8R8A8_UNORM &&
         dst.Format == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB));
    if (g_cd.legacy_format[eye]) capture_diag_inc(CD_LEGACY_FORMAT);
    if (!eye) { cap->width = dst.Width; cap->height = dst.Height; }
    g_cd.reason[eye] = reason;
    return reason == 0;
}
static void submit_capture_copy(void *ctx, int eye) {
    DG_SUBMIT_CAPTURE *cap = (DG_SUBMIT_CAPTURE *)ctx;
    cap->raw[eye] = g_store[eye].raw;
    cap->fov[eye] = g_store[eye].fov;
    cap->model_arm[eye] = g_store[eye].model_arm;
    g_ctx->lpVtbl->CopyResource(g_ctx,
        (ID3D11Resource *)g_images[eye][cap->idx[eye]].texture,
        (ID3D11Resource *)g_store[eye].texture);
    if (!cap->stereo) pixel_probe_destination(g_images[eye][cap->idx[eye]].texture, eye);
}
static const DG_CAPTURE_SNAPSHOT_OPS g_submit_capture_ops = {
    submit_capture_lock, submit_capture_unlock,
    submit_capture_valid, submit_capture_copy
};

/* Own all waited destinations until the coherent copy has been queued. A
   failed acquire releases earlier waited eyes. A failed wait must instead
   leave its acquired/unwaited image for session teardown, never release it.
   The tests call this production helper with real D3D textures and mocked XR. */
static int submit_capture_images(DG_SUBMIT_CAPTURE *capture, const char **fatal) {
    XrSwapchainImageAcquireInfo ai = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    XrSwapchainImageWaitInfo wi = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    XrSwapchainImageReleaseInfo ri = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    int eye, waited = 0, copied = 0, eyes = capture->stereo ? 2 : 1;
    XrResult r;
    *fatal = NULL;
    wi.timeout = XR_INFINITE_DURATION;
    for (eye = 0; eye < eyes; ++eye) {
        g_cd.attempted_acquire[eye] = 1;
        r = pfn_AcquireImage(g_swap[eye], &ai, &capture->idx[eye]);
        g_cd.acquire[eye] = r;
        if (XR_FAILED(r)) { capture_diag_inc(CD_ACQUIRE); break; }
        g_cd.attempted_wait[eye] = 1;
        r = pfn_WaitImage(g_swap[eye], &wi);
        g_cd.wait[eye] = r;
        if (XR_FAILED(r) || r == XR_TIMEOUT_EXPIRED) {
            capture_diag_inc(CD_WAIT);
            log_xr_fail("xrWaitSwapchainImage", r);
            *fatal = "xrWaitSwapchainImage";
            break;
        }
        ++waited;
    }
    if (waited == eyes) {
        capture_diag_snapshot_begin();
        copied = dg_capture_snapshot(&g_submit_capture_ops, capture, eyes);
        if (!copied) { capture_diag_inc(CD_SNAPSHOT); g_cd.gate = 5; }
    }
    if (copied) pixel_probe_before_release();
    for (eye = 0; eye < waited; ++eye) {
        g_cd.attempted_release[eye] = 1;
        r = pfn_ReleaseImage(g_swap[eye], &ri);
        g_cd.release[eye] = r;
        if (XR_FAILED(r)) {
            capture_diag_inc(CD_RELEASE);
            log_xr_fail("xrReleaseSwapchainImage", r);
            *fatal = "xrReleaseSwapchainImage";
        }
    }
    return copied && !*fatal;
}

#include "dg_xr_radar.inl"
static XrSwapchain health_radar_swap(void) { return g_radar_swap; }

static const char *xr_frame_loop(void) {
    while (g_run) {
        XrFrameWaitInfo fwi;
        XrFrameState fs;
        XrFrameBeginInfo fbi;
        XrFrameEndInfo fei;
        XrCompositionLayerProjectionView pview[2];
        XrCompositionLayerProjection proj;
        XrCompositionLayerQuad quad, radial_quad, radar_quad, health_quad;
        const XrCompositionLayerBaseHeader *layers[10];
        XrCompositionLayerQuad grip_quads[6];
        XrView view[2];
        DG_XR_FRAME published;
        DG_MODEL_ARM submitted_arm={0};
        int published_status;
        int views_valid = 0;
        int screen_on, radar_ready;
        XrResult r;

        capture_diag_begin();
        pixel_probe_tick(0, 0, XR_SUCCESS);
        if (!pump_events()) return "session lost or exiting";

        if (!g_session_running) {
            /* Parked by the runtime (STOPPING without a READY yet). Say so
               after 5 s and then every 30 s, with the state we are in, so a
               dark headset can be read from the log: this thread alive and
               waiting, versus a runtime that never handed the session
               back. Logging only - the wait itself is unchanged. */
            ULONGLONG now = GetTickCount64();
            if (!g_not_running_since) {
                g_not_running_since = now;
                g_not_running_bucket = 0;
            } else {
                ULONGLONG idle = now - g_not_running_since;
                unsigned bucket = idle < 5000 ? 0 : (unsigned)(idle / 30000) + 1;
                if (bucket > g_not_running_bucket) {
                    g_not_running_bucket = bucket;
                    g_log("  xr: session not running for %lu s - state %s; "
                          "waiting for the runtime to hand it back "
                          "(headset asleep or taken off? Link paused?)"
                          " (tick %llu ms)\r\n",
                          (unsigned long)(idle / 1000), xr_state_name(g_state),
                          (unsigned long long)now);
                }
            }
            capture_diag_emit();
            Sleep(20);
            continue;
        }
        if (g_not_running_since) {
            g_log("  xr: session running again after %lu s (tick %llu ms)\r\n",
                  (unsigned long)((GetTickCount64() - g_not_running_since) / 1000),
                  (unsigned long long)GetTickCount64());
            g_not_running_since = 0;
        }

        memset(&fwi, 0, sizeof(fwi)); fwi.type = XR_TYPE_FRAME_WAIT_INFO;
        memset(&fs,  0, sizeof(fs));  fs.type  = XR_TYPE_FRAME_STATE;
        r = pfn_WaitFrame(g_sess, &fwi, &fs);
        if (XR_FAILED(r)) { log_xr_fail("xrWaitFrame", r); return "xrWaitFrame"; }
        capture_diag_inc(CD_FRAME);
        g_cd.should_render = fs.shouldRender;
        g_cd.predicted_time = fs.predictedDisplayTime;
        g_cd.predicted_period = fs.predictedDisplayPeriod;

        memset(&fbi, 0, sizeof(fbi)); fbi.type = XR_TYPE_FRAME_BEGIN_INFO;
        r = pfn_BeginFrame(g_sess, &fbi);
        if (XR_FAILED(r)) { log_xr_fail("xrBeginFrame", r); return "xrBeginFrame"; }

        /* Locate before publishing the pose: the seqlock record for this
           predicted display time carries both the pose and this frame's mono
           union FOV. */
        if (fs.shouldRender &&
            (g_state == XR_SESSION_STATE_FOCUSED ||
             g_state == XR_SESSION_STATE_VISIBLE)) {
            XrViewLocateInfo vli;
            XrViewState vs;
            uint32_t got = 0;
            const XrViewStateFlags need_v =
                XR_VIEW_STATE_ORIENTATION_VALID_BIT | XR_VIEW_STATE_POSITION_VALID_BIT;
            memset(&vli, 0, sizeof(vli));
            vli.type = XR_TYPE_VIEW_LOCATE_INFO;
            vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
            vli.displayTime = fs.predictedDisplayTime;
            vli.space = g_local_space;
            memset(&vs, 0, sizeof(vs)); vs.type = XR_TYPE_VIEW_STATE;
            memset(view, 0, sizeof(view));
            view[0].type = view[1].type = XR_TYPE_VIEW;
            if (XR_SUCCEEDED(pfn_LocateViews(g_sess, &vli, &vs, 2, &got, view)) &&
                got == 2 && (vs.viewStateFlags & need_v) == need_v) {
                views_valid = 1;
            }
        }

        InterlockedIncrement(&g_st_frames);
        if (g_state == XR_SESSION_STATE_FOCUSED ||
            g_state == XR_SESSION_STATE_VISIBLE) {
            poll_controller();
            sample_pose(fs.predictedDisplayTime, view, views_valid);
        } else {
            if (g_tracking_was_valid && !g_tracking_lost_logged) {
                g_log("  xr: tracking lost\r\n");
                g_tracking_lost_logged = 1;
            }
            g_tracking_was_valid = 0;
            g_tracking_since = 0;
            DG_XR_FRAME z; memset(&z, 0, sizeof(z)); publish(&z, 0, 0);
        }

        /* Zero layers is still the default and still legal - it is what keeps
           the no-device, no-capture and lost-tracking paths harmless. A layer
           is added only when everything needed for an honest one is present. */
        memset(&fei, 0, sizeof(fei));
        fei.type = XR_TYPE_FRAME_END_INFO;
        fei.displayTime = fs.predictedDisplayTime;
        fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
        fei.layerCount = 0;
        fei.layers = NULL;

        screen_on = InterlockedCompareExchange(&g_screen_req, 0, 0) != 0;
        if (screen_on != g_screen_shown) {
            drop_capture_stores();
            g_screen_shown = screen_on;
            g_screen_have_anchor = 0;
            InterlockedExchange(&g_screen_logged, 0);
        }
        if (screen_on && g_screen_have_anchor &&
            (g_screen_menu_seen != g_screen_menu_seq ||
             g_screen_recenter_seen != dg_xr_recenter_count() ||
             g_screen_anchor_dist !=
                 screen_clamp_m(g_cfg.theater_dist, 2.0)))
            g_screen_have_anchor = 0;   /* re-anchor: recentre or live dist */
        if (screen_on && !g_screen_have_anchor && views_valid) {
            XrSpaceLocation loc;
            const XrSpaceLocationFlags need =
                XR_SPACE_LOCATION_POSITION_VALID_BIT |
                XR_SPACE_LOCATION_ORIENTATION_VALID_BIT;
            memset(&loc, 0, sizeof(loc));
            loc.type = XR_TYPE_SPACE_LOCATION;
            if (XR_SUCCEEDED(pfn_LocateSpace(g_view_space, g_local_space,
                                             fs.predictedDisplayTime, &loc)) &&
                (loc.locationFlags & need) == need) {
                DG_XR_RAW_POSE head;
                head.qx = loc.pose.orientation.x;
                head.qy = loc.pose.orientation.y;
                head.qz = loc.pose.orientation.z;
                head.qw = loc.pose.orientation.w;
                head.px = loc.pose.position.x;
                head.py = loc.pose.position.y;
                head.pz = loc.pose.position.z;
                g_screen_anchor_dist = screen_clamp_m(g_cfg.theater_dist, 2.0);
                dg_xr_screen_pose(&head, g_screen_anchor_dist,
                                  &g_screen_anchor);
                g_screen_have_anchor = 1;
                g_screen_menu_seen = g_screen_menu_seq;
                g_screen_recenter_seen = dg_xr_recenter_count();
            }
            /* No valid head this frame: no anchor, and below that means the
               mono PROJECTION fallback, never a guessed quad pose. */
        }

        published_status = dg_xr_get_stereo(&published);
        g_cd.views_valid = views_valid;
        g_cd.published = published_status;
        g_cd.screen = screen_on;
        g_cd.anchor = g_screen_have_anchor;
        if (!fs.shouldRender) { g_cd.gate = 1; capture_diag_inc(CD_SKIP_RENDER); }
        else if (!views_valid) { g_cd.gate = 2; capture_diag_inc(CD_SKIP_VIEWS); }
        else if (!ensure_swapchains()) { g_cd.gate = 3; capture_diag_inc(CD_SKIP_SWAP); }
        else if ((published_status & 3) != 3) { g_cd.gate = 4; capture_diag_inc(CD_SKIP_POSE); }
        else {
            int i, stereo = g_cfg.stereo != 0;
            /* The screen is one flat frame for both eyes - the store the
               Present side is filling under the verdict is the MONO one, so
               the submission below must read it as mono whatever the stereo
               knob says. Restored the instant the verdict drops. */
            if (screen_on) stereo = 0;
            DG_SUBMIT_CAPTURE capture;
            int copied;
            const char *fatal;
            memset(&capture, 0, sizeof(capture));
            capture.stereo = stereo;
            {
                copied = submit_capture_images(&capture, &fatal);
                if (fatal) { capture_diag_emit(); return fatal; }
                if (copied && screen_on &&
                    g_screen_have_anchor) {
                    /* The theater: the captured mono frame on a core-OpenXR
                       quad at the FIXED anchor pose in LOCAL space. Height
                       follows the backbuffer's aspect so nothing is
                       stretched; the compositor reprojects the layer at HMD
                       refresh, so free look is judder-free even when the
                       game hitches. Submitted INSTEAD of the projection -
                       one screen, nothing behind it but the runtime's
                       bioscope-dark void. */
                    double w = screen_clamp_m(g_cfg.theater_width, 2.0);
                    memset(&quad, 0, sizeof(quad));
                    quad.type = XR_TYPE_COMPOSITION_LAYER_QUAD;
                    quad.space = g_local_space;
                    quad.eyeVisibility = XR_EYE_VISIBILITY_BOTH;
                    quad.subImage.swapchain = g_swap[0];
                    quad.subImage.imageArrayIndex = 0;
                    quad.subImage.imageRect.offset.x = 0;
                    quad.subImage.imageRect.offset.y = 0;
                    quad.subImage.imageRect.extent.width = (int32_t)capture.width;
                    quad.subImage.imageRect.extent.height = (int32_t)capture.height;
                    quad.pose.orientation.x = (float)g_screen_anchor.qx;
                    quad.pose.orientation.y = (float)g_screen_anchor.qy;
                    quad.pose.orientation.z = (float)g_screen_anchor.qz;
                    quad.pose.orientation.w = (float)g_screen_anchor.qw;
                    quad.pose.position.x = (float)g_screen_anchor.px;
                    quad.pose.position.y = (float)g_screen_anchor.py;
                    quad.pose.position.z = (float)g_screen_anchor.pz;
                    quad.size.width = (float)w;
                    quad.size.height = (float)(capture.height && capture.width
                        ? w * (double)capture.height / (double)capture.width : w * 0.5625);
                    layers[0] = (XrCompositionLayerBaseHeader *)&quad;
                    pixel_probe_quad(&quad);
                    fei.layerCount = 1; fei.layers = layers;
                    InterlockedExchange(&g_submitting, 1);
                    InterlockedIncrement(&g_st_screen_frames);
                    capture_diag_inc(CD_QUAD);
                    if (InterlockedCompareExchange(&g_screen_logged, 1, 0)
                            == 0)
                        g_log("  xr: theater quad up - %.2f x %.2f m at "
                              "%.2f m, LOCAL space, yaw-anchored\r\n",
                              quad.size.width, quad.size.height,
                              g_screen_anchor_dist);
                } else if (copied) {
                    submitted_arm=capture.model_arm[0];
                    if(stereo && capture.model_arm[1].ms>submitted_arm.ms)
                        submitted_arm=capture.model_arm[1];
                    for (i = 0; i < 2; i++) {
                        DG_PROJ_FOV fov;
                        memset(&pview[i], 0, sizeof(pview[i]));
                        pview[i].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
                        if (stereo) {
                            fov = capture.fov[i];
                            pview[i].pose.orientation.x = (float)capture.raw[i].qx;
                            pview[i].pose.orientation.y = (float)capture.raw[i].qy;
                            pview[i].pose.orientation.z = (float)capture.raw[i].qz;
                            pview[i].pose.orientation.w = (float)capture.raw[i].qw;
                            pview[i].pose.position.x = (float)capture.raw[i].px;
                            pview[i].pose.position.y = (float)capture.raw[i].py;
                            pview[i].pose.position.z = (float)capture.raw[i].pz;
                            pview[i].subImage.swapchain = g_swap[i];
                        } else {
                            fov = capture.fov[0];
                            pview[i].pose = view[i].pose;
                            pview[i].subImage.swapchain = g_swap[0];
                        }
                        pview[i].fov.angleLeft = (float)fov.left;
                        pview[i].fov.angleRight = (float)fov.right;
                        pview[i].fov.angleUp = (float)fov.up;
                        pview[i].fov.angleDown = (float)fov.down;
                        pview[i].subImage.imageArrayIndex = 0;
                        pview[i].subImage.imageRect.offset.x = 0;
                        pview[i].subImage.imageRect.offset.y = 0;
                        pview[i].subImage.imageRect.extent.width = (int32_t)capture.width;
                        pview[i].subImage.imageRect.extent.height = (int32_t)capture.height;
                    }
                    memset(&proj, 0, sizeof(proj)); proj.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION;
                    proj.space = g_local_space; proj.viewCount = 2; proj.views = pview;
                    layers[0] = (XrCompositionLayerBaseHeader *)&proj;
                    fei.layerCount = 1; fei.layers = layers;
                    InterlockedExchange(&g_submitting, 1);
                    capture_diag_inc(CD_PROJECTION);
                }
            }
        }
        /* Wrist radar (vr_radar_wrist, default off): decided and copied first,
           slotted in between the projection and the radial menu afterwards -
           the menu's own readiness test must still find the projection alone,
           so its capacity stays 2 and the radar takes the third slot. */
        radar_ready = radar_prepare(&fei,&radar_quad,fs.shouldRender,screen_on,&published,published_status,GetTickCount64(),&submitted_arm);
        radial_append(&fei,layers,2,&radial_quad,fs.shouldRender,screen_on,GetTickCount64());
        if (radar_ready) radar_insert(&fei,layers,4,&radar_quad);
        health_append(&fei,layers,4,&health_quad,fs.shouldRender,screen_on,views_valid,
                      fs.predictedDisplayTime,GetTickCount64());
        gripdbg_append(&fei,layers,10,grip_quads,fs.shouldRender,screen_on,views_valid,
                       fs.predictedDisplayTime,GetTickCount64());
        if (!fei.layerCount) capture_diag_inc(CD_ZERO);
        g_cd.end_attempted = 1;
        r = pfn_EndFrame(g_sess, &fei);
        g_cd.end = r;
        pixel_probe_tick(XR_SUCCEEDED(r) && g_state == XR_SESSION_STATE_FOCUSED &&
                         fei.layerCount && layers[0]->type == XR_TYPE_COMPOSITION_LAYER_QUAD,
                         1, r);
        if (XR_FAILED(r)) capture_diag_inc(CD_END_ERROR);
        capture_diag_emit();
        if (XR_FAILED(r)) { log_xr_fail("xrEndFrame", r); return "xrEndFrame"; }
    }
    return NULL;                        /* g_run cleared: an ordinary stop */
}

#define XR_MAX_RESTARTS 20

static DWORD WINAPI xr_thread(LPVOID unused) {
    int restarts = 0;
    (void)unused;

    while (g_run) {
        const char *why;

        if (!xr_init()) {
            int no_hmd = g_no_hmd;
            xr_teardown();
            /* Waiting for a headset is not a restart and must not spend one:
               the user may be minutes away from putting it on, and 20 tries at
               2 s would give up after forty seconds. Logged once, so a long
               wait costs one line rather than a screenful. */
            if (no_hmd) {
                if (!g_no_hmd_logged) {
                    g_log("  xr: no headset available yet - waiting for one "
                          "(put the headset on, or start Link)\r\n");
                    g_no_hmd_logged = 1;
                }
                Sleep(2000);
                continue;
            }
            /* A first failure means there is no usable runtime, which is a
               supported outcome: the caller carries on flat. A LATER one means
               a runtime that was working has gone, so it is worth waiting for
               it to come back. */
            if (restarts == 0) break;
            why = "session could not be rebuilt";
        } else {
            if (g_no_hmd_logged) {
                g_log("  xr: headset appeared\r\n");
                g_no_hmd_logged = 0;
            }
            why = xr_frame_loop();
            xr_teardown();
        }

        if (!g_run || !why) break;

        /* The runtime dropping a session must not cost the rest of the run.
           The game, its device and the Present hook are all still healthy, and
           the seated origin is process-sticky by design, so rebuilding the
           session puts the picture back where it was instead of leaving the
           headset dark until the next launch. */
        if (++restarts > XR_MAX_RESTARTS) {
            g_log("  xr: %s - giving up after %d restarts\r\n", why, restarts - 1);
            break;
        }
        g_log("  xr: %s - restarting session (%d/%d)\r\n",
              why, restarts, XR_MAX_RESTARTS);
        {   DG_XR_FRAME z; memset(&z, 0, sizeof(z)); publish(&z, 0, 0); }
        g_waiting_logged = 0;
        Sleep(2000);
    }

    {   DG_XR_FRAME z; memset(&z, 0, sizeof(z)); publish(&z, 0, 0); }
    g_log("  xr: thread stopped\r\n");
    InterlockedExchange(&g_started, 0);
    return 0;
}

int dg_xr_start(void (*log)(const char *fmt, ...)) {
    g_log = log;
    if (!InterlockedCompareExchange(&g_cap_cs_ready, 0, 0)) {
        InitializeCriticalSection(&g_cap_cs);
        InterlockedExchange(&g_cap_cs_ready, 1);
    }
    if (InterlockedCompareExchange(&g_started, 1, 0) != 0) return 1;  /* already up */
    InterlockedExchange(&g_submitting, 0);
    InterlockedExchange(&g_have_capture, 0);
    InterlockedExchange(&g_fov_mismatch_logged, 0);
    InterlockedExchange(&g_run, 1);
    InterlockedExchange(&g_st_frames, 0);
    InterlockedExchange(&g_st_valid, 0);
    InterlockedExchange(&g_st_invalid, 0);
    InterlockedExchange(&g_st_errors, 0);
    memset((void *)g_cd_count, 0, sizeof(g_cd_count));
    memset(&g_cd, 0, sizeof(g_cd));
    InterlockedExchange64(&g_cd_producer_failure, 0);
    pixel_probe_init();
    g_thread = CreateThread(NULL, 0, xr_thread, NULL, 0, NULL);
    if (!g_thread) { InterlockedExchange(&g_started, 0); return 0; }
    return 1;
}

void dg_xr_stop(void) {
    InterlockedExchange(&g_run, 0);
    if (g_thread) {
        WaitForSingleObject(g_thread, 3000);
        CloseHandle(g_thread);
        g_thread = NULL;
    }
}

#endif /* DG_XR_NO_RUNTIME */
