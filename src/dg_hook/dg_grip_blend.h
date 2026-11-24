#ifndef DG_GRIP_BLEND_H
#define DG_GRIP_BLEND_H
#include <string.h>
#include <math.h>
#include <stddef.h>

#define DG_GRIP_MS_MAX      2000
#define DG_GRIP_DT_MAX_S    0.1
#define DG_GRIP_DEFAULT_GRAB    180
#define DG_GRIP_DEFAULT_RELEASE 220
/* vr_grip_ms=off reproduces the transitions before this module: support hand
   75 ms linear, blade block an instant snap. */
#define DG_GRIP_LEGACY_SUPPORT_MS 75

/* Marker modes (DG_BRIDGE_CONFIG.grip_mode). 0 = key absent = default. */
#define DG_GRIP_MODE_DEFAULT 0
#define DG_GRIP_MODE_SMOOTH  1
#define DG_GRIP_MODE_LINEAR  2
#define DG_GRIP_MODE_OFF     3
/* Which hand transition asks (dg_grip_times_for). */
#define DG_GRIP_SUPPORT 0   /* left hand onto the weapon's support grip */
#define DG_GRIP_BRACE   1   /* left hand onto the blade while blocking */

typedef struct { int grab_ms, release_ms, linear; } DG_GRIP_TIMES;
typedef struct { long long time_ns; double phase; int have; } DG_GRIP_BLEND;

static double dg_grip_ease(double a)
{
    if (!(a > 0)) return 0;          /* also NaN */
    if (a >= 1) return 1;
    return a * a * (3 - 2 * a);
}

static double dg_grip_blend_output(double phase, const DG_GRIP_TIMES *t)
{
    if (!(phase > 0)) return 0;
    if (phase >= 1) return 1;
    return (t && t->linear) ? phase : dg_grip_ease(phase);
}

/* One step of `dt` seconds towards `want` (0/1). A duration of 0 (or below)
   jumps. dt that is not positive (0, negative, NaN) does not move the phase;
   more than DG_GRIP_DT_MAX_S counts as DG_GRIP_DT_MAX_S. */
static double dg_grip_blend_advance(double phase, int want, double dt, const DG_GRIP_TIMES *t)
{
    int dur;
    want = want ? 1 : 0;
    if (!(phase >= 0 && phase <= 1)) phase = 0;
    dur = t ? (want ? t->grab_ms : t->release_ms) : 0;
    if (dur <= 0) return (double)want;
    if (!(dt > 0)) return phase;
    if (dt > DG_GRIP_DT_MAX_S) dt = DG_GRIP_DT_MAX_S;
    phase += (want ? 1 : -1) * dt / (dur / 1000.0);
    if (phase < 0) phase = 0;
    if (phase > 1) phase = 1;
    return phase;
}

static void dg_grip_blend_reset(DG_GRIP_BLEND *b) { memset(b, 0, sizeof *b); }

/* Stateful form, for a hand whose transition has no latch of its own. The
   first call only takes the time; later calls advance by the time difference.
   A clock that goes back is taken as the new time without advancing. */
static double dg_grip_blend_step(DG_GRIP_BLEND *b, int want, long long now_ns,
                                 const DG_GRIP_TIMES *t)
{
    double dt = 0;
    if (!b->have) { b->have = 1; b->time_ns = now_ns; }
    else if (now_ns > b->time_ns) {
        dt = (double)(now_ns - b->time_ns) * 1e-9;
        b->time_ns = now_ns;
    } else if (now_ns < b->time_ns) b->time_ns = now_ns;
    b->phase = dg_grip_blend_advance(b->phase, want, dt, t);
    return dg_grip_blend_output(b->phase, t);
}

/* from + w*(to-from), exact at the ends: w <= 0 gives `from`, w >= 1 gives
   `to` bit for bit (the transitions this replaced assigned directly). */
static void dg_grip_mix3(const double from[3], const double to[3], double w, double out[3])
{
    int k;
    if (!(w > 0)) { double c[3]; memcpy(c, from, sizeof c); memcpy(out, c, sizeof c); return; }
    if (w >= 1) { double c[3]; memcpy(c, to, sizeof c); memcpy(out, c, sizeof c); return; }
    for (k = 0; k < 3; k++) out[k] = from[k] + w * (to[k] - from[k]);
}

/* The durations a marker mode gives one transition. */
static void dg_grip_times_for(int mode, int grab_ms, int release_ms, int which, DG_GRIP_TIMES *t)
{
    if (mode == DG_GRIP_MODE_OFF) {
        t->linear = 1;
        t->grab_ms = t->release_ms = which == DG_GRIP_SUPPORT ? DG_GRIP_LEGACY_SUPPORT_MS : 0;
        return;
    }
    if (mode != DG_GRIP_MODE_SMOOTH && mode != DG_GRIP_MODE_LINEAR) {
        t->grab_ms = DG_GRIP_DEFAULT_GRAB; t->release_ms = DG_GRIP_DEFAULT_RELEASE; t->linear = 0;
        return;
    }
    if (grab_ms < 0 || grab_ms > DG_GRIP_MS_MAX || release_ms < 0 || release_ms > DG_GRIP_MS_MAX) {
        t->grab_ms = DG_GRIP_DEFAULT_GRAB; t->release_ms = DG_GRIP_DEFAULT_RELEASE; t->linear = 0;
        return;
    }
    t->grab_ms = grab_ms; t->release_ms = release_ms; t->linear = mode == DG_GRIP_MODE_LINEAR;
}

/* Marker value of vr_grip_ms (n characters at s, no terminator needed):
     off            the transitions as they were before (A/B)
     G,R            grab G ms, release R ms, smoothstep (0..2000 each)
     G,R,linear     the same durations on a straight line
     G,R,smooth     explicit form of the default curve
   Returns 0 for anything else. */
static int dg_grip_parse(const char *s, size_t n, int *mode, int *grab_ms, int *release_ms)
{
    int v[2] = {0, 0}, i = 0, digits;
    size_t p = 0;
    if (n == 3 && (s[0] == 'o' || s[0] == 'O') && (s[1] == 'f' || s[1] == 'F') &&
        (s[2] == 'f' || s[2] == 'F')) {
        *mode = DG_GRIP_MODE_OFF; *grab_ms = *release_ms = 0; return 1;
    }
    for (i = 0; i < 2; i++) {
        digits = 0;
        while (p < n && s[p] >= '0' && s[p] <= '9') {
            v[i] = v[i] * 10 + (s[p] - '0'); p++;
            if (++digits > 4 || v[i] > DG_GRIP_MS_MAX) return 0;
        }
        if (!digits) return 0;
        if (i == 0) { if (p >= n || s[p] != ',') return 0; p++; }
    }
    *mode = DG_GRIP_MODE_SMOOTH;
    if (p < n) {
        size_t r = n - p;
        if (s[p] != ',') return 0;
        p++; r--;
        if (r == 6 && !_strnicmp(s + p, "linear", 6)) *mode = DG_GRIP_MODE_LINEAR;
        else if (r == 6 && !_strnicmp(s + p, "smooth", 6)) *mode = DG_GRIP_MODE_SMOOTH;
        else return 0;
    }
    *grab_ms = v[0]; *release_ms = v[1];
    return 1;
}

/* One LONG for cross-thread publication: mode | grab << 4 | release << 16. */
static long dg_grip_pack(int mode, int grab_ms, int release_ms)
{
    if (mode < 0 || mode > 3) mode = DG_GRIP_MODE_DEFAULT;
    if (grab_ms < 0 || grab_ms > DG_GRIP_MS_MAX) grab_ms = 0;
    if (release_ms < 0 || release_ms > DG_GRIP_MS_MAX) release_ms = 0;
    return (long)(mode | (grab_ms << 4) | (release_ms << 16));
}
static void dg_grip_unpack(long packed, int which, DG_GRIP_TIMES *t)
{
    dg_grip_times_for((int)(packed & 15), (int)((packed >> 4) & 0xFFF),
                      (int)((packed >> 16) & 0xFFF), which, t);
}
#endif
