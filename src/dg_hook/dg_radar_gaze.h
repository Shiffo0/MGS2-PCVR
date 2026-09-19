/* dg_radar_gaze.h - wrist radar: pose composition and the look-at-your-wrist
 * gate (2026-09-18). Pure: no OpenXR, no D3D, no clock of its own - the caller
 * passes the time, so radar_gaze_test.c can drive it from a table.
 *
 * Conventions are OpenXR's: right-handed, -Z forward, +Y up, metres, unit
 * quaternions (x,y,z,w). A quad layer's visible face is its +Z.
 *
 * There is no eye tracking, so "looking at the wrist" is the HEAD direction.
 * People look at a wrist mostly with their eyes; the head follows about half
 * way. The gaze ray is therefore the head's forward pitched DOWN by a fixed
 * angle (vr_radar_gaze_pitch) before it is compared with head->quad. */
#ifndef DG_RADAR_GAZE_H
#define DG_RADAR_GAZE_H
#include <math.h>

typedef struct { double qx, qy, qz, qw, px, py, pz; } DG_RADAR_POSE;

#define DG_RADAR_FACE_OPEN_DEG   50.0   /* quad normal vs quad->head, to open  */
#define DG_RADAR_FACE_CLOSE_DEG  65.0   /* ... past this it closes             */
#define DG_RADAR_GAZE_CLOSE_ADD  10.0   /* closes at gaze_deg + this           */
#define DG_RADAR_OPEN_HOLD_MS    120u
#define DG_RADAR_CLOSE_HOLD_MS   250u

typedef struct {
    int open;                   /* the verdict: 1 = show the layer */
    int timing;                 /* a transition is being held */
    unsigned long long since;   /* when that hold started (caller's clock, ms) */
} DG_RADAR_GATE;

static void dg_radar_rotate(const DG_RADAR_POSE *q, const double v[3], double out[3]) {
    /* v' = v + 2w(u x v) + 2 u x (u x v), u = (qx,qy,qz) */
    double ux = q->qx, uy = q->qy, uz = q->qz, w = q->qw;
    double cx = uy * v[2] - uz * v[1], cy = uz * v[0] - ux * v[2], cz = ux * v[1] - uy * v[0];
    double dx = uy * cz - uz * cy, dy = uz * cx - ux * cz, dz = ux * cy - uy * cx;
    out[0] = v[0] + 2.0 * (w * cx + dx);
    out[1] = v[1] + 2.0 * (w * cy + dy);
    out[2] = v[2] + 2.0 * (w * cz + dz);
}

/* out = parent * child: a pose given in the parent's frame, expressed in the
   frame the parent itself is given in. out may alias neither input. */
static void dg_radar_pose_compose(const DG_RADAR_POSE *parent, const DG_RADAR_POSE *child, DG_RADAR_POSE *out) {
    double p[3], r[3], n;
    p[0] = child->px; p[1] = child->py; p[2] = child->pz;
    dg_radar_rotate(parent, p, r);
    out->px = parent->px + r[0]; out->py = parent->py + r[1]; out->pz = parent->pz + r[2];
    out->qx = parent->qw * child->qx + parent->qx * child->qw + parent->qy * child->qz - parent->qz * child->qy;
    out->qy = parent->qw * child->qy - parent->qx * child->qz + parent->qy * child->qw + parent->qz * child->qx;
    out->qz = parent->qw * child->qz + parent->qx * child->qy - parent->qy * child->qx + parent->qz * child->qw;
    out->qw = parent->qw * child->qw - parent->qx * child->qx - parent->qy * child->qy - parent->qz * child->qz;
    n = sqrt(out->qx * out->qx + out->qy * out->qy + out->qz * out->qz + out->qw * out->qw);
    if (n > 1e-12) { out->qx /= n; out->qy /= n; out->qz /= n; out->qw /= n; }
    else { out->qx = out->qy = out->qz = 0.0; out->qw = 1.0; }
}

/* The marker's offset (metres) and rotation (degrees) as a pose in the grip
   frame. Rotation order is yaw, pitch, roll about the quad's OWN axes:
   q = Ry(ry) * Rx(rx) * Rz(rz). So ry/rx aim the quad's normal and rz then
   turns the picture about that normal, whatever the first two are. */
static void dg_radar_offset_pose(const double off[3], const double rot_deg[3], DG_RADAR_POSE *out) {
    const double k = 3.14159265358979323846 / 360.0;       /* half angle, radians */
    DG_RADAR_POSE x, y, z, yx;
    x.qx = sin(rot_deg[0] * k); x.qy = 0; x.qz = 0; x.qw = cos(rot_deg[0] * k); x.px = x.py = x.pz = 0;
    y.qx = 0; y.qy = sin(rot_deg[1] * k); y.qz = 0; y.qw = cos(rot_deg[1] * k); y.px = y.py = y.pz = 0;
    z.qx = 0; z.qy = 0; z.qz = sin(rot_deg[2] * k); z.qw = cos(rot_deg[2] * k); z.px = z.py = z.pz = 0;
    dg_radar_pose_compose(&y, &x, &yx);
    dg_radar_pose_compose(&yx, &z, out);
    out->px = off[0]; out->py = off[1]; out->pz = off[2];
}

static double dg_radar_angle_deg(const double a[3], const double b[3]) {
    double aa = a[0] * a[0] + a[1] * a[1] + a[2] * a[2], bb = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    double c;
    if (!(aa > 1e-12) || !(bb > 1e-12)) return 180.0;
    c = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / sqrt(aa * bb);
    if (c > 1.0) c = 1.0; else if (c < -1.0) c = -1.0;
    return acos(c) * (180.0 / 3.14159265358979323846);
}

/* Both poses in the SAME space. theta = pitched-down head forward vs
   head->quad; phi = quad normal vs quad->head. 0 = no usable geometry (quad
   inside the head, NaN): the caller must treat that as "not looking". */
static int dg_radar_gaze_angles(const DG_RADAR_POSE *head, const DG_RADAR_POSE *quad, double pitch_down_deg,
                                double *theta_deg, double *phi_deg) {
    const double rad = 3.14159265358979323846 / 180.0;
    double g[3], gw[3], n[3], nw[3], v[3], back[3], d2;
    g[0] = 0.0; g[1] = -sin(pitch_down_deg * rad); g[2] = -cos(pitch_down_deg * rad);
    n[0] = 0.0; n[1] = 0.0; n[2] = 1.0;
    dg_radar_rotate(head, g, gw);
    dg_radar_rotate(quad, n, nw);
    v[0] = quad->px - head->px; v[1] = quad->py - head->py; v[2] = quad->pz - head->pz;
    d2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    *theta_deg = *phi_deg = 180.0;
    if (!(d2 > 1e-4) || !(d2 < 1e4)) return 0;             /* under 1 cm or over 100 m: not a wrist */
    back[0] = -v[0]; back[1] = -v[1]; back[2] = -v[2];
    *theta_deg = dg_radar_angle_deg(gw, v);
    *phi_deg = dg_radar_angle_deg(nw, back);
    return 1;
}

static void dg_radar_gate_reset(DG_RADAR_GATE *g) { g->open = 0; g->timing = 0; g->since = 0; }

/* One step of the hysteresis. gaze_deg <= 0 disables the gate: always open,
   so the wrist placement can be tuned before the gate is. valid = 0 (no
   geometry) closes at once - a lost hand must not leave a quad hanging. */
static int dg_radar_gate_step(DG_RADAR_GATE *g, int valid, double theta_deg, double phi_deg,
                              double gaze_deg, unsigned long long now_ms) {
    int want;
    if (!(gaze_deg > 0.0)) { g->open = 1; g->timing = 0; return 1; }
    if (!valid) { dg_radar_gate_reset(g); return 0; }
    if (!g->open) want = theta_deg < gaze_deg && phi_deg < DG_RADAR_FACE_OPEN_DEG;
    else want = theta_deg <= gaze_deg + DG_RADAR_GAZE_CLOSE_ADD && phi_deg <= DG_RADAR_FACE_CLOSE_DEG;   /* NaN closes */
    if (want == g->open) { g->timing = 0; return g->open; }
    if (!g->timing) { g->timing = 1; g->since = now_ms; }
    if (now_ms - g->since >= (g->open ? DG_RADAR_CLOSE_HOLD_MS : DG_RADAR_OPEN_HOLD_MS)) {
        g->open = want; g->timing = 0;
    }
    return g->open;
}
#endif
