/* Adapted from MGS4-PCVR, Copyright 2026 Shiffo0, MIT. */
static void ls_rotate(const float q[4], const float v[3], float o[3]) {
    float x = q[0], y = q[1], z = q[2], w = q[3], t[3];
    t[0] = 2 * (y * v[2] - z * v[1]); t[1] = 2 * (z * v[0] - x * v[2]); t[2] = 2 * (x * v[1] - y * v[0]);
    o[0] = v[0] + w * t[0] + (y * t[2] - z * t[1]); o[1] = v[1] + w * t[1] + (z * t[0] - x * t[2]); o[2] = v[2] + w * t[2] + (x * t[1] - y * t[0]);
}
static float ls_dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static int ls_finite3(const float v[3]) { return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]); }
static int ls_finite4(const float v[4]) { return ls_finite3(v) && isfinite(v[3]); }
static void ls_quat_from_axes(const float X[3], const float Y[3], const float Z[3], float q[4]) {
    float tr = X[0] + Y[1] + Z[2], s;
    if (tr > 0) { s = sqrtf(tr + 1) * 2; q[3] = .25f * s; q[0] = (Y[2] - Z[1]) / s; q[1] = (Z[0] - X[2]) / s; q[2] = (X[1] - Y[0]) / s; }
    else if (X[0] > Y[1] && X[0] > Z[2]) { s = sqrtf(1 + X[0] - Y[1] - Z[2]) * 2; q[3] = (Y[2] - Z[1]) / s; q[0] = .25f * s; q[1] = (Y[0] + X[1]) / s; q[2] = (Z[0] + X[2]) / s; }
    else if (Y[1] > Z[2]) { s = sqrtf(1 + Y[1] - X[0] - Z[2]) * 2; q[3] = (Z[0] - X[2]) / s; q[0] = (Y[0] + X[1]) / s; q[1] = .25f * s; q[2] = (Z[1] + Y[2]) / s; }
    else { s = sqrtf(1 + Z[2] - X[0] - Y[1]) * 2; q[3] = (X[1] - Y[0]) / s; q[0] = (Z[0] + X[2]) / s; q[1] = (Z[1] + Y[2]) / s; q[2] = .25f * s; }
}
static int dg_menu_beam_pose(const float ro[3], const float rq[4], float start, float len, const float head[3], float q[4], float p[3]) {
    const float fwd0[3] = { 0, 0, -1 }; float d[3], c[3], z[3], x[3], t, n; int i;
    if (!ro || !rq || !head || !q || !p || !ls_finite3(ro) || !ls_finite4(rq) || !ls_finite3(head) || !(len > 0) || !isfinite(start)) return 0;
    ls_rotate(rq, fwd0, d);
    n = sqrtf(ls_dot(d, d)); if (!(n > 0.5f)) return 0;
    for (i = 0; i < 3; ++i) { d[i] /= n; c[i] = ro[i] + d[i] * (start + len * 0.5f); z[i] = head[i] - c[i]; }
    t = ls_dot(z, d); for (i = 0; i < 3; ++i) z[i] -= t * d[i];             /* towards the head, square to the beam */
    n = sqrtf(ls_dot(z, z));
    if (!(n > 1e-4f)) { float up[3] = { 0, 1, 0 }; if (fabsf(d[1]) > 0.9f) { up[1] = 0; up[2] = 1; } t = ls_dot(up, d); for (i = 0; i < 3; ++i) z[i] = up[i] - t * d[i]; n = sqrtf(ls_dot(z, z)); }
    for (i = 0; i < 3; ++i) z[i] /= n;
    x[0] = d[1] * z[2] - d[2] * z[1]; x[1] = d[2] * z[0] - d[0] * z[2]; x[2] = d[0] * z[1] - d[1] * z[0];   /* X = Y x Z, Y = beam */
    ls_quat_from_axes(x, d, z, q);
    memcpy(p, c, 12);
    return 1;
}
