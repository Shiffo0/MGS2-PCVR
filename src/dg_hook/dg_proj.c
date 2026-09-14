/* dg_proj.c - per-eye asymmetric projection math.
 *
 * The game uses row vectors.  In the live projection the horizontal NDC
 * coordinate is
 *
 *     ndc_x = (x / z) * m[0][0] + m[2][0]
 *
 * and the vertical expression has the matching m[1][1]/m[2][1] form.  The
 * depth mapping is in m[2][2] and m[3][2].  Consequently this helper changes
 * exactly four elements and never reconstructs, reads, or edits the depth
 * mapping.  That is WHY the eye override cannot inherit the near/far defect
 * of a rebuilt projection.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dg_proj.h"

#define DG_PROJ_PI 3.14159265358979323846

static float dg_proj_sign(float value) {
    /* A valid game projection has non-zero scale terms.  Treating an invalid
       zero as positive keeps this small helper deterministic and avoids
       inventing a new matrix convention when the input is unusable. */
    return value < 0.0f ? -1.0f : 1.0f;
}

static void dg_proj_terms(const MAT *projection, const DG_PROJ_FOV *fov,
                          float *sx, float *xoff, float *sy, float *yoff) {
    double tan_l, tan_r, tan_u, tan_d;
    double xden, yden;
    double sign_x, sign_y;

    tan_l = tan(fov->left);
    tan_r = tan(fov->right);
    tan_u = tan(fov->up);
    tan_d = tan(fov->down);
    xden = tan_r - tan_l;
    yden = tan_u - tan_d;
    sign_x = (double)dg_proj_sign(projection->m[0][0]);
    sign_y = (double)dg_proj_sign(projection->m[1][1]);

    *sx = (float)(2.0 * sign_x / xden);
    *xoff = (float)(-sign_x * (tan_r + tan_l) / xden);
    *sy = (float)(2.0 * sign_y / yden);
    *yoff = (float)(-sign_y * (tan_u + tan_d) / yden);
}

static void dg_proj_retarget_axis(float old_scale, float old_centre,
                                  double old_left, double old_right,
                                  double new_left, double new_right,
                                  float *new_scale, float *new_centre) {
    double tan_old_left = tan(old_left);
    double tan_old_right = tan(old_right);
    double tan_new_left = tan(new_left);
    double tan_new_right = tan(new_right);
    double old_span = tan_old_right - tan_old_left;
    double new_span = tan_new_right - tan_new_left;
    double half_extent = (double)old_scale * old_span / 2.0;
    double centre = (double)old_centre +
                    (double)old_scale * (tan_old_right + tan_old_left) / 2.0;

    *new_scale = (float)(2.0 * half_extent / new_span);
    *new_centre = (float)(centre - half_extent *
                          (tan_new_right + tan_new_left) / new_span);
}

void dg_proj_retarget(MAT *projection, const DG_PROJ_FOV *old_fov,
                      const DG_PROJ_FOV *new_fov) {
    float sx, cx, sy, cy;

    dg_proj_retarget_axis(projection->m[0][0], projection->m[2][0],
                          old_fov->left, old_fov->right,
                          new_fov->left, new_fov->right,
                          &sx, &cx);
    dg_proj_retarget_axis(projection->m[1][1], projection->m[2][1],
                          old_fov->down, old_fov->up,
                          new_fov->down, new_fov->up,
                          &sy, &cy);
    projection->m[0][0] = sx;
    projection->m[2][0] = cx;
    projection->m[1][1] = sy;
    projection->m[2][1] = cy;
}

void dg_proj_replace(MAT *projection, const DG_PROJ_FOV *fov) {
    float sx, xoff, sy, yoff;

    dg_proj_terms(projection, fov, &sx, &xoff, &sy, &yoff);
    projection->m[0][0] = sx;
    projection->m[1][1] = sy;
    projection->m[2][0] = xoff;
    projection->m[2][1] = yoff;
}

void dg_proj_add(MAT *projection, const DG_PROJ_FOV *fov) {
    float sx, xoff, sy, yoff;

    dg_proj_terms(projection, fov, &sx, &xoff, &sy, &yoff);
    projection->m[0][0] = sx;
    projection->m[1][1] = sy;
    projection->m[2][0] += xoff;
    projection->m[2][1] += yoff;
}

#ifdef DG_PROJ_TEST

static int g_checks;
static int g_failures;

static void check_close(const char *name, float actual, float expected,
                        float tolerance) {
    g_checks++;
    if ((float)fabs((double)actual - (double)expected) <= tolerance)
        printf("PASS: %s\n", name);
    else {
        g_failures++;
        printf("FAIL: %s (actual %.9f expected %.9f)\n", name, actual, expected);
    }
}

static void check_bits(const char *name, const void *left, const void *right,
                       size_t bytes) {
    g_checks++;
    if (memcmp(left, right, bytes) == 0)
        printf("PASS: %s\n", name);
    else {
        g_failures++;
        printf("FAIL: %s\n", name);
    }
}

static void check_true(const char *name, int condition) {
    g_checks++;
    if (condition)
        printf("PASS: %s\n", name);
    else {
        g_failures++;
        printf("FAIL: %s\n", name);
    }
}

static void identity_with_markers(MAT *m) {
    int r, c;
    memset(m, 0, sizeof(*m));
    for (r = 0; r < 4; r++) m->m[r][r] = 1.0f;
    m->m[2][2] = 123.25f;
    m->m[3][2] = -456.5f;
    m->m[3][3] = 789.75f;
    for (c = 0; c < 4; c++) m->m[2][c] += (float)c * 0.125f;
    m->m[2][2] = 123.25f;
}

static float radians_from_degrees(float degrees) {
    return degrees * (float)(DG_PROJ_PI / 180.0);
}

static float ndc(float ratio, float scale, float offset) {
    return ratio * scale + offset;
}

static void test_symmetric(void) {
    MAT m;
    DG_PROJ_FOV fov;
    float half;

    identity_with_markers(&m);
    fov.left = radians_from_degrees(-33.69f);
    fov.right = radians_from_degrees(33.69f);
    fov.up = radians_from_degrees(20.0f);
    fov.down = radians_from_degrees(-20.0f);
    dg_proj_replace(&m, &fov);
    half = (float)tan(33.69 * DG_PROJ_PI / 180.0);
    check_close("symmetric horizontal scale", m.m[0][0], 1.0f / half, 0.000001f);
    check_close("symmetric horizontal offset", m.m[2][0], 0.0f, 0.000001f);
}

static void test_measured_values(void) {
    MAT m;
    DG_PROJ_FOV fov;

    identity_with_markers(&m);
    fov.left = radians_from_degrees(-33.69f);
    fov.right = radians_from_degrees(33.69f);
    fov.up = radians_from_degrees(20.0f);
    fov.down = radians_from_degrees(-20.0f);
    m.m[1][1] = -3.202857f;
    dg_proj_replace(&m, &fov);
    check_close("measured widescreen m[0][0]", m.m[0][0], 1.5f, 0.00001f);
    check_true("measured vertical sign inherited", m.m[1][1] < 0.0f);
}

static void test_round_trip(void) {
    MAT m;
    DG_PROJ_FOV fov;
    float left, right, up, down;

    identity_with_markers(&m);
    fov.left = radians_from_degrees(-35.0f);
    fov.right = radians_from_degrees(50.0f);
    fov.up = radians_from_degrees(40.0f);
    fov.down = radians_from_degrees(-25.0f);
    dg_proj_replace(&m, &fov);
    left = (float)tan(fov.left);
    right = (float)tan(fov.right);
    up = (float)tan(fov.up);
    down = (float)tan(fov.down);
    check_close("asymmetric horizontal left edge", ndc(left, m.m[0][0], m.m[2][0]), -1.0f, 0.000002f);
    check_close("asymmetric horizontal right edge", ndc(right, m.m[0][0], m.m[2][0]), 1.0f, 0.000002f);
    check_close("asymmetric vertical down edge", ndc(down, m.m[1][1], m.m[2][1]), -1.0f, 0.000002f);
    check_close("asymmetric vertical up edge", ndc(up, m.m[1][1], m.m[2][1]), 1.0f, 0.000002f);
}

static void test_add_and_depth(void) {
    MAT before, m;
    DG_PROJ_FOV fov;
    float old_xoff, old_yoff;

    identity_with_markers(&m);
    m.m[2][0] = 0.25f;
    m.m[2][1] = -0.5f;
    old_xoff = m.m[2][0];
    old_yoff = m.m[2][1];
    before = m;
    fov.left = radians_from_degrees(-35.0f);
    fov.right = radians_from_degrees(50.0f);
    fov.up = radians_from_degrees(40.0f);
    fov.down = radians_from_degrees(-25.0f);
    dg_proj_add(&m, &fov);
    check_close("add preserves existing horizontal offset", m.m[2][0], old_xoff - 0.259808f, 0.00001f);
    check_close("add preserves existing vertical offset", m.m[2][1], old_yoff - 0.285575f, 0.00001f);
    check_bits("depth mapping remains bitwise unchanged", &m.m[2][2], &before.m[2][2], sizeof(float));
    check_bits("far/depth row remains bitwise unchanged", &m.m[3][2], &before.m[3][2], sizeof(float));
}

static DG_PROJ_FOV fov_degrees(double left, double right, double down, double up) {
    DG_PROJ_FOV fov;
    fov.left = left * DG_PROJ_PI / 180.0;
    fov.right = right * DG_PROJ_PI / 180.0;
    fov.down = down * DG_PROJ_PI / 180.0;
    fov.up = up * DG_PROJ_PI / 180.0;
    return fov;
}

static void set_depth_markers(MAT *m) {
    m->m[2][2] = 123.25f;
    m->m[3][2] = -456.5f;
}

static void test_ndc_regression(void) {
    MAT actual, expected;
    DG_PROJ_FOV old_fov = fov_degrees(-33.69, 33.69, -20.0, 20.0);
    DG_PROJ_FOV new_fov = fov_degrees(-35.0, 50.0, -25.0, 40.0);

    identity_with_markers(&actual);
    dg_proj_replace(&actual, &old_fov);
    expected = actual;
    dg_proj_retarget(&actual, &old_fov, &new_fov);
    dg_proj_replace(&expected, &new_fov);
    check_close("NDC retarget matches existing result", actual.m[0][0],
                expected.m[0][0], 0.000001f);
    check_close("NDC retarget matches existing y result", actual.m[1][1],
                expected.m[1][1], 0.000001f);
}

static void test_screen_space(void) {
    MAT m;
    DG_PROJ_FOV old_fov = fov_degrees(-33.69, 33.69, -20.0, 20.0);
    DG_PROJ_FOV same_fov = old_fov;
    double half = tan(33.69 * DG_PROJ_PI / 180.0);

    identity_with_markers(&m);
    m.m[0][0] = 384.0f;
    m.m[1][1] = 573.952f;
    m.m[2][0] = 2048.0f;
    m.m[2][1] = 2048.0f;
    dg_proj_retarget(&m, &old_fov, &same_fov);
    check_close("screen-space recovered horizontal half extent", (float)(m.m[0][0] * half), 256.0f, 0.01f);
    check_close("screen-space centre remains horizontal", m.m[2][0], 2048.0f, 0.0001f);
    check_close("screen-space centre remains vertical", m.m[2][1], 2048.0f, 0.0001f);
}

static void test_identity_and_round_trip(void) {
    MAT original, identity, round_trip;
    DG_PROJ_FOV old_fov = fov_degrees(-30.0, 42.0, -18.0, 36.0);
    DG_PROJ_FOV new_fov = fov_degrees(-47.0, 28.0, -31.0, 22.0);

    identity_with_markers(&identity);
    identity.m[0][0] = 384.0f;
    identity.m[1][1] = 573.952f;
    identity.m[2][0] = 2048.0f;
    identity.m[2][1] = 2048.0f;
    original = identity;
    dg_proj_retarget(&identity, &old_fov, &old_fov);
    check_close("identity horizontal scale", identity.m[0][0], original.m[0][0], 0.00001f);
    check_close("identity horizontal centre", identity.m[2][0], original.m[2][0], 0.00001f);
    check_close("identity vertical scale", identity.m[1][1], original.m[1][1], 0.00001f);
    check_close("identity vertical centre", identity.m[2][1], original.m[2][1], 0.00001f);

    round_trip = original;
    dg_proj_retarget(&round_trip, &old_fov, &new_fov);
    dg_proj_retarget(&round_trip, &new_fov, &old_fov);
    check_close("round trip horizontal scale", round_trip.m[0][0], original.m[0][0], 0.0001f);
    check_close("round trip horizontal centre", round_trip.m[2][0], original.m[2][0], 0.0001f);
    check_close("round trip vertical scale", round_trip.m[1][1], original.m[1][1], 0.0001f);
    check_close("round trip vertical centre", round_trip.m[2][1], original.m[2][1], 0.0001f);
}

static void test_edges_and_asymmetric(void) {
    MAT ndc_m, screen_m;
    DG_PROJ_FOV old_fov = fov_degrees(-33.69, 33.69, -20.0, 20.0);
    DG_PROJ_FOV new_fov = fov_degrees(-47.0, 28.0, -31.0, 22.0);
    float old_left = (float)tan(old_fov.left);
    float old_right = (float)tan(old_fov.right);
    float old_down = (float)tan(old_fov.down);
    float old_up = (float)tan(old_fov.up);

    identity_with_markers(&ndc_m);
    ndc_m.m[0][0] = 1.5f;
    ndc_m.m[1][1] = 2.562286f;
    ndc_m.m[2][0] = 0.0f;
    ndc_m.m[2][1] = 0.0f;
    screen_m = ndc_m;
    screen_m.m[0][0] = 384.0f;
    screen_m.m[1][1] = 573.952f;
    screen_m.m[2][0] = 2048.0f;
    screen_m.m[2][1] = 2048.0f;
    dg_proj_retarget(&ndc_m, &old_fov, &new_fov);
    dg_proj_retarget(&screen_m, &old_fov, &new_fov);

    check_close("NDC new left preserves old edge", ndc((float)tan(new_fov.left), ndc_m.m[0][0], ndc_m.m[2][0]), ndc(old_left, 1.5f, 0.0f), 0.00002f);
    check_close("NDC new right preserves old edge", ndc((float)tan(new_fov.right), ndc_m.m[0][0], ndc_m.m[2][0]), ndc(old_right, 1.5f, 0.0f), 0.00002f);
    check_close("NDC new down preserves old edge", ndc((float)tan(new_fov.down), ndc_m.m[1][1], ndc_m.m[2][1]), ndc(old_down, 2.562286f, 0.0f), 0.00002f);
    check_close("NDC new up preserves old edge", ndc((float)tan(new_fov.up), ndc_m.m[1][1], ndc_m.m[2][1]), ndc(old_up, 2.562286f, 0.0f), 0.00002f);
    check_close("screen new left preserves old edge", ndc((float)tan(new_fov.left), screen_m.m[0][0], screen_m.m[2][0]), ndc(old_left, 384.0f, 2048.0f), 0.01f);
    check_close("screen new right preserves old edge", ndc((float)tan(new_fov.right), screen_m.m[0][0], screen_m.m[2][0]), ndc(old_right, 384.0f, 2048.0f), 0.01f);
    check_close("screen new down preserves old edge", ndc((float)tan(new_fov.down), screen_m.m[1][1], screen_m.m[2][1]), ndc(old_down, 573.952f, 2048.0f), 0.01f);
    check_close("screen new up preserves old edge", ndc((float)tan(new_fov.up), screen_m.m[1][1], screen_m.m[2][1]), ndc(old_up, 573.952f, 2048.0f), 0.01f);
}

static void test_depth_untouched(void) {
    MAT before, m;
    DG_PROJ_FOV old_fov = fov_degrees(-33.69, 33.69, -20.0, 20.0);
    DG_PROJ_FOV new_fov = fov_degrees(-47.0, 28.0, -31.0, 22.0);

    identity_with_markers(&m);
    m.m[0][0] = 384.0f;
    m.m[1][1] = 573.952f;
    m.m[2][0] = 2048.0f;
    m.m[2][1] = 2048.0f;
    set_depth_markers(&m);
    before = m;
    dg_proj_retarget(&m, &old_fov, &new_fov);
    check_bits("retarget depth mapping remains bitwise unchanged", &m.m[2][2], &before.m[2][2], sizeof(float));
    check_bits("retarget far/depth row remains bitwise unchanged", &m.m[3][2], &before.m[3][2], sizeof(float));
}

int main(void) {
    test_symmetric();
    test_measured_values();
    test_round_trip();
    test_add_and_depth();
    test_ndc_regression();
    test_screen_space();
    test_identity_and_round_trip();
    test_edges_and_asymmetric();
    test_depth_untouched();
    printf("%d assertions, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

#endif
