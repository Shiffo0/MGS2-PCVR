/* dg_rec.c - flight recorder ring, serialization, and file format.
 *
 * Pure in the house sense: no windows.h, no game, no clock. The single-writer
 * append path is plain volatile stores (see the header for the /volatile:ms
 * reliance); file I/O happens only through a FILE* the caller owns, which is
 * what lets the desk tests and the desk replay use the identical code the
 * hook dumps with.
 */

#define _CRT_SECURE_NO_WARNINGS  /* sscanf on numeric-only header fields */
#include <math.h>
#include <string.h>
#include "dg_rec.h"

/* The dedupe span: everything that is INPUT, nothing that is identity. It
   starts at head[0] and ends at the end of the pair block; qpc sits before
   it and the four u32s after it, so one offset and one length cover it
   exactly. The pair block is deliberately inside: an animation base moving
   under a byte-identical controller is a real change of the pipeline's
   input surface and must not fold away. */
#define DG_REC_CMP_OFF  ((size_t)8)
#define DG_REC_CMP_LEN  (sizeof(double[7]) + 2 * sizeof(DG_REC_HAND) + \
                         sizeof(DG_REC_PAIRSTATE))

void dg_rec_reset(DG_REC_RING *r)
{
    if (!r) return;
    memset(r, 0, sizeof *r);
}

static void pack_pose(const DG_XR_HAND_POSE *p, DG_REC_POSE *out)
{
    out->q[0] = p->raw_local.qx; out->q[1] = p->raw_local.qy;
    out->q[2] = p->raw_local.qz; out->q[3] = p->raw_local.qw;
    out->p[0] = p->raw_local.px; out->p[1] = p->raw_local.py;
    out->p[2] = p->raw_local.pz;
    out->sample_seq = p->sample_seq;
    out->xr_time = p->xr_time;
    out->flags = (p->position_valid ? 1u : 0u)
               | (p->orientation_valid ? 2u : 0u)
               | (p->active ? 4u : 0u)
               | (p->tracked ? 8u : 0u);
    out->pose_age_ms = p->pose_age_ms;
}

static void unpack_pose(const DG_REC_POSE *r, unsigned int hand,
                        unsigned int kind, DG_XR_HAND_POSE *out)
{
    memset(out, 0, sizeof *out);
    out->hand = hand;
    out->kind = kind;
    out->raw_local.qx = r->q[0]; out->raw_local.qy = r->q[1];
    out->raw_local.qz = r->q[2]; out->raw_local.qw = r->q[3];
    out->raw_local.px = r->p[0]; out->raw_local.py = r->p[1];
    out->raw_local.pz = r->p[2];
    out->sample_seq = r->sample_seq;
    out->xr_time = r->xr_time;
    out->position_valid = (r->flags >> 0) & 1u;
    out->orientation_valid = (r->flags >> 1) & 1u;
    out->active = (r->flags >> 2) & 1u;
    out->tracked = (r->flags >> 3) & 1u;
    out->pose_age_ms = r->pose_age_ms;
}

static void pack_hand(const DG_XR_HAND *h, DG_REC_HAND *out)
{
    pack_pose(&h->grip, &out->grip);
    pack_pose(&h->aim, &out->aim);
    out->trigger_press_seq = h->trigger_press_seq;
    out->trigger_release_seq = h->trigger_release_seq;
    out->trigger_value = h->trigger_value;
    out->squeeze_value = h->squeeze_value;
    out->thumbstick_x = h->thumbstick_x;
    out->thumbstick_y = h->thumbstick_y;
    out->buttons = (h->trigger_click ? 1u : 0u)
                 | (h->squeeze_click ? 2u : 0u)
                 | (h->thumbstick_click ? 4u : 0u)
                 | (h->primary_button ? 8u : 0u)
                 | (h->secondary_button ? 16u : 0u)
                 | (h->menu_button ? 32u : 0u);
    out->pad0 = 0;
}

static void unpack_hand(const DG_REC_HAND *r, unsigned int hand,
                        DG_XR_HAND *out)
{
    memset(out, 0, sizeof *out);
    out->hand = hand;
    unpack_pose(&r->grip, hand, DG_XR_POSE_GRIP, &out->grip);
    unpack_pose(&r->aim, hand, DG_XR_POSE_AIM, &out->aim);
    out->trigger_press_seq = r->trigger_press_seq;
    out->trigger_release_seq = r->trigger_release_seq;
    out->trigger_value = r->trigger_value;
    out->squeeze_value = r->squeeze_value;
    out->thumbstick_x = r->thumbstick_x;
    out->thumbstick_y = r->thumbstick_y;
    out->trigger_click = (r->buttons >> 0) & 1u;
    out->squeeze_click = (r->buttons >> 1) & 1u;
    out->thumbstick_click = (r->buttons >> 2) & 1u;
    out->primary_button = (r->buttons >> 3) & 1u;
    out->secondary_button = (r->buttons >> 4) & 1u;
    out->menu_button = (r->buttons >> 5) & 1u;
}

void dg_rec_pack(const DG_XR_FRAME *f, long long qpc, unsigned int stream_id,
                 unsigned int pair_id, unsigned int present_frame,
                 unsigned int eye, DG_REC_FRAME *out)
{
#if DG_ENABLE_DIAGNOSTICS

    /* memset first: the struct has no holes by construction, but "by
       construction" is exactly the claim a future field should not be able
       to silently weaken. A total byte image keeps memcmp meaningful. */
    memset(out, 0, sizeof *out);
    /* The drift's absence value is not zero (0.0 is a perfect-agreement
       measurement), so the blank frame must carry it explicitly. A caller
       that fills the pair block overwrites this; one that has nothing
       leaves the honest "not measured". */
    out->pair.rest_drift_deg = -1.0f;
    out->qpc = qpc;
    out->head[0] = f->head_raw.qx; out->head[1] = f->head_raw.qy;
    out->head[2] = f->head_raw.qz; out->head[3] = f->head_raw.qw;
    out->head[4] = f->head_raw.px; out->head[5] = f->head_raw.py;
    out->head[6] = f->head_raw.pz;
    pack_hand(&f->left_hand, &out->hand[0]);
    pack_hand(&f->right_hand, &out->hand[1]);
    out->present_frame = present_frame;
    out->eye = eye;
    out->stream_id = stream_id;
    out->pair_id = pair_id;

#else
if (out) memset(out,0,sizeof *out);
#endif
}

void dg_rec_unpack(const DG_REC_FRAME *r, DG_XR_FRAME *out)
{
#if DG_ENABLE_DIAGNOSTICS

    memset(out, 0, sizeof *out);
    out->head_raw.qx = r->head[0]; out->head_raw.qy = r->head[1];
    out->head_raw.qz = r->head[2]; out->head_raw.qw = r->head[3];
    out->head_raw.px = r->head[4]; out->head_raw.py = r->head[5];
    out->head_raw.pz = r->head[6];
    unpack_hand(&r->hand[0], DG_XR_HAND_LEFT, &out->left_hand);
    unpack_hand(&r->hand[1], DG_XR_HAND_RIGHT, &out->right_hand);

#else
if (out) memset(out,0,sizeof *out);
#endif
}

static int camera_basis_valid(const MAT *m)
{
    float dot, norm, det;
    int i, j;

    for (i = 0; i < 3; i++) {
        norm = 0.0f;
        for (j = 0; j < 3; j++) {
            float v = m->m[i][j];
            if (!isfinite(v)) return 0;
            norm += v * v;
        }
        if (fabsf(norm - 1.0f) > 0.01f) return 0;
    }
    for (i = 0; i < 3; i++) {
        for (j = i + 1; j < 3; j++) {
            dot = m->m[i][0] * m->m[j][0]
                + m->m[i][1] * m->m[j][1]
                + m->m[i][2] * m->m[j][2];
            if (fabsf(dot) > 0.01f) return 0;
        }
    }
    det = m->m[0][0] * (m->m[1][1] * m->m[2][2]
                      - m->m[1][2] * m->m[2][1])
        - m->m[0][1] * (m->m[1][0] * m->m[2][2]
                      - m->m[1][2] * m->m[2][0])
        + m->m[0][2] * (m->m[1][0] * m->m[2][1]
                      - m->m[1][1] * m->m[2][0]);
    return fabsf(fabsf(det) - 1.0f) <= 0.01f;
}

void dg_rec_pair_camera(DG_REC_PAIRSTATE *pair, const MAT *camera_world,
                        const MAT *projection)
{
#if DG_ENABLE_DIAGNOSTICS

    float m00, m11, m23;
    int i, j;

    if (!pair) return;
    memset(pair->camera_world, 0, sizeof pair->camera_world);
    memset(pair->camera_proj, 0, sizeof pair->camera_proj);
    pair->flags &= ~DG_REC_PAIR_F_CAMERA;
    if (!camera_world || !projection ||
        (pair->flags & DG_REC_PAIR_F_FRAMES) == 0 ||
        !camera_basis_valid(camera_world)) return;

    m00 = projection->m[0][0];
    m11 = projection->m[1][1];
    m23 = projection->m[2][3];
    if (!isfinite(m00) || !isfinite(m11) || !isfinite(m23) ||
        m00 == 0.0f || m11 == 0.0f || m23 == 0.0f ||
        fabsf(fabsf(m23) - 1.0f) > 0.01f) return;

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            pair->camera_world[i][j] = camera_world->m[i][j];
    pair->camera_proj[0] = m00;
    pair->camera_proj[1] = m11;
    pair->camera_proj[2] = m23;
    pair->flags |= DG_REC_PAIR_F_CAMERA;

#else

#endif
}

int dg_rec_capture(DG_REC_RING *r, const DG_REC_FRAME *f)
{
#if DG_ENABLE_DIAGNOSTICS

    DG_REC_SLOT *s;
    long idx;

    if (!r || !f) return 0;
    r->seen++;
    /* Absolute filtering advances on every camera visit, including repeated
       input and invalid visits. Omitting one would change stereo/timing state. */
    if (!f->absolute.present && r->have_last && !r->last.absolute.present &&
        memcmp((const char *)&r->last + DG_REC_CMP_OFF,
               (const char *)f + DG_REC_CMP_OFF, DG_REC_CMP_LEN) == 0) {
        r->dup++;
        return 0;
    }
    idx = r->appended;
    s = &r->slot[idx % DG_REC_CAP];
    s->seq = -1;                 /* in progress: readers drop this slot */
    s->f = *f;
    s->seq = idx + 1;            /* published, stamped with its lap */
    r->appended = idx + 1;
    r->last = *f;
    r->have_last = 1;
    return 1;

#else
return 0;
#endif
}

long dg_rec_write(DG_REC_RING *r, long long qpf, FILE *out, long *torn_out)
{
#if DG_ENABLE_DIAGNOSTICS

    long total, first, i, written = 0, torn = 0;

    if (torn_out) *torn_out = 0;
    if (!r || !out) return -1;
    total = r->appended;
    first = (total > DG_REC_CAP) ? total - DG_REC_CAP : 0;

    /* The header is text on purpose: a recording found on disk months from
       now says what it is when opened in anything at all. record_bytes is
       the contract the reader enforces. */
    if (fprintf(out,
                "%s\n"
                "record_bytes=%u\n"
                "fields=qpc i64; head q4 p3 f64; per-hand[L,R]: grip(q4 p3 "
                "seq u64 xrtime i64 flags u32 age u32) aim(same) press_seq "
                "u64 release_seq u64 trigger f32 squeeze f32 stick_x f32 "
                "stick_y f32 buttons u32 pad u32; pairstate(base q4, live "
                "q4, root q4, root0 q4, rest q4, ctrlrest q4, demand q4, "
                "desired q4, wtype u32, pflags u32, restdrift f32, "
                "frameword u32, v4: joints3-6 f32x12 ikelbow f32x3 ikwrist "
                "f32x3 iktarget f32x3 q4w f32x4 q5w f32x4 foreaxis f32x3 "
                "rawtwist f32 rawswing f32 fitfrac f32 headyaw f32 stickyaw "
                "f32 aimyaw f32; v5: camera_world f32x9 camera_proj "
                "f32x3); present u32 eye u32 stream u32 pair u32; "
                "v6 absolute: raw camera/view/AIM, validity/identity/dt, "
                "filter checkpoint, observed target\n"
                "qpf=%lld\n"
                "first_index=%ld\n"
                "count_hint=%ld\n"
                "seen=%ld\n"
                "dup=%ld\n"
                "end\n",
                DG_REC_MAGIC, (unsigned int)sizeof(DG_REC_FRAME), qpf,
                first, total - first, r->seen, r->dup) < 0)
        return -1;

    for (i = first; i < total; i++) {
        DG_REC_SLOT *s = &r->slot[i % DG_REC_CAP];
        DG_REC_FRAME copy;
        long s1 = s->seq;
        if (s1 != i + 1) { torn++; continue; }
        copy = s->f;
        if (s->seq != s1) { torn++; continue; }
        if (fwrite(&copy, sizeof copy, 1, out) != 1) return -1;
        written++;
    }
    if (torn_out) *torn_out = torn;
    return written;

#else
return 0;
#endif
}

int dg_rec_read_open(FILE *in, long *count_out, long long *qpf_out,
                     int *v1_out)
{
#if DG_ENABLE_DIAGNOSTICS

    char line[512];
    long count = -1;
    long long qpf = 0;
    unsigned int rb = 0, want;
    int v1 = DG_REC_COMPAT_NONE;

    if (count_out) *count_out = 0;
    if (qpf_out) *qpf_out = 0;
    if (v1_out) *v1_out = 0;
    if (!in || !fgets(line, sizeof line, in)) return 0;
    if (strncmp(line, DG_REC_MAGIC, strlen(DG_REC_MAGIC)) == 0)
        v1 = DG_REC_COMPAT_NONE;
    else if (strncmp(line, DG_REC_MAGIC_V5, strlen(DG_REC_MAGIC_V5)) == 0)
        v1 = DG_REC_COMPAT_V5;
    else if (strncmp(line, DG_REC_MAGIC_V4, strlen(DG_REC_MAGIC_V4)) == 0)
        v1 = DG_REC_COMPAT_V4;
    else if (strncmp(line, DG_REC_MAGIC_V3, strlen(DG_REC_MAGIC_V3)) == 0)
        v1 = DG_REC_COMPAT_V3;
    else if (strncmp(line, DG_REC_MAGIC_V2, strlen(DG_REC_MAGIC_V2)) == 0)
        v1 = DG_REC_COMPAT_V2;
    else if (strncmp(line, DG_REC_MAGIC_V1, strlen(DG_REC_MAGIC_V1)) == 0)
        v1 = DG_REC_COMPAT_V1;
    else
        return 0;
    for (;;) {
        if (!fgets(line, sizeof line, in)) return 0;
        if (strncmp(line, "end", 3) == 0) break;
        if (sscanf(line, "record_bytes=%u", &rb) == 1) continue;
        if (sscanf(line, "qpf=%lld", &qpf) == 1) continue;
        if (sscanf(line, "count_hint=%ld", &count) == 1) continue;
        /* Unknown header lines are tolerated - the header may grow - but the
           two load-bearing ones are not optional, checked below. */
    }
    /* Each magic vouches for exactly one record size; a mismatch is a file
       written by a layout this build does not know, refused rather than
       misread. */
    want = (v1 == DG_REC_COMPAT_V1) ? (unsigned int)DG_REC_V1_BYTES
         : (v1 == DG_REC_COMPAT_V2) ? (unsigned int)DG_REC_V2_BYTES
         : (v1 == DG_REC_COMPAT_V3) ? (unsigned int)DG_REC_V3_BYTES
         : (v1 == DG_REC_COMPAT_V4) ? (unsigned int)DG_REC_V4_BYTES
         : (v1 == DG_REC_COMPAT_V5) ? (unsigned int)DG_REC_V5_BYTES
         : (unsigned int)sizeof(DG_REC_FRAME);
    if (rb != want) return 0;
    if (count < 0) return 0;
    if (count_out) *count_out = count;
    if (qpf_out) *qpf_out = qpf;
    if (v1_out) *v1_out = v1;
    return 1;

#else
return 0;
#endif
}

int dg_rec_read_next(FILE *in, int v1, DG_REC_FRAME *out)
{
#if DG_ENABLE_DIAGNOSTICS

    if (!in || !out) return 0;
    memset(out, 0, sizeof *out);
    if (v1 == DG_REC_COMPAT_NONE)
        return fread(out, sizeof *out, 1, in) == 1;
    if (v1 == DG_REC_COMPAT_V5)
        return fread(out, DG_REC_V5_BYTES, 1, in) == 1;
    if (v1 == DG_REC_COMPAT_V3) {

        memset(&out->pair, 0, sizeof out->pair);
        if (fread(out, DG_REC_V1_PREFIX, 1, in) != 1) return 0;
        if (fread(&out->pair, DG_REC_V3_PAIR_BYTES, 1, in) != 1) return 0;
        out->pair.flags &= ~DG_REC_PAIR_F_CAMERA;
        return fread(&out->present_frame, 4 * sizeof(unsigned int), 1, in)
               == 1;
    }
    if (v1 == DG_REC_COMPAT_V2) {

        memset(&out->pair, 0, sizeof out->pair);
        if (fread(out, DG_REC_V1_PREFIX, 1, in) != 1) return 0;
        if (fread(&out->pair, DG_REC_V2_PAIR_BYTES, 1, in) != 1) return 0;
        out->pair.rest_drift_deg = -1.0f;
        out->pair.frame_word = 0;      /* older files carry no frame */
        out->pair.flags &= ~DG_REC_PAIR_F_CAMERA;
        return fread(&out->present_frame, 4 * sizeof(unsigned int), 1, in)
               == 1;
    }
    if (v1 == DG_REC_COMPAT_V4) {

        memset(&out->pair, 0, sizeof out->pair);
        if (fread(out, DG_REC_V1_PREFIX, 1, in) != 1) return 0;
        if (fread(&out->pair, DG_REC_V4_PAIR_BYTES, 1, in) != 1) return 0;
        out->pair.flags &= ~DG_REC_PAIR_F_CAMERA;
        return fread(&out->present_frame, 4 * sizeof(unsigned int), 1, in)
               == 1;
    }
    /* A v1 record is the same bytes with no pair block: prefix, then the
       u32 tail. The pair comes back all-zero, flags included - which reads
       as "the bridge had nothing", exactly what was true; the drift is not
       zero but "not measured". */
    memset(&out->pair, 0, sizeof out->pair);
    out->pair.rest_drift_deg = -1.0f;
    if (fread(out, DG_REC_V1_PREFIX, 1, in) != 1) return 0;
    return fread(&out->present_frame, 4 * sizeof(unsigned int), 1, in) == 1;

#else
return 0;
#endif
}
