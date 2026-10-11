#pragma once













#include <d3d11_1.h>
#include <wrl/client.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "dg_clip_rounding.h"
using Microsoft::WRL::ComPtr;

/* dg_cb_retarget.hlsl for one bank. rows = the 16 words of gVS_Pers
   (c[Pers+k][i] = P^T[i][k]); D and Sinv row-major as in the shader's cbuffer.
   out[i*4+j] = sum_k D[k][i] * rows[k*4+j] when the prefix is affine, else 0.
   Returns 1 retargeted, 0 projective (emptied). */
static int dgRetargetRowsCpu(const uint32_t in[16], uint32_t out[16], const float D[16],
                             const float Sinv[16], float tolerance, const float sourceRounding[4] = nullptr) {
    float r[16], o[16];
    memcpy(r, in, sizeof r);
    for (int i = 0; i < 4; i++) {
        float x = 0, scale = 0;
        for (int k = 0; k < 4; k++) { float t = r[k * 4 + i] * Sinv[k * 4 + 3]; x += t; scale += fabsf(t); }
        float want = i == 3 ? 1.0f : 0.0f;
        /* The port folds legacy clip z/w into z'=(z+w)/2. Its operands can
           almost cancel before this bank is stored as float. Bound that
           operand rounding through Sinv; keep the unit homogeneous check. */
        float z = r[8 + i], w = r[12 + i];
        float rounding = (5.960464477539063e-8f * 0.5f) *
                         (fabsf(2 * z - w) + fabsf(w)) * fabsf(Sinv[11]);



        if (sourceRounding && !(fabsf(x - want) <= tolerance * (scale + 1) + rounding)) {
            float sourceError = 0;
            for (int j = 0; j < 4; j++) {
                float prefix = 0;
                for (int k = 0; k < 4; k++) prefix += r[k * 4 + i] * Sinv[k * 4 + j];
                sourceError += fabsf(prefix) * sourceRounding[j];
            }
            rounding += sourceError * fabsf(Sinv[11]);
        }
        float bound = tolerance * (scale + 1) + rounding;
        if (!_finite(bound) || !(fabsf(x - want) <= bound)) { memset(out, 0, 64); return 0; }
    }
    for (int e = 0; e < 16; e++) {
        int i = e / 4, j = e % 4;
        float s = 0;
        for (int k = 0; k < 4; k++) s += D[k * 4 + i] * r[k * 4 + j];
        o[e] = s;
    }
    memcpy(out, o, sizeof o);
    return 1;
}


/* Split camera layout (29 Sep, the game's effect shaders): c[pers-4..pers-1] = VIEW V (rigid),
   c[pers..pers+3] = PROJECTION P, clip = P*(V*v) - measured live: P*V = this frame's camera to
   0.1%, V orthonormal to 7e-6, P's x skew +-0.24251 per eye (the mirrored lens asymmetry). The
   plain check sees "P = X*S*Q" as projective. Here: when P*V is the frame's camera (Sc = S^T, game
   clip), the other eye gets P' = P with its x skew mirrored and V' = P'^-1 * Tc (Tc = T^T); V' must
   come out rigid, else P' = P and V' = P^-1 * Tc (exact on screen, view space not rigid).
   Returns 0 no match (the caller empties the rows), 1 mirrored + rigid, 2 exact fallback. */
static bool dgInverse4(const double m[16], double o[16]) {
    double a[4][8];
    for (int r = 0; r < 4; r++) for (int c = 0; c < 8; c++) a[r][c] = c < 4 ? m[r * 4 + c] : (c - 4 == r);
    for (int c = 0; c < 4; c++) {
        int piv = c; for (int r = c + 1; r < 4; r++) if (fabs(a[r][c]) > fabs(a[piv][c])) piv = r;
        if (fabs(a[piv][c]) < 1e-12) return false;
        if (piv != c) for (int k = 0; k < 8; k++) { double t = a[c][k]; a[c][k] = a[piv][k]; a[piv][k] = t; }
        double inv = 1.0 / a[c][c]; for (int k = 0; k < 8; k++) a[c][k] *= inv;
        for (int r = 0; r < 4; r++) if (r != c) { double f = a[r][c]; if (f != 0) for (int k = 0; k < 8; k++) a[r][k] -= f * a[c][k]; }
    }
    for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) o[r * 4 + c] = a[r][c + 4];
    return true;
}
static void dgMul4(const double a[16], const double b[16], double o[16]) {
    for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) { double s = 0; for (int k = 0; k < 4; k++) s += a[r * 4 + k] * b[k * 4 + c]; o[r * 4 + c] = s; }
}
static bool dgRigid(const double v[16]) {
    if (fabs(v[12]) > 1e-4 || fabs(v[13]) > 1e-4 || fabs(v[14]) > 1e-4 || fabs(v[15] - 1) > 1e-4) return false;
    for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
        double s = 0; for (int k = 0; k < 3; k++) s += v[i * 4 + k] * v[j * 4 + k];
        if (fabs(s - (i == j)) > 2e-3) return false;
    }
    return true;
}
static int dgSplitRetarget(const float vIn[16], const float pIn[16], const float Sc[16], const float Tc[16], float vOut[16], float pOut[16]) {
    double V[16], P[16], M[16];
    for (int i = 0; i < 16; i++) { V[i] = vIn[i]; P[i] = pIn[i]; }
    if (!dgRigid(V)) return 0;
    dgMul4(P, V, M);
    /* per element, against the size of its own terms (a row's translation is thousands of units:
       scaling by the row's largest value hid a 50% error in a rotation term) */
    for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) {
        double terms = 0; for (int k = 0; k < 4; k++) terms += fabs(P[r * 4 + k] * V[k * 4 + c]);
        if (fabs(M[r * 4 + c] - Sc[r * 4 + c]) > 1e-3 * terms + 1e-6) return 0;
    }
    double T[16], Pb[16], Pi[16], Vb[16];
    for (int i = 0; i < 16; i++) T[i] = Tc[i];
    memcpy(Pb, P, sizeof Pb); Pb[2] = -P[2];                    /* mirrored lens asymmetry */
    int kind = 1;
    if (!dgInverse4(Pb, Pi)) return 0;
    dgMul4(Pi, T, Vb);
    if (!dgRigid(Vb)) {
        kind = 2; memcpy(Pb, P, sizeof Pb);
        if (!dgInverse4(Pb, Pi)) return 0;
        dgMul4(Pi, T, Vb);
    }
    for (int i = 0; i < 16; i++) { vOut[i] = (float)Vb[i]; pOut[i] = (float)Pb[i]; }
    return kind;
}

/* Banks with their own depth mapping (10 Oct 2026, effect-parity tour: rain/splash particles, light pools, glow,
   sky - shaders C2290FAE, F26DFC8A, 265C303F, 0A541F15, E77E38FC). Their rows are L*X*S with X affine, where L
   only rewrites clip z from (z, w): the effect has its own near/far (e.g. a pure projection over view-space
   vertices), so the plain check sees a projective prefix and the rows are emptied (nothing drawn in the second
   eye). Solve L's two unknowns (row 2 of L^-1 = (0,0,u,v)) from the four "row 3 of S^-1 * L^-1 * R = (0,0,0,1)"
   equations in the least-squares sense, accept only when all four hold to the same relative tolerance, and
   retarget as L * D^T * L^-1 * R: x, y and w follow the other eye, depth keeps the effect's own mapping. With the
   same camera (D = I) this returns R. Returns 1 retargeted, 0 not this layout (out untouched). */
struct DgZProjStats { unsigned long long guarded = 0; double uMin = 1e30, uMax = 0, mixMax = 0; };
static DgZProjStats dgZProjStats;   /* what was accepted / refused by the conditioning guard (heartbeat) */
static int dgZProjRetarget(const uint32_t in[16], uint32_t out[16], const float D[16], const float Sinv[16], float tolerance) {
    float f[16]; memcpy(f, in, 64);
    double R[16]; for (int i = 0; i < 16; i++) { if (!_finite(f[i])) return 0; R[i] = f[i]; }
    const double s0 = Sinv[3], s1 = Sinv[7], s2 = Sinv[11], s3 = Sinv[15];
    double a[4], b[4], c[4];
    for (int j = 0; j < 4; j++) { a[j] = s2 * R[8 + j]; b[j] = s2 * R[12 + j]; c[j] = (j == 3 ? 1.0 : 0.0) - (s0 * R[j] + s1 * R[4 + j] + s3 * R[12 + j]); }
    /* each equation weighted by its own size (the same measure the acceptance uses): unweighted, a translation
       column of 1e5 drowns the other three and a decal bank with a pure depth offset looked ill-conditioned
       (live 10 Oct, shader 0A541F15: light pools) */
    double aa = 0, ab = 0, bb = 0, ac = 0, bc = 0;
    for (int j = 0; j < 4; j++) {
        double w = 1.0 / (fabs(s0 * R[j]) + fabs(s1 * R[4 + j]) + fabs(s2 * R[8 + j]) + fabs(s3 * R[12 + j]) + 1.0); w *= w;
        aa += w * a[j] * a[j]; ab += w * a[j] * b[j]; bb += w * b[j] * b[j]; ac += w * a[j] * c[j]; bc += w * b[j] * c[j];
    }
    double det = aa * bb - ab * ab;
    /* numerical floor only (live decal bank: 1.3e-10); the per-equation residual check below decides */
    if (!(fabs(det) > 1e-14 * aa * bb) || !(aa > 0) || !(bb > 0)) return 0;
    double u = (ac * bb - bc * ab) / det, v = (bc * aa - ac * ab) / det;
    if (!_finite(u) || !_finite(v) || fabs(u) < 1e-12) return 0;
    /* conditioning (live 10/11 Oct, 651EF615: random white quads in the second eye after the weighted solve): a real
       depth remap has 1/kappa = u near 1 (0.1..10) and mixes in little of the w row; an ill-conditioned L amplifies
       the float noise of D (~1e-7, even with the same camera) into a broken depth, and a hidden quad passes */
    double r2 = 0, r3 = 0; for (int j = 0; j < 4; j++) { r2 = fmax(r2, fabs(R[8 + j])); r3 = fmax(r3, fabs(R[12 + j])); }
    double mix = fabs(v) * r3 / (fabs(u) * r2 + 1e-30);
    if (!(fabs(u) > 0.1 && fabs(u) < 10.0) || !(mix < 1.0)) { dgZProjStats.guarded++; return 0; }   /* mix ~ noise amplification - 1 */
    for (int j = 0; j < 4; j++) {
        double res = u * a[j] + v * b[j] - c[j];
        double scale = fabs(s0 * R[j]) + fabs(s1 * R[4 + j]) + fabs(u * s2 * R[8 + j]) + fabs((s3 + v * s2) * R[12 + j]);
        if (!(fabs(res) <= tolerance * (scale + 1))) return 0;
    }
    double X[16], Y[16];
    for (int j = 0; j < 4; j++) { X[j] = R[j]; X[4 + j] = R[4 + j]; X[8 + j] = u * R[8 + j] + v * R[12 + j]; X[12 + j] = R[12 + j]; }
    for (int i = 0; i < 4; i++) for (int j = 0; j < 4; j++) { double s = 0; for (int k = 0; k < 4; k++) s += (double)D[k * 4 + i] * X[k * 4 + j]; Y[i * 4 + j] = s; }
    float o[16];
    for (int j = 0; j < 4; j++) { o[j] = (float)Y[j]; o[4 + j] = (float)Y[4 + j]; o[8 + j] = (float)((Y[8 + j] - v * Y[12 + j]) / u); o[12 + j] = (float)Y[12 + j]; }
    for (float x : o) if (!_finite(x)) return 0;
    memcpy(out, o, 64);
    if (fabs(u) < dgZProjStats.uMin) dgZProjStats.uMin = fabs(u);
    if (fabs(u) > dgZProjStats.uMax) dgZProjStats.uMax = fabs(u);
    if (mix > dgZProjStats.mixMax) dgZProjStats.mixMax = mix;
    return 1;
}

/* What the CPU knows about one game constant buffer: its bytes as last
   uploaded and the byte range [start, end) that upload covered. */
struct DgBankShadow { const uint8_t *bytes; UINT width, start, end; };
typedef const DgBankShadow *(*DgBankLookup)(ID3D11Buffer *buffer, void *ctx);

/* The bytes of the bound range [begin, begin+bytes) the shadow can vouch for,
   or the reason it cannot. */
static const char *dgBankRange(UINT width, UINT first, UINT count, UINT pers, const DgBankShadow *s,
                               UINT *begin, UINT *bytes) {
    if (pers < 16 || pers > 60) return "pers_index";
    if (first > 4096 || count > 4096 || count < pers + 4) return "buffer_range";
    const UINT b = first * 16, need = (pers + 4) * 16;
    if (width > 65536 || b >= width) return "buffer_range";
    if (!s || !s->bytes || s->width != width) return "no_shadow";
    UINT end = b + count * 16;
    if (end > width) end = width;
    if (end > s->end) end = s->end;
    if (s->start > b || end < b + need) return "shadow_range";
    *begin = b; *bytes = end - b; return nullptr;
}
/* The retargeted range as the GPU route would produce it. */
static void dgExpectedBank(const DgBankShadow *s, UINT begin, UINT bytes, UINT pers, const float delta[16],
                           const float sinv[16], float tolerance, uint8_t *out, const float sourceRounding[4] = nullptr) {
    uint32_t in[16], rows[16];
    memcpy(out, s->bytes + begin, bytes);
    memcpy(in, s->bytes + begin + pers * 16, 64);
    dgRetargetRowsCpu(in, rows, delta, sinv, tolerance, sourceRounding);
    memcpy(out + pers * 16, rows, 64);
}

struct SecondCameraCpu {
    static constexpr UINT ringSize = 8u << 20;       /* ~1 KB per draw: thousands of draws per wrap */
    static constexpr float tolerance = 1e-4f;         /* = SecondCameraGpu::tolerance */
    ComPtr<ID3D11Buffer> ring;
    UINT offset = ringSize;                           /* first bind discards */
    bool checked = false, supported = false;
    const char *unsupported = "not_checked";
    /* this frame's cameras for the split layout: Sc = S^T, Tc = T^T (game clip); counters */
    bool camsValid = false; float Sc[16]{}, Tc[16]{}, sourceRounding[4]{};
    unsigned long long splitMirrored = 0, splitExact = 0, zprojRetargeted = 0;
    bool zprojOff = false;                              /* vr_se_mode bit 256: old behaviour (rows emptied) */
    void setCams(const float S[16], const float T[16]) {
        for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) { Sc[r * 4 + c] = S[c * 4 + r]; Tc[r * 4 + c] = T[c * 4 + r]; }
        dgClipSourceRounding(S, sourceRounding);
        camsValid = true;
    }
    /* bound for one draw */
    ComPtr<ID3D11Buffer> original; ComPtr<ID3D11DeviceContext1> c1; UINT first = 0, count = 0; bool bound = false;

    bool init(ID3D11Device *d) {
        if (checked) return supported;
        checked = true;
        D3D11_FEATURE_DATA_D3D11_OPTIONS o{};
        if (FAILED(d->CheckFeatureSupport(D3D11_FEATURE_D3D11_OPTIONS, &o, sizeof o))) { unsupported = "feature_query"; return false; }
        if (!o.ConstantBufferOffsetting) { unsupported = "no_cb_offsetting"; return false; }
        if (!o.MapNoOverwriteOnDynamicConstantBuffer) { unsupported = "no_cb_no_overwrite"; return false; }
        D3D11_BUFFER_DESC b{}; b.ByteWidth = ringSize; b.Usage = D3D11_USAGE_DYNAMIC;
        b.BindFlags = D3D11_BIND_CONSTANT_BUFFER; b.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        if (FAILED(d->CreateBuffer(&b, nullptr, &ring))) { unsupported = "ring_create"; return false; }
        supported = true; unsupported = nullptr; return true;
    }





    const char *stageBound(ID3D11Device *d, ID3D11DeviceContext *c, ID3D11Buffer *buffer, UINT first, UINT count, int pers,
                           const float delta[16], const float sinv[16], DgBankLookup lookup, void *ctx, bool allowWrap,
                           UINT *f, UINT *n, int *projective) {
        if (!init(d)) return unsupported;
        if (!buffer) return "missing_cb";
        D3D11_BUFFER_DESC desc{}; buffer->GetDesc(&desc);
        const DgBankShadow *s = lookup ? lookup(buffer, ctx) : nullptr;
        UINT begin = 0, bytes = 0;
        if (pers >= 0) {
            if (const char *why = dgBankRange(desc.ByteWidth, first, count, (UINT)pers, s, &begin, &bytes)) return why;
        } else {
            if (first > 4096 || count > 4096 || !count) return "buffer_range";
            begin = first * 16;
            if (desc.ByteWidth > 65536 || begin >= desc.ByteWidth) return "buffer_range";
            if (!s || !s->bytes || s->width != desc.ByteWidth) return "no_shadow";
            UINT end = begin + count * 16;
            if (end > desc.ByteWidth) end = desc.ByteWidth;
            if (end > s->end) end = s->end;
            if (s->start > begin || end <= begin) return "shadow_range";
            bytes = end - begin;
        }
        const UINT alloc = (bytes + 255) & ~255u;
        D3D11_MAP type = D3D11_MAP_WRITE_NO_OVERWRITE;
        if (offset + alloc > ringSize) { if (!allowWrap) return "ring_wrap"; type = D3D11_MAP_WRITE_DISCARD; offset = 0; }
        D3D11_MAPPED_SUBRESOURCE m{};
        if (FAILED(c->Map(ring.Get(), 0, type, 0, &m))) return "ring_map";
        uint8_t *dst = (uint8_t *)m.pData + offset;
        memcpy(dst, s->bytes + begin, bytes);
        int ok = 1;
        if (pers >= 0) {
            uint32_t rows[16], in[16];
            memcpy(in, s->bytes + begin + pers * 16, 64);
            ok = dgRetargetRowsCpu(in, rows, delta, sinv, tolerance, sourceRounding);
            if (!ok && camsValid && pers >= 4) {         /* the split layout (effect shaders) */
                float v[16], pr[16], vo[16], po[16];
                memcpy(v, s->bytes + begin + (pers - 4) * 16, 64); memcpy(pr, in, 64);
                int kind = dgSplitRetarget(v, pr, Sc, Tc, vo, po);
                if (kind) {
                    memcpy(dst + (pers - 4) * 16, vo, 64); memcpy(rows, po, 64); ok = 1;
                    if (kind == 1) ++splitMirrored; else ++splitExact;
                }
            }
            if (!ok && !zprojOff && dgZProjRetarget(in, rows, delta, sinv, tolerance)) { ok = 1; ++zprojRetargeted; }
            memcpy(dst + pers * 16, rows, 64);            /* write-combined memory: written, never read */
        }
        c->Unmap(ring.Get(), 0);
        *f = offset / 16; *n = alloc / 16;
        offset += alloc;
        if (projective) *projective = !ok;
        return nullptr;
    }
    /* The compute probe only implements the affine bank layout. A draw that
       needs the existing split V/P conversion must stay on the CPU route;
       substituting a compute bank for this one draw would erase its P rows.
       Inspect only periodic probe candidates, with the same range/family
       guards as stageBound. Missing shadows still take the normal fallback. */
    bool usesSplitBank(ID3D11Device *d, ID3D11DeviceContext *c, const float delta[16], const float sinv[16],
                       UINT pers, DgBankLookup lookup, void *ctx) {
        if (!camsValid || !init(d)) return false;
        if (!c1 && FAILED(c->QueryInterface(IID_PPV_ARGS(&c1)))) return false;
        ComPtr<ID3D11Buffer> buffer; UINT f = 0, n = 0;
        c1->VSGetConstantBuffers1(0, 1, &buffer, &f, &n);
        if (!buffer) return false;
        D3D11_BUFFER_DESC desc{}; buffer->GetDesc(&desc);
        const DgBankShadow *s = lookup ? lookup(buffer.Get(), ctx) : nullptr;
        UINT begin = 0, bytes = 0;
        if (dgBankRange(desc.ByteWidth, f, n, pers, s, &begin, &bytes)) return false;
        uint32_t in[16], out[16]; memcpy(in, s->bytes + begin + pers * 16, 64);
        if (dgRetargetRowsCpu(in, out, delta, sinv, tolerance, sourceRounding)) return false;
        if (!zprojOff && dgZProjRetarget(in, out, delta, sinv, tolerance)) return true;   /* the compute probe would empty it */
        float v[16], p[16], vo[16], po[16];
        memcpy(v, s->bytes + begin + (pers - 4) * 16, 64); memcpy(p, in, 64);
        return dgSplitRetarget(v, p, Sc, Tc, vo, po) != 0;
    }
    /* Binds the retargeted copy of the bound cb0 range for the draw about to be
       replayed. Returns nullptr when bound (projective = 1 when the rows were
       emptied), else the reason nothing was bound. */
    const char *bind(ID3D11Device *d, ID3D11DeviceContext *c, const float delta[16], const float sinv[16],
                     UINT pers, DgBankLookup lookup, void *ctx, int *projective) {
        if (bound) return "already_bound";
        if (!init(d)) return unsupported;
        if (!c1 && FAILED(c->QueryInterface(IID_PPV_ARGS(&c1)))) return "no_context1";
        original.Reset(); first = count = 0;
        c1->VSGetConstantBuffers1(0, 1, &original, &first, &count);
        if (!original) return "missing_cb0";
        UINT f = 0, n = 0;
        if (const char *why = stageBound(d, c, original.Get(), first, count, (int)pers, delta, sinv, lookup, ctx, true, &f, &n, projective)) {
            original.Reset(); return why;
        }
        ID3D11Buffer *r = ring.Get();
        c1->VSSetConstantBuffers1(0, 1, &r, &f, &n);
        bound = true;
        return nullptr;
    }
    void restore(ID3D11DeviceContext *c) {
        (void)c;
        if (!bound) return;
        ID3D11Buffer *p = original.Get();
        c1->VSSetConstantBuffers1(0, 1, &p, &first, &count);
        bound = false; original.Reset();
    }
};
