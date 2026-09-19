// dg_ui2d.inl - per-eye placement of the game's flat 2D sprites (2026-09-18).
//
// Measured with dg_cb_probe (artifacts/cb_probe_20260918): the camera
// viewfinder overlay and the HUD are passes of non-indexed 4-vertex quads
// whose vertex-shader register bank holds PS2 screen-space constants and no
// view-projection:
//     c1..c4 identity      c5 = (1/256, -1/224, 0, 1)  scale
//     c6 = (-1, 1, 1, 0)   NDC offset                 c8 = (2048, 2048, 4096, 0)
// so every quad lands on the SAME NDC position in both eye images. The eye
// frusta are asymmetric and mirrored (projection m20 ~ -/+0.24): NDC x = 0
// lies ~15 degrees off straight-ahead in opposite directions per eye, and the
// overlay cannot be fused - "I see the camera overlay twice".
//
// Correction: for those draws only, x' = s*x + x0 (+ convergence), y' = s*y,
// where x0 is the NDC x of straight-ahead for the eye being rendered. x0 is
// not taken from our own bookkeeping - rendering lags the camera seam by two
// updates - but read off the world*VP matrices the game itself uploads for
// the same frame (c29..c32): x0 = (cx . cw) / (cw . cw).
//
// Mechanism. The game keeps the register bank on the CPU and uploads it in
// 0x45AA0: Map(WRITE_DISCARD), memcpy(mapped, bank, len), Unmap. The `call
// memcpy` there is retargeted to dg_ui2d_upload, which copies and then only
// LEARNS (x0 from perspective banks; "the game uploaded, the GPU copy is
// pristine"). The patch itself happens at draw time, on the two native draw
// call sites this module already owns: if the bank carries the 2D signature,
// the GPU buffer is re-uploaded from the bank - patched for wanted sprites,
// pristine for everything else that shares the constants (the final
// full-screen blit and a background quad do). The CPU bank is never written,
// so nothing accumulates and nothing leaks into the other eye.
//
// Which draws are sprites is decided by vertex-shader BYTECODE hash
// (CreateVertexShader is observed, FNV-1a 64), because the constants alone
// do not separate sprites from blits and shader pointers are per session.
// With no hash list configured nothing is patched; candidates are counted
// per hash for the heartbeat so the list can be set from the log.

// ---------------------------------------------------------------- config
struct Ui2dCfg { LONG on = 0, scaleMils = 750, convE5 = 1200, sign = 1, hold = 0; UINT64 vs[8]{}; unsigned vsCount = 0; };
Ui2dCfg ui2dCfg; SRWLOCK ui2dCfgLock = SRWLOCK_INIT;

// ---------------------------------------------------------------- shader registry
struct VsEntry { void *ptr; UINT64 hash; UINT32 size; BYTE *bytes; };
constexpr unsigned vsCap = 8192;                 // power of two
constexpr size_t vsBytesMax = 48u << 20;
VsEntry vsTable[vsCap]; SRWLOCK vsLock = SRWLOCK_INIT; size_t vsBytesTotal = 0; unsigned vsKnown = 0;
Hook vsHook;
using VsCreate = HRESULT(STDMETHODCALLTYPE *)(ID3D11Device *, const void *, SIZE_T, ID3D11ClassLinkage *, ID3D11VertexShader **);

UINT64 fnv1a64(const void *p, size_t n) {
    UINT64 h = 1469598103934665603ull; auto b = (const BYTE *)p;
    for (size_t i = 0; i < n; i++) { h ^= b[i]; h *= 1099511628211ull; }
    return h ? h : 1;
}
unsigned vsSlot(void *ptr) { return (unsigned)(((uintptr_t)ptr >> 4) * 2654435761u) & (vsCap - 1); }
void vsRemember(void *ptr, const void *code, size_t n) {
    UINT64 h = fnv1a64(code, n);
    AcquireSRWLockExclusive(&vsLock);
    for (unsigned i = 0, s = vsSlot(ptr); i < vsCap; i++, s = (s + 1) & (vsCap - 1)) {
        VsEntry &e = vsTable[s];
        if (e.ptr && e.ptr != ptr) continue;
        if (!e.ptr) vsKnown++;
        if (e.bytes) { vsBytesTotal -= e.size; HeapFree(GetProcessHeap(), 0, e.bytes); e.bytes = nullptr; }
        e.ptr = ptr; e.hash = h; e.size = (UINT32)n;
        if (vsBytesTotal + n <= vsBytesMax) { e.bytes = (BYTE *)HeapAlloc(GetProcessHeap(), 0, n); if (e.bytes) { memcpy(e.bytes, code, n); vsBytesTotal += n; } }
        break;
    }
    ReleaseSRWLockExclusive(&vsLock);
}
UINT64 vsHashOf(void *ptr, const BYTE **bytes, UINT32 *size) {
    UINT64 h = 0; if (!ptr) return 0;
    AcquireSRWLockShared(&vsLock);
    for (unsigned i = 0, s = vsSlot(ptr); i < vsCap; i++, s = (s + 1) & (vsCap - 1)) {
        const VsEntry &e = vsTable[s];
        if (!e.ptr) break;
        if (e.ptr == ptr) { h = e.hash; if (bytes) *bytes = e.bytes; if (size) *size = e.size; break; }
    }
    ReleaseSRWLockShared(&vsLock);
    return h;
}
HRESULT STDMETHODCALLTYPE createVs(ID3D11Device *d, const void *code, SIZE_T n, ID3D11ClassLinkage *l, ID3D11VertexShader **out) {
    HRESULT hr = ((VsCreate)vsHook.next)(d, code, n, l, out);
    if (SUCCEEDED(hr) && out && *out && code && n && n < (16u << 20)) vsRemember(*out, code, n);
    return hr;
}

// ---------------------------------------------------------------- pure parts (desk-testable)
// NDC x of straight-ahead, from a transposed world*VP at c29..c32. 0 = refuse.
// Reason codes of ui2dClassify: 0 learned, 1 no projection row, 2 degenerate
// axes, 3 SYMMETRIC frustum (a mono camera: the game's own, not an eye),
// 4 offset out of range, 5 scale out of range, 6 axes not orthogonal.
int ui2dClassify(const float *f, float *x0out, float *xwOut);
bool ui2dLearnX0(const float *f, float *x0out) { float xw; return ui2dClassify(f, x0out, &xw) == 0; }
int ui2dClassifyRows(const float *cx, const float *cy, const float *cw, float *x0out, float *xwOut);
int ui2dClassify(const float *f, float *x0out, float *xwOut) { return ui2dClassifyRows(f + 116, f + 120, f + 128, x0out, xwOut); }
// Object eye (2026-09-18 night). The main world shaders (544F1B61.., 0674FB1F..)
// do not read c29 at all: their reflection says position = gVS_Pers(c20, byte
// 320) * gVS_Mat0(c16) * v, with gVS_Pers = world * eye_pers. So the c29 votes
// say nothing about the bulk of the scene. The same test on the c20 rows is
// tallied per frame, stall-free, to see with WHICH eye the scene is drawn when
// stereo breaks from certain view angles (desktop mirror goes mono there).
LONG ui2dObjPos = 0, ui2dObjNeg = 0;
volatile LONG ui2dLastObjSign = 0, ui2dLastObjPos = 0, ui2dLastObjNeg = 0;   // sign: +1/-1, 2 = mixed, 0 = none
int ui2dClassifyRows(const float *cx, const float *cy, const float *cw, float *x0out, float *xwOut) {
    double ww = (double)cw[0] * cw[0] + (double)cw[1] * cw[1] + (double)cw[2] * cw[2];
    *xwOut = 0;
    if (!(ww > 1e-12) || !(ww < 1e12)) return 1;
    double xw = ((double)cx[0] * cw[0] + (double)cx[1] * cw[1] + (double)cx[2] * cw[2]) / ww;
    double yw = ((double)cy[0] * cw[0] + (double)cy[1] * cw[1] + (double)cy[2] * cw[2]) / ww;
    double a[3], b[3], aa = 0, bb = 0, ab = 0;
    for (int i = 0; i < 3; i++) { a[i] = cx[i] - xw * cw[i]; b[i] = cy[i] - yw * cw[i]; aa += a[i] * a[i]; bb += b[i] * b[i]; ab += a[i] * b[i]; }
    if (!(aa > 0) || !(bb > 0)) return 2;
    double m00 = sqrt(aa / ww), m11 = sqrt(bb / ww);
    *xwOut = (float)xw;
    if (!(m00 >= 0.3 && m00 <= 3.0 && m11 >= 0.3 && m11 <= 3.0)) return 5;
    if (fabs(ab) > 0.02 * sqrt(aa * bb)) return 6;                                       // x and y view axes must be orthogonal
    if (!(fabs(yw) <= 0.6) || !(fabs(xw) <= 0.6)) return 4;
    if (!(fabs(xw) >= 0.02)) return 3;                                                   // symmetric (mono/theater) frusta are refused
    *x0out = (float)xw; return 0;
}
bool ui2dSignature(const float *f) {
    static const float ident[16] = { 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    return !memcmp(f + 4, ident, sizeof ident) && f[32] == 2048.0f && f[33] == 2048.0f && f[34] == 4096.0f && f[35] == 0.0f &&
           f[23] == 1.0f && f[22] == 0.0f && f[20] > 0.0f && f[20] < 0.05f;
}
void ui2dPatch(float *f, float x0, float scale, float conv, int sign) {
    float sx0 = sign < 0 ? -x0 : x0;
    float shift = sx0 + (sx0 >= 0 ? conv : -conv);    // nearer = further toward the nasal side, which is the side x0 points to
    f[20] *= scale; f[21] *= scale;
    f[24] = f[24] * scale + shift; f[25] *= scale;
}
// The upload call site: 4C 63 83 2C 09 00 00 / 48 8B 93 18 09 00 00 / 48 8B 4C 24 30 / E8 rel32 / 48 8B 05
bool ui2dFindSite(const BYTE *p, size_t n, size_t *callOffset) {
    static const BYTE head[] = { 0x4c,0x63,0x83,0x2c,0x09,0x00,0x00, 0x48,0x8b,0x93,0x18,0x09,0x00,0x00, 0x48,0x8b,0x4c,0x24,0x30, 0xe8 };
    unsigned hits = 0;
    for (size_t i = 0; i + sizeof head + 7 <= n; i++) {
        if (p[i] != 0x4c || memcmp(p + i, head, sizeof head)) continue;
        if (p[i + 24] != 0x48 || p[i + 25] != 0x8b || p[i + 26] != 0x05) continue;
        hits++; *callOffset = i + 19;
    }
    return hits == 1;
}

// ---------------------------------------------------------------- live state (render thread)
const void *ui2dBank = nullptr; size_t ui2dBankLen = 0;
volatile LONG ui2dGpuPatched = 0; float ui2dGpuX0 = 0;
float ui2dX0 = 0; DWORD ui2dX0Tick = 0; bool ui2dHaveX0 = false;
// Flicker fix (2026-09-18 pm). Live the overlay fused but now and then jumped
// back to double for a frame: x0 was simply the LAST perspective matrix the
// game uploaded before the sprite pass, and some pass carries another
// projection (+0.4619 was seen once against the display's 0.2425). Now:
//  - the SIGN is the majority of all perspective uploads since the last
//    Present (the main pass alone casts hundreds of votes; a stray pass cannot
//    flip it), and at least ui2dMinVotes are required, else the frame counts
//    as having no camera (menus, loading) and nothing is patched;
//  - the MAGNITUDE is the display's own frustum asymmetry, published by the
//    camera seam from the FOV it submits to the headset. It does not change
//    with the camera item's zoom, which scales the game's projection but not
//    the layer the sprites end up in. Without it the learned value is used.
constexpr LONG ui2dMinVotes = 8;
LONG ui2dVotesPos = 0, ui2dVotesNeg = 0;
// Camera-less frames (2026-09-18 evening). Measured live with B9846587: the
// Present eye label ALWAYS matched the rendered frustum (0 of 1132 wrong), but
// about one Present in ten carries no perspective uploads at all while the
// game still draws its sprites. Those frames fell through the "no camera =
// patch nothing" rule and showed the HUD unplaced for one frame: the flicker,
// in the camera and in normal play alike. Such a Present re-reads the same
// camera handoff, so it goes to the headset under the PREVIOUS frame's eye.
// vr_ui2d_hold: 0 = old rule, 1 (default) = place the sprites as in the last
// voted frame, 2 = as its opposite eye. Bounded by ui2dHoldMs so menus and
// load screens still get nothing.
// Live 173C97DB: neither hold mode removed the flicker, and EVERY voted frame
// has listed-shader draws ahead of its votes (the background quad that opens
// the main pass) which hold then misplaces. Default back to 0. What a
// camera-less Present actually contains is measured instead: all native draws
// per frame, averaged apart for voted and camera-less frames.

// frames are: ordinary eye frames (SYMMETRIC 0, the few votes they have carry
// the right sign) in which fewer than ui2dMinVotes uploads pass the strict
// rigid-world test, because most of the view is scaled or skinned geometry.
// It depends on where the player looks, hence "tied to rotation". So the
// votes stay the truth where they exist, and TEACH which sign belongs to which
// eye of the camera seam's handoff (dg_ui2d_eye, published on every valid
// handoff write); a frame without enough votes whose scene pass is under way
// (ui2dSceneMin projection-row uploads, so the background quad that opens the
// main pass stays untouched) takes its sign from the handoff eye instead.
constexpr LONG ui2dSceneMin = 8, ui2dEyeLearnMin = 50;
constexpr DWORD ui2dEyeMs = 250;
volatile LONG ui2dEyeNow = 0; volatile LONG ui2dEyeTick = 0;      // 0 none, 1 left, 2 right
LONG ui2dEyeN[2][2] = {};                                         // [eye][sign < 0], taught by voted frames
LONG ui2dFrameScene = 0, ui2dFrameEyeAtDraw = 0;
volatile LONG ui2dEyeAgree = 0, ui2dEyeDisagree = 0, ui2dEyeUsed = 0;
// +1 / -1 once one sign dominates that eye 4:1 over at least ui2dEyeLearnMin frames, else 0.
int ui2dEyeSign(LONG eye) {
    if (eye != 1 && eye != 2) return 0;
    LONG a = ui2dEyeN[eye - 1][0], b = ui2dEyeN[eye - 1][1];
    if (a + b < ui2dEyeLearnMin) return 0;
    return a >= 4 * b ? 1 : b >= 4 * a ? -1 : 0;
}
LONG ui2dEyeFresh() {
    LONG e = InterlockedCompareExchange(&ui2dEyeNow, 0, 0);
    return (e && GetTickCount() - (DWORD)InterlockedCompareExchange(&ui2dEyeTick, 0, 0) <= ui2dEyeMs) ? e : 0;
}
LONG ui2dFrameDrawsAll = 0;
// Back-buffer probe (2026-09-18 night). 27BF42CB live: the Presents that carry
// an unchanged picture still issue 1000+ draws with the other eye's constants,
// so the draw count cannot tell them apart. Counted instead: the draws whose
// render target IS the swap chain's back buffer, and the texture the first of
// them samples. Every non-indexed draw is checked, every 64th indexed one.
void *ui2dBackbufferPtr = nullptr;                 // resource identity only, never dereferenced
// Final-buffer trace (2A0C7AF7 live: the back buffer gets exactly one blit per
// frame, from two textures A/B that alternate every Present - also from the bad
// angle, where the PICTURE nevertheless repeats. So something writes one eye's
// picture into the other eye's final texture). While armed (dg_ui2d_trace, for
// the frames of an eye dump) every draw whose render target is A or B is
// grouped by target, vertex shader and whether it SAMPLES the other / the same
// final texture, and logged at the frame's Present.
// Previous-frame feedback (2026-09-18 night, found with the final-buffer trace).
// The engine keeps two final textures and blits them to the back buffer in
// turn; with alternate-eye rendering one is always the left eye, the other the
// right. Once per frame a 4-vertex sprite draws the OTHER final texture - the
// previous frame, i.e. the other EYE - full screen into the current one (the
// scene-history blur). From some view angles it is opaque: the right-eye frame
// becomes a byte-identical copy of the left-eye picture (mono on the desktop
// mirror, double in the headset), while every matrix and every other draw is
// healthy. In stereo a previous-frame sample is wrong by construction, so that
// one draw is not forwarded. The final textures are learned from the blit.
// Second trace (D21112F2 live: with the feedback sprite gone the right-eye
// picture is STILL a byte-identical copy of the left one from the bad angle, and
// no draw into a final texture samples the other one). While a trace is armed:
//  - every texture copy / resolve that touches a final texture or the back
//    buffer is noted (CopyResource, CopySubresourceRegion, ResolveSubresource on
//    the immediate context; chained like the shader registry, observation only);

//    (pointer, size), blend state and write mask.
struct Ui2dCopyNote { const char *kind; void *dst, *src; LONG at; };
struct Ui2dQuadNote { LONG at; int target; UINT64 vs; void *src; UINT sw, sh; int blend, sb, db; UINT mask; };
Ui2dCopyNote ui2dCopies[48]; volatile LONG ui2dCopyCount = 0; Ui2dQuadNote ui2dQuads[64]; unsigned ui2dQuadCount = 0;   // copies may come from another thread (XR capture)
// D3D11 swaps the immediate context's VTABLE when multithread protection is switched (seen on WARP: a hook
// written at install time was gone later), so the observers are kept per vtable and re-armed with each trace.
struct Ui2dCopySet { void **vt; Hook reg, res, rsv; };
Ui2dCopySet ui2dCopySets[4]; unsigned ui2dCopySetCount = 0;
Ui2dCopySet *ui2dCopySetOf(ID3D11DeviceContext *c) { void **vt = *(void ***)c; for (unsigned i = 0; i < ui2dCopySetCount; i++) if (ui2dCopySets[i].vt == vt) return &ui2dCopySets[i]; return ui2dCopySetCount ? &ui2dCopySets[0] : nullptr; }
using CopyRes = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11Resource *, ID3D11Resource *);
using CopyReg = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11Resource *, UINT, UINT, UINT, UINT, ID3D11Resource *, UINT, const D3D11_BOX *);
using ResolveFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11Resource *, UINT, ID3D11Resource *, UINT, DXGI_FORMAT);
extern volatile LONG ui2dTraceLeft; extern void *ui2dTraceTarget[2]; extern void *ui2dBackbufferPtr; extern LONG ui2dFrameDrawsAll;
const char *ui2dTexName(void *p) { return !p ? "null" : p == ui2dTraceTarget[0] ? "A" : p == ui2dTraceTarget[1] ? "B" : p == ui2dBackbufferPtr ? "BACKBUFFER" : "other"; }
void ui2dNoteCopy(const char *kind, void *dst, void *src) {
    if (ui2dTraceLeft <= 0) return;
    bool rel = dst == ui2dTraceTarget[0] || dst == ui2dTraceTarget[1] || dst == ui2dBackbufferPtr || src == ui2dTraceTarget[0] || src == ui2dTraceTarget[1] || src == ui2dBackbufferPtr;
    if (rel) { LONG i = InterlockedIncrement(&ui2dCopyCount) - 1; if (i >= 0 && i < 48) ui2dCopies[i] = Ui2dCopyNote{ kind, dst, src, ui2dFrameDrawsAll }; }
}
void STDMETHODCALLTYPE copyRes(ID3D11DeviceContext *c, ID3D11Resource *dst, ID3D11Resource *src) { ui2dNoteCopy("CopyResource", dst, src); ((CopyRes)ui2dCopySetOf(c)->res.next)(c, dst, src); }
void STDMETHODCALLTYPE copyReg(ID3D11DeviceContext *c, ID3D11Resource *dst, UINT ds, UINT x, UINT y, UINT z, ID3D11Resource *src, UINT ss, const D3D11_BOX *box) {
    ui2dNoteCopy("CopySubresourceRegion", dst, src); ((CopyReg)ui2dCopySetOf(c)->reg.next)(c, dst, ds, x, y, z, src, ss, box); }
void STDMETHODCALLTYPE resolveSub(ID3D11DeviceContext *c, ID3D11Resource *dst, UINT ds, ID3D11Resource *src, UINT ss, DXGI_FORMAT f) {
    ui2dNoteCopy("ResolveSubresource", dst, src); ((ResolveFn)ui2dCopySetOf(c)->rsv.next)(c, dst, ds, src, ss, f); }
// Present thread (install, and every time a trace is armed): make sure the context's CURRENT vtable carries the observers.
void ui2dEnsureCopyHooks() {
    if (!context.Get() || stopping) return;
    void **ct = *(void ***)context.Get();
    if (ct[47] == (void *)copyRes) return;
    // The runtime may rewrite the slots of a vtable we already hold (seen on WARP): reuse that set, do not burn a new one.
    for (unsigned i = 0; i < ui2dCopySetCount; i++) if (ui2dCopySets[i].vt == ct) {
        Ui2dCopySet &old = ui2dCopySets[i]; Hook *hs[3] = { &old.reg, &old.res, &old.rsv };
        unsigned slots[3] = { 46, 47, 57 }; void *ours[3] = { (void *)copyReg, (void *)copyRes, (void *)resolveSub }; unsigned again = 0;
        for (int k = 0; k < 3; k++) if (ct[slots[k]] != ours[k]) { *hs[k] = { ct + slots[k], ct[slots[k]], ours[k] }; if (writeSlot(*hs[k], hs[k]->next, hs[k]->ours)) again++; }
        if (logger) logger("ui2d: copy observers re-armed %u slots on vtable %p\r\n", again, (void *)ct);
        return;
    }
    if (ui2dCopySetCount >= 4) { if (logger) logger("ui2d: copy observers: no free vtable set\r\n"); return; }
    Ui2dCopySet &set = ui2dCopySets[ui2dCopySetCount]; set.vt = ct;
    set.reg = { ct + 46, ct[46], (void *)copyReg }; set.res = { ct + 47, ct[47], (void *)copyRes }; set.rsv = { ct + 57, ct[57], (void *)resolveSub };
    ui2dCopySetCount++;                                            // published before the slots change: a call through them must find its set
    unsigned ok = 0; for (Hook *h : { &set.reg, &set.res, &set.rsv }) { if (h->next && writeSlot(*h, h->next, h->ours)) ok++; else h->slot = nullptr; }
    if (logger) logger("ui2d: copy observers attached %u of 3 on vtable %p (set %u)\r\n", ok, (void *)ct, ui2dCopySetCount);
}
volatile LONG ui2dFeedbackOn = 0, ui2dFeedbackSkipped = 0, ui2dFeedbackChecked = 0;
void *ui2dFinalTex[2] = { nullptr, nullptr };
// true = do not forward this non-indexed draw.
void ui2dTraceQuad(ID3D11DeviceContext *c) {
    MultiLock tqLock;
    ID3D11RenderTargetView *rtv = nullptr; c->OMGetRenderTargets(1, &rtv, nullptr);
    if (!rtv) return;
    ID3D11Resource *res = nullptr; rtv->GetResource(&res);
    int t = res && res == ui2dTraceTarget[0] ? 0 : res && res == ui2dTraceTarget[1] ? 1 : -1;
    if (t >= 0 && ui2dQuadCount < 64) {
        Ui2dQuadNote q{}; q.at = ui2dFrameDrawsAll; q.target = t;
        ID3D11ShaderResourceView *srv = nullptr; c->PSGetShaderResources(0, 1, &srv);
        if (srv) { ID3D11Resource *sr = nullptr; srv->GetResource(&sr); q.src = sr;
            if (sr) { D3D11_RESOURCE_DIMENSION dim; sr->GetType(&dim); if (dim == D3D11_RESOURCE_DIMENSION_TEXTURE2D) { D3D11_TEXTURE2D_DESC td; ((ID3D11Texture2D *)sr)->GetDesc(&td); q.sw = td.Width; q.sh = td.Height; } sr->Release(); }
            srv->Release(); }
        ID3D11VertexShader *vs = nullptr; ID3D11ClassInstance *ci[1]; UINT nci = 0; c->VSGetShader(&vs, ci, &nci); q.vs = vsHashOf(vs, nullptr, nullptr); if (vs) vs->Release();
        ID3D11BlendState *bs = nullptr; FLOAT bf[4]; UINT sm = 0; c->OMGetBlendState(&bs, bf, &sm);
        if (bs) { D3D11_BLEND_DESC bd; bs->GetDesc(&bd); q.blend = bd.RenderTarget[0].BlendEnable; q.sb = bd.RenderTarget[0].SrcBlend; q.db = bd.RenderTarget[0].DestBlend; q.mask = bd.RenderTarget[0].RenderTargetWriteMask; bs->Release(); } else q.mask = 0xF;
        ui2dQuads[ui2dQuadCount++] = q;
    }
    if (res) res->Release();
    rtv->Release();
}
bool ui2dSkipDraw(ID3D11DeviceContext *c, UINT vertexCount) {
    if (ui2dTraceLeft > 0 && vertexCount == 4 && c == context.Get() && !stopping) ui2dTraceQuad(c);
    if (!ui2dFeedbackOn || vertexCount != 4 || !ui2dFinalTex[0] || !ui2dFinalTex[1] || c != context.Get() || stopping) return false;
    MultiLock fbLock; bool skip = false;
    ID3D11RenderTargetView *rtv = nullptr; c->OMGetRenderTargets(1, &rtv, nullptr);
    if (!rtv) return false;
    ID3D11Resource *res = nullptr; rtv->GetResource(&res);
    int t = res == ui2dFinalTex[0] ? 0 : res == ui2dFinalTex[1] ? 1 : -1;
    if (t >= 0) {
        InterlockedIncrement(&ui2dFeedbackChecked);
        ID3D11ShaderResourceView *srv = nullptr; c->PSGetShaderResources(0, 1, &srv);
        if (srv) { ID3D11Resource *sr = nullptr; srv->GetResource(&sr); skip = sr && sr == ui2dFinalTex[1 - t]; if (sr) sr->Release(); srv->Release(); }
    }
    if (res) res->Release();
    rtv->Release();
    if (skip) InterlockedIncrement(&ui2dFeedbackSkipped);
    return skip;
}
struct Ui2dTraceGroup { int target, rel; UINT64 vs; LONG count, first, last; UINT vpw, vph; };
volatile LONG ui2dTraceLeft = 0; void *ui2dTraceTarget[2] = { nullptr, nullptr };
Ui2dTraceGroup ui2dTraceGroups[48]; unsigned ui2dTraceCount = 0; LONG ui2dTraceOverflow = 0, ui2dTraceFrameNo = 0;
LONG ui2dFrameBbDraws = 0, ui2dFrameBbIndexed = 0; void *ui2dFrameBbSrv = nullptr;
volatile LONG ui2dLastBbDraws = -1, ui2dLastBbIndexed = 0; void *ui2dLastBbSrv = nullptr;
// What a camera-less frame uploaded, by ui2dClassify reason, kept for the
// Present callback (dg_ui2d_last_frame_detail): draws, uploads, reasons 0..6,
// and the last offset seen on a symmetric / out-of-range projection.
struct Ui2dFrameDetail { LONG draws, uploads, why[7], pos, neg, sprites; float symXw, farXw; };
Ui2dFrameDetail ui2dCur{}, ui2dLast{};
volatile LONG64 ui2dDrawsVoted = 0, ui2dDrawsNoCam = 0;
volatile LONG ui2dNoCamMin = 0x7fffffff, ui2dNoCamMax = 0;
constexpr DWORD ui2dHoldMs = 250;
float ui2dPrevX0 = 0; DWORD ui2dPrevTick = 0; bool ui2dHavePrev = false;
LONG ui2dFrameSprites = 0, ui2dFrameHeld = 0, ui2dFrameEarly = 0;   // this frame: sprites that passed every filter / placed from the held value / drawn before the frame had its votes
volatile LONG ui2dFramesVoted = 0, ui2dFramesZero = 0, ui2dFramesFew = 0, ui2dFramesHeld = 0, ui2dFramesBare = 0, ui2dFramesEarly = 0, ui2dHeldDraws = 0;
volatile LONG ui2dDisplayMagE6 = 0;                // |NDC x of straight-ahead| * 1e6, 0 = unknown
UINT ui2dBbW = 0, ui2dBbH = 0;
BYTE *ui2dSite = nullptr; INT32 ui2dOldRel = 0, ui2dNewRel = 0; void *ui2dStub = nullptr; bool ui2dSiteInstalled = false;
volatile LONG ui2dUploads = 0, ui2dPersp = 0, ui2dSigDraws = 0, ui2dPatched = 0, ui2dRestored = 0, ui2dErrors = 0;
volatile LONG ui2dSkipKind = 0, ui2dSkipStale = 0, ui2dSkipViewport = 0, ui2dSkipDepth = 0, ui2dSkipVs = 0;
struct Ui2dCand { UINT64 hash; LONG count; }; Ui2dCand ui2dCand[12];

void ui2dCandidate(UINT64 h) {
    for (auto &c : ui2dCand) { if (c.hash == h) { c.count++; return; } if (!c.hash) { c.hash = h; c.count = 1; return; } }
}

void *ui2dUpload(void *dst, const void *src, size_t n) {
    memcpy(dst, src, n);
    InterlockedIncrement(&ui2dUploads);
    ui2dBank = src; ui2dBankLen = n;
    InterlockedExchange(&ui2dGpuPatched, 0);                 // the game's own bytes are on the GPU now
    float x0 = 0, xw = 0; int why = n >= 0x240 ? ui2dClassify((const float *)src, &x0, &xw) : 1;
    ui2dCur.uploads++; ui2dCur.why[why]++;
    if (n >= 0x180) { float ox0 = 0, oxw = 0; if (ui2dClassifyRows((const float *)src + 80, (const float *)src + 84, (const float *)src + 92, &ox0, &oxw) == 0) { if (ox0 > 0) ui2dObjPos++; else ui2dObjNeg++; } }
    if (why == 3) ui2dCur.symXw = xw; else if (why == 4) ui2dCur.farXw = xw;
    if (why == 0 || why == 5 || why == 6) ui2dFrameScene++;      // a projection row: the scene pass is under way
    if (why == 0) {
        if (x0 > 0) ui2dVotesPos++; else ui2dVotesNeg++;
        LONG pos = ui2dVotesPos, neg = ui2dVotesNeg;
        if (pos + neg >= ui2dMinVotes && pos != neg) {
            LONG magE6 = InterlockedCompareExchange(&ui2dDisplayMagE6, 0, 0);
            float mag = magE6 ? (float)magE6 / 1000000.0f : (float)fabs(x0);
            ui2dX0 = pos > neg ? mag : -mag; ui2dX0Tick = GetTickCount(); ui2dHaveX0 = true;
        }
        InterlockedIncrement(&ui2dPersp);
    }
    return dst;
}
// Frame boundary (Present thread = the draw thread): the votes are per frame.
// The finished frame's verdict is kept for the Present callback, which runs
// right after this: +1 / -1 = which frustum the frame was rendered with
// (majority of its perspective uploads), 0 = no camera this frame.
volatile LONG ui2dLastFrameSign = 0;
void ui2dOnPresent() {
    if (ui2dTraceLeft > 0) {
        if (logger) {
            logger("ui2d trace frame %ld: %u groups (overflow %ld), %ld draws; A=%p B=%p\r\n", ui2dTraceFrameNo, ui2dTraceCount, ui2dTraceOverflow, ui2dFrameDrawsAll, ui2dTraceTarget[0], ui2dTraceTarget[1]);
            for (unsigned i = 0; i < ui2dTraceCount; i++) { const Ui2dTraceGroup &g = ui2dTraceGroups[i];
                logger("ui2d trace   into %s  samples %s  vs %016llX  draws %ld (#%ld..#%ld)  viewport %ux%u\r\n", g.target == 0 ? "A" : g.target == 1 ? "B" : "BACKBUFFER",
                       g.target < 2 ? ((g.rel & 1) ? ((g.rel & 2) ? "OTHER+self" : "OTHER final texture") : (g.rel & 2) ? "self" : "-") : ((g.rel & 4) ? "A" : (g.rel & 8) ? "B" : "-"),
                       (unsigned long long)g.vs, g.count, g.first, g.last, g.vpw, g.vph); }
        }
        if (logger) {
            for (unsigned i = 0; i < ui2dQuadCount; i++) { const Ui2dQuadNote &q = ui2dQuads[i];
                logger("ui2d trace   quad #%ld into %s  vs %016llX  source %s %p %ux%u  blend %d (src %d dst %d) mask %X\r\n", q.at, q.target ? "B" : "A",
                       (unsigned long long)q.vs, ui2dTexName(q.src), q.src, q.sw, q.sh, q.blend, q.sb, q.db, q.mask); }
            LONG nc = InterlockedCompareExchange(&ui2dCopyCount, 0, 0); if (nc > 48) nc = 48;
            for (LONG i = 0; i < nc; i++) { const Ui2dCopyNote &k = ui2dCopies[i];
                logger("ui2d trace   COPY after draw #%ld: %s  %s (%p) <- %s (%p)\r\n", k.at, k.kind, ui2dTexName(k.dst), k.dst, ui2dTexName(k.src), k.src); }
        }
        ui2dQuadCount = 0; InterlockedExchange(&ui2dCopyCount, 0);
        ui2dTraceCount = 0; ui2dTraceOverflow = 0; ui2dTraceFrameNo++; InterlockedDecrement(&ui2dTraceLeft);
    }
    LONG pos = ui2dVotesPos, neg = ui2dVotesNeg;
    InterlockedExchange(&ui2dLastFrameSign, (pos + neg >= ui2dMinVotes && pos != neg) ? (pos > neg ? 1 : -1) : 0);
    if (ui2dHaveX0) { ui2dPrevX0 = ui2dX0; ui2dPrevTick = ui2dX0Tick; ui2dHavePrev = true; InterlockedIncrement(&ui2dFramesVoted); InterlockedAdd64(&ui2dDrawsVoted, ui2dFrameDrawsAll); }
    else {
        InterlockedIncrement(pos + neg ? &ui2dFramesFew : &ui2dFramesZero); InterlockedAdd64(&ui2dDrawsNoCam, ui2dFrameDrawsAll);
        if (ui2dFrameDrawsAll < ui2dNoCamMin) ui2dNoCamMin = ui2dFrameDrawsAll;
        if (ui2dFrameDrawsAll > ui2dNoCamMax) ui2dNoCamMax = ui2dFrameDrawsAll;
    }
    if (ui2dHaveX0 && ui2dFrameEyeAtDraw) { LONG &n = ui2dEyeN[ui2dFrameEyeAtDraw - 1][ui2dX0 < 0 ? 1 : 0]; if (n < 1000000) n++; }
    ui2dFrameScene = 0; ui2dFrameEyeAtDraw = 0;
    { LONG op = ui2dObjPos, on = ui2dObjNeg, hi = op > on ? op : on, lo = op > on ? on : op;
      InterlockedExchange(&ui2dLastObjPos, op); InterlockedExchange(&ui2dLastObjNeg, on);
      InterlockedExchange(&ui2dLastObjSign, op + on < ui2dMinVotes ? 0 : hi >= 4 * lo ? (op > on ? 1 : -1) : 2);
      ui2dObjPos = ui2dObjNeg = 0; }
    InterlockedExchange(&ui2dLastBbDraws, ui2dBackbufferPtr ? ui2dFrameBbDraws : -1); InterlockedExchange(&ui2dLastBbIndexed, ui2dFrameBbIndexed);
    ui2dLastBbSrv = ui2dFrameBbSrv; ui2dFrameBbDraws = ui2dFrameBbIndexed = 0; ui2dFrameBbSrv = nullptr;
    ui2dCur.draws = ui2dFrameDrawsAll; ui2dCur.pos = pos; ui2dCur.neg = neg; ui2dCur.sprites = ui2dFrameSprites;
    ui2dLast = ui2dCur; ui2dCur = Ui2dFrameDetail{};
    ui2dFrameDrawsAll = 0;
    // Frames that showed sprites without a camera of their own: held = placed
    // from the previous frame, bare = left where the game put them (the flash).
    if (!ui2dHaveX0 && ui2dFrameSprites) InterlockedIncrement(ui2dFrameHeld ? &ui2dFramesHeld : &ui2dFramesBare);
    // early = a frame WITH a camera whose sprites came before its votes: there the held value is the other eye's.
    if (ui2dHaveX0 && ui2dFrameEarly) InterlockedIncrement(&ui2dFramesEarly);
    ui2dFrameSprites = ui2dFrameHeld = ui2dFrameEarly = 0;
    ui2dVotesPos = ui2dVotesNeg = 0; ui2dHaveX0 = false;
}

// Draw path, before the draw is forwarded. kind 0 = DrawIndexed, 1 = Draw.
void ui2dOnDraw(ID3D11DeviceContext *c, unsigned kind) {
    ui2dFrameDrawsAll++;
    if (ui2dTraceLeft > 0 && c == context.Get() && !stopping) {
        MultiLock trLock;
        ID3D11RenderTargetView *rtv = nullptr; c->OMGetRenderTargets(1, &rtv, nullptr);
        if (rtv) {
            ID3D11Resource *res = nullptr; rtv->GetResource(&res);
            int t = res && res == ui2dTraceTarget[0] ? 0 : res && res == ui2dTraceTarget[1] ? 1 : res && res == ui2dBackbufferPtr ? 2 : -1;
            if (t >= 0) {
                int rel = 0; ID3D11ShaderResourceView *srv[2] = { nullptr, nullptr }; c->PSGetShaderResources(0, 2, srv);
                for (auto s : srv) if (s) {
                    ID3D11Resource *sr = nullptr; s->GetResource(&sr);
                    if (sr && (sr == ui2dTraceTarget[0] || sr == ui2dTraceTarget[1])) rel |= (t < 2 && sr == ui2dTraceTarget[t]) ? 2 : (t < 2 ? 1 : (sr == ui2dTraceTarget[0] ? 4 : 8));
                    if (sr) sr->Release(); s->Release();
                }
                ID3D11VertexShader *vs = nullptr; ID3D11ClassInstance *ci[1]; UINT nci = 0; c->VSGetShader(&vs, ci, &nci);
                UINT64 h = vsHashOf(vs, nullptr, nullptr); if (vs) vs->Release();
                D3D11_VIEWPORT vp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]; UINT nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE; c->RSGetViewports(&nvp, vp);
                unsigned i = 0;
                for (; i < ui2dTraceCount; i++) if (ui2dTraceGroups[i].target == t && ui2dTraceGroups[i].rel == rel && ui2dTraceGroups[i].vs == h) break;
                if (i == ui2dTraceCount) {
                    if (ui2dTraceCount < 48) { ui2dTraceGroups[i] = Ui2dTraceGroup{ t, rel, h, 0, ui2dFrameDrawsAll, 0, nvp ? (UINT)vp[0].Width : 0, nvp ? (UINT)vp[0].Height : 0 }; ui2dTraceCount++; }
                    else { ui2dTraceOverflow++; i = 48; }
                }
                if (i < 48) { ui2dTraceGroups[i].count++; ui2dTraceGroups[i].last = ui2dFrameDrawsAll; }
            }
            if (res) res->Release();
            rtv->Release();
        }
    }
    if (ui2dBackbufferPtr && c == context.Get() && !stopping && (kind == 1 || (ui2dFrameDrawsAll & 63) == 0)) {
        MultiLock bbLock;
        ID3D11RenderTargetView *rtv = nullptr; c->OMGetRenderTargets(1, &rtv, nullptr);
        if (rtv) {
            ID3D11Resource *res = nullptr; rtv->GetResource(&res);
            if (res == ui2dBackbufferPtr) {
                if (kind != 1) ui2dFrameBbIndexed++;
                else {
                    if (!ui2dFrameBbDraws) {
                        ID3D11ShaderResourceView *srv = nullptr; c->PSGetShaderResources(0, 1, &srv);
                        if (srv) { ID3D11Resource *sr = nullptr; srv->GetResource(&sr); ui2dFrameBbSrv = sr;
                                   if (sr && sr != ui2dFinalTex[0] && sr != ui2dFinalTex[1]) { ui2dFinalTex[1] = ui2dFinalTex[0]; ui2dFinalTex[0] = sr; }   // the two most recent blit sources
                                   if (sr) sr->Release(); srv->Release(); }
                    }
                    ui2dFrameBbDraws++;
                }
            }
            if (res) res->Release();
            rtv->Release();
        }
    }
    const float *f = (const float *)ui2dBank; size_t n = ui2dBankLen;
    if (!f || n < 0x240 || n > (1u << 20) || !ui2dSignature(f)) return;
    LONG on = ui2dCfg.on;
    if (!on && !InterlockedCompareExchange(&ui2dGpuPatched, 0, 0)) return;        // idle and nothing to undo
    if (c != context.Get() || stopping) return;
    InterlockedIncrement(&ui2dSigDraws);
    bool want = on != 0;
    if (want && kind != 1) { want = false; InterlockedIncrement(&ui2dSkipKind); }
    // No camera of its own yet this frame: fall back on the previous voted
    // frame (vr_ui2d_hold), decided after the other filters so that only real
    // sprites are counted.
    bool fresh = ui2dHaveX0 && GetTickCount() - ui2dX0Tick <= 500;
    float useX0 = ui2dX0; bool held = false;
    if (want && !fresh) {
        LONG hold = ui2dCfg.hold;
        if (hold && ui2dHavePrev && GetTickCount() - ui2dPrevTick <= ui2dHoldMs) { useX0 = hold == 2 ? -ui2dPrevX0 : ui2dPrevX0; held = true; }
    }
    // The handoff eye: checked against the votes where both exist, used where the votes fall short.
    bool byEye = false;
    if (want) {
        LONG eye = ui2dEyeFresh(); int es = ui2dEyeSign(eye);
        if (fresh) {
            if (!ui2dFrameEyeAtDraw) ui2dFrameEyeAtDraw = eye;
            if (es) InterlockedIncrement((es > 0) == (ui2dX0 > 0) ? &ui2dEyeAgree : &ui2dEyeDisagree);
        } else if (!held && es && ui2dFrameScene >= ui2dSceneMin) {
            LONG magE6 = InterlockedCompareExchange(&ui2dDisplayMagE6, 0, 0);
            float mag = magE6 ? (float)magE6 / 1000000.0f : ui2dHavePrev ? (float)fabs(ui2dPrevX0) : 0.0f;
            if (mag > 0) { useX0 = es > 0 ? mag : -mag; held = byEye = true; }
        }
    }
    MultiLock lock;
    if (want) {
        D3D11_VIEWPORT vp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]; UINT nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
        c->RSGetViewports(&nvp, vp);
        if (!nvp || !ui2dBbW || vp[0].TopLeftX != 0 || vp[0].TopLeftY != 0 || (UINT)vp[0].Width != ui2dBbW || (UINT)vp[0].Height != ui2dBbH) { want = false; InterlockedIncrement(&ui2dSkipViewport); }
    }
    if (want) {
        ID3D11DepthStencilState *dss = nullptr; UINT ref = 0; c->OMGetDepthStencilState(&dss, &ref);
        D3D11_DEPTH_STENCIL_DESC dd{}; dd.DepthEnable = TRUE; if (dss) { dss->GetDesc(&dd); dss->Release(); }
        if (!dd.DepthEnable) { want = false; InterlockedIncrement(&ui2dSkipDepth); }     // the final full-screen blit
    }
    if (want) {
        ID3D11VertexShader *vs = nullptr; ID3D11ClassInstance *ci[1]; UINT nci = 0; c->VSGetShader(&vs, ci, &nci);
        UINT64 h = vsHashOf(vs); if (vs) vs->Release();
        bool listed = false;
        AcquireSRWLockShared(&ui2dCfgLock);
        bool any = false;
        for (unsigned i = 0; i < ui2dCfg.vsCount; i++) { if (ui2dCfg.vs[i] == h) listed = true; if (ui2dCfg.vs[i] == ~0ull) any = true; }
        ReleaseSRWLockShared(&ui2dCfgLock);
        ui2dCandidate(h ? h : 1);
        // vr_ui2d_vs=any: diagnostic fallback when no shader identity is available -
        // every draw that passes the kind/viewport/depth filters counts as a sprite.
        if (!any && (!h || !listed)) { want = false; InterlockedIncrement(&ui2dSkipVs); }
    }
    if (want) {
        ui2dFrameSprites++; if (!fresh) ui2dFrameEarly++;
        if (!fresh && !held) { want = false; InterlockedIncrement(&ui2dSkipStale); }
        else if (held) { ui2dFrameHeld++; InterlockedIncrement(&ui2dHeldDraws); if (byEye) InterlockedIncrement(&ui2dEyeUsed); }
    }
    LONG patched = InterlockedCompareExchange(&ui2dGpuPatched, 0, 0);
    if (!want && !patched) return;                                              // pristine wanted, pristine there
    if (want && patched && ui2dGpuX0 == useX0) return;                          // already patched for this eye
    ID3D11Buffer *b = nullptr; c->VSGetConstantBuffers(0, 1, &b);
    if (!b) { InterlockedIncrement(&ui2dErrors); return; }
    D3D11_BUFFER_DESC desc{}; b->GetDesc(&desc);
    D3D11_MAPPED_SUBRESOURCE m{};
    if (desc.Usage != D3D11_USAGE_DYNAMIC || !(desc.CPUAccessFlags & D3D11_CPU_ACCESS_WRITE) || desc.ByteWidth < n ||
        FAILED(c->Map(b, 0, D3D11_MAP_WRITE_DISCARD, 0, &m))) { b->Release(); InterlockedIncrement(&ui2dErrors); return; }
    memcpy(m.pData, f, n);
    if (want) {
        ui2dPatch((float *)m.pData, useX0, (float)ui2dCfg.scaleMils / 1000.0f, (float)ui2dCfg.convE5 / 100000.0f, (int)ui2dCfg.sign);
        ui2dGpuX0 = useX0; InterlockedIncrement(&ui2dPatched);
    } else InterlockedIncrement(&ui2dRestored);
    c->Unmap(b, 0); b->Release();
    InterlockedExchange(&ui2dGpuPatched, want ? 1 : 0);
}

// ---------------------------------------------------------------- install / remove
void ui2dInstall(ID3D11Device *d) {
    auto table = *(void ***)d;
    vsHook = { table + 12, table[12], (void *)createVs };
    // Observation only, so a slot somebody else already redirected (MGSHDFix does,
    // measured 2026-09-18) is chained rather than refused: our entry records the
    // bytecode the GAME passed in and calls whatever was there. The owner must be
    // a loaded module; anything else is refused. Removal restores exactly the
    // pointer found here, and only while the slot still holds ours.
    HMODULE owner = nullptr; char ownerName[MAX_PATH] = "?";
    bool native = runtimePointer(vsHook.next);
    bool known = native || (GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)vsHook.next, &owner) && owner);
    if (known && !native) { GetModuleFileNameA(owner, ownerName, MAX_PATH); ownerName[MAX_PATH - 1] = 0; }
    if (known && writeSlot(vsHook, vsHook.next, vsHook.ours)) {
        if (native) log("ui2d: vertex-shader registry attached");
        else if (logger) logger("ui2d: vertex-shader registry attached, chained behind %s\r\n", ownerName);
    } else { vsHook = {}; log("ui2d: vertex-shader registry REFUSED (slot owner is not a loaded module, or the write failed)"); }
    // Copy observers on the immediate context's vtable (46 CopySubresourceRegion, 47 CopyResource, 57 ResolveSubresource).
    ui2dEnsureCopyHooks();
#ifndef DG_DRAW_TRIAL_TEST
    HMODULE exe = GetModuleHandleW(nullptr);
    auto dos = (IMAGE_DOS_HEADER *)exe; auto nt = (IMAGE_NT_HEADERS64 *)((BYTE *)exe + dos->e_lfanew);
    auto sec = IMAGE_FIRST_SECTION(nt);
    for (unsigned i = 0; i < nt->FileHeader.NumberOfSections && !ui2dSite; i++, sec++) {
        if (!(sec->Characteristics & IMAGE_SCN_MEM_EXECUTE)) continue;
        BYTE *begin = (BYTE *)exe + sec->VirtualAddress; size_t size = sec->Misc.VirtualSize, off = 0;
        if (ui2dFindSite(begin, size, &off)) ui2dSite = begin + off;
    }
    if (!ui2dSite) { log("ui2d: upload site NOT found - sprite placement unavailable"); return; }
    memcpy(&ui2dOldRel, ui2dSite + 1, 4);
    BYTE *target = ui2dSite + 5 + ui2dOldRel;
    HMODULE targetOwner = nullptr;
    if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)target, &targetOwner) || targetOwner != exe) { log("ui2d: upload site call target is foreign - refused"); ui2dSite = nullptr; return; }
    ui2dStub = nearMemory(ui2dSite);
    if (!ui2dStub) { log("ui2d: no near memory for the upload stub"); ui2dSite = nullptr; return; }
    BYTE code[12] = { 0x48, 0xb8, 0,0,0,0,0,0,0,0, 0xff, 0xe0 };   // mov rax, imm64 ; jmp rax
    UINT64 fn = (UINT64)(ULONG_PTR)&ui2dUpload; memcpy(code + 2, &fn, 8); memcpy(ui2dStub, code, sizeof code);
    DWORD old, ignored;
    if (!VirtualProtect(ui2dStub, 4096, PAGE_EXECUTE_READ, &old)) { ui2dSite = nullptr; return; }
    FlushInstructionCache(GetCurrentProcess(), ui2dStub, sizeof code);
    intptr_t rel = (BYTE *)ui2dStub - (ui2dSite + 5);
    if (rel < INT32_MIN || rel > INT32_MAX) { ui2dSite = nullptr; return; }
    ui2dNewRel = (INT32)rel;
    if (!VirtualProtect(ui2dSite + 1, 4, PAGE_EXECUTE_READWRITE, &old)) { ui2dSite = nullptr; return; }
    memcpy(ui2dSite + 1, &ui2dNewRel, 4);
    VirtualProtect(ui2dSite + 1, 4, old, &ignored); FlushInstructionCache(GetCurrentProcess(), ui2dSite, 5);
    ui2dSiteInstalled = true;
    if (logger) logger("ui2d: upload site rva 0x%llX retargeted (one call, CPU bank untouched); default off\r\n",
                       (unsigned long long)(ui2dSite - (BYTE *)exe));
#endif
}
void ui2dRemove() {
    if (ui2dSiteInstalled) {
        INT32 now; memcpy(&now, ui2dSite + 1, 4);
        DWORD old, ignored;
        if (now == ui2dNewRel && VirtualProtect(ui2dSite + 1, 4, PAGE_EXECUTE_READWRITE, &old)) {
            memcpy(ui2dSite + 1, &ui2dOldRel, 4); VirtualProtect(ui2dSite + 1, 4, old, &ignored); FlushInstructionCache(GetCurrentProcess(), ui2dSite, 5);
        }
        ui2dSiteInstalled = false;      // the stub stays: a call may be in flight
    }
    if (vsHook.slot && *vsHook.slot == vsHook.ours) writeSlot(vsHook, vsHook.ours, vsHook.next);
    for (unsigned i = 0; i < ui2dCopySetCount; i++) for (Hook *h : { &ui2dCopySets[i].reg, &ui2dCopySets[i].res, &ui2dCopySets[i].rsv }) if (h->slot && *h->slot == h->ours) writeSlot(*h, h->ours, h->next);
}
