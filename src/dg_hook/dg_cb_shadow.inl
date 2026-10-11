// CPU shadow of the game's constant-buffer uploads (second eye, step 2).
// The game uploads its register bank in 0x45AA0: Map(WRITE_DISCARD), memcpy,
// Unmap; the memcpy is already retargeted to ui2dUpload (dg_ui2d.inl), which
// now also hands (dst, src, n) here. Which buffer dst belongs to is learned
// from the Map that returned it: an observer on the immediate context's Map
// slot (14) remembers, on the Present/draw thread only, the last buffer mapped
// for writing and its pointer, and invalidates the shadow of any tracked buffer
// that is mapped again (its contents are about to change - also when ui2d
// re-uploads a patched bank). A shadow therefore only exists for bytes the
// game itself wrote since that buffer's last Map. The observer never changes
// arguments or results. Kept per vtable like the ui2d copy observers: D3D11
// swaps the context's vtable when multithread protection is switched.
struct CbShadowEntry { ID3D11Buffer *buffer; UINT width, start, end; bool valid; uint8_t bytes[65536]; };
constexpr unsigned cbShadowCap = 16;
CbShadowEntry cbShadow[cbShadowCap]; unsigned cbShadowNext = 0;
DgBankShadow cbShadowView;
ID3D11Buffer *cbLastMapBuffer = nullptr; uint8_t *cbLastMapData = nullptr; UINT cbLastMapWidth = 0;
volatile LONG cbShadowUploads = 0, cbShadowUntracked = 0, cbShadowInvalidated = 0;
using MapFn = HRESULT (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11Resource *, UINT, D3D11_MAP, UINT, D3D11_MAPPED_SUBRESOURCE *);
// Batched replay: before the game writes something a pending batch still reads,
// the batch is replayed (dg_draw_batch.h). The observers below only look and
// call batchHazard; batchHazard (after SecondEyeLive) decides.
enum { FL_CAP, FL_WRAP, FL_FALLBACK, FL_SAMPLE, FL_MAP, FL_TARGETS, FL_WRITE, FL_CLEAR, FL_DISPATCH, FL_PRESENT, FL_FRAME, FL_KINDS };
static const char *const flushNames[FL_KINDS] = { "cap", "wrap", "fallback", "sample", "map", "targets", "write", "clear", "dispatch", "present", "frame" };
void batchHazard(ID3D11DeviceContext *c, int kind, ID3D11Resource *r);           // r null: always flush
void batchTargets(ID3D11DeviceContext *c, UINT n, ID3D11RenderTargetView *const *rt, ID3D11DepthStencilView *ds, bool uavs);
void batchMirrorClearRt(ID3D11DeviceContext *c, ID3D11RenderTargetView *v, const FLOAT col[4]);   /* stage A: the post target's clears for B3/B4 */
void batchMirrorClearDs(ID3D11DeviceContext *c, ID3D11DepthStencilView *v, UINT f, FLOAT z, UINT8 s);
enum { HZ_MAP, HZ_OMSET, HZ_OMSETUAV, HZ_DISPATCH, HZ_DISPATCHIND, HZ_UPDATE, HZ_CLEARRT, HZ_CLEARUAVU, HZ_CLEARUAVF, HZ_CLEARDS, HZ_MIPS, HZ_DIIN, HZ_DIN, HZ_COUNT };
static const unsigned hzSlots[HZ_COUNT] = { 14, 33, 34, 41, 42, 48, 50, 51, 52, 53, 54, 20, 21 };
void censusInstanced(ID3D11DeviceContext *c, UINT verts, bool indexed);   /* census only: instanced draws are never replayed */
struct CbMapSet { void **vt; Hook h[HZ_COUNT]; };
CbMapSet cbMapSets[4]; unsigned cbMapSetCount = 0;
volatile LONG cbRearms = 0; void *cbSeen[HZ_COUNT];
void *hzNext(ID3D11DeviceContext *c, int k) {
    void **vt = *(void ***)c;
    for (unsigned i = 0; i < cbMapSetCount; i++) if (cbMapSets[i].vt == vt) return cbMapSets[i].h[k].next;
    return cbMapSets[0].h[k].next;
}
CbShadowEntry *cbShadowFind(const void *b) { for (auto &e : cbShadow) if (e.buffer == b) return &e; return nullptr; }
HRESULT STDMETHODCALLTYPE cbMapObserved(ID3D11DeviceContext *c, ID3D11Resource *r, UINT sub, D3D11_MAP type, UINT flags, D3D11_MAPPED_SUBRESOURCE *m) {
    MapFn next = (MapFn)hzNext(c, HZ_MAP);

    if (type != D3D11_MAP_READ) batchHazard(c, FL_MAP, r);          // before the contents change
    HRESULT hr = next(c, r, sub, type, flags, m);
    if (ownerThread && GetCurrentThreadId() == ownerThread) {
        if (CbShadowEntry *e = cbShadowFind(r)) { if (e->valid) InterlockedIncrement(&cbShadowInvalidated); e->valid = false; }
        cbLastMapBuffer = nullptr; cbLastMapData = nullptr; cbLastMapWidth = 0;
        if (SUCCEEDED(hr) && m && m->pData && sub == 0 && type != D3D11_MAP_READ) {
            D3D11_RESOURCE_DIMENSION dim = D3D11_RESOURCE_DIMENSION_UNKNOWN; r->GetType(&dim);
            if (dim == D3D11_RESOURCE_DIMENSION_BUFFER) {
                D3D11_BUFFER_DESC bd{}; static_cast<ID3D11Buffer *>(r)->GetDesc(&bd);
                if ((bd.BindFlags & D3D11_BIND_CONSTANT_BUFFER) && bd.ByteWidth <= sizeof cbShadow[0].bytes) {
                    cbLastMapBuffer = static_cast<ID3D11Buffer *>(r); cbLastMapData = (uint8_t *)m->pData; cbLastMapWidth = bd.ByteWidth;
                }
            }
        }
    }
    return hr;
}
// From ui2dUpload, after the game's memcpy into the mapped buffer.
void cbShadowUpload(void *dst, const void *src, size_t n) {
    if (!ownerThread || GetCurrentThreadId() != ownerThread || !cbLastMapBuffer ||
        (uint8_t *)dst < cbLastMapData || (size_t)((uint8_t *)dst - cbLastMapData) + n > cbLastMapWidth) {
        InterlockedIncrement(&cbShadowUntracked); return;
    }
    UINT off = (UINT)((uint8_t *)dst - cbLastMapData);
    CbShadowEntry *e = cbShadowFind(cbLastMapBuffer);
    if (!e) { e = &cbShadow[cbShadowNext++ % cbShadowCap]; e->buffer = cbLastMapBuffer; e->valid = false; }
    memcpy(e->bytes + off, src, n);
    if (e->valid && off <= e->end && off + n >= e->start) {          // same Map, contiguous: extend
        if (off < e->start) e->start = off;
        if (off + n > e->end) e->end = off + (UINT)n;
    } else { e->start = off; e->end = off + (UINT)n; }
    e->width = cbLastMapWidth; e->valid = true;
    InterlockedIncrement(&cbShadowUploads);
}
const DgBankShadow *cbShadowLookup(ID3D11Buffer *b, void *) {
    CbShadowEntry *e = cbShadowFind(b);
    if (!e || !e->valid) return nullptr;
    cbShadowView = { e->bytes, e->width, e->start, e->end };
    return &cbShadowView;
}
using OmSetFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, UINT, ID3D11RenderTargetView *const *, ID3D11DepthStencilView *);
using OmSetUavFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, UINT, ID3D11RenderTargetView *const *, ID3D11DepthStencilView *, UINT, UINT, ID3D11UnorderedAccessView *const *, const UINT *);
using DispatchFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, UINT, UINT, UINT);
using DispatchIndFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11Buffer *, UINT);
using UpdateFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11Resource *, UINT, const D3D11_BOX *, const void *, UINT, UINT);
using ClearRtFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11RenderTargetView *, const FLOAT[4]);
using ClearUavUFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11UnorderedAccessView *, const UINT[4]);
using ClearUavFFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11UnorderedAccessView *, const FLOAT[4]);
using ClearDsFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11DepthStencilView *, UINT, FLOAT, UINT8);
using MipsFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, ID3D11ShaderResourceView *);
void STDMETHODCALLTYPE hzOmSet(ID3D11DeviceContext *c, UINT n, ID3D11RenderTargetView *const *rt, ID3D11DepthStencilView *ds) {
     batchTargets(c, n, rt, ds, false); ((OmSetFn)hzNext(c, HZ_OMSET))(c, n, rt, ds); }
void STDMETHODCALLTYPE hzOmSetUav(ID3D11DeviceContext *c, UINT n, ID3D11RenderTargetView *const *rt, ID3D11DepthStencilView *ds, UINT us, UINT un, ID3D11UnorderedAccessView *const *u, const UINT *uc) {

    bool uavs = false; if (un != D3D11_KEEP_UNORDERED_ACCESS_VIEWS && u) for (UINT i = 0; i < un; i++) if (u[i]) uavs = true;
    if (n != D3D11_KEEP_RENDER_TARGETS_AND_DEPTH_STENCIL || uavs) batchTargets(c, n == D3D11_KEEP_RENDER_TARGETS_AND_DEPTH_STENCIL ? ~0u : n, rt, ds, uavs);
    ((OmSetUavFn)hzNext(c, HZ_OMSETUAV))(c, n, rt, ds, us, un, u, uc); }
void STDMETHODCALLTYPE hzDispatch(ID3D11DeviceContext *c, UINT x, UINT y, UINT z) { batchHazard(c, FL_DISPATCH, nullptr); ((DispatchFn)hzNext(c, HZ_DISPATCH))(c, x, y, z); }
void STDMETHODCALLTYPE hzDispatchInd(ID3D11DeviceContext *c, ID3D11Buffer *b, UINT o) { batchHazard(c, FL_DISPATCH, nullptr); ((DispatchIndFn)hzNext(c, HZ_DISPATCHIND))(c, b, o); }
void STDMETHODCALLTYPE hzUpdate(ID3D11DeviceContext *c, ID3D11Resource *r, UINT s, const D3D11_BOX *b, const void *p, UINT rp, UINT dp) {
    batchHazard(c, FL_WRITE, r); ((UpdateFn)hzNext(c, HZ_UPDATE))(c, r, s, b, p, rp, dp); }
void STDMETHODCALLTYPE hzClearRt(ID3D11DeviceContext *c, ID3D11RenderTargetView *v, const FLOAT col[4]) { batchHazard(c, FL_CLEAR, nullptr); batchMirrorClearRt(c, v, col); ((ClearRtFn)hzNext(c, HZ_CLEARRT))(c, v, col); }
void STDMETHODCALLTYPE hzClearUavU(ID3D11DeviceContext *c, ID3D11UnorderedAccessView *v, const UINT x[4]) { batchHazard(c, FL_CLEAR, nullptr); ((ClearUavUFn)hzNext(c, HZ_CLEARUAVU))(c, v, x); }
void STDMETHODCALLTYPE hzClearUavF(ID3D11DeviceContext *c, ID3D11UnorderedAccessView *v, const FLOAT x[4]) { batchHazard(c, FL_CLEAR, nullptr); ((ClearUavFFn)hzNext(c, HZ_CLEARUAVF))(c, v, x); }
void STDMETHODCALLTYPE hzClearDs(ID3D11DeviceContext *c, ID3D11DepthStencilView *v, UINT f, FLOAT z, UINT8 s) { batchHazard(c, FL_CLEAR, nullptr); batchMirrorClearDs(c, v, f, z, s); ((ClearDsFn)hzNext(c, HZ_CLEARDS))(c, v, f, z, s); }
void STDMETHODCALLTYPE hzMips(ID3D11DeviceContext *c, ID3D11ShaderResourceView *v) { batchHazard(c, FL_WRITE, nullptr); ((MipsFn)hzNext(c, HZ_MIPS))(c, v); }
using DiinFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, UINT, UINT, UINT, INT, UINT);
using DinFn = void (STDMETHODCALLTYPE *)(ID3D11DeviceContext *, UINT, UINT, UINT, UINT);
void STDMETHODCALLTYPE hzDiin(ID3D11DeviceContext *c, UINT n, UINT inst, UINT si, INT bv, UINT sinst) { ((DiinFn)hzNext(c, HZ_DIIN))(c, n, inst, si, bv, sinst);  }
void STDMETHODCALLTYPE hzDin(ID3D11DeviceContext *c, UINT n, UINT inst, UINT sv, UINT sinst) { ((DinFn)hzNext(c, HZ_DIN))(c, n, inst, sv, sinst);  }
static void *const hzOurs[HZ_COUNT] = { (void *)cbMapObserved, (void *)hzOmSet, (void *)hzOmSetUav, (void *)hzDispatch, (void *)hzDispatchInd, (void *)hzUpdate,
                                        (void *)hzClearRt, (void *)hzClearUavU, (void *)hzClearUavF, (void *)hzClearDs, (void *)hzMips, (void *)hzDiin, (void *)hzDin };
/* All observers in the current vtable (cheap: 11 pointer compares). Live 27 Sep:
   d3d11 itself rewrites slots 41-54 at least once a frame. */
bool cbShadowArmed() {
    if (!context.Get()) return false;
    void **ct = *(void ***)context.Get();
    for (int k = 0; k < HZ_COUNT; k++) if (ct[hzSlots[k]] != hzOurs[k]) return false;
    return true;
}
// Draw thread, once per frame: the context's CURRENT vtable carries the observers.
void cbShadowEnsure() {
    if (!context.Get() || stopping) return;
    void **ct = *(void ***)context.Get();
    bool all = true; for (int k = 0; k < HZ_COUNT; k++) { cbSeen[k] = ct[hzSlots[k]]; if (cbSeen[k] != hzOurs[k]) all = false; }
    if (all) return;
    CbMapSet *set = nullptr;
    for (unsigned i = 0; i < cbMapSetCount; i++) if (cbMapSets[i].vt == ct) set = &cbMapSets[i];
    bool fresh = !set;
    if (fresh) {
        if (cbMapSetCount >= 4) return;
        set = &cbMapSets[cbMapSetCount]; set->vt = ct;
        for (int k = 0; k < HZ_COUNT; k++) {
            /* A vtable the runtime copied from one we patched already holds our
               observer: its next is the one recorded for that earlier vtable -
               never the observer itself (that would recurse). */
            void *next = ct[hzSlots[k]];
            if (next == hzOurs[k]) { next = nullptr; for (unsigned i = 0; i < cbMapSetCount && !next; i++) if (cbMapSets[i].h[k].next != hzOurs[k]) next = cbMapSets[i].h[k].next; }
            set->h[k] = { ct + hzSlots[k], next, hzOurs[k] };
        }
        cbMapSetCount++;                                          // published before the slots change
    }
    unsigned ok = 0;
    for (int k = 0; k < HZ_COUNT; k++) {
        if (ct[hzSlots[k]] == hzOurs[k]) { if (set->h[k].next) ok++; continue; }
        set->h[k] = { ct + hzSlots[k], ct[hzSlots[k]], hzOurs[k] };  // the runtime may have rewritten the slot
        if (set->h[k].next && writeSlot(set->h[k], set->h[k].next, set->h[k].ours)) ok++; else set->h[k].slot = nullptr;
    }
    if (!fresh && InterlockedIncrement(&cbRearms) > 3) return;     /* counted in the heartbeat */
    if (logger) logger("second_eye: context observers (Map + batch hazards) %s %u of %u on vtable %p%s\r\n", fresh ? "attached" : "re-armed", ok, (unsigned)HZ_COUNT, (void *)ct, fresh ? "" : " - found in the slots:");
    if (!fresh && logger) for (int k = 0; k < HZ_COUNT; k++) {
        void *was = cbSeen[k]; HMODULE m = nullptr; char path[MAX_PATH] = "?"; const char *name = path;
        if (was && GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCSTR)was, &m) && m &&
            GetModuleFileNameA(m, path, sizeof path)) { const char *b = strrchr(path, '\\'); name = b ? b + 1 : path; }
        logger("  slot %u: %p %s+0x%llX%s\r\n", hzSlots[k], was, name, (unsigned long long)((uintptr_t)was - (uintptr_t)m), was == hzOurs[k] ? " (ours)" : "");
    }
}
void cbShadowRemove() {
    for (unsigned i = 0; i < cbMapSetCount; i++) for (Hook &h : cbMapSets[i].h) if (h.slot && *h.slot == h.ours) writeSlot(h, h.ours, h.next);
    for (auto &e : cbShadow) e.valid = false;
}
