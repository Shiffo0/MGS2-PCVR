#pragma once
/* Batched second-eye replay (27 Sep 2026).
 * Live and on the desk (rt_switch_bench): switching the DEPTH target around each
 * replayed draw costs ~11.5 us of GPU time per draw at 4K MSAA4 (colour only:
 * 0.5 us); ~1000 world draws a frame made the game a slide show. Replaying the
 * frame's world draws in batches - one target switch per batch - costs ~0.15 us.
 *
 * A record is the pipeline state of one world draw, taken right after the
 * game's own draw, with every constant buffer the CPU can vouch for copied into
 * the ring (cb0 of the vertex shader retargeted to the other eye, exactly as
 * the CPU route binds it). The batch is replayed into B later; until then the
 * game may change the contents of what the records still reference (vertex,
 * index and non-shadowed constant buffers, textures). DgBatch keeps that set,
 * and the caller must flush BEFORE any write to it (Map, UpdateSubresource,
 * copies, clears, compute, a draw into another target). The captured state is
 * bounded (slots below); `unboundedSlots` says whether a draw uses more, and
 * such a draw is not recorded. Hull, domain and geometry shaders must be null.
 * Flush restores every slot it touched to the game's current state. */
#include <d3d11_1.h>
#include <wrl/client.h>
#include <vector>
#include <string.h>
#include "dg_second_camera_cpu.h"
using Microsoft::WRL::ComPtr;

typedef void (STDMETHODCALLTYPE *DgIndexedFn)(ID3D11DeviceContext *, UINT, UINT, INT);
typedef void (STDMETHODCALLTYPE *DgDrawFn)(ID3D11DeviceContext *, UINT, UINT);
/* Stage A (effects in the second eye): a replacement for a game shader-resource view (the game's
   main colour/depth read by the post pass -> the second eye's own), or null to keep the game's. */
typedef ID3D11ShaderResourceView *(*DgSrvSwap)(ID3D11ShaderResourceView *game, void *ctx);

struct DgCb { ID3D11Buffer *b = nullptr; UINT first = 0, count = 0; };
struct DgDrawState {
    static constexpr UINT vsCb = 4, vsSrv = 8, vsSam = 4, psCb = 8, psSrv = 16, psSam = 16, vbs = 8, vps = 4, gsCb = 4, gsSrv = 8, gsSam = 4;
    /* raw pointers, each holding one reference (released by clear) */
    ID3D11VertexShader *vs = nullptr; DgCb vcb[vsCb]; ID3D11ShaderResourceView *vsrv[vsSrv]{}; ID3D11SamplerState *vsam[vsSam]{};
    ID3D11PixelShader *ps = nullptr; DgCb pcb[psCb]; ID3D11ShaderResourceView *psrv[psSrv]{}; ID3D11SamplerState *psam[psSam]{};
    /* geometry stage (29 Sep: part of the game's particles expand in a geometry shader) */
    ID3D11GeometryShader *gs = nullptr; DgCb gcb[gsCb]; ID3D11ShaderResourceView *gsrv[gsSrv]{}; ID3D11SamplerState *gsam[gsSam]{};
    ID3D11InputLayout *layout = nullptr; ID3D11Buffer *vb[vbs]{}; UINT stride[vbs]{}, offset[vbs]{};
    ID3D11Buffer *ib = nullptr; DXGI_FORMAT ibf = DXGI_FORMAT_UNKNOWN; UINT ibo = 0; D3D11_PRIMITIVE_TOPOLOGY topo = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED;
    ID3D11RasterizerState *rs = nullptr; D3D11_VIEWPORT vp[vps]{}; UINT nvp = 0; D3D11_RECT sc[vps]{}; UINT nsc = 0;
    ID3D11BlendState *blend = nullptr; float bf[4]{}; UINT mask = 0; ID3D11DepthStencilState *dss = nullptr; UINT ref = 0;

    void capture(ID3D11DeviceContext1 *c) {
        c->VSGetShader(&vs, nullptr, nullptr);
        { ID3D11Buffer *b[vsCb]{}; UINT f[vsCb]{}, n[vsCb]{}; c->VSGetConstantBuffers1(0, vsCb, b, f, n); for (UINT i = 0; i < vsCb; i++) vcb[i] = { b[i], f[i], n[i] }; }
        c->VSGetShaderResources(0, vsSrv, vsrv); c->VSGetSamplers(0, vsSam, vsam);
        c->GSGetShader(&gs, nullptr, nullptr);
        if (gs) { ID3D11Buffer *b[gsCb]{}; UINT f[gsCb]{}, n[gsCb]{}; c->GSGetConstantBuffers1(0, gsCb, b, f, n); for (UINT i = 0; i < gsCb; i++) gcb[i] = { b[i], f[i], n[i] };
                  c->GSGetShaderResources(0, gsSrv, gsrv); c->GSGetSamplers(0, gsSam, gsam); }
        c->PSGetShader(&ps, nullptr, nullptr);
        { ID3D11Buffer *b[psCb]{}; UINT f[psCb]{}, n[psCb]{}; c->PSGetConstantBuffers1(0, psCb, b, f, n); for (UINT i = 0; i < psCb; i++) pcb[i] = { b[i], f[i], n[i] }; }
        c->PSGetShaderResources(0, psSrv, psrv); c->PSGetSamplers(0, psSam, psam);
        c->IAGetInputLayout(&layout); c->IAGetVertexBuffers(0, vbs, vb, stride, offset);
        c->IAGetIndexBuffer(&ib, &ibf, &ibo); c->IAGetPrimitiveTopology(&topo);
        c->RSGetState(&rs); nvp = vps; c->RSGetViewports(&nvp, vp); nsc = vps; c->RSGetScissorRects(&nsc, sc);
        c->OMGetBlendState(&blend, bf, &mask); c->OMGetDepthStencilState(&dss, &ref);
    }
    template<class T> static void rel(T *&p) { if (p) { p->Release(); p = nullptr; } }
    void clear() {
        rel(vs); for (auto &x : vcb) { rel(x.b); x = {}; } for (auto &x : vsrv) rel(x); for (auto &x : vsam) rel(x);
        rel(ps); for (auto &x : pcb) { rel(x.b); x = {}; } for (auto &x : psrv) rel(x); for (auto &x : psam) rel(x);
        rel(gs); for (auto &x : gcb) { rel(x.b); x = {}; } for (auto &x : gsrv) rel(x); for (auto &x : gsam) rel(x);
        rel(layout); for (auto &x : vb) rel(x); rel(ib); rel(rs); rel(blend); rel(dss);
        nvp = nsc = 0;
    }
    /* Replace a constant-buffer binding (the reference moves to `b`). */
    static void rebind(DgCb &cb, ID3D11Buffer *b, UINT first, UINT count) { if (b) b->AddRef(); rel(cb.b); cb = { b, first, count }; }
    /* Sets this state; only what differs from `prev` when given. */
    void apply(ID3D11DeviceContext1 *c, const DgDrawState *p) const {
        auto cbs = [](const DgCb *a, const DgCb *b, UINT n) { for (UINT i = 0; i < n; i++) if (a[i].b != b[i].b || a[i].first != b[i].first || a[i].count != b[i].count) return true; return false; };
        if (!p || p->vs != vs) c->VSSetShader(vs, nullptr, 0);
        if (!p || cbs(p->vcb, vcb, vsCb)) { ID3D11Buffer *b[vsCb]; UINT f[vsCb], n[vsCb]; for (UINT i = 0; i < vsCb; i++) { b[i] = vcb[i].b; f[i] = vcb[i].first; n[i] = vcb[i].count; } c->VSSetConstantBuffers1(0, vsCb, b, f, n); }
        if (!p || memcmp(p->vsrv, vsrv, sizeof vsrv)) c->VSSetShaderResources(0, vsSrv, vsrv);
        if (!p || memcmp(p->vsam, vsam, sizeof vsam)) c->VSSetSamplers(0, vsSam, vsam);
        /* geometry stage: set whenever this record or the previous one uses it (a null GS clears it) */
        if (!p ? true : (p->gs != gs || (gs && (cbs(p->gcb, gcb, gsCb) || memcmp(p->gsrv, gsrv, sizeof gsrv) || memcmp(p->gsam, gsam, sizeof gsam))))) {
            c->GSSetShader(gs, nullptr, 0);
            if (gs) { ID3D11Buffer *b[gsCb]; UINT f[gsCb], n[gsCb]; for (UINT i = 0; i < gsCb; i++) { b[i] = gcb[i].b; f[i] = gcb[i].first; n[i] = gcb[i].count; }
                      c->GSSetConstantBuffers1(0, gsCb, b, f, n); c->GSSetShaderResources(0, gsSrv, gsrv); c->GSSetSamplers(0, gsSam, gsam); }
        }
        if (!p || p->ps != ps) c->PSSetShader(ps, nullptr, 0);
        if (!p || cbs(p->pcb, pcb, psCb)) { ID3D11Buffer *b[psCb]; UINT f[psCb], n[psCb]; for (UINT i = 0; i < psCb; i++) { b[i] = pcb[i].b; f[i] = pcb[i].first; n[i] = pcb[i].count; } c->PSSetConstantBuffers1(0, psCb, b, f, n); }
        if (!p || memcmp(p->psrv, psrv, sizeof psrv)) c->PSSetShaderResources(0, psSrv, psrv);
        if (!p || memcmp(p->psam, psam, sizeof psam)) c->PSSetSamplers(0, psSam, psam);
        if (!p || p->layout != layout) c->IASetInputLayout(layout);
        if (!p || memcmp(p->vb, vb, sizeof vb) || memcmp(p->stride, stride, sizeof stride) || memcmp(p->offset, offset, sizeof offset)) c->IASetVertexBuffers(0, vbs, vb, stride, offset);
        if (!p || p->ib != ib || p->ibf != ibf || p->ibo != ibo) c->IASetIndexBuffer(ib, ibf, ibo);
        if (!p || p->topo != topo) c->IASetPrimitiveTopology(topo);
        if (!p || p->rs != rs) c->RSSetState(rs);
        if (!p || p->nvp != nvp || memcmp(p->vp, vp, sizeof vp)) c->RSSetViewports(nvp, vp);
        if (!p || p->nsc != nsc || memcmp(p->sc, sc, sizeof sc)) c->RSSetScissorRects(nsc, sc);
        if (!p || p->blend != blend || memcmp(p->bf, bf, sizeof bf) || p->mask != mask) c->OMSetBlendState(blend, bf, mask);
        if (!p || p->dss != dss || p->ref != ref) c->OMSetDepthStencilState(dss, ref);
    }
};

/* 1 when the bound pipeline uses a slot or stage DgDrawState does not carry. */
static const char *dgUnboundedSlots(ID3D11DeviceContext *c) {
    { ID3D11HullShader *h = nullptr; c->HSGetShader(&h, nullptr, nullptr); if (h) { h->Release(); return "hull_shader"; } }
    { ID3D11DomainShader *s = nullptr; c->DSGetShader(&s, nullptr, nullptr); if (s) { s->Release(); return "domain_shader"; } }
    return nullptr;
}
/* The slow part of the check (sampled by the caller): nothing bound beyond the carried slots. */
static const char *dgUnboundedSlotsFull(ID3D11DeviceContext *c) {
    ID3D11Buffer *b[14]{}; ID3D11ShaderResourceView *s[128]{}; ID3D11SamplerState *m[16]{};
    auto any = [](IUnknown **p, UINT n) { bool r = false; for (UINT i = 0; i < n; i++) if (p[i]) { r = true; p[i]->Release(); p[i] = nullptr; } return r; };
    c->VSGetConstantBuffers(DgDrawState::vsCb, 14 - DgDrawState::vsCb, b); if (any((IUnknown **)b, 14 - DgDrawState::vsCb)) return "vs_cb_slot";
    c->PSGetConstantBuffers(DgDrawState::psCb, 14 - DgDrawState::psCb, b); if (any((IUnknown **)b, 14 - DgDrawState::psCb)) return "ps_cb_slot";
    c->VSGetShaderResources(DgDrawState::vsSrv, 128 - DgDrawState::vsSrv, s); if (any((IUnknown **)s, 128 - DgDrawState::vsSrv)) return "vs_srv_slot";
    c->PSGetShaderResources(DgDrawState::psSrv, 128 - DgDrawState::psSrv, s); if (any((IUnknown **)s, 128 - DgDrawState::psSrv)) return "ps_srv_slot";
    c->VSGetSamplers(DgDrawState::vsSam, 16 - DgDrawState::vsSam, m); if (any((IUnknown **)m, 16 - DgDrawState::vsSam)) return "vs_sampler_slot";
    { ID3D11Buffer *v[32]{}; UINT st[32], of[32]; c->IAGetVertexBuffers(DgDrawState::vbs, 32 - DgDrawState::vbs, v, st, of); if (any((IUnknown **)v, 32 - DgDrawState::vbs)) return "vertex_buffer_slot"; }
    return nullptr;
}

struct DgDrawRec {
    DgDrawState st; DgIndexedFn draw = nullptr; DgDrawFn plain = nullptr; UINT n = 0, first = 0; INT base = 0;
    /* own target (the post pass: B3/B4) - one reference each; none = the batch's target */
    bool ownTarget = false; ID3D11RenderTargetView *rt = nullptr; ID3D11DepthStencilView *ds = nullptr;
};


struct DgRecordArgs {
    int pers = -1; DgIndexedFn indexed = nullptr; DgDrawFn plain = nullptr; UINT n = 0, first = 0; INT base = 0;
    bool ownTarget = false; ID3D11RenderTargetView *rt = nullptr; ID3D11DepthStencilView *ds = nullptr;
    DgSrvSwap swap = nullptr; void *swapCtx = nullptr;
};

struct DgBatch {
    static constexpr UINT cap = 512;                   /* records before a forced flush */
    std::vector<DgDrawRec> recs = std::vector<DgDrawRec>(cap);
    UINT used = 0;
    /* resources the pending records read: open addressing, cleared per flush */
    static constexpr UINT setSize = 4096;
    std::vector<void *> set = std::vector<void *>(setSize);
    std::vector<UINT> setUsed;
    /* SRV -> resource, cached (views live long) */
    struct ViewRes { void *view, *res; }; ViewRes views[256]{}; UINT viewNext = 0;

    bool pending() const { return used > 0; }
    bool references(const void *r) const {
        if (!used || !r) return false;
        for (UINT h = (UINT)(((uintptr_t)r >> 4) * 2654435761u) % setSize, k = 0; k < setSize; k++, h = (h + 1) % setSize) {
            if (set[h] == r) return true;
            if (!set[h]) return false;
        }
        return false;
    }
    void addRef(void *r) {
        if (!r) return;
        for (UINT h = (UINT)(((uintptr_t)r >> 4) * 2654435761u) % setSize, k = 0; k < setSize; k++, h = (h + 1) % setSize) {
            if (set[h] == r) return;
            if (!set[h]) { set[h] = r; setUsed.push_back(h); return; }
        }
    }
    void *resourceOf(ID3D11ShaderResourceView *v) {
        for (auto &x : views) if (x.view == v) return x.res;
        ID3D11Resource *r = nullptr; v->GetResource(&r); if (r) r->Release();
        views[viewNext++ % 256] = { v, r };
        return r;
    }
    bool full() const { return used >= cap; }
    /* Records the draw the game just made. `cpu` stages the constant buffers;
       returns nullptr, or why nothing was recorded ("ring_wrap": flush, then retry). */
    const char *record(ID3D11Device *d, ID3D11DeviceContext1 *c, SecondCameraCpu &cpu, const float delta[16], const float sinv[16],
                       UINT pers, DgBankLookup lookup, void *ctx, DgIndexedFn draw, UINT n, UINT first, INT base, int *projective) {
        DgRecordArgs a; a.pers = (int)pers; a.indexed = draw; a.n = n; a.first = first; a.base = base;
        return recordEx(d, c, cpu, delta, sinv, lookup, ctx, a, projective);
    }
    const char *recordEx(ID3D11Device *d, ID3D11DeviceContext1 *c, SecondCameraCpu &cpu, const float delta[16], const float sinv[16],
                         DgBankLookup lookup, void *ctx, const DgRecordArgs &a, int *projective) {
        if (full()) return "batch_full";
        if (!a.indexed && !a.plain) return "no_draw_entry";
        if (const char *why = dgUnboundedSlots(c)) return why;
        DgDrawRec &r = recs[used];
        r.st.clear(); r.st.capture(c);

        UINT f = 0, cnt = 0;
        const char *why = nullptr;
        bool canWrap = !pending();                       /* only the record's first ring copy may rewind */
        if (a.pers >= 16) {
            /* the ring may rewind (DISCARD) only while nothing is pending, and only
               for this record's first copy - its later copies must not erase it */
            why = cpu.stageBound(d, c, r.st.vcb[0].b, r.st.vcb[0].first, r.st.vcb[0].count, a.pers, delta, sinv, lookup, ctx, canWrap, &f, &cnt, projective);
            if (why) { r.st.clear(); return why; }
            DgDrawState::rebind(r.st.vcb[0], cpu.ring.Get(), f, cnt);
            canWrap = false;
        } else if (projective) *projective = 0;
        /* every other constant buffer: a ring copy when shadowed, else referenced */
        auto other = [&](DgCb &cb) -> const char * {
            if (!cb.b) return nullptr;
            UINT f2 = 0, n2 = 0;
            const char *w = cpu.stageBound(d, c, cb.b, cb.first, cb.count, -1, delta, sinv, lookup, ctx, canWrap, &f2, &n2, nullptr);
            if (!w) { DgDrawState::rebind(cb, cpu.ring.Get(), f2, n2); canWrap = false; return nullptr; }
            if (!strcmp(w, "ring_wrap")) return w;
            addRef(cb.b); return nullptr;
        };
        for (UINT i = a.pers >= 16 ? 1 : 0; i < DgDrawState::vsCb; i++) if ((why = other(r.st.vcb[i]))) { r.st.clear(); return why; }
        for (UINT i = 0; i < DgDrawState::psCb; i++) if ((why = other(r.st.pcb[i]))) { r.st.clear(); return why; }
        for (UINT i = 0; i < DgDrawState::gsCb; i++) if ((why = other(r.st.gcb[i]))) { r.st.clear(); return why; }
        for (auto v : r.st.vb) addRef(v);
        addRef(r.st.ib);
        /* textures: the game's main colour/depth become the second eye's own (swap) */
        auto swapAll = [&](ID3D11ShaderResourceView **v, UINT n) -> bool {
            for (UINT i = 0; i < n; i++) {
                if (!v[i] || !a.swap) continue;
                ID3D11ShaderResourceView *mine = a.swap(v[i], a.swapCtx);
                if (mine == v[i]) continue;
                if (!mine) return false;
                mine->AddRef(); v[i]->Release(); v[i] = mine;
            }
            return true;
        };
        if (!swapAll(r.st.vsrv, DgDrawState::vsSrv) || !swapAll(r.st.psrv, DgDrawState::psSrv) || !swapAll(r.st.gsrv, DgDrawState::gsSrv)) { r.st.clear(); return "srv_swap"; }
        for (auto v : r.st.gsrv) if (v) addRef(resourceOf(v));
        for (auto v : r.st.vsrv) if (v) addRef(resourceOf(v));
        for (auto v : r.st.psrv) if (v) addRef(resourceOf(v));
        r.draw = a.indexed; r.plain = a.indexed ? nullptr : a.plain; r.n = a.n; r.first = a.first; r.base = a.base;
        r.ownTarget = a.ownTarget; r.rt = a.rt; r.ds = a.ds;
        if (r.rt) r.rt->AddRef();
        if (r.ds) r.ds->AddRef();
        ++used;
        return nullptr;
    }
    /* Replays the pending records into (rt, ds) and restores the game's state. */
    UINT flush(ID3D11DeviceContext1 *c, ID3D11RenderTargetView *rt, ID3D11DepthStencilView *ds) {
        if (!used) return 0;
        DgDrawState saved; saved.capture(c);
        ID3D11RenderTargetView *ort[8]{}; ID3D11DepthStencilView *ods = nullptr; c->OMGetRenderTargets(8, ort, &ods);
        ID3D11RenderTargetView *curRt = nullptr; ID3D11DepthStencilView *curDs = nullptr; bool set = false;
        const DgDrawState *prev = nullptr;
        for (UINT i = 0; i < used; i++) {
            DgDrawRec &r = recs[i];
            ID3D11RenderTargetView *wantRt = r.ownTarget ? r.rt : rt; ID3D11DepthStencilView *wantDs = r.ownTarget ? r.ds : ds;
            if (!set || wantRt != curRt || wantDs != curDs) { c->OMSetRenderTargets(1, &wantRt, wantDs); curRt = wantRt; curDs = wantDs; set = true; }
            r.st.apply(c, prev);
            if (r.draw) r.draw(c, r.n, r.first, r.base); else r.plain(c, r.n, r.first);
            prev = &r.st;
        }
        saved.apply(c, nullptr);
        c->OMSetRenderTargets(8, ort, ods);
        for (auto v : ort) if (v) v->Release();
        if (ods) ods->Release();
        saved.clear();
        UINT n = used;
        clear();
        return n;
    }
    void clear() {
        for (UINT i = 0; i < used; i++) {
            recs[i].st.clear();
            if (recs[i].rt) { recs[i].rt->Release(); recs[i].rt = nullptr; }
            if (recs[i].ds) { recs[i].ds->Release(); recs[i].ds = nullptr; }
            recs[i].ownTarget = false;
        }
        used = 0;
        for (UINT h : setUsed) set[h] = nullptr;
        setUsed.clear();
        for (auto &x : views) x = {};                   /* a released view's address may come back */
        viewNext = 0;
    }
};
