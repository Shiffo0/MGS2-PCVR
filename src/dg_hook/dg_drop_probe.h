#pragma once

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <d3d11.h>
#include <wrl/client.h>
#include <vector>

struct DgDropProbe {
    using Factory = HRESULT(STDMETHODCALLTYPE *)(ID3D11Device *, const D3D11_QUERY_DESC *, ID3D11Query **);
    static constexpr unsigned ring = 8, slots = 2048;
    static constexpr UINT64 visible = 2048;      /* native samples for "clearly visible" (4K x MSAA4 = 33M) */
    struct Slot {
        Microsoft::WRL::ComPtr<ID3D11Query> on, sn, orr, sr;   /* occlusion/statistics, native/replay */
        UINT64 hash = 0; UINT n = 0, first = 0; INT base = 0; int pers = -1;
        void *vb = nullptr, *ib = nullptr; UINT depthEnable = 0, depthFunc = 0, depthWrite = 0, stencil = 0;
        bool replayIssued = false, projective = false, rows = false;
        float pre[16]{}, prs[16]{};                 /* bank rows pers-4.. and pers.. as the game wrote them */
        bool natCopied = false, repCopied = false;  /* bank rows copied on the GPU into the frame's staging */
        int cap = -1;                               /* vertex capture index in the frame, -1 none */
        bool measuredOk = false; UINT64 msn = 0, msr = 0, nco = 0, rco = 0, rps = 0;   /* read back (occluder search) */
        bool capNat = false, capRep = false;
    };
    struct Frame { std::vector<Slot> s; unsigned used = 0, frameNo = 0; bool pending = false; DWORD tick = 0; SYSTEMTIME clock{};
                   float D[16]{}, Sc[16]{}, Tc[16]{}; int sourceEye = -1;
                   Microsoft::WRL::ComPtr<ID3D11Buffer> stage;      /* slots x 256 bytes: native rows, replay rows */
                   Microsoft::WRL::ComPtr<ID3D11Buffer> vstage;     /* caps x 2 x capBytes: native, replay clip positions */
                   Microsoft::WRL::ComPtr<ID3D11Query> soq[8][2]; unsigned caps = 0; };
    static constexpr UINT bankBytes = 128, slotBytes = 256;
    static constexpr UINT capsPerFrame = 4, capPrims = 1024, capBytes = capPrims * 3 * 16;
    Microsoft::WRL::ComPtr<ID3D11Buffer> soBuf;
    struct SoGs { UINT64 hash; Microsoft::WRL::ComPtr<ID3D11GeometryShader> gs; bool failed; };
    SoGs soGs[64]{}; unsigned soGsN = 0;
    unsigned long long vertCaptures = 0, vertSkipped = 0, vertFail = 0, vertLogged = 0;
    const char *vertLastSkip = "none";
    unsigned long long bankCopies = 0, bankSkipped = 0, bankMapMiss = 0, stageFail = 0;
    Frame fr[ring]; unsigned cur = 0; bool open = false, enabled = false;  /* off by default; the game build sets it from vr_se_mode bit 16 */
    Factory factory = nullptr; ID3D11Device *dev = nullptr;
    /* counters (since the last report) */
    unsigned long long measured = 0, flagged = 0, framesFlagged = 0, framesRead = 0, framesLate = 0, framesFull = 0, createFail = 0, logged = 0;
    unsigned long long ratio[5]{};                  /* visible natively: replay/native <1%, <10%, <50%, <90%, >=90% */
    DWORD logSecond = 0; unsigned logThisSecond = 0;
    int k = -1;                                     /* slot of the draw in flight */
    /* per-draw history, open addressing on the identity key */
    struct Hist { UINT64 key; unsigned okStreak, dropFrame, lastFrame; bool dropped; unsigned drops, occl; };
    UINT64 burstMin = 100000; unsigned burstDrops = 2;     /* a burst frame: >= burstDrops losses of >= burstMin native samples */
    unsigned long long bursts = 0, occluderLines = 0;
    static constexpr unsigned histSize = 1u << 14;
    std::vector<Hist> hist = std::vector<Hist>(histSize);
    unsigned long long transitions = 0, restores = 0, steadyDrops = 0, histFull = 0;
    Hist *histOf(UINT64 key) {
        for (unsigned h = (unsigned)((key * 0x9E3779B97F4A7C15ull) >> 50) & (histSize - 1), n = 0; n < histSize; n++, h = (h + 1) & (histSize - 1)) {
            if (hist[h].key == key) return &hist[h];
            if (!hist[h].key) { hist[h] = Hist{ key, 0, 0, 0, false, 0, 0 }; return &hist[h]; }
        }
        ++histFull; return nullptr;
    }

    bool make(Slot &s) {
        if (s.on) return true;
        D3D11_QUERY_DESC q{ D3D11_QUERY_OCCLUSION, 0 };
        if (FAILED(factory(dev, &q, &s.on)) || FAILED(factory(dev, &q, &s.orr))) { ++createFail; s.on.Reset(); return false; }
        q.Query = D3D11_QUERY_PIPELINE_STATISTICS;
        if (FAILED(factory(dev, &q, &s.sn)) || FAILED(factory(dev, &q, &s.sr))) { ++createFail; s.on.Reset(); s.orr.Reset(); return false; }
        return true;
    }
    /* Frame start (first replayed world draw). The ring slot must have been read. */
    void beginFrame(unsigned frameNo, const float D[16], const float Sc[16], const float Tc[16], int sourceEye) {
        open = false;
        if (!enabled || !factory) return;
        Frame &f = fr[cur];
        if (f.pending) { ++framesLate; return; }
        if (f.s.empty()) f.s.resize(slots);
        if (!f.stage && dev) {
            D3D11_BUFFER_DESC bd{ slots * slotBytes, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0, 0 };
            if (FAILED(dev->CreateBuffer(&bd, nullptr, &f.stage))) { ++stageFail; f.stage.Reset(); }
        }
        f.used = 0; f.caps = 0; f.frameNo = frameNo; f.tick = GetTickCount(); GetLocalTime(&f.clock); f.sourceEye = sourceEye;
        memcpy(f.D, D, 64); memcpy(f.Sc, Sc, 64); memcpy(f.Tc, Tc, 64);
        open = true;
    }
    /* Around the game's own draw. createAllowed: the factory may be called now. */
    template<class Inside> void beginNative(ID3D11DeviceContext *c, Inside inside, UINT64 hash, UINT n, UINT first, INT base, int pers,
                                           const float *pre, const float *prs) {
        k = -1;
        if (!open) return;
        Frame &f = fr[cur];
        if (f.used >= slots) { ++framesFull; return; }
        Slot &s = f.s[f.used];
        bool ok; inside(true); ok = make(s); inside(false);
        if (!ok) return;
        k = (int)f.used++;
        s.hash = hash; s.n = n; s.first = first; s.base = base; s.pers = pers; s.replayIssued = false; s.projective = false;
        s.natCopied = s.repCopied = false; s.cap = -1; s.capNat = s.capRep = false;
        s.measuredOk = false; s.msn = s.msr = s.nco = s.rco = s.rps = 0;
        s.rows = pre && prs; if (s.rows) { memcpy(s.pre, pre, 64); memcpy(s.prs, prs, 64); }
        { ID3D11Buffer *vb = nullptr; UINT st = 0, of = 0; c->IAGetVertexBuffers(0, 1, &vb, &st, &of); s.vb = vb; if (vb) vb->Release(); }
        { ID3D11Buffer *ib = nullptr; DXGI_FORMAT fm{}; UINT of = 0; c->IAGetIndexBuffer(&ib, &fm, &of); s.ib = ib; if (ib) ib->Release(); }
        { ID3D11DepthStencilState *d = nullptr; UINT r = 0; c->OMGetDepthStencilState(&d, &r);
          if (d) { D3D11_DEPTH_STENCIL_DESC x{}; d->GetDesc(&x); s.depthEnable = x.DepthEnable; s.depthFunc = x.DepthFunc; s.depthWrite = x.DepthWriteMask; s.stencil = x.StencilEnable; d->Release(); }
          else { s.depthEnable = 1; s.depthFunc = D3D11_COMPARISON_LESS; s.depthWrite = 1; s.stencil = 0; } }
        c->Begin(s.on.Get()); c->Begin(s.sn.Get());
    }
    /* The rows pers-4..pers+3 of the bound bank, copied on the GPU (command order = what the draw
       used). cb/byteOffset: the constant buffer bound at the bank slot and the byte offset of row
       pers-4 in it. Call between beginNative and endNative, and after the replay's bind. */
    bool copyRows(ID3D11DeviceContext *c, ID3D11Buffer *cb, UINT byteOffset, UINT dst) {
        if (k < 0 || !cb) { ++bankSkipped; return false; }
        Frame &f = fr[cur]; if (!f.stage) { ++bankSkipped; return false; }
        D3D11_BUFFER_DESC bd{}; cb->GetDesc(&bd);
        if (byteOffset + bankBytes > bd.ByteWidth) { ++bankSkipped; return false; }
        D3D11_BOX box{ byteOffset, 0, 0, byteOffset + bankBytes, 1, 1 };
        c->CopySubresourceRegion(f.stage.Get(), 0, (UINT)k * slotBytes + dst, 0, 0, cb, 0, &box); ++bankCopies;
        return true;
    }
    void captureNative(ID3D11DeviceContext *c, ID3D11Buffer *cb, UINT byteOffset) {
        if (k >= 0 && copyRows(c, cb, byteOffset, 0)) fr[cur].s[k].natCopied = true;
    }
    void captureReplay(ID3D11DeviceContext *c, ID3D11Buffer *cb, UINT byteOffset) {
        if (k >= 0 && copyRows(c, cb, byteOffset, bankBytes)) fr[cur].s[k].repCopied = true;
    }
    /* identity key of slot i of frame f (same as read() uses) */
    static UINT64 keyOf(const Frame &f, unsigned i) {
        const Slot &s = f.s[i];
        UINT64 key = s.hash * 0x100000001B3ull ^ ((UINT64)s.n << 40) ^ ((UINT64)s.first << 8) ^ (UINT64)(uint32_t)s.base ^
                     (UINT64)(uintptr_t)s.vb * 31 ^ (UINT64)(uintptr_t)s.ib * 131 ^ ((UINT64)s.depthWrite << 63);
        unsigned occ = 0; for (unsigned q2 = 0; q2 < i && q2 < 64; q2++) { const Slot &o = f.s[i - 1 - q2]; if (o.hash == s.hash && o.n == s.n && o.first == s.first && o.vb == s.vb && o.depthWrite == s.depthWrite) occ++; }
        key ^= (UINT64)occ * 0xC2B2AE3D27D4EB4Full; if (!key) key = 1;
        return key;
    }
    /* After beginNative: is the draw in flight a suspect (dropped before) worth a vertex capture?
       prims = primitives the draw makes (strip: n-2). */
    bool wantVerts(UINT prims) {
        if (k < 0) return false;
        Frame &f = fr[cur]; Slot &s = f.s[k];
        if (f.caps >= capsPerFrame) return false;
        UINT64 key = keyOf(f, (unsigned)k);
        bool suspect = false;
        for (unsigned h = (unsigned)((key * 0x9E3779B97F4A7C15ull) >> 50) & (histSize - 1), n = 0; n < histSize; n++, h = (h + 1) & (histSize - 1)) {
            if (hist[h].key == key) { suspect = hist[h].drops > 0 || hist[h].occl > 0; break; }
            if (!hist[h].key) break;
        }
        if (!suspect) return false;
        if (prims == 0 || prims > capPrims) { ++vertSkipped; vertLastSkip = "too_many_primitives"; return false; }
        s.cap = (int)f.caps++;
        return true;
    }

    template<class DrawOnce> void captureVerts(ID3D11DeviceContext *c, const BYTE *vsCode, UINT vsSize, UINT64 vsHash, bool replay, DrawOnce drawOnce) {
        if (k < 0) return;
        Frame &f = fr[cur]; Slot &s = f.s[k];
        if (s.cap < 0 || !dev || !vsCode || !vsSize) return;
        auto skip = [&](const char *why) { ++vertSkipped; vertLastSkip = why; };
        { ID3D11GeometryShader *g = nullptr; c->GSGetShader(&g, nullptr, nullptr); if (g) { g->Release(); skip("game_gs_bound"); return; } }
        { ID3D11HullShader *h = nullptr; c->HSGetShader(&h, nullptr, nullptr); if (h) { h->Release(); skip("tessellation"); return; } }
        { ID3D11Buffer *so = nullptr; c->SOGetTargets(1, &so); if (so) { so->Release(); skip("game_so_bound"); return; } }
        if (!soBuf) { D3D11_BUFFER_DESC bd{ capBytes, D3D11_USAGE_DEFAULT, D3D11_BIND_STREAM_OUTPUT, 0, 0, 0 };
                      if (FAILED(dev->CreateBuffer(&bd, nullptr, &soBuf))) { ++vertFail; soBuf.Reset(); return; } }
        if (!f.vstage) { D3D11_BUFFER_DESC bd{ capsPerFrame * 2 * capBytes, D3D11_USAGE_STAGING, 0, D3D11_CPU_ACCESS_READ, 0, 0 };
                         if (FAILED(dev->CreateBuffer(&bd, nullptr, &f.vstage))) { ++vertFail; f.vstage.Reset(); return; } }
        ID3D11GeometryShader *gs = nullptr;
        for (unsigned i = 0; i < soGsN; i++) if (soGs[i].hash == vsHash) { gs = soGs[i].gs.Get(); if (soGs[i].failed) { skip("so_gs_failed"); return; } break; }
        if (!gs) {
            if (soGsN >= 64) { skip("so_gs_cache_full"); return; }
            SoGs &e = soGs[soGsN++]; e.hash = vsHash;
            D3D11_SO_DECLARATION_ENTRY decl{ 0, "SV_Position", 0, 0, 4, 0 }; UINT stride = 16;
            if (FAILED(dev->CreateGeometryShaderWithStreamOutput(vsCode, vsSize, &decl, 1, &stride, 1, D3D11_SO_NO_RASTERIZED_STREAM, nullptr, &e.gs))) { e.failed = true; ++vertFail; return; }
            gs = e.gs.Get();
        }
        auto &q = f.soq[s.cap][replay ? 1 : 0];
        if (!q) { D3D11_QUERY_DESC qd{ D3D11_QUERY_SO_STATISTICS, 0 }; if (!factory || FAILED(factory(dev, &qd, &q))) { ++vertFail; q.Reset(); return; } }
        ID3D11Buffer *sb = soBuf.Get(); UINT off = 0;
        c->GSSetShader(gs, nullptr, 0); c->SOSetTargets(1, &sb, &off);
        c->Begin(q.Get()); drawOnce(); c->End(q.Get());
        ID3D11Buffer *none = nullptr; c->SOSetTargets(1, &none, &off); c->GSSetShader(nullptr, nullptr, 0);
        D3D11_BOX box{ 0, 0, 0, capBytes, 1, 1 };
        c->CopySubresourceRegion(f.vstage.Get(), 0, ((UINT)s.cap * 2 + (replay ? 1 : 0)) * capBytes, 0, 0, soBuf.Get(), 0, &box);
        (replay ? s.capRep : s.capNat) = true; ++vertCaptures;
    }
    template<class Log> void logVerts(ID3D11DeviceContext *c, Frame &f, Slot &s, double q, const unsigned char *vb, Log log) {
        if (s.cap < 0 || !(s.capNat || s.capRep)) return;
        if (!lineBudget(log)) return;
        ++vertLogged;
        log("drop_probe: verts %02u:%02u:%02u.%03u frame %u vs %016llX n %u first %u vb %p | replay/native %.3f%s\r\n",
            f.clock.wHour, f.clock.wMinute, f.clock.wSecond, f.clock.wMilliseconds, f.frameNo, (unsigned long long)s.hash, s.n, s.first, s.vb, q,
            q < 0 ? " (not visible natively)" : q < 0.05 ? " DROPPED" : "");
        if (!vb) { log("  verts: staging not ready\r\n"); return; }
        for (int r = 0; r < 2; r++) {
            if (!(r ? s.capRep : s.capNat)) continue;
            D3D11_QUERY_DATA_SO_STATISTICS st{};
            UINT64 prims = c->GetData(f.soq[s.cap][r].Get(), &st, sizeof st, D3D11_ASYNC_GETDATA_DONOTFLUSH) == S_OK ? st.NumPrimitivesWritten : 0;
            UINT nv = (UINT)(prims * 3 > capPrims * 3 ? capPrims * 3 : prims * 3);
            const float *v = (const float *)(vb + ((UINT)s.cap * 2 + r) * capBytes);
            float lo[4] = { 1e30f, 1e30f, 1e30f, 1e30f }, hi[4] = { -1e30f, -1e30f, -1e30f, -1e30f }; UINT wneg = 0, inside = 0;
            for (UINT i = 0; i < nv; i++) {
                const float *p = v + i * 4; float w = p[3];
                if (!(w > 0)) { ++wneg; continue; }
                float e[4] = { p[0] / w, p[1] / w, p[2] / w, w };
                for (int j = 0; j < 4; j++) { if (e[j] < lo[j]) lo[j] = e[j]; if (e[j] > hi[j]) hi[j] = e[j]; }
                if (fabsf(e[0]) <= 1 && fabsf(e[1]) <= 1 && e[2] >= 0 && e[2] <= 1) ++inside;
            }
            log("  %s: prims %llu verts %u w<=0 %u inside %u | x/w %.4g to %.4g y/w %.4g to %.4g z/w %.4g to %.4g w %.6g to %.6g\r\n",
                r ? "replay" : "native", prims, nv, wneg, inside, lo[0], hi[0], lo[1], hi[1], lo[2], hi[2], lo[3], hi[3]);
            if (nv) log("  %s v0 %.6g %.6g %.6g %.6g | v1 %.6g %.6g %.6g %.6g | v2 %.6g %.6g %.6g %.6g\r\n", r ? "replay" : "native",
                        v[0], v[1], v[2], v[3], nv > 1 ? v[4] : 0, nv > 1 ? v[5] : 0, nv > 1 ? v[6] : 0, nv > 1 ? v[7] : 0,
                        nv > 2 ? v[8] : 0, nv > 2 ? v[9] : 0, nv > 2 ? v[10] : 0, nv > 2 ? v[11] : 0);
        }
    }
    static float maxDiff(const float *a, const float *b, int n) { float m = 0; for (int i = 0; i < n; i++) { float d = fabsf(a[i] - b[i]); if (!(d <= m)) m = d; } return m; }
    void endNative(ID3D11DeviceContext *c) { if (k < 0) return; Slot &s = fr[cur].s[k]; c->End(s.sn.Get()); c->End(s.on.Get()); }
    void beginReplay(ID3D11DeviceContext *c) { if (k < 0) return; Slot &s = fr[cur].s[k]; c->Begin(s.orr.Get()); c->Begin(s.sr.Get()); }
    void endReplay(ID3D11DeviceContext *c, bool projective) {
        if (k < 0) return; Slot &s = fr[cur].s[k]; c->End(s.sr.Get()); c->End(s.orr.Get()); s.replayIssued = true; s.projective = projective; k = -1;
    }
    void endFrame() { if (!open) return; open = false; Frame &f = fr[cur]; if (f.used) { f.pending = true; cur = (cur + 1) % ring; } }

    /* At Present: read every finished frame (never waits). */
    template<class Log> void poll(ID3D11DeviceContext *c, Log log) {
        for (unsigned i = 0; i < ring; i++) {
            Frame &f = fr[(cur + i) % ring];
            if (!f.pending || !f.used) continue;
            /* GPU order: when the last replay statistics are in, the frame is done */
            Slot &last = f.s[f.used - 1];
            D3D11_QUERY_DATA_PIPELINE_STATISTICS probe{};
            ID3D11Query *lq = last.replayIssued ? last.sr.Get() : last.sn.Get();
            HRESULT hr = c->GetData(lq, &probe, sizeof probe, D3D11_ASYNC_GETDATA_DONOTFLUSH);
            if (hr == S_FALSE) continue;
            read(c, f, log);
            f.pending = false;
        }
    }
    template<class Log> void read(ID3D11DeviceContext *c, Frame &f, Log log) {
        ++framesRead; unsigned flagsHere = 0;
        const unsigned char *bank = nullptr; D3D11_MAPPED_SUBRESOURCE mp{};
        const unsigned char *vbank = nullptr; D3D11_MAPPED_SUBRESOURCE vmp{}; bool vmapped = false;
        auto mapVerts = [&]() {
            if (vmapped || !f.vstage) return; vmapped = true;
            if (SUCCEEDED(c->Map(f.vstage.Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &vmp))) vbank = (const unsigned char *)vmp.pData;
        };
        bool mapped = false;
        auto mapBank = [&]() {                       /* lazily, only when a transition is logged */
            if (mapped || !f.stage) return; mapped = true;
            if (SUCCEEDED(c->Map(f.stage.Get(), 0, D3D11_MAP_READ, D3D11_MAP_FLAG_DO_NOT_WAIT, &mp))) bank = (const unsigned char *)mp.pData; else ++bankMapMiss;
        };
        unsigned bigDrops = 0, lastBig = 0; UINT64 bigNative = 0;
        for (unsigned i = 0; i < f.used; i++) {
            Slot &s = f.s[i];
            UINT64 sn = 0, sr = 0; D3D11_QUERY_DATA_PIPELINE_STATISTICS pn{}, pr{};
            if (c->GetData(s.on.Get(), &sn, sizeof sn, D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK) continue;
            c->GetData(s.sn.Get(), &pn, sizeof pn, D3D11_ASYNC_GETDATA_DONOTFLUSH);
            if (!s.replayIssued) continue;
            if (c->GetData(s.orr.Get(), &sr, sizeof sr, D3D11_ASYNC_GETDATA_DONOTFLUSH) != S_OK) continue;
            c->GetData(s.sr.Get(), &pr, sizeof pr, D3D11_ASYNC_GETDATA_DONOTFLUSH);
            ++measured;
            s.measuredOk = true; s.msn = sn; s.msr = sr; s.nco = pn.CPrimitives; s.rco = pr.CPrimitives; s.rps = pr.PSInvocations;
            /* identity: same draw in the same frame position class; occurrence keeps repeats apart */
            UINT64 key = keyOf(f, i);
            if (s.cap >= 0) { mapVerts(); logVerts(c, f, s, sn ? (double)sr / (double)sn : -1.0, vbank, log); }
            if (sn < visible) continue;
            double q = (double)sr / (double)sn;
            ratio[q < 0.01 ? 0 : q < 0.1 ? 1 : q < 0.5 ? 2 : q < 0.9 ? 3 : 4]++;
            Hist *h = histOf(key);
            bool dropNow = q < 0.05;
            if (dropNow) { ++flagged; ++flagsHere; }
            if (!h) continue;
            if (!dropNow) {
                if (h->dropped && q >= 0.5) {
                    h->dropped = false; ++restores;
                    if (lineBudget(log))
                        log("drop_probe: RESTORED %02u:%02u:%02u.%03u frame %u vs %016llX n %u first %u vb %p after %u frames; native %llu replay %llu\r\n",
                            f.clock.wHour, f.clock.wMinute, f.clock.wSecond, f.clock.wMilliseconds, f.frameNo, (unsigned long long)s.hash, s.n, s.first, s.vb,
                            f.frameNo - h->dropFrame, sn, sr);
                }
                if (q >= 0.5) h->okStreak++;
                h->lastFrame = f.frameNo;
                continue;
            }
            if (h->dropped || h->okStreak < 3) { if (!h->dropped) ++steadyDrops; h->okStreak = 0; h->lastFrame = f.frameNo; continue; }
            /* visible in the replay before, gone now: the brief loss */
            h->dropped = true; h->dropFrame = f.frameNo; h->okStreak = 0; h->lastFrame = f.frameNo; ++transitions; ++h->drops;
            if (sn >= burstMin) { ++bigDrops; bigNative += sn; lastBig = i; }   /* a burst counts new losses, not steady ones */
            if (!lineBudget(log)) continue;
            ++logged;
            log("drop_probe: TRANSITION %02u:%02u:%02u.%03u frame %u draw %u/%u vs %016llX n %u first %u base %d pers %d vb %p ib %p depth %u func %u write %u stencil %u projective %d | native samples %llu clip %llu->%llu ps %llu | replay samples %llu ia %llu clip %llu->%llu ps %llu | source eye %d\r\n",
                f.clock.wHour, f.clock.wMinute, f.clock.wSecond, f.clock.wMilliseconds, f.frameNo, i, f.used, (unsigned long long)s.hash, s.n, s.first, s.base, s.pers, s.vb, s.ib,
                s.depthEnable, s.depthFunc, s.depthWrite, s.stencil, (int)s.projective,
                sn, pn.CInvocations, pn.CPrimitives, pn.PSInvocations, sr, pr.IAPrimitives, pr.CInvocations, pr.CPrimitives, pr.PSInvocations, f.sourceEye);
            log("  D %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g\r\n",
                f.D[0], f.D[1], f.D[2], f.D[3], f.D[4], f.D[5], f.D[6], f.D[7], f.D[8], f.D[9], f.D[10], f.D[11], f.D[12], f.D[13], f.D[14], f.D[15]);
            if (s.rows)
                log("  rows c%d.. %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g ; c%d.. %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g\r\n",
                    s.pers - 4, s.pre[0], s.pre[1], s.pre[2], s.pre[3], s.pre[4], s.pre[5], s.pre[6], s.pre[7], s.pre[8], s.pre[9], s.pre[10], s.pre[11], s.pre[12], s.pre[13], s.pre[14], s.pre[15],
                    s.pers, s.prs[0], s.prs[1], s.prs[2], s.prs[3], s.prs[4], s.prs[5], s.prs[6], s.prs[7], s.prs[8], s.prs[9], s.prs[10], s.prs[11], s.prs[12], s.prs[13], s.prs[14], s.prs[15]);
            logBank(f, s, i, mapBank, bank, log);
        }
        if (mapped && bank) c->Unmap(f.stage.Get(), 0);
        if (vmapped && vbank) c->Unmap(f.vstage.Get(), 0);
        if (flagsHere) ++framesFlagged;
        if (bigDrops >= burstDrops) occluders(f, bigDrops, bigNative, lastBig, log);
    }
    /* A burst frame: the depth-writing draws before the last large loss that pass clearly more samples
       in the replay than natively, largest surplus first. They become suspects (vertex capture). */
    template<class Log> void occluders(Frame &f, unsigned bigDrops, UINT64 bigNative, unsigned lastBig, Log log) {
        ++bursts;
        unsigned best[6]; INT64 surplus[6]; unsigned nb = 0;
        for (unsigned i = 0; i < lastBig; i++) {
            const Slot &s = f.s[i];
            if (!s.measuredOk || !s.depthWrite || !s.depthEnable) continue;
            if (s.msr < burstMin / 2 || s.msr <= 2 * s.msn) continue;
            INT64 sp = (INT64)(s.msr - s.msn);
            unsigned at = nb < 6 ? nb++ : 6;
            if (at == 6) { if (sp <= surplus[5]) continue; at = 5; }
            while (at > 0 && surplus[at - 1] < sp) { best[at] = best[at - 1]; surplus[at] = surplus[at - 1]; at--; }
            best[at] = i; surplus[at] = sp;
        }
        if (!lineBudget(log)) return;
        log("drop_probe: BURST %02u:%02u:%02u.%03u frame %u: %u large new losses (%llu native samples) up to draw %u; replay-only depth writers before them: %u\r\n",
            f.clock.wHour, f.clock.wMinute, f.clock.wSecond, f.clock.wMilliseconds, f.frameNo, bigDrops, (unsigned long long)bigNative, lastBig, nb);
        for (unsigned j = 0; j < nb; j++) {
            const Slot &s = f.s[best[j]];
            if (Hist *h = histOf(keyOf(f, best[j]))) h->occl++;
            if (!lineBudget(log)) continue;
            ++occluderLines;
            log("  occluder? draw %u vs %016llX n %u first %u base %d pers %d vb %p write %u func %u | native samples %llu clip-out %llu | replay samples %llu clip-out %llu ps %llu\r\n",
                best[j], (unsigned long long)s.hash, s.n, s.first, s.base, s.pers, s.vb, s.depthWrite, s.depthFunc,
                (unsigned long long)s.msn, (unsigned long long)s.nco, (unsigned long long)s.msr, (unsigned long long)s.rco, (unsigned long long)s.rps);
        }
    }
    template<class Map, class Log> void logBank(Frame &f, Slot &s, unsigned i, Map mapBank, const unsigned char *&bank, Log log) {
        if (!s.natCopied && !s.repCopied) { log("  gpu bank: not captured\r\n"); return; }
        mapBank();
        if (!bank) { log("  gpu bank: staging not ready\r\n"); return; }
        const float *gn = (const float *)(bank + i * slotBytes), *gr = (const float *)(bank + i * slotBytes + bankBytes);
        auto rows = [&](const char *what, const float *r) {
            log("  gpu %s c%d.. %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g ; c%d.. %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g | %.6g %.6g %.6g %.6g\r\n",
                what, s.pers - 4, r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15],
                s.pers, r[16], r[17], r[18], r[19], r[20], r[21], r[22], r[23], r[24], r[25], r[26], r[27], r[28], r[29], r[30], r[31]);
        };
        if (s.natCopied) rows("native", gn);
        if (s.repCopied) rows("replay", gr);
        /* expected replay rows pers..: out[i*4+j] = sum_k D[k*4+i] * rows[k*4+j] (affine case) */
        float ex[16]; for (int e = 0; e < 16; e++) { int a = e / 4, b = e % 4; float t = 0; for (int q = 0; q < 4; q++) t += f.D[q * 4 + a] * gn[16 + q * 4 + b]; ex[e] = t; }
        log("  bank check: shadow-vs-gpu native c%d.. %.6g c%d.. %.6g | replay-vs-expected c%d.. %.6g | replay-vs-native c%d.. %.6g\r\n",
            s.pers - 4, s.natCopied && s.rows ? maxDiff(s.pre, gn, 16) : -1.f, s.pers, s.natCopied && s.rows ? maxDiff(s.prs, gn + 16, 16) : -1.f,
            s.pers, s.natCopied && s.repCopied ? maxDiff(gr + 16, ex, 16) : -1.f, s.pers - 4, s.natCopied && s.repCopied ? maxDiff(gr, gn, 16) : -1.f);
    }
    /* at most 40 transition/restore lines a second (each transition is 3 lines) */
    template<class Log> bool lineBudget(Log) {
        DWORD now = GetTickCount();
        if (now / 1000 != logSecond) { logSecond = now / 1000; logThisSecond = 0; }
        return ++logThisSecond <= 40;
    }
    template<class Log> void report(Log log) {
        log("drop_probe: frames read %llu (late %llu, full %llu) draws compared %llu; visible natively, replay/native <1%%/<10%%/<50%%/<90%%/>=90%%: %llu/%llu/%llu/%llu/%llu; below 5%%: %llu in %llu frames, of which TRANSITIONS %llu (logged %llu), RESTORED %llu, steady %llu; history full %llu; query create failures %llu\r\n",
            framesRead, framesLate, framesFull, measured, ratio[0], ratio[1], ratio[2], ratio[3], ratio[4], flagged, framesFlagged, transitions, logged, restores, steadyDrops, histFull, createFail);
        log("drop_probe: bank copies %llu skipped %llu staging-not-ready %llu staging create failures %llu\r\n", bankCopies, bankSkipped, bankMapMiss, stageFail);
        log("drop_probe: vertex captures %llu logged %llu skipped %llu (last %s) failures %llu; burst frames %llu occluder lines %llu\r\n", vertCaptures, vertLogged, vertSkipped, vertLastSkip, vertFail, bursts, occluderLines);
        bankCopies = bankSkipped = bankMapMiss = 0; vertCaptures = vertLogged = vertSkipped = 0; bursts = occluderLines = 0;
        framesRead = framesLate = framesFull = measured = flagged = framesFlagged = logged = 0; memset(ratio, 0, sizeof ratio);
        transitions = restores = steadyDrops = 0;
    }
};
