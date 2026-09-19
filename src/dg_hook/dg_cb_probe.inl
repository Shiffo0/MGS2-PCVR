// dg_cb_probe.inl - read-only constant-buffer capture (2026-09-18).
//
// Question this answers: does the game hand the GPU a view-projection matrix
// per draw in a vertex-shader constant buffer? If our own eye_pers (or
// world * eye_pers) shows up there, a second eye can be produced by issuing
// each draw again with M * inverse(VP_this_eye) * VP_other_eye - the way
// generic stereo injectors work. If the port transforms vertices on the CPU,
// that route is dead and no amount of draw replay will give a second eye.
// It also labels the flat 2D draws (camera overlay, HUD): no VP, depth off.
//
// Read-only by construction: per captured draw the bound VS constant buffers
// are copied into OUR staging buffers and mapped for read. No pipeline state
// is set, no game memory is written, no draw is added or skipped. The Map
// stalls the GPU once per buffer, so a capture is a visible hitch of a second
// or so - bounded to cbFrames frames, then it is over.
//
// Included inside dg_draw_trial.cpp's anonymous namespace: it uses that
// file's device/context/multi/lifetime/logger and rides its two native
// call-site callbacks, which already see every game draw.

UINT64 vsHashOf(void *ptr, const BYTE **bytes = nullptr, UINT32 *size = nullptr);   // dg_ui2d.inl

constexpr unsigned cbSlots = 8;          // VS constant-buffer slots looked at
constexpr unsigned cbMaxBytes = 16384;   // per buffer (the game's register bank is 10240), the rest is counted as truncated
constexpr unsigned cbMaxDraws = 4000;
constexpr unsigned cbFrames = 2;         // one left + one right eye frame
constexpr size_t   cbArenaBytes = 64u << 20;
constexpr unsigned cbCamCount = 8;
constexpr unsigned cbMaxCaptures = 8;    // per process

#pragma pack(push, 1)
struct CbFileHeader {
    char magic[8];                       // "DGCBP1\0\0"
    UINT32 version, draws, frames, truncated, errors, camCount, camStride, reserved;
    UINT64 arenaBytes, startTick;
};
struct CbCamOut { INT32 seq, eye; UINT64 tick; float eye_pers[16], pers[16], eye_inv[16], eye_world[16]; };
struct CbDrawRec {
    UINT32 size, frame, index, kind, count, first; INT32 base; UINT32 thread;
    UINT64 vs, rtv, dsv, layout;
    float viewport[6];
    UINT32 topology, depthEnable, depthWrite, ncb;
    UINT64 tick;
    UINT64 vsHash;                       // version 2: FNV-1a 64 of the vertex shader's bytecode, 0 = unknown
};
struct CbBufRec { UINT32 slot, byteWidth, captured, usage; UINT64 ptr; };
#pragma pack(pop)

struct CbCam { volatile LONG seq; LONG eye; ULONGLONG tick; float m[4][16]; };
CbCam cbCams[cbCamCount];
volatile LONG cbCamNext = 0;

// 0 idle, 1 armed (waits for a Present), 2 capturing, 3 complete (waits for the writer)
volatile LONG cbState = 0;
BYTE *cbArena = nullptr;
size_t cbUsed = 0;
unsigned cbDraws = 0, cbFrame = 0, cbTruncated = 0, cbErrors = 0, cbCaptures = 0;
ULONGLONG cbStartTick = 0;
char cbToken[24] = "";
struct CbStaging { UINT bytes = 0; ComPtr<ID3D11Buffer> buffer; };
CbStaging cbStaging[24];
unsigned cbStagingCount = 0;

ID3D11Buffer *cbStagingFor(ID3D11Device *d, UINT bytes) {
    for (unsigned i = 0; i < cbStagingCount; i++)
        if (cbStaging[i].bytes == bytes) return cbStaging[i].buffer.Get();
    if (cbStagingCount >= sizeof cbStaging / sizeof cbStaging[0]) return nullptr;
    D3D11_BUFFER_DESC desc{};
    desc.ByteWidth = bytes; desc.Usage = D3D11_USAGE_STAGING; desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    ComPtr<ID3D11Buffer> b;
    if (FAILED(d->CreateBuffer(&desc, nullptr, &b))) return nullptr;
    cbStaging[cbStagingCount].bytes = bytes; cbStaging[cbStagingCount].buffer = b;
    return cbStaging[cbStagingCount++].buffer.Get();
}

// The draw path. kind 0 = DrawIndexed, 1 = Draw. Runs BEFORE the draw is
// forwarded, so the buffers hold what this draw will use.
void cbOnDraw(ID3D11DeviceContext *c, unsigned kind, UINT count, UINT first, INT base) {
    if (InterlockedCompareExchange(&cbState, 0, 0) != 2) return;
    if (!TryAcquireSRWLockShared(&lifetime)) return;
    struct Release { ~Release() { ReleaseSRWLockShared(&lifetime); } } release;
    if (c != context.Get() || stopping || !cbArena) return;
    MultiLock lock;
    if (cbDraws >= cbMaxDraws || cbUsed + sizeof(CbDrawRec) + cbSlots * (sizeof(CbBufRec) + cbMaxBytes) > cbArenaBytes) {
        cbTruncated++; return;
    }
    auto *rec = (CbDrawRec *)(cbArena + cbUsed);
    memset(rec, 0, sizeof *rec);
    rec->frame = cbFrame; rec->index = cbDraws; rec->kind = kind; rec->count = count; rec->first = first;
    rec->base = base; rec->thread = GetCurrentThreadId(); rec->tick = GetTickCount64();
    {
        ID3D11VertexShader *vs = nullptr; ID3D11ClassInstance *ci[1]; UINT nci = 0;
        c->VSGetShader(&vs, ci, &nci); rec->vs = (UINT64)(ULONG_PTR)vs; rec->vsHash = vsHashOf(vs); if (vs) vs->Release();
        ID3D11RenderTargetView *rt = nullptr; ID3D11DepthStencilView *ds = nullptr;
        c->OMGetRenderTargets(1, &rt, &ds);
        rec->rtv = (UINT64)(ULONG_PTR)rt; rec->dsv = (UINT64)(ULONG_PTR)ds;
        if (rt) rt->Release(); if (ds) ds->Release();
        ID3D11InputLayout *il = nullptr; c->IAGetInputLayout(&il); rec->layout = (UINT64)(ULONG_PTR)il; if (il) il->Release();
        D3D11_VIEWPORT vp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]; UINT nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
        c->RSGetViewports(&nvp, vp);
        if (nvp) { rec->viewport[0] = vp[0].TopLeftX; rec->viewport[1] = vp[0].TopLeftY; rec->viewport[2] = vp[0].Width;
                   rec->viewport[3] = vp[0].Height; rec->viewport[4] = vp[0].MinDepth; rec->viewport[5] = vp[0].MaxDepth; }
        D3D11_PRIMITIVE_TOPOLOGY topo = D3D11_PRIMITIVE_TOPOLOGY_UNDEFINED; c->IAGetPrimitiveTopology(&topo); rec->topology = (UINT32)topo;
        ID3D11DepthStencilState *dss = nullptr; UINT ref = 0; c->OMGetDepthStencilState(&dss, &ref);
        if (dss) { D3D11_DEPTH_STENCIL_DESC dd{}; dss->GetDesc(&dd); rec->depthEnable = dd.DepthEnable; rec->depthWrite = dd.DepthWriteMask; dss->Release(); }
        else { rec->depthEnable = 1; rec->depthWrite = 1; }   // the D3D11 default state
    }
    size_t at = cbUsed + sizeof *rec;
    ID3D11Buffer *bufs[cbSlots]{};
    c->VSGetConstantBuffers(0, cbSlots, bufs);
    for (unsigned s = 0; s < cbSlots; s++) {
        ID3D11Buffer *b = bufs[s];
        if (!b) continue;
        D3D11_BUFFER_DESC desc{}; b->GetDesc(&desc);
        UINT want = desc.ByteWidth < cbMaxBytes ? desc.ByteWidth : cbMaxBytes;
        auto *br = (CbBufRec *)(cbArena + at);
        br->slot = s; br->byteWidth = desc.ByteWidth; br->captured = 0; br->usage = (UINT32)desc.Usage; br->ptr = (UINT64)(ULONG_PTR)b;
        BYTE *dst = cbArena + at + sizeof *br;
        ID3D11Buffer *stage = desc.ByteWidth && desc.ByteWidth <= (1u << 20) ? cbStagingFor(device.Get(), desc.ByteWidth) : nullptr;
        if (stage) {
            c->CopyResource(stage, b);
            D3D11_MAPPED_SUBRESOURCE m{};
            if (SUCCEEDED(c->Map(stage, 0, D3D11_MAP_READ, 0, &m))) {
                memcpy(dst, m.pData, want); c->Unmap(stage, 0); br->captured = want;
                if (want < desc.ByteWidth) cbTruncated++;
            } else cbErrors++;
        } else cbErrors++;
        at += sizeof *br + ((br->captured + 7u) & ~7u);
        rec->ncb++;
        b->Release();
    }
    rec->size = (UINT32)(at - cbUsed);
    cbUsed = at; cbDraws++;
}

// Frame boundary, from dg_draw_trial_present (already under the life and
// multithread locks, on the Present thread - the thread the draws come from).
// The camera ring as it stood when the capture ENDED. Capture c (moving head)
// matched only 200 draws of its second frame because the ring was read up to a
// second later, at write time, when the cameras of the captured frames had
// long been overwritten (8 slots ~ 130 ms).
CbCamOut cbCamSnap[cbCamCount];
void cbSnapshotCams() {
    for (unsigned i = 0; i < cbCamCount; i++) {
        CbCamOut o{};
        for (int tries = 0; tries < 8; tries++) {            // seqlock read; a torn slot is kept as seq -1
            LONG a = InterlockedCompareExchange(&cbCams[i].seq, 0, 0);
            o.seq = a; o.eye = cbCams[i].eye; o.tick = cbCams[i].tick;
            memcpy(o.eye_pers, cbCams[i].m[0], 64); memcpy(o.pers, cbCams[i].m[1], 64);
            memcpy(o.eye_inv, cbCams[i].m[2], 64); memcpy(o.eye_world, cbCams[i].m[3], 64);
            if (!(a & 1) && a == InterlockedCompareExchange(&cbCams[i].seq, 0, 0)) break;
            o.seq = -1;
        }
        cbCamSnap[i] = o;
    }
}
void cbOnPresent() {
    LONG s = InterlockedCompareExchange(&cbState, 0, 0);
    if (s == 1) { cbFrame = 0; InterlockedExchange(&cbState, 2); }
    else if (s == 2 && ++cbFrame >= cbFrames) { cbSnapshotCams(); InterlockedExchange(&cbState, 3); }
}

bool cbWrite(const char *path) {
    FILE *f = nullptr;
    if (fopen_s(&f, path, "wb") || !f) return false;
    CbFileHeader h{};
    memcpy(h.magic, "DGCBP1\0\0", 8);
    h.version = 2; h.draws = cbDraws; h.frames = cbFrames; h.truncated = cbTruncated; h.errors = cbErrors;
    h.camCount = cbCamCount; h.camStride = sizeof(CbCamOut); h.arenaBytes = cbUsed; h.startTick = cbStartTick;
    bool ok = fwrite(&h, sizeof h, 1, f) == 1;
    for (unsigned i = 0; ok && i < cbCamCount; i++) ok = fwrite(&cbCamSnap[i], sizeof cbCamSnap[i], 1, f) == 1;
    if (ok && cbUsed) ok = fwrite(cbArena, 1, cbUsed, f) == cbUsed;
    fclose(f);
    return ok;
}

// Worker thread, once a second. `text` is the content of dg_draw_trial.on:
// "cb_capture <token>" arms one capture whenever the token is new.
void cbPoll(const char *markerPath, const char *text, size_t n) {
    if (InterlockedCompareExchange(&cbState, 0, 0) == 3) {
        char path[MAX_PATH];
        if (!strcpy_s(path, markerPath)) {
            char *leaf = strrchr(path, '\\');
            char name[64]; sprintf_s(name, "logs\\dg_cb_%llu.bin", (unsigned long long)GetTickCount64());
            if (leaf) strcpy_s(leaf + 1, sizeof(path) - (leaf + 1 - path), name); else strcpy_s(path, name);
            bool ok = cbWrite(path);
            // Version 2: the bytecode of every distinct vertex shader the capture saw, once, beside it.
            unsigned dumped = 0;
            for (size_t at = 0; at < cbUsed;) {
                auto *r = (CbDrawRec *)(cbArena + at); at += r->size;
                const BYTE *code = nullptr; UINT32 size = 0;
                if (!r->vsHash || !vsHashOf((void *)(ULONG_PTR)r->vs, &code, &size) || !code) continue;
                char vp[MAX_PATH]; strcpy_s(vp, markerPath); char *vl = strrchr(vp, '\\');
                char vn[64]; sprintf_s(vn, "logs\\dg_vs_%016llX.dxbc", (unsigned long long)r->vsHash);
                if (vl) strcpy_s(vl + 1, sizeof(vp) - (vl + 1 - vp), vn); else strcpy_s(vp, vn);
                if (GetFileAttributesA(vp) != INVALID_FILE_ATTRIBUTES) continue;
                FILE *vf = nullptr; if (!fopen_s(&vf, vp, "wb") && vf) { fwrite(code, 1, size, vf); fclose(vf); dumped++; }
            }
            if (logger) logger("cb_probe: vertex shaders dumped %u\r\n", dumped);
            if (logger) logger("cb_probe: captured frames=%u draws=%u bytes=%llu truncated=%u errors=%u staging_sizes=%u -> %s%s\r\n",
                               cbFrames, cbDraws, (unsigned long long)cbUsed, cbTruncated, cbErrors, cbStagingCount, path, ok ? "" : " (WRITE FAILED)");
        }
        InterlockedExchange(&cbState, 0);
    }
    if (n < 10 || memcmp(text, "cb_capture", 10)) return;
    char token[24] = ""; size_t k = 0;
    for (size_t i = 10; i < n && k + 1 < sizeof token; i++)
        if (text[i] > ' ') token[k++] = text[i];
    token[k] = 0;
    if (!strcmp(token, cbToken) && cbCaptures) return;
    if (InterlockedCompareExchange(&cbState, 0, 0) != 0) return;
    LifeLock life;
    if (!installed) { if (logger && strcmp(token, cbToken)) logger("cb_probe: refused=no_api_owner\r\n"); strcpy_s(cbToken, token); cbCaptures++; return; }
    if (cbCaptures >= cbMaxCaptures) return;
    if (!cbArena) cbArena = (BYTE *)VirtualAlloc(nullptr, cbArenaBytes, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (!cbArena) { if (logger) logger("cb_probe: refused=allocation\r\n"); strcpy_s(cbToken, token); cbCaptures++; return; }
    strcpy_s(cbToken, token); cbCaptures++;
    cbUsed = 0; cbDraws = 0; cbFrame = 0; cbTruncated = 0; cbErrors = 0; cbStartTick = GetTickCount64();
    InterlockedExchange(&cbState, 1);
    if (logger) logger("cb_probe: armed token=%s (capture %u of %u; read-only, expect a short hitch)\r\n", cbToken, cbCaptures, cbMaxCaptures);
}
