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













// The draw path. kind 0 = DrawIndexed, 1 = Draw. Runs BEFORE the draw is
// forwarded, so the buffers hold what this draw will use.
void cbOnDraw(ID3D11DeviceContext *c, unsigned kind, UINT count, UINT first, INT base) {





























































}

// Frame boundary, from dg_draw_trial_present (already under the life and
// multithread locks, on the Present thread - the thread the draws come from).
// The camera ring as it stood when the capture ENDED. Capture c (moving head)
// matched only 200 draws of its second frame because the ring was read up to a
// second later, at write time, when the cameras of the captured frames had
// long been overwritten (8 slots ~ 130 ms).
CbCamOut cbCamSnap[cbCamCount];








































// Worker thread, once a second. `text` is the content of dg_draw_trial.on:
// "cb_capture <token>" arms one capture whenever the token is new.

















































