enum {
    CD_ATTEMPT, CD_STORED, CD_INPUT, CD_NO_FOV, CD_GET_BUFFER, CD_CREATE,
    CD_FRAME, CD_SKIP_RENDER, CD_SKIP_VIEWS, CD_SKIP_SWAP, CD_SKIP_POSE,
    CD_ACQUIRE, CD_WAIT, CD_RELEASE, CD_SNAPSHOT, CD_LEGACY_FORMAT,
    CD_QUAD, CD_PROJECTION, CD_ZERO, CD_END_ERROR, CD_COUNT
};
enum {
    CR_STORE = 1, CR_INDEX = 2, CR_POSE = 4, CR_SIZE = 8,
    CR_MIPS = 16, CR_ARRAY = 32, CR_SAMPLE = 64, CR_FORMAT = 128
};
static volatile LONG64 g_cd_count[CD_COUNT];
static volatile LONG64 g_cd_producer_failure;
static struct {
    ULONGLONG last_log, snapshot_ms;
    int logged, should_render, views_valid, published, screen, anchor;
    int gate;
    unsigned reason[2], idx[2], images[2];
    int checked[2], store_valid[2], legacy_format[2];
    D3D11_TEXTURE2D_DESC src[2], dst[2];
    uint64_t capture_id[2], capture_ms[2];
    int64_t predicted_time, predicted_period;
    XrResult acquire[2], wait[2], release[2], end;
    int end_attempted;
    int attempted_acquire[2], attempted_wait[2], attempted_release[2];
} g_cd;
static void capture_diag_inc(int counter){}
static void capture_diag_producer_fail(int reason, HRESULT hr){}
static void capture_diag_begin(void){}
static void capture_diag_snapshot_begin(void){}
static void capture_diag_emit(void){}
