/* Runtime-only capture evidence, included after XR/capture globals.
 * Producer never logs; counters and last HRESULT are interlocked. XR emits
 * at most one three-line report per two seconds, outside the capture lock.
 * success means a copy was QUEUED, not GPU completion or headset visibility.
 */
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
static volatile LONG64 g_cd_producer_failure; /* reason:HRESULT, one atomic tuple */
/* Fields below belong only to the XR thread. */
static struct {
    ULONGLONG last_log, snapshot_ms;
    int logged, should_render, views_valid, published, screen, anchor;
    int gate; /* 1 render, 2 views, 3 swapchain, 4 published pose, 5 snapshot */
    unsigned reason[2], idx[2], images[2];
    int checked[2], store_valid[2], legacy_format[2];
    D3D11_TEXTURE2D_DESC src[2], dst[2];
    uint64_t capture_id[2], capture_ms[2];
    int64_t predicted_time, predicted_period;
    XrResult acquire[2], wait[2], release[2], end;
    int end_attempted;
    int attempted_acquire[2], attempted_wait[2], attempted_release[2];
} g_cd;

static void capture_diag_inc(int counter) {
    InterlockedIncrement64(&g_cd_count[counter]);
}
static void capture_diag_producer_fail(int reason, HRESULT hr) {
    capture_diag_inc(reason);
    InterlockedExchange64(&g_cd_producer_failure,
        (LONG64)(((uint64_t)(unsigned)reason << 32) | (uint32_t)hr));
}
static void capture_diag_begin(void) {
    /* Keep the last descriptor snapshot even if this frame never reaches
       copying. Its own timestamp prevents stale evidence posing as current. */
    g_cd.should_render = g_cd.views_valid = g_cd.published = 0;
    g_cd.screen = g_cd.anchor = g_cd.gate = 0;
    g_cd.predicted_time = g_cd.predicted_period = 0;
    g_cd.end = XR_SUCCESS; g_cd.end_attempted = 0;
    memset(g_cd.acquire, 0, sizeof(g_cd.acquire));
    memset(g_cd.wait, 0, sizeof(g_cd.wait));
    memset(g_cd.release, 0, sizeof(g_cd.release));
    memset(g_cd.attempted_acquire, 0, sizeof(g_cd.attempted_acquire));
    memset(g_cd.attempted_wait, 0, sizeof(g_cd.attempted_wait));
    memset(g_cd.attempted_release, 0, sizeof(g_cd.attempted_release));
}
static void capture_diag_snapshot_begin(void) {
    g_cd.snapshot_ms = GetTickCount64();
    memset(g_cd.src, 0, sizeof(g_cd.src)); memset(g_cd.dst, 0, sizeof(g_cd.dst));
    memset(g_cd.reason, 0, sizeof(g_cd.reason)); memset(g_cd.idx, 0, sizeof(g_cd.idx));
    memset(g_cd.images, 0, sizeof(g_cd.images));
    memset(g_cd.store_valid, 0, sizeof(g_cd.store_valid));
    memset(g_cd.checked, 0, sizeof(g_cd.checked));
    memset(g_cd.legacy_format, 0, sizeof(g_cd.legacy_format));
    memset(g_cd.capture_id, 0, sizeof(g_cd.capture_id));
    memset(g_cd.capture_ms, 0, sizeof(g_cd.capture_ms));
}
static void capture_diag_emit(void) {
    unsigned i;
    ULONGLONG now = GetTickCount64();
    LONG64 n[CD_COUNT];
    uint64_t producer_failure;
#if !DG_ENABLE_DIAGNOSTICS
    /* First five minutes, at most 31 snapshots / 93 lines per process.
       Keep capture failures observable without enabling GPU probes. */
    static unsigned support_reports;
    if (support_reports >= 31) return;
    if (g_cd.logged && now - g_cd.last_log < 10000) return;
    ++support_reports;
#else
    if (g_cd.logged && now - g_cd.last_log < 2000) return;
#endif
    g_cd.logged = 1; g_cd.last_log = now;
    for (i = 0; i < CD_COUNT; ++i)
        n[i] = InterlockedCompareExchange64(&g_cd_count[i], 0, 0);
    producer_failure = (uint64_t)InterlockedCompareExchange64(&g_cd_producer_failure, 0, 0);
    g_log("  OpenXR capture: tick %llu state %d producer attempt/queued %lld/%lld"
          " refuse input/fov/getbuf/create %lld/%lld/%lld/%lld last %ld hr 0x%08lX"
          " frames %lld gate %d render/views/pub/screen/anchor %d/%d/%d/%d/%d"
          " skip render/views/swap/pose %lld/%lld/%lld/%lld"
          " fail acquire/wait/release/snapshot %lld/%lld/%lld/%lld legacy-format %lld"
          " layer-build quad/projection/zero %lld/%lld/%lld end-errors %lld end %d:%d"
          " predicted %lld period %lld\r\n",
          (unsigned long long)now, (int)g_state, n[CD_ATTEMPT], n[CD_STORED],
          n[CD_INPUT], n[CD_NO_FOV], n[CD_GET_BUFFER], n[CD_CREATE],
          (long)(producer_failure >> 32), (unsigned long)(uint32_t)producer_failure,
          n[CD_FRAME], g_cd.gate, g_cd.should_render, g_cd.views_valid,
          g_cd.published, g_cd.screen, g_cd.anchor,
          n[CD_SKIP_RENDER], n[CD_SKIP_VIEWS], n[CD_SKIP_SWAP], n[CD_SKIP_POSE],
          n[CD_ACQUIRE], n[CD_WAIT], n[CD_RELEASE], n[CD_SNAPSHOT], n[CD_LEGACY_FORMAT],
          n[CD_QUAD], n[CD_PROJECTION], n[CD_ZERO], n[CD_END_ERROR],
          g_cd.end_attempted, (int)g_cd.end,
          (long long)g_cd.predicted_time, (long long)g_cd.predicted_period);
    /* Fixed two eyes, one compact line each. Zeros with no attempted acquire
       mean not reached this frame, not a successful API call. */
    for (i = 0; i < 2; ++i) {
        const D3D11_TEXTURE2D_DESC *s = &g_cd.src[i], *d = &g_cd.dst[i];
        g_log("  OpenXR capture eye %u: snapshot-ms %llu checked %d valid %d reason 0x%02X idx %u/%u"
              " calls a/w/r %d:%d/%d:%d/%d:%d legacy-format %d"
              " capture %llu age-ms %llu"
              " src %ux%u fmt %u mip/array/sample/quality %u/%u/%u/%u"
              " dst %ux%u fmt %u mip/array/sample/quality %u/%u/%u/%u\r\n",
              i, (unsigned long long)g_cd.snapshot_ms, g_cd.checked[i],
              g_cd.store_valid[i], g_cd.reason[i], g_cd.idx[i], g_cd.images[i],
              g_cd.attempted_acquire[i], (int)g_cd.acquire[i],
              g_cd.attempted_wait[i], (int)g_cd.wait[i],
              g_cd.attempted_release[i], (int)g_cd.release[i], g_cd.legacy_format[i],
              (unsigned long long)g_cd.capture_id[i],
              (unsigned long long)(g_cd.capture_id[i] ? now - g_cd.capture_ms[i] : 0),
              s->Width, s->Height, (unsigned)s->Format, s->MipLevels, s->ArraySize,
              s->SampleDesc.Count, s->SampleDesc.Quality,
              d->Width, d->Height, (unsigned)d->Format, d->MipLevels, d->ArraySize,
              d->SampleDesc.Count, d->SampleDesc.Quality);
    }

}
