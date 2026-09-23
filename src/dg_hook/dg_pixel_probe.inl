/* Opt-in diagnostic, not a rendering fix. All mutable state is under g_cap_cs.
 * Three attempts per process, one in flight, only armed after a focused quad.
 * Private staging copies perturb GPU scheduling. No Flush, wait loop, blocking
 * Map, or access to an XR image after release. A pending pre-release result is
 * evidence, not permission to force completion. Late reads use staging only.
 */
enum { PP_IDLE, PP_ARMED, PP_SOURCE, PP_GPU, PP_DONE };
typedef struct {
    unsigned count, nonblack, lo[4], hi[4];
    uint64_t hash;
    int valid;
} DG_PIXEL_STATS;
static struct {
    int enabled, phase, before_attempted, before_ready, read_before;
    unsigned attempts, width, height, format, actual_format[3], polls;
    ULONGLONG next_ms, arm_ms, capture_ms, submit_ms, read_ms;
    uint64_t capture_id, frame;
    ID3D11Texture2D *stage[3];
    ID3D11Query *event;
    HRESULT hr, before_hr, map_hr[3];
    DG_PIXEL_STATS stats[3];
    const char *outcome;
    double producer_us, submit_us, read_us, pose_us;
    int quad_valid, release_attempted, end_attempted;
    XrResult head_result, release_result, end_result;
    XrSpaceLocation head;
    XrCompositionLayerQuad quad;
    XrTime display_time;
} g_pp;
static int g_pp_initialized;























/* RGB/A bytes, no gamma conversion. Hash and nonblack describe only the same
 * deterministic <=64x36 grid in each full-size staging texture, NOT all pixels.
 * RowPitch padding is excluded. Coordinate formula includes all four edges. */




















/* Called only for an armed mono capture, inside the producer's capture lock.
 * A later producer overwrite is never silently relabelled as this sample. */
static void pixel_probe_source(ID3D11Texture2D *back, int mono) {





































}
/* Called after the normal destination copy, still in the snapshot lock. */
static void pixel_probe_destination(ID3D11Texture2D *dst, int eye) {





















}
/* One query per call, never spin/flush. Caller holds g_cap_cs. */

































static void pixel_probe_before_release(void) {














}
/* XR thread only; retain the ACTUAL built quad and a head location at the same
 * predicted time. This happens before EndFrame; no anchor is changed. */
static void pixel_probe_quad(const XrCompositionLayerQuad *quad) {
























}






























































/* EndFrame result is attached before servicing or arming the next sample. */
static void pixel_probe_tick(int arm, int end_attempted, XrResult end_result) {






























}
static void pixel_probe_teardown(void) {

















}
static void pixel_probe_init(void) {


















}
