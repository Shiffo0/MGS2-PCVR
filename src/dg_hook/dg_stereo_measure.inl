#include <float.h>
#define SM_IMAGES 128
typedef struct {
    uint64_t qpc, ms, xr_frame, capture[2], capture_ms[2];
    int kind, eye, mono, has_raw, has_meta, has_fov, mark;
    int screen, anchor, gate, checked[2], valid[2], copied_mask;
    unsigned reason[2], idx[2];
    int layer, end_attempted, result;
    int64_t predicted;
    DG_STEREO_PRESENT present;
    DG_XR_RAW_POSE raw[2]; DG_PROJ_FOV fov[2];
    DG_NEAR_META meta;
    XrPosef submitted_pose[2]; XrFovf submitted_fov[2];
    XrPosef quad_pose; XrExtent2Df quad_size;
    int same_swapchain;
} SM_EVENT;
typedef struct {
    uint64_t frame, capture[2], qpc, read_qpc;
    unsigned w[2], h[2], fmt[2], width[2], height[2];
    int eyes, mark, ready, outcome;
    double copy_us, read_us;
} SM_IMAGE;
static struct {
    int initialized, enabled, active, finished, f8, f9, copied_mask, checked_mask;
    unsigned n, overflow, images, burst, mark, skipped_busy, skipped_budget;
    unsigned unsupported, predicated, gpu_failed, gpu_timeout, pending;
    uint64_t start_ms, next_image_ms, pending_ms, qpf, last_copy_frame;
    double max_read_us, max_copy_us;
    SM_EVENT *events; unsigned char *pixels;
    SM_IMAGE image[SM_IMAGES];
    DG_STEREO_PRESENT present;
    DG_XR_RAW_POSE copied_raw[2]; DG_PROJ_FOV copied_fov[2];
    uint64_t copied_id[2], copied_ms[2];
    ID3D11Texture2D *stage[2]; D3D11_TEXTURE2D_DESC desc[2]; ID3D11Query *query;
    int pending_index, pending_mask;
    HANDLE writer;
    char dir[MAX_PATH];
} g_sm;
#include "dg_stereo_path.inl"
int dg_xr_measure_control(char*dir,unsigned capacity,unsigned*mark){return 0;}
void dg_xr_measure_present(const DG_STEREO_PRESENT *p){}
static void sm_producer(int kind,int eye,const DG_XR_RAW_POSE *raw,
                        const DG_PROJ_FOV *fov,const DG_NEAR_META *meta,uint64_t id){}
static void sm_copy(int eye,int stereo,ID3D11Texture2D *dst){}
static void sm_tick(void){}
static void sm_submit(const XrFrameEndInfo *fei){}
static void sm_init(void){}
static void sm_teardown(void){}
static void sm_shutdown(void){}
