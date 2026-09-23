

#ifndef NP_ROOT
#define NP_ROOT "logs\\pcvr_near"
#endif
#include <float.h>
#pragma warning(push)
#pragma warning(disable:4996)
static struct {
    unsigned token, last_token, mask, polls;
    ULONGLONG next_poll, start_ms;
    ID3D11Texture2D *texture[2]; ID3D11Query *query;
    D3D11_TEXTURE2D_DESC desc[2];
    DG_NEAR_META meta[2]; DG_XR_RAW_POSE raw[2]; DG_PROJ_FOV fov[2];
    uint64_t capture_id[2], qpc[2];
} g_np;
static void near_probe_free(void) {









}

















































static void near_probe_capture(ID3D11Texture2D *back,int eye,int mono,const DG_XR_RAW_POSE *raw,const DG_PROJ_FOV *fov,const DG_NEAR_META *meta,uint64_t id){

































}
#pragma warning(pop)
