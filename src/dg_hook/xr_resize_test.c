/* Production swapchain lifecycle with real WARP textures and scripted OpenXR. */
#include <stdarg.h>
#include "dg_xr.c"
int dg_bridge_model_arm_snapshot(DG_MODEL_ARM *p){memset(p,0,sizeof *p);return 0;}
int dg_bridge_grip_debug_snapshot(DG_GRIP_DEBUG *p){memset(p,0,sizeof *p);return 0;}
int dg_bridge_health_snapshot(dg_health_view *p){memset(p,0,sizeof *p);return 0;}
static int checks, failures, creates, destroys, inject_resize, fail_create_at, releases;
static ID3D11Texture2D *runtime_tex[2];
static unsigned support_lines;
#define CHECK(x) do {checks++;if(!(x)){failures++;printf("FAIL line %d: %s\n",__LINE__,#x);}}while(0)
static void logger(const char *fmt,...){if(strstr(fmt,"OpenXR capture"))support_lines++;}
static ID3D11Texture2D *tex(unsigned w,unsigned h){
    D3D11_TEXTURE2D_DESC d;ID3D11Texture2D *t=NULL;
    memset(&d,0,sizeof d);d.Width=w;d.Height=h;d.MipLevels=d.ArraySize=1;
    d.Format=DXGI_FORMAT_B8G8R8A8_UNORM;d.SampleDesc.Count=1;
    CHECK(SUCCEEDED(g_dev->lpVtbl->CreateTexture2D(g_dev,&d,NULL,&t)));return t;
}
static XrResult XRAPI_PTR formats(XrSession s,uint32_t cap,uint32_t *n,int64_t *out){
    (void)s;*n=1;if(cap)out[0]=DXGI_FORMAT_B8G8R8A8_UNORM;return XR_SUCCESS;
}
static XrResult XRAPI_PTR create(XrSession s,const XrSwapchainCreateInfo *ci,XrSwapchain *out){
    int e=runtime_tex[0]?1:0;(void)s;creates++;
    if(creates==fail_create_at)return XR_ERROR_RUNTIME_FAILURE;
    runtime_tex[e]=tex(ci->width,ci->height);*out=(XrSwapchain)(uintptr_t)(e+1);
    if(inject_resize){EnterCriticalSection(&g_cap_cs);g_sw=12;g_sh=6;LeaveCriticalSection(&g_cap_cs);inject_resize=0;}
    return XR_SUCCESS;
}
static XrResult XRAPI_PTR destroy(XrSwapchain sc){
    int e=(int)(uintptr_t)sc-1;destroys++;runtime_tex[e]->lpVtbl->Release(runtime_tex[e]);runtime_tex[e]=NULL;return XR_SUCCESS;
}
static XrResult XRAPI_PTR images(XrSwapchain sc,uint32_t cap,uint32_t *n,XrSwapchainImageBaseHeader *out){
    *n=1;if(cap)((XrSwapchainImageD3D11KHR*)out)->texture=runtime_tex[(int)(uintptr_t)sc-1];return XR_SUCCESS;
}
static XrResult XRAPI_PTR acquire(XrSwapchain s,const XrSwapchainImageAcquireInfo *i,uint32_t *idx){(void)s;(void)i;*idx=0;return XR_SUCCESS;}
static XrResult XRAPI_PTR waitimg(XrSwapchain s,const XrSwapchainImageWaitInfo *i){(void)s;(void)i;return XR_SUCCESS;}
static XrResult XRAPI_PTR release(XrSwapchain s,const XrSwapchainImageReleaseInfo *i){(void)s;(void)i;releases++;return XR_SUCCESS;}
static void stores(unsigned w,unsigned h){
    int e;EnterCriticalSection(&g_cap_cs);g_sw=w;g_sh=h;
    for(e=0;e<2;e++){
        if(g_store[e].texture)g_store[e].texture->lpVtbl->Release(g_store[e].texture);
        g_store[e].texture=tex(w,h);g_store[e].valid=1;g_store[e].raw.qw=1;
    }LeaveCriticalSection(&g_cap_cs);
}
static void copy_check(void){
    DG_SUBMIT_CAPTURE cap;const char *fatal;memset(&cap,0,sizeof cap);cap.stereo=1;
    CHECK(submit_capture_images(&cap,&fatal));CHECK(!fatal);
    CHECK(cap.width==g_swap_w && cap.height==g_swap_h);
}
int main(void){
    int e;HRESULT hr;
    SetErrorMode(SEM_FAILCRITICALERRORS|SEM_NOGPFAULTERRORBOX);
    g_log=logger;InitializeCriticalSection(&g_cap_cs);g_cap_cs_ready=1;
    hr=D3D11CreateDevice(NULL,D3D_DRIVER_TYPE_WARP,NULL,0,NULL,0,D3D11_SDK_VERSION,&g_dev,NULL,&g_ctx);
    if(FAILED(hr)){printf("FAIL WARP %lx\n",(unsigned long)hr);return 1;}
    g_adopted=g_dev;pfn_EnumScFormats=formats;pfn_CreateSwapchain=create;
    pfn_DestroySwapchain=destroy;pfn_EnumScImages=images;
    pfn_AcquireImage=acquire;pfn_WaitImage=waitimg;pfn_ReleaseImage=release;
    stores(4,4);CHECK(ensure_swapchains());copy_check();
    CHECK(ensure_swapchains() && creates==2 && destroys==0);
    stores(8,4);CHECK(ensure_swapchains());CHECK(creates==4 && destroys==2);copy_check();
    stores(4,4);inject_resize=1;CHECK(ensure_swapchains());
    CHECK(g_swap_w==4 && g_swap_h==4 && g_sw==12 && g_sh==6);
    stores(12,6);CHECK(ensure_swapchains());CHECK(g_swap_w==12 && g_swap_h==6);copy_check();
    stores(16,8);fail_create_at=creates+2;CHECK(!ensure_swapchains());
    CHECK(!g_swap[0] && !g_swap[1] && !g_images[0] && !g_images[1]);
    fail_create_at=0;CHECK(ensure_swapchains());copy_check();CHECK(releases==8);
    capture_diag_emit();CHECK(support_lines==3);
    capture_diag_emit();CHECK(support_lines==3);
    for(e=0;e<40;e++){g_cd.logged=0;capture_diag_emit();}
    CHECK(support_lines==93);
    destroy_projection_swapchains();
    for(e=0;e<2;e++)g_store[e].texture->lpVtbl->Release(g_store[e].texture);
    g_ctx->lpVtbl->Release(g_ctx);g_dev->lpVtbl->Release(g_dev);DeleteCriticalSection(&g_cap_cs);
    printf("%s resize: %d checks, %d failures\n",failures?"FAIL":"PASS",checks,failures);return failures?1:0;
}
