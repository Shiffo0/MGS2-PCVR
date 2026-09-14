/* XR-thread-owned radial resources. Producer only publishes copied CPU views;
 * no XR/D3D call, allocation, wait or capture lock on the producer path. */
#include "dg_radial_overlay.h"
static SRWLOCK g_radial_mail_lock = SRWLOCK_INIT;
static dg_radial_overlay_model g_radial_mail;
static volatile LONG64 g_radial_generation = 1;
static volatile LONG64 g_radial_ready_generation;
static volatile LONG64 g_radial_ready_ms;
static volatile LONG64 g_radial_publish_seq; /* monotonic, never reset */
static volatile LONG64 g_radial_hidden_through;
static uint64_t g_radial_session_epoch, g_radial_frame_seq;
static unsigned g_radial_max_layers;
static int g_radial_quarantined;
static dg_radial_overlay g_radial_overlay;
static XrSwapchain g_radial_swap;
static XrSwapchainImageD3D11KHR *g_radial_images;
static unsigned g_radial_image_count;
static uint32_t g_radial_pixels[DG_RADIAL_VIEW_PIXELS];

void dg_xr_radial_invalidate(void) {
    InterlockedExchange64(&g_radial_ready_generation, 0);
    InterlockedIncrement64(&g_radial_generation);
}
void dg_xr_radial_hide(void) {
    LONG64 seq=InterlockedCompareExchange64(&g_radial_publish_seq,0,0);
    LONG64 old=InterlockedCompareExchange64(&g_radial_hidden_through,0,0);
    while (old<seq) {
        LONG64 observed=InterlockedCompareExchange64(&g_radial_hidden_through,seq,old);
        if (observed==old) break;
        old=observed;
    }
}
uint64_t dg_xr_radial_generation(void) {
    uint64_t ready = (uint64_t)InterlockedCompareExchange64(&g_radial_ready_generation,0,0);
    uint64_t stamp = (uint64_t)InterlockedCompareExchange64(&g_radial_ready_ms,0,0);
    uint64_t now = GetTickCount64();
    if (!ready || now < stamp || now-stamp > 100 ||
        ready != (uint64_t)InterlockedCompareExchange64(&g_radial_generation,0,0)) return 0;
    return ready;
}
int dg_xr_radial_publish(const dg_radial_view *view, uint64_t generation,
                         uint64_t context, uint64_t catalog) {
    int accepted = 0;
    if (!view || !generation || !context || !catalog ||
        dg_xr_radial_generation()!=generation ||
        !TryAcquireSRWLockExclusive(&g_radial_mail_lock)) return 0;
    if (dg_xr_radial_generation()==generation) {
        g_radial_mail.view=*view;
        /* Session is stamped by the XR consumer; generation fences session,
         * reference and focus transitions without reading XR-owned state. */
        g_radial_mail.reference_epoch=generation;
        g_radial_mail.context_epoch=context;
        g_radial_mail.catalog_version=catalog;
        g_radial_mail.sequence=(uint64_t)InterlockedIncrement64(&g_radial_publish_seq);
        g_radial_mail.sample_ms=GetTickCount64();
        accepted=1;
    }
    ReleaseSRWLockExclusive(&g_radial_mail_lock);
    return accepted;
}
static int radial_io(const char *op, XrResult r) {
    if (r==XR_SUCCESS) return DG_RADIAL_IO_OK;
    if (g_log) g_log("  xr radial: %s failed (%d); overlay disabled for this session\r\n",op,(int)r);
    return DG_RADIAL_IO_ERROR;
}
static int radial_destroy(void *unused, uint64_t handle) {
    XrResult r; (void)unused;
    if (!g_radial_swap || handle!=(uint64_t)(uintptr_t)g_radial_swap || !pfn_DestroySwapchain)
        return DG_RADIAL_IO_ERROR;
    r=pfn_DestroySwapchain(g_radial_swap);
    if (radial_io("destroy",r)!=DG_RADIAL_IO_OK) return DG_RADIAL_IO_ERROR;
    g_radial_swap=XR_NULL_HANDLE;
    free(g_radial_images); g_radial_images=NULL; g_radial_image_count=0;
    return DG_RADIAL_IO_OK;
}
static int radial_create(void *unused, unsigned width, unsigned height,
                          uint64_t *handle, unsigned *images) {
    int64_t *formats=NULL, want=0;
    uint32_t count=0, got=0, i;
    XrSwapchainCreateInfo ci;
    XrResult r;
    (void)unused; *handle=0; *images=0;
    if (!g_sess || !g_dev || !g_ctx || g_radial_swap || g_radial_quarantined ||
        width!=DG_RADIAL_VIEW_SIZE || height!=DG_RADIAL_VIEW_SIZE ||
        !pfn_EnumScFormats || !pfn_CreateSwapchain || !pfn_EnumScImages || !pfn_DestroySwapchain)
        return DG_RADIAL_IO_ERROR;
    r=pfn_EnumScFormats(g_sess,0,&count,NULL);
    if (radial_io("format count",r)!=DG_RADIAL_IO_OK || !count || count>4096) return DG_RADIAL_IO_ERROR;
    formats=(int64_t *)calloc(count,sizeof(*formats));
    if (!formats) return DG_RADIAL_IO_ERROR;
    r=pfn_EnumScFormats(g_sess,count,&got,formats);
    if (radial_io("formats",r)==DG_RADIAL_IO_OK && got<=count) {
        for(i=0;i<got;i++) if(formats[i]==DXGI_FORMAT_R8G8B8A8_UNORM) want=formats[i];
        for(i=0;i<got;i++) if(formats[i]==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) want=formats[i];
    }
    free(formats);
    if (!want) return DG_RADIAL_IO_ERROR;
    memset(&ci,0,sizeof(ci)); ci.type=XR_TYPE_SWAPCHAIN_CREATE_INFO;
    ci.usageFlags=XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
    ci.format=want; ci.sampleCount=1; ci.width=width; ci.height=height;
    ci.faceCount=ci.arraySize=ci.mipCount=1;
    r=pfn_CreateSwapchain(g_sess,&ci,&g_radial_swap);
    if (radial_io("create",r)!=DG_RADIAL_IO_OK || !g_radial_swap) goto fail;
    r=pfn_EnumScImages(g_radial_swap,0,&count,NULL);
    if (radial_io("image count",r)!=DG_RADIAL_IO_OK || !count || count>256) goto fail;
    g_radial_images=(XrSwapchainImageD3D11KHR *)calloc(count,sizeof(*g_radial_images));
    if (!g_radial_images) goto fail;
    for(i=0;i<count;i++) g_radial_images[i].type=XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR;
    r=pfn_EnumScImages(g_radial_swap,count,&got,(XrSwapchainImageBaseHeader *)g_radial_images);
    if (radial_io("images",r)!=DG_RADIAL_IO_OK || !got || got>count) goto fail;
    g_radial_image_count=got;
    *handle=(uint64_t)(uintptr_t)g_radial_swap; *images=got;
    return DG_RADIAL_IO_OK;
fail:
    /* If rollback fails retain the orphan separately for parent teardown;
     * never publish a partially enumerated handle to the helper. */
    if (g_radial_swap) radial_destroy(NULL,(uint64_t)(uintptr_t)g_radial_swap);
    return DG_RADIAL_IO_ERROR;
}
static int radial_acquire(void *unused, uint64_t handle, unsigned *image) {
    XrSwapchainImageAcquireInfo ai={XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_radial_swap || !pfn_AcquireImage) return DG_RADIAL_IO_ERROR;
    return radial_io("acquire",pfn_AcquireImage(g_radial_swap,&ai,image));
}
static int radial_wait(void *unused, uint64_t handle) {
    XrSwapchainImageWaitInfo wi={XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    XrResult r; (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_radial_swap || !pfn_WaitImage) return DG_RADIAL_IO_ERROR;
    wi.timeout=0; r=pfn_WaitImage(g_radial_swap,&wi);
    if (r==XR_TIMEOUT_EXPIRED) return DG_RADIAL_IO_TIMEOUT;
    return radial_io("wait",r);
}
static int radial_upload(void *unused, uint64_t handle, unsigned image,
                          const void *rgba, size_t bytes) {
    D3D11_TEXTURE2D_DESC d;
    ID3D11Texture2D *t;
    ID3D11Device *owner=NULL;
    int valid;
    (void)unused;
    if (!rgba || bytes!=DG_RADIAL_VIEW_PIXELS*4u || !g_dev || !g_ctx || !g_cap_cs_ready ||
        handle!=(uint64_t)(uintptr_t)g_radial_swap || image>=g_radial_image_count ||
        !g_radial_images || !(t=g_radial_images[image].texture)) return DG_RADIAL_IO_ERROR;
    t->lpVtbl->GetDesc(t,&d); t->lpVtbl->GetDevice(t,&owner);
    valid=owner==g_dev && d.Width==DG_RADIAL_VIEW_SIZE && d.Height==DG_RADIAL_VIEW_SIZE &&
        d.ArraySize==1 && d.MipLevels==1 && d.SampleDesc.Count==1 &&
        d.Usage==D3D11_USAGE_DEFAULT &&
        (d.Format==DXGI_FORMAT_R8G8B8A8_UNORM || d.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ||
         d.Format==DXGI_FORMAT_R8G8B8A8_TYPELESS);
    if(owner) owner->lpVtbl->Release(owner);
    if (!valid || FAILED(g_dev->lpVtbl->GetDeviceRemovedReason(g_dev))) return DG_RADIAL_IO_ERROR;
    /* Only the D3D call is under the capture lock; XR waits never are.
     * UpdateSubresource consumes CPU bytes before return, no retained scratch. */
    EnterCriticalSection(&g_cap_cs);
    g_ctx->lpVtbl->UpdateSubresource(g_ctx,(ID3D11Resource *)t,0,NULL,rgba,DG_RADIAL_VIEW_SIZE*4u,0);
    LeaveCriticalSection(&g_cap_cs);
    return SUCCEEDED(g_dev->lpVtbl->GetDeviceRemovedReason(g_dev)) ? DG_RADIAL_IO_OK : DG_RADIAL_IO_ERROR;
}
static int radial_release(void *unused, uint64_t handle) {
    XrSwapchainImageReleaseInfo ri={XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_radial_swap || !pfn_ReleaseImage) return DG_RADIAL_IO_ERROR;
    return radial_io("release",pfn_ReleaseImage(g_radial_swap,&ri));
}
static const dg_radial_overlay_ops g_radial_ops={NULL,radial_create,radial_acquire,
    radial_wait,radial_upload,radial_release,radial_destroy};
static void radial_new_session(void) {
    dg_xr_radial_invalidate();
    if (g_radial_quarantined || g_radial_swap) return;
    dg_radial_overlay_init(&g_radial_overlay,++g_radial_session_epoch);
    g_radial_frame_seq=0;
}
static void radial_stop(int stopped) {
    dg_xr_radial_invalidate();
    if (!stopped || g_radial_quarantined) return;
    if (g_radial_overlay.handle) dg_radial_overlay_teardown(&g_radial_overlay,&g_radial_ops,1);
    else if (g_radial_swap) radial_destroy(NULL,(uint64_t)(uintptr_t)g_radial_swap);
}
static void radial_parent_destroyed(int destroyed) {
    if (!destroyed) { g_radial_quarantined=1; return; }
    /* Successful xrDestroySession implicitly destroys remaining child handles,
     * including a failed-create orphan or an acquired image after session loss. */
    g_radial_swap=XR_NULL_HANDLE;
    free(g_radial_images); g_radial_images=NULL; g_radial_image_count=0;
    memset(&g_radial_overlay,0,sizeof(g_radial_overlay));
}
static void radial_append(XrFrameEndInfo *end, const XrCompositionLayerBaseHeader **layers,
                           unsigned capacity, XrCompositionLayerQuad *quad,
                           int should_render, int theater, uint64_t now) {
    dg_radial_overlay_frame f;
    dg_radial_overlay_model m;
    dg_radial_overlay_layer layer;
    int ready;
    uint64_t generation=(uint64_t)InterlockedCompareExchange64(&g_radial_generation,0,0);
    if (g_radial_quarantined) { dg_xr_radial_invalidate(); return; }
    memset(&f,0,sizeof(f)); memset(&m,0,sizeof(m));
    f.session_epoch=g_radial_session_epoch; f.sequence=++g_radial_frame_seq; f.now_ms=now;
    f.running=g_session_running; f.focused=g_state==XR_SESSION_STATE_FOCUSED;
    f.should_render=should_render; f.theater_active=theater;
    f.base_layers=end->layerCount; f.max_layers=g_radial_max_layers<capacity?g_radial_max_layers:capacity;
    f.projection_ready=end->layerCount==1 && end->layers &&
        end->layers[0]->type==XR_TYPE_COMPOSITION_LAYER_PROJECTION;
    ready=f.running && f.focused && f.should_render && !theater && f.projection_ready &&
        f.max_layers>f.base_layers && g_view_space && !g_radial_overlay.fault &&
        g_radial_overlay.session_epoch && !g_radial_quarantined;
    if (!ready) {
        if (InterlockedCompareExchange64(&g_radial_ready_generation,0,0)) dg_xr_radial_invalidate();
        generation=(uint64_t)InterlockedCompareExchange64(&g_radial_generation,0,0);
    } else {
        InterlockedExchange64(&g_radial_ready_ms,(LONG64)now);
        InterlockedExchange64(&g_radial_ready_generation,(LONG64)generation);
    }
    if (TryAcquireSRWLockShared(&g_radial_mail_lock)) {
        m=g_radial_mail; ReleaseSRWLockShared(&g_radial_mail_lock);
    }
    m.session_epoch=f.session_epoch;
    if (m.sequence<=(uint64_t)InterlockedCompareExchange64(&g_radial_hidden_through,0,0))
        m.view.visible=0;
    f.context_epoch=m.context_epoch; f.catalog_version=m.catalog_version;
    f.reference_epoch=generation;
    if (!ready || generation!=(uint64_t)InterlockedCompareExchange64(&g_radial_generation,0,0)) f.focused=0;
    layer=dg_radial_overlay_step(&g_radial_overlay,&g_radial_ops,&f,&m,
                                g_radial_pixels,DG_RADIAL_VIEW_PIXELS);
    if (g_radial_overlay.fault) dg_xr_radial_invalidate();
    if (!layer.submit || generation!=(uint64_t)InterlockedCompareExchange64(&g_radial_generation,0,0) ||
        m.sequence<=(uint64_t)InterlockedCompareExchange64(&g_radial_hidden_through,0,0)) return;
    memset(quad,0,sizeof(*quad)); quad->type=XR_TYPE_COMPOSITION_LAYER_QUAD;
    quad->layerFlags=XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
                     XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
    quad->space=g_view_space; quad->eyeVisibility=XR_EYE_VISIBILITY_BOTH;
    quad->subImage.swapchain=(XrSwapchain)(uintptr_t)layer.handle;
    quad->subImage.imageRect.extent.width=(int32_t)layer.rect_w;
    quad->subImage.imageRect.extent.height=(int32_t)layer.rect_h;
    quad->pose.orientation.w=layer.orientation_w; quad->pose.position.z=layer.position_z;
    quad->size.width=layer.width_m; quad->size.height=layer.height_m;
    layers[end->layerCount++]=(const XrCompositionLayerBaseHeader *)quad;
    end->layers=layers;
}
