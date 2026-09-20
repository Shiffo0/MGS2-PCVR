/* XR-thread-owned LIFE resources. Same format, upload and bounded-wait
 * contract as radial; independent handle so neither HUD overwrites the other. */
#include "dg_grip_debug.h"
static XrSwapchain g_gripdbg_gpu_swap;
static XrSwapchainImageD3D11KHR *g_gripdbg_gpu_images;
static unsigned g_gripdbg_gpu_image_count;
static int g_gripdbg_gpu_quarantined;
static dg_radial_overlay g_gripdbg_overlay;
static uint32_t g_gripdbg_pixels[DG_GRIPDBG_PIXELS];
static int g_gripdbg_wrist_held, g_gripdbg_logged;
static int gripdbg_gpu_io(const char *op, XrResult r) {
    if (r==XR_SUCCESS) return DG_RADIAL_IO_OK;
    if (g_log) g_log("  xr grip debug: %s failed (%d); overlay disabled for this session\r\n",op,(int)r);
    return DG_RADIAL_IO_ERROR;
}
static int gripdbg_gpu_destroy(void *unused, uint64_t handle) {
    XrResult r; (void)unused;
    if (!g_gripdbg_gpu_swap || handle!=(uint64_t)(uintptr_t)g_gripdbg_gpu_swap || !pfn_DestroySwapchain)
        return DG_RADIAL_IO_ERROR;
    r=pfn_DestroySwapchain(g_gripdbg_gpu_swap);
    if (gripdbg_gpu_io("destroy",r)!=DG_RADIAL_IO_OK) return DG_RADIAL_IO_ERROR;
    g_gripdbg_gpu_swap=XR_NULL_HANDLE;
    free(g_gripdbg_gpu_images); g_gripdbg_gpu_images=NULL; g_gripdbg_gpu_image_count=0;
    return DG_RADIAL_IO_OK;
}
static int gripdbg_gpu_create(void *unused, unsigned width, unsigned height,
                          uint64_t *handle, unsigned *images) {
    int64_t *formats=NULL, want=0;
    uint32_t count=0, got=0, i;
    XrSwapchainCreateInfo ci;
    XrResult r;
    (void)unused; *handle=0; *images=0;
    if (!g_sess || !g_dev || !g_ctx || g_gripdbg_gpu_swap || g_gripdbg_gpu_quarantined ||
        width!=DG_GRIPDBG_SIZE || height!=DG_GRIPDBG_SIZE ||
        !pfn_EnumScFormats || !pfn_CreateSwapchain || !pfn_EnumScImages || !pfn_DestroySwapchain)
        return DG_RADIAL_IO_ERROR;
    r=pfn_EnumScFormats(g_sess,0,&count,NULL);
    if (gripdbg_gpu_io("format count",r)!=DG_RADIAL_IO_OK || !count || count>4096) return DG_RADIAL_IO_ERROR;
    formats=(int64_t *)calloc(count,sizeof(*formats));
    if (!formats) return DG_RADIAL_IO_ERROR;
    r=pfn_EnumScFormats(g_sess,count,&got,formats);
    if (gripdbg_gpu_io("formats",r)==DG_RADIAL_IO_OK && got<=count) {
        for(i=0;i<got;i++) if(formats[i]==DXGI_FORMAT_R8G8B8A8_UNORM) want=formats[i];
        for(i=0;i<got;i++) if(formats[i]==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) want=formats[i];
    }
    free(formats);
    if (!want) return DG_RADIAL_IO_ERROR;
    memset(&ci,0,sizeof(ci)); ci.type=XR_TYPE_SWAPCHAIN_CREATE_INFO;
    ci.usageFlags=XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
    ci.format=want; ci.sampleCount=1; ci.width=width; ci.height=height;
    ci.faceCount=ci.arraySize=ci.mipCount=1;
    r=pfn_CreateSwapchain(g_sess,&ci,&g_gripdbg_gpu_swap);
    if (gripdbg_gpu_io("create",r)!=DG_RADIAL_IO_OK || !g_gripdbg_gpu_swap) goto fail;
    r=pfn_EnumScImages(g_gripdbg_gpu_swap,0,&count,NULL);
    if (gripdbg_gpu_io("image count",r)!=DG_RADIAL_IO_OK || !count || count>256) goto fail;
    g_gripdbg_gpu_images=(XrSwapchainImageD3D11KHR *)calloc(count,sizeof(*g_gripdbg_gpu_images));
    if (!g_gripdbg_gpu_images) goto fail;
    for(i=0;i<count;i++) g_gripdbg_gpu_images[i].type=XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR;
    r=pfn_EnumScImages(g_gripdbg_gpu_swap,count,&got,(XrSwapchainImageBaseHeader *)g_gripdbg_gpu_images);
    if (gripdbg_gpu_io("images",r)!=DG_RADIAL_IO_OK || !got || got>count) goto fail;
    g_gripdbg_gpu_image_count=got;
    *handle=(uint64_t)(uintptr_t)g_gripdbg_gpu_swap; *images=got;
    return DG_RADIAL_IO_OK;
fail:
    /* If rollback fails retain the orphan separately for parent teardown;
     * never publish a partially enumerated handle to the helper. */
    if (g_gripdbg_gpu_swap) gripdbg_gpu_destroy(NULL,(uint64_t)(uintptr_t)g_gripdbg_gpu_swap);
    return DG_RADIAL_IO_ERROR;
}
static int gripdbg_gpu_acquire(void *unused, uint64_t handle, unsigned *image) {
    XrSwapchainImageAcquireInfo ai={XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_gripdbg_gpu_swap || !pfn_AcquireImage) return DG_RADIAL_IO_ERROR;
    return gripdbg_gpu_io("acquire",pfn_AcquireImage(g_gripdbg_gpu_swap,&ai,image));
}
static int gripdbg_gpu_wait(void *unused, uint64_t handle) {
    XrSwapchainImageWaitInfo wi={XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    XrResult r; (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_gripdbg_gpu_swap || !pfn_WaitImage) return DG_RADIAL_IO_ERROR;
    wi.timeout=0; r=pfn_WaitImage(g_gripdbg_gpu_swap,&wi);
    if (r==XR_TIMEOUT_EXPIRED) return DG_RADIAL_IO_TIMEOUT;
    return gripdbg_gpu_io("wait",r);
}
static int gripdbg_gpu_upload(void *unused, uint64_t handle, unsigned image,
                          const void *rgba, size_t bytes) {
    D3D11_TEXTURE2D_DESC d;
    ID3D11Texture2D *t;
    ID3D11Device *owner=NULL;
    int valid;
    (void)unused;
    if (!rgba || bytes!=DG_GRIPDBG_PIXELS*4u || !g_dev || !g_ctx || !g_cap_cs_ready ||
        handle!=(uint64_t)(uintptr_t)g_gripdbg_gpu_swap || image>=g_gripdbg_gpu_image_count ||
        !g_gripdbg_gpu_images || !(t=g_gripdbg_gpu_images[image].texture)) return DG_RADIAL_IO_ERROR;
    t->lpVtbl->GetDesc(t,&d); t->lpVtbl->GetDevice(t,&owner);
    valid=owner==g_dev && d.Width==DG_GRIPDBG_SIZE && d.Height==DG_GRIPDBG_SIZE &&
        d.ArraySize==1 && d.MipLevels==1 && d.SampleDesc.Count==1 &&
        d.Usage==D3D11_USAGE_DEFAULT &&
        (d.Format==DXGI_FORMAT_R8G8B8A8_UNORM || d.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ||
         d.Format==DXGI_FORMAT_R8G8B8A8_TYPELESS);
    if(owner) owner->lpVtbl->Release(owner);
    if (!valid || FAILED(g_dev->lpVtbl->GetDeviceRemovedReason(g_dev))) return DG_RADIAL_IO_ERROR;
    /* Only the D3D call is under the capture lock; XR waits never are.
     * UpdateSubresource consumes CPU bytes before return, no retained scratch. */
    EnterCriticalSection(&g_cap_cs);
    g_ctx->lpVtbl->UpdateSubresource(g_ctx,(ID3D11Resource *)t,0,NULL,rgba,DG_GRIPDBG_SIZE*4u,0);
    LeaveCriticalSection(&g_cap_cs);
    return SUCCEEDED(g_dev->lpVtbl->GetDeviceRemovedReason(g_dev)) ? DG_RADIAL_IO_OK : DG_RADIAL_IO_ERROR;
}
static int gripdbg_gpu_release(void *unused, uint64_t handle) {
    XrSwapchainImageReleaseInfo ri={XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_gripdbg_gpu_swap || !pfn_ReleaseImage) return DG_RADIAL_IO_ERROR;
    return gripdbg_gpu_io("release",pfn_ReleaseImage(g_gripdbg_gpu_swap,&ri));
}
static const dg_radial_overlay_ops g_gripdbg_ops={NULL,gripdbg_gpu_create,gripdbg_gpu_acquire,
    gripdbg_gpu_wait,gripdbg_gpu_upload,gripdbg_gpu_release,gripdbg_gpu_destroy};
static void gripdbg_new_session(void) {
    if(g_gripdbg_gpu_quarantined || g_gripdbg_gpu_swap) return;
    memset(&g_gripdbg_overlay,0,sizeof g_gripdbg_overlay);
    g_gripdbg_wrist_held=g_gripdbg_logged=0;
}
static void gripdbg_stop(int stopped) {
    if(!stopped || g_gripdbg_gpu_quarantined) return;
    if(g_gripdbg_overlay.handle) dg_radial_overlay_teardown(&g_gripdbg_overlay,&g_gripdbg_ops,1);
    else if(g_gripdbg_gpu_swap) gripdbg_gpu_destroy(NULL,(uint64_t)(uintptr_t)g_gripdbg_gpu_swap);
}
static void gripdbg_parent_destroyed(int destroyed) {
    if(!destroyed) {g_gripdbg_gpu_quarantined=1;return;}
    g_gripdbg_gpu_swap=XR_NULL_HANDLE;
    free(g_gripdbg_gpu_images);g_gripdbg_gpu_images=NULL;g_gripdbg_gpu_image_count=0;
    memset(&g_gripdbg_overlay,0,sizeof g_gripdbg_overlay);
}
static void gripdbg_quad(XrCompositionLayerQuad *q,int tile,const double pos[3],float w,float h) {
    memset(q,0,sizeof *q);q->type=XR_TYPE_COMPOSITION_LAYER_QUAD;
    q->layerFlags=XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT|XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
    q->space=g_local_space;q->eyeVisibility=XR_EYE_VISIBILITY_BOTH;
    q->subImage.swapchain=g_gripdbg_gpu_swap;
    q->subImage.imageRect.offset.x=(tile%2)*128;q->subImage.imageRect.offset.y=(tile/2)*128;
    q->subImage.imageRect.extent.width=128;q->subImage.imageRect.extent.height=tile==3?32:128;
    q->pose.orientation.w=1;q->pose.position.x=(float)pos[0];q->pose.position.y=(float)pos[1];q->pose.position.z=(float)pos[2];
    q->size.width=w;q->size.height=h;
}
static void gripdbg_append(XrFrameEndInfo *end,const XrCompositionLayerBaseHeader **layers,
    unsigned capacity,XrCompositionLayerQuad q[6],int render,int theater,int views,XrTime when,uint64_t now) {
#if DG_ENABLE_DIAGNOSTICS

    DG_GRIP_DEBUG s;double a[3],b[3],mid[3],distance=0;int ready,k;
    XrSpaceLocation head={XR_TYPE_SPACE_LOCATION};
    const XrSpaceLocationFlags need=XR_SPACE_LOCATION_ORIENTATION_VALID_BIT|XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT|
        XR_SPACE_LOCATION_POSITION_VALID_BIT|XR_SPACE_LOCATION_POSITION_TRACKED_BIT;
    ready=g_cfg.grip_debug && !g_gripdbg_gpu_quarantined && g_session_running &&
        g_state==XR_SESSION_STATE_FOCUSED && render && !theater && views &&
        end->layerCount && end->layers && end->layers[0]->type==XR_TYPE_COMPOSITION_LAYER_PROJECTION &&
        end->layerCount+6<=capacity && end->layerCount+6<=g_radial_max_layers && g_local_space && g_view_space;
    if(ready)ready=dg_bridge_grip_debug_snapshot(&s) && dg_grip_debug_points(&s,now,a,b,&distance);
    if(ready)ready=pfn_LocateSpace && pfn_LocateSpace(g_view_space,g_local_space,when,&head)==XR_SUCCESS && (head.locationFlags&need)==need;
    if(ready)dg_grip_debug_raster(&s,distance,g_gripdbg_pixels);
    if(!dg_health_upload(&g_gripdbg_overlay,&g_gripdbg_ops,ready,g_gripdbg_pixels))return;
    for(k=0;k<3;k++)gripdbg_quad(&q[k],0,a,(float)(2*DG_M9_GRAB_RADIUS_M),(float)(2*DG_M9_GRAB_RADIUS_M));
    q[1].pose.orientation.x=.70710678f;q[1].pose.orientation.w=.70710678f;
    q[2].pose.orientation.y=.70710678f;q[2].pose.orientation.w=.70710678f;
    gripdbg_quad(&q[3],1,b,.008f,.008f);q[3].pose.orientation=head.pose.orientation;
    for(k=0;k<3;k++)mid[k]=(a[k]+b[k])*.5;
    gripdbg_quad(&q[4],2,mid,(float)(distance>.0001?distance:.0001),.016f);
    {
        double h[3]={head.pose.position.x,head.pose.position.y,head.pose.position.z},rot[4];
        if(dg_grip_line_q(a,b,h,rot)) {
            q[4].pose.orientation.x=(float)rot[0];q[4].pose.orientation.y=(float)rot[1];
            q[4].pose.orientation.z=(float)rot[2];q[4].pose.orientation.w=(float)rot[3];
        } else q[4].pose.orientation=head.pose.orientation;
        /* OpenXR quads are one-sided. Flip each great circle towards the head. */
        if(h[2]<a[2]){q[0].pose.orientation.x=1;q[0].pose.orientation.w=0;}
        if(h[1]>a[1])q[1].pose.orientation.x=-.70710678f;
        if(h[0]<a[0])q[2].pose.orientation.y=-.70710678f;
    }
    mid[1]+=.055;gripdbg_quad(&q[5],3,mid,.12f,.03f);q[5].pose.orientation=head.pose.orientation;
    for(k=0;k<6;k++)layers[end->layerCount++]=(const XrCompositionLayerBaseHeader *)&q[k];
    end->layers=layers;
    if(!g_gripdbg_logged && g_log){g_log("  M9 debug: actual hit-test sphere/point/line/mm submitted; headset placement unverified\r\n");g_gripdbg_logged=1;}

#else

#endif
}
