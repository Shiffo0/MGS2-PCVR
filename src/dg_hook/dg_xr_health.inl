/* XR-thread-owned LIFE resources. Same format, upload and bounded-wait
 * contract as radial; independent handle so neither HUD overwrites the other. */
#include "dg_health.h"
static XrSwapchain g_health_gpu_swap;
static XrSwapchainImageD3D11KHR *g_health_gpu_images;
static unsigned g_health_gpu_image_count;
static int g_health_gpu_quarantined;
static dg_radial_overlay g_health_overlay;
static uint32_t g_health_pixels[DG_HEALTH_PIXELS];
static int g_health_wrist_held, g_health_logged;
static int health_gpu_io(const char *op, XrResult r) {
    if (r==XR_SUCCESS) return DG_RADIAL_IO_OK;
    if (g_log) g_log("  xr LIFE: %s failed (%d); overlay disabled for this session\r\n",op,(int)r);
    return DG_RADIAL_IO_ERROR;
}
static int health_gpu_destroy(void *unused, uint64_t handle) {
    XrResult r; (void)unused;
    if (!g_health_gpu_swap || handle!=(uint64_t)(uintptr_t)g_health_gpu_swap || !pfn_DestroySwapchain)
        return DG_RADIAL_IO_ERROR;
    r=pfn_DestroySwapchain(g_health_gpu_swap);
    if (health_gpu_io("destroy",r)!=DG_RADIAL_IO_OK) return DG_RADIAL_IO_ERROR;
    g_health_gpu_swap=XR_NULL_HANDLE;
    free(g_health_gpu_images); g_health_gpu_images=NULL; g_health_gpu_image_count=0;
    return DG_RADIAL_IO_OK;
}
static int health_gpu_create(void *unused, unsigned width, unsigned height,
                          uint64_t *handle, unsigned *images) {
    int64_t *formats=NULL, want=0;
    uint32_t count=0, got=0, i;
    XrSwapchainCreateInfo ci;
    XrResult r;
    (void)unused; *handle=0; *images=0;
    if (!g_sess || !g_dev || !g_ctx || g_health_gpu_swap || g_health_gpu_quarantined ||
        width!=DG_HEALTH_SIZE || height!=DG_HEALTH_SIZE ||
        !pfn_EnumScFormats || !pfn_CreateSwapchain || !pfn_EnumScImages || !pfn_DestroySwapchain)
        return DG_RADIAL_IO_ERROR;
    r=pfn_EnumScFormats(g_sess,0,&count,NULL);
    if (health_gpu_io("format count",r)!=DG_RADIAL_IO_OK || !count || count>4096) return DG_RADIAL_IO_ERROR;
    formats=(int64_t *)calloc(count,sizeof(*formats));
    if (!formats) return DG_RADIAL_IO_ERROR;
    r=pfn_EnumScFormats(g_sess,count,&got,formats);
    if (health_gpu_io("formats",r)==DG_RADIAL_IO_OK && got<=count) {
        for(i=0;i<got;i++) if(formats[i]==DXGI_FORMAT_R8G8B8A8_UNORM) want=formats[i];
        for(i=0;i<got;i++) if(formats[i]==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB) want=formats[i];
    }
    free(formats);
    if (!want) return DG_RADIAL_IO_ERROR;
    memset(&ci,0,sizeof(ci)); ci.type=XR_TYPE_SWAPCHAIN_CREATE_INFO;
    ci.usageFlags=XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
    ci.format=want; ci.sampleCount=1; ci.width=width; ci.height=height;
    ci.faceCount=ci.arraySize=ci.mipCount=1;
    r=pfn_CreateSwapchain(g_sess,&ci,&g_health_gpu_swap);
    if (health_gpu_io("create",r)!=DG_RADIAL_IO_OK || !g_health_gpu_swap) goto fail;
    r=pfn_EnumScImages(g_health_gpu_swap,0,&count,NULL);
    if (health_gpu_io("image count",r)!=DG_RADIAL_IO_OK || !count || count>256) goto fail;
    g_health_gpu_images=(XrSwapchainImageD3D11KHR *)calloc(count,sizeof(*g_health_gpu_images));
    if (!g_health_gpu_images) goto fail;
    for(i=0;i<count;i++) g_health_gpu_images[i].type=XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR;
    r=pfn_EnumScImages(g_health_gpu_swap,count,&got,(XrSwapchainImageBaseHeader *)g_health_gpu_images);
    if (health_gpu_io("images",r)!=DG_RADIAL_IO_OK || !got || got>count) goto fail;
    g_health_gpu_image_count=got;
    *handle=(uint64_t)(uintptr_t)g_health_gpu_swap; *images=got;
    return DG_RADIAL_IO_OK;
fail:
    /* If rollback fails retain the orphan separately for parent teardown;
     * never publish a partially enumerated handle to the helper. */
    if (g_health_gpu_swap) health_gpu_destroy(NULL,(uint64_t)(uintptr_t)g_health_gpu_swap);
    return DG_RADIAL_IO_ERROR;
}
static int health_gpu_acquire(void *unused, uint64_t handle, unsigned *image) {
    XrSwapchainImageAcquireInfo ai={XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
    (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_health_gpu_swap || !pfn_AcquireImage) return DG_RADIAL_IO_ERROR;
    return health_gpu_io("acquire",pfn_AcquireImage(g_health_gpu_swap,&ai,image));
}
static int health_gpu_wait(void *unused, uint64_t handle) {
    XrSwapchainImageWaitInfo wi={XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
    XrResult r; (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_health_gpu_swap || !pfn_WaitImage) return DG_RADIAL_IO_ERROR;
    wi.timeout=0; r=pfn_WaitImage(g_health_gpu_swap,&wi);
    if (r==XR_TIMEOUT_EXPIRED) return DG_RADIAL_IO_TIMEOUT;
    return health_gpu_io("wait",r);
}
static int health_gpu_upload(void *unused, uint64_t handle, unsigned image,
                          const void *rgba, size_t bytes) {
    D3D11_TEXTURE2D_DESC d;
    ID3D11Texture2D *t;
    ID3D11Device *owner=NULL;
    int valid;
    (void)unused;
    if (!rgba || bytes!=DG_HEALTH_PIXELS*4u || !g_dev || !g_ctx || !g_cap_cs_ready ||
        handle!=(uint64_t)(uintptr_t)g_health_gpu_swap || image>=g_health_gpu_image_count ||
        !g_health_gpu_images || !(t=g_health_gpu_images[image].texture)) return DG_RADIAL_IO_ERROR;
    t->lpVtbl->GetDesc(t,&d); t->lpVtbl->GetDevice(t,&owner);
    valid=owner==g_dev && d.Width==DG_HEALTH_SIZE && d.Height==DG_HEALTH_SIZE &&
        d.ArraySize==1 && d.MipLevels==1 && d.SampleDesc.Count==1 &&
        d.Usage==D3D11_USAGE_DEFAULT &&
        (d.Format==DXGI_FORMAT_R8G8B8A8_UNORM || d.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB ||
         d.Format==DXGI_FORMAT_R8G8B8A8_TYPELESS);
    if(owner) owner->lpVtbl->Release(owner);
    if (!valid || FAILED(g_dev->lpVtbl->GetDeviceRemovedReason(g_dev))) return DG_RADIAL_IO_ERROR;
    /* Only the D3D call is under the capture lock; XR waits never are.
     * UpdateSubresource consumes CPU bytes before return, no retained scratch. */
    EnterCriticalSection(&g_cap_cs);
    g_ctx->lpVtbl->UpdateSubresource(g_ctx,(ID3D11Resource *)t,0,NULL,rgba,DG_HEALTH_SIZE*4u,0);
    LeaveCriticalSection(&g_cap_cs);
    return SUCCEEDED(g_dev->lpVtbl->GetDeviceRemovedReason(g_dev)) ? DG_RADIAL_IO_OK : DG_RADIAL_IO_ERROR;
}
static int health_gpu_release(void *unused, uint64_t handle) {
    XrSwapchainImageReleaseInfo ri={XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
    (void)unused;
    if (handle!=(uint64_t)(uintptr_t)g_health_gpu_swap || !pfn_ReleaseImage) return DG_RADIAL_IO_ERROR;
    return health_gpu_io("release",pfn_ReleaseImage(g_health_gpu_swap,&ri));
}
static const dg_radial_overlay_ops g_health_ops={NULL,health_gpu_create,health_gpu_acquire,
    health_gpu_wait,health_gpu_upload,health_gpu_release,health_gpu_destroy};
static void health_new_session(void) {
    if(g_health_gpu_quarantined || g_health_gpu_swap) return;
    memset(&g_health_overlay,0,sizeof g_health_overlay);
    g_health_wrist_held=g_health_logged=0;
}
static void health_stop(int stopped) {
    if(!stopped || g_health_gpu_quarantined) return;
    if(g_health_overlay.handle) dg_radial_overlay_teardown(&g_health_overlay,&g_health_ops,1);
    else if(g_health_gpu_swap) health_gpu_destroy(NULL,(uint64_t)(uintptr_t)g_health_gpu_swap);
}
static void health_parent_destroyed(int destroyed) {
    if(!destroyed) {g_health_gpu_quarantined=1;return;}
    g_health_gpu_swap=XR_NULL_HANDLE;
    free(g_health_gpu_images);g_health_gpu_images=NULL;g_health_gpu_image_count=0;
    memset(&g_health_overlay,0,sizeof g_health_overlay);
}
static void health_append(XrFrameEndInfo *end,const XrCompositionLayerBaseHeader **layers,
                          unsigned capacity,XrCompositionLayerQuad *quad,
                          int should_render,int theater,int views_valid,XrTime when,uint64_t now) {
    dg_health_view v;
    XrSpaceLocation grip={XR_TYPE_SPACE_LOCATION};
    const XrSpaceLocationFlags need=XR_SPACE_LOCATION_POSITION_VALID_BIT |
        XR_SPACE_LOCATION_ORIENTATION_VALID_BIT | XR_SPACE_LOCATION_POSITION_TRACKED_BIT |
        XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT;
    double p[3],q[4],wrist[3]={0,0,0};
    int mode=0,items=0,ready;
    const XrCompositionLayerQuad *wrist_radar=NULL;
    ready=!g_cfg.life_disabled && !g_health_gpu_quarantined && g_session_running &&
        g_state==XR_SESSION_STATE_FOCUSED && should_render && views_valid && !theater &&
        end->layerCount && end->layers && end->layers[0]->type==XR_TYPE_COMPOSITION_LAYER_PROJECTION &&
        end->layerCount<capacity && end->layerCount<g_radial_max_layers && g_view_space;
    if(ready && dg_bridge_health_snapshot(&v)) {
        unsigned scan;
        if(g_cfg.radar_mode==2 && health_radar_swap()) for(scan=1;scan<end->layerCount;scan++) {
            const XrCompositionLayerBaseHeader *h=end->layers[scan];
            if(h && h->type==XR_TYPE_COMPOSITION_LAYER_QUAD &&
               ((const XrCompositionLayerQuad *)h)->subImage.swapchain==health_radar_swap())
                wrist_radar=(const XrCompositionLayerQuad *)h;
        }
        if(g_cfg.radar_mode==2) g_health_wrist_held=wrist_radar!=NULL;
        else if(g_hand_action[HAND_LEFT].grip_active && g_hand_runtime[HAND_LEFT].grip_space &&
           pfn_LocateSpace(g_hand_runtime[HAND_LEFT].grip_space,g_view_space,when,&grip)==XR_SUCCESS &&
           (grip.locationFlags&need)==need) {
            p[0]=grip.pose.position.x;p[1]=grip.pose.position.y;p[2]=grip.pose.position.z;
            q[0]=grip.pose.orientation.x;q[1]=grip.pose.orientation.y;
            q[2]=grip.pose.orientation.z;q[3]=grip.pose.orientation.w;
            g_health_wrist_held=dg_health_wrist(p,q,g_health_wrist_held,wrist);
        } else g_health_wrist_held=0;
        /* Only a successfully appended item radial makes LIFE persistent. */
        unsigned layer;int radial_submitted=0;
        for(layer=1;layer<end->layerCount;layer++) {
            const XrCompositionLayerBaseHeader *h=end->layers[layer];
            if(h && h->type==XR_TYPE_COMPOSITION_LAYER_QUAD && g_radial_swap &&
               ((const XrCompositionLayerQuad *)h)->subImage.swapchain==g_radial_swap)
                radial_submitted=1;
        }
        if(radial_submitted && TryAcquireSRWLockShared(&g_radial_mail_lock)) {
            items=g_radial_mail.view.visible && g_radial_mail.view.kind==1 &&
                now>=g_radial_mail.sample_ms && now-g_radial_mail.sample_ms<=100;
            ReleaseSRWLockShared(&g_radial_mail_lock);
        }
        mode=dg_health_mode(&v,now,g_health_wrist_held,items);
    } else g_health_wrist_held=0;
    if(mode) dg_health_raster(&v.sample,g_health_pixels);
    /* Hidden frames still drain a timed-out acquisition, never submit it. */
    if(!dg_health_upload(&g_health_overlay,&g_health_ops,mode!=0,g_health_pixels)) return;
    memset(quad,0,sizeof *quad);quad->type=XR_TYPE_COMPOSITION_LAYER_QUAD;
    quad->layerFlags=XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
        XR_COMPOSITION_LAYER_UNPREMULTIPLIED_ALPHA_BIT;
    quad->space=g_view_space;quad->eyeVisibility=XR_EYE_VISIBILITY_BOTH;
    quad->subImage.swapchain=(XrSwapchain)(uintptr_t)g_health_overlay.handle;
    quad->subImage.imageRect.extent.width=256;quad->subImage.imageRect.extent.height=64;
    quad->pose.orientation.w=1;
    if(mode==1) {
        if(wrist_radar) {
            /* Same display-time space and orientation as the submitted radar.
               Place LIFE below its lower edge in the radar's own plane. */
            XrQuaternionf r=wrist_radar->pose.orientation;
            float down;
            quad->space=wrist_radar->space;quad->pose=wrist_radar->pose;
            quad->size.width=wrist_radar->size.width;
            down=-(wrist_radar->size.height*.5f+quad->size.width*.125f+.008f);
            quad->pose.position.x+=2*(r.x*r.y-r.w*r.z)*down;
            quad->pose.position.y+=(1-2*(r.x*r.x+r.z*r.z))*down;
            quad->pose.position.z+=2*(r.y*r.z+r.w*r.x)*down;
        } else {
            quad->pose.position.x=(float)wrist[0];quad->pose.position.y=(float)wrist[1];
            quad->pose.position.z=(float)wrist[2];quad->size.width=.19f;
        }
    } else {
        quad->pose.position.y=items?-.42f:-.23f;quad->pose.position.z=-1.2f;quad->size.width=.32f;
    }
    quad->size.height=quad->size.width*.25f;
    layers[end->layerCount++]=(const XrCompositionLayerBaseHeader *)quad;end->layers=layers;
    if(!g_health_logged && g_log) {
        g_log("  LIFE: first quad submitted (mode %s, value %d/%d); headset placement unverified\r\n",
            mode==1?"wrist":"event",v.sample.value,v.sample.maximum);g_health_logged=1;
    }
}
