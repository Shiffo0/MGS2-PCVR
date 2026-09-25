

#include "dg_radar_gaze.h"
#include "dg_weapon_watch_pose.h"

#define DG_AMMO_FRESH_MS      250u    /* no capture this recent = the game is not drawing a ammo */
#define DG_AMMO_AVAILABLE_MS  500u    /* how long "the wrist layer is up" stays true for the draw hook */
#define DG_AMMO_FIXED_WIDTH_M 0.30f
#define DG_AMMO_FIXED_Y_M     (-0.10f)
#define DG_AMMO_FIXED_Z_M     (-0.60f)

/* Why the layer was not submitted this frame; the heartbeat prints the last one. */
enum { AMMO_SHOWN = 0, AMMO_OFF, AMMO_FAULT, AMMO_SESSION, AMMO_THEATER, AMMO_NO_SLOT,
       AMMO_STALE, AMMO_HAND, AMMO_SWAPCHAIN, AMMO_GATE, AMMO_COPY, AMMO_WHY_COUNT };
static const char *const g_ammo_why_name[AMMO_WHY_COUNT] = {
    "shown", "off", "FAULT", "session-not-focused", "theater-or-no-projection", "no-layer-slot",
    "no-fresh-capture", "right-arm-unavailable", "swapchain", "gate-closed", "copy" };

/* Store: written by the draw thread, read by the XR thread, both under g_cap_cs. */
static ID3D11Texture2D *g_ammo_store;
static uint32_t g_ammo_store_w, g_ammo_store_h;
static DXGI_FORMAT g_ammo_store_format;
static uint64_t g_ammo_store_id, g_ammo_store_ms;
/* XR thread only. */
static XrSwapchain g_ammo_swap;
static XrSwapchainImageD3D11KHR *g_ammo_images;
static uint32_t g_ammo_image_count, g_ammo_sw, g_ammo_sh;
static int64_t g_ammo_swap_format;
static uint64_t g_ammo_copied_id;
static int g_ammo_fault;
static DG_RADAR_GATE g_ammo_gate;
/* Shared, interlocked. */
static volatile LONG64 g_ammo_available_ms;
static volatile LONG g_ammo_st_captures, g_ammo_st_capture_errors, g_ammo_st_submitted,
                     g_ammo_st_errors, g_ammo_st_why = AMMO_OFF, g_ammo_st_gate,
                     g_ammo_st_theta, g_ammo_st_phi, g_ammo_st_size, g_ammo_st_format;

int dg_xr_ammo_capture(void *d3d11_texture2d) {
    ID3D11Texture2D *src = (ID3D11Texture2D *)d3d11_texture2d;
    ID3D11Device *dev = NULL;
    ID3D11DeviceContext *ctx = NULL;
    D3D11_TEXTURE2D_DESC td;
    int stored = 0;
    uint64_t now, up;
    if (!src || !g_cfg.radar_mode || !InterlockedCompareExchange(&g_cap_cs_ready, 0, 0) ||
        !InterlockedCompareExchange(&g_started, 0, 0)) return -1;
    src->lpVtbl->GetDesc(src, &td);
    if (!dg_capture_format_group(td.Format) || td.SampleDesc.Count != 1 ||
        td.MipLevels != 1 || td.ArraySize != 1 || !td.Width || !td.Height) {
        InterlockedIncrement(&g_ammo_st_capture_errors); return -1;
    }
    src->lpVtbl->GetDevice(src, &dev);
    if (dev) dev->lpVtbl->GetImmediateContext(dev, &ctx);
    if (!dev || !ctx) {
        if (dev) dev->lpVtbl->Release(dev);
        InterlockedIncrement(&g_ammo_st_capture_errors); return -1;
    }
    EnterCriticalSection(&g_cap_cs);
    if (g_ammo_store) {
        ID3D11Device *owner = NULL;
        g_ammo_store->lpVtbl->GetDevice(g_ammo_store, &owner);
        /* Resolution change or a new device: drop it and let it be rebuilt. */
        if (owner != dev || g_ammo_store_w != td.Width || g_ammo_store_h != td.Height ||
            g_ammo_store_format != td.Format) {
            g_ammo_store->lpVtbl->Release(g_ammo_store); g_ammo_store = NULL;
        }
        if (owner) owner->lpVtbl->Release(owner);
    }
    if (!g_ammo_store) {
        D3D11_TEXTURE2D_DESC cd = td;
        cd.Usage = D3D11_USAGE_DEFAULT;
        cd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        cd.CPUAccessFlags = 0;
        cd.MiscFlags = 0;
        if (FAILED(dev->lpVtbl->CreateTexture2D(dev, &cd, NULL, &g_ammo_store))) g_ammo_store = NULL;
        g_ammo_store_w = td.Width; g_ammo_store_h = td.Height; g_ammo_store_format = td.Format;
    }
    if (g_ammo_store) {
        ctx->lpVtbl->CopyResource(ctx, (ID3D11Resource *)g_ammo_store, (ID3D11Resource *)src);
        g_ammo_store_id++;
        g_ammo_store_ms = GetTickCount64();
        stored = 1;
    }
    LeaveCriticalSection(&g_cap_cs);
    ctx->lpVtbl->Release(ctx);
    dev->lpVtbl->Release(dev);
    if (!stored) { InterlockedIncrement(&g_ammo_st_capture_errors); return -1; }
    InterlockedIncrement(&g_ammo_st_captures);
    now = GetTickCount64();
    up = (uint64_t)InterlockedCompareExchange64(&g_ammo_available_ms, 0, 0);
    return up && now >= up && now - up <= DG_AMMO_AVAILABLE_MS;
}

/* One line for the heartbeat. Any thread; the store fields are read under the lock. */
void dg_xr_ammo_stats(char *out, size_t n) {
    uint64_t ms = 0, now = GetTickCount64();
    LONG why = InterlockedCompareExchange(&g_ammo_st_why, 0, 0);
    LONG size = InterlockedCompareExchange(&g_ammo_st_size, 0, 0);
    if (InterlockedCompareExchange(&g_cap_cs_ready, 0, 0)) {
        EnterCriticalSection(&g_cap_cs); ms = g_ammo_store_ms; LeaveCriticalSection(&g_cap_cs);
    }
    if (why < 0 || why >= AMMO_WHY_COUNT) why = AMMO_FAULT;
    _snprintf_s(out, n, _TRUNCATE,
        "mode %s  captures %ld (refused %ld) last %lld ms ago  layer submitted %ld  now %s  gate %s (gaze %ld face %ld deg)"
        "  swapchain %ldx%ld fmt %ld  errors %ld",
        g_cfg.radar_mode == 1 ? "fixed" : g_cfg.radar_mode == 2 ? "wrist" : "off",
        (long)g_ammo_st_captures, (long)g_ammo_st_capture_errors, ms ? (long long)(now - ms) : -1LL,
        (long)g_ammo_st_submitted, g_ammo_why_name[why],
        g_cfg.radar_mode == 1 ? "bypassed" : !(g_cfg.radar_gaze_deg > 0.0) ? "disabled" : g_ammo_st_gate ? "open" : "closed",
        (long)g_ammo_st_theta, (long)g_ammo_st_phi,
        (long)(size >> 16), (long)(size & 0xffff), (long)g_ammo_st_format, (long)g_ammo_st_errors);
}

static void ammo_fail(const char *op, long code) {
    InterlockedIncrement(&g_ammo_st_errors);
    if (!g_ammo_fault && g_log)
        g_log("  xr ammo: %s failed (%ld); wrist ammo disabled until the session is rebuilt\r\n", op, code);
    g_ammo_fault = 1;
}

static void ammo_destroy_swapchain(void) {
    if (g_ammo_swap && pfn_DestroySwapchain) {
        XrResult r = pfn_DestroySwapchain(g_ammo_swap);
        if (XR_FAILED(r)) InterlockedIncrement(&g_ammo_st_errors);
    }
    g_ammo_swap = XR_NULL_HANDLE;
    free(g_ammo_images); g_ammo_images = NULL; g_ammo_image_count = 0;
    g_ammo_sw = g_ammo_sh = 0; g_ammo_swap_format = 0; g_ammo_copied_id = 0;
    InterlockedExchange(&g_ammo_st_size, 0); InterlockedExchange(&g_ammo_st_format, 0);
}

/* From xr_teardown, while the session still exists. A new session starts clean. */
static void ammo_teardown(void) {
    ammo_destroy_swapchain();
    g_ammo_fault = 0;
    dg_radar_gate_reset(&g_ammo_gate);
    InterlockedExchange64(&g_ammo_available_ms, 0);
    InterlockedExchange(&g_ammo_st_why, AMMO_OFF);
    if (g_cap_cs_ready) {
        EnterCriticalSection(&g_cap_cs);
        if (g_ammo_store) { g_ammo_store->lpVtbl->Release(g_ammo_store); g_ammo_store = NULL; }
        g_ammo_store_id = g_ammo_store_ms = 0;
        LeaveCriticalSection(&g_cap_cs);
    }
}

/* Same size and colour family as the store, so CopyResource needs no blit.
   UNORM before SRGB - the eye swapchains' choice, so the ammo is as bright
   as the picture it was cut out of. */
static int ammo_ensure_swapchain(uint32_t w, uint32_t h, DXGI_FORMAT store_format) {
    int group = dg_capture_format_group(store_format);
    int64_t *formats = NULL, want = 0, first, second;
    uint32_t count = 0, got = 0, i;
    XrSwapchainCreateInfo ci;
    XrResult r;
    if (g_ammo_swap) {
        if (g_ammo_sw == w && g_ammo_sh == h &&
            dg_capture_format_group((DXGI_FORMAT)g_ammo_swap_format) == group) return 1;
        ammo_destroy_swapchain();          /* resolution change; no image is ever held across frames */
    }
    if (!group || !w || !h || !g_sess || !g_dev || !g_ctx || !pfn_EnumScFormats || !pfn_CreateSwapchain ||
        !pfn_EnumScImages || !pfn_DestroySwapchain) { ammo_fail("swapchain preconditions", 0); return 0; }
    first = group == 1 ? DXGI_FORMAT_B8G8R8A8_UNORM : DXGI_FORMAT_R8G8B8A8_UNORM;
    second = group == 1 ? DXGI_FORMAT_B8G8R8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    r = pfn_EnumScFormats(g_sess, 0, &count, NULL);
    if (XR_FAILED(r) || !count || count > 4096) { ammo_fail("format count", (long)r); return 0; }
    formats = (int64_t *)calloc(count, sizeof(*formats));
    if (!formats) { ammo_fail("format list allocation", 0); return 0; }
    r = pfn_EnumScFormats(g_sess, count, &got, formats);
    if (XR_SUCCEEDED(r) && got <= count) {
        for (i = 0; i < got && !want; i++) if (formats[i] == first) want = first;
        for (i = 0; i < got && !want; i++) if (formats[i] == second) want = second;
    }
    free(formats);
    if (XR_FAILED(r)) { ammo_fail("formats", (long)r); return 0; }
    if (!want) { ammo_fail("no swapchain format in the texture's colour family", (long)store_format); return 0; }
    memset(&ci, 0, sizeof(ci)); ci.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
    ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
    ci.format = want; ci.sampleCount = 1; ci.width = w; ci.height = h;
    ci.faceCount = ci.arraySize = ci.mipCount = 1;
    r = pfn_CreateSwapchain(g_sess, &ci, &g_ammo_swap);
    if (XR_FAILED(r) || !g_ammo_swap) { g_ammo_swap = XR_NULL_HANDLE; ammo_fail("create swapchain", (long)r); return 0; }
    count = 0;
    r = pfn_EnumScImages(g_ammo_swap, 0, &count, NULL);
    if (XR_FAILED(r) || !count || count > 256) { ammo_fail("image count", (long)r); ammo_destroy_swapchain(); return 0; }
    g_ammo_images = (XrSwapchainImageD3D11KHR *)calloc(count, sizeof(*g_ammo_images));
    if (!g_ammo_images) { ammo_fail("image list allocation", 0); ammo_destroy_swapchain(); return 0; }
    for (i = 0; i < count; i++) g_ammo_images[i].type = XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR;
    got = 0;
    r = pfn_EnumScImages(g_ammo_swap, count, &got, (XrSwapchainImageBaseHeader *)g_ammo_images);
    if (XR_FAILED(r) || !got || got > count) { ammo_fail("images", (long)r); ammo_destroy_swapchain(); return 0; }
    g_ammo_image_count = got; g_ammo_sw = w; g_ammo_sh = h; g_ammo_swap_format = want; g_ammo_copied_id = 0;
    InterlockedExchange(&g_ammo_st_size, (LONG)(((w & 0xffff) << 16) | (h & 0xffff)));
    InterlockedExchange(&g_ammo_st_format, (LONG)want);
    if (g_log) g_log("  xr ammo: swapchain %ux%u fmt %lld (texture fmt %d), %u images\r\n",
                     w, h, (long long)want, (int)store_format, got);
    return 1;
}

/* Store -> swapchain image, only when the store holds a capture the swapchain
   has not seen. A failed wait leaves its image to session teardown (it may not
   be released); every other path releases what it acquired. */
static int ammo_copy(uint64_t id) {
    XrSwapchainImageAcquireInfo ai = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    XrSwapchainImageWaitInfo wi = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    XrSwapchainImageReleaseInfo ri = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    uint32_t index = 0;
    int copied = 0;
    XrResult r;
    if (id == g_ammo_copied_id) return 1;
    if (!pfn_AcquireImage || !pfn_WaitImage || !pfn_ReleaseImage) { ammo_fail("entry points", 0); return 0; }
    r = pfn_AcquireImage(g_ammo_swap, &ai, &index);
    if (XR_FAILED(r)) { ammo_fail("acquire", (long)r); return 0; }
    wi.timeout = 1;                  /* 1 ns: never park the frame loop on a side layer */
    r = pfn_WaitImage(g_ammo_swap, &wi);
    if (XR_FAILED(r) || r == XR_TIMEOUT_EXPIRED) { ammo_fail("wait", (long)r); return 0; }
    EnterCriticalSection(&g_cap_cs);
    if (g_ammo_store && g_ammo_images && index < g_ammo_image_count && g_ammo_images[index].texture && g_dev && g_ctx) {
        ID3D11Texture2D *dst = g_ammo_images[index].texture;
        D3D11_TEXTURE2D_DESC sd, dd;
        ID3D11Device *so = NULL, *dn = NULL;
        g_ammo_store->lpVtbl->GetDesc(g_ammo_store, &sd); dst->lpVtbl->GetDesc(dst, &dd);
        g_ammo_store->lpVtbl->GetDevice(g_ammo_store, &so); dst->lpVtbl->GetDevice(dst, &dn);
        if (so == g_dev && dn == g_dev && sd.Width == dd.Width && sd.Height == dd.Height &&
            sd.MipLevels == dd.MipLevels && sd.ArraySize == dd.ArraySize &&
            sd.SampleDesc.Count == dd.SampleDesc.Count && sd.SampleDesc.Quality == dd.SampleDesc.Quality &&
            dg_capture_format_compatible(sd.Format, dd.Format)) {
            g_ctx->lpVtbl->CopyResource(g_ctx, (ID3D11Resource *)dst, (ID3D11Resource *)g_ammo_store);
            copied = 1;
        }
        if (so) so->lpVtbl->Release(so);
        if (dn) dn->lpVtbl->Release(dn);
    }
    LeaveCriticalSection(&g_cap_cs);
    r = pfn_ReleaseImage(g_ammo_swap, &ri);
    if (XR_FAILED(r)) { ammo_fail("release", (long)r); return 0; }
    if (!copied) { ammo_fail("store and swapchain image do not match", 0); return 0; }
    g_ammo_copied_id = id;
    return 1;
}

static void ammo_pose_from_raw(const DG_XR_RAW_POSE *in, DG_RADAR_POSE *out) {
    out->qx = in->qx; out->qy = in->qy; out->qz = in->qz; out->qw = in->qw;
    out->px = in->px; out->py = in->py; out->pz = in->pz;
}

static int ammo_skip(int why) { InterlockedExchange(&g_ammo_st_why, why); return 0; }

/* XR thread, once per frame, after the projection layer has been decided and
   BEFORE radial_append. Fills *quad and returns 1 when the ammo is to be
   shown this frame; ammo_insert then slots it in. Two steps because the
   radial menu's own readiness test wants to find the projection alone. */
static int ammo_prepare(const XrFrameEndInfo *end, XrCompositionLayerQuad *quad, int should_render,
                         int theater, const DG_XR_FRAME *frame, int frame_status, uint64_t now,
                         const DG_MODEL_ARM *model) {
    int mode = g_cfg.radar_mode==2 ? 2 : 0, open = 1;
    uint32_t w, h;
    uint64_t id, ms;
    DXGI_FORMAT format;
    double width;
    if (mode != 1 && mode != 2) {
        if (g_ammo_gate.open || g_ammo_gate.timing) dg_radar_gate_reset(&g_ammo_gate);
        return ammo_skip(AMMO_OFF);
    }
    if (g_ammo_fault) return ammo_skip(AMMO_FAULT);
    if (!g_session_running || g_state != XR_SESSION_STATE_FOCUSED || !should_render) {
        dg_radar_gate_reset(&g_ammo_gate); return ammo_skip(AMMO_SESSION);
    }
    /* Only ever ON TOP OF the stereo/mono projection: never over the theater
       quad (menus, cutscenes) and never as the only layer. */
    if (theater || !end->layerCount || !end->layers ||
        end->layers[0]->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
        dg_radar_gate_reset(&g_ammo_gate); return ammo_skip(AMMO_THEATER);
    }
    if (g_radial_max_layers <= end->layerCount) return ammo_skip(AMMO_NO_SLOT);
    EnterCriticalSection(&g_cap_cs);
    id = g_ammo_store ? g_ammo_store_id : 0; ms = g_ammo_store_ms;
    w = g_ammo_store_w; h = g_ammo_store_h; format = g_ammo_store_format;
    LeaveCriticalSection(&g_cap_cs);
    /* Native weapon-panel visibility is preserved. Reject old captures
       instead of showing a frozen weapon or ammunition count. */
    if (!id || now < ms || now - ms > DG_AMMO_FRESH_MS) {
        dg_radar_gate_reset(&g_ammo_gate); return ammo_skip(AMMO_STALE);
    }
    memset(quad, 0, sizeof(*quad));
    quad->type = XR_TYPE_COMPOSITION_LAYER_QUAD;
    quad->layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;    /* premultiplied */
    quad->eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    if (mode == 1) {
        /* First live check: head-locked like the radial menu (VIEW space),
           always in view, no gate. */
        if (!g_view_space) return ammo_skip(AMMO_SESSION);
        quad->space = g_view_space;
        quad->pose.orientation.w = 1.0f;
        quad->pose.position.y = DG_AMMO_FIXED_Y_M; quad->pose.position.z = DG_AMMO_FIXED_Z_M;
        width = DG_AMMO_FIXED_WIDTH_M;
        dg_radar_gate_reset(&g_ammo_gate);
    } else {
        DG_RADAR_POSE head,local;
        DG_MODEL_ARM current;
        double theta=180.0,phi=180.0;
        int valid;
        width=.12; /* 120 mm capture region includes transparent native margins. */
        /* Capture owns the rendered model pose. Current bridge admission can
           revoke it immediately, but a newer controller never moves it. */
        if(!(frame_status&1) || !raw_pose_usable(&frame->head_raw) || !g_local_space ||
           !dg_bridge_right_model_arm_snapshot(&current) || !model ||
           current.stream!=model->stream || !w || !h ||
           !dg_watch_pose(model,width,width*(double)h/(double)w,now,&local)) {
            dg_radar_gate_reset(&g_ammo_gate);return ammo_skip(AMMO_HAND);
        }
        ammo_pose_from_raw(&frame->head_raw,&head);
        valid=dg_watch_angles(&head,&local,&theta,&phi);
        open=dg_watch_gate_step(&g_ammo_gate,valid,theta,phi,now);
        InterlockedExchange(&g_ammo_st_theta,(LONG)theta);InterlockedExchange(&g_ammo_st_phi,(LONG)phi);
        quad->space=g_local_space;
        quad->pose.orientation.x=(float)local.qx;quad->pose.orientation.y=(float)local.qy;
        quad->pose.orientation.z=(float)local.qz;quad->pose.orientation.w=(float)local.qw;
        quad->pose.position.x=(float)local.px;quad->pose.position.y=(float)local.py;
        quad->pose.position.z=(float)local.pz;
    }
    InterlockedExchange(&g_ammo_st_gate, open);
    if (!ammo_ensure_swapchain(w, h, format)) return ammo_skip(g_ammo_fault ? AMMO_FAULT : AMMO_SWAPCHAIN);
    /* From here the ammo is on the wrist for whoever looks: the draw hook may withhold the HUD one. */
    InterlockedExchange64(&g_ammo_available_ms, (LONG64)now);
    if (!open) return ammo_skip(AMMO_GATE);
    if (!ammo_copy(id)) { InterlockedExchange64(&g_ammo_available_ms, 0); return ammo_skip(AMMO_COPY); }
    quad->subImage.swapchain = g_ammo_swap;
    quad->subImage.imageRect.extent.width = (int32_t)g_ammo_sw;
    quad->subImage.imageRect.extent.height = (int32_t)g_ammo_sh;
    quad->size.width = (float)width;
    quad->size.height = (float)(width * (double)g_ammo_sh / (double)g_ammo_sw);     /* the texture's aspect */
    return 1;
}

/* AFTER radial_append: directly above the projection, under the radial menu
   if that is up (layers are composited in order, so the menu wins). With no
   slot left the ammo yields. */
static void ammo_insert(XrFrameEndInfo *end, const XrCompositionLayerBaseHeader **layers,
                         unsigned capacity, const XrCompositionLayerQuad *quad) {
    unsigned max_layers = g_radial_max_layers < capacity ? g_radial_max_layers : capacity, i;
    if (!end->layerCount || end->layers != layers || end->layerCount >= max_layers ||
        layers[0]->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) { ammo_skip(AMMO_NO_SLOT); return; }
    for (i = end->layerCount; i > 1; i--) layers[i] = layers[i - 1];
    layers[1] = (const XrCompositionLayerBaseHeader *)quad;
    end->layerCount++;
    InterlockedIncrement(&g_ammo_st_submitted);
    InterlockedExchange(&g_ammo_st_why, AMMO_SHOWN);
}

