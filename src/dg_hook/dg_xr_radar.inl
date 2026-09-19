

#include "dg_radar_gaze.h"

#define DG_RADAR_FRESH_MS      250u    /* no capture this recent = the game is not drawing a radar */
#define DG_RADAR_AVAILABLE_MS  500u    /* how long "the wrist layer is up" stays true for the draw hook */
#define DG_RADAR_FIXED_WIDTH_M 0.30f
#define DG_RADAR_FIXED_Y_M     (-0.10f)
#define DG_RADAR_FIXED_Z_M     (-0.60f)

/* Why the layer was not submitted this frame; the heartbeat prints the last one. */
enum { RADAR_SHOWN = 0, RADAR_OFF, RADAR_FAULT, RADAR_SESSION, RADAR_THEATER, RADAR_NO_SLOT,
       RADAR_STALE, RADAR_HAND, RADAR_SWAPCHAIN, RADAR_GATE, RADAR_COPY, RADAR_WHY_COUNT };
static const char *const g_radar_why_name[RADAR_WHY_COUNT] = {
    "shown", "off", "FAULT", "session-not-focused", "theater-or-no-projection", "no-layer-slot",
    "no-fresh-capture", "left-hand-not-tracked", "swapchain", "gate-closed", "copy" };

/* Store: written by the draw thread, read by the XR thread, both under g_cap_cs. */
static ID3D11Texture2D *g_radar_store;
static uint32_t g_radar_store_w, g_radar_store_h;
static DXGI_FORMAT g_radar_store_format;
static uint64_t g_radar_store_id, g_radar_store_ms;
/* XR thread only. */
static XrSwapchain g_radar_swap;
static XrSwapchainImageD3D11KHR *g_radar_images;
static uint32_t g_radar_image_count, g_radar_sw, g_radar_sh;
static int64_t g_radar_swap_format;
static uint64_t g_radar_copied_id;
static int g_radar_fault;
static DG_RADAR_GATE g_radar_gate;
/* Shared, interlocked. */
static volatile LONG64 g_radar_available_ms;
static volatile LONG g_radar_st_captures, g_radar_st_capture_errors, g_radar_st_submitted,
                     g_radar_st_errors, g_radar_st_why = RADAR_OFF, g_radar_st_gate,
                     g_radar_st_theta, g_radar_st_phi, g_radar_st_size, g_radar_st_format;

int dg_xr_radar_capture(void *d3d11_texture2d) {
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
        InterlockedIncrement(&g_radar_st_capture_errors); return -1;
    }
    src->lpVtbl->GetDevice(src, &dev);
    if (dev) dev->lpVtbl->GetImmediateContext(dev, &ctx);
    if (!dev || !ctx) {
        if (dev) dev->lpVtbl->Release(dev);
        InterlockedIncrement(&g_radar_st_capture_errors); return -1;
    }
    EnterCriticalSection(&g_cap_cs);
    if (g_radar_store) {
        ID3D11Device *owner = NULL;
        g_radar_store->lpVtbl->GetDevice(g_radar_store, &owner);
        /* Resolution change or a new device: drop it and let it be rebuilt. */
        if (owner != dev || g_radar_store_w != td.Width || g_radar_store_h != td.Height ||
            g_radar_store_format != td.Format) {
            g_radar_store->lpVtbl->Release(g_radar_store); g_radar_store = NULL;
        }
        if (owner) owner->lpVtbl->Release(owner);
    }
    if (!g_radar_store) {
        D3D11_TEXTURE2D_DESC cd = td;
        cd.Usage = D3D11_USAGE_DEFAULT;
        cd.BindFlags = D3D11_BIND_SHADER_RESOURCE;
        cd.CPUAccessFlags = 0;
        cd.MiscFlags = 0;
        if (FAILED(dev->lpVtbl->CreateTexture2D(dev, &cd, NULL, &g_radar_store))) g_radar_store = NULL;
        g_radar_store_w = td.Width; g_radar_store_h = td.Height; g_radar_store_format = td.Format;
    }
    if (g_radar_store) {
        ctx->lpVtbl->CopyResource(ctx, (ID3D11Resource *)g_radar_store, (ID3D11Resource *)src);
        g_radar_store_id++;
        g_radar_store_ms = GetTickCount64();
        stored = 1;
    }
    LeaveCriticalSection(&g_cap_cs);
    ctx->lpVtbl->Release(ctx);
    dev->lpVtbl->Release(dev);
    if (!stored) { InterlockedIncrement(&g_radar_st_capture_errors); return -1; }
    InterlockedIncrement(&g_radar_st_captures);
    now = GetTickCount64();
    up = (uint64_t)InterlockedCompareExchange64(&g_radar_available_ms, 0, 0);
    return up && now >= up && now - up <= DG_RADAR_AVAILABLE_MS;
}

/* One line for the heartbeat. Any thread; the store fields are read under the lock. */
void dg_xr_radar_stats(char *out, size_t n) {
    uint64_t ms = 0, now = GetTickCount64();
    LONG why = InterlockedCompareExchange(&g_radar_st_why, 0, 0);
    LONG size = InterlockedCompareExchange(&g_radar_st_size, 0, 0);
    if (InterlockedCompareExchange(&g_cap_cs_ready, 0, 0)) {
        EnterCriticalSection(&g_cap_cs); ms = g_radar_store_ms; LeaveCriticalSection(&g_cap_cs);
    }
    if (why < 0 || why >= RADAR_WHY_COUNT) why = RADAR_FAULT;
    _snprintf_s(out, n, _TRUNCATE,
        "mode %s  captures %ld (refused %ld) last %lld ms ago  layer submitted %ld  now %s  gate %s (gaze %ld face %ld deg)"
        "  swapchain %ldx%ld fmt %ld  errors %ld",
        g_cfg.radar_mode == 1 ? "fixed" : g_cfg.radar_mode == 2 ? "wrist" : "off",
        (long)g_radar_st_captures, (long)g_radar_st_capture_errors, ms ? (long long)(now - ms) : -1LL,
        (long)g_radar_st_submitted, g_radar_why_name[why],
        g_cfg.radar_mode == 1 ? "bypassed" : !(g_cfg.radar_gaze_deg > 0.0) ? "disabled" : g_radar_st_gate ? "open" : "closed",
        (long)g_radar_st_theta, (long)g_radar_st_phi,
        (long)(size >> 16), (long)(size & 0xffff), (long)g_radar_st_format, (long)g_radar_st_errors);
}

static void radar_fail(const char *op, long code) {
    InterlockedIncrement(&g_radar_st_errors);
    if (!g_radar_fault && g_log)
        g_log("  xr radar: %s failed (%ld); wrist radar disabled until the session is rebuilt\r\n", op, code);
    g_radar_fault = 1;
}

static void radar_destroy_swapchain(void) {
    if (g_radar_swap && pfn_DestroySwapchain) {
        XrResult r = pfn_DestroySwapchain(g_radar_swap);
        if (XR_FAILED(r)) InterlockedIncrement(&g_radar_st_errors);
    }
    g_radar_swap = XR_NULL_HANDLE;
    free(g_radar_images); g_radar_images = NULL; g_radar_image_count = 0;
    g_radar_sw = g_radar_sh = 0; g_radar_swap_format = 0; g_radar_copied_id = 0;
    InterlockedExchange(&g_radar_st_size, 0); InterlockedExchange(&g_radar_st_format, 0);
}

/* From xr_teardown, while the session still exists. A new session starts clean. */
static void radar_teardown(void) {
    radar_destroy_swapchain();
    g_radar_fault = 0;
    dg_radar_gate_reset(&g_radar_gate);
    InterlockedExchange64(&g_radar_available_ms, 0);
    InterlockedExchange(&g_radar_st_why, RADAR_OFF);
    if (g_cap_cs_ready) {
        EnterCriticalSection(&g_cap_cs);
        if (g_radar_store) { g_radar_store->lpVtbl->Release(g_radar_store); g_radar_store = NULL; }
        g_radar_store_id = g_radar_store_ms = 0;
        LeaveCriticalSection(&g_cap_cs);
    }
}

/* Same size and colour family as the store, so CopyResource needs no blit.
   UNORM before SRGB - the eye swapchains' choice, so the radar is as bright
   as the picture it was cut out of. */
static int radar_ensure_swapchain(uint32_t w, uint32_t h, DXGI_FORMAT store_format) {
    int group = dg_capture_format_group(store_format);
    int64_t *formats = NULL, want = 0, first, second;
    uint32_t count = 0, got = 0, i;
    XrSwapchainCreateInfo ci;
    XrResult r;
    if (g_radar_swap) {
        if (g_radar_sw == w && g_radar_sh == h &&
            dg_capture_format_group((DXGI_FORMAT)g_radar_swap_format) == group) return 1;
        radar_destroy_swapchain();          /* resolution change; no image is ever held across frames */
    }
    if (!group || !w || !h || !g_sess || !g_dev || !g_ctx || !pfn_EnumScFormats || !pfn_CreateSwapchain ||
        !pfn_EnumScImages || !pfn_DestroySwapchain) { radar_fail("swapchain preconditions", 0); return 0; }
    first = group == 1 ? DXGI_FORMAT_B8G8R8A8_UNORM : DXGI_FORMAT_R8G8B8A8_UNORM;
    second = group == 1 ? DXGI_FORMAT_B8G8R8A8_UNORM_SRGB : DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    r = pfn_EnumScFormats(g_sess, 0, &count, NULL);
    if (XR_FAILED(r) || !count || count > 4096) { radar_fail("format count", (long)r); return 0; }
    formats = (int64_t *)calloc(count, sizeof(*formats));
    if (!formats) { radar_fail("format list allocation", 0); return 0; }
    r = pfn_EnumScFormats(g_sess, count, &got, formats);
    if (XR_SUCCEEDED(r) && got <= count) {
        for (i = 0; i < got && !want; i++) if (formats[i] == first) want = first;
        for (i = 0; i < got && !want; i++) if (formats[i] == second) want = second;
    }
    free(formats);
    if (XR_FAILED(r)) { radar_fail("formats", (long)r); return 0; }
    if (!want) { radar_fail("no swapchain format in the texture's colour family", (long)store_format); return 0; }
    memset(&ci, 0, sizeof(ci)); ci.type = XR_TYPE_SWAPCHAIN_CREATE_INFO;
    ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
    ci.format = want; ci.sampleCount = 1; ci.width = w; ci.height = h;
    ci.faceCount = ci.arraySize = ci.mipCount = 1;
    r = pfn_CreateSwapchain(g_sess, &ci, &g_radar_swap);
    if (XR_FAILED(r) || !g_radar_swap) { g_radar_swap = XR_NULL_HANDLE; radar_fail("create swapchain", (long)r); return 0; }
    count = 0;
    r = pfn_EnumScImages(g_radar_swap, 0, &count, NULL);
    if (XR_FAILED(r) || !count || count > 256) { radar_fail("image count", (long)r); radar_destroy_swapchain(); return 0; }
    g_radar_images = (XrSwapchainImageD3D11KHR *)calloc(count, sizeof(*g_radar_images));
    if (!g_radar_images) { radar_fail("image list allocation", 0); radar_destroy_swapchain(); return 0; }
    for (i = 0; i < count; i++) g_radar_images[i].type = XR_TYPE_SWAPCHAIN_IMAGE_D3D11_KHR;
    got = 0;
    r = pfn_EnumScImages(g_radar_swap, count, &got, (XrSwapchainImageBaseHeader *)g_radar_images);
    if (XR_FAILED(r) || !got || got > count) { radar_fail("images", (long)r); radar_destroy_swapchain(); return 0; }
    g_radar_image_count = got; g_radar_sw = w; g_radar_sh = h; g_radar_swap_format = want; g_radar_copied_id = 0;
    InterlockedExchange(&g_radar_st_size, (LONG)(((w & 0xffff) << 16) | (h & 0xffff)));
    InterlockedExchange(&g_radar_st_format, (LONG)want);
    if (g_log) g_log("  xr radar: swapchain %ux%u fmt %lld (texture fmt %d), %u images\r\n",
                     w, h, (long long)want, (int)store_format, got);
    return 1;
}

/* Store -> swapchain image, only when the store holds a capture the swapchain
   has not seen. A failed wait leaves its image to session teardown (it may not
   be released); every other path releases what it acquired. */
static int radar_copy(uint64_t id) {
    XrSwapchainImageAcquireInfo ai = { XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO };
    XrSwapchainImageWaitInfo wi = { XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO };
    XrSwapchainImageReleaseInfo ri = { XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO };
    uint32_t index = 0;
    int copied = 0;
    XrResult r;
    if (id == g_radar_copied_id) return 1;
    if (!pfn_AcquireImage || !pfn_WaitImage || !pfn_ReleaseImage) { radar_fail("entry points", 0); return 0; }
    r = pfn_AcquireImage(g_radar_swap, &ai, &index);
    if (XR_FAILED(r)) { radar_fail("acquire", (long)r); return 0; }
    wi.timeout = 50000000;                  /* 50 ms in ns: never park the frame loop on a side layer */
    r = pfn_WaitImage(g_radar_swap, &wi);
    if (XR_FAILED(r) || r == XR_TIMEOUT_EXPIRED) { radar_fail("wait", (long)r); return 0; }
    EnterCriticalSection(&g_cap_cs);
    if (g_radar_store && g_radar_images && index < g_radar_image_count && g_radar_images[index].texture && g_dev && g_ctx) {
        ID3D11Texture2D *dst = g_radar_images[index].texture;
        D3D11_TEXTURE2D_DESC sd, dd;
        ID3D11Device *so = NULL, *dn = NULL;
        g_radar_store->lpVtbl->GetDesc(g_radar_store, &sd); dst->lpVtbl->GetDesc(dst, &dd);
        g_radar_store->lpVtbl->GetDevice(g_radar_store, &so); dst->lpVtbl->GetDevice(dst, &dn);
        if (so == g_dev && dn == g_dev && sd.Width == dd.Width && sd.Height == dd.Height &&
            sd.MipLevels == dd.MipLevels && sd.ArraySize == dd.ArraySize &&
            sd.SampleDesc.Count == dd.SampleDesc.Count && sd.SampleDesc.Quality == dd.SampleDesc.Quality &&
            dg_capture_format_compatible(sd.Format, dd.Format)) {
            g_ctx->lpVtbl->CopyResource(g_ctx, (ID3D11Resource *)dst, (ID3D11Resource *)g_radar_store);
            copied = 1;
        }
        if (so) so->lpVtbl->Release(so);
        if (dn) dn->lpVtbl->Release(dn);
    }
    LeaveCriticalSection(&g_cap_cs);
    r = pfn_ReleaseImage(g_radar_swap, &ri);
    if (XR_FAILED(r)) { radar_fail("release", (long)r); return 0; }
    if (!copied) { radar_fail("store and swapchain image do not match", 0); return 0; }
    g_radar_copied_id = id;
    return 1;
}

static void radar_pose_from_raw(const DG_XR_RAW_POSE *in, DG_RADAR_POSE *out) {
    out->qx = in->qx; out->qy = in->qy; out->qz = in->qz; out->qw = in->qw;
    out->px = in->px; out->py = in->py; out->pz = in->pz;
}

static int radar_skip(int why) { InterlockedExchange(&g_radar_st_why, why); return 0; }

/* XR thread, once per frame, after the projection layer has been decided and
   BEFORE radial_append. Fills *quad and returns 1 when the radar is to be
   shown this frame; radar_insert then slots it in. Two steps because the
   radial menu's own readiness test wants to find the projection alone. */
static int radar_prepare(const XrFrameEndInfo *end, XrCompositionLayerQuad *quad, int should_render,
                         int theater, const DG_XR_FRAME *frame, int frame_status, uint64_t now) {
    int mode = g_cfg.radar_mode, open = 1;
    uint32_t w, h;
    uint64_t id, ms;
    DXGI_FORMAT format;
    DG_RADAR_POSE offset;
    double width;
    if (mode != 1 && mode != 2) {
        if (g_radar_gate.open || g_radar_gate.timing) dg_radar_gate_reset(&g_radar_gate);
        return radar_skip(RADAR_OFF);
    }
    if (g_radar_fault) return radar_skip(RADAR_FAULT);
    if (!g_session_running || g_state != XR_SESSION_STATE_FOCUSED || !should_render) {
        dg_radar_gate_reset(&g_radar_gate); return radar_skip(RADAR_SESSION);
    }
    /* Only ever ON TOP OF the stereo/mono projection: never over the theater
       quad (menus, cutscenes) and never as the only layer. */
    if (theater || end->layerCount != 1 || !end->layers ||
        end->layers[0]->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) {
        dg_radar_gate_reset(&g_radar_gate); return radar_skip(RADAR_THEATER);
    }
    if (g_radial_max_layers <= end->layerCount) return radar_skip(RADAR_NO_SLOT);
    EnterCriticalSection(&g_cap_cs);
    id = g_radar_store ? g_radar_store_id : 0; ms = g_radar_store_ms;
    w = g_radar_store_w; h = g_radar_store_h; format = g_radar_store_format;
    LeaveCriticalSection(&g_cap_cs);
    /* The game renders its radar pass only while the radar is active (not in
       alert/evasion, menus, codec): an old capture is a frozen map. */
    if (!id || now < ms || now - ms > DG_RADAR_FRESH_MS) {
        dg_radar_gate_reset(&g_radar_gate); return radar_skip(RADAR_STALE);
    }
    memset(quad, 0, sizeof(*quad));
    quad->type = XR_TYPE_COMPOSITION_LAYER_QUAD;
    quad->layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;    /* premultiplied */
    quad->eyeVisibility = XR_EYE_VISIBILITY_BOTH;
    if (mode == 1) {
        /* First live check: head-locked like the radial menu (VIEW space),
           always in view, no gate. */
        if (!g_view_space) return radar_skip(RADAR_SESSION);
        quad->space = g_view_space;
        quad->pose.orientation.w = 1.0f;
        quad->pose.position.y = DG_RADAR_FIXED_Y_M; quad->pose.position.z = DG_RADAR_FIXED_Z_M;
        width = DG_RADAR_FIXED_WIDTH_M;
        dg_radar_gate_reset(&g_radar_gate);
    } else {
        const DG_XR_HAND_POSE *grip = &frame->left_hand.grip;
        DG_RADAR_POSE head, hand, local;
        double theta = 180.0, phi = 180.0;
        int valid;
        if (!(frame_status & 1) || !grip->active || !grip->tracked || !grip->position_valid ||
            !grip->orientation_valid || !raw_pose_usable(&grip->raw_local) ||
            !raw_pose_usable(&frame->head_raw) || !g_hand_runtime[HAND_LEFT].grip_space || !g_local_space) {
            dg_radar_gate_reset(&g_radar_gate); return radar_skip(RADAR_HAND);
        }
        dg_radar_offset_pose(g_cfg.radar_offset, g_cfg.radar_rot, &offset);
        radar_pose_from_raw(&frame->head_raw, &head);
        radar_pose_from_raw(&grip->raw_local, &hand);
        dg_radar_pose_compose(&hand, &offset, &local);
        /* Head and grip were both located in LOCAL for this frame's display time. */
        valid = dg_radar_gaze_angles(&head, &local, g_cfg.radar_gaze_pitch, &theta, &phi);
        open = dg_radar_gate_step(&g_radar_gate, valid, theta, phi, g_cfg.radar_gaze_deg, now);
        InterlockedExchange(&g_radar_st_theta, (LONG)theta); InterlockedExchange(&g_radar_st_phi, (LONG)phi);
        if (g_cfg.radar_space_local) {
            /* Fallback: the same pose composed by us in LOCAL (swims a little when the game hitches). */
            quad->space = g_local_space;
            quad->pose.orientation.x = (float)local.qx; quad->pose.orientation.y = (float)local.qy;
            quad->pose.orientation.z = (float)local.qz; quad->pose.orientation.w = (float)local.qw;
            quad->pose.position.x = (float)local.px; quad->pose.position.y = (float)local.py; quad->pose.position.z = (float)local.pz;
        } else {
            /* The grip action space itself is the layer's space: the runtime
               locates it at display time, so the quad stays on the hand at
               headset rate whatever the game's frame rate is. */
            quad->space = g_hand_runtime[HAND_LEFT].grip_space;
            quad->pose.orientation.x = (float)offset.qx; quad->pose.orientation.y = (float)offset.qy;
            quad->pose.orientation.z = (float)offset.qz; quad->pose.orientation.w = (float)offset.qw;
            quad->pose.position.x = (float)offset.px; quad->pose.position.y = (float)offset.py; quad->pose.position.z = (float)offset.pz;
        }
        width = g_cfg.radar_size;
        if (!(width >= 0.03 && width <= 0.50)) width = 0.09;
    }
    InterlockedExchange(&g_radar_st_gate, open);
    if (!radar_ensure_swapchain(w, h, format)) return radar_skip(g_radar_fault ? RADAR_FAULT : RADAR_SWAPCHAIN);
    /* From here the radar is on the wrist for whoever looks: the draw hook may withhold the HUD one. */
    InterlockedExchange64(&g_radar_available_ms, (LONG64)now);
    if (!open) return radar_skip(RADAR_GATE);
    if (!radar_copy(id)) { InterlockedExchange64(&g_radar_available_ms, 0); return radar_skip(RADAR_COPY); }
    quad->subImage.swapchain = g_radar_swap;
    quad->subImage.imageRect.extent.width = (int32_t)g_radar_sw;
    quad->subImage.imageRect.extent.height = (int32_t)g_radar_sh;
    quad->size.width = (float)width;
    quad->size.height = (float)(width * (double)g_radar_sh / (double)g_radar_sw);     /* the texture's aspect */
    return 1;
}

/* AFTER radial_append: directly above the projection, under the radial menu
   if that is up (layers are composited in order, so the menu wins). With no
   slot left the radar yields. */
static void radar_insert(XrFrameEndInfo *end, const XrCompositionLayerBaseHeader **layers,
                         unsigned capacity, const XrCompositionLayerQuad *quad) {
    unsigned max_layers = g_radial_max_layers < capacity ? g_radial_max_layers : capacity, i;
    if (!end->layerCount || end->layers != layers || end->layerCount >= max_layers ||
        layers[0]->type != XR_TYPE_COMPOSITION_LAYER_PROJECTION) { radar_skip(RADAR_NO_SLOT); return; }
    for (i = end->layerCount; i > 1; i--) layers[i] = layers[i - 1];
    layers[1] = (const XrCompositionLayerBaseHeader *)quad;
    end->layerCount++;
    InterlockedIncrement(&g_radar_st_submitted);
    InterlockedExchange(&g_radar_st_why, RADAR_SHOWN);
}
