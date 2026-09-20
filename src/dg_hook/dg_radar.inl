// dg_radar.inl - wrist radar, draw-hook half (2026-09-18).
//
// Measured (artifacts/radar_wrist_research_20260918): once per frame the game
// composites the Soliton radar map onto the HUD with ONE non-indexed 4-vertex
// draw whose viewport does not start at (0,0) - 690x501 at (2865,265) on a
// 3840x2160 target, i.e. 92*W/512 by 104*H/448 - depth test off, and pixel-
// shader SRV slot 0 = a static premultiplied-alpha texture exactly as large as
// that viewport. In the capture it is the only draw of the frame with an
// offset viewport. While the radar is not active (alert, evasion, menus,
// codec) the pass is not rendered at all and the texture goes stale, so the
// XR side judges freshness by the tick of the last capture.
//
// This half only RECOGNISES the draw by that structure, hands the texture to
// the XR side (dg_xr_radar_capture, which copies it into its own store) and,
// with vr_radar_hud=off, reports "do not forward" - the ui2dSkipDraw mechanism.
// The radar's frame, mode text and counter are separate HUD sprites and are
// not touched.
//
// Lock order. The XR thread takes its capture critical section and THEN calls
// D3D, which enters the device's multithread lock - the lock MultiLock holds.
// So the state is read under MultiLock in radarInspect, the lock is released
// when that function returns, and only then is the XR side called. The
// texture is kept alive by the reference GetResource handed out.

struct RadarFacts {
    UINT vertexCount = 0;
    bool haveViewport = false; float vpX = 0, vpY = 0, vpW = 0, vpH = 0;
    UINT bbW = 0, bbH = 0;
    bool depthEnable = true;
    bool srvIsTex2D = false; UINT texW = 0, texH = 0, mips = 0, samples = 0, arraySize = 0; DXGI_FORMAT format = DXGI_FORMAT_UNKNOWN;
};
// 0 = the radar composite. Otherwise why not: 1 vertex count, 2 no viewport /
// back-buffer size unknown, 3 viewport at the origin, 4 viewport not smaller
// than the back buffer, 5 depth test on, 6 SRV0 not a 2D texture, 7 texture
// size differs from the viewport by more than 1 px, 8 not BGRA8/RGBA8, 9 mips
// / MSAA / array. Ordered cheap to dear: radarInspect stops gathering at 1..4.
constexpr int radarWhyCount = 10;
bool radarFormatOk(DXGI_FORMAT f) {
    return f == DXGI_FORMAT_B8G8R8A8_UNORM || f == DXGI_FORMAT_B8G8R8A8_UNORM_SRGB || f == DXGI_FORMAT_B8G8R8A8_TYPELESS ||
           f == DXGI_FORMAT_R8G8B8A8_UNORM || f == DXGI_FORMAT_R8G8B8A8_UNORM_SRGB || f == DXGI_FORMAT_R8G8B8A8_TYPELESS;
}
int radarClassify(const RadarFacts &f) {
    if (f.vertexCount != 4) return 1;
    if (!f.haveViewport || !f.bbW || !f.bbH || !(f.vpW >= 1.0f) || !(f.vpH >= 1.0f)) return 2;
    if (f.vpX == 0.0f && f.vpY == 0.0f) return 3;
    if (!((UINT)f.vpW < f.bbW && (UINT)f.vpH < f.bbH)) return 4;
    if (f.depthEnable) return 5;
    if (!f.srvIsTex2D) return 6;
    if (f.mips != 1 || f.samples != 1 || f.arraySize != 1) return 9;
    if (!radarFormatOk(f.format)) return 8;
    UINT w = (UINT)f.vpW, h = (UINT)f.vpH;
    if ((f.texW > w ? f.texW - w : w - f.texW) > 1 || (f.texH > h ? f.texH - h : h - f.texH) > 1) return 7;
    return 0;
}

volatile LONG radarOn = 0, radarHudOff = 0, radarFirstPerson = 0;
volatile LONG radarHits = 0, radarAvailable = 0, radarSkipped = 0, radarExtra = 0, radarFramesMulti = 0, radarRefused = 0;
volatile LONG radarWhy[radarWhyCount] = {};
LONG radarFrameHits = 0;                                     // draw thread only
UINT radarLastVp[4] = {}, radarLastTex[3] = {};              // x, y, w, h / w, h, format of the last hit

#ifdef DG_DRAW_TRIAL_TEST
int (*radarTestSink)(ID3D11Texture2D *) = nullptr;           // stands in for the XR side at the desk
int radarDeliver(ID3D11Texture2D *t) { return radarTestSink ? radarTestSink(t) : -1; }
#else
extern "C" int dg_xr_radar_capture(void *d3d11_texture2d);
int radarDeliver(ID3D11Texture2D *t) { return dg_xr_radar_capture(t); }
#endif

// Reads the pipeline under MultiLock and lets go of it before returning.
int radarInspect(ID3D11DeviceContext *c, UINT vertexCount, ID3D11Texture2D **out) {
    RadarFacts f; f.vertexCount = vertexCount; f.bbW = ui2dBbW; f.bbH = ui2dBbH;
    if (vertexCount != 4) return radarClassify(f);
    MultiLock lock;
    D3D11_VIEWPORT vp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]; UINT nvp = D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
    c->RSGetViewports(&nvp, vp);
    if (nvp) { f.haveViewport = true; f.vpX = vp[0].TopLeftX; f.vpY = vp[0].TopLeftY; f.vpW = vp[0].Width; f.vpH = vp[0].Height; }
    { RadarFacts early = f; early.depthEnable = false; int why = radarClassify(early); if (why >= 1 && why <= 4) return why; }   // nearly every quad ends here
    ID3D11DepthStencilState *dss = nullptr; UINT ref = 0; c->OMGetDepthStencilState(&dss, &ref);
    if (dss) { D3D11_DEPTH_STENCIL_DESC dd{}; dss->GetDesc(&dd); f.depthEnable = dd.DepthEnable != FALSE; dss->Release(); }   // no state object = the default = depth on
    ComPtr<ID3D11Texture2D> tex;
    ID3D11ShaderResourceView *srv = nullptr; c->PSGetShaderResources(0, 1, &srv);
    if (srv) {
        ID3D11Resource *res = nullptr; srv->GetResource(&res);
        if (res) {
            D3D11_RESOURCE_DIMENSION dim; res->GetType(&dim);
            if (dim == D3D11_RESOURCE_DIMENSION_TEXTURE2D && SUCCEEDED(res->QueryInterface(IID_PPV_ARGS(&tex)))) {
                D3D11_TEXTURE2D_DESC td; tex->GetDesc(&td);
                f.srvIsTex2D = true; f.texW = td.Width; f.texH = td.Height; f.mips = td.MipLevels; f.samples = td.SampleDesc.Count; f.arraySize = td.ArraySize; f.format = td.Format;
            }
            res->Release();
        }
        srv->Release();
    }
    int why = radarClassify(f);
    if (!why) {
        radarLastVp[0] = (UINT)f.vpX; radarLastVp[1] = (UINT)f.vpY; radarLastVp[2] = (UINT)f.vpW; radarLastVp[3] = (UINT)f.vpH;
        radarLastTex[0] = f.texW; radarLastTex[1] = f.texH; radarLastTex[2] = (UINT)f.format;
        *out = tex.Detach();
    }
    return why;
}
// true = do not forward this non-indexed draw. Runs after the per-draw observers, so their counts do not
// depend on vr_radar_hud. Only the first composite of a frame is captured or withheld; a second draw that
// fits the structure is counted (extra) and left alone - it is not known which of them would be the radar.
bool radarOnDraw(ID3D11DeviceContext *c, UINT vertexCount) {
    if (!radarOn || vertexCount != 4 || c != context.Get() || stopping) return false;
    ComPtr<ID3D11Texture2D> tex;
    int why = radarInspect(c, vertexCount, &tex);             // MultiLock is taken AND released in there
    InterlockedIncrement(&radarWhy[why]);
    if (why || !tex) return false;
    if (radarFrameHits++) { InterlockedIncrement(&radarExtra); return false; }
    InterlockedIncrement(&radarHits);
    int shown = radarDeliver(tex.Get());                      // -1 refused, 0 stored, 1 stored and the wrist layer is available
    if (shown < 0) InterlockedIncrement(&radarRefused);
    if (shown > 0) InterlockedIncrement(&radarAvailable);
    // The one-shot draw trial must never find its draw withheld; it is off (requested == 0) in normal play.
    bool skip = radarHudOff && radarFirstPerson && shown > 0 &&
        InterlockedCompareExchange(&requested, 0, 0) != 1;
    if (skip) InterlockedIncrement(&radarSkipped);
    return skip;
}
// Frame boundary (Present thread = the draw thread).
void radarOnPresent() {
    if (radarFrameHits > 1) InterlockedIncrement(&radarFramesMulti);
    radarFrameHits = 0;
}
