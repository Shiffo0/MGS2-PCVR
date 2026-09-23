/* Present-thread smoke test. Binding identity only, NOT resource contents,
 * game caches, hidden counters, or permission to replay a native consumer. */
#ifndef DG_STATE_ROUNDTRIP_H
#define DG_STATE_ROUNDTRIP_H
#include <d3d11_1.h>
#include <string.h>
typedef struct DG_STATE_SAMPLE {
    ID3D11Buffer *vb, *ib, *gs_cb;
    ID3D11InputLayout *layout;
    ID3D11GeometryShader *gs;
    ID3D11RenderTargetView *rt[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT];
    ID3D11DepthStencilView *depth;
    ID3D11RasterizerState *raster;
    UINT stride, offset, ib_offset, viewport_count;
    DXGI_FORMAT ib_format;
    D3D11_PRIMITIVE_TOPOLOGY topology;
    D3D11_VIEWPORT viewports[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE];
} DG_STATE_SAMPLE;





















/* 1 = sampled bindings restored, 0 = mismatch, negative = unsupported/refused.
 * All fallible setup precedes the swap. No commands between swap and restore. */






































#endif
