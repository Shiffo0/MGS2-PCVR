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
static void dg_state_sample(ID3D11DeviceContext *c, DG_STATE_SAMPLE *s) {
    memset(s,0,sizeof *s);
    c->lpVtbl->IAGetVertexBuffers(c,0,1,&s->vb,&s->stride,&s->offset);
    c->lpVtbl->IAGetIndexBuffer(c,&s->ib,&s->ib_format,&s->ib_offset);
    c->lpVtbl->IAGetInputLayout(c,&s->layout);
    c->lpVtbl->IAGetPrimitiveTopology(c,&s->topology);
    c->lpVtbl->GSGetShader(c,&s->gs,NULL,NULL);
    c->lpVtbl->GSGetConstantBuffers(c,0,1,&s->gs_cb);
    c->lpVtbl->OMGetRenderTargets(c,D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT,s->rt,&s->depth);
    c->lpVtbl->RSGetState(c,&s->raster);
    s->viewport_count=D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;
    c->lpVtbl->RSGetViewports(c,&s->viewport_count,s->viewports);
}
static void dg_state_sample_release(DG_STATE_SAMPLE *s) {
    UINT i;
#define DG_SR(x) if(s->x) s->x->lpVtbl->Release(s->x)
    DG_SR(vb); DG_SR(ib); DG_SR(gs_cb); DG_SR(layout); DG_SR(gs);
    DG_SR(depth); DG_SR(raster);
    for(i=0;i<D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT;i++) { DG_SR(rt[i]); }
#undef DG_SR
}
/* 1 = sampled bindings restored, 0 = mismatch, negative = unsupported/refused.
 * All fallible setup precedes the swap. No commands between swap and restore. */
static int dg_state_roundtrip(ID3D11Device *device) {
    ID3D11Device1 *d=NULL;
    ID3D11DeviceContext *base=NULL;
    ID3D11DeviceContext1 *c=NULL;
    ID3DDeviceContextState *scratch=NULL,*original=NULL,*returned=NULL;
    D3D_FEATURE_LEVEL level;
    DG_STATE_SAMPLE before,after;
    HRESULT hr;
    int result=-1;
    if(!device) return -2;
    hr=device->lpVtbl->QueryInterface(device,&IID_ID3D11Device1,(void**)&d);
    if(FAILED(hr)) goto done;
    device->lpVtbl->GetImmediateContext(device,&base);
    if(!base) goto done;
    hr=base->lpVtbl->QueryInterface(base,&IID_ID3D11DeviceContext1,(void**)&c);
    if(FAILED(hr)) goto done;
    level=device->lpVtbl->GetFeatureLevel(device);
    hr=d->lpVtbl->CreateDeviceContextState(d,
        (device->lpVtbl->GetCreationFlags(device)&D3D11_CREATE_DEVICE_SINGLETHREADED)?D3D11_1_CREATE_DEVICE_CONTEXT_STATE_SINGLETHREADED:0,
        &level,1,D3D11_SDK_VERSION,&IID_ID3D11Device,NULL,&scratch);
    if(FAILED(hr)||!scratch) goto done;
    dg_state_sample(base,&before);
    c->lpVtbl->SwapDeviceContextState(c,scratch,&original);
    /* Restore immediately, including a NULL/default original state. */
    c->lpVtbl->SwapDeviceContextState(c,original,&returned);
    dg_state_sample(base,&after);
    result=memcmp(&before,&after,sizeof before)==0;
    dg_state_sample_release(&after);
    dg_state_sample_release(&before);
done:
    if(returned) returned->lpVtbl->Release(returned);
    if(original) original->lpVtbl->Release(original);
    if(scratch) scratch->lpVtbl->Release(scratch);
    if(c) c->lpVtbl->Release(c);
    if(base) base->lpVtbl->Release(base);
    if(d) d->lpVtbl->Release(d);
    return result;
}
#endif
