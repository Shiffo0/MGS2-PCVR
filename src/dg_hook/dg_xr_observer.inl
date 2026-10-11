/* Private deferred context: never mutate the game's graphics pipeline. The
   immediate context executes with state restoration, under the capture lock.
   All cached objects are released at XR teardown. No eye texture is written. */
#include <d3dcompiler.h>
#include "dg_observer_layout.h"
#include "dg_observer_arm_guard.h"
#include "dg_observer_shader.h"
static struct {
    ID3D11DeviceContext *dc;
    ID3D11VertexShader *vs;
    ID3D11PixelShader *ps;
    ID3D11Buffer *cb;
    ID3D11SamplerState *sampler;
    ID3D11RasterizerState *rs;
    int failed;
} g_ob;
#define OB_RELEASE(p) do { if(p){(p)->lpVtbl->Release(p);(p)=NULL;} } while(0)
#include "dg_observer_capture.inl"
static void observer_free(void) {
    oc_free();
    OB_RELEASE(g_ob.dc);OB_RELEASE(g_ob.vs);OB_RELEASE(g_ob.ps);
    OB_RELEASE(g_ob.cb);OB_RELEASE(g_ob.sampler);OB_RELEASE(g_ob.rs);
    g_ob.failed=0;
}
static int observer_init(void) {
    typedef HRESULT (WINAPI *COMPILE)(LPCVOID,SIZE_T,LPCSTR,const D3D_SHADER_MACRO*,
        ID3DInclude*,LPCSTR,LPCSTR,UINT,UINT,ID3DBlob**,ID3DBlob**);
    HMODULE dll; COMPILE compile; ID3DBlob *v=NULL,*p=NULL,*err=NULL;
    D3D11_BUFFER_DESC bd; D3D11_SAMPLER_DESC sd; D3D11_RASTERIZER_DESC rd; int ok=0;
    if(g_ob.dc)return 1;
    if(g_ob.failed)return 0;
    dll=LoadLibraryExW(L"d3dcompiler_47.dll",NULL,LOAD_LIBRARY_SEARCH_SYSTEM32);
    if(!dll)goto done;
    compile=(COMPILE)GetProcAddress(dll,"D3DCompile");if(!compile)goto done;
    if(FAILED(compile(observer_shader,sizeof observer_shader-1,"observer",NULL,NULL,"vs","vs_4_0",0,0,&v,&err)))goto done;
    OB_RELEASE(err);
    if(FAILED(compile(observer_shader,sizeof observer_shader-1,"observer",NULL,NULL,"ps","ps_4_0",0,0,&p,&err)))goto done;
    if(FAILED(g_dev->lpVtbl->CreateVertexShader(g_dev,v->lpVtbl->GetBufferPointer(v),v->lpVtbl->GetBufferSize(v),NULL,&g_ob.vs)))goto done;
    if(FAILED(g_dev->lpVtbl->CreatePixelShader(g_dev,p->lpVtbl->GetBufferPointer(p),p->lpVtbl->GetBufferSize(p),NULL,&g_ob.ps)))goto done;
    memset(&bd,0,sizeof bd);bd.ByteWidth=sizeof(DG_OBSERVER_LAYOUT);bd.Usage=D3D11_USAGE_DEFAULT;bd.BindFlags=D3D11_BIND_CONSTANT_BUFFER;
    if(FAILED(g_dev->lpVtbl->CreateBuffer(g_dev,&bd,NULL,&g_ob.cb)))goto done;
    memset(&sd,0,sizeof sd);sd.Filter=D3D11_FILTER_MIN_MAG_LINEAR_MIP_POINT;
    sd.AddressU=sd.AddressV=sd.AddressW=D3D11_TEXTURE_ADDRESS_CLAMP;sd.MaxLOD=D3D11_FLOAT32_MAX;
    if(FAILED(g_dev->lpVtbl->CreateSamplerState(g_dev,&sd,&g_ob.sampler)))goto done;
    memset(&rd,0,sizeof rd);rd.FillMode=D3D11_FILL_SOLID;rd.CullMode=D3D11_CULL_NONE;rd.DepthClipEnable=TRUE;
    if(FAILED(g_dev->lpVtbl->CreateRasterizerState(g_dev,&rd,&g_ob.rs)))goto done;
    if(FAILED(g_dev->lpVtbl->CreateDeferredContext(g_dev,0,&g_ob.dc)))goto done;
    ok=1;
done:
    if(!ok && g_log)g_log("  desktop observer: shader initialization failed%s%s\r\n",err?": ":"",err?(char*)err->lpVtbl->GetBufferPointer(err):"");
    OB_RELEASE(v);OB_RELEASE(p);OB_RELEASE(err);if(dll)FreeLibrary(dll);
    if(!ok){observer_free();g_ob.failed=1;}
    return ok;
}
/* Returns 1 for a rendered desktop; 0 leaves the original desktop untouched.
   mode 0 left, 1 right, 2 both (right dominant). All modes crop to 16:9. */
int dg_xr_observer_fill(void *swapchain,int mode) {
    IDXGISwapChain *sc=(IDXGISwapChain*)swapchain;
    ID3D11Texture2D *back=NULL;ID3D11RenderTargetView *rt=NULL;
    ID3D11ShaderResourceView *srv[2]={NULL,NULL};ID3D11CommandList *list=NULL;
    D3D11_TEXTURE2D_DESC td,sd;D3D11_VIEWPORT vp;DG_OBSERVER_LAYOUT layout;
    ID3D11DeviceContext *c;uint64_t now,delta;int valid[2]={0,0},use[2],i,ok=0;



    if(!sc||!g_cap_cs_ready||mode<0||mode>2)return 0;
    if(FAILED(sc->lpVtbl->GetBuffer(sc,0,&IID_ID3D11Texture2D,(void**)&back))||!back)return 0;
    back->lpVtbl->GetDesc(back,&td);
    EnterCriticalSection(&g_cap_cs);
    if(!g_ctx||!g_dev||td.SampleDesc.Count!=1||!td.Width||!td.Height)goto done;



    now=GetTickCount64();
    for(i=0;i<2;i++)if(g_store[i].texture&&g_store[i].valid&&now-g_store[i].capture_ms<=DG_OBSERVER_MAX_AGE_MS) {
        g_store[i].texture->lpVtbl->GetDesc(g_store[i].texture,&sd);
        valid[i]=sd.Width==td.Width&&sd.Height==td.Height&&sd.Format==td.Format&&sd.SampleDesc.Count==1;
    }
    if(mode==2) {
        delta=g_store[0].capture_ms>g_store[1].capture_ms?g_store[0].capture_ms-g_store[1].capture_ms:g_store[1].capture_ms-g_store[0].capture_ms;
        if(!valid[0]||!valid[1]||delta>25||!observer_pose_pair(&g_store[0].raw,&g_store[1].raw))
            mode=valid[1]?1:0;
    }
    if(mode!=2&&!valid[mode])goto done;
    if(!observer_layout(&g_store[0].fov,&g_store[1].fov,mode,&layout)) {
        if(mode!=2||!observer_layout(&g_store[0].fov,&g_store[1].fov,1,&layout))goto done;
        mode=1;
    }
    if(mode==2)observer_arm_guard(&layout,
        &g_store[0].model_arm,&g_store[1].model_arm,
        &g_store[0].right_model_arm,&g_store[1].right_model_arm,
        &g_store[0].raw,&g_store[1].raw,g_store[0].capture_ms,g_store[1].capture_ms);
    if(!observer_init())goto done;
    use[0]=mode==2?0:mode;use[1]=mode==2?1:mode;
    for(i=0;i<2;i++)if(FAILED(g_dev->lpVtbl->CreateShaderResourceView(g_dev,(ID3D11Resource*)g_store[use[i]].texture,NULL,&srv[i])))goto done;
    if(FAILED(g_dev->lpVtbl->CreateRenderTargetView(g_dev,(ID3D11Resource*)back,NULL,&rt)))goto done;
    memset(&vp,0,sizeof vp);vp.Width=(float)td.Width;vp.Height=vp.Width*9.0f/16.0f;
    if(vp.Height>(float)td.Height){vp.Height=(float)td.Height;vp.Width=vp.Height*16.0f/9.0f;}
    vp.TopLeftX=((float)td.Width-vp.Width)*.5f;vp.TopLeftY=((float)td.Height-vp.Height)*.5f;vp.MaxDepth=1;
    c=g_ob.dc;c->lpVtbl->ClearState(c);
    c->lpVtbl->UpdateSubresource(c,(ID3D11Resource*)g_ob.cb,0,NULL,&layout,0,0);
    c->lpVtbl->OMSetRenderTargets(c,1,&rt,NULL);
    { const float black[4]={0,0,0,1};c->lpVtbl->ClearRenderTargetView(c,rt,black); }
    c->lpVtbl->RSSetViewports(c,1,&vp);c->lpVtbl->RSSetState(c,g_ob.rs);
    c->lpVtbl->IASetPrimitiveTopology(c,D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
    c->lpVtbl->VSSetShader(c,g_ob.vs,NULL,0);c->lpVtbl->PSSetShader(c,g_ob.ps,NULL,0);
    c->lpVtbl->PSSetConstantBuffers(c,0,1,&g_ob.cb);c->lpVtbl->PSSetShaderResources(c,0,2,srv);
    c->lpVtbl->PSSetSamplers(c,0,1,&g_ob.sampler);c->lpVtbl->Draw(c,3,0);



    if(SUCCEEDED(c->lpVtbl->FinishCommandList(c,FALSE,&list))&&list) {
        g_ctx->lpVtbl->ExecuteCommandList(g_ctx,list,TRUE);ok=1;
    }else { observer_free();g_ob.failed=1; }
done:




    OB_RELEASE(list);OB_RELEASE(rt);OB_RELEASE(srv[0]);OB_RELEASE(srv[1]);
    LeaveCriticalSection(&g_cap_cs);OB_RELEASE(back);return ok;
}
#undef OB_RELEASE
