#pragma once

/* Right-multiply by the port's clip conversion Q. */
static void dgGameClip(const float in[16],float out[16]){
 for(int r=0;r<4;r++){const float*a=in+r*4;float*o=out+r*4;o[0]=a[0];o[1]=-a[1];o[2]=0.5f*a[2]+0.5f*a[3];o[3]=a[3];}
}
#include "dg_draw_trial_core.h"
#include "dg_geometry_probe.h"
#include "dg_second_camera_gpu.h"

struct FramePass {
 ComPtr<ID3D11Texture2D> color,depth;ComPtr<ID3D11RenderTargetView> rt;ComPtr<ID3D11DepthStencilView> ds;
 ComPtr<ID3D11ShaderResourceView> depthView;
};
enum FrameStatus { FRAME_NONE, FRAME_OPEN, FRAME_ABORTED, FRAME_CLOSED };
struct FrameVerdict {
 const char*status="INSUFFICIENT";GeoVerdict geo;bool pass=false;
 UINT checked=0,otherCamera=0;double contradictionShare=0,equalShare=0;
};
static const double frameMaxContradiction=0.01;   /* of the A-written pixels */

struct FrameCapture {
 FrameStatus status=FRAME_NONE;const char*stopReason=nullptr;
 D3D11_TEXTURE2D_DESC colorDesc{},depthDesc{};DXGI_FORMAT colorRead=DXGI_FORMAT_UNKNOWN;
 D3D11_RENDER_TARGET_VIEW_DESC rtvDesc{};D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc{};
 FramePass a,b;ComPtr<ID3D11Texture2D> cleared,native;ComPtr<ID3D11ShaderResourceView> clearedView,nativeView;
 ComPtr<ID3D11RenderTargetView> gameRT;ComPtr<ID3D11DepthStencilView> gameDS;ComPtr<ID3D11Texture2D> gameDepth;
 SecondCameraGpu gpu;float source[16]{},sinvF[16]{},delta[16]{},identity[16]{};bool lessNearer=true;float clearDepth=1;
 double sinv[16]{},m[16]{},minv[16]{},camera[3]{};UINT replayed=0;
 bool gameClip=true;
 GeometryProbe probe;UINT raw[GEO_WORDS]{},flags[SecondCameraGpu::flagWords]{};bool probed=false,probeRead=false,flagsRead=false;

 bool makePass(ID3D11Device*d,FramePass&p,DXGI_FORMAT colorTypeless){
  auto x=colorDesc;x.Format=colorTypeless;x.BindFlags=D3D11_BIND_RENDER_TARGET;x.CPUAccessFlags=0;x.MiscFlags=0;x.Usage=D3D11_USAGE_DEFAULT;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&p.color))||FAILED(d->CreateRenderTargetView(p.color.Get(),&rtvDesc,&p.rt)))return false;
  x=depthDesc;x.Format=DXGI_FORMAT_R24G8_TYPELESS;x.BindFlags=D3D11_BIND_DEPTH_STENCIL|D3D11_BIND_SHADER_RESOURCE;x.CPUAccessFlags=0;x.MiscFlags=0;x.Usage=D3D11_USAGE_DEFAULT;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&p.depth))||FAILED(d->CreateDepthStencilView(p.depth.Get(),&dsvDesc,&p.ds)))return false;
  D3D11_SHADER_RESOURCE_VIEW_DESC v{};v.ViewDimension=D3D11_SRV_DIMENSION_TEXTURE2DMS;v.Format=DXGI_FORMAT_R24_UNORM_X8_TYPELESS;
  return SUCCEEDED(d->CreateShaderResourceView(p.depth.Get(),&v,&p.depthView));
 }
 bool makeDepthCopy(ID3D11Device*d,ComPtr<ID3D11Texture2D>&t,ComPtr<ID3D11ShaderResourceView>&v){
  auto x=depthDesc;x.Format=DXGI_FORMAT_R24G8_TYPELESS;x.BindFlags=D3D11_BIND_SHADER_RESOURCE;x.CPUAccessFlags=0;x.MiscFlags=0;x.Usage=D3D11_USAGE_DEFAULT;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&t)))return false;
  D3D11_SHADER_RESOURCE_VIEW_DESC s{};s.ViewDimension=D3D11_SRV_DIMENSION_TEXTURE2DMS;s.Format=DXGI_FORMAT_R24_UNORM_X8_TYPELESS;
  return SUCCEEDED(d->CreateShaderResourceView(t.Get(),&s,&v));
 }
 /* At the first selected draw, with the game's targets bound. S/T: the
    consumed camera pair. Returns a refusal reason or nullptr. */
 const char*begin(ID3D11Device*d,ID3D11DeviceContext*c,const float S[16],const float T[16]){
  if(status!=FRAME_NONE)return "already_started";
  ID3D11RenderTargetView*rts[8]{};ID3D11DepthStencilView*ds=nullptr;c->OMGetRenderTargets(8,rts,&ds);
  gameRT.Attach(rts[0]);gameDS.Attach(ds);bool extra=false;for(int i=1;i<8;i++)if(rts[i]){extra=true;rts[i]->Release();}
  if(extra||!gameRT||!gameDS)return "render_target_count";
  gameRT->GetDesc(&rtvDesc);gameDS->GetDesc(&dsvDesc);
  if(rtvDesc.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2DMS||dsvDesc.ViewDimension!=D3D11_DSV_DIMENSION_TEXTURE2DMS||dsvDesc.Flags)return "frame_requires_msaa";
  ComPtr<ID3D11Resource>r;ComPtr<ID3D11Texture2D>ct;gameRT->GetResource(&r);if(FAILED(r.As(&ct)))return "color_texture";ct->GetDesc(&colorDesc);
  r.Reset();gameDS->GetResource(&r);if(FAILED(r.As(&gameDepth)))return "depth_texture";gameDepth->GetDesc(&depthDesc);
  if(colorDesc.Width!=depthDesc.Width||colorDesc.Height!=depthDesc.Height||colorDesc.SampleDesc.Count!=depthDesc.SampleDesc.Count||colorDesc.SampleDesc.Count<2)return "target_mismatch";
  DXGI_FORMAT typeless,read,stencil,dt,dr,ds2;
  if(!msaaFormats(colorDesc.Format,false,typeless,read,stencil)||!msaaFormats(depthDesc.Format,true,dt,dr,ds2))return "format";
  colorRead=read;
  /* Clear value from the pass's depth function. */
  ComPtr<ID3D11DepthStencilState>dss;UINT ref=0;c->OMGetDepthStencilState(&dss,&ref);
  D3D11_COMPARISON_FUNC fn=D3D11_COMPARISON_LESS;if(dss){D3D11_DEPTH_STENCIL_DESC x{};dss->GetDesc(&x);fn=x.DepthFunc;}
  if(fn==D3D11_COMPARISON_LESS||fn==D3D11_COMPARISON_LESS_EQUAL){lessNearer=true;clearDepth=1;}
  else if(fn==D3D11_COMPARISON_GREATER||fn==D3D11_COMPARISON_GREATER_EQUAL){lessNearer=false;clearDepth=0;}
  else return "depth_func";
  /* Budget: two colour and four depth multisampled textures. */
  unsigned long long need=6ull*colorDesc.Width*colorDesc.Height*colorDesc.SampleDesc.Count*4ull;
  {ComPtr<IDXGIDevice>dx;ComPtr<IDXGIAdapter>ad;ComPtr<IDXGIAdapter3>a3;DXGI_QUERY_VIDEO_MEMORY_INFO mi{};
   if(FAILED(d->QueryInterface(IID_PPV_ARGS(&dx)))||FAILED(dx->GetAdapter(&ad))||FAILED(ad.As(&a3))||FAILED(a3->QueryVideoMemoryInfo(0,DXGI_MEMORY_SEGMENT_GROUP_LOCAL,&mi)))return "msaa_budget_unknown";
   if(mi.CurrentUsage>mi.Budget||mi.Budget-mi.CurrentUsage<2*need)return "msaa_budget";}
  if(!makePass(d,a,typeless)||!makePass(d,b,typeless)||!makeDepthCopy(d,cleared,clearedView)||!makeDepthCopy(d,native,nativeView))return "allocation";
  /* Private targets never alias the game's. */
  if(a.color.Get()==ct.Get()||b.color.Get()==ct.Get()||a.depth.Get()==gameDepth.Get()||b.depth.Get()==gameDepth.Get())return "aliasing";
  float black[4]{};for(FramePass*p:{&a,&b}){c->ClearRenderTargetView(p->rt.Get(),black);c->ClearDepthStencilView(p->ds.Get(),D3D11_CLEAR_DEPTH|D3D11_CLEAR_STENCIL,clearDepth,0);}
  c->CopyResource(cleared.Get(),a.depth.Get());
  float Sg[16],Tg[16];if(gameClip){dgGameClip(S,Sg);dgGameClip(T,Tg);}else{memcpy(Sg,S,64);memcpy(Tg,T,64);}
  if(!geoMatrices(Sg,Tg,sinv,m,camera,minv))return "camera_matrices";
  double dd[16];if(!dg_second_camera_delta(Sg,Tg,dd))return "camera_matrices";
  for(int i=0;i<16;i++){delta[i]=(float)dd[i];identity[i]=(i%5)==0;source[i]=Sg[i];sinvF[i]=(float)sinv[i];}
  if(gameClip)gpu.setSourceCamera(Sg);else memset(gpu.sourceRounding,0,sizeof gpu.sourceRounding);
  if(!gpu.init(d)||!probe.init(d))return "compute_create";
  gpu.resetFlags(c);status=FRAME_OPEN;return nullptr;
 }
 /* After the native draw: the two replays. `draw` issues the same draw call.
    Any refusal or state difference aborts the capture (no PASS possible). */
 /* pers: constant index of the shader's gVS_Pers (c16/c20/c24); slot: its row
    in the per-shader counters. */
 template<class Draw> const char*replay(ID3D11Device*d,ID3D11DeviceContext*c,Draw draw,UINT pers=20,UINT slot=0){
  if(status!=FRAME_OPEN)return "not_open";
  ID3D11RenderTargetView*rts[8]{};ID3D11DepthStencilView*ds=nullptr;c->OMGetRenderTargets(8,rts,&ds);
  bool same=rts[0]==gameRT.Get()&&ds==gameDS.Get();for(int i=1;i<8;i++)if(rts[i])same=false;
  if(!same){for(auto v:rts)if(v)v->Release();if(ds)ds->Release();return "other_target";}   /* skipped, not an abort */
  /* A replay may only write the private targets: no stream output, no OM UAVs. */
  {ID3D11Buffer*so[4]{};c->SOGetTargets(4,so);bool side=false;for(auto v:so)if(v){side=true;v->Release();}
   ID3D11UnorderedAccessView*u[8]{};c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,8,u);for(auto v:u)if(v){side=true;v->Release();}
   if(side){for(auto v:rts)if(v)v->Release();if(ds)ds->Release();return "side_writes";}}          /* skipped, not an abort */
  TrialPipelineState before(c);const char*why=nullptr;
  FramePass*passes[2]={&a,&b};const float*matrices[2]={identity,delta};
  for(int k=0;k<2&&!why;k++){
   why=gpu.bind(d,c,matrices[k],sinvF,k==0,pers,slot);if(why)break;
   auto rt=passes[k]->rt.Get();c->OMSetRenderTargets(1,&rt,passes[k]->ds.Get());
   draw();
   gpu.restore(c);
  }
  gpu.restore(c);c->OMSetRenderTargets(8,rts,ds);for(auto v:rts)if(v)v->Release();if(ds)ds->Release();
  if(why){stop(why);return why;}
  TrialPipelineState after(c);if(!before.equals(after)){stop("state_changed_after_replay");return stopReason;}
  ++replayed;return nullptr;
 }
 void stop(const char*why){if(status==FRAME_OPEN||status==FRAME_NONE){status=FRAME_ABORTED;stopReason=why;}}
 /* At the next Present: the game's depth as it stands, then the probe. */
 void close(ID3D11DeviceContext*c){
  if(status==FRAME_OPEN)status=FRAME_CLOSED;
  if(native&&gameDepth)c->CopyResource(native.Get(),gameDepth.Get());
 }
 /* 0 pending, 1 done, -1 failure. Call once per Present after close(). */
 int poll(ID3D11DeviceContext*c){
  if(!native)return -1;
  if(!probed){
   if(!probe.dispatch(c,clearedView.Get(),a.depthView.Get(),b.depthView.Get(),colorDesc.Width,colorDesc.Height,sinv,m,camera,minv,nativeView.Get(),lessNearer))return -1;
   gpu.queueFlags(c);probed=true;return 0;
  }
  if(!probeRead){int r=probe.read(c,raw);if(r<=0)return r;probeRead=true;}
  if(!flagsRead){int r=gpu.readFlags(c,flags);if(r<=0)return r;flagsRead=true;}
  return 1;
 }
 FrameVerdict verdict()const{
  FrameVerdict v;GeoTotals t;t.add(raw);v.geo=geoJudge(t);GeoRaw r(raw);
  v.checked=flags[0];v.otherCamera=flags[1];
  if(r.sameWritten){v.contradictionShare=(double)r.contradictions/r.sameWritten;v.equalShare=(double)r.sameEqual/r.sameWritten;}
  if(status==FRAME_ABORTED)v.status="ABORTED";
  else if(status!=FRAME_CLOSED||!replayed)v.status="INSUFFICIENT";
  else if(!r.sameWritten||v.contradictionShare>frameMaxContradiction)v.status="INVALID";
  else if(!v.geo.complete)v.status="INSUFFICIENT";
  else v.status=v.geo.pass?"PASS":"FAIL";
  v.pass=!strcmp(v.status,"PASS");
  return v;
 }
};
