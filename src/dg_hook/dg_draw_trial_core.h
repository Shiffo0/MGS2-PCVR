/* One direct D3D draw only. No native game re-entry. */
#pragma once
#include <d3d11_4.h>
#include <wrl/client.h>
#include <cstring>
using Microsoft::WRL::ComPtr;
#include "dg_trial_pipeline_state.h"
#include "dg_second_camera_buffer.h"
#include "dg_msaa_compare.h"
struct TrialTarget {
 ComPtr<ID3D11Texture2D> source,copy,initial,normal,repeat,sourceAfter;
 D3D11_TEXTURE2D_DESC desc{};
 /* Multisampled: initial/normal/sourceAfter and the replay copy stay
    multisampled; `repeat` is the copy itself. Compared on the GPU. */
 bool msaa=false;MsaaCompare gpu;int evaluation=0;UINT counts[MSAA_PAIRS*2]{};
 ComPtr<ID3D11ShaderResourceView> views[MSAA_PAIRS*4];
 bool prepareMsaa(ID3D11Device*d,bool depth){
  DXGI_FORMAT typeless,read,stencil;
  if(!msaaFormats(desc.Format,depth,typeless,read,stencil))return false;
  if(!desc.Width||!desc.Height||desc.Width>4096||desc.Height>4096||desc.ArraySize!=1||desc.MipLevels!=1||
     desc.SampleDesc.Count>8||desc.MiscFlags||desc.Usage!=D3D11_USAGE_DEFAULT)return false;
  auto x=desc;x.Format=typeless;x.CPUAccessFlags=0;x.MiscFlags=0;
  x.BindFlags=(depth?D3D11_BIND_DEPTH_STENCIL:D3D11_BIND_RENDER_TARGET)|D3D11_BIND_SHADER_RESOURCE;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&copy)))return false;
  x.BindFlags=D3D11_BIND_SHADER_RESOURCE;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&initial))||FAILED(d->CreateTexture2D(&x,nullptr,&normal))||
     FAILED(d->CreateTexture2D(&x,nullptr,&sourceAfter)))return false;
  /* Pairs: replay (normal,copy), contribution (initial,normal), source (normal,sourceAfter). */
  ID3D11Texture2D*pairs[MSAA_PAIRS][2]={{normal.Get(),copy.Get()},{initial.Get(),normal.Get()},{normal.Get(),sourceAfter.Get()}};
  UINT per=depth?4:2;
  for(UINT p=0;p<MSAA_PAIRS;p++)for(UINT k=0;k<2;k++) {
   D3D11_SHADER_RESOURCE_VIEW_DESC v{};v.ViewDimension=D3D11_SRV_DIMENSION_TEXTURE2DMS;v.Format=read;
   if(FAILED(d->CreateShaderResourceView(pairs[p][k],&v,&views[p*per+k*(depth?2:1)])))return false;
   if(depth){v.Format=stencil;if(FAILED(d->CreateShaderResourceView(pairs[p][k],&v,&views[p*per+k*2+1])))return false;}
  }
  msaa=true;return gpu.init(d,depth);
 }
 /* 0 pending, 1 counts ready, -1 failure. The first call queues the work. */
 int evaluate(ID3D11DeviceContext*c){
  if(evaluation==2)return 1;
  if(evaluation==0){
   ID3D11ShaderResourceView*v[MSAA_PAIRS*4]{};for(UINT i=0;i<MSAA_PAIRS*4;i++)v[i]=views[i].Get();
   if(!gpu.dispatch(c,v,desc.Width,desc.Height,desc.SampleDesc.Count))return -1;
   evaluation=1;return 0;
  }
  int r=gpu.read(c,counts);if(r==1)evaluation=2;return r;
 }
 void reevaluate(){evaluation=0;}   /* tests: after changing a snapshot */
 bool prepare(ID3D11Device* d,ID3D11Resource* r,bool depth) {
  if(FAILED(r->QueryInterface(IID_PPV_ARGS(&source))))return false;
  source->GetDesc(&desc);
  if(desc.SampleDesc.Count>1)return prepareMsaa(d,depth);
  bool format=depth?(desc.Format==DXGI_FORMAT_R24G8_TYPELESS||desc.Format==DXGI_FORMAT_D24_UNORM_S8_UINT||desc.Format==DXGI_FORMAT_R32_TYPELESS||desc.Format==DXGI_FORMAT_D32_FLOAT):
   (desc.Format==DXGI_FORMAT_R8G8B8A8_UNORM||desc.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB||desc.Format==DXGI_FORMAT_B8G8R8A8_UNORM||desc.Format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB);
  if(!format||!desc.Width||!desc.Height||desc.Width>4096||desc.Height>4096||desc.ArraySize!=1||desc.MipLevels!=1||desc.SampleDesc.Count!=1||desc.SampleDesc.Quality||desc.MiscFlags||desc.Usage!=D3D11_USAGE_DEFAULT)return false;
  auto x=desc;x.BindFlags=depth?D3D11_BIND_DEPTH_STENCIL:D3D11_BIND_RENDER_TARGET;x.CPUAccessFlags=0;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&copy)))return false;
  x.Usage=D3D11_USAGE_STAGING;x.BindFlags=0;x.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
  return SUCCEEDED(d->CreateTexture2D(&x,nullptr,&initial))&&SUCCEEDED(d->CreateTexture2D(&x,nullptr,&normal))&&SUCCEEDED(d->CreateTexture2D(&x,nullptr,&repeat))&&SUCCEEDED(d->CreateTexture2D(&x,nullptr,&sourceAfter));
 }
 // Compare original target immediately after normal draw versus after replay.
 int compareSource(ID3D11DeviceContext*c,unsigned long long& mismatch){
  D3D11_MAPPED_SUBRESOURCE a{},b{};mismatch=0;
  if(msaa){int e=evaluate(c);if(e==1)mismatch=counts[MSAA_PAIR_SOURCE*2];return e;}
  HRESULT hr=c->Map(normal.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&a);
  if(FAILED(hr))return hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;
  hr=c->Map(sourceAfter.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&b);
  if(SUCCEEDED(hr)){for(UINT y=0;y<desc.Height;y++)for(UINT x=0;x<desc.Width*4;x++)mismatch+=((BYTE*)a.pData)[y*a.RowPitch+x]!=((BYTE*)b.pData)[y*b.RowPitch+x];c->Unmap(sourceAfter.Get(),0);}
  c->Unmap(normal.Get(),0);return SUCCEEDED(hr)?1:hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;
 }
 // Read only normal-vs-initial snapshots. Never changes the source target.
 int contribution(ID3D11DeviceContext*c,bool& changed,unsigned* changedPixels=nullptr){
  D3D11_MAPPED_SUBRESOURCE a{},b{};changed=false;unsigned pixels=0;if(changedPixels)*changedPixels=0;
  if(msaa){int e=evaluate(c);if(e==1){pixels=counts[MSAA_PAIR_CONTRIBUTION*2+1];changed=pixels!=0;if(changedPixels)*changedPixels=pixels;}return e;}
  HRESULT hr=c->Map(initial.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&a);
  if(FAILED(hr))return hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;
  hr=c->Map(normal.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&b);
  if(SUCCEEDED(hr)){
   for(UINT y=0;y<desc.Height;y++){
    const BYTE* ar=(BYTE*)a.pData+y*a.RowPitch;const BYTE* br=(BYTE*)b.pData+y*b.RowPitch;
    if(!memcmp(ar,br,desc.Width*4))continue;
    for(UINT x=0;x<desc.Width;x++)pixels+=memcmp(ar+x*4,br+x*4,4)!=0;
   }
   changed=pixels!=0;if(changedPixels)*changedPixels=pixels;
   c->Unmap(normal.Get(),0);
  }
  c->Unmap(initial.Get(),0);return SUCCEEDED(hr)?1:hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;
 }
 void before(ID3D11DeviceContext*c){c->CopyResource(copy.Get(),source.Get());c->CopyResource(initial.Get(),source.Get());}
 // 0 pending, -1 failure, 1 compared. Row padding is deliberately excluded.
 int compare(ID3D11DeviceContext*c,unsigned long long& mismatch,unsigned long long& changed){
  if(msaa){int e=evaluate(c);mismatch=changed=0;if(e==1){mismatch=counts[MSAA_PAIR_REPLAY*2];changed=counts[MSAA_PAIR_CONTRIBUTION*2];}return e;}
  ID3D11Texture2D* t[]={initial.Get(),normal.Get(),repeat.Get()};D3D11_MAPPED_SUBRESOURCE m[3]{};unsigned mapped=0;HRESULT hr=S_OK;
  for(;mapped<3;mapped++){hr=c->Map(t[mapped],0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m[mapped]);if(FAILED(hr))break;}
  mismatch=changed=0;
  if(mapped==3){for(UINT y=0;y<desc.Height;y++)for(UINT x=0;x<desc.Width*4;x++){
   auto a=((unsigned char*)m[0].pData)[y*m[0].RowPitch+x];auto b=((unsigned char*)m[1].pData)[y*m[1].RowPitch+x];auto z=((unsigned char*)m[2].pData)[y*m[2].RowPitch+x];mismatch+=b!=z;changed+=a!=b;
  }}
  for(unsigned i=0;i<mapped;i++)c->Unmap(t[i],0);
  return mapped==3?1:(hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1);
 }
};
/* A multisampled trial allocates four full-size multisampled textures per
   target (4K MSAA4: ~0.5 GB each for colour and depth). Refuse unless the
   adapter's local video memory budget has twice that free. */
static const char*msaaBudget(ID3D11Device*d,ID3D11Resource*r){
 ComPtr<ID3D11Texture2D>t;if(FAILED(r->QueryInterface(IID_PPV_ARGS(&t))))return nullptr;
 D3D11_TEXTURE2D_DESC x{};t->GetDesc(&x);if(x.SampleDesc.Count<=1)return nullptr;
 unsigned long long need=2ull*4ull*x.Width*x.Height*x.SampleDesc.Count*4ull; /* colour + depth */
 ComPtr<IDXGIDevice>dx;ComPtr<IDXGIAdapter>a;ComPtr<IDXGIAdapter3>a3;DXGI_QUERY_VIDEO_MEMORY_INFO m{};
 if(FAILED(d->QueryInterface(IID_PPV_ARGS(&dx)))||FAILED(dx->GetAdapter(&a))||FAILED(a.As(&a3))||
    FAILED(a3->QueryVideoMemoryInfo(0,DXGI_MEMORY_SEGMENT_GROUP_LOCAL,&m)))return "msaa_budget_unknown";
 if(m.CurrentUsage>m.Budget||m.Budget-m.CurrentUsage<2*need)return "msaa_budget";
 return nullptr;
}
struct DrawTrial {
 TrialTarget color,depth;
 ComPtr<ID3D11RenderTargetView> originalRT,trialRT;
 ComPtr<ID3D11DepthStencilView> originalDS,trialDS;
 bool noContribution=false; const char* selectionFailure=nullptr;
 bool hasDepth=false; bool omRestored=false; bool pipelineRestored=false;
 const char* prepare(ID3D11Device*d,ID3D11DeviceContext*c){
  ID3D11Predicate*p=nullptr;BOOL pred=FALSE;c->GetPredication(&p,&pred);if(p){p->Release();return "predication";}
  ID3D11Buffer* so[4]{};c->SOGetTargets(4,so);bool bad=false;for(auto v:so)if(v){bad=true;v->Release();}if(bad)return "stream_output";
  UINT slots=d->GetFeatureLevel()>=D3D_FEATURE_LEVEL_11_1?64:8;
  ID3D11UnorderedAccessView*u[64]{};c->CSGetUnorderedAccessViews(0,slots,u);for(UINT i=0;i<slots;i++)if(u[i]){bad=true;u[i]->Release();u[i]=nullptr;}
  c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,slots,u);for(UINT i=0;i<slots;i++)if(u[i]){bad=true;u[i]->Release();}if(bad)return "uav";
  ID3D11RenderTargetView*rt[8]{};ID3D11DepthStencilView*ds=nullptr;c->OMGetRenderTargets(8,rt,&ds);
  originalRT.Attach(rt[0]);originalDS.Attach(ds);for(unsigned i=1;i<8;i++)if(rt[i]){bad=true;rt[i]->Release();}if(bad||!originalRT)return "render_target_count";
  D3D11_RENDER_TARGET_VIEW_DESC rv{};originalRT->GetDesc(&rv);
  if(!(rv.ViewDimension==D3D11_RTV_DIMENSION_TEXTURE2DMS||(rv.ViewDimension==D3D11_RTV_DIMENSION_TEXTURE2D&&!rv.Texture2D.MipSlice)))return "rtv_subresource";
  ComPtr<ID3D11Resource>resource;originalRT->GetResource(&resource);
  if(const char*why=msaaBudget(d,resource.Get()))return why;
  if(!color.prepare(d,resource.Get(),false))return "color_format_or_allocation";
  if(FAILED(d->CreateRenderTargetView(color.copy.Get(),&rv,&trialRT)))return "rtv_create";
  if(originalDS){D3D11_DEPTH_STENCIL_VIEW_DESC dv{};originalDS->GetDesc(&dv);
   if(dv.Flags||!(dv.ViewDimension==D3D11_DSV_DIMENSION_TEXTURE2DMS||(dv.ViewDimension==D3D11_DSV_DIMENSION_TEXTURE2D&&!dv.Texture2D.MipSlice)))return "dsv_subresource";
   resource.Reset();originalDS->GetResource(&resource);if(!depth.prepare(d,resource.Get(),true))return "depth_format_or_allocation";
   if(depth.desc.Width!=color.desc.Width||depth.desc.Height!=color.desc.Height)return "depth_extent";
   if(depth.msaa!=color.msaa||depth.desc.SampleDesc.Count!=color.desc.SampleDesc.Count)return "depth_samples";
   if(FAILED(d->CreateDepthStencilView(depth.copy.Get(),&dv,&trialDS)))return "dsv_create";hasDepth=true;
  }
  // Refuse target feedback through any SRV stage, including unbound read-only DSV cases.
  for(unsigned stage=0;stage<6;stage++){
   ID3D11ShaderResourceView*s[128]{};
   switch(stage){case 0:c->VSGetShaderResources(0,128,s);break;case 1:c->HSGetShaderResources(0,128,s);break;case 2:c->DSGetShaderResources(0,128,s);break;case 3:c->GSGetShaderResources(0,128,s);break;case 4:c->PSGetShaderResources(0,128,s);break;case 5:c->CSGetShaderResources(0,128,s);break;}
   for(auto v:s)if(v){ComPtr<ID3D11Resource>r;v->GetResource(&r);if(r.Get()==color.source.Get()||r.Get()==depth.source.Get())bad=true;v->Release();}
  }
  if(bad)return "target_feedback";
  if(FAILED(d->GetDeviceRemovedReason()))return "device_removed";
  return nullptr;
 }
 template<class Draw,class Verify> bool run(ID3D11DeviceContext*c,Draw draw,Verify verify,bool requireContribution=false,SecondCameraBuffer*second=nullptr){
  color.before(c);if(hasDepth)depth.before(c);
  draw(); // normal downstream API exactly once
  if(!verify())return false; // normal draw happened; never call it again on refusal
  c->CopyResource(color.normal.Get(),color.source.Get());if(hasDepth)c->CopyResource(depth.normal.Get(),depth.source.Get());
  if(requireContribution){selectionFailure="survey_only";return false;}
  TrialPipelineState stateBefore(c);
  if(second&&!second->bind(c)){selectionFailure="second_camera_binding_changed";return false;}
  struct RestoreOM {ID3D11DeviceContext*c;ID3D11RenderTargetView*r;ID3D11DepthStencilView*d;bool armed=true;
   void restore(){if(armed){c->OMSetRenderTargets(1,&r,d);armed=false;}}~RestoreOM(){restore();}
  } restore{c,originalRT.Get(),originalDS.Get()};
  auto rt=trialRT.Get();c->OMSetRenderTargets(1,&rt,trialDS.Get());
  draw(); // same captured downstream API, no native re-entry
  if(second)second->restore();
  restore.restore();
  TrialPipelineState stateAfter(c);pipelineRestored=stateBefore.equals(stateAfter);
  ID3D11RenderTargetView* checkRT[8]{};ID3D11DepthStencilView*checkDS=nullptr;
  c->OMGetRenderTargets(8,checkRT,&checkDS);omRestored=checkRT[0]==originalRT.Get()&&checkDS==originalDS.Get();
  for(unsigned i=0;i<8;++i){if(i&&checkRT[i])omRestored=false;if(checkRT[i])checkRT[i]->Release();}if(checkDS)checkDS->Release();
  c->CopyResource(color.sourceAfter.Get(),color.source.Get());if(hasDepth)c->CopyResource(depth.sourceAfter.Get(),depth.source.Get());
  if(!color.msaa)c->CopyResource(color.repeat.Get(),color.copy.Get());
  if(hasDepth&&!depth.msaa)c->CopyResource(depth.repeat.Get(),depth.copy.Get());
  return true;
 }
 template<class Draw> bool run(ID3D11DeviceContext*c,Draw draw){return run(c,draw,[](){return true;});}
};
