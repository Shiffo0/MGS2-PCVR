/* One direct D3D draw only. No native game re-entry. */
#pragma once
#include <d3d11_4.h>
#include <wrl/client.h>
#include <cstring>
using Microsoft::WRL::ComPtr;
struct TrialTarget {
 ComPtr<ID3D11Texture2D> source,copy,initial,normal,repeat;
 D3D11_TEXTURE2D_DESC desc{};
 bool prepare(ID3D11Device* d,ID3D11Resource* r,bool depth) {
  if(FAILED(r->QueryInterface(IID_PPV_ARGS(&source))))return false;
  source->GetDesc(&desc);
  bool format=depth?(desc.Format==DXGI_FORMAT_R24G8_TYPELESS||desc.Format==DXGI_FORMAT_D24_UNORM_S8_UINT||desc.Format==DXGI_FORMAT_R32_TYPELESS||desc.Format==DXGI_FORMAT_D32_FLOAT):
   (desc.Format==DXGI_FORMAT_R8G8B8A8_UNORM||desc.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB||desc.Format==DXGI_FORMAT_B8G8R8A8_UNORM||desc.Format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB);
  if(!format||!desc.Width||!desc.Height||desc.Width>4096||desc.Height>4096||desc.ArraySize!=1||desc.MipLevels!=1||desc.SampleDesc.Count!=1||desc.SampleDesc.Quality||desc.MiscFlags||desc.Usage!=D3D11_USAGE_DEFAULT)return false;
  auto x=desc;x.BindFlags=depth?D3D11_BIND_DEPTH_STENCIL:D3D11_BIND_RENDER_TARGET;x.CPUAccessFlags=0;
  if(FAILED(d->CreateTexture2D(&x,nullptr,&copy)))return false;
  x.Usage=D3D11_USAGE_STAGING;x.BindFlags=0;x.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
  return SUCCEEDED(d->CreateTexture2D(&x,nullptr,&initial))&&SUCCEEDED(d->CreateTexture2D(&x,nullptr,&normal))&&SUCCEEDED(d->CreateTexture2D(&x,nullptr,&repeat));
 }
 void before(ID3D11DeviceContext*c){c->CopyResource(copy.Get(),source.Get());c->CopyResource(initial.Get(),source.Get());}
 // 0 pending, -1 failure, 1 compared. Row padding is deliberately excluded.
 int compare(ID3D11DeviceContext*c,unsigned long long& mismatch,unsigned long long& changed){
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
struct DrawTrial {
 TrialTarget color,depth;
 ComPtr<ID3D11RenderTargetView> originalRT,trialRT;
 ComPtr<ID3D11DepthStencilView> originalDS,trialDS;
 bool hasDepth=false;
 const char* prepare(ID3D11Device*d,ID3D11DeviceContext*c){
  ID3D11Predicate*p=nullptr;BOOL pred=FALSE;c->GetPredication(&p,&pred);if(p){p->Release();return "predication";}
  ID3D11Buffer* so[4]{};c->SOGetTargets(4,so);bool bad=false;for(auto v:so)if(v){bad=true;v->Release();}if(bad)return "stream_output";
  UINT slots=d->GetFeatureLevel()>=D3D_FEATURE_LEVEL_11_1?64:8;
  ID3D11UnorderedAccessView*u[64]{};c->CSGetUnorderedAccessViews(0,slots,u);for(UINT i=0;i<slots;i++)if(u[i]){bad=true;u[i]->Release();u[i]=nullptr;}
  c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,slots,u);for(UINT i=0;i<slots;i++)if(u[i]){bad=true;u[i]->Release();}if(bad)return "uav";
  ID3D11RenderTargetView*rt[8]{};ID3D11DepthStencilView*ds=nullptr;c->OMGetRenderTargets(8,rt,&ds);
  originalRT.Attach(rt[0]);originalDS.Attach(ds);for(unsigned i=1;i<8;i++)if(rt[i]){bad=true;rt[i]->Release();}if(bad||!originalRT)return "render_target_count";
  D3D11_RENDER_TARGET_VIEW_DESC rv{};originalRT->GetDesc(&rv);if(rv.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2D||rv.Texture2D.MipSlice)return "rtv_subresource";
  ComPtr<ID3D11Resource>resource;originalRT->GetResource(&resource);
  if(!color.prepare(d,resource.Get(),false))return "color_format_or_allocation";
  if(FAILED(d->CreateRenderTargetView(color.copy.Get(),&rv,&trialRT)))return "rtv_create";
  if(originalDS){D3D11_DEPTH_STENCIL_VIEW_DESC dv{};originalDS->GetDesc(&dv);
   if(dv.ViewDimension!=D3D11_DSV_DIMENSION_TEXTURE2D||dv.Texture2D.MipSlice||dv.Flags)return "dsv_subresource";
   resource.Reset();originalDS->GetResource(&resource);if(!depth.prepare(d,resource.Get(),true))return "depth_format_or_allocation";
   if(depth.desc.Width!=color.desc.Width||depth.desc.Height!=color.desc.Height)return "depth_extent";
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
 template<class Draw,class Verify> bool run(ID3D11DeviceContext*c,Draw draw,Verify verify){
  color.before(c);if(hasDepth)depth.before(c);
  draw(); // normal downstream API exactly once
  if(!verify())return false; // normal draw happened; never call it again on refusal
  c->CopyResource(color.normal.Get(),color.source.Get());if(hasDepth)c->CopyResource(depth.normal.Get(),depth.source.Get());
  struct RestoreOM {ID3D11DeviceContext*c;ID3D11RenderTargetView*r;ID3D11DepthStencilView*d;bool armed=true;
   void restore(){if(armed){c->OMSetRenderTargets(1,&r,d);armed=false;}}~RestoreOM(){restore();}
  } restore{c,originalRT.Get(),originalDS.Get()};
  auto rt=trialRT.Get();c->OMSetRenderTargets(1,&rt,trialDS.Get());
  draw(); // same captured downstream API, no native re-entry
  restore.restore();
  c->CopyResource(color.repeat.Get(),color.copy.Get());if(hasDepth)c->CopyResource(depth.repeat.Get(),depth.copy.Get());
  return true;
 }
 template<class Draw> bool run(ID3D11DeviceContext*c,Draw draw){return run(c,draw,[](){return true;});}
};
