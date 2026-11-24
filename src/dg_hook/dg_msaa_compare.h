#pragma once

#include <d3d11_1.h>
#include <wrl/client.h>
#include "dg_msaa_compare_color_cs.h"
#include "dg_msaa_compare_depth_cs.h"
using Microsoft::WRL::ComPtr;

/* The pairs every multisampled trial target compares. */
enum { MSAA_PAIR_REPLAY=0, MSAA_PAIR_CONTRIBUTION=1, MSAA_PAIR_SOURCE=2, MSAA_PAIRS=3 };

struct MsaaCsState {
 ID3D11DeviceContext*c=nullptr;ComPtr<ID3D11DeviceContext1>c1;
 ID3D11ComputeShader*shader=nullptr;ID3D11ClassInstance*classes[256]{};UINT classCount=256;
 ID3D11Buffer*cb=nullptr;UINT first=0,count=0;bool ranged=false;
 ID3D11ShaderResourceView*srv[4]{};ID3D11UnorderedAccessView*uav[2]{};
 explicit MsaaCsState(ID3D11DeviceContext*ctx):c(ctx){
  c->CSGetShader(&shader,classes,&classCount);
  if(SUCCEEDED(c->QueryInterface(IID_PPV_ARGS(&c1)))){c1->CSGetConstantBuffers1(0,1,&cb,&first,&count);ranged=true;}
  else c->CSGetConstantBuffers(0,1,&cb);
  c->CSGetShaderResources(0,4,srv);c->CSGetUnorderedAccessViews(0,2,uav);
 }
 ~MsaaCsState(){
  UINT keep[2]={~0u,~0u};
  c->CSSetShader(shader,classes,classCount);
  if(ranged)c1->CSSetConstantBuffers1(0,1,&cb,&first,&count);else c->CSSetConstantBuffers(0,1,&cb);
  c->CSSetShaderResources(0,4,srv);c->CSSetUnorderedAccessViews(0,2,uav,keep);
  if(shader)shader->Release();for(UINT i=0;i<classCount&&i<256;i++)if(classes[i])classes[i]->Release();
  if(cb)cb->Release();for(auto v:srv)if(v)v->Release();for(auto v:uav)if(v)v->Release();
 }
};

struct MsaaCompare {
 ComPtr<ID3D11ComputeShader> cs;ComPtr<ID3D11Buffer> cb,counters,staging;ComPtr<ID3D11UnorderedAccessView> uav;
 bool depth=false;
 bool init(ID3D11Device*d,bool isDepth){
  depth=isDepth;
  if(FAILED(isDepth?d->CreateComputeShader(g_msaa_compare_depth,sizeof g_msaa_compare_depth,nullptr,&cs):
                    d->CreateComputeShader(g_msaa_compare_color,sizeof g_msaa_compare_color,nullptr,&cs)))return false;
  D3D11_BUFFER_DESC b{};b.ByteWidth=16;b.Usage=D3D11_USAGE_DYNAMIC;b.BindFlags=D3D11_BIND_CONSTANT_BUFFER;b.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
  if(FAILED(d->CreateBuffer(&b,nullptr,&cb)))return false;
  b={};b.ByteWidth=MSAA_PAIRS*8;b.BindFlags=D3D11_BIND_UNORDERED_ACCESS;b.MiscFlags=D3D11_RESOURCE_MISC_BUFFER_ALLOW_RAW_VIEWS;
  if(FAILED(d->CreateBuffer(&b,nullptr,&counters)))return false;
  D3D11_UNORDERED_ACCESS_VIEW_DESC u{};u.Format=DXGI_FORMAT_R32_TYPELESS;u.ViewDimension=D3D11_UAV_DIMENSION_BUFFER;
  u.Buffer.NumElements=MSAA_PAIRS*2;u.Buffer.Flags=D3D11_BUFFER_UAV_FLAG_RAW;
  if(FAILED(d->CreateUnorderedAccessView(counters.Get(),&u,&uav)))return false;
  b={};b.ByteWidth=MSAA_PAIRS*8;b.Usage=D3D11_USAGE_STAGING;b.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
  return SUCCEEDED(d->CreateBuffer(&b,nullptr,&staging));
 }
 /* views: per pair 2 (colour: A,B) or 4 (depth: DA,SA,DB,SB) SRVs, pairs in
    MSAA_PAIR order. Queues the comparisons and the counter readback copy. */
 bool dispatch(ID3D11DeviceContext*c,ID3D11ShaderResourceView*const*views,UINT w,UINT h,UINT samples){
  MsaaCsState keep(c);
  UINT zero[4]{};c->ClearUnorderedAccessViewUint(uav.Get(),zero);
  auto u=uav.Get();UINT none=~0u;c->CSSetUnorderedAccessViews(0,1,&u,&none);
  c->CSSetShader(cs.Get(),nullptr,0);auto b=cb.Get();c->CSSetConstantBuffers(0,1,&b);
  UINT per=depth?4:2;
  for(UINT pair=0;pair<MSAA_PAIRS;pair++){
   D3D11_MAPPED_SUBRESOURCE m{};if(FAILED(c->Map(cb.Get(),0,D3D11_MAP_WRITE_DISCARD,0,&m)))return false;
   UINT p[4]={w,h,samples,pair};memcpy(m.pData,p,sizeof p);c->Unmap(cb.Get(),0);
   c->CSSetShaderResources(0,per,views+pair*per);
   c->Dispatch((w+7)/8,(h+7)/8,1);
   ID3D11ShaderResourceView*clear[4]{};c->CSSetShaderResources(0,4,clear);
  }
  c->CopyResource(staging.Get(),counters.Get());
  return true;
 }
 /* 0 pending, 1 read (out: samples/pixels per pair, interleaved), -1 failure. */
 int read(ID3D11DeviceContext*c,UINT out[MSAA_PAIRS*2]){
  D3D11_MAPPED_SUBRESOURCE m{};HRESULT hr=c->Map(staging.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
  if(FAILED(hr))return hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;
  memcpy(out,m.pData,MSAA_PAIRS*8);c->Unmap(staging.Get(),0);return 1;
 }
};

/* Typeless family of a supported target format, and the views to read it. */
static bool msaaFormats(DXGI_FORMAT f,bool depth,DXGI_FORMAT&typeless,DXGI_FORMAT&read,DXGI_FORMAT&stencil){
 stencil=DXGI_FORMAT_UNKNOWN;
 if(depth){
  if(f!=DXGI_FORMAT_R24G8_TYPELESS&&f!=DXGI_FORMAT_D24_UNORM_S8_UINT)return false;
  typeless=DXGI_FORMAT_R24G8_TYPELESS;read=DXGI_FORMAT_R24_UNORM_X8_TYPELESS;stencil=DXGI_FORMAT_X24_TYPELESS_G8_UINT;return true;
 }
 switch(f){
  case DXGI_FORMAT_R8G8B8A8_UNORM:case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB:
   typeless=DXGI_FORMAT_R8G8B8A8_TYPELESS;read=DXGI_FORMAT_R8G8B8A8_UNORM;return true;
  case DXGI_FORMAT_B8G8R8A8_UNORM:case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB:
   typeless=DXGI_FORMAT_B8G8R8A8_TYPELESS;read=DXGI_FORMAT_B8G8R8A8_UNORM;return true;
  default:return false;
 }
}
