#pragma once

#include <d3d11_1.h>
#include <wrl/client.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "dg_msaa_compare.h"
#include "dg_cb_retarget_cs.h"
#include "dg_clip_rounding.h"
using Microsoft::WRL::ComPtr;

struct SecondCameraGpu {
 ComPtr<ID3D11ComputeShader> cs;ComPtr<ID3D11Buffer> params,src,dst,bank,flags,flagsStaging;
 ComPtr<ID3D11ShaderResourceView> srv;ComPtr<ID3D11UnorderedAccessView> uav,flagsView;UINT width=0;
 static constexpr float tolerance=1e-4f;
 float sourceRounding[4]{};
 void setSourceCamera(const float S[16]){dgClipSourceRounding(S,sourceRounding);}
 static constexpr UINT flagSlots=16,flagWords=3+4*32+2*flagSlots+1;
 /* bound for one draw */
 ComPtr<ID3D11Buffer> original;ComPtr<ID3D11DeviceContext1> c1;UINT first=0,count=0;bool bound=false;
 bool init(ID3D11Device*d){
  if(cs)return true;
  if(FAILED(d->CreateComputeShader(g_cb_retarget,sizeof g_cb_retarget,nullptr,&cs)))return false;
  D3D11_BUFFER_DESC b{};b.ByteWidth=176;b.Usage=D3D11_USAGE_DYNAMIC;b.BindFlags=D3D11_BIND_CONSTANT_BUFFER;b.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
  if(FAILED(d->CreateBuffer(&b,nullptr,&params)))return false;
  b={};b.ByteWidth=flagWords*4;b.BindFlags=D3D11_BIND_UNORDERED_ACCESS;if(FAILED(d->CreateBuffer(&b,nullptr,&flags)))return false;
  D3D11_UNORDERED_ACCESS_VIEW_DESC u{};u.Format=DXGI_FORMAT_R32_UINT;u.ViewDimension=D3D11_UAV_DIMENSION_BUFFER;u.Buffer.NumElements=flagWords;
  if(FAILED(d->CreateUnorderedAccessView(flags.Get(),&u,&flagsView)))return false;
  b={};b.ByteWidth=flagWords*4;b.Usage=D3D11_USAGE_STAGING;b.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
  return SUCCEEDED(d->CreateBuffer(&b,nullptr,&flagsStaging));
 }
 void resetFlags(ID3D11DeviceContext*c){UINT zero[4]{};c->ClearUnorderedAccessViewUint(flagsView.Get(),zero);}
 /* Queue the flags readback; then readFlags until 1 (0 pending, -1 failure). */
 void queueFlags(ID3D11DeviceContext*c){c->CopyResource(flagsStaging.Get(),flags.Get());}
 int readFlags(ID3D11DeviceContext*c,UINT out[flagWords]){
  D3D11_MAPPED_SUBRESOURCE m{};HRESULT hr=c->Map(flagsStaging.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
  if(FAILED(hr))return hr==DXGI_ERROR_WAS_STILL_DRAWING?0:-1;memcpy(out,m.pData,flagWords*4);c->Unmap(flagsStaging.Get(),0);return 1;
 }
 bool size(ID3D11Device*d,UINT bytes){
  if(bytes==width)return true;
  width=0;src.Reset();dst.Reset();bank.Reset();srv.Reset();uav.Reset();
  D3D11_BUFFER_DESC b{};b.ByteWidth=bytes;b.BindFlags=D3D11_BIND_SHADER_RESOURCE;
  if(FAILED(d->CreateBuffer(&b,nullptr,&src)))return false;
  b.BindFlags=D3D11_BIND_UNORDERED_ACCESS;if(FAILED(d->CreateBuffer(&b,nullptr,&dst)))return false;
  b.BindFlags=D3D11_BIND_CONSTANT_BUFFER;if(FAILED(d->CreateBuffer(&b,nullptr,&bank)))return false;
  D3D11_SHADER_RESOURCE_VIEW_DESC s{};s.Format=DXGI_FORMAT_R32_UINT;s.ViewDimension=D3D11_SRV_DIMENSION_BUFFER;s.Buffer.NumElements=bytes/4;
  if(FAILED(d->CreateShaderResourceView(src.Get(),&s,&srv)))return false;
  D3D11_UNORDERED_ACCESS_VIEW_DESC u{};u.Format=DXGI_FORMAT_R32_UINT;u.ViewDimension=D3D11_UAV_DIMENSION_BUFFER;u.Buffer.NumElements=bytes/4;
  if(FAILED(d->CreateUnorderedAccessView(dst.Get(),&u,&uav)))return false;
  width=bytes;return true;
 }
 /* Prepares and binds the private bank for the draw about to be replayed:
    c20..23 = transpose(delta)*c20..23 when their prefix over the source camera
    (inverse `sinv`) is affine, else empty.
    `countDraw` adds the draw to the flags (once per draw, not per pass).
    Returns a refusal reason, or nullptr when bound. */
 const char*bind(ID3D11Device*d,ID3D11DeviceContext*c,const float delta[16],const float sinv[16],bool countDraw,UINT pers=20,UINT slot=0){
  if(bound)return "already_bound";
  original.Reset();first=count=0;
  c->QueryInterface(IID_PPV_ARGS(&c1));
  if(c1)c1->VSGetConstantBuffers1(0,1,&original,&first,&count);else c->VSGetConstantBuffers(0,1,&original);
  if(!original)return "missing_cb0";
  D3D11_BUFFER_DESC desc{};original->GetDesc(&desc);
  UINT begin=c1?first*16:0;
  /* gVS_Pers is constant pers..pers+3 of the bound range (c16, c20 or c24). */
  if(pers<16||pers>60||slot>=flagSlots)return "pers_index";
  const UINT need=(pers+4)*16;
  if(c1&&(first>4096||count>4096||count<pers+4))return "buffer_range";
  if(desc.ByteWidth<need||desc.ByteWidth>65536||desc.ByteWidth%16||begin>desc.ByteWidth||desc.ByteWidth-begin<need)return "buffer_range";
  if(!init(d)||!size(d,desc.ByteWidth))return "private_cb_create";
  {
   MsaaCsState keep(c);
   c->CopyResource(src.Get(),original.Get());
   D3D11_MAPPED_SUBRESOURCE m{};if(FAILED(c->Map(params.Get(),0,D3D11_MAP_WRITE_DISCARD,0,&m)))return "params_map";
   float*f=(float*)m.pData;memcpy(f,delta,64);UINT*u=(UINT*)(f+16);u[0]=begin/4;u[1]=desc.ByteWidth/4;f[18]=tolerance;u[3]=countDraw;
   memcpy(f+20,sinv,64);u[20]=pers;u[21]=slot;memcpy(f+40,sourceRounding,16);c->Unmap(params.Get(),0);
   c->CSSetShader(cs.Get(),nullptr,0);auto p=params.Get();c->CSSetConstantBuffers(0,1,&p);
   auto s=srv.Get();c->CSSetShaderResources(0,1,&s);ID3D11UnorderedAccessView*u2[2]={uav.Get(),flagsView.Get()};UINT keepCount[2]={~0u,~0u};c->CSSetUnorderedAccessViews(0,2,u2,keepCount);
   c->Dispatch((desc.ByteWidth/4+63)/64,1,1);
   ID3D11ShaderResourceView*ns=nullptr;c->CSSetShaderResources(0,1,&ns);ID3D11UnorderedAccessView*nu[2]{};c->CSSetUnorderedAccessViews(0,2,nu,keepCount);
   c->CopyResource(bank.Get(),dst.Get());
  }
  auto b=bank.Get();
  if(c1)c1->VSSetConstantBuffers1(0,1,&b,&first,&count);else c->VSSetConstantBuffers(0,1,&b);
  bound=true;return nullptr;
 }
 void restore(ID3D11DeviceContext*c){
  if(!bound)return;auto p=original.Get();
  if(c1)c1->VSSetConstantBuffers1(0,1,&p,&first,&count);else c->VSSetConstantBuffers(0,1,&p);
  bound=false;original.Reset();
 }
};

static bool dgDumpBmp(ID3D11Device*d,ID3D11DeviceContext*c,ID3D11Texture2D*ms,DXGI_FORMAT read,const char*path){
 if(!ms||!path)return false;
 D3D11_TEXTURE2D_DESC x{};ms->GetDesc(&x);
 bool bgra=read==DXGI_FORMAT_B8G8R8A8_UNORM;if(!bgra&&read!=DXGI_FORMAT_R8G8B8A8_UNORM)return false;
 D3D11_TEXTURE2D_DESC r=x;r.SampleDesc.Count=1;r.SampleDesc.Quality=0;r.Format=read;r.BindFlags=0;r.CPUAccessFlags=0;r.MiscFlags=0;r.Usage=D3D11_USAGE_DEFAULT;
 ComPtr<ID3D11Texture2D> resolved,staging;if(FAILED(d->CreateTexture2D(&r,nullptr,&resolved)))return false;
 r.Usage=D3D11_USAGE_STAGING;r.CPUAccessFlags=D3D11_CPU_ACCESS_READ;if(FAILED(d->CreateTexture2D(&r,nullptr,&staging)))return false;
 if(x.SampleDesc.Count>1)c->ResolveSubresource(resolved.Get(),0,ms,0,read);else c->CopyResource(resolved.Get(),ms);
 c->CopyResource(staging.Get(),resolved.Get());
 D3D11_MAPPED_SUBRESOURCE m{};if(FAILED(c->Map(staging.Get(),0,D3D11_MAP_READ,0,&m)))return false;
 FILE*f=nullptr;bool ok=false;
 if(!fopen_s(&f,path,"wb")&&f){
  UINT row=x.Width*4;unsigned char h[54]{};UINT size=54+row*x.Height;
  h[0]='B';h[1]='M';memcpy(h+2,&size,4);UINT off=54;memcpy(h+10,&off,4);UINT hs=40;memcpy(h+14,&hs,4);
  INT w=(INT)x.Width,ht=-(INT)x.Height;memcpy(h+18,&w,4);memcpy(h+22,&ht,4);h[26]=1;h[28]=32;UINT img=row*x.Height;memcpy(h+34,&img,4);
  ok=fwrite(h,1,54,f)==54;
  unsigned char*line=(unsigned char*)malloc(row);
  for(UINT y=0;ok&&line&&y<x.Height;y++){const unsigned char*s=(const unsigned char*)m.pData+y*m.RowPitch;
   if(bgra)memcpy(line,s,row);else for(UINT i=0;i<x.Width;i++){line[i*4]=s[i*4+2];line[i*4+1]=s[i*4+1];line[i*4+2]=s[i*4];line[i*4+3]=s[i*4+3];}
   ok=fwrite(line,1,row,f)==row;}
  free(line);fclose(f);if(!line)ok=false;
 }
 c->Unmap(staging.Get(),0);return ok;
}
