#pragma once
/* Multisampled draw-trial targets (live 26 Sep: the game's main pass renders
 * to 3840x2160 MSAA4, B8G8R8A8_UNORM colour and D24_UNORM_S8_UINT depth).
 * Staging textures cannot be multisampled and a resolve averages samples, so
 * snapshots stay multisampled and a compute shader compares pairs sample by
 * sample, bit-exact: colour, depth and stencil. Only the counters (differing
 * samples, pixels with any differing sample) are read back. The compute
 * bindings it touches are saved and restored exactly. */
#include <d3d11_1.h>
#include <wrl/client.h>
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
