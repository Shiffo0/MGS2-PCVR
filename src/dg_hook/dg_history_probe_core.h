#ifndef DG_HISTORY_PROBE_CORE_H
#define DG_HISTORY_PROBE_CORE_H
#include <d3d11.h>
#include <wrl/client.h>
#include <cstring>
#include <cstdint>
using Microsoft::WRL::ComPtr;
/* Two explicitly requested attempts per process. A refused/late attempt is
   consumed; no automatic retry and no reuse of an earlier token. */
struct DgHistoryArm {
 enum {IDLE,WAITING,ARMED,TAKEN};int phase=IDLE;unsigned attempts=0;
 uint64_t due=0;char seen[2][49]{};
 bool request(const char*k,uint64_t now){
  if(phase!=IDLE||attempts>=2||!k)return false;
  size_t n=strlen(k);if(!n||n>48)return false;
  for(size_t i=0;i<n;i++)if(!((k[i]>='a'&&k[i]<='z')||(k[i]>='A'&&k[i]<='Z')||(k[i]>='0'&&k[i]<='9')||k[i]=='_'||k[i]=='-'))return false;
  for(unsigned i=0;i<attempts;i++)if(!strcmp(seen[i],k))return false;
  strcpy_s(seen[attempts++],k);due=now+5000;phase=WAITING;return true;
 }
 /* 1 arm next frame, -1 expired/no eligible draw in the armed frame. */
 int present(uint64_t now){
  if(phase==WAITING&&now>=due){if(now>due+1000){phase=IDLE;return -1;}phase=ARMED;return 1;}
  if(phase==ARMED){phase=IDLE;return -1;}return 0;
 }
 bool take(uint64_t now){if(phase!=ARMED)return false;if(now>due+1000){phase=IDLE;return false;}phase=TAKEN;return true;}
 void finish(){phase=IDLE;}
};
/* Read-only, ordered snapshots. No pipeline setters or source writes. Refuse
   view/subresource shapes this probe cannot represent as a complete image. */
static bool dgHistoryColour(ID3D11Device*d,ID3D11DeviceContext*c,ID3D11Texture2D*s,ComPtr<ID3D11Texture2D>&out){
 out.Reset();if(!d||!c||!s)return false;D3D11_TEXTURE2D_DESC x{};s->GetDesc(&x);
 if(x.MipLevels!=1||x.ArraySize!=1||!x.Width||!x.Height||x.Width>8192||x.Height>8192||!x.SampleDesc.Count)return false;
 if(x.Format!=DXGI_FORMAT_B8G8R8A8_UNORM&&x.Format!=DXGI_FORMAT_B8G8R8A8_UNORM_SRGB&&x.Format!=DXGI_FORMAT_B8G8R8A8_TYPELESS)return false;
 if(x.SampleDesc.Count>1&&x.Format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB)return false;
 if(x.SampleDesc.Count>1){UINT support=0;if(FAILED(d->CheckFormatSupport(DXGI_FORMAT_B8G8R8A8_UNORM,&support))||!(support&D3D11_FORMAT_SUPPORT_MULTISAMPLE_RESOLVE))return false;}
 x.SampleDesc.Count=1;x.SampleDesc.Quality=0;x.Format=DXGI_FORMAT_B8G8R8A8_UNORM;x.Usage=D3D11_USAGE_DEFAULT;x.BindFlags=0;x.CPUAccessFlags=0;x.MiscFlags=0;
 if(FAILED(d->CreateTexture2D(&x,nullptr,&out)))return false;
 if(s){D3D11_TEXTURE2D_DESC sd{};s->GetDesc(&sd);if(sd.SampleDesc.Count>1)c->ResolveSubresource(out.Get(),0,s,0,DXGI_FORMAT_B8G8R8A8_UNORM);else c->CopyResource(out.Get(),s);}return true;
}
static bool dgHistoryBank(ID3D11Device*d,ID3D11DeviceContext*c,ID3D11Buffer*s,ComPtr<ID3D11Buffer>&out){
 out.Reset();if(!s)return false;D3D11_BUFFER_DESC x{};s->GetDesc(&x);if(!x.ByteWidth||x.ByteWidth>4*1024*1024)return false;
 x.Usage=D3D11_USAGE_DEFAULT;x.BindFlags=0;x.CPUAccessFlags=0;x.MiscFlags=0;x.StructureByteStride=0;
 if(FAILED(d->CreateBuffer(&x,nullptr,&out)))return false;c->CopyResource(out.Get(),s);return true;
}
#endif
