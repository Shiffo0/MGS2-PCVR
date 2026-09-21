#include "dg_present.c"
/* Radar draw callback is outside the factory test; no draws are issued. */
int dg_xr_radar_capture(void *texture) { (void)texture; return 0; }
static int fails;
#define CHECK(x) do { if (!(x)) { printf("FAIL line %d: %s\n",__LINE__,#x); ++fails; } } while(0)
int main(void) {
 IDXGIFactory *f=NULL; IDXGIFactory1 *f1=NULL; IDXGIAdapter *a=NULL;
 ID3D11Device *dev=NULL; ID3D11Texture2D *tex=NULL; IDXGIResource *res=NULL;
 HANDLE shared=NULL; D3D11_TEXTURE2D_DESC d={0}; HRESULT hr;
 hr=hook_create_factory(&IID_IDXGIFactory,(void**)&f); CHECK(SUCCEEDED(hr)&&f);
 if(!f) return 1;
 CHECK(g_factory_patch.slot && g_create_sc);
 CHECK(SUCCEEDED(f->lpVtbl->QueryInterface(f,&IID_IDXGIFactory1,(void**)&f1)));
 CHECK(SUCCEEDED(f->lpVtbl->EnumAdapters(f,0,&a)));
 if(a) {
  hr=D3D11CreateDevice(a,D3D_DRIVER_TYPE_UNKNOWN,NULL,0,NULL,0,D3D11_SDK_VERSION,&dev,NULL,NULL);
  CHECK(SUCCEEDED(hr));
 }
 if(dev) {
  d.Width=d.Height=16; d.MipLevels=d.ArraySize=1; d.Format=DXGI_FORMAT_R8G8B8A8_UNORM;
  d.SampleDesc.Count=1; d.BindFlags=D3D11_BIND_RENDER_TARGET|D3D11_BIND_SHADER_RESOURCE;
  d.MiscFlags=D3D11_RESOURCE_MISC_SHARED;
  CHECK(SUCCEEDED(dev->lpVtbl->CreateTexture2D(dev,&d,NULL,&tex)));
  if(tex) {
   CHECK(SUCCEEDED(tex->lpVtbl->QueryInterface(tex,&IID_IDXGIResource,(void**)&res)));
   if(res) CHECK(SUCCEEDED(res->lpVtbl->GetSharedHandle(res,&shared)) && shared);
  }
 }
 rollback(); CHECK(!g_factory_patch.slot);
 if(res) res->lpVtbl->Release(res); if(tex) tex->lpVtbl->Release(tex);
 if(dev) dev->lpVtbl->Release(dev); if(a) a->lpVtbl->Release(a);
 if(f1) f1->lpVtbl->Release(f1); f->lpVtbl->Release(f);
 printf("factory/shared-texture regression: %s\n",fails?"FAIL":"PASS"); return fails?1:0;
}
