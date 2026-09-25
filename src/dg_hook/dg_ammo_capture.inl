// Replay only native HUD sprite shaders into an initially transparent target.
// The game's own icon, bullet strip, font, colors and alpha remain authoritative.
// Capture the complete lower-right HUD region through the screen edges.
// The previous tight virtual-512x384 crop truncated live artwork.
// Only sprite draws are replayed; no scene texture or backbuffer is copied.
constexpr float ammoX=.75f,ammoY=.70f;
constexpr float ammoW=.25f,ammoH=.30f;
ComPtr<ID3D11Texture2D> ammoTexture;
ComPtr<ID3D11RenderTargetView> ammoRT;
ComPtr<ID3D11Buffer> ammoCB;
ComPtr<ID3D11DepthStencilState> ammoDepth;
ComPtr<ID3D11RasterizerState> ammoRaster;
ComPtr<ID3D11BlendState> ammoBlend;
UINT ammoHeight=0,ammoCbBytes=0;
unsigned ammoDraws=0;
bool ammoCapturedDraw=false;
ULONGLONG ammoReadyMs=0;
void ammoReset(){ammoTexture.Reset();ammoRT.Reset();ammoCB.Reset();ammoDepth.Reset();ammoRaster.Reset();ammoBlend.Reset();ammoHeight=ammoCbBytes=ammoDraws=0;}




extern "C" int dg_xr_ammo_capture(void*);
int ammoDeliver(ID3D11Texture2D*t){return dg_xr_ammo_capture(t);}

bool ammoShader(UINT64 h){return h==0x25506B36206636CAull||h==0x4E028ADD1EA96D0Bull;}
// CPU register bank remains pristine even if stereo HUD placement changed the
// bound GPU buffer. Transform that original bank directly into the crop.
void ammoCrop(float*f){f[20]/=ammoW;f[24]=(f[24]-(2*ammoX-1))/ammoW-1;
 f[21]/=ammoH;f[25]=(f[25]-(1-2*ammoY))/ammoH+1;}
bool ammoResources(UINT height,UINT cbBytes){
 if(height<16||height>1024||cbBytes<0x240||cbBytes>65536||cbBytes%16)return false;
 if(ammoHeight!=height||ammoCbBytes!=cbBytes)ammoReset();
 if(ammoTexture)return true;
 D3D11_TEXTURE2D_DESC td{};td.Width=512;td.Height=height;td.MipLevels=td.ArraySize=1;
 td.Format=DXGI_FORMAT_R8G8B8A8_UNORM;td.SampleDesc.Count=1;td.Usage=D3D11_USAGE_DEFAULT;td.BindFlags=D3D11_BIND_RENDER_TARGET|D3D11_BIND_SHADER_RESOURCE;
 D3D11_BUFFER_DESC bd{};bd.ByteWidth=cbBytes;bd.Usage=D3D11_USAGE_DYNAMIC;bd.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;bd.BindFlags=D3D11_BIND_CONSTANT_BUFFER;
 D3D11_DEPTH_STENCIL_DESC dd{};dd.DepthEnable=FALSE;dd.DepthFunc=D3D11_COMPARISON_ALWAYS;
 D3D11_RASTERIZER_DESC rd{};rd.FillMode=D3D11_FILL_SOLID;rd.CullMode=D3D11_CULL_NONE;rd.DepthClipEnable=TRUE;rd.ScissorEnable=TRUE;
 if(FAILED(device->CreateTexture2D(&td,nullptr,&ammoTexture))||FAILED(device->CreateRenderTargetView(ammoTexture.Get(),nullptr,&ammoRT))||
    FAILED(device->CreateBuffer(&bd,nullptr,&ammoCB))||FAILED(device->CreateDepthStencilState(&dd,&ammoDepth))||FAILED(device->CreateRasterizerState(&rd,&ammoRaster))){ammoReset();return false;}
 ammoHeight=height;ammoCbBytes=cbBytes;return true;
}
void ammoOnDraw(ID3D11DeviceContext*c,UINT n,UINT first){
 ammoCapturedDraw=false;
 if(!radarOn||!radarFirstPerson||n!=4||c!=context.Get()||stopping||inside||requested==1||!ui2dSceneAllowed())return;
 const float*f=(const float*)ui2dBank;
 if(!f||ui2dBankLen<0x240||ui2dBankLen>65536||!ui2dSignature(f))return;
 MultiLock lock;
 if(ui2dSceneTexture(c))return;
 ComPtr<ID3D11VertexShader>vs;UINT nc=0;c->VSGetShader(&vs,nullptr,&nc);if(!ammoShader(vsHashOf(vs.Get())))return;
 D3D11_VIEWPORT vp[16];UINT nv=16;c->RSGetViewports(&nv,vp);
 if(nv!=1||!ui2dBbW||!ui2dBbH||vp[0].TopLeftX!=0||vp[0].TopLeftY!=0||vp[0].Width!=ui2dBbW||vp[0].Height!=ui2dBbH)return;
 ComPtr<ID3D11DepthStencilState>depth;UINT ref;c->OMGetDepthStencilState(&depth,&ref);
 D3D11_DEPTH_STENCIL_DESC dd{};dd.DepthEnable=TRUE;if(depth)depth->GetDesc(&dd);if(!dd.DepthEnable)return;
 ComPtr<ID3D11Predicate>predicate;BOOL pred;c->GetPredication(&predicate,&pred);if(predicate)return;
 ID3D11Buffer*so[4]{};bool bad=false;c->SOGetTargets(4,so);for(auto v:so)if(v){v->Release();bad=true;}if(bad)return;
 ID3D11UnorderedAccessView*u[64]{};UINT slots=device->GetFeatureLevel()>=D3D_FEATURE_LEVEL_11_1?64:8;
 c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,slots,u);for(UINT i=0;i<slots;i++)if(u[i]){u[i]->Release();bad=true;}if(bad)return;
 ComPtr<ID3D11GeometryShader>gs;c->GSGetShader(&gs,nullptr,&nc);if(gs)return;
 ComPtr<ID3D11HullShader>hs;c->HSGetShader(&hs,nullptr,&nc);if(hs)return;
 ComPtr<ID3D11DomainShader>ds;c->DSGetShader(&ds,nullptr,&nc);if(ds)return;
 ID3D11RenderTargetView*rt[8]{};ComPtr<ID3D11DepthStencilView>dsv;c->OMGetRenderTargets(8,rt,&dsv);
 for(unsigned i=1;i<8;i++)if(rt[i])bad=true;
 ComPtr<ID3D11RenderTargetView>oldRT;oldRT.Attach(rt[0]);for(unsigned i=1;i<8;i++)if(rt[i])rt[i]->Release();if(bad||!oldRT)return;
 ComPtr<ID3D11Buffer>oldCB;c->VSGetConstantBuffers(0,1,&oldCB);if(!oldCB)return;
 D3D11_BUFFER_DESC cbd{};oldCB->GetDesc(&cbd);if(cbd.ByteWidth<ui2dBankLen)return;
 if(!ammoResources((UINT)(512*ammoH/ ammoW*ui2dBbH/ui2dBbW+.5f),cbd.ByteWidth))return;
 ComPtr<ID3D11BlendState>oldBlend;float factors[4];UINT sampleMask;c->OMGetBlendState(&oldBlend,factors,&sampleMask);
 D3D11_BLEND_DESC blend{};
 if(oldBlend)oldBlend->GetDesc(&blend);else{blend.RenderTarget[0].RenderTargetWriteMask=D3D11_COLOR_WRITE_ENABLE_ALL;}
 // Native RGB blend retained, alpha accumulated as coverage for XR composition.
 auto&b=blend.RenderTarget[0];b.SrcBlendAlpha=D3D11_BLEND_ONE;b.DestBlendAlpha=D3D11_BLEND_INV_SRC_ALPHA;b.BlendOpAlpha=D3D11_BLEND_OP_ADD;
 if(FAILED(device->CreateBlendState(&blend,&ammoBlend)))return;
 D3D11_MAPPED_SUBRESOURCE mapped{};if(FAILED(c->Map(ammoCB.Get(),0,D3D11_MAP_WRITE_DISCARD,0,&mapped)))return;
 memset(mapped.pData,0,cbd.ByteWidth);memcpy(mapped.pData,f,ui2dBankLen);ammoCrop((float*)mapped.pData);c->Unmap(ammoCB.Get(),0);
 ComPtr<ID3D11RasterizerState>oldRaster;c->RSGetState(&oldRaster);
 D3D11_RECT scissors[16];UINT ns=16;c->RSGetScissorRects(&ns,scissors);
 D3D11_RECT crop={0,0,512,(LONG)ammoHeight};
 D3D11_RASTERIZER_DESC rd{};if(oldRaster)oldRaster->GetDesc(&rd);
 if(rd.ScissorEnable){if(ns!=1)return;
  crop.left=max(0L,(LONG)floor((scissors[0].left/vp[0].Width-ammoX)/ammoW*512));
  crop.right=min(512L,(LONG)ceil((scissors[0].right/vp[0].Width-ammoX)/ammoW*512));
  crop.top=max(0L,(LONG)floor((scissors[0].top/vp[0].Height-ammoY)/ammoH*ammoHeight));
  crop.bottom=min((LONG)ammoHeight,(LONG)ceil((scissors[0].bottom/vp[0].Height-ammoY)/ammoH*ammoHeight));
  if(crop.left>=crop.right||crop.top>=crop.bottom)return;
 }
 if(!ammoDraws){float clear[4]={};c->ClearRenderTargetView(ammoRT.Get(),clear);}
 D3D11_VIEWPORT target={0,0,512,(FLOAT)ammoHeight,0,1};ID3D11RenderTargetView*targetRT=ammoRT.Get();ID3D11Buffer*targetCB=ammoCB.Get();
 c->OMSetRenderTargets(1,&targetRT,nullptr);c->OMSetDepthStencilState(ammoDepth.Get(),0);c->OMSetBlendState(ammoBlend.Get(),factors,sampleMask);
 c->RSSetState(ammoRaster.Get());c->RSSetScissorRects(1,&crop);c->RSSetViewports(1,&target);c->VSSetConstantBuffers(0,1,&targetCB);
 inside=true;auto drawApi=(Draw)(*(void***)c)[13];drawApi(c,n,first);inside=false;
 ID3D11RenderTargetView*restoreRT=oldRT.Get();ID3D11Buffer*restoreCB=oldCB.Get();
 c->VSSetConstantBuffers(0,1,&restoreCB);c->RSSetViewports(nv,vp);c->RSSetScissorRects(ns,scissors);c->RSSetState(oldRaster.Get());
 c->OMSetBlendState(oldBlend.Get(),factors,sampleMask);c->OMSetDepthStencilState(depth.Get(),ref);c->OMSetRenderTargets(1,&restoreRT,dsv.Get());
 ammoDraws++;
 ammoCapturedDraw=true;
}
bool ammoHideDraw(ID3D11DeviceContext*c,UINT n,UINT first){
 // Capture precedes suppression, including while the inspection gate is shut.
 if(!ammoCapturedDraw||!radarFirstPerson||!ui2dSceneAllowed())return false;
 MultiLock lock;
 D3D11_VIEWPORT vp;UINT nv=1;c->RSGetViewports(&nv,&vp);if(nv!=1)return false;
 ComPtr<ID3D11RasterizerState>old;c->RSGetState(&old);
 D3D11_RASTERIZER_DESC rd{};rd.FillMode=D3D11_FILL_SOLID;rd.CullMode=D3D11_CULL_BACK;rd.DepthClipEnable=TRUE;if(old)old->GetDesc(&rd);
 D3D11_RECT prior[16];UINT ns=16;c->RSGetScissorRects(&ns,prior);if(rd.ScissorEnable&&ns!=1)return false;
 D3D11_RECT full={0,0,(LONG)vp.Width,(LONG)vp.Height};if(rd.ScissorEnable)full=prior[0];
 float scale=ui2dGpuPatched?(float)ui2dCfg.scaleMils/1000.f:1.f;
 float shift=0;if(ui2dGpuPatched){float x=ui2dCfg.sign<0?-ui2dGpuX0:ui2dGpuX0;shift=x+(x>=0?1:-1)*(float)ui2dCfg.convE5/100000.f;}
 LONG left=(LONG)floor(((2*ammoX-1)*scale+shift+1)*vp.Width*.5f);
 LONG right=(LONG)ceil(((2*(ammoX+ammoW)-1)*scale+shift+1)*vp.Width*.5f);
 LONG top=(LONG)floor((1-(1-2*ammoY)*scale)*vp.Height*.5f);
 LONG bottom=(LONG)ceil((1-(1-2*(ammoY+ammoH))*scale)*vp.Height*.5f);
 left=max(left,full.left);right=min(right,full.right);top=max(top,full.top);bottom=min(bottom,full.bottom);
 if(left>=right||top>=bottom)return false;
 rd.ScissorEnable=TRUE;ComPtr<ID3D11RasterizerState>clip;if(FAILED(device->CreateRasterizerState(&rd,&clip)))return false;
 // Disjoint rectangles preserve every original HUD pixel outside the watch
 // panel. No global HUD flag: that would prevent native artwork generation.
 D3D11_RECT parts[4]={{full.left,full.top,full.right,top},{full.left,bottom,full.right,full.bottom},
                     {full.left,top,left,bottom},{right,top,full.right,bottom}};
 c->RSSetState(clip.Get());auto api=(Draw)(*(void***)c)[13];inside=true;
 for(auto&r:parts)if(r.left<r.right&&r.top<r.bottom){c->RSSetScissorRects(1,&r);api(c,n,first);}
 inside=false;c->RSSetScissorRects(ns,prior);c->RSSetState(old.Get());return true;
}
void ammoOnPresent(){
 // The caller holds lifetime, but MUST NOT hold MultiLock across delivery.
 ammoReadyMs=0;
 if(ammoDraws&&radarOn&&radarFirstPerson&&ui2dSceneAllowed()&&ammoTexture&&ammoDeliver(ammoTexture.Get())>0)ammoReadyMs=GetTickCount64();
 ammoDraws=0;
}
