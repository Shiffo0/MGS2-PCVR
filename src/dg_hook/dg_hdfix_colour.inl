/* MGSHDFix "Correct Gamma Levels" for the replayed eye (10 Oct 2026). MGSHDFix runs its colour correction at
   Present on the swap-chain buffer (src/fixes/color_correction.cpp: SweetFX vibrance 0.15, then a smootherstep
   luma curve blended by 0.225). The native eye is captured from that buffer after it; the replayed eye is
   published from our own texture and missed it (live 10 Oct: native T3 == buffer after the game's last draw,
   median 25; buffer at Present median 19; replay 25). Same shader, applied to our texture in place.
   Enabled when <game>\plugins\MGSHDFix.settings says "Correct Gamma Levels=1"; vr_se_mode bit 64 = off. */
#include <d3dcompiler.h>
#pragma comment(lib,"d3dcompiler.lib")
struct DgHdfixColour {
 int enabled=-1;   /* -1 not read yet (the desk check sets 1) */
 bool failed=false;
 ComPtr<ID3D11VertexShader>vs;ComPtr<ID3D11PixelShader>ps;ComPtr<ID3D11SamplerState>smp;ComPtr<ID3D11RasterizerState>rs;ComPtr<ID3D11DepthStencilState>dss;
 ComPtr<ID3D11Texture2D>temp;ComPtr<ID3D11ShaderResourceView>tempSrv;D3D11_TEXTURE2D_DESC tempDesc{};
 unsigned long long applied=0;
 static int readSetting(const char*path){
  FILE*f=nullptr;if(fopen_s(&f,path,"rb")||!f)return 0;char line[256];int on=0;
  while(fgets(line,sizeof line,f)){const char*k="Correct Gamma Levels=";if(!strncmp(line,k,strlen(k))){on=atoi(line+strlen(k))!=0;break;}}
  fclose(f);return on;
 }
 bool on(){
  if(enabled<0){char p[MAX_PATH];DWORD n=GetModuleFileNameA(nullptr,p,MAX_PATH);char*s=n&&n<MAX_PATH?strrchr(p,'\\'):nullptr;
   bool loaded=GetModuleHandleA("MGSHDFix.asi")!=nullptr;   /* only when MGSHDFix really runs in this process */
   if(s&&loaded){strcpy_s(s+1,MAX_PATH-(s+1-p),"plugins\\MGSHDFix.settings");enabled=readSetting(p);}else enabled=0;
   if(logger)logger("second_eye: MGSHDFix colour correction for the replayed eye: %s\r\n",!loaded?"off (MGSHDFix not loaded)":enabled?"on (MGSHDFix loaded, Correct Gamma Levels=1)":"off (Correct Gamma Levels absent or 0)");}
  return enabled>0&&!failed;
 }
 bool init(ID3D11Device*d){
  if(ps)return true;if(failed)return false;

  static const char*code=
   "Texture2D backBuffer:register(t0);SamplerState smpl:register(s0);\n"
   "void VS(uint id:SV_VertexID,out float4 pos:SV_Position,out float2 uv:TexCoord){uv=float2((id<<1)&2,id&2);pos=float4(uv*float2(2.0,-2.0)+float2(-1.0,1.0),0.0,1.0);}\n"
   "float4 PS(float4 pos:SV_Position,float2 uv:TexCoord):SV_Target{float3 color=backBuffer.Sample(smpl,uv).rgb;\n"
   " static const float3 coefLuma=float3(0.212656,0.715158,0.072186);const float kVibrance=0.15;float luma=dot(coefLuma,color);\n"
   " float max_color=max(color.r,max(color.g,color.b));float min_color=min(color.r,min(color.g,color.b));float color_saturation=max_color-min_color;\n"
   " color=lerp(luma,color,1.0+(kVibrance*(1.0-(sign(kVibrance)*color_saturation))));\n"
   " const float kContrast=0.225;luma=dot(coefLuma,color);float3 chroma=color-luma;float s=luma;s=s*s*s*(s*(s*6.0-15.0)+10.0);luma=lerp(luma,s,kContrast);color=luma+chroma;\n"
   " return float4(color,1.0);}\n";
  ComPtr<ID3DBlob>a,b,err;
  if(FAILED(D3DCompile(code,strlen(code),"hdfix_colour",nullptr,nullptr,"VS","vs_5_0",0,0,&a,&err))||FAILED(D3DCompile(code,strlen(code),"hdfix_colour",nullptr,nullptr,"PS","ps_5_0",0,0,&b,&err))||
     FAILED(d->CreateVertexShader(a->GetBufferPointer(),a->GetBufferSize(),nullptr,&vs))||FAILED(d->CreatePixelShader(b->GetBufferPointer(),b->GetBufferSize(),nullptr,&ps))){
   failed=true;if(logger)logger("second_eye: MGSHDFix colour correction: shader setup FAILED (%s); replayed eye without it\r\n",err?(const char*)err->GetBufferPointer():"create");return false;}
  D3D11_SAMPLER_DESC sd{};sd.Filter=D3D11_FILTER_MIN_MAG_MIP_LINEAR;sd.AddressU=sd.AddressV=sd.AddressW=D3D11_TEXTURE_ADDRESS_CLAMP;sd.MaxLOD=D3D11_FLOAT32_MAX;
  D3D11_RASTERIZER_DESC rd{};rd.FillMode=D3D11_FILL_SOLID;rd.CullMode=D3D11_CULL_NONE;rd.DepthClipEnable=TRUE;
  D3D11_DEPTH_STENCIL_DESC dd{};dd.DepthEnable=FALSE;dd.DepthWriteMask=D3D11_DEPTH_WRITE_MASK_ZERO;dd.DepthFunc=D3D11_COMPARISON_ALWAYS;
  if(FAILED(d->CreateSamplerState(&sd,&smp))||FAILED(d->CreateRasterizerState(&rd,&rs))||FAILED(d->CreateDepthStencilState(&dd,&dss))){failed=true;ps.Reset();return false;}
  return true;
 }
 /* t/rt: our texture and its target view; corrected in place. Restores every state it touches. */
 bool apply(ID3D11Device*d,ID3D11DeviceContext*c,ID3D11Texture2D*t,ID3D11RenderTargetView*rt){
  if(!t||!rt||!init(d))return false;
  D3D11_TEXTURE2D_DESC x{};t->GetDesc(&x);
  if(!temp||tempDesc.Width!=x.Width||tempDesc.Height!=x.Height||tempDesc.Format!=x.Format){
   temp.Reset();tempSrv.Reset();D3D11_TEXTURE2D_DESC y=x;y.BindFlags=D3D11_BIND_SHADER_RESOURCE;y.Usage=D3D11_USAGE_DEFAULT;y.CPUAccessFlags=0;y.MiscFlags=0;y.MipLevels=1;
   if(FAILED(d->CreateTexture2D(&y,nullptr,&temp))||FAILED(d->CreateShaderResourceView(temp.Get(),nullptr,&tempSrv))){temp.Reset();tempSrv.Reset();return false;}
   tempDesc=x;}
  c->CopyResource(temp.Get(),t);
  /* save */
  ID3D11RenderTargetView*oRt[8]{};ID3D11DepthStencilView*oDs=nullptr;c->OMGetRenderTargets(8,oRt,&oDs);
  ID3D11BlendState*oBs=nullptr;FLOAT oBf[4]{};UINT oMask=0;c->OMGetBlendState(&oBs,oBf,&oMask);
  ID3D11DepthStencilState*oDss=nullptr;UINT oRef=0;c->OMGetDepthStencilState(&oDss,&oRef);
  ID3D11RasterizerState*oRs=nullptr;c->RSGetState(&oRs);
  D3D11_VIEWPORT oVp[D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE]{};UINT nVp=D3D11_VIEWPORT_AND_SCISSORRECT_OBJECT_COUNT_PER_PIPELINE;c->RSGetViewports(&nVp,oVp);
  ID3D11VertexShader*oVs=nullptr;ID3D11ClassInstance*vci[256]{};UINT nvci=256;c->VSGetShader(&oVs,vci,&nvci);
  ID3D11PixelShader*oPs=nullptr;ID3D11ClassInstance*pci[256]{};UINT npci=256;c->PSGetShader(&oPs,pci,&npci);
  ID3D11GeometryShader*oGs=nullptr;ID3D11ClassInstance*gci[256]{};UINT ngci=256;c->GSGetShader(&oGs,gci,&ngci);
  ID3D11HullShader*oHs=nullptr;ID3D11ClassInstance*hci[256]{};UINT nhci=256;c->HSGetShader(&oHs,hci,&nhci);
  ID3D11DomainShader*oDsh=nullptr;ID3D11ClassInstance*dci[256]{};UINT ndci=256;c->DSGetShader(&oDsh,dci,&ndci);
  ID3D11InputLayout*oIl=nullptr;c->IAGetInputLayout(&oIl);D3D11_PRIMITIVE_TOPOLOGY oTopo;c->IAGetPrimitiveTopology(&oTopo);
  ID3D11ShaderResourceView*oSrv=nullptr;c->PSGetShaderResources(0,1,&oSrv);ID3D11SamplerState*oSmp=nullptr;c->PSGetSamplers(0,1,&oSmp);
  /* draw */
  c->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);c->IASetInputLayout(nullptr);
  c->VSSetShader(vs.Get(),nullptr,0);c->PSSetShader(ps.Get(),nullptr,0);c->GSSetShader(nullptr,nullptr,0);c->HSSetShader(nullptr,nullptr,0);c->DSSetShader(nullptr,nullptr,0);
  ID3D11ShaderResourceView*s=tempSrv.Get();c->PSSetShaderResources(0,1,&s);ID3D11SamplerState*sm=smp.Get();c->PSSetSamplers(0,1,&sm);
  c->RSSetState(rs.Get());D3D11_VIEWPORT vp{0,0,(FLOAT)x.Width,(FLOAT)x.Height,0,1};c->RSSetViewports(1,&vp);
  c->OMSetDepthStencilState(dss.Get(),0);c->OMSetBlendState(nullptr,nullptr,0xFFFFFFFF);c->OMSetRenderTargets(1,&rt,nullptr);
  c->Draw(3,0);
  /* restore */
  c->OMSetRenderTargets(8,oRt,oDs);c->OMSetBlendState(oBs,oBf,oMask);c->OMSetDepthStencilState(oDss,oRef);c->RSSetState(oRs);c->RSSetViewports(nVp,oVp);
  c->VSSetShader(oVs,vci,nvci);c->PSSetShader(oPs,pci,npci);c->GSSetShader(oGs,gci,ngci);c->HSSetShader(oHs,hci,nhci);c->DSSetShader(oDsh,dci,ndci);
  c->IASetInputLayout(oIl);c->IASetPrimitiveTopology(oTopo);c->PSSetShaderResources(0,1,&oSrv);c->PSSetSamplers(0,1,&oSmp);
  for(auto r:oRt)if(r)r->Release();
  IUnknown*rel[]={oDs,oBs,oDss,oRs,oVs,oPs,oGs,oHs,oDsh,oIl,oSrv,oSmp};for(auto u:rel)if(u)u->Release();
  for(UINT i=0;i<nvci;i++)if(vci[i])vci[i]->Release();for(UINT i=0;i<npci;i++)if(pci[i])pci[i]->Release();for(UINT i=0;i<ngci;i++)if(gci[i])gci[i]->Release();
  for(UINT i=0;i<nhci;i++)if(hci[i])hci[i]->Release();for(UINT i=0;i<ndci;i++)if(dci[i])dci[i]->Release();
  ++applied;return true;
 }
};
