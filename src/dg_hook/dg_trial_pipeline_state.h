// Read-only snapshot of getter-visible D3D11 pipeline bindings.
// Retain getter references until comparison to prevent pointer reuse.
// Excludes resource contents, query progress, SO offsets and hidden UAV counters.
#pragma once
#include <vector>
struct TrialPipelineState {
 struct Section{const char* name;size_t offset;};
 std::vector<Section> sections;
 void mark(const char* name){sections.push_back({name,bytes.size()});}
 unsigned long long differences(const TrialPipelineState& other)const{
  if(sections.size()!=other.sections.size())return ~0ull;
  unsigned long long mask=0;
  for(size_t i=0;i<sections.size();i++){
   size_t a=sections[i].offset,b=other.sections[i].offset;
   size_t an=(i+1<sections.size()?sections[i+1].offset:bytes.size())-a;
   size_t bn=(i+1<other.sections.size()?other.sections[i+1].offset:other.bytes.size())-b;
   if(an!=bn||memcmp(bytes.data()+a,other.bytes.data()+b,an))mask|=1ull<<i;
  }return mask;
 }
 std::vector<unsigned char> bytes;
 std::vector<ComPtr<IUnknown>> refs;
 template<class T> void value(const T& v){auto b=reinterpret_cast<const unsigned char*>(&v);bytes.insert(bytes.end(),b,b+sizeof(v));}
 template<class T> void objects(T** v,unsigned n){for(unsigned i=0;i<n;i++){value(v[i]);if(v[i]){ComPtr<IUnknown> r;r.Attach(v[i]);refs.push_back(std::move(r));}}}
 explicit TrialPipelineState(ID3D11DeviceContext*c){
 mark("context1");ComPtr<ID3D11DeviceContext1> c1;c->QueryInterface(IID_PPV_ARGS(&c1));bool ranges=!!c1;value(ranges);
 {
 mark("VS.shader_classes");ID3D11VertexShader* shader=nullptr;ID3D11ClassInstance* classes[256]{};UINT count=256;
 c->VSGetShader(&shader,classes,&count);objects(&shader,1);value(count);objects(classes,count);
 mark("VS.constant_buffers");ID3D11Buffer* cb[14]{};c->VSGetConstantBuffers(0,14,cb);objects(cb,14);
 mark("VS.buffer_ranges");if(c1){UINT first[14]{},num[14]{};ID3D11Buffer* ranged[14]{};c1->VSGetConstantBuffers1(0,14,ranged,first,num);objects(ranged,14);value(first);value(num);}
 mark("VS.resources");ID3D11ShaderResourceView* srv[128]{};c->VSGetShaderResources(0,128,srv);objects(srv,128);
 mark("VS.samplers");ID3D11SamplerState* sam[16]{};c->VSGetSamplers(0,16,sam);objects(sam,16);
 }
 {
 mark("HS.shader_classes");ID3D11HullShader* shader=nullptr;ID3D11ClassInstance* classes[256]{};UINT count=256;
 c->HSGetShader(&shader,classes,&count);objects(&shader,1);value(count);objects(classes,count);
 mark("HS.constant_buffers");ID3D11Buffer* cb[14]{};c->HSGetConstantBuffers(0,14,cb);objects(cb,14);
 mark("HS.buffer_ranges");if(c1){UINT first[14]{},num[14]{};ID3D11Buffer* ranged[14]{};c1->HSGetConstantBuffers1(0,14,ranged,first,num);objects(ranged,14);value(first);value(num);}
 mark("HS.resources");ID3D11ShaderResourceView* srv[128]{};c->HSGetShaderResources(0,128,srv);objects(srv,128);
 mark("HS.samplers");ID3D11SamplerState* sam[16]{};c->HSGetSamplers(0,16,sam);objects(sam,16);
 }
 {
 mark("DS.shader_classes");ID3D11DomainShader* shader=nullptr;ID3D11ClassInstance* classes[256]{};UINT count=256;
 c->DSGetShader(&shader,classes,&count);objects(&shader,1);value(count);objects(classes,count);
 mark("DS.constant_buffers");ID3D11Buffer* cb[14]{};c->DSGetConstantBuffers(0,14,cb);objects(cb,14);
 mark("DS.buffer_ranges");if(c1){UINT first[14]{},num[14]{};ID3D11Buffer* ranged[14]{};c1->DSGetConstantBuffers1(0,14,ranged,first,num);objects(ranged,14);value(first);value(num);}
 mark("DS.resources");ID3D11ShaderResourceView* srv[128]{};c->DSGetShaderResources(0,128,srv);objects(srv,128);
 mark("DS.samplers");ID3D11SamplerState* sam[16]{};c->DSGetSamplers(0,16,sam);objects(sam,16);
 }
 {
 mark("GS.shader_classes");ID3D11GeometryShader* shader=nullptr;ID3D11ClassInstance* classes[256]{};UINT count=256;
 c->GSGetShader(&shader,classes,&count);objects(&shader,1);value(count);objects(classes,count);
 mark("GS.constant_buffers");ID3D11Buffer* cb[14]{};c->GSGetConstantBuffers(0,14,cb);objects(cb,14);
 mark("GS.buffer_ranges");if(c1){UINT first[14]{},num[14]{};ID3D11Buffer* ranged[14]{};c1->GSGetConstantBuffers1(0,14,ranged,first,num);objects(ranged,14);value(first);value(num);}
 mark("GS.resources");ID3D11ShaderResourceView* srv[128]{};c->GSGetShaderResources(0,128,srv);objects(srv,128);
 mark("GS.samplers");ID3D11SamplerState* sam[16]{};c->GSGetSamplers(0,16,sam);objects(sam,16);
 }
 {
 mark("PS.shader_classes");ID3D11PixelShader* shader=nullptr;ID3D11ClassInstance* classes[256]{};UINT count=256;
 c->PSGetShader(&shader,classes,&count);objects(&shader,1);value(count);objects(classes,count);
 mark("PS.constant_buffers");ID3D11Buffer* cb[14]{};c->PSGetConstantBuffers(0,14,cb);objects(cb,14);
 mark("PS.buffer_ranges");if(c1){UINT first[14]{},num[14]{};ID3D11Buffer* ranged[14]{};c1->PSGetConstantBuffers1(0,14,ranged,first,num);objects(ranged,14);value(first);value(num);}
 mark("PS.resources");ID3D11ShaderResourceView* srv[128]{};c->PSGetShaderResources(0,128,srv);objects(srv,128);
 mark("PS.samplers");ID3D11SamplerState* sam[16]{};c->PSGetSamplers(0,16,sam);objects(sam,16);
 }
 {
 mark("CS.shader_classes");ID3D11ComputeShader* shader=nullptr;ID3D11ClassInstance* classes[256]{};UINT count=256;
 c->CSGetShader(&shader,classes,&count);objects(&shader,1);value(count);objects(classes,count);
 mark("CS.constant_buffers");ID3D11Buffer* cb[14]{};c->CSGetConstantBuffers(0,14,cb);objects(cb,14);
 mark("CS.buffer_ranges");if(c1){UINT first[14]{},num[14]{};ID3D11Buffer* ranged[14]{};c1->CSGetConstantBuffers1(0,14,ranged,first,num);objects(ranged,14);value(first);value(num);}
 mark("CS.resources");ID3D11ShaderResourceView* srv[128]{};c->CSGetShaderResources(0,128,srv);objects(srv,128);
 mark("CS.samplers");ID3D11SamplerState* sam[16]{};c->CSGetSamplers(0,16,sam);objects(sam,16);
 }

 mark("IA.layout");ID3D11InputLayout* layout=nullptr;c->IAGetInputLayout(&layout);objects(&layout,1);
 mark("IA.vertex_buffers");ID3D11Buffer* vb[32]{};UINT strides[32]{},offsets[32]{};c->IAGetVertexBuffers(0,32,vb,strides,offsets);objects(vb,32);value(strides);value(offsets);
 mark("IA.index_buffer");ID3D11Buffer* ib=nullptr;DXGI_FORMAT fmt{};UINT off=0;c->IAGetIndexBuffer(&ib,&fmt,&off);objects(&ib,1);value(fmt);value(off);
 mark("IA.topology");D3D11_PRIMITIVE_TOPOLOGY topology{};c->IAGetPrimitiveTopology(&topology);value(topology);
 mark("RS.state");ID3D11RasterizerState* rs=nullptr;c->RSGetState(&rs);objects(&rs,1);
 mark("RS.viewports");D3D11_VIEWPORT vp[16]{};UINT nv=16;c->RSGetViewports(&nv,vp);value(nv);for(UINT i=0;i<nv;i++)value(vp[i]);
 mark("RS.scissors");D3D11_RECT rect[16]{};UINT nr=16;c->RSGetScissorRects(&nr,rect);value(nr);for(UINT i=0;i<nr;i++)value(rect[i]);
 mark("OM.targets");ID3D11RenderTargetView* rt[8]{};ID3D11DepthStencilView* ds=nullptr;c->OMGetRenderTargets(8,rt,&ds);objects(rt,8);objects(&ds,1);
 mark("OM.blend");ID3D11BlendState* blend=nullptr;FLOAT factors[4]{};UINT mask=0;c->OMGetBlendState(&blend,factors,&mask);objects(&blend,1);value(factors);value(mask);
 mark("OM.depth_stencil");ID3D11DepthStencilState* depth=nullptr;UINT stencil=0;c->OMGetDepthStencilState(&depth,&stencil);objects(&depth,1);value(stencil);
 ComPtr<ID3D11Device> dev;c->GetDevice(&dev);UINT slots=dev->GetFeatureLevel()>=D3D_FEATURE_LEVEL_11_1?64:8;
 mark("CS.uav");ID3D11UnorderedAccessView* u[64]{};c->CSGetUnorderedAccessViews(0,slots,u);objects(u,slots);
 mark("OM.uav");for(auto& v:u)v=nullptr;c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,slots,u);objects(u,slots);
 mark("SO.targets");ID3D11Buffer* so[4]{};c->SOGetTargets(4,so);objects(so,4);
 mark("predicate");ID3D11Predicate* pred=nullptr;BOOL predValue=FALSE;c->GetPredication(&pred,&predValue);objects(&pred,1);value(predValue);
 }
 bool equals(const TrialPipelineState& other)const{return bytes==other.bytes;}
};
