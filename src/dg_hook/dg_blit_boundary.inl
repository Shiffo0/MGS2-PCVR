constexpr unsigned BP_MAX=64,BP_W=320,BP_H=180,BP_BYTES=BP_W*BP_H*3,BP_BUFFERS=60,BP_BUFFER_BUDGET=262144;
struct BpBinding { uintptr_t view=0,resource=0;UINT format=0,dimension=0,mip=0,mips=0; };
struct BpBuffer {uintptr_t resource=0;UINT size=0,stride=0,offset=0,first=0,count=0;int outcome=0;std::vector<BYTE> data;};
struct BpRecord {
 uint64_t before=0,after=0,presentQpc=0,readQpc=0;DWORD thread=0;unsigned mark=0,present=0,vertices=0,first=0;
 int eye=-1,outcome=0;bool mgOriginal=false;unsigned frameBlits=0;uintptr_t target=0,vs=0,ps=0,layout=0;UINT64 vsHash=0;
 UINT sw=0,sh=0,w=0,h=0,format=0,topology=0,sampleMask=0,stencilRef=0;
 D3D11_VIEWPORT vp[16]{};UINT vpCount=16;D3D11_RECT rect[16]{};UINT rectCount=16;
 D3D11_BLEND_DESC blend{};FLOAT blendFactor[4]{};D3D11_DEPTH_STENCIL_DESC depth{};D3D11_RASTERIZER_DESC raster{};
 bool blendDefault=false,depthDefault=false,rasterDefault=false,cbRanges=false;
 UINT targetViewFormat=0;uintptr_t depthTarget=0;uintptr_t samplerPtr[16]{};D3D11_SAMPLER_DESC samplers[16]{};
 BpBinding bindings[128];BpBuffer buffers[BP_BUFFERS];std::vector<BYTE> pixels;
 double enqueueUs=0,readUs=0;
};
struct BpGpu {
 ComPtr<ID3D11Texture2D> stage[4],back;D3D11_TEXTURE2D_DESC desc{};
 ComPtr<ID3D11Buffer> buffers[BP_BUFFERS];UINT sizes[BP_BUFFERS]{};
 ComPtr<ID3D11Query> query;
 int index=-1;bool late=false;ULONGLONG deadline=0;
};
struct BpState {
 std::vector<BpRecord> records;BpGpu gpu;bool active=false,started=false,finished=false;
 unsigned n=0,burst=0,mark=0,lastPresent=0,frameBlits=0,busy=0,budget=0,unsupported=0,predicated=0,failed=0,timeouts=0,foreign=0;
 uint64_t qpf=0;char dir[MAX_PATH]{};HANDLE writer=nullptr;
} bp;
volatile LONG bpEnabled=0;
void bpTagRoute(bool sampled,bool mg){if(sampled&&bp.gpu.index>=0)bp.records[bp.gpu.index].mgOriginal=mg;}
uint64_t bpQpc(){LARGE_INTEGER q;QueryPerformanceCounter(&q);return q.QuadPart;}
double bpUs(uint64_t q){return bp.qpf?(bpQpc()-q)*1000000.0/bp.qpf:0;}
void bpRead(){
 auto&g=bp.gpu;if(g.index<0||!g.late)return;auto&r=bp.records[g.index];if(r.outcome)return;auto start=bpQpc();
 if(GetTickCount64()>g.deadline){r.outcome=4;bp.timeouts++;g.index=-1;g.back.Reset();return;}
 BOOL ready=FALSE;HRESULT hr=context->GetData(g.query.Get(),&ready,sizeof ready,D3D11_ASYNC_GETDATA_DONOTFLUSH);
 if(hr==S_FALSE||(hr==S_OK&&!ready))return;
 if(FAILED(hr)){r.outcome=3;bp.failed++;g.index=-1;g.back.Reset();return;}
 for(unsigned i=0;i<4;i++){
  D3D11_MAPPED_SUBRESOURCE m{};hr=context->Map(g.stage[i].Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
  if(hr==DXGI_ERROR_WAS_STILL_DRAWING)return;
  if(FAILED(hr)){r.outcome=3;bp.failed++;g.index=-1;g.back.Reset();return;}
  BYTE*out=r.pixels.data()+i*BP_BYTES;bool bgra=r.format==DXGI_FORMAT_B8G8R8A8_UNORM||r.format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB;
  for(unsigned y=0;y<r.h;y++)for(unsigned x=0;x<r.w;x++){
   unsigned sx=r.w>1?(unsigned)((uint64_t)x*(r.sw-1)/(r.w-1)):0,sy=r.h>1?(unsigned)((uint64_t)y*(r.sh-1)/(r.h-1)):0;
   const BYTE*p=(const BYTE*)m.pData+(size_t)sy*m.RowPitch+sx*4;*out++=p[bgra?2:0];*out++=p[1];*out++=p[bgra?0:2];
  }context->Unmap(g.stage[i].Get(),0);
 }
 for(unsigned i=0;i<BP_BUFFERS;i++)if(r.buffers[i].outcome==1){
  auto&b=r.buffers[i];D3D11_MAPPED_SUBRESOURCE m{};hr=context->Map(g.buffers[i].Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
  if(hr==DXGI_ERROR_WAS_STILL_DRAWING)return;
  if(FAILED(hr)){b.outcome=3;bp.failed++;continue;}
  try {b.data.resize(b.size);memcpy(b.data.data(),m.pData,b.size);b.outcome=2;}
  catch(...){b.outcome=3;bp.failed++;}
  context->Unmap(g.buffers[i].Get(),0);
 }
 r.readQpc=bpQpc();r.readUs=bpUs(start);r.outcome=1;g.index=-1;g.back.Reset();
}
void bpBuffer(unsigned slot,ID3D11Buffer*source,UINT stride,UINT offset,UINT first,UINT count,unsigned&total,BpRecord&r){
 if(!source)return;auto&b=r.buffers[slot];b.resource=(uintptr_t)source;b.stride=stride;b.offset=offset;b.first=first;b.count=count;
 D3D11_BUFFER_DESC d{};source->GetDesc(&d);b.size=d.ByteWidth;
 if(!b.size||b.size>BP_BUFFER_BUDGET-total){b.outcome=4;return;}total+=b.size;
 auto&g=bp.gpu;
 if(g.sizes[slot]!=b.size){g.buffers[slot].Reset();g.sizes[slot]=b.size;}
 if(!g.buffers[slot]){d.Usage=D3D11_USAGE_STAGING;d.BindFlags=0;d.CPUAccessFlags=D3D11_CPU_ACCESS_READ;d.MiscFlags=0;d.StructureByteStride=0;
  if(FAILED(device->CreateBuffer(&d,nullptr,&g.buffers[slot]))){b.outcome=3;bp.failed++;return;}}
 context->CopyResource(g.buffers[slot].Get(),source);b.outcome=1;
}
void bpState(BpRecord&r){
 ComPtr<ID3D11VertexShader>vs;ComPtr<ID3D11PixelShader>ps;context->VSGetShader(&vs,nullptr,nullptr);context->PSGetShader(&ps,nullptr,nullptr);
 r.vs=(uintptr_t)vs.Get();r.ps=(uintptr_t)ps.Get();r.vsHash=vsHashOf(vs.Get(),nullptr,nullptr);
 ComPtr<ID3D11InputLayout>layout;context->IAGetInputLayout(&layout);r.layout=(uintptr_t)layout.Get();D3D11_PRIMITIVE_TOPOLOGY topology;context->IAGetPrimitiveTopology(&topology);r.topology=topology;
 context->RSGetViewports(&r.vpCount,r.vp);context->RSGetScissorRects(&r.rectCount,r.rect);
 ComPtr<ID3D11BlendState>blend;context->OMGetBlendState(&blend,r.blendFactor,&r.sampleMask);r.blendDefault=!blend;if(blend)blend->GetDesc(&r.blend);
 ComPtr<ID3D11DepthStencilState>depth;context->OMGetDepthStencilState(&depth,&r.stencilRef);r.depthDefault=!depth;if(depth)depth->GetDesc(&r.depth);
 ComPtr<ID3D11RasterizerState>raster;context->RSGetState(&raster);r.rasterDefault=!raster;if(raster)raster->GetDesc(&r.raster);
 ID3D11ShaderResourceView*views[128]{};context->PSGetShaderResources(0,128,views);
 for(unsigned i=0;i<128;i++)if(views[i]){auto&b=r.bindings[i];D3D11_SHADER_RESOURCE_VIEW_DESC d{};views[i]->GetDesc(&d);ComPtr<ID3D11Resource>res;views[i]->GetResource(&res);
  b.view=(uintptr_t)views[i];b.resource=(uintptr_t)res.Get();b.format=d.Format;b.dimension=d.ViewDimension;
  if(d.ViewDimension==D3D11_SRV_DIMENSION_TEXTURE2D){b.mip=d.Texture2D.MostDetailedMip;b.mips=d.Texture2D.MipLevels;}views[i]->Release();}
 ID3D11SamplerState*samplers[16]{};context->PSGetSamplers(0,16,samplers);
 for(unsigned i=0;i<16;i++)if(samplers[i]){r.samplerPtr[i]=(uintptr_t)samplers[i];samplers[i]->GetDesc(&r.samplers[i]);samplers[i]->Release();}
 ComPtr<ID3D11DepthStencilView>dsv;context->OMGetRenderTargets(0,nullptr,&dsv);if(dsv){ComPtr<ID3D11Resource>res;dsv->GetResource(&res);r.depthTarget=(uintptr_t)res.Get();}
 unsigned total=0;ComPtr<ID3D11DeviceContext1>c1;context.As(&c1);r.cbRanges=!!c1;
 for(unsigned stage=0;stage<2;stage++){
  ID3D11Buffer*buf[14]{};UINT first[14]{},count[14]{};
  if(c1){if(stage)c1->PSGetConstantBuffers1(0,14,buf,first,count);else c1->VSGetConstantBuffers1(0,14,buf,first,count);}
  else{if(stage)context->PSGetConstantBuffers(0,14,buf);else context->VSGetConstantBuffers(0,14,buf);}
  for(unsigned i=0;i<14;i++)if(buf[i]){bpBuffer(stage*14+i,buf[i],0,0,first[i],count[i],total,r);buf[i]->Release();}
 }
 ID3D11Buffer*vb[32]{};UINT stride[32]{},offset[32]{};context->IAGetVertexBuffers(0,32,vb,stride,offset);
 for(unsigned i=0;i<32;i++)if(vb[i]){bpBuffer(28+i,vb[i],stride[i],offset[i],0,0,total,r);vb[i]->Release();}
}
bool bpBefore(ID3D11DeviceContext*c,UINT n,UINT first){
 if(!bp.active||n!=4||c!=context.Get()||stopping)return false;
 bpRead();ComPtr<ID3D11RenderTargetView>rt;context->OMGetRenderTargets(1,&rt,nullptr);if(!rt)return false;
 ComPtr<ID3D11Resource>target;rt->GetResource(&target);if(target.Get()!=ui2dBackbufferPtr)return false;
 ++bp.frameBlits;if(bp.frameBlits!=1||!bp.burst)return false;
 if(bp.gpu.index>=0){bp.busy++;return false;}if(bp.n==BP_MAX){bp.budget++;bp.burst=0;return false;}
 ComPtr<ID3D11Predicate>pred;BOOL pv;context->GetPredication(&pred,&pv);if(pred){bp.predicated++;return false;}
 ComPtr<ID3D11ShaderResourceView>srv;context->PSGetShaderResources(0,1,&srv);
 ComPtr<ID3D11Resource>source;if(srv)srv->GetResource(&source);
 ComPtr<ID3D11Texture2D>back,src;if(FAILED(target.As(&back))||!source||FAILED(source.As(&src))){bp.unsupported++;return false;}
 D3D11_TEXTURE2D_DESC d{},sd{};back->GetDesc(&d);src->GetDesc(&sd);
 D3D11_RENDER_TARGET_VIEW_DESC rd{};rt->GetDesc(&rd);D3D11_SHADER_RESOURCE_VIEW_DESC sv{};srv->GetDesc(&sv);
 auto supported=[](const D3D11_TEXTURE2D_DESC&x){return x.Width&&x.Height&&x.Width<=4096&&x.Height<=4096&&x.ArraySize==1&&x.MipLevels==1&&x.SampleDesc.Count==1&&!x.SampleDesc.Quality&&(x.Format==DXGI_FORMAT_B8G8R8A8_UNORM||x.Format==DXGI_FORMAT_R8G8B8A8_UNORM||x.Format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB||x.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB);};
 if(!supported(d)||!supported(sd)||d.Width!=sd.Width||d.Height!=sd.Height||d.Format!=sd.Format||rd.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2D||rd.Texture2D.MipSlice||sv.ViewDimension!=D3D11_SRV_DIMENSION_TEXTURE2D||sv.Texture2D.MostDetailedMip){bp.unsupported++;return false;}
 auto begin=bpQpc();auto&g=bp.gpu;
 if(g.desc.Width!=d.Width||g.desc.Height!=d.Height||g.desc.Format!=d.Format){for(auto&t:g.stage)t.Reset();g.desc=d;}
 d.Usage=D3D11_USAGE_STAGING;d.BindFlags=0;d.MiscFlags=0;d.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
 for(auto&t:g.stage)if(!t&&FAILED(device->CreateTexture2D(&d,nullptr,&t))){bp.failed++;return false;}
 if(!g.query){D3D11_QUERY_DESC q{D3D11_QUERY_EVENT,0};if(FAILED(device->CreateQuery(&q,&g.query))){bp.failed++;return false;}}
 auto&r=bp.records[bp.n];r.before=begin;r.thread=GetCurrentThreadId();r.mark=bp.mark;r.vertices=n;r.first=first;r.target=(uintptr_t)back.Get();
 r.targetViewFormat=rd.Format;r.sw=d.Width;r.sh=d.Height;r.format=d.Format;r.w=d.Width<BP_W?d.Width:BP_W;r.h=(UINT)((uint64_t)d.Height*r.w/d.Width);
 if(r.h>BP_H){r.h=BP_H;r.w=(UINT)((uint64_t)d.Width*BP_H/d.Height);}if(!r.w||!r.h){bp.unsupported++;return false;}
 context->CopyResource(g.stage[0].Get(),src.Get());context->CopyResource(g.stage[1].Get(),back.Get());bpState(r);
 g.index=(int)bp.n++;g.back=back;g.late=false;bp.burst--;r.enqueueUs=bpUs(begin);return true;
}
void bpAfter(bool sampled){if(!sampled)return;auto start=bpQpc();auto&g=bp.gpu;auto&r=bp.records[g.index];context->CopyResource(g.stage[2].Get(),g.back.Get());r.after=bpQpc();r.enqueueUs+=bpUs(start);wwEnsureHooks();wwOpen((unsigned)g.index,r.mark,context.Get(),g.back.Get());}
void bpOnPresent(){
 wwClose(context.Get());bpRead();auto&g=bp.gpu;if(g.index<0||g.late)return;auto start=bpQpc();auto&r=bp.records[g.index];
 ComPtr<ID3D11Predicate>pred;BOOL pv;context->GetPredication(&pred,&pv);
 if(pred){r.outcome=6;bp.predicated++;g.late=true;r.presentQpc=bpQpc();return;}
 context->CopyResource(g.stage[3].Get(),g.back.Get());context->End(g.query.Get());g.late=true;g.deadline=GetTickCount64()+1000;r.presentQpc=bpQpc();r.frameBlits=bp.frameBlits;r.enqueueUs+=bpUs(start);
}
void bpHex(FILE*f,const void*p,size_t n){auto b=(const BYTE*)p;fputc('"',f);for(size_t i=0;i<n;i++)fprintf(f,"%02X",b[i]);fputc('"',f);}
DWORD WINAPI bpWrite(void*){
 char path[MAX_PATH];FILE*f=nullptr;bool ok=true;sprintf_s(path,"%s\\blit.jsonl",bp.dir);if(fopen_s(&f,path,"wx")||!f)return 1;
 fprintf(f,"{\"type\":\"header\",\"schema\":1,\"qpf\":%llu,\"planes\":[\"source_before\",\"back_before\",\"back_after\",\"back_present\"],\"buffer_slots\":\"0-13 VS CB;14-27 PS CB;28-59 IA VB\"}\n",bp.qpf);
 for(unsigned k=0;k<bp.n;k++){auto&r=bp.records[k];
  fprintf(f,"{\"type\":\"blit\",\"sample\":%u,\"present\":%u,\"eye\":%d,\"mark\":%u,\"outcome\":%d,\"qpc_before\":%llu,\"qpc_after\":%llu,\"qpc_present\":%llu,\"qpc_read\":%llu,\"thread\":%lu,\"frame_blits\":%u,\"vertices\":%u,\"first_vertex\":%u,\"source_width\":%u,\"source_height\":%u,\"width\":%u,\"height\":%u,\"format\":%u,\"enqueue_us\":%.3f,\"read_us\":%.3f,\"target\":\"%llX\",\"vs\":\"%llX\",\"vs_hash\":\"%016llX\",\"ps\":\"%llX\",\"layout\":\"%llX\",\"topology\":%u,\"sample_mask\":%u,\"stencil_ref\":%u,\"blend_default\":%d,\"depth_default\":%d,\"raster_default\":%d,\"cb_ranges\":%d,",k,r.present,r.eye,r.mark,r.outcome,r.before,r.after,r.presentQpc,r.readQpc,r.thread,r.frameBlits,r.vertices,r.first,r.sw,r.sh,r.w,r.h,r.format,r.enqueueUs,r.readUs,(unsigned long long)r.target,(unsigned long long)r.vs,r.vsHash,(unsigned long long)r.ps,(unsigned long long)r.layout,r.topology,r.sampleMask,r.stencilRef,r.blendDefault,r.depthDefault,r.rasterDefault,r.cbRanges);
  fprintf(f,"\"native_route\":\"%s\",",r.mgOriginal?"MGSHDFix-original":"D3D11-direct");
  fprintf(f,"\"target_view_format\":%u,\"depth_target\":\"%llX\",\"ps_samplers\":[",r.targetViewFormat,(unsigned long long)r.depthTarget);
  bool sc=false;for(unsigned i=0;i<16;i++)if(r.samplerPtr[i]){if(sc)fputc(',',f);sc=true;fprintf(f,"{\"slot\":%u,\"object\":\"%llX\",\"desc_hex\":",i,(unsigned long long)r.samplerPtr[i]);bpHex(f,&r.samplers[i],sizeof r.samplers[i]);fputc('}',f);}fputs("],\"blend_desc_hex\":",f);bpHex(f,&r.blend,sizeof r.blend);fputs(",\"blend_factor_hex\":",f);bpHex(f,r.blendFactor,sizeof r.blendFactor);fputs(",\"depth_desc_hex\":",f);bpHex(f,&r.depth,sizeof r.depth);fputs(",\"raster_desc_hex\":",f);bpHex(f,&r.raster,sizeof r.raster);fprintf(f,",\"viewport_count\":%u,\"viewports_hex\":",r.vpCount);bpHex(f,r.vp,r.vpCount*sizeof r.vp[0]);fprintf(f,",\"scissor_count\":%u,\"scissors_hex\":",r.rectCount);bpHex(f,r.rect,r.rectCount*sizeof r.rect[0]);
  fputs(",\"ps_resources\":[",f);bool comma=false;for(unsigned i=0;i<128;i++)if(r.bindings[i].view){auto&v=r.bindings[i];if(comma)fputc(',',f);comma=true;fprintf(f,"{\"slot\":%u,\"view\":\"%llX\",\"resource\":\"%llX\",\"format\":%u,\"dimension\":%u,\"mip\":%u,\"mips\":%u}",i,(unsigned long long)v.view,(unsigned long long)v.resource,v.format,v.dimension,v.mip,v.mips);}fputs("],\"buffers\":[",f);
  comma=false;for(unsigned i=0;i<BP_BUFFERS;i++)if(r.buffers[i].resource){auto&b=r.buffers[i];if(comma)fputc(',',f);comma=true;fprintf(f,"{\"slot\":%u,\"resource\":\"%llX\",\"size\":%u,\"stride\":%u,\"offset\":%u,\"first_constant\":%u,\"constant_count\":%u,\"outcome\":%d}",i,(unsigned long long)b.resource,b.size,b.stride,b.offset,b.first,b.count,b.outcome);
   if(r.outcome==1&&b.outcome==2){FILE*o=nullptr;sprintf_s(path,"%s\\blit_%03u_buf_%02u.bin",bp.dir,k,i);if(fopen_s(&o,path,"wbx")||!o){ok=false;continue;}if(fwrite(b.data.data(),1,b.data.size(),o)!=b.data.size())ok=false;if(fclose(o))ok=false;}}
  fputs("]}\n",f);
  if(r.outcome==1)for(unsigned i=0;i<4;i++){FILE*o=nullptr;sprintf_s(path,"%s\\blit_%03u_%u.ppm",bp.dir,k,i);if(fopen_s(&o,path,"wbx")||!o){ok=false;continue;}fprintf(o,"P6\n%u %u\n255\n",r.w,r.h);if(fwrite(r.pixels.data()+i*BP_BYTES,3,(size_t)r.w*r.h,o)!=(size_t)r.w*r.h)ok=false;if(fclose(o))ok=false;}
 }
 fprintf(f,"{\"type\":\"summary\",\"samples\":%u,\"busy\":%u,\"budget\":%u,\"unsupported\":%u,\"predicated\":%u,\"failed\":%u,\"timeouts\":%u,\"foreign_api\":%u}\n",bp.n,bp.busy,bp.budget,bp.unsupported,bp.predicated,bp.failed,bp.timeouts,bp.foreign);
 wwWrite(f);if(ferror(f))ok=false;if(fclose(f))ok=false;if(ok){sprintf_s(path,"%s\\BLIT_COMPLETE.txt",bp.dir);if(!fopen_s(&f,path,"wx")&&f){fputs("Bounded blit writer complete; not headset acceptance.\n",f);if(fclose(f))ok=false;}else ok=false;}
 if(logger)logger("  blit boundary: %s samples=%u -> %s\r\n",ok?"saved":"WRITE FAILED",bp.n,bp.dir);return ok?0:1;
}
void bpFinish(bool abandon){
 if(bp.finished||!bp.started||bp.active)return;
 if(bp.gpu.index>=0){if(!abandon)return;bp.records[bp.gpu.index].outcome=5;bp.gpu.index=-1;bp.gpu.back.Reset();}
 wwFreeze();bp.finished=true;bp.writer=CreateThread(nullptr,0,bpWrite,nullptr,0,nullptr);if(!bp.writer&&logger)logger("  blit boundary: writer creation FAILED\r\n");
}
void bpControl(const char*dir,int active,unsigned mark,unsigned present,int eye){
 if(bp.finished)return;
 if(!bp.started&&active){if(!dir||!*dir||strcpy_s(bp.dir,dir))return;
  try{bp.records.resize(BP_MAX);for(auto&r:bp.records)r.pixels.resize(4*BP_BYTES);}catch(...){bp.finished=true;bp.failed++;return;}
  LARGE_INTEGER q;QueryPerformanceFrequency(&q);bp.qpf=q.QuadPart;bp.started=true;bp.burst=6;bp.mark=mark;
  if(logger)logger("  blit boundary: started max=%u planes=4 next-frame capture, -> %s\r\n",BP_MAX,bp.dir);
 }
 if(!bp.started)return;
 if(active)wwEnsureHooks();
 // This callback belongs to the Present just observed, before the next draw.
 if(bp.gpu.index>=0&&bp.gpu.late){auto&r=bp.records[bp.gpu.index];if(!r.present){r.present=present;r.eye=eye;}if(r.outcome){bp.gpu.index=-1;bp.gpu.back.Reset();}}
 bp.lastPresent=present;bp.frameBlits=0;bp.active=active!=0;
 if(bp.active&&mark!=bp.mark){bp.mark=mark;bp.burst=6;if(logger)logger("  blit boundary: mark=%u armed after Present=%u burst=6\r\n",mark,present);}
 InterlockedExchange(&bpEnabled,bp.active&&(bp.burst||bp.gpu.index>=0)?1:0);
 bpFinish(false);
}
void bpStop(){InterlockedExchange(&bpEnabled,0);bp.active=false;bpFinish(true);if(bp.writer){WaitForSingleObject(bp.writer,INFINITE);CloseHandle(bp.writer);bp.writer=nullptr;}bp.gpu=BpGpu{};}
