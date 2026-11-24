// Private second-eye trial: preserve the game's selected function and original
// clear arguments; a bounded log of the native clears (observation only).
Site nativeClearSites[2];
ComPtr<ID3D11DeviceContext1> nativeClearContext1;
constexpr unsigned nativeClearCapacity=64,nativeClearEpochs=8;
struct NativeClearRecord {
 unsigned api=0,epoch=0,frame=0,state=0,width=0,height=0,samples=0,numRects=0;
 DWORD thread=0;void*ctx=nullptr;void*view=nullptr;void*resource=nullptr;void*function=nullptr;
 FLOAT rgba[4]{};D3D11_RECT rect{};bool colour=false,rtv=false,isMain=false,full=false;
};
NativeClearRecord nativeClearRecords[nativeClearCapacity];
unsigned nativeClearCount=0,nativeClearEmitted=0,nativeClearTotal=0,nativeClearStart=0;
bool nativeClearStarted=false,nativeClearDone=false,nativeClearInstalled=false;

void nativeClearFallback(unsigned api,ID3D11DeviceContext*c,ID3D11View*v,const FLOAT*colour,const D3D11_RECT*rects,UINT numRects){
 if(!v||!colour||stopping||!ownerThread||GetCurrentThreadId()!=ownerThread||
    (c!=context.Get()&&c!=(ID3D11DeviceContext*)nativeClearContext1.Get()))return;
 auto&e=secondEye;
 if(inside||e.flushing||!secondEyeLive())return;
 noteMissedClearSlots(e.guardEpoch);
 if(api){sceneClearViewNote(v,colour,rects,numRects,"call-site trace (fallback)");++traceFallbackNotes;return;}
 ComPtr<ID3D11RenderTargetView>rt;if(FAILED(v->QueryInterface(IID_PPV_ARGS(&rt))))return;
 ComPtr<ID3D11Resource>res;rt->GetResource(&res);
 e.noteSceneClear(rt.Get(),res.Get(),colour,"call-site trace (fallback)");++traceFallbackNotes;
}
void nativeClearObserve(unsigned api,ID3D11DeviceContext*c,ID3D11View*v,const FLOAT*colour,
                        const D3D11_RECT*rects,UINT numRects,void*function){
 if(!nativeClearInstalled||stopping||inside||nativeClearDone||!secondEyeLive()||!secondEye.frames||
    GetCurrentThreadId()!=ownerThread||
    (c!=context.Get()&&c!=(ID3D11DeviceContext*)nativeClearContext1.Get()))return;
 auto&e=secondEye;
 if(!nativeClearStarted){nativeClearStarted=true;nativeClearStart=e.guardEpoch;}
 if(e.guardEpoch-nativeClearStart>=nativeClearEpochs)return;
 ++nativeClearTotal;if(nativeClearCount==nativeClearCapacity)return;
 auto&r=nativeClearRecords[nativeClearCount++];r.api=api;r.epoch=e.guardEpoch;r.frame=e.frames;
 r.state=e.state;r.thread=GetCurrentThreadId();r.ctx=c;r.view=v;r.function=function;r.numRects=numRects;
 if(colour){r.colour=true;memcpy(r.rgba,colour,sizeof r.rgba);}
 if(rects&&numRects)r.rect=rects[0];
 ComPtr<ID3D11RenderTargetView>rt;ComPtr<ID3D11Resource>resource;ComPtr<ID3D11Texture2D>t;
 if(v&&SUCCEEDED(v->QueryInterface(IID_PPV_ARGS(&rt)))){
  r.rtv=true;rt->GetResource(&resource);r.resource=resource.Get();r.isMain=resource.Get()==e.mainColorRes;
  if(resource&&SUCCEEDED(resource.As(&t))){D3D11_TEXTURE2D_DESC d{};t->GetDesc(&d);
   r.width=d.Width;r.height=d.Height;r.samples=d.SampleDesc.Count;
   r.full=api==0||numRects==0||(numRects==1&&rects&&r.rect.left==0&&r.rect.top==0&&
        r.rect.right==(LONG)d.Width&&r.rect.bottom==(LONG)d.Height);
  }
 }
}
using NativeClearRt=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,ID3D11RenderTargetView*,const FLOAT*);
using NativeClearView=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext1*,ID3D11View*,const FLOAT*,const D3D11_RECT*,UINT);
void STDMETHODCALLTYPE nativeClearRt(ID3D11DeviceContext*c,ID3D11RenderTargetView*v,const FLOAT*colour,NativeClearRt next){
 LifeLock life;unsigned seen=clearObserverCalls;next(c,v,colour);
 if(nativeClearInstalled&&!stopping){MultiLock serial;
  if(clearObserverCalls==seen)nativeClearFallback(0,c,v,colour,nullptr,0);
  nativeClearObserve(0,c,v,colour,nullptr,0,(void*)next);
 }
}
void STDMETHODCALLTYPE nativeClearView(ID3D11DeviceContext1*c,ID3D11View*v,const FLOAT*colour,
                                      const D3D11_RECT*rects,UINT numRects,NativeClearView next){
 LifeLock life;unsigned seen=clearObserverCalls;next(c,v,colour,rects,numRects);
 if(nativeClearInstalled&&!stopping){MultiLock serial;
  if(clearObserverCalls==seen)nativeClearFallback(1,(ID3D11DeviceContext*)c,v,colour,rects,numRects);
  nativeClearObserve(1,(ID3D11DeviceContext*)c,v,colour,rects,numRects,(void*)next);
 }
}
void nativeClearTracePresent(){
 if(!nativeClearStarted||nativeClearDone||!logger)return;
 for(;nativeClearEmitted<nativeClearCount;++nativeClearEmitted){auto&r=nativeClearRecords[nativeClearEmitted];
  logger("native_clear: api=%s epoch=%u frame=%u state=%u thread=%lu context=%p view=%p resource=%p function=%p rtv=%u main=%u size=%ux%u msaa=%u colour_valid=%u rgba=%.9g,%.9g,%.9g,%.9g rects=%u first_rect=%ld,%ld,%ld,%ld full=%u observe_only=1\r\n",
   r.api?"ClearView":"ClearRenderTargetView",r.epoch,r.frame,r.state,r.thread,r.ctx,r.view,r.resource,r.function,
   r.rtv,r.isMain,r.width,r.height,r.samples,r.colour,r.rgba[0],r.rgba[1],r.rgba[2],r.rgba[3],
   r.numRects,r.rect.left,r.rect.top,r.rect.right,r.rect.bottom,r.full);
 }
 if(secondEye.guardEpoch-nativeClearStart+1>=nativeClearEpochs){nativeClearDone=true;
  logger("native_clear: complete epochs=%u records=%u calls=%u overflow=%u image_copies=0 logger_rendering_changes=0 scene_colour_fallback=1\r\n",
   nativeClearEpochs,nativeClearCount,nativeClearTotal,nativeClearTotal-nativeClearCount);
 }
}
bool prepareNativeClearStub(Site&s,unsigned api){
 s.stub=nearMemory(s.address);if(!s.stub)return false;
 if(!api){
  // Three original arguments, original selected function becomes fourth.
  BYTE code[]={0x4c,0x8b,0x88,0x90,0x01,0,0,0x48,0xb8,0,0,0,0,0,0,0,0,0xff,0xe0};
  uint64_t callback=(uint64_t)(ULONG_PTR)&nativeClearRt;memcpy(code+9,&callback,8);memcpy(s.stub,code,sizeof code);
 }else{
  // Preserve argument 5 before moving RSP; selected function is argument 6.
  BYTE code[]={0x4c,0x8b,0x54,0x24,0x28,0x4c,0x8b,0x98,0x20,0x04,0,0,
   0x48,0x83,0xec,0x38,0x4c,0x89,0x54,0x24,0x20,0x4c,0x89,0x5c,0x24,0x28,
   0x48,0xb8,0,0,0,0,0,0,0,0,0xff,0xd0,0x48,0x83,0xc4,0x38,0xc3};
  uint64_t callback=(uint64_t)(ULONG_PTR)&nativeClearView;memcpy(code+28,&callback,8);memcpy(s.stub,code,sizeof code);
  const BYTE unwindInfo[]={1,16,1,0,16,0x62,0,0};memcpy((BYTE*)s.stub+0x100,unwindInfo,sizeof unwindInfo);
  s.unwind.BeginAddress=0;s.unwind.EndAddress=sizeof code;s.unwind.UnwindData=0x100;
  if(!RtlAddFunctionTable(&s.unwind,1,(DWORD64)s.stub))return false;
 }
 DWORD old;if(!VirtualProtect(s.stub,4096,PAGE_EXECUTE_READ,&old))return false;
 FlushInstructionCache(GetCurrentProcess(),s.stub,4096);
 intptr_t rel=(BYTE*)s.stub-(s.address+5);if(rel<INT32_MIN||rel>INT32_MAX)return false;
 memcpy(s.patch,s.original,10);s.patch[0]=0xe8;INT32 displacement=(INT32)rel;
 memcpy(s.patch+1,&displacement,4);s.patch[5]=0x90;return true;
}
bool installNativeClearTrace(uint64_t base){
 const unsigned rvas[]={0x42964,0x42982};
 const BYTE expected[2][10]={{0xff,0x90,0x90,1,0,0,0xeb,0x1c,0x48,0x8b},
                           {0xff,0x90,0x20,4,0,0,0x48,0x83,0xc6,8}};
 for(unsigned i=0;i<2;++i){auto&s=nativeClearSites[i];s.address=(BYTE*)(ULONG_PTR)(base+rvas[i]);SIZE_T got=0;
  if(!ReadProcessMemory(GetCurrentProcess(),s.address,s.original,10,&got)||got!=10||memcmp(s.original,expected[i],10))return false;
 }
 if(FAILED(context.As(&nativeClearContext1)))return false;
 for(unsigned i=0;i<2;++i)if(!prepareNativeClearStub(nativeClearSites[i],i))return false;
 for(auto&s:nativeClearSites)if(!codeWrite(s,s.original,s.patch)){
  // Roll back only our just-installed sites. Never remove forwarding sites on stop.
  for(auto&previous:nativeClearSites)if(!memcmp(previous.address,previous.patch,10))codeWrite(previous,previous.patch,previous.original);
  return false;
 }
 nativeClearInstalled=true;return true;
}
