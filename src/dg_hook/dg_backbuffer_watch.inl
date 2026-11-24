/* Bounded metadata-only backbuffer writer watch. Native calls are forwarded
 * exactly once, with no observer lock held across the call. Snapshots describe
 * call attempts/completion, NOT successful pixel writes. No pixel copy here. */
constexpr unsigned WW_EVENTS=4096,WW_ADDRESSES=64,WW_HOPS=6,WW_SETS=4,WW_APIS=10;
static const unsigned wwSlots[WW_APIS]={12,13,20,21,38,39,40,50,58,30};
static const char*wwNames[]={"DrawIndexed","Draw","DrawIndexedInstanced","DrawInstanced","DrawAuto","DrawIndexedInstancedIndirect","DrawInstancedIndirect","ClearRenderTargetView","ExecuteCommandList","CopyResource","CopySubresourceRegion","ResolveSubresource","SetPredication"};
struct WwHop {uintptr_t address=0,next=0,module=0,allocation=0;DWORD protect=0,type=0;BYTE bytes[16]{};unsigned read=0;char path[MAX_PATH]{};};
struct WwAddress {uintptr_t address=0;unsigned mark=0,hits=0,hops=0;bool rejectedEntry=false,rejectedWrapper=false;WwHop chain[WW_HOPS];};
struct WwEvent {uint64_t begin=0,end=0;unsigned sample=0,kind=0;DWORD thread=0;uintptr_t context=0,dst=0,src=0,api=0,caller=0,predicate=0;unsigned apiId=0,callerId=0,beginMask=0,endMask=0;uint64_t args[6]{};UINT box[6]{};unsigned boxState=0;bool possibleOnly=false,predicateValue=false,completed=false;};
struct WwWindow {uint64_t begin=0,end=0;uintptr_t back=0,context=0;unsigned mark=0,beginMask=0,endMask=0;bool closed=false;};
struct WwSet {void**vt=nullptr;Hook h[WW_APIS];};
static struct {
 SRWLOCK lock=SRWLOCK_INIT;volatile LONG window=0;bool frozen=false;
 unsigned events=0,eventOverflow=0,addresses=0,addressOverflow=0,windows=0,foreign=0,ignoredContext=0,installFailures=0;
 WwEvent event[WW_EVENTS];WwAddress address[WW_ADDRESSES];WwWindow win[64];
} ww;
WwSet wwSets[WW_SETS];unsigned wwSetCount=0;
uint64_t wwQpc(){LARGE_INTEGER q;QueryPerformanceCounter(&q);return q.QuadPart;}
unsigned wwCopyMask(void**);void wwRestoreCopies(void**);unsigned wwMask(ID3D11DeviceContext*);
bool wwRead(void*p,void*out,SIZE_T n){SIZE_T got=0;return p&&ReadProcessMemory(GetCurrentProcess(),p,out,n,&got)&&got==n;}
// Read bytes only; never execute an unknown trampoline. Unknown opcode ends
// the chain honestly. References keep each module alive while naming it.
void wwResolve(WwAddress&a,void*root){
 void*p=root;
 for(unsigned i=0;i<WW_HOPS&&p;i++){
  bool loop=false;for(unsigned j=0;j<i;j++)if(a.chain[j].address==(uintptr_t)p)loop=true;if(loop)break;
  auto&h=a.chain[a.hops++];h.address=(uintptr_t)p;MEMORY_BASIC_INFORMATION m{};
  if(VirtualQuery(p,&m,sizeof m)){h.allocation=(uintptr_t)m.AllocationBase;h.protect=m.Protect;h.type=m.Type;}
  HMODULE module=nullptr;if(GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,(LPCSTR)p,&module)){
   h.module=(uintptr_t)module;DWORD n=GetModuleFileNameA(module,h.path,MAX_PATH);if(n>=MAX_PATH)h.path[MAX_PATH-1]=0;FreeLibrary(module);}
  if(!wwRead(p,h.bytes,16))break;h.read=16;
  auto*b=h.bytes;void*next=nullptr;
  if(b[0]==0xe9){INT32 d;memcpy(&d,b+1,4);next=(BYTE*)p+5+d;}
  else if(b[0]==0xeb)next=(BYTE*)p+2+(signed char)b[1];
  else if(b[0]==0xff&&b[1]==0x25){INT32 d;memcpy(&d,b+2,4);wwRead((BYTE*)p+6+d,&next,sizeof next);}
  else if(b[0]==0x48&&b[1]==0xb8&&b[10]==0xff&&b[11]==0xe0)memcpy(&next,b+2,8);
  else if(b[0]==0x49&&b[1]==0xbb&&b[10]==0x41&&b[11]==0xff&&b[12]==0xe3)memcpy(&next,b+2,8);
  h.next=(uintptr_t)next;p=next;
 }
}
// Caller holds ww.lock. Each address is resampled once per marker window.
unsigned wwAddress(void*p,unsigned mark){
 if(!p)return 0;
 for(unsigned i=0;i<ww.addresses;i++)if(ww.address[i].address==(uintptr_t)p&&ww.address[i].mark==mark){ww.address[i].hits++;return i+1;}
 if(ww.addresses==WW_ADDRESSES){ww.addressOverflow++;return 0;}
 auto&a=ww.address[ww.addresses++];a.address=(uintptr_t)p;a.mark=mark;a.hits=1;wwResolve(a,p);return ww.addresses;
}
void wwForeign(void*api,void*wrapper,unsigned mark){
 AcquireSRWLockExclusive(&ww.lock);if(!ww.frozen){ww.foreign++;unsigned a=wwAddress(api,mark);if(a)ww.address[a-1].rejectedEntry=true;if(wrapper!=api){unsigned w=wwAddress(wrapper,mark);if(w)ww.address[w-1].rejectedWrapper=true;}}ReleaseSRWLockExclusive(&ww.lock);
}
int wwBegin(unsigned kind,ID3D11DeviceContext*c,ID3D11Resource*dst,ID3D11Resource*src,void*api,void*caller,uint64_t a=0,uint64_t b=0,uint64_t d=0,uint64_t e=0,uint64_t f=0,uint64_t g=0){
 LONG window=InterlockedCompareExchange(&ww.window,0,0);if(!window||kind>12)return -1;
 uintptr_t target=0,owner=0;unsigned mark=0;
 AcquireSRWLockShared(&ww.lock);if(!ww.frozen&&ww.window==window){target=ww.win[window-1].back;owner=ww.win[window-1].context;mark=ww.win[window-1].mark;}ReleaseSRWLockShared(&ww.lock);
 if(!target)return -1;
 if(owner!=(uintptr_t)c){AcquireSRWLockExclusive(&ww.lock);if(!ww.frozen)ww.ignoredContext++;ReleaseSRWLockExclusive(&ww.lock);return -1;}
 bool relevant=kind==8||kind==12;
 if(kind<=6){ID3D11RenderTargetView*rt[8]{};c->OMGetRenderTargets(8,rt,nullptr);for(auto*v:rt)if(v){ID3D11Resource*r=nullptr;v->GetResource(&r);if((uintptr_t)r==target){relevant=true;dst=r;}if(r)r->Release();v->Release();}}
 else if(kind!=8&&kind!=12)relevant=(uintptr_t)dst==target;
 if(!relevant)return -1;
 ID3D11Predicate*pred=nullptr;BOOL value=FALSE;c->GetPredication(&pred,&value);uintptr_t predicate=(uintptr_t)pred;if(pred)pred->Release();
 unsigned mask=wwMask(c);int ticket=-1;AcquireSRWLockExclusive(&ww.lock);
 if(!ww.frozen&&ww.window==window){
  if(ww.events==WW_EVENTS)ww.eventOverflow++;
  else{ticket=(int)ww.events++;auto&r=ww.event[ticket];r.begin=wwQpc();r.sample=window-1;r.kind=kind;r.thread=GetCurrentThreadId();r.context=(uintptr_t)c;r.dst=(uintptr_t)dst;r.src=(uintptr_t)src;r.api=(uintptr_t)api;r.caller=(uintptr_t)caller;r.predicate=predicate;r.predicateValue=value!=FALSE;r.possibleOnly=kind==8||kind==12;r.beginMask=mask;r.args[0]=a;r.args[1]=b;r.args[2]=d;r.args[3]=e;r.args[4]=f;r.args[5]=g;r.apiId=wwAddress(api,mark);r.callerId=wwAddress(caller,mark);}
 }ReleaseSRWLockExclusive(&ww.lock);return ticket;
}
void wwEnd(int ticket){if(ticket<0)return;AcquireSRWLockExclusive(&ww.lock);if(!ww.frozen&&(unsigned)ticket<ww.events){ww.event[ticket].end=wwQpc();ww.event[ticket].completed=true;ww.event[ticket].endMask=wwMask((ID3D11DeviceContext*)ww.event[ticket].context);}ReleaseSRWLockExclusive(&ww.lock);}
void wwBox(int ticket,const D3D11_BOX*box){if(ticket<0||!box)return;UINT data[6]{};bool valid=wwRead((void*)box,data,sizeof data);AcquireSRWLockExclusive(&ww.lock);if(!ww.frozen&&(unsigned)ticket<ww.events){ww.event[ticket].boxState=valid?1:2;if(valid)memcpy(ww.event[ticket].box,data,sizeof data);}ReleaseSRWLockExclusive(&ww.lock);}
template<unsigned S>void STDMETHODCALLTYPE wwDI(ID3D11DeviceContext*c,UINT n,UINT first,INT base){auto f=(Indexed)wwSets[S].h[0].next;int t=wwBegin(0,c,nullptr,nullptr,(void*)f,_ReturnAddress(),n,first,(uint64_t)(int64_t)base);f(c,n,first,base);wwEnd(t);}
template<unsigned S>void STDMETHODCALLTYPE wwD(ID3D11DeviceContext*c,UINT n,UINT first){auto f=(Draw)wwSets[S].h[1].next;int t=wwBegin(1,c,nullptr,nullptr,(void*)f,_ReturnAddress(),n,first);f(c,n,first);wwEnd(t);}
using WwDII=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,UINT,UINT,UINT,INT,UINT);
template<unsigned S>void STDMETHODCALLTYPE wwDII(ID3D11DeviceContext*c,UINT a,UINT b,UINT d,INT e,UINT f0){auto f=(WwDII)wwSets[S].h[2].next;int t=wwBegin(2,c,nullptr,nullptr,(void*)f,_ReturnAddress(),a,b,d,(uint64_t)(int64_t)e,f0);f(c,a,b,d,e,f0);wwEnd(t);}
using WwDI2=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,UINT,UINT,UINT,UINT);
template<unsigned S>void STDMETHODCALLTYPE wwDI2(ID3D11DeviceContext*c,UINT a,UINT b,UINT d,UINT e){auto f=(WwDI2)wwSets[S].h[3].next;int t=wwBegin(3,c,nullptr,nullptr,(void*)f,_ReturnAddress(),a,b,d,e);f(c,a,b,d,e);wwEnd(t);}
using WwAuto=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*);
template<unsigned S>void STDMETHODCALLTYPE wwA(ID3D11DeviceContext*c){auto f=(WwAuto)wwSets[S].h[4].next;int t=wwBegin(4,c,nullptr,nullptr,(void*)f,_ReturnAddress());f(c);wwEnd(t);}
using WwIndirect=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,ID3D11Buffer*,UINT);
template<unsigned S,unsigned K>void STDMETHODCALLTYPE wwIndirect(ID3D11DeviceContext*c,ID3D11Buffer*b,UINT offset){auto f=(WwIndirect)wwSets[S].h[K].next;int t=wwBegin(K,c,nullptr,b,(void*)f,_ReturnAddress(),offset);f(c,b,offset);wwEnd(t);}
using WwClear=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,ID3D11RenderTargetView*,const FLOAT*);
template<unsigned S>void STDMETHODCALLTYPE wwClear(ID3D11DeviceContext*c,ID3D11RenderTargetView*v,const FLOAT*color){auto f=(WwClear)wwSets[S].h[7].next;if(!InterlockedCompareExchange(&ww.window,0,0)){f(c,v,color);return;}ComPtr<ID3D11Resource>r;if(InterlockedCompareExchange(&ww.window,0,0)&&v)v->GetResource(&r);UINT bits[4]{};wwRead((void*)color,bits,sizeof bits);int t=wwBegin(7,c,r.Get(),nullptr,(void*)f,_ReturnAddress(),bits[0],bits[1],bits[2],bits[3]);f(c,v,color);wwEnd(t);}
using WwExecute=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,ID3D11CommandList*,BOOL);
template<unsigned S>void STDMETHODCALLTYPE wwExecute(ID3D11DeviceContext*c,ID3D11CommandList*list,BOOL restore){auto f=(WwExecute)wwSets[S].h[8].next;int t=wwBegin(8,c,nullptr,nullptr,(void*)f,_ReturnAddress(),(uintptr_t)list,restore);f(c,list,restore);wwEnd(t);}
using WwPred=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,ID3D11Predicate*,BOOL);
template<unsigned S>void STDMETHODCALLTYPE wwPred(ID3D11DeviceContext*c,ID3D11Predicate*p,BOOL value){auto f=(WwPred)wwSets[S].h[9].next;int t=wwBegin(12,c,nullptr,nullptr,(void*)f,_ReturnAddress(),(uintptr_t)p,value);f(c,p,value);wwEnd(t);}
template<unsigned S>void wwFunctions(void**f){f[0]=(void*)wwDI<S>;f[1]=(void*)wwD<S>;f[2]=(void*)wwDII<S>;f[3]=(void*)wwDI2<S>;f[4]=(void*)wwA<S>;f[5]=(void*)wwIndirect<S,5>;f[6]=(void*)wwIndirect<S,6>;f[7]=(void*)wwClear<S>;f[8]=(void*)wwExecute<S>;f[9]=(void*)wwPred<S>;}
// Unwrap ONLY this recorder's exact callbacks, never arbitrary foreign hooks.
void*wwUnwrap(void*p){for(unsigned depth=0;depth<WW_SETS;depth++){void*next=p;for(unsigned i=0;i<wwSetCount;i++)for(auto&h:wwSets[i].h)if(h.ours==p&&h.next)next=h.next;if(next==p)break;p=next;}return p;}
void wwEnsureHooks(){
 if(!context||stopping)return;MultiLock multiLock;void**vt=*(void***)context.Get();
 for(unsigned i=0;i<wwSetCount;i++)if(wwSets[i].vt==vt){
  // Restore ONLY an exact original entry, never a different replacement.
  for(auto&h:wwSets[i].h)if(h.slot&&*h.slot==h.next&&!writeSlot(h,h.next,h.ours))ww.installFailures++;
  wwRestoreCopies(vt);return;
 }
 if(wwSetCount==WW_SETS){ww.installFailures++;return;}
 unsigned s=wwSetCount;void*f[WW_APIS]{};switch(s){case 0:wwFunctions<0>(f);break;case 1:wwFunctions<1>(f);break;case 2:wwFunctions<2>(f);break;case 3:wwFunctions<3>(f);break;}
 auto&set=wwSets[s];set.vt=vt;for(unsigned k=0;k<WW_APIS;k++)set.h[k]={vt+wwSlots[k],wwUnwrap(vt[wwSlots[k]]),f[k]};
 wwSetCount++;for(unsigned k=0;k<WW_APIS;k++){auto&h=set.h[k];void*expected=*h.slot;if(!h.next||!writeSlot(h,expected,h.ours)){h.slot=nullptr;ww.installFailures++;}}
 wwRestoreCopies(vt);
}
unsigned wwMask(ID3D11DeviceContext*c){
 if(!c)return 0;void**vt=*(void***)c;unsigned mask=wwCopyMask(vt);
 for(unsigned k=0;k<WW_APIS;k++)for(unsigned s=0;s<wwSetCount;s++)if(vt[wwSlots[k]]==wwSets[s].h[k].ours)mask|=1u<<(k==9?12:k);
 return mask;
}
void wwOpen(unsigned sample,unsigned mark,ID3D11DeviceContext*c,ID3D11Resource*back){
 if(sample>=64)return;unsigned mask=wwMask(c);AcquireSRWLockExclusive(&ww.lock);if(!ww.frozen){auto&w=ww.win[sample];w.begin=wwQpc();w.back=(uintptr_t)back;w.context=(uintptr_t)c;w.mark=mark;w.beginMask=mask;if(sample>=ww.windows)ww.windows=sample+1;InterlockedExchange(&ww.window,(LONG)sample+1);}ReleaseSRWLockExclusive(&ww.lock);
}
void wwClose(ID3D11DeviceContext*c){unsigned mask=wwMask(c);AcquireSRWLockExclusive(&ww.lock);LONG n=InterlockedExchange(&ww.window,0);if(n&&!ww.frozen){auto&w=ww.win[n-1];w.end=wwQpc();w.endMask=mask;w.closed=true;}ReleaseSRWLockExclusive(&ww.lock);}
void wwFreeze(){AcquireSRWLockExclusive(&ww.lock);InterlockedExchange(&ww.window,0);ww.frozen=true;ReleaseSRWLockExclusive(&ww.lock);}
void wwRemove(){wwFreeze();for(unsigned s=0;s<wwSetCount;s++)for(auto&h:wwSets[s].h)if(h.slot&&*h.slot==h.ours)writeSlot(h,h.ours,h.next);}
void wwString(FILE*f,const char*s){fputc('"',f);for(auto*p=(const unsigned char*)s;*p;p++){if(*p=='"'||*p=='\\')fputc('\\',f);if(*p<32||*p>=127)fprintf(f,"\\u%04x",*p);else fputc(*p,f);}fputc('"',f);}
void wwWrite(FILE*f){
 for(unsigned i=0;i<ww.addresses;i++){auto&a=ww.address[i];fprintf(f,"{\"type\":\"writer_address\",\"id\":%u,\"address\":\"%llX\",\"mark\":%u,\"hits\":%u,\"rejected_entry\":%d,\"rejected_wrapper\":%d,\"chain\":[",i+1,(unsigned long long)a.address,a.mark,a.hits,a.rejectedEntry,a.rejectedWrapper);
  for(unsigned j=0;j<a.hops;j++){auto&h=a.chain[j];if(j)fputc(',',f);fprintf(f,"{\"address\":\"%llX\",\"next\":\"%llX\",\"module_base\":\"%llX\",\"allocation_base\":\"%llX\",\"memory_type\":%lu,\"protect\":%lu,\"bytes\":\"",(unsigned long long)h.address,(unsigned long long)h.next,(unsigned long long)h.module,(unsigned long long)h.allocation,h.type,h.protect);for(unsigned k=0;k<h.read;k++)fprintf(f,"%02X",h.bytes[k]);fputs("\",\"module\":",f);wwString(f,h.path);fputc('}',f);}fputs("]}\n",f);}
 for(unsigned i=0;i<ww.windows;i++){auto&w=ww.win[i];if(!w.begin)continue;fprintf(f,"{\"type\":\"writer_window\",\"sample\":%u,\"mark\":%u,\"begin\":%llu,\"end\":%llu,\"closed\":%d,\"backbuffer\":\"%llX\",\"context\":\"%llX\",\"begin_hook_mask\":%u,\"end_hook_mask\":%u}\n",i,w.mark,w.begin,w.end,w.closed,(unsigned long long)w.back,(unsigned long long)w.context,w.beginMask,w.endMask);}
 for(unsigned i=0;i<ww.events;i++){auto&e=ww.event[i];fprintf(f,"{\"type\":\"writer_call\",\"id\":%u,\"sample\":%u,\"kind\":\"%s\",\"begin\":%llu,\"end\":%llu,\"completed\":%d,\"possible_only\":%d,\"begin_hook_mask\":%u,\"end_hook_mask\":%u,\"thread\":%lu,\"context\":\"%llX\",\"dst\":\"%llX\",\"src\":\"%llX\",\"api\":\"%llX\",\"caller\":\"%llX\",\"api_id\":%u,\"caller_id\":%u,\"predicate\":\"%llX\",\"predicate_value\":%d,\"args\":[%llu,%llu,%llu,%llu,%llu,%llu],\"box_state\":%u,\"box\":[%u,%u,%u,%u,%u,%u]}\n",i,e.sample,wwNames[e.kind],e.begin,e.end,e.completed,e.possibleOnly,e.beginMask,e.endMask,e.thread,(unsigned long long)e.context,(unsigned long long)e.dst,(unsigned long long)e.src,(unsigned long long)e.api,(unsigned long long)e.caller,e.apiId,e.callerId,(unsigned long long)e.predicate,e.predicateValue,e.args[0],e.args[1],e.args[2],e.args[3],e.args[4],e.args[5],e.boxState,e.box[0],e.box[1],e.box[2],e.box[3],e.box[4],e.box[5]);}
 fprintf(f,"{\"type\":\"writer_summary\",\"events\":%u,\"event_overflow\":%u,\"addresses\":%u,\"address_overflow\":%u,\"foreign_observations\":%u,\"ignored_context\":%u,\"install_failures\":%u,\"vtable_sets\":%u,\"required_hook_mask\":8191}\n",ww.events,ww.eventOverflow,ww.addresses,ww.addressOverflow,ww.foreign,ww.ignoredContext,ww.installFailures,wwSetCount);
}
