#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <new>
#include <atomic>
#include <initializer_list>
#include <cstdio>
#include <cmath>
#include <intrin.h>
#include <bcrypt.h>
#pragma comment(lib,"bcrypt.lib")
#include "dg_draw_trial.h"
#include "dg_draw_trial_core.h"
#include "dg_native_state_probe.h"
#include "dg_native_state_layout.h"
namespace {
using Indexed=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,UINT,UINT,INT);
using Draw=void(STDMETHODCALLTYPE*)(ID3D11DeviceContext*,UINT,UINT);
using QueryCreate=HRESULT(STDMETHODCALLTYPE*)(ID3D11Device*,const D3D11_QUERY_DESC*,ID3D11Query**);
using PredicateCreate=HRESULT(STDMETHODCALLTYPE*)(ID3D11Device*,const D3D11_QUERY_DESC*,ID3D11Predicate**);
using CounterCreate=HRESULT(STDMETHODCALLTYPE*)(ID3D11Device*,const D3D11_COUNTER_DESC*,ID3D11Counter**);
using QueryCreate1=HRESULT(STDMETHODCALLTYPE*)(ID3D11Device3*,const D3D11_QUERY_DESC1*,ID3D11Query1**);
struct Hook{void**slot=nullptr;void*next=nullptr;void*ours=nullptr;};
struct Site{BYTE*address=nullptr;BYTE original[10]{},patch[10]{};void*stub=nullptr;RUNTIME_FUNCTION unwind{};};
Hook hooks[4];unsigned hookCount=3;Site sites[2];
ComPtr<ID3D11Device3>modernDevice;
ComPtr<ID3D11Device>device;ComPtr<ID3D11DeviceContext>context;ComPtr<ID3D11Multithread>multi;
SRWLOCK lifetime=SRWLOCK_INIT;
volatile LONG requested=0,stopping=0,scopeCreated=0;
void*volatile queryOwner=nullptr;
void*volatile modernQueryOwner=nullptr;
bool installed=false,everAttached=false,oldProtected=false;
DWORD ownerThread=0;DrawTrial*pending=nullptr;unsigned polls=0,requestPresents=0;
void(*logger)(const char*,...)=nullptr;
thread_local bool inside=false;
struct LifeLock{LifeLock(){AcquireSRWLockShared(&lifetime);}~LifeLock(){ReleaseSRWLockShared(&lifetime);}};
struct MultiLock{ID3D11Multithread*m;MultiLock():m(multi.Get()){if(m)m->Enter();}~MultiLock(){if(m)m->Leave();}};
void log(const char*s){if(logger)logger("draw_trial: %s\r\n",s);}

// One immutable slot per creation attempt. Writers never log or allocate.
constexpr LONG queryCapacity=256;
struct QueryRecord {
 volatile LONG ready=0,emitted=0;
 LONG id=0,parent=0; unsigned api=0,type=0,flags=0,contextType=0;
 DWORD thread=0; ULONGLONG tick=0; void*caller=nullptr; void*object=nullptr;
 bool descriptor=false,blocking=false; HRESULT result=E_PENDING;
};
QueryRecord queryRecords[queryCapacity];
volatile LONG queryCount=0,queryOverflow=0,queryOverflowReported=0;
thread_local LONG queryParent=0;
struct QueryObservation {
 QueryRecord*r=nullptr; LONG previous=0; bool tracked=false;
 QueryObservation(bool own,unsigned api,bool valid,unsigned type,unsigned flags,
                  unsigned contextType,bool blocking,void*caller) {
  if(!own)return;tracked=true;previous=queryParent;
  LONG id=InterlockedIncrement(&queryCount);queryParent=id;
  if(id>queryCapacity){InterlockedIncrement(&queryOverflow);return;}
  r=&queryRecords[id-1];r->id=id;r->parent=previous;r->api=api;
  r->descriptor=valid;r->type=type;r->flags=flags;r->contextType=contextType;
  r->blocking=blocking;r->caller=caller;r->thread=GetCurrentThreadId();
  r->tick=GetTickCount64();
 }
 void finish(HRESULT result,void*object){if(r){r->result=result;r->object=SUCCEEDED(result)?object:nullptr;InterlockedExchange(&r->ready,1);}}
 ~QueryObservation(){if(tracked)queryParent=previous;}
};
void flushQueries(){
 if(!logger)return;
 for(auto&r:queryRecords){
  if(!InterlockedCompareExchange(&r.ready,0,0)||InterlockedCompareExchange(&r.emitted,1,0))continue;
  HMODULE module=nullptr;char path[MAX_PATH]="unresolved";unsigned long long rva=0;
  if(GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS,(LPCSTR)r.caller,&module)){
   if(!GetModuleFileNameA(module,path,MAX_PATH))strcpy_s(path,"unresolved");
   path[MAX_PATH-1]=0;rva=(unsigned long long)((uintptr_t)r.caller-(uintptr_t)module);
  }
  logger("draw_query: id=%ld parent=%ld api=%u descriptor=%u type=%u flags=0x%X context=%u blocking=%u hr=0x%08X object=%p caller=%p module=%s base=%p rva=0x%llX thread=%lu tick=%llu\r\n",
   r.id,r.parent,r.api,r.descriptor,r.type,r.flags,r.contextType,r.blocking,
   (unsigned)r.result,r.object,r.caller,path,module,rva,r.thread,r.tick);
  if(module)FreeLibrary(module);
 }
 LONG lost=InterlockedCompareExchange(&queryOverflow,0,0);
 if(lost!=InterlockedExchange(&queryOverflowReported,lost))
  logger("draw_query: overflow=%ld capacity=%ld coverage=incomplete\r\n",lost,queryCapacity);
}

bool runtimePointer(void*p){
 for(unsigned depth=0;depth<4;depth++){
  HMODULE module=nullptr;if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCWSTR)p,&module)||module!=GetModuleHandleW(L"d3d11.dll"))return false;
  BYTE b[16]{};SIZE_T got=0;if(!ReadProcessMemory(GetCurrentProcess(),p,b,sizeof b,&got)||got!=sizeof b)return false;
  if(b[0]==0xe9){INT32 rel;memcpy(&rel,b+1,4);p=(char*)p+5+rel;continue;}
  if(b[0]==0xeb){p=(char*)p+2+(signed char)b[1];continue;}
  if(b[0]==0xff&&b[1]==0x25){INT32 rel;memcpy(&rel,b+2,4);void*next=nullptr;if(!ReadProcessMemory(GetCurrentProcess(),(char*)p+6+rel,&next,8,&got)||got!=8)return false;p=next;continue;}
  return true;
 }return false;
}
bool writeSlot(Hook&h,void*expected,void*value){DWORD old,ignored;if(!VirtualProtect(h.slot,8,PAGE_READWRITE,&old))return false;
 bool ok=InterlockedCompareExchangePointer((void*volatile*)h.slot,value,expected)==expected;
 if(!VirtualProtect(h.slot,8,old,&ignored)){if(ok)InterlockedCompareExchangePointer((void*volatile*)h.slot,expected,value);return false;}return ok;
}
bool codeWrite(Site&s,const BYTE*expected,const BYTE*value){DWORD old,ignored;if(memcmp(s.address,expected,10)||!VirtualProtect(s.address,10,PAGE_EXECUTE_READWRITE,&old))return false;
 memcpy(s.address,value,10);FlushInstructionCache(GetCurrentProcess(),s.address,10);
 if(!VirtualProtect(s.address,10,old,&ignored)){memcpy(s.address,expected,10);FlushInstructionCache(GetCurrentProcess(),s.address,10);VirtualProtect(s.address,10,old,&ignored);return false;}return true;
}
bool forwardOwned();
void restoreForward();
bool sitesOwned(){if(!forwardOwned())return false;for(auto&s:sites)if(!s.address||memcmp(s.address,s.patch,10))return false;for(unsigned i=0;i<hookCount;i++)if(!hooks[i].slot||*hooks[i].slot!=hooks[i].ours)return false;return true;}
void restoreAll(){restoreForward();for(auto&s:sites)if(s.address&&!memcmp(s.address,s.patch,10))codeWrite(s,s.patch,s.original);for(auto&h:hooks)if(h.slot&&*h.slot==h.ours)writeSlot(h,h.ours,h.next);}
#ifdef DG_DRAW_TRIAL_TEST
NS_READ testNativeReader=nullptr;
#endif
int readNative(void*,uint64_t a,void*p,unsigned n){
#ifdef DG_DRAW_TRIAL_TEST
 if(testNativeReader)return testNativeReader(nullptr,a,p,n);
#endif
 SIZE_T got=0;return ReadProcessMemory(GetCurrentProcess(),(LPCVOID)(ULONG_PTR)a,p,n,&got)&&got==n;
}
bool forwardApi(void*api);
template<class F>void intercept(ID3D11DeviceContext*c,F next,void*api,const char*kind,bool forwarded=false){
 if(InterlockedCompareExchange(&requested,0,0)!=1){next();return;}
 // Never wait on lifetime while holding the external callback mutex.
 if(!TryAcquireSRWLockShared(&lifetime)){next();return;}
 struct ReleaseLife{~ReleaseLife(){ReleaseSRWLockShared(&lifetime);}} life;
 if(c!=context.Get()||!installed||stopping||inside){next();return;}MultiLock lock;
 if(InterlockedCompareExchange(&requested,2,1)!=1){next();return;}
 const char*reason=nullptr;
 if(!sitesOwned())reason="hook_ownership_changed";
 else if(scopeCreated)reason="query_scope_created_since_device_init";
 else if(!(forwarded?forwardApi(api):runtimePointer(api)))reason="foreign_API_entry";
 else if(!multi->GetMultithreadProtected())reason="context_not_serialized";
 else if(!ownerThread||ownerThread!=GetCurrentThreadId())reason="not_present_thread";
 uint64_t base=(uint64_t)(ULONG_PTR)GetModuleHandleW(nullptr);NS_SAMPLE before{},after{};
 if(!reason&&!ns_layout(base,readNative,nullptr))reason="native_layout";
 DrawTrial*t=nullptr;if(!reason){t=new(std::nothrow)DrawTrial;if(!t)reason="allocation";}
 if(!reason)reason=t->prepare(device.Get(),c);
 if(!reason&&!ns_take(&before,base,readNative,nullptr))reason="native_witness_read";
 if(reason){if(logger)logger("draw_trial: refused=%s extra_draws=0\r\n",reason);delete t;next();return;}
 inside=true;
 bool repeated=t->run(c,next,[&](){return SUCCEEDED(device->GetDeviceRemovedReason())&&sitesOwned()&&!scopeCreated&&ns_take(&after,base,readNative,nullptr)&&!memcmp(&before,&after,sizeof before);});
 inside=false;
 if(!repeated){log("refused=state_changed_after_normal extra_draws=0");delete t;return;}
 bool witness=ns_take(&after,base,readNative,nullptr)&&!memcmp(&before,&after,sizeof before);
 if(logger)logger("draw_trial: submitted kind=%s thread=%lu extra_draws=1 native_witness_equal=%d timing_perturbed=possible width=%u height=%u depth=%d\r\n",kind,GetCurrentThreadId(),witness,t->color.desc.Width,t->color.desc.Height,t->hasDepth);
 if(!witness){log("native_witness_changed; result invalid");delete t;return;}pending=t;polls=0;
}
void ui2dOnDraw(ID3D11DeviceContext*,unsigned);void ui2dOnPresent();bool ui2dSkipDraw(ID3D11DeviceContext*,UINT); // dg_ui2d.inl
bool radarOnDraw(ID3D11DeviceContext*,UINT);void radarOnPresent(); // dg_radar.inl
#include "dg_cb_probe.inl"
#include "dg_mgshd_forward.inl"
void STDMETHODCALLTYPE indexed(ID3D11DeviceContext*c,UINT n,UINT first,INT base){
 ui2dOnDraw(c,0);            // 2D sprite placement per eye; idle unless vr_ui2d=1
 cbOnDraw(c,0,n,first,base); // read-only constant-buffer capture, idle unless armed
 void*api=(*(void***)c)[12];auto f=(Indexed)api;
 LONG route=forwardStatus();
 if(route==FORWARD_WAITING||route==FORWARD_BOUND||route==FORWARD_REJECTED||route==FORWARD_STOPPED){ // No outer locks.
  NativeForwardScope scope;f(c,n,first,base);return;
 }
 intercept(c,[=](){f(c,n,first,base);},api,"DrawIndexed");
}
void STDMETHODCALLTYPE draw(ID3D11DeviceContext*c,UINT n,UINT first){if(ui2dSkipDraw(c,n))return; // previous-frame feedback in stereo: not forwarded
 ui2dOnDraw(c,1);cbOnDraw(c,1,n,first,0);
 if(radarOnDraw(c,n))return; // wrist radar: the composite's texture is captured; with vr_radar_hud=off the draw is not forwarded. No lock held across it.
 void*api=(*(void***)c)[13];auto f=(Draw)api;intercept(c,[=](){f(c,n,first);},api,"Draw");}

// Geometry-only trial: timing queries may include the extra draw duration.
// Unknown descriptors/flags/context variants remain conservative refusals.
bool queryBlocksTrial(bool valid,unsigned type,unsigned flags,unsigned contextType){
 if(!valid||flags||contextType)return true;
 return type!=D3D11_QUERY_EVENT && type!=D3D11_QUERY_TIMESTAMP &&
        type!=D3D11_QUERY_TIMESTAMP_DISJOINT;
}
HRESULT STDMETHODCALLTYPE createQuery(ID3D11Device*d,const D3D11_QUERY_DESC*desc,ID3D11Query**q){
 bool own=d==InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr);
 bool block=queryBlocksTrial(desc!=nullptr,desc?desc->Query:0,desc?desc->MiscFlags:0,0);
 QueryObservation observation(own,0,desc!=nullptr,desc?desc->Query:0,desc?desc->MiscFlags:0,0,block,_ReturnAddress());
 if(own&&block)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((QueryCreate)hooks[0].next)(d,desc,q);
 observation.finish(hr,SUCCEEDED(hr)&&q?*q:nullptr);return hr;
}
HRESULT STDMETHODCALLTYPE createPredicate(ID3D11Device*d,const D3D11_QUERY_DESC*desc,ID3D11Predicate**q){
 bool own=d==InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr);
 QueryObservation observation(own,1,desc!=nullptr,desc?desc->Query:0,desc?desc->MiscFlags:0,0,true,_ReturnAddress());
 if(own)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((PredicateCreate)hooks[1].next)(d,desc,q);
 observation.finish(hr,SUCCEEDED(hr)&&q?*q:nullptr);return hr;
}
HRESULT STDMETHODCALLTYPE createCounter(ID3D11Device*d,const D3D11_COUNTER_DESC*desc,ID3D11Counter**q){
 bool own=d==InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr);
 QueryObservation observation(own,2,desc!=nullptr,desc?desc->Counter:0,desc?desc->MiscFlags:0,0,true,_ReturnAddress());
 if(own)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((CounterCreate)hooks[2].next)(d,desc,q);
 observation.finish(hr,SUCCEEDED(hr)&&q?*q:nullptr);return hr;
}
HRESULT STDMETHODCALLTYPE createQuery1(ID3D11Device3*d,const D3D11_QUERY_DESC1*desc,ID3D11Query1**q){
 bool own=d==InterlockedCompareExchangePointer(&modernQueryOwner,nullptr,nullptr);
 bool block=queryBlocksTrial(desc!=nullptr,desc?desc->Query:0,desc?desc->MiscFlags:0,desc?desc->ContextType:0);
 QueryObservation observation(own,3,desc!=nullptr,desc?desc->Query:0,desc?desc->MiscFlags:0,desc?desc->ContextType:0,block,_ReturnAddress());
 if(own&&block)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((QueryCreate1)hooks[3].next)(d,desc,q);
 observation.finish(hr,SUCCEEDED(hr)&&q?*q:nullptr);return hr;
}
void*nearMemory(BYTE*site){SYSTEM_INFO info;GetSystemInfo(&info);uintptr_t center=(uintptr_t)site&~(uintptr_t)(info.dwAllocationGranularity-1);
 for(uintptr_t step=info.dwAllocationGranularity;step<0x70000000;step+=info.dwAllocationGranularity){for(int sign:{-1,1}){uintptr_t a=sign<0?center-step:center+step;if(sign<0&&center<step)continue;void*p=VirtualAlloc((void*)a,4096,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);if(p)return p;}}return nullptr;}
#include "dg_ui2d.inl"
#include "dg_radar.inl"
bool prepareSites(uint64_t base){
 const unsigned rvas[]={0x53bf4,0x53b75};const BYTE expected[2][10]={{0xff,0x50,0x60,0x48,0x8b,0x05,0x82,0x16,0x38,0x01},{0xff,0x50,0x68,0x48,0x8b,0x05,0x01,0x17,0x38,0x01}};
 for(unsigned i=0;i<2;i++){auto&s=sites[i];BYTE*address=(BYTE*)(ULONG_PTR)(base+rvas[i]);BYTE bytes[10];SIZE_T got=0;
  if(!ReadProcessMemory(GetCurrentProcess(),address,bytes,10,&got)||got!=10||memcmp(bytes,expected[i],10))return false;s.address=address;memcpy(s.original,bytes,10);}
 for(unsigned i=0;i<2;i++){auto&s=sites[i];s.stub=nearMemory(s.address);if(!s.stub)return false;
  // Called by the original caller with its already prepared register args.
  // Reserve shadow/alignment space, call C, reproduce the overwritten RIP load.
  BYTE code[]={0x48,0x83,0xec,0x28,0x48,0xb8,0,0,0,0,0,0,0,0,0xff,0xd0,0x48,0xb8,0,0,0,0,0,0,0,0,0x48,0x8b,0x00,0x48,0x83,0xc4,0x28,0xc3};
  uint64_t callback=(uint64_t)(ULONG_PTR)(i?(void*)draw:(void*)indexed),global=base+0x13d5280;memcpy(code+6,&callback,8);memcpy(code+18,&global,8);memcpy(s.stub,code,sizeof code);
  const BYTE unwindInfo[]={1,4,1,0,4,0x42,0,0};memcpy((BYTE*)s.stub+0x100,unwindInfo,sizeof unwindInfo);
  s.unwind.BeginAddress=0;s.unwind.EndAddress=sizeof code;s.unwind.UnwindData=0x100;
  if(!RtlAddFunctionTable(&s.unwind,1,(DWORD64)s.stub))return false;
  DWORD old;if(!VirtualProtect(s.stub,4096,PAGE_EXECUTE_READ,&old))return false;FlushInstructionCache(GetCurrentProcess(),s.stub,sizeof code);
  intptr_t rel=(BYTE*)s.stub-(s.address+5);if(rel<INT32_MIN||rel>INT32_MAX)return false;memset(s.patch,0x90,10);s.patch[0]=0xe8;INT32 displacement=(INT32)rel;memcpy(s.patch+1,&displacement,4);
 }return true;
}
}
extern "C" void dg_draw_trial_attach(ID3D11Device*d,void(*out)(const char*,...)){
 AcquireSRWLockExclusive(&lifetime);if(everAttached){ReleaseSRWLockExclusive(&lifetime);return;}everAttached=true;logger=out;device=d;d->GetImmediateContext(&context);
 if(!context||FAILED(context.As(&multi))){log("attach refused: serialization unavailable");ReleaseSRWLockExclusive(&lifetime);return;}
 oldProtected=multi->GetMultithreadProtected()!=FALSE;auto table=*(void***)d;void*ours[]={(void*)createQuery,(void*)createPredicate,(void*)createCounter};bool ok=true;
 for(unsigned i=0;i<3;i++){hooks[i]={table+24+i,table[24+i],ours[i]};if(!runtimePointer(hooks[i].next))ok=false;}
 if(SUCCEEDED(d->QueryInterface(IID_PPV_ARGS(&modernDevice)))){auto mt=*(void***)modernDevice.Get();hooks[3]={mt+60,mt[60],(void*)createQuery1};hookCount=4;if(!runtimePointer(hooks[3].next))ok=false;}
 InterlockedExchangePointer(&queryOwner,d);InterlockedExchangePointer(&modernQueryOwner,modernDevice.Get());
#ifdef DG_DRAW_TRIAL_TEST
 extern uint64_t testSiteBase;
 uint64_t base=testSiteBase;
#else
 uint64_t base=(uint64_t)(ULONG_PTR)GetModuleHandleW(nullptr);
#endif
 if(ok)ok=prepareSites(base);
 if(ok){HMODULE pin=nullptr;ok=GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)&indexed,&pin)!=FALSE;}
 if(ok){multi->SetMultithreadProtected(TRUE);for(unsigned i=0;i<hookCount;i++){auto&h=hooks[i];if(!writeSlot(h,h.next,h.ours)){ok=false;break;}}}
 if(ok)for(auto&s:sites)if(!codeWrite(s,s.original,s.patch)){ok=false;break;}
 if(!ok){restoreAll();multi->SetMultithreadProtected(oldProtected);log("attach refused: code/API ownership; no draw trial");}
 else{installed=true;ui2dInstall(d);beginForward();log("attached: native API callsites only; query policy=timing_allowed_geometry_only; indexed=deferred_validated_forward; default off");}
 ReleaseSRWLockExclusive(&lifetime);
}
extern "C" void dg_draw_trial_poll(const char*marker){flushQueries();pollForward();
 if(marker){ // The constant-buffer capture has its own request word and is independent of the one-shot draw trial.
  char cp[MAX_PATH],cbuf[64];size_t cn=0;
  if(!strcpy_s(cp,marker)){auto cl=strrchr(cp,'\\');bool named=cl?!strcpy_s(cl+1,sizeof(cp)-(cl+1-cp),"dg_draw_trial.on"):!strcpy_s(cp,"dg_draw_trial.on");
   FILE*cf=nullptr;if(named&&!fopen_s(&cf,cp,"rb")&&cf){cn=fread(cbuf,1,sizeof cbuf,cf);fclose(cf);}
   cbPoll(marker,cbuf,cn);}
 }
 if(!marker||InterlockedCompareExchange(&requested,0,0))return;
 char path[MAX_PATH],buf[32];if(strcpy_s(path,marker))return;auto leaf=strrchr(path,'\\');if(leaf){if(strcpy_s(leaf+1,sizeof(path)-(leaf+1-path),"dg_draw_trial.on"))return;}else strcpy_s(path,"dg_draw_trial.on");
 FILE*f=nullptr;if(fopen_s(&f,path,"rb")||!f)return;size_t n=fread(buf,1,sizeof buf,f);fclose(f);
 if(n==14&&!memcmp(buf,"identical_draw",14)){LifeLock life;if(!installed){InterlockedExchange(&requested,2);log("refused=no_api_owner extra_draws=0");}else if(forwardStatus()==FORWARD_WAITING){log("request deferred: forward binding pending");}
 else if(forwardStatus()==FORWARD_REJECTED){InterlockedExchange(&requested,2);log("refused=forward_binding_unavailable extra_draws=0");}
 else InterlockedCompareExchange(&requested,1,0);}
}
extern "C" void dg_draw_trial_present(){LifeLock life;if(!installed||stopping)return;MultiLock lock;if(!ownerThread)ownerThread=GetCurrentThreadId();
 cbOnPresent();ui2dOnPresent();radarOnPresent();
 if(InterlockedCompareExchange(&requested,0,0)==1&&++requestPresents>120){InterlockedExchange(&requested,2);log("refused=no_direct_draw extra_draws=0");}
 if(!pending)return;if(ownerThread!=GetCurrentThreadId()||++polls>120||FAILED(device->GetDeviceRemovedReason())){log("readback timeout/thread-change/device-loss; no retry");delete pending;pending=nullptr;return;}
 unsigned long long cm=0,cc=0,dm=0,dc=0;int a=pending->color.compare(context.Get(),cm,cc),b=pending->hasDepth?pending->depth.compare(context.Get(),dm,dc):1;if(a==0||b==0)return;
 if(logger)logger("draw_trial: readback valid=%d color_diff_bytes=%llu depth_diff_bytes=%llu normal_changed_color=%llu normal_changed_depth=%llu nontrivial=%d\r\n",a==1&&b==1,cm,dm,cc,dc,cc+dc>0);delete pending;pending=nullptr;
}
extern "C" void dg_draw_trial_stop(){InterlockedExchange(&stopping,1);AcquireSRWLockExclusive(&lifetime);bool owned=sitesOwned();
 // Code sites remain forwarding-only until process exit. Unlike pointer slots,
 // ten instruction bytes cannot safely be restored amid concurrent execution.
 restoreForward();setForwardStatus(FORWARD_STOPPED);ui2dRemove();
 for(auto&h:hooks)if(h.slot&&*h.slot==h.ours)writeSlot(h,h.ours,h.next);
 installed=false;delete pending;pending=nullptr;
 InterlockedExchangePointer(&queryOwner,nullptr);InterlockedExchangePointer(&modernQueryOwner,nullptr);
 // Keep serialization enabled until the device dies. Turning it off rewrites
 // runtime dispatch while a normal, forwarding-only draw may still be active.
 multi.Reset();context.Reset();modernDevice.Reset();device.Reset();
 // Retain tiny executable stubs and pin this module: a caller may have fetched
 // a stub before restoration. Its C callback now forwards without a trial.
 ReleaseSRWLockExclusive(&lifetime);
}
// Camera-seam publisher for the constant-buffer capture. Called from the VEH
// camera hook: interlocked counters and four 64-byte copies, nothing else.
extern "C" void dg_cb_probe_camera(int eye,const float*eye_pers,const float*pers,const float*eye_inv,const float*eye_world){
 LONG n=InterlockedIncrement(&cbCamNext);CbCam&c=cbCams[(unsigned)(n-1)%cbCamCount];
 InterlockedIncrement(&c.seq); // odd: writer active
 c.eye=eye;c.tick=GetTickCount64();
 memcpy(c.m[0],eye_pers,64);memcpy(c.m[1],pers,64);memcpy(c.m[2],eye_inv,64);memcpy(c.m[3],eye_world,64);
 InterlockedIncrement(&c.seq);
}
// 2D sprite placement: configuration, back-buffer size and the heartbeat line.
extern "C" void dg_ui2d_configure(int on,int scale_mils,int conv_e5,int sign,const unsigned long long*vs,unsigned count){
 AcquireSRWLockExclusive(&ui2dCfgLock);
 ui2dCfg.scaleMils=scale_mils<300?300:scale_mils>1000?1000:scale_mils;ui2dCfg.convE5=conv_e5<0?0:conv_e5>10000?10000:conv_e5;
 ui2dCfg.sign=sign<0?-1:1;ui2dCfg.vsCount=0;for(unsigned i=0;i<count&&i<8;i++)if(vs[i])ui2dCfg.vs[ui2dCfg.vsCount++]=vs[i];
 ReleaseSRWLockExclusive(&ui2dCfgLock);InterlockedExchange(&ui2dCfg.on,on?1:0);
}
extern "C" void dg_ui2d_backbuffer(unsigned w,unsigned h){ui2dBbW=w;ui2dBbH=h;}
// Present thread only, right after dg_draw_trial_present(): the frame that just ended.
extern "C" long dg_ui2d_last_frame_detail(char*out,size_t n){
 const Ui2dFrameDetail&d=ui2dLast;
 sprintf_s(out,n,"draws %ld uploads %ld votes +%ld/-%ld sprites %ld  no-row %ld degenerate %ld SYMMETRIC %ld (xw %+.4f) far %ld (xw %+.4f) scale %ld skew %ld",
  d.draws,d.uploads,d.pos,d.neg,d.sprites,d.why[1],d.why[2],d.why[3],d.symXw,d.why[4],d.farXw,d.why[5],d.why[6]);
 return d.draws;
}
// Camera seam (VEH): the eye of the handoff it just published. Interlocked only.
extern "C" void dg_ui2d_eye(int eye){InterlockedExchange(&ui2dEyeTick,(LONG)GetTickCount());InterlockedExchange(&ui2dEyeNow,eye==1?2:eye==0?1:0);}
// Present thread only: the eye the finished frame's OBJECT shaders were drawn with (c20 votes).
extern "C" int dg_ui2d_last_frame_obj(long*pos,long*neg){if(pos)*pos=ui2dLastObjPos;if(neg)*neg=ui2dLastObjNeg;return (int)ui2dLastObjSign;}
// Present thread only: native Draw/DrawIndexed calls the finished frame issued.
extern "C" long dg_ui2d_last_frame_draws(void){return ui2dLast.draws;}
// Present thread: the back buffer's identity, and what the finished frame drew into it

extern "C" void dg_ui2d_backbuffer_ptr(void*p){ui2dBackbufferPtr=p;}
extern "C" long dg_ui2d_last_frame_bb(long*indexed,void**srv){if(indexed)*indexed=ui2dLastBbIndexed;if(srv)*srv=ui2dLastBbSrv;return ui2dLastBbDraws;}
// Arms the final-buffer trace for the next `frames` frames; a and b are the two final textures (identity only).
extern "C" void dg_ui2d_trace(int frames,void*a,void*b){ui2dEnsureCopyHooks();ui2dTraceTarget[0]=a;ui2dTraceTarget[1]=b;ui2dTraceCount=0;ui2dTraceOverflow=0;ui2dTraceFrameNo=0;InterlockedExchange(&ui2dTraceLeft,frames<0?0:frames>16?16:frames);}
// Present thread: 1 while alternate-eye stereo gameplay is being captured and vr_feedback_skip is on.
extern "C" void dg_ui2d_feedback(int on){InterlockedExchange(&ui2dFeedbackOn,on?1:0);}
extern "C" void dg_ui2d_feedback_stats(long*skipped,long*checked){if(skipped)*skipped=ui2dFeedbackSkipped;if(checked)*checked=ui2dFeedbackChecked;}
extern "C" void dg_ui2d_hold(int mode){InterlockedExchange(&ui2dCfg.hold,mode<0?0:mode>2?2:mode);}
extern "C" int dg_ui2d_last_frame_sign(void){return (int)InterlockedCompareExchange(&ui2dLastFrameSign,0,0);}
// Camera seam (VEH): |NDC x of straight-ahead| of the frustum submitted to the headset.
extern "C" void dg_ui2d_display(double left,double right){
 double l=tan(left),r=tan(right),d=r-l;if(!(d>1e-6))return;double m=fabs((r+l)/d);
 if(m>=0.02&&m<=0.6)InterlockedExchange(&ui2dDisplayMagE6,(LONG)(m*1000000.0+0.5));
}
extern "C" void dg_ui2d_stats(char*out,size_t n){
 char cand[400]="";size_t at=0;
 for(auto&c:ui2dCand)if(c.hash&&at+40<sizeof cand)at+=sprintf_s(cand+at,sizeof cand-at," %016llX:%ld",(unsigned long long)c.hash,c.count);
 sprintf_s(out,n,"ui2d: on %ld site %s vs-known %u  uploads %ld persp %ld x0 %+.4f display %.4f votes +%ld/-%ld  sig-draws %ld patched %ld restored %ld errors %ld  skip kind %ld stale %ld viewport %ld depth %ld vs %ld  bb %ux%u  scale %.2f conv %.4f sign %+ld list %u  hold %ld frames voted %ld zero %ld few %ld  sprite-frames held %ld BARE %ld early %ld  held-draws %ld  eye-src L +%ld/-%ld R +%ld/-%ld agree %ld DISAGREE %ld used %ld  draws/frame voted %.0f no-camera %.0f (min %ld max %ld)  candidates%s",
  ui2dCfg.on,ui2dSiteInstalled?"ok":"MISSING",vsKnown,ui2dUploads,ui2dPersp,ui2dHaveX0?ui2dX0:0.0f,ui2dDisplayMagE6/1000000.0,ui2dVotesPos,ui2dVotesNeg,ui2dSigDraws,ui2dPatched,ui2dRestored,ui2dErrors,
  ui2dSkipKind,ui2dSkipStale,ui2dSkipViewport,ui2dSkipDepth,ui2dSkipVs,ui2dBbW,ui2dBbH,ui2dCfg.scaleMils/1000.0,ui2dCfg.convE5/100000.0,ui2dCfg.sign,ui2dCfg.vsCount,ui2dCfg.hold,ui2dFramesVoted,ui2dFramesZero,ui2dFramesFew,ui2dFramesHeld,ui2dFramesBare,ui2dFramesEarly,ui2dHeldDraws,ui2dEyeN[0][0],ui2dEyeN[0][1],ui2dEyeN[1][0],ui2dEyeN[1][1],ui2dEyeAgree,ui2dEyeDisagree,ui2dEyeUsed,ui2dFramesVoted?(double)ui2dDrawsVoted/ui2dFramesVoted:0.0,(ui2dFramesZero+ui2dFramesFew)?(double)ui2dDrawsNoCam/(ui2dFramesZero+ui2dFramesFew):0.0,ui2dNoCamMin==0x7fffffff?0L:(long)ui2dNoCamMin,(long)ui2dNoCamMax,cand);
}
// Wrist radar, draw-hook half: the switches and the heartbeat's counters.
extern "C" void dg_radar_configure(int on,int hud_off){InterlockedExchange(&radarHudOff,hud_off?1:0);InterlockedExchange(&radarOn,on?1:0);}
extern "C" void dg_radar_view(int first_person){InterlockedExchange(&radarFirstPerson,first_person?1:0);}
extern "C" void dg_radar_stats(char*out,size_t n){
 _snprintf_s(out,n,_TRUNCATE,"composite draws %ld (extra in frame %ld, frames with more than one %ld)  NOT FORWARDED %ld  wrist-up %ld refused %ld  last viewport %ux%u at (%u,%u) texture %ux%u fmt %u  rejected: origin %ld bigger %ld depth-on %ld no-tex2d %ld size %ld format %ld not-plain %ld no-viewport %ld",
  radarHits,radarExtra,radarFramesMulti,radarSkipped,radarAvailable,radarRefused,radarLastVp[2],radarLastVp[3],radarLastVp[0],radarLastVp[1],radarLastTex[0],radarLastTex[1],radarLastTex[2],
  radarWhy[3],radarWhy[4],radarWhy[5],radarWhy[6],radarWhy[7],radarWhy[8],radarWhy[9],radarWhy[2]);
}
