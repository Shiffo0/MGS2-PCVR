#include "dg_build_profile.h"
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <new>
#include <vector>
#include <atomic>
#include <initializer_list>
#include <cstdio>
#include <cmath>
#include <intrin.h>
#include <bcrypt.h>
#pragma comment(lib,"bcrypt.lib")
#include "dg_draw_trial.h"
#include "dg_draw_trial_core.h"
#include "dg_context_trace.h"
#include "dg_native_state_probe.h"
#include "dg_native_state_layout.h"
#include "dg_draw_observation.h"
#include "dg_magwell_gpu.h"
#include "dg_replay_dropout_trace.inl"
#include "dg_replay_sample_banks.inl"
#include "dg_replay_execution_probe.h"
#include "dg_drop_probe.h"
namespace {
DgMagwellGpu magwellGpu;
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
bool secondCameraRequest=false;
UINT64 vsHashOf(void*,const BYTE**,UINT32*);
DWORD ownerThread=0;DrawTrial*pending=nullptr;unsigned polls=0,requestPresents=0;
void(*logger)(const char*,...)=nullptr;
thread_local bool inside=false;
// Private bounded contour queries never enclose native/replayed draws. Use the
// captured original factory, not the public hook that tracks native scopes.
HRESULT STDMETHODCALLTYPE magwellQuery(ID3D11Device*d,const D3D11_QUERY_DESC*q,ID3D11Query**out) {
 if(!inside||d!=InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr)||!hooks[0].next||!q||q->Query!=D3D11_QUERY_OCCLUSION||q->MiscFlags)return E_FAIL;
 return ((QueryCreate)hooks[0].next)(d,q,out);
}

struct LifeLock{LifeLock(){AcquireSRWLockShared(&lifetime);}~LifeLock(){ReleaseSRWLockShared(&lifetime);}};
DgReplayExecutionProbe replayExecution;
#include "dg_replay_near.h"
extern volatile LONG seNearMm;        /* vr_se_near (defined with seMode below) */
extern volatile LONG seMode;

static void dgReplaySameCamera(const float Sg[16],float Tg[16]){if(InterlockedCompareExchange(&seMode,0,0)&128)memcpy(Tg,Sg,16*sizeof(float));}
DgDropProbe dropProbe;                 /* per-draw native/replay comparison (vr_se_mode bit 16; forces the immediate world route) */
HRESULT STDMETHODCALLTYPE replayExecutionQuery(ID3D11Device*d,const D3D11_QUERY_DESC*q,ID3D11Query**out){
 if(!inside||scopeCreated||d!=InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr)||!hooks[0].next||!q||q->MiscFlags||
    (q->Query!=D3D11_QUERY_OCCLUSION&&q->Query!=D3D11_QUERY_PIPELINE_STATISTICS&&
     q->Query!=D3D11_QUERY_SO_STATISTICS))return E_FAIL;
 return ((QueryCreate)hooks[0].next)(d,q,out);
}
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
bool forwardMonitor();
void restoreForward();
void forwardReport();
bool sitesOwned(){if(!forwardOwned())return false;for(auto&s:sites)if(!s.address||memcmp(s.address,s.patch,10))return false;for(unsigned i=0;i<hookCount;i++)if(!hooks[i].slot||*hooks[i].slot!=hooks[i].ours)return false;return true;}
void restoreAll(){restoreForward();for(auto&s:sites)if(s.address&&!memcmp(s.address,s.patch,10))codeWrite(s,s.patch,s.original);for(auto&h:hooks)if(h.slot&&*h.slot==h.ours)writeSlot(h,h.ours,h.next);}
int readNative(void*,uint64_t a,void*p,unsigned n){
 SIZE_T got=0;return ReadProcessMemory(GetCurrentProcess(),(LPCVOID)(ULONG_PTR)a,p,n,&got)&&got==n;
}
bool forwardApi(void*api);
bool mgDrawApi(void*api);
bool initializedApi(ID3D11DeviceContext*c,void*api);

volatile LONG seDirect=1;
unsigned directFrame=0,directSeen=~0u,directN=0;void*directPtr[4]{};bool directOk[4]{};
bool directCode(void*api){   /* the code part: per frame, so a hook placed later is seen within one frame */
 if(directSeen!=directFrame){directSeen=directFrame;directN=0;}
 for(unsigned i=0;i<directN;i++)if(directPtr[i]==api)return directOk[i];
 bool ok=api&&runtimePointer(api);
 if(directN<4){directPtr[directN]=api;directOk[directN++]=ok;}
 return ok;
}

unsigned directSlotSeen=~0u,directSlotN=0;void*directSlot[4]{};
void directSlotNote(void*api){
 if(directSlotSeen!=directFrame){directSlotSeen=directFrame;directSlotN=0;}
 for(unsigned i=0;i<directSlotN;i++)if(directSlot[i]==api)return;
 if(directSlotN<4)directSlot[directSlotN++]=api;
}
bool directSlotKnown(ID3D11DeviceContext*c,void*api){
 if((*(void***)c)[12]==api)return true;
 if(directSlotSeen!=directFrame)return false;
 for(unsigned i=0;i<directSlotN;i++)if(directSlot[i]==api)return true;
 return false;
}

enum { S12_DETOURED, S12_DIRECT, S12_REFUSED, S12_KINDS };
struct S12Stat{unsigned long long calls,routed,replayed,liveNoRoute;};S12Stat s12[S12_KINDS]{};
void*s12Last[S12_KINDS]{};unsigned s12Lines=0,s12Epoch=~0u,s12Pos=0;
thread_local unsigned long long interceptSerial=0;   /* +1 at every intercept entry */
static unsigned long long d3dRva(void*p){static HMODULE m=GetModuleHandleW(L"d3d11.dll");return p&&m&&(BYTE*)p>=(BYTE*)m?(unsigned long long)((BYTE*)p-(BYTE*)m):0ull;}
#include "dg_late_draw.h"
#include "dg_trial_ready.h"

constexpr bool nativeClearLightSession=1!=0;
TrialReady lateReady;SRWLOCK lateReadyLock=SRWLOCK_INIT;
void* volatile lateXrContext=nullptr;
std::atomic<bool> latePlainDraw{false};
std::atomic<bool> latePlainImmediate{false};

#include "dg_geometry_probe.h"
bool geometryRequest=false;
constexpr unsigned geoMaxAttempts=8;constexpr DWORD geoWindowMs=10000;
struct GeoKey{void*api;UINT n,first;INT base;};
struct GeoRun{
 GeometryProbe probe;GeoTotals totals;UINT raw[GEO_WORDS]{};
 unsigned attempts=0;DWORD started=0;bool dispatched=false,read=false,finished=false;
 double sinv[16]{},m[16]{},minv[16]{},camera[3]{};float source[16]{},target[16]{};UINT width=0,height=0;GeoKey keys[geoMaxAttempts]{};
} geo;
bool geoSeen(void*api,UINT n,UINT first,INT base){for(unsigned i=0;i<geo.attempts&&i<geoMaxAttempts;i++){auto&k=geo.keys[i];if(k.api==api&&k.n==n&&k.first==first&&k.base==base)return true;}return false;}
/* Only a viewport covering the whole bound target maps NDC to pixels as the probe assumes. */
bool geoFullViewport(ID3D11DeviceContext*c){
 D3D11_VIEWPORT vp[2]{};UINT n=2;c->RSGetViewports(&n,vp);if(n!=1)return false;
 ComPtr<ID3D11RenderTargetView>rt;c->OMGetRenderTargets(1,&rt,nullptr);if(!rt)return false;
 ComPtr<ID3D11Resource>r;rt->GetResource(&r);ComPtr<ID3D11Texture2D>t;if(FAILED(r.As(&t)))return false;
 D3D11_TEXTURE2D_DESC d{};t->GetDesc(&d);
 return vp[0].TopLeftX==0&&vp[0].TopLeftY==0&&vp[0].Width==(float)d.Width&&vp[0].Height==(float)d.Height&&vp[0].MinDepth==0&&vp[0].MaxDepth==1;
}
void geoFinish(const char*reason){
 if(!geometryRequest||geo.finished)return;geo.finished=true;
 GeoVerdict v=geoJudge(geo.totals);static const char*names[]={"near","mid","far"};
 for(int b=0;b<GEO_BINS;b++){auto&g=geo.totals.bin[b];auto&r=v.bin[b];
  if(logger)logger("geometry: bin=%s pixels=%llu distance=%.0f..%.0f inliers=%llu share=%.2f unwritten=%llu nonplanar=%llu residual_beyond_window=%llu flat=%llu outside=%llu info_x/y=%.2f/%.2f shift_px=(%+.3f,%+.3f) measured_x/y=%d/%d predicted_dx/dy=%+.2f/%+.2f tested=%d pass=%d\r\n",
   names[b],g.pixels,g.pixels?g.minZ:0.f,g.maxZ,g.inliers,r.inlierShare,g.rejected,g.nonplanar,g.residual,g.flat,g.outside,r.infoX,r.infoY,r.sx,r.sy,r.measuredX,r.measuredY,r.meanDx,r.meanDy,r.tested,r.pass);}
 if(logger)logger("geometry: verdict=%s attempts=%u stop=%s complete=%d vertical=%d horizontal=%d limits: |shift|<=%.1fpx inlier_share>=%.1f min_inliers=%u\r\n",
  !v.complete?"INCOMPLETE":v.pass?"PASS":"FAIL",geo.attempts,reason,v.complete,v.vertical,v.horizontal,geoMaxError,geoMinInlierShare,geoMinInliers);
}
bool plainImmediateWord(const char*p,size_t n){return n==sizeof("late_plain_immediate")-1&&!memcmp(p,"late_plain_immediate",sizeof("late_plain_immediate")-1);}
bool plainSurveyWord(const char*p,size_t n){return n==sizeof("late_plain_draw")-1&&!memcmp(p,"late_plain_draw",sizeof("late_plain_draw")-1);}
constexpr bool surveyEnabled=true;
constexpr unsigned minTrialPixels=256;
bool sufficientTrialPixels(unsigned color,unsigned depth){return color>=minTrialPixels||depth>=minTrialPixels;}
struct SmallDrawKey{void*api;UINT count,first;INT base;};
SmallDrawKey smallDraws[64]{};unsigned smallDrawCount=0;
bool smallDrawSeen(void*api,UINT count,UINT first,INT base){for(unsigned i=0;i<smallDrawCount;i++){auto &k=smallDraws[i];if(k.api==api&&k.count==count&&k.first==first&&k.base==base)return true;}return false;}
unsigned lateSkipped=0,lateCandidates=0;DWORD lateSearchStarted=0;
DWORD lateMatchStarted=0;unsigned lateReselections=0;
bool lateSurvey=false,lateMatch=false;UINT lateCount=0,lateFirst=0;INT lateBase=0;void*lateApi=nullptr;
#include "dg_trial_target_match.h"
#include "dg_trial_vertex_diagnosis.h"
TrialVertexBindings lateVertexBindings;
TrialPipelineState* latePipeline=nullptr;
ComPtr<ID3D11RenderTargetView> lateRT;ComPtr<ID3D11DepthStencilView> lateDS;
unsigned targetMismatchCount=0;
unsigned matchChecks=0,matchSamples=0,matchKeyCounts[4]{},matchSectionCounts[64]{};
void reportLateMatch(){
 if(!logger)return;
 logger("draw_trial: match_summary checked=%u api=%u count=%u first_index=%u base_vertex=%u\r\n",matchChecks,matchKeyCounts[0],matchKeyCounts[1],matchKeyCounts[2],matchKeyCounts[3]);
 if(latePipeline)for(size_t i=0;i<latePipeline->sections.size();i++)if(matchSectionCounts[i])logger("draw_trial: match_difference section=%s count=%u\r\n",latePipeline->sections[i].name,matchSectionCounts[i]);
}
void observeLateMatch(void*api,UINT count,UINT first,INT base,const TrialPipelineState& current,ID3D11DeviceContext*c=nullptr){
 ++matchChecks;bool keys[]={api!=lateApi,count!=lateCount,first!=lateFirst,base!=lateBase};
 for(unsigned i=0;i<4;i++)matchKeyCounts[i]+=keys[i];
 auto mask=latePipeline->differences(current);
 for(size_t i=0;i<latePipeline->sections.size();i++)if(mask&(1ull<<i))++matchSectionCounts[i];
 if(logger&&matchSamples<8&&(mask||keys[0]||keys[1]||keys[2]||keys[3])){
  ++matchSamples;logger("draw_trial: match_sample=%u api=%p/%p count=%u/%u first=%u/%u base=%d/%d pipeline_mask=0x%llX\r\n",matchSamples,lateApi,api,lateCount,count,lateFirst,first,lateBase,base,mask);
  for(size_t i=0;i<latePipeline->sections.size();i++)if(mask&(1ull<<i))logger("draw_trial: sample_difference section=%s\r\n",latePipeline->sections[i].name);
  if(c){TrialVertexBindings bindings;bindings.capture(c);lateVertexBindings.report(bindings,logger);}
 }
}
void clearLateMatch(){delete latePipeline;latePipeline=nullptr;lateRT.Reset();lateDS.Reset();lateVertexBindings.reset();lateMatch=false;}

struct LateReason {const char* name=nullptr;unsigned count=0;};
LateReason lateReasons[16];
volatile LONG lateNativeIndexed=0,lateForwardIndexed=0,lateForwardScopeMiss=0,lateForwardReturnMiss=0;
void lateSkip(const char* reason,UINT n,bool indexed,ID3D11DeviceContext*c){
 for(auto &v:lateReasons){if(!v.name){v.name=reason;if(!strcmp(reason,"rtv_subresource")||!strcmp(reason,"dsv_subresource"))lateDescribeViews(c,logger);if(logger)logger("draw_trial: late first_skip reason=%s count=%u indexed=%d\r\n",reason,n,indexed);}if(!strcmp(v.name,reason)){++v.count;return;}}
}
void lateRouteReport();
void lateSkipReport(){lateRouteReport();if(logger)logger("draw_trial: late routes native_indexed=%ld forwarded_indexed=%ld scope_miss=%ld return_miss=%ld\r\n",lateNativeIndexed,lateForwardIndexed,lateForwardScopeMiss,lateForwardReturnMiss);for(auto &v:lateReasons)if(v.name&&logger)logger("draw_trial: late skip_total reason=%s count=%u\r\n",v.name,v.count);}
 const char* lateLastSkip="await_xr_protection";
bool ui2dSceneAllowed();
bool ammoSceneAllowed();

#include "dg_frame_capture.h"
#include "dg_vs_rdef.h"
bool frameRequest=false;

struct FrameShader{UINT64 hash;UINT pers;const char*name;};
const FrameShader frameShaders[]={
 {UINT64_C(0x544F1B615F77E0FD),20,"544F"},{UINT64_C(0x0674FB1F50787575),16,"0674"},{UINT64_C(0x265C303FB5F8AC08),20,"265C"},
 {UINT64_C(0xC2290FAE0D108C11),20,"C229"},{UINT64_C(0x042AADC96CE2E7E0),20,"042A"},{UINT64_C(0xC8863A7C6D3B6385),20,"C886"},
 {UINT64_C(0xF26DFC8A346BF156),20,"F26D"},{UINT64_C(0x17358D26F1EBC890),16,"1735"},{UINT64_C(0xB8C37DBB337B0208),16,"B8C3"},
 {UINT64_C(0x0A541F153C5036D2),20,"0A54"},{UINT64_C(0x40E35BC10B4C6464),20,"40E3"},{UINT64_C(0xB17D8B11DD324A79),20,"B17D"},
 {UINT64_C(0x880FE8A3C0914E41),24,"880F"},{UINT64_C(0x7000B69E85B87805),20,"7000"},{UINT64_C(0x57536C2AF5B84024),20,"5753"}};
const UINT64 frameScreenShaders[]={UINT64_C(0x4E028ADD1EA96D0B),UINT64_C(0x25506B36206636CA),UINT64_C(0xA460673B02B58339),
 UINT64_C(0x0B14AD7A410A4DD4),UINT64_C(0xC44D207FC8A9A0B8),UINT64_C(0x16CECDC15E6B5EE7),UINT64_C(0x1B3FA09A7AEF1AB0),
 UINT64_C(0x465E5F2FE0EFD968),UINT64_C(0x611C8F0F6ADF21F1),UINT64_C(0x6FDF1958BADEA3C9),UINT64_C(0xB1386EC0590A47E9),UINT64_C(0xB356EBFAD8F1B18E)};
static_assert(sizeof frameShaders/sizeof frameShaders[0]<SecondCameraGpu::flagSlots,"shader slots (last one: RDEF-resolved)");
constexpr UINT frameRdefSlot=SecondCameraGpu::flagSlots-1;
constexpr unsigned frameMaxDraws=4096;

constexpr unsigned frameMinReplays=64,frameMaxIntervals=30,frameMaxExtraDraws=8192;
struct FrameRun{
 enum Phase{IDLE,WAIT_PRESENT,OPEN,PROBE,DONE} phase=IDLE;
 FrameCapture*cap=nullptr;int sequence=0;
 unsigned otherShader=0,selection=0,noCamera=0,otherTarget=0,sideWrites=0,guardSkipped=0;const char*firstGuard=nullptr;
 unsigned viaInitialized=0,viaForwarded=0;   /* which validated route the replayed draws came through */
 unsigned intervals=0,discardedExtra=0;      /* discarded intervals and the extra draws they cost */
 unsigned noPair[10]{},noPairReplayed=0,beforePair=0,listSame=0,listDiffers=0,screenShader=0,unknownShader=0;
 UINT64 unknownHash[4]{};unsigned unknownHashes=0;
 unsigned rdefResolved=0,rdefMismatch=0,beginSkipped=0;const char*firstBeginSkip=nullptr;

 UINT64 rdefHash[64]{};int rdefPers[64]{};unsigned rdefCount=0;
 void resetCounters(){otherShader=selection=noCamera=otherTarget=sideWrites=guardSkipped=viaInitialized=viaForwarded=0;firstGuard=nullptr;
  memset(noPair,0,sizeof noPair);noPairReplayed=beforePair=listSame=listDiffers=screenShader=unknownShader=0;beginSkipped=0;firstBeginSkip=nullptr;}
 char dir[MAX_PATH]{};
} frameRun;
void frameLogMatrices(const float S[16],const float T[16],const FrameCapture&f){
 if(!logger)return;
 logger("frame: begin width=%u height=%u samples=%u depth_func=%s clear_depth=%.0f camera_centre=(%.3f,%.3f,%.3f) game_clip=%s\r\n",f.colorDesc.Width,f.colorDesc.Height,
  f.colorDesc.SampleDesc.Count,f.lessNearer?"less":"greater",f.clearDepth,f.camera[0],f.camera[1],f.camera[2],f.gameClip?"S*Q (y mirrored, z=(z+w)/2)":"S");
 for(int r=0;r<4;r++)logger("frame: row%d S=(%.9g,%.9g,%.9g,%.9g) T=(%.9g,%.9g,%.9g,%.9g)\r\n",r,S[r*4],S[r*4+1],S[r*4+2],S[r*4+3],T[r*4],T[r*4+1],T[r*4+2],T[r*4+3]);
}
void frameReport(int probe){
 auto&f=frameRun;FrameCapture&cap=*f.cap;
 if(logger)logger("frame: replayed=%u other_shader=%u selection_skipped=%u no_camera=%u other_target=%u side_writes=%u guard_skipped=%u (first %s) route initialized/forwarded=%u/%u stop=%s probe=%d\r\n",
  cap.replayed,f.otherShader,f.selection,f.noCamera,f.otherTarget,f.sideWrites,f.guardSkipped,f.firstGuard?f.firstGuard:"none",f.viaInitialized,f.viaForwarded,cap.stopReason?cap.stopReason:"none",probe);
 if(logger){
  logger("frame: shaders table=%u rdef_resolved=%u screen_space=%u unknown=%u (first %016llX %016llX %016llX %016llX) begin_skipped=%u (first %s)\r\n",
   (unsigned)(sizeof frameShaders/sizeof frameShaders[0]),f.rdefResolved,f.screenShader,f.unknownShader,f.unknownHash[0],f.unknownHash[1],f.unknownHash[2],f.unknownHash[3],
   f.beginSkipped,f.firstBeginSkip?f.firstBeginSkip:"none");
  for(unsigned k=0;k<f.rdefCount;k++)logger("frame: shader %016llX gVS_Pers=%d (rdef; -1 screen, -2 not b0, -3 unreadable)\r\n",f.rdefHash[k],f.rdefPers[k]);
  logger("frame: camera pair per list: replayed_without_own_pair=%u before_first_pair=%u other_list_same_D=%u other_list_different_D=%u no_pair_reason inactive/lock/thread/no_consumer/second_consumer/stale/no_pair/epoch=%u/%u/%u/%u/%u/%u/%u/%u\r\n",
   f.noPairReplayed,f.beforePair,f.listSame,f.listDiffers,f.noPair[1],f.noPair[2],f.noPair[3],f.noPair[4],f.noPair[5],f.noPair[6],f.noPair[7],f.noPair[8]);
  for(unsigned k=0;k<sizeof frameShaders/sizeof frameShaders[0];k++){UINT checked=cap.flags[131+2*k],projective=cap.flags[132+2*k];
   if(checked)logger("frame: shader %s pers=c%u draws=%u projective_prefix=%u\r\n",frameShaders[k].name,frameShaders[k].pers,checked,projective);}
  if(cap.flags[131+2*frameRdefSlot])logger("frame: shaders via rdef draws=%u projective_prefix=%u\r\n",cap.flags[131+2*frameRdefSlot],cap.flags[132+2*frameRdefSlot]);
 }
 /* The first banks as the GPU saw them, and their prefix X = P^T * inverse(S). */
 for(UINT n=0;n<4&&n<cap.flags[2]&&logger;n++){
  float bank[32];memcpy(bank,cap.flags+3+n*32,sizeof bank);
  for(int r=0;r<8;r++)logger("frame: bank%u pers-4+%d=(%.9g,%.9g,%.9g,%.9g)\r\n",n,r,bank[r*4],bank[r*4+1],bank[r*4+2],bank[r*4+3]);
  double x[16]{};for(int i=0;i<4;i++)for(int j=0;j<4;j++)for(int k=0;k<4;k++)x[i*4+j]+=(double)bank[(4+k)*4+i]*cap.sinv[k*4+j];
  for(int r=0;r<4;r++)logger("frame: bank%u prefix X row%d=(%.6g,%.6g,%.6g,%.6g)\r\n",n,r,x[r*4],x[r*4+1],x[r*4+2],x[r*4+3]);
 }
 FrameVerdict v=cap.verdict();GeoTotals t;t.add(cap.raw);GeoRaw raw(cap.raw);
 if(logger)logger("frame: draws_checked=%u projective_prefix=%u same_camera_written=%u equal=%.4f contradictions=%.4f (limit %.2f) native_only=%u\r\n",
  v.checked,v.otherCamera,raw.sameWritten,v.equalShare,v.contradictionShare,frameMaxContradiction,raw.nativeOnly);
 GeoPrediction a;
 if(raw.samples&&geoPredict(cap.sinv,cap.m,cap.camera,cap.colorDesc.Width,cap.colorDesc.Height,raw.minDepth,raw.maxDepth,a)&&logger)
  logger("frame: predict depth=%.6g/%.6g/%.6g distance=%.0f/%.0f/%.0f dx=%+.2f/%+.2f/%+.2f max_abs_dy=%.3f/%.3f/%.3f fit a=%+.3f b=%+.1f\r\n",
   a.depth[0],a.depth[1],a.depth[2],a.distance[0],a.distance[1],a.distance[2],a.dxMean[0],a.dxMean[1],a.dxMean[2],a.dyMaxAbs[0],a.dyMaxAbs[1],a.dyMaxAbs[2],a.a,a.b);
 static const char*names[]={"near","mid","far"};
 for(int b=0;b<GEO_BINS;b++){auto&g=t.bin[b];auto&r=v.geo.bin[b];
  if(logger)logger("frame: bin=%s pixels=%llu distance=%.0f..%.0f inliers=%llu share=%.2f unwritten=%llu nonplanar=%llu residual_beyond_window=%llu flat=%llu outside=%llu info_x/y=%.2f/%.2f shift_px=(%+.3f,%+.3f) measured_x/y=%d/%d predicted_dx/dy=%+.2f/%+.2f tested=%d pass=%d\r\n",
   names[b],g.pixels,g.pixels?g.minZ:0.f,g.maxZ,g.inliers,r.inlierShare,g.rejected,g.nonplanar,g.residual,g.flat,g.outside,r.infoX,r.infoY,r.sx,r.sy,r.measuredX,r.measuredY,r.meanDx,r.meanDy,r.tested,r.pass);}
 if(logger)logger("frame: verdict=%s complete=%d vertical=%d horizontal=%d scope=selected_world_draws_one_present_interval integrity=bindings_restored_checked_resources_desk_only\r\n",
  v.status,v.geo.complete,v.geo.vertical,v.geo.horizontal);
 if(probe==1&&cap.a.color){
  char pa[MAX_PATH],pb[MAX_PATH];DWORD tick=GetTickCount();
  sprintf_s(pa,"%sdg_frame_%lu_same_camera.bmp",f.dir,tick);sprintf_s(pb,"%sdg_frame_%lu_other_eye.bmp",f.dir,tick);
  bool okA=dgDumpBmp(device.Get(),context.Get(),cap.a.color.Get(),cap.colorRead,pa),okB=dgDumpBmp(device.Get(),context.Get(),cap.b.color.Get(),cap.colorRead,pb);
  if(logger)logger("frame: images same_camera=%s (%d) other_eye=%s (%d)\r\n",pa,okA,pb,okB);
 }
}

template<class F>void frameDraw(ID3D11DeviceContext*c,F next,void*api,UINT drawCount,bool indexedDraw,bool initialized,bool forwarded){
 auto&f=frameRun;
 if(f.phase!=FrameRun::OPEN||(f.cap&&f.cap->status!=FRAME_OPEN)||!indexedDraw||!(initialized||forwarded)){next();return;}
 ComPtr<ID3D11VertexShader>shader;c->VSGetShader(&shader,nullptr,nullptr);
 const BYTE*code=nullptr;UINT32 codeSize=0;
 UINT64 hash=vsHashOf(shader.Get(),&code,&codeSize);int slot=-1;UINT pers=0;
 for(int k=0;k<(int)(sizeof frameShaders/sizeof frameShaders[0]);k++)if(frameShaders[k].hash==hash){slot=k;pers=frameShaders[k].pers;break;}
 /* Outside the table: the shader's own reflection says where gVS_Pers is
    (vs_rdef_test: 27/27 dumped game shaders agree with the table). */
 int rdef=-3;
 if(slot<0&&hash){
  bool cached=false;for(unsigned k=0;k<f.rdefCount;k++)if(f.rdefHash[k]==hash){rdef=f.rdefPers[k];cached=true;break;}
  if(!cached){rdef=code?dgVsPersIndex(code,codeSize):-3;if(f.rdefCount<64){f.rdefHash[f.rdefCount]=hash;f.rdefPers[f.rdefCount++]=rdef;}if(rdef>=16)++f.rdefResolved;}
  if(rdef>=16&&rdef<=60){slot=(int)frameRdefSlot;pers=(UINT)rdef;}
 }
 if(slot<0){
  bool screen=rdef==-1;for(UINT64 h:frameScreenShaders)if(h==hash)screen=true;
  if(screen)++f.screenShader;
  else{++f.unknownShader;bool seen=false;for(unsigned k=0;k<f.unknownHashes;k++)if(f.unknownHash[k]==hash)seen=true;if(!seen&&f.unknownHashes<4)f.unknownHash[f.unknownHashes++]=hash;}
  ++f.otherShader;next();return;
 }
 const char*sel=lateTarget(c,drawCount,true,ui2dSceneAllowed(),false);
 if(sel&&strcmp(sel,"depth_disabled")){++f.selection;next();return;}   /* depth-less draws belong to the image too */
 float S[16],T[16];int seq=0;
 bool pair=dg_render_second_camera(S,T,&seq)!=0;
 if(!pair){int r=dg_render_second_camera_reason();if(r>=0&&r<10)++f.noPair[r];++f.noCamera;
  if(!f.cap){++f.beforePair;next();return;}                  /* no D for this interval yet */
 }
 const char*reason=nullptr;
 if(!sitesOwned())reason="hook_ownership_changed";
 else if(initialized?!initializedApi(c,api):!forwardApi(api))reason=initialized?"initialized_entry_changed":"forward_entry_changed";
 else if(scopeCreated)reason="query_scope_created_since_device_init";
 else if(!multi->GetMultithreadProtected())reason="context_not_serialized";
 else if(!ownerThread||ownerThread!=GetCurrentThreadId())reason="not_present_thread";
 else if(!f.cap){
  f.cap=new(std::nothrow)FrameCapture;
  if(!f.cap)reason="allocation";
  else if((reason=f.cap->begin(device.Get(),c,S,T))==nullptr){f.sequence=seq;frameLogMatrices(S,T,*f.cap);}
  else if(!strcmp(reason,"render_target_count")||!strcmp(reason,"frame_requires_msaa")||!strcmp(reason,"target_mismatch")||
          !strcmp(reason,"format")||!strcmp(reason,"depth_func")){

   if(!f.beginSkipped++)f.firstBeginSkip=reason;delete f.cap;f.cap=nullptr;next();return;
  }
 }
 else if(pair&&seq!=f.sequence){
  /* Another list with its own camera pair: the same D? */
  float Sg[16],Tg[16];dgGameClip(S,Sg);dgGameClip(T,Tg);dgReplayNearApply(Tg,InterlockedCompareExchange(&seNearMm,0,0));dgReplaySameCamera(Sg,Tg);double dd[16];double worst=1e9;
  if(dg_second_camera_delta(Sg,Tg,dd)){worst=0;for(int k=0;k<16;k++)worst=fmax(worst,fabs(dd[k]-(double)f.cap->delta[k]));}
  if(worst>1e-4){++f.listDiffers;next();return;}
  ++f.listSame;
 }
 if(!reason&&f.cap&&(f.cap->replayed>=frameMaxDraws||f.discardedExtra+2*(f.cap->replayed+1)>frameMaxExtraDraws))reason="draw_limit";
 /* A guard that fails before any replay: this draw is not replayed in A nor B
    (consistent), counted. Starting failures and the limit end the capture. */
 bool fatal=!f.cap||f.cap->status!=FRAME_OPEN||(reason&&!strcmp(reason,"draw_limit"));
 if(reason&&!fatal){if(!f.guardSkipped++)f.firstGuard=reason;next();return;}
 if(reason){
  if(f.cap)f.cap->stop(reason);
  if(logger)logger("frame: stopped reason=%s replayed=%u extra_draws_so_far=%u\r\n",reason,f.cap?f.cap->replayed:0,f.cap?2*f.cap->replayed:0);
  next();return;
 }
 next();                                                     /* the game's draw, unchanged */
 inside=true;const char*why=f.cap->replay(device.Get(),c,next,pers,(UINT)slot);inside=false;
 if(!why){if(initialized)++f.viaInitialized;else ++f.viaForwarded;if(!pair)++f.noPairReplayed;}
 if(why&&!strcmp(why,"other_target"))++f.otherTarget;
 else if(why&&!strcmp(why,"side_writes"))++f.sideWrites;
 else if(why&&logger)logger("frame: stopped reason=%s replayed=%u\r\n",why,f.cap->replayed);
}

extern "C" int dg_xr_capture_texture(void*,int,const void*,const void*);
extern "C" int dg_xr_snapshot_stores(void*out[2]);
#pragma comment(lib, "user32.lib")      /* GetAsyncKeyState: Ctrl+F10 image dump */
extern void*ui2dBackbufferPtr;                /* dg_ui2d.inl: the swap chain's buffer, identity only */

volatile LONG seMode=0;
volatile LONG seNearMm=0;             /* vr_se_near=<mm>: replay-eye near plane (dg_replay_near.h), 0 = off */
char secondEyeDir[MAX_PATH]{};       /* <game>\\logs\\, from the marker path */
#include "dg_second_camera_cpu.h"
#include "dg_draw_batch.h"
#include "dg_clear_view.h"
#include "dg_cb_shadow.inl"
void noteMissedClearSlots(unsigned epoch);

extern "C" long dg_xr_ctx_work_seq(void);
extern "C" long dg_xr_ctx_work_seq_none(void){return 0;}
#pragma comment(linker, "/alternatename:dg_xr_ctx_work_seq=dg_xr_ctx_work_seq_none")
long afterPresentXrSeq=0;bool afterPresentXrValid=false;
unsigned long long xrWindowBusy[2]{},xrWindowQuiet[2]{};
void ui2dEnsureCopyHooks();
#include "dg_post_probe_api.h"
#define DG_POST_MAPPED(g,m) postProbeMapped(g,m)
#include "dg_history_probe_core.h"
#include "dg_hdfix_colour.inl"
extern volatile LONG ui2dGpuPatched;   /* dg_ui2d.inl (below): the bound sprite bank is the ui2d-patched one */
extern volatile LONG ui2dSceneCopyLearned,ui2dSkipCopy;   /* dg_ui2d.inl: scene copies learned / draws on them left unplaced */
volatile LONG ammoHiddenWhileBuilding=0;   /* dg_ammo_capture.inl: hidden natively while B was being built - never recorded for B */
struct SecondEyeLive{
 DG_MAGWELL magwell{};float magwellTarget[16]{};bool magwellDone=false;
 enum State{IDLE,BUILDING,COMPLETE} state=IDLE;
 FramePass b;D3D11_TEXTURE2D_DESC colorDesc{},depthDesc{};D3D11_RENDER_TARGET_VIEW_DESC rtvDesc{};D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc{};
 ComPtr<ID3D11RenderTargetView>gameRT;ComPtr<ID3D11DepthStencilView>gameDS;
 SecondCameraGpu gpu;float delta[16]{},sinvF[16]{},clearDepth=0;int sourceEye=-1,sequence=0;

 SecondCameraCpu cpu;bool cpuOff=false;const char*cpuOffReason=nullptr,*cpuMissLast="none";
 unsigned long long cpuBinds=0,gpuBinds=0,cpuMiss=0,sinceCheck=0;static constexpr unsigned cpuCheckEvery=2048;
 ComPtr<ID3D11Buffer>checkStaging;UINT checkWidth=0,checkBegin=0,checkBytes=0,checkPers=0,checkWait=0;bool checkPending=false;float checkScale=0;
 std::vector<uint8_t>checkWant;unsigned checkOk=0,checkBad=0,checkDropped=0,checkSplitSkipped=0;
 /* Batched replay (dg_draw_batch.h): world draws are recorded and replayed into B in batches, one
    depth-target switch per batch instead of two per draw. Draws the batch cannot take (no shadow,
    a GPU-route sample) flush it and use the immediate route. */

 struct GpuFrame{ComPtr<ID3D11Query>dis,start,end,fb[32];UINT n=0;bool open=false,queued=false;};GpuFrame gq[6];UINT gqCur=0;
 double gqB=0,gqSpan=0;unsigned gqFrames=0,gqDisjoint=0,gqLost=0;
 void gqBegin(ID3D11Device*d,ID3D11DeviceContext*c){
  GpuFrame&g=gq[gqCur];if(g.queued||g.open)return;
  if(!g.dis){D3D11_QUERY_DESC q{D3D11_QUERY_TIMESTAMP_DISJOINT};if(FAILED(d->CreateQuery(&q,&g.dis)))return;q.Query=D3D11_QUERY_TIMESTAMP;
   if(FAILED(d->CreateQuery(&q,&g.start))||FAILED(d->CreateQuery(&q,&g.end)))return;for(auto&x:g.fb)if(FAILED(d->CreateQuery(&q,&x)))return;}
  c->Begin(g.dis.Get());c->End(g.start.Get());g.n=0;g.open=true;
 }
 void gqMark(ID3D11DeviceContext*c){GpuFrame&g=gq[gqCur];if(g.open&&g.n<32)c->End(g.fb[g.n++].Get());}
 void gqEnd(ID3D11DeviceContext*c){GpuFrame&g=gq[gqCur];if(!g.open)return;if(g.n&1)g.n--;c->End(g.end.Get());c->End(g.dis.Get());g.open=false;g.queued=true;gqCur=(gqCur+1)%6;}
 void gqPoll(ID3D11DeviceContext*c){
  for(auto&g:gq){if(!g.queued)continue;D3D11_QUERY_DATA_TIMESTAMP_DISJOINT dj{};
   HRESULT hr=c->GetData(g.dis.Get(),&dj,sizeof dj,D3D11_ASYNC_GETDATA_DONOTFLUSH);if(hr==S_FALSE)continue;g.queued=false;if(FAILED(hr)){++gqLost;continue;}
   if(dj.Disjoint||!dj.Frequency){++gqDisjoint;continue;}
   UINT64 a=0,e=0;if(c->GetData(g.start.Get(),&a,8,D3D11_ASYNC_GETDATA_DONOTFLUSH)!=S_OK||c->GetData(g.end.Get(),&e,8,D3D11_ASYNC_GETDATA_DONOTFLUSH)!=S_OK){++gqLost;continue;}
   double b=0;bool ok=true;for(UINT i=0;i+1<g.n;i+=2){UINT64 x=0,y=0;if(c->GetData(g.fb[i].Get(),&x,8,D3D11_ASYNC_GETDATA_DONOTFLUSH)!=S_OK||c->GetData(g.fb[i+1].Get(),&y,8,D3D11_ASYNC_GETDATA_DONOTFLUSH)!=S_OK){ok=false;break;}if(y>x)b+=(double)(y-x);}
   if(!ok){++gqLost;continue;}
   gqB+=b*1000.0/dj.Frequency;gqSpan+=(e>a?(double)(e-a):0)*1000.0/dj.Frequency;++gqFrames;}
 }

 unsigned guardEpoch=0,guardSeen=~0u;bool guardSites=false;void*guardApi[4]{};bool guardApiOk[4]{};UINT guardApis=0;
 unsigned long long guardFull=0;LONGLONG pathQpc=0;unsigned long long rearmedPending=0;
 bool guardOk(ID3D11DeviceContext*c,void*api,bool initialized,const char**why){
  if(guardSeen!=guardEpoch){guardSeen=guardEpoch;guardApis=0;guardSites=forwardMonitor();guardSites=sitesOwned()&&guardSites;++guardFull;}   /* monitor first (side effects: LOST + put back); the frame that sees a loss refuses as a whole */
  if(!guardSites){*why="hook_ownership_changed";return false;}
  if(initialized&&(c!=context.Get()||(!InterlockedCompareExchange(&seDirect,0,0)&&(*(void***)c)[12]!=api))){*why="entry_changed";return false;}
  for(UINT i=0;i<guardApis;i++)if(guardApi[i]==api){if(!guardApiOk[i])*why="entry_changed";return guardApiOk[i];}
  bool ok=initialized?initializedApi(c,api):forwardApi(api);++guardFull;
  if(guardApis<4){guardApi[guardApis]=api;guardApiOk[guardApis++]=ok;}
  if(!ok)*why="entry_changed";return ok;
 }

 void*mainColorRes=nullptr,*mainDepthRes=nullptr;
 struct PostPair{void*rt=nullptr,*ds=nullptr;ID3D11RenderTargetView*grtv=nullptr;ID3D11DepthStencilView*gdsv=nullptr;   /* the game's (compared only) */
  ComPtr<ID3D11Texture2D>b3,b4;ComPtr<ID3D11RenderTargetView>b3rt;ComPtr<ID3D11DepthStencilView>b4ds;unsigned long long lastUse=0;};

 static constexpr unsigned ppSlots=4,ppRelearnBudget=4;   /* budget: relearns per second before the warning */
 PostPair pp[ppSlots];int ppCur=-1;
 unsigned long long ppEvicted=0,ppEvictedAtStat=0,ppBudgetWarnings=0;unsigned long long ppWindowTick=0;unsigned ppWindowLearns=0;
 unsigned ppInUse()const{unsigned n=0;for(auto&q:pp)if(q.rt)++n;return n;}
 bool postUsed=false;const char*postRefusal="none";
 unsigned long long postRecorded=0,postRefused=0,postOther=0,postMain=0,postClears=0,postPublished=0,postLearned=0,postGs=0;
 struct Swap{ID3D11ShaderResourceView*game;void*res;ComPtr<ID3D11ShaderResourceView>mine;};Swap swaps[16];UINT swapN=0;
 unsigned plainEpoch=~0u;void*plainApi=nullptr;bool plainVal=false;
 float srcClip[16]{};                                   /* this frame's source camera, game clip (samples) */
 struct ProjSample{UINT64 h;};ProjSample projSeen[8];UINT projN=0;
 bool evidencePending=false,evidenceManual=false;unsigned evidenceShots=0;
 bool plainOk(void*api){        /* a non-indexed draw entry we may call ourselves: plain d3d11, checked once per frame */
  if(plainEpoch!=guardEpoch||plainApi!=api){plainEpoch=guardEpoch;plainApi=api;plainVal=api&&runtimePointer(api);}
  return plainVal;
 }
 int ppFind(void*R){for(int k=0;k<(int)ppSlots;k++)if(pp[k].rt&&pp[k].rt==R)return k;return -1;}
 bool isPostView(ID3D11RenderTargetView*rt,ID3D11DepthStencilView*ds){for(auto&q:pp)if(q.grtv&&rt==q.grtv&&ds==q.gdsv)return true;return false;}

 unsigned long long csDraws[ppSlots][ppSlots]{},csPatched=0,csAll=0;unsigned csLogged=0;
 void noteCopySampler(ID3D11DeviceContext*c,int k,bool fk,int ht,UINT64 h,UINT n){
  if(k<0)return;   /* draws into a post pair only: the final colour pass and the half-size targets (k<0) read B3 by design */
  ID3D11ShaderResourceView*v=nullptr;c->PSGetShaderResources(0,1,&v);if(!v)return;
  ID3D11Resource*r=nullptr;v->GetResource(&r);void*res=r;if(r)r->Release();v->Release();
  int src=-1;for(int j=0;j<(int)ppSlots&&src<0;j++)if(pp[j].rt&&pp[j].rt==res&&j!=k)src=j;
  if(src<0)return;
  D3D11_VIEWPORT vp{};UINT nvp=1;c->RSGetViewports(&nvp,&vp);
  D3D11_RECT sr{};UINT nsr=1;c->RSGetScissorRects(&nsr,&sr);
  BOOL sc=FALSE;{ComPtr<ID3D11RasterizerState>rs;c->RSGetState(&rs);if(rs){D3D11_RASTERIZER_DESC rd{};rs->GetDesc(&rd);sc=rd.ScissorEnable;}}
  bool patched=InterlockedCompareExchange(&ui2dGpuPatched,0,0)!=0;
  ++csAll;++csDraws[k][src];if(patched)++csPatched;
  if(logger&&csLogged<12){++csLogged;
   logger("second_eye: copy sampler: draw into pair %d reads pair %d (B gets that pair's own B3) vs %016llX n %u viewport %d,%d %dx%d (%u) scissor %s %ld,%ld-%ld,%ld sprite bank %s; B goes to slot %d (0 = left) (first 12 logged)\r\n",
    k,src,(unsigned long long)h,n,(int)vp.TopLeftX,(int)vp.TopLeftY,(int)vp.Width,(int)vp.Height,nvp,sc?"on":"off",
    nsr?sr.left:0L,nsr?sr.top:0L,nsr?sr.right:0L,nsr?sr.bottom:0L,patched?"PATCHED by ui2d (B inherits it)":"pristine",sourceEye);}
 }
 #include "dg_post_half.inl"
 #include "dg_post_final.inl"
 #include "dg_scene_clear.inl"
  static ID3D11ShaderResourceView*postSwap(ID3D11ShaderResourceView*g,void*ctx){
  auto&e=*(SecondEyeLive*)ctx;
  ID3D11Resource*r=nullptr;g->GetResource(&r);void*res=r;if(r)r->Release();
  int ht=e.halfFind(res);if(ht>=0&&e.half[ht].written!=e.frames){e.postHalfFailed=true;++e.halfRefused;return DG_POST_MAPPED(g,nullptr);}
   for(UINT k=0;k<e.swapN;k++)if(e.swaps[k].game==g&&e.swaps[k].res==res)return DG_POST_MAPPED(g,e.swaps[k].mine?e.swaps[k].mine.Get():g);
  ID3D11Resource*mine=res==e.mainColorRes?(ID3D11Resource*)e.b.color.Get():res==e.mainDepthRes?(ID3D11Resource*)e.b.depth.Get():nullptr;
  for(auto&q:e.pp)if(!mine&&res){if(res==q.rt)mine=q.b3.Get();else if(res==q.ds)mine=q.b4.Get();}   /* the other final image = history */
  if(!mine&&ht>=0)mine=e.half[ht].colour.Get();
   ComPtr<ID3D11ShaderResourceView>v;
   if(mine){D3D11_SHADER_RESOURCE_VIEW_DESC dsc{};g->GetDesc(&dsc);if(FAILED(device->CreateShaderResourceView(mine,&dsc,&v)))return DG_POST_MAPPED(g,nullptr);}
  if(e.swapN<16)e.swaps[e.swapN++]={g,res,v};
  else if(v)e.swapOverflow=v; // borrowed return must outlive the local ComPtr
  return DG_POST_MAPPED(g,v?v.Get():g);
 }
 bool readsMain(ID3D11DeviceContext*c){
  ID3D11ShaderResourceView*sv[8]{};c->PSGetShaderResources(0,8,sv);bool hit=false;
  for(auto v:sv)if(v){ID3D11Resource*r=nullptr;v->GetResource(&r);if(r&&(r==mainColorRes||r==mainDepthRes))hit=true;if(r)r->Release();v->Release();}
  return hit;
 }
 const char*refusePost(const char*w){++postRefused;postRefusal=w;return w;}
 int learnPost(ID3D11Device*d,ID3D11DeviceContext*c,ID3D11RenderTargetView*grt,ID3D11DepthStencilView*gds,void*R,void*D){
  int k=-1;for(int i=0;i<(int)ppSlots&&k<0;i++)if(!pp[i].rt)k=i;                   /* an empty place first */
  bool evict=k<0;if(evict){k=0;for(int i=1;i<(int)ppSlots;i++)if(pp[i].lastUse<pp[k].lastUse)k=i;}   /* else the strictly oldest */
  ComPtr<ID3D11Texture2D>t;if(FAILED(((ID3D11Resource*)R)->QueryInterface(IID_PPV_ARGS(&t)))){refusePost("post_not_texture");return -1;}
  D3D11_TEXTURE2D_DESC td{};t->GetDesc(&td);td.BindFlags|=D3D11_BIND_SHADER_RESOURCE|D3D11_BIND_RENDER_TARGET;td.CPUAccessFlags=0;td.MiscFlags=0;td.Usage=D3D11_USAGE_DEFAULT;
  PostPair n;D3D11_RENDER_TARGET_VIEW_DESC rd{};grt->GetDesc(&rd);
  if(FAILED(d->CreateTexture2D(&td,nullptr,&n.b3))||FAILED(d->CreateRenderTargetView(n.b3.Get(),&rd,&n.b3rt))){refusePost("post_allocation");return -1;}
  bool shared=false,ownDepth=false;UINT dsFormat=0;
  if(D){
   for(int i=0;i<(int)ppSlots&&!shared;i++)if(i!=k&&pp[i].ds==D&&pp[i].b4){n.b4=pp[i].b4;n.b4ds=pp[i].b4ds;shared=true;}   /* the final images share one persistent depth buffer */
   if(!shared){ComPtr<ID3D11Texture2D>z;if(FAILED(((ID3D11Resource*)D)->QueryInterface(IID_PPV_ARGS(&z)))){refusePost("post_depth_not_texture");return -1;}
    D3D11_TEXTURE2D_DESC zd{};z->GetDesc(&zd);zd.BindFlags|=D3D11_BIND_DEPTH_STENCIL;zd.CPUAccessFlags=0;zd.MiscFlags=0;zd.Usage=D3D11_USAGE_DEFAULT;
    D3D11_DEPTH_STENCIL_VIEW_DESC dd{};gds->GetDesc(&dd);dsFormat=(UINT)dd.Format;
    if(FAILED(d->CreateTexture2D(&zd,nullptr,&n.b4))||FAILED(d->CreateDepthStencilView(n.b4.Get(),&dd,&n.b4ds))){refusePost("post_depth_allocation");return -1;}
    ownDepth=true;}}
  /* Earlier records retain their resources; flush in order before changing identity/cache (as learnHalf). */
  flushBatch(c,FL_TARGETS);
  /* A fresh allocation holds whatever the memory held: cleared once, here; a shared depth keeps its contents. */
  {const FLOAT z[4]{0,0,0,1};c->ClearRenderTargetView(n.b3rt.Get(),z);}
  if(ownDepth){UINT f=D3D11_CLEAR_DEPTH;if(dsFormat==DXGI_FORMAT_D24_UNORM_S8_UINT||dsFormat==DXGI_FORMAT_D32_FLOAT_S8X24_UINT)f|=D3D11_CLEAR_STENCIL;
   c->ClearDepthStencilView(n.b4ds.Get(),f,clearDepth,0);}
  n.rt=R;n.ds=D;n.grtv=grt;n.gdsv=gds;pp[k]=n;clearPostSwaps();++postLearned;if(evict)++ppEvicted;
  if(logger)logger("second_eye: post pass pair %d learned: target %ux%u fmt %u depth %s (%s); the second eye gets its own copies (pairs in use %u/%u%s)\r\n",k,td.Width,td.Height,(unsigned)td.Format,D?"yes":"none",
   shared?"shared with another pair":"own",ppInUse(),ppSlots,evict?", replaced the least recently used":"");
  unsigned long long now=GetTickCount64();if(now-ppWindowTick>=1000){ppWindowTick=now;ppWindowLearns=0;}
  if(++ppWindowLearns==ppRelearnBudget+1){++ppBudgetWarnings;
   if(logger&&ppBudgetWarnings<=32)logger("second_eye: post pass pairs learned %u times within 1 s (budget %u, pairs in use %u/%u, replaced %llu): a further full-screen target keeps evicting%s\r\n",
    ppWindowLearns,ppRelearnBudget,ppInUse(),ppSlots,ppEvicted,ppBudgetWarnings==32?"; further warnings suppressed":"");}
  return k;
 }
 /* The first projective banks of the post pass, with this frame's camera (what the game puts there instead). */
 void projSample(ID3D11DeviceContext*c,UINT64 h,int pers,bool gsBound,const char*stage="post",UINT drawCount=0,UINT first=0,INT base=0){
  if(!h||pers<16||pers>60)return;
  for(UINT k=0;k<projN;k++)if(projSeen[k].h==h)return;
  if(projN>=8||!logger)return;projSeen[projN++]={h};
  ComPtr<ID3D11Buffer>b;UINT f=0,n=0;if(!ctx1)return;ctx1->VSGetConstantBuffers1(0,1,&b,&f,&n);
  const DgBankShadow*s=b?cbShadowLookup(b.Get(),nullptr):nullptr;
  if(!s||s->start>f*16||s->end<f*16+(UINT)(pers+4)*16){logger("second_eye: projective post bank vs %016llX pers %d gs %d: no shadow\r\n",(unsigned long long)h,pers,gsBound);return;}
  const float*r=(const float*)(s->bytes+f*16+pers*16);const float*o=(const float*)(s->bytes+f*16+(pers-4)*16);
  logger("bank_reject: stage=%s frame=%u source_eye=%d sequence=%d vs=%016llX pers=%d gs=%d draw=%u first=%u base=%d ms=%llu precision=float_roundtrip limit=8 observe_only=1\r\n",
   stage,frames,sourceEye,sequence,(unsigned long long)h,pers,gsBound,drawCount,first,base,(unsigned long long)GetTickCount64());
  logger("second_eye: projective post bank vs %016llX pers %d gs %d rows %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g\r\n",
   (unsigned long long)h,pers,gsBound,r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7],r[8],r[9],r[10],r[11],r[12],r[13],r[14],r[15]);
  logger("  rows before (c%d..): %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g\r\n",pers-4,
   o[0],o[1],o[2],o[3],o[4],o[5],o[6],o[7],o[8],o[9],o[10],o[11],o[12],o[13],o[14],o[15]);
  logger("  this frame's camera (game clip): %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g\r\n",
   srcClip[0],srcClip[1],srcClip[2],srcClip[3],srcClip[4],srcClip[5],srcClip[6],srcClip[7],srcClip[8],srcClip[9],srcClip[10],srcClip[11],srcClip[12],srcClip[13],srcClip[14],srcClip[15]);
  logger("  target camera (game clip, column rows): %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g | %.9g %.9g %.9g %.9g\r\n",
   cpu.Tc[0],cpu.Tc[1],cpu.Tc[2],cpu.Tc[3],cpu.Tc[4],cpu.Tc[5],cpu.Tc[6],cpu.Tc[7],cpu.Tc[8],cpu.Tc[9],cpu.Tc[10],cpu.Tc[11],cpu.Tc[12],cpu.Tc[13],cpu.Tc[14],cpu.Tc[15]);
  float prefix[4]{},terms[4]{};for(int i=0;i<4;i++)for(int k=0;k<4;k++){float t=r[k*4+i]*sinvF[k*4+3];prefix[i]+=t;terms[i]+=fabsf(t);}
  logger("  rejected prefix last column: %.9g %.9g %.9g %.9g; term sums: %.9g %.9g %.9g %.9g; tolerance %.9g\r\n",
   prefix[0],prefix[1],prefix[2],prefix[3],terms[0],terms[1],terms[2],terms[3],SecondCameraCpu::tolerance);
 }

 void postDraw(ID3D11Device*d,ID3D11DeviceContext*c,void*api,bool indexed,UINT n,UINT first,INT base){
  if(state!=BUILDING||batchOff||cpuOff||!b.color||(InterlockedCompareExchange(&seMode,0,0)&2)){++postOther;return;}
  ID3D11RenderTargetView*rts[2]{};ID3D11DepthStencilView*dsv=nullptr;c->OMGetRenderTargets(2,rts,&dsv);
  struct Rel{ID3D11RenderTargetView**r;ID3D11DepthStencilView*d;~Rel(){for(int k=0;k<2;k++)if(r[k])r[k]->Release();if(d)d->Release();}}rel{rts,dsv};
  if(!rts[0]||rts[1]){++postOther;return;}
  ID3D11Resource*rr=nullptr;rts[0]->GetResource(&rr);void*R=rr;if(rr)rr->Release();
   PostProbeScope probe(c,(ID3D11Resource*)R,n,first,base,indexed);
   if(R==mainColorRes){++postMain;return;}                      /* before the world pass (history/background): stage B */
  void*D=nullptr;if(dsv){ID3D11Resource*dr=nullptr;dsv->GetResource(&dr);D=dr;if(dr)dr->Release();}
  int k=ppFind(R),ht=-1;bool fk=false;
  if(k<0&&finalEligible(c,R,D)){if(!learnFinal(d,rts[0],(ID3D11Resource*)R))return;fk=true;
   if(evidenceManual&&evidencePending&&!finSnapBb){dgHistoryColour(d,c,(ID3D11Texture2D*)R,finSnapBb);dgHistoryColour(d,c,(ID3D11Texture2D*)pp[ppCur].rt,finSnapT3);}
  }
  else if(k<0){
   ComPtr<ID3D11Texture2D>t;if(!R||FAILED(((ID3D11Resource*)R)->QueryInterface(IID_PPV_ARGS(&t)))){++postOther;return;}
   D3D11_TEXTURE2D_DESC td{};t->GetDesc(&td);
   bool full=td.SampleDesc.Count==1&&td.Width==colorDesc.Width&&td.Height==colorDesc.Height&&R!=ui2dBackbufferPtr&&readsMain(c);
   if(full){if((k=learnPost(d,c,rts[0],dsv,R,D))<0)return;}
   else if(halfEligible(c,(ID3D11Resource*)R,D,indexed,n)){if((ht=learnHalf(d,rts[0],(ID3D11Resource*)R))<0)return;}
   else{if(halfFind(R)>=0){postHalfFailed=true;++halfRefused;refusePost("half_unowned_input_or_state");}else ++postOther;return;}
  }
  if(k>=0&&D!=pp[k].ds){refusePost("depth_target_changed");return;}
  ID3D11RenderTargetView*ownRt=fk?fin.rt.Get():ht>=0?half[ht].rt.Get():pp[k].b3rt.Get();
  ID3D11Texture2D*ownColour=fk?fin.colour.Get():ht>=0?half[ht].colour.Get():pp[k].b3.Get();
  ID3D11DepthStencilView*ownDepth=fk||ht>=0?nullptr:pp[k].ds?pp[k].b4ds.Get():nullptr;
  {ID3D11Buffer*so[4]{};c->SOGetTargets(4,so);bool side=false;for(auto v:so)if(v){side=true;v->Release();}
   ID3D11UnorderedAccessView*u[8]{};c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,8,u);for(auto v:u)if(v){side=true;v->Release();}
   if(side){refusePost("side_writes");return;}}
  if(fk){const FLOAT z[4]{0,0,0,1};c->ClearRenderTargetView(ownRt,z);}   /* nothing stale under a blended final draw */
  int pers=-1;UINT64 h=0;{ComPtr<ID3D11VertexShader>vs;c->VSGetShader(&vs,nullptr,nullptr);const BYTE*code=nullptr;UINT32 size=0;h=vsHashOf(vs.Get(),&code,&size);
   for(auto&fs:frameShaders)if(fs.hash==h){pers=(int)fs.pers;break;}
   if(pers<0&&h){auto&f=frameRun;bool cached=false;for(unsigned q=0;q<f.rdefCount;q++)if(f.rdefHash[q]==h){pers=f.rdefPers[q];cached=true;break;}
    if(!cached){pers=code?dgVsPersIndex(code,size):-3;if(f.rdefCount<64){f.rdefHash[f.rdefCount]=h;f.rdefPers[f.rdefCount++]=pers;}}}}
  bool gsBound=false;{ComPtr<ID3D11GeometryShader>g;c->GSGetShader(&g,nullptr,nullptr);gsBound=!!g;}
  noteCopySampler(c,k,fk,ht,h,n);
  if(!ctx1)c->QueryInterface(IID_PPV_ARGS(&ctx1));
  if(!ctx1){refusePost("no_context1");return;}
  DgRecordArgs a;a.pers=pers>=16&&pers<=60?pers:-1;if(indexed)a.indexed=(DgIndexedFn)api;else a.plain=(DgDrawFn)api;
  a.n=n;a.first=first;a.base=base;a.ownTarget=true;a.rt=ownRt;a.ds=ownDepth;a.swap=postSwap;a.swapCtx=this;
   if(probe.selected()){flushBatch(c,FL_SAMPLE);probe.target(c,ownColour);}
   int proj=0;const char*r=batch.recordEx(d,ctx1.Get(),cpu,delta,sinvF,cbShadowLookup,nullptr,a,&proj);
  if(r&&!strcmp(r,"ring_wrap")){flushBatch(c,FL_WRAP);r=batch.recordEx(d,ctx1.Get(),cpu,delta,sinvF,cbShadowLookup,nullptr,a,&proj);}
  if(r){
    probe.refused(r);
    if(ht>=0){postHalfFailed=true;++halfRefused;}if(fk)fin.written=~0ull;refusePost(r);return;}
  ++postRecorded;if(fk){fin.written=frames;++finRecorded;}else if(ht>=0){half[ht].written=frames;++halfDraws;}else{pp[k].lastUse=frames;ppCur=k;postUsed=true;}if(gsBound)++postGs;
   if(probe.selected()){flushBatch(c,FL_SAMPLE);probe.done(c);}
  if(proj){++projectiveSkips;projSample(c,h,a.pers,gsBound,"post",n,first,base);}
  if(batch.full())flushBatch(c,FL_CAP);
 }
 DgBatch batch;ComPtr<ID3D11DeviceContext1>ctx1;bool batchOff=false,flushing=false;const char*batchOffReason=nullptr,*recordRefusal="none";
 unsigned long long recorded=0,recordRefused=0,flushes=0,flushed=0,flushWhy[FL_KINDS]{};UINT batchMax=0,sinceSlotCheck=0,flushVerify=0;LONGLONG flushQpc=0;
 void flushBatch(ID3D11DeviceContext*c,int why){
  if(flushing||!batch.pending()||!b.rt)return;
  if(!ctx1&&FAILED(c->QueryInterface(IID_PPV_ARGS(&ctx1)))){batch.clear();return;}
  flushing=true;LARGE_INTEGER t0,t1;QueryPerformanceCounter(&t0);
  TrialPipelineState*before=(++flushVerify%64)==1?new(std::nothrow)TrialPipelineState(c):nullptr;
  gqMark(c);UINT n=batch.flush(ctx1.Get(),b.rt.Get(),b.ds.Get());gqMark(c);
  if(before){TrialPipelineState after(c);++verifyChecks;unsigned long long diff=before->differences(after);delete before;
   if(diff){batchOff=true;batchOffReason="state_changed_after_flush";if(logger)logger("second_eye: batch OFF: pipeline state differs after a flush (mask %llx); immediate route only\r\n",diff);}}
  QueryPerformanceCounter(&t1);flushQpc+=t1.QuadPart-t0.QuadPart;
  ++flushes;flushed+=n;if(n>batchMax)batchMax=n;if(why>=0&&why<FL_KINDS)++flushWhy[why];flushing=false;
 }
 bool disabled=false;const char*disableReason=nullptr;
 unsigned frames=0,published=0,refused=0,eyeMismatch=0,empty=0,verifyChecks=0,otherTarget=0,sideWrites=0,listDiffers=0,guardSkipped=0,projectiveSkips=0;
 unsigned long long draws=0,drawsSinceVerify=0;LONGLONG qpc=0;unsigned statFrames=0;
 /* Support (issue 10): why no second-eye frame starts. Unsynchronised counters, like guardSkipped. */
 unsigned long long idleCat[16]{},beginRefused=0;const char*beginWhy=nullptr;unsigned framesAtStat=0,beginLines=0;
 const char*disable(const char*why){if(!disabled){disabled=true;disableReason=why;if(logger)logger("second_eye: disabled reason=%s after %u frames\r\n",why,frames);}return why;}
 /* Targets follow the game's main pass; rebuilt only when its descriptors change. */
 const char*prepare(ID3D11Device*d,ID3D11DeviceContext*c){
  ID3D11RenderTargetView*rts[8]{};ID3D11DepthStencilView*ds=nullptr;c->OMGetRenderTargets(8,rts,&ds);
  ComPtr<ID3D11RenderTargetView>rt;rt.Attach(rts[0]);ComPtr<ID3D11DepthStencilView>dsv;dsv.Attach(ds);
  bool extra=false;for(int i=1;i<8;i++)if(rts[i]){extra=true;rts[i]->Release();}
  if(extra||!rt||!dsv)return "render_target_count";
  D3D11_RENDER_TARGET_VIEW_DESC rv{};D3D11_DEPTH_STENCIL_VIEW_DESC dv{};rt->GetDesc(&rv);dsv->GetDesc(&dv);
  if(rv.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2DMS||dv.ViewDimension!=D3D11_DSV_DIMENSION_TEXTURE2DMS||dv.Flags)return "requires_msaa";
  ComPtr<ID3D11Resource>r;ComPtr<ID3D11Texture2D>ct,dt;rt->GetResource(&r);if(FAILED(r.As(&ct)))return "color_texture";
  r.Reset();dsv->GetResource(&r);if(FAILED(r.As(&dt)))return "depth_texture";
  D3D11_TEXTURE2D_DESC cd{},zd{};ct->GetDesc(&cd);dt->GetDesc(&zd);
  if(cd.Width!=zd.Width||cd.Height!=zd.Height||cd.SampleDesc.Count!=zd.SampleDesc.Count||cd.SampleDesc.Count<2)return "target_mismatch";
  DXGI_FORMAT typeless,read,stencil,t2,r2,s2;
  if(!msaaFormats(cd.Format,false,typeless,read,stencil)||!msaaFormats(zd.Format,true,t2,r2,s2))return "format";
  if(read!=DXGI_FORMAT_B8G8R8A8_UNORM)return "format_not_bgra";
  ComPtr<ID3D11DepthStencilState>dss;UINT ref=0;c->OMGetDepthStencilState(&dss,&ref);
  D3D11_COMPARISON_FUNC fn=D3D11_COMPARISON_LESS;if(dss){D3D11_DEPTH_STENCIL_DESC x{};dss->GetDesc(&x);fn=x.DepthFunc;}
  float clear;if(fn==D3D11_COMPARISON_LESS||fn==D3D11_COMPARISON_LESS_EQUAL)clear=1;else if(fn==D3D11_COMPARISON_GREATER||fn==D3D11_COMPARISON_GREATER_EQUAL)clear=0;else return "depth_func";
  bool same=b.color&&!memcmp(&cd,&colorDesc,sizeof cd)&&!memcmp(&zd,&depthDesc,sizeof zd)&&!memcmp(&rv,&rtvDesc,sizeof rv)&&!memcmp(&dv,&dsvDesc,sizeof dv);
  if(!same){
   for(auto&h:half)h=PostHalf{};
   b=FramePass{};colorDesc=cd;depthDesc=zd;rtvDesc=rv;dsvDesc=dv;
   auto x=cd;x.Format=typeless;x.BindFlags=D3D11_BIND_RENDER_TARGET|D3D11_BIND_SHADER_RESOURCE;x.CPUAccessFlags=0;x.MiscFlags=0;x.Usage=D3D11_USAGE_DEFAULT;
   if(FAILED(d->CreateTexture2D(&x,nullptr,&b.color))||FAILED(d->CreateRenderTargetView(b.color.Get(),&rtvDesc,&b.rt)))return disable("allocation");
   x=zd;x.Format=DXGI_FORMAT_R24G8_TYPELESS;x.BindFlags=D3D11_BIND_DEPTH_STENCIL|D3D11_BIND_SHADER_RESOURCE;x.CPUAccessFlags=0;x.MiscFlags=0;x.Usage=D3D11_USAGE_DEFAULT;
   if(FAILED(d->CreateTexture2D(&x,nullptr,&b.depth))||FAILED(d->CreateDepthStencilView(b.depth.Get(),&dsvDesc,&b.ds)))return disable("allocation");
   if(b.color.Get()==ct.Get()||b.depth.Get()==dt.Get())return disable("aliasing");
   if(!gpu.init(d))return disable("compute_create");
   swapN=0;for(auto&s:swaps)s.mine.Reset();
  }
  mainColorRes=ct.Get();mainDepthRes=dt.Get();
  gameRT=rt;gameDS=dsv;clearDepth=clear;return nullptr;
 }
 /* First eligible draw of a frame: fresh target, this frame's D. */
 const char*beginFrame(ID3D11Device*d,ID3D11DeviceContext*c,const float S[16],const float T[16],int eye,int seq){
  if(const char*why=prepare(d,c))return why;
  float Sg[16],Tg[16];dgGameClip(S,Sg);dgGameClip(T,Tg);dgReplayNearApply(Tg,InterlockedCompareExchange(&seNearMm,0,0));dgReplaySameCamera(Sg,Tg);memcpy(srcClip,Sg,sizeof srcClip);cpu.setCams(Sg,Tg);gpu.setSourceCamera(Sg);
  double dd[16],si[16];float identity[16]{};for(int i=0;i<4;i++)identity[i*5]=1;
  if(!dg_second_camera_delta(Sg,Tg,dd)||!dg_second_camera_delta(Sg,identity,si))return "camera_matrices";
  for(int i=0;i<16;i++){delta[i]=(float)dd[i];sinvF[i]=(float)si[i];}
  flushBatch(c,FL_FRAME);gqBegin(d,c);postUsed=false;postHalfFailed=false;clearPostSwaps();
  float sceneColour[4]{};takeSceneClear(sceneColour);
  if(sceneClearRoute&&!strcmp(sceneClearRoute,"held last colour"))noteMissedClearSlots(guardEpoch);
  if(afterPresentXrValid&&sceneClearRoute){int fb=!strcmp(sceneClearRoute,"call-site trace (fallback)");
   if(dg_xr_ctx_work_seq()!=afterPresentXrSeq)++xrWindowBusy[fb];else ++xrWindowQuiet[fb];}
  c->ClearRenderTargetView(b.rt.Get(),sceneColour);c->ClearDepthStencilView(b.ds.Get(),D3D11_CLEAR_DEPTH|D3D11_CLEAR_STENCIL,clearDepth,0);
  cbShadowEnsure();ui2dEnsureCopyHooks();
  sourceEye=eye;sequence=seq;state=BUILDING;++frames;cpu.zprojOff=(InterlockedCompareExchange(&seMode,0,0)&256)!=0;
  magwellDone=false;memcpy(magwellTarget,Tg,sizeof magwellTarget);dg_render_magwell(&magwell,seq);
  return nullptr;
 }
 template<class Draw> void replay(ID3D11Device*d,ID3D11DeviceContext*c,Draw draw,UINT pers,UINT slot,void*api,UINT n,UINT first,INT base,UINT64 hash=0){
  ID3D11RenderTargetView*rts[8]{};ID3D11DepthStencilView*ds=nullptr;c->OMGetRenderTargets(8,rts,&ds);
  struct Rel{ID3D11RenderTargetView**r;ID3D11DepthStencilView*d;~Rel(){for(int i=0;i<8;i++)if(r[i])r[i]->Release();if(d)d->Release();}}rel{rts,ds};
  bool same=rts[0]==gameRT.Get()&&ds==gameDS.Get();for(int i=1;i<8;i++)if(rts[i])same=false;
  if(!same){++otherTarget;return;}
  {ID3D11Buffer*so[4]{};c->SOGetTargets(4,so);bool side=false;for(auto v:so)if(v){side=true;v->Release();}
   ID3D11UnorderedAccessView*u[8]{};c->OMGetRenderTargetsAndUnorderedAccessViews(0,nullptr,nullptr,0,8,u);for(auto v:u)if(v){side=true;v->Release();}
   if(side){++sideWrites;return;}}
  LONG mode=InterlockedCompareExchange(&seMode,0,0);
  if(dropProbe.enabled)mode|=1;
  bool sample=!cpuOff&&!checkPending&&++sinceCheck>=cpuCheckEvery;
  if(sample&&cpu.usesSplitBank(d,c,delta,sinvF,pers,cbShadowLookup,nullptr)){
   sample=false;sinceCheck=0;++checkSplitSkipped;  /* unsupported probe layout: render the ordinary CPU bank */
  }
  if(sample)rtEvent(RT_GPU_SAMPLE,guardEpoch,frames,sourceEye,sequence,hash,n,first,base,pers);
  if(!batchOff&&!cpuOff&&api&&!(mode&(1|4))){
   if(!sample){
    if(!cbShadowArmed()){if(batch.pending()){++rearmedPending;flushBatch(c,FL_FALLBACK);}cbShadowEnsure();}
    if(!ctx1)c->QueryInterface(IID_PPV_ARGS(&ctx1));
    int proj=0;const char*r=ctx1?batch.record(d,ctx1.Get(),cpu,delta,sinvF,pers,cbShadowLookup,nullptr,(DgIndexedFn)api,n,first,base,&proj):"no_context1";
    if(r&&!strcmp(r,"ring_wrap")){flushBatch(c,FL_WRAP);r=batch.record(d,ctx1.Get(),cpu,delta,sinvF,pers,cbShadowLookup,nullptr,(DgIndexedFn)api,n,first,base,&proj);}
    if(!r){
     ++recorded;++cpuBinds;++draws;if(proj){++projectiveSkips;projSample(c,hash,pers,false,"world",n,first,base);
      rtEvent(RT_WORLD_PROJECTIVE,guardEpoch,frames,sourceEye,sequence,hash,n,first,base,pers,1);
     }
     if(batch.full()||(mode&8))flushBatch(c,FL_CAP);
     if(++sinceSlotCheck>=64){sinceSlotCheck=0;if(const char*w=dgUnboundedSlotsFull(c)){flushBatch(c,FL_FALLBACK);batchOff=true;batchOffReason=w;
      if(logger)logger("second_eye: batch OFF: the game binds beyond the recorded slots (%s); immediate route only\r\n",w);}}
     return;
    }
    ++recordRefused;recordRefusal=r;
   }
   flushBatch(c,sample?FL_SAMPLE:FL_FALLBACK);                 /* the immediate route below must come after the batch */
  }
  bool verify=++drawsSinceVerify>=512;TrialPipelineState*before=nullptr;
  if(verify){drawsSinceVerify=0;before=new(std::nothrow)TrialPipelineState(c);}
  bool viaCpu=false,cpuProjective=false;
  if(batch.pending())flushBatch(c,FL_FALLBACK);                /* switched modes: nothing may pass the batch */
  if(!cpuOff&&!sample&&!(mode&4)){int proj=0;const char*r=cpu.bind(d,c,delta,sinvF,pers,cbShadowLookup,nullptr,&proj);
   if(!r){viaCpu=true;++cpuBinds;if(proj){++projectiveSkips;cpuProjective=true;}}
   else{++cpuMiss;cpuMissLast=r;if(cpu.checked&&!cpu.supported){cpuOff=true;cpuOffReason=r;if(logger)logger("second_eye: cpu route unavailable (%s); GPU route only\r\n",r);}}}
  if(!viaCpu){
   const char*why=gpu.bind(d,c,delta,sinvF,true,pers,slot);
   if(why){delete before;disable(why);return;}
   ++gpuBinds;if(sample){
    D3D11_BUFFER_DESC bd{};gpu.original->GetDesc(&bd);
    const DgBankShadow*sh=cbShadowLookup(gpu.original.Get(),nullptr);UINT start=0,bytes=0;
    bool valid=!dgBankRange(bd.ByteWidth,gpu.first,gpu.count,pers,sh,&start,&bytes);
    captureSampleBank(guardEpoch,frames,sourceEye,sequence,hash,n,first,base,pers,
     bd.ByteWidth,start,valid?bytes:0,gpu.count,valid?sh->bytes+start:nullptr,delta,sinvF,cpu.Sc,cpu.Tc,cpu.sourceRounding);
    queueCheck(d,c,pers);
   }
  }
  auto rt=b.rt.Get();c->OMSetRenderTargets(1,&rt,b.ds.Get());
  bool observe=sample&&replayExecution.begin(d,c,replayExecutionQuery,logger,GetTickCount64(),dg_replay_sample_banks.written,hash,frames,n,first,base);
  if(dropProbe.k>=0&&pers>=4){ComPtr<ID3D11DeviceContext1>r1;
   if(SUCCEEDED(c->QueryInterface(IID_PPV_ARGS(&r1)))){ComPtr<ID3D11Buffer>bs;UINT fs=0,ns=0;r1->VSGetConstantBuffers1(0,1,&bs,&fs,&ns);
    dropProbe.captureReplay(c,bs.Get(),fs*16+(pers-4)*16);}}   /* what the replay really uses */
  if(dropProbe.k>=0&&api){const BYTE*vc=nullptr;UINT32 vn=0;ComPtr<ID3D11VertexShader>vsh;c->VSGetShader(&vsh,nullptr,nullptr);UINT64 vh=vsHashOf(vsh.Get(),&vc,&vn);
   dropProbe.captureVerts(c,vc,vn,vh,true,[&]{((DgIndexedFn)api)(c,n,first,base);});}   /* same, with the replay's bank */
  dropProbe.beginReplay(c);
  draw();
  dropProbe.endReplay(c,cpuProjective);
  if(observe)replayExecution.end(c);
  if(viaCpu)cpu.restore(c);else gpu.restore(c);c->OMSetRenderTargets(8,rts,ds);
  if(cpuProjective)projSample(c,hash,pers,false,"world",n,first,base);
  if(cpuProjective)rtEvent(RT_WORLD_PROJECTIVE,guardEpoch,frames,sourceEye,sequence,hash,n,first,base,pers,2);
  if(before){TrialPipelineState after(c);++verifyChecks;bool ok=before->equals(after);delete before;if(!ok){disable("state_changed_after_replay");return;}}
  ++draws;
 }
 /* Affine-layout draw chosen as a sample: its bank (from the GPU copy of the
    game's buffer) is compared with the affine CPU result. Split V/P draws are
    excluded above: this oracle does not model their additional conversion. */
 void queueCheck(ID3D11Device*d,ID3D11DeviceContext*c,UINT pers){
  const DgBankShadow*s=cbShadowLookup(gpu.original.Get(),nullptr);if(!s)return;   /* try the next draw */
  D3D11_BUFFER_DESC desc{};gpu.original->GetDesc(&desc);UINT begin=0,bytes=0;
  if(dgBankRange(desc.ByteWidth,gpu.first,gpu.count,pers,s,&begin,&bytes))return;
  if(!checkStaging||checkWidth!=gpu.width){checkStaging.Reset();checkWidth=0;
   D3D11_BUFFER_DESC sd{};sd.ByteWidth=gpu.width;sd.Usage=D3D11_USAGE_STAGING;sd.CPUAccessFlags=D3D11_CPU_ACCESS_READ;
   if(FAILED(d->CreateBuffer(&sd,nullptr,&checkStaging)))return;checkWidth=gpu.width;}
  checkWant.resize(bytes);dgExpectedBank(s,begin,bytes,pers,delta,sinvF,SecondCameraCpu::tolerance,checkWant.data(),cpu.sourceRounding);

  {float mi=0,md=0,in[16];memcpy(in,s->bytes+begin+pers*16,64);for(float v:in)mi=fmaxf(mi,fabsf(v));for(float v:delta)md=fmaxf(md,fabsf(v));checkScale=4*mi*md;}
  c->CopyResource(checkStaging.Get(),gpu.bank.Get());
  checkPending=true;checkBegin=begin;checkBytes=bytes;checkPers=pers;checkWait=0;sinceCheck=0;
 }
 void pollCheck(ID3D11DeviceContext*c){
  if(!checkPending)return;
  D3D11_MAPPED_SUBRESOURCE m{};HRESULT hr=c->Map(checkStaging.Get(),0,D3D11_MAP_READ,D3D11_MAP_FLAG_DO_NOT_WAIT,&m);
  if(hr==DXGI_ERROR_WAS_STILL_DRAWING){if(++checkWait>8){checkPending=false;++checkDropped;}return;}
  checkPending=false;if(FAILED(hr)){++checkDropped;return;}
  const uint8_t*g=(const uint8_t*)m.pData+checkBegin;unsigned bad=0,at=~0u;uint32_t ga=0,cb=0;
  for(UINT w=0;w<checkBytes/4;w++){uint32_t a,b2;memcpy(&a,g+w*4,4);memcpy(&b2,&checkWant[w*4],4);bool ok=a==b2;
   if(!ok&&w>=checkPers*4&&w<checkPers*4+16){float fa,fb;memcpy(&fa,&a,4);memcpy(&fb,&b2,4);ok=fabsf(fa-fb)<=1e-5f*(fabsf(fa)+fabsf(fb)+checkScale)+1e-30f;}
   if(!ok){if(!bad){at=w;ga=a;cb=b2;}++bad;}}
  c->Unmap(checkStaging.Get(),0);
  if(!bad){++checkOk;return;}
  ++checkBad;
  if(logger)logger("second_eye: cpu/gpu bank MISMATCH %u of %u words, first word %u (pers %u) gpu=%08X cpu=%08X; check %u\r\n",bad,checkBytes/4,at,checkPers,ga,cb,checkBad);
  if(checkBad>=3&&!cpuOff){cpuOff=true;cpuOffReason="cross_check_mismatch";if(logger)logger("second_eye: cpu route OFF after %u mismatches; GPU route only\r\n",checkBad);}
 }
} secondEye;
bool secondEyeLive(){return dg_second_eye_enabled()&&!frameRequest&&!secondEye.disabled&&InterlockedCompareExchange(&requested,0,0)!=1;}
/* Context observers (dg_cb_shadow.inl, ui2d copies): the game is about to write. Only the
   draw thread's own calls on the game's context count; our flush's own calls are ignored. */
unsigned long long mapFlushCount(void){return secondEye.flushes;}
void batchHazard(ID3D11DeviceContext*c,int kind,ID3D11Resource*r){
 auto&e=secondEye;
 if(!e.batch.pending()||e.flushing||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 if(r&&!e.batch.references(r))return;
 e.flushBatch(c,kind);
}

enum { CE_REPLAYED, CE_NONINDEXED, CE_NOROUTE, CE_NOCAMERA, CE_SELECT, CE_NOPAIR, CE_GUARD, CE_BEGIN, CE_LISTDIFF, CE_OTHERTARGET, CE_SIDEWRITES, CE_INSTANCED, CE_KINDS };
static const char *const ceNames[CE_KINDS] = { "replayed", "non_indexed", "no_route", "shader_without_camera_bank", "target_check", "no_camera_pair", "guard", "frame_begin", "list_differs", "other_target", "side_writes", "instanced" };
struct Census {
    bool frame = false; unsigned frames = 0, counter = 0;
    unsigned long long cat[CE_KINDS]{}, notMain = 0;
    struct Sel { const char *why; unsigned long long n; } sel[16]{};
    struct Sh { UINT64 hash; int cat; unsigned long long n; UINT verts; int indexed, blend, depthEnable, depthWrite, topo, pers; const char *why; } sh[64]{};
    unsigned shN = 0;
    void addSel(const char *w) { for (auto &s : sel) { if (s.why == w) { s.n++; return; } if (!s.why) { s.why = w; s.n = 1; return; } } }
    void reset() { frames = 0; memset(cat, 0, sizeof cat); notMain = 0; memset(sel, 0, sizeof sel); memset(sh, 0, sizeof sh); shN = 0; }
};
Census census;

struct CeTarget {
    void *res; UINT w, h, samples, bind; DXGI_FORMAT fmt; bool bb, fromMain, mainColor, mainDepth; int dsv;
    unsigned long long draws, cam, pos[3];
    struct { UINT64 h; int pers; unsigned long long n; int blend, depth; UINT verts; int srv[4]; } sh[6];
};
CeTarget ceT[48]; unsigned ceTN = 0;
void *ceMainDepthRes = nullptr;
bool ceWant = true;                      /* record the pass timeline of the next census frame */
struct CeSeg { int t; char kind; unsigned draws, cam; };
CeSeg ceSeg[96]; unsigned ceSegN = 0; bool ceRecording = false, ceRecorded = false;
unsigned ceMainSoFar = 0, ceMainPrev = 0, ceOtherFrame = 0, ceDumps = 0; double ceOtherAvg = 0; DWORD ceLastDump = 0;
void *ceMainRes = nullptr;
struct CePers { UINT64 h; int pers; } cePers[128]; unsigned cePersN = 0;
extern void *ui2dBackbufferPtr;
static int cePersOf(ID3D11DeviceContext *c, UINT64 *hash) {
    ID3D11VertexShader *vs = nullptr; c->VSGetShader(&vs, nullptr, nullptr);
    const BYTE *code = nullptr; UINT32 size = 0; UINT64 h = vsHashOf(vs, &code, &size); if (vs) vs->Release();
    *hash = h;
    for (unsigned i = 0; i < cePersN; i++) if (cePers[i].h == h) return cePers[i].pers;
    int p = code ? dgVsPersIndex(code, size) : -3;
    if (cePersN < 128) cePers[cePersN++] = { h, p };
    return p;
}
static int ceTarget(void *res) {
    if (!res) return -2;
    for (unsigned i = 0; i < ceTN; i++) if (ceT[i].res == res) return (int)i;
    if (ceTN >= 48) return -3;
    CeTarget &t = ceT[ceTN]; memset(&t, 0, sizeof t); t.res = res; t.bb = res == ui2dBackbufferPtr; t.dsv = -1;
    D3D11_RESOURCE_DIMENSION dim{}; ((ID3D11Resource *)res)->GetType(&dim);
    if (dim == D3D11_RESOURCE_DIMENSION_TEXTURE2D) { D3D11_TEXTURE2D_DESC x{}; ((ID3D11Texture2D *)res)->GetDesc(&x); t.w = x.Width; t.h = x.Height; t.samples = x.SampleDesc.Count; t.fmt = x.Format; t.bind = x.BindFlags; }
    return (int)ceTN++;
}
static void ceTimeline(int t, char kind, bool cam) {
    if (!ceRecording) return;
    if (kind == 'd' && ceSegN && ceSeg[ceSegN - 1].t == t && ceSeg[ceSegN - 1].kind == 'd') { ceSeg[ceSegN - 1].draws++; ceSeg[ceSegN - 1].cam += cam; return; }
    if (ceSegN < 96) ceSeg[ceSegN++] = { t, kind, kind == 'd' ? 1u : 0u, kind == 'd' && cam ? 1u : 0u };
}
static void ceMainIds() {
    if (ceMainRes || !secondEye.gameRT) return;
    ID3D11Resource *r = nullptr; secondEye.gameRT->GetResource(&r); ceMainRes = r; if (r) r->Release();
    if (secondEye.gameDS) { r = nullptr; secondEye.gameDS->GetResource(&r); ceMainDepthRes = r; if (r) r->Release(); }
}
static int ceMark(void *res) {
    int i = ceTarget(res);
    if (i >= 0) { if (res == ceMainRes) ceT[i].mainColor = true; if (res == ceMainDepthRes) ceT[i].mainDepth = true; }
    return i;
}
static void censusOther(ID3D11DeviceContext *c, ID3D11RenderTargetView *rt, UINT verts) {
    census.notMain++; ceOtherFrame++; ceMainIds();
    void *res = nullptr; if (rt) { ID3D11Resource *r = nullptr; rt->GetResource(&r); res = r; if (r) r->Release(); }
    int ti = ceMark(res);
    if (ti >= 0) {
        ID3D11DepthStencilView *dv = nullptr; c->OMGetRenderTargets(0, nullptr, &dv);
        if (dv) { ID3D11Resource *r = nullptr; dv->GetResource(&r); int di = ceMark(r); if (r) r->Release(); dv->Release(); ceT[ti].dsv = di; }
        else ceT[ti].dsv = -1;
    }
    UINT64 h = 0; int pers = cePersOf(c, &h); bool cam = pers >= 16 && pers <= 60;
    ceTimeline(ti, 'd', cam);
    if (ti < 0) return;
    CeTarget &t = ceT[ti]; t.draws++; t.cam += cam;
    t.pos[ceMainSoFar == 0 ? 0 : (ceMainPrev && ceMainSoFar * 20 < ceMainPrev * 19) ? 1 : 2]++;
    for (auto &s : t.sh) {
        if (s.n && s.h == h) { s.n++; return; }
        if (!s.n) {
            s.h = h; s.pers = pers; s.n = 1; s.verts = verts;
            { ID3D11ShaderResourceView *sv[8]{}; c->PSGetShaderResources(0, 8, sv); int k = 0;
              for (int i = 0; i < 4; i++) s.srv[i] = -9;
              for (auto v : sv) if (v) { ID3D11Resource *r = nullptr; v->GetResource(&r); if (k < 4) s.srv[k++] = ceMark(r); if (r) r->Release(); v->Release(); } }
            ID3D11BlendState *b = nullptr; FLOAT f[4]; UINT m = 0; c->OMGetBlendState(&b, f, &m);
            if (b) { D3D11_BLEND_DESC bd{}; b->GetDesc(&bd); s.blend = bd.RenderTarget[0].BlendEnable; b->Release(); }
            ID3D11DepthStencilState *ds = nullptr; UINT r = 0; c->OMGetDepthStencilState(&ds, &r);
            if (ds) { D3D11_DEPTH_STENCIL_DESC dd{}; ds->GetDesc(&dd); s.depth = dd.DepthEnable; ds->Release(); } else s.depth = 1;
            return;
        }
    }
}
/* Copies and resolves (ui2d's copy observers), census frames only. */
void censusCopy(ID3D11DeviceContext *c, char kind, ID3D11Resource *dst, ID3D11Resource *src) {
    if (!census.frame || c != context.Get() || !ownerThread || GetCurrentThreadId() != ownerThread) return;
    if (!ceMainRes && secondEye.gameRT) { ID3D11Resource *r = nullptr; secondEye.gameRT->GetResource(&r); ceMainRes = r; if (r) r->Release(); }
    int ti = ceTarget(dst);
    if (ti >= 0 && src && src == ceMainRes) ceT[ti].fromMain = true;
    ceTimeline(ti, kind, false);
}
static void ceReportTargets(double f) {
    for (int shown = 0; shown < 12; shown++) {
        int best = -1; for (unsigned i = 0; i < ceTN; i++) if (ceT[i].draws && (best < 0 || ceT[i].draws > ceT[best].draws)) best = (int)i;
        if (best < 0) break;
        CeTarget &t = ceT[best];
        unsigned long long all = t.pos[0] + t.pos[1] + t.pos[2]; if (!all) all = 1;
        logger("  target T%d %ux%u x%u fmt %u bind 0x%X%s%s%s%s depth T%d: %.1f draws/frame, %.1f with a camera bank; before/during/after world pass %llu/%llu/%llu%%\r\n",
               best, t.w, t.h, t.samples, (unsigned)t.fmt, t.bind, t.bb ? " BACKBUFFER" : "", t.fromMain ? " <-main" : "",
               t.mainColor ? " = MAIN COLOUR" : "", t.mainDepth ? " = MAIN DEPTH" : "", t.dsv,
               t.draws / f, t.cam / f, 100 * t.pos[0] / all, 100 * t.pos[1] / all, 100 * t.pos[2] / all);
        for (auto &s : t.sh) if (s.n) logger("     vs %016llX gVS_Pers %d: %.1f/frame verts %u blend %d depth %d ps-textures T%d T%d T%d T%d\r\n", (unsigned long long)s.h, s.pers, s.n / f, s.verts, s.blend, s.depth, s.srv[0], s.srv[1], s.srv[2], s.srv[3]);
        t.draws = 0;
    }
    for (unsigned i = 0; i < ceTN; i++) logger("  resource T%u %ux%u x%u fmt %u bind 0x%X%s%s%s%s\r\n", i, ceT[i].w, ceT[i].h, ceT[i].samples, (unsigned)ceT[i].fmt, ceT[i].bind,
        ceT[i].bb ? " BACKBUFFER" : "", ceT[i].fromMain ? " <-main" : "", ceT[i].mainColor ? " = MAIN COLOUR" : "", ceT[i].mainDepth ? " = MAIN DEPTH" : "");
    for (unsigned i = 0; i < ceTN; i++) { ceT[i].draws = ceT[i].cam = 0; memset(ceT[i].pos, 0, sizeof ceT[i].pos); memset(ceT[i].sh, 0, sizeof ceT[i].sh); }
    if (ceRecorded && ceSegN) {
        char line[1024]; size_t n = 0; line[0] = 0;
        logger("  pass timeline of one frame (M = main image, Tn = target, c/r = copy/resolve into):\r\n");
        for (unsigned i = 0; i < ceSegN; i++) {
            char item[64]; const CeSeg &s = ceSeg[i];
            char tn[16]; if (s.t == -1) strcpy_s(tn, "M"); else if (s.t < 0) strcpy_s(tn, "?"); else sprintf_s(tn, "T%d", s.t);
            if (s.kind == 'd') sprintf_s(item, "%s:%u%s ", tn, s.draws, s.cam ? "*" : "");
            else sprintf_s(item, "%c>%s ", s.kind, tn);
            size_t k = strlen(item);
            if (n + k >= 900) { logger("    %s\r\n", line); n = 0; line[0] = 0; }
            memcpy(line + n, item, k + 1); n += k;
        }
        if (n) logger("    %s (* = draws with a camera bank)\r\n", line);
    }
    ceSegN = 0; ceRecorded = false; ceRecording = false; ceWant = true;
}

static void censusDraw(ID3D11DeviceContext *c, int cat, const char *why, UINT verts, bool indexed, ID3D11RenderTargetView *gameRT) {
    if (!census.frame || !gameRT) return;
    ID3D11RenderTargetView *rt = nullptr; c->OMGetRenderTargets(1, &rt, nullptr);
    bool main = rt == gameRT;
    if (!main) { censusOther(c, rt, verts); if (rt) rt->Release(); return; }   /* another view of the main texture shows as "= MAIN COLOUR" */
    if (rt) rt->Release();
    ceMainSoFar++; ceTimeline(-1, 'd', cat == CE_REPLAYED);
    census.cat[cat]++;
    if (cat == CE_SELECT && why) census.addSel(why);
    if (cat == CE_REPLAYED) return;
    ID3D11VertexShader *vs = nullptr; c->VSGetShader(&vs, nullptr, nullptr);
    const BYTE *code = nullptr; UINT32 size = 0; UINT64 h = vsHashOf(vs, &code, &size); if (vs) vs->Release();
    for (unsigned i = 0; i < census.shN; i++) if (census.sh[i].hash == h && census.sh[i].cat == cat) { census.sh[i].n++; return; }
    if (census.shN >= 64) return;
    auto &s = census.sh[census.shN++];
    s.hash = h; s.cat = cat; s.n = 1; s.verts = verts; s.indexed = indexed; s.why = why;
    s.pers = code ? dgVsPersIndex(code, size) : -3;
    ID3D11BlendState *b = nullptr; FLOAT f[4]; UINT m = 0; c->OMGetBlendState(&b, f, &m);
    if (b) { D3D11_BLEND_DESC bd{}; b->GetDesc(&bd); s.blend = bd.RenderTarget[0].BlendEnable; b->Release(); } else s.blend = 0;
    ID3D11DepthStencilState *ds = nullptr; UINT r = 0; c->OMGetDepthStencilState(&ds, &r);
    if (ds) { D3D11_DEPTH_STENCIL_DESC dd{}; ds->GetDesc(&dd); s.depthEnable = dd.DepthEnable; s.depthWrite = dd.DepthWriteMask == D3D11_DEPTH_WRITE_MASK_ALL; ds->Release(); } else { s.depthEnable = 1; s.depthWrite = 1; }
    D3D11_PRIMITIVE_TOPOLOGY t{}; c->IAGetPrimitiveTopology(&t); s.topo = (int)t;
}

#include "dg_parity_gpu.inl"
#include "dg_parity_cov.inl"
void censusInstanced(ID3D11DeviceContext*c,UINT verts,bool indexed){
 if(covActive(c)){ComPtr<ID3D11VertexShader>vs;c->VSGetShader(&vs,nullptr,nullptr);coverage.add(vsHashOf(vs.Get(),nullptr,nullptr),DgCoverage::NATIVE,CE_INSTANCED);}
 if(!census.frame||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 censusDraw(c,CE_INSTANCED,nullptr,verts,indexed,secondEye.gameRT.Get());
}
static void censusReport() {
    if (!logger || !census.frames) return;
    double f = census.frames;
    logger("second_eye census: %u frames sampled; per frame into the main image:", census.frames);
    {   int any = 0;
        for (int k = 0; k < CE_KINDS; k++) if (census.cat[k]) { logger(" %s=%.1f", ceNames[k], census.cat[k] / f); any = 1; }
        if (!any) logger(" none (no sampled draw went into the main image: title, menu or theater, or xr tracking lost - no frame published)"); }
    logger("; into other targets=%.1f\r\n", census.notMain / f);
    for (auto &s : census.sel) if (s.why) logger("  target_check refusal %s: %.1f/frame\r\n", s.why, s.n / f);
    ceReportTargets(f);
    /* the 16 largest missing groups */
    for (int shown = 0; shown < 16; shown++) {
        int best = -1; for (unsigned i = 0; i < census.shN; i++) if (census.sh[i].n && (best < 0 || census.sh[i].n > census.sh[best].n)) best = (int)i;
        if (best < 0) break;
        auto &s = census.sh[best];
        logger("  missing vs %016llX %s%s%s: %.1f/frame verts %u %s topo %d blend %d depth %d/%d gVS_Pers %d\r\n",
               (unsigned long long)s.hash, ceNames[s.cat], s.why ? "/" : "", s.why ? s.why : "", s.n / f, s.verts, s.indexed ? "indexed" : "non-indexed",
               s.topo, s.blend, s.depthEnable, s.depthWrite, s.pers);
        s.n = 0;
    }
    census.reset();
}

unsigned long long clearViewNoted=0,clearViewForgot=0,clearViewIgnored=0;

unsigned clearObserverCalls=0;

volatile LONG afterPresentCalls=0,afterPresentTaken=0,afterPresentRearmed=0;

unsigned long long missTaken[HZ_TAKEN_KINDS]{};unsigned missTakenLastMask=0,missTakenEpoch=~0u;
void noteMissedClearSlots(unsigned epoch){
 if(epoch==missTakenEpoch)return;
 missTakenEpoch=epoch;missTakenLastMask=hzTakenMask();++missTaken[hzTakenKind(missTakenLastMask)];
}
unsigned long long traceFallbackNotes=0;
/* The ClearView decision, shared by the observer and the diagnostic fallback;
   route = who saw the call. The caller has checked thread and context. */
void sceneClearViewNote(ID3D11View*v,const FLOAT col[4],const D3D11_RECT*rects,UINT n,const char*route){
 auto&e=secondEye;
 if(inside||e.flushing||!secondEyeLive())return;
 ComPtr<ID3D11RenderTargetView>rt;if(FAILED(v->QueryInterface(IID_PPV_ARGS(&rt)))){++clearViewIgnored;return;}
 ComPtr<ID3D11Resource>res;rt->GetResource(&res);if(!res){++clearViewIgnored;return;}
 D3D11_RENDER_TARGET_VIEW_DESC vd{};rt->GetDesc(&vd);UINT w=0,h=0;ComPtr<ID3D11Texture2D>t;
 if(SUCCEEDED(res.As(&t))){D3D11_TEXTURE2D_DESC d{};t->GetDesc(&d);w=d.Width;h=d.Height;}
 switch(dg_clear_view_decide(vd.ViewDimension==D3D11_RTV_DIMENSION_TEXTURE2DMS,w,h,(const DG_CV_RECT*)rects,n,col)){
 case DG_CV_NOTE:e.noteSceneClear(rt.Get(),res.Get(),col,route);++clearViewNoted;break;
 case DG_CV_FORGET:e.forgetSceneClear(res.Get());++clearViewForgot;break;
 default:++clearViewIgnored;break;
 }
}
void batchMirrorClearView(ID3D11DeviceContext*c,ID3D11View*v,const FLOAT col[4],const D3D11_RECT*rects,UINT n){
 if(!v||!col||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 ++clearObserverCalls;
 sceneClearViewNote(v,col,rects,n,"ClearView observer");
}
void batchMirrorClearRt(ID3D11DeviceContext*c,ID3D11RenderTargetView*v,const FLOAT col[4]){
 auto&e=secondEye;
 if(!v||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 ++clearObserverCalls;
 ID3D11Resource*r=nullptr;v->GetResource(&r);void*res=r;if(r)r->Release();
 if(!inside&&!e.flushing&&secondEyeLive())e.noteSceneClear(v,(ID3D11Resource*)res,col,"ClearRenderTargetView observer");
 for(auto&q:e.pp)if(q.rt&&res==q.rt&&q.b3rt){e.flushBatch(c,FL_CLEAR);c->ClearRenderTargetView(q.b3rt.Get(),col);++e.postClears;}
}
void batchMirrorClearDs(ID3D11DeviceContext*c,ID3D11DepthStencilView*v,UINT f,FLOAT z,UINT8 s){
 auto&e=secondEye;
 if(!v||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 ID3D11Resource*r=nullptr;v->GetResource(&r);void*res=r;if(r)r->Release();
 if(!inside&&!e.flushing&&e.state==SecondEyeLive::BUILDING&&res==e.mainDepthRes){
  uint32_t zbits=0;memcpy(&zbits,&z,sizeof(zbits));
  rtEvent(RT_WORLD_DEPTH_CLEAR,e.guardEpoch,e.frames,e.sourceEye,e.sequence,0,0,0,0,f,(uint64_t(zbits)<<32)|s);
 }
 bool done=false;for(auto&q:e.pp)if(!done&&q.ds&&res==q.ds&&q.b4ds){e.flushBatch(c,FL_CLEAR);c->ClearDepthStencilView(q.b4ds.Get(),f,z,s);++e.postClears;done=true;}
}
void batchTargets(ID3D11DeviceContext*c,UINT n,ID3D11RenderTargetView*const*rt,ID3D11DepthStencilView*ds,bool uavs){
 auto&e=secondEye;
 if(!e.batch.pending()||e.flushing||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 bool same=!uavs&&n!=~0u&&n>=1&&rt&&((rt[0]==e.gameRT.Get()&&ds==e.gameDS.Get())||e.isPostView(rt[0],ds));
 for(UINT i=1;same&&i<n;i++)if(rt[i])same=false;
 if(n==~0u&&!uavs)same=true;                                  /* keep targets, no new UAVs */
 if(!same)e.flushBatch(c,FL_TARGETS);
}
template<class F>int secondEyeDrawBody(ID3D11DeviceContext*c,F next,void*api,UINT drawCount,bool indexedDraw,bool initialized,bool forwarded,UINT firstIndex,INT baseVertex,const char**why);
/* Times the whole second-eye path per draw except the game's own draw. */
template<class F>void secondEyeDraw(ID3D11DeviceContext*c,F next,void*api,UINT drawCount,bool indexedDraw,bool initialized,bool forwarded,UINT firstIndex,INT baseVertex){
 LARGE_INTEGER a,b;LONGLONG own=0;QueryPerformanceCounter(&a);
 auto timed=[&](){LARGE_INTEGER x,y;QueryPerformanceCounter(&x);next();QueryPerformanceCounter(&y);own+=y.QuadPart-x.QuadPart;};
 auto&e=secondEye;unsigned long long ot=e.otherTarget,sw=e.sideWrites;const char*why=nullptr;
 bool cov=covActive(c);CovSnap covS{};if(cov)covS=covTake();
 int cat=secondEyeDrawBody(c,timed,api,drawCount,indexedDraw,initialized,forwarded,firstIndex,baseVertex,&why);
 if(cat>=0&&cat<16&&e.state!=SecondEyeLive::BUILDING)++e.idleCat[cat];   /* where a draw ended while no frame was being built */
 if(cat==CE_REPLAYED&&e.otherTarget!=ot)cat=CE_OTHERTARGET;else if(cat==CE_REPLAYED&&e.sideWrites!=sw)cat=CE_SIDEWRITES;
 if(cov)covRecord(c,covS,cat);
 if(cat>=0)censusDraw(c,cat,why,drawCount,indexedDraw,e.gameRT.Get());
 QueryPerformanceCounter(&b);secondEye.pathQpc+=b.QuadPart-a.QuadPart-own;
}
template<class F>int secondEyeDrawBody(ID3D11DeviceContext*c,F next,void*api,UINT drawCount,bool indexedDraw,bool initialized,bool forwarded,UINT firstIndex,INT baseVertex,const char**why){
 auto&e=secondEye;
 if(inside){next();return -1;}
 /* Stage A: after the world pass, a draw into another target (the post pass: effects). */
 if(e.state==SecondEyeLive::BUILDING&&e.gameRT){
  ID3D11RenderTargetView*rt0=nullptr;c->OMGetRenderTargets(1,&rt0,nullptr);bool other=rt0&&rt0!=e.gameRT.Get();if(rt0)rt0->Release();
  if(other){
   const char*gw=nullptr;
   bool ok=indexedDraw&&(initialized||forwarded)?e.guardOk(c,api,initialized,&gw):e.plainOk(api);
   ok=ok&&!scopeCreated&&multi->GetMultithreadProtected()&&ownerThread&&ownerThread==GetCurrentThreadId();
   if(ok && !e.magwellDone && e.readsMain(c)) {
    e.magwellDone=true;
    if(e.magwell.valid && !e.disabled && dg_bridge_magwell_allowed()) {
     inside=true;e.flushBatch(c,FL_TARGETS);
     auto now=GetTickCount64();
     bool a=magwellGpu.renderPair(device.Get(),c,e.gameRT.Get(),e.gameDS.Get(),e.b.rt.Get(),e.b.ds.Get(),e.magwell,e.srcClip,e.magwellTarget,e.colorDesc.Width,e.colorDesc.Height,e.clearDepth==0,now,logger,magwellQuery);

     inside=false;
     static unsigned reports=0;if(logger && reports++<4)logger("magwell: USP contour pair=%d; visual alignment unverified\r\n",a);
    }
   }
   next();
   if(ok){inside=true;e.postDraw(device.Get(),c,api,indexedDraw,drawCount,firstIndex,baseVertex);inside=false;}
   else e.refusePost(gw?gw:"draw_entry");
   return CE_OTHERTARGET;
  }
 }
 if(!indexedDraw||!(initialized||forwarded)){next();return indexedDraw?CE_NOROUTE:CE_NONINDEXED;}
 ComPtr<ID3D11VertexShader>shader;c->VSGetShader(&shader,nullptr,nullptr);
 const BYTE*code=nullptr;UINT32 codeSize=0;UINT64 hash=vsHashOf(shader.Get(),&code,&codeSize);int slot=-1;UINT pers=0;
 for(int k=0;k<(int)(sizeof frameShaders/sizeof frameShaders[0]);k++)if(frameShaders[k].hash==hash){slot=k;pers=frameShaders[k].pers;break;}
 if(slot<0&&hash){auto&f=frameRun;int rdef=-3;bool cached=false;
  for(unsigned k=0;k<f.rdefCount;k++)if(f.rdefHash[k]==hash){rdef=f.rdefPers[k];cached=true;break;}
  if(!cached){rdef=code?dgVsPersIndex(code,codeSize):-3;if(f.rdefCount<64){f.rdefHash[f.rdefCount]=hash;f.rdefPers[f.rdefCount++]=rdef;}}
  if(rdef>=16&&rdef<=60){slot=(int)frameRdefSlot;pers=(UINT)rdef;}}
 if(slot<0){next();return CE_NOCAMERA;}
 const char*sel=lateTarget(c,drawCount,true,ui2dSceneAllowed(),false);
 if(sel&&strcmp(sel,"depth_disabled")){*why=sel;next();return CE_SELECT;}
 float S[16],T[16];int seq=0;bool pair=dg_render_second_camera(S,T,&seq)!=0;
 if(!pair&&e.state!=SecondEyeLive::BUILDING){next();return CE_NOPAIR;}      /* no D for this frame yet */
 const char*reason=nullptr;
 if(!e.guardOk(c,api,initialized,&reason)){}
 else if(scopeCreated)reason="query_scope";
 else if(!multi->GetMultithreadProtected())reason="context_not_serialized";
 else if(!ownerThread||ownerThread!=GetCurrentThreadId())reason="not_present_thread";
 if(reason){++e.guardSkipped;*why=reason;next();return CE_GUARD;}
 if(e.state!=SecondEyeLive::BUILDING){
  if(const char*bw=e.beginFrame(device.Get(),c,S,T,dg_render_second_camera_eye(),seq)){
   ++e.beginRefused;
   if(bw!=e.beginWhy){e.beginWhy=bw;
    if(logger&&e.beginLines<12){++e.beginLines;logger("second_eye: a frame could not start: %s - Two Eye rendering waits and the headset keeps showing AER\r\n",bw);}}
   next();return CE_BEGIN;}
  e.beginWhy=nullptr;
  dropProbe.factory=replayExecutionQuery;dropProbe.dev=device.Get();
  dropProbe.beginFrame(e.frames,e.delta,e.cpu.Sc,e.cpu.Tc,e.sourceEye);
 } else if(pair&&seq!=e.sequence){
  float Sg[16],Tg[16];dgGameClip(S,Sg);dgGameClip(T,Tg);dgReplayNearApply(Tg,InterlockedCompareExchange(&seNearMm,0,0));dgReplaySameCamera(Sg,Tg);double dd[16];double worst=1e9;
  if(dg_second_camera_delta(Sg,Tg,dd)){worst=0;for(int k=0;k<16;k++)worst=fmax(worst,fabs(dd[k]-(double)e.delta[k]));}
  if(worst>1e-4){++e.listDiffers;next();return CE_LISTDIFF;}
 }
 {  /* the bank rows as the game wrote them (CPU shadow), for the log of a dropped draw */
  const float*pre=nullptr,*prs=nullptr;ComPtr<ID3D11DeviceContext1>c1;
  if(SUCCEEDED(c->QueryInterface(IID_PPV_ARGS(&c1)))){ComPtr<ID3D11Buffer>b0;UINT f0=0,n0=0;c1->VSGetConstantBuffers1(0,1,&b0,&f0,&n0);
   const DgBankShadow*sh=b0?cbShadowLookup(b0.Get(),nullptr):nullptr;UINT at=f0*16+(pers>=4?pers-4:0)*16;
   if(sh&&pers>=4&&sh->start<=at&&sh->end>=at+128){pre=(const float*)(sh->bytes+at);prs=pre+16;}}
  dropProbe.beginNative(c,[](bool v){inside=v;},hash,drawCount,firstIndex,baseVertex,(int)pers,pre,prs);
  if(dropProbe.k>=0&&c1&&pers>=4){ComPtr<ID3D11Buffer>bs;UINT fs=0,ns=0;c1->VSGetConstantBuffers1(0,1,&bs,&fs,&ns);
   dropProbe.captureNative(c,bs.Get(),fs*16+(pers-4)*16);}  /* what the game's draw really uses */
 }
 next();                                                     /* the game's draw, unchanged */
 dropProbe.endNative(c);
 if(dropProbe.k>=0&&api&&dropProbe.wantVerts(drawCount>2?drawCount-2:0)){         /* suspect: clip positions with the game's bank */
  inside=true;dropProbe.captureVerts(c,code,codeSize,hash,false,[&]{((DgIndexedFn)api)(c,drawCount,firstIndex,baseVertex);});inside=false;}
 LARGE_INTEGER t0,t1;QueryPerformanceCounter(&t0);
 inside=true;e.replay(device.Get(),c,next,pers,(UINT)slot,api,drawCount,firstIndex,baseVertex,hash);inside=false;
 QueryPerformanceCounter(&t1);e.qpc+=t1.QuadPart-t0.QuadPart;
 return CE_REPLAYED;
}
void historyProbePresent();
void nativeClearTracePresent();
void secondEyePresent(){
 auto&e=secondEye;
 if(context&&ownerThread&&GetCurrentThreadId()==ownerThread)replayExecution.poll(context.Get(),GetTickCount64());
 if(ownerThread&&GetCurrentThreadId()==ownerThread)
  rtFinish(e.guardEpoch,e.frames,e.sourceEye,e.sequence,e.state);
 nativeClearTracePresent();
 ++e.guardEpoch;e.discardSceneClears();
 dropProbe.endFrame();
 if(context&&logger)dropProbe.poll(context.Get(),logger);
 if(context){if(secondEyeLive())e.flushBatch(context.Get(),FL_PRESENT);else e.batch.clear();e.gqEnd(context.Get());e.gqPoll(context.Get());}
 historyProbePresent();
 if(census.frame){
  census.frames++;ceMainPrev=ceMainSoFar;
  if(ceRecording){ceRecording=false;ceRecorded=true;}

  bool spike=ceOtherAvg>0&&ceOtherFrame>2*ceOtherAvg+50;
  ceOtherAvg=ceOtherAvg?0.9*ceOtherAvg+0.1*ceOtherFrame:ceOtherFrame;
  /* B is only current when this frame had a camera pair (the game's own cameras, e.g. a knockdown, are not ours) */
  bool current=e.state==SecondEyeLive::BUILDING;   /* began this frame; publish sets IDLE */
  if(!nativeClearLightSession&&spike&&current&&ceDumps<5&&GetTickCount()-ceLastDump>30000&&context&&e.gameRT&&e.b.color&&secondEyeDir[0]){
   ceLastDump=GetTickCount();++ceDumps;
   ComPtr<ID3D11Resource>r;e.gameRT->GetResource(&r);ComPtr<ID3D11Texture2D>t;char pa[MAX_PATH],pb[MAX_PATH],pc[MAX_PATH];
   sprintf_s(pa,"%sdg_second_eye_%lu_auto_main.bmp",secondEyeDir,ceLastDump);sprintf_s(pb,"%sdg_second_eye_%lu_auto_B.bmp",secondEyeDir,ceLastDump);
   sprintf_s(pc,"%sdg_second_eye_%lu_auto_final.bmp",secondEyeDir,ceLastDump);
   bool okA=SUCCEEDED(r.As(&t))&&dgDumpBmp(device.Get(),context.Get(),t.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pa);
   bool okB=dgDumpBmp(device.Get(),context.Get(),e.b.color.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pb);
   bool okC=ui2dBackbufferPtr&&dgDumpBmp(device.Get(),context.Get(),(ID3D11Texture2D*)ui2dBackbufferPtr,DXGI_FORMAT_B8G8R8A8_UNORM,pc);
   if(logger)logger("second_eye: automatic images (effect draws %u vs usual %.0f): main=%d B=%d final=%d %s; native eye %d\r\n",ceOtherFrame,ceOtherAvg,okA,okB,okC,pa,e.sourceEye);}
 }
 ceMainSoFar=0;ceOtherFrame=0;ceMainRes=nullptr;ceMainDepthRes=nullptr;
 census.frame=(++census.counter%4)==0;
 if(census.frame&&ceWant&&!ceRecording&&!ceRecorded){ceWant=false;ceRecording=true;}   /* next census frame after a report */
 /* Ctrl+F10: the game's main image (world pass of the native eye) and B, side by side as BMP. */
 {static bool was=false;bool now=false;
  if(now&&!was&&context&&e.gameRT&&e.b.color&&secondEyeDir[0]){
   ComPtr<ID3D11Resource>r;e.gameRT->GetResource(&r);ComPtr<ID3D11Texture2D>t;
   char pa[MAX_PATH],pb[MAX_PATH];unsigned long tick=GetTickCount();
   sprintf_s(pa,"%sdg_second_eye_%lu_native.bmp",secondEyeDir,tick);sprintf_s(pb,"%sdg_second_eye_%lu_B.bmp",secondEyeDir,tick);
   bool okA=SUCCEEDED(r.As(&t))&&dgDumpBmp(device.Get(),context.Get(),t.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pa);
   bool okB=dgDumpBmp(device.Get(),context.Get(),e.b.color.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pb);
   if(logger)logger("second_eye: images saved native=%s (%d) B=%s (%d); native eye %d\r\n",pa,okA,pb,okB,e.sourceEye);}
  was=now;}

 {static bool was11=false;bool now11=false;
  if(now11&&!was11){e.evidencePending=true;e.evidenceManual=true;if(logger)logger("second_eye: Ctrl+F11 - per-eye headset images at the next published frame\r\n");}
  was11=now11;}
 if(e.state==SecondEyeLive::BUILDING)e.state=SecondEyeLive::COMPLETE;
 if(context)e.pollCheck(context.Get());
 if(++e.statFrames>=300&&logger){
  logger("second_eye: bank route cpu=%llu gpu=%llu cpu_miss=%llu (last %s) cross_check ok=%u mismatch=%u dropped=%u projective=%u cpu=%s shadow uploads=%ld untracked=%ld invalidated=%ld split_probe_skipped=%u\r\n",
   e.cpuBinds,e.gpuBinds,e.cpuMiss,e.cpuMissLast,e.checkOk,e.checkBad,e.checkDropped,e.projectiveSkips,e.cpuOff?e.cpuOffReason:"on",
   cbShadowUploads,cbShadowUntracked,cbShadowInvalidated,e.checkSplitSkipped);
  {LARGE_INTEGER qf;QueryPerformanceFrequency(&qf);
   logger("second_eye: map-hazard flushes by map type: write-discard %llu, write-no-overwrite %llu, other %llu\r\n",mapFlushByType[0],mapFlushByType[1],mapFlushByType[2]);
   logger("second_eye: batch recorded=%llu refused=%llu (last %s) flushes=%llu avg=%.1f max=%u why cap/wrap/fallback/sample/map/targets/write/clear/dispatch/present/frame=%llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu/%llu flush_cpu=%.2f ms/frame batch=%s\r\n",
    e.recorded,e.recordRefused,e.recordRefusal,e.flushes,e.flushes?(double)e.flushed/e.flushes:0.0,e.batchMax,
    e.flushWhy[0],e.flushWhy[1],e.flushWhy[2],e.flushWhy[3],e.flushWhy[4],e.flushWhy[5],e.flushWhy[6],e.flushWhy[7],e.flushWhy[8],e.flushWhy[9],e.flushWhy[10],
    e.frames?1000.0*e.flushQpc/qf.QuadPart/e.frames:0.0,e.batchOff?e.batchOffReason:"on");
   logger("second_eye: gpu per frame: B batches %.2f ms, first world draw to Present %.2f ms (%u frames%s, disjoint %u, lost %u); observers re-armed %ld times (%llu with a batch pending)\r\n",
    e.gqFrames?e.gqB/e.gqFrames:0.0,e.gqFrames?e.gqSpan/e.gqFrames:0.0,e.gqFrames,
    e.gqFrames?"":" - no second-eye frame was timed: none published in this window",
    e.gqDisjoint,e.gqLost,cbRearms,e.rearmedPending);
   logger("second_eye: post pass recorded=%llu (with a geometry shader %llu) refused=%llu (last %s) main-before-world=%llu other-targets=%llu clears=%llu published-with-effects=%llu pairs learned=%llu in use=%u/%u replaced=%llu (+%llu this window) relearn budget warnings=%llu\r\n",
    e.postRecorded,e.postGs,e.postRefused,e.postRefusal,e.postMain,e.postOther,e.postClears,e.postPublished,e.postLearned,
    e.ppInUse(),SecondEyeLive::ppSlots,e.ppEvicted,e.ppEvicted-e.ppEvictedAtStat,e.ppBudgetWarnings);
   e.ppEvictedAtStat=e.ppEvicted;
   {char m[256]="";size_t at=0;for(unsigned k=0;k<SecondEyeLive::ppSlots;k++)for(unsigned j=0;j<SecondEyeLive::ppSlots;j++)if(e.csDraws[k][j]&&at+48<sizeof m)
     at+=sprintf_s(m+at,sizeof m-at," pair %u<-%u %llu",k,j,e.csDraws[k][j]);
    logger("second_eye: copy samplers (post draws reading another post target, e.g. the PSG lens on the backup copy) %llu, with a ui2d-patched sprite bank %llu;%s; ui2d scene copies learned %ld skipped %ld; ammo-panel draws hidden while B was built (not in B) %ld\r\n",
     e.csAll,e.csPatched,at?m:" none",ui2dSceneCopyLearned,ui2dSkipCopy,ammoHiddenWhileBuilding);}
   logger("second_eye: final colour pass recorded=%llu published=%llu learned=%llu later swap-chain draws skipped=%llu (vr_se_mode bit 32 = off); MGSHDFix colour applied=%llu failed=%llu (bit 64 = off)\r\n",
    e.finRecorded,e.finPublished,e.finLearned,e.finSkipped,e.hdfix.applied,e.hdfixFailed);
   logger("second_eye: split camera banks (view + projection) converted: mirrored %llu, exact fallback %llu; own-depth effect banks retargeted %llu (1/kappa %.3g..%.3g, w mix max %.2g; refused ill-conditioned %llu); still projective %u\r\n",
    e.cpu.splitMirrored,e.cpu.splitExact,e.cpu.zprojRetargeted,dgZProjStats.uMin<1e29?dgZProjStats.uMin:0.0,dgZProjStats.uMax,dgZProjStats.mixMax,dgZProjStats.guarded,e.projectiveSkips);
   logger("second_eye: cpu per frame: whole second-eye draw path %.2f ms (the game's own draws excluded), full guard checks %.1f\r\n",
    e.frames?1000.0*e.pathQpc/qf.QuadPart/e.frames:0.0,e.frames?(double)e.guardFull/e.frames:0.0);
   censusReport();
   dropProbe.report(logger);
   e.gqB=e.gqSpan=0;e.gqFrames=0;}
  LARGE_INTEGER f;QueryPerformanceFrequency(&f);
  logger("second_eye: frames=%u published=%u refused=%u b_slot_not_source=%u empty=%u draws=%llu (%.0f/frame) replay_cpu=%.2f ms/frame other_target=%u side_writes=%u list_differs=%u guard_skipped=%u verify=%u disabled=%s without_route=%llu\r\n",
   e.frames,e.published,e.refused,e.eyeMismatch,e.empty,e.draws,e.frames?(double)e.draws/e.frames:0.0,e.frames?1000.0*e.qpc/f.QuadPart/e.frames:0.0,
   e.otherTarget,e.sideWrites,e.listDiffers,e.guardSkipped,e.verifyChecks,e.disabled?e.disableReason:"no",
   s12[S12_DETOURED].liveNoRoute+s12[S12_DIRECT].liveNoRoute+s12[S12_REFUSED].liveNoRoute);
  logger("second_eye: slot 12 routes (direct route %s): detoured d3d11+0x%llX calls %llu routed %llu replayed %llu; undetoured d3d11 (last d3d11+0x%llX) calls %llu routed %llu replayed %llu; "
   "not plain d3d11 (last %p) calls %llu routed %llu replayed %llu; while live without a route: detoured %llu undetoured %llu not plain %llu (replayed = main image; the rest of routed is the post pass)\r\n",
   InterlockedCompareExchange(&seDirect,0,0)?"on":"off",d3dRva(s12Last[0]),s12[0].calls,s12[0].routed,s12[0].replayed,d3dRva(s12Last[1]),s12[1].calls,s12[1].routed,s12[1].replayed,
   s12Last[2],s12[2].calls,s12[2].routed,s12[2].replayed,s12[0].liveNoRoute,s12[1].liveNoRoute,s12[2].liveNoRoute);
  forwardReport();
  for(auto&x:s12)x=S12Stat{};

  logger("second_eye: scene clear colour matched %llu missing %llu held last colour %llu (last from the %s); per route: ClearView observer %llu, ClearRenderTargetView observer %llu, call-site trace (fallback) %llu; ClearView observer noted %llu forgot %llu ignored %llu\r\n",
   e.sceneClearMatched,e.sceneClearMissing,e.sceneClearHeld,e.sceneClearRoute?e.sceneClearRoute:"-",e.sceneClearByRoute[0],e.sceneClearByRoute[1],e.sceneClearByRoute[2],
   clearViewNoted,clearViewForgot,clearViewIgnored);

  if(e.draws)logger("second_eye: per 1000 draws: whole second-eye draw path %.3f ms, in-frame observer re-arms %.2f, fallback flushes %.2f (%.0f draws/frame); slot writes batched %ld single %ld\r\n",
   e.frames?1000.0*e.pathQpc/f.QuadPart/(double)e.draws*1000.0:0.0,1000.0*cbRearms/e.draws,1000.0*e.rearmedPending/e.draws,
   e.frames?(double)e.draws/e.frames:0.0,(long)cbBatchedWrites,(long)cbSlotWrites);
  logger("second_eye: XR-thread context work between Present and the clear: observer frames busy %llu quiet %llu; trace-fallback frames busy %llu quiet %llu\r\n",
   xrWindowBusy[0],xrWindowQuiet[0],xrWindowBusy[1],xrWindowQuiet[1]);
  logger("second_eye: missed clears, observer slots back at that moment (once per frame): group 41-54/20/21 with 132 (Flush/Present signature) %llu, group without 132 (ClearRenderTargetView signature) %llu, slot 14 among them %llu, other %llu, none %llu (last mask 0x%X); CB shadows dropped after a lost slot 14 %ld\r\n",
   missTaken[HZ_TAKEN_GROUP132],missTaken[HZ_TAKEN_GROUP],missTaken[HZ_TAKEN_ALL14],missTaken[HZ_TAKEN_OTHER],missTaken[HZ_TAKEN_NONE],missTakenLastMask,(long)cbShadowStaleDrops);
  logger("second_eye: observers after Present: %ld Presents, slots found taken back %ld, re-armed %ld; after ClearRenderTargetView: %ld slots re-armed in %ld calls (ClearView slot %ld); trace fallback notes %llu\r\n",
   (long)afterPresentCalls,(long)afterPresentTaken,(long)afterPresentRearmed,(long)hzClearRearms,(long)hzClearRearmCalls,(long)hzViewRearms,traceFallbackNotes);
  /* A whole window without a second-eye frame while camera draws were seen (gameplay, not a title or menu): say where they ended. */
  unsigned long long idleCamera=0;for(int k=0;k<CE_KINDS&&k<16;k++)if(k!=CE_NONINDEXED&&k!=CE_NOROUTE&&k!=CE_NOCAMERA&&k!=CE_INSTANCED)idleCamera+=e.idleCat[k];
  if(e.frames==e.framesAtStat&&idleCamera&&secondEyeLive()){
   char line[640];size_t n=0;line[0]=0;
   for(int k=0;k<CE_KINDS&&k<16;k++)if(e.idleCat[k]&&n<560){int w=sprintf_s(line+n,sizeof line-n," %s=%llu",ceNames[k],e.idleCat[k]);if(w>0)n+=(size_t)w;}
   logger("second_eye: no frame started in this window; draws ended at:%s; frame start refused %llu times since launch (last reason %s)\r\n",
    n?line:" none seen",e.beginRefused,e.beginWhy?e.beginWhy:"none");}
  memset(e.idleCat,0,sizeof e.idleCat);e.framesAtStat=e.frames;
  e.statFrames=0;}
}
void framePresent(){
 auto&f=frameRun;
 switch(f.phase){
 case FrameRun::WAIT_PRESENT:f.phase=FrameRun::OPEN;log("frame: opened at Present; selected world draws until the next Present, each replayed twice (same camera, other eye)");break;
 case FrameRun::OPEN:
  if(!f.cap)break;                                           /* no selected draw yet: stays open; the selection timeout ends it */
  ++f.intervals;
  if(f.cap->status==FRAME_OPEN&&f.cap->replayed<frameMinReplays&&f.intervals<frameMaxIntervals&&
     f.discardedExtra+2*f.cap->replayed+2*frameMinReplays<=frameMaxExtraDraws){
   f.discardedExtra+=2*f.cap->replayed;
   if(logger)logger("frame: interval %u discarded replayed=%u (< %u) other_shader=%u no_camera=%u extra_draws_spent=%u/%u; next interval\r\n",
    f.intervals,f.cap->replayed,frameMinReplays,f.otherShader,f.noCamera,f.discardedExtra,frameMaxExtraDraws);
   delete f.cap;f.cap=nullptr;f.resetCounters();
   break;
  }
  f.cap->close(context.Get());f.phase=FrameRun::PROBE;InterlockedExchange(&requested,2);
  if(logger)logger("frame: closed at Present interval=%u replayed=%u extra_draws=%u (+%u in discarded intervals) status=%d\r\n",f.intervals,f.cap->replayed,2*f.cap->replayed,f.discardedExtra,(int)f.cap->status);
  break;
 case FrameRun::PROBE:{
  int r=f.cap->poll(context.Get());if(r==0)break;
  frameReport(r);delete f.cap;f.cap=nullptr;f.phase=FrameRun::DONE;break;}
 default:break;
 }
}
template<class F>void intercept(ID3D11DeviceContext*c,F next,void*api,const char*kind,bool forwarded=false,UINT drawCount=0,bool indexedDraw=false,bool initialized=false,UINT firstIndex=0,INT baseVertex=0,bool plainOriginal=false){
 ++interceptSerial;

 if(InterlockedCompareExchange(&requested,0,0)!=1&&!secondEyeLive()){next();return;}
 // Never wait on lifetime while holding the external callback mutex.
 if(!TryAcquireSRWLockShared(&lifetime)){next();return;}
 struct ReleaseLife{~ReleaseLife(){ReleaseSRWLockShared(&lifetime);}} life;
 if(c!=context.Get()||!installed||stopping||inside){next();return;}
 if(InterlockedCompareExchangePointer(&lateXrContext,nullptr,nullptr)!=c || !multi->GetMultithreadProtected()){next();return;}
 MultiLock lock;
 if(secondEyeLive()){secondEyeDraw(c,next,api,drawCount,indexedDraw,initialized,forwarded,firstIndex,baseVertex);return;}
 if(frameRequest){frameDraw(c,next,api,drawCount,indexedDraw,initialized,forwarded);return;}
 SecondCameraBuffer secondCamera;
 float cameraSource[16]{},cameraTarget[16]{};int cameraSequence=0;
 if(secondCameraRequest){
  if(!indexedDraw||!dg_render_second_camera(cameraSource,cameraTarget,&cameraSequence)){next();return;}
  ComPtr<ID3D11VertexShader>shader;c->VSGetShader(&shader,nullptr,nullptr);
  if(vsHashOf(shader.Get(),nullptr,nullptr)!=UINT64_C(0x544F1B615F77E0FD)){next();return;}
  if(geometryRequest){
   if(geo.started&&GetTickCount()-geo.started>=geoWindowMs){InterlockedExchange(&requested,2);geoFinish("window_10s");next();return;}
   if(drawCount<geoMinIndices||geoSeen(api,drawCount,firstIndex,baseVertex)||!geoFullViewport(c)){next();return;}
  }
 }
 if(latePlainImmediate){
  if(!plainOriginal){next();return;}
  if(GetTickCount()-lateSearchStarted>=5000){InterlockedExchange(&requested,2);log("refused=direct_selection_timeout extra_draws=0");next();return;}
 }
 const char* selection=lateTarget(c,drawCount,indexedDraw,ui2dSceneAllowed(),latePlainDraw);
 if(selection){lateLastSkip=selection;++lateSkipped;lateSkip(selection,drawCount,indexedDraw,c);next();return;}
 if(surveyEnabled&&!latePlainImmediate&&!secondCameraRequest){
 if(!initialized&&!forwarded&&!plainOriginal){next();return;} // Validated direct entry or owned adapter original only.
 if(!lateSearchStarted)lateSearchStarted=GetTickCount();
 if(GetTickCount()-lateSearchStarted>=5000 || (!lateMatch&&lateCandidates>=64)){
  reportLateMatch();clearLateMatch();InterlockedExchange(&requested,2);log("refused=contribution_search_limit extra_draws=0");next();return;
 }
 if(lateMatch){
  TrialPipelineState current(c);
  observeLateMatch(api,drawCount,firstIndex,baseVertex,current,c);
  bool matches=api==lateApi&&drawCount==lateCount&&firstIndex==lateFirst&&baseVertex==lateBase&&trialPipelineSelectionEqual(*latePipeline,current);
  if(matches&&!trialTargetsEqual(c,lateRT.Get(),lateDS.Get())){if(targetMismatchCount++==0)log("target_selection refused=descriptor_mismatch; continue_search");matches=false;}
  if(!matches){
   // Only abandon an unmatched plain-Draw survey. This never authorizes replay:

   if(!plainOriginal||!latePlainDraw||lateReselections>=2||GetTickCount()-lateMatchStarted<500||lateCandidates>=64||smallDrawSeen(api,drawCount,firstIndex,baseVertex)){next();return;}
   ++lateReselections;reportLateMatch();clearLateMatch();
   if(logger)logger("draw_trial: reselect=%u/2 unmatched_survey_expired; new_survey_required=1 extra_draws=0\r\n",lateReselections);
  }else{
   if(!latePipeline->equals(current))log("target_selection equivalent_descriptors=1 identity_changed=1; current targets used; contribution still unproven");
   reportLateMatch();log("matched previously contributing survey; final contribution still requires readback");
  }
 }
 if(!lateMatch){if(smallDrawSeen(api,drawCount,firstIndex,baseVertex)){next();return;}++lateCandidates;}
 }
 if(logger)logger("draw_trial: late selected count=%u indexed=%d skipped=%u protection=1 visible_contribution=UNPROVEN\r\n",drawCount,indexedDraw,lateSkipped);
 if(InterlockedCompareExchange(&requested,2,1)!=1){next();return;}
 dg_context_trace_emit(logger,"draw_request",device.Get(),c,multi.Get(),1);
 const char*reason=nullptr;
 if(!sitesOwned())reason="hook_ownership_changed";
 else if(initialized&&!initializedApi(c,api))reason="initialized_entry_changed";
 else if(scopeCreated)reason="query_scope_created_since_device_init";
 else if(!(plainOriginal?mgDrawApi(api):forwarded?forwardApi(api):runtimePointer(api)))reason="foreign_API_entry";
 else if(!multi->GetMultithreadProtected())reason="context_not_serialized";
 else if(!ownerThread||ownerThread!=GetCurrentThreadId())reason="not_present_thread";
 uint64_t base=(uint64_t)(ULONG_PTR)GetModuleHandleW(nullptr);NS_SAMPLE before{},after{};
 if(!reason&&!ns_layout(base,readNative,nullptr))reason="native_layout";
 DrawTrial*t=nullptr;if(!reason){t=new(std::nothrow)DrawTrial;if(!t)reason="allocation";}
 if(!reason)reason=t->prepare(device.Get(),c);
 if(!reason&&secondCameraRequest)reason=secondCamera.prepare(device.Get(),c,UINT64_C(0x544F1B615F77E0FD),cameraSource,cameraTarget);
 double geoSinv[16],geoM[16],geoMinv[16],geoCamera[3];
 if(!reason&&geometryRequest&&!(t->hasDepth&&t->depth.msaa))reason="geometry_requires_msaa_depth";
 if(!reason&&geometryRequest&&!geoMatrices(cameraSource,cameraTarget,geoSinv,geoM,geoCamera,geoMinv))reason="geometry_matrices";
 if(!reason&&!ns_take(&before,base,readNative,nullptr))reason="native_witness_read";
 if(reason){if(logger)logger("draw_trial: refused=%s extra_draws=0\r\n",reason);delete t;
  geoFinish("attempt_refused");
  next();return;}
 bool survey=surveyEnabled&&!latePlainImmediate&&!lateMatch&&!secondCameraRequest;
 if(survey){clearLateMatch();latePipeline=new(std::nothrow)TrialPipelineState(c);
  if(!latePipeline){delete t;log("refused=survey_allocation extra_draws=0");next();return;}
  lateVertexBindings.capture(c);
  lateCount=drawCount;lateFirst=firstIndex;lateBase=baseVertex;lateApi=api;lateRT=t->originalRT;lateDS=t->originalDS;
 }
 inside=true;
 bool repeated=t->run(c,next,[&](){return
 InterlockedCompareExchangePointer(&lateXrContext,nullptr,nullptr)==c && multi->GetMultithreadProtected() &&
 (!drawObservation||drawObservation->good(1)) && (!initialized||initializedApi(c,api)) && (!plainOriginal||mgDrawApi(api)) && SUCCEEDED(device->GetDeviceRemovedReason())&&sitesOwned()&&!scopeCreated&&ns_take(&after,base,readNative,nullptr)&&!memcmp(&before,&after,sizeof before);}
 ,survey,secondCameraRequest?&secondCamera:nullptr
 );
 inside=false;
 if(!repeated){
  if(t->selectionFailure&&!strcmp(t->selectionFailure,"survey_only")){
   lateSurvey=true;pending=t;polls=0;return;
  }
  clearLateMatch();geoFinish("state_changed_after_normal");
  log("refused=state_changed_after_normal extra_draws=0");delete t;return;
 }
 clearLateMatch();
 bool observationValid=!drawObservation||drawObservation->good(2);
 bool witness=ns_take(&after,base,readNative,nullptr)&&!memcmp(&before,&after,sizeof before);
 if(logger)logger("draw_trial: submitted kind=%s thread=%lu extra_draws=1 native_witness_equal=%d timing_perturbed=possible width=%u height=%u depth=%d\r\n",kind,GetCurrentThreadId(),witness,t->color.desc.Width,t->color.desc.Height,t->hasDepth);
 if(secondCameraRequest&&logger)logger("second_camera: consumed_sequence=%d current_world_draw=1 private_cb=1 private_targets=1 source_shader=544F1B615F77E0FD blocking_proof_only=1 stereo_pair_complete=0\r\n",cameraSequence);
 if(!observationValid||!witness)geoFinish("attempt_invalid");
 if(!observationValid){log("original_observation_invalid; result invalid");delete t;return;}
 if(!witness){log("native_witness_changed; result invalid");delete t;return;}pending=t;polls=0;
 if(geometryRequest){
  if(!geo.started)geo.started=GetTickCount();
  geo.keys[geo.attempts]={api,drawCount,firstIndex,baseVertex};++geo.attempts;
  memcpy(geo.sinv,geoSinv,sizeof geoSinv);memcpy(geo.m,geoM,sizeof geoM);memcpy(geo.camera,geoCamera,sizeof geoCamera);memcpy(geo.minv,geoMinv,sizeof geoMinv);
  memcpy(geo.source,cameraSource,sizeof geo.source);memcpy(geo.target,cameraTarget,sizeof geo.target);
  geo.width=t->color.desc.Width;geo.height=t->color.desc.Height;geo.dispatched=geo.read=false;

  if(logger){logger("geometry: attempt=%u/%u count=%u first=%u base=%d camera_centre=(%.3f,%.3f,%.3f)\r\n",geo.attempts,geoMaxAttempts,drawCount,firstIndex,baseVertex,geoCamera[0],geoCamera[1],geoCamera[2]);
   for(int r=0;r<4;r++)logger("geometry: row%d S=(%.9g,%.9g,%.9g,%.9g) T=(%.9g,%.9g,%.9g,%.9g) M=(%.9g,%.9g,%.9g,%.9g)\r\n",r,
    cameraSource[r*4],cameraSource[r*4+1],cameraSource[r*4+2],cameraSource[r*4+3],cameraTarget[r*4],cameraTarget[r*4+1],cameraTarget[r*4+2],cameraTarget[r*4+3],geoM[r*4],geoM[r*4+1],geoM[r*4+2],geoM[r*4+3]);}
 }

}
#include "dg_backbuffer_watch.inl"
bool bpBefore(ID3D11DeviceContext*,UINT,UINT);void bpAfter(bool);
void bpTagRoute(bool,bool);
bool bpWanted();void bpForeign(void*,void*);
bool mgDrawApi(void*);
template<class F> void blitForward(ID3D11DeviceContext*c,UINT n,UINT first,void*api,F next,bool mgForward=false){
 if(n!=4||!bpWanted()||requested||c!=context.Get()||stopping||!ownerThread||ownerThread!=GetCurrentThreadId()){next();return;}
 if(!TryAcquireSRWLockShared(&lifetime)){next();return;}
 struct Unlock{bool held=true;void release(){if(held){ReleaseSRWLockShared(&lifetime);held=false;}}~Unlock(){release();}} unlock;
 if(stopping||!installed){unlock.release();next();return;}
 void*guardApi=wwUnwrap(api);
 if(!(mgForward?mgDrawApi(api):runtimePointer(guardApi))){bpForeign(guardApi,api);unlock.release();next();return;}
 MultiLock lock;bool sample=bpBefore(c,n,first);bpTagRoute(sample,mgForward);next();bpAfter(sample);
}
void ui2dTraceDraw(ID3D11DeviceContext*,UINT,int);void ui2dOnDraw(ID3D11DeviceContext*,unsigned);void ui2dOnPresent();bool ui2dSkipDraw(ID3D11DeviceContext*,UINT); // dg_ui2d.inl
bool ammoHideDraw(ID3D11DeviceContext*,UINT,UINT);void ammoOnDraw(ID3D11DeviceContext*,UINT,UINT);void ammoOnPresent();
bool radarOnDraw(ID3D11DeviceContext*,UINT);void radarOnPresent(); // dg_radar.inl
#include "dg_cb_probe.inl"
#include "dg_mgshd_forward.inl"
#include "dg_mgshd_draw.inl"
static void s12Count(ID3D11DeviceContext*c,void*api,UINT n,UINT first,bool liveBound,bool routed,bool replayed){
 if(s12Epoch!=secondEye.guardEpoch){s12Epoch=secondEye.guardEpoch;s12Pos=0;}unsigned pos=s12Pos++;
 int k=boundEntry(api)?S12_DETOURED:directApi(c,api)?S12_DIRECT:S12_REFUSED;
 auto&s=s12[k];++s.calls;if(routed)++s.routed;if(replayed)++s.replayed;s12Last[k]=api;
 if(!liveBound||routed)return;
 ++s.liveNoRoute;
 if(s12Lines>=16||!logger)return;++s12Lines;
 static const char*const kn[S12_KINDS]={"the MGSHDFix-detoured entry","an undetoured d3d11 entry","an entry that is not plain d3d11 code"};
 ComPtr<ID3D11VertexShader>vsh;c->VSGetShader(&vsh,nullptr,nullptr);const BYTE*code=nullptr;UINT32 size=0;UINT64 h=vsHashOf(vsh.Get(),&code,&size);
 logger("second_eye: slot 12 without a route %u: draw %u in frame %u, slot 12 = %s d3d11+0x%llX (%p), indices %u first %u, vs %016llX, direct route %s\r\n",
  s12Lines,pos,secondEye.guardEpoch,kn[k],d3dRva(api),api,n,first,(unsigned long long)h,InterlockedCompareExchange(&seDirect,0,0)?"on":"off");
}
static void indexedRoute(ID3D11DeviceContext*c,UINT n,UINT first,INT base);

bool s12Live(ID3D11DeviceContext*c){return c==context.Get()&&secondEyeLive();}
void STDMETHODCALLTYPE indexed(ID3D11DeviceContext*c,UINT n,UINT first,INT base){
 void*api=(*(void***)c)[12];
 bool liveBound=s12Live(c);
 auto is=interceptSerial;auto dr=secondEye.draws;
 indexedRoute(c,n,first,base);
 if(c==context.Get()&&ownerThread&&GetCurrentThreadId()==ownerThread)s12Count(c,api,n,first,liveBound,interceptSerial!=is,secondEye.draws!=dr);
}
static void indexedRoute(ID3D11DeviceContext*c,UINT n,UINT first,INT base){
 if(InterlockedCompareExchange(&requested,0,0)==1)InterlockedIncrement(&lateNativeIndexed);
 ui2dOnDraw(c,0);            // 2D sprite placement per eye; idle unless vr_ui2d=1
 cbOnDraw(c,0,n,first,base); // read-only constant-buffer capture, idle unless armed
 void*api=(*(void***)c)[12];auto f=(Indexed)api;
 LONG route=forwardStatus();

 // Never the MGSHDFix-detoured entry (that draw replays through the forward); all intercept guards apply.
 // vr_se_direct=off: the old route, only the separately fingerprinted initialized entry.
 bool direct=InterlockedCompareExchange(&seDirect,0,0)!=0;
 bool routeOpen=route==FORWARD_NONE||route==FORWARD_WAITING||route==FORWARD_BOUND||route==FORWARD_REJECTED;   /* not DIRECT (unchanged, below), not STOPPED */
 /* hand detoured entries to the poll: owner context only, per-frame cached directCode, never a bound entry */
 if(routeOpen&&c==context.Get())noteDetoured(c,api,directCode(api));
 if((requested==1||secondEyeLive()) && routeOpen && c==context.Get() && !boundEntry(api) && (direct?directApi(c,api):initializedAddress(api))){
  if(direct)directSlotNote(api);
  intercept(c,[=](){f(c,n,first,base);},api,direct?"DrawIndexed/direct":"DrawIndexed/validated-initialized",false,n,true,true,first,base);return;
 }
 if(route!=FORWARD_DIRECT){ // NONE/WAITING/BOUND/REJECTED/STOPPED: native only; a bound forward replays inside. No outer locks.
  NativeForwardScope scope;
  bool observing=InterlockedCompareExchange(&requested,0,0)==1;auto serial=lateForwardSerial;
  f(c,n,first,base);
  if(observing)lateRouteResult(route,api,serial,n,c);
  return;
 }
 intercept(c,[=](){f(c,n,first,base);},api,"DrawIndexed",false,n,true);
}
#include "dg_history_probe.inl"
#include "dg_post_probe.inl"
void STDMETHODCALLTYPE draw(ID3D11DeviceContext*c,UINT n,UINT first){
 HistoryDrawScope history(c,n,first);
 if(ui2dSkipDraw(c,n)){ui2dTraceDraw(c,n,1);return;} // previous-frame feedback in stereo: not forwarded
 ammoOnDraw(c,n,first);
 ui2dOnDraw(c,1);cbOnDraw(c,1,n,first,0);
 history.used();
 if(ammoHideDraw(c,n,first))return;
 if(radarOnDraw(c,n)){ui2dTraceDraw(c,n,2);return;} // wrist radar: the composite's texture is captured; with vr_radar_hud=off the draw is not forwarded. No lock held across it.
 ui2dTraceDraw(c,n,0);
 void*api=(*(void***)c)[13];auto f=(Draw)api;
 if(latePlainDraw && InterlockedCompareExchange(&requested,0,0)==1){NativePlainScope scope;history.forwarded();f(c,n,first);return;}
 intercept(c,[=,&history](){blitForward(c,n,first,api,[=,&history](){history.forwarded();f(c,n,first);});},api,"Draw",false,n,false,false,first,0);
}

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
#include "dg_native_clear_trace.inl"
#include "dg_ui2d.inl"
void wwRestoreCopies(void**v){for(unsigned i=0;i<ui2dCopySetCount;i++)if(ui2dCopySets[i].vt==v){auto&s=ui2dCopySets[i];for(Hook*h:{&s.reg,&s.res,&s.rsv})if(h->slot&&*h->slot==h->next&&!writeSlot(*h,h->next,h->ours))ww.installFailures++;}}
unsigned wwCopyMask(void**v){return (v[47]==(void*)copyRes?1u<<9:0)|(v[46]==(void*)copyReg?1u<<10:0)|(v[57]==(void*)resolveSub?1u<<11:0);}
#include "dg_blit_boundary.inl"
bool bpWanted(){return InterlockedCompareExchange(&bpEnabled,0,0)!=0;}void bpForeign(void*api,void*wrapper){bp.foreign++;wwForeign(api,wrapper,bp.mark);}
#include "dg_radar.inl"
#include "dg_ammo_capture.inl"
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

 oldProtected=multi->GetMultithreadProtected()!=FALSE;
 dg_context_trace_emit(logger,"draw_attach",device.Get(),context.Get(),multi.Get(),1);
 if(logger)logger("draw_trial: thread protection preserved=%d forced=0; extra trial draw requires existing protection\r\n",oldProtected);
auto table=*(void***)d;void*ours[]={(void*)createQuery,(void*)createPredicate,(void*)createCounter};bool ok=true;
 for(unsigned i=0;i<3;i++){hooks[i]={table+24+i,table[24+i],ours[i]};if(!runtimePointer(hooks[i].next))ok=false;}
 if(SUCCEEDED(d->QueryInterface(IID_PPV_ARGS(&modernDevice)))){auto mt=*(void***)modernDevice.Get();hooks[3]={mt+60,mt[60],(void*)createQuery1};hookCount=4;if(!runtimePointer(hooks[3].next))ok=false;}
 InterlockedExchangePointer(&queryOwner,d);InterlockedExchangePointer(&modernQueryOwner,modernDevice.Get());
 uint64_t base=(uint64_t)(ULONG_PTR)GetModuleHandleW(nullptr);
 if(ok)ok=prepareSites(base);
 if(ok){HMODULE pin=nullptr;ok=GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)&indexed,&pin)!=FALSE;}
 if(ok){for(unsigned i=0;i<hookCount;i++){auto&h=hooks[i];if(!writeSlot(h,h.next,h.ours)){ok=false;break;}}}
 if(ok)for(auto&s:sites)if(!codeWrite(s,s.original,s.patch)){ok=false;break;}
 if(!ok){restoreAll();log("attach refused: code/API ownership; no draw trial");}
 else{installed=true;ui2dInstall(d);beginForward();
 if(installNativeClearTrace(base))log("native clear call-site trace armed: logs ClearRT and ClearView (8 epochs/max64); scene colour only as a fallback for a call the context observers missed; image copies=0; automatic evidence BMP readbacks disabled");
 else log("native clear trace refused: API/code witness; existing render route retained");
 log("attached: native API callsites only; query policy=timing_allowed_geometry_only; indexed=deferred_validated_forward; default off");
}
 ReleaseSRWLockExclusive(&lifetime);
}
extern "C" void dg_draw_trial_xr_ready(ID3D11Device*d,ID3D11DeviceContext*c){
 LifeLock life;
 if(!d&&!c){InterlockedExchangePointer(&lateXrContext,nullptr);return;}
 if(installed&&!stopping&&d==device.Get()&&c==context.Get()&&multi&&multi->GetMultithreadProtected()){
  InterlockedExchangePointer(&lateXrContext,c);log("late XR protection acknowledged; explicit request required");
 }
}
extern "C" void dg_draw_trial_poll(const char*marker){
flushQueries();
 if(marker)forwardReadMarker("");   /* vr_se_fwd / vr_se_fwd_all, before the poll */
 pollForward();
 if(marker){
  LONG dirWant=1;FILE*g=nullptr;
  if(!fopen_s(&g,marker,"rb")&&g){char db[8192];size_t dn=fread(db,1,sizeof db-1,g);fclose(g);db[dn]=0;
   for(char*q=db;(q=strstr(q,"vr_se_direct="))!=nullptr;q+=13)
    if(q==db||q[-1]==' '||q[-1]=='\t'||q[-1]=='\n'||q[-1]=='\r')dirWant=(!strncmp(q+13,"off",3)&&(q[16]==0||q[16]==' '||q[16]=='\t'||q[16]=='\r'||q[16]=='\n'))?0:1;}
  LONG dirWas=InterlockedExchange(&seDirect,dirWant);
  if(dirWas!=dirWant&&logger)logger("second_eye: direct slot-12 route %s\r\n",dirWant?"on (any undetoured d3d11 entry)":"OFF (vr_se_direct=off: only the fingerprinted initialized entry, as before v16)");
 }
 if(marker){
  FILE*f=nullptr;LONG want=0;
  if(0){char buf[1];buf[0]=0;
   for(char*q=buf;(q=strstr(q,"vr_se_mode="))!=nullptr;q+=11){
    if(q==buf||q[-1]==' '||q[-1]=='\t'||q[-1]=='\n'||q[-1]=='\r'){long v=strtol(q+11,nullptr,10);if(v>=0&&v<=1023)want=(LONG)v;}}}
  LONG was=InterlockedExchange(&seMode,want);
  {LONG nm=0;FILE*g=nullptr;
   if(0){char nb[1];nb[0]=0;
    for(char*q=nb;(q=strstr(q,"vr_se_near="))!=nullptr;q+=11)
     if(q==nb||q[-1]==' '||q[-1]=='\t'||q[-1]=='\n'||q[-1]=='\r'){long v=strtol(q+11,nullptr,10);if(v>=0&&v<=1000)nm=(LONG)v;}}
   LONG wasN=InterlockedExchange(&seNearMm,nm);
   if(wasN!=nm&&logger)logger("second_eye: replay near plane %ld mm -> %ld mm (z scale %.2f; 0 = the game's own ~50 mm)\r\n",wasN,nm,(double)dgReplayNearScale(nm));}
  dropProbe.enabled=(want&16)!=0;
  if(was!=want&&logger)logger("second_eye: bisect mode %ld ->%ld:%s%s%s%s%s%s%s%s%s\r\n",was,want,want?"":" normal",
   (want&(1|4|16))?" world-immediate":"",(want&2)?" no-post":"",(want&4)?" gpu-bank":"",(want&8)?" flush-each-record":"",(want&16)?" drop-probe":"",
   (want&32)?" no-final-copy":"",(want&64)?" no-hdfix-colour":"",(want&128)?" SAME-CAMERA (parity test)":"");
  if(was!=want&&logger&&(want&256))logger("second_eye: bisect mode bit 256: own-depth effect banks emptied again (old behaviour)\r\n");
 }
 if(marker&&!secondEyeDir[0]){char t[MAX_PATH];if(!strcpy_s(t,marker)){char*sl=strrchr(t,'\\');if(sl)sl[1]=0;else t[0]=0;strcat_s(t,"logs\\");strcpy_s(secondEyeDir,t);}}
 if(marker){ // The constant-buffer capture has its own request word and is independent of the one-shot draw trial.
  char cp[MAX_PATH],cbuf[64];size_t cn=0;
  if(!strcpy_s(cp,marker)){auto cl=strrchr(cp,'\\');bool named=cl?!strcpy_s(cl+1,sizeof(cp)-(cl+1-cp),"dg_draw_trial.on"):!strcpy_s(cp,"dg_draw_trial.on");
   FILE*cf=nullptr;if(named&&!fopen_s(&cf,cp,"rb")&&cf){cn=fread(cbuf,1,sizeof cbuf,cf);fclose(cf);}
   cbPoll(marker,cbuf,cn);
   pollMgDraw(plainSurveyWord(cbuf,cn)||plainImmediateWord(cbuf,cn));
  }
 }
 if(!marker||InterlockedCompareExchange(&requested,0,0))return;
 char path[MAX_PATH],buf[32];if(strcpy_s(path,marker))return;auto leaf=strrchr(path,'\\');if(leaf){if(strcpy_s(leaf+1,sizeof(path)-(leaf+1-path),"dg_draw_trial.on"))return;}else strcpy_s(path,"dg_draw_trial.on");
 FILE*f=nullptr;if(fopen_s(&f,path,"rb")||!f)return;size_t n=fread(buf,1,sizeof buf,f);fclose(f);
 const bool geometryWord=n==22&&!memcmp(buf,"second_camera_geometry",22);
 const bool frameWord=n==19&&!memcmp(buf,"second_camera_frame",19);
 const bool cameraRequest=(n==18&&!memcmp(buf,"second_camera_draw",18))||geometryWord;
 const bool exactRequest=(n==17&&!memcmp(buf,"late_indexed_draw",17))||cameraRequest||frameWord;
 const bool immediateRequest=plainImmediateWord(buf,n);
 const bool plainRequest=plainSurveyWord(buf,n)||immediateRequest;
 if(exactRequest
 ||plainRequest
 ){LifeLock life;
 bool prerequisites=installed && InterlockedCompareExchangePointer(&lateXrContext,nullptr,nullptr)==context.Get() && multi && multi->GetMultithreadProtected() && (plainRequest?mgDrawApi(mgDrawHook.next):(forwardStatus()==FORWARD_BOUND || forwardStatus()==FORWARD_DIRECT || ((forwardStatus()==FORWARD_NONE||forwardStatus()==FORWARD_WAITING||forwardStatus()==FORWARD_REJECTED)&&InterlockedCompareExchange(&seDirect,0,0))));
 LONG epochBefore=InterlockedCompareExchange(&ui2dSceneEpoch,0,0);
 LONG64 sample=InterlockedCompareExchange64(&ui2dSceneSample,0,0);
 LONG epochAfter=InterlockedCompareExchange(&ui2dSceneEpoch,0,0);
 AcquireSRWLockExclusive(&lateReadyLock);
 bool stable=prerequisites && epochBefore==epochAfter && (sample&1) && lateReady.ready(GetTickCount(),(DWORD)epochAfter);
 ReleaseSRWLockExclusive(&lateReadyLock);
 if(!stable)return;
 secondCameraRequest=cameraRequest;geometryRequest=geometryWord;frameRequest=frameWord;
 if(frameWord){
  frameRun.phase=FrameRun::WAIT_PRESENT;
  strcpy_s(frameRun.dir,path);char*slash=strrchr(frameRun.dir,'\\');if(slash)slash[1]=0;else frameRun.dir[0]=0;
  strcat_s(frameRun.dir,"logs\\");
  log("frame request armed; opens at the next Present; world draws of 15 known shaders (main targets) replayed twice each; intervals with < 64 replays discarded, max 30 intervals, max 8192 extra draws in total; no retry after a refusal");
 }
 if(geometryWord)log("second_camera geometry request armed; current indexed world draws; max_extra_draws=8 one_per_frame distinct_draws window_ms=10000; no retry after a refusal");
 else if(cameraRequest)log("second_camera request armed; current indexed world draw; max_extra_draws=1; no automatic retry");
 latePlainDraw=plainRequest;
 latePlainImmediate=immediateRequest;
 if(plainRequest){
  if(immediateRequest)lateSearchStarted=GetTickCount();
  InterlockedCompareExchange(&requested,1,0);
  log(immediateRequest?"plain immediate request armed; owned original only; max_extra_draws=1 contribution_requires_final_readback":"plain draw request armed; owned original only");return;
 }
 if(logger)logger("draw_trial: stable_gameplay_ready duration_ms=2000 min_samples=8 max_gap_ms=500; selection window starts now\r\n");
 if(!installed){InterlockedExchange(&requested,2);log("refused=no_api_owner extra_draws=0");}else if(forwardStatus()==FORWARD_WAITING&&!InterlockedCompareExchange(&seDirect,0,0)){log("request deferred: forward binding pending");}
 else if(forwardStatus()==FORWARD_REJECTED&&!InterlockedCompareExchange(&seDirect,0,0)){InterlockedExchange(&requested,2);log("refused=forward_binding_unavailable extra_draws=0");}
 else InterlockedCompareExchange(&requested,1,0);}

}
extern "C" void dg_draw_trial_present(){
 ++directFrame;
LifeLock life;if(!installed||stopping)return;ammoOnPresent();MultiLock lock;if(!ownerThread)ownerThread=GetCurrentThreadId();
 bpOnPresent();cbOnPresent();ui2dOnPresent();radarOnPresent();
 const unsigned requestLimit=300;
 if(InterlockedCompareExchange(&requested,0,0)==1&&++requestPresents>requestLimit){InterlockedExchange(&requested,2);
 reportLateMatch();clearLateMatch();lateSkipReport();
 if(logger)logger("draw_trial: refused=late_selection_timeout last=%s skipped=%u extra_draws=0\r\n",lateLastSkip,lateSkipped);
 geoFinish("selection_timeout");
 if(frameRequest&&frameRun.phase==FrameRun::OPEN&&!frameRun.cap){frameRun.phase=FrameRun::DONE;
  if(logger)logger("frame: verdict=INSUFFICIENT stop=selection_timeout replayed=0 extra_draws=0 other_shader=%u selection_skipped=%u no_camera=%u guard_skipped=%u (first %s) forward_status=%ld copied=%u records_bound=%u\r\n",
   frameRun.otherShader,frameRun.selection,frameRun.noCamera,frameRun.guardSkipped,frameRun.firstGuard?frameRun.firstGuard:"none",forwardStatus(),forwardFirstCopied(),boundCount());}
 }
 if(frameRequest)framePresent();
 if(dg_second_eye_enabled())secondEyePresent();
 if(!pending)return;if(ownerThread!=GetCurrentThreadId()||++polls>120||FAILED(device->GetDeviceRemovedReason())){log("readback timeout/thread-change/device-loss; no retry");delete pending;pending=nullptr;
 lateSurvey=false;clearLateMatch();geoFinish("readback_timeout");
 return;}
 /* Geometry: one probe dispatch per attempt on its own snapshots, read without waiting. */
 if(geometryRequest&&!lateSurvey&&!geo.read){
  auto&v=pending->depth.views;
  if(!geo.dispatched){
   if(!geo.probe.init(device.Get())||!geo.probe.dispatch(context.Get(),v[MSAA_PAIR_CONTRIBUTION*4].Get(),v[MSAA_PAIR_REPLAY*4].Get(),v[MSAA_PAIR_REPLAY*4+2].Get(),
      pending->depth.desc.Width,pending->depth.desc.Height,geo.sinv,geo.m,geo.camera,geo.minv)){log("geometry: probe_dispatch_failed");delete pending;pending=nullptr;geoFinish("probe_dispatch_failed");return;}
   geo.dispatched=true;return;
  }
  int g=geo.probe.read(context.Get(),geo.raw);if(g==0)return;
  if(g<0){log("geometry: probe_read_failed");delete pending;pending=nullptr;geoFinish("probe_read_failed");return;}
  geo.read=true;
 }
 if(lateSurvey){
  unsigned cp=0,dp=0;bool cc=false,dc=false;int a=pending->color.contribution(context.Get(),cc,&cp),b=pending->hasDepth?pending->depth.contribution(context.Get(),dc,&dp):1;
  if(a==0||b==0)return;
  bool valid=a==1&&b==1;
  if(logger)logger("draw_trial: survey candidate=%u valid=%d color_changed=%d depth_changed=%d extra_draws=0\r\n",lateCandidates,valid,cc,dc);
  if(logger)logger("draw_trial: survey_pixels color=%u depth=%u minimum=%u sufficient=%d\r\n",cp,dp,minTrialPixels,valid&&sufficientTrialPixels(cp,dp));
  delete pending;pending=nullptr;lateSurvey=false;
  if(!valid){clearLateMatch();log("refused=survey_readback_failed extra_draws=0");return;}
  lateMatch=sufficientTrialPixels(cp,dp);
  if(lateMatch)lateMatchStarted=GetTickCount();
  // Exclude a proven tiny contributor for the remainder of this bounded search.
  // Zero draws may become visible later (occlusion), so those remain eligible.
  if(!lateMatch&&(cc||dc)&&smallDrawCount<64)smallDraws[smallDrawCount++]={lateApi,lateCount,lateFirst,lateBase};
  if(!lateMatch)clearLateMatch();
  InterlockedExchange(&requested,1);return;
 }
 unsigned long long cm=0,cc=0,dm=0,dc=0;int a=pending->color.compare(context.Get(),cm,cc),b=pending->hasDepth?pending->depth.compare(context.Get(),dm,dc):1;if(a==0||b==0)return;
 unsigned long long sourceColor=0,sourceDepth=0;int sc=pending->color.compareSource(context.Get(),sourceColor),sd=pending->hasDepth?pending->depth.compareSource(context.Get(),sourceDepth):1;
 if(sc==0||sd==0)return;
 unsigned cp=0,dp=0;bool colorChanged=false,depthChanged=false;
 int ca=pending->color.contribution(context.Get(),colorChanged,&cp),da=pending->hasDepth?pending->depth.contribution(context.Get(),depthChanged,&dp):1;
 if(ca==0||da==0)return;
 if(logger)logger("draw_trial: final_pixels valid=%d color=%u depth=%u minimum=%u sufficient=%d\r\n",ca==1&&da==1,cp,dp,minTrialPixels,ca==1&&da==1&&sufficientTrialPixels(cp,dp));
 if(secondCameraRequest&&logger)logger("second_camera: output_changed=%d native_contribution=%d source_targets_unchanged=%d bindings_restored=%d full_stereo=UNPROVEN timing=PERTURBED\r\n",a==1&&b==1&&(cm||dm),ca==1&&da==1&&sufficientTrialPixels(cp,dp),sc==1&&sd==1&&!sourceColor&&!sourceDepth,pending->pipelineRestored&&pending->omRestored);
 if(latePlainImmediate&&logger)logger("draw_trial: direct_result accepted=%d extra_draws=1 no_retry=1 scope=current_draw_measured_targets_and_getter_state\r\n",a==1&&b==1&&sc==1&&sd==1&&ca==1&&da==1&&!cm&&!dm&&!sourceColor&&!sourceDepth&&pending->pipelineRestored&&pending->omRestored&&sufficientTrialPixels(cp,dp));
 if(pending->color.msaa&&logger)logger("draw_trial: msaa samples=%u compare=gpu_per_sample_bit_exact units: diff/changed=samples, contribution=pixels\r\n",pending->color.desc.SampleDesc.Count);
 if(logger)logger("draw_trial: pipeline getter_bindings_equal=%d scope=D3D11_bindings_and_CB_ranges excludes=resource_contents_query_progress_SO_offsets_UAV_counters\r\n",pending->pipelineRestored);
 if(logger)logger("draw_trial: integrity readback_valid=%d original_color_diff_bytes=%llu original_depth_diff_bytes=%llu om_restored=%d color_format=%u depth_format=%u width=%u height=%u scope=original_targets_and_OM_only\r\n",sc==1&&sd==1,sourceColor,sourceDepth,pending->omRestored,(unsigned)pending->color.desc.Format,(unsigned)pending->depth.desc.Format,pending->color.desc.Width,pending->color.desc.Height);
 if(logger)logger("draw_trial: readback valid=%d color_diff_bytes=%llu depth_diff_bytes=%llu normal_changed_color=%llu normal_changed_depth=%llu nontrivial=%d\r\n",a==1&&b==1,cm,dm,cc,dc,cc+dc>0);
 if(geometryRequest&&geo.read&&!geo.finished){
  bool intact=a==1&&b==1&&sc==1&&sd==1&&!sourceColor&&!sourceDepth&&pending->pipelineRestored&&pending->omRestored;
  delete pending;pending=nullptr;
  if(!intact){geoFinish("integrity_failed");return;}
  geo.totals.add(geo.raw);
  unsigned long long in[GEO_BINS];for(int k=0;k<GEO_BINS;k++)in[k]=geo.raw[k*GEO_STRIDE+1];
  {GeoRaw raw(geo.raw);GeoPrediction a;
   if(logger){logger("geometry: attempt=%u native_written_samples=%u depth=%.9g..%.9g\r\n",geo.attempts,raw.samples,raw.minDepth,raw.maxDepth);
    for(UINT i=0;i<GEO_SAMPLES&&i<raw.samples;i++){auto&q=raw.sample[i];
     logger("geometry: sample%u pixel=(%u,%u) depth=%.9g n*Sinv=(%.6g,%.6g,%.6g,%.6g) n*M=(%.6g,%.6g,%.6g,%.6g)\r\n",i,q.x,q.y,q.depth,q.world[0],q.world[1],q.world[2],q.world[3],q.clip[0],q.clip[1],q.clip[2],q.clip[3]);}}
   if(raw.samples&&geoPredict(geo.sinv,geo.m,geo.camera,geo.width,geo.height,raw.minDepth,raw.maxDepth,a)&&logger)
    logger("geometry: attempt=%u predict depth=%.6g/%.6g/%.6g distance=%.0f/%.0f/%.0f dx=%+.2f/%+.2f/%+.2f dx_range_first=%+.2f..%+.2f max_abs_dy=%.3f/%.3f/%.3f fit a=%+.3f b=%+.1f residual=%.1e\r\n",
     geo.attempts,a.depth[0],a.depth[1],a.depth[2],a.distance[0],a.distance[1],a.distance[2],a.dxMean[0],a.dxMean[1],a.dxMean[2],a.dxMin[0],a.dxMax[0],a.dyMaxAbs[0],a.dyMaxAbs[1],a.dyMaxAbs[2],a.a,a.b,a.fitResidual);
   else if(logger)logger("geometry: attempt=%u prediction=unavailable samples=%u\r\n",geo.attempts,raw.samples);
   if(logger&&raw.predictedPixels){
    logger("geometry: attempt=%u centroid native=(%.1f,%.1f) predicted=(%.1f,%.1f) replay=(%.1f,%.1f) replay_pixels=%u predicted_pixels=%u shift predicted=(%+.1f,%+.1f) actual=(%+.1f,%+.1f) difference=(%+.1f,%+.1f)\r\n",
     geo.attempts,raw.nativeX,raw.nativeY,raw.predictedX,raw.predictedY,raw.replayX,raw.replayY,raw.replayPixels,raw.predictedPixels,
     raw.predictedX-raw.nativeX,raw.predictedY-raw.nativeY,raw.replayX-raw.nativeX,raw.replayY-raw.nativeY,raw.replayX-raw.predictedX,raw.replayY-raw.predictedY);
    for(int k=0;k<GEO_BINS;k++){const UINT*r=geo.raw+k*GEO_STRIDE;if(r[0])
     logger("geometry: attempt=%u bin=%d pixels=%u inliers=%u outside=%u unwritten=%u flat=%u nonplanar=%u residual_beyond_window=%u\r\n",geo.attempts,k,r[0],r[1],r[2],r[3],r[4],r[31],r[32]);}}
   else if(logger)logger("geometry: attempt=%u centroid replay=(%.1f,%.1f) replay_pixels=%u predicted_pixels=0\r\n",geo.attempts,raw.replayX,raw.replayY,raw.replayPixels);}
  if(logger)logger("geometry: attempt=%u measured pixels near/mid/far=%u/%u/%u inliers=%llu/%llu/%llu cumulative_inliers=%llu/%llu/%llu\r\n",geo.attempts,
   geo.raw[0],geo.raw[GEO_STRIDE],geo.raw[2*GEO_STRIDE],in[0],in[1],in[2],geo.totals.bin[0].inliers,geo.totals.bin[1].inliers,geo.totals.bin[2].inliers);
  if(geoJudge(geo.totals).complete){geoFinish("all_bins_measured");return;}
  if(geo.attempts>=geoMaxAttempts){geoFinish("attempt_limit");return;}
  if(GetTickCount()-geo.started>=geoWindowMs){geoFinish("window_10s");return;}
  log("geometry: next attempt armed; new selection, distinct draw");
  InterlockedExchange(&requested,1);return;
 }
 delete pending;pending=nullptr;

}

extern "C" void dg_draw_trial_after_present(){
 LifeLock life;if(!installed||stopping||!context||!ownerThread||GetCurrentThreadId()!=ownerThread)return;

 if(!dg_second_eye_enabled()){secondEye.sceneClearHeldValid=false;return;}
 MultiLock lock;InterlockedIncrement(&afterPresentCalls);
 if(cbShadowArmed())return;
 InterlockedIncrement(&afterPresentTaken);cbShadowEnsure(false);
 afterPresentXrSeq=dg_xr_ctx_work_seq();afterPresentXrValid=true;
 if(cbShadowArmed())InterlockedIncrement(&afterPresentRearmed);
}
extern "C" void dg_draw_trial_stop(){InterlockedExchange(&stopping,1);AcquireSRWLockExclusive(&lifetime);bool owned=sitesOwned();
 // Code sites remain forwarding-only until process exit. Unlike pointer slots,
 // ten instruction bytes cannot safely be restored amid concurrent execution.
 magwellGpu=DgMagwellGpu{};ammoReset();bpStop();wwRemove();restoreMgDraw();restoreForward();InterlockedExchange(&forwardModuleState,FM_STOPPED);forwardRecompute();ui2dRemove();
 cbShadowRemove();secondEye.batch.clear();
 for(auto&h:hooks)if(h.slot&&*h.slot==h.ours)writeSlot(h,h.ours,h.next);
 postProbeDiscard();historyProbe.release();historyProbe.arm.finish();
 installed=false;
 delete pending;pending=nullptr;
 clearLateMatch();lateSurvey=false;
 InterlockedExchangePointer(&queryOwner,nullptr);InterlockedExchangePointer(&modernQueryOwner,nullptr);
 // Preserve the host serialization setting during shutdown too. Changing it rewrites
 // runtime dispatch while a normal, forwarding-only draw may still be active.
 multi.Reset();context.Reset();modernDevice.Reset();device.Reset();
 // Retain tiny executable stubs and pin this module: a caller may have fetched
 // a stub before restoration. Its C callback now forwards without a trial.
 ReleaseSRWLockExclusive(&lifetime);
}
// Camera-seam publisher for the constant-buffer capture. Called from the VEH
// camera hook: interlocked counters and four 64-byte copies, nothing else.
extern "C" void dg_cb_probe_camera(int eye,const float*eye_pers,const float*pers,const float*eye_inv,const float*eye_world){

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

extern "C" void dg_ui2d_eye(int eye){InterlockedExchange(&ui2dEyeTick,(LONG)GetTickCount());InterlockedExchange(&ui2dEyeNow,eye==1?2:eye==0?1:0);}
// Present thread only: the eye the finished frame's OBJECT shaders were drawn with (c20 votes).
extern "C" int dg_ui2d_last_frame_obj(long*pos,long*neg){if(pos)*pos=ui2dLastObjPos;if(neg)*neg=ui2dLastObjNeg;return (int)ui2dLastObjSign;}
// Present thread only: native Draw/DrawIndexed calls the finished frame issued.
extern "C" long dg_ui2d_last_frame_draws(void){return ui2dLast.draws;}
// Present thread: the back buffer's identity, and what the finished frame drew into it

extern "C" void dg_ui2d_backbuffer_ptr(void*p){ui2dBackbufferPtr=p;
if(!p){ui2dFrameSource.Reset();ui2dLastSource.Reset();}
}
extern "C" void *dg_ui2d_measure_source(void){
return ui2dLastBbDraws==1 && !ui2dLastBbIndexed ? ui2dLastSource.Get() : nullptr;
}
extern "C" long dg_ui2d_last_frame_bb(long*indexed,void**srv){if(indexed)*indexed=ui2dLastBbIndexed;if(srv)*srv=ui2dLastBbSrv;return ui2dLastBbDraws;}
// Arms the final-buffer trace for the next `frames` frames; a and b are the two final textures (identity only).
extern "C" void dg_ui2d_trace(int frames,void*a,void*b){
ui2dEnsureCopyHooks();ui2dTraceTarget[0]=a;ui2dTraceTarget[1]=b;ui2dQuadCount=0;InterlockedExchange(&ui2dCopyCount,0);ui2dTraceCount=0;ui2dTraceOverflow=0;ui2dTraceFrameNo=0;InterlockedExchange(&ui2dTraceLeft,frames<0?0:frames>16?16:frames);
}
// Present thread: 1 while alternate-eye stereo gameplay is being captured and vr_feedback_skip is on.
extern "C" void dg_ui2d_feedback(int on){InterlockedExchange(&ui2dFeedbackOn,on?1:0);}
extern "C" void dg_ui2d_feedback_stats(long*skipped,long*checked){if(skipped)*skipped=ui2dFeedbackSkipped;if(checked)*checked=ui2dFeedbackChecked;}
extern "C" void dg_ui2d_hold(int mode){InterlockedExchange(&ui2dCfg.hold,mode<0?0:mode>2?2:mode);}
extern "C" void dg_ui2d_gameplay(int allowed){
 if(!allowed)InterlockedIncrement(&ui2dSceneEpoch);
 InterlockedExchange64(&ui2dSceneSample,((LONG64)GetTickCount()<<1)|(allowed?1:0));
 DWORD now=GetTickCount();
 AcquireSRWLockExclusive(&lateReadyLock);
 lateReady.step(now,now,(DWORD)InterlockedCompareExchange(&ui2dSceneEpoch,0,0),allowed!=0);
 ReleaseSRWLockExclusive(&lateReadyLock);
}
extern "C" void dg_ammo_gameplay(int allowed){
 InterlockedExchange64(&ammoSceneSample,((LONG64)GetTickCount()<<1)|(allowed?1:0));
}
extern "C" int dg_ui2d_last_frame_sign(void){return (int)InterlockedCompareExchange(&ui2dLastFrameSign,0,0);}

extern "C" void dg_ui2d_display(double left,double right){
 double l=tan(left),r=tan(right),d=r-l;if(!(d>1e-6))return;double m=fabs((r+l)/d);
 if(m>=0.02&&m<=0.6)InterlockedExchange(&ui2dDisplayMagE6,(LONG)(m*1000000.0+0.5));
}
extern "C" void dg_ui2d_stats(char*out,size_t n){
 char cand[400]="";size_t at=0;
 for(auto&c:ui2dCand)if(c.hash&&at+40<sizeof cand)at+=sprintf_s(cand+at,sizeof cand-at," %016llX:%ld",(unsigned long long)c.hash,c.count);
 sprintf_s(out,n,"ui2d: on %ld gameplay %d blocked %ld effect %ld site %s vs-known %u  uploads %ld persp %ld x0 %+.4f display %.4f votes +%ld/-%ld  sig-draws %ld patched %ld restored %ld errors %ld  skip kind %ld stale %ld viewport %ld depth %ld vs %ld  bb %ux%u  scale %.2f conv %.4f sign %+ld list %u  hold %ld frames voted %ld zero %ld few %ld  sprite-frames held %ld BARE %ld early %ld  held-draws %ld  eye-src L +%ld/-%ld R +%ld/-%ld agree %ld DISAGREE %ld used %ld  draws/frame voted %.0f no-camera %.0f (min %ld max %ld)  candidates%s",
  ui2dCfg.on,ui2dSceneAllowed()?1:0,ui2dSkipScene,ui2dSkipEffect,ui2dSiteInstalled?"ok":"MISSING",vsKnown,ui2dUploads,ui2dPersp,ui2dHaveX0?ui2dX0:0.0f,ui2dDisplayMagE6/1000000.0,ui2dVotesPos,ui2dVotesNeg,ui2dSigDraws,ui2dPatched,ui2dRestored,ui2dErrors,
  ui2dSkipKind,ui2dSkipStale,ui2dSkipViewport,ui2dSkipDepth,ui2dSkipVs,ui2dBbW,ui2dBbH,ui2dCfg.scaleMils/1000.0,ui2dCfg.convE5/100000.0,ui2dCfg.sign,ui2dCfg.vsCount,ui2dCfg.hold,ui2dFramesVoted,ui2dFramesZero,ui2dFramesFew,ui2dFramesHeld,ui2dFramesBare,ui2dFramesEarly,ui2dHeldDraws,ui2dEyeN[0][0],ui2dEyeN[0][1],ui2dEyeN[1][0],ui2dEyeN[1][1],ui2dEyeAgree,ui2dEyeDisagree,ui2dEyeUsed,ui2dFramesVoted?(double)ui2dDrawsVoted/ui2dFramesVoted:0.0,(ui2dFramesZero+ui2dFramesFew)?(double)ui2dDrawsNoCam/(ui2dFramesZero+ui2dFramesFew):0.0,ui2dNoCamMin==0x7fffffff?0L:(long)ui2dNoCamMin,(long)ui2dNoCamMax,cand);
 {size_t at=strlen(out);if(at+1<n)_snprintf_s(out+at,n-at,_TRUNCATE,"  scene-copies learned %ld skipped %ld",ui2dSceneCopyLearned,ui2dSkipCopy);}
}
// Wrist radar, draw-hook half: the switches and the heartbeat's counters.
extern "C" void dg_radar_configure(int on,int hud_off){InterlockedExchange(&radarHudOff,hud_off?1:0);InterlockedExchange(&radarOn,on?1:0);}
extern "C" void dg_radar_view(int first_person){InterlockedExchange(&radarFirstPerson,first_person?1:0);}
extern "C" void dg_radar_stats(char*out,size_t n){
 _snprintf_s(out,n,_TRUNCATE,"composite draws %ld (extra in frame %ld, frames with more than one %ld)  NOT FORWARDED %ld  wrist-up %ld refused %ld  last viewport %ux%u at (%u,%u) texture %ux%u fmt %u  rejected: origin %ld bigger %ld depth-on %ld no-tex2d %ld size %ld format %ld not-plain %ld no-viewport %ld",
  radarHits,radarExtra,radarFramesMulti,radarSkipped,radarAvailable,radarRefused,radarLastVp[2],radarLastVp[3],radarLastVp[0],radarLastVp[1],radarLastTex[0],radarLastTex[1],radarLastTex[2],
  radarWhy[3],radarWhy[4],radarWhy[5],radarWhy[6],radarWhy[7],radarWhy[8],radarWhy[9],radarWhy[2]);
}

extern "C" void dg_blit_boundary_control(const char*dir,int active,unsigned mark,unsigned present,int eye){LifeLock life;if(!stopping)bpControl(dir,active,mark,present,eye);}

/* Called from the Present path right after the native eye's capture (same
   thread, same context), with the slot the native image did not take. B was
   built with the other camera of its pair, which the accepted slot mapping
   stores in the slot of the pair's source eye. Returns 1 when B was stored -
   the Present path holds the native eye still only while that keeps happening. */
extern "C" void dg_psg_dump_pair(unsigned id, unsigned frame, unsigned age, unsigned phase)
{
 if(!secondEyeDir[0] || !context || !device || ownerThread!=GetCurrentThreadId()) return;
 void *stores[2]{};
 int saved[2]{};
 dg_xr_snapshot_stores(stores);
 for(unsigned eye=0;eye<2;++eye){
  if(!stores[eye]) continue;
  char path[MAX_PATH];
  sprintf_s(path,"%spsg_%lu_%u_%u_%s.bmp",secondEyeDir,GetCurrentProcessId(),id,phase,eye?"R":"L");
  saved[eye]=dgDumpBmp(device.Get(),context.Get(),(ID3D11Texture2D*)stores[eye],DXGI_FORMAT_B8G8R8A8_UNORM,path);
  ((IUnknown*)stores[eye])->Release();
 }
 if(logger) logger("PSG sight images: pid=%lu id=%u phase=%u present=%u age_ms=%u L=%d R=%d\r\n",
                   GetCurrentProcessId(),id,phase,frame,age,saved[0],saved[1]);
}
extern "C" int dg_second_eye_publish(int slot,const void*raw,const void*fov){
 auto&e=secondEye;
 if(!secondEyeLive()||e.state==SecondEyeLive::IDLE||!e.b.color){++e.empty;
 historyProbe.publish(0,slot);
 return 0;}

 LifeLock life;if(!installed||stopping)return 0;
 if(context&&ownerThread==GetCurrentThreadId())e.flushBatch(context.Get(),FL_PRESENT);   /* B complete before it is stored */

 if(slot!=e.sourceEye)++e.eyeMismatch;
 bool post=!(InterlockedCompareExchange(&seMode,0,0)&2)&&e.postUsed&&!e.postHalfFailed&&e.ppCur>=0&&e.pp[e.ppCur].b3;
 ID3D11Texture2D*b3=post?e.pp[e.ppCur].b3.Get():nullptr;
 bool finOk=post&&e.fin.colour&&e.fin.written==e.frames&&!(InterlockedCompareExchange(&seMode,0,0)&32);   /* the game's final colour step, replayed */
 /* MGSHDFix colour correction (the native eye gets it at Present): on our private final image only - B3 is history. */
 if(finOk&&!(InterlockedCompareExchange(&seMode,0,0)&64)&&context&&ownerThread==GetCurrentThreadId()&&e.hdfix.on()){
  bool was=inside;inside=true;if(!e.hdfix.apply(device.Get(),context.Get(),e.fin.colour.Get(),e.fin.rt.Get()))++e.hdfixFailed;inside=was;}
 int ok=dg_xr_capture_texture(finOk?(void*)e.fin.colour.Get():post?(void*)b3:(void*)e.b.color.Get(),slot,raw,fov);
 if(ok&&post)++e.postPublished;if(ok&&finOk)++e.finPublished;

 if(ok&&post&&(e.postPublished==1800||e.postPublished==5400))e.evidencePending=true;
 if((e.evidenceManual||!nativeClearLightSession)&&e.evidencePending&&ok&&post&&secondEyeDir[0]&&(e.evidenceManual||e.evidenceShots<2)&&(!e.evidenceManual||e.finSnapBb||!e.fin.colour)&&context){   /* manual: wait for the final-draw snapshot (next frame) */
  e.evidencePending=false;e.evidenceManual=false;++e.evidenceShots;
  void*st[2]{};int n=dg_xr_snapshot_stores(st);unsigned long tick=GetTickCount();char pa[MAX_PATH];int okL=0,okR=0,okF=0,okB=0;
  sprintf_s(pa,"%sdg_second_eye_%lu_headset_left.bmp",secondEyeDir,tick);if(st[0])okL=dgDumpBmp(device.Get(),context.Get(),(ID3D11Texture2D*)st[0],DXGI_FORMAT_B8G8R8A8_UNORM,pa);
  sprintf_s(pa,"%sdg_second_eye_%lu_headset_right.bmp",secondEyeDir,tick);if(st[1])okR=dgDumpBmp(device.Get(),context.Get(),(ID3D11Texture2D*)st[1],DXGI_FORMAT_B8G8R8A8_UNORM,pa);
  sprintf_s(pa,"%sdg_second_eye_%lu_game_final.bmp",secondEyeDir,tick);if(ui2dBackbufferPtr)okF=dgDumpBmp(device.Get(),context.Get(),(ID3D11Texture2D*)ui2dBackbufferPtr,DXGI_FORMAT_B8G8R8A8_UNORM,pa);
  sprintf_s(pa,"%sdg_second_eye_%lu_B3.bmp",secondEyeDir,tick);okB=dgDumpBmp(device.Get(),context.Get(),b3,DXGI_FORMAT_B8G8R8A8_UNORM,pa);
  if(finOk){sprintf_s(pa,"%sdg_second_eye_%lu_B_final.bmp",secondEyeDir,tick);dgDumpBmp(device.Get(),context.Get(),e.fin.colour.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pa);}
  if(e.finSnapT3){sprintf_s(pa,"%sdg_second_eye_%lu_native_T3_at_final.bmp",secondEyeDir,tick);dgDumpBmp(device.Get(),context.Get(),e.finSnapT3.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pa);}
  if(e.finSnapBb){sprintf_s(pa,"%sdg_second_eye_%lu_native_bb_after_final.bmp",secondEyeDir,tick);dgDumpBmp(device.Get(),context.Get(),e.finSnapBb.Get(),DXGI_FORMAT_B8G8R8A8_UNORM,pa);}
  e.finSnapT3.Reset();e.finSnapBb.Reset();
  for(auto t:st)if(t)((IUnknown*)t)->Release();
  if(logger)logger("second_eye: evidence images %lu (stores %d): headset left=%d right=%d game final=%d B3=%d; B went to slot %d (0 = left), the render eye was %d\r\n",
   tick,n,okL,okR,okF,okB,slot,e.sourceEye);
 }
 if(ok)++e.published;else ++e.refused;
 historyProbe.publish(ok,slot);
 e.state=SecondEyeLive::IDLE;
 return ok;
}
