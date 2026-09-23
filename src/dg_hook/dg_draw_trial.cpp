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









bool forwardApi(void*api);
template<class F>void intercept(ID3D11DeviceContext*c,F next,void*api,const char*kind,bool forwarded=false){





























next();

}
#include "dg_backbuffer_watch.inl"
bool bpBefore(ID3D11DeviceContext*,UINT,UINT);void bpAfter(bool);
void bpTagRoute(bool,bool);
bool bpWanted();void bpForeign(void*,void*);
bool mgDrawApi(void*);
template<class F> void blitForward(ID3D11DeviceContext*c,UINT n,UINT first,void*api,F next,bool mgForward=false){









 next();

}
void ui2dTraceDraw(ID3D11DeviceContext*,UINT,int);void ui2dOnDraw(ID3D11DeviceContext*,unsigned);void ui2dOnPresent();bool ui2dSkipDraw(ID3D11DeviceContext*,UINT); // dg_ui2d.inl
bool radarOnDraw(ID3D11DeviceContext*,UINT);void radarOnPresent(); // dg_radar.inl
#include "dg_cb_probe.inl"
#include "dg_mgshd_forward.inl"
#include "dg_mgshd_draw.inl"
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
void STDMETHODCALLTYPE draw(ID3D11DeviceContext*c,UINT n,UINT first){if(ui2dSkipDraw(c,n)){ui2dTraceDraw(c,n,1);return;} // previous-frame feedback in stereo: not forwarded
 ui2dOnDraw(c,1);cbOnDraw(c,1,n,first,0);
 if(radarOnDraw(c,n)){ui2dTraceDraw(c,n,2);return;} // wrist radar: the composite's texture is captured; with vr_radar_hud=off the draw is not forwarded. No lock held across it.
 ui2dTraceDraw(c,n,0);
 void*api=(*(void***)c)[13];auto f=(Draw)api;intercept(c,[=](){blitForward(c,n,first,api,[=](){f(c,n,first);});},api,"Draw");}

// Geometry-only trial: timing queries may include the extra draw duration.
// Unknown descriptors/flags/context variants remain conservative refusals.



































void*nearMemory(BYTE*site){SYSTEM_INFO info;GetSystemInfo(&info);uintptr_t center=(uintptr_t)site&~(uintptr_t)(info.dwAllocationGranularity-1);
 for(uintptr_t step=info.dwAllocationGranularity;step<0x70000000;step+=info.dwAllocationGranularity){for(int sign:{-1,1}){uintptr_t a=sign<0?center-step:center+step;if(sign<0&&center<step)continue;void*p=VirtualAlloc((void*)a,4096,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);if(p)return p;}}return nullptr;}
#include "dg_ui2d.inl"




#include "dg_blit_boundary.inl"



                              void bpForeign(void*,void*){}

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
 // Preserve the host setting: enabling D3D protection causes stereo ghosting in headset A/B tests.
 oldProtected=multi->GetMultithreadProtected()!=FALSE;
 if(logger)logger("draw_trial: thread protection preserved=%d forced=0; extra trial draw requires existing protection\r\n",oldProtected);






 bool ok=true; hookCount=0;





 uint64_t base=(uint64_t)(ULONG_PTR)GetModuleHandleW(nullptr);

 if(ok)ok=prepareSites(base);
 if(ok){HMODULE pin=nullptr;ok=GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)&indexed,&pin)!=FALSE;}
 if(ok){for(unsigned i=0;i<hookCount;i++){auto&h=hooks[i];if(!writeSlot(h,h.next,h.ours)){ok=false;break;}}}
 if(ok)for(auto&s:sites)if(!codeWrite(s,s.original,s.patch)){ok=false;break;}
 if(!ok){restoreAll();log("attach refused: code/API ownership; no draw trial");}
 else{installed=true;ui2dInstall(d);beginForward();



 log("attached: gameplay HUD/radar only; diagnostic GPU hooks excluded");

}
 ReleaseSRWLockExclusive(&lifetime);
}
extern "C" void dg_draw_trial_poll(const char*marker){


















}
extern "C" void dg_draw_trial_present(){









LifeLock life; if(!installed || stopping)return; MultiLock lock; ui2dOnPresent(); radarOnPresent();

}
extern "C" void dg_draw_trial_stop(){InterlockedExchange(&stopping,1);AcquireSRWLockExclusive(&lifetime);bool owned=sitesOwned();
 // Code sites remain forwarding-only until process exit. Unlike pointer slots,
 // ten instruction bytes cannot safely be restored amid concurrent execution.
 bpStop();wwRemove();restoreMgDraw();restoreForward();setForwardStatus(FORWARD_STOPPED);ui2dRemove();
 for(auto&h:hooks)if(h.slot&&*h.slot==h.ours)writeSlot(h,h.ours,h.next);
 installed=false;



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
// Camera seam (VEH): the eye of the handoff it just published. Interlocked only.
extern "C" void dg_ui2d_eye(int eye){InterlockedExchange(&ui2dEyeTick,(LONG)GetTickCount());InterlockedExchange(&ui2dEyeNow,eye==1?2:eye==0?1:0);}
// Present thread only: the eye the finished frame's OBJECT shaders were drawn with (c20 votes).
extern "C" int dg_ui2d_last_frame_obj(long*pos,long*neg){if(pos)*pos=ui2dLastObjPos;if(neg)*neg=ui2dLastObjNeg;return (int)ui2dLastObjSign;}
// Present thread only: native Draw/DrawIndexed calls the finished frame issued.
extern "C" long dg_ui2d_last_frame_draws(void){return ui2dLast.draws;}
// Present thread: the back buffer's identity, and what the finished frame drew into it

extern "C" void dg_ui2d_backbuffer_ptr(void*p){ui2dBackbufferPtr=p;



}
extern "C" void *dg_ui2d_measure_source(void){



return nullptr;

}
extern "C" long dg_ui2d_last_frame_bb(long*indexed,void**srv){if(indexed)*indexed=ui2dLastBbIndexed;if(srv)*srv=ui2dLastBbSrv;return ui2dLastBbDraws;}
// Arms the final-buffer trace for the next `frames` frames; a and b are the two final textures (identity only).
extern "C" void dg_ui2d_trace(int frames,void*a,void*b){





}
// Present thread: 1 while alternate-eye stereo gameplay is being captured and vr_feedback_skip is on.
extern "C" void dg_ui2d_feedback(int on){InterlockedExchange(&ui2dFeedbackOn,on?1:0);}
extern "C" void dg_ui2d_feedback_stats(long*skipped,long*checked){if(skipped)*skipped=ui2dFeedbackSkipped;if(checked)*checked=ui2dFeedbackChecked;}
extern "C" void dg_ui2d_hold(int mode){InterlockedExchange(&ui2dCfg.hold,mode<0?0:mode>2?2:mode);}
extern "C" void dg_ui2d_gameplay(int allowed){
 if(!allowed)InterlockedIncrement(&ui2dSceneEpoch);
 InterlockedExchange64(&ui2dSceneSample,((LONG64)GetTickCount()<<1)|(allowed?1:0));
}
extern "C" int dg_ui2d_last_frame_sign(void){return (int)InterlockedCompareExchange(&ui2dLastFrameSign,0,0);}
// Camera seam (VEH): |NDC x of straight-ahead| of the frustum submitted to the headset.
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
