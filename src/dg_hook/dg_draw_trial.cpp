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
#include <d3d11_4.h>
#include <wrl/client.h>
using Microsoft::WRL::ComPtr;
#include "dg_trial_pipeline_state.h"
#include "dg_second_camera_buffer.h"
using Microsoft::WRL::ComPtr;
#include "dg_context_trace.h"
#include "dg_native_state_probe.h"
#include "dg_native_state_layout.h"
#include "dg_draw_observation.h"
#include "dg_magwell_gpu.h"






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
DWORD ownerThread=0;void*pending=nullptr;unsigned polls=0,requestPresents=0;



void(*logger)(const char*,...)=nullptr;
thread_local bool inside=false;
// Private bounded contour queries never enclose native/replayed draws. Use the
// captured original factory, not the public hook that tracks native scopes.





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
bool mgDrawApi(void*api);
bool initializedApi(ID3D11DeviceContext*c,void*api);

#include "dg_late_draw.h"
#include "dg_frame_capture.h"
#include "dg_vs_rdef.h"
void* volatile lateXrContext=nullptr;
volatile LONG lateForwardIndexed=0,lateForwardScopeMiss=0,lateForwardReturnMiss=0;
bool frameRequest=false;
bool ui2dSceneAllowed();
struct RenderShaderCache { UINT64 rdefHash[64]{};int rdefPers[64]{};unsigned rdefCount=0; } frameRun;
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
extern "C" int dg_xr_capture_texture(void*,int,const void*,const void*);
extern "C" int dg_xr_snapshot_stores(void*out[2]);
#pragma comment(lib, "user32.lib")      /* GetAsyncKeyState: Ctrl+F10 image dump */
extern void*ui2dBackbufferPtr;                /* dg_ui2d.inl: the swap chain's buffer, identity only */
/* Bisect switches (5 Oct, disappearing surfaces in the replayed eye): marker key vr_se_mode=<bits>,
   read live from dg_hook.on once a second (the main parser ignores unknown numeric keys). 0 = normal.
     1  world replay without batching (the immediate route: target switch per draw)
     2  no post pass for the second eye (the headset gets the bare world pass B)
     4  world bank through the GPU compute route instead of the CPU ring (implies 1)
     8  batching kept, but each record flushed at once (record/apply kept, no deferral)
    16  per-draw native/replay comparison (dg_drop_probe.h; implies 1)
   Each bit removes one part of the chain; which one makes the surfaces stay tells where they go. */
volatile LONG seMode=0;
volatile LONG seNearMm=0;             /* vr_se_near=<mm>: replay-eye near plane (dg_replay_near.h), 0 = off */
char secondEyeDir[MAX_PATH]{};       /* <game>\\logs\\, from the marker path */
#include "dg_second_camera_cpu.h"
#include "dg_draw_batch.h"
#include "dg_cb_shadow.inl"
void ui2dEnsureCopyHooks();
#include "dg_post_probe_api.h"



#define DG_POST_MAPPED(g,m) (m)




#include "dg_hdfix_colour.inl"
struct SecondEyeLive{
 DG_MAGWELL magwell{};float magwellTarget[16]{};bool magwellDone=false;
 enum State{IDLE,BUILDING,COMPLETE} state=IDLE;
 FramePass b;D3D11_TEXTURE2D_DESC colorDesc{},depthDesc{};D3D11_RENDER_TARGET_VIEW_DESC rtvDesc{};D3D11_DEPTH_STENCIL_VIEW_DESC dsvDesc{};
 ComPtr<ID3D11RenderTargetView>gameRT;ComPtr<ID3D11DepthStencilView>gameDS;
 SecondCameraGpu gpu;float delta[16]{},sinvF[16]{},clearDepth=0;int sourceEye=-1,sequence=0;
 /* Step 2: the CPU route (dg_second_camera_cpu.h) binds the bank from the shadow of the
    game's upload; the GPU route stays for draws without a shadow and, every
    cpuCheckEvery draws, as the live cross-check of the CPU bank (3 mismatches: CPU off). */
 SecondCameraCpu cpu;bool cpuOff=false;const char*cpuOffReason=nullptr,*cpuMissLast="none";
 unsigned long long cpuBinds=0,gpuBinds=0,cpuMiss=0,sinceCheck=0;static constexpr unsigned cpuCheckEvery=2048;
 ComPtr<ID3D11Buffer>checkStaging;UINT checkWidth=0,checkBegin=0,checkBytes=0,checkPers=0,checkWait=0;bool checkPending=false;float checkScale=0;
 std::vector<uint8_t>checkWant;unsigned checkOk=0,checkBad=0,checkDropped=0,checkSplitSkipped=0;
 /* Batched replay (dg_draw_batch.h): world draws are recorded and replayed into B in batches, one
    depth-target switch per batch instead of two per draw. Draws the batch cannot take (no shadow,
    a GPU-route sample) flush it and use the immediate route. */
 /* Guards, once per frame (live 27 Sep, 53606271: the game thread took 23.5 ms a frame with the GPU
    idle - sitesOwned/forwardApi/initializedApi read code bytes with ReadProcessMemory and resolve
    modules on EVERY world draw, ~800 a frame). Ownership of our hooks and the identity of the
    DrawIndexed entry do not change within a frame; they are checked on the first world draw of
    each frame and for each new entry address. Per draw stays only what is free: the context,
    and that the vtable slot still holds that same validated entry. */
 unsigned guardEpoch=0,guardSeen=~0u;bool guardSites=false;void*guardApi[4]{};bool guardApiOk[4]{};UINT guardApis=0;
 unsigned long long guardFull=0;LONGLONG pathQpc=0;unsigned long long rearmedPending=0;
 bool guardOk(ID3D11DeviceContext*c,void*api,bool initialized,const char**why){
  if(guardSeen!=guardEpoch){guardSeen=guardEpoch;guardApis=0;guardSites=sitesOwned();++guardFull;}
  if(!guardSites){*why="hook_ownership_changed";return false;}
  if(initialized&&(c!=context.Get()||(*(void***)c)[12]!=api)){*why="entry_changed";return false;}
  for(UINT i=0;i<guardApis;i++)if(guardApi[i]==api){if(!guardApiOk[i])*why="entry_changed";return guardApiOk[i];}
  bool ok=initialized?initializedApi(c,api):forwardApi(api);++guardFull;
  if(guardApis<4){guardApi[guardApis]=api;guardApiOk[guardApis++]=ok;}
  if(!ok)*why="entry_changed";return ok;
 }
 /* Stage A (27-29 Sep): the post pass for the second eye. Measured (222E335F): after the world pass the
    game draws into T3 (main-image size, single-sample) with depth T4: one full-screen draw that reads
    main colour + main depth (a shader resolve), then ~500 effect draws (smoke, fire, sparks, trails)
    whose banks carry the camera (gVS_Pers 20/24) and which also read main colour/depth. T3 goes to
    the back buffer. The second eye gets its own B3/B4: the same draws are recorded into them, main
    colour/depth swapped for B's own, the camera retargeted where the bank has one; the headset
    then gets B3 instead of the bare world pass. Live 29 Sep (1922BEBE): the game alternates between
    TWO final images (ping-pong), so there are two pairs, each learned once. */
 void*mainColorRes=nullptr,*mainDepthRes=nullptr;
 struct PostPair{void*rt=nullptr,*ds=nullptr;ID3D11RenderTargetView*grtv=nullptr;ID3D11DepthStencilView*gdsv=nullptr;   /* the game's (compared only) */
  ComPtr<ID3D11Texture2D>b3,b4;ComPtr<ID3D11RenderTargetView>b3rt;ComPtr<ID3D11DepthStencilView>b4ds;unsigned long long lastUse=0;};
 PostPair pp[2];int ppCur=-1;
 bool postUsed=false;const char*postRefusal="none";
 unsigned long long postRecorded=0,postRefused=0,postOther=0,postMain=0,postClears=0,postPublished=0,postLearned=0,postGs=0;
 struct Swap{ID3D11ShaderResourceView*game;void*res;ComPtr<ID3D11ShaderResourceView>mine;};Swap swaps[16];UINT swapN=0;
 unsigned plainEpoch=~0u;void*plainApi=nullptr;bool plainVal=false;
 float srcClip[16]{};
 struct ProjSample{UINT64 h;};ProjSample projSeen[8];UINT projN=0;
 bool evidencePending=false,evidenceManual=false;unsigned evidenceShots=0;   /* evidenceManual: Ctrl+F11 (10 Oct) */
 bool plainOk(void*api){        /* a non-indexed draw entry we may call ourselves: plain d3d11, checked once per frame */
  if(plainEpoch!=guardEpoch||plainApi!=api){plainEpoch=guardEpoch;plainApi=api;plainVal=api&&runtimePointer(api);}
  return plainVal;
 }
 int ppFind(void*R){for(int k=0;k<2;k++)if(pp[k].rt&&pp[k].rt==R)return k;return -1;}
 bool isPostView(ID3D11RenderTargetView*rt,ID3D11DepthStencilView*ds){for(auto&q:pp)if(q.grtv&&rt==q.grtv&&ds==q.gdsv)return true;return false;}
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
 int learnPost(ID3D11Device*d,ID3D11RenderTargetView*grt,ID3D11DepthStencilView*gds,void*R,void*D){
  int k=!pp[0].rt?0:!pp[1].rt?1:(pp[0].lastUse<=pp[1].lastUse?0:1);
  ComPtr<ID3D11Texture2D>t;if(FAILED(((ID3D11Resource*)R)->QueryInterface(IID_PPV_ARGS(&t)))){refusePost("post_not_texture");return -1;}
  D3D11_TEXTURE2D_DESC td{};t->GetDesc(&td);td.BindFlags|=D3D11_BIND_SHADER_RESOURCE|D3D11_BIND_RENDER_TARGET;td.CPUAccessFlags=0;td.MiscFlags=0;td.Usage=D3D11_USAGE_DEFAULT;
  PostPair n;D3D11_RENDER_TARGET_VIEW_DESC rd{};grt->GetDesc(&rd);
  if(FAILED(d->CreateTexture2D(&td,nullptr,&n.b3))||FAILED(d->CreateRenderTargetView(n.b3.Get(),&rd,&n.b3rt))){refusePost("post_allocation");return -1;}
  if(D){int o=1-k;
   if(pp[o].ds==D&&pp[o].b4){n.b4=pp[o].b4;n.b4ds=pp[o].b4ds;}          /* both final images share one depth buffer */
   else{ComPtr<ID3D11Texture2D>z;if(FAILED(((ID3D11Resource*)D)->QueryInterface(IID_PPV_ARGS(&z)))){refusePost("post_depth_not_texture");return -1;}
    D3D11_TEXTURE2D_DESC zd{};z->GetDesc(&zd);zd.BindFlags|=D3D11_BIND_DEPTH_STENCIL;zd.CPUAccessFlags=0;zd.MiscFlags=0;zd.Usage=D3D11_USAGE_DEFAULT;
    D3D11_DEPTH_STENCIL_VIEW_DESC dd{};gds->GetDesc(&dd);
    if(FAILED(d->CreateTexture2D(&zd,nullptr,&n.b4))||FAILED(d->CreateDepthStencilView(n.b4.Get(),&dd,&n.b4ds))){refusePost("post_depth_allocation");return -1;}}}
  n.rt=R;n.ds=D;n.grtv=grt;n.gdsv=gds;pp[k]=n;clearPostSwaps();++postLearned;
  if(logger)logger("second_eye: post pass pair %d learned: target %ux%u fmt %u depth %s (%s); the second eye gets its own copies\r\n",k,td.Width,td.Height,(unsigned)td.Format,D?"yes":"none",
   D&&pp[1-k].ds==D?"shared with the other pair":"own");
  return k;
 }
 /* The first projective banks of the post pass, with this frame's camera (what the game puts there instead). */

 /* A draw after the world pass into another target (the game's own draw has already run). */
 void postDraw(ID3D11Device*d,ID3D11DeviceContext*c,void*api,bool indexed,UINT n,UINT first,INT base){
  if(state!=BUILDING||batchOff||cpuOff||!b.color||(InterlockedCompareExchange(&seMode,0,0)&2)){++postOther;return;}
  ID3D11RenderTargetView*rts[2]{};ID3D11DepthStencilView*dsv=nullptr;c->OMGetRenderTargets(2,rts,&dsv);
  struct Rel{ID3D11RenderTargetView**r;ID3D11DepthStencilView*d;~Rel(){for(int k=0;k<2;k++)if(r[k])r[k]->Release();if(d)d->Release();}}rel{rts,dsv};
  if(!rts[0]||rts[1]){++postOther;return;}
  ID3D11Resource*rr=nullptr;rts[0]->GetResource(&rr);void*R=rr;if(rr)rr->Release();



   if(R==mainColorRes){++postMain;return;}                      /* before the world pass (history/background): stage B */
  void*D=nullptr;if(dsv){ID3D11Resource*dr=nullptr;dsv->GetResource(&dr);D=dr;if(dr)dr->Release();}
  int k=ppFind(R),ht=-1;bool fk=false;
  if(k<0&&finalEligible(c,R,D)){if(!learnFinal(d,rts[0],(ID3D11Resource*)R))return;fk=true;



  }
  else if(k<0){
   ComPtr<ID3D11Texture2D>t;if(!R||FAILED(((ID3D11Resource*)R)->QueryInterface(IID_PPV_ARGS(&t)))){++postOther;return;}
   D3D11_TEXTURE2D_DESC td{};t->GetDesc(&td);
   bool full=td.SampleDesc.Count==1&&td.Width==colorDesc.Width&&td.Height==colorDesc.Height&&R!=ui2dBackbufferPtr&&readsMain(c);
   if(full){if((k=learnPost(d,rts[0],dsv,R,D))<0)return;}
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
  if(!ctx1)c->QueryInterface(IID_PPV_ARGS(&ctx1));
  if(!ctx1){refusePost("no_context1");return;}
  DgRecordArgs a;a.pers=pers>=16&&pers<=60?pers:-1;if(indexed)a.indexed=(DgIndexedFn)api;else a.plain=(DgDrawFn)api;
  a.n=n;a.first=first;a.base=base;a.ownTarget=true;a.rt=ownRt;a.ds=ownDepth;a.swap=postSwap;a.swapCtx=this;



   int proj=0;const char*r=batch.recordEx(d,ctx1.Get(),cpu,delta,sinvF,cbShadowLookup,nullptr,a,&proj);
  if(r&&!strcmp(r,"ring_wrap")){flushBatch(c,FL_WRAP);r=batch.recordEx(d,ctx1.Get(),cpu,delta,sinvF,cbShadowLookup,nullptr,a,&proj);}
  if(r){



    if(ht>=0){postHalfFailed=true;++halfRefused;}if(fk)fin.written=~0ull;refusePost(r);return;}
  ++postRecorded;if(fk){fin.written=frames;++finRecorded;}else if(ht>=0){half[ht].written=frames;++halfDraws;}else{pp[k].lastUse=frames;ppCur=k;postUsed=true;}if(gsBound)++postGs;



  if(proj){++projectiveSkips;}
  if(batch.full())flushBatch(c,FL_CAP);
 }
 DgBatch batch;ComPtr<ID3D11DeviceContext1>ctx1;bool batchOff=false,flushing=false;const char*batchOffReason=nullptr,*recordRefusal="none";
 unsigned long long recorded=0,recordRefused=0,flushes=0,flushed=0,flushWhy[FL_KINDS]{};UINT batchMax=0,sinceSlotCheck=0,flushVerify=0;LONGLONG flushQpc=0;
 void flushBatch(ID3D11DeviceContext*c,int why){
  if(flushing||!batch.pending()||!b.rt)return;
  if(!ctx1&&FAILED(c->QueryInterface(IID_PPV_ARGS(&ctx1)))){batch.clear();return;}
  flushing=true;
  TrialPipelineState*before=(++flushVerify%64)==1?new(std::nothrow)TrialPipelineState(c):nullptr;
  UINT n=batch.flush(ctx1.Get(),b.rt.Get(),b.ds.Get());
  if(before){TrialPipelineState after(c);++verifyChecks;unsigned long long diff=before->differences(after);delete before;
   if(diff){batchOff=true;batchOffReason="state_changed_after_flush";if(logger)logger("second_eye: batch OFF: pipeline state differs after a flush (mask %llx); immediate route only\r\n",diff);}}

  ++flushes;flushed+=n;if(n>batchMax)batchMax=n;if(why>=0&&why<FL_KINDS)++flushWhy[why];flushing=false;
 }
 bool disabled=false;const char*disableReason=nullptr;
 unsigned frames=0,published=0,refused=0,eyeMismatch=0,empty=0,verifyChecks=0,otherTarget=0,sideWrites=0,listDiffers=0,guardSkipped=0,projectiveSkips=0;
 unsigned long long draws=0,drawsSinceVerify=0;LONGLONG qpc=0;unsigned statFrames=0;
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
  if(read!=DXGI_FORMAT_B8G8R8A8_UNORM)return "format_not_bgra";               /* the headset store is B8G8R8A8 */
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
  float Sg[16],Tg[16];dgGameClip(S,Sg);dgGameClip(T,Tg);memcpy(srcClip,Sg,sizeof srcClip);cpu.setCams(Sg,Tg);gpu.setSourceCamera(Sg);
  double dd[16],si[16];float identity[16]{};for(int i=0;i<4;i++)identity[i*5]=1;
  if(!dg_second_camera_delta(Sg,Tg,dd)||!dg_second_camera_delta(Sg,identity,si))return "camera_matrices";
  for(int i=0;i<16;i++){delta[i]=(float)dd[i];sinvF[i]=(float)si[i];}
  flushBatch(c,FL_FRAME);postUsed=false;postHalfFailed=false;clearPostSwaps();
  float sceneColour[4]{};takeSceneClear(sceneColour);c->ClearRenderTargetView(b.rt.Get(),sceneColour);c->ClearDepthStencilView(b.ds.Get(),D3D11_CLEAR_DEPTH|D3D11_CLEAR_STENCIL,clearDepth,0);
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



  bool sample=!cpuOff&&!checkPending&&++sinceCheck>=cpuCheckEvery;
  if(sample&&cpu.usesSplitBank(d,c,delta,sinvF,pers,cbShadowLookup,nullptr)){
   sample=false;sinceCheck=0;++checkSplitSkipped;  /* unsupported probe layout: render the ordinary CPU bank */
  }



  if(!batchOff&&!cpuOff&&api&&!(mode&(1|4))){
   if(!sample){
    if(!cbShadowArmed()){if(batch.pending()){++rearmedPending;flushBatch(c,FL_FALLBACK);}cbShadowEnsure();}
    if(!ctx1)c->QueryInterface(IID_PPV_ARGS(&ctx1));
    int proj=0;const char*r=ctx1?batch.record(d,ctx1.Get(),cpu,delta,sinvF,pers,cbShadowLookup,nullptr,(DgIndexedFn)api,n,first,base,&proj):"no_context1";
    if(r&&!strcmp(r,"ring_wrap")){flushBatch(c,FL_WRAP);r=batch.record(d,ctx1.Get(),cpu,delta,sinvF,pers,cbShadowLookup,nullptr,(DgIndexedFn)api,n,first,base,&proj);}
    if(!r){
     ++recorded;++cpuBinds;++draws;if(proj){++projectiveSkips;



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







    queueCheck(d,c,pers);
   }
  }
  auto rt=b.rt.Get();c->OMSetRenderTargets(1,&rt,b.ds.Get());











  draw();




  if(viaCpu)cpu.restore(c);else gpu.restore(c);c->OMSetRenderTargets(8,rts,ds);



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
  /* Live 27 Sep (5D9ACEFD): a row sum that nearly cancels (8.2e-5 from terms ~1) differs in its last
     bits between the GPU (fused multiply-add) and the CPU; relative to the RESULT that looked like a
     mismatch and switched the CPU route off. The bound is the size of the TERMS: 4*max|D|*max|row|. */
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
void batchHazard(ID3D11DeviceContext*c,int kind,ID3D11Resource*r){
 auto&e=secondEye;
 if(!e.batch.pending()||e.flushing||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 if(r&&!e.batch.references(r))return;
 e.flushBatch(c,kind);
}

/* Census (step 4 preparation, 27 Sep): which draws INTO THE GAME'S MAIN IMAGE the second eye
   does not replay, and why. Every 4th frame, per draw, after the draw: a category and, for the
   missing ones, the vertex shader with a few properties. Reported every 300 frames. Changes nothing. */
enum { CE_REPLAYED, CE_NONINDEXED, CE_NOROUTE, CE_NOCAMERA, CE_SELECT, CE_NOPAIR, CE_GUARD, CE_BEGIN, CE_LISTDIFF, CE_OTHERTARGET, CE_SIDEWRITES, CE_INSTANCED, CE_KINDS };
void batchMirrorClearRt(ID3D11DeviceContext*c,ID3D11RenderTargetView*v,const FLOAT col[4]){
 auto&e=secondEye;
 if(!v||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 ID3D11Resource*r=nullptr;v->GetResource(&r);void*res=r;if(r)r->Release();
 if(!inside&&!e.flushing&&secondEyeLive())e.noteSceneClear(v,(ID3D11Resource*)res,col);
 for(auto&q:e.pp)if(q.rt&&res==q.rt&&q.b3rt){e.flushBatch(c,FL_CLEAR);c->ClearRenderTargetView(q.b3rt.Get(),col);++e.postClears;}
}
void batchMirrorClearDs(ID3D11DeviceContext*c,ID3D11DepthStencilView*v,UINT f,FLOAT z,UINT8 s){
 auto&e=secondEye;
 if(!v||c!=context.Get()||!ownerThread||GetCurrentThreadId()!=ownerThread)return;
 ID3D11Resource*r=nullptr;v->GetResource(&r);void*res=r;if(r)r->Release();






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
 const char*why=nullptr;
 secondEyeDrawBody(c,next,api,drawCount,indexedDraw,initialized,forwarded,firstIndex,baseVertex,&why);
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
     bool a=magwellGpu.renderPair(device.Get(),c,e.gameRT.Get(),e.gameDS.Get(),e.b.rt.Get(),e.b.ds.Get(),e.magwell,e.srcClip,e.magwellTarget,e.colorDesc.Width,e.colorDesc.Height,e.clearDepth==0,now);

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
  if(e.beginFrame(device.Get(),c,S,T,dg_render_second_camera_eye(),seq)){next();return CE_BEGIN;}




 } else if(pair&&seq!=e.sequence){
  float Sg[16],Tg[16];dgGameClip(S,Sg);dgGameClip(T,Tg);double dd[16];double worst=1e9;
  if(dg_second_camera_delta(Sg,Tg,dd)){worst=0;for(int k=0;k<16;k++)worst=fmax(worst,fabs(dd[k]-(double)e.delta[k]));}
  if(worst>1e-4){++e.listDiffers;next();return CE_LISTDIFF;}
 }











 next();                                                     /* the game's draw, unchanged */






 inside=true;e.replay(device.Get(),c,next,pers,(UINT)slot,api,drawCount,firstIndex,baseVertex,hash);inside=false;

 return CE_REPLAYED;
}
void historyProbePresent();



void secondEyePresent(){
 auto&e=secondEye;++e.guardEpoch;e.discardSceneClears();
 if(context){if(secondEyeLive())e.flushBatch(context.Get(),FL_PRESENT);else e.batch.clear();}
 if(e.state==SecondEyeLive::BUILDING)e.state=SecondEyeLive::COMPLETE;
 if(context)e.pollCheck(context.Get());
}

























template<class F>void intercept(ID3D11DeviceContext*c,F next,void*api,const char*kind,bool forwarded=false,UINT drawCount=0,bool indexedDraw=false,bool initialized=false,UINT firstIndex=0,INT baseVertex=0,bool plainOriginal=false){
 if(!secondEyeLive()){next();return;}
 if(!TryAcquireSRWLockShared(&lifetime)){next();return;}
 struct ReleaseLife{~ReleaseLife(){ReleaseSRWLockShared(&lifetime);}} life;
 if(c!=context.Get()||!installed||stopping||inside){next();return;}
 if(InterlockedCompareExchangePointer(&lateXrContext,nullptr,nullptr)!=c||!multi->GetMultithreadProtected()){next();return;}
 MultiLock lock;
 secondEyeDraw(c,next,api,drawCount,indexedDraw,initialized,forwarded,firstIndex,baseVertex);
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
bool ammoHideDraw(ID3D11DeviceContext*,UINT,UINT);void ammoOnDraw(ID3D11DeviceContext*,UINT,UINT);void ammoOnPresent();
bool radarOnDraw(ID3D11DeviceContext*,UINT);void radarOnPresent(); // dg_radar.inl
#include "dg_cb_probe.inl"
#include "dg_mgshd_forward.inl"
#include "dg_mgshd_draw.inl"
void STDMETHODCALLTYPE indexed(ID3D11DeviceContext*c,UINT n,UINT first,INT base){


 ui2dOnDraw(c,0);            // 2D sprite placement per eye; idle unless vr_ui2d=1
 cbOnDraw(c,0,n,first,base); // read-only constant-buffer capture, idle unless armed
 void*api=(*(void***)c)[12];auto f=(Indexed)api;
 LONG route=forwardStatus();

 // Only the separately fingerprinted initialized entry. Never the MGSHDFix
 // detoured entry or an initialization wrapper; all intercept guards apply.
 if((requested==1||secondEyeLive()) && route==FORWARD_BOUND && c==context.Get() && api!=forwardEntry && initializedAddress(api)){
  intercept(c,[=](){f(c,n,first,base);},api,"DrawIndexed/validated-initialized",false,n,true,true,first,base);return;
 }

 if(route==FORWARD_WAITING||route==FORWARD_BOUND||route==FORWARD_REJECTED||route==FORWARD_STOPPED){ // No outer locks.
  NativeForwardScope scope;


  f(c,n,first,base);


  return;
 }
 intercept(c,[=](){f(c,n,first,base);},api,"DrawIndexed",false,n,true);
}
#include "dg_history_probe.inl"
#include "dg_post_probe.inl"
void STDMETHODCALLTYPE draw(ID3D11DeviceContext*c,UINT n,UINT first){



 if(ui2dSkipDraw(c,n)){ui2dTraceDraw(c,n,1);return;} // previous-frame feedback in stereo: not forwarded
 ammoOnDraw(c,n,first);
 ui2dOnDraw(c,1);cbOnDraw(c,1,n,first,0);



 if(ammoHideDraw(c,n,first))return;
 if(radarOnDraw(c,n)){ui2dTraceDraw(c,n,2);return;} // wrist radar: the composite's texture is captured; with vr_radar_hud=off the draw is not forwarded. No lock held across it.
 ui2dTraceDraw(c,n,0);
 void*api=(*(void***)c)[13];auto f=(Draw)api;








 intercept(c,[=](){blitForward(c,n,first,api,[=](){f(c,n,first);});},api,"Draw",false,n,false,false,first,0);

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

 if(own&&block)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((QueryCreate)hooks[0].next)(d,desc,q);
 return hr;
}
HRESULT STDMETHODCALLTYPE createPredicate(ID3D11Device*d,const D3D11_QUERY_DESC*desc,ID3D11Predicate**q){
 bool own=d==InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr);

 if(own)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((PredicateCreate)hooks[1].next)(d,desc,q);
 return hr;
}
HRESULT STDMETHODCALLTYPE createCounter(ID3D11Device*d,const D3D11_COUNTER_DESC*desc,ID3D11Counter**q){
 bool own=d==InterlockedCompareExchangePointer(&queryOwner,nullptr,nullptr);

 if(own)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((CounterCreate)hooks[2].next)(d,desc,q);
 return hr;
}
HRESULT STDMETHODCALLTYPE createQuery1(ID3D11Device3*d,const D3D11_QUERY_DESC1*desc,ID3D11Query1**q){
 bool own=d==InterlockedCompareExchangePointer(&modernQueryOwner,nullptr,nullptr);
 bool block=queryBlocksTrial(desc!=nullptr,desc?desc->Query:0,desc?desc->MiscFlags:0,desc?desc->ContextType:0);

 if(own&&block)InterlockedExchange(&scopeCreated,1);
 HRESULT hr=((QueryCreate1)hooks[3].next)(d,desc,q);
 return hr;
}
void*nearMemory(BYTE*site){SYSTEM_INFO info;GetSystemInfo(&info);uintptr_t center=(uintptr_t)site&~(uintptr_t)(info.dwAllocationGranularity-1);
 for(uintptr_t step=info.dwAllocationGranularity;step<0x70000000;step+=info.dwAllocationGranularity){for(int sign:{-1,1}){uintptr_t a=sign<0?center-step:center+step;if(sign<0&&center<step)continue;void*p=VirtualAlloc((void*)a,4096,MEM_RESERVE|MEM_COMMIT,PAGE_READWRITE);if(p)return p;}}return nullptr;}



#include "dg_ui2d.inl"




#include "dg_blit_boundary.inl"



                              void bpForeign(void*,void*){}

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
 // Preserve the host setting: enabling D3D protection causes stereo ghosting in headset A/B tests.
 oldProtected=multi->GetMultithreadProtected()!=FALSE;
 dg_context_trace_emit(logger,"draw_attach",device.Get(),context.Get(),multi.Get(),DG_ENABLE_DIAGNOSTICS);
 if(logger)logger("draw_trial: thread protection preserved=%d forced=0; extra trial draw requires existing protection\r\n",oldProtected);
#if 1
auto table=*(void***)d;void*ours[]={(void*)createQuery,(void*)createPredicate,(void*)createCounter};bool ok=true;
 for(unsigned i=0;i<3;i++){hooks[i]={table+24+i,table[24+i],ours[i]};if(!runtimePointer(hooks[i].next))ok=false;}
 if(SUCCEEDED(d->QueryInterface(IID_PPV_ARGS(&modernDevice)))){auto mt=*(void***)modernDevice.Get();hooks[3]={mt+60,mt[60],(void*)createQuery1};hookCount=4;if(!runtimePointer(hooks[3].next))ok=false;}
 InterlockedExchangePointer(&queryOwner,d);InterlockedExchangePointer(&modernQueryOwner,modernDevice.Get());
#else
 bool ok=true; hookCount=0;
#endif




 uint64_t base=(uint64_t)(ULONG_PTR)GetModuleHandleW(nullptr);

 if(ok)ok=prepareSites(base);
 if(ok){HMODULE pin=nullptr;ok=GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)&indexed,&pin)!=FALSE;}
 if(ok){for(unsigned i=0;i<hookCount;i++){auto&h=hooks[i];if(!writeSlot(h,h.next,h.ours)){ok=false;break;}}}
 if(ok)for(auto&s:sites)if(!codeWrite(s,s.original,s.patch)){ok=false;break;}
 if(!ok){restoreAll();log("attach refused: code/API ownership; no draw trial");}
 else{installed=true;ui2dInstall(d);beginForward();







 log("attached: player rendering, HUD and radar; experimental Both Eyes available");

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

























































































}
extern "C" void dg_draw_trial_present(){
 LifeLock life;if(!installed||stopping)return;ammoOnPresent();MultiLock lock;
 if(!ownerThread)ownerThread=GetCurrentThreadId();
 ui2dOnPresent();radarOnPresent();
 if(dg_second_eye_enabled())secondEyePresent();
}
extern "C" void dg_draw_trial_stop(){InterlockedExchange(&stopping,1);AcquireSRWLockExclusive(&lifetime);bool owned=sitesOwned();
 // Code sites remain forwarding-only until process exit. Unlike pointer slots,
 // ten instruction bytes cannot safely be restored amid concurrent execution.
 magwellGpu=DgMagwellGpu{};ammoReset();bpStop();wwRemove();restoreMgDraw();restoreForward();setForwardStatus(FORWARD_STOPPED);ui2dRemove();

 cbShadowRemove();secondEye.batch.clear();

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








extern "C" int dg_second_eye_publish(int slot,const void*raw,const void*fov){
 auto&e=secondEye;
 if(!secondEyeLive()||e.state==SecondEyeLive::IDLE||!e.b.color){++e.empty;



 return 0;}
 /* No MultiLock here (live 27 Sep, C254C15C: process froze, then vanished):
    dg_xr_capture_texture takes the capture lock, and the XR thread takes that
    lock BEFORE its own context calls, which enter the D3D multithread lock
    (protection is on). D3D lock -> capture lock here against capture lock ->
    D3D lock there deadlocked both threads. The native capture runs on this
    thread without the D3D lock too; the capture lock serializes the store. */
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




 if(ok)++e.published;else ++e.refused;



 e.state=SecondEyeLive::IDLE;
 return ok;
}





