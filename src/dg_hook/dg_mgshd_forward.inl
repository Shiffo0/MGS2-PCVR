// Recognises the structure of MGSHDFix's DrawIndexed hooks at runtime instead of one pinned build:
// per detoured d3d11 entry the chain of safetyhook detours is walked to the INNERMOST hook, its
// safetyhook object is found in MGSHDFix's writable sections, and its trampoline pointer (object+0x20)
// is replaced by one of four once-written forward records. The native wrapper executes once, without
// our locks; only its synchronous original-call forwarding may claim the one-shot trial.
#include "dg_mgshd_signature.h"
// FORWARD_NONE is last so the existing values 0-4 (and lateRouteBucket(100)==4) keep their meaning.
enum ForwardState { FORWARD_DIRECT, FORWARD_WAITING, FORWARD_BOUND, FORWARD_REJECTED, FORWARD_STOPPED, FORWARD_NONE };
constexpr unsigned FORWARD_STATES=6;
volatile LONG forwardState=FORWARD_DIRECT;
LONG forwardStatus(){return InterlockedCompareExchange(&forwardState,0,0);}
// Raw store: tests only. Production derives the state in forwardRecompute() (one write rule).
void setForwardStatus(LONG value){InterlockedExchange(&forwardState,value);}
// Module outcome, the first of the three inputs of forwardRecompute().
enum { FM_DIRECT, FM_PRESENT, FM_PIN_FAILED, FM_STOPPED };
volatile LONG forwardModuleState=FM_DIRECT;
BYTE*forwardModule=nullptr;
thread_local unsigned nativeForwardDepth=0;
thread_local bool nativeForwardAvailable=false;
struct NativeForwardScope{
 NativeForwardScope(){if(nativeForwardDepth++==0)nativeForwardAvailable=true;}
 ~NativeForwardScope(){if(--nativeForwardDepth==0)nativeForwardAvailable=false;}
};
bool localBytes(const void*p,void*out,SIZE_T n){SIZE_T got=0;return ReadProcessMemory(GetCurrentProcess(),p,out,n,&got)&&got==n;}
bool sameBytes(void*p,const void*expected,SIZE_T n){
 BYTE b[sizeof(mgshdCallbackBytes)];return n<=sizeof b&&localBytes(p,b,n)&&!memcmp(b,expected,n);
}
bool fileSha256(HMODULE module,BYTE digest[32]){
 wchar_t path[32768];DWORD n=GetModuleFileNameW(module,path,32768);if(!n||n>=32768)return false;
 HANDLE file=CreateFileW(path,GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
 if(file==INVALID_HANDLE_VALUE)return false;
 BCRYPT_ALG_HANDLE alg=nullptr;BCRYPT_HASH_HANDLE hash=nullptr;static BYTE buf[65536];
 bool ok=BCryptOpenAlgorithmProvider(&alg,BCRYPT_SHA256_ALGORITHM,nullptr,0)>=0;
 if(ok)ok=BCryptCreateHash(alg,&hash,nullptr,0,nullptr,0,0)>=0;
 DWORD got=0;
 while(ok){if(!ReadFile(file,buf,sizeof buf,&got,nullptr)){ok=false;break;}if(!got)break;ok=BCryptHashData(hash,buf,got,0)>=0;}
 if(ok)ok=BCryptFinishHash(hash,digest,32,0)>=0;
 if(hash)BCryptDestroyHash(hash);if(alg)BCryptCloseAlgorithmProvider(alg,0);CloseHandle(file);return ok;
}
// Full SHA256 BDBFBDC8FFF2B318C3597DB9C69DB6C583ED8B7504685505F3112D6676356CC3 = MGSHDFix 4.0.0.
const BYTE mgshdSha400[32]={0xbd,0xbf,0xbd,0xc8,0xff,0xf2,0xb3,0x18,0xc3,0x59,0x7d,0xb9,0xc6,0x9d,0xb6,0xc5,0x83,0xed,0x8b,0x75,0x04,0x68,0x55,0x05,0xf3,0x11,0x2d,0x66,0x76,0x35,0x6c,0xc3};
bool fileIdentity(HMODULE module){BYTE d[32];return fileSha256(module,d)&&!memcmp(d,mgshdSha400,32);}
const char*knownMgshd(const BYTE d[32]){
 if(!memcmp(d,mgshdSha400,32))return "4.0.0";
 const BYTE p504[4]={0x67,0xf3,0x5f,0x2e};
 return !memcmp(d,p504,4)?"5.0.4":"unknown";
}

bool decodeForwardWith(bool(*read)(const void*,void*,SIZE_T),BYTE*original,BYTE*&entry,unsigned&copied){
 BYTE bytes[12];if(!read(original,bytes,sizeof bytes))return false;
 const BYTE seven[]={0x48,0x8b,0xc4,0x48,0x89,0x58,0x10};
 const BYTE five[]={0x48,0x89,0x5c,0x24,0x10};
 if(!memcmp(bytes,seven,7)&&bytes[7]==0xe9)copied=7;
 else if(!memcmp(bytes,five,5)&&bytes[5]==0xe9)copied=5;
 else return false;
 INT32 offset;memcpy(&offset,bytes+copied+1,4);
 uintptr_t resume=(uintptr_t)original+copied+5+(intptr_t)offset;
 if(resume<copied)return false;entry=(BYTE*)(resume-copied);return true;
}
static bool localRead(const void*p,void*out,SIZE_T n){return localBytes(p,out,n);}
bool decodeForward(BYTE*original,BYTE*&entry,unsigned&copied){return decodeForwardWith(localRead,original,entry,copied);}
struct ForwardRoute{BYTE*entry=nullptr;BYTE*relay=nullptr;unsigned copied=0;};
const char*forwardRouteWhy="none";

bool readForwardRoute(BYTE*module,BYTE*original,ForwardRoute&route){
 if(!decodeForward(original,route.entry,route.copied)){forwardRouteWhy="trampoline_prologue";return false;}
 HMODULE owner=nullptr;
 if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCWSTR)route.entry,&owner)||owner!=GetModuleHandleW(L"d3d11.dll")){forwardRouteWhy="entry_not_d3d11";return false;}
 BYTE entry[5],relay[14];if(!localBytes(route.entry,entry,sizeof entry)||entry[0]!=0xe9){forwardRouteWhy="entry_not_jmp";return false;}
 INT32 offset;memcpy(&offset,entry+1,4);route.relay=(BYTE*)((uintptr_t)route.entry+5+(intptr_t)offset);
 if(!localBytes(route.relay,relay,sizeof relay)){forwardRouteWhy="relay_unreadable";return false;}
 const BYTE indirect[]={0xff,0x25,0,0,0,0};if(memcmp(relay,indirect,6)){forwardRouteWhy="relay_not_indirect_jmp";return false;}
 void*callback=nullptr;memcpy(&callback,relay+6,8);
 if(callback!=module+0xb5150||!sameBytes(callback,mgshdCallbackBytes,sizeof mgshdCallbackBytes)){forwardRouteWhy="callback_not_mgshdfix";return false;}
 if(!runtimePointer(route.entry+route.copied)){forwardRouteWhy="resume_not_runtime";return false;}
 forwardRouteWhy="none";return true;
}

/* ---------------- chain reader (read-only, never under a lock) ---------------- */
// safetyhook InlineHook layout (x64 MSVC, identical in 4.0.0 and 5.0.4; design F1).
constexpr size_t SH_TARGET=0x00,SH_DEST=0x08,SH_TRAMP=0x20,SH_TRAMP_SIZE=0x28,SH_ENABLED=0xA0,SH_TYPE=0xA4,SH_SIZE=0xA8;
struct ChainEnv{
 bool(*read)(const void*,void*,SIZE_T);   // trampolines/relays: safetyhook memory (ReadProcessMemory in the game)
 bool(*inD3d11)(const void*);
 bool(*inHdText)(const void*);            // MGSHDFix executable section
 bool(*isFunction)(const void*);          // has an unwind entry (RtlLookupFunctionEntry)
 bool(*inAnyModule)(const void*);
 bool(*executable)(const void*);
 bool(*resumeOk)(const void*);            // runtimePointer on entry+copied
};
// Copy of MGSHDFix's writable sections, taken without a lock; 8-byte aligned reads are atomic on x64,
// a half-written object fails the two-poll stability requirement.
struct WritableSnap{
 static constexpr unsigned MAX=6;
 BYTE*base[MAX]{};SIZE_T size[MAX]{};BYTE*copy[MAX]{};SIZE_T cap[MAX]{};bool ok[MAX]{};unsigned n=0;
};
struct ChainResult{
 BYTE*entry=nullptr;BYTE*object=nullptr;void**slot=nullptr;BYTE*trampoline=nullptr;BYTE*callback=nullptr;
 unsigned copied=0,depth=0;BYTE inner[16]{};
 BYTE*layers[8]{};   // callback per layer, [0] = outermost .. [depth] = innermost (logging only, not part of same())
 bool same(const ChainResult&o)const{return entry==o.entry&&object==o.object&&trampoline==o.trampoline&&callback==o.callback&&copied==o.copied&&depth==o.depth&&!memcmp(inner,o.inner,16);}
};
static bool snapQ(const WritableSnap&s,BYTE*at,SIZE_T n,void*out){
 for(unsigned i=0;i<s.n;i++)if(s.ok[i]&&at>=s.base[i]&&at+n<=s.base[i]+s.size[i]){memcpy(out,s.copy[i]+(at-s.base[i]),n);return true;}
 return false;
}
// Every 8-byte aligned B with [B]==target and [B+8]==destination whose whole object lies in one section.
static unsigned findObjects(const WritableSnap&s,BYTE*target,BYTE*dest,BYTE**first){
 unsigned count=0;
 for(unsigned i=0;i<s.n;i++){
  if(!s.ok[i]||s.size[i]<SH_SIZE)continue;
  SIZE_T start=((uintptr_t)s.base[i]+7&~(uintptr_t)7)-(uintptr_t)s.base[i];
  for(SIZE_T o=start;o+SH_SIZE<=s.size[i];o+=8){
   BYTE*q0;memcpy(&q0,s.copy[i]+o,8);if(q0!=target)continue;
   BYTE*q1;memcpy(&q1,s.copy[i]+o+8,8);if(q1!=dest)continue;
   if(!count&&first)*first=s.base[i]+o;++count;
  }
 }
 return count;
}
bool walkChain(const ChainEnv&env,const WritableSnap&snap,BYTE*E,ChainResult&out,const char**why){
 auto fail=[&](const char*r){*why=r;return false;};
 if(!env.inD3d11(E))return fail("entry_not_d3d11");
 BYTE e[6];if(!env.read(E,e,sizeof e))return fail("entry_unreadable");
 if(e[0]==0xff&&e[1]==0x25)return fail("entry_ff_hook");          // safetyhook FF variant: not supported
 if(e[0]!=0xe9)return fail("entry_not_detoured");
 INT32 rel;memcpy(&rel,e+1,4);BYTE*relay=E+5+rel;
 for(unsigned depth=0;depth<8;depth++){
  BYTE r[14];if(!env.read(relay,r,sizeof r))return fail("relay_unreadable");
  const BYTE indirect[]={0xff,0x25,0,0,0,0};if(memcmp(r,indirect,6))return fail("relay_not_indirect_jmp");
  BYTE*cb;memcpy(&cb,r+6,8);
  out.layers[depth]=cb;
  if(!env.inHdText(cb)||!env.isFunction(cb))return fail("callback_not_mgshdfix_function");
  BYTE*B=nullptr;unsigned found=findObjects(snap,E,cb,&B);
  if(!found)return fail("no_safetyhook_object");
  if(found>1)return fail("object_ambiguous");
  BYTE*T=nullptr;UINT64 size=0;BYTE enabled=0;DWORD type=0;
  if(!snapQ(snap,B+SH_TRAMP,8,&T)||!snapQ(snap,B+SH_TRAMP_SIZE,8,&size)||!snapQ(snap,B+SH_ENABLED,1,&enabled)||!snapQ(snap,B+SH_TYPE,4,&type))return fail("no_safetyhook_object");
  if(!T||!size||env.inAnyModule(T)||!env.executable(T))return fail("trampoline_invalid");
  if(enabled!=1||type!=1)return fail("hook_not_enabled_e9");
  BYTE t[16];if(!env.read(T,t,sizeof t))return fail("trampoline_invalid");
  if(t[0]==0xe9){memcpy(&rel,t+1,4);relay=T+5+rel;continue;}      // chained: one hook further in
  BYTE*e2=nullptr;unsigned copied=0;
  if(decodeForwardWith(env.read,T,e2,copied)&&e2==E&&env.resumeOk(E+copied)){
   out.entry=E;out.object=B;out.slot=(void**)(B+SH_TRAMP);out.trampoline=T;out.callback=cb;out.copied=copied;out.depth=depth;memcpy(out.inner,t,16);
   *why="none";return true;
  }
  return fail("innermost_prologue_unknown");
 }
 return fail("chain_too_long");
}
// Production environment.
struct HdText{BYTE*lo[4]{};BYTE*hi[4]{};unsigned n=0;} hdText;
static bool envInD3d11(const void*p){HMODULE m=nullptr;return GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCWSTR)p,&m)&&m==GetModuleHandleW(L"d3d11.dll");}
static bool envInHdText(const void*p){for(unsigned i=0;i<hdText.n;i++)if((const BYTE*)p>=hdText.lo[i]&&(const BYTE*)p<hdText.hi[i])return true;return false;}
static bool envIsFunction(const void*p){DWORD64 base=0;return RtlLookupFunctionEntry((DWORD64)(uintptr_t)p,&base,nullptr)!=nullptr;}
static bool envInAnyModule(const void*p){HMODULE m=nullptr;return GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCWSTR)p,&m)!=FALSE;}
static bool envExecutable(const void*p){MEMORY_BASIC_INFORMATION mb{};if(!VirtualQuery(p,&mb,sizeof mb)||mb.State!=MEM_COMMIT)return false;
 return (mb.Protect&(PAGE_EXECUTE|PAGE_EXECUTE_READ|PAGE_EXECUTE_READWRITE|PAGE_EXECUTE_WRITECOPY))!=0&&!(mb.Protect&PAGE_GUARD);}
static bool envResumeOk(const void*p){return runtimePointer((void*)p);}
const ChainEnv liveChainEnv{localRead,envInD3d11,envInHdText,envIsFunction,envInAnyModule,envExecutable,envResumeOk};
WritableSnap liveSnap;
// Sections of the module image: executable ranges (callbacks, callers) and writable ranges (objects).
bool mapModuleSections(BYTE*m){
 auto dos=(IMAGE_DOS_HEADER*)m;if(dos->e_magic!=IMAGE_DOS_SIGNATURE)return false;
 auto nt=(IMAGE_NT_HEADERS64*)(m+dos->e_lfanew);if(nt->Signature!=IMAGE_NT_SIGNATURE)return false;
 auto sec=IMAGE_FIRST_SECTION(nt);hdText=HdText{};liveSnap.n=0;
 for(unsigned i=0;i<nt->FileHeader.NumberOfSections;i++){
  BYTE*lo=m+sec[i].VirtualAddress;SIZE_T sz=sec[i].Misc.VirtualSize;
  if((sec[i].Characteristics&IMAGE_SCN_MEM_EXECUTE)&&hdText.n<4){hdText.lo[hdText.n]=lo;hdText.hi[hdText.n++]=lo+sz;}
  if((sec[i].Characteristics&IMAGE_SCN_MEM_WRITE)&&liveSnap.n<WritableSnap::MAX&&sz){liveSnap.base[liveSnap.n]=lo;liveSnap.size[liveSnap.n++]=sz;}
 }
 return hdText.n&&liveSnap.n;
}
void takeSnap(WritableSnap&s){
 for(unsigned i=0;i<s.n;i++){
  if(s.cap[i]<s.size[i]){free(s.copy[i]);s.copy[i]=(BYTE*)malloc(s.size[i]);s.cap[i]=s.copy[i]?s.size[i]:0;}
  s.ok[i]=s.copy[i]&&localBytes(s.base[i],s.copy[i],s.size[i]);
 }
}

/* ---------------- forward records: written once, before publication ---------------- */
enum { REC_FREE=0, REC_BOUND=1, REC_LOST=2 };
constexpr unsigned FWD_RECORDS=4;
struct FwdRecord{
 BYTE*entry;BYTE*object;void**slot;BYTE*trampoline;BYTE*callback;unsigned copied,depth;BYTE inner[16];
 Hook hook;                       // {slot, next=trampoline, ours=forwardedIndexedN<I>}
 volatile LONG state;             // FREE -> BOUND -> LOST (never back to FREE); LOST -> BOUND only for the same chain
 bool filled;                     // fields written (FREE until a publication succeeded)
 void*callersA[8];void*callersB[4];unsigned nA,nB;   // validated return addresses (owner thread)
};
static unsigned long long fwdRva(const void*p){static HMODULE m=GetModuleHandleW(L"d3d11.dll");return p&&m&&(const BYTE*)p>=(const BYTE*)m?(unsigned long long)((const BYTE*)p-(const BYTE*)m):0ull;}
FwdRecord fwd[FWD_RECORDS]{};volatile LONG fwdUsed=0;
volatile LONG fwdCallerUnknown=0,fwdSecondReplayed=0,fwdSecondObserved=0,fwdSeenOverflow=0;
LONG recState(const FwdRecord&r){return InterlockedCompareExchange((volatile LONG*)&r.state,0,0);}
unsigned recordsUsed(){LONG n=InterlockedCompareExchange(&fwdUsed,0,0);return n<0?0:n>(LONG)FWD_RECORDS?FWD_RECORDS:(unsigned)n;}
FwdRecord*boundRecordFor(void*entry){for(unsigned i=0;i<recordsUsed();i++)if(recState(fwd[i])==REC_BOUND&&fwd[i].entry==entry)return &fwd[i];return nullptr;}
bool boundEntry(void*api){return api&&boundRecordFor(api)!=nullptr;}
unsigned forwardFirstCopied(){for(unsigned i=0;i<recordsUsed();i++)if(recState(fwd[i])==REC_BOUND)return fwd[i].copied;return 0;}
unsigned boundCount(){unsigned n=0;for(unsigned i=0;i<recordsUsed();i++)if(recState(fwd[i])==REC_BOUND)++n;return n;}
// The read-only chain witness of a bound record (no side effects).
const char*recordChainFault(const FwdRecord&r){
 BYTE e=0;if(!localBytes(r.entry,&e,1)||e!=0xe9)return "entry_changed";     // the target may change: outer hooks may be added
 BYTE t[16];if(!localBytes(r.trampoline,t,16)||memcmp(t,r.inner,16))return "inner_bytes_changed";
 BYTE en=0;DWORD ty=0;if(!localBytes(r.object+SH_ENABLED,&en,1)||!localBytes(r.object+SH_TYPE,&ty,4)||en!=1||ty!=1)return "hook_disabled";
 return nullptr;
}
const char*recordFault(const FwdRecord&r){
 void*now=nullptr;if(!r.slot||!localBytes(r.slot,&now,8)||now!=r.hook.ours)return "slot_not_ours";
 return recordChainFault(r);
}
// Pure question for stop/trial/frame paths: every bound record still ours and intact.
bool forwardOwned(){for(unsigned i=0;i<recordsUsed();i++)if(recState(fwd[i])==REC_BOUND&&recordFault(fwd[i]))return false;return true;}
bool fwdApi(void*api){
 if(!api)return false;
 for(unsigned i=0;i<recordsUsed();i++)if(recState(fwd[i])==REC_BOUND&&fwd[i].hook.next==api)return !recordFault(fwd[i]);
 return false;
}
bool forwardApi(void*api){return fwdApi(api);}

/* ---------------- detoured entries seen by `indexed` (render thread -> poll) ---------------- */
void*volatile detouredSeen[4]{};
// Owner thread; c==context, not direct, not the entry of a bound record, not yet listed. Never cleared.
void noteDetoured(ID3D11DeviceContext*c,void*api,bool direct){
 if(!api||c!=context.Get()||direct||boundEntry(api))return;
 for(auto&s:detouredSeen){
  void*cur=InterlockedCompareExchangePointer(&s,nullptr,nullptr);
  if(cur==api)return;
  if(!cur){if(InterlockedCompareExchangePointer(&s,api,nullptr)==nullptr)return;if(s==api)return;}
 }
 InterlockedIncrement(&fwdSeenOverflow);   // also counts a pointer outside d3d11 that took a place
}

/* ---------------- state: one write rule ---------------- */
enum { FWD_CHAIN=0, FWD_PINNED=1 };
volatile LONG fwdMode=FWD_CHAIN,fwdModeWant=FWD_CHAIN,fwdAll=0;bool fwdModeLatched=false,fwdModeChangeLogged=false;
struct EntryProbe{ChainResult cand;unsigned stable,attempts,refusals;const char*lastWhy;ULONGLONG nextTry;volatile LONG backoff;};
EntryProbe probes[4]{};   // index = detouredSeen index; poll thread only (backoff is read by forwardRecompute)
volatile LONG pinnedRejected=0;
LONG forwardCompute(){
 LONG m=InterlockedCompareExchange(&forwardModuleState,0,0);
 if(m==FM_DIRECT)return FORWARD_DIRECT;
 if(m==FM_STOPPED)return FORWARD_STOPPED;
 if(m==FM_PIN_FAILED)return FORWARD_REJECTED;
 if(boundCount())return FORWARD_BOUND;
 if(InterlockedCompareExchange(&fwdMode,0,0)==FWD_PINNED)return InterlockedCompareExchange(&pinnedRejected,0,0)?FORWARD_REJECTED:FORWARD_WAITING;
 unsigned seen=0,waiting=0;
 for(unsigned i=0;i<4;i++){void*e=InterlockedCompareExchangePointer(&detouredSeen[i],nullptr,nullptr);if(!e)continue;++seen;
  if(!InterlockedCompareExchange(&probes[i].backoff,0,0))++waiting;}
 if(!seen)return FORWARD_NONE;
 return waiting?FORWARD_WAITING:FORWARD_REJECTED;
}
// Callers: the poll (lifetime exclusive) after a publication or refusal; the render thread (shared) after LOST.
void forwardRecompute(){InterlockedExchange(&forwardState,forwardCompute());}
const char*forwardStateName(LONG s){static const char*const n[]={"DIRECT","WAITING","BOUND","REJECTED","STOPPED","NONE"};return s>=0&&s<(LONG)FORWARD_STATES?n[s]:"?";}

/* ---------------- per-frame monitoring (side effects) ---------------- */
// Once per frame from guardOk, render thread under lifetime-shared. A fault marks the record LOST,
// puts the real trampoline back if the slot is still ours, and re-derives the state.
bool forwardMonitor(){
 bool all=true;
 for(unsigned i=0;i<recordsUsed();i++){
  FwdRecord&r=fwd[i];if(recState(r)!=REC_BOUND)continue;
  const char*why=recordFault(r);if(!why)continue;
  all=false;InterlockedExchange(&r.state,REC_LOST);
  void*now=nullptr;if(localBytes(r.slot,&now,8)&&now==r.hook.ours)writeSlot(r.hook,r.hook.ours,r.hook.next);
  if(logger)logger("draw_trial: forward lost entry=d3d11+0x%llX record=%u why=%s\r\n",fwdRva(r.entry),i,why);
  forwardRecompute();
 }
 return all;
}
void restoreForward(){
 for(unsigned i=0;i<recordsUsed();i++){FwdRecord&r=fwd[i];if(r.filled&&r.hook.slot&&*r.hook.slot==r.hook.ours)writeSlot(r.hook,r.hook.ours,r.hook.next);}
}
// DLL_PROCESS_DETACH: no lifetime lock (a terminated thread may have held it), no logging; VirtualQuery,
// VirtualProtect and an interlocked exchange only. If MGSHDFix's destructor ran first the slot is no longer ours.
extern "C" void dg_draw_trial_detach(){
 for(unsigned i=0;i<FWD_RECORDS;i++){
  FwdRecord&r=fwd[i];if(!r.filled||!r.hook.slot)continue;
  MEMORY_BASIC_INFORMATION mb{};
  if(!VirtualQuery(r.hook.slot,&mb,sizeof mb)||mb.State!=MEM_COMMIT||(mb.Protect&(PAGE_NOACCESS|PAGE_GUARD))||!mb.Protect)continue;
  if(InterlockedCompareExchangePointer((void*volatile*)r.hook.slot,nullptr,nullptr)!=r.hook.ours)continue;
  writeSlot(r.hook,r.hook.ours,r.hook.next);
 }
}

/* ---------------- caller check (owner thread) ---------------- */
enum { CALLER_NONE=0, CALLER_A=1, CALLER_B=2 };
static bool callR64Before(const BYTE*w,size_t n){return n>=2&&w[n-2]==0xff&&(w[n-1]&0xf8)==0xd0;}   // FF D0..D7, also 41 FF D0..D7
// Pure byte classifier on the 0x40 bytes before `ret`; `slot` = the record's slot address.
// A: call r64 before ret and a rip-relative load (REX 48/4C, 8B, modrm mod=00 rm=101) of the slot in the window,
//    or 'FF 15 disp32' ending at ret through the slot.  B: call r64 before ret, 'mov r64,[reg+20h]' and
//    'cmp qword ptr [reg+28h],0' on the SAME base register (the non-inlined InlineHook::stdcall helper).
int classifyCaller(const BYTE*w,uintptr_t ret,void**slot){
 const size_t N=0x40;uintptr_t base=ret-N;
 if(w[N-6]==0xff&&w[N-5]==0x15){INT32 d;memcpy(&d,w+N-4,4);if(ret+(intptr_t)d==(uintptr_t)slot)return CALLER_A;}
 if(!callR64Before(w,N))return CALLER_NONE;
 for(size_t i=0;i+7<=N-2;i++)
  if((w[i]==0x48||w[i]==0x4c)&&w[i+1]==0x8b&&(w[i+2]&0xc7)==0x05){INT32 d;memcpy(&d,w+i+3,4);if(base+i+7+(intptr_t)d==(uintptr_t)slot)return CALLER_A;}
 int movReg[8];unsigned nm=0;
 for(size_t i=0;i+4<=N-2;i++){
  BYTE rex=w[i];if((rex&0xf8)!=0x48)continue;BYTE m=w[i+2];
  if(w[i+1]==0x8b&&(m>>6)==1&&(m&7)!=4&&w[i+3]==0x20&&nm<8)movReg[nm++]=(m&7)|((rex&1)<<3);
 }
 for(size_t i=0;i+5<=N-2&&nm;i++){
  BYTE rex=w[i];if((rex&0xf8)!=0x48)continue;BYTE m=w[i+2];
  if(w[i+1]==0x83&&(m>>6)==1&&((m>>3)&7)==7&&(m&7)!=4&&w[i+3]==0x28&&w[i+4]==0x00){int reg=(m&7)|((rex&1)<<3);for(unsigned k=0;k<nm;k++)if(movReg[k]==reg)return CALLER_B;}
 }
 return CALLER_NONE;
}
struct UnknownCaller{void*ret[16];unsigned n;} unknownCallers{};
int callerClass(unsigned I,void*ret){
 FwdRecord&r=fwd[I];
 for(unsigned i=0;i<r.nA;i++)if(r.callersA[i]==ret)return CALLER_A;
 for(unsigned i=0;i<r.nB;i++)if(r.callersB[i]==ret)return CALLER_B;
 if(!envInHdText(ret)||!envInHdText((BYTE*)ret-0x40))return CALLER_NONE;
 BYTE w[0x40];if(!localBytes((BYTE*)ret-0x40,w,sizeof w))return CALLER_NONE;
 int k=classifyCaller(w,(uintptr_t)ret,r.slot);
 if(k==CALLER_A&&r.nA<8)r.callersA[r.nA++]=ret;
 if(k==CALLER_B&&r.nB<4)r.callersB[r.nB++]=ret;
 if(k!=CALLER_A){
  bool logged=false;for(unsigned i=0;i<unknownCallers.n;i++)if(unknownCallers.ret[i]==ret)logged=true;
  if(!logged&&unknownCallers.n<16){unknownCallers.ret[unknownCallers.n++]=ret;
   if(logger){char hex[0x81];for(unsigned i=0;i<0x40;i++)sprintf_s(hex+2*i,3,"%02X",w[i]);
    logger("draw_trial: forward caller %s record=%u ret=+0x%llX bytes=%s\r\n",k==CALLER_B?"class_b":"unknown",I,(unsigned long long)((BYTE*)ret-forwardModule),hex);}}
 }
 return k;
}

bool directApi(ID3D11DeviceContext*c,void*api){
 return api&&c==context.Get()&&c->GetType()==D3D11_DEVICE_CONTEXT_IMMEDIATE&&!boundEntry(api)&&directCode(api);
}
#include "dg_initialized_signature.h"
/* Called for every indexed draw: the d3d11 base is looked up once (the module is
   pinned by the process for its lifetime). */
bool initializedAddress(void*api){static HMODULE m=GetModuleHandleW(L"d3d11.dll");return m&&(api==(BYTE*)m+0x154260||api==(BYTE*)m+initializedRvaKb5124010);}
bool initializedBody(const BYTE*b,size_t n){return n==sizeof initializedBytes&&(!memcmp(b,initializedBytes,n)||!memcmp(b,initializedBytesKb5124010,n));}
bool initializedApi(ID3D11DeviceContext*c,void*api){
 if(InterlockedCompareExchange(&seDirect,0,0))return directApi(c,api)&&directSlotKnown(c,api);
 if(!initializedAddress(api)||c!=context.Get()||c->GetType()!=D3D11_DEVICE_CONTEXT_IMMEDIATE||(*(void***)c)[12]!=api)return false;
 BYTE bytes[sizeof initializedBytes]{};SIZE_T got=0;
 return ReadProcessMemory(GetCurrentProcess(),api,bytes,sizeof bytes,&got)&&got==sizeof bytes&&initializedBody(bytes,sizeof bytes);
}
// Count forwarding on this thread across exactly one native callback invocation.
// Aggregate route state separately; WAITING traffic is not missing BOUND traffic.
thread_local unsigned long long lateForwardSerial=0;
volatile LONG lateRouteCalls[FORWARD_STATES]{},lateRouteNoForward[FORWARD_STATES]{},lateRouteSamples=0;
static unsigned lateRouteBucket(LONG route){return route>=0&&route<(LONG)FORWARD_STATES?(unsigned)route:4;}
void lateRouteReport(){if(logger)for(unsigned i=0;i<FORWARD_STATES;i++)logger("draw_trial: route_bucket state=%u native_calls=%ld no_forward_callback=%ld\r\n",i,lateRouteCalls[i],lateRouteNoForward[i]);}
void lateRouteResult(LONG route,void*api,unsigned long long before,UINT n,ID3D11DeviceContext*c=nullptr){
 unsigned bucket=lateRouteBucket(route);InterlockedIncrement(&lateRouteCalls[bucket]);
 if(lateForwardSerial!=before)return;
 InterlockedIncrement(&lateRouteNoForward[bucket]);
 FwdRecord*r=boundRecordFor(api);
 if(route!=FORWARD_BOUND||!r||InterlockedIncrement(&lateRouteSamples)>8)return;
 void*slot=nullptr;bool readable=r->slot&&localBytes(r->slot,&slot,sizeof slot);
 BYTE bytes[5]{};bool apiReadable=localBytes(api,bytes,sizeof bytes);
 ComPtr<ID3D11Device> observedDevice;ComPtr<ID3D11Multithread> observedMulti;
 if(c){c->GetDevice(&observedDevice);c->QueryInterface(IID_PPV_ARGS(&observedMulti));}
 if(logger)logger("draw_trial: missed_context context=%p tracked_context=%p same_context=%d device=%p same_device=%d device_flags=0x%X context_type=%u protected=%d vtable=%p current_api=%p\r\n",c,context.Get(),c==context.Get(),observedDevice.Get(),observedDevice.Get()==device.Get(),observedDevice?observedDevice->GetCreationFlags():~0u,c?(unsigned)c->GetType():~0u,observedMulti?(int)observedMulti->GetMultithreadProtected():-1,c?*(void***)c:nullptr,c?(*(void***)c)[12]:nullptr);
 if(logger)logger("draw_trial: bound_no_forward count=%u api=%p record=%u entry=d3d11+0x%llX slot_readable=%d slot_owned=%d chain_valid=%d api_readable=%d prefix=%02X%02X%02X%02X%02X thread=%lu\r\n",n,api,(unsigned)(r-fwd),fwdRva(r->entry),readable,readable&&slot==r->hook.ours,recordChainFault(*r)==nullptr,apiReadable,bytes[0],bytes[1],bytes[2],bytes[3],bytes[4],GetCurrentThreadId());
}
static void forwardedIndexedAt(unsigned I,ID3D11DeviceContext*c,UINT n,UINT first,INT base,void*caller){
 FwdRecord&r=fwd[I];auto original=(Indexed)r.hook.next;
 ++lateForwardSerial;
 if(recState(r)!=REC_BOUND){original(c,n,first,base);return;}   // a LOST record not yet put back: the original only
 bool observing=InterlockedCompareExchange(&requested,0,0)==1;
 if(observing){InterlockedIncrement(&lateForwardIndexed);if(!nativeForwardDepth||!nativeForwardAvailable)InterlockedIncrement(&lateForwardScopeMiss);}
 if(!nativeForwardDepth){observedOriginal(c,(void*)original,12,n,first,base,false,[&](){original(c,n,first,base);});return;}
 if(!nativeForwardAvailable){
  // A later forwarding within the same native draw (Fixed Alpha's repair pass, an outer Blood Stains pass).
  if(InterlockedCompareExchange(&fwdAll,0,0)&&callerClass(I,caller)!=CALLER_NONE){
   InterlockedIncrement(&fwdSecondReplayed);
   intercept(c,[=](){observedOriginal(c,(void*)original,12,n,first,base,!drawObservation||fwdApi((void*)original),[&](){original(c,n,first,base);});},r.hook.next,"DrawIndexed/MGSHDFix-original-2",true,n,true,false,first,base);
   return;
  }
  InterlockedIncrement(&fwdSecondObserved);
  observedOriginal(c,(void*)original,12,n,first,base,false,[&](){original(c,n,first,base);});return;
 }
 nativeForwardAvailable=false;   // the FIRST forwarding always consumes the flag; the class only decides its replay
 if(callerClass(I,caller)!=CALLER_A){
  InterlockedIncrement(&fwdCallerUnknown);
  if(observing)InterlockedIncrement(&lateForwardReturnMiss);
  observedOriginal(c,(void*)original,12,n,first,base,false,[&](){original(c,n,first,base);});return;
 }
 bool observationAllowed=!drawObservation||(nativeForwardDepth==1&&fwdApi((void*)original));
 intercept(c,[=](){observedOriginal(c,(void*)original,12,n,first,base,!drawObservation||(observationAllowed&&fwdApi((void*)original)),[&](){original(c,n,first,base);});},r.hook.next,"DrawIndexed/MGSHDFix-original",true,n,true,false,first,base);
}
template<int I> void STDMETHODCALLTYPE forwardedIndexedN(ID3D11DeviceContext*c,UINT n,UINT first,INT base){
 forwardedIndexedAt(I,c,n,first,base,_ReturnAddress());
}
void*const forwardFunctions[FWD_RECORDS]={(void*)forwardedIndexedN<0>,(void*)forwardedIndexedN<1>,(void*)forwardedIndexedN<2>,(void*)forwardedIndexedN<3>};
bool forwardRefused(const char*reason){if(logger)logger("draw_trial: forward refused=%s\r\n",reason);return false;}
void beginForward(){

 HMODULE module=GetModuleHandleW(L"MGSHDFix.asi");
 if(!module){InterlockedExchange(&forwardModuleState,FM_DIRECT);forwardRecompute();return;}
 forwardModule=(BYTE*)module;
 HMODULE pin=nullptr;
 if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)module,&pin)||!mapModuleSections(forwardModule)){

  forwardRefused("module_pin");InterlockedExchange(&forwardModuleState,FM_PIN_FAILED);forwardRecompute();return;
 }
 InterlockedExchange(&forwardModuleState,FM_PRESENT);forwardRecompute();
 log("forward pending: MGSHDFix present; the hook chain is read once a detoured DrawIndexed entry is seen");

}
// Publication: the only part under lifetime-exclusive. Re-read and compare, choose a record, swap the slot.
bool publishForward(const ChainResult&c,unsigned attempts){
 BYTE e0=0;BYTE*q0=nullptr,*q1=nullptr,*T=nullptr;BYTE en=0;DWORD ty=0;BYTE t[16];
 if(!localBytes(c.entry,&e0,1)||e0!=0xe9||!localBytes(c.object,&q0,8)||q0!=c.entry||!localBytes(c.object+8,&q1,8)||q1!=c.callback||
    !localBytes(c.object+SH_TRAMP,&T,8)||T!=c.trampoline||!localBytes(c.object+SH_ENABLED,&en,1)||en!=1||!localBytes(c.object+SH_TYPE,&ty,4)||ty!=1||
    !localBytes(c.trampoline,t,16)||memcmp(t,c.inner,16))
  return false;   // changed between scan and publication: stability back to 0 (caller)
 int idx=-1;unsigned used=recordsUsed();
 for(unsigned i=0;i<used&&idx<0;i++){FwdRecord&r=fwd[i];
  if(recState(r)==REC_LOST&&r.entry==c.entry&&r.object==c.object&&(void**)r.slot==c.slot&&r.trampoline==c.trampoline&&!memcmp(r.inner,c.inner,16))idx=(int)i;}
 bool reuse=idx>=0;
 for(unsigned i=0;i<used&&idx<0;i++)if(recState(fwd[i])==REC_FREE)idx=(int)i;   // filled but never published: refillable
 if(idx<0){
  if(used>=FWD_RECORDS){static bool said=false;if(!said&&logger)logger("draw_trial: forward records exhausted\r\n");said=true;return false;}
  idx=(int)used;
 }
 FwdRecord&r=fwd[idx];
 if(!reuse){   // fields are written only while FREE, before writeSlot
  r.entry=c.entry;r.object=c.object;r.slot=c.slot;r.trampoline=c.trampoline;r.callback=c.callback;r.copied=c.copied;r.depth=c.depth;memcpy(r.inner,c.inner,16);
  r.hook={c.slot,c.trampoline,forwardFunctions[idx]};r.nA=r.nB=0;r.filled=true;
  if((unsigned)idx>=used)InterlockedExchange(&fwdUsed,idx+1);
 }
 if(!writeSlot(r.hook,c.trampoline,r.hook.ours)){forwardRefused("slot_publish_changed_or_failed");return false;}
 InterlockedExchange(&r.state,REC_BOUND);
 if(logger)logger("draw_trial: forward attached entry=d3d11+0x%llX object=+0x%llX slot=+0x%llX callback=+0x%llX depth=%u copied=%u record=%d attempts=%u%s\r\n",
  fwdRva(c.entry),(unsigned long long)(c.object-forwardModule),(unsigned long long)((BYTE*)c.slot-forwardModule),(unsigned long long)(c.callback-forwardModule),c.depth,c.copied,idx,attempts,reuse?" (same chain re-bound)":"");
 if(logger&&c.depth<8){char lay[8*20+1]={0};size_t at=0;
  for(unsigned i=0;i<=c.depth;i++){BYTE*l=c.layers[i];at+=(size_t)sprintf_s(lay+at,sizeof lay-at,"%s+0x%llX",i?" > ":"",l?(unsigned long long)(l-forwardModule):0ull);}
  logger("draw_trial: forward layers entry=d3d11+0x%llX outermost..innermost callbacks %s\r\n",fwdRva(c.entry),lay);}
 return true;
}

bool pinnedCandidate(ChainResult&c,const char**why){
 auto slot=(void**)(forwardModule+0x2f68f0);void*original=nullptr;ForwardRoute route;
 if(!localBytes(slot,&original,8)||!original){*why="original_slot_empty";return false;}
 if(!readForwardRoute(forwardModule,(BYTE*)original,route)){*why=forwardRouteWhy;return false;}
 c.entry=route.entry;c.object=(BYTE*)slot-SH_TRAMP;c.slot=slot;c.trampoline=(BYTE*)original;c.callback=forwardModule+0xb5150;c.copied=route.copied;c.depth=0;
 if(!localBytes(original,c.inner,16)){*why="trampoline_unreadable";return false;}
 *why="none";return true;
}
static bool logAttempt(unsigned a){return a==1||a==8||a==32||a==120||(a>120&&a%60==0);}
unsigned pinnedAttempts=0,pinnedStable=0;ChainResult pinnedCand;
// One poll pass. Scan without any lock; only publication and the state recomputation take lifetime-exclusive.
void probeForward(){
 if(stopping||!installed||InterlockedCompareExchange(&forwardModuleState,0,0)!=FM_PRESENT)return;
 static bool identityLogged=false;
 if(!identityLogged){identityLogged=true;BYTE d[32]{};bool ok=fileSha256((HMODULE)forwardModule,d);wchar_t path[MAX_PATH]{};GetModuleFileNameW((HMODULE)forwardModule,path,MAX_PATH);
  if(logger)logger("draw_trial: mgshdfix module=%ls sha256=%02X%02X%02X%02X known=%s mode=%s\r\n",path,d[0],d[1],d[2],d[3],ok?knownMgshd(d):"unreadable",InterlockedCompareExchange(&fwdModeWant,0,0)==FWD_PINNED?"pinned":"chain");}
 // The mode is latched before the first probe and never switched while a record is BOUND.
 LONG want=InterlockedCompareExchange(&fwdModeWant,0,0);
 if(!fwdModeLatched||!boundCount()){InterlockedExchange(&fwdMode,want);fwdModeLatched=true;fwdModeChangeLogged=false;}
 else if(want!=InterlockedCompareExchange(&fwdMode,0,0)&&!fwdModeChangeLogged){fwdModeChangeLogged=true;log("vr_se_fwd change ignored while bound");}
 ULONGLONG now=GetTickCount64();bool changed=false;
 if(InterlockedCompareExchange(&fwdMode,0,0)==FWD_PINNED){
  if(boundCount()||InterlockedCompareExchange(&pinnedRejected,0,0))return;
  ++pinnedAttempts;
  if(pinnedAttempts==1&&(!fileIdentity((HMODULE)forwardModule)||!sameBytes(forwardModule+0xb5150,mgshdCallbackBytes,sizeof mgshdCallbackBytes))){
   forwardRefused("pinned_identity");InterlockedExchange(&pinnedRejected,1);
   AcquireSRWLockExclusive(&lifetime);forwardRecompute();ReleaseSRWLockExclusive(&lifetime);return;}
  ChainResult c;const char*why="none";
  if(!pinnedCandidate(c,&why)){pinnedStable=0;if(logAttempt(pinnedAttempts)&&logger)logger("draw_trial: forward waiting mode=pinned why=%s attempts=%u\r\n",why,pinnedAttempts);}
  else{if(pinnedStable&&c.same(pinnedCand))++pinnedStable;else{pinnedCand=c;pinnedStable=1;}
   if(pinnedStable>=2){AcquireSRWLockExclusive(&lifetime);if(!publishForward(c,pinnedAttempts))pinnedStable=0;forwardRecompute();ReleaseSRWLockExclusive(&lifetime);}}

  if(forwardCompute()!=forwardStatus()){AcquireSRWLockExclusive(&lifetime);forwardRecompute();ReleaseSRWLockExclusive(&lifetime);}
  return;
 }
 const ChainEnv*env=&liveChainEnv;WritableSnap*snap=&liveSnap;
 bool snapped=false;
 for(unsigned i=0;i<4;i++){
  BYTE*E=(BYTE*)InterlockedCompareExchangePointer(&detouredSeen[i],nullptr,nullptr);if(!E)continue;
  if(boundRecordFor(E))continue;                     // a LOST record whose slot was put back counts as unbound
  EntryProbe&p=probes[i];
  if(p.nextTry&&now<p.nextTry)continue;              // back-off: once per 60 s after 3 equal refusals
  if(!snapped){
   takeSnap(*snap);snapped=true;
  }
  ++p.attempts;ChainResult c;const char*why="none";
  if(!walkChain(*env,*snap,E,c,&why)){
   p.stable=0;p.refusals=(p.lastWhy&&!strcmp(p.lastWhy,why))?p.refusals+1:1;p.lastWhy=why;
   bool inBackoff=InterlockedCompareExchange(&p.backoff,0,0)!=0;
   if(p.refusals>=3){p.nextTry=now+60000;if(!inBackoff){InterlockedExchange(&p.backoff,1);changed=true;}}
   if((inBackoff||logAttempt(p.attempts))&&logger)
    logger("draw_trial: forward waiting entry=d3d11+0x%llX why=%s attempts=%u%s\r\n",fwdRva(E),why,p.attempts,p.refusals>=3?" (retry every 60 s)":"");
   continue;
  }
  if(InterlockedCompareExchange(&p.backoff,0,0)){InterlockedExchange(&p.backoff,0);changed=true;}
  p.nextTry=0;p.refusals=0;p.lastWhy=nullptr;
  if(p.stable&&c.same(p.cand))++p.stable;else{p.cand=c;p.stable=1;}
  if(p.stable<1)continue;
  AcquireSRWLockExclusive(&lifetime);
  if(!publishForward(c,p.attempts))p.stable=0;
  forwardRecompute();
  ReleaseSRWLockExclusive(&lifetime);
 }
 if(changed||forwardCompute()!=forwardStatus()){AcquireSRWLockExclusive(&lifetime);forwardRecompute();ReleaseSRWLockExclusive(&lifetime);}
}
ULONGLONG forwardNextPoll=0;
void pollForward(){

 LONG m=InterlockedCompareExchange(&forwardModuleState,0,0);
 if(m!=FM_PRESENT)return;   // DIRECT, pin failure and STOPPED: no chain to read
 ULONGLONG now=GetTickCount64();if(now<forwardNextPoll)return;forwardNextPoll=now+250;
 probeForward();

}
// Marker switches, read on the worker thread before the poll: vr_se_fwd=chain|pinned, vr_se_fwd_all=on|off.
static bool markerWord(const char*buf,const char*key,const char*value){
 size_t k=strlen(key),v=strlen(value);bool hit=false;
 for(const char*q=buf;(q=strstr(q,key))!=nullptr;q+=k)
  if(q==buf||q[-1]==' '||q[-1]=='\t'||q[-1]=='\n'||q[-1]=='\r'){char e=q[k+v];hit=!strncmp(q+k,value,v)&&(e==0||e==' '||e=='\t'||e=='\r'||e=='\n');}
 return hit;
}
void forwardReadMarker(const char*buf){
 LONG mode=markerWord(buf,"vr_se_fwd=","pinned")?FWD_PINNED:FWD_CHAIN;
 LONG was=InterlockedExchange(&fwdModeWant,mode);
 if(was!=mode&&fwdModeLatched&&logger)logger("draw_trial: vr_se_fwd=%s requested\r\n",mode==FWD_PINNED?"pinned":"chain");
 LONG all=markerWord(buf,"vr_se_fwd_all=","on")?1:0;
 if(InterlockedExchange(&fwdAll,all)!=all&&logger)logger("draw_trial: vr_se_fwd_all=%s\r\n",all?"on":"off");
}
void forwardReport(){
 if(!logger)return;
 LONG s=forwardStatus();unsigned seen=0;for(auto&e:detouredSeen)if(InterlockedCompareExchangePointer(&e,nullptr,nullptr))++seen;
 logger("second_eye: forwards state=%s bound=%u/%u caller_unknown=%ld second_replayed=%ld second_observed=%ld seen_overflow=%ld mode=%s all=%s\r\n",
  forwardStateName(s),boundCount(),seen,fwdCallerUnknown,fwdSecondReplayed,fwdSecondObserved,fwdSeenOverflow,
  InterlockedCompareExchange(&fwdMode,0,0)==FWD_PINNED?"pinned":"chain",InterlockedCompareExchange(&fwdAll,0,0)?"on":"off");
}
