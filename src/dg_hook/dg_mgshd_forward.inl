// Version-specific adapter. The native wrapper executes once, without our locks.
// Only its synchronous original-call forwarding may claim the one-shot trial.
#include "dg_mgshd_signature.h"
enum ForwardState { FORWARD_DIRECT, FORWARD_WAITING, FORWARD_BOUND, FORWARD_REJECTED, FORWARD_STOPPED };
volatile LONG forwardState=FORWARD_DIRECT;
unsigned forwardAttempts=0,forwardStable=0,forwardCopied=0;
ULONGLONG forwardNextPoll=0;
void*forwardCandidate=nullptr;
const char*forwardLastReason="not_polled";
const char*forwardRouteWhy="none";
Hook forwardHook;
LONG forwardStatus(){return InterlockedCompareExchange(&forwardState,0,0);}
void setForwardStatus(LONG value){InterlockedExchange(&forwardState,value);}
BYTE*forwardModule=nullptr;BYTE*forwardEntry=nullptr;BYTE*forwardRelay=nullptr;
BYTE*forwardOriginal=nullptr;void*forwardReturn=nullptr;
thread_local unsigned nativeForwardDepth=0;
thread_local bool nativeForwardAvailable=false;
struct NativeForwardScope{
 NativeForwardScope(){if(nativeForwardDepth++==0)nativeForwardAvailable=true;}
 ~NativeForwardScope(){if(--nativeForwardDepth==0)nativeForwardAvailable=false;}
};
bool localBytes(void*p,void*out,SIZE_T n){SIZE_T got=0;return ReadProcessMemory(GetCurrentProcess(),p,out,n,&got)&&got==n;}
bool sameBytes(void*p,const void*expected,SIZE_T n){
 BYTE b[sizeof(mgshdCallbackBytes)];return n<=sizeof b&&localBytes(p,b,n)&&!memcmp(b,expected,n);
}


















// Decode only the two live-verified trampoline prologues. The original
// trampoline determines the detoured entry, never a transient context vtable.
bool decodeForward(BYTE*original,BYTE*&entry,unsigned&copied){
 BYTE bytes[12];if(!localBytes(original,bytes,sizeof bytes))return false;
 const BYTE seven[]={0x48,0x8b,0xc4,0x48,0x89,0x58,0x10};
 const BYTE five[]={0x48,0x89,0x5c,0x24,0x10};
 if(!memcmp(bytes,seven,7)&&bytes[7]==0xe9)copied=7;
 else if(!memcmp(bytes,five,5)&&bytes[5]==0xe9)copied=5;
 else return false;
 INT32 offset;memcpy(&offset,bytes+copied+1,4);
 uintptr_t resume=(uintptr_t)original+copied+5+(intptr_t)offset;
 if(resume<copied)return false;entry=(BYTE*)(resume-copied);return true;
}
struct ForwardRoute{BYTE*entry=nullptr;BYTE*relay=nullptr;unsigned copied=0;};
ForwardRoute forwardCandidateRoute;



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
bool forwardChain(){



 ForwardRoute route;
 return readForwardRoute(forwardModule,forwardOriginal,route)&&route.entry==forwardEntry&&route.relay==forwardRelay&&route.copied==forwardCopied;
}
bool forwardOwned(){return forwardStatus()!=FORWARD_BOUND||(*forwardHook.slot==forwardHook.ours&&forwardChain());}
bool forwardApi(void*api){return forwardStatus()==FORWARD_BOUND&&api==forwardHook.next&&forwardOwned();}
#include "dg_initialized_signature.h"
/* Called for every indexed draw: the d3d11 base is looked up once (the module is
   pinned by the process for its lifetime). */
bool initializedAddress(void*api){static HMODULE m=GetModuleHandleW(L"d3d11.dll");return m&&(api==(BYTE*)m+0x154260||api==(BYTE*)m+initializedRvaKb5124010);}
bool initializedBody(const BYTE*b,size_t n){return n==sizeof initializedBytes&&(!memcmp(b,initializedBytes,n)||!memcmp(b,initializedBytesKb5124010,n));}
bool initializedApi(ID3D11DeviceContext*c,void*api){
 if(!initializedAddress(api)||c!=context.Get()||c->GetType()!=D3D11_DEVICE_CONTEXT_IMMEDIATE||(*(void***)c)[12]!=api)return false;
 BYTE bytes[sizeof initializedBytes]{};SIZE_T got=0;
 return ReadProcessMemory(GetCurrentProcess(),api,bytes,sizeof bytes,&got)&&got==sizeof bytes&&initializedBody(bytes,sizeof bytes);
}
void restoreForward(){if(forwardHook.slot&&*forwardHook.slot==forwardHook.ours)writeSlot(forwardHook,forwardHook.ours,forwardHook.next);}

// Count forwarding on this thread across exactly one native callback invocation.
// Aggregate route state separately; WAITING traffic is not missing BOUND traffic.
thread_local unsigned long long lateForwardSerial=0;
volatile LONG lateRouteCalls[5]{},lateRouteNoForward[5]{},lateRouteSamples=0;
































// All fields below are written under lifetime-exclusive before publishing the
// pointer hook. Native entries consult only the atomic state until publication.
void beginForward(){














}














































