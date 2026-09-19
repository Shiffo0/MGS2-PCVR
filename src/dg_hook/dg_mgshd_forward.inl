// Version-specific adapter. The native wrapper executes once, without our locks.
// Only its synchronous original-call forwarding may claim the one-shot trial.
#include "dg_mgshd_signature.h"
enum ForwardState { FORWARD_DIRECT, FORWARD_WAITING, FORWARD_BOUND, FORWARD_REJECTED, FORWARD_STOPPED };
volatile LONG forwardState=FORWARD_DIRECT;
unsigned forwardAttempts=0,forwardStable=0,forwardCopied=0;
ULONGLONG forwardNextPoll=0;
void*forwardCandidate=nullptr;
const char*forwardLastReason="not_polled";
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
bool fileIdentity(HMODULE module){
 wchar_t path[32768];DWORD n=GetModuleFileNameW(module,path,32768);if(!n||n>=32768)return false;
 HANDLE file=CreateFileW(path,GENERIC_READ,FILE_SHARE_READ,nullptr,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,nullptr);
 if(file==INVALID_HANDLE_VALUE)return false;
 BCRYPT_ALG_HANDLE alg=nullptr;BCRYPT_HASH_HANDLE hash=nullptr;BYTE digest[32],buf[65536];
 bool ok=BCryptOpenAlgorithmProvider(&alg,BCRYPT_SHA256_ALGORITHM,nullptr,0)>=0;
 if(ok)ok=BCryptCreateHash(alg,&hash,nullptr,0,nullptr,0,0)>=0;
 DWORD got=0;
 while(ok){if(!ReadFile(file,buf,sizeof buf,&got,nullptr)){ok=false;break;}if(!got)break;ok=BCryptHashData(hash,buf,got,0)>=0;}
 if(ok)ok=BCryptFinishHash(hash,digest,sizeof digest,0)>=0;
 const BYTE expected[]={0xbd,0xbf,0xbd,0xc8,0xff,0xf2,0xb3,0x18,0xc3,0x59,0x7d,0xb9,0xc6,0x9d,0xb6,0xc5,0x83,0xed,0x8b,0x75,0x04,0x68,0x55,0x05,0xf3,0x11,0x2d,0x66,0x76,0x35,0x6c,0xc3};
 // Full SHA256 BDBFBDC8FFF2B318C3597DB9C69DB6C583ED8B7504685505F3112D6676356CC3.
 if(ok)ok=!memcmp(digest,expected,32);
 if(hash)BCryptDestroyHash(hash);if(alg)BCryptCloseAlgorithmProvider(alg,0);CloseHandle(file);return ok;
}
#ifdef DG_DRAW_TRIAL_TEST
bool testForwardChain=false;
#endif
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
#ifdef DG_DRAW_TRIAL_TEST
bool (*testRouteReader)(BYTE*,BYTE*,ForwardRoute&)=nullptr;
#endif
bool readForwardRoute(BYTE*module,BYTE*original,ForwardRoute&route){
#ifdef DG_DRAW_TRIAL_TEST
 if(testRouteReader)return testRouteReader(module,original,route);
#endif
 if(!decodeForward(original,route.entry,route.copied))return false;
 HMODULE owner=nullptr;
 if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCWSTR)route.entry,&owner)||owner!=GetModuleHandleW(L"d3d11.dll"))return false;
 BYTE entry[5],relay[14];if(!localBytes(route.entry,entry,sizeof entry)||entry[0]!=0xe9)return false;
 INT32 offset;memcpy(&offset,entry+1,4);route.relay=(BYTE*)((uintptr_t)route.entry+5+(intptr_t)offset);
 if(!localBytes(route.relay,relay,sizeof relay))return false;
 const BYTE indirect[]={0xff,0x25,0,0,0,0};if(memcmp(relay,indirect,6))return false;
 void*callback=nullptr;memcpy(&callback,relay+6,8);
 if(callback!=module+0xb5150||!sameBytes(callback,mgshdCallbackBytes,sizeof mgshdCallbackBytes))return false;
 return runtimePointer(route.entry+route.copied);
}
bool forwardChain(){
#ifdef DG_DRAW_TRIAL_TEST
 if(testForwardChain)return true;
#endif
 ForwardRoute route;
 return readForwardRoute(forwardModule,forwardOriginal,route)&&route.entry==forwardEntry&&route.relay==forwardRelay&&route.copied==forwardCopied;
}
bool forwardOwned(){return forwardStatus()!=FORWARD_BOUND||(*forwardHook.slot==forwardHook.ours&&forwardChain());}
bool forwardApi(void*api){return forwardStatus()==FORWARD_BOUND&&api==forwardHook.next&&forwardOwned();}
void restoreForward(){if(forwardHook.slot&&*forwardHook.slot==forwardHook.ours)writeSlot(forwardHook,forwardHook.ours,forwardHook.next);}
void STDMETHODCALLTYPE forwardedIndexed(ID3D11DeviceContext*c,UINT n,UINT first,INT base){
 auto original=(Indexed)forwardHook.next;
 if(!nativeForwardDepth||!nativeForwardAvailable||_ReturnAddress()!=forwardReturn){original(c,n,first,base);return;}
 nativeForwardAvailable=false; // Only the first original forwarding, never post-wrapper work.
 intercept(c,[=](){original(c,n,first,base);},forwardHook.next,"DrawIndexed/MGSHDFix-original",true);
}
bool forwardRefused(const char*reason){if(logger)logger("draw_trial: forward refused=%s\r\n",reason);return false;}
// All fields below are written under lifetime-exclusive before publishing the
// pointer hook. Native entries consult only the atomic state until publication.
void beginForward(){
 HMODULE module=GetModuleHandleW(L"MGSHDFix.asi");
 if(!module){setForwardStatus(FORWARD_DIRECT);return;}
 forwardModule=(BYTE*)module;
 HMODULE pin=nullptr;
 if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)module,&pin)){
  forwardRefused("module_pin");setForwardStatus(FORWARD_REJECTED);return;
 }
 setForwardStatus(FORWARD_WAITING);log("forward pending: waiting for validated MGSHDFix original slot");
}
void rejectForward(const char*reason){forwardRefused(reason);setForwardStatus(FORWARD_REJECTED);}
void probeForward(){
 // Caller owns lifetime-exclusive. No external mutex, D3D API or code patch.
 if(forwardStatus()!=FORWARD_WAITING||stopping||!installed)return;
 ++forwardAttempts;
 if(forwardAttempts==1){
  if(!fileIdentity((HMODULE)forwardModule)){rejectForward("file_identity");return;}
  if(!sameBytes(forwardModule+0xb5150,mgshdCallbackBytes,sizeof mgshdCallbackBytes)){rejectForward("callback_bytes");return;}
 }
 auto slot=(void**)(forwardModule+0x2f68f0);void*original=nullptr;ForwardRoute route;
 bool readable=localBytes(slot,&original,8);
 if(!readable||!original){forwardLastReason=readable?"original_slot_empty":"original_slot_unreadable";forwardStable=0;}
 else if(!readForwardRoute(forwardModule,(BYTE*)original,route)){forwardLastReason="chain_not_ready_or_unsupported";forwardStable=0;}
 else {
  if(original!=forwardCandidate||route.entry!=forwardCandidateRoute.entry||route.relay!=forwardCandidateRoute.relay||route.copied!=forwardCandidateRoute.copied){forwardCandidate=original;forwardCandidateRoute=route;forwardStable=1;}
  else ++forwardStable;
  if(forwardStable>=2){
   forwardOriginal=(BYTE*)original;forwardEntry=route.entry;forwardRelay=route.relay;forwardCopied=route.copied;
   forwardReturn=forwardModule+0xb51e3;forwardHook={slot,original,(void*)forwardedIndexed};
   if(!writeSlot(forwardHook,original,forwardHook.ours)){rejectForward("slot_publish_changed_or_failed");return;}
   setForwardStatus(FORWARD_BOUND);
   if(logger)logger("draw_trial: forward attached: deferred MGSHDFix original; copied=%u attempts=%u entry=%p; wrapper once\r\n",forwardCopied,forwardAttempts,forwardEntry);
   return;
  }
  forwardLastReason="stabilizing";
 }
 if(forwardAttempts>=120){if(logger)logger("draw_trial: forward timeout last=%s attempts=%u\r\n",forwardLastReason,forwardAttempts);setForwardStatus(FORWARD_REJECTED);}
}
void pollForward(){
 if(forwardStatus()!=FORWARD_WAITING)return;
 ULONGLONG now=GetTickCount64();if(now<forwardNextPoll)return;forwardNextPoll=now+250;
 AcquireSRWLockExclusive(&lifetime);probeForward();ReleaseSRWLockExclusive(&lifetime);
}
