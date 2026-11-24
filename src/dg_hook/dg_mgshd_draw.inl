// Observe the original call inside the exact MGSHDFix Draw wrapper. Never
// acquire lifetime or D3D locks around that external wrapper's mutex.
#include "dg_mgshd_draw_signature.h"
#include "dg_mgshd_nested_signature.h"
#include "dg_mgshd_protected_draw_signature.h"
bool mgDrawNested=false;
bool mgDrawCallback(BYTE*module);
bool mgDrawCaller(BYTE*module,void*caller){return mgDrawNested?(caller==module+0x13411d||caller==module+0x1341a7):caller==module+0xa2120;}
bool decodeNestedOriginal(BYTE*original,BYTE*&entry,unsigned&copied){
 BYTE b[16];if(!localBytes(original,b,sizeof b))return false;
 if(!memcmp(b,mgNestedEntryBytes,11)&&b[11]==0xe9)copied=11;
 else if(!memcmp(b,mgProtectedDrawBytes,5)&&b[5]==0xe9)copied=5;
 else return false;
 INT32 rel;memcpy(&rel,b+copied+1,4);entry=(BYTE*)((uintptr_t)original+copied+5+(intptr_t)rel-copied);return true;
}
bool decodeOuterJump(BYTE*original,BYTE*&nestedRelay,BYTE*&resume){
 BYTE b[10];if(!localBytes(original,b,sizeof b)||b[0]!=0xe9||b[5]!=0xe9)return false;
 INT32 rel;memcpy(&rel,b+1,4);nestedRelay=(BYTE*)((uintptr_t)original+5+(intptr_t)rel);
 memcpy(&rel,b+6,4);resume=(BYTE*)((uintptr_t)original+10+(intptr_t)rel);return true;
}
bool readNestedDrawRouteAt(BYTE*module,BYTE*original,BYTE*runtime,ForwardRoute&route,const char**reason=nullptr){
 auto fail=[&](const char*s){if(reason)*reason=s;return false;};
 if(reason)*reason="validated_nested";
 BYTE body[sizeof mgNestedBytes];if(!localBytes(module+0x134030,body,sizeof body)||memcmp(body,mgNestedBytes,sizeof body))return fail("nested_callback_bytes");
 if(!decodeNestedOriginal(original,route.entry,route.copied))return fail("nested_original_prologue");
 bool protectedEntry=route.copied==5;
 auto expected=protectedEntry?mgProtectedDrawBytes:mgNestedEntryBytes;
 size_t length=protectedEntry?sizeof mgProtectedDrawBytes:sizeof mgNestedEntryBytes;
 if(!runtime||route.entry!=runtime+(protectedEntry?0x1543e0:0x154540))return fail("nested_entry_address");
 BYTE tail[sizeof mgProtectedDrawBytes];size_t tailSize=length-route.copied;
 if(!localBytes(route.entry+route.copied,tail,tailSize)||memcmp(tail,expected+route.copied,tailSize))return fail("nested_entry_body");
 BYTE detour[5],relay[14];if(!localBytes(route.entry,detour,5)||detour[0]!=0xe9)return fail("nested_entry_detour");
 INT32 rel;memcpy(&rel,detour+1,4);route.relay=route.entry+5+rel;
 const BYTE indirect[]={0xff,0x25,0,0,0,0};
 if(!localBytes(route.relay,relay,14)||memcmp(relay,indirect,6))return fail("nested_relay");
 void*callback=nullptr;memcpy(&callback,relay+6,8);
 // Both wrappers detour the same D3D entry. Its current relay belongs to the
 // outer wrapper; that wrapper's saved relocated jump reaches the inner one.
 if(callback!=module+0xa2090)return fail("nested_outer_relay_callback");
 if(!mgDrawCallback(module))return fail("nested_outer_callback_bytes");
 BYTE*outer=nullptr;BYTE*innerRelay=nullptr;BYTE*resume=nullptr;
 if(!localBytes(module+0x2f6060,&outer,8)||!outer||!decodeOuterJump(outer,innerRelay,resume)||resume!=route.entry+5)return fail("nested_outer_original");
 if(!localBytes(innerRelay,relay,14)||memcmp(relay,indirect,6))return fail("nested_inner_relay");
 memcpy(&callback,relay+6,8);if(callback!=module+0x134030)return fail("nested_inner_callback");
 return true;
}
bool readNestedDrawRoute(BYTE*module,BYTE*original,ForwardRoute&route,const char**reason=nullptr){
 return readNestedDrawRouteAt(module,original,(BYTE*)GetModuleHandleW(L"d3d11.dll"),route,reason);
}
Hook mgDrawHook;
BYTE*mgDrawModule=nullptr;
ForwardRoute mgDrawRoute;
unsigned mgDrawAttempts=0,mgDrawStable=0;
void*mgDrawCandidate=nullptr;
ULONGLONG mgDrawNextPoll=0;
bool mgDrawDone=false;
thread_local unsigned nativePlainDepth=0;
thread_local bool nativePlainAvailable=false;
struct NativePlainScope{
 NativePlainScope(){if(nativePlainDepth++==0)nativePlainAvailable=true;}
 ~NativePlainScope(){if(--nativePlainDepth==0)nativePlainAvailable=false;}
};
bool mgDrawCallback(BYTE*module){BYTE bytes[sizeof mgDrawBytes];return localBytes(module+0xa2090,bytes,sizeof bytes)&&!memcmp(bytes,mgDrawBytes,sizeof bytes);}
bool readMgDrawRoute(BYTE*module,BYTE*original,ForwardRoute&route,const char**reason=nullptr){
 auto fail=[&](const char*why){if(reason)*reason=why;return false;};
 if(reason)*reason="validated";
 if(!mgDrawCallback(module))return fail("callback_bytes");
 if(!decodeForward(original,route.entry,route.copied))return fail("original_prologue");
 HMODULE owner=nullptr;
 if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,(LPCWSTR)route.entry,&owner)||owner!=GetModuleHandleW(L"d3d11.dll"))return fail("entry_owner");
 BYTE entry[5],relay[14];if(!localBytes(route.entry,entry,5))return fail("entry_unreadable");
 if(entry[0]!=0xe9)return fail("entry_not_detoured");
 INT32 offset;memcpy(&offset,entry+1,4);route.relay=route.entry+5+offset;
 const BYTE indirect[]={0xff,0x25,0,0,0,0};
 if(!localBytes(route.relay,relay,14))return fail("relay_unreadable");
 if(memcmp(relay,indirect,6))return fail("relay_opcode");
 void*callback=nullptr;memcpy(&callback,relay+6,8);
 if(callback!=module+0xa2090)return fail("relay_callback");
 if(!runtimePointer(route.entry+route.copied))return fail("resume_owner");
 return true;
}
bool mgDrawRead(BYTE*module,BYTE*original,ForwardRoute&route,const char**reason=nullptr){
 return mgDrawNested?readNestedDrawRoute(module,original,route,reason):readMgDrawRoute(module,original,route,reason);
}
bool mgDrawReportAttempt(unsigned attempt){return attempt==1||attempt==8||attempt==32||attempt==120;}
void mgDrawReport(const char*reason,void*original,const ForwardRoute&route){
 if(!logger||!mgDrawReportAttempt(mgDrawAttempts))return;
 BYTE bytes[12]{};bool readable=original&&localBytes(original,bytes,sizeof bytes);
 char hex[25]{};for(unsigned i=0;i<sizeof bytes;i++)sprintf_s(hex+i*2,3,"%02X",bytes[i]);
 auto runtime=(BYTE*)GetModuleHandleW(L"d3d11.dll");
 logger("draw_trial: mg_draw waiting attempt=%u reason=%s original=%p readable=%d bytes=%s entry=%p copied=%u relay=%p runtime=%p entry_rva=0x%llX; extra_draws=0\r\n",mgDrawAttempts,reason,original,readable,hex,route.entry,route.copied,route.relay,runtime,(unsigned long long)(route.entry&&runtime?(uintptr_t)route.entry-(uintptr_t)runtime:0));
}
bool mgDrawApi(void*api){
 if(!mgDrawHook.slot||api!=mgDrawHook.next||*mgDrawHook.slot!=mgDrawHook.ours)return false;
 ForwardRoute r;return mgDrawRead(mgDrawModule,(BYTE*)api,r)&&r.entry==mgDrawRoute.entry&&r.relay==mgDrawRoute.relay&&r.copied==mgDrawRoute.copied;
}
void mgDrawObserve(ID3D11DeviceContext*c,UINT n,UINT first,void*caller){
 auto original=(Draw)mgDrawHook.next;
 if(latePlainDraw && requested==1 && !stopping && c==context.Get() && nativePlainDepth && nativePlainAvailable && mgDrawCaller(mgDrawModule,caller)){
  nativePlainAvailable=false;
  bool observationAllowed=!drawObservation||(nativePlainDepth==1&&mgDrawApi((void*)original));
  intercept(c,[=](){observedOriginal(c,(void*)original,13,n,first,0,!drawObservation||(observationAllowed&&mgDrawApi((void*)original)),[&](){original(c,n,first);});},(void*)original,"Draw/MGSHDFix-original",false,n,false,false,first,0,true);
  return;
 }
 // The saved original remains immutable even after unhooking.
 if(drawObservation)drawObservation->invalid=true; // Unselected adapter work cannot satisfy an armed observation.
 if(mgDrawNested){original(c,n,first);return;} // Existing blit observer stays on its original outer route.
 if(stopping||c!=context.Get()||caller!=mgDrawModule+0xa2120||(!bpWanted()&&!InterlockedCompareExchange(&ww.window,0,0))||!mgDrawApi((void*)original)){original(c,n,first);return;}
 int ticket=wwBegin(1,c,nullptr,nullptr,(void*)original,caller,n,first);
 blitForward(c,n,first,(void*)original,[=](){original(c,n,first);},true);
 wwEnd(ticket);
}
void STDMETHODCALLTYPE mgDrawOriginal(ID3D11DeviceContext*c,UINT n,UINT first){mgDrawObserve(c,n,first,_ReturnAddress());}
void restoreMgDraw(){if(mgDrawHook.slot&&*mgDrawHook.slot==mgDrawHook.ours)writeSlot(mgDrawHook,mgDrawHook.ours,mgDrawHook.next);}
void pollMgDraw(bool nestedRequested=false){
 if(mgDrawDone||stopping||!installed)return;
 ULONGLONG now=GetTickCount64();if(now<mgDrawNextPoll)return;mgDrawNextPoll=now+250;
 AcquireSRWLockExclusive(&lifetime);
 struct Unlock{~Unlock(){ReleaseSRWLockExclusive(&lifetime);}}unlock;
 if(stopping)return;
 if(!mgDrawModule){
  HMODULE module=GetModuleHandleW(L"MGSHDFix.asi");if(!module)return;
  HMODULE pin=nullptr;
  if(!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_PIN,(LPCWSTR)module,&pin)||!fileIdentity(module)||!mgDrawCallback((BYTE*)module)){
   mgDrawDone=true;log("mg_draw refused=module_identity_or_callback");return;
  }
  mgDrawModule=(BYTE*)module;
  mgDrawNested=nestedRequested;
 }
 ++mgDrawAttempts;void*original=nullptr;ForwardRoute route;
 auto slot=(void**)(mgDrawModule+(mgDrawNested?0x2fa6a0:0x2f6060));
 const char*reason="slot_unreadable";
 bool slotRead=localBytes(slot,&original,8);
 if(slotRead)reason=original?"unvalidated":"slot_null";
 if(slotRead&&original&&mgDrawRead(mgDrawModule,(BYTE*)original,route,&reason)){
  if(original==mgDrawCandidate&&route.entry==mgDrawRoute.entry&&route.relay==mgDrawRoute.relay&&route.copied==mgDrawRoute.copied)++mgDrawStable;
  else{mgDrawCandidate=original;mgDrawRoute=route;mgDrawStable=1;}
  if(mgDrawStable>=2){
   mgDrawHook={slot,original,(void*)mgDrawOriginal};mgDrawDone=true;
   if(!writeSlot(mgDrawHook,original,mgDrawHook.ours)){mgDrawHook.slot=nullptr;log("mg_draw refused=slot_changed_or_publish_failed");return;}
   if(logger)logger("draw_trial: mg_draw attached original_slot=%X callback=%X copied=%u attempts=%u; original once\r\n",mgDrawNested?0x2fa6a0:0x2f6060,mgDrawNested?0x134030:0xa2090,route.copied,mgDrawAttempts);
   return;
  }
 }else mgDrawStable=0;
 mgDrawReport(reason,original,route);
 if(mgDrawAttempts>=120){mgDrawDone=true;log("mg_draw refused=chain_timeout_or_unsupported");}
}
