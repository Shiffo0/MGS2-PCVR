/* Diagnostic only. Single render-thread writer; external readers validate each
 * slot's even publication stamp twice. Fixed storage, no GPU work or I/O.
 * Fields are uint64_t to give the external reader a simple versioned ABI. */
#include <stdint.h>
#include <stddef.h>
#include <string.h>
enum { RT_FRAME=1, RT_WORLD_PROJECTIVE=2, RT_GPU_SAMPLE=3, RT_WORLD_DEPTH_CLEAR=4 };
struct ReplayTraceSlot { volatile LONG64 stamp; uint64_t v[15]; };
struct ReplayTraceBuffer {
 uint64_t magic,version,stride,capacity;
 volatile LONG64 written;
 uint64_t reserved[3];
 ReplayTraceSlot slots[8192];
};
extern "C" __declspec(dllexport) ReplayTraceBuffer dg_replay_dropout_trace = {
 0x5250545241434531ULL,1,sizeof(ReplayTraceSlot),8192,0,{} ,{}
};
static_assert(sizeof(ReplayTraceSlot)==128,"trace slot ABI");
static_assert(offsetof(ReplayTraceBuffer,slots)==64,"trace header ABI");
static uint64_t rtCounts[5]{},rtSuppressed=0;
static void rtWrite(uint64_t kind,uint64_t epoch,uint64_t frame,int eye,int sequence,
 uint64_t shader,uint64_t n,uint64_t first,int64_t base,uint64_t detail0,uint64_t detail1) {
 auto &b=dg_replay_dropout_trace;
 LONG64 id=b.written+1;
 auto &s=b.slots[(id-1)%b.capacity];
 InterlockedExchange64(&s.stamp,id*2-1);
 uint64_t v[15]={kind,epoch,frame,(uint64_t)(int64_t)eye,(uint64_t)(int64_t)sequence,
  GetTickCount64(),shader,n,first,(uint64_t)base,detail0,detail1,0,0,0};
 memcpy(s.v,v,sizeof(v));
 InterlockedExchange64(&s.stamp,id*2);
 InterlockedExchange64(&b.written,id);
}
static void rtEvent(uint64_t kind,uint64_t epoch,uint64_t frame,int eye,int sequence,
 uint64_t shader=0,uint64_t n=0,uint64_t first=0,int64_t base=0,uint64_t d0=0,uint64_t d1=0) {
 if(kind<2||kind>4)return;
 if(++rtCounts[kind]>4){++rtSuppressed;return;}
 rtWrite(kind,epoch,frame,eye,sequence,shader,n,first,base,d0,d1);
}
static void rtFinish(uint64_t epoch,uint64_t frame,int eye,int sequence,uint64_t state) {
 rtWrite(RT_FRAME,epoch,frame,eye,sequence,0,rtCounts[2],rtCounts[3],rtCounts[4],rtSuppressed,state);
 memset(rtCounts,0,sizeof(rtCounts));rtSuppressed=0;
}
