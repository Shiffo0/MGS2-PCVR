/* Experimental native-render observation. No second render is invoked here.
 * Shares B78 phase hooks; optional return mode temporarily redirects DR3
 * on the consumer thread and restores its entry address at return.
 * Fixed storage; VEH does no I/O, allocation, or blocking lock acquisition.
 * Include once in dg_hook.c (and in the standalone fixture).
 */
#ifndef DG_PAIR_PROBE_H
#define DG_PAIR_PROBE_H
#include "dg_build_profile.h"
#include <stdint.h>
#include <io.h>
#include <fcntl.h>
#define PP_CAPACITY DG_DIAGNOSTIC_CAPACITY(16384)
#define PP_STAGE_CALL 0x89DFDull
#define PP_STAGE_RETURN 0x89E00ull
#define PP_CONSUMER_CALL 0xFBF2ull
#define PP_CONSUMER_RETURN 0xFBF7ull
#define PP_DR_MASK (0xF0ull | (0xFFull << 24))
enum { PP_OFF, PP_RECORDING, PP_DONE, PP_INVALID };
enum { PP_STAGE_IN=1, PP_STAGE_OUT, PP_CONSUMER_IN,
       PP_CONSUMER_OUT, PP_PRESENT };
typedef struct {
    LONGLONG qpc;
    DWORD thread;
    unsigned event, present, stage, which, phase;
    ULONG64 channel, target, table, start, end, cursor, head, tail;
} PP_ROW;
static SRWLOCK pp_lock=SRWLOCK_INIT;
static PP_ROW pp_rows[PP_CAPACITY];
static unsigned pp_count;
static volatile LONG pp_status, pp_present_id, pp_started, pp_dumped;
static ULONG64 pp_base;
static DWORD pp_begin_ms;
static const char *pp_failure;
static char pp_output[MAX_PATH];
static char pp_request_marker[MAX_PATH], pp_request_output[MAX_PATH];
static ULONG64 pp_request_base;
static int pp_attempted,pp_return_mode;
static volatile LONG pp_pending_tid;
static ULONG64 pp_pending_rsp;
static unsigned pp_pending_which;
/* Only the measured same-thread call site is eligible. Unknown callers fail closed. */
static int pp_pending_context(const CONTEXT *c) {
return 0;
}

static void pp_fail(const char *reason) {
    if (pp_status==PP_RECORDING) {
        InterlockedCompareExchangePointer((void *volatile *)&pp_failure,
                                          (void *)reason,NULL);
        InterlockedCompareExchange(&pp_status,PP_INVALID,PP_RECORDING);
    }
}
static int pp_marker(const char *text) {
    const char *word="observe";
    if (!text) return 0;
    while (*text==' ' || *text=='\t' || *text=='\r' || *text=='\n') ++text;
    while (*word) if (*text++ != *word++) return 0;
    while (*text==' ' || *text=='\t' || *text=='\r' || *text=='\n') ++text;
    return !*text;
}
static int pp_signature(ULONG64 base) {
    static const unsigned char stage[]={0xff,0x14,0x30,0xff,0xc7,0x48,0x8d,
        0x76,0x08,0x3b,0xbb,0x18,0x03,0x00,0x00,0x7c,0xe2};
    static const unsigned char consumer[]={0x40,0x55,0x41,0x54,0x41,0x56,0x41,0x57,
        0x48,0x8d,0x6c,0x24,0xc1,0x48,0x81,0xec,0xa8,0,0,0,0xe8,7,0x8f,1,0};
    __try {
        return !memcmp((void *)(base+PP_STAGE_CALL),stage,sizeof stage) &&
            !memcmp((void *)(base+0xDB30ull),consumer,sizeof consumer);
    } __except(EXCEPTION_EXECUTE_HANDLER) { return 0; }
}
static int cq_mode,cq_used,cq_pending,cp_mode;
static void cp_prepare(void);
static void pp_begin(ULONG64 base, const char *marker, const char *output,
                     void (*log)(const char *,...)) {

}
static void pp_prepare(ULONG64 base,const char *marker,const char *output) {

}
/* Worker poll: the owner can request capture after reaching gameplay.
   One request attempt per session, including a malformed/refused request. */
static int pp_poll(void (*log)(const char *,...)) {
return 0;
}
static int pp_store(PP_ROW *row) {
    LARGE_INTEGER now;
    if (pp_status!=PP_RECORDING) return 0;
    QueryPerformanceCounter(&now); row->qpc=now.QuadPart;
    row->thread=GetCurrentThreadId();
    row->present=(unsigned)InterlockedCompareExchange(&pp_present_id,0,0);
    if (!TryAcquireSRWLockExclusive(&pp_lock)) {
        pp_fail("concurrent_writer_event_loss"); return 0;
    }
    if (pp_status!=PP_RECORDING) { ReleaseSRWLockExclusive(&pp_lock); return 0; }
    if (pp_count>=PP_CAPACITY) {
        pp_fail("event_overflow"); ReleaseSRWLockExclusive(&pp_lock); return 0;
    }
    pp_rows[pp_count++]=*row;
    InterlockedExchange(&pp_started,1);
    ReleaseSRWLockExclusive(&pp_lock);
    return 1;
}
static int pp_native(PP_ROW *r) {
    ULONG64 rec;
    static const unsigned stage_rvas[19]={0x996f0,0x85480,0x99640,0x99eb0,
        0x9e060,0x97a70,0x9c4a0,0x9e070,0x97a80,0x892e0,0x91bb0,0x928f0,
        0x99640,0x96690,0x9cd80,0x99640,0x99640,0x996e0,0x3930};
    __try {
        unsigned i, count;
        if (r->which>1) return 0;
        r->phase=*(unsigned *)(pp_base+0x16511e0);
        if (r->channel) {
            if (r->channel<pp_base+0x15522a0 ||
                r->channel>=pp_base+0x15522a0+5*0x530 ||
                (r->channel-pp_base-0x15522a0)%0x530) return 0;
            count=*(unsigned *)(r->channel+0x318);
            r->table=*(ULONG64 *)(r->channel+0x320);
            if (!count || count>64 || r->stage>=count) return 0;
            if (r->channel==pp_base+0x15522a0) {
                if (count!=19 || r->table!=pp_base+0x9481c0) return 0;
                for (i=0;i<19;++i)
                    if (((ULONG64 *)r->table)[i]!=pp_base+stage_rvas[i]) return 0;
            }
            r->target=((ULONG64 *)r->table)[r->stage];
        }
        rec=pp_base+0xcd4e20+r->which*0x28;
        r->start=*(ULONG64 *)(rec); r->end=*(ULONG64 *)(rec+8);
        r->cursor=*(ULONG64 *)(rec+16); r->head=*(ULONG64 *)(rec+24);
        r->tail=*(ULONG64 *)(rec+32);
        if (!r->start || r->start>r->cursor || r->cursor>r->end) return 0;
        return 1;
    } __except(EXCEPTION_EXECUTE_HANDLER) { return 0; }
}
#include "dg_command_census.inl"
/* Called from phase_context AFTER the existing render-link event. The
 * host owns the execution trap; this observer must not consume/change it. */
static void pp_shared_event(unsigned bit,const CONTEXT *c) {

}
static void pp_return_arm(CONTEXT *c) {

}
static int pp_return_context(CONTEXT *c) {
return 0;
}
static void pp_present(void) {

}
/* Worker only. Stop publication before taking a blocking lock or doing I/O. */
static void pp_flush(int finish,void (*log)(const char *,...)) {

}
#endif
