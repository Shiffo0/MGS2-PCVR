/* Experimental native-render observation. No second render is invoked here.
 * Shares B78 phase hooks; optional return mode temporarily redirects DR3
 * on the consumer thread and restores its entry address at return.
 * Fixed storage; VEH does no I/O, allocation, or blocking lock acquisition.
 * Include once in dg_hook.c (and in the standalone fixture).
 */
#ifndef DG_PAIR_PROBE_H
#define DG_PAIR_PROBE_H
#include <stdint.h>
#include <io.h>
#include <fcntl.h>
#define PP_CAPACITY 16384
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
    return pp_pending_tid==(LONG)GetCurrentThreadId() &&
        c->Dr3==pp_base+PP_CONSUMER_RETURN;
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
static void pp_begin(ULONG64 base, const char *marker, const char *output,
                     void (*log)(const char *,...)) {
    FILE *f=NULL;
    char text[64]={0};
    size_t n;
    /* Called only between sessions, after the old debug hooks are drained. */
    pp_base=0; pp_count=0; pp_status=PP_OFF; pp_present_id=0;
    pp_started=0; pp_dumped=0; pp_failure=NULL;
    if (fopen_s(&f,marker,"rb") || !f) return;
    n=fread(text,1,sizeof(text)-1,f);
    if (ferror(f) || !feof(f) || n==sizeof(text)-1 || strlen(text)!=n) text[0]=0;
    fclose(f);
    pp_return_mode=!strcmp(text,"observe_return");pp_pending_tid=0;
    if(pp_return_mode)strcpy_s(text,sizeof text,"observe");
    if (!pp_marker(text)) {
        log("  pair probe: refused marker; exact value observe or observe_return required\r\n");
        return;
    }
    if (!pp_signature(base)) {
        log("  pair probe: refused native signature mismatch; AFR unchanged\r\n");
        return;
    }
    strcpy_s(pp_output,sizeof pp_output,output);
    pp_base=base; pp_begin_ms=GetTickCount(); pp_status=PP_RECORDING;
    log("  pair probe: OBSERVE ONLY %s; 8 Presents / 16384 events / 10 seconds; no second render\r\n",pp_return_mode?"shared entries plus same-thread consumer return":"shared entries (no returns)");
}
static void pp_prepare(ULONG64 base,const char *marker,const char *output) {
    pp_base=0; pp_status=PP_OFF; pp_attempted=0;
    pp_request_base=base;
    strcpy_s(pp_request_marker,sizeof pp_request_marker,marker);
    strcpy_s(pp_request_output,sizeof pp_request_output,output);
}
/* Worker poll: the owner can request capture after reaching gameplay.
   One request attempt per session, including a malformed/refused request. */
static int pp_poll(void (*log)(const char *,...)) {
    if (pp_attempted || !pp_request_base ||
        GetFileAttributesA(pp_request_marker)==INVALID_FILE_ATTRIBUTES) return 0;
    pp_attempted=1;
    pp_begin(pp_request_base,pp_request_marker,pp_request_output,log);
    return pp_status==PP_RECORDING;
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
/* Called from phase_context AFTER the existing render-link event. The
 * host owns the execution trap; this observer must not consume/change it. */
static void pp_shared_event(unsigned bit,const CONTEXT *c) {
    PP_ROW row;
    if (pp_status!=PP_RECORDING) return;
    memset(&row,0,sizeof row);
    if (bit==4) {
        row.channel=c->Rcx; row.stage=(unsigned)c->Rdi;
        row.which=(unsigned)c->Rdx; row.event=PP_STAGE_IN;
        if (c->Rsi!=(ULONG64)row.stage*8) { pp_fail("stage_index_mismatch"); return; }
    } else if (bit==8) {
        row.which=(unsigned)c->Rcx; row.event=PP_CONSUMER_IN;
    } else { pp_fail("unknown_shared_event"); return; }
    if (!pp_native(&row)) { pp_fail("native_state_mismatch"); return; }
    if (bit==4 && row.table!=c->Rax) { pp_fail("stage_table_register_mismatch"); return; }
    pp_store(&row);
}
static void pp_return_arm(CONTEXT *c) {
    ULONG64 target;
    if(!pp_return_mode || pp_status!=PP_RECORDING)return;
    __try {target=*(ULONG64*)c->Rsp;
        if(target!=pp_base+PP_CONSUMER_RETURN || c->Rcx>1 ||
           memcmp((void*)(pp_base+PP_CONSUMER_CALL),"\xe8\x39\xdf\xff\xff",5)) {
            pp_fail("consumer_return_anchor_mismatch");return;
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){pp_fail("consumer_return_stack_unreadable");return;}
    if(InterlockedCompareExchange(&pp_pending_tid,(LONG)GetCurrentThreadId(),0)) {
        pp_fail("overlapping_consumer");return;
    }
    pp_pending_rsp=c->Rsp+8;pp_pending_which=(unsigned)c->Rcx;
    c->Dr3=target;
}
static int pp_return_context(CONTEXT *c) {
    PP_ROW row;
    if(!pp_pending_context(c) || c->Rip!=pp_base+PP_CONSUMER_RETURN ||
       (c->Dr6&15)!=8 || (c->Dr7&0xff000050ull)!=0x50)return 0;
    if(c->Rsp!=pp_pending_rsp)pp_fail("consumer_return_stack_mismatch");
    else if(pp_status==PP_RECORDING) {
        memset(&row,0,sizeof row);row.event=PP_CONSUMER_OUT;row.which=pp_pending_which;
        if(pp_native(&row))pp_store(&row);else pp_fail("return_native_state_mismatch");
    }
    c->Dr3=pp_base+0xdb30;c->Dr6&=~8ull;c->EFlags|=0x10000;
    InterlockedExchange(&pp_pending_tid,0);
    if(pp_present_id>=8)InterlockedCompareExchange(&pp_status,PP_DONE,PP_RECORDING);
    return 1;
}
static void pp_present(void) {
    PP_ROW row;
    if (pp_status!=PP_RECORDING || !pp_started) return;
    memset(&row,0,sizeof row); row.event=PP_PRESENT;
    if (pp_store(&row) && InterlockedIncrement(&pp_present_id)>=8 && !pp_pending_tid)
        InterlockedCompareExchange(&pp_status,PP_DONE,PP_RECORDING);
}
/* Worker only. Stop publication before taking a blocking lock or doing I/O. */
static void pp_flush(int finish,void (*log)(const char *,...)) {
    FILE *f=NULL;
    HANDLE file;
    int fd;
    unsigned i;
    LARGE_INTEGER frequency;
    if (!pp_base || pp_dumped) return;
    if (pp_status==PP_RECORDING && (finish || GetTickCount()-pp_begin_ms>=10000))
        pp_fail(finish?"session_ended_early":"sample_timeout");
    if (pp_status==PP_RECORDING) return;
    AcquireSRWLockExclusive(&pp_lock);
    file=CreateFileA(pp_output,GENERIC_WRITE,FILE_SHARE_READ,NULL,CREATE_NEW,
                     FILE_ATTRIBUTE_NORMAL,NULL);
    fd=file==INVALID_HANDLE_VALUE ? -1 : _open_osfhandle((intptr_t)file,_O_WRONLY|_O_TEXT);
    if (fd>=0) f=_fdopen(fd,"w");
    if (!f) {
        if (fd>=0) _close(fd);
        else if (file!=INVALID_HANDLE_VALUE) CloseHandle(file);
        log("  pair probe: output unavailable (never overwrite): %s\r\n",pp_output);
        pp_dumped=1; ReleaseSRWLockExclusive(&pp_lock); return;
    }
    QueryPerformanceFrequency(&frequency);
    fprintf(f,"# observation_only=1 capture_kind=%s status=%ld reason=%s qpc_hz=%lld base=0x%llx simulation_id=UNKNOWN\n",
        pp_return_mode?"shared_consumer_returns":"shared_entries",pp_status,pp_failure?pp_failure:"none",frequency.QuadPart,pp_base);
    fprintf(f,"sequence,qpc,thread,event,present,stage,which,phase,channel,target,table,start,end,cursor,head,tail\n");
    for(i=0;i<pp_count;++i) {
        PP_ROW *r=&pp_rows[i];
        fprintf(f,"%u,%lld,%lu,%u,%u,%u,%u,%u,0x%llx,0x%llx,0x%llx,0x%llx,0x%llx,0x%llx,0x%llx,0x%llx\n",
            i,r->qpc,r->thread,r->event,r->present,r->stage,r->which,r->phase,
            r->channel,r->target,r->table,r->start,r->end,r->cursor,r->head,r->tail);
    }
    { int failed=ferror(f); if (fclose(f)) failed=1;
      log("  pair probe: %s %u events status=%ld reason=%s -> %s; NOT render-only proof\r\n",
          failed?"WRITE FAILED":"wrote",pp_count,pp_status,pp_failure?pp_failure:"none",pp_output); }
    pp_dumped=1;
    ReleaseSRWLockExclusive(&pp_lock);
}
#endif
