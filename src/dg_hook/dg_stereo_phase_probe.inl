/* Optional observation alongside the separate render-link correction.
 * Opt-in once per process via captures/request.txt (integer 1..9999).
 * DR2 observes world screen-stage dispatch; DR3 command-consumer entry.
 * A 10-second trace is timing evidence, NOT final-draw/pixel identity proof.
 */
#include "dg_pair_probe.h"
#define PHASE_ROOT "logs\\pcvr_phase"
#define PHASE_CAP DG_DIAGNOSTIC_CAPACITY(8192)
typedef struct {
    LONGLONG qpc;
    DWORD tid;
    int kind, buffer, eye, valid, handoff_seq, present;
    ULONG64 dr0,dr1,dr2,dr3,dr7,route_gate;
    int route_valid;
} PHASE_EVENT;
typedef struct {DWORD tid;int set_ok,read_ok,match;ULONG64 dr0,dr1,dr2,dr3,dr7;} PHASE_REG;
static PHASE_REG g_phase_regs[256];
static unsigned g_phase_reg_count,g_phase_reg_dropped;
static PHASE_EVENT g_phase_events[PHASE_CAP];
static SRWLOCK g_phase_lock = SRWLOCK_INIT;
static volatile LONG g_phase_enabled, g_phase_dropped, g_phase_faults;
static LONG g_phase_count;
static ULONG64 g_phase_stage_addr, g_phase_consume_addr;
static ULONGLONG g_phase_start;
static unsigned g_phase_token;
static int g_phase_attempted;
static int arm_all(int arm);

static int phase_signatures(const unsigned char *stage,const unsigned char *consumer)
{
    static const unsigned char s[]={0x48,0x8b,0x83,0x20,3,0,0,0x41,0x8b,0xd7,
        0x48,0x8b,0xcb,0xff,0x14,0x30,0xff,0xc7,0x48,0x8d,0x76,8};
    static const unsigned char c[]={0x40,0x55,0x41,0x54,0x41,0x56,0x41,0x57,
        0x48,0x8d,0x6c,0x24,0xc1,0x48,0x81,0xec,0xa8,0,0,0,
        0xe8,7,0x8f,1,0};
    return !memcmp(stage,s,sizeof s)&&!memcmp(consumer,c,sizeof c);
}
static void phase_init(void)
{
    g_phase_stage_addr=g_phase_consume_addr=0;
    __try {
        if (phase_signatures((const unsigned char*)(g_base+0x89df0),
                             (const unsigned char*)(g_base+0xdb30))) {
            g_phase_stage_addr=g_base+0x89dfd;
            g_phase_consume_addr=g_base+0xdb30;
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) {}
    logf_("  stereo phase: observation-only, anchors %s; request %s\\request.txt\r\n",
        g_phase_stage_addr?"verified":"UNAVAILABLE",PHASE_ROOT);
}
static void phase_record(int kind,int buffer)
{
#if DG_ENABLE_DIAGNOSTICS

    PHASE_EVENT e;
    DG_HOOK_HANDOFF h;
    LARGE_INTEGER q;
    LONG before,after;
    if (!InterlockedCompareExchange(&g_phase_enabled,0,0)) return;
    if (!TryAcquireSRWLockExclusive(&g_phase_lock)) {
        InterlockedIncrement(&g_phase_dropped); return;
    }
    if (!InterlockedCompareExchange(&g_phase_enabled,0,0)) {
        ReleaseSRWLockExclusive(&g_phase_lock); return;
    }
    memset(&e,0,sizeof e);memset(&h,0,sizeof h);
    QueryPerformanceCounter(&q);e.qpc=q.QuadPart;e.tid=GetCurrentThreadId();
    e.kind=kind;e.buffer=buffer;e.eye=-1;
    before=InterlockedCompareExchange(&g_handoff_seq,0,0);
    e.valid=handoff_read(&h);
    after=InterlockedCompareExchange(&g_handoff_seq,0,0);
    e.valid=e.valid&&h.valid&&before==after&&!(after&1);
    e.handoff_seq=(int)after;
    if (e.valid)e.eye=h.eye;
    e.present=(int)InterlockedCompareExchange(&g_present_frame,0,0);
    if(g_phase_count<PHASE_CAP)g_phase_events[g_phase_count++]=e;
    else InterlockedIncrement(&g_phase_dropped);
    ReleaseSRWLockExclusive(&g_phase_lock);

#else

#endif
}
/* Worker only, target remains suspended. No second SetThreadContext. */
static void phase_registration(HANDLE t,DWORD tid,int set_ok)
{
#if DG_ENABLE_DIAGNOSTICS

    CONTEXT c;PHASE_REG *r;
    if(!g_phase_enabled)return;
    if(g_phase_reg_count>=256){g_phase_reg_dropped++;return;}
    r=&g_phase_regs[g_phase_reg_count++];memset(r,0,sizeof *r);
    r->tid=tid;r->set_ok=set_ok;
    memset(&c,0,sizeof c);c.ContextFlags=CONTEXT_DEBUG_REGISTERS;
    r->read_ok=GetThreadContext(t,&c)!=0;
    if(!r->read_ok)return;
    r->dr0=c.Dr0;r->dr1=c.Dr1;r->dr2=c.Dr2;r->dr3=c.Dr3;r->dr7=c.Dr7;
    r->match=set_ok&&c.Dr2==g_phase_stage_addr&&c.Dr3==g_phase_consume_addr&&
        (c.Dr7&0xff0000f0ull)==0x50;

#else

#endif
}
/* Observation of registers delivered on the already-working camera seam. */
static void phase_camera_context(const CONTEXT *c)
{
#if DG_ENABLE_DIAGNOSTICS

    PHASE_EVENT e;LARGE_INTEGER q;
    if(!g_phase_enabled)return;
    if(!TryAcquireSRWLockExclusive(&g_phase_lock)){InterlockedIncrement(&g_phase_dropped);return;}
    if(!g_phase_enabled){ReleaseSRWLockExclusive(&g_phase_lock);return;}
    memset(&e,0,sizeof e);QueryPerformanceCounter(&q);e.qpc=q.QuadPart;
    e.kind=5;e.tid=GetCurrentThreadId();e.buffer=-1;e.eye=-1;
    e.present=(int)InterlockedCompareExchange(&g_present_frame,0,0);
    e.dr0=c->Dr0;e.dr1=c->Dr1;e.dr2=c->Dr2;e.dr3=c->Dr3;e.dr7=c->Dr7;
    __try {e.route_gate=*(volatile ULONG64*)(g_base+0x1553ef0);e.route_valid=1;}
    __except(EXCEPTION_EXECUTE_HANDLER) {}
    if(g_phase_count<PHASE_CAP)g_phase_events[g_phase_count++]=e;
    else InterlockedIncrement(&g_phase_dropped);
    ReleaseSRWLockExclusive(&g_phase_lock);

#else

#endif
}
/* Outgoing capture arguments, NOT proof of GPU/copy/submission success. */
static void phase_link_record(const DG_HOOK_HANDOFF *h,int sequence)
{
#if DG_ENABLE_DIAGNOSTICS

    PHASE_EVENT e;LARGE_INTEGER q;
    if(!g_phase_enabled)return;
    if(!TryAcquireSRWLockExclusive(&g_phase_lock)){InterlockedIncrement(&g_phase_dropped);return;}
    if(!g_phase_enabled){ReleaseSRWLockExclusive(&g_phase_lock);return;}
    memset(&e,0,sizeof e);QueryPerformanceCounter(&q);e.qpc=q.QuadPart;
    e.kind=6;e.tid=GetCurrentThreadId();e.buffer=-1;e.eye=h?h->eye:-1;
    e.valid=h?h->valid:0;e.handoff_seq=sequence;e.present=(int)g_present_frame;
    if(g_phase_count<PHASE_CAP)g_phase_events[g_phase_count++]=e;
    else InterlockedIncrement(&g_phase_dropped);
    ReleaseSRWLockExclusive(&g_phase_lock);

#else

#endif
}
/* Return exactly the owned execution-breakpoint bit, never a foreign trap. */
static unsigned phase_owned(const CONTEXT *c)
{
    unsigned bits=(unsigned)c->Dr6&15u;
    if(g_phase_stage_addr&&c->Rip==g_phase_stage_addr&&bits==4)return 4;
    if(g_phase_consume_addr&&c->Rip==g_phase_consume_addr&&bits==8)return 8;
    return 0;
}
static int phase_context(CONTEXT *c)
{
    unsigned bit;
    if(pp_return_context(c))return 1;
    bit=phase_owned(c);
    if(!bit)return 0;
#ifdef DG_CAPTURE_IDENTITY
    ci_native(bit,c);
#endif
    render_link_event(bit,c);
    pp_shared_event(bit,c); /* observes; never changes debug slots or context */
    if(InterlockedCompareExchange(&g_phase_enabled,0,0)) {
        InterlockedIncrement(&g_phase_faults);
        if(bit==8)phase_record(3,(int)c->Rcx);
        else if(c->Rcx==g_chan0&&c->Rsi==3*8)phase_record(2,(int)c->Rdx);
    }
    if(bit==8)pp_return_arm(c);
    c->Dr6&=~(ULONG64)bit;c->EFlags|=0x10000;
    return 1;
}
static void phase_finish(const char *reason)
{
#if DG_ENABLE_DIAGNOSTICS

    char path[MAX_PATH];FILE *f;LARGE_INTEGER freq;LONG i;int ok=1;
    if(!InterlockedExchange(&g_phase_enabled,0))return;
    arm_all(g_armed!=0); /* Remove DR2/3; existing camera and blur remain. */
    AcquireSRWLockExclusive(&g_phase_lock);
    sprintf_s(path,sizeof path,PHASE_ROOT "\\phase_%u.jsonl",g_phase_token);
    f=NULL;fopen_s(&f,path,"wx");QueryPerformanceFrequency(&freq);
    if(f) {
        fprintf(f,"{\"format\":\"DG_PHASE3\",\"qpf\":%lld,\"token\":%u,\"count\":%ld,\"dropped\":%ld,\"faults\":%ld,\"reason\":\"%s\",\"stage_addr\":%llu,\"consumer_addr\":%llu,\"registration_dropped\":%u,\"registrations\":[",
            freq.QuadPart,g_phase_token,g_phase_count,g_phase_dropped,g_phase_faults,reason,
            g_phase_stage_addr,g_phase_consume_addr,g_phase_reg_dropped);
        for(i=0;i<(LONG)g_phase_reg_count;i++) {
            PHASE_REG *r=&g_phase_regs[i];
            fprintf(f,"%s{\"tid\":%lu,\"set_ok\":%d,\"read_ok\":%d,\"match\":%d,\"dr0\":%llu,\"dr1\":%llu,\"dr2\":%llu,\"dr3\":%llu,\"dr7\":%llu}",
                i?",":"",r->tid,r->set_ok,r->read_ok,r->match,r->dr0,r->dr1,r->dr2,r->dr3,r->dr7);
        }
        fprintf(f,"],\"final_pixels_proven\":false}\n");
        for(i=0;i<g_phase_count;i++) {
            PHASE_EVENT *e=&g_phase_events[i];
            fprintf(f,"{\"qpc\":%lld,\"tid\":%lu,\"kind\":%d,\"buffer\":%d,\"eye\":%d,\"valid\":%d,\"handoff_seq\":%d,\"present\":%d,\"dr0\":%llu,\"dr1\":%llu,\"dr2\":%llu,\"dr3\":%llu,\"dr7\":%llu,\"route_gate\":%llu,\"route_valid\":%d}\n",
                e->qpc,e->tid,e->kind,e->buffer,e->eye,e->valid,e->handoff_seq,e->present,
                e->dr0,e->dr1,e->dr2,e->dr3,e->dr7,e->route_gate,e->route_valid);
        }
        if(ferror(f))ok=0;if(fclose(f))ok=0;
    } else ok=0;
    ReleaseSRWLockExclusive(&g_phase_lock);
    logf_("  stereo phase: %s events %ld dropped %ld -> %s (%s)\r\n",
        ok?"saved":"FAILED",g_phase_count,g_phase_dropped,path,reason);

#else

#endif
}
/* Worker thread only: no filesystem IO, waits or thread suspension in VEH. */
static void phase_poll(void)
{
#if DG_ENABLE_DIAGNOSTICS

    FILE *f=NULL;unsigned token=0;char extra;ULONGLONG now=GetTickCount64();
    if(InterlockedCompareExchange(&g_phase_enabled,0,0)) {
        if(now-g_phase_start>=10000||g_phase_faults>100000)
            phase_finish(g_phase_faults>100000?"fault_budget":"complete");
        return;
    }
    if(g_phase_attempted||!g_phase_stage_addr||g_source!=SRC_XR||!g_stereo||!g_armed)return;
    fopen_s(&f,PHASE_ROOT "\\request.txt","r");if(!f)return;
    if(fscanf_s(f,"%u %c",&token,&extra,1u)!=1)token=0;fclose(f);
    if(!token||token>9999)return;
    g_phase_attempted=1;g_phase_token=token;g_phase_start=now;
    InterlockedExchange(&g_phase_enabled,1);
    logf_("  stereo phase: started token %u, ten seconds, observer threads %d\r\n",token,arm_all(1));

#else

#endif
}
