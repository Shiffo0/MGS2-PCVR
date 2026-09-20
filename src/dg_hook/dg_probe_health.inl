/* Observation only: no register mutation, exceptions consumed or game reads.
 * Installs are read back while already suspended by the existing owner.
 * VEH samples are bounded and emitted only from the worker. */
#define DG_PROBE_HEALTH 1
typedef struct {DWORD tid;unsigned bits;ULONG64 qpc,rip,dr0,dr1,dr2,dr3,dr7;} PD_ROW;
typedef struct {DWORD tid,error;int set_ok,read_ok,match;ULONG64 dr2,dr3,dr7;} PD_REG;
static PD_REG pd_regs[256];static unsigned pd_reg_count,pd_reg_lost;
static volatile LONG pd_mode,pd_pending,pd_total,pd_camera,pd_blur,pd_b2,pd_b3;
static volatile LONG pd_hud,pd_phase,pd_unknown,pd_lost;
static volatile LONG pd_open_fail,pd_suspend_fail,pd_get_fail,pd_set_fail,pd_resume_fail;
static volatile LONG pd_installs,pd_read_fail,pd_mismatch;
static volatile LONG pd_snapshot_fail;
static ULONG64 pd_expect2,pd_expect3,pd_expect7;
static SRWLOCK pd_lock=SRWLOCK_INIT;
static PD_ROW pd_rows[64];static unsigned pd_count;
static int pd_match(const CONTEXT *c,ULONG64 d2,ULONG64 d3,ULONG64 d7) {
    return c->Dr2==d2 && c->Dr3==d3 && (c->Dr7&0xff0000f0ull)==(d7&0xff0000f0ull);
}
static void pd_begin(int mode,ULONG64 d2,ULONG64 d3,ULONG64 d7) {
#if DG_ENABLE_DIAGNOSTICS

    if(!mode)return;
    pd_expect2=d2;pd_expect3=d3;pd_expect7=d7;
    InterlockedExchange(&pd_mode,mode);InterlockedExchange(&pd_pending,1);
    logf_("PROBEHEALTH install mode %d expected_dr2 %llx dr3 %llx dr7mask %llx\r\n",mode,d2,d3,d7&0xff0000f0ull);

#else

#endif
}
static void pd_registration(HANDLE t,DWORD tid,int set_ok) {
#if DG_ENABLE_DIAGNOSTICS

    CONTEXT c;PD_REG *r;if(!pd_mode)return;
    if(pd_reg_count==256){pd_reg_lost++;return;}
    r=&pd_regs[pd_reg_count++];memset(r,0,sizeof *r);r->tid=tid;r->set_ok=set_ok;
    memset(&c,0,sizeof c);c.ContextFlags=CONTEXT_DEBUG_REGISTERS;
    if(!GetThreadContext(t,&c)){r->error=GetLastError();InterlockedIncrement(&pd_read_fail);return;}
    r->read_ok=1;r->dr2=c.Dr2;r->dr3=c.Dr3;r->dr7=c.Dr7;
    r->match=set_ok&&pd_match(&c,pd_expect2,pd_expect3,pd_expect7);
    InterlockedIncrement(&pd_installs);
    if(!r->match)InterlockedIncrement(&pd_mismatch);

#else

#endif
}
static void pd_raw(const CONTEXT *c,ULONG64 camera,ULONG64 blur) {
#if DG_ENABLE_DIAGNOSTICS

    unsigned i,bits;DWORD tid;PD_ROW *r;LARGE_INTEGER q;
    if(!pd_mode)return;
    bits=(unsigned)c->Dr6&15u;InterlockedIncrement(&pd_total);
    if((bits&1)&&c->Rip==camera)InterlockedIncrement(&pd_camera);
    if((bits&2)&&c->Rip==blur)InterlockedIncrement(&pd_blur);
    if(bits&4)InterlockedIncrement(&pd_b2);
    if(bits&8)InterlockedIncrement(&pd_b3);
    if(!TryAcquireSRWLockExclusive(&pd_lock)){InterlockedIncrement(&pd_lost);return;}
    tid=GetCurrentThreadId();
    for(i=0;i<pd_count;i++)if(pd_rows[i].tid==tid&&pd_rows[i].bits==bits)break;
    if(i==64){InterlockedIncrement(&pd_lost);ReleaseSRWLockExclusive(&pd_lock);return;}
    if(i==pd_count)pd_count++;
    r=&pd_rows[i];r->tid=tid;r->bits=bits;r->rip=c->Rip;
    QueryPerformanceCounter(&q);r->qpc=q.QuadPart;
    r->dr0=c->Dr0;r->dr1=c->Dr1;r->dr2=c->Dr2;r->dr3=c->Dr3;r->dr7=c->Dr7;
    ReleaseSRWLockExclusive(&pd_lock);

#else

#endif
}
static void pd_result(int owner) {
#if DG_ENABLE_DIAGNOSTICS

    if(!pd_mode)return;
    if(owner==1)InterlockedIncrement(&pd_hud);
    else if(owner==2)InterlockedIncrement(&pd_phase);
    else InterlockedIncrement(&pd_unknown);

#else

#endif
}
static void pd_drain(int mode) {
#if DG_ENABLE_DIAGNOSTICS

    PD_ROW rows[64];unsigned n,i;
    if(!pd_mode && !pd_pending)return;
    AcquireSRWLockExclusive(&pd_lock);n=pd_count;
    memcpy(rows,pd_rows,n*sizeof *rows);pd_count=0;ReleaseSRWLockExclusive(&pd_lock);
    logf_("PROBEHEALTH counts mode %ld raw %ld camera %ld blur %ld bit2 %ld bit3 %ld hud_owned %ld phase_owned %ld unclaimed23 %ld lost %ld install_reads %ld mismatch %ld errors open/suspend/get/set/readback/resume %ld/%ld/%ld/%ld/%ld/%ld\r\n",
        pd_mode,pd_total,pd_camera,pd_blur,pd_b2,pd_b3,pd_hud,pd_phase,pd_unknown,pd_lost,
        pd_installs,pd_mismatch,pd_open_fail,pd_suspend_fail,pd_get_fail,pd_set_fail,pd_read_fail,pd_resume_fail);
    /* Never log while an installation target is suspended. */
    for(i=0;i<pd_reg_count;i++){PD_REG *r=&pd_regs[i];logf_("PROBEHEALTH registration tid %lu set %d read %d match %d error %lu dr2 %llx dr3 %llx dr7 %llx\r\n",r->tid,r->set_ok,r->read_ok,r->match,r->error,r->dr2,r->dr3,r->dr7);}
    pd_reg_count=0;
    if(pd_reg_lost||pd_snapshot_fail)logf_("PROBEHEALTH registration_lost %u snapshot_fail %ld\r\n",pd_reg_lost,pd_snapshot_fail);
    for(i=0;i<n;i++){PD_ROW *r=&rows[i];logf_("PROBEHEALTH context qpc %llu tid %lu bits %x rip %llx dr0 %llx dr1 %llx dr2 %llx dr3 %llx dr7 %llx\r\n",r->qpc,r->tid,r->bits,r->rip,r->dr0,r->dr1,r->dr2,r->dr3,r->dr7);}
    if(!mode){InterlockedExchange(&pd_mode,0);InterlockedExchange(&pd_pending,0);}

#else

#endif
}
