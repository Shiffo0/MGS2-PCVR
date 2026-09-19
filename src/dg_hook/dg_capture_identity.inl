/* Bounded observation only; never changes capture selection or debug slots. */
#define DG_CAPTURE_IDENTITY 1
typedef struct {
    LONGLONG qpc;DWORD tid;LONG frame;int kind,slot,sequence;
    uint64_t capture;DG_HOOK_HANDOFF h;
} CI_ROW;
static CI_ROW ci_rows[256];
static unsigned ci_count;
static LONG ci_lost;
static SRWLOCK ci_lock=SRWLOCK_INIT;
static void ci_record(int kind,int slot,int sequence,const DG_HOOK_HANDOFF *h,uint64_t capture) {
    CI_ROW r;LARGE_INTEGER q;
    if(pp_status!=PP_RECORDING)return;
    memset(&r,0,sizeof r);QueryPerformanceCounter(&q);r.qpc=q.QuadPart;
    r.tid=GetCurrentThreadId();r.frame=g_present_frame;r.kind=kind;r.slot=slot;r.sequence=sequence;r.capture=capture;
    if(h)r.h=*h;
    if(!TryAcquireSRWLockExclusive(&ci_lock)){InterlockedIncrement(&ci_lost);return;}
    if(ci_count<256)ci_rows[ci_count++]=r;else InterlockedIncrement(&ci_lost);
    ReleaseSRWLockExclusive(&ci_lock);
}
static void ci_native(unsigned bit,const CONTEXT *c) {
    DG_HOOK_HANDOFF h;LONG before,after;
    if(pp_status!=PP_RECORDING)return;
    if(bit==4 && c->Rcx==g_chan0 && c->Rsi==24) {
        memset(&h,0,sizeof h);before=InterlockedCompareExchange(&g_handoff_seq,0,0);
        if(!handoff_read(&h))h.valid=0;
        after=InterlockedCompareExchange(&g_handoff_seq,0,0);
        if(before!=after || (after&1))h.valid=0;
        ci_record(1,(int)c->Rdx,(int)after,&h,0);
    } else if(bit==8)ci_record(2,(int)c->Rcx,0,NULL,0);
}
static void ci_published(int eye,uint64_t id) {
    DG_HOOK_HANDOFF h;memset(&h,0,sizeof h);h.eye=eye;
    ci_record(4,-1,0,&h,id);
}
/* Worker only; publication can race draining but each row is copied atomically. */
static void ci_drain(void) {
    static CI_ROW copy[256];unsigned n,i;LONG lost;
    AcquireSRWLockExclusive(&ci_lock);n=ci_count;memcpy(copy,ci_rows,n*sizeof *copy);ci_count=0;
    lost=ci_lost;ReleaseSRWLockExclusive(&ci_lock);
    for(i=0;i<n;i++) {
        CI_ROW *r=&copy[i];
        logf_("CAPIDENT qpc %lld tid %lu frame %ld kind %d slot %d seq %d valid %d eye %d capture %llu pose %.17g %.17g %.17g %.17g %.17g %.17g %.17g fov %.17g %.17g %.17g %.17g\r\n",
        r->qpc,r->tid,r->frame,r->kind,r->slot,r->sequence,r->h.valid,r->h.eye,r->capture,
        r->h.raw.qx,r->h.raw.qy,r->h.raw.qz,r->h.raw.qw,r->h.raw.px,r->h.raw.py,r->h.raw.pz,
        r->h.fov.left,r->h.fov.right,r->h.fov.up,r->h.fov.down);
    }
    if(n)logf_("CAPIDENT health lost %ld\r\n",lost);
}
