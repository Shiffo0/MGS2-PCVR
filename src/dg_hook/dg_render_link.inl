#include "dg_render_link.h"
static SRWLOCK g_link_lock=SRWLOCK_INIT;
static DG_RENDER_LINK g_link;
static volatile LONG g_link_epoch,g_link_requested,g_render_link_active;
static LONG g_link_seen_epoch;
static volatile LONG g_link_hits,g_link_stage_hits,g_link_consumer_hits;
static volatile LONG g_link_accepted,g_link_refused,g_link_contention;
static volatile LONG g_link_register_mismatch;
static LONG g_link_last_hits;
static void render_link_camera(const CONTEXT *c,ULONG64 stage,ULONG64 consumer) {
    if(g_render_link_active&&(c->Dr2!=stage||(c->Dr3!=consumer
#ifdef DG_PAIR_PROBE_H
        && !pp_pending_context(c)
#endif
        )||
        (c->Dr7&0xff0000f0ull)!=0x50)) {
        InterlockedIncrement(&g_link_register_mismatch);InterlockedIncrement(&g_link_epoch);
    }
}
static void render_link_reset(void){InterlockedIncrement(&g_link_epoch);}
static int render_link_lock(void) {
    LONG epoch;
    if(!TryAcquireSRWLockExclusive(&g_link_lock)) {
        InterlockedIncrement(&g_link_contention);render_link_reset();return 0;
    }
    epoch=InterlockedCompareExchange(&g_link_epoch,0,0);
    if(epoch!=g_link_seen_epoch){dg_link_reset(&g_link);g_link_seen_epoch=epoch;}
    return 1;
}
static void render_link_event(unsigned bit,const CONTEXT *c) {
    DG_HOOK_HANDOFF h;LONG before,after;
    int buffer;uint32_t frame;uint64_t ms;
    if(!g_render_link_active)return;
    InterlockedIncrement(&g_link_hits);
    if(!g_armed||g_source!=SRC_XR||!g_stereo){render_link_reset();return;}
    if(bit==4 && (c->Rcx!=g_chan0||c->Rsi!=24))return;
    if(!render_link_lock())return;
    frame=(uint32_t)InterlockedCompareExchange(&g_present_frame,0,0);ms=GetTickCount64();
    if(bit==4) {
        InterlockedIncrement(&g_link_stage_hits);
        memset(&h,0,sizeof h);
        before=InterlockedCompareExchange(&g_handoff_seq,0,0);
        if(!handoff_read(&h))h.valid=0;
        after=InterlockedCompareExchange(&g_handoff_seq,0,0);
        if(before!=after||(after&1))h.valid=0;
        buffer=c->Rdx<=1?(int)c->Rdx:-1;
        dg_link_stage(&g_link,buffer,GetCurrentThreadId(),frame,ms,&h,(int)after);
    } else {
        InterlockedIncrement(&g_link_consumer_hits);
        buffer=c->Rcx<=1?(int)c->Rcx:-1;
        dg_link_consume(&g_link,buffer,GetCurrentThreadId(),frame,ms);
    }
    ReleaseSRWLockExclusive(&g_link_lock);
}
static int render_link_take(DG_HOOK_HANDOFF *h,int *sequence) {
    int ok=0;memset(h,0,sizeof *h);*sequence=0;
    if(g_render_link_active&&render_link_lock()) {
        ok=dg_link_present(&g_link,GetCurrentThreadId(),(uint32_t)g_present_frame,
                           GetTickCount64(),h,sequence);
        /* A lost event while we held the lock invalidates this candidate. */
        if(g_link_seen_epoch!=InterlockedCompareExchange(&g_link_epoch,0,0))ok=0;
        ReleaseSRWLockExclusive(&g_link_lock);
    }
    if(!ok)memset(h,0,sizeof *h);
    return ok;
}
