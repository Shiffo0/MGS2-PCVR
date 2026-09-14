/* Diagnostic only. RW hardware watches record accesses, NOT assumed reads.
 * Captures are a separate event stream: thread/time correlation must be
 * checked before attributing an access to a draw. Never modify packet bytes.
 */
#define DG_HUD_WATCH_BUILD 1
#define HUD_WATCH_REQUEST "logs\\pcvr_hud_watch.txt"
#define HUD_ENTRY_RVA 0x128ceaull
#define HUD_SUBMIT_RVA 0x128cd1ull
#define HUD_TEXTURED_RVA 0x1290ceull
static int arm_all(int arm);
static volatile LONG hud_active,hud_budget,hud_dropped;
static ULONG64 hud_watch[2]; /* publish while inactive; immutable during run */
static ULONG64 hud_cursor;
static ULONGLONG hud_until;
static int hud_used;
static SRWLOCK hud_lock=SRWLOCK_INIT;
typedef struct HUD_EVENT {
    ULONG64 qpc, rip, address, cap, regs[14];
    float xy[8];
    DWORD tid; LONG frame; int kind,slot,eye;
} HUD_EVENT;
static HUD_EVENT hud_events[1024];
static unsigned hud_count;

static void hud_push(HUD_EVENT *e) {
    LARGE_INTEGER q;
    QueryPerformanceCounter(&q);e->qpc=q.QuadPart;
    e->tid=GetCurrentThreadId();e->frame=g_present_frame;
    if(!TryAcquireSRWLockExclusive(&hud_lock)){InterlockedIncrement(&hud_dropped);return;}
    if(hud_count<1024)hud_events[hud_count++]=*e;
    else InterlockedIncrement(&hud_dropped);
    ReleaseSRWLockExclusive(&hud_lock);
}
static void hud_capture(int eye, uint64_t capture_id) {
    HUD_EVENT e;
    if(!hud_active)return;
    memset(&e,0,sizeof e);e.kind=1;e.eye=eye;e.cap=capture_id;
    hud_push(&e);
}
static int hud_context(CONTEXT *c) {
    unsigned bits=(unsigned)c->Dr6&15u;
    int entry=bits==4 && c->Rip==g_base+HUD_ENTRY_RVA &&
        c->Dr2==g_base+HUD_ENTRY_RVA && (c->Dr7&0x0f000010ull)==0x10;
    int textured=c->Rip==g_base+HUD_TEXTURED_RVA;
    int submit=bits==8 && c->Dr2==g_base+HUD_ENTRY_RVA && c->Rip==c->Dr3 &&
        (textured || c->Rip==g_base+HUD_SUBMIT_RVA) && (c->Dr7&0xff000050ull)==0x50;
    int access=bits==8 && c->Dr2==g_base+HUD_ENTRY_RVA &&
        c->Dr3>=0x10000 && (c->Dr7&0xff000050ull)==0xf0000050ull;
    HUD_EVENT e;
    if(!entry&&!submit&&!access)return 0;
    if(hud_active && InterlockedDecrement(&hud_budget)>=0) {
        if(entry && c->Rbx==hud_cursor) {
            /* Type-13 entry precedes the R15 branch and preserves RBX/R15.
             * Rebind DR3 on this thread to the actual textured/plain CALL. */
            c->Dr3=g_base+((DWORD)c->R15?HUD_TEXTURED_RVA:HUD_SUBMIT_RVA);
            c->Dr7=(c->Dr7&~0xf00000c0ull)|0x40ull;
            memset(&e,0,sizeof e);e.kind=3;e.address=c->Rbx;e.rip=c->Dr3;
            e.slot=(DWORD)c->R15!=0;hud_push(&e);
        } else if(submit && c->Rbx==hud_cursor) {
            /* Binding is local to this thread. Never guess that a different
             * rendering thread inherited this dynamic watch. */
            memset(&e,0,sizeof e);e.kind=2;e.eye=-1;e.address=c->Rcx;e.rip=c->Rbx;e.slot=textured;
            __try {
                if(c->Rcx<0x10000 || c->Rcx>0x00007fffffffffd7ull || (c->Rcx&3)) {
                    InterlockedIncrement(&hud_dropped);c->Dr7&=~0xf00000c0ull;
                } else {
                    memcpy(&e.xy[0],&c->Xmm1,4);memcpy(&e.xy[1],&c->Xmm2,4);
                    if(textured) {
                        memcpy(&e.xy[2],&c->Xmm3,4);e.xy[3]=*(float*)(c->Rsp+0x20);
                        e.xy[4]=*(float*)(c->Rsp+0x28);e.xy[5]=*(float*)(c->Rsp+0x30);
                        e.xy[6]=*(float*)(c->Rsp+0x38);e.xy[7]=*(float*)(c->Rsp+0x40);
                    } else {
                        e.xy[2]=*(float*)(c->Rsp+0x20);e.xy[3]=*(float*)(c->Rsp+0x28);
                        e.xy[4]=*(float*)(c->Rsp+0x38);e.xy[5]=*(float*)(c->Rsp+0x40);
                        e.xy[6]=*(float*)(c->Rsp+0x50);e.xy[7]=*(float*)(c->Rsp+0x58);
                    }
                    hud_push(&e);
                    /* Opcode9 stores float X0 at +0x1c; +0xc is UV!
                     * Opcode4 stores packed 16-bit X0/Y0 at +0xc. */
                    c->Dr3=c->Rcx+(textured?0x1c:0xc);
                    c->Dr7=(c->Dr7&~0xf00000c0ull)|0xf0000040ull;
                }
            } __except(EXCEPTION_EXECUTE_HANDLER) {
                InterlockedIncrement(&hud_dropped);c->Dr7&=~0xf00000c0ull;
            }
        } else if(submit) {
            c->Dr7&=~0xf00000c0ull; /* stale/foreign command: no attribution */
        } else if(access) {
            memset(&e,0,sizeof e);e.slot=1;e.eye=-1;e.rip=c->Rip;e.address=c->Dr3;
            e.regs[0]=c->Rax;e.regs[1]=c->Rbx;e.regs[2]=c->Rcx;e.regs[3]=c->Rdx;
            e.regs[4]=c->Rsi;e.regs[5]=c->Rdi;e.regs[6]=c->R8;e.regs[7]=c->R9;
            e.regs[8]=c->R10;e.regs[9]=c->R11;e.regs[10]=c->R12;
            e.regs[11]=c->R13;e.regs[12]=c->R14;e.regs[13]=c->R15;
            hud_push(&e);

            if(c->Rip==g_base+0x941eb && c->Dr3==c->Rsi+0x1c &&
               c->Rbx>=0x10000 && c->Rbx<0x00007fffffffffc0ull && !(c->Rbx&3)) {
                e.kind=4;e.address=c->Rsi+0x1c;e.cap=c->Rbx+0x1c;
                hud_push(&e);c->Dr3=e.cap;
                /* Keep RW4: the immediate copy-write is recorded explicitly,
                 * followed by downstream accesses. No command bytes change. */
            }
        }
    }
    if(!hud_active || hud_budget<=0) {
        /* Per-thread immediate stop; monitor removes watches everywhere. */
        c->Dr7&=~0xff0000f0ull;
    }
    c->Dr6&=~(ULONG64)bits;
    if(entry||submit)c->EFlags|=0x10000; /* original instruction exactly once */
    return 1;
}
static void hud_debug_registers(CONTEXT *c,int arm) {
    if(arm && hud_active && !g_phase_enabled && !g_render_link_active && pp_status!=PP_RECORDING) {
        c->Dr2=g_base+HUD_ENTRY_RVA;c->Dr3=0;
        c->Dr7=(c->Dr7&~0xff0000f0ull)|0x10ull;
    }
}
/* Same getter signatures and bounded layout traversal as the verified reader.
 * Only a non-empty SP_POLY cursor is eligible; SP_EMPTY fields are not pointers.
 */
static int hud_find_cursor(ULONG64 out[3]) {
    static const unsigned char getter[]={0x4c,0x8b,0x05,0xc1,0x9d,0x57,0x01,0x4d,0x85,0xc0,0x74,0x16,0x83,0xf9,0x1f,0x77,0x11,0x48,0x63,0xc1,0x49,0x8b,0x4c,0xc0,0x58,0x48,0x85,0xc9,0x0f,0x85,0x5e,0xe9,0,0,0x33,0xc0,0xc3};
    static const unsigned char lookup[]={0x48,0x63,0x41,0x50,0x45,0x33,0xc0,0x85,0xc0,0x7e,0x20,0x4c,0x8b,0x51,0x58,0x4c,0x8b,0xc8,0x49,0x8b,0xc2,0x41,0x8b,0xc8,0x39,0x10,0x74,0x12,0x41,0xff,0xc0,0x48,0xff,0xc1,0x48,0x83,0xc0,0x68,0x49,0x3b,0xc9,0x7c,0xed,0x33,0xc0,0xc3,0x49,0x63,0xc0,0x48,0x6b,0xc0,0x68,0x49,0x3,0xc2,0x74,0xf1,0x48,0x8b,0x40,0x8,0xc3};
    static const unsigned char submit_sig[]={0xe8,0x6a,0xbd,0xf6,0xff};
    static const unsigned char vertex_read[]={0xf3,0x44,0x0f,0x10,0xa3,0x98,0,0,0};
    static const unsigned char vertex_write[]={0x66,0x41,0x89,0x42,0x0c};
    ULONG64 manager,slots[32],again[32],found[3]={0,0,0};int h,matches=0;
    __try {
        if(memcmp((void*)(g_base+0x11f560),getter,sizeof getter)||
           memcmp((void*)(g_base+0x12dee0),lookup,sizeof lookup)||
           memcmp((void*)(g_base+HUD_SUBMIT_RVA),submit_sig,sizeof submit_sig)||
           memcmp((void*)(g_base+HUD_TEXTURED_RVA),"\xe8\xdd\xbc\xf6\xff",5)||
           memcmp((void*)(g_base+0x128fa3),"\x45\x85\xff\x0f\x84\x3b\x01\0\0",9)||
           memcmp((void*)(g_base+0x941e7),"\x0f\x10\x4e\x10\x0f\x11\x4b\x10",8)||
           memcmp((void*)(g_base+0x94dc8),"\xf3\x41\x0f\x11\x4a\x1c",6)||
           memcmp((void*)(g_base+0x94e14),"\xc6\x01\x09",3)||
           memcmp((void*)(g_base+0x128cea),vertex_read,sizeof vertex_read)||
           memcmp((void*)(g_base+0x94b1d),vertex_write,sizeof vertex_write)||
           *(unsigned*)(g_base+0x129d54+13*4)!=0x128cea)return 0;
        manager=*(ULONG64*)(g_base+0x1699328);if(!manager)return 0;
        memcpy(slots,(void*)(manager+0x58),sizeof slots);
        for(h=0;h<32;h++)if(slots[h]) {
            ULONG64 data=slots[h],parts=*(ULONG64*)(data+0x58),cursor=0;
            int n=*(int*)(data+0x50),i,has_small_cursor=0;
            if(n<0||n>2048||(!parts&&n))return 0;
            for(i=0;i<n;i++) {
                unsigned code=*(unsigned*)(parts+0x68ull*i);
                if(code==3481206)cursor=*(ULONG64*)(parts+0x68ull*i+8);
                if(code==2319263)has_small_cursor=1;
            }
            if(cursor&&has_small_cursor) {
                if(*(unsigned*)(cursor+0x28)!=13)return 0;
                found[2]=cursor;
                found[0]=*(ULONG64*)(cursor+0x48);found[1]=*(ULONG64*)(cursor+0x50);
                if(found[0]<0x10000||found[1]<0x10000||found[0]==found[1]||
                   ((found[0]|found[1])&7))return 0;
                /* Validate addresses once, on the un-watched monitor thread. */
                { volatile ULONG64 a=*(ULONG64*)found[0],b=*(ULONG64*)found[1];(void)a;(void)b; }
                if(parts!=*(ULONG64*)(data+0x58)||n!=*(int*)(data+0x50))return 0;
                matches++;
            }
        }
        memcpy(again,(void*)(manager+0x58),sizeof again);
        if(manager!=*(ULONG64*)(g_base+0x1699328)||memcmp(slots,again,sizeof slots))return 0;
    } __except(EXCEPTION_EXECUTE_HANDLER) {return 0;}
    if(matches!=1)return 0;
    out[0]=found[0];out[1]=found[1];out[2]=found[2];return 1;
}
static void hud_flush(void) {
    static HUD_EVENT copy[1024];unsigned n,i;
    AcquireSRWLockExclusive(&hud_lock);n=hud_count;
    memcpy(copy,hud_events,n*sizeof *copy);hud_count=0;
    ReleaseSRWLockExclusive(&hud_lock);
    for(i=0;i<n;i++) {
        HUD_EVENT *e=&copy[i];
        if(e->kind==1)logf_("HUDVERT capture qpc %llu tid %lu frame %ld eye %d id %llu\r\n",e->qpc,e->tid,e->frame,e->eye,e->cap);
        else if(e->kind==4)logf_("HUDVERT transfer qpc %llu tid %lu frame %ld source %llx destination %llx rip_after %llx\r\n",e->qpc,e->tid,e->frame,e->address,e->cap,e->rip);
        else if(e->kind==3)logf_("HUDVERT entry qpc %llu tid %lu frame %ld sprite %llx call %llx textured %d\r\n",e->qpc,e->tid,e->frame,e->address,e->rip,e->slot);
        else if(e->kind==2)logf_("HUDVERT submit qpc %llu tid %lu frame %ld command %llx sprite %llx textured %d xy %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\r\n",e->qpc,e->tid,e->frame,e->address,e->rip,e->slot,e->xy[0],e->xy[1],e->xy[2],e->xy[3],e->xy[4],e->xy[5],e->xy[6],e->xy[7]);
        else logf_("HUDVERT access qpc %llu tid %lu frame %ld address %llx rip_after %llx regs %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx %llx\r\n",e->qpc,e->tid,e->frame,e->address,e->rip,e->regs[0],e->regs[1],e->regs[2],e->regs[3],e->regs[4],e->regs[5],e->regs[6],e->regs[7],e->regs[8],e->regs[9],e->regs[10],e->regs[11],e->regs[12],e->regs[13]);
    }
}
static void hud_stop(void) {
    if(InterlockedExchange(&hud_active,0)) {
        arm_all(1);hud_flush();
        logf_("HUDVERT stop dropped %ld budget_remaining %ld; same-thread dynamic watch, accesses include writes\r\n",hud_dropped,hud_budget);
    }
}
static void hud_poll(void) {
    ULONG64 found[3];LARGE_INTEGER f;
    if(hud_active) {
        if(GetTickCount64()>=hud_until||hud_budget<=0||!g_armed||g_source!=SRC_XR||!g_stereo||
           g_phase_enabled||g_render_link_active||pp_status==PP_RECORDING||!hud_find_cursor(found)||found[0]!=hud_watch[0]||found[1]!=hud_watch[1]||found[2]!=hud_cursor)hud_stop();
        else hud_flush();
        return;
    }
    if(hud_used||GetFileAttributesA(HUD_WATCH_REQUEST)==INVALID_FILE_ATTRIBUTES)return;
    hud_used=1; /* one attempt per process; stale request cannot loop */
    if(!g_armed||g_source!=SRC_XR||!g_stereo||g_phase_enabled||g_render_link_active||pp_status==PP_RECORDING||!hud_find_cursor(found)) {
        logf_("HUDVERT refused: camera cursor/signatures unavailable, inactive stereo or debug slots occupied\r\n");return;
    }
    hud_watch[0]=found[0];hud_watch[1]=found[1];hud_cursor=found[2];hud_budget=4096;hud_dropped=0;
    hud_until=GetTickCount64()+6000;QueryPerformanceFrequency(&f);
    logf_("HUDVERT start qpf %llu base %llx sprite %llx entry %llx; branch CALL then opcode-specific RW4 watch\r\n",f.QuadPart,g_base,hud_cursor,g_base+HUD_ENTRY_RVA);
    InterlockedExchange(&hud_active,1);
    logf_("HUDVERT armed threads %d\r\n",arm_all(1));
}
