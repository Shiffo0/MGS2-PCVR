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






























}
/* Worker only, target remains suspended. No second SetThreadContext. */
static void phase_registration(HANDLE t,DWORD tid,int set_ok)
{

















}
/* Observation of registers delivered on the already-working camera seam. */
static void phase_camera_context(const CONTEXT *c)
{



















}
/* Outgoing capture arguments, NOT proof of GPU/copy/submission success. */
static void phase_link_record(const DG_HOOK_HANDOFF *h,int sequence)
{
















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

































}
/* Worker thread only: no filesystem IO, waits or thread suspension in VEH. */
static void phase_poll(void)
{



















}
