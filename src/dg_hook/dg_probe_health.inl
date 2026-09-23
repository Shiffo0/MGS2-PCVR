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



static void pd_begin(int mode,ULONG64 d2,ULONG64 d3,ULONG64 d7) {










}
static void pd_registration(HANDLE t,DWORD tid,int set_ok) {















}
static void pd_raw(const CONTEXT *c,ULONG64 camera,ULONG64 blur) {






















}
static void pd_result(int owner) {










}
static void pd_drain(int mode) {



















}
