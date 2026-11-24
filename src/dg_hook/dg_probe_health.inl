#define DG_PROBE_HEALTH 1
static volatile LONG pd_mode,pd_pending,pd_total,pd_camera,pd_blur,pd_b2,pd_b3;
static volatile LONG pd_open_fail,pd_suspend_fail,pd_get_fail,pd_set_fail,pd_resume_fail;
static volatile LONG pd_snapshot_fail;
static void pd_begin(int mode,ULONG64 d2,ULONG64 d3,ULONG64 d7){}
static void pd_registration(HANDLE t,DWORD tid,int set_ok){}
static void pd_raw(const CONTEXT *c,ULONG64 camera,ULONG64 blur){}
static void pd_result(int owner){}
static void pd_drain(int mode){}
