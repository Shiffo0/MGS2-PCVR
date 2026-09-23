/* Bounded observation only; never changes capture selection or debug slots. */
#define DG_CAPTURE_IDENTITY 1
typedef struct {
    LONGLONG qpc;DWORD tid;LONG frame;int kind,slot,sequence;
    uint64_t capture;DG_HOOK_HANDOFF h;
} CI_ROW;
static CI_ROW ci_rows[DG_DIAGNOSTIC_CAPACITY(256)];
static unsigned ci_count;
static LONG ci_lost;
static SRWLOCK ci_lock=SRWLOCK_INIT;
static void ci_record(int kind,int slot,int sequence,const DG_HOOK_HANDOFF *h,uint64_t capture) {














}
static void ci_native(unsigned bit,const CONTEXT *c) {















}
static void ci_published(int eye,uint64_t id) {








}
/* Worker only; publication can race draining but each row is copied atomically. */
static void ci_drain(void) {

















}
