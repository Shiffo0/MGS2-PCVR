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
static HUD_EVENT hud_events[DG_DIAGNOSTIC_CAPACITY(1024)];
static unsigned hud_count;
















static void hud_capture(int eye, uint64_t capture_id) {










}
static int hud_context(CONTEXT *c) {











































































return 0;

}
static void hud_debug_registers(CONTEXT *c,int arm) {










}
/* Same getter signatures and bounded layout traversal as the verified reader.
 * Only a non-empty SP_POLY cursor is eligible; SP_EMPTY fields are not pointers.
 */




































































static void hud_stop(void) {










}
static void hud_poll(void) {























}
