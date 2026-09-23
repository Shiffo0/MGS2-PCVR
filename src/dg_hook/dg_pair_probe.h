/* Experimental native-render observation. No second render is invoked here.
 * Shares B78 phase hooks; optional return mode temporarily redirects DR3
 * on the consumer thread and restores its entry address at return.
 * Fixed storage; VEH does no I/O, allocation, or blocking lock acquisition.
 * Include once in dg_hook.c (and in the standalone fixture).
 */
#ifndef DG_PAIR_PROBE_H
#define DG_PAIR_PROBE_H
#include "dg_build_profile.h"
#include <stdint.h>
#include <io.h>
#include <fcntl.h>
#define PP_CAPACITY DG_DIAGNOSTIC_CAPACITY(16384)
#define PP_STAGE_CALL 0x89DFDull
#define PP_STAGE_RETURN 0x89E00ull
#define PP_CONSUMER_CALL 0xFBF2ull
#define PP_CONSUMER_RETURN 0xFBF7ull
#define PP_DR_MASK (0xF0ull | (0xFFull << 24))
enum { PP_OFF, PP_RECORDING, PP_DONE, PP_INVALID };
enum { PP_STAGE_IN=1, PP_STAGE_OUT, PP_CONSUMER_IN,
       PP_CONSUMER_OUT, PP_PRESENT };
typedef struct {
    LONGLONG qpc;
    DWORD thread;
    unsigned event, present, stage, which, phase;
    ULONG64 channel, target, table, start, end, cursor, head, tail;
} PP_ROW;
static SRWLOCK pp_lock=SRWLOCK_INIT;
static PP_ROW pp_rows[PP_CAPACITY];
static unsigned pp_count;
static volatile LONG pp_status, pp_present_id, pp_started, pp_dumped;
static ULONG64 pp_base;
static DWORD pp_begin_ms;
static const char *pp_failure;
static char pp_output[MAX_PATH];
static char pp_request_marker[MAX_PATH], pp_request_output[MAX_PATH];
static ULONG64 pp_request_base;
static int pp_attempted,pp_return_mode;
static volatile LONG pp_pending_tid;
static ULONG64 pp_pending_rsp;
static unsigned pp_pending_which;
/* Only the measured same-thread call site is eligible. Unknown callers fail closed. */
static int pp_pending_context(const CONTEXT *c) {






return 0;

}


























static int cq_mode,cq_used,cq_pending,cp_mode;
static void cp_prepare(void);


































static void pp_prepare(ULONG64 base,const char *marker,const char *output) {










}
/* Worker poll: the owner can request capture after reaching gameplay.
   One request attempt per session, including a malformed/refused request. */
static int pp_poll(void (*log)(const char *,...)) {









return 0;

}

















































#include "dg_command_census.inl"
/* Called from phase_context AFTER the existing render-link event. The
 * host owns the execution trap; this observer must not consume/change it. */
static void pp_shared_event(unsigned bit,const CONTEXT *c) {



















}
static void pp_return_arm(CONTEXT *c) {





















}
static int pp_return_context(CONTEXT *c) {
















return 0;

}
static void pp_present(void) {











}
/* Worker only. Stop publication before taking a blocking lock or doing I/O. */
static void pp_flush(int finish,void (*log)(const char *,...)) {










































}
#endif
