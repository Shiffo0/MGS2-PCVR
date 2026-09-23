#include "dg_command_census.h"

static CQ_SAMPLE cq_samples[8];
static SRWLOCK cq_lock=SRWLOCK_INIT;
static DWORD cq_threads[8];static unsigned cq_slots[8];
static LONGLONG cq_qpc[8][2];
static const unsigned cq_rvas[]={0xaa58e1,0x8526f0,0xaa58d8,0xaa58e0,0xaa58c0,0xaa58dc,0xaa4770};
static const unsigned cq_sizes[]={1,4,4,1,8,4,4};
static uint64_t cq_globals[8][2][sizeof(cq_rvas)/sizeof(cq_rvas[0])];










#include "dg_payload_probe.inl"



















/* Worker writes comment records so original pair CSV remains parseable. */















