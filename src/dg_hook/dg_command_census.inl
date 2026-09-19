#include "dg_command_census.h"

static CQ_SAMPLE cq_samples[8];
static SRWLOCK cq_lock=SRWLOCK_INIT;
static DWORD cq_threads[8];static unsigned cq_slots[8];
static LONGLONG cq_qpc[8][2];
static const unsigned cq_rvas[]={0xaa58e1,0x8526f0,0xaa58d8,0xaa58e0,0xaa58c0,0xaa58dc,0xaa4770};
static const unsigned cq_sizes[]={1,4,4,1,8,4,4};
static uint64_t cq_globals[8][2][sizeof(cq_rvas)/sizeof(cq_rvas[0])];
static int cq_read(void *ctx,uint64_t a,void *dst,unsigned n) {
 (void)ctx;__try {memcpy(dst,(void*)(ULONG_PTR)a,n);return 1;}
 __except(EXCEPTION_EXECUTE_HANDLER){return 0;}
}
static int cq_state(int index,int side) {
 unsigned i;LARGE_INTEGER q;QueryPerformanceCounter(&q);cq_qpc[index][side]=q.QuadPart;
 for(i=0;i<sizeof(cq_rvas)/sizeof(cq_rvas[0]);i++)
  if(!cq_read(NULL,pp_base+cq_rvas[i],&cq_globals[index][side][i],cq_sizes[i]))return 0;
 return 1;
}
#include "dg_payload_probe.inl"
static void cq_entry(const PP_ROW *r) {
 CQ_SAMPLE *s;int n;if(!cq_mode)return;
 if(cq_pending>=0 || cq_used>=8){pp_fail("census_pair_overflow");return;}
 if(!TryAcquireSRWLockExclusive(&cq_lock)){pp_fail("census_contention");return;}
 n=cq_used++;cq_pending=n;s=&cq_samples[n];cq_threads[n]=GetCurrentThreadId();cq_slots[n]=r->which;
 if(!cq_begin(s,r->start,r->end,r->cursor,r->head,r->tail,cq_read,NULL)||!cq_state(n,0)) {
  pp_fail("census_entry_incomplete");ReleaseSRWLockExclusive(&cq_lock);return;
 }
 cp_take(n,0);
 ReleaseSRWLockExclusive(&cq_lock);
}
static void cq_return(const PP_ROW *r) {
 int n=cq_pending;if(!cq_mode)return;
 if(n<0 || cq_threads[n]!=GetCurrentThreadId() || cq_slots[n]!=r->which){pp_fail("census_return_owner");return;}
 if(!TryAcquireSRWLockExclusive(&cq_lock)){pp_fail("census_contention");return;}
 if(!cq_end(&cq_samples[n],r->start,r->end,r->cursor,r->head,r->tail,cq_read,NULL)||!cq_state(n,1))pp_fail("census_return_incomplete");
 if(!cq_samples[n].error)cp_take(n,1);
 cq_pending=-1;ReleaseSRWLockExclusive(&cq_lock);
}
/* Worker writes comment records so original pair CSV remains parseable. */
static void cq_dump(FILE *f) {
 int n;unsigned i;if(!cq_mode)return;
 AcquireSRWLockExclusive(&cq_lock);
 fprintf(f,"# CENSUS version=1 max_nodes=%d pending=%d pairs=%d payload_mode=%d\n",CQ_LIMIT,cq_pending,cq_used,cp_mode);
 for(n=0;n<cq_used;n++) {
  CQ_SAMPLE *s=&cq_samples[n];
  fprintf(f,"# CQPAIR %d %lu %u %d %u %u %lld %lld %llx %llx %llx %llx %llx\n",n,cq_threads[n],cq_slots[n],s->error,s->count,s->changed,cq_qpc[n][0],cq_qpc[n][1],s->start,s->end,s->cursor,s->head,s->tail);
  for(i=0;i<s->count;i++) {
   CQ_NODE *v=&s->nodes[i];fprintf(f,"# CQNODE %d %u %llx %llx %llx %llx %llx %llx %llx\n",n,i,v->address,v->before[0],v->before[1],v->before[2],v->after[0],v->after[1],v->after[2]);
  }
  for(i=0;i<sizeof(cq_rvas)/sizeof(cq_rvas[0]);i++)fprintf(f,"# CQGLOBAL %d %x %u %llx %llx\n",n,cq_rvas[i],cq_sizes[i],cq_globals[n][0][i],cq_globals[n][1][i]);
 }
 cp_dump(f);
 ReleaseSRWLockExclusive(&cq_lock);
}
