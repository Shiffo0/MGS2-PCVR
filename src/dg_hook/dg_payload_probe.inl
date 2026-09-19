
#include "dg_payload_probe.h"
#include "dg_payload_fields.h"
#include "dg_external_probe.h"
#include "dg_native_state_probe.h"
#include "dg_native_state_layout.h"
static NS_SAMPLE ns_samples[2][2];
static int ns_ready;
static CE_SAMPLE ce_samples[2][2];
static int ce_read(void *ctx,uint64_t a,void *dst,unsigned n){
 uint64_t cur=a,end;MEMORY_BASIC_INFORMATION m;
 if(!a||!n||a>UINT64_MAX-n)return 0;end=a+n;
 while(cur<end){uint64_t next;DWORD prot;
  if(!VirtualQuery((void*)(ULONG_PTR)cur,&m,sizeof m)||m.State!=MEM_COMMIT||(m.Protect&(PAGE_GUARD|PAGE_NOACCESS)))return 0;
  prot=m.Protect&0xff;if(prot!=PAGE_READONLY&&prot!=PAGE_READWRITE&&prot!=PAGE_WRITECOPY&&prot!=PAGE_EXECUTE_READ&&prot!=PAGE_EXECUTE_READWRITE&&prot!=PAGE_EXECUTE_WRITECOPY)return 0;
  if((uint64_t)(ULONG_PTR)m.BaseAddress>UINT64_MAX-m.RegionSize)return 0;next=(uint64_t)(ULONG_PTR)m.BaseAddress+m.RegionSize;if(next<=cur)return 0;cur=next;
 }
 return cq_read(ctx,a,dst,n);
}
#include <intrin.h>
static CP_SAMPLE cp_samples[2];
static int cp_seen[2],cp_pair[2],cp_tls_ready;
static unsigned cp_tls_index;
static uint64_t cp_tls[2][2][4];
static int cp_tls_valid[2][2];
static LONGLONG cp_times[2][2][2];
static void cp_prepare(void){
 unsigned char b[64];ULONG64 m;unsigned i;
 memset(ns_samples,0,sizeof ns_samples);
 ns_ready=cp_mode&&ns_layout(g_base,cq_read,NULL);
 for(i=0;i<2;i++){cp_seen[i]=0;cp_pair[i]=-1;cp_samples[i].count=cp_samples[i].bytes=cp_samples[i].changed=0;cp_samples[i].complete=0;cp_samples[i].error=0;cp_tls_valid[i][0]=cp_tls_valid[i][1]=0;}
 for(i=0;i<2;i++){ce_reset(&ce_samples[i][0]);ce_reset(&ce_samples[i][1]);}
 memset(cp_times,0,sizeof cp_times);memset(cp_tls,0,sizeof cp_tls);cp_tls_ready=0;if(!cp_mode)return;
 m=(ULONG64)GetModuleHandleA("MGSHDFix.asi");if(!m)return;
 {static const unsigned char cb[]={0x48,0x83,0xec,0x28,0x48,0x8b,0x89,0x60,0x1,0x0,0x0,0xba,0x8,0x0,0x0,0x0,0xe8,0xab,0xb9,0xf7,0xff,0x8b,0x15,0x81,0xbb,0x24,0x0,0x65,0x48,0x8b,0xc,0x25,0x58,0x0,0x0,0x0,0x41,0xb8,0x90,0x1,0x0,0x0,0x48,0x8b,0xc,0xd1,0x49,0x89,0x4,0x8,0x48,0x83,0xc4,0x28,0xc3};static const unsigned char wr[]={0x48,0x89,0x5c,0x24,0x8,0x57,0x48,0x83,0xec,0x20,0x48,0x8b,0xd9,0xe8,0x8e,0xfc,0xff,0xff,0x48,0x8d,0x3d,0x27,0xf,0x24,0x0,0x48,0x89,0x7c,0x24,0x38,0x48,0x8b,0xcf,0xe8,0x9e,0xe3,0x9,0x0,0x85,0xc0,0xf,0x85,0x80,0x0,0x0,0x0,0x81,0x3d,0x54,0xf,0x24,0x0,0xff,0xff,0xff,0x7f,0x74,0x5f,0x48,0x8b,0x5,0xcf,0xe,0x24};
 if(!cq_read(NULL,m+0xb5590,b,sizeof cb)||memcmp(b,cb,sizeof cb)||!cq_read(NULL,m+0xb5930,b,sizeof wr)||memcmp(b,wr,sizeof wr))return;}
 if(!cq_read(NULL,m+0x30112c,&cp_tls_index,4)||cp_tls_index>1023)return;
 cp_tls_ready=1;
}
static void cp_thread_state(unsigned slot,unsigned side){
 uint64_t table,block;static const unsigned off[]={0x160,0x170,0x178,0x190};unsigned i;
 cp_tls_valid[slot][side]=0;if(!cp_tls_ready)return;
 table=__readgsqword(0x58);
 if(!table||!cq_read(NULL,table+8ull*cp_tls_index,&block,8)||!block)return;
 for(i=0;i<4;i++){cp_tls[slot][side][i]=0;if(!cq_read(NULL,block+off[i],&cp_tls[slot][side][i],i?8:1))return;}
 cp_tls_valid[slot][side]=1;
}
static void cp_take(int n,unsigned side){
 unsigned slot=cq_slots[n];LARGE_INTEGER t;CP_SAMPLE *p;
 if(!cp_mode)return;if(slot>1){pp_fail("payload_slot");return;}
 if(!side){if(cp_seen[slot])return;cp_seen[slot]=1;cp_pair[slot]=n;}
 if(cp_pair[slot]!=n)return;
 p=&cp_samples[slot];cp_thread_state(slot,side);QueryPerformanceCounter(&t);cp_times[slot][side][0]=t.QuadPart;
 if(ns_ready)ns_take(&ns_samples[slot][side],g_base,ce_read,NULL);
 if((!side&&!cp_plan(p,&cq_samples[n]))||!cp_copy(p,side,cq_read,NULL)||!cp_companion(p,&cq_samples[n],side))pp_fail("payload_incomplete");
 else if(!ce_take(&ce_samples[slot][side],&cq_samples[n],p,side,ce_read,NULL))pp_fail("external_incomplete");
 QueryPerformanceCounter(&t);cp_times[slot][side][1]=t.QuadPart;
}
static void cp_dump(FILE *f){
 unsigned slot,i,j;if(!cp_mode)return;
 fprintf(f,"# NATIVE_STATE version=1 layout=%d cpu_only=1 restore=0\n",ns_ready);
 fprintf(f,"# PAYLOAD version=1 budget=%u max_ranges=%u complete=%d tls_layout=%d\n",CP_BYTES,CP_LIMIT,cp_seen[0]&&cp_seen[1]&&cp_samples[0].complete&&cp_samples[1].complete,cp_tls_ready);
 for(slot=0;slot<2;slot++){
  CP_SAMPLE *p=&cp_samples[slot];
  ns_dump(f,slot,0,&ns_samples[slot][0]);ns_dump(f,slot,1,&ns_samples[slot][1]);
  ce_dump(f,slot,0,&ce_samples[slot][0]);ce_dump(f,slot,1,&ce_samples[slot][1]);
  fprintf(f,"# CPPPAIR %u %d %d %u %u %u %d %lld %lld %lld %lld\n",slot,cp_pair[slot],p->error,p->count,p->bytes,p->changed,p->complete,cp_times[slot][0][0],cp_times[slot][0][1],cp_times[slot][1][0],cp_times[slot][1][1]);
  fprintf(f,"# CPTLS %u %d %d",slot,cp_tls_valid[slot][0],cp_tls_valid[slot][1]);
  for(i=0;i<2;i++)for(j=0;j<4;j++)fprintf(f," %llx",cp_tls[slot][i][j]);fprintf(f,"\n");
  /* Never print uninitialized or partial data as byte changes. */
  if(cp_pair[slot]>=0&&cp_pair[slot]<cq_used)cpf_dump(f,slot,cp_pair[slot],&cq_samples[cp_pair[slot]],p);
  if(!p->complete||p->error)continue;
  for(i=0;i<p->count;i++){
   CP_RANGE *r=&p->ranges[i];fprintf(f,"# CPRANGE %u %u %llx %u\n",slot,i,r->address,r->size);
   for(j=0;j<r->size;j++)if(p->before[r->offset+j]!=p->after[r->offset+j])fprintf(f,"# CPBYTE %u %u %u %02x %02x\n",slot,i,j,p->before[r->offset+j],p->after[r->offset+j]);
  }
 }
}
