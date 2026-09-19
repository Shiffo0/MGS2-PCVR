/* Read-only, bounded CPU witnesses at the native consumer entry/return.
 * No restore API: these bytes alone cannot authorize native render replay. */
#ifndef DG_NATIVE_STATE_PROBE_H
#define DG_NATIVE_STATE_PROBE_H
#include <stdint.h>
#include <string.h>
#include <stdio.h>
typedef int (*NS_READ)(void*,uint64_t,void*,unsigned);
typedef struct { const char *name; unsigned offset,size; } NS_FIELD;
static const NS_FIELD ns_fields[]={
 {"draw_count",0x174,4},{"descriptor_header",0x6c0,16},
 {"descriptor_state",0x6d0,0x138},{"binding_keys",0x808,16},
 {"index_base",0x818,4},{"binding_source_count_dirty",0x908,13}
};
typedef struct {
 uint64_t root,pools[2];
 unsigned char state[365],pool_state[2][20],global_count[4];
 int complete,error;
} NS_SAMPLE;
/* Explicit widths: pool+8 GPU identity; +1c cursor; +38 descriptor cursor
 * and +3c discard flag. Pointers are identities, never followed to GPU data. */
static int ns_take(NS_SAMPLE *s,uint64_t base,NS_READ read,void *opaque) {
 uint64_t again,roots[2];unsigned i,at=0;
 memset(s,0,sizeof *s);
 if(!base||base>UINT64_MAX-0x13d5288ull){s->error=1;return 0;}
#define NS_R(a,p,n) if(!read(opaque,(a),(p),(n))){s->error=2;return 0;}
 NS_R(base+0x13d5280,&s->root,8);
 if(!s->root||s->root>UINT64_MAX-0x915){s->error=1;return 0;}
 NS_R(s->root+0x630,s->pools,16);
 for(i=0;i<sizeof(ns_fields)/sizeof(ns_fields[0]);i++){
  NS_R(s->root+ns_fields[i].offset,s->state+at,ns_fields[i].size);
  at+=ns_fields[i].size;
 }
 for(i=0;i<2;i++){
  if(!s->pools[i]||s->pools[i]>UINT64_MAX-0x40){s->error=1;return 0;}
  NS_R(s->pools[i]+8,s->pool_state[i],8);
  NS_R(s->pools[i]+0x1c,s->pool_state[i]+8,4);
  NS_R(s->pools[i]+0x38,s->pool_state[i]+12,8);
 }
 NS_R(base+0x98add0,s->global_count,4);
 NS_R(base+0x13d5280,&again,8);
 NS_R(s->root+0x630,roots,16);
 if(again!=s->root||memcmp(roots,s->pools,16)){s->error=3;return 0;}
 s->complete=1;return 1;
#undef NS_R
}
static void ns_hex(FILE *f,const unsigned char *p,unsigned n){
 unsigned i;for(i=0;i<n;i++)fprintf(f,"%02x",p[i]);
}
static void ns_dump(FILE *f,unsigned slot,unsigned side,const NS_SAMPLE *s){
 unsigned i,at=0;
 fprintf(f,"# NSSTATE %u %u %d %d %llx %llx %llx\n",slot,side,s->complete,s->error,
  (unsigned long long)s->root,(unsigned long long)s->pools[0],(unsigned long long)s->pools[1]);
 if(!s->complete)return;
 for(i=0;i<sizeof(ns_fields)/sizeof(ns_fields[0]);i++){
  fprintf(f,"# NSFIELD %u %u %s ",slot,side,ns_fields[i].name);
  ns_hex(f,s->state+at,ns_fields[i].size);fputc('\n',f);at+=ns_fields[i].size;
 }
 for(i=0;i<2;i++){
  fprintf(f,"# NSFIELD %u %u pool%u ",slot,side,i);
  ns_hex(f,s->pool_state[i],20);fputc('\n',f);
 }
 fprintf(f,"# NSFIELD %u %u global_draw_count ",slot,side);
 ns_hex(f,s->global_count,4);fputc('\n',f);
}
#endif
