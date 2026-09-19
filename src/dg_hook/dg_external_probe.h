/* Bounded read-only external graph snapshots. No allocation or output in capture. */
#ifndef DG_EXTERNAL_PROBE_H
#define DG_EXTERNAL_PROBE_H
#include "dg_payload_probe.h"
#define CE_LIMIT 4096
#define CE_BYTES (512*1024)
enum {CE_OK,CE_SOURCE,CE_ADDRESS,CE_LIMIT_ERROR,CE_READ,CE_LAYOUT};
enum {CE_OBJECT=1,CE_DESCRIPTOR,CE_LOOKUP,CE_RECORDS};
typedef struct {uint64_t address;unsigned size,offset,kind;} CE_RANGE;
typedef struct {unsigned count,bytes;int error,complete;CE_RANGE ranges[CE_LIMIT];unsigned char data[CE_BYTES];} CE_SAMPLE;
static void ce_reset(CE_SAMPLE *s){s->count=s->bytes=0;s->error=0;s->complete=0;}
static const unsigned char *ce_add(CE_SAMPLE *s,uint64_t a,unsigned n,unsigned kind,CQ_READ read,void *ctx){
 unsigned i;CE_RANGE *r;
 if(!a||!n||a>UINT64_MAX-n||(a&3)){s->error=CE_ADDRESS;return NULL;}
 for(i=0;i<s->count;i++){r=&s->ranges[i];if(r->address==a&&r->size==n&&r->kind==kind)return s->data+r->offset;}
 if(s->count==CE_LIMIT||n>CE_BYTES-s->bytes){s->error=CE_LIMIT_ERROR;return NULL;}
 if(!read(ctx,a,s->data+s->bytes,n)){s->error=CE_READ;return NULL;}
 r=&s->ranges[s->count++];r->address=a;r->size=n;r->offset=s->bytes;r->kind=kind;s->bytes+=n;
 return s->data+r->offset;
}
static uint64_t ce_u64(const unsigned char *b,unsigned o){uint64_t v;memcpy(&v,b+o,8);return v;}
static unsigned ce_u32(const unsigned char *b,unsigned o){unsigned v;memcpy(&v,b+o,4);return v;}
static const unsigned char *ce_payload(const CP_SAMPLE *p,uint64_t a,unsigned n,unsigned side){
 unsigned lo=0,hi=p->count;const CP_RANGE *r;
 if(p->error||p->bytes>CP_BYTES||side>1)return NULL;
 while(lo<hi){unsigned m=lo+(hi-lo)/2;if(p->ranges[m].address<=a)lo=m+1;else hi=m;}
 if(!lo)return NULL;r=&p->ranges[lo-1];
 if(a-r->address>r->size||n>r->size-(a-r->address)||r->offset>p->bytes||r->size>p->bytes-r->offset)return NULL;
 return (side?p->after:p->before)+r->offset+(unsigned)(a-r->address);
}
static int ce_take(CE_SAMPLE *s,const CQ_SAMPLE *q,const CP_SAMPLE *p,unsigned side,CQ_READ read,void *ctx){
 unsigned i;ce_reset(s);
 if(q->error||p->error){s->error=CE_SOURCE;return 0;}
 for(i=0;i<q->count;i++){
  unsigned op=(unsigned)q->nodes[i].before[0];const unsigned char *b,*h,*v;uint64_t obj,begin,end,base,start,a;unsigned ix,hi=0,count,j;
  if(op!=12&&op!=33)continue;
  b=ce_payload(p,q->nodes[i].before[1],cp_size(op),side);if(!b){s->error=CE_SOURCE;return 0;}
  if(op==12){for(j=0;j<2;j++){obj=ce_u64(b,0x20+8*j);if(obj&&!ce_add(s,obj,12,CE_DESCRIPTOR,read,ctx))return 0;}continue;}
  obj=ce_u64(b,0x188);if(obj&7){s->error=CE_ADDRESS;return 0;}
  h=ce_add(s,obj,0x68,CE_OBJECT,read,ctx);if(!h)return 0;
  base=ce_u64(h,0x28);begin=ce_u64(h,0x58);end=ce_u64(h,0x60);ix=ce_u32(b,0x190);
  if(end<begin||((end-begin)&3)||(begin&3)){s->error=CE_LAYOUT;return 0;}
  /* The native helper sign-extends EDX, then uses an unsigned range test. */
  if(ix<0x80000000u&&(uint64_t)ix<(end-begin)/4){
   a=begin+4ull*ix;v=ce_add(s,a,4,CE_LOOKUP,read,ctx);if(!v)return 0;hi=ce_u32(v,0)>>16;
  }
  /* Native add uses a 32-bit register: reject wrap rather than invent a range. */
  if(ce_u32(b,0x1b0)>0xffffffffu-hi){s->error=CE_LAYOUT;return 0;}
  start=(uint64_t)ce_u32(b,0x1b0)+hi;count=ce_u32(b,0x1b4);
  if(!count)continue;
  if(count>CE_BYTES/0x60||base>UINT64_MAX-start*0x60){s->error=CE_LIMIT_ERROR;return 0;}
  a=base+start*0x60;if(!base||!ce_add(s,a,count*0x60,CE_RECORDS,read,ctx))return 0;
 }
 s->complete=1;return 1;
}
static void ce_dump(FILE *f,unsigned slot,unsigned side,const CE_SAMPLE *s){
 unsigned i,j;fprintf(f,"# CEXT %u %u %d %d %u %u\n",slot,side,s->complete,s->error,s->count,s->bytes);
 /* Export full retained bytes, including unchanged values; never reread live memory. */
 for(i=0;i<s->count;i++){const CE_RANGE *r=&s->ranges[i];fprintf(f,"# CERANGE %u %u %u %u %llx %u ",slot,side,i,r->kind,r->address,r->size);
  for(j=0;j<r->size;j++)fprintf(f,"%02x",s->data[r->offset+j]);fprintf(f,"\n");}
}
#endif
