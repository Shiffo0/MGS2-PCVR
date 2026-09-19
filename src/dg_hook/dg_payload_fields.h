/* Worker-only extraction from retained snapshots. Never reads game memory. */
#ifndef DG_PAYLOAD_FIELDS_H
#define DG_PAYLOAD_FIELDS_H
#include "dg_payload_probe.h"
static unsigned cpf_offsets(unsigned op,unsigned *o){
 switch(op){case 12:o[0]=0x20;o[1]=0x28;return 2;case 32:o[0]=8;return 1;
 case 33:o[0]=0x188;o[1]=0x198;o[2]=0x1c0;o[3]=0x1b0;o[4]=0x1b4;o[5]=0x1b8;o[6]=0x1f0;o[7]=0x230;o[8]=0x240;o[9]=0x190;return 10;default:return 0;}
}
static unsigned cpf_width(unsigned op,unsigned off){return op==33&&(off==0x190||off==0x1b0||off==0x1b4||off==0x1b8)?4:8;}
static int cpf_get_width(const CP_SAMPLE *p,uint64_t payload,unsigned off,unsigned width,uint64_t *before,uint64_t *after){
 unsigned lo=0,hi=p->count;uint64_t a;const CP_RANGE *r;unsigned at;
 if((width!=4&&width!=8)||!p->complete||p->error||payload>UINT64_MAX-off)return 0;a=payload+off;
 while(lo<hi){unsigned mid=lo+(hi-lo)/2;if(p->ranges[mid].address<=a)lo=mid+1;else hi=mid;}
 if(!lo)return 0;r=&p->ranges[lo-1];
 if(a-r->address>r->size||r->size-(a-r->address)<width)return 0;
 if(r->offset>p->bytes||r->size>p->bytes-r->offset||p->bytes>CP_BYTES)return 0;
 at=r->offset+(unsigned)(a-r->address);*before=*after=0;memcpy(before,p->before+at,width);memcpy(after,p->after+at,width);return 1;
}
static int cpf_get(const CP_SAMPLE *p,uint64_t a,unsigned off,uint64_t *b,uint64_t *e){return cpf_get_width(p,a,off,8,b,e);}
static void cpf_dump(FILE *f,unsigned slot,int pair,const CQ_SAMPLE *q,const CP_SAMPLE *p){
 unsigned i,j,off[10],count=0,missing=0;
 for(i=0;i<q->count;i++){
  unsigned op=(unsigned)q->nodes[i].before[0],n=cpf_offsets(op,off);uint64_t a=q->nodes[i].before[1];
  for(j=0;j<n;j++){uint64_t b=0,e=0;unsigned width=cpf_width(op,off[j]);int ok=cpf_get_width(p,a,off[j],width,&b,&e);count++;missing+=!ok;
   if(ok)fprintf(f,"# CPFIELD %u %d %u %u %llx %x %u %llx %llx\n",slot,pair,i,op,a,off[j],width,b,e);
   else fprintf(f,"# CPFIELD_MISSING %u %d %u %u %llx %x %u\n",slot,pair,i,op,a,off[j],width);
  }
 }
 fprintf(f,"# CPFIELDS slot=%u expected=%u missing=%u\n",slot,count,missing);
}
#endif
