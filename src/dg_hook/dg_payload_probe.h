#ifndef DG_PAYLOAD_PROBE_H
#define DG_PAYLOAD_PROBE_H
#include "dg_command_census.h"
#define CP_LIMIT 4096
#define CP_BYTES (1024*1024)
typedef struct {uint64_t address;unsigned size,offset;} CP_RANGE;
typedef struct {unsigned count,bytes,changed;int error,complete;CP_RANGE ranges[CP_LIMIT];unsigned char before[CP_BYTES],after[CP_BYTES];} CP_SAMPLE;
enum {CP_OK,CP_RANGE_ERROR,CP_RANGE_LIMIT,CP_BYTE_LIMIT,CP_READ_ERROR,CP_LAYOUT_ERROR};
static unsigned cp_size(unsigned op){switch(op){case 11:return 0x90;case 12:return 0x52;case 32:return 0x20;case 33:return 0x248;default:return 0;}}
static void cp_sift(CP_RANGE *r,unsigned n,unsigned root){
 unsigned child;CP_RANGE t;
 while(root<n/2){child=root*2+1;if(child+1<n&&r[child].address<r[child+1].address)child++;
 if(r[root].address>=r[child].address)break;t=r[root];r[root]=r[child];r[child]=t;root=child;}
}
static int cp_plan(CP_SAMPLE *p,const CQ_SAMPLE *q){
 unsigned i,j,n;CP_RANGE t;uint64_t end,prev;
 p->count=p->bytes=p->changed=0;p->error=CP_OK;p->complete=0;
 if(q->error){p->error=CP_RANGE_ERROR;return 0;}
 for(i=0;i<q->count;i++){
  unsigned size=cp_size((unsigned)q->nodes[i].before[0]);uint64_t a=q->nodes[i].before[1];if(!size)continue;
  if((a&7)||a<q->start||a>q->cursor||size>q->cursor-a){p->error=CP_RANGE_ERROR;return 0;}
  if(p->count==CP_LIMIT){p->error=CP_RANGE_LIMIT;return 0;}
  p->ranges[p->count].address=a;p->ranges[p->count++].size=size;
 }
 n=p->count;for(i=n/2;i>0;i--)cp_sift(p->ranges,n,i-1);
 for(i=n;i>1;i--){t=p->ranges[0];p->ranges[0]=p->ranges[i-1];p->ranges[i-1]=t;cp_sift(p->ranges,i-1,0);}
 for(i=0,j=0;i<n;i++){
  end=p->ranges[i].address+p->ranges[i].size;
  if(j && p->ranges[i].address<=(prev=p->ranges[j-1].address+p->ranges[j-1].size)){
   if(end>prev){uint64_t len=end-p->ranges[j-1].address;if(len>CP_BYTES){p->error=CP_BYTE_LIMIT;return 0;}p->ranges[j-1].size=(unsigned)len;}
  }else p->ranges[j++]=p->ranges[i];
 }
 p->count=j;
 for(i=0;i<j;i++){if(p->ranges[i].size>CP_BYTES-p->bytes){p->error=CP_BYTE_LIMIT;return 0;}p->ranges[i].offset=p->bytes;p->bytes+=p->ranges[i].size;}
 return 1;
}
static int cp_copy(CP_SAMPLE *p,int side,CQ_READ read,void *ctx){
 unsigned i;if(p->error)return 0;
 for(i=0;i<p->count;i++){CP_RANGE *r=&p->ranges[i];if(!read(ctx,r->address,(side?p->after:p->before)+r->offset,r->size)){p->error=CP_READ_ERROR;return 0;}}
 if(side){p->changed=0;for(i=0;i<p->bytes;i++)p->changed+=p->before[i]!=p->after[i];p->complete=1;}return 1;
}

/* Validate copied companion pointer, never dereference the game pointer. */
static int cp_companion(CP_SAMPLE *p,const CQ_SAMPLE *q,int side){
 unsigned i;if(p->error)return 0;
 for(i=0;i<q->count;i++)if((unsigned)q->nodes[i].before[0]==33){
  uint64_t a=q->nodes[i].before[1],ptr=0,field;unsigned lo=0,hi=p->count,at;CP_RANGE *r;
  if(a>UINT64_MAX-0x248)goto bad;field=a+0x198;
  while(lo<hi){unsigned m=lo+(hi-lo)/2;if(p->ranges[m].address<=field)lo=m+1;else hi=m;}
  if(!lo)goto bad;r=&p->ranges[lo-1];
  if(field-r->address>r->size||r->size-(field-r->address)<8)goto bad;
  at=r->offset+(unsigned)(field-r->address);if(p->bytes>CP_BYTES||at>p->bytes||p->bytes-at<8)goto bad;
  memcpy(&ptr,(side?p->after:p->before)+at,8);
  if(ptr!=a+0x1d0)goto bad;
 }
 return 1;
 bad:p->error=CP_LAYOUT_ERROR;p->complete=0;return 0;
}

#endif
