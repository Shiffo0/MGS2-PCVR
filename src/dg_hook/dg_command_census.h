/* Pure bounded census; no payload dereferences, allocation, or native writes. */
#ifndef DG_COMMAND_CENSUS_H
#define DG_COMMAND_CENSUS_H
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#define CQ_LIMIT 65536
#define CQ_HASH 131072
typedef int (*CQ_READ)(void*,uint64_t,void*,unsigned);
typedef struct {uint64_t address,before[3],after[3];} CQ_NODE;
typedef struct {
 uint64_t start,end,cursor,head,tail;unsigned count,changed;int error;
 CQ_NODE nodes[CQ_LIMIT];uint64_t seen[CQ_HASH];
} CQ_SAMPLE;
enum {CQ_OK,CQ_RANGE,CQ_READ_FAIL,CQ_CYCLE,CQ_OVERFLOW,CQ_ARENA_CHANGED};
static int cq_range(const CQ_SAMPLE *s,uint64_t a) {
 return !(a&7) && a>=s->start && a<=s->cursor && s->cursor-a>=24;
}
static int cq_begin(CQ_SAMPLE *s,uint64_t start,uint64_t end,uint64_t cursor,uint64_t head,uint64_t tail,CQ_READ read,void *ctx) {
 uint64_t a=head;unsigned h;
 /* Reset only bookkeeping/hash; initialize node bytes when actually used. */
 memset(s,0,offsetof(CQ_SAMPLE,nodes));memset(s->seen,0,sizeof s->seen);s->start=start;s->end=end;s->cursor=cursor;s->head=head;s->tail=tail;
 if(!start||cursor<start||end<cursor){s->error=CQ_RANGE;return 0;}
 while(a) {
  if(!cq_range(s,a)){s->error=CQ_RANGE;break;}
  h=(unsigned)((a>>3)^(a>>17))&(CQ_HASH-1);
  while(s->seen[h]&&s->seen[h]!=a)h=(h+1)&(CQ_HASH-1);
  if(s->seen[h]){s->error=CQ_CYCLE;break;}
  if(s->count==CQ_LIMIT){s->error=CQ_OVERFLOW;break;}
  s->seen[h]=a;memset(&s->nodes[s->count],0,sizeof s->nodes[0]);s->nodes[s->count].address=a;
  if(!read(ctx,a,s->nodes[s->count].before,24)){s->error=CQ_READ_FAIL;break;}
  a=s->nodes[s->count++].before[2];
 }
 return s->error==CQ_OK;
}
static int cq_end(CQ_SAMPLE *s,uint64_t start,uint64_t end,uint64_t cursor,uint64_t head,uint64_t tail,CQ_READ read,void *ctx) {
 unsigned i;if(s->error)return 0;
 if(s->start!=start||s->end!=end||s->cursor!=cursor||s->head!=head||s->tail!=tail){s->error=CQ_ARENA_CHANGED;return 0;}
 for(i=0;i<s->count;i++) {
  if(!read(ctx,s->nodes[i].address,s->nodes[i].after,24)){s->error=CQ_READ_FAIL;return 0;}
  if(memcmp(s->nodes[i].before,s->nodes[i].after,24))s->changed++;
 }
 return 1;
}
#endif
