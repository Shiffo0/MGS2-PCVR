/* Caller serializes access. Payload is the complete camera handoff, not only eye. */
#ifndef DG_RENDER_LINK_H
#define DG_RENDER_LINK_H
#include <stdint.h>
#include <string.h>
typedef struct {
    DG_HOOK_HANDOFF payload;
    uint64_t ms;
    uint32_t frame;
    int sequence, valid, seen;
} DG_LINK_STAMP;
typedef struct {
    DG_LINK_STAMP prepared[2], pending;
    uint32_t owner;
    unsigned consumers;
} DG_RENDER_LINK;
static void dg_link_reset(DG_RENDER_LINK *s) { memset(s,0,sizeof *s); }
static int dg_link_thread(DG_RENDER_LINK *s,uint32_t tid) {
    if(!tid)return 0;
    if(!s->owner)s->owner=tid;
    if(s->owner!=tid){dg_link_reset(s);return 0;}
    return 1;
}
static int dg_link_fresh(const DG_LINK_STAMP *p,uint32_t frame,uint64_t ms) {
    return p->valid&&p->payload.valid&&(p->payload.eye==0||p->payload.eye==1)&&
        (uint32_t)(frame-p->frame)<=2&&ms>=p->ms&&ms-p->ms<=100;
}
static void dg_link_stage(DG_RENDER_LINK *s,int buffer,uint32_t tid,uint32_t frame,
                          uint64_t ms,const DG_HOOK_HANDOFF *h,int sequence) {
    DG_LINK_STAMP *p;
    if(!dg_link_thread(s,tid))return;
    if(buffer<0||buffer>1){dg_link_reset(s);return;}
    p=&s->prepared[buffer];
    /* Repeated preparation in one frame is ambiguous, not last-writer-wins. */
    if(p->seen&&p->frame==frame){p->valid=0;return;}
    memset(p,0,sizeof *p);p->seen=1;p->frame=frame;p->ms=ms;p->sequence=sequence;
    if(h){p->payload=*h;p->valid=h->valid&&!(sequence&1);}
}
static void dg_link_consume(DG_RENDER_LINK *s,int buffer,uint32_t tid,
                            uint32_t frame,uint64_t ms) {
    if(!dg_link_thread(s,tid))return;
    if(s->consumers<2)s->consumers++;
    if(buffer<0||buffer>1){memset(&s->pending,0,sizeof s->pending);return;}
    if(s->consumers==1&&dg_link_fresh(&s->prepared[buffer],frame,ms))
        s->pending=s->prepared[buffer];
    else memset(&s->pending,0,sizeof s->pending);
    s->prepared[buffer].valid=0; /* A preparation can be consumed only once. */
}
static int dg_link_present(DG_RENDER_LINK *s,uint32_t tid,uint32_t frame,uint64_t ms,
                           DG_HOOK_HANDOFF *out,int *sequence) {
    int ok;
    memset(out,0,sizeof *out);*sequence=0;
    if(!dg_link_thread(s,tid))return 0;
    ok=s->consumers==1&&dg_link_fresh(&s->pending,frame,ms);
    if(ok){*out=s->pending.payload;*sequence=s->pending.sequence;}
    s->consumers=0;memset(&s->pending,0,sizeof s->pending);
    return ok;
}
#endif
