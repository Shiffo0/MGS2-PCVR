#ifndef DG_CAMERA_SHUTTER_H
#define DG_CAMERA_SHUTTER_H
#include <stdint.h>
typedef struct {
    uint64_t sample,source,actor,manager,now_ms;
    unsigned age_ms,mask;
    int valid,denied,down,ready,item,desired_item,native_down,pad_allowed;
} DG_CAMERA_INPUT;
typedef struct {
    uint64_t sample,source,actor,manager,now_ms;
    int seen,blocked,down,item,ready;
} DG_CAMERA_SHUTTER;
static int dg_camera_item(int item) {return item==15 || item==21;}
/* One native status pulse. No retry, press-field write or held repeat. */
static unsigned dg_camera_shutter_step(DG_CAMERA_SHUTTER *s,const DG_CAMERA_INPUT *p) {
    int identity=!s->seen || p->source!=s->source || p->actor!=s->actor ||
        p->manager!=s->manager || p->item!=s->item;
    int fresh=identity || p->sample>s->sample;
    int valid=p->valid && p->sample && p->source && p->actor && p->manager &&
        p->age_ms<=100 && p->now_ms>=s->now_ms &&
        (identity || (p->sample>=s->sample && (fresh || p->down==s->down)));
    unsigned pulse=0;
    if(identity || !p->ready || !s->ready)s->blocked=1;
    if(!valid || p->denied || !p->pad_allowed || p->native_down ||
       !dg_camera_item(p->item) || p->desired_item!=p->item || !p->mask ||
       (p->mask&(p->mask-1)))s->blocked=1;
    else if(p->ready && fresh) {
        if(!p->down)s->blocked=0;
        else if(!s->blocked){pulse=p->mask;s->blocked=1;}
    }
    if(identity || p->sample>s->sample)s->sample=p->sample;
    if(p->now_ms>s->now_ms)s->now_ms=p->now_ms;
    s->source=p->source;s->actor=p->actor;s->manager=p->manager;
    s->item=p->item;s->down=p->down;s->ready=p->ready;s->seen=1;
    return pulse;
}
#endif
