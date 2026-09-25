#ifndef DG_CAMERA_ZOOM_H
#define DG_CAMERA_ZOOM_H
#include <math.h>
#include <stdint.h>
typedef struct {
    uint64_t sample,source,manager,time;
    float y;
    int seen,blocked,direction;
} DG_CAMERA_ZOOM;
/* Repeated coherent samples hold input, but never advance the native camera.
 * Invalid/context-lost samples latch neutral rearm. */
static int dg_camera_zoom_step(DG_CAMERA_ZOOM *s,uint64_t sample,uint64_t source,
    uint64_t manager,uint64_t now,unsigned age,int allowed,float y) {
    int identity=!s->seen || source!=s->source || manager!=s->manager;
    int fresh=identity || sample>s->sample;
    int valid=allowed && sample && source && manager && age<=100 &&
        y>=-1 && y<=1 && now>=s->time &&
        (identity || (sample>=s->sample && (fresh || y==s->y)));
    if(identity){s->blocked=1;s->direction=0;}
    if(!valid){s->blocked=1;s->direction=0;}
    else {
        if(fresh && fabs((double)y)<=0.20){s->blocked=0;s->direction=0;}
        if(!s->blocked) {
            if(y>=0.30f)s->direction=1;
            else if(y<=-0.30f)s->direction=-1;
            else if((s->direction>0 && y<=0.20f)||(s->direction<0 && y>=-0.20f))s->direction=0;
        }
    }
    if(identity || sample>s->sample)s->sample=sample;
    if(now>s->time)s->time=now;
    s->source=source;s->manager=manager;s->seen=1;s->y=y;
    return s->direction;
}




static int dg_psg_zoom_step(DG_CAMERA_ZOOM *s,uint64_t sample,uint64_t source,
    uint64_t manager,uint64_t now,unsigned age,int allowed,float grip) {
    int identity=!s->seen || source!=s->source || manager!=s->manager;
    int valid=allowed && sample && source && manager && age<=100 &&
        (grip==-1.0f || grip==0.0f || grip==1.0f) && now>=s->time &&
        (identity || sample>=s->sample);
    if(identity)s->blocked=1;
    if(grip==0.0f)s->blocked=0;
    s->direction=(valid && !s->blocked)?(grip>0?1:grip<0?-1:0):0;
    if(identity || sample>s->sample)s->sample=sample;
    if(now>s->time)s->time=now;
    s->source=source;s->manager=manager;s->seen=1;s->y=grip;
    return s->direction;
}
/* Only add a local synthetic direction when physical native input is neutral.
 * No global pad writes, no synthetic invocation of the native actor. */
static void dg_camera_zoom_merge(int direction,unsigned out_mask,unsigned in_mask,
    uint64_t *status,uint64_t *out_pressure,uint64_t *in_pressure) {
    if(!out_mask || !in_mask || (out_mask&(out_mask-1)) || (in_mask&(in_mask-1)) ||
       out_mask==in_mask || (*status&(out_mask|in_mask)))return;
    if(direction>0){*status|=in_mask;*in_pressure=255;}
    if(direction<0){*status|=out_mask;*out_pressure=255;}
}
#endif
