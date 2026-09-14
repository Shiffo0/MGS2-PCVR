#ifndef DG_ZOOM_PROJECTION_H
#define DG_ZOOM_PROJECTION_H
#include <math.h>
#include <stdint.h>
#include <string.h>
typedef struct {
    uint64_t identity;
    double reference_x,reference_y,gain;
    int last_eye,have_eye,stable;
    double candidate_x,candidate_y;
} DG_ZOOM_PROJECTION;
/* Reference is an observed native projection at the fully zoomed-out stop.
 * No assumed angle-to-FOV conversion. AFR latches on the first eye and keeps
 * that scale for the second eye and repeated visits to the same camera pass. */
static double dg_zoom_projection_step(DG_ZOOM_PROJECTION *s,uint64_t id,int valid,
    float angle,double sx,double sy,int stereo,int eye) {
    int boundary=!stereo || !s->have_eye || (eye==0 && s->last_eye!=0);
    if(!valid || !id || !(sx>0 && sx<10000 && sy>0 && sy<10000)) {
        s->gain=1;s->have_eye=0;s->stable=0;
        s->candidate_x=s->candidate_y=0;return 1;
    }
    if(id!=s->identity) {
        memset(s,0,sizeof *s);s->identity=id;s->gain=1;boundary=1;
    }
    /* Retail capture: angle2 -> native1.5, angle24 -> native18.
     * Sample only independent eye-pair boundaries, after interpolation settles. */
    if(boundary && !s->reference_x) {
        if(angle>=2.0f && angle<=2.01f) {
            if(s->candidate_x && fabs(sx/s->candidate_x-1)<0.0001 &&
               fabs(sy/s->candidate_y-1)<0.0001) ++s->stable;
            else {s->candidate_x=sx;s->candidate_y=sy;s->stable=1;}
            if(s->stable>=3) {s->reference_x=sx;s->reference_y=sy;}
        } else {s->stable=0;s->candidate_x=s->candidate_y=0;}
    }
    if(boundary && s->reference_x) {
        double x=sx/s->reference_x,y=sy/s->reference_y;
        s->gain=(x>=0.99 && y>=0.99 && fabs(x-y)<=0.02*x)?fmin(4.0,fmax(1.0,(x+y)*0.5)):1.0;
    }
    s->last_eye=eye;s->have_eye=1;return s->gain;
}
static void dg_zoom_projection_apply(MAT *m,double gain) {
    if(gain>1 && gain<=4) {
        m->m[0][0]=(float)(m->m[0][0]*gain);
        m->m[1][1]=(float)(m->m[1][1]*gain);
    }
}
#endif
