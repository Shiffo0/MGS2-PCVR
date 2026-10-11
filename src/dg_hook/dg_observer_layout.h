#ifndef DG_OBSERVER_LAYOUT_H
#define DG_OBSERVER_LAYOUT_H
#include <math.h>


typedef struct { float eye[2][4], crop[4], options[4], arm[2][4], arm_mode[4]; } DG_OBSERVER_LAYOUT;
static int observer_bounds(const DG_PROJ_FOV *f, float b[4]) {
    double a[4]={f->left,f->right,f->down,f->up}; int i;
    for(i=0;i<4;i++) {
        if(!isfinite(a[i]) || fabs(a[i])>=1.55)return 0;
        b[i]=(float)tan(a[i]);
    }
    return b[1]-b[0]>.001f && b[3]-b[2]>.001f;
}
static int observer_layout(const DG_PROJ_FOV *left,const DG_PROJ_FOV *right,
                           int mode,DG_OBSERVER_LAYOUT *o) {
    float l,r,b,t,w,h,cx,cy; int e=mode==0?0:1;
    memset(o,0,sizeof *o);
    if(mode<0 || mode>2)return 0;
    if(!observer_bounds(e?right:left,o->eye[e]))return 0;
    memcpy(o->eye[!e],o->eye[e],sizeof o->eye[e]);
    l=o->eye[e][0];r=o->eye[e][1];b=o->eye[e][2];t=o->eye[e][3];
    if(mode==2) {
        if(!observer_bounds(left,o->eye[0]))return 0;
        /* Shared vertical band and horizontally overlapping views: no holes. */
        if(o->eye[0][1]<=l || o->eye[0][0]>=r)return 0;
        l=fminf(l,o->eye[0][0]);r=fmaxf(r,o->eye[0][1]);
        b=fmaxf(b,o->eye[0][2]);t=fminf(t,o->eye[0][3]);
    }
    if(t<=b)return 0;
    w=r-l;h=t-b;
    if(w/h>16.0f/9.0f)w=h*(16.0f/9.0f);else h=w*(9.0f/16.0f);
    cx=(l+r)*.5f;cy=(b+t)*.5f;
    o->crop[0]=cx-w*.5f;o->crop[1]=cx+w*.5f;
    o->crop[2]=cy-h*.5f;o->crop[3]=cy+h*.5f;
    o->options[0]=(float)mode;
    /* Feather only a small boundary strip; never average the whole overlap. */
    o->options[1]=fminf(.02f,(fminf(o->eye[0][1],o->eye[1][1])-
                              fmaxf(o->eye[0][0],o->eye[1][0]))*.1f);
    return 1;
}
static int observer_pose_pair(const DG_XR_RAW_POSE *a,const DG_XR_RAW_POSE *b) {
    double dot=a->qx*b->qx+a->qy*b->qy+a->qz*b->qz+a->qw*b->qw;
    double aa=a->qx*a->qx+a->qy*a->qy+a->qz*a->qz+a->qw*a->qw;
    double bb=b->qx*b->qx+b->qy*b->qy+b->qz*b->qz+b->qw*b->qw;
    double dx=a->px-b->px,dy=a->py-b->py,dz=a->pz-b->pz;
    /* No depth or rotational reprojection. Refuse tilted/canted or temporally
       divergent eyes instead of silently stitching incompatible projections. */
    return isfinite(dot)&&aa>.9&&aa<1.1&&bb>.9&&bb<1.1 &&
           dot*dot/(aa*bb)>.99998 && dx*dx+dy*dy+dz*dz<.0225;
}
#endif
