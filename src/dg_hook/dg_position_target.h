/* Pure camera-consistent position anchor. Units: raw XR metres, game scale.
   Camera rows are camera axes in world. B=(1,-1,-1), as dg_aim_target.
   Calibration preserves the already accepted grip point. Scale applies to
   controller displacement, NEVER to eye separation or head displacement. */
#ifndef DG_POSITION_TARGET_H
#define DG_POSITION_TARGET_H
#include <math.h>
#include <string.h>
#include "dg_ik.h"
typedef struct {
    int enabled, valid;
    double camera[16], view[7], grip[3], units;
} DG_POSITION_INPUT;
typedef struct {
    int ready;
    double grip0[3], virtual0[3], scale, units;
} DG_POSITION_STATE;
static void dg_position_rotate(const double q[4],const double v[3],double o[3]) {
    double t[3]={2*(q[1]*v[2]-q[2]*v[1]),2*(q[2]*v[0]-q[0]*v[2]),2*(q[0]*v[1]-q[1]*v[0])};
    o[0]=v[0]+q[3]*t[0]+q[1]*t[2]-q[2]*t[1];
    o[1]=v[1]+q[3]*t[1]+q[2]*t[0]-q[0]*t[2];
    o[2]=v[2]+q[3]*t[2]+q[0]*t[1]-q[1]*t[0];
}
static int dg_position_frame(const DG_POSITION_INPUT *in,double q[4]) {
    double m[3][3],cq[4],inv[4],b[4]={1,0,0,0},n=0;
    int i,j,k;
    if(!in || !in->valid || !isfinite(in->units) || in->units<=0) return 0;
    for(i=0;i<16;i++) if(!isfinite(in->camera[i])) return 0;
    for(i=0;i<7;i++) if(!isfinite(in->view[i])) return 0;
    for(i=0;i<3;i++) if(!isfinite(in->grip[i])) return 0;
    for(i=0;i<4;i++) n+=in->view[i]*in->view[i];
    if(fabs(n-1)>1e-6 || fabs(in->camera[15]-1)>1e-4) return 0;
    for(i=0;i<3;i++) {
        if(fabs(in->camera[i*4+3])>1e-4) return 0;
        for(j=0;j<3;j++) {
            double d=0;
            m[i][j]=in->camera[i*4+j];
            for(k=0;k<3;k++) d+=in->camera[i*4+k]*in->camera[j*4+k];
            if(fabs(d-(i==j?1:0))>1e-4) return 0;
        }
    }
    if(!dg_ik_basis_quat(m,cq)) return 0;
    dg_ik_quat_conj(in->view,inv);
    dg_ik_quat_mul(cq,b,q);dg_ik_quat_mul(q,inv,q);
    return dg_ik_quat_normalize(q);
}
static int dg_position_calibrate(DG_POSITION_STATE *s,const DG_POSITION_INPUT *in,
                                  const double accepted[3],double scale) {
    DG_POSITION_STATE next;
    double q[4],inv[4],v[3],raw[3];int i;
    if(!s || !accepted || !isfinite(scale) || scale<=0 || !dg_position_frame(in,q)) return 0;
    for(i=0;i<3;i++) {if(!isfinite(accepted[i]))return 0;v[i]=(accepted[i]-in->camera[12+i])/in->units;}
    dg_ik_quat_conj(q,inv);dg_position_rotate(inv,v,raw);
    memset(&next,0,sizeof next);next.ready=1;next.scale=scale;next.units=in->units;
    for(i=0;i<3;i++) {
        next.grip0[i]=in->grip[i];next.virtual0[i]=in->view[4+i]+raw[i];
        if(!isfinite(next.virtual0[i])) return 0;
    }
    *s=next;return 1;
}
static int dg_position_target(const DG_POSITION_STATE *s,const DG_POSITION_INPUT *in,double out[3]) {
    double q[4],v[3],world[3];int i;
    if(!s || !s->ready || !out || !dg_position_frame(in,q) || in->units!=s->units) return 0;
    for(i=0;i<3;i++) v[i]=(s->virtual0[i]+s->scale*(in->grip[i]-s->grip0[i])-in->view[4+i])*in->units;
    dg_position_rotate(q,v,world);
    for(i=0;i<3;i++){world[i]+=in->camera[12+i];if(!isfinite(world[i]))return 0;}
    memcpy(out,world,sizeof world);return 1;
}
#endif
