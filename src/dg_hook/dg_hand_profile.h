/* Durable grip-local alignment: no native pointers or world headings. */
#ifndef DG_HAND_PROFILE_H
#define DG_HAND_PROFILE_H
#include "dg_free_wrist.h"
typedef struct { double rotation[2][4]; } DG_HAND_PROFILE;
typedef struct {
    unsigned long seen,request,stream;
    unsigned int last_ms,requested_ms;
    int initialized,eligible;
} DG_HAND_REQUEST;
static unsigned long dg_hand_request_step(DG_HAND_REQUEST *s,unsigned long presses,
    int eligible,unsigned long stream,unsigned int now) {
    if(!s->initialized) {s->seen=presses;s->initialized=1;}
    if(presses!=s->seen) {
        s->seen=presses;s->request=0;
        /* Do not turn an old menu/cutscene button into a later calibration.
           Both sides of the button edge must be live, safe unarmed samples. */
        if(eligible && s->eligible && stream==s->stream &&
           (unsigned int)(now-s->last_ms)<=100u) {
            s->request=presses;s->requested_ms=now;
        }
    }
    if(!eligible || stream!=s->stream || (unsigned int)(now-s->requested_ms)>2000u)
        s->request=0;
    s->stream=stream;s->eligible=eligible;s->last_ms=now;
    return s->request;
}
static void dg_hand_profile_default(DG_HAND_PROFILE *p) {
    int h; memset(p,0,sizeof *p);
    /* Native -Y forward, +Z up, matching the absolute hand-axis contract.
       Free hands use the controller GRIP orientation, not weapon AIM. */
    for(h=0;h<2;h++)p->rotation[h][1]=p->rotation[h][2]=0.70710678118654752440;
}
static int dg_hand_profile_valid(const DG_HAND_PROFILE *p) {
    int h,k; if(!p)return 0;
    for(h=0;h<2;h++) {
        double n=0;
        for(k=0;k<4;k++) {if(!isfinite(p->rotation[h][k]))return 0;n+=p->rotation[h][k]*p->rotation[h][k];}
        if(fabs(n-1)>1e-6)return 0;
    }
    return 1;
}
static int dg_hand_profile_capture(const DG_FREE_WRIST_INPUT *in,
                                   const double neutral[4],double offset[4]) {
    double q[4],r[4],inv[4];
    if(!in || !in->enabled || !in->valid)return 0;
    memcpy(q,in->world,sizeof q);memcpy(r,neutral,sizeof r);
    if(!dg_ik_quat_normalize(q)||!dg_ik_quat_normalize(r))return 0;
    dg_ik_quat_conj(q,inv);dg_ik_quat_mul(inv,r,offset);
    return dg_ik_quat_normalize(offset);
}
/* Absolute raw grip relative to the current rendered view. No animation
   anchor or saved XR origin. The skeleton solver still enforces reach. */
static int dg_hand_profile_position(const DG_POSITION_INPUT *in,double out[3]) {
    double frame[4],v[3],w[3];int k;
    if(!in || !in->enabled || !dg_position_frame(in,frame))return 0;
    for(k=0;k<3;k++)v[k]=(in->grip[k]-in->view[4+k])*in->units;
    dg_position_rotate(frame,v,w);
    for(k=0;k<3;k++) {out[k]=in->camera[12+k]+w[k];if(!isfinite(out[k]))return 0;}
    return 1;
}
#endif
