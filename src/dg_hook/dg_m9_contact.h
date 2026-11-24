/* Visual attachment only. Controller travel/chamber state stay in dg_m9_slide.
 * All lengths here are native units; no guessed controller-to-palm offset. */
#ifndef DG_M9_CONTACT_H
#define DG_M9_CONTACT_H
/* Same curve as every other hand transition (dg_grip_blend.h). The timing stays its own:
   100 ms measured from the moment of acquisition / release, not a stepped phase, because
   the contact point is re-anchored at acquisition and the release starts from the last
   drawn pose. */
#include "dg_grip_blend.h"

#define DG_M9_GRASP_REAR_INSET 40.0 /* palm centre forward of the slide's rear end */
#define DG_M9_PALM_LIFT 15.0        /* palm bone line above the slide's top surface */

static int dg_m9_grasp_point(const float t[3],const float lo[3],const float hi[3],double out[3])
{
    double c[3],e[3],up[2],n,top,inset;int k;
    for(k=0;k<3;k++)out[k]=t[k];
    for(k=0;k<3;k++) {
        if(!(fabs(lo[k])<=1000) || !(fabs(hi[k])<=1000) || !(hi[k]-lo[k]>=1))return 0; /* NaN fails too */
        c[k]=t[k]+0.5*(lo[k]+hi[k]);e[k]=0.5*(hi[k]-lo[k]);
    }
    n=hypot(c[0],c[2]);
    if(!(n>1))return 0;
    up[0]=c[0]/n;up[1]=c[2]/n;
    top=fabs(e[0]*up[0])+fabs(e[2]*up[1])+DG_M9_PALM_LIFT;
    inset=DG_M9_GRASP_REAR_INSET<e[1] ? DG_M9_GRASP_REAR_INSET : e[1];
    out[0]=c[0]+up[0]*top;
    out[1]=t[1]+hi[1]-inset;
    out[2]=c[2]+up[1]*top;
    return 1;
}
typedef struct {
    int active,releasing,palm; /* palm: the palm-down grip is in use (else the pre-fix finger contact) */
    uint64_t start_ms,release_ms;
    double offset[3],wrist_relative[4],slide_grip[4];
    double last_wrist[3],last_q[4];
} DG_M9_CONTACT;
/* Palm down on the slide: rows {fingers, palm normal, fingers x normal} of the hand
   go to {across the slide, -up, ...}. The finger direction keeps the side the
   player approached from, the palm side is the one that faced the slide at
   acquisition (the hand model's palm/back cannot be told apart from its joint
   positions, the player's own hand can). 0 = no usable geometry. */
static int dg_m9_palm_grip(const double relative[4],const double finger_local[3],
    const double normal_local[3],const double slide[3],double grip[4])
{
    double up[3],n_l[3],f_l[3],b_l[3],f_w[3],n_w[3],across[3],rows[3][3],ql[4],qt[4],len,d;int k;
    len=hypot(slide[0],slide[2]);
    if(!(len>1))return 0;
    up[0]=slide[0]/len;up[1]=0;up[2]=slide[2]/len;
    len=sqrt(normal_local[0]*normal_local[0]+normal_local[1]*normal_local[1]+normal_local[2]*normal_local[2]);
    if(!(len>1e-6))return 0;
    for(k=0;k<3;k++)n_l[k]=normal_local[k]/len;
    d=finger_local[0]*n_l[0]+finger_local[1]*n_l[1]+finger_local[2]*n_l[2];
    for(k=0;k<3;k++)f_l[k]=finger_local[k]-d*n_l[k];
    len=sqrt(f_l[0]*f_l[0]+f_l[1]*f_l[1]+f_l[2]*f_l[2]);
    if(!(len>1e-6))return 0;
    for(k=0;k<3;k++)f_l[k]/=len;
    dg_position_rotate(relative,n_l,n_w);
    if(n_w[0]*up[0]+n_w[2]*up[2]>0)for(k=0;k<3;k++)n_l[k]=-n_l[k];
    b_l[0]=f_l[1]*n_l[2]-f_l[2]*n_l[1];b_l[1]=f_l[2]*n_l[0]-f_l[0]*n_l[2];b_l[2]=f_l[0]*n_l[1]-f_l[1]*n_l[0];
    across[0]=up[2];across[1]=0;across[2]=-up[0]; /* +Y x up */
    dg_position_rotate(relative,f_l,f_w);
    if(f_w[0]*across[0]+f_w[2]*across[2]<0)for(k=0;k<3;k++)across[k]=-across[k];
    for(k=0;k<3;k++){rows[0][k]=f_l[k];rows[1][k]=n_l[k];rows[2][k]=b_l[k];}
    if(!dg_ik_basis_quat(rows,ql))return 0;
    for(k=0;k<3;k++){rows[0][k]=across[k];rows[1][k]=-up[k];}
    rows[2][0]=across[1]*-up[2]-across[2]*-up[1];
    rows[2][1]=across[2]*-up[0]-across[0]*-up[2];
    rows[2][2]=across[0]*-up[1]-across[1]*-up[0];
    if(!dg_ik_basis_quat(rows,qt))return 0;
    dg_ik_quat_conj(ql,ql);dg_ik_quat_mul(qt,ql,grip);
    return dg_ik_quat_normalize(grip);
}
/* hand_local: the hand point put on the slide (palm centre; the distal finger
   centroid when normal_local is NULL, the pre-fix behaviour). finger_local: the
   finger direction. */
static int dg_m9_contact_step(DG_M9_CONTACT *s,uint64_t now,
    const double weapon_q[4],const double weapon_pos[3],const double slide[3],
    const double free_wrist[3],const double free_q[4],const double finger_local[3],
    const double hand_local[3],const double normal_local[3],
    double wrist[3],double wrist_q[4])
{
    double inv[4],delta[3],v[3],contact[3],a;int k;
    if(!s->active || now<s->start_ms) {
        dg_ik_quat_conj(weapon_q,inv);
        dg_ik_quat_mul(inv,free_q,s->wrist_relative);
        s->palm=hand_local && normal_local &&
            dg_m9_palm_grip(s->wrist_relative,finger_local,normal_local,slide,s->slide_grip);
        dg_position_rotate(free_q,s->palm?hand_local:finger_local,v);
        for(k=0;k<3;k++)delta[k]=free_wrist[k]+v[k]-weapon_pos[k];
        dg_position_rotate(inv,delta,contact);
        for(k=0;k<3;k++)s->offset[k]=contact[k]-slide[k];
        if(!s->palm) {
        /* Bring the fingers across the slide. Preserve the side of approach and
           roll about the finger direction. The slide travels on local Y. */
        dg_position_rotate(s->wrist_relative,finger_local,v);
        {
            double radial=hypot(v[0],v[2]),length=hypot(radial,v[1]);
            double x=radial>1e-6 ? v[0]/radial : 1;
            double z=radial>1e-6 ? v[2]/radial : 0;
            double turn[4]={v[1]*z,0,-v[1]*x,length+radial};
            if(!dg_ik_quat_normalize(turn))return 0;
            dg_ik_quat_mul(turn,s->wrist_relative,s->slide_grip);
        }
        }
        s->start_ms=now;s->active=1;s->releasing=0;
    }
    a=(double)(now-s->start_ms)/100.0;if(a>1)a=1;
    a=dg_grip_ease(a); /* 100 ms acquisition, zero endpoint velocity */
    for(k=0;k<3;k++)contact[k]=slide[k]+(1-a)*s->offset[k];
    dg_position_rotate(weapon_q,contact,delta);
    { double relative[4];
      dg_pose_quat_blend(s->wrist_relative,s->slide_grip,a,relative);
      dg_ik_quat_mul(weapon_q,relative,wrist_q); }
    if(!dg_ik_quat_normalize(wrist_q))return 0;
    dg_position_rotate(wrist_q,s->palm && hand_local?hand_local:finger_local,v);
    for(k=0;k<3;k++)wrist[k]=weapon_pos[k]+delta[k]-v[k];
    memcpy(s->last_wrist,wrist,sizeof s->last_wrist);
    memcpy(s->last_q,wrist_q,sizeof s->last_q);
    return 1;
}
static int dg_m9_contact_release(DG_M9_CONTACT *s,uint64_t now,
    const double free_wrist[3],const double free_q[4],double wrist[3],double wrist_q[4])
{
    double a;int k;
    if(s->active){s->active=0;s->releasing=1;s->release_ms=now;}
    if(!s->releasing)return 0;
    if(now<s->release_ms || now-s->release_ms>=100){s->releasing=0;return 0;}
    a=dg_grip_ease((double)(now-s->release_ms)/100.0);
    for(k=0;k<3;k++)wrist[k]=s->last_wrist[k]+a*(free_wrist[k]-s->last_wrist[k]);
    dg_pose_quat_blend(s->last_q,free_q,a,wrist_q);
    return 1;
}
#endif
