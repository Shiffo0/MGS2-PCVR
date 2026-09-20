/* Visual attachment only. Controller travel/chamber state stay in dg_m9_slide.
 * All lengths here are native units; no guessed controller-to-palm offset. */
#ifndef DG_M9_CONTACT_H
#define DG_M9_CONTACT_H
typedef struct {
    int active,releasing;
    uint64_t start_ms,release_ms;
    double offset[3],wrist_relative[4];
    double last_wrist[3],last_q[4];
} DG_M9_CONTACT;
static int dg_m9_contact_step(DG_M9_CONTACT *s,uint64_t now,
    const double weapon_q[4],const double weapon_pos[3],const double slide[3],
    const double free_wrist[3],const double free_q[4],const double finger_local[3],
    double wrist[3],double wrist_q[4])
{
    double inv[4],delta[3],v[3],contact[3],a;int k;
    if(!s->active || now<s->start_ms) {
        dg_ik_quat_conj(weapon_q,inv);
        dg_position_rotate(free_q,finger_local,v);
        for(k=0;k<3;k++)delta[k]=free_wrist[k]+v[k]-weapon_pos[k];
        dg_position_rotate(inv,delta,contact);
        for(k=0;k<3;k++)s->offset[k]=contact[k]-slide[k];
        dg_ik_quat_mul(inv,free_q,s->wrist_relative);
        s->start_ms=now;s->active=1;s->releasing=0;
    }
    a=(double)(now-s->start_ms)/100.0;if(a>1)a=1;
    a=a*a*(3-2*a); /* 100 ms acquisition, zero endpoint velocity */
    for(k=0;k<3;k++)contact[k]=slide[k]+(1-a)*s->offset[k];
    dg_position_rotate(weapon_q,contact,delta);
    dg_ik_quat_mul(weapon_q,s->wrist_relative,wrist_q);
    if(!dg_ik_quat_normalize(wrist_q))return 0;
    dg_position_rotate(wrist_q,finger_local,v);
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
    a=(double)(now-s->release_ms)/100.0;a=a*a*(3-2*a);
    for(k=0;k<3;k++)wrist[k]=s->last_wrist[k]+a*(free_wrist[k]-s->last_wrist[k]);
    dg_pose_quat_blend(s->last_q,free_q,a,wrist_q);
    return 1;
}
#endif
