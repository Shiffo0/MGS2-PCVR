/* Same coherent XR record as the ordinary controls, with raw LOCAL positions.
 * Right AIM supplies barrel direction; right GRIP supplies the wrist origin. */
static void m9_sample_from_frame(const DG_XR_FRAME *f,int allowed,DG_M9_SAMPLE *s) {
    const DG_XR_HAND_POSE *l,*r,*a;double lp[3],rp[3],q[4];
    memset(s,0,sizeof *s);
    if(!f || g_source!=SRC_XR || !g_left_arm ||
       !InterlockedCompareExchange(&g_arm_absolute_aim,0,0) ||
       !_finite(g_xrcfg.scale) || g_xrcfg.scale<334 || g_xrcfg.scale>10000 ||
       (g_arm_track!=ARM_TRACK_R_GRIP && g_arm_track!=ARM_TRACK_R_AIM))return;
    l=&f->left_hand.grip;r=&f->right_hand.grip;a=&f->right_hand.aim;
    s->source=g_arm_pose_stream_id;s->sequence=l->sample_seq;
    s->units_per_metre=g_xrcfg.scale;s->grip=f->left_hand.squeeze_click!=0;
    s->trigger=f->right_hand.trigger_click!=0;s->allowed=allowed;
    if(!l->active||!r->active||!a->active||!l->tracked||!r->tracked||!a->tracked||
       !l->position_valid||!r->position_valid||!a->orientation_valid||
       l->pose_age_ms>100||r->pose_age_ms>100||a->pose_age_ms>100||
       !s->sequence||s->sequence!=r->sample_seq||s->sequence!=a->sample_seq)return;
    lp[0]=l->raw_local.px;lp[1]=l->raw_local.py;lp[2]=l->raw_local.pz;
    rp[0]=r->raw_local.px;rp[1]=r->raw_local.py;rp[2]=r->raw_local.pz;
    q[0]=a->raw_local.qx;q[1]=a->raw_local.qy;q[2]=a->raw_local.qz;q[3]=a->raw_local.qw;
    memcpy(s->origin,rp,sizeof rp);memcpy(s->aim_q,q,sizeof q);
    s->valid=dg_m9_relative(lp,rp,q,s->local);
}
