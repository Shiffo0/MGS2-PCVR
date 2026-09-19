static void blade_forward(const DG_XR_RAW_POSE *p,double v[3]) {
    v[0]=-2*(p->qx*p->qz+p->qw*p->qy);
    v[1]=-2*(p->qy*p->qz-p->qw*p->qx);
    v[2]=-(1-2*(p->qx*p->qx+p->qy*p->qy));
}
static void blade_sample_from_frame(const DG_XR_FRAME *f,int allowed,
    uint64_t now,uint64_t context,DG_BLADE_SAMPLE *s) {
    const DG_XR_HAND_POSE *p,*a; double n; int k;
    memset(s,0,sizeof *s);
    if(!f || g_source!=SRC_XR ||
       (g_arm_track!=ARM_TRACK_R_GRIP && g_arm_track!=ARM_TRACK_R_AIM) ||
       (!InterlockedCompareExchange(&g_arm_absolute_aim,0,0) ||
        !InterlockedCompareExchange(&g_arm_hand,0,0)))return;
    p=&f->right_hand.grip;a=&f->right_hand.aim;
    s->source=g_arm_pose_stream_id;s->sequence=p->sample_seq;s->context=context;
    s->time_ns=p->xr_time;s->now_ms=now;s->allowed=allowed;
    s->grip=f->right_hand.squeeze_click!=0;
    if(!p->active || !p->tracked || !p->position_valid || !p->orientation_valid ||
       !a->active || !a->tracked || !a->orientation_valid ||
       p->pose_age_ms>100 || a->pose_age_ms>100 || !p->sample_seq ||
       p->sample_seq!=a->sample_seq || p->xr_time!=a->xr_time)return;
    s->hand[0]=p->raw_local.px;s->hand[1]=p->raw_local.py;s->hand[2]=p->raw_local.pz;
    s->head[0]=f->head_raw.px;s->head[1]=f->head_raw.py;s->head[2]=f->head_raw.pz;
    n=f->head_raw.qx*f->head_raw.qx+f->head_raw.qy*f->head_raw.qy+
      f->head_raw.qz*f->head_raw.qz+f->head_raw.qw*f->head_raw.qw;
    if(!_finite(n) || fabs(n-1)>.001)return;
    n=a->raw_local.qx*a->raw_local.qx+a->raw_local.qy*a->raw_local.qy+
      a->raw_local.qz*a->raw_local.qz+a->raw_local.qw*a->raw_local.qw;
    if(!_finite(n) || fabs(n-1)>.001)return;
    blade_forward(&a->raw_local,s->forward);blade_forward(&f->head_raw,s->head_forward);
    s->valid=1;
    for(k=0;k<3;k++)if(!_finite(s->hand[k]) || !_finite(s->head[k]))s->valid=0;
}
