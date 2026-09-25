static void stinger_sample_from_frame(const DG_XR_FRAME *f,int allowed,
    uint64_t now,uint64_t context,DG_STINGER_INPUT *s) {
    const DG_XR_HAND_POSE *p,*a;
    memset(s,0,sizeof *s);
    if(!f || g_source!=SRC_XR ||
       (g_arm_track!=ARM_TRACK_R_GRIP && g_arm_track!=ARM_TRACK_R_AIM) ||
       !g_arm_absolute_aim || !g_arm_hand)return;
    p=&f->right_hand.grip;a=&f->right_hand.aim;
    s->source=g_arm_pose_stream_id;s->context=context;s->sequence=p->sample_seq;
    s->now_ms=now;s->age_ms=p->pose_age_ms>a->pose_age_ms?p->pose_age_ms:a->pose_age_ms;
    s->down=f->right_hand.squeeze_click!=0;s->allowed=allowed;
    s->valid=p->active && p->tracked && p->orientation_valid && p->position_valid &&
        a->active && a->tracked && a->orientation_valid && p->sample_seq &&
        p->sample_seq==a->sample_seq && p->xr_time==a->xr_time && s->age_ms<=100;
}
