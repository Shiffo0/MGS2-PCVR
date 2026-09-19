static int test_m9_input(void) {
    DG_XR_FRAME f;DG_M9_SAMPLE s;DG_XR_HAND_POSE *p[3];
    int i,bad=0,checks=0,old_source=g_source,old_track=g_arm_track,old_left=g_left_arm,old_absolute=g_arm_absolute_aim;
    DG_XR_CONFIG old_cfg=g_xrcfg;
#define MC(x) do{checks++;if(!(x)){bad++;printf("FAIL M9 input %d: %s\n",__LINE__,#x);}}while(0)
    memset(&f,0,sizeof f);p[0]=&f.left_hand.grip;p[1]=&f.right_hand.grip;p[2]=&f.right_hand.aim;
    for(i=0;i<3;i++) {p[i]->active=p[i]->tracked=p[i]->position_valid=p[i]->orientation_valid=1;
        p[i]->sample_seq=10;p[i]->raw_local.qw=1;}
    g_arm_absolute_aim=1;g_source=SRC_XR;g_left_arm=1;g_arm_track=ARM_TRACK_R_GRIP;g_xrcfg.scale=1000;
    f.left_hand.grip.raw_local.pz=.05;f.left_hand.squeeze_click=1;
    m9_sample_from_frame(&f,1,&s);MC(s.valid && s.allowed && s.grip && fabs(s.local[1]-.05)<1e-9);
    m9_sample_from_frame(&f,0,&s);MC(s.valid && !s.allowed);
    for(i=0;i<3;i++) {
        p[i]->tracked=0;m9_sample_from_frame(&f,1,&s);MC(!s.valid);p[i]->tracked=1;
        p[i]->sample_seq=11;m9_sample_from_frame(&f,1,&s);MC(!s.valid);p[i]->sample_seq=10;
        p[i]->pose_age_ms=101;m9_sample_from_frame(&f,1,&s);MC(!s.valid);p[i]->pose_age_ms=0;
    }
    g_arm_track=ARM_TRACK_L_GRIP;m9_sample_from_frame(&f,1,&s);MC(!s.valid);
    g_arm_track=ARM_TRACK_R_GRIP;g_left_arm=0;m9_sample_from_frame(&f,1,&s);MC(!s.valid);
    m9_sample_from_frame(NULL,1,&s);MC(!s.valid);
    {const char *words[]={"on","ON","off","onjunk","","1"};
     for(i=0;i<6;i++){char text[96];POSE pose;double seconds;int source,track,map[4],hand;
        DG_XR_CONFIG xc;DG_BRIDGE_CONFIG bc;ARM_POS_CFG ap;
        sprintf_s(text,sizeof text,"source=xr\nvr_m9_slide=%s\n",words[i]);
        MC(parse_config(text,&pose,&seconds,&source,&xc,&bc,&track,map,&hand,&ap));
        MC(bc.m9_slide==(i<2));
     }}
    g_source=old_source;g_arm_track=old_track;g_left_arm=old_left;g_xrcfg=old_cfg;g_arm_absolute_aim=old_absolute;
    printf("M9 input/config: %d checks, %d failures\n",checks,bad);
#undef MC
    return bad?1:0;
}
