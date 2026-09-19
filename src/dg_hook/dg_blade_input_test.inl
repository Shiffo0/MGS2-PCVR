static int test_blade_input(void) {
    DG_XR_FRAME f={0};DG_BLADE_SAMPLE s;DG_XR_HAND_POSE *p[2];
    int i,bad=0,checks=0,source=g_source,track=g_arm_track,absolute=g_arm_absolute_aim,hand=g_arm_hand;
#define BC(x) do{checks++;if(!(x)){bad++;printf("FAIL blade input %d: %s\n",__LINE__,#x);}}while(0)
    p[0]=&f.right_hand.grip;p[1]=&f.right_hand.aim;f.head_raw.qw=1;
    for(i=0;i<2;i++){p[i]->active=p[i]->tracked=p[i]->position_valid=p[i]->orientation_valid=1;
        p[i]->sample_seq=10;p[i]->xr_time=1000000;p[i]->raw_local.qw=1;}
    g_source=SRC_XR;g_arm_track=ARM_TRACK_R_GRIP;g_arm_absolute_aim=g_arm_hand=1;
    f.right_hand.grip.raw_local.pz=-.3;f.right_hand.squeeze_click=1;
    blade_sample_from_frame(&f,1,100,7,&s);
    BC(s.valid && s.allowed && s.grip && s.context==7 && s.forward[2]==-1 && s.hand[2]==-.3);
    blade_sample_from_frame(&f,0,100,7,&s);BC(s.valid && !s.allowed);
    for(i=0;i<2;i++) {
        p[i]->tracked=0;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);p[i]->tracked=1;
        p[i]->sample_seq++;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);p[i]->sample_seq--;
        p[i]->xr_time++;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);p[i]->xr_time--;
        p[i]->pose_age_ms=101;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);p[i]->pose_age_ms=0;
    }
    f.head_raw.qw=0;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);f.head_raw.qw=1;
    g_source=SRC_SCRIPT;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);g_source=SRC_XR;
    g_arm_track=ARM_TRACK_L_GRIP;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);g_arm_track=ARM_TRACK_R_GRIP;
    g_arm_hand=0;blade_sample_from_frame(&f,1,100,7,&s);BC(!s.valid);g_arm_hand=1;
    blade_sample_from_frame(NULL,1,100,7,&s);BC(!s.valid);
    {
        const char *words[]={"on","ON","off","onjunk","1"};
        for(i=0;i<5;i++) {
            char text[96];POSE pose;double seconds;int src,tr,map[4],h;
            DG_XR_CONFIG xc;DG_BRIDGE_CONFIG cfg;ARM_POS_CFG ap;
            sprintf_s(text,sizeof text,"source=xr\nvr_hf_blade=%s\n",words[i]);
            BC(parse_config(text,&pose,&seconds,&src,&xc,&cfg,&tr,map,&h,&ap)==(i<3));
            if(i<3)BC(cfg.hf_blade==(i<2));
        }
    }
    g_source=source;g_arm_track=track;g_arm_absolute_aim=absolute;g_arm_hand=hand;
    printf("blade input/config: %d checks, %d failures\n",checks,bad);
#undef BC
    return bad?1:0;
}
