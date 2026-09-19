/* The rest must not depend on the body heading at the moment it is captured.
   One fixed physical left hand (local rotations given heading-free), captured
   under the live frame at several headings: the stored rest, read back
   heading-free, is always the plain y,z mirror of those local rotations. */
static int t_hand_pose_mirror_heading(void)
{
    static unsigned char blob[DG_OBJS_ARRAY+55*0x180],actor[0x240],player[0xD40],mc[0x70];
    static float adjust[55*4];
    static const short headings[6]={0,512,1024,1539,-700,2047};
    void *saved=malloc(sizeof g_b);
    DG_LEFT_STATE saved_left=g_left;
    ULONGLONG arm=(ULONGLONG)(ULONG_PTR)(actor+0x60);
    DG_BRIDGE_ARM_TARGET t;
    int h,j,bad=0;double worst=0;
#define HH_CHECK(x) do{if(!(x)){bad++;printf("FAIL hand pose heading %d: %s\n",__LINE__,#x);}}while(0)
    if(!saved)return 1;
    memcpy(saved,&g_b,sizeof g_b);
    for(h=0;h<6;h++) {
        DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;
        double left[18][4],local_free[17][4],wrist_free[4],fore[4],q[4],world[4],identity[4]={0,0,0,1};
        memset(&g_b,0,sizeof g_b);memset(&g_hand_pose,0,sizeof g_hand_pose);memset(&g_hand_pose_rest,0,sizeof g_hand_pose_rest);
        memset(&g_left,0,sizeof g_left);memset(blob,0,sizeof blob);memset(actor,0,sizeof actor);
        memset(player,0,sizeof player);memset(mc,0,sizeof mc);memset(adjust,0,sizeof adjust);
        *(ULONGLONG *)(actor+0x60)=(ULONGLONG)(ULONG_PTR)blob;
        *(ULONGLONG *)(actor+0x68)=(ULONGLONG)(ULONG_PTR)mc;
        *(ULONGLONG *)(actor+0x228)=(ULONGLONG)(ULONG_PTR)(player+0xCF4);
        *(ULONGLONG *)(player+0xBA8)=arm;*(LONG *)(player+0xBB0)=6;
        *(LONG *)(mc+0x14)=55;*(ULONGLONG *)(mc+0x48)=(ULONGLONG)(ULONG_PTR)adjust;
        for(j=0;j<55;j++){float *m=(float *)(blob+DG_OBJS_ARRAY+j*0x180);m[0]=m[5]=m[10]=m[15]=1;adjust[j*4+3]=1;}
        for(j=0;j<17;j++){int p=hand_pose_parents[j];g_b.skel_parents[21+j]=p;g_b.skel_parents[38+j]=p==6?10:p+17;}
        g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&arm;
        g_b.skel_stride=0x180;g_b.skel_parents_read=55;g_b.ik_active=1;g_left.active=1;
        g_b.adjust_frame=1;*(short *)(player+0x82)=headings[h];
        frame.live=1;frame.valid=arm_frame_yaw(headings[h],frame.q);HH_CHECK(frame.valid);
        memset(&t,0,sizeof t);t.left_enabled=t.left_valid=t.write=1;t.stream_id=1;t.pair_id=1;
        /* the same physical hand at every heading: forearm, wrist and finger locals are heading-free */
        th_axis(0.3,0.9,0.2,25,fore);th_axis(0.5,0.2,0.8,40,wrist_free);
        HH_CHECK(world_quat_to_adjust(&frame,fore,fore));
        HH_CHECK(world_quat_to_adjust(&frame,wrist_free,q));dg_ik_quat_mul(fore,q,left[0]);
        for(j=0;j<17;j++) {
            int p=hand_pose_parents[j]==6?0:hand_pose_parents[j]-20;
            th_axis(1,0.3,-0.2,-18-j*3,local_free[j]);
            HH_CHECK(world_quat_to_adjust(&frame,local_free[j],q));
            dg_ik_quat_mul(left[p],q,left[j+1]);
        }
        HH_CHECK(adjust_quat_to_world(&frame,fore,world));th_write_basis((float *)(blob+DG_OBJS_ARRAY+9*0x180),world);
        for(j=0;j<18;j++) {
            HH_CHECK(adjust_quat_to_world(&frame,left[j],world));
            th_write_basis((float *)(blob+DG_OBJS_ARRAY+(j?37+j:10)*0x180),world);
            th_write_basis((float *)(blob+DG_OBJS_ARRAY+(j?20+j:6)*0x180),identity);
        }
        g_b.c_ticks=1;hand_pose_now(arm,&t);
        HH_CHECK(g_hand_pose.active && g_hand_pose_rest.valid);
        for(j=0;j<17;j++) {
            double want[4],got[4],a;
            memcpy(want,local_free[j],sizeof want);want[1]=-want[1];want[2]=-want[2];
            HH_CHECK(adjust_quat_to_world(&g_hand_pose_rest.frame,g_hand_pose_rest.local[j],got));
            a=th_angle_between(got,want);if(a>worst)worst=a;
            HH_CHECK(a<0.05);
        }
        {
            double want[4],got[4],a;
            memcpy(want,wrist_free,sizeof want);want[1]=-want[1];want[2]=-want[2];
            HH_CHECK(adjust_quat_to_world(&g_hand_pose_rest.frame,g_hand_pose_rest.wrist,got));
            a=th_angle_between(got,want);if(a>worst)worst=a;
            HH_CHECK(a<0.05);
        }
        hand_pose_release();
    }
    memset(&g_hand_pose_rest,0,sizeof g_hand_pose_rest);memset(&g_hand_pose,0,sizeof g_hand_pose);
    memcpy(&g_b,saved,sizeof g_b);free(saved);g_left=saved_left;
    printf("  %s hand pose mirror: rest captured at 6 body headings is the same heading-free mirror (worst %.4f deg)\n",bad?"FAIL":"ok",worst);
#undef HH_CHECK
    return bad!=0;
}
static int t_hand_pose_mirror(void)
{
    static unsigned char blob[DG_OBJS_ARRAY+55*0x180],actor[0x240],player[0xD40],mc[0x70];
    static float adjust[55*4];
    void *saved=malloc(sizeof g_b);
    DG_LEFT_STATE saved_left=g_left;
    ULONGLONG arm=(ULONGLONG)(ULONG_PTR)(actor+0x60);
    DG_BRIDGE_ARM_TARGET t;
    DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;
    double left[18][4],right[18][4],native[17][4],expected[18][4];
    double q[4],a[4],world[4];
    double fixed_local_world[17][4],fixed_wrist_world[4];
    int j,k,iteration,bad=0;
#define HP_CHECK(x) do{if(!(x)){bad++;printf("FAIL hand pose %d: %s\n",__LINE__,#x);}}while(0)
    if(!saved)return 1;
    memcpy(saved,&g_b,sizeof g_b);memset(&g_b,0,sizeof g_b);memset(&g_hand_pose,0,sizeof g_hand_pose);
    {
        int n;
        memset(&g_hand_observe,0,sizeof g_hand_observe);
        HP_CHECK(hand_observe_step(100,1,1,0));
        HP_CHECK(!hand_observe_step(100,1,1,0));
        for(n=2;n<=24;n++)HP_CHECK(hand_observe_step(100,1,n,0));
        HP_CHECK(!hand_observe_step(100,1,25,0));
        HP_CHECK(hand_observe_step(100,1,26,0.8f));
        HP_CHECK(hand_observe_step(100,1,27,0));
        HP_CHECK(hand_observe_step(200,1,28,0));
        HP_CHECK(hand_observe_step(200,2,1,0));
        HP_CHECK(hand_observe_step(200,2,2,-1)); /* turn left */
        HP_CHECK(!hand_observe_step(200,2,3,2));
        for(n=2;n<1000;n++)hand_observe_step(200,2,n,(n%2)?0.8f:0);
        HP_CHECK(g_hand_observe.total==600);
        HP_CHECK(!hand_observe_step(300,3,1001,0.8f));
        memset(&g_hand_observe,0,sizeof g_hand_observe);
    }
    memset(&g_hand_pose_rest,0,sizeof g_hand_pose_rest);
    memset(&g_left,0,sizeof g_left);memset(blob,0,sizeof blob);memset(actor,0,sizeof actor);
    memset(player,0,sizeof player);memset(mc,0,sizeof mc);memset(adjust,0,sizeof adjust);
    *(ULONGLONG *)(actor+0x60)=(ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(actor+0x68)=(ULONGLONG)(ULONG_PTR)mc;
    *(ULONGLONG *)(actor+0x228)=(ULONGLONG)(ULONG_PTR)(player+0xCF4);
    *(ULONGLONG *)(player+0xBA8)=arm;*(LONG *)(player+0xBB0)=6;
    *(LONG *)(mc+0x14)=55;*(ULONGLONG *)(mc+0x48)=(ULONGLONG)(ULONG_PTR)adjust;
    for(j=0;j<55;j++) {
        float *m=(float *)(blob+DG_OBJS_ARRAY+j*0x180);
        m[0]=m[5]=m[10]=m[15]=1;adjust[j*4+3]=1;
    }
    for(j=0;j<17;j++) {
        int p=hand_pose_parents[j];g_b.skel_parents[21+j]=p;
        g_b.skel_parents[38+j]=p==6?10:p+17;
        th_axis(0.2,1.0,0.3,12+j,native[j]);
    }
    g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&arm;
    g_b.skel_stride=0x180;g_b.skel_parents_read=55;g_b.ik_active=1;g_left.active=1;
    memset(&t,0,sizeof t);t.left_enabled=t.left_valid=t.write=1;t.stream_id=1;
    /* Every sample uses different wrist orientations and asymmetric native
       fingers. Rebuild the engine from local rotations plus owned adjustments,
       then run the real writer. This detects cumulative parent feedback. */
    for(iteration=0;iteration<80;iteration++) {
        if(iteration==30 || iteration==60) {
            *(LONG *)(player+0xB90)=1;hand_pose_now(arm,&t);
            HP_CHECK(!g_hand_pose.active && g_hand_pose_rest.valid);
            *(LONG *)(player+0xB90)=0;
            t.stream_id++; /* equip may rebuild the tracking stream */
        }
        if(iteration>=40) {
            static const short headings[8]={0,512,1024,2047,-2048,-1024,-512,0};
            g_b.adjust_frame=1;*(short *)(player+0x82)=headings[(iteration-40)%8];
            frame.live=1;frame.valid=arm_frame_yaw(*(short *)(player+0x82),frame.q);
            HP_CHECK(frame.valid);
        }
        th_axis(0,1,0,iteration*2.0,right[0]);
        th_axis(1,0,0,-iteration*1.3,left[0]);
        /* The camera seam reads the previous hierarchy; SetPos will consume
           the NEW wrist command together with these finger adjustments. */
        th_axis(0.4,0.8,0.3,35+iteration*1.7,expected[0]);
        HP_CHECK(adjust_quat_to_world(&frame,expected[0],g_b.hand_desired));
        g_b.hand_have_desired=1;
        for(j=0;j<17;j++) {
            int p=hand_pose_parents[j]==6?0:hand_pose_parents[j]-20;
            th_axis(1,0.3,-0.2,-18-j+(iteration?iteration*0.8:0),q);
            dg_ik_quat_mul(left[p],q,left[j+1]);
            /* Later native animation/trigger poses never redefine rest. */
            th_axis(1,0.3,-0.2,-18-j,q);
            q[1]=-q[1];q[2]=-q[2];
            /* Preserve the physical local pose while the actor heading
               changes the adjustment coordinate frame, as stick turn does. */
            if(!iteration)HP_CHECK(adjust_quat_to_world(&frame,q,fixed_local_world[j]));
            HP_CHECK(world_quat_to_adjust(&frame,fixed_local_world[j],q));
            dg_ik_quat_mul(expected[p],q,expected[j+1]);
            dg_ik_quat_mul(right[p],native[j],q);
            for(k=0;k<4;k++)a[k]=adjust[(21+j)*4+k];
            dg_ik_quat_mul(a,q,right[j+1]);
        }
        for(j=0;j<18;j++) {
            HP_CHECK(adjust_quat_to_world(&frame,right[j],world));
            th_write_basis((float *)(blob+DG_OBJS_ARRAY+(j?20+j:6)*0x180),world);
            HP_CHECK(adjust_quat_to_world(&frame,left[j],world));
            th_write_basis((float *)(blob+DG_OBJS_ARRAY+(j?37+j:10)*0x180),world);
        }
        g_b.c_ticks++;t.pair_id++;hand_pose_now(arm,&t);
        HP_CHECK(g_hand_pose.active);
        if(!iteration)HP_CHECK(adjust_quat_to_world(&frame,g_hand_pose_rest.wrist,fixed_wrist_world));
        {
            double identity[4]={0,0,0,1},wrist_target[4];
            g_b.arm_map_unarmed=1;g_b.skel_parents[6]=5;g_b.skel_parents[10]=9;
            HP_CHECK(unarmed_wrist_target(arm,&frame,identity,identity,0,wrist_target));
            /* Native forearm 5 stays identity in world in this fixture.
               Engine heading changes, but the cached local rest does not. */
            HP_CHECK(th_angle_between(wrist_target,fixed_wrist_world)<.05);
            if(iteration==0) {
                double twist[4],twisted_target[4];
                float *fore=(float *)(blob+DG_OBJS_ARRAY+5*0x180);
                /* Diagnostic, not a fix acceptance test: keep the commanded
                   IK rotations/rest constant and rotate only native forearm. */
                th_axis(1,0,0,30,twist);th_write_basis(fore,twist);
                HP_CHECK(unarmed_wrist_target(arm,&frame,identity,identity,0,twisted_target));
                printf("  hand diagnostic: native forearm 30 deg, unchanged IK/rest -> wrist target %.2f deg\n",
                    th_angle_between(wrist_target,twisted_target));
                th_write_basis(fore,identity);
            }
        }
        memcpy(right[0],expected[0],sizeof q);
        for(j=0;j<17;j++) {
            int p=hand_pose_parents[j]==6?0:hand_pose_parents[j]-20;
            dg_ik_quat_mul(right[p],native[j],q);
            for(k=0;k<4;k++)a[k]=adjust[(21+j)*4+k];
            dg_ik_quat_mul(a,q,right[j+1]);
            /* Slots are floats: compare orientations, not accumulated length
               roundoff from multiplying a whole branch of test quaternions. */
            HP_CHECK(dg_ik_quat_normalize(right[j+1]));
            HP_CHECK(th_angle_between(right[j+1],expected[j+1])<0.05);
        }
    }
    HP_CHECK(*(ULONGLONG *)(mc+0x38)==DG_HAND_POSE_MASK);
    /* Left hand, arms and wrist slots remain native/other-owned. */
    for(j=0;j<55;j++)if(j<21||j>=38)for(k=0;k<4;k++)HP_CHECK(adjust[j*4+k]==(k==3?1:0));
    *(LONG *)(player+0xB90)=1;hand_pose_now(arm,&t);
    HP_CHECK(!g_hand_pose.active && *(ULONGLONG *)(mc+0x38)==0);
    *(LONG *)(player+0xB90)=0;g_b.c_ticks++;hand_pose_now(arm,&t);HP_CHECK(g_hand_pose.active);
    t.left_valid=0;hand_pose_now(arm,&t);HP_CHECK(!g_hand_pose.active);
    t.left_valid=1;g_b.c_ticks++;hand_pose_now(arm,&t);HP_CHECK(g_hand_pose.active);
    adjust[21*4]=0.123f;hand_pose_release();
    HP_CHECK(adjust[21*4]==0.123f && (*(ULONGLONG *)(mc+0x38)&(1ULL<<21)));
    *(ULONGLONG *)(mc+0x38)=0;adjust[21*4]=0;
    g_b.skel_parents[54]=0;g_b.c_ticks++;hand_pose_now(arm,&t);HP_CHECK(!g_hand_pose.active);
    memset(&g_hand_pose_rest,0,sizeof g_hand_pose_rest);
    memcpy(&g_b,saved,sizeof g_b);free(saved);g_left=saved_left;
    printf("  %s hand pose rest: 80 changing native poses, wrist stability, two equip cycles, ownership/loss\n",bad?"FAIL":"ok");
#undef HP_CHECK
    return bad!=0;
}
