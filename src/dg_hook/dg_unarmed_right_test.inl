static int t_unarmed_right(void)
{
    enum { STRIDE=0x180, JOINTS=11 };
    static unsigned char blob[DG_OBJS_ARRAY+JOINTS*STRIDE], actor[0x240], player[0xD40], mc[0x70];
    static float adjust[55*4];
    void *saved=malloc(sizeof g_b);
    ULONGLONG arm=(ULONGLONG)(ULONG_PTR)(actor+0x60);
    DG_BRIDGE_ARM_TARGET t;
    double left_engine[4],left_world[4],expected_engine[4],expected_world[4];
    DG_ADJ_FRAME native_frame;
    TQ_RIG rig={{0,0,0},{200,0,0},{100,173.20508,0}};
    double before[3],after[3];
    int i,j,bad=0;
#define UR_CHECK(x) do {if(!(x)){bad++;printf("FAIL unarmed right %d: %s\n",__LINE__,#x);}}while(0)
    {
        DG_ARM_REFERENCE ref={0};DG_BRIDGE_ARM_TARGET rt={0};
        double root[4],ctrl[4],stick=12,expect[4],turn[4],saved_root[4];
        rt.calibration_id=1;rt.calibration_camera[3]=1;
        th_axis(1,0,0,20,root);th_axis(0,0,1,15,ctrl);
        memcpy(saved_root,root,sizeof root);
        arm_reference(&ref,&rt,root,ctrl,&stick);
        th_axis(0,1,0,80,root);th_axis(0,0,1,70,ctrl);stick=99;
        arm_reference(&ref,&rt,root,ctrl,&stick);
        UR_CHECK(th_angle_between(root,saved_root)<.001 && stick==12);
        th_axis(0,0,1,15,expect);UR_CHECK(th_angle_between(ctrl,expect)<.001);
        th_axis(0,1,0,60,turn);memcpy(rt.calibration_camera,turn,sizeof turn);
        dg_ik_quat_mul(turn,saved_root,expect);
        arm_reference(&ref,&rt,root,ctrl,&stick);
        UR_CHECK(th_angle_between(root,expect)<.001);
        /* A retained rig sees the changed camera every pair. Rebase from
           the original reference, never accumulate the camera delta. */
        for(i=0;i<20;i++) {
            arm_reference(&ref,&rt,root,ctrl,&stick);
            UR_CHECK(th_angle_between(root,expect)<.001 && stick==12);
        }
        rt.calibration_id++;th_axis(0,1,0,10,root);stick=77;
        arm_reference(&ref,&rt,root,ctrl,&stick);
        UR_CHECK(stick==77 && ref.id==2);
    }
    if(!saved) return 1;
    memcpy(saved,&g_b,sizeof g_b);memset(&g_b,0,sizeof g_b);
    memset(blob,0,sizeof blob);memset(actor,0,sizeof actor);memset(player,0,sizeof player);
    memset(mc,0,sizeof mc);memset(adjust,0,sizeof adjust);
    *(ULONGLONG *)(actor+0x60)=(ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(actor+0x68)=(ULONGLONG)(ULONG_PTR)mc;
    *(ULONGLONG *)(actor+0x228)=(ULONGLONG)(ULONG_PTR)(player+0xCF4);
    *(ULONGLONG *)(player+0xBA8)=arm;*(LONG *)(player+0xBB0)=6;
    *(LONG *)(mc+0x14)=55;*(ULONGLONG *)(mc+0x48)=(ULONGLONG)(ULONG_PTR)adjust;
    for(i=0;i<55;i++) adjust[i*4+3]=1;
    for(i=0;i<JOINTS;i++) {
        float *m=(float *)(blob+DG_OBJS_ARRAY+i*STRIDE);
        m[0]=m[5]=m[10]=m[15]=1;
    }
    ((float *)(blob+DG_OBJS_ARRAY+3*STRIDE))[14]=100;
    g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&arm;
    g_b.adjust_frame=1;g_b.skel_stride=STRIDE;g_b.skel_parents_read=JOINTS;
    g_b.skel_region_end=sizeof blob;
    for(i=3;i<7;i++) g_b.skel_parents[i]=i-1;
    g_b.skel_parents[7]=2;g_b.skel_parents[8]=7;g_b.skel_parents[9]=8;g_b.skel_parents[10]=9;
    native_frame=ADJ_FRAME_LEGACY;native_frame.live=1;
    native_frame.valid=arm_frame_yaw(0,native_frame.q);
    th_axis(0,1,0,40,left_engine);
    memcpy(expected_engine,left_engine,sizeof left_engine);
    expected_engine[1]=-expected_engine[1];expected_engine[2]=-expected_engine[2];
    UR_CHECK(adjust_quat_to_world(&native_frame,left_engine,left_world));
    UR_CHECK(adjust_quat_to_world(&native_frame,expected_engine,expected_world));
    th_write_basis((float *)(blob+DG_OBJS_ARRAY+10*STRIDE),left_world);
    memset(&t,0,sizeof t);t.write=1;t.weight=1;t.stream_id=1;t.hand_quat[3]=1;
    t.absolute_aim=1;t.position.enabled=1; /* no pistol selection, as with None */
    t.wrist_view[0]=770;
    for(j=0;j<2;j++) {
        for(i=0;i<10;i++) {
            tq_engine(blob,STRIDE,adjust,0,&rig,NULL);
            g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);
        }
        tq_engine(blob,STRIDE,adjust,0,&rig,j?after:before);
        UR_CHECK(g_b.ik_active && g_b.c_arm_pairs_accepted>0 && g_b.arm_map_unarmed);
        t.wrist_view[1]=-120;
    }
    UR_CHECK(vdist3(before,after)>20); /* controller translation moves real solver output */
    UR_CHECK(!g_b.camera_position.ready); /* no invalid pistol position calibration */
    { /* During native crawl camera/arm-root interpolation, a stationary
         grip must follow camera height without recapturing the neutral. */
        double standing[3],lowered[3];
        t.position.valid=1;t.position.units=1000;
        t.position.camera[0]=t.position.camera[5]=
            t.position.camera[10]=t.position.camera[15]=1;
        t.position.view[3]=1;
        for(i=0;i<8;i++) {
            tq_engine(blob,STRIDE,adjust,0,&rig,NULL);
            g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);
        }
        tq_engine(blob,STRIDE,adjust,0,&rig,standing);
        UR_CHECK(g_b.ik_active && g_b.camera_position.ready);
        t.position.camera[13]=-40;
        tq_engine(blob,STRIDE,adjust,0,&rig,NULL);
        g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);
        tq_engine(blob,STRIDE,adjust,0,&rig,lowered);
        UR_CHECK(g_b.ik_active && fabs((lowered[1]-standing[1])+40)<.05);
        UR_CHECK(fabs(lowered[0]-standing[0])<.05 && fabs(lowered[2]-standing[2])<.05);
        t.position.camera[13]=0;
        tq_engine(blob,STRIDE,adjust,0,&rig,NULL);
        g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);
        tq_engine(blob,STRIDE,adjust,0,&rig,lowered);
        UR_CHECK(vdist3(standing,lowered)<.05);
        t.position.valid=0;
    }
    t.unarmed_hand_write=1;
    for(i=0;i<5;i++) {tq_engine(blob,STRIDE,adjust,0,&rig,NULL);g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);}
    UR_CHECK(g_b.hand_have_rest);
    UR_CHECK(th_angle_between(g_b.hand_rest_view,expected_world)<0.05);
    {
        DG_LEFT_STATE saved_left=g_left;
        double up[4],fore[4],chain[4],moved[4],rest[4],identity[4]={0,0,0,1};
        float saved_slots[8];
        ULONGLONG saved_mask=*(ULONGLONG *)(mc+0x38);
        memcpy(saved_slots,adjust+32,sizeof saved_slots);
        memset(&g_left,0,sizeof g_left);
        g_left.active=1;g_left.arm=arm;g_left.objs=(ULONGLONG)(ULONG_PTR)blob;
        g_left.mctrl=(ULONGLONG)(ULONG_PTR)mc;g_left.adjust=(ULONGLONG)(ULONG_PTR)adjust;
        th_axis(1,0,0,25,up);th_axis(0,0,1,-10,fore);
        for(i=0;i<4;i++) {g_left.cached[i]=adjust[32+i]=(float)up[i];g_left.cached[4+i]=adjust[36+i]=(float)fore[i];}
        *(ULONGLONG *)(mc+0x38)|=DG_LEFT_MASK;
        dg_ik_quat_mul(fore,up,chain);dg_ik_quat_mul(chain,left_engine,moved);
        UR_CHECK(adjust_quat_to_world(&native_frame,moved,left_world));
        th_write_basis((float *)(blob+DG_OBJS_ARRAY+10*STRIDE),left_world);
        UR_CHECK(unarmed_hand_rest(arm,identity,&native_frame,rest));
        UR_CHECK(th_angle_between(rest,expected_world)<0.05);
        { /* Controller-owned left wrist must not change mirrored right rest. */
            double twist[4],tracked[4];
            th_axis(0,0,1,65,twist);
            for(i=0;i<4;i++)g_left.wrist_cached[i]=(float)twist[i];
            g_left.wrist_active=1;
            dg_ik_quat_mul(twist,moved,tracked);
            UR_CHECK(adjust_quat_to_world(&native_frame,tracked,left_world));
            th_write_basis((float *)(blob+DG_OBJS_ARRAY+10*STRIDE),left_world);
            UR_CHECK(unarmed_hand_rest(arm,identity,&native_frame,rest));
            UR_CHECK(th_angle_between(rest,expected_world)<0.05);
            g_left.wrist_active=0;
        }
        adjust[32]=0.123f;
        UR_CHECK(!unarmed_hand_rest(arm,identity,&native_frame,rest));
        memcpy(adjust+32,saved_slots,sizeof saved_slots);*(ULONGLONG *)(mc+0x38)=saved_mask;
        g_left=saved_left;
        UR_CHECK(adjust_quat_to_world(&native_frame,left_engine,left_world));
        th_write_basis((float *)(blob+DG_OBJS_ARRAY+10*STRIDE),left_world);
    }
    {
        double identity[4]={0,0,0,1},want[4],got[4],q[4],wf[4],wh[4];
        float parent[16],wrist[16],right[16];
        float *pm=(float *)(blob+DG_OBJS_ARRAY+9*STRIDE);
        float *wm=(float *)(blob+DG_OBJS_ARRAY+10*STRIDE);
        float *rm=(float *)(blob+DG_OBJS_ARRAY+5*STRIDE);
        memcpy(parent,pm,sizeof parent);memcpy(wrist,wm,sizeof wrist);memcpy(right,rm,sizeof right);
        UR_CHECK(adjust_quat_to_world(&native_frame,identity,wf));
        th_write_basis(pm,wf);th_write_basis(rm,wf);
        UR_CHECK(adjust_quat_to_world(&native_frame,left_engine,wh));th_write_basis(wm,wh);
        UR_CHECK(unarmed_wrist_target(arm,&native_frame,identity,identity,0,want));
        UR_CHECK(th_angle_between(want,expected_world)<.05);
        th_axis(1,0,0,55,q);dg_ik_quat_mul(q,left_engine,wh);
        UR_CHECK(adjust_quat_to_world(&native_frame,wh,wh));th_write_basis(wm,wh);
        UR_CHECK(adjust_quat_to_world(&native_frame,q,wf));th_write_basis(pm,wf);
        UR_CHECK(unarmed_wrist_target(arm,&native_frame,identity,identity,0,got));
        UR_CHECK(th_angle_between(want,got)<.05);
        memcpy(pm,parent,sizeof parent);memcpy(wm,wrist,sizeof wrist);memcpy(rm,right,sizeof right);
    }
    t.hand_quat[2]=sin(0.2);t.hand_quat[3]=cos(0.2);
    for(i=0;i<5;i++) {tq_engine(blob,STRIDE,adjust,0,&rig,NULL);g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);}
    UR_CHECK(g_b.c_arm_hand_written>0);
    {
        double rest[4],first[4],want[4],result[4];DG_FREE_WRIST_STATE pure={0};
        t.free_right.enabled=t.free_right.valid=1;t.free_right.world[3]=1;
        for(i=0;i<5;i++){tq_engine(blob,STRIDE,adjust,0,&rig,NULL);g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);}
        UR_CHECK(g_b.free_right.ready && g_b.hand_have_desired);
        memcpy(rest,g_b.free_right.rest0,sizeof rest);memcpy(first,g_b.hand_desired,sizeof first);
        UR_CHECK(dg_free_wrist_step(&pure,&t.free_right,rest,result));
        th_axis(0,0,1,20,t.free_right.world);
        for(i=0;i<5;i++){tq_engine(blob,STRIDE,adjust,0,&rig,NULL);g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);}
        UR_CHECK(g_b.hand_have_desired && th_angle_between(first,g_b.hand_desired)>5);
        dg_ik_quat_mul(t.free_right.world,rest,want);
        UR_CHECK(dg_free_wrist_step(&pure,&t.free_right,rest,result));
        UR_CHECK(th_angle_between(result,want)<.001);
        t.free_right.valid=0;arm_ik_now(arm,&t);
        UR_CHECK(!g_b.free_right.ready && !g_b.hand_command_valid);
        t.free_right.valid=1;
    }
    *(LONG *)(player+0xB90)=1;t.pair_id++;g_b.c_ticks++;arm_ik_now(arm,&t);
    UR_CHECK(!g_b.arm_map_unarmed && !g_b.ik_active); /* invalid pistol still refused */
    UR_CHECK(!g_b.free_right.ready);
    *(LONG *)(player+0xB90)=0;
    for(i=0;i<10;i++) {tq_engine(blob,STRIDE,adjust,0,&rig,NULL);g_b.c_ticks++;t.pair_id++;arm_ik_now(arm,&t);}
    UR_CHECK(g_b.ik_active);
    t.write=0;arm_ik_now(arm,&t);UR_CHECK(!g_b.ik_active);
    UR_CHECK((*(ULONGLONG *)(mc+0x38)&((1ULL<<4)|(1ULL<<5)))==0);
    memcpy(&g_b,saved,sizeof g_b);free(saved);
    printf("  %s unarmed right: controller movement, equip transition, tracking loss\n",bad?"FAIL":"ok");
#undef UR_CHECK
    return bad!=0;
}
