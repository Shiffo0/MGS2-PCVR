#include "dg_left_proximity_measured.h"
static int t_twohand_latch(void)
{
    DG_TWOHAND s;
    int i,bad=0;
    double b;
#define LH_CHECK(x) do { if(!(x)) { bad++; printf("  FAIL left line %d: %s\n",__LINE__,#x); } } while(0)
    dg_twohand_reset(&s);
    for(i=0;i<3;i++) {
        b=dg_twohand_step(&s,1,1,.1,1+i,1000000000LL+i*16666667LL);
        LH_CHECK(!s.engaged && b==0);
    }
    for(i=0;i<100;i++) dg_twohand_step(&s,1,1,.1,s.seq,s.time);
    LH_CHECK(!s.engaged);
    for(i=3;i<30;i++) dg_twohand_step(&s,1,1,.1,1+i,1000000000LL+i*16666667LL);
    LH_CHECK(s.engaged && s.blend==1);
    dg_twohand_step(&s,1,1,.21,31,1500000010LL);
    LH_CHECK(s.engaged && s.blend==1);
    dg_twohand_step(&s,1,1,.25,32,1516666677LL);
    LH_CHECK(!s.engaged && s.blend<1);
    for(i=0;i<12;i++) dg_twohand_step(&s,1,1,.25,33+i,1533333344LL+i*16666667LL);
    LH_CHECK(s.blend==0);
    for(i=0;i<30;i++) dg_twohand_step(&s,1,1,.1,45+i,1733333348LL+i*16666667LL);
    LH_CHECK(s.blend==1);
    LH_CHECK(dg_twohand_step(&s,0,1,.1,s.seq,s.time)==0 && !s.engaged);
    for(i=0;i<30;i++) dg_twohand_step(&s,1,1,.1,100+i,3000000000LL+i*16666667LL);
    LH_CHECK(s.blend==1);
    LH_CHECK(dg_twohand_step(&s,1,0,.1,s.seq,s.time)==0);
    dg_twohand_step(&s,1,1,.1,150,5000000000LL);
    dg_twohand_step(&s,1,1,.1,151,5500000000LL);
    LH_CHECK(!s.engaged && s.blend==0);
    dg_twohand_reset(&s);
    for(i=0;i<(int)(sizeof left_proximity_measured/sizeof left_proximity_measured[0]);i++) {
        dg_twohand_step(&s,1,1,left_proximity_measured[i].distance,
                       left_proximity_measured[i].seq,left_proximity_measured[i].time);
        if(i==12) LH_CHECK(s.engaged && s.blend==1);
    }
    LH_CHECK(s.engaged && s.blend==1); /* User's close-controller gesture must attach. */
    printf("  %s left proximity: unique samples, dwell, hysteresis, release, loss, gaps\n",bad?"FAIL":"ok");
    return bad != 0;
}

/* Synthetic left hierarchy with deliberately unequal bone lengths and a
   leftward chest branch. Compose via the test engine, then re-index 4/5/6
   into 8/9/10; the shipping left implementation never participates. */
static void tl_engine(unsigned char *blob,int stride,const float *adjust,
                      const TQ_RIG *rig, double wrist[3])
{
    unsigned char scratch[DG_OBJS_ARRAY+7*0x180];
    float slots[28];
    int j;
    memset(scratch,0,sizeof scratch); memset(slots,0,sizeof slots);
    for(j=0;j<7;j++) {
        float *m=(float *)(scratch+DG_OBJS_ARRAY+j*stride);
        m[0]=m[5]=m[10]=m[15]=1;
        slots[j*4+3]=1;
    }
    memcpy(slots+16,adjust+32,8*sizeof(float));
    tq_engine(scratch,stride,slots,0,rig,wrist);
    { /* Joint 10 rotates the wrist basis, not its position. */
        double q[4],world[4],basis[3][3],live[4];int r,k;
        DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;frame.live=1;
        frame.valid=arm_frame_yaw(0,frame.q);
        float *m=(float *)(scratch+DG_OBJS_ARRAY+6*stride);
        for(k=0;k<4;k++)q[k]=adjust[40+k];
        for(r=0;r<3;r++)for(k=0;k<3;k++)basis[r][k]=m[r*4+k];
        if(dg_ik_quat_normalize(q) && adjust_quat_to_world(&frame,q,world) &&
           dg_ik_basis_quat(basis,live)) {
            dg_ik_quat_mul(world,live,q);th_write_basis(m,q);
        }
    }
    for(j=0;j<3;j++) memcpy(blob+DG_OBJS_ARRAY+(8+j)*stride,
                           scratch+DG_OBJS_ARRAY+(4+j)*stride,64);
}

static void tl_m9_fingers(unsigned char *blob,int stride,double contact[3])
{
    static const int tips[5]={41,44,47,51,54};
    float *hand=(float *)(blob+DG_OBJS_ARRAY+10*stride);int i,k;
    for(k=0;k<3;k++)contact[k]=hand[12+k]+80*hand[8+k];
    for(i=0;i<5;i++) {
        float *tip=(float *)(blob+DG_OBJS_ARRAY+tips[i]*stride);
        memcpy(tip,hand,64);
        for(k=0;k<3;k++)tip[12+k]=(float)(contact[k]+(i-2)*10*hand[k]);
    }
}
static int t_left_seam(void)
{
    enum { STRIDE=0x180, JOINTS=55 };
    static unsigned char blob[DG_OBJS_ARRAY+JOINTS*STRIDE];
    static unsigned char actor[0x240],mc[0x70];
    static float adjust[55*4];
    void *saved=malloc(sizeof g_b);
    DG_LEFT_STATE saved_left=g_left;
    DG_BRIDGE_ARM_TARGET t;
    TQ_RIG rig={{0,0,0},{-180,0,0},{-90,-120,0}};
    ULONGLONG arm=(ULONGLONG)(ULONG_PTR)(actor+0x60), right_mask=(1ULL<<4)|(1ULL<<5)|(1ULL<<6)|(1ULL<<10);
    float right[12],hand[4];
    double wrist[3],expected[3],first[3];
    int bad=0,i,j,k;
    if(!saved) return 1;
    memcpy(saved,&g_b,sizeof g_b); memset(&g_b,0,sizeof g_b);
    memset(&g_left,0,sizeof g_left); memset(blob,0,sizeof blob);
    memset(actor,0,sizeof actor); memset(mc,0,sizeof mc); memset(adjust,0,sizeof adjust);
    *(ULONGLONG *)(actor+0x60)=(ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(actor+0x68)=(ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc+0x14)=55; *(ULONGLONG *)(mc+0x48)=(ULONGLONG)(ULONG_PTR)adjust;
    for(i=0;i<55;i++) adjust[i*4+3]=1;
    /* Deliberately nonidentity other-owned slots. They must survive bytewise. */
    adjust[4*4]=.2f; adjust[5*4+1]=.3f; adjust[6*4+2]=.4f; adjust[10*4]=.5f;
    memcpy(right,adjust+16,sizeof right); memcpy(hand,adjust+40,sizeof hand);
    *(ULONGLONG *)(mc+0x38)=right_mask;
    for(i=0;i<JOINTS;i++) {
        float *m=(float *)(blob+DG_OBJS_ARRAY+i*STRIDE);
        m[0]=m[5]=m[10]=m[15]=1;
    }
    ((float *)(blob+DG_OBJS_ARRAY+7*STRIDE))[12]=-100;
    g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&arm;
    g_b.skel_stride=STRIDE; g_b.skel_parents_read=JOINTS;
    g_b.skel_parents[7]=2;g_b.skel_parents[8]=7;
    g_b.skel_parents[9]=8;g_b.skel_parents[10]=9;
    /* Live frame zero is identity, avoiding dependence on the legacy frame. */
    g_b.adjust_frame=0;
    memset(&t,0,sizeof t); t.left_enabled=t.left_valid=1;t.left_weight=1;
    t.stream_id=1; t.player_reach_view=330;
    t.left_wrist_view[0]=-220;t.left_wrist_view[1]=-140;t.left_wrist_view[2]=35;
    /* Legacy adjustment needs the engine's fixed conversion, so establish a
       resolved zero-heading player for this fixture below. */
    {
        static unsigned char player[0xD40];
        memset(player,0,sizeof player);
        *(ULONGLONG *)(actor+0x228)=(ULONGLONG)(ULONG_PTR)(player+0xCF4);
        *(ULONGLONG *)(player+0xBA8)=arm;*(LONG *)(player+0xBB0)=6;*(LONG *)(player+0xB90)=1;
        g_b.adjust_frame=1;
    }
    for(i=0;i<12;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        g_b.c_ticks++;t.pair_id++;
        left_arm_now(arm,&t,0);
    }
    LH_CHECK(g_left.active && g_left.accepted>0);
    /* Visibility needs the current solved left owner, not just a toggle. */
    {
        ULONGLONG pw = *(ULONGLONG *)(actor+0x228) - 0xCF4;
        LH_CHECK(!left_arm_show_unarmed(arm,&t)); /* equipped weapon */
        *(LONG *)(ULONG_PTR)(pw+0xB90)=0;
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
        LH_CHECK(g_left.active && left_arm_show_unarmed(arm,&t));
        t.left_valid=0;LH_CHECK(!left_arm_show_unarmed(arm,&t));t.left_valid=1;
        t.left_enabled=0;LH_CHECK(!left_arm_show_unarmed(arm,&t));t.left_enabled=1;
        t.pair_id++;LH_CHECK(!left_arm_show_unarmed(arm,&t));t.pair_id--;
        t.stream_id++;LH_CHECK(!left_arm_show_unarmed(arm,&t));t.stream_id--;
        LH_CHECK(!left_arm_show_unarmed(arm+16,&t));
        LH_CHECK(!left_arm_show_unarmed(arm,NULL));
        *(LONG *)(ULONG_PTR)(pw+0xBB0)=5;
        LH_CHECK(!left_arm_show_unarmed(arm,&t));
        *(LONG *)(ULONG_PTR)(pw+0xBB0)=6;
        *(LONG *)(ULONG_PTR)(pw+0xB90)=1;
    }
    tl_engine(blob,STRIDE,adjust,&rig,first);
    /* Away from the soft zone, expected reach = 0.99 of controller reach. */
    for(k=0;k<3;k++) expected[k]=t.left_wrist_view[k]*.99;
    LH_CHECK(vdist3(first,expected)<.02);
    for(i=0;i<120;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
    }
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    LH_CHECK(vdist3(first,wrist)<.02);
    LH_CHECK(memcmp(right,adjust+16,sizeof right)==0 && memcmp(hand,adjust+40,sizeof hand)==0);
    LH_CHECK((*(ULONGLONG *)(mc+0x38)&right_mask)==right_mask);
    /* Same pair loss must release before the duplicate path. */
    t.left_valid=0;left_arm_now(arm,&t,0);
    LH_CHECK(!g_left.active && !(*(ULONGLONG *)(mc+0x38)&DG_LEFT_MASK));
    LH_CHECK(memcmp(right,adjust+16,sizeof right)==0 && memcmp(hand,adjust+40,sizeof hand)==0);
    /* Reacquire; an external slot owner wins, and is never zeroed by release. */
    t.left_valid=1;
    for(i=0;i<12;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
    }
    LH_CHECK(g_left.active);
    adjust[32]=.123f;left_arm_release();LH_CHECK(adjust[32]==.123f);
    LH_CHECK(*(ULONGLONG *)(mc+0x38)&(1ULL<<8));
    *(ULONGLONG *)(mc+0x38)=right_mask;
    for(j=8;j<=9;j++) for(k=0;k<4;k++) adjust[j*4+k]=(k==3)?1.0f:0.0f;
    /* Camera/raw-space position must stay independent of the old shoulder
       mapping. The synthetic hierarchy actually applies published slots. */
    t.left_position.enabled=t.left_position.valid=1;
    t.left_position.units=1000;
    t.left_position.camera[0]=t.left_position.camera[5]=
        t.left_position.camera[10]=t.left_position.camera[15]=1;
    t.left_position.view[3]=1;
    for(i=0;i<12;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;
        left_arm_now(arm,&t,0);
    }
    LH_CHECK(g_left.active && g_left.camera_position.ready);
    tl_engine(blob,STRIDE,adjust,&rig,first);
    /* Camera moves 40 mm right with controller stationary in raw space. */
    t.left_position.camera[12]=40;
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    memcpy(expected,first,sizeof expected);expected[0]+=40;
    LH_CHECK(g_left.active && vdist3(wrist,expected)<.02);
    /* Physical head translation cancels camera translation at full scale. */
    t.left_position.view[4]=.04;
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    LH_CHECK(vdist3(wrist,first)<.02);
    /* Left controller travel uses the arm scale (330/330 * .99). */
    t.left_position.grip[0]=.05;
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    memcpy(expected,first,sizeof expected);expected[0]+=49.5;
    LH_CHECK(vdist3(wrist,expected)<.02);
    /* Equipment must preserve an accepted independent left calibration,
       even while the right arm recalibrates to a different native pose. */
    t.position.enabled=1;
    g_b.camera_position=g_left.camera_position;
    g_b.camera_position.virtual0[0]+=.025;
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,1);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    LH_CHECK(g_left.active && vdist3(wrist,expected)<.02);
    /* Right-arm settling must not release the already calibrated left arm. */
    g_b.camera_position.ready=0;
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    LH_CHECK(g_left.active && vdist3(wrist,expected)<.02);
    /* None retains its independent left mapping when the right camera
       calibration is unavailable, including native crawl camera movement. */
    g_b.arm_map_unarmed=1;g_b.camera_position.ready=0;
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,1);
    LH_CHECK(g_left.active);
    {
        DG_POSITION_STATE calibration=g_left.camera_position;
        t.left_position.camera[13]=-40;
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        expected[1]-=40;
        LH_CHECK(g_left.active && vdist3(wrist,expected)<.02);
        LH_CHECK(!memcmp(&calibration,&g_left.camera_position,sizeof calibration));
        t.left_position.camera[13]=0;
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        expected[1]+=40;
        LH_CHECK(g_left.active && vdist3(wrist,expected)<.02);
    }
    g_b.arm_map_unarmed=0;
    t.position.enabled=0;
    t.left_position.valid=0;left_arm_now(arm,&t,0);
    LH_CHECK(!g_left.active && !g_left.camera_position.ready);
    LH_CHECK(memcmp(right,adjust+16,sizeof right)==0 && memcmp(hand,adjust+40,sizeof hand)==0);
    memset(&t.left_position,0,sizeof t.left_position);
    /* Support uses the newly committed right wrist, not the stale skeleton. */
    t.twohand_enabled=t.hands_coherent=t.aim_write=1;t.aim_weapon_id=1;
    t.hands_distance_m=.08;g_b.fire_mode=DG_FIRE_MODE_ON;g_b.fire.state=DG_FIRE_HOLD;
    g_b.have_pred_wrist=g_b.hand_have_desired=1;
    g_b.hand_desired[3]=1;g_b.rec_pair.base_now[3]=1;
    g_b.rec_pair.flags=DG_REC_PAIR_F_BASE;
    g_b.rec_pair.joint_world[3][0]=-230;g_b.rec_pair.joint_world[3][1]=-120;
    g_b.pred_wrist[0]=-160;g_b.pred_wrist[1]=-110;g_b.pred_wrist[2]=30;
    for(i=0;i<45;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;
        t.hands_distance_m=left_proximity_measured[i<24 ? i : 23].distance;
        t.left_sample_seq=100+i;t.left_sample_time=1000000000LL+i*16666667LL;
        t.aim_sample_seq=t.left_sample_seq;t.aim_sample_time=t.left_sample_time;
        left_arm_now(arm,&t,1);
    }
    LH_CHECK(g_left.active && g_left.anchor_valid && g_left.grip.blend==1);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    for(k=0;k<3;k++) expected[k]=g_b.pred_wrist[k]+g_left.anchor[k];
    LH_CHECK(vdist3(wrist,expected)<.02);
    /* A 90-degree rotation transports the native -X support offset to +Z. */
    g_b.hand_desired[1]=sqrt(.5);g_b.hand_desired[3]=sqrt(.5);
    t.pair_id++;g_b.c_ticks++;t.left_sample_seq++;t.left_sample_time+=16666667;
    t.aim_sample_seq=t.left_sample_seq;t.aim_sample_time=t.left_sample_time;
    left_arm_now(arm,&t,1);tl_engine(blob,STRIDE,adjust,&rig,wrist);
    expected[0]=g_b.pred_wrist[0]; expected[1]=g_b.pred_wrist[1];
    expected[2]=g_b.pred_wrist[2]+40;
    LH_CHECK(vdist3(wrist,expected)<.02);
    /* Changing weapon invalidates the old latch, even with identical pointers. */
    t.aim_weapon_id=2;t.pair_id++;g_b.c_ticks++;t.left_sample_seq++;
    t.left_sample_time+=16666667;t.aim_sample_seq=t.left_sample_seq;
    t.aim_sample_time=t.left_sample_time;
    left_arm_now(arm,&t,1);
    LH_CHECK(g_left.grip.blend==0 && g_left.weapon==2);
    tl_engine(blob,STRIDE,adjust,&rig,wrist);
    t.pair_id++;g_b.c_ticks++;left_arm_now(arm,&t,0);
    LH_CHECK(g_left.grip.blend==0); /* No stale right commit reuse. */
    /* Missing player frame must beat the same-pair replay and relinquish
       only our own slots on the still-current arm. */
    { ULONGLONG saved_link=*(ULONGLONG *)(actor+0x228);
      *(ULONGLONG *)(actor+0x228)=0;left_arm_now(arm,&t,0);
      LH_CHECK(!g_left.active && !(*(ULONGLONG *)(mc+0x38)&DG_LEFT_MASK));
      *(ULONGLONG *)(actor+0x228)=saved_link; }
    for(i=0;i<12;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
    }
    LH_CHECK(g_left.active);
    g_b.armed=0;dg_bridge_arm_seam_now(&t);
    LH_CHECK(!g_left.active && !(*(ULONGLONG *)(mc+0x38)&DG_LEFT_MASK));
    for(i=0;i<12;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
    }
    LH_CHECK(g_left.active);
    g_b.armed=1;g_b.fps.state=DG_FPS_ACTIVE;g_b.s_late_unsafe=1;
    dg_bridge_arm_seam_now(&t);
    LH_CHECK(!g_left.active && !(*(ULONGLONG *)(mc+0x38)&DG_LEFT_MASK));
    LH_CHECK(memcmp(right,adjust+16,sizeof right)==0 && memcmp(hand,adjust+40,sizeof hand)==0);
    /* Free left wrist: drive the real seam through three-axis sweeps and
       reconstruct the resulting hierarchy, independently of its solver. */
    g_b.s_late_unsafe=0;
    *(ULONGLONG *)(mc+0x38)&=~(1ULL<<10);
    memset(adjust+40,0,16);adjust[43]=1;
    t.free_left.enabled=t.free_left.valid=1;t.free_left.world[3]=1;
    t.twohand_enabled=0;
    g_b.arm_map_unarmed=1;
    for(i=0;i<12;i++) {
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
    }
    LH_CHECK(g_left.active && g_left.wrist_active && g_left.wrist.ready);
    { /* Independent neutral reference: render the solved arm with no wrist
         adjustment. The initial free wrist must retain that local relation,
         not the old animation's world direction. */
        float neutral[55*4];double got[4],want[4],rows[3][3],neutral_pos[3];
        int r,c;
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
        for(r=0;r<3;r++)for(c=0;c<3;c++)rows[r][c]=
            ((float *)(blob+DG_OBJS_ARRAY+10*STRIDE))[r*4+c];
        LH_CHECK(dg_ik_basis_quat(rows,got));
        memcpy(neutral,adjust,sizeof neutral);
        memset(neutral+40,0,16);neutral[43]=1;
        tl_engine(blob,STRIDE,neutral,&rig,neutral_pos);
        for(r=0;r<3;r++)for(c=0;c<3;c++)rows[r][c]=
            ((float *)(blob+DG_OBJS_ARRAY+10*STRIDE))[r*4+c];
        LH_CHECK(dg_ik_basis_quat(rows,want));
        LH_CHECK(th_angle_between(got,want)<.05);
        LH_CHECK(vdist3(wrist,neutral_pos)<.001);
        tl_engine(blob,STRIDE,adjust,&rig,wrist);
    }
    {
        double rest[4],got[4],want[4],rows[3][3],worst=0;
        int axis,step,r,c;
        memcpy(rest,g_left.wrist.rest0,sizeof rest);
        for(axis=0;axis<3;axis++)for(step=-6;step<=6;step++) {
            th_axis(axis==0,axis==1,axis==2,step*10,t.free_left.world);
            tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;
            left_arm_now(arm,&t,0);tl_engine(blob,STRIDE,adjust,&rig,wrist);
            for(r=0;r<3;r++)for(c=0;c<3;c++)rows[r][c]=
                ((float *)(blob+DG_OBJS_ARRAY+10*STRIDE))[r*4+c];
            LH_CHECK(dg_ik_basis_quat(rows,got));
            dg_ik_quat_mul(t.free_left.world,rest,want);
            { double error=th_angle_between(got,want);if(error>worst)worst=error;
              LH_CHECK(error<.05 && g_left.wrist_active); }
        }
        printf("  left free wrist: 39 three-axis hierarchy poses, worst %.6f deg\n",worst);
    }
    { /* Equip/None transitions retain the calibrated free wrist.
         Check rendered orientation, not just saved state. */
        int mode,r,c;
        DG_FREE_WRIST_STATE neutral=g_left.wrist;
        double got[4],want[4],rows[3][3];
        t.left_stream_id=t.stream_id;
        for(mode=0;mode<=1;mode++) {
            t.stream_id++; /* actual weapon selection advances main pose stream */
            g_b.arm_map_unarmed=mode;
            th_axis(0,1,0,mode?35:-25,t.free_left.world);
            tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;
            left_arm_now(arm,&t,0);
            LH_CHECK(g_left.wrist_active);
            LH_CHECK(!memcmp(&g_left.wrist,&neutral,sizeof neutral));
            tl_engine(blob,STRIDE,adjust,&rig,wrist);
            for(r=0;r<3;r++)for(c=0;c<3;c++)rows[r][c]=
                ((float *)(blob+DG_OBJS_ARRAY+10*STRIDE))[r*4+c];
            LH_CHECK(dg_ik_basis_quat(rows,got));
            LH_CHECK(dg_free_wrist_step(&neutral,&t.free_left,neutral.rest0,want));
            LH_CHECK(th_angle_between(got,want)<.05);
        }
    }
    { /* Explicit physical recenter must still discard the accepted neutral. */
        t.left_stream_id=t.stream_id+1;
        tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;
        left_arm_now(arm,&t,0);
        LH_CHECK(!g_left.active && !g_left.wrist.ready);
        for(i=0;i<12;i++) {
            tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;
            left_arm_now(arm,&t,0);
        }
        LH_CHECK(g_left.active && g_left.wrist.ready);
        LH_CHECK(th_angle_between(g_left.wrist.controller0,t.free_left.world)<.001);
    }
    { /* Support mode relinquishes only wrist ownership, retaining position. */
        double identity[4]={0,0,0,1};DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;
        frame.live=1;frame.valid=arm_frame_yaw(0,frame.q);
        left_wrist_now((ULONGLONG)(ULONG_PTR)blob+DG_OBJS_ARRAY,STRIDE,
            (ULONGLONG)(ULONG_PTR)mc,(ULONGLONG)(ULONG_PTR)adjust,
            &frame,&t,identity,identity,1,1,NULL);
        LH_CHECK(!g_left.wrist_active && g_left.active);
        for(i=0;i<3;i++) {tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);}
        LH_CHECK(g_left.wrist_active);
    }
    { /* Actual left seam -> independent engine hierarchy -> distal fingers.
         The native input/grab tolerance must not persist as a visual gap. */
        static const int parents[17]={10,38,39,40,38,42,43,38,45,46,38,48,49,50,48,52,53};
        void *saved_m9=malloc(sizeof g_m9);LONG enabled=g_m9_enabled;
        DG_FREE_WRIST_STATE neutral=g_left.wrist;
        double contact[3],got[4],want[4],rows[3][3];int r,c;
        LH_CHECK(saved_m9!=NULL);
        if(saved_m9) {
            memcpy(saved_m9,&g_m9,sizeof g_m9);memset(&g_m9,0,sizeof g_m9);
            for(i=0;i<17;i++)g_b.skel_parents[38+i]=parents[i];
            t.aim_weapon_id=1;g_b.arm_map_unarmed=0;
            g_b.have_pred_wrist=g_b.hand_have_desired=1;
            memset(g_b.hand_desired,0,sizeof g_b.hand_desired);g_b.hand_desired[3]=1;
            tl_engine(blob,STRIDE,adjust,&rig,wrist);tl_m9_fingers(blob,STRIDE,contact);
            for(k=0;k<3;k++)g_b.pred_wrist[k]=contact[k];g_b.pred_wrist[0]+=30;
            g_m9_enabled=g_m9.active=g_m9.render=1;g_m9.state.source=t.stream_id;g_m9.out.attached=1;
            g_m9.state.now_ms=GetTickCount64();g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,1);
            LH_CHECK(g_left.slide_contact.active && g_left.wrist_world_valid);
            g_left.slide_contact.start_ms=GetTickCount64()-100;
            for(i=0;i<4;i++) {
                tl_engine(blob,STRIDE,adjust,&rig,wrist);tl_m9_fingers(blob,STRIDE,contact);
                g_m9.state.now_ms=GetTickCount64();g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,1);
                tl_engine(blob,STRIDE,adjust,&rig,wrist);tl_m9_fingers(blob,STRIDE,contact);
                LH_CHECK(g_left.wrist_world_valid && vdist3(contact,g_b.pred_wrist)<.05);
            }
            th_axis(0,1,0,30,g_b.hand_desired);
            g_m9.state.now_ms=GetTickCount64();g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,1);
            tl_engine(blob,STRIDE,adjust,&rig,wrist);tl_m9_fingers(blob,STRIDE,contact);
            for(r=0;r<3;r++)for(c=0;c<3;c++)rows[r][c]=((float *)(blob+DG_OBJS_ARRAY+10*STRIDE))[r*4+c];
            LH_CHECK(dg_ik_basis_quat(rows,got));
            dg_ik_quat_mul(g_b.hand_desired,g_left.slide_contact.wrist_relative,want);
            LH_CHECK(th_angle_between(got,want)<.05 && vdist3(contact,g_b.pred_wrist)<.05);
            LH_CHECK(!memcmp(&neutral,&g_left.wrist,sizeof neutral));
            g_m9.out.attached=0;g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,1);
            LH_CHECK(g_left.slide_contact.releasing);
            g_left.slide_contact.release_ms=GetTickCount64()-100;
            tl_engine(blob,STRIDE,adjust,&rig,wrist);tl_m9_fingers(blob,STRIDE,contact);
            g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,1);
            LH_CHECK(!g_left.slide_contact.active && !g_left.slide_contact.releasing);
            LH_CHECK(!memcmp(&neutral,&g_left.wrist,sizeof neutral));
            memcpy(&g_m9,saved_m9,sizeof g_m9);free(saved_m9);g_m9_enabled=enabled;
            g_b.have_pred_wrist=g_b.hand_have_desired=0;
        }
    }
    t.free_left.valid=0;left_arm_now(arm,&t,0); /* same pair revokes */
    LH_CHECK(!g_left.wrist_active && !g_left.wrist.ready && !(*(ULONGLONG *)(mc+0x38)&(1ULL<<10)));
    t.free_left.valid=1;
    for(i=0;i<3;i++) {tl_engine(blob,STRIDE,adjust,&rig,wrist);g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);}
    LH_CHECK(g_left.wrist_active);
    { /* Shipping left solver + independently reconstructed native hierarchy.
         Releases for view/scene/tracking changes must reuse numeric alignment. */
        DG_HAND_PROFILE profile;double want[4],got[4],rows[3][3];int epoch,r,c;
        dg_hand_profile_default(&profile);
        t.persistent_hands=1;t.free_left.persistent=1;
        memcpy(t.free_left.alignment,profile.rotation[0],sizeof t.free_left.alignment);
        t.left_position.enabled=t.left_position.valid=1;t.left_position.units=1000;
        memset(t.left_position.camera,0,sizeof t.left_position.camera);
        memset(t.left_position.view,0,sizeof t.left_position.view);t.left_position.view[3]=1;
        for(i=0;i<4;i++)t.left_position.camera[i*5]=1;
        t.left_position.grip[0]=-.12;t.left_position.grip[1]=-.12;t.left_position.grip[2]=-.12;
        t.aim_weapon_id=0;t.twohand_enabled=0;
        for(epoch=0;epoch<4;epoch++) {
            left_arm_release();t.left_stream_id++;g_b.arm_map_unarmed=epoch%2;
            th_axis(0,1,0,epoch*20,t.free_left.world);
            dg_ik_quat_mul(t.free_left.world,t.free_left.alignment,want);
            for(i=0;i<12;i++) {
                tl_engine(blob,STRIDE,adjust,&rig,wrist);
                g_b.c_ticks++;t.pair_id++;left_arm_now(arm,&t,0);
            }
            LH_CHECK(g_left.active && g_left.wrist_active && !g_left.camera_position.ready);
            tl_engine(blob,STRIDE,adjust,&rig,wrist);
            for(r=0;r<3;r++)for(c=0;c<3;c++)rows[r][c]=
                ((float *)(blob+DG_OBJS_ARRAY+10*STRIDE))[r*4+c];
            LH_CHECK(dg_ik_basis_quat(rows,got));
            LH_CHECK(th_angle_between(got,want)<.05);
        }
    }
    adjust[40]=.123f;left_arm_release();LH_CHECK(adjust[40]==.123f);
    LH_CHECK(*(ULONGLONG *)(mc+0x38)&(1ULL<<10)); /* foreign owner wins */
    LH_CHECK(memcmp(right,adjust+16,sizeof right)==0);
    memcpy(&g_b,saved,sizeof g_b);free(saved);g_left=saved_left;
    printf("  %s left seam: asymmetric chain, feedback, isolated ownership, loss, support commit\n",bad?"FAIL":"ok");
    return bad != 0;
}
#undef LH_CHECK
