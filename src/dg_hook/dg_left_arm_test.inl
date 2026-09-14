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
    for(j=0;j<3;j++) memcpy(blob+DG_OBJS_ARRAY+(8+j)*stride,
                           scratch+DG_OBJS_ARRAY+(4+j)*stride,64);
}

static int t_left_seam(void)
{
    enum { STRIDE=0x180, JOINTS=21 };
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
    memcpy(&g_b,saved,sizeof g_b);free(saved);g_left=saved_left;
    printf("  %s left seam: asymmetric chain, feedback, isolated ownership, loss, support commit\n",bad?"FAIL":"ok");
    return bad != 0;
}
#undef LH_CHECK
