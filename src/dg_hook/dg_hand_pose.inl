/* None-only mirrored native finger pose. The 17-node branches are validated
   against the observed 55-joint model. No mesh or animation data is copied. */
#define DG_HAND_POSE_FIRST 21
#define DG_HAND_POSE_COUNT 17
#define DG_HAND_POSE_MASK (((1ULL<<17)-1)<<21)
static const int hand_pose_parents[17]={6,21,22,23,21,25,26,21,28,29,21,31,32,33,31,35,36};
static struct {
    ULONGLONG arm,objs,mc,adj;
    unsigned long stream;
    LONG tick;
    int active;
    float before[68],cached[68];
} g_hand_pose;
/* Numeric rest survives equip/release. These identities are compared only;
   all current pointers/topology are validated before using the rotations. */
static struct {
    ULONGLONG arm,objs,mc,adj;
    int valid;
    double local[17][4];
    double wrist[4];
    DG_ADJ_FRAME frame; /* coordinate basis at capture, not the current actor yaw */
} g_hand_pose_rest;

static void hand_pose_inv(const double q[4],double out[4])
{ out[0]=-q[0];out[1]=-q[1];out[2]=-q[2];out[3]=q[3]; }

/* Bounded read-only burst during stick turning and after stopping.
   Each row labels measured hierarchy and the NEW command separately. */
static struct {ULONGLONG arm; unsigned long stream; unsigned long long pair;
    int have,down,left,total;} g_hand_observe;
static int hand_observe_step(ULONGLONG arm,unsigned long stream,unsigned long long pair,float stick)
{
    int down=stick>0.15f?1:stick< -0.15f?-1:0;
    if(!(stick>=-1 && stick<=1) || !arm || !pair || g_hand_observe.total>=600)return 0;
    if(g_hand_observe.have && g_hand_observe.arm==arm &&
       g_hand_observe.stream==stream && g_hand_observe.pair==pair)return 0;
    if(!g_hand_observe.have || g_hand_observe.arm!=arm || g_hand_observe.stream!=stream ||
       down || g_hand_observe.down!=down)g_hand_observe.left=24;
    g_hand_observe.have=1;g_hand_observe.arm=arm;g_hand_observe.stream=stream;
    g_hand_observe.pair=pair;g_hand_observe.down=down;
    if(!g_hand_observe.left)return 0;
    g_hand_observe.left--;g_hand_observe.total++;return 1;
}

static void hand_pose_solve(const double right[18][4],const double pose[17][4],
                            const double previous[17][4],const double wrist[4],double out[17][4])
{
    double desired[18][4];
    int j,k;
    memcpy(desired[0],wrist,sizeof desired[0]);
    for(j=0;j<17;j++) {
        int p=hand_pose_parents[j]==6 ? 0:hand_pose_parents[j]-20;
        double inv[4],local[4],native[4],pre[4],want[4];
        hand_pose_inv(previous[j],inv);
        dg_ik_quat_mul(inv,right[j+1],pre);
        hand_pose_inv(right[p],inv);
        dg_ik_quat_mul(inv,pre,native);
        memcpy(local,pose[j],sizeof local);
        dg_ik_quat_mul(desired[p],local,want);
        dg_ik_quat_mul(desired[p],native,pre);
        hand_pose_inv(pre,inv);
        dg_ik_quat_mul(want,inv,out[j]);
        for(k=0;k<4;k++) desired[j+1][k]=want[k];
    }
}

static int hand_slots_owned(ULONGLONG arm,ULONGLONG objs,ULONGLONG mc,ULONGLONG adj)
{
    return g_b.a.gm_player_arm_body &&
        *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body==arm &&
        plausible_ptr(arm) && region_end(arm)>=arm+16 &&
        *(volatile ULONGLONG *)(ULONG_PTR)arm==objs &&
        *(volatile ULONGLONG *)(ULONG_PTR)(arm+8)==mc &&
        plausible_ptr(mc) && region_end(mc)>=mc+0x50 &&
        RD32(mc+0x14)>=55 &&
        *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x48)==adj &&
        plausible_ptr(adj) && region_end(adj)>=adj+55*16;
}
static int hand_pose_owner(void) {
    return hand_slots_owned(g_hand_pose.arm,g_hand_pose.objs,g_hand_pose.mc,g_hand_pose.adj);
}

static const unsigned char blade_flex_joints[]={22,23,25,26,28,29,32,33,35,36};
static struct {
    ULONGLONG arm,objs,mc,adj,mask;
    float before[20][4],cached[20][4];
} blade_fingers;

static void blade_fingers_release(void)
{
    if(blade_fingers.mask && hand_slots_owned(blade_fingers.arm,blade_fingers.objs,
                                              blade_fingers.mc,blade_fingers.adj)) {
        for(int i=0;i<20;i++) {
            int j=blade_flex_joints[i%10]+(i>=10?17:0);
            float *slot=(float *)(uintptr_t)(blade_fingers.adj+j*16);
            uint64_t bit=1ULL<<j;
            if((blade_fingers.mask & bit) && !memcmp(slot,blade_fingers.cached[i],16)) {
                memcpy(slot,blade_fingers.before[i],16);
                *(uint64_t *)(uintptr_t)(blade_fingers.mc+0x38)&=~bit;
            }
        }
    }
    memset(&blade_fingers,0,sizeof blade_fingers);
}

/* Curl the two proximal joints of each digit around the across-palm axis.
   Knuckles are proximal to their own correction, so holding does not integrate
   the angle. Wrist position and controller tracking remain independently driven. */
static int grip_fingers_apply(ULONGLONG arm,ULONGLONG player,double weight,unsigned sides)
{
    ULONGLONG objs=*(ULONGLONG *)(uintptr_t)arm,mc=*(ULONGLONG *)(uintptr_t)(arm+8),adj;
    DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;
    int stride=g_b.skel_stride;
    float slots[20][4];uint64_t mask=0;
    if(!(weight>0 && weight<=1) || stride<64 || g_b.skel_parents_read<55 ||
       !plausible_ptr(mc) || region_end(mc)<mc+0x50)return 0;
    adj=*(ULONGLONG *)(uintptr_t)(mc+0x48);
    if(!hand_slots_owned(arm,objs,mc,adj) || region_end(objs)<objs+DG_OBJS_ARRAY+54ULL*stride+64)return 0;
    if(blade_fingers.mask && (blade_fingers.arm!=arm || blade_fingers.adj!=adj ||
       blade_fingers.objs!=objs || blade_fingers.mc!=mc))blade_fingers_release();
    if(blade_fingers.mask){
        unsigned previous=(blade_fingers.mask&(1ull<<22)?1u:0u)|(blade_fingers.mask&(1ull<<39)?2u:0u);
        if(previous!=sides)blade_fingers_release();
    }
    if(g_b.adjust_frame==1) {
        frame.live=1;frame.valid=arm_frame_yaw(*(short *)(uintptr_t)(player+0x82),frame.q);
        if(!frame.valid)return 0;
    }
    memset(slots,0,sizeof slots);
    for(int side=0;side<2;side++) {
        if(!(sides&(1u<<side)))continue;
        int offset=side*17;
        const float *index=(float *)(uintptr_t)(objs+DG_OBJS_ARRAY+(25+offset)*(ULONGLONG)stride);
        const float *little=(float *)(uintptr_t)(objs+DG_OBJS_ARRAY+(35+offset)*(ULONGLONG)stride);
        double axis[3],len=0;
        for(int j=0;j<17;j++) {
            int parent=hand_pose_parents[j];
            if(g_b.skel_parents[21+offset+j]!=(side?(parent==6?10:parent+17):parent))return 0;
        }
        for(int k=0;k<3;k++){axis[k]=(index[12+k]-little[12+k])*(side?-1:1);len+=axis[k]*axis[k];}
        len=sqrt(len);if(!(len>1 && len<250))return 0;
        for(int i=0;i<10;i++) {
            int n=side*10+i,j=blade_flex_joints[i]+offset;
            uint64_t bit=1ULL<<j;double world[4],q[4];
            double half=(i<2?20.0:(i%2?45.0:38.0))*weight*0.00872664626;
            const float *slot=(float *)(uintptr_t)(adj+j*16);
            if((*(uint64_t *)(uintptr_t)(mc+0x38)&bit)!=(blade_fingers.mask&bit) ||
               ((blade_fingers.mask&bit) && memcmp(slot,blade_fingers.cached[n],16)))return 0;
            for(int k=0;k<3;k++)world[k]=axis[k]*sin(half)/len;
            world[3]=cos(half);
            if(!world_quat_to_adjust(&frame,world,q))return 0;
            for(int k=0;k<4;k++)slots[n][k]=(float)q[k];
            mask|=bit;
        }
    }
    for(int n=0;n<20;n++) {
        int j=blade_flex_joints[n%10]+(n>=10?17:0);
        float *slot=(float *)(uintptr_t)(adj+j*16);
        if(!(mask&(1ull<<j)))continue;
        if(!blade_fingers.mask)memcpy(blade_fingers.before[n],slot,16);
        memcpy(slot,slots[n],16);
    }
    memcpy(blade_fingers.cached,slots,sizeof slots);
    blade_fingers.arm=arm;blade_fingers.objs=objs;blade_fingers.mc=mc;blade_fingers.adj=adj;
    blade_fingers.mask=mask;*(uint64_t *)(uintptr_t)(mc+0x38)|=mask;return 1;
}
static int blade_fingers_apply(ULONGLONG arm,ULONGLONG player,double weight)
{
    return grip_fingers_apply(arm,player,weight,3);
}
static void hand_pose_release(void)
{
    int j;
    blade_fingers_release();
    if(g_hand_pose.active && hand_pose_owner()) for(j=0;j<17;j++) {
        float *slot=(float *)(ULONG_PTR)(g_hand_pose.adj+(21+j)*16);
        if(!memcmp(slot,g_hand_pose.cached+j*4,16)) {
            memcpy(slot,g_hand_pose.before+j*4,16);
            *(volatile ULONGLONG *)(ULONG_PTR)(g_hand_pose.mc+0x38)&=~(1ULL<<(21+j));
        }
    }
    memset(&g_hand_pose,0,sizeof g_hand_pose);
}
static int hand_pose_quat(ULONGLONG base,int id,int stride,const DG_ADJ_FRAME *frame,double q[4])
{
    ULONGLONG address=base+(ULONGLONG)id*stride;
    double rows[3][3],world[4];int r,c;
    if(!looks_like_matrix(address)) return 0;
    for(r=0;r<3;r++)for(c=0;c<3;c++)
        rows[r][c]=((volatile float *)(ULONG_PTR)address)[r*4+c];
    return dg_ik_basis_quat(rows,world) && world_quat_to_adjust(frame,world,q);
}

static int hand_pose_mirror_local(const DG_ADJ_FRAME *frame,const double local[4],double out[4])
{
    double m[4];
    if(frame && frame->live) { if(!adjust_quat_to_world(frame,local,m))return 0; }
    else memcpy(m,local,sizeof m);
    m[1]=-m[1];m[2]=-m[2];
    if(!dg_ik_quat_normalize(m))return 0;
    if(frame && frame->live)return world_quat_to_adjust(frame,m,out);
    memcpy(out,m,sizeof m);return 1;
}
static void hand_pose_now(ULONGLONG arm,const DG_BRIDGE_ARM_TARGET *t)
{
    ULONGLONG current=0,pw=0,objs,mc,adj,base,mask;
    LONG weapon=-1,stride=g_b.skel_stride;
    DG_ADJ_FRAME frame=ADJ_FRAME_LEGACY;
    double right[18][4],left[18][4],previous[17][4],next[17][4],wrist[4],pose[17][4];
    float slots[68];int j,k;
    if(t && t->optics.valid && g_left.active && optics_configured() &&
       resolve_player(&current,&pw,&weapon)==DG_RESOLVE_OK && current==arm){
        int parked=0;
        if(optics_locomotion(pw,&parked) && parked){
            if(g_hand_pose.active)hand_pose_release();
            if(!grip_fingers_apply(arm,pw,.72,2))blade_fingers_release();
            return;
        }
    }
    if(t && t->aim_weapon_id==13 && t->aim_arm==arm && t->write && t->left_valid &&
       g_b.ik_active && g_left.active && t->brace_weight>0 &&
       resolve_player(&current,&pw,&weapon)==DG_RESOLVE_OK && current==arm && weapon==13) {
        if(g_hand_pose.active)hand_pose_release();
        if(!blade_fingers_apply(arm,pw,t->brace_weight))blade_fingers_release();
        return;
    }
    if(blade_fingers.mask)blade_fingers_release();
    if(!t || !t->left_enabled || !t->left_valid || !t->write ||
        !g_b.ik_active || !g_left.active ||
        resolve_player(&current,&pw,&weapon)!=DG_RESOLVE_OK || current!=arm || weapon!=0 ||
        region_end(pw)<pw+0xCF8 || (RD32(pw+0xCF4)&0x10) || /* native combo */
        (g_b.s_late_player_lo&0x20900) || /* attack, hold, enemy pull */
        g_b.skel_parents_read<55 || stride<64) goto refuse;
    for(j=0;j<17;j++) {
        int p=hand_pose_parents[j];
        if(g_b.skel_parents[21+j]!=p ||
           g_b.skel_parents[38+j]!=(p==6?10:p+17)) goto refuse;
    }
    objs=*(volatile ULONGLONG *)(ULONG_PTR)arm;
    mc=*(volatile ULONGLONG *)(ULONG_PTR)(arm+8);
    if(!plausible_ptr(objs)||!plausible_ptr(mc)||region_end(mc)<mc+0x50||RD32(mc+0x14)<55) goto refuse;
    adj=*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x48);
    base=objs+DG_OBJS_ARRAY;
    if(!plausible_ptr(adj)||region_end(adj)<adj+55*16||region_end(objs)<base+54*(ULONGLONG)stride+64) goto refuse;
    if(g_hand_pose.active && (g_hand_pose.arm!=arm || g_hand_pose.objs!=objs ||
       g_hand_pose.mc!=mc || g_hand_pose.adj!=adj || g_hand_pose.stream!=t->stream_id)) hand_pose_release();
    mask=*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_HAND_POSE_MASK;
    if(g_hand_pose.active) {
        if(mask!=DG_HAND_POSE_MASK || memcmp((void *)(ULONG_PTR)(adj+21*16),g_hand_pose.cached,sizeof slots)) goto refuse;
        if(g_hand_pose.tick==g_b.c_ticks) return;
    } else if(mask) goto refuse;
    if(g_b.adjust_frame==1) {
        frame.live=1;frame.valid=arm_frame_yaw(*(volatile short *)(ULONG_PTR)(pw+0x82),frame.q);
        if(!frame.valid) goto refuse;
    }
    if(!hand_pose_quat(base,6,stride,&frame,right[0]) || !hand_pose_quat(base,10,stride,&frame,left[0])) goto refuse;
    for(j=0;j<17;j++) {
        if(!hand_pose_quat(base,21+j,stride,&frame,right[j+1]) ||
           !hand_pose_quat(base,38+j,stride,&frame,left[j+1])) goto refuse;
        for(k=0;k<4;k++)previous[j][k]=g_hand_pose.active?g_hand_pose.cached[j*4+k]:(k==3?1:0);
    }
    if(!g_hand_pose_rest.valid || g_hand_pose_rest.arm!=arm ||
       g_hand_pose_rest.objs!=objs || g_hand_pose_rest.mc!=mc || g_hand_pose_rest.adj!=adj) {
        double rest[17][4],fore[4],inv_fore[4],wrist[4];
        if(!hand_pose_quat(base,9,stride,&frame,fore))goto refuse;
        memcpy(wrist,left[0],sizeof wrist);
        if(g_left.wrist_seen[3]!=0 || g_left.wrist_seen[0]!=0 ||
           g_left.wrist_seen[1]!=0 || g_left.wrist_seen[2]!=0) {
            hand_pose_inv(g_left.wrist_seen,inv_fore);dg_ik_quat_mul(inv_fore,wrist,wrist);
        }
        hand_pose_inv(fore,inv_fore);dg_ik_quat_mul(inv_fore,wrist,wrist);
        if(!hand_pose_mirror_local(&frame,wrist,wrist))goto refuse;
        for(j=0;j<17;j++) {
            int p=hand_pose_parents[j]==6?0:hand_pose_parents[j]-20;
            double inv[4];
            hand_pose_inv(left[p],inv);
            dg_ik_quat_mul(inv,left[j+1],rest[j]);
            if(!hand_pose_mirror_local(&frame,rest[j],rest[j]))goto refuse;
        }
        memcpy(g_hand_pose_rest.local,rest,sizeof rest);
        memcpy(g_hand_pose_rest.wrist,wrist,sizeof wrist);
        g_hand_pose_rest.frame=frame;
        g_hand_pose_rest.arm=arm;g_hand_pose_rest.objs=objs;
        g_hand_pose_rest.mc=mc;g_hand_pose_rest.adj=adj;g_hand_pose_rest.valid=1;
        if(g_b.log)g_b.log("  hands: captured unarmed finger rest; retained across equip\r\n");
    }
    memcpy(wrist,right[0],sizeof wrist);
    if(g_b.hand_have_desired && !world_quat_to_adjust(&frame,g_b.hand_desired,wrist))goto refuse;
    /* Adjustments are engine-space rotations. Conjugate around the wrist
       SetPos is about to produce, not the hierarchy's previous wrist. */
    /* A local quaternion was formed from two conjugated world orientations.
       It still carries that adjustment basis: F^-1 * local_world * F.
       Re-express the captured value before using it under a new actor yaw. */
    for(j=0;j<17;j++) {
        double local_world[4];
        if(!adjust_quat_to_world(&g_hand_pose_rest.frame,g_hand_pose_rest.local[j],local_world) ||
           !world_quat_to_adjust(&frame,local_world,pose[j]))goto refuse;
    }
    hand_pose_solve(right,pose,previous,wrist,next);
    if(g_b.log && hand_observe_step(arm,t->stream_id,t->pair_id,t->observed_right_stick_x)) {
        double fore[4];
        if(hand_pose_quat(base,5,stride,&frame,fore)) {
            g_b.log("  hand turn: tick=%ld pair=%llu stream=%lu stick_x=%.4f stick_yaw_rad=%.6f body_yaw_word=%d frame_live=%d\r\n",
                g_b.c_ticks,(unsigned long long)t->pair_id,t->stream_id,t->observed_right_stick_x,
                t->stick_yaw_rad,(int)*(volatile short *)(ULONG_PTR)(pw+0x82),frame.live);
            g_b.log("  hand observe: tick=%ld pair=%llu stream=%lu actor=%llX weapon=%ld trigger=%.4f ctrl=%.5f,%.5f,%.5f,%.5f fore=%.5f,%.5f,%.5f,%.5f wrist_seen=%.5f,%.5f,%.5f,%.5f wrist_next=%.5f,%.5f,%.5f,%.5f\r\n",
                g_b.c_ticks,(unsigned long long)t->pair_id,t->stream_id,arm,weapon,t->observed_right_trigger,
                t->hand_quat[0],t->hand_quat[1],t->hand_quat[2],t->hand_quat[3],
                fore[0],fore[1],fore[2],fore[3],right[0][0],right[0][1],right[0][2],right[0][3],
                wrist[0],wrist[1],wrist[2],wrist[3]);
            for(j=0;j<17;j++) {
                int p=hand_pose_parents[j]==6?0:hand_pose_parents[j]-20;
                double inv[4],local[4],pre[4];
                hand_pose_inv(previous[j],inv);dg_ik_quat_mul(inv,right[j+1],pre);
                hand_pose_inv(right[p],inv);dg_ik_quat_mul(inv,pre,local);
                g_b.log("  hand joint: pair=%llu stream=%lu joint=%d joint_seen=%.5f,%.5f,%.5f,%.5f native_local_estimate=%.5f,%.5f,%.5f,%.5f adj_owned=%.5f,%.5f,%.5f,%.5f adj_next=%.5f,%.5f,%.5f,%.5f\r\n",
                    (unsigned long long)t->pair_id,t->stream_id,21+j,
                    right[j+1][0],right[j+1][1],right[j+1][2],right[j+1][3],local[0],local[1],local[2],local[3],
                    previous[j][0],previous[j][1],previous[j][2],previous[j][3],
                    next[j][0],next[j][1],next[j][2],next[j][3]);
            }
        }
    }
    for(j=0;j<17;j++) {
        double n=0;
        for(k=0;k<4;k++){n+=next[j][k]*next[j][k];slots[j*4+k]=(float)next[j][k];}
        if(!isfinite(n)||fabs(n-1)>0.01)goto refuse;
    }
    if(!g_hand_pose.active) {
        memcpy(g_hand_pose.before,(void *)(ULONG_PTR)(adj+21*16),sizeof slots);
        if(g_b.log)g_b.log("  hands: unarmed right fingers mirror left (17 joints)\r\n");
    }
    g_hand_pose.arm=arm;g_hand_pose.objs=objs;g_hand_pose.mc=mc;g_hand_pose.adj=adj;
    g_hand_pose.stream=t->stream_id;g_hand_pose.tick=g_b.c_ticks;
    memcpy((void *)(ULONG_PTR)(adj+21*16),slots,sizeof slots);
    memcpy(g_hand_pose.cached,slots,sizeof slots);g_hand_pose.active=1;
    *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=DG_HAND_POSE_MASK;
    return;
refuse:
    if(g_b.log && t && hand_observe_step(arm,t->stream_id,t->pair_id,t->observed_right_stick_x))
        g_b.log("  hand observe refused: tick=%ld pair=%llu stream=%lu actor=%llX weapon=%ld trigger=%.4f write=%d right=%d left=%d status=%08X\r\n",
            g_b.c_ticks,(unsigned long long)t->pair_id,t->stream_id,arm,weapon,t->observed_right_trigger,
            t->write,g_b.ik_active,g_left.active,(unsigned)g_b.s_late_player_lo);
    hand_pose_release();
}
