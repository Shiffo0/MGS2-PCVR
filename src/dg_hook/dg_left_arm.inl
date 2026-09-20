/* Included after the right solver: independent ownership of joints 8/9.
   Joint 10 is independently owned only while free-wrist tracking is valid. */
#include "dg_twohand.h"
#include "dg_m9_contact.h"
#define DG_LEFT_MASK ((1ULL << 8) | (1ULL << 9))
typedef struct {
    ULONGLONG arm, objs, mctrl, adjust;
    unsigned long stream, pair;
    LONG settle;
    int active, initialized, root_valid;
    float cached[8];
    int wrist_active;
    float wrist_before[4],wrist_cached[4];
    double wrist_seen[4];
    DG_FREE_WRIST_STATE wrist;
    DG_ARM_MAP_STATE map;
    DG_POSITION_STATE camera_position;
    double root0[4], stick0;
    int stick_valid;
    DG_TWOHAND grip;
    unsigned long long weapon;
    double anchor[3];
    int anchor_valid;
    DG_M9_CONTACT slide_contact;
    double wrist_world[4];
    int wrist_world_valid;
    unsigned long accepted, refused;
} DG_LEFT_STATE;
static DG_LEFT_STATE g_left;
static volatile LONG g_left_accepted, g_left_refused, g_left_support_pairs;
static volatile LONG g_left_status, g_left_target[3], g_left_blend;
static volatile LONG g_left_reason;
/* Per unique solved pair: aim/coherence, right commit, anchor, reach,
   distance, dwell, attached. Keep these across releases for run diagnosis. */
static volatile LONG g_left_support_gate[7];

/* Weapon-only pose streams can retain the physical left-hand calibration.
   Older callers use the main stream; owner and explicit calibration changes
   still invalidate the left epoch. */
static unsigned long left_calibration_stream(const DG_BRIDGE_ARM_TARGET *t)
{
    return t->left_stream_id ? t->left_stream_id : t->stream_id;
}

static int left_owner(ULONGLONG *mc, ULONGLONG *adj)
{
    ULONGLONG arm;
    if (!g_b.a.gm_player_arm_body) return 0;
    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm) || arm != g_left.arm ||
        region_end(arm) < arm + 0x10) return 0;
    if (*(volatile ULONGLONG *)(ULONG_PTR)arm != g_left.objs) return 0;
    *mc = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 8);
    if (!plausible_ptr(*mc) || *mc != g_left.mctrl ||
        region_end(*mc) < *mc + 0x50 || RD32(*mc + 0x14) < 21 ||
        RD32(*mc + 0x14) > 255) return 0;
    *adj = *(volatile ULONGLONG *)(ULONG_PTR)(*mc + 0x48);
    return plausible_ptr(*adj) && *adj == g_left.adjust &&
        region_end(*adj) >= *adj + 11 * 16;
}
/* Called only after the seam's fresh FPS/safety checks and left solve.
   The native actor owns visibility again on its next update. This does not
   alter ARM_INVISIBLE, weapon selection, or the general arm-show setting. */
static int left_arm_show_unarmed(ULONGLONG arm, const DG_BRIDGE_ARM_TARGET *t)
{
    ULONGLONG current = 0, mc, adj;
    LONG weapon = -1;
    return t && t->left_enabled && t->left_valid && g_left.active &&
        g_left.arm == arm && g_left.stream == left_calibration_stream(t) &&
        g_left.pair == t->pair_id && left_owner(&mc, &adj) &&
        resolve_player(&current, NULL, &weapon) == DG_RESOLVE_OK &&
        current == arm && weapon == 0; /* WP_None */
}

static void left_wrist_release(void)
{
    g_left.wrist_world_valid=0;
    ULONGLONG mc,adj;
    if(g_left.wrist_active && left_owner(&mc,&adj) &&
       !memcmp((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_cached,16)) {
        memcpy((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_before,16);
        *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&=~(1ULL<<10);
    }
    g_left.wrist_active=0;memset(&g_left.wrist,0,sizeof g_left.wrist);
}

/* The observed hierarchy contains last pair's wrist adjustment, whereas the
   next finger-rest capture runs after this pair's write. Keep both distinct. */
static void left_wrist_now(ULONGLONG base,int stride,ULONGLONG mc,ULONGLONG adj,
    const DG_ADJ_FRAME *frame,const DG_BRIDGE_ARM_TARGET *t,
    const double qa[4],const double qb[4],int had_active,double support_blend,
    const double *contact_q)
{
    DG_IK_HAND_IN in;double rows[3][3],native[4],chain[4],inv[4],want[4],q[4],a[4];
    const double identity[4]={0,0,0,1};int r,k;
    g_left.wrist_world_valid=0;
    memcpy(g_left.wrist_seen,identity,sizeof identity);
    if(g_left.wrist_active)for(k=0;k<4;k++)g_left.wrist_seen[k]=g_left.wrist_cached[k];
    if(!t->free_left.enabled || !t->free_left.valid || support_blend>0) {
        left_wrist_release();return;
    }
    if(g_left.wrist_active) {
        if(!(*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&(1ULL<<10)) ||
           memcmp((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_cached,16)) {
            left_wrist_release();return;
        }
    } else if(*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&(1ULL<<10))return;
    memset(&in,0,sizeof in);
    for(r=0;r<3;r++)for(k=0;k<3;k++)
        rows[r][k]=((volatile float *)(ULONG_PTR)(base+10*(ULONGLONG)stride))[r*4+k];
    if(!dg_ik_basis_quat(rows,in.live))goto refuse_wrist;
    for(r=0;r<3;r++) {
        for(k=0;k<4;k++)a[k]=r==2 ? g_left.wrist_seen[k] :
            had_active ? g_left.cached[r*4+k] : identity[k];
        if(!adjust_quat_to_world(frame,a,r==0?in.prev_upper:r==1?in.prev_fore:in.prev_hand))goto refuse_wrist;
    }
    dg_ik_quat_mul(in.prev_fore,in.prev_upper,chain);
    dg_ik_quat_mul(in.prev_hand,chain,chain);dg_ik_quat_conj(chain,inv);
    dg_ik_quat_mul(inv,in.live,native);
    if(!adjust_quat_to_world(frame,qa,in.new_upper) ||
       !adjust_quat_to_world(frame,qb,in.new_fore))goto refuse_wrist;
    /* None starts with the native wrist relation on the TRACKED forearm.
       Capturing the original animation's world direction instead leaves the
       palm pointing back into an arm that IK has already moved elsewhere.
       Only the initial reference changes; subsequent controller rotations
       remain independent of the arm's position and native animation. */
    /* Equipment changes the native animation, not the controller neutral.
       Keep the accepted free-hand reference until tracking/owner/stream loss
       or an explicit calibration releases it. */
    if(g_b.arm_map_unarmed) {
        dg_ik_quat_mul(in.new_fore,in.new_upper,chain);
        dg_ik_quat_mul(chain,native,native);
    }
    if(!dg_free_wrist_step(&g_left.wrist,&t->free_left,native,want))goto refuse_wrist;
    if(contact_q)memcpy(want,contact_q,sizeof want);
    memcpy(in.desired,want,sizeof want);
    if(!dg_ik_hand_adjust(&in,q))goto refuse_wrist;
    dg_pose_quat_blend(identity,q,t->left_weight,q);
    if(!world_quat_to_adjust(frame,q,a))goto refuse_wrist;
    if(!g_left.wrist_active)memcpy(g_left.wrist_before,(void *)(ULONG_PTR)(adj+10*16),16);
    for(k=0;k<4;k++)g_left.wrist_cached[k]=(float)a[k];
    memcpy((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_cached,16);
    *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=1ULL<<10;g_left.wrist_active=1;
    memcpy(g_left.wrist_world,want,sizeof want);g_left.wrist_world_valid=1;return;
refuse_wrist:
    left_wrist_release();
}

#include "dg_left_model.inl"
static void left_arm_release(void)
{
    dg_bridge_model_arm_clear();
    hand_pose_release();
    left_wrist_release();
    ULONGLONG mc, adj;
    int j, k;
    if (g_left.active && left_owner(&mc, &adj)) {
        /* Relinquish only slots still containing this writer's bytes. */
        for (j = 8; j <= 9; j++) {
            volatile float *q = (volatile float *)(ULONG_PTR)(adj + j * 16);
            int ours = 1;
            for (k = 0; k < 4; k++)
                if (q[k] != g_left.cached[(j-8)*4+k]) ours = 0;
            if (ours) {
                q[0] = q[1] = q[2] = 0; q[3] = 1;
                *(volatile ULONGLONG *)(ULONG_PTR)(mc + 0x38) &= ~(1ULL << j);
            }
        }
    }
    memset(&g_left, 0, sizeof g_left);
    InterlockedExchange(&g_left_status,0);
    InterlockedExchange(&g_left_blend,0);
}

/* Measured 55-joint hand: palm38/48 and five three-joint finger chains.
 * Their distal-joint centroid is an inspectable grip contact, not a guessed
 * wrist offset or a claim about the skinned mesh surface. */
static int left_m9_finger_contact(ULONGLONG base,int stride,ULONGLONG end,double local[3])
{
    static const int parents[17]={10,38,39,40,38,42,43,38,45,46,38,48,49,50,48,52,53};
    static const int tips[5]={41,44,47,51,54};
    double rows[3][3],q[4],inv[4],delta[3]={0,0,0},len=0;
    int i,k;
    if(stride<64 || g_b.skel_parents_read<55 || end<base+54*(ULONGLONG)stride+64)return 0;
    for(i=0;i<17;i++)if(g_b.skel_parents[38+i]!=parents[i])return 0;
    if(!looks_like_matrix(base+10*(ULONGLONG)stride))return 0;
    for(i=0;i<3;i++)for(k=0;k<3;k++)rows[i][k]=((float *)(ULONG_PTR)(base+10*(ULONGLONG)stride))[i*4+k];
    if(!dg_ik_basis_quat(rows,q))return 0;
    for(i=0;i<5;i++) {
        float *tip=(float *)(ULONG_PTR)(base+tips[i]*(ULONGLONG)stride);
        float *wrist=(float *)(ULONG_PTR)(base+10*(ULONGLONG)stride);
        if(!looks_like_matrix((ULONGLONG)(ULONG_PTR)tip))return 0;
        for(k=0;k<3;k++)delta[k]+=(tip[12+k]-wrist[12+k])/5.0;
    }
    dg_ik_quat_conj(q,inv);arm_quat_rotate(inv,delta,local);
    for(k=0;k<3;k++)len+=local[k]*local[k];
    return _finite(len) && len>1 && len<250.0*250.0;
}
static void left_arm_now(ULONGLONG arm, const DG_BRIDGE_ARM_TARGET *t,
                         int right_committed)
{
    static const int joint_ids[5] = {2,7,8,9,10};
    ULONGLONG objs, mc, adj, base, end, pw = 0, pa = 0, pe;
    LONG stride;
    DG_ADJ_FRAME frame = ADJ_FRAME_LEGACY;
    DG_ARM_MAP_IN mi;
    DG_ARM_MAP_OUT mo;
    DG_IK_IN ii;
    DG_IK_OUT io;
    DG_IK_ORIENT_IN oi;
    DG_IK_ORIENT_OUT oo;
    double live[5][3], clean[5][3], root[4][4], rb[3][3], rq[4];
    double cq[4] = {0,0,0,1}, inv[4], rel[4], norm;
    double target[3], support[3], delta[3], qa[4], qb[4], blend = 0;
    float slots[8];
    int j,k,r, aiming, had_active, reason=1, support_gate=0;
    double finger_local[3],contact_q[4];int contact_drive=0,finger_valid=0;
    if (!t || !t->left_enabled || !t->left_valid || !isfinite(t->left_weight) ||
        t->left_weight <= 0 || t->left_weight > 1) {
        InterlockedExchange(&g_left_reason,0);
        left_arm_release(); return;
    }
    if (!plausible_ptr(arm) || region_end(arm) < arm + 0x10) goto refuse;
    objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
    mc = *(volatile ULONGLONG *)(ULONG_PTR)(arm+8);
    if (!plausible_ptr(objs) || !plausible_ptr(mc) ||
        region_end(mc) < mc+0x50 || RD32(mc+0x14) < 21 || RD32(mc+0x14)>255)
        goto refuse;
    adj = *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x48);
    if (!plausible_ptr(adj) || region_end(adj) < adj+11*16) goto refuse;
    if (!g_left.initialized || g_left.arm != arm || g_left.objs != objs ||
        g_left.mctrl != mc || g_left.adjust != adj || g_left.stream != left_calibration_stream(t)) {
        left_arm_release();
        g_left.arm=arm; g_left.objs=objs; g_left.mctrl=mc; g_left.adjust=adj;
        g_left.stream=left_calibration_stream(t); g_left.settle=g_b.c_ticks+2;
        g_left.initialized=1; return;
    }
    /* Validate current owner and heading before duplicate handling. */
    reason=2;
    if (!left_owner(&mc,&adj)) goto refuse;
    reason=3;
    if (g_b.adjust_frame == 1) {
        if (resolve_player(&pa,&pw,NULL) != DG_RESOLVE_OK || pa != arm) goto refuse;
        pe=region_end(pw);
        if (!pe || pe < pw+0x84) goto refuse;
        frame.live=1;
        frame.valid=arm_frame_yaw(*(volatile short *)(ULONG_PTR)(pw+0x82),frame.q);
        if (!frame.valid) goto refuse;
    }
    aiming = t->twohand_enabled && t->hands_coherent && t->aim_write &&
        t->aim_sample_seq == t->left_sample_seq &&
        t->aim_sample_time == t->left_sample_time &&
        g_b.fire_mode == DG_FIRE_MODE_ON && g_b.fire.state == DG_FIRE_HOLD;
    if (reload_owns_left()) aiming=0;
    if (!aiming) dg_twohand_reset(&g_left.grip);
    /* Revoke invalid position even on the closing eye of a cached pair. */
    if(t->left_position.enabled) {
        double position_frame[4];
        if(!dg_position_frame(&t->left_position,position_frame)) goto refuse;
    }
    if(!t->free_left.enabled || !t->free_left.valid)left_wrist_release();
    if (t->pair_id == g_left.pair) return;
    g_left.pair=t->pair_id;
    if (g_b.c_ticks <= g_left.settle) return;
    reason=4;
    stride=g_b.skel_stride;
    if (stride < 64 || g_b.skel_parents_read <= 10 ||
        g_b.skel_parents[7]!=2 || g_b.skel_parents[8]!=7 ||
        g_b.skel_parents[9]!=8 || g_b.skel_parents[10]!=9) goto refuse;
    base=objs+DG_OBJS_ARRAY; end=region_end(objs);
    reason=5;
    if (!end || base+10*(ULONGLONG)stride+64>end || !looks_like_matrix(base)) goto refuse;
    for (r=0;r<4;r++) for(k=0;k<4;k++)
        root[r][k]=((volatile float *)(ULONG_PTR)base)[r*4+k];
    for(r=0;r<3;r++) for(k=0;k<3;k++) rb[r][k]=root[r][k];
    if (!dg_ik_basis_quat(rb,rq)) goto refuse;
    for(j=0;j<5;j++) {
        ULONGLONG m=base+joint_ids[j]*(ULONGLONG)stride;
        if (!looks_like_matrix(m)) goto refuse;
        for(k=0;k<3;k++) live[j][k]=((volatile float *)(ULONG_PTR)m)[12+k];
    }
    memcpy(clean,live,sizeof clean);
    had_active=g_left.active;
    reason=6;
    if (had_active) {
        if ((*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_MASK)!=DG_LEFT_MASK)
            goto refuse;
        for(j=0;j<8;j++) {
            slots[j]=((volatile float *)(ULONG_PTR)(adj+8*16))[j];
            if (slots[j]!=g_left.cached[j]) goto refuse;
        }
        if (!arm_remove_cached_adjust(&frame,live,slots,clean)) goto refuse;
    } else if (*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_MASK) goto refuse;
    reason=7;
    memset(&mi,0,sizeof mi);
    mi.anchor_shoulder=1; mi.player_reach=t->player_reach_view;
    mi.upper=vdist3(clean[2],clean[3]); mi.fore=vdist3(clean[3],clean[4]);
    if (!arm_world_to_view(root,clean[2],mi.shoulder_view) ||
        !arm_world_to_view(root,clean[4],mi.native_wrist_view)) goto refuse;
    if (!g_left.root_valid) {
        memcpy(g_left.root0,rq,sizeof rq); g_left.root_valid=1;
        g_left.stick0=t->stick_yaw_rad; g_left.stick_valid=t->stick_yaw_valid;
    }
    arm_reference(&g_b.left_reference,t,g_left.root0,NULL,&g_left.stick0);
    dg_ik_quat_conj(rq,inv); dg_ik_quat_mul(inv,g_left.root0,rel);
    norm=sqrt(rel[1]*rel[1]+rel[3]*rel[3]);
    if (!(norm>1e-6)) goto refuse;
    cq[1]=rel[1]/norm; cq[3]=rel[3]/norm;
    if (g_b.arm_comp==1 && t->stick_yaw_valid && g_left.stick_valid) {
        double software=(g_b.follow_head_sign<0 ? -1:1)*(t->stick_yaw_rad-g_left.stick0);
        double half=atan2(cq[1],cq[3])+software*0.5;
        cq[1]=sin(half); cq[3]=cos(half);
    }
    arm_quat_rotate(cq,t->left_wrist_view,mi.controller_view);
    arm_quat_rotate(cq,t->left_shoulder_view,mi.player_shoulder);
    /* Both raw grips share one physical tracking space. Reusing the accepted
       right-hand mapping preserves their relative separation; calibrating
       the left from its native support animation bakes in a second offset.
       Seed only an uncalibrated left hand: equipment must not replace a
       previously accepted unarmed mapping or depend on right-arm settling. */
    if(t->left_position.enabled && t->position.enabled && !g_b.arm_map_unarmed &&
       !g_left.camera_position.ready) {
        if(!right_committed || !g_b.camera_position.ready) goto refuse;
        g_left.camera_position=g_b.camera_position;
    }
    if(t->left_position.enabled && g_left.camera_position.ready) {
        double camera_target[3];
        if(!dg_position_target(&g_left.camera_position,&t->left_position,camera_target) ||
           !arm_world_to_view(root,camera_target,mi.desired_view)) goto refuse;
        mi.explicit_target=1;
    }
    if (!dg_arm_map_step(&g_left.map,&mi,&mo)) {
        if (mo.flags&DG_ARM_MAP_F_CALIBRATED) return;
        goto refuse;
    }
    arm_view_to_world(root,mo.target_view,target);
    if(t->left_position.enabled && !g_left.camera_position.ready &&
       !dg_position_calibrate(&g_left.camera_position,&t->left_position,
                              target,mo.scale)) goto refuse;
    /* Slide attachment owns the hand ahead of ordinary support grip. */
    {double slide[3];if(t->aim_weapon_id==1 && m9_hand_contact(slide,t->stream_id)) {
        aiming=0;dg_twohand_reset(&g_left.grip);
    }}
    /* Capture a native support offset only from the right solve that finished
       on THIS invocation. No previous-pair desired pose is a valid anchor. */
    if (g_left.weapon != t->aim_weapon_id) {
        g_left.weapon=t->aim_weapon_id; g_left.anchor_valid=0;
        dg_twohand_reset(&g_left.grip);
    }
    if (aiming && right_committed && g_b.have_pred_wrist && g_b.hand_have_desired &&
        (g_b.rec_pair.flags&DG_REC_PAIR_F_BASE)) {
        support_gate=2;
        /* Keep sampling while free: the native aiming animation may still
           be entering its support pose on the first trigger-HOLD frame.
           Freeze the offset only when proximity actually engages it. */
        if (!g_left.anchor_valid || (!g_left.grip.engaged && g_left.grip.blend==0)) {
            for(k=0;k<3;k++) delta[k]=clean[4][k]-g_b.rec_pair.joint_world[3][k];
            arm_quat_unrotate(g_b.rec_pair.base_now,delta,g_left.anchor);
            norm=sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
            g_left.anchor_valid=_finite(norm) && norm<300;
        }
        if (g_left.anchor_valid) {
            arm_quat_rotate(g_b.hand_desired,g_left.anchor,delta);
            for(k=0;k<3;k++) support[k]=g_b.pred_wrist[k]+delta[k];
            norm=vdist3(clean[2],support);
            support_gate=3;
            if (norm < (mi.upper+mi.fore)*0.99 && norm > fabs(mi.upper-mi.fore)+1) {
                blend=dg_twohand_step(&g_left.grip,1,1,t->hands_distance_m,
                    t->left_sample_seq,t->left_sample_time);
                support_gate=blend>0 ? 6 :
                    (t->hands_distance_m>DG_TWOHAND_ENTER_M ? 4 : 5);
            } else dg_twohand_reset(&g_left.grip);
        }
    } else {
        support_gate=aiming ? 1 : 0;
        dg_twohand_reset(&g_left.grip);
    }
    InterlockedIncrement(&g_left_support_gate[support_gate]);
    if(blend>0) for(k=0;k<3;k++) target[k]+=(support[k]-target[k])*blend;
    reason=8;
    memset(&ii,0,sizeof ii);
    ii.upper=mi.upper; ii.fore=mi.fore; ii.max_reach_frac=.99;
    ii.min_elbow_deg=15; ii.max_elbow_deg=175;
    for(k=0;k<3;k++) { ii.shoulder[k]=clean[2][k]; ii.target[k]=target[k]; }
    delta[0]=clean[1][0]-clean[0][0]; delta[1]=0; delta[2]=clean[1][2]-clean[0][2];
    norm=sqrt(delta[0]*delta[0]+delta[2]*delta[2]);
    if (!(norm>1e-6)) goto refuse;
    ii.pole[0]=clean[2][0]+(mi.upper+mi.fore)*delta[0]/norm;
    ii.pole[1]=clean[2][1]-(mi.upper+mi.fore);
    ii.pole[2]=clean[2][2]+(mi.upper+mi.fore)*delta[2]/norm;
    if(!dg_ik_solve(&ii,&io)) goto refuse;
    finger_valid=left_m9_finger_contact(base,stride,end,finger_local);
    {
        double slide[3],free_q[4],contact_target[3];
        DG_FREE_WRIST_STATE preview=g_left.wrist;
        DG_IK_OUT contacted;
        int attached=right_committed && g_b.have_pred_wrist && g_b.hand_have_desired &&
            t->aim_weapon_id==1 && m9_hand_contact(slide,t->stream_id);
        int can_contact=finger_valid && preview.ready && g_left.wrist_world_valid && t->left_weight==1 &&
            t->free_left.enabled && t->free_left.valid && !reload_owns_left() && blend==0;
        if(can_contact && dg_free_wrist_step(&preview,&t->free_left,g_left.wrist_world,free_q)) {
            if(attached)contact_drive=dg_m9_contact_step(&g_left.slide_contact,GetTickCount64(),
                g_b.hand_desired,g_b.pred_wrist,slide,io.wrist,free_q,finger_local,contact_target,contact_q);
            else contact_drive=dg_m9_contact_release(&g_left.slide_contact,GetTickCount64(),
                io.wrist,free_q,contact_target,contact_q);
            if(contact_drive) {
                for(k=0;k<3;k++)ii.target[k]=contact_target[k];
                /* Refuse contact the physical arm cannot reach: accepting an
                   IK clamp would silently recreate the visible hand gap. */
                if(dg_ik_solve(&ii,&contacted) && vdist3(contacted.wrist,contact_target)<1.0)io=contacted;
                else {contact_drive=0;memset(&g_left.slide_contact,0,sizeof g_left.slide_contact);if(attached)m9_contact_cancel();}
            }
        } else {
            memset(&g_left.slide_contact,0,sizeof g_left.slide_contact);
            if(attached)m9_contact_cancel();
        }
    }
    memset(&oi,0,sizeof oi);
    for(k=0;k<3;k++) {
        oi.shoulder[k]=clean[2][k]; oi.elbow[k]=io.elbow[k]; oi.wrist[k]=io.wrist[k];
        oi.rest_upper[k]=clean[3][k]-clean[2][k];
        oi.rest_fore[k]=clean[4][k]-clean[3][k];
    }
    reason=9;
    if(!dg_ik_orient(&oi,&oo) || !world_quat_to_adjust(&frame,oo.upper_quat,qa) ||
        !world_quat_to_adjust(&frame,oo.fore_quat,qb)) goto refuse;
    { const double identity[4]={0,0,0,1};
      dg_pose_quat_blend(identity,qa,t->left_weight,qa);
      dg_pose_quat_blend(identity,qb,t->left_weight,qb); }
    reason=10;
    if(!left_owner(&mc,&adj)) goto refuse;
    left_model_publish(base,stride,&frame,t,qa,qb,had_active,io.elbow,io.wrist);
    left_wrist_now(base,stride,mc,adj,&frame,t,qa,qb,had_active,blend,contact_drive?contact_q:NULL);
    if(contact_drive && !g_left.wrist_world_valid) {
        m9_contact_cancel();goto refuse; /* no positional-only attachment */
    }
    for(j=0;j<8;j++) {
        float v=(float)(j<4 ? qa[j] : qb[j-4]);
        ((volatile float *)(ULONG_PTR)(adj+8*16))[j]=v;
        g_left.cached[j]=v;
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=DG_LEFT_MASK;
    g_left.active=1; g_left.accepted++;
    if(right_committed && g_b.have_pred_wrist && g_b.hand_have_desired &&
       finger_valid && g_left.wrist_world_valid) {
        double contact[3];arm_quat_rotate(g_left.wrist_world,finger_local,contact);
        for(k=0;k<3;k++)contact[k]+=io.wrist[k];
        m9_observe_hand(t,contact);
    }
    InterlockedIncrement(&g_left_accepted);
    InterlockedExchange(&g_left_reason,0);
    if (blend>0) InterlockedIncrement(&g_left_support_pairs);
    InterlockedExchange(&g_left_status,blend>0 ? 2 : 1);
    InterlockedExchange(&g_left_blend,f2l((float)blend));
    for(k=0;k<3;k++) InterlockedExchange(&g_left_target[k],f2l((float)io.wrist[k]));
    return;
refuse:
    InterlockedIncrement(&g_left_refused);
    InterlockedExchange(&g_left_reason,reason);
    left_arm_release();
}
