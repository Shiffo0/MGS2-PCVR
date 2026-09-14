/* Included after the right solver: independent ownership of joints 8/9.
   Joint 10 remains native; this feature controls the wrist position. */
#include "dg_twohand.h"
#define DG_LEFT_MASK ((1ULL << 8) | (1ULL << 9))
typedef struct {
    ULONGLONG arm, objs, mctrl, adjust;
    unsigned long stream, pair;
    LONG settle;
    int active, initialized, root_valid;
    float cached[8];
    DG_ARM_MAP_STATE map;
    double root0[4], stick0;
    int stick_valid;
    DG_TWOHAND grip;
    unsigned long long weapon;
    double anchor[3];
    int anchor_valid;
    unsigned long accepted, refused;
} DG_LEFT_STATE;
static DG_LEFT_STATE g_left;
static volatile LONG g_left_accepted, g_left_refused, g_left_support_pairs;
static volatile LONG g_left_status, g_left_target[3], g_left_blend;
static volatile LONG g_left_reason;
/* Per unique solved pair: aim/coherence, right commit, anchor, reach,
   distance, dwell, attached. Keep these across releases for run diagnosis. */
static volatile LONG g_left_support_gate[7];

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
        region_end(*adj) >= *adj + 10 * 16;
}
/* Called only after the seam's fresh FPS/safety checks and left solve.
   The native actor owns visibility again on its next update. This does not
   alter ARM_INVISIBLE, weapon selection, or the general arm-show setting. */
static int left_arm_show_unarmed(ULONGLONG arm, const DG_BRIDGE_ARM_TARGET *t)
{
    ULONGLONG current = 0, mc, adj;
    LONG weapon = -1;
    return t && t->left_enabled && t->left_valid && g_left.active &&
        g_left.arm == arm && g_left.stream == t->stream_id &&
        g_left.pair == t->pair_id && left_owner(&mc, &adj) &&
        resolve_player(&current, NULL, &weapon) == DG_RESOLVE_OK &&
        current == arm && weapon == 0; /* WP_None */
}

static void left_arm_release(void)
{
    hand_pose_release();
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
    if (!plausible_ptr(adj) || region_end(adj) < adj+10*16) goto refuse;
    if (!g_left.initialized || g_left.arm != arm || g_left.objs != objs ||
        g_left.mctrl != mc || g_left.adjust != adj || g_left.stream != t->stream_id) {
        left_arm_release();
        g_left.arm=arm; g_left.objs=objs; g_left.mctrl=mc; g_left.adjust=adj;
        g_left.stream=t->stream_id; g_left.settle=g_b.c_ticks+2;
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
    if (!aiming) dg_twohand_reset(&g_left.grip);
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
    if (!dg_arm_map_step(&g_left.map,&mi,&mo)) {
        if (mo.flags&DG_ARM_MAP_F_CALIBRATED) return;
        goto refuse;
    }
    arm_view_to_world(root,mo.target_view,target);
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
    for(j=0;j<8;j++) {
        float v=(float)(j<4 ? qa[j] : qb[j-4]);
        ((volatile float *)(ULONG_PTR)(adj+8*16))[j]=v;
        g_left.cached[j]=v;
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=DG_LEFT_MASK;
    g_left.active=1; g_left.accepted++;
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
