/* Establish a neutral right wrist from the mirrored LEFT native wrist,
   rather than preserving the native right weapon/SetPos rest direction.
   Only calibration uses the opposite hand. Subsequent right controller
   deltas still drive the right wrist independently. */
static int unarmed_hand_rest(ULONGLONG arm,const double root_inv[4],
                             const DG_ADJ_FRAME *frame,double out[4])
{
    ULONGLONG objs,base,mc,adj;
    double left[4],root[4],inv[4],relative[4],mirrored[4],world[4];
    double aq[4],upper[4],fore[4],chain[4];
    int j,k,stride=g_b.skel_stride;
    if(!g_b.arm_map_unarmed || g_b.skel_parents_read<=10 || stride<64 ||
       g_b.skel_parents[7]!=2 || g_b.skel_parents[8]!=7 ||
       g_b.skel_parents[9]!=8 || g_b.skel_parents[10]!=9) return 0;
    objs=*(volatile ULONGLONG *)(ULONG_PTR)arm;
    base=objs+DG_OBJS_ARRAY;
    if(!plausible_ptr(objs)||region_end(objs)<base+10*(ULONGLONG)stride+64 ||
       !hand_pose_quat(base,10,stride,frame,left) ||
       !hand_pose_quat(base,0,stride,frame,root))return 0;
    if(g_left.active) {
        if(g_left.arm!=arm || !left_owner(&mc,&adj) ||
           (*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_MASK)!=DG_LEFT_MASK ||
           memcmp((void *)(ULONG_PTR)(adj+8*16),g_left.cached,sizeof g_left.cached))return 0;
        /* Cached left adjustments are already in the engine frame, as are
           left/root above. Strip their inherited rotation before mirroring. */
        for(j=0;j<2;j++) {
            for(k=0;k<4;k++)aq[k]=g_left.cached[j*4+k];
            if(!dg_ik_quat_normalize(aq))return 0;
            memcpy(j?fore:upper,aq,sizeof aq);
        }
        dg_ik_quat_mul(fore,upper,chain);hand_pose_inv(chain,inv);
        dg_ik_quat_mul(inv,left,left);
    }
    hand_pose_inv(root,inv);
    dg_ik_quat_mul(inv,left,relative);
    relative[1]=-relative[1];relative[2]=-relative[2];
    dg_ik_quat_mul(root,relative,mirrored);
    if(!adjust_quat_to_world(frame,mirrored,world))return 0;
    quat_rebase(root_inv,world,out);
    return dg_ik_quat_normalize(out);
}

/* Native local wrist relation is the stable rest, not the controller pose at
   FPS entry. Upper/fore IK still track the right controller's position. */
static int unarmed_wrist_target(ULONGLONG arm,const DG_ADJ_FRAME *frame,
    const double q4[4],const double q5[4],int cached,double out[4])
{
    ULONGLONG objs=*(volatile ULONGLONG *)(ULONG_PTR)arm,base;
    double left[4],lf[4],rf[4],inv[4],local[4],chain[4],a[4],b[4],world[4];
    int k,stride=g_b.skel_stride;
    if(!g_b.arm_map_unarmed || g_b.skel_parents_read<=10 || stride<64 ||
       g_b.skel_parents[6]!=5 || g_b.skel_parents[10]!=9)return 0;
    base=objs+DG_OBJS_ARRAY;
    if(!plausible_ptr(objs)||region_end(objs)<base+10*(ULONGLONG)stride+64 ||
       !hand_pose_quat(base,10,stride,frame,left) ||
       !hand_pose_quat(base,9,stride,frame,lf) ||
       !hand_pose_quat(base,5,stride,frame,rf))return 0;
    hand_pose_inv(lf,inv);dg_ik_quat_mul(inv,left,local);
    local[1]=-local[1];local[2]=-local[2];
    /* The palm must not inherit a later native wrist animation either. The
       None finger writer captures the complete local rest on this rig. */
    if(g_hand_pose_rest.valid && g_hand_pose_rest.arm==arm &&
       g_hand_pose_rest.objs==objs &&
       g_hand_pose_rest.mc==*(volatile ULONGLONG *)(ULONG_PTR)(arm+8)) {
        double local_world[4];
        if(!adjust_quat_to_world(&g_hand_pose_rest.frame,g_hand_pose_rest.wrist,local_world) ||
           !world_quat_to_adjust(frame,local_world,local))return 0;
    }
    if(cached) {
        for(k=0;k<4;k++){a[k]=g_b.arm_map_cached_adjust[k];b[k]=g_b.arm_map_cached_adjust[4+k];}
        dg_ik_quat_mul(b,a,chain);hand_pose_inv(chain,inv);
        dg_ik_quat_mul(inv,rf,rf);
    }
    dg_ik_quat_mul(q5,q4,chain);dg_ik_quat_mul(chain,rf,rf);
    dg_ik_quat_mul(rf,local,world);
    return adjust_quat_to_world(frame,world,out) && dg_ik_quat_normalize(out);
}
