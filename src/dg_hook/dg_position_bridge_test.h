/* DG_HOOK_TEST only: included after bridge fixture helpers.
   The callback maps fixed raw XR poses with the production hook route.
   This exercises arm_ik_now, not a copied organic compensation formula.
   Expected camera-space wrist offset is independently specified: 495 mm
   forward, from a 500 mm player vector and 300+300 mm character bones.
   The synthetic hierarchy uses legacy adjust axes; it is not GPU evidence. */
#ifndef DG_POSITION_BRIDGE_TEST_H
#define DG_POSITION_BRIDGE_TEST_H

static int position_bridge_apply_slots(unsigned char *blob, int stride,
                                       const float *adjust)
{
    float *s = (float *)(blob + DG_OBJS_ARRAY + 4 * stride);
    float *e = (float *)(blob + DG_OBJS_ARRAY + 5 * stride);
    float *w = (float *)(blob + DG_OBJS_ARRAY + 6 * stride);
    double q4[4], q5[4], a[4], upper[3], fore[3], u[3], f[3], t[3];
    int k;
    for (k = 0; k < 4; k++) a[k] = adjust[16 + k];
    if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, a, q4)) return 0;
    for (k = 0; k < 4; k++) a[k] = adjust[20 + k];
    if (!adjust_quat_to_world(&ADJ_FRAME_LEGACY, a, q5)) return 0;
    for (k = 0; k < 3; k++) {
        upper[k] = e[12+k] - s[12+k];
        fore[k] = w[12+k] - e[12+k];
    }
    arm_quat_rotate(q4, upper, u);
    arm_quat_rotate(q4, fore, t);
    arm_quat_rotate(q5, t, f);
    for (k = 0; k < 3; k++) {
        e[12+k] = (float)(s[12+k] + u[k]);
        w[12+k] = (float)(s[12+k] + u[k] + f[k]);
    }
    return 1;
}

int dg_bridge_test_position_turn(
    void (*make_target)(double software_rad, DG_BRIDGE_ARM_TARGET *out))
{
    enum { STRIDE = 0x180, JOINTS = 7 };
    static unsigned char saved[sizeof g_b];
    static unsigned char blob[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char rig0[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char clean[DG_OBJS_ARRAY + JOINTS * STRIDE];
    static unsigned char obj[0x40], mc[0x70];
    static float adjust[55 * 4];
    ULONGLONG arm = (ULONGLONG)(ULONG_PTR)obj;
    DG_BRIDGE_ARM_TARGET target;
    double turn[4], prev_elbow[3], max_target = 0.0, max_wrist = 0.0;
    double max_sim = 0.0, max_elbow_step = 0.0;
    int leg, step, j, k, bad = 0, measured = 0;
    if (!make_target) return 1;
    memcpy(saved, &g_b, sizeof g_b);
    memset(&g_b, 0, sizeof g_b);
    memset(blob, 0, sizeof blob);
    memset(obj, 0, sizeof obj);
    memset(mc, 0, sizeof mc);
    memset(adjust, 0, sizeof adjust);
    adjust[27] = 1.0f;
    *(ULONGLONG *)(obj + 0x00) = (ULONGLONG)(ULONG_PTR)blob;
    *(ULONGLONG *)(obj + 0x08) = (ULONGLONG)(ULONG_PTR)mc;
    *(LONG *)(mc + 0x14) = 55;
    *(ULONGLONG *)(mc + 0x48) = (ULONGLONG)(ULONG_PTR)adjust;
    for (j = 0; j < JOINTS; j++) {
        float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
        m[0] = m[5] = m[10] = m[15] = 1.0f;
    }
    /* Nondegenerate pole points +X; shoulder at origin; two 300 mm bones. */
    ((float *)(blob + DG_OBJS_ARRAY + 3 * STRIDE))[12] = 100.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[12] = 180.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 5 * STRIDE))[14] = 240.0f;
    ((float *)(blob + DG_OBJS_ARRAY + 6 * STRIDE))[14] = 480.0f;
    memcpy(rig0, blob, sizeof blob);
    g_b.skel_stride = STRIDE;
    g_b.skel_parents_read = JOINTS;
    g_b.skel_region_end = sizeof blob;
    g_b.skel_parents[3] = 2; g_b.skel_parents[4] = 3;
    g_b.skel_parents[5] = 4; g_b.skel_parents[6] = 5;
    g_b.a.gm_player_arm_body = (ULONGLONG)(ULONG_PTR)&arm;
    g_b.arm_anchor = 1;
    g_b.arm_comp = 1;
    g_b.follow_head_sign = 1;
    arm_map_forget();
    make_target(0.0, &target);
    target.stream_id = 901; target.pair_id = 1;
    arm_ik_now(arm, &target);
    g_b.c_ticks = DG_ADJ_SETTLE_TICKS + 1;
    target.pair_id++;
    arm_ik_now(arm, &target);
    if (!g_b.arm_root_have_q0 || !g_b.stick_yaw0_have) bad++;

    /* Incremental out/back sweep contains 0,30,90,0. Second sweep has
       smooth organic body lag, returning to zero at both endpoints. */
    for (leg = 0; leg < 2; leg++) for (step = 0; step <= 36; step++) {
        double deg = (step <= 18 ? step : 36-step) * 5.0;
        double rad = deg * 3.14159265358979323846 / 180.0;
        double lag = leg ? -12.0 * sin(rad) : 0.0;
        double c = cos(rad), s = sin(rad), v[3], camera[3], err;
        LONG accepted = g_b.c_arm_pairs_accepted;
        memcpy(blob, rig0, sizeof blob);
        th_axis(0.0, 1.0, 0.0, deg + lag, turn);
        th_turn_rig(blob + DG_OBJS_ARRAY, STRIDE, JOINTS, turn);
        memcpy(clean, blob, sizeof blob);
        /* Model the previous slots carried by the newly posed hierarchy,
           so arm_ik_now exercises actual cached-adjust removal as well. */
        if (g_b.arm_map_cache_valid &&
            !position_bridge_apply_slots(blob, STRIDE, adjust)) bad++;
        make_target(rad, &target);
        target.stream_id = 901; target.pair_id = 3 + leg*37 + step;
        g_b.c_ticks++;
        arm_ik_now(arm, &target);
        if (g_b.c_arm_pairs_accepted != accepted + 1 ||
            !(g_b.rec_pair.flags & DG_REC_PAIR_F_ARM)) { bad++; continue; }
        measured++;
        for (j = 0; j < 2; j++) {
            for (k = 0; k < 3; k++) v[k] =
                (j ? g_b.rec_pair.ik_wrist[k] : g_b.rec_pair.ik_target[k]) -
                g_b.rec_pair.joint_world[1][k];
            camera[0] = c*v[0] - s*v[2];
            camera[1] = v[1]; camera[2] = s*v[0] + c*v[2];
            err = sqrt(camera[0]*camera[0] + camera[1]*camera[1] +
                       (camera[2]-495.0)*(camera[2]-495.0));
            if (j) { if (err > max_wrist) max_wrist = err; }
            else if (err > max_target) max_target = err;
            if (!_finite(err) || err > 0.1) bad++;
        }
        /* Apply this pair's published slots to a fresh animation skeleton.
           This is a desk hierarchy simulation, not a live renderer oracle. */
        if (!position_bridge_apply_slots(clean, STRIDE, adjust)) bad++;
        for (k = 0; k < 3; k++) v[k] =
            ((float *)(clean + DG_OBJS_ARRAY + 6*STRIDE))[12+k] -
            g_b.rec_pair.ik_wrist[k];
        err = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
        if (err > max_sim) max_sim = err;
        if (!_finite(err) || err > 0.1) bad++;
        for (k = 0; k < 3; k++) v[k] = g_b.rec_pair.ik_elbow[k] -
            g_b.rec_pair.joint_world[1][k];
        camera[0] = c*v[0]-s*v[2]; camera[1] = v[1];
        camera[2] = s*v[0]+c*v[2];
        if (step) {
            err = vdist3(camera, prev_elbow);
            if (err > max_elbow_step) max_elbow_step = err;
            if (!_finite(err) || err > 30.0) bad++;
        }
        memcpy(prev_elbow, camera, sizeof camera);
    }
    if (measured != 74) bad++;
    /* A local aim-pose shoulder translation must not translate the target.
       Carry the whole rig through world locomotion at the same time: freezing
       a world-space wrist would pass the first requirement but fail this one.
       Amplitude comes from the trigger capture, here applied in root-local
       space as a controlled fixture rather than claiming native replay. */
    {
        const double shift[3] = {84.568, 37.059, 122.110};
        double target_error = 0.0, endpoint_error = 0.0;
        int captured = 0;
        for (step = 0; step <= 60; ++step) {
            double blend = step <= 30 ? step / 30.0 : (60-step) / 30.0;
            double walk[3] = {step * 2.0, 0.0, step * -1.5};
            double expected[3] = {walk[0], walk[1], walk[2] + 495.0};
            double v[3], err;
            LONG accepted = g_b.c_arm_pairs_accepted;
            memcpy(blob, rig0, sizeof blob);
            for (j = 0; j < JOINTS; ++j) {
                float *m = (float *)(blob + DG_OBJS_ARRAY + j * STRIDE);
                for (k = 0; k < 3; ++k)
                    m[12+k] += (float)(walk[k] + (j >= 3 ? blend * shift[k] : 0.0));
            }
            memcpy(clean, blob, sizeof blob);
            if (g_b.arm_map_cache_valid &&
                !position_bridge_apply_slots(blob, STRIDE, adjust)) bad++;
            make_target(0.0, &target);
            target.stream_id = 901; target.pair_id = 100 + step;
            g_b.c_ticks++;
            arm_ik_now(arm, &target);
            if (g_b.c_arm_pairs_accepted != accepted + 1 ||
                !(g_b.rec_pair.flags & DG_REC_PAIR_F_ARM)) { bad++; continue; }
            captured++;
            for (k = 0; k < 3; ++k) v[k] = g_b.rec_pair.ik_target[k];
            err = vdist3(v, expected);
            if (err > target_error) target_error = err;
            if (!_finite(err) || err > 0.1) bad++;
            if (!position_bridge_apply_slots(clean, STRIDE, adjust)) bad++;
            for (k = 0; k < 3; ++k)
                v[k] = ((float *)(clean + DG_OBJS_ARRAY + 6*STRIDE))[12+k];
            err = vdist3(v, expected);
            if (err > endpoint_error) endpoint_error = err;
            if (!_finite(err) || err > 0.1) bad++;
        }
        if (captured != 61) bad++;
        printf("  %s position animated shoulder + locomotion: %d pairs, "
               "target %.6f, synthetic wrist %.6f mm worst\n",
               bad ? "FAIL" : "ok", captured, target_error, endpoint_error);
    }
    /* Recorded failure shape: camera rises while the arm root translates
       sideways/back and turns, plus a local animated shoulder displacement.
       Exercise the actual world-target override and downstream joint writes. */
    {
        double worst=0.0, endpoint=0.0;
        int count=0;
        for(step=0;step<=60;step++) {
            double b=step<=30?step/30.0:(60-step)/30.0;
            double expected[3]={0,169*b,297},v[3],err;
            LONG accepted=g_b.c_arm_pairs_accepted;
            memcpy(blob,rig0,sizeof blob);
            for(j=3;j<JOINTS;j++) {
                float *m=(float *)(blob+DG_OBJS_ARRAY+j*STRIDE);
                m[12]+=(float)(84.568*b);m[13]+=(float)(37.059*b);m[14]+=(float)(122.110*b);
            }
            th_axis(0,1,0,14*b,turn);
            th_turn_rig(blob+DG_OBJS_ARRAY,STRIDE,JOINTS,turn);
            for(j=0;j<JOINTS;j++) {
                float *m=(float *)(blob+DG_OBJS_ARRAY+j*STRIDE);
                m[12]+=(float)(105*b);m[14]-=(float)(51*b);
            }
            memcpy(clean,blob,sizeof blob);
            if(g_b.arm_map_cache_valid && !position_bridge_apply_slots(blob,STRIDE,adjust))bad++;
            make_target(0,&target);
            for(k=0;k<3;k++)target.wrist_view[k]=target.player_shoulder_view[k]+(k==2?300:0);
            target.stream_id=901;target.pair_id=200+step;
            target.position.enabled=target.position.valid=1;
            target.position.units=1000;target.position.view[3]=1;
            target.position.grip[2]=-.5;
            for(k=0;k<4;k++)target.position.camera[5*k]=1;
            target.position.camera[13]=169*b;
            g_b.c_ticks++;arm_ik_now(arm,&target);
            if(g_b.c_arm_pairs_accepted!=accepted+1 || !(g_b.rec_pair.flags&DG_REC_PAIR_F_ARM)){bad++;continue;}
            count++;
            for(k=0;k<3;k++)v[k]=g_b.rec_pair.ik_target[k];
            err=vdist3(v,expected);if(err>worst)worst=err;if(!_finite(err)||err>.1)bad++;
            if(!position_bridge_apply_slots(clean,STRIDE,adjust))bad++;
            for(k=0;k<3;k++)v[k]=((float *)(clean+DG_OBJS_ARRAY+6*STRIDE))[12+k];
            err=vdist3(v,expected);if(err>endpoint)endpoint=err;if(!_finite(err)||err>.1)bad++;
        }
        if(count!=61)bad++;
        printf("  %s camera position bridge: %d pairs, target %.6f, synthetic endpoint %.6f mm\n",bad?"FAIL":"ok",count,worst,endpoint);
    }
    arm_map_forget();
    if(g_b.camera_position.ready)bad++;
    memcpy(&g_b, saved, sizeof g_b);
    printf("  %-6s position production bridge: %d pairs; target %.6f mm, "
           "solver %.6f mm, synthetic slot endpoint %.6f mm; "
           "max elbow step %.6f mm (5 degree sweep, organic lag)\n",
           bad ? "FAIL" : "ok", measured, max_target, max_wrist,
           max_sim, max_elbow_step);
    return bad ? 1 : 0;
}
#endif
