/* Include inside DG_HOOK_TEST, after hook helpers and before main.
 * Scope: real hook position mapping + real arm mapper + world IK under an
 * explicit covariant camera/root fixture. This does NOT execute the bridge's
 * compensation/adjust publication or prove achieved engine/draw positions.
 */
#ifndef DG_POSITION_TURN_TEST_H
#define DG_POSITION_TURN_TEST_H
#include "dg_arm_map.h"
#include "dg_ik.h"

static void position_turn_rotate(double degrees, const double p[3], double o[3])
{
    double a = degrees * 3.14159265358979323846 / 180.0;
    double c = cos(a), s = sin(a);
    o[0] = c*p[0] + s*p[2]; o[1] = p[1]; o[2] = c*p[2] - s*p[0];
}

/* Companion bridge fixture callback. The caller supplies pair/stream IDs
 * after this call. At zero turn the independent shoulder-relative vector is
 * (0,0,500) mm. All hook state is restored before returning. */
static void position_turn_make_bridge_target(double software_rad,
                                             DG_BRIDGE_ARM_TARGET *out)
{
    DG_XR_CONFIG cfg;
    DG_XR_RAW_POSE head, hand;
    DG_XR_REL_POSE relative;
    double saved_sign[3], saved_shoulder[3], saved_q[4];
    double saved_omega=g_arm_frame_omega0, saved_turn=dg_xr_turn_offset_rad();
    LONG saved_source=g_source;
    int saved_frame=g_arm_frame, saved_have=g_arm_frame_have;
    int saved_absolute=g_arm_absolute_aim, k;
    for(k=0;k<3;k++) { saved_sign[k]=g_arm_pos_sign[k]; saved_shoulder[k]=g_arm_shoulder_mm[k]; }
    for(k=0;k<4;k++) { saved_q[k]=g_arm_frame_q[k]; g_arm_frame_q[k]=(k==3)?1.0:0.0; }
    g_source=SRC_XR; g_arm_absolute_aim=1;
    g_arm_frame=DG_XR_REL_FRAME_ROOM; g_arm_frame_have=1; g_arm_frame_omega0=0;
    g_arm_pos_sign[0]=-1; g_arm_pos_sign[1]=g_arm_pos_sign[2]=1;
    g_arm_shoulder_mm[0]=180; g_arm_shoulder_mm[1]=200; g_arm_shoulder_mm[2]=40;
    memset(&cfg,0,sizeof cfg); cfg.scale=1000;
    cfg.x_sign=cfg.y_sign=cfg.z_sign=1; cfg.yaw_sign=cfg.pitch_sign=cfg.roll_sign=1;
    memset(&head,0,sizeof head); head.qw=1; head.px=.3; head.py=1.6; head.pz=-.2;
    hand=head; hand.px+=.18; hand.py-=.20; hand.pz-=.54;
    dg_xr_test_set_turn_offset(software_rad);
    memset(out,0,sizeof *out);
    arm_hand_to_view(&head,&hand,&cfg,&relative,out->wrist_view);
    arm_player_shoulder_view(&head,&cfg,1,out->player_shoulder_view);
    out->write=1; out->weight=1; out->player_reach_view=600;
    out->head_yaw_valid=out->stick_yaw_valid=1;
    out->head_yaw_rad=out->stick_yaw_rad=software_rad;
    out->hand_quat[3]=1;
    dg_xr_test_set_turn_offset(saved_turn);
    g_source=saved_source; g_arm_absolute_aim=saved_absolute;
    g_arm_frame=saved_frame; g_arm_frame_have=saved_have; g_arm_frame_omega0=saved_omega;
    for(k=0;k<4;k++) g_arm_frame_q[k]=saved_q[k];
    for(k=0;k<3;k++) { g_arm_pos_sign[k]=saved_sign[k]; g_arm_shoulder_mm[k]=saved_shoulder[k]; }
}

/* Independent raw XR fixtures. Each local position is specified in metres,
 * +X right/+Y up/-Z forward. Repeat after three physical heading calibrations
 * and nonzero software origins; expected game-view values are literal axes,
 * not obtained by calling the production quaternion/frame helpers. */
static int test_position_range_recalibration(void)
{
    static const double poses[][3] = {
        {.18,-.20,-.50}, {.08,-.18,-.16}, {.25,-.55,-.10},
        {.18,-.20,-.80}, {-.25,-.20,-.35}, {.18,.35,-.25},
        {.18,-.65,-.35}
    };
    static const double headings[] = {0,67,-123};
    static const double origins[] = {0,.7,-1.3};
    static const double turns[] = {0,30,90,-90,0};
    double saved_q[4], saved_sign[3], saved_shoulder[3];
    double saved_omega=g_arm_frame_omega0, saved_turn=dg_xr_turn_offset_rad();
    LONG saved_source=g_source;
    int saved_absolute=g_arm_absolute_aim, saved_frame=g_arm_frame;
    int saved_have=g_arm_frame_have, h, p, t, k, cases=0, bad=0;
    long saved_freezes=g_arm_frame_freezes;
    double worst=0;
    DG_XR_CONFIG cfg;
    for(k=0;k<4;k++) saved_q[k]=g_arm_frame_q[k];
    for(k=0;k<3;k++) {
        saved_sign[k]=g_arm_pos_sign[k];
        saved_shoulder[k]=g_arm_shoulder_mm[k];
    }
    g_source=SRC_XR; g_arm_absolute_aim=1; g_arm_frame=DG_XR_REL_FRAME_ROOM;
    g_arm_pos_sign[0]=-1; g_arm_pos_sign[1]=g_arm_pos_sign[2]=1;
    g_arm_shoulder_mm[0]=180; g_arm_shoulder_mm[1]=200; g_arm_shoulder_mm[2]=40;
    memset(&cfg,0,sizeof cfg); cfg.scale=1000;
    cfg.x_sign=cfg.y_sign=cfg.z_sign=1;
    cfg.yaw_sign=cfg.pitch_sign=cfg.roll_sign=1;
    for(h=0;h<3;h++) {
        DG_XR_RAW_POSE head;
        double a=headings[h]*3.14159265358979323846/180.0;
        double c=cos(a), s=sin(a);
        memset(&head,0,sizeof head);
        head.qy=sin(a/2); head.qw=cos(a/2);
        head.px=.4+h*.3; head.py=1.6-h*.1; head.pz=-.7+h*.2;
        /* A new calibration replaces the preceding heading and omega. */
        dg_xr_test_set_turn_offset(origins[h]);
        arm_frame_freeze(&head);
        for(p=0;p<7;p++) for(t=0;t<5;t++) {
            DG_XR_RAW_POSE hand=head;
            DG_XR_REL_POSE relative;
            double wrist[3], shoulder[3];
            double expected[3]={-1000*poses[p][0],1000*poses[p][1],-1000*poses[p][2]};
            static const double expected_shoulder[3]={-180,-200,40};
            hand.px+=c*poses[p][0]+s*poses[p][2];
            hand.py+=poses[p][1];
            hand.pz+=c*poses[p][2]-s*poses[p][0];
            dg_xr_test_set_turn_offset(origins[h]+turns[t]*3.14159265358979323846/180.0);
            arm_hand_to_view(&head,&hand,&cfg,&relative,wrist);
            arm_player_shoulder_view(&head,&cfg,1,shoulder);
            for(k=0;k<3;k++) {
                double error=fabs(wrist[k]-expected[k]);
                if(error>worst) worst=error;
                if(!isfinite(wrist[k]) || error>1e-6 ||
                   !isfinite(shoulder[k]) || fabs(shoulder[k]-expected_shoulder[k])>1e-6) bad++;
            }
            cases++;
        }
    }
    dg_xr_test_set_turn_offset(saved_turn);
    g_source=saved_source; g_arm_absolute_aim=saved_absolute;
    g_arm_frame=saved_frame; g_arm_frame_have=saved_have;
    g_arm_frame_omega0=saved_omega; g_arm_frame_freezes=saved_freezes;
    for(k=0;k<4;k++) g_arm_frame_q[k]=saved_q[k];
    for(k=0;k<3;k++) { g_arm_pos_sign[k]=saved_sign[k]; g_arm_shoulder_mm[k]=saved_shoulder[k]; }
    printf("  %s position range/recalibration: %d raw poses, seven reaches, three replaced headings/software origins, five turns; %.9f mm worst (%d failures; input mapping only)\n",
           bad?"FAIL":"ok",cases,worst,bad);
    return bad?1:0;
}

static int test_position_turn_covariance(void)
{
    static const double anchors[] = {0,30,90,0};
    static const double shoulder[3] = {-150,-180,30};
    static const double want_hand[3] = {-220,-100,420};
    static const double want_player_shoulder[3] = {-180,-200,40};
    static const double pole[3] = {0,-1,0};
    double want_target[3], want_elbow[3], unit[3], bend[3];
    double saved_sign[3], saved_shoulder[3], saved_q[4];
    double saved_omega = g_arm_frame_omega0;
    double saved_turn = dg_xr_turn_offset_rad();
    LONG saved_source = g_source;
    int saved_frame = g_arm_frame, saved_have = g_arm_frame_have;
    int saved_absolute = g_arm_absolute_aim;
    long saved_freezes = g_arm_frame_freezes;
    DG_XR_CONFIG cfg;
    DG_XR_RAW_POSE head, hand;
    DG_XR_REL_POSE relative;
    DG_ARM_MAP_STATE state;
    double scale = .99*550.0/600.0, length = 0, dot, n, along, height;
    double worst = 0;
    int i, k, bad = 0, input_bad = 0, wrist_bad = 0, elbow_bad = 0;

    for(k=0;k<3;k++) {
        saved_sign[k]=g_arm_pos_sign[k]; saved_shoulder[k]=g_arm_shoulder_mm[k];
        want_target[k]=shoulder[k]+scale*(want_hand[k]-want_player_shoulder[k]);
        unit[k]=want_target[k]-shoulder[k]; length+=unit[k]*unit[k];
    }
    /* Independent triangle oracle, fixed baseline and downward bend branch.
       No production transform, mapper result or IK result defines expected. */
    length=sqrt(length);
    for(k=0;k<3;k++) unit[k]/=length;
    dot=-unit[1]; n=0;
    for(k=0;k<3;k++) { bend[k]=pole[k]-dot*unit[k]; n+=bend[k]*bend[k]; }
    n=sqrt(n); along=(300.0*300.0-250.0*250.0+length*length)/(2*length);
    height=sqrt(300.0*300.0-along*along);
    for(k=0;k<3;k++) want_elbow[k]=shoulder[k]+along*unit[k]+height*bend[k]/n;
    for(k=0;k<4;k++) saved_q[k]=g_arm_frame_q[k];
    g_source=SRC_XR; g_arm_absolute_aim=1; g_arm_frame=DG_XR_REL_FRAME_ROOM;
    g_arm_pos_sign[0]=-1; g_arm_pos_sign[1]=g_arm_pos_sign[2]=1;
    g_arm_shoulder_mm[0]=180; g_arm_shoulder_mm[1]=200; g_arm_shoulder_mm[2]=40;
    memset(&cfg,0,sizeof cfg);
    cfg.scale=1000; cfg.x_sign=cfg.y_sign=cfg.z_sign=1;
    cfg.yaw_sign=cfg.pitch_sign=cfg.roll_sign=1;
    memset(&head,0,sizeof head); head.qw=1; head.px=.3; head.py=1.6; head.pz=-.2;
    hand=head; hand.px+=.22; hand.py-=.10; hand.pz-=.42;
    dg_xr_test_set_turn_offset(0); arm_frame_freeze(&head);
    dg_arm_map_reset(&state);

    /* Named anchors first, then 1-degree out/back to include onset, reversal
       and return without re-zeroing state. Camera and root both yaw here. */
    for(i=0;i<185;i++) {
        double yaw=i<4 ? anchors[i] : (i<95 ? i-4 : 184-i);
        double wrist_view[3], sh_view[3], camera_wrist[3], camera_elbow[3];
        DG_ARM_MAP_IN mi; DG_ARM_MAP_OUT mo;
        DG_IK_IN ii; DG_IK_OUT io;
        dg_xr_test_set_turn_offset(yaw*3.14159265358979323846/180.0);
        arm_hand_to_view(&head,&hand,&cfg,&relative,wrist_view);
        arm_player_shoulder_view(&head,&cfg,1,sh_view);
        memset(&mi,0,sizeof mi);
        mi.anchor_shoulder=1; mi.player_reach=600; mi.upper=300; mi.fore=250;
        for(k=0;k<3;k++) {
            mi.controller_view[k]=wrist_view[k]; mi.player_shoulder[k]=sh_view[k];
            mi.shoulder_view[k]=shoulder[k]; mi.native_wrist_view[k]=want_target[k];
            if(fabs(wrist_view[k]-want_hand[k])>1e-6 ||
               fabs(sh_view[k]-want_player_shoulder[k])>1e-6) { bad++; input_bad++; }
        }
        if(i==0) {
            if(dg_arm_map_step(&state,&mi,&mo) ||
               !(mo.flags&DG_ARM_MAP_F_NOT_READY)) bad++;
        }
        if(!dg_arm_map_step(&state,&mi,&mo)) { bad++; continue; }
        memset(&ii,0,sizeof ii);
        position_turn_rotate(yaw,shoulder,ii.shoulder);
        position_turn_rotate(yaw,mo.target_view,ii.target);
        position_turn_rotate(yaw,pole,ii.pole);
        /* dg_ik_solve consumes a world POINT (subtracts shoulder), whereas
           pole above and the independent oracle specify a bend DIRECTION. */
        for(k=0;k<3;k++) ii.pole[k]+=ii.shoulder[k];
        ii.upper=300; ii.fore=250; ii.max_reach_frac=.99;
        ii.min_elbow_deg=15; ii.max_elbow_deg=175;
        if(!dg_ik_solve(&ii,&io)) { bad++; continue; }
        if(io.clamped || mo.flags) bad++;
        position_turn_rotate(-yaw,io.wrist,camera_wrist);
        position_turn_rotate(-yaw,io.elbow,camera_elbow);
        for(k=0;k<3;k++) {
            double error=fabs(camera_wrist[k]-want_target[k]);
            if(error>worst) worst=error;
            if(error>1e-5) { bad++; wrist_bad++; }
            if(fabs(camera_elbow[k]-want_elbow[k])>1e-5) { bad++; elbow_bad++; }
        }
    }
    dg_xr_test_set_turn_offset(saved_turn);
    g_source=saved_source; g_arm_absolute_aim=saved_absolute;
    g_arm_frame=saved_frame; g_arm_frame_have=saved_have;
    g_arm_frame_omega0=saved_omega; g_arm_frame_freezes=saved_freezes;
    for(k=0;k<4;k++) g_arm_frame_q[k]=saved_q[k];
    for(k=0;k<3;k++) {
        g_arm_pos_sign[k]=saved_sign[k]; g_arm_shoulder_mm[k]=saved_shoulder[k];
    }
    printf("  %s position turn covariance: raw controller fixed, 0/30/90/0 and 1-degree out/back, target+elbow camera invariance; %.6f mm worst component (%d failures; no engine claim)\n",
           bad?"FAIL":"ok",worst,bad);
    if(bad) printf("    position failures: input=%d wrist=%d elbow=%d other=%d\n",
                  input_bad,wrist_bad,elbow_bad,bad-input_bad-wrist_bad-elbow_bad);
    return (bad ? 1 : 0) + test_position_range_recalibration();
}
#endif
