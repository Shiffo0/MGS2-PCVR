/* Exercises release entry points with deliberately enabled legacy diagnostics. */
#if DG_ENABLE_DIAGNOSTICS
#error This test requires the release profile
#endif
static int test_release_profile(void) {
    const char *keys[]={"vr_rec","vr_rec_dump","vr_eye_dump","vr_aim_probe",
        "vr_geometry_probe","vr_hand_probe","vr_work_probe","vr_skel_probe",
        "vr_adjust_probe","vr_turn_probe","vr_grip_debug","vr_eye_truth"};
    unsigned i,upper; int bad=0,checks=0;
    for (upper=0;upper<2;upper++) for(i=0;i<sizeof(keys)/sizeof(keys[0]);i++) {
        char text[512],key[64]; POSE p; double seconds=3600;
        int source,track,map[4],hand,ok; unsigned j;
        DG_XR_CONFIG xc; DG_BRIDGE_CONFIG bc; ARM_POS_CFG ap;
        strcpy_s(key,sizeof key,keys[i]);
        if(upper)for(j=0;key[j];j++)if(key[j]>='a'&&key[j]<='z')key[j]-='a'-'A';
        sprintf_s(text,sizeof text,"source=xr\n%s=on\nvr_m9_slide=on\nvr_hf_blade=on\nvr_move=on\n",key);
        ok=parse_config(text,&p,&seconds,&source,&xc,&bc,&track,map,&hand,&ap);
        checks++;
        if(!ok || source!=SRC_XR || !bc.m9_slide || !bc.hf_blade ||
           g_rec_cfg_on || g_rec_cfg_dump || g_eye_dump_cfg ||
           xc.grip_debug || bc.hand_probe || bc.work_probe || bc.skel_probe ||
           bc.adjust_probe || bc.turn_probe || dg_aim_capture_enabled() ||
           dg_aim_capture_native_enabled()) bad++;
    }
    dg_aim_capture_configure(1);dg_aim_capture_native_configure(1);
    checks++;if(dg_aim_capture_enabled()||dg_aim_capture_native_enabled())bad++;
    /* Invalid pointers are safe here only because the release entry points
       must return before opening files, touching GPU objects or recording. */
    dg_draw_trial_poll((const char*)1);
    dg_present_poll_state_probe((const char*)1);
    dg_cb_probe_camera(0,(const float*)1,(const float*)1,(const float*)1,(const float*)1);
    dg_aim_capture_observe(0,0,NULL,NULL,NULL,0,NULL,0);
    dg_aim_capture_native_observe(0,NULL,NULL);
    checks++;if(dg_aim_capture_dump((const char*)1)!=0)bad++;
    checks++;if(dg_rec_capture(NULL,NULL)!=0 || dg_rec_write(NULL,0,NULL,NULL)!=0)bad++;
    checks++;if(rec_dump("release test")!=0 || rec_write_atomic((const char*)1,NULL)!=0)bad++;
    pp_prepare(1,(const char*)1,(const char*)1);
    checks++;if(pp_poll(NULL)!=0 || pp_return_context(NULL)!=0 || hud_context(NULL)!=0)bad++;
    phase_poll();hud_poll();eye_dump_step((IDXGISwapChain*)1,0);
    checks++;if(DG_REC_CAP!=1 || PP_CAPACITY!=1 || PHASE_CAP!=1)bad++;
    printf("%s release profile: %d checks, %d failures; legacy settings ignored, gameplay settings preserved, diagnostic entry points inert\n",bad?"FAIL":"PASS",checks,bad);
    return bad?1:0;
}
