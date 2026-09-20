static int test_menu_gameover_producer(void)
{
    DG_XR_FRAME frame={0};DG_MENU_IN in={0};DG_MENU_OUT out;DG_MENU_STATE state={0};
    LONG hand=g_menu_hand_right;int bad=0;unsigned status;
    g_menu_hand_right=1;frame.right_hand.grip.active=1;
    in.deadzone=.5;in.trigger_click=.55;in.confirm_bit=0x80040u;in.cancel_bit=0x40020u;
    /* Live camera handoffs continue: flat=0. Explicit game-over admission
       must still reach the actual XR hand extraction and edge producer. */
    in.front_end=menu_frontend_allowed(0,0,1,0);
    menu_input_from_frame(&in,&frame);dg_menu_step(&state,&in,&out);
    if(!in.front_end || out.write)bad++;
    frame.right_hand.primary_button=1;menu_input_from_frame(&in,&frame);
    dg_menu_step(&state,&in,&out);
    status=dg_menu_native_confirm(out.status,(out.flags&DG_MENU_F_CONFIRM)!=0,0,1);
    if(!out.write || !(out.flags&DG_MENU_F_CONFIRM) || !(status&DG_MENU_PAD_STA))bad++;
    dg_menu_step(&state,&in,&out);if(out.write)bad++;
    frame.right_hand.primary_button=0;menu_input_from_frame(&in,&frame);dg_menu_step(&state,&in,&out);
    frame.right_hand.primary_button=1;frame.right_hand.grip.pose_age_ms=101;
    menu_input_from_frame(&in,&frame);dg_menu_step(&state,&in,&out);if(out.write)bad++;
    if(menu_frontend_allowed(0,0,0,0) || menu_frontend_allowed(0,0,1,1))bad++;
    g_menu_hand_right=hand;
    printf("  %-6s XR game-over producer: active camera, A edge, held A, stale sample, Home capture\n",bad?"FAIL":"ok");
    return bad;
}
