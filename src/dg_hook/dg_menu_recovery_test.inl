static int t_xr_menu_recovery(void)
{
    DG_ANCHORS saved=g_b.a;LONG armed=g_b.armed,mode=g_b.menu_mode,script=g_b.script_menu_only;
    DWORD rec[16]={0},game=0x240u,menu=0;uint64_t arm=0;
    DG_BRIDGE_MENU cmd={0};int bad=0;
    DWORD *st=(DWORD *)((unsigned char *)rec+DG_GV_PAD_STATUS_OFFSET);
    DWORD *pr=(DWORD *)((unsigned char *)rec+DG_GV_PAD_PRESS_OFFSET);
    memset(&g_b.a,0,sizeof g_b.a);g_b.armed=1;g_b.menu_mode=2;g_b.script_menu_only=0;
    g_b.a.gv_pad_data_direct=(uint64_t)rec;
    g_b.a.gm_menu_status=g_b.a.gm_menu_status_scn=(uint64_t)&menu;
    g_b.a.gm_game_status=g_b.a.gm_game_status_scn=(uint64_t)&game;
    g_b.a.gm_player_arm_body=(uint64_t)&arm;
    cmd.allow=DG_MENU_ALLOW_XR_CONFIRM;cmd.status=0x00080040u;
    dg_bridge_menu_now(&cmd);pad_seam_tick();
    if(!(*pr&DG_PAD_TITLE_ENTER) || *st!=*pr || (*pr&DG_PAD_START))bad++;
    *st=*pr=0;pad_seam_tick();if(*st || *pr)bad++; /* once only */
    /* Live title backdrop is CUT_IN|PAUSE_DISABLE (session.log:175), not
       zero. Startup/demo and an arm-owned cut-in must never gain ENTER. */
    game=0x10000040u;dg_bridge_menu_now(&cmd);pad_seam_tick();
    if(*pr&DG_PAD_TITLE_ENTER)bad++;
    *st=*pr=0;game=0x240u;arm=0x12345;dg_bridge_menu_now(&cmd);pad_seam_tick();
    if(*pr&DG_PAD_TITLE_ENTER)bad++;
    *st=*pr=0;arm=0;dg_bridge_menu_now(&cmd);arm=0x12345;game=0;pad_seam_tick();
    if(*pr || *st)bad++; /* a pending title confirm cannot enter gameplay */
    arm=0;game=0;dg_bridge_menu_now(&cmd);pad_seam_tick();
    if(!(*pr&DG_PAD_TITLE_ENTER))bad++; /* ordinary no-arm frontend still works */
    *st=*pr=0;
    arm=0x12345;game=0x80004000u;dg_bridge_menu_now(&cmd);pad_seam_tick();
    if(!dg_bridge_menu_gameover_now())bad++;
    game|=0x08000000u;if(dg_bridge_menu_gameover_now())bad++;game=0x80004000u;
    menu=0x400u;if(dg_bridge_menu_gameover_now())bad++;menu=0;
    if(!(*pr&DG_PAD_START) || !(*pr&0x40u) || (*pr&DG_PAD_TITLE_ENTER))bad++;
    *st=*pr=0;dg_bridge_menu_now(&cmd);g_script_menu_deadline=GetTickCount64();pad_seam_tick();
    if(*st || *pr)bad++; /* expires even when c_ticks does not advance */
    dg_bridge_menu_now(&cmd);game=0;pad_seam_tick();if(*st || *pr)bad++;
    game=0x80004000u; /* a queued Continue never leaks into resumed gameplay */
    dg_bridge_menu_now(&cmd);dg_bridge_menu_now(NULL);pad_seam_tick();if(*st || *pr)bad++;
    menu=0x100u;dg_bridge_menu_now(&cmd);pad_seam_tick();if(*st || *pr)bad++;
    menu=0x200u;dg_bridge_menu_now(&cmd);pad_seam_tick();if(*st || *pr)bad++;
    menu=0;*st=*pr=DG_MENU_PAD_SEL;dg_bridge_menu_now(&cmd);pad_seam_tick();
    if(*st!=DG_MENU_PAD_SEL || *pr!=DG_MENU_PAD_SEL)bad++;
    *st=*pr=0;game=0;cmd.allow=DG_MENU_ALLOW_XR;cmd.status=DG_MENU_PAD_D;
    dg_bridge_menu_now(&cmd);pad_seam_tick();if(*pr!=DG_MENU_PAD_D)bad++;
    script_menu_clear(0);g_b.a=saved;g_b.armed=armed;g_b.menu_mode=mode;g_b.script_menu_only=script;
    printf("  %-6s XR menu recovery: title ENTER, Continue START, single consume, expiry, cancellation and panel gates\n",bad?"FAIL":"ok");
    return bad;
}
