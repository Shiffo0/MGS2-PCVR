static void ia_dummy_provider(void *u,int a,int f,uint64_t n,uint64_t t,
    DG_BRIDGE_CONTROLS_FRAME *o) { (void)u;(void)a;(void)f;(void)n;(void)t;(void)o; }
static int t_interact_native_writer(void) {
    union { ULONGLONG align; unsigned char b[0x40]; } padmem;
    union { ULONGLONG align; unsigned char b[0x300]; } actor;
    union { ULONGLONG align; unsigned char b[0xD20]; } player;
    DG_ANCHORS saved_anchors=g_b.a;
    DG_BRIDGE_CONTROLS_PROVIDER saved_provider=g_controls_provider;
    DG_BRIDGE_CONTROLS_FRAME saved_frame=g_controls_frame;
    DG_INTERACT_ADAPTER saved_adapter=g_interact_adapter;
    dg_radial_inventory_image saved_image=g_interact_image;
    dg_radial_inventory_anchors saved_player=g_interact_player;
    LONG saved_armed=g_b.armed,saved_menu=g_b.script_menu_only;
    LONG saved_late=g_b.s_late_unsafe,saved_tick=g_b.c_ticks;
    DG_FIRE saved_fire=g_b.fire;
    int saved_allowed=g_controls_allowed,saved_active=g_controls_active;
    int saved_ladder=g_controls_ladder;
    int saved_special=g_controls_special;
    LONG saved_late_samples=g_b.c_late_status;
    uint64_t saved_lease=g_controls_lease;
    LONG mask=8,index=4;
    ULONGLONG arm=(ULONGLONG)(ULONG_PTR)(actor.b+0x60);
    ULONGLONG pad=(ULONGLONG)(ULONG_PTR)(padmem.b+4);
    ULONGLONG player_slot=(ULONGLONG)(ULONG_PTR)player.b;
    ULONGLONG no_arm=0,player_status=0;
    DWORD game_status=0,menu_status=0;
    int checks=0,bad=0;
#define IA_CHECK(x) do { ++checks; if (!(x)) { ++bad; printf("FAIL native interaction %d: %s\n",__LINE__,#x); } } while(0)
#define IA_PASS(level,safe) do { memset(padmem.b+4,0,sizeof padmem.b-4); \
    ++g_b.c_ticks; ++g_controls_frame.interact.sample; \
    g_controls_frame.interact.levels=(level); \
    g_controls_lease=GetTickCount64(); interact_tick(safe); } while(0)
    memset(&padmem,0,sizeof padmem); memset(&actor,0,sizeof actor);
    memset(&player,0,sizeof player); memset(&g_interact_adapter,0,sizeof g_interact_adapter);
    *(LONG *)padmem.b=1;
    *(ULONGLONG *)(actor.b+0x228)=(ULONGLONG)(ULONG_PTR)(player.b+0xCF4);
    *(ULONGLONG *)(player.b+0xBA8)=arm; *(LONG *)(player.b+0xBB0)=6;
    g_b.a.player_pad=pad; g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&no_arm;
    g_b.a.gm_player_status=(ULONGLONG)(ULONG_PTR)&player_status;
    g_b.a.gm_game_status=g_b.a.gm_game_status_scn=(ULONGLONG)(ULONG_PTR)&game_status;
    g_b.a.gm_menu_status=g_b.a.gm_menu_status_scn=(ULONGLONG)(ULONG_PTR)&menu_status;
    memset(&g_interact_image,0,sizeof g_interact_image);
    memset(&g_interact_player,0,sizeof g_interact_player);
    g_interact_image.base=g_interact_player.module_base=1;
    g_interact_image.size=g_interact_player.module_size=1;
    g_interact_image.approved_identity=1;g_interact_image.read=interact_read;
    g_interact_player.valid_bits=DG_RINV_ACTOR;
    g_interact_player.player_slot=(uint64_t)(ULONG_PTR)&player_slot;
    g_interact_player.actor_weapon_offset=0xB90;
    g_interact_player.actor_item_offset=0xB94;
    g_b.a.pad_weapon=(ULONGLONG)(ULONG_PTR)&mask;
    g_b.a.pad_press_weapon=(ULONGLONG)(ULONG_PTR)&index;
    g_b.armed=1;g_b.script_menu_only=0;g_b.s_late_unsafe=0;
    g_b.c_ticks=200;g_b.fire.state=DG_FIRE_IDLE;
    IA_CHECK(dg_bridge_controller_gameplay_now());
    player_slot=0;IA_CHECK(!dg_bridge_controller_gameplay_now());
    player_slot=(ULONGLONG)(ULONG_PTR)player.b;
    g_controls_provider=ia_dummy_provider;g_controls_allowed=1;g_controls_active=1;
    memset(&g_controls_frame,0,sizeof g_controls_frame);
    g_controls_frame.interact.epoch=1;g_controls_frame.interact.valid=1;
    IA_PASS(0,1); IA_PASS(DG_IA_ACTION,1);
    IA_CHECK(RD32(pad+4)==0x10 && RD32(pad+8)==0x10);
    IA_PASS(DG_IA_ACTION,1); IA_CHECK(RD32(pad+4)==0x10 && !RD32(pad+8));
    IA_PASS(0,1); IA_CHECK(RD32(pad+12)==0x10);
    IA_PASS(DG_IA_CAPTURE,1);
    IA_CHECK(RD32(pad+4)==8 && RD32(pad+8)==8 && padmem.b[4+0x18+4]==255);
    g_controls_frame.interact.choke_seq=9;IA_PASS(DG_IA_CAPTURE|DG_IA_MELEE,1);
    IA_CHECK(RD32(pad+4)==8 && RD32(pad+8)==8);
    IA_PASS(DG_IA_CAPTURE,0); IA_CHECK(!RD32(pad+4) && !RD32(pad+12));
    IA_PASS(DG_IA_CAPTURE,1); IA_CHECK(!RD32(pad+4));
    IA_PASS(0,1); IA_PASS(DG_IA_POSTURE,1); IA_CHECK(RD32(pad+8)==0x40);
    g_b.s_late_unsafe=1; IA_PASS(DG_IA_POSTURE,1); IA_CHECK(!RD32(pad+4));
    g_b.s_late_unsafe=0; IA_PASS(0,1); *(LONG *)(player.b+0xB90)=1;
    IA_PASS(DG_IA_CAPTURE,1); IA_CHECK(!RD32(pad+4));
    *(LONG *)(player.b+0xB90)=0; IA_PASS(0,1); *(LONG *)padmem.b=0;
    IA_PASS(DG_IA_ACTION,1); IA_CHECK(!RD32(pad+4));
    *(LONG *)padmem.b=1; IA_PASS(0,1); IA_PASS(DG_IA_CODEC,1);
    IA_CHECK(g_interact_codec_pending!=0);
    g_controls_frame.interact.suppressed=DG_IA_CODEC;IA_PASS(DG_IA_CODEC,1);
    IA_CHECK(!g_interact_codec_pending);
    g_controls_frame.interact.suppressed=0;IA_PASS(0,1);IA_PASS(DG_IA_CODEC,1);
    IA_CHECK(g_interact_codec_pending!=0);
    ++g_controls_frame.interact.epoch;IA_PASS(0,1);IA_CHECK(!g_interact_codec_pending);
    IA_PASS(DG_IA_CODEC,1);IA_CHECK(g_interact_codec_pending!=0);
    --g_controls_frame.interact.sample;interact_tick(1);IA_CHECK(!g_interact_codec_pending);
    IA_PASS(0,1);IA_PASS(0,1);IA_PASS(DG_IA_CAPTURE,1);IA_CHECK(RD32(pad+4)==8);
    player_slot=0;IA_PASS(DG_IA_CAPTURE,1);IA_CHECK(!RD32(pad+4) && !RD32(pad+12));
    player_slot=(ULONGLONG)(ULONG_PTR)player.b;IA_PASS(DG_IA_CAPTURE,1);IA_CHECK(!RD32(pad+4));
    g_b.c_late_status=0;player_status=0x10000;g_controls_ladder=1;g_controls_allowed=0;
    IA_CHECK(dg_bridge_controller_ladder_now());
    g_controls_frame.interact.ladder=1;++g_controls_frame.interact.epoch;
    IA_PASS(0,0);IA_PASS(DG_IA_UP,0);IA_CHECK(RD32(pad+4)==0x1000);
    player_status|=1;IA_CHECK(!dg_bridge_controller_ladder_now());
    IA_PASS(DG_IA_UP,0);IA_CHECK(!RD32(pad+4));
    player_status=0x12000;IA_CHECK(!dg_bridge_controller_ladder_now());
    player_status=0x10000;game_status=0x10000000;IA_CHECK(!dg_bridge_controller_ladder_now());
    game_status=0;player_slot=0;IA_CHECK(!dg_bridge_controller_ladder_now());
    player_slot=(ULONGLONG)(ULONG_PTR)player.b;player_status=0x1000;
    IA_CHECK(dg_bridge_controller_special_now()==DG_CONTROLS_BEYOND);
    g_controls_special=DG_CONTROLS_BEYOND;g_controls_ladder=0;
    g_controls_frame.interact.ladder=0;g_controls_frame.interact.special=DG_CONTROLS_BEYOND;
    ++g_controls_frame.interact.epoch;IA_PASS(0,0);IA_PASS(DG_IA_PEEP_LEFT|DG_IA_PEEP_RIGHT,0);
    IA_CHECK(RD32(pad+4)==3 && padmem.b[4+0x18+10]==255 && padmem.b[4+0x18+11]==255);
    player_status|=0x2000;IA_PASS(DG_IA_PEEP_LEFT,0);IA_CHECK(!RD32(pad+4));
    player_status=0x81;IA_CHECK(dg_bridge_controller_special_now()==DG_CONTROLS_LOCKER);
    g_controls_special=DG_CONTROLS_LOCKER;g_controls_frame.interact.special=DG_CONTROLS_LOCKER;
    ++g_controls_frame.interact.epoch;IA_PASS(0,0);IA_PASS(DG_IA_MELEE,0);IA_CHECK(RD32(pad+8)==0x20);
    player_status|=0x2000;IA_CHECK(!dg_bridge_controller_special_now());
    g_b.a=saved_anchors;g_b.armed=saved_armed;g_b.script_menu_only=saved_menu;
    g_b.s_late_unsafe=saved_late;g_b.c_ticks=saved_tick;g_b.fire=saved_fire;
    g_controls_provider=saved_provider;g_controls_frame=saved_frame;
    g_interact_adapter=saved_adapter;g_controls_allowed=saved_allowed;
    g_controls_active=saved_active;g_controls_lease=saved_lease;
    g_interact_image=saved_image;g_interact_player=saved_player;
    g_controls_ladder=saved_ladder;g_b.c_late_status=saved_late_samples;
    g_controls_special=saved_special;
    printf("native interaction writer: %d checks, %d failures\n",checks,bad);
#undef IA_PASS
#undef IA_CHECK
    return bad?1:0;
}

static int t_interact_codec_direct(void) {
    DG_ANCHORS saved=g_b.a;
    DG_BRIDGE_CONTROLS_PROVIDER provider=g_controls_provider;
    LONG armed=g_b.armed,script=g_b.script_menu_only,late=g_b.s_late_unsafe;
    LONG64 pending=g_interact_codec_pending;
    uint64_t lease=g_controls_lease;
    int allowed=g_controls_allowed;
    int special=g_controls_special,ladder=g_controls_ladder;
    ULONGLONG player=0;
    DWORD game=0,menu=0,scn=0,direct[10]={0};
    int checks=0,bad=0;
#define CODEC_CHECK(x) do { ++checks; if (!(x)) { ++bad; printf("FAIL codec direct %d: %s\n",__LINE__,#x); } } while(0)
#define CODEC_QUEUE() do { g_controls_lease=GetTickCount64(); \
    g_interact_codec_pending=(LONG64)g_controls_lease; } while(0)
    g_b.a.gm_player_status=(ULONGLONG)(ULONG_PTR)&player;
    g_b.a.gm_game_status=(ULONGLONG)(ULONG_PTR)&game;
    g_b.a.gm_game_status_scn=(ULONGLONG)(ULONG_PTR)&scn;
    g_b.a.gm_menu_status=(ULONGLONG)(ULONG_PTR)&menu;
    g_b.a.gm_menu_status_scn=(ULONGLONG)(ULONG_PTR)&scn;
    g_b.a.gv_pad_data_direct=(ULONGLONG)(ULONG_PTR)direct;
    g_b.armed=1;g_b.script_menu_only=0;g_b.s_late_unsafe=0;
    g_controls_provider=ia_dummy_provider;g_controls_allowed=1;
    CODEC_CHECK(!dg_bridge_codec_input_now());
    menu=0x400;CODEC_CHECK(dg_bridge_codec_input_now());
    menu=0x500;CODEC_CHECK(!dg_bridge_codec_input_now());
    menu=0x400;g_b.script_menu_only=1;CODEC_CHECK(!dg_bridge_codec_input_now());
    g_b.script_menu_only=0;g_b.armed=0;CODEC_CHECK(!dg_bridge_codec_input_now());
    g_b.armed=1;menu=0;
    CODEC_QUEUE();dg_bridge_controls_context_special(0,DG_CONTROLS_BEYOND);
    CODEC_CHECK(!g_interact_codec_pending);
    g_controls_allowed=1;
    direct[1]=0x10;CODEC_QUEUE();interact_codec_seam();
    CODEC_CHECK(direct[1]==(0x10|DG_MENU_PAD_SEL) && direct[2]==DG_MENU_PAD_SEL);
    memset(direct,0,sizeof direct);interact_codec_seam();CODEC_CHECK(!direct[1]);
    CODEC_QUEUE();g_interact_codec_pending-=101;interact_codec_seam();CODEC_CHECK(!direct[1]);
    CODEC_QUEUE();menu=0x400;interact_codec_seam();CODEC_CHECK(!direct[1]);menu=0;
    CODEC_QUEUE();game=0x10000000;interact_codec_seam();CODEC_CHECK(!direct[1]);game=0;
    CODEC_QUEUE();g_controls_allowed=0;interact_codec_seam();CODEC_CHECK(!direct[1]);g_controls_allowed=1;
    CODEC_QUEUE();direct[1]=DG_PAD_START;interact_codec_seam();CODEC_CHECK(direct[1]==DG_PAD_START);
    memset(direct,0,sizeof direct);CODEC_QUEUE();g_b.s_late_unsafe=1;
    interact_codec_seam();CODEC_CHECK(!direct[1]);
    g_b.a=saved;g_b.armed=armed;g_b.script_menu_only=script;g_b.s_late_unsafe=late;
    g_controls_provider=provider;g_controls_allowed=allowed;g_controls_lease=lease;
    g_controls_special=special;g_controls_ladder=ladder;
    g_interact_codec_pending=pending;
    printf("codec direct writer: %d checks, %d failures\n",checks,bad);
#undef CODEC_QUEUE
#undef CODEC_CHECK
    return bad?1:0;
}
