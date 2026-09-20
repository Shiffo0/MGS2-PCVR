/* Real bridge admission against readable native-layout fixtures. The pure
   settling timer is tested in dg_hook.c; no game process is involved here. */
static int t_view_calibration_gate(void)
{
    void *saved=malloc(sizeof g_b);
    dg_radial_inventory_image saved_image=g_interact_image;
    dg_radial_inventory_anchors saved_player=g_interact_player;
    unsigned char actor[0x400]={0},camera[0x40]={0},player[0xE00]={0};
    ULONGLONG arm=(ULONGLONG)(ULONG_PTR)(actor+0x60);
    uint64_t player_ptr=(uint64_t)(ULONG_PTR)player;
    volatile ULONGLONG status=0;
    volatile LONG native=1,game=0,scenario_game=0,menu=0,scenario_menu=0;
    long generation;
    unsigned long long identity;
    int bad=0,i;
#define VC_CHECK(x) do {if(!(x)){bad++;printf("  FAIL view calibration line %d: %s\n",__LINE__,#x);}}while(0)
    if(!saved)return 1;
    memcpy(saved,&g_b,sizeof g_b);memset(&g_b,0,sizeof g_b);
    memset(&g_interact_image,0,sizeof g_interact_image);
    memset(&g_interact_player,0,sizeof g_interact_player);
    g_interact_image.base=1;g_interact_image.size=1;
    g_interact_image.approved_identity=1;g_interact_image.read=interact_read;
    g_interact_player.module_base=1;g_interact_player.module_size=1;
    g_interact_player.valid_bits=DG_RINV_ACTOR;
    g_interact_player.player_slot=(uint64_t)(ULONG_PTR)&player_ptr;
    g_interact_player.actor_weapon_offset=0xB90;
    g_interact_player.actor_item_offset=0xB94;
    *(ULONGLONG *)(actor+0x248)=(ULONGLONG)(ULONG_PTR)camera;
    *(LONG *)(camera+0x2c)=1;
    g_b.a.gbp_active=(ULONGLONG)(ULONG_PTR)&native;
    g_b.a.gm_player_status=(ULONGLONG)(ULONG_PTR)&status;
    g_b.a.gm_game_status=(ULONGLONG)(ULONG_PTR)&game;
    g_b.a.gm_game_status_scn=(ULONGLONG)(ULONG_PTR)&scenario_game;
    g_b.a.gm_menu_status=(ULONGLONG)(ULONG_PTR)&menu;
    g_b.a.gm_menu_status_scn=(ULONGLONG)(ULONG_PTR)&scenario_menu;
    g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&arm;
    g_b.armed=1;g_b.fps.state=DG_FPS_ACTIVE;g_b.fps.desired=1;
    g_b.calibration_ready_arm=(LONG64)arm;g_b.c_explicit_entries=7;
    VC_CHECK(dg_bridge_view_calibration_now(&generation,&identity)==1);
    VC_CHECK(generation==7 && identity==arm); /* None requires no weapon object */
    for(i=0;i<11;i++) {
        switch(i) {
        case 0:g_b.calibration_ready_arm=0;break;
        case 1:g_b.calibration_ready_arm=(LONG64)arm+16;break;
        case 2:native=0;break;
        case 3:*(LONG *)(camera+0x2c)=0;break;
        case 4:g_b.fps.state=DG_FPS_REQUEST_ENTER;break;
        case 5:g_b.fps.state=DG_FPS_REQUEST_LEAVE;break;
        case 6:g_b.owner=1;break;
        case 7:g_b.script_menu_only=1;break;
        case 8:menu=DG_MENU_UNSAFE_MASK;break;
        case 9:scenario_game=DG_GAME_UNSAFE_MASK;break;
        case 10:status=DG_PLAYER_UNSAFE_MASK;break;
        }
        generation=99;identity=99;
        VC_CHECK(!dg_bridge_view_calibration_now(&generation,&identity));
        VC_CHECK(!generation && !identity);
        g_b.calibration_ready_arm=(LONG64)arm;native=1;
        *(LONG *)(camera+0x2c)=1;g_b.fps.state=DG_FPS_ACTIVE;
        g_b.owner=g_b.script_menu_only=0;menu=scenario_game=0;status=0;
    }
    /* Third person requires completed explicit leave and native OFF; an
       unsafe suspension or in-flight native leave cannot calibrate it. */
    g_b.fps.state=DG_FPS_OFF;g_b.fps.desired=0;native=0;
    VC_CHECK(!dg_bridge_view_calibration_now(&generation,&identity));
    g_b.c_left=3;
    VC_CHECK(dg_bridge_view_calibration_now(&generation,&identity)==2);
    VC_CHECK(generation==3 && identity==player_ptr);
    for(i=0;i<7;i++) {
        switch(i) {
        case 0:native=1;break;
        case 1:g_b.fps.state=DG_FPS_SUSPENDED;break;
        case 2:g_b.fps.desired=1;break;
        case 3:scenario_menu=DG_MENU_UNSAFE_MASK;break;
        case 4:game=DG_GAME_UNSAFE_MASK;break;
        case 5:player_ptr=0;break;
        case 6:g_b.armed=0;break;
        }
        VC_CHECK(!dg_bridge_view_calibration_now(&generation,&identity));
        VC_CHECK(!generation && !identity);
        native=0;g_b.fps.state=DG_FPS_OFF;g_b.fps.desired=0;
        scenario_menu=game=0;player_ptr=(uint64_t)(ULONG_PTR)player;g_b.armed=1;
    }
    memcpy(&g_b,saved,sizeof g_b);free(saved);
    g_interact_image=saved_image;g_interact_player=saved_player;
    printf("  %s view calibration: real bridge None/FPS, explicit leave/native OFF, readiness identity, unsafe and ownership gates\n",bad?"FAIL":"ok");
#undef VC_CHECK
    return bad!=0;
}
