static int t_hanging_visibility(void)
{
    union { ULONGLONG align; unsigned char b[0x80]; } arm,objs,evm,other;
    DG_ANCHORS anchors=g_b.a;
    DG_HANGING_VISIBILITY saved=g_hanging_visibility;
    LONG armed=g_b.armed,active=1;
    ULONGLONG arm_slot=(ULONGLONG)(ULONG_PTR)arm.b,player=0x1000;
    LONG game=0,menu=0;
    int bad=0,checks=0;
#define HV_CHECK(x) do { ++checks;if(!(x)){++bad;printf("FAIL hanging visibility %d: %s\n",__LINE__,#x);} }while(0)
    memset(&arm,0,sizeof arm);memset(&objs,0,sizeof objs);
    memset(&evm,0,sizeof evm);memset(&other,0,sizeof other);
    memset(&g_hanging_visibility,0,sizeof g_hanging_visibility);
    *(ULONGLONG *)arm.b=(ULONGLONG)(ULONG_PTR)objs.b;
    *(ULONGLONG *)(arm.b+0x30)=(ULONGLONG)(ULONG_PTR)evm.b;
    g_b.a.gm_player_arm_body=(ULONGLONG)(ULONG_PTR)&arm_slot;
    g_b.a.gm_player_status=(ULONGLONG)(ULONG_PTR)&player;
    g_b.a.gbp_active=(ULONGLONG)(ULONG_PTR)&active;
    g_b.a.gm_game_status=g_b.a.gm_game_status_scn=(ULONGLONG)(ULONG_PTR)&game;
    g_b.a.gm_menu_status=g_b.a.gm_menu_status_scn=(ULONGLONG)(ULONG_PTR)&menu;
    g_b.armed=1;
    *(LONG *)(objs.b+0x58)=0x22;*(LONG *)(evm.b+0x58)=0x44;
    hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==0x1022 && *(LONG *)(evm.b+0x58)==0x144);
    /* Native pose refresh may clear flags: enforce both channels. */
    *(LONG *)(objs.b+0x58)=0x26;*(LONG *)(evm.b+0x58)=0x48;
    hanging_visibility_now();player=0;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==0x26 && *(LONG *)(evm.b+0x58)==0x48);
    /* Pre-existing hidden bits belong to native code, even on exit. */
    player=0x1000;*(LONG *)(objs.b+0x58)=0x1001;*(LONG *)(evm.b+0x58)=0x101;
    hanging_visibility_now();player=0;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==0x1001 && *(LONG *)(evm.b+0x58)==0x101);
    /* Replacement never writes the old allocation on exit. */
    player=0x1000;*(LONG *)(objs.b+0x58)=1;*(LONG *)(evm.b+0x58)=1;
    hanging_visibility_now();arm_slot=(ULONGLONG)(ULONG_PTR)other.b;
    hanging_visibility_now();
    HV_CHECK(!g_hanging_visibility.arm && *(LONG *)(objs.b+0x58)==0x1001);
    arm_slot=(ULONGLONG)(ULONG_PTR)arm.b;
    *(LONG *)(objs.b+0x58)=1;*(LONG *)(evm.b+0x58)=1;
    hanging_visibility_now();
    *(ULONGLONG *)arm.b=(ULONGLONG)(ULONG_PTR)other.b;
    player=0;hanging_visibility_now();
    HV_CHECK(!g_hanging_visibility.arm && *(LONG *)(objs.b+0x58)==0x1001 &&
        *(LONG *)(other.b+0x58)==0 && *(LONG *)(evm.b+0x58)==0x101);
    *(ULONGLONG *)arm.b=(ULONGLONG)(ULONG_PTR)objs.b;player=0x1000;
    arm_slot=(ULONGLONG)(ULONG_PTR)arm.b;
    *(LONG *)(objs.b+0x58)=3;*(LONG *)(evm.b+0x58)=5;
    active=0;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==3 && *(LONG *)(evm.b+0x58)==5);
    active=1;game=0x10000000;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==3 && !g_hanging_visibility.arm);
    game=0;hanging_visibility_now();active=0;player=0;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==0x1003 && g_hanging_visibility.arm);
    active=1;player=0x2000;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==0x1003 && g_hanging_visibility.arm);
    player=0;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==3 && *(LONG *)(evm.b+0x58)==5 && !g_hanging_visibility.arm);
    active=1;player=0x1000;*(LONG *)(objs.b+0x58)=3;*(LONG *)(evm.b+0x58)=5;
    hanging_visibility_now();g_b.armed=0;player=0;hanging_visibility_now();
    HV_CHECK(*(LONG *)(objs.b+0x58)==0x1003 && *(LONG *)(evm.b+0x58)==0x105 && !g_hanging_visibility.arm);
    g_b.a=anchors;g_b.armed=armed;g_hanging_visibility=saved;
    printf("hanging visibility: %d checks, %d failures\n",checks,bad);
#undef HV_CHECK
    return bad;
}
