

typedef struct {
    ULONGLONG arm,objs,evm;
    LONG obj_owned,evm_owned;
} DG_HANGING_VISIBILITY;
static DG_HANGING_VISIBILITY g_hanging_visibility;
static void hanging_visibility_reset(void)
{
    memset(&g_hanging_visibility,0,sizeof g_hanging_visibility);
}
static int hanging_visibility_region(ULONGLONG p,ULONGLONG size)
{
    return plausible_ptr(p) && size<=0x1000 && region_end(p)>=p+size;
}
static void hanging_visibility_now(void)
{
    DG_HANGING_VISIBILITY *s=&g_hanging_visibility;
    ULONGLONG arm,objs,evm,player;
    unsigned game,menu;
    int native,hang,safe;
    LONG of,ef;
    if(!InterlockedCompareExchange(&g_b.armed,0,0))goto forget;
    if (!hanging_visibility_region(g_b.a.gm_player_arm_body,8) ||
        !hanging_visibility_region(g_b.a.gm_player_status,8) ||
        !g_b.a.gbp_active || region_end(g_b.a.gbp_active)<g_b.a.gbp_active+4 ||
        !g_b.a.gm_game_status || region_end(g_b.a.gm_game_status)<g_b.a.gm_game_status+4 ||
        !g_b.a.gm_game_status_scn || region_end(g_b.a.gm_game_status_scn)<g_b.a.gm_game_status_scn+4 ||
        !g_b.a.gm_menu_status || region_end(g_b.a.gm_menu_status)<g_b.a.gm_menu_status+4 ||
        !g_b.a.gm_menu_status_scn || region_end(g_b.a.gm_menu_status_scn)<g_b.a.gm_menu_status_scn+4)
        goto forget;
    arm=*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if(!hanging_visibility_region(arm,0x38))goto forget;
    objs=*(volatile ULONGLONG *)(ULONG_PTR)arm;
    evm=*(volatile ULONGLONG *)(ULONG_PTR)(arm+0x30);
    if(!hanging_visibility_region(objs,0x5c) || !hanging_visibility_region(evm,0x5c))goto forget;
    if(s->arm!=arm || s->objs!=objs || s->evm!=evm)memset(s,0,sizeof *s);
    player=*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status;
    game=(unsigned)RD32(g_b.a.gm_game_status)|(unsigned)RD32(g_b.a.gm_game_status_scn);
    menu=(unsigned)RD32(g_b.a.gm_menu_status)|(unsigned)RD32(g_b.a.gm_menu_status_scn);
    native=RD32(g_b.a.gbp_active)!=0;
    safe=!(game&DG_GAME_UNSAFE_MASK) && !(menu&DG_MENU_UNSAFE_MASK);
    hang=g_b.armed && native && safe && (player&0x1000ULL);
    of=RD32(objs+0x58);ef=RD32(evm+0x58);
    if(hang) {
        if(!s->arm) {
            s->arm=arm;s->objs=objs;s->evm=evm;
            s->obj_owned=(~of)&0x1000;s->evm_owned=(~ef)&0x100;
        }
        WR32(objs+0x58,of|0x1000);WR32(evm+0x58,ef|0x100);
        return;
    }
    /* Never make a cutscene/third-person/dead arm visible. Normal native FPS
       can restore only our added bits; all foreign flags remain untouched. */
    if(s->arm) {
        /* Climbing back first passes FORCE/native camera changes. Retain
           same-chain debt through that transition without making any write;
           repay only when normal FPS owns the subjective mesh again. */
        if(!native || !safe || (player&DG_PLAYER_UNSAFE_MASK))return;
        WR32(objs+0x58,of&~s->obj_owned);WR32(evm+0x58,ef&~s->evm_owned);
    }
forget:
    hanging_visibility_reset();
}
