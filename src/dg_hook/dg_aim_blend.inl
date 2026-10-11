




static DG_DETOUR g_aim_blend_hook;
static ULONGLONG g_aim_blend_base, g_aim_blend_lease_ms;
static int g_aim_blend_resolved;
static unsigned long g_aim_blend_filtered;

static uint64_t aim_blend_branches(const LONG *parents,int count,int right,int left)
{
    uint64_t mask=0; int j;
    if(!parents || count!=55 || parents[4]!=3 || parents[5]!=4 || parents[6]!=5 ||
       parents[8]!=7 || parents[9]!=8 || parents[10]!=9) return 0;
    for(j=0;j<count;j++) {
        int p=parents[j];
        if(p>=j || p < -1) return 0;
        if((right && j==4) || (left && j==8) ||
           (p>=0 && (mask&(1ULL<<p)))) mask|=1ULL<<j;
    }
    return mask;
}

static uint64_t aim_blend_owned(ULONGLONG mc,ULONGLONG now)
{
    ULONGLONG arm,objs,adj,flags,player=0,current=0;
    LONG weapon=-1,motion,age; short command[3]; LONG tick,pair;
    int left=0,k;
    if(!g_b.armed || g_b.fps.state!=DG_FPS_ACTIVE || g_b.s_late_unsafe ||
       g_b.script_menu_only || !g_b.ik_active || !g_b.arm_map_cache_valid ||
       !mc || mc!=g_b.ik_owned_mctrl || !g_aim_blend_lease_ms ||
       now<g_aim_blend_lease_ms || now-g_aim_blend_lease_ms>100 ||
       !g_b.hand_command_requested || !hand_command_read(command,&tick,&pair))return 0;
    age=(LONG)((DWORD)g_b.c_ticks-(DWORD)tick);
    if(age<0 || age>DG_HAND_COMMAND_TICKS ||
       resolve_hand_player(&current,&player,&weapon)!=DG_RESOLVE_OK ||
       (weapon!=1 && weapon!=2 && weapon!=8) || reload_native_animation())return 0;
    arm=g_b.ik_owned_arm;
    if(current!=arm || !plausible_ptr(arm) || region_end(arm)<arm+0x1b0 ||
       region_end(mc)<mc+0x60 || RD32(mc+0x14)!=55)return 0;


    motion=RD32(arm+0x1ac);
    if(motion!=0 && motion!=1 && motion!=2 && motion!=4 &&
       motion!=6 && motion!=7 && motion!=8 && motion!=10)return 0;
    if(RD32(arm+0x1a8)!=weapon)return 0;
    if(!g_b.a.gm_game_status || !g_b.a.gm_game_status_scn ||
       !g_b.a.gm_menu_status || !g_b.a.gm_menu_status_scn || !g_b.a.gm_player_status)return 0;
    if(((RD32(g_b.a.gm_game_status)|RD32(g_b.a.gm_game_status_scn))&DG_GAME_UNSAFE_MASK) ||
       ((RD32(g_b.a.gm_menu_status)|RD32(g_b.a.gm_menu_status_scn))&DG_MENU_UNSAFE_MASK) ||
       (*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status&DG_PLAYER_UNSAFE_MASK))return 0;
    objs=*(ULONGLONG *)(ULONG_PTR)arm;
    adj=*(ULONGLONG *)(ULONG_PTR)(mc+0x48);
    if(objs!=g_b.arm_map_objs || *(ULONGLONG *)(ULONG_PTR)(arm+8)!=mc ||
       adj!=g_b.ik_owned_adjust || !plausible_ptr(adj) || region_end(adj)<adj+55*16)return 0;
    flags=*(ULONGLONG *)(ULONG_PTR)(mc+0x38);
    if((flags&0x70)!=0x70 || (g_b.ik_owned_mask&0x30)!=0x30)return 0;
    for(k=0;k<8;k++)if(((float *)(ULONG_PTR)(adj+4*16))[k]!=g_b.arm_map_cached_adjust[k])return 0;
    if(g_left.active && g_left.arm==arm && g_left.objs==objs &&
       g_left.mctrl==mc && g_left.adjust==adj && (flags&DG_LEFT_MASK)==DG_LEFT_MASK) {
        left=1;
        for(k=0;k<8;k++)if(((float *)(ULONG_PTR)(adj+8*16))[k]!=g_left.cached[k])left=0;
    }
    return aim_blend_branches((const LONG *)g_b.skel_parents,g_b.skel_parents_read,1,left);
}

/* At +681550 RSI=motion control, RDI=rotation mask. The detour precedes
 * MOV R15,[RSI+58]; TEST EBX,EBX. The translation loop reloads RDI at +68158d. */
static void aim_blend_native(void *rsp)
{
    uint64_t *r=(uint64_t *)rsp-16,mask=0;
    if(!r[8])return;
    __try { mask=aim_blend_owned(r[9],GetTickCount64()); }
    __except(EXCEPTION_EXECUTE_HANDLER) {return;}
    if(mask&r[8]) {r[8]&=~mask;g_aim_blend_filtered++;}
}
static void aim_blend_resolve(const LiveImage *im)
{
    g_aim_blend_resolved=0;g_aim_blend_base=im->base;g_aim_blend_lease_ms=0;
    g_aim_blend_filtered=0;
    if(!reload_match(im,0x68154c,"\x48\x8b\x7e\x30\x4c\x8b\x7e\x58\x85\xdb\x7e\x3c",12) ||
       !reload_match(im,0x68158d,"\x48\x8b\x7e\x30\x8b\x5e\x14\xf6\x46\x18\x02",11) ||
       !reload_match(im,0x681569,"\xf3\x0f\x10\x5e\x20\x4d\x8b\xc6\x49\x8b\xd7\x49\x8b\xce\xe8\x04\xe6\xa3\xff",19) ||
       !reload_match(im,0x57d37a,"\xe8\xf1\x09\x10\x00\x89\xb3\x0c\x02\x00\x00",11))return;
    g_aim_blend_resolved=1;
}
static void aim_blend_install(void)
{
    const char *why="retail witnesses unavailable";int ok=0;
    if(g_aim_blend_resolved)ok=dg_detour_install_ex(&g_aim_blend_hook,
        (void *)(ULONG_PTR)(g_aim_blend_base+0x681550),aim_blend_native,
        (void *)(ULONG_PTR)(g_aim_blend_base+0x681542),
        (void *)(ULONG_PTR)(g_aim_blend_base+0x681594),&why,1);
    if(g_b.log)g_b.log("  aim blend: owned pistol rotation filter %s (%s)\r\n",ok?"installed":"unavailable",why);
}
static void aim_blend_stop(void)
{
    g_aim_blend_lease_ms=0;
    dg_detour_remove(&g_aim_blend_hook);
    if(g_b.log)g_b.log("  aim blend: filtered %lu native rotation passes\r\n",g_aim_blend_filtered);
}
