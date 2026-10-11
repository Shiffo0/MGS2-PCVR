static int magwell_waiting(void) {
    return g_reload.weapon==2 && reload_fresh() && reload_visual_ready() &&
        g_reload.valid && g_reload.waiting && g_reload.empty && g_reload.reserve &&
        !g_reload.anim && !g_reload.gesture.permit && !g_b.script_menu_only;
}
int dg_bridge_magwell_allowed(void) {
    int ok=0;if(TryAcquireSRWLockShared(&g_reload_lock)) {
        ok=magwell_waiting();ReleaseSRWLockShared(&g_reload_lock);
    }return ok;
}
/* Called at native list preparation; the result travels with that exact
   list's camera handoff, never with the latest/current render-thread pose. */
int dg_bridge_magwell_snapshot(DG_MAGWELL *out) {
    int ok=0;uint64_t arm,weapon,objs,root;double point[3];
    memset(out,0,sizeof *out);
    if(!TryAcquireSRWLockShared(&g_reload_lock))return 0;
    __try {
        if(!magwell_waiting()||!g_b.a.gm_player_arm_body)__leave;
        arm=*(uint64_t *)(ULONG_PTR)g_b.a.gm_player_arm_body;
        if(!arm||arm!=g_reload.arm||*(uint64_t *)(ULONG_PTR)(g_reload.actor+0x68)!=arm||
           *(int *)(ULONG_PTR)(g_reload.actor+0x80)!=2||
           *(int *)(ULONG_PTR)(g_reload.player+0xb90)!=2||
           *(uint64_t *)(ULONG_PTR)(g_reload.player+0xba8)!=arm)__leave;
        weapon=*(uint64_t *)(ULONG_PTR)(g_reload.actor+0x60);if(!weapon)__leave;
        objs=*(uint64_t *)(ULONG_PTR)weapon;if(!objs||(*(unsigned *)(ULONG_PTR)(objs+0x58)&0x1000))__leave;
        root=*(uint64_t *)(ULONG_PTR)(objs+0x40);if(!root)root=objs;
        if(!reload_point(root,dg_usp_magwell_mouth,point))__leave;
        memcpy(out->world,(const void *)(ULONG_PTR)root,64);
        for(int i=0;i<3;i++)out->world[12+i]=(float)point[i];
        out->ms=GetTickCount64();out->valid=1;ok=1;
    } __except(EXCEPTION_EXECUTE_HANDLER){ok=0;}
    ReleaseSRWLockShared(&g_reload_lock);
    if(!ok)memset(out,0,sizeof *out);return ok;
}
