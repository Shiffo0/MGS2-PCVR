/* Hooks stay installed for the session; only game-thread state is switched. */
static volatile LONG g_mod_feature_want, g_mod_capture;
static int g_mod_capture_ticks;
void dg_bridge_mod_menu_capture(int capture) {
    InterlockedExchange(&g_mod_capture,capture!=0);
}
void dg_bridge_mod_menu_request(unsigned values) {
    InterlockedExchange(&g_mod_feature_want,(LONG)(values&6));
}
void dg_bridge_mod_menu_status(unsigned *actual,unsigned *available) {
    unsigned a=0,v=0;
    AcquireSRWLockShared(&g_m9_lock);
    if(g_m9.live && !g_m9.fault){v|=4;if(g_m9_enabled)a|=4;}
    ReleaseSRWLockShared(&g_m9_lock);
    AcquireSRWLockShared(&g_reload_lock);
    if(g_reload.live && !g_reload.fault){v|=2;if(g_reload.enabled)a|=2;}
    ReleaseSRWLockShared(&g_reload_lock);
    *actual=a;*available=v;
}
static void mod_features_tick(int safe,int neutral) {
    unsigned want=(unsigned)InterlockedCompareExchange(&g_mod_feature_want,0,0);
    if(!safe || !neutral)return;
    AcquireSRWLockExclusive(&g_reload_lock);
    /* Let native animation/events finish. The caller soft-cancels the held
       weapon for three ticks before releasing a menu-owned pad. */
    if(!g_reload.anim && g_reload.live && !g_reload.fault &&
       g_reload.enabled!=((want&2)!=0)) {
        g_reload.enabled=(want&2)!=0;
        g_reload.active=g_reload.waiting=g_reload.neutral=g_reload.driving=0;
        g_reload.cancel_ticks=g_reload.safe=g_reload.body_seek=g_reload.arm_seek=0;
        dg_reload_gesture_reset(&g_reload.gesture);
        if(g_b.log)g_b.log("  mod menu: VR pistol reload %s\r\n",g_reload.enabled?"on":"off");
    }
    ReleaseSRWLockExclusive(&g_reload_lock);
    AcquireSRWLockExclusive(&g_m9_lock);
    if(g_m9.live && !g_m9.fault && g_m9_enabled!=((want&4)!=0)) {
        g_m9.epoch++;dg_m9_reset(&g_m9.state);
        memset(&g_m9.out,0,sizeof g_m9.out);
        g_m9.active=g_m9.render=g_m9.blocked=0;
        InterlockedExchange(&g_m9_events,0);
        InterlockedExchange(&g_m9_enabled,(want&4)!=0);
        if(g_b.log)g_b.log("  mod menu: M9 slide %s\r\n",g_m9_enabled?"on":"off");
    }
    ReleaseSRWLockExclusive(&g_m9_lock);
}
/* Called after all gameplay input writers. Preserve native subject stance,
   cancel pistol fire through pressure (never manufacture a firing release). */
static void mod_menu_pad(int safe) {
    LONG mask,index,subject;
    uint64_t p=g_b.a.player_pad;
    if(!safe || !g_mod_capture || !p || !g_b.a.pad_weapon)return;
    mask=RD32(g_b.a.pad_weapon);index=g_b.a.pad_press_weapon?RD32(g_b.a.pad_press_weapon):-1;
    subject=g_b.a.pad_subject?RD32(g_b.a.pad_subject):0;
    *(volatile LONG *)(ULONG_PTR)(p+DG_PAD_STATUS_OFFSET)&=subject|mask;
    *(volatile LONG *)(ULONG_PTR)(p+DG_PAD_PRESS_OFFSET)=0;
    *(volatile LONG *)(ULONG_PTR)(p+DG_PAD_RELEASE_OFFSET)=0;
    *(volatile unsigned char *)(ULONG_PTR)(p+DG_PAD_LEFT_DX_OFFSET)=128;
    *(volatile unsigned char *)(ULONG_PTR)(p+DG_PAD_LEFT_DY_OFFSET)=128;
    *(volatile unsigned char *)(ULONG_PTR)(p+DG_PAD_RIGHT_DX_OFFSET)=128;
    *(volatile unsigned char *)(ULONG_PTR)(p+DG_PAD_RIGHT_DX_OFFSET+1)=128;
    if(!reload_native_animation())m9_block_pad(mask,index);
}



static void mod_menu_start_pad(void) {
    uint64_t records[2]={g_b.a.gv_pad_data_direct,g_b.a.gv_pad_data};int i;
    if(!g_mod_capture)return;
    InterlockedExchange(&g_b.start_pending,0);
    for(i=0;i<2;i++)if(records[i]) {
        *(volatile DWORD *)(ULONG_PTR)(records[i]+DG_GV_PAD_STATUS_OFFSET)&=~DG_PAD_START;
        *(volatile DWORD *)(ULONG_PTR)(records[i]+DG_GV_PAD_PRESS_OFFSET)&=~DG_PAD_START;
    }
    if(g_b.a.gv_pad_press)*(volatile DWORD *)(ULONG_PTR)g_b.a.gv_pad_press&=~DG_PAD_START;
}
