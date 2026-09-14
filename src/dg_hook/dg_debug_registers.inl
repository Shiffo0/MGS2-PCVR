/* Shared by production and the hardware test; caller suspends target thread. */
static int set_dr(HANDLE t, int arm) {
    CONTEXT c;
    memset(&c,0,sizeof c);c.ContextFlags=CONTEXT_DEBUG_REGISTERS;
    if(!GetThreadContext(t,&c)){
#ifdef DG_PROBE_HEALTH
        if(pd_mode)InterlockedIncrement(&pd_get_fail);
#endif
        return 0;
    }
    c.Dr0=arm?g_hook_addr:0;
    c.Dr1=arm?g_scene_blur_addr:0;
    c.Dr2=arm&&(g_phase_enabled||g_render_link_active||pp_status==PP_RECORDING)?g_phase_stage_addr:0;
    c.Dr3=arm&&(g_phase_enabled||g_render_link_active||pp_status==PP_RECORDING)?g_phase_consume_addr:0;
    c.Dr6=0;
    c.Dr7=arm?0x401ull|(c.Dr1?4ull:0ull)|(c.Dr2?16ull:0ull)|(c.Dr3?64ull:0ull):0;
#ifdef DG_HUD_WATCH_BUILD
    hud_debug_registers(&c,arm);
#endif
    if(SetThreadContext(t,&c))return 1;
#ifdef DG_PROBE_HEALTH
    if(pd_mode)InterlockedIncrement(&pd_set_fail);
#endif
    return 0;
}
