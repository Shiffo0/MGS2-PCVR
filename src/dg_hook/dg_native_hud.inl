

static ULONGLONG region_end(ULONGLONG address);
static DG_DETOUR g_hud_life_detour,g_hud_frame_detour;
static ULONGLONG g_native_hud_base;
static volatile LONG g_native_hud_requested;
static int g_native_hud_live;
void dg_bridge_native_hud_configure(int enabled) {
    InterlockedExchange(&g_native_hud_requested,enabled!=0);
}
static int native_hud_active(void) {
    return g_native_hud_live && g_native_hud_requested && g_b.armed &&
        g_b.fps.state==DG_FPS_ACTIVE && !g_b.script_menu_only && !g_b.s_late_unsafe;
}
static void native_hud_life(void *rsp) {
    uint64_t gauge=((uint64_t *)rsp-16)[11],prim;
    if(!native_hud_active())return;
    __try {
        /* Level zero is the player's LIFE gauge; other levels include bosses/O2. */
        if(!gauge || *(short *)(ULONG_PTR)(gauge+0x24)!=0)return;
        prim=*(uint64_t *)(ULONG_PTR)(gauge+0x38);
        if(prim)*(unsigned *)(ULONG_PTR)prim|=0x100;
    } __except(EXCEPTION_EXECUTE_HANDLER) { }
}
static void native_hud_frame(void *rsp) {
    uint64_t work=((uint64_t *)rsp-16)[11],sprite[4];int i;
    if(!native_hud_active() || !work)return;
    __try {
        for(i=0;i<4;i++) {
            sprite[i]=*(uint64_t *)(ULONG_PTR)(work+0x770+i*8);
            if(!sprite[i] || region_end(sprite[i])<sprite[i]+0x34)return;
        }
        for(i=0;i<4;i++)*(unsigned *)(ULONG_PTR)(sprite[i]+0x30)|=0x8000;
    } __except(EXCEPTION_EXECUTE_HANDLER) { }
}
static void native_hud_resolve(const LiveImage *im) {
    g_native_hud_base=0;
    if(m9_match(im,0x576841,(const unsigned char *)"\x66\x44\x39\x73\x24",5) &&
       m9_match(im,0x5769c3,(const unsigned char *)"\x48\x8b\x4b\x38\xe8\x64\x99\x10\x00",9) &&
       m9_match(im,0x5769da,(const unsigned char *)"\x8b\x15\x3c\x31\x17\x01",6) &&
       m9_match(im,0x67ff40,(const unsigned char *)"\x81\x09\x00\x01\x00\x00\xc3",7) &&
       m9_match(im,0x11df81,(const unsigned char *)"\x48\x8b\x83\x70\x07\x00\x00\x81\x48\x30\x00\x80\x00\x00",14) &&
       m9_match(im,0x11e075,(const unsigned char *)"\x48\x8b\x83\x88\x07\x00\x00\x81\x60\x30\xff\x7f\xff\xff\x48\x8b\x8b\x70\x07\x00\x00",21))
        g_native_hud_base=im->base;
}
static void native_hud_stop(void) {
    g_native_hud_live=0;
    dg_detour_remove(&g_hud_life_detour);dg_detour_remove(&g_hud_frame_detour);
}
static void native_hud_install(void) {
    const char *why="retail witnesses unavailable";uint64_t b=g_native_hud_base;
    if(b && dg_detour_install_ex(&g_hud_life_detour,(void *)(ULONG_PTR)(b+0x5769da),
       native_hud_life,(void *)(ULONG_PTR)(b+0x576730),(void *)(ULONG_PTR)(b+0x576a7d),&why,1) &&
       dg_detour_install_ex(&g_hud_frame_detour,(void *)(ULONG_PTR)(b+0x11e083),
       native_hud_frame,(void *)(ULONG_PTR)(b+0x11df40),(void *)(ULONG_PTR)(b+0x11e16f),&why,1))g_native_hud_live=1;
    if(!g_native_hud_live)native_hud_stop();
    if(g_b.log)g_b.log("  native HUD: LIFE + four radar frame sprites %s (%s)\r\n",
        g_native_hud_live?"hooks installed":"unchanged",why);
}
