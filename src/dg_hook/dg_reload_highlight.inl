#include "dg_reload_highlight.h"
static DG_DETOUR g_reload_highlight_hook;
static uint64_t g_reload_highlight_base;
static int g_reload_highlight_resolved;
static unsigned long g_reload_highlight_passes;



static int reload_highlight_owned(uint64_t objs,uint64_t unit,uint64_t now) {
    int yes=0; uint64_t arm,mag;
    if(!objs || !g_b.armed || g_b.fps.state!=DG_FPS_ACTIVE ||
       g_b.fire_mode!=DG_FIRE_MODE_ON || g_b.s_late_unsafe || g_b.script_menu_only ||
       !g_b.a.gm_player_arm_body)return 0;
    arm=*(uint64_t *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if(!arm || (*(unsigned *)(ULONG_PTR)(objs+0x58)&0x1000))return 0;
    if(TryAcquireSRWLockShared(&g_m9_lock)) {
        __try {
            yes=g_m9_enabled && g_m9.live && g_m9.active && g_m9.render && !g_m9.fault &&
                g_m9.objs==objs && unit==objs+0x110+2*0x180 &&
                g_m9.state.state==M9_NEEDS_RACK &&
                now>=g_m9.stamp && now-g_m9.stamp<=100 &&
                now>=g_m9.state.now_ms && now-g_m9.state.now_ms<=100 &&
                m9_current_owner(arm,g_m9.ctrl,objs);
        } __finally {ReleaseSRWLockShared(&g_m9_lock);}
    }
    if(yes)return 1;
    if(TryAcquireSRWLockShared(&g_reload_lock)) {
        __try {
            if(g_reload.weapon!=2 && reload_fresh() && g_reload.valid && g_reload.waiting && g_reload.empty &&
               g_reload.reserve && !g_reload.anim && !g_reload.gesture.permit &&
               g_reload.arm==arm && reload_visual_ready() &&
               *(uint64_t *)(ULONG_PTR)(g_reload.actor+0x68)==arm &&
               *(int *)(ULONG_PTR)(g_reload.player+0xb90)==g_reload.weapon &&
               *(uint64_t *)(ULONG_PTR)(g_reload.player+0xba8)==arm) {
                mag=*(uint64_t *)(ULONG_PTR)(g_reload.actor+0x70);
                yes=mag==objs && unit==objs+0x110 &&
                    *(short *)(ULONG_PTR)(objs+0x64)==1;
            }
        } __finally {ReleaseSRWLockShared(&g_reload_lock);}
    }
    return yes;
}






static void reload_highlight_native(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    if(!g_reload_highlight_resolved)return;
    __try {
        if(r[6]>=0x18 && reload_highlight_owned(r[9],r[11],GetTickCount64()) &&
           dg_reload_highlight_colour((float *)(ULONG_PTR)(r[6]-0x18)))
            g_reload_highlight_passes++;
    } __except(EXCEPTION_EXECUTE_HANDLER) { /* Refuse this sample. */ }
}
static void reload_highlight_resolve(const LiveImage *im) {
    g_reload_highlight_base=im->base;g_reload_highlight_resolved=0;
    g_reload_highlight_passes=0;
    if(!reload_match(im,0x9b99c,"\x0f\xbf\x51\x64\x48\x8d\x99\x10\x01\x00\x00",11) ||
       !reload_match(im,0x9bdb8,"\x48\x8b\x8b\xd8\x00\x00\x00\x8b\x41\x40\x41\x89\x41\xe8",14) ||
       !reload_match(im,0x9be12,"\x8b\x41\x70\x41\x89\x41\x18\x8b\x41\x74\x41\x89\x41\x1c\x8b\x41\x78\x41\x89\x41\x20\x8b\x41\x7c\x41\x89\x41\x24",28) ||
       !reload_match(im,0x9be85,"\x41\x8b\x42\xe8\x41\x89\x41\x28\x41\x8b\x42\xec\x41\x89\x41\x2c",16) ||
       !reload_match(im,0x4d73e7,"\x48\x8b\x47\x70\x81\x60\x58\xff\xcf\xff\xff",11))return;
    g_reload_highlight_resolved=1;
}
static void reload_highlight_install(void) {
    const char *why="retail witnesses unavailable"; int ok=0;
    if(g_reload_highlight_resolved)ok=dg_detour_install_ex(&g_reload_highlight_hook,
        (void *)(ULONG_PTR)(g_reload_highlight_base+0x9be85),reload_highlight_native,
        (void *)(ULONG_PTR)(g_reload_highlight_base+0x9b980),
        (void *)(ULONG_PTR)(g_reload_highlight_base+0x9c1bf),&why,1);
    if(g_b.log)g_b.log("  reload highlight: Codec green %s (%s)\r\n",ok?"installed":"unavailable",why);
}
static void reload_highlight_stop(void) {
    dg_detour_remove(&g_reload_highlight_hook);
    if(g_b.log)g_b.log("  reload highlight: %lu native part colour copies\r\n",g_reload_highlight_passes);
}
