

#include "dg_radial_pause_retail.h"
static struct {
    volatile LONG owned;
    volatile LONG64 resumed_ms;
    volatile LONG *level;
    void (*sound)(int,int,int);
    int visible, selected, kind, confirmed;
    uint64_t context;
    int deferred;
    dg_radial_commit_intent intent;
    uint64_t player,inventory;
} g_radial_pause;
int dg_bridge_radial_paused(void) {
    return InterlockedCompareExchange(&g_radial_pause.owned,0,0)!=0;
}
int dg_bridge_radial_context_held(void) {
    uint64_t now=GetTickCount64();
    uint64_t resumed=(uint64_t)InterlockedCompareExchange64(&g_radial_pause.resumed_ms,0,0);
    return dg_bridge_radial_paused() || (resumed && now>=resumed && now-resumed<=100);
}
static void radial_pause_release(void) {
    uint64_t player=0;
    uint32_t menu=0,scenario=0;
    if (!InterlockedExchange(&g_radial_pause.owned,0)) return;
    InterlockedExchange64(&g_radial_pause.resumed_ms,(LONG64)GetTickCount64());
    /* A native inventory menu can take over the same bit. Leave its pause
     * intact; never restore an old complete pause word over another owner. */
    (void)radial_live_read(NULL,g_b.a.gm_player_status,&player,8);
    (void)radial_live_read(NULL,g_b.a.gm_menu_status,&menu,4);
    (void)radial_live_read(NULL,g_b.a.gm_menu_status_scn,&scenario,4);
    if (!(player&UINT64_C(0x08000000)) && !((menu|scenario)&0x300))
        InterlockedAnd(g_radial_pause.level,~4L);
}
static void radial_pause_cancel(void) {
    radial_pause_release();
    InterlockedExchange64(&g_radial_pause.resumed_ms,0);
    g_radial_pause.deferred=0;
    g_radial_pause.visible=0;
    g_radial_pause.selected=-1;
}
static int radial_pause_witness(const LiveImage *im,uint32_t rva,
    const unsigned char *bytes,size_t size) {
    return all_valid(im->valid,rva,size,im->size) &&
        !memcmp(im->bytes+rva,bytes,size);
}
static void radial_pause_resolve(const LiveImage *im) {
    MEMORY_BASIC_INFORMATION m;
    memset(&g_radial_pause,0,sizeof g_radial_pause);
    g_radial_pause.selected=-1;
    if (!radial_pause_witness(im,0x1ecaf6,radial_weapon_witness,sizeof radial_weapon_witness) ||
        !radial_pause_witness(im,0x1eda18,radial_item_witness,sizeof radial_item_witness) ||
        !radial_pause_witness(im,0x6faf0,radial_sound_witness,sizeof radial_sound_witness) ||
        !all_valid(im->valid,0x17dbc7c,4,im->size)) return;
    if (!VirtualQuery((void *)(ULONG_PTR)(im->base+0x17dbc7c),&m,sizeof m) ||
        m.State!=MEM_COMMIT || (m.Protect&PAGE_GUARD) ||
        (m.Protect&0xff)!=PAGE_READWRITE) return;
    g_radial_pause.level=(volatile LONG *)(ULONG_PTR)(im->base+0x17dbc7c);
    g_radial_pause.sound=(void (*)(int,int,int))(ULONG_PTR)(im->base+0x6faf0);
}
void dg_bridge_radial_feedback(int visible,int selected,int kind,uint64_t context,int accept) {
    if (!g_controls_active || !g_radial_pause.level || !g_radial_pause.sound) return;
    if (visible && !g_radial_pause.visible) g_radial_pause.sound(0x20,0x3f,0x15);
    else if (!visible && g_radial_pause.visible && !g_radial_pause.confirmed) g_radial_pause.sound(0x20,0x3f,0x14);
    else if (visible && selected>=0 && (selected!=g_radial_pause.selected ||
             kind!=g_radial_pause.kind || context!=g_radial_pause.context))
        g_radial_pause.sound(0x20,0x3f,0x17);
    g_radial_pause.visible=visible;g_radial_pause.selected=selected;
    g_radial_pause.kind=kind;g_radial_pause.context=context;g_radial_pause.confirmed=0;
    if (!visible) {
        radial_pause_release();
        if (g_radial_pause.deferred && accept) {
            g_radial_pause.deferred=0;
            dg_bridge_radial_offer(&g_radial_pause.intent,g_radial_pause.player,
                g_radial_pause.inventory,1,GetTickCount64());
        }
        g_radial_pause.deferred=0;
    }
    else if (!dg_bridge_radial_paused() &&
             g_radial_game.commit.phase==DG_RADIAL_COMMIT_IDLE &&
             InterlockedCompareExchange(g_radial_pause.level,4,0)==0)
        InterlockedExchange(&g_radial_pause.owned,1);
}
