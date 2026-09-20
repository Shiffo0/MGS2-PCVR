#include "dg_mod_menu.h"
static DG_MOD_MENU g_mod_menu;
static volatile LONG g_mod_stereo_override=-1;
static volatile LONG g_mod_input_capture;
/* Only controls_provide owns the UI state. The worker reads a single atomic
   override and uses the established stereo configure/invalidation path. */
static void mod_menu_input(int allowed,const DG_XR_FRAME *f,uint64_t generation,
                           dg_radial_view *view) {
    static const int vk[6]={VK_HOME,VK_UP,VK_DOWN,VK_RETURN,VK_LEFT,VK_RIGHT};
    static const char *labels[3]={"VIEW","VR PISTOL RELOAD","M9 SLIDE"};
    unsigned keys=0,actual,available,changed;
    DWORD pid=0;int i,focus,neutral=0;
    GetWindowThreadProcessId(GetForegroundWindow(),&pid);
    focus=pid==GetCurrentProcessId();
    for(i=0;i<6;i++)if(GetAsyncKeyState(vk[i])&0x8000)keys|=1u<<i;
    dg_bridge_mod_menu_status(&actual,&available);
    actual|=InterlockedCompareExchange(&g_stereo,0,0)?1u:0u;
    available|=1;
    if(f) {
        const DG_XR_HAND *l=&f->left_hand,*r=&f->right_hand;
        neutral=l->grip.active && r->grip.active && l->grip.tracked && r->grip.tracked &&
            l->grip.sample_seq && l->grip.sample_seq==r->grip.sample_seq &&
            l->grip.pose_age_ms<=100 && r->grip.pose_age_ms<=100 &&
            !l->trigger_click && !r->trigger_click && l->trigger_value<.1f && r->trigger_value<.1f &&
            !l->squeeze_click && !r->squeeze_click && !l->primary_button && !l->secondary_button &&
            !r->primary_button && !r->secondary_button && !l->menu_button && !r->menu_button &&
            !l->thumbstick_click && !r->thumbstick_click &&
            fabs(l->thumbstick_x)<.1f && fabs(l->thumbstick_y)<.1f &&
            fabs(r->thumbstick_x)<.1f && fabs(r->thumbstick_y)<.1f;
    }
    changed=dg_mod_menu_step(&g_mod_menu,keys,focus && generation &&
        g_source==SRC_XR && allowed==DG_CONTROLS_GAMEPLAY,neutral,actual,available);
    if(changed&1)InterlockedExchange(&g_mod_stereo_override,(g_mod_menu.requested&1)!=0);
    if(changed&6)dg_bridge_mod_menu_request(g_mod_menu.requested);
    /* Keep rearm state across unsafe scenes, but never capture title/codec input. */
    InterlockedExchange(&g_mod_input_capture,g_mod_menu.capture && allowed==DG_CONTROLS_GAMEPLAY);
    dg_bridge_mod_menu_capture(g_mod_menu.capture && allowed==DG_CONTROLS_GAMEPLAY);
    memset(view,0,sizeof *view);
    if(!g_mod_menu.open)return;
    view->visible=1;view->kind=2;view->count=3;
    view->selected=g_mod_menu.selected;view->eligible=available;
    for(i=0;i<3;i++) {
        unsigned bit=1u<<i;
        strcpy_s(view->labels[i],17,labels[i]);
        strcpy_s(view->labels[i+3],17,!(available&bit)?"UNAVAILABLE":
            ((actual^g_mod_menu.requested)&bit)?"PENDING":
            i==0?((actual&bit)?"STEREO":"MONO"):((actual&bit)?"ON":"OFF"));
    }
}
