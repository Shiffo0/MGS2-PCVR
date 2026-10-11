#include "dg_mod_menu.h"
static DG_MOD_MENU g_mod_menu;
static volatile LONG g_mod_stereo_override=-1;
static volatile LONG g_mod_input_capture;
/* Only controls_provide owns the UI state. The worker reads a single atomic
   override and uses the established stereo configure/invalidation path. */
static void mod_menu_input(int allowed,const DG_XR_FRAME *f,uint64_t generation,
                           dg_radial_view *view) {
    static const int vk[6]={VK_HOME,VK_UP,VK_DOWN,VK_RETURN,VK_LEFT,VK_RIGHT};
    static const char *labels[DG_MM_ITEMS]={"VIEW","VR PISTOL RELOAD","M9 SLIDE","DESKTOP OBSERVER","VR RENDERING"};
    unsigned keys=0,actual,available,changed;
    DWORD pid=0;int i,focus,neutral=0;
    GetWindowThreadProcessId(GetForegroundWindow(),&pid);
    focus=pid==GetCurrentProcessId();
    for(i=0;i<6;i++)if(GetAsyncKeyState(vk[i])&0x8000)keys|=1u<<i;
    dg_bridge_mod_menu_status(&actual,&available);
    actual|=InterlockedCompareExchange(&g_stereo,0,0)?1u:0u;
    actual|=observer_on()?8u:0u;
    available|=1|8;
#if 1
    /* Second eye (replayed other eye): only in stereo with verified link anchors. */
    actual|=second_eye_wanted()?16u:0u;
    if(InterlockedCompareExchange(&g_stereo,0,0)&&g_phase_stage_addr&&g_phase_consume_addr)available|=16;
#endif
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
    if(changed&8) {
        int next=dg_mod_observer_next((actual&8)!=0,observer_mode());
        InterlockedExchange(&g_mod_observer_override,next>=0);
        if(next>=0)InterlockedExchange(&g_mod_observer_mode_override,next);
        g_mod_menu.requested=(g_mod_menu.requested&~8u)|(next>=0?8u:0u);
    }
    if(changed&16)InterlockedExchange(&g_mod_second_eye_override,(g_mod_menu.requested&16)!=0);
    if(changed&6)dg_bridge_mod_menu_request(g_mod_menu.requested);
    /* Keep rearm state across unsafe scenes, but never capture title/codec input. */
    InterlockedExchange(&g_mod_input_capture,g_mod_menu.capture && allowed==DG_CONTROLS_GAMEPLAY);
    dg_bridge_mod_menu_capture(g_mod_menu.capture && allowed==DG_CONTROLS_GAMEPLAY);
    memset(view,0,sizeof *view);
    if(!g_mod_menu.open)return;
    view->visible=1;view->kind=2;view->count=DG_MM_ITEMS;
    view->selected=g_mod_menu.selected;view->eligible=available;
    for(i=0;i<DG_MM_ITEMS;i++) {
        unsigned bit=1u<<i;
        strcpy_s(view->labels[i],17,labels[i]);
        strcpy_s(view->labels[i+DG_MM_ITEMS],17,!(available&bit)?"UNAVAILABLE":
            ((actual^g_mod_menu.requested)&bit)?"PENDING":
            i==0?((actual&bit)?"STEREO":"MONO"):
#if 1
            /* Second eye asked for but no replay stored in the last 30 Presents
               (route not attached yet, link idle): say so instead of ON. */
            (i==4&&(actual&bit)&&(LONG)(InterlockedCompareExchange(&g_present_frame,0,0)-
                InterlockedCompareExchange(&g_second_eye_frame,0,0))>30)?"WAITING":
#endif
            (i==3&&(actual&bit))?(observer_mode()==2?"BOTH EYES (EXP)":observer_mode()==1?"RIGHT EYE":"LEFT EYE"):
            i==4?((actual&bit)?"BOTH EYES (EXP)":"AER"):
            ((actual&bit)?"ON":"OFF"));
    }
}
