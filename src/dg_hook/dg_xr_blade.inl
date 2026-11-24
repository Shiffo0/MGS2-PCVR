#include "dg_blade_haptic.h"
static volatile LONG g_blade_haptic_events,g_blade_haptic_stamp;
void dg_xr_blade_feedback(unsigned events) {
    if(!(events&(DG_BLADE_HAPTIC_HIT|DG_BLADE_HAPTIC_DEFLECT)))return;
    InterlockedExchange(&g_blade_haptic_stamp,(LONG)GetTickCount());
    InterlockedOr(&g_blade_haptic_events,(LONG)events);
}
static void xr_blade_haptic_drain(void) {
    PFN_xrApplyHapticFeedback apply=NULL;
    LONG events=InterlockedExchange(&g_blade_haptic_events,0);
    DG_BLADE_PULSE p;XrHapticActionInfo info;XrHapticVibration v;int h;
    if(!events || (DWORD)(GetTickCount()-(DWORD)g_blade_haptic_stamp)>DG_BLADE_HAPTIC_MAX_AGE_MS ||
       g_state!=XR_SESSION_STATE_FOCUSED || !g_input_ready || !g_input_attached ||
       !g_haptic_action || !dg_blade_haptic_plan((unsigned)events,&p))return;
    if(!resolve("xrApplyHapticFeedback",&apply) || !apply)return;
    for(h=0;h<2;h++) {
        int bit=h==HAND_LEFT?DG_BLADE_HAPTIC_LEFT:DG_BLADE_HAPTIC_RIGHT;
        if(!(p.hands&bit) || !g_hand_runtime[h].path)continue;
        memset(&info,0,sizeof info);info.type=XR_TYPE_HAPTIC_ACTION_INFO;
        info.action=g_haptic_action;info.subactionPath=g_hand_runtime[h].path;
        memset(&v,0,sizeof v);v.type=XR_TYPE_HAPTIC_VIBRATION;
        v.amplitude=p.amplitude;v.duration=p.duration_ns;v.frequency=XR_FREQUENCY_UNSPECIFIED;
        apply(g_sess,&info,(const XrHapticBaseHeader *)&v);
    }
}
