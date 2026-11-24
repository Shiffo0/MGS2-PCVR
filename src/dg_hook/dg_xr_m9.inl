#include "dg_pistol_reload.h"
static volatile LONG g_m9_haptic_events,g_m9_haptic_stamp;
void dg_xr_m9_feedback(unsigned events) {
    if(!(events&(1u|2u|8u|DG_RELOAD_EV_ALL)))return;
    InterlockedExchange(&g_m9_haptic_stamp,(LONG)GetTickCount());
    InterlockedOr(&g_m9_haptic_events,(LONG)events);
}

static void xr_reload_haptic_amp(LONG events,int right,float *amplitude,XrDuration *duration) {
    *amplitude=0;*duration=0;
    if(events&DG_RELOAD_EV_FILL){*amplitude=0.60f;*duration=40000000;return;}
    if(right && (events&DG_RELOAD_EV_EJECT)){*amplitude=0.50f;*duration=35000000;return;}
    if(right && (events&DG_RELOAD_EV_CLICK)){*amplitude=0.15f;*duration=12000000;return;}

    if(!right && (events&DG_RELOAD_EV_GUIDE)){*amplitude=0.25f;*duration=20000000;}
}
static void xr_m9_haptic_drain(void) {
    PFN_xrApplyHapticFeedback apply=NULL;
    LONG events=InterlockedExchange(&g_m9_haptic_events,0);
    XrHapticActionInfo info;XrHapticVibration v;
    if(!events || (DWORD)(GetTickCount()-(DWORD)g_m9_haptic_stamp)>100 ||
       g_state!=XR_SESSION_STATE_FOCUSED || !g_input_ready || !g_input_attached ||
       !g_haptic_action || !g_hand_runtime[HAND_LEFT].path)return;
    if(!resolve("xrApplyHapticFeedback",&apply) || !apply)return;
    memset(&info,0,sizeof info);info.type=XR_TYPE_HAPTIC_ACTION_INFO;
    info.action=g_haptic_action;info.subactionPath=g_hand_runtime[HAND_LEFT].path;
    memset(&v,0,sizeof v);v.type=XR_TYPE_HAPTIC_VIBRATION;
    v.frequency=XR_FREQUENCY_UNSPECIFIED;
    if(events&(1|2|8)) {
        v.amplitude=(events&2)?0.55f:0.20f;v.duration=(events&2)?45000000:20000000;
        apply(g_sess,&info,(const XrHapticBaseHeader *)&v);
    } else if(events&(DG_RELOAD_EV_FILL|DG_RELOAD_EV_GUIDE)) {
        xr_reload_haptic_amp(events,0,&v.amplitude,&v.duration);
        apply(g_sess,&info,(const XrHapticBaseHeader *)&v);
    }
    if((events&DG_RELOAD_EV_ALL) && g_hand_runtime[HAND_RIGHT].path) {
        xr_reload_haptic_amp(events,1,&v.amplitude,&v.duration);
        if(v.amplitude>0) {
            info.subactionPath=g_hand_runtime[HAND_RIGHT].path;
            apply(g_sess,&info,(const XrHapticBaseHeader *)&v);
        }
    }
}
