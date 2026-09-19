/* Queue only from the game thread. OpenXR haptics run on the input owner,
 * expire after 100 ms and never survive focus/session loss. */
static volatile LONG g_m9_haptic_events,g_m9_haptic_stamp;
void dg_xr_m9_feedback(unsigned events) {
    if(!(events&(1u|2u|8u)))return;
    InterlockedExchange(&g_m9_haptic_stamp,(LONG)GetTickCount());
    InterlockedOr(&g_m9_haptic_events,(LONG)events);
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
    v.amplitude=(events&2)?0.55f:0.20f;v.duration=(events&2)?45000000:20000000;
    v.frequency=XR_FREQUENCY_UNSPECIFIED;
    apply(g_sess,&info,(const XrHapticBaseHeader *)&v);
}
