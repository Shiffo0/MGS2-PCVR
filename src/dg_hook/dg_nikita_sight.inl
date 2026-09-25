/* Reuse the native Nikita layout. A local JZ witness selects native invisible
 * versus animation/display; global SGT bits and camera/FOV are never written.
 * In-flight rockets retain the native sight/camera policy unconditionally. */
static struct { uint64_t base;int resolved,live;volatile LONG fault;DG_DETOUR detour; } g_nikita_sight;
static SRWLOCK g_nikita_mode_lock=SRWLOCK_INIT;
static DG_NIKITA_MODE g_nikita_mode;
static int g_nikita_claim;
static void nikita_revoke(void) {
    AcquireSRWLockExclusive(&g_nikita_mode_lock);
    dg_nikita_mode_reset(&g_nikita_mode);g_nikita_claim=0;
    ReleaseSRWLockExclusive(&g_nikita_mode_lock);
}
static int nikita_context(DG_NIKITA_BINDING *n) {



    uint64_t arm,now=GetTickCount64();int alive;
    if((!g_nikita_sight.live) || (g_nikita_sight.fault) || (!g_b.armed) || (g_b.owner) ||
       (g_b.script_menu_only) || (g_b.s_late_unsafe) || (g_b.fps.state!=DG_FPS_ACTIVE) ||
       (!g_controls_allowed) || (g_controls_special) ||
       (now<g_controls_lease) || (now-g_controls_lease>100) ||
       (!interact_read(NULL,g_b.a.gm_player_arm_body,&arm,8)) ||
       (!interact_read(NULL,g_nikita_sight.base+0x17df7e4,&alive,4)) || (alive!=0) ||
       (!dg_aim_capture_nikita_binding((uintptr_t)g_nikita_sight.base,arm,n)))return 0;
    return 1;
}
static void nikita_tick(int safe) {
    DG_NIKITA_BINDING n;DG_NIKITA_INPUT in=g_controls_frame.nikita;
    int selected=safe && g_controls_active && nikita_context(&n);
    in.allowed=in.allowed && selected;
    AcquireSRWLockExclusive(&g_nikita_mode_lock);
    g_nikita_claim=selected;
    dg_nikita_mode_step(&g_nikita_mode,&in,selected?n.actor:0);
    ReleaseSRWLockExclusive(&g_nikita_mode_lock);
}
static int nikita_mode_now(uint64_t actor,int *scope) {
    uint64_t now=GetTickCount64();int valid;
    AcquireSRWLockShared(&g_nikita_mode_lock);
    valid=g_nikita_mode.active && actor==g_nikita_mode.actor &&
        now>=g_nikita_mode.stamp && now-g_nikita_mode.stamp<=100;
    *scope=valid && g_nikita_mode.scope;
    ReleaseSRWLockShared(&g_nikita_mode_lock);return valid;
}
static void nikita_sight_branch(void *rsp) {
    uint64_t *regs=(uint64_t *)rsp-16,child,parent,fn;DG_NIKITA_BINDING n;
    int scope,handle;unsigned sight_status;
    if(!TryAcquireSRWLockShared(&g_controls_lock))return;
    __try {
        if(nikita_context(&n) && nikita_mode_now(n.actor,&scope) &&
           interact_read(NULL,n.actor+0x38,&child,8) && child==regs[11] &&
           interact_read(NULL,child+0x40,&parent,8) && parent==n.actor &&
           interact_read(NULL,child+8,&fn,8) && fn==g_nikita_sight.base+0x4f8df0 &&
           interact_read(NULL,child+0x58,&handle,4) && handle>=0 &&
           interact_read(NULL,g_nikita_sight.base+0x17b4974,&sight_status,4) && !(sight_status&0x8000)) {
            /* Saved RFLAGS at slot 15. JZ skips invisible() when ZF=1. */
            if(scope)regs[15]|=0x40;else regs[15]&=~UINT64_C(0x40);
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){g_nikita_sight.fault=1;}
    ReleaseSRWLockShared(&g_controls_lock);
}
static void nikita_sight_resolve(const LiveImage *im) {
    memset(&g_nikita_sight,0,sizeof g_nikita_sight);
    if(!m9_match(im,0x4f8e30,(const unsigned char *)"\xf6\x05\x3d\xbb\x2b\x01\x02\x74\x47\x8b\x4b\x58\xba\x34\x46\x2a\0\xe8\x1a\x67\xc2\xff",22) ||
       !m9_match(im,0x4f97f9,(const unsigned char *)"\x48\x8d\x15\xf0\xf5\xff\xff",7) ||
       !m9_match(im,0x4eb5c1,(const unsigned char *)"\x43\xc7\x84\xbe\xe4\xf7\x7d\x01\x01\0\0\0",12) ||
       !m9_match(im,0xa9498,(const unsigned char *)"\x48\x89\x51\x38",4) ||
       !m9_match(im,0xa94a0,(const unsigned char *)"\x48\x89\x4a\x40",4))return;
    g_nikita_sight.base=im->base;g_nikita_sight.resolved=1;
}
static void nikita_sight_stop(void) {
    g_nikita_sight.live=0;nikita_revoke();dg_detour_remove(&g_nikita_sight.detour);
}
static void nikita_sight_install(void) {
    const char *why="retail sight witnesses unavailable";uint64_t b=g_nikita_sight.base;
    if(g_nikita_sight.resolved)
        g_nikita_sight.live=dg_detour_install_ex(&g_nikita_sight.detour,
            (void *)(uintptr_t)(b+0x4f8e30),nikita_sight_branch,
            (void *)(uintptr_t)(b+0x4f8df0),(void *)(uintptr_t)(b+0x4f8eb0),&why,4);
    if(g_b.log)g_b.log("  Nikita grip sight: %s (%s); native missile view retained\r\n",
        g_nikita_sight.live?"installed":"unavailable",why);
}
