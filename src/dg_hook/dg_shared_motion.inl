


#define DG_MOTION_MOBILE 1u
#define DG_MOTION_RELOAD 2u
static void mobile_arm_pose(void *rsp);
static void reload_motion(void *rsp);
static DG_DETOUR g_motion_hook;
static unsigned g_motion_users;
static void shared_motion(void *rsp) {
    if(g_motion_users & DG_MOTION_MOBILE) mobile_arm_pose(rsp);
    if(g_motion_users & DG_MOTION_RELOAD) reload_motion(rsp);
}
static int motion_acquire(uint64_t base,unsigned user,const char **why) {
    void *target=(void *)(uintptr_t)(base+0x67dd70);
    if(g_motion_users) {
        if(!g_motion_hook.installed || g_motion_hook.target!=target) {
            *why="shared motion target mismatch";return 0;
        }
    } else if(!dg_detour_install_ex(&g_motion_hook,target,shared_motion,
        target,(void *)(uintptr_t)(base+0x67ddcb),why,1))return 0;
    g_motion_users|=user;
    return 1;
}
static void motion_release(unsigned user) {
    g_motion_users&=~user;
    if(!g_motion_users)dg_detour_remove(&g_motion_hook);
}
