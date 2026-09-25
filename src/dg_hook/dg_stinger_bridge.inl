


#include "dg_stinger_aim.h"
static DG_DETOUR g_stinger_detour[4];
static SRWLOCK g_stinger_mode_lock=SRWLOCK_INIT;
static DG_STINGER_MODE g_stinger_mode;
static int g_stinger_claim;
static void stinger_revoke(void) {
    AcquireSRWLockExclusive(&g_stinger_mode_lock);
    dg_stinger_mode_reset(&g_stinger_mode);g_stinger_claim=0;
    ReleaseSRWLockExclusive(&g_stinger_mode_lock);
}
/* Used only by the game tick while holding the controls lease. */
static int stinger_claim(void) {return g_stinger_claim;}

static struct { uint64_t base; int resolved; volatile LONG live,fault; } g_stinger;
int dg_bridge_stinger_enabled(void) {return g_stinger.live && !g_stinger.fault;}



static const unsigned stinger_sites[4]={0x715394,0x715569,0x4a0979,0x4a0bb1};
static const unsigned char stinger_bytes[4][12]={
    {0x4d,0x8b,0x86,0xa0,0,0,0,0x85,0xca},
    {0x41,0x0f,0xb6,0x86,0x40,1,0,0,0x3c,8},
    {0xf3,0x0f,0x5c,0x3d,0x53,0x4a,0x1d,0x01},
    {0x0f,0x28,0xc8,0x0f,0x57,0x0d,0xc5,0x33,0x28,0}};
static const unsigned stinger_lengths[4]={9,10,8,10};
static void stinger_resolve(const LiveImage *im) {
    unsigned i; uint64_t ctor;
    memset(&g_stinger,0,sizeof g_stinger);
    if(!all_valid(im->valid,0x97e6a8,8,im->size))return;
    memcpy(&ctor,im->bytes+0x97e6a8,8); /* weapon table: M9 + (7-1)*40 */
    if(ctor!=im->base+0x715bb0)return;
    for(i=0;i<4;i++)if(!m9_match(im,stinger_sites[i],stinger_bytes[i],stinger_lengths[i]))return;
    if(!m9_match(im,0x715c60,(const unsigned char *)"\x48\x89\xb3\xe8\0\0\0\x48\x89\xbb\xf0\0\0\0\x48\x89\xab\xf8\0\0\0",21) ||
       !m9_match(im,0x8913e,(const unsigned char *)"\x48\x8d\x15\x5b\x91\x4c\x01",7) ||
       !m9_match(im,0x4a0f54,(const unsigned char *)"\x48\x8d\x15\x65\x44\x1d\x01\x48\x8d\x0d\x6e\x44\x1d\x01",14))return;
    if(!m9_match(im,0x97e6b8,(const unsigned char *)"\x60\x40\0\0",4) ||
       !m9_match(im,0x53b5ea,(const unsigned char *)"\x4c\x8d\x8b\x94\x0b\0\0\xff\xd0",9))return;
    g_stinger.base=im->base;g_stinger.resolved=1;
}
/* Caller holds the controls lease. Pointer access is inside an SEH boundary.
   No dependence on draw-time matrices here: Act is still preparing them. */
static int stinger_context(uint64_t expected_actor,uint64_t *object_out) {



    ULONGLONG arm=0,player=0;int weapon=0;uint64_t object,actor,now=GetTickCount64();
    LONG age=(LONG)((DWORD)g_b.c_ticks-(DWORD)g_b.hand_command_tick);
    if((!dg_bridge_stinger_enabled()) || (!g_b.armed) || (g_b.s_late_unsafe) ||
       (g_b.fps.state!=DG_FPS_ACTIVE) || (!g_controls_allowed) ||
       (now<g_controls_lease) || (now-g_controls_lease>100) ||
       (!g_b.hand_command_requested) || (!g_b.hand_command_valid) || (age<0) || (age>DG_HAND_COMMAND_TICKS) ||
       (resolve_player(&arm,&player,&weapon)!=DG_RESOLVE_OK) || (weapon!=7))return 0;
    object=*(uint64_t *)(uintptr_t)(g_stinger.base+0x17df688);
    if(object<0x100a0)return 0;
    actor=object-0xa0;
    if(expected_actor && expected_actor!=actor)return 0;
    if((*(uint64_t *)(uintptr_t)(actor+0xe8)!=player+0xba8) ||
       (*(uint64_t *)(uintptr_t)(actor+0xf0)!=player+0xbb0) ||
       (*(uint64_t *)(uintptr_t)(actor+0xf8)!=player+0xb94) ||
       (*(int *)(uintptr_t)(player+0xbb0)!=6))return 0;
    object=*(uint64_t *)(uintptr_t)object;
    if((object<0x10000) || (*(uint64_t *)(uintptr_t)(object+0x40)!=0))return 0;



    *object_out=object;return 1;
}
/* Game-tick producer and actor consumers share the controls lease. A second
   lock protects the published mode from camera/context readers. */
static void stinger_tick(int safe) {
    uint64_t object=0,actor=0;DG_STINGER_INPUT in=g_controls_frame.stinger;
    int selected=0;
    __try {
        if(safe && g_controls_active && !g_b.script_menu_only &&
           !g_controls_special && g_b.a.player_pad && RD32(g_b.a.player_pad-4) &&
           stinger_context(0,&object)) {
            actor=*(uint64_t *)(uintptr_t)(g_stinger.base+0x17df688)-0xa0;
            selected=1;
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){InterlockedExchange(&g_stinger.fault,1);}
    in.allowed=in.allowed && selected;
    AcquireSRWLockExclusive(&g_stinger_mode_lock);
    g_stinger_claim=selected;
    dg_stinger_mode_step(&g_stinger_mode,&in,actor);
    ReleaseSRWLockExclusive(&g_stinger_mode_lock);
}
static int stinger_mode_now(uint64_t actor,int *scope) {
    uint64_t now=GetTickCount64();int valid;
    AcquireSRWLockShared(&g_stinger_mode_lock);
    valid=g_stinger_mode.active && actor==g_stinger_mode.actor &&
        now>=g_stinger_mode.stamp && now-g_stinger_mode.stamp<=100;
    *scope=valid && g_stinger_mode.scope;
    ReleaseSRWLockShared(&g_stinger_mode_lock);
    return valid;
}
static void stinger_model(void *original_rsp) {
    uint64_t *regs=(uint64_t *)original_rsp-16,object;int scope;
    if(!TryAcquireSRWLockShared(&g_controls_lock))return;
    __try {
        /* RCX is the local status; RDX the runtime subject mask. Set only
           these two Act tests, so no FPS toggle edge leaks into gameplay. */
        if(stinger_context(regs[1],&object) && stinger_mode_now(regs[1],&scope) && (uint32_t)regs[12]) {
            if(scope)regs[13]&=~(uint64_t)(uint32_t)regs[12];
            else regs[13]|=(uint32_t)regs[12];
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){InterlockedExchange(&g_stinger.fault,1);}
    ReleaseSRWLockShared(&g_controls_lock);
}
static void stinger_score_impl(void *original_rsp,float *saved67,int existing) {
    uint64_t object,actor; int axis; float root[4][4],projection[4][4],target[4],display[2],delta[2];
    (void)original_rsp;
    if(!TryAcquireSRWLockShared(&g_controls_lock))return;
    __try {
        if(stinger_context(0,&object)) {
            memcpy(root,(void *)(uintptr_t)object,sizeof root);
            actor=*(uint64_t *)(uintptr_t)(g_stinger.base+0x17df688)-0xa0;



            if(*(unsigned char *)(uintptr_t)(actor+0x140)==0)
                for(axis=0;axis<3;axis++)root[3][axis]-=1000.f*root[1][axis];
            memcpy(projection,(void *)(uintptr_t)(g_stinger.base+0x15522a0),sizeof projection);
            memcpy(target,(void *)(uintptr_t)(g_stinger.base+0x16753c0),sizeof target);
            memcpy(display,(void *)(uintptr_t)(g_stinger.base+0x16753d0),sizeof display);
            dg_stinger_delta(root,projection,target,display,delta);
            saved67[0]=delta[0];saved67[4]=delta[1]+(existing?display[1]:0);
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){InterlockedExchange(&g_stinger.fault,1);}
    ReleaseSRWLockShared(&g_controls_lock);
}
static void stinger_score(void *rsp,float *saved){stinger_score_impl(rsp,saved,0);}
static void stinger_existing(void *rsp,float *saved){stinger_score_impl(rsp,saved,1);}
static void stinger_stop(void) {
    unsigned i;InterlockedExchange(&g_stinger.live,0);stinger_revoke();
    for(i=0;i<4;i++)dg_detour_remove(&g_stinger_detour[i]);
}
static void stinger_install(void) {
    unsigned i;const char *why="retail witnesses unavailable";
    if(g_stinger.resolved)for(i=0;i<4;i++) {
        if(!dg_detour_install_ex(&g_stinger_detour[i],(void *)(uintptr_t)(g_stinger.base+stinger_sites[i]),
           i<2?(void *)stinger_model:(i==2?(void *)stinger_existing:(void *)stinger_score),
           (void *)(uintptr_t)(g_stinger.base+(i<2?0x7151f0:0x4a0800)),
           (void *)(uintptr_t)(g_stinger.base+(i<2?0x715840:0x4a0d10)),&why,i<2?1:3))break;
    }
    else i=0;
    if(i==4)InterlockedExchange(&g_stinger.live,1);else stinger_stop();
    if(g_b.log)g_b.log("  Stinger FPS: controller model and native lock-on %s (%s)\r\n",
                         g_stinger.live?"installed":"unavailable",why);
}
