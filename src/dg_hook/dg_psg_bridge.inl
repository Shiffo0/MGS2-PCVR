
#include "dg_psg_aim.h"
static DG_DETOUR g_psg_detour;
static SRWLOCK g_psg_lock=SRWLOCK_INIT;
static struct { uint64_t base; int resolved; volatile LONG live,fault; } g_psg;
static struct {
    int valid,weapon; uint64_t actor,player,scope,arm_camera,ms;
    LONG tick,entry; float shot[4][4];
} g_psg_camera;
static volatile LONG g_psg_applied,g_psg_refused;
static const unsigned char psg_seam[]={0x41,0x8b,0x86,0x24,1,0,0,0x48,0x8d,0x4d,0xe7};






static int psg_context(uint64_t expected,uint64_t *actor_out,uint64_t *player_out,
                       uint64_t *scope_out,uint64_t *camera_out,int *weapon_out) {



    ULONGLONG player,arm; LONG weapon; DG_CAMERA_GATE gate;uint64_t actor,object,scope,now=GetTickCount64();
    if((!g_psg.live) || (g_psg.fault) || (!g_b.armed) || (g_b.owner) || (g_b.script_menu_only) ||
       (g_b.s_late_unsafe) || (!g_controls_allowed) || (now<g_controls_lease) || (now-g_controls_lease>100) ||
       (!dg_bridge_camera_gate_now(&gate)) ||
       (resolve_player(&arm,&player,&weapon)!=DG_RESOLVE_OK) || ((weapon!=4 && weapon!=19)))return 0;
    object=*(uint64_t *)(uintptr_t)(g_psg.base+0x17df688);
    if(object<0x100a0)return 0;
    actor=object-0xa0;
    if(expected && expected!=actor)return 0;
    if((*(uint64_t *)(uintptr_t)(actor+0xe8)!=player+0xba8) ||
       (*(uint64_t *)(uintptr_t)(actor+0xf0)!=player+0xbb0) ||
       ((*(uint64_t *)(uintptr_t)(actor+0xf8)!=player+0xb94 && *(uint64_t *)(uintptr_t)(actor+0xf8)!=player+0xbe0)) ||
       (*(int *)(uintptr_t)(actor+0x120)!=weapon) ||
       (*(int *)(uintptr_t)(actor+0x124)!=(weapon==4?4:1)) ||
       (!*(uint64_t *)(uintptr_t)(actor+0x110)))return 0;
    scope=*(uint64_t *)(uintptr_t)(actor+0x108);
    if((!plausible_ptr(scope)) || (region_end(scope)<scope+0x98) || (!RD32(scope+0x2c)))return 0;
    *actor_out=actor;*player_out=player;*scope_out=scope;*camera_out=gate.camera;*weapon_out=(int)weapon;
    return 1;
}
void dg_bridge_psg_camera(const float world[4][4]) {
    uint64_t actor,player,scope,camera;int weapon;float shot[4][4];
    /* Invalidate without waiting in the camera exception handler. */
    InterlockedExchange((volatile LONG *)&g_psg_camera.valid,0);
    if(!world)return;
    if(!dg_psg_matrix(world,shot) || !TryAcquireSRWLockShared(&g_controls_lock))return;
    __try {
        if(psg_context(0,&actor,&player,&scope,&camera,&weapon) && TryAcquireSRWLockExclusive(&g_psg_lock)) {
            g_psg_camera.valid=0;
            g_psg_camera.actor=actor;g_psg_camera.player=player;g_psg_camera.scope=scope;
            g_psg_camera.arm_camera=camera;g_psg_camera.weapon=weapon;
            g_psg_camera.ms=GetTickCount64();g_psg_camera.tick=g_b.c_ticks;
            g_psg_camera.entry=g_b.c_explicit_entries;
            memcpy(g_psg_camera.shot,shot,sizeof shot);
            InterlockedExchange((volatile LONG *)&g_psg_camera.valid,1);
            ReleaseSRWLockExclusive(&g_psg_lock);
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){InterlockedExchange(&g_psg.fault,1);}
    ReleaseSRWLockShared(&g_controls_lock);
}
static void psg_shot(void *original_rsp) {
    /* Saved stack order is r15,r14,r13,...,rbp,...,rax,flags. */
    uint64_t *regs=(uint64_t *)original_rsp-16,actor,player,scope,camera,now;
    int weapon=0;LONG age,count=0;float *dst;int applied=0;
    float native[4][4],used[4][4];
    if(!TryAcquireSRWLockShared(&g_controls_lock))return;
    if(!TryAcquireSRWLockShared(&g_psg_lock)){ReleaseSRWLockShared(&g_controls_lock);return;}
    __try {
        now=GetTickCount64();age=(LONG)((DWORD)g_b.c_ticks-(DWORD)g_psg_camera.tick);
        if(g_psg_camera.valid && now>=g_psg_camera.ms && now-g_psg_camera.ms<=50 && age>=0 && age<=2 &&
           g_psg_camera.entry==g_b.c_explicit_entries &&
           psg_context(regs[1],&actor,&player,&scope,&camera,&weapon) &&
           actor==g_psg_camera.actor && player==g_psg_camera.player && scope==g_psg_camera.scope &&
           camera==g_psg_camera.arm_camera && weapon==g_psg_camera.weapon) {
            dst=(float *)(uintptr_t)(regs[10]-0x19); /* native local world */
            /* Stack window from the verified Act prologue, never an arbitrary pointer. */
            if((uintptr_t)dst==(uintptr_t)original_rsp+0x50) {
                memcpy(native,dst,sizeof native);memcpy(used,g_psg_camera.shot,sizeof used);
                memcpy(dst,used,sizeof used);applied=1;
            }
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){InterlockedExchange(&g_psg.fault,1);}
    if(applied)count=InterlockedIncrement(&g_psg_applied);else InterlockedIncrement(&g_psg_refused);
    ReleaseSRWLockShared(&g_psg_lock);ReleaseSRWLockShared(&g_controls_lock);
    /* Bounded support evidence, no render-loop logging or player input data. */
    if(applied && count<=8 && g_b.log)g_b.log(
        "  PSG head shot: weapon=%d actor=%llx age_ticks=%ld native_dir=%.5f,%.5f,%.5f head_dir=%.5f,%.5f,%.5f origin_delta=%.2f,%.2f,%.2f\r\n",
        weapon,(unsigned long long)actor,age,-native[1][0],-native[1][1],-native[1][2],
        -used[1][0],-used[1][1],-used[1][2],used[3][0]-native[3][0],used[3][1]-native[3][1],used[3][2]-native[3][2]);
}
static void psg_resolve(const LiveImage *im) {
    uint64_t ctor;unsigned ids[2]={4,19},targets[2]={0x713bd0,0x713c00};unsigned i;
    memset(&g_psg,0,sizeof g_psg);dg_bridge_psg_camera(NULL);
    for(i=0;i<2;i++) {
        unsigned at=0x97e5b8+(ids[i]-1)*40;
        if(!all_valid(im->valid,at,8,im->size))return;
        memcpy(&ctor,im->bytes+at,8);if(ctor!=im->base+targets[i])return;
    }
    if(!m9_match(im,0x713480,(const unsigned char *)"\x48\x89\x5c\x24\x10\x48\x89\x74\x24\x18\x55\x57\x41\x56\x48\x8d\x6c\x24\xb9\x48\x81\xec\xb0\0\0\0",26) ||
       !m9_match(im,0x713b1a,(const unsigned char *)"\x48\x89\x3d\x6f\xbb\x0c\x01\x48\x89\x35\x60\xbb\x0c\x01",14) ||
       !m9_match(im,0x713b4a,(const unsigned char *)"\x89\x83\x24\x01\0\0\x8b\x44\x24\x60\x89\x83\0\x01\0\0\x89\x8b\x20\x01\0\0",22) ||
       !m9_match(im,0x713507,(const unsigned char *)"\x49\x83\xbe\x10\x01\0\0\0",8) ||
       !m9_match(im,0x713895,psg_seam,sizeof psg_seam) ||
       !m9_match(im,0x7138ca,(const unsigned char *)"\xe8\x11\x7e\xdb\xff",5) ||
       !m9_match(im,0x713807,(const unsigned char *)"\x49\x8b\x8e\x08\x01\0\0",7) ||
       !m9_match(im,0x713813,(const unsigned char *)"\x48\x8b\x81\x90\0\0\0",7) ||
       !m9_match(im,0x71353b,(const unsigned char *)"\x48\x8b\x15\x5e\xc1\x0c\x01",7) || /* rdx=[0x17df6a0] zoom parent */
       !m9_match(im,0x713b67,(const unsigned char *)"\x4c\x89\xbb\xe8\0\0\0\x4c\x89\xb3\xf0\0\0\0\x48\x89\xab\xf8\0\0\0",21))return;
    g_psg.base=im->base;g_psg.resolved=1;
}
static void psg_stop(void) {
    InterlockedExchange(&g_psg.live,0);dg_bridge_psg_camera(NULL);dg_detour_remove(&g_psg_detour);
}
static void psg_install(void) {
    const char *why="retail witnesses unavailable";
    if(g_psg.resolved && dg_detour_install_ex(&g_psg_detour,(void *)(uintptr_t)(g_psg.base+0x713895),
       (void *)psg_shot,(void *)(uintptr_t)(g_psg.base+0x713480),
       (void *)(uintptr_t)(g_psg.base+0x71398d),&why,1))InterlockedExchange(&g_psg.live,1);
    if(g_b.log)g_b.log("  PSG head aim: %s (%s)\r\n",g_psg.live?"installed":"unavailable",why);
}
