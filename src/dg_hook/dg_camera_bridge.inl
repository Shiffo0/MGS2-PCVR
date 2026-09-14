#include "dg_camera_shutter.h"
#include "dg_camera_resolve.inl"
static DG_CAMERA_ANCHORS g_camera_a;
static DG_DETOUR g_camera_detour;
static DG_CAMERA_SHUTTER g_camera_owner;
static SRWLOCK g_camera_lock=SRWLOCK_INIT;
static uint64_t g_camera_manager,g_camera_actor,g_camera_seen,g_camera_conflict;
static volatile LONG g_camera_fire_latch,g_camera_pulses,g_camera_lost,g_camera_native_shutters;
static unsigned short g_camera_state=99;
static int camera_inventory(dg_radial_inventory_snapshot *s) {
    return (dg_radial_inventory_sample(&g_radial_game.image,&g_radial_game.inventory,s)&
        (DG_RINV_ACTOR|DG_RINV_DESIRED))==(DG_RINV_ACTOR|DG_RINV_DESIRED);
}
/* Observes the manager already dispatched by the game. The saved RCX is
 * at original RSP-24 (flags,RAX,RCX). No callback/state writes. */
static void camera_observe_callback(void *sp) {
    uint64_t manager,fn,des;int item;unsigned short state;dg_radial_inventory_snapshot inv;
    uint64_t now=GetTickCount64();
    if(!g_camera_a.valid || !interact_read(NULL,(uint64_t)(ULONG_PTR)sp-24,&manager,8) ||
       !camera_inventory(&inv) || !dg_camera_item(inv.actor_item) ||
       !interact_read(NULL,manager+8,&fn,8) || fn!=g_camera_a.callback ||
       !interact_read(NULL,manager+0x20,&des,8) || des!=g_camera_a.destroy ||
       !interact_read(NULL,manager+0xf0,&item,4) || item!=inv.actor_item ||
       !interact_read(NULL,manager+0xd8,&state,2))return;
    if(!TryAcquireSRWLockExclusive(&g_camera_lock)){InterlockedExchange(&g_camera_lost,1);return;}
    if(g_camera_manager && manager!=g_camera_manager && now>=g_camera_seen && now-g_camera_seen<=100)
        g_camera_conflict=now+100;
    if(manager==g_camera_manager && g_camera_state==0 && state==1)
        InterlockedIncrement(&g_camera_native_shutters);
    g_camera_state=state;
    g_camera_manager=manager;g_camera_actor=inv.player_identity;g_camera_seen=now;
    ReleaseSRWLockExclusive(&g_camera_lock);
}
/* Called only under ACTION's pre-mask lock: one owner of stack status. */
static unsigned camera_pre_mask(const DG_ACTION_SAMPLE *p,unsigned raw,unsigned flags,
                                unsigned pm,unsigned sm,int lease) {
    DG_CAMERA_INPUT in;dg_radial_inventory_snapshot inv,again;
    uint64_t fn,des,pad,pad2,manager=0,actor=0,seen=0,conflict=0;
    unsigned mask=0,pause=1;unsigned short state=99,time=0;
    int item=-1,child=0x80,frames=1,channel=-1;unsigned result;
    memset(&in,0,sizeof in);
    in.sample=p->sample;in.source=p->source;in.now_ms=p->now_ms;in.age_ms=p->age_ms;
    in.valid=p->valid;in.down=p->shutter_down;in.denied=p->denied||p->shutter_denied;
    if(InterlockedExchange(&g_camera_lost,0))in.denied=1;
    if(!camera_inventory(&inv))goto step;
    in.actor=inv.player_identity;in.item=inv.actor_item;in.desired_item=inv.desired_item;
    if(!g_camera_a.valid || !g_camera_detour.installed || !dg_camera_item(in.item))goto step;
    if(!TryAcquireSRWLockShared(&g_camera_lock)){in.denied=1;goto step;}
    manager=g_camera_manager;actor=g_camera_actor;seen=g_camera_seen;conflict=g_camera_conflict;
    ReleaseSRWLockShared(&g_camera_lock);
    in.manager=manager;
    if(!manager || actor!=in.actor || in.now_ms<seen || in.now_ms-seen>100 || in.now_ms<conflict ||
       !interact_read(NULL,manager+8,&fn,8)||fn!=g_camera_a.callback ||
       !interact_read(NULL,manager+0x20,&des,8)||des!=g_camera_a.destroy ||
       !interact_read(NULL,manager+0xd0,&pad,8)||pad!=g_camera_a.pad ||
       !interact_read(NULL,g_camera_a.channel,&channel,4)||channel!=0 ||
       !interact_read(NULL,g_camera_a.pause,&pause,4)||pause ||
       !interact_read(NULL,g_camera_a.mask,&mask,4) ||
       !interact_read(NULL,manager+0xd8,&state,2)||
       !interact_read(NULL,manager+0xda,&time,2)||
       !interact_read(NULL,manager+0xe0,&child,4)||
       !interact_read(NULL,manager+0xf8,&frames,4)||
       !interact_read(NULL,manager+0xf0,&item,4)||item!=in.item ||
       !camera_inventory(&again)||again.player_identity!=in.actor ||
       again.actor_item!=in.item || again.desired_item!=in.desired_item ||
       !interact_read(NULL,manager+0xd0,&pad2,8)||pad2!=pad)goto step;
    /* DIRECT_TICK(30) is either 25 or 30 in the verified helper.
     * Require >=30 before native increments it; no wall-clock guess. */
    in.ready=state==0 && time>=30 && child!=0x80 && frames<=0;
    in.mask=mask;in.native_down=(raw&mask)!=0;
    in.pad_allowed=lease && !(flags&(0x103|0x1000|0x4000|0x30)) &&
        (!(flags&4)||(pm&mask)) && (!(flags&8)||(sm&mask));
step:
    result=dg_camera_shutter_step(&g_camera_owner,&in);
    if(result)InterlockedIncrement(&g_camera_pulses);
    return result;
}
/* Camera ownership includes opening/saving/equip transitions. A held trigger
 * cannot fall through to a weapon after unequip or observer loss. */
static int camera_fire_block(int valid,double value) {
    dg_radial_inventory_snapshot inv;int camera=0;
    if(camera_inventory(&inv))camera=dg_camera_item(inv.actor_item)||dg_camera_item(inv.desired_item);
    else if(g_camera_a.valid || InterlockedCompareExchange(&g_camera_fire_latch,0,0))return 1;
    if(camera){InterlockedExchange(&g_camera_fire_latch,1);return 1;}
    if(InterlockedCompareExchange(&g_camera_fire_latch,0,0)) {
        if(valid && value>=0.0 && value<0.10)InterlockedExchange(&g_camera_fire_latch,0);
        return 1;
    }
    return 0;
}
static void camera_resolve(const LiveImage *im) {
    camera_resolve_image(im,g_b.a.gv_pad_data,g_b.a.pad_weapon,&g_camera_a);
    if(g_b.log)g_b.log("  camera shutter anchors: %s\r\n",g_camera_a.valid?"resolved":"unavailable");
}
static void camera_install(void) {
    const char *why="anchors or shared input unavailable";
    if(g_camera_a.valid && g_action_detour.installed && dg_detour_install_ex(&g_camera_detour,
       (void*)(ULONG_PTR)g_camera_a.callback,(void*)camera_observe_callback,
       (void*)(ULONG_PTR)g_camera_a.callback,(void*)(ULONG_PTR)g_camera_a.end,&why,1)) {
        if(g_b.log)g_b.log("  camera shutter observer installed; NORMAL port 0\r\n");
    } else if(g_b.log)g_b.log("  camera shutter disabled: %s\r\n",why);
}
