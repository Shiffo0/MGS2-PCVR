#include "dg_camera_zoom.h"
#include "dg_zoom_resolve.inl"
static DG_ZOOM_ANCHORS g_zoom_a;
static DG_DETOUR g_zoom_detour;
static DG_CAMERA_ZOOM g_zoom_owner,g_psg_zoom_owner;
static volatile LONG g_zoom_direction,g_zoom_applied;
int dg_bridge_camera_stick_owned(void) {
    dg_radial_inventory_snapshot inv;
    return camera_inventory(&inv) && (dg_camera_item(inv.actor_item)||dg_camera_item(inv.desired_item));
}
/* Called under action lock. Revalidate the actual manager->zoom instance,
 * the observer lease, actor, parent, native state and all pad restrictions. */
static int zoom_observe(uint64_t expected,uint64_t now,uint64_t *manager_out,
                        uint64_t *camera_out,unsigned *om,unsigned *imask) {
    uint64_t manager,actor,seen,conflict,zoom,fn,des,pad,parent,parent2,cam,check;
    dg_radial_inventory_snapshot inv,again;
    unsigned pause,flags,pm,sm;int item,channel,child,frames,oi,ii,on;unsigned short state,timer;
    if(!g_zoom_a.valid || !g_zoom_detour.installed || !g_action_enabled || !g_action_lease ||
       now<g_action_lease || now-g_action_lease>100 || !camera_inventory(&inv) ||
       !dg_camera_item(inv.actor_item)||inv.actor_item!=inv.desired_item)return 0;
    if(!TryAcquireSRWLockShared(&g_camera_lock))return 0;
    manager=g_camera_manager;actor=g_camera_actor;seen=g_camera_seen;conflict=g_camera_conflict;
    ReleaseSRWLockShared(&g_camera_lock);
    if(!manager||actor!=inv.player_identity || now<seen || now-seen>100 || now<conflict ||
       !interact_read(NULL,manager+8,&fn,8)||fn!=g_camera_a.callback ||
       !interact_read(NULL,manager+0x20,&des,8)||des!=g_camera_a.destroy ||
       !interact_read(NULL,manager+0x100,&zoom,8)||!zoom||(expected && zoom!=expected)||
       !interact_read(NULL,zoom+8,&fn,8)||fn!=g_zoom_a.callback ||
       !interact_read(NULL,zoom+0x68,&pad,8)||pad!=g_camera_a.pad ||
       !interact_read(NULL,zoom+0x60,&parent,8)||!parent ||
       !interact_read(NULL,g_zoom_a.parent_slot,&parent2,8)||parent2!=parent ||
       !interact_read(NULL,parent+0x2c,&on,4)||!on ||
       !interact_read(NULL,zoom+0x58,&cam,8)||!cam ||
       !interact_read(NULL,manager+0xf0,&item,4)||item!=inv.actor_item ||
       !interact_read(NULL,manager+0xd8,&state,2)||state ||
       !interact_read(NULL,manager+0xda,&timer,2)||timer<30 ||
       !interact_read(NULL,manager+0xe0,&child,4)||child==0x80 ||
       !interact_read(NULL,manager+0xf8,&frames,4)||frames>0 ||
       !interact_read(NULL,g_camera_a.channel,&channel,4)||channel ||
       !interact_read(NULL,g_camera_a.pause,&pause,4)||pause ||
       !interact_read(NULL,g_zoom_a.out_mask,om,4)||!*om||(*om&(*om-1)) ||
       !interact_read(NULL,g_zoom_a.in_mask,imask,4)||!*imask||(*imask&(*imask-1))||*om==*imask ||
       !interact_read(NULL,g_zoom_a.out_index,&oi,4)||oi<0||oi>=12 ||
       !interact_read(NULL,g_zoom_a.in_index,&ii,4)||ii<0||ii>=12||oi==ii ||
       !interact_read(NULL,pad+36,&flags,4)||(flags&(0x103|0x1000|0x4000|0x30)) ||
       !interact_read(NULL,g_action_a.mask_prg,&pm,4)||
       !interact_read(NULL,g_action_a.mask_scn,&sm,4)||
       ((flags&4)&&((pm&(*om|*imask))!=(*om|*imask))) ||
       ((flags&8)&&((sm&(*om|*imask))!=(*om|*imask))))return 0;
    if(!camera_inventory(&again)||again.player_identity!=actor||again.actor_item!=item||again.desired_item!=item||
       !interact_read(NULL,manager+0x100,&check,8)||check!=zoom)return 0;
    *manager_out=manager;*camera_out=cam;return 1;
}
/* Stub saved GPR layout relative to original RSP: RSI -56, RBP -48,
 * R12 -104, R13 -112. Write only these local restored register values. */
static void zoom_consumer(void *original_rsp) {
    unsigned char *sp=original_rsp;uint64_t work,manager=0,cam=0,actor=0;unsigned om=0,imask=0,shutter=0,status=0;
    DG_ACTION_SAMPLE p={0};int allowed=0,hatch=0,direction,psg=0;uint64_t now=GetTickCount64();
    if(!TryAcquireSRWLockExclusive(&g_action_lock)){InterlockedExchange(&g_action_lost,1);return;}
    if(g_action_provider)g_action_provider(&p);



    if(interact_read(NULL,(uint64_t)(ULONG_PTR)sp-56,&work,8) &&
       psg_zoom_observe(work,&manager,&cam,&om,&imask)) {
        psg=1;allowed=p.valid&&!p.denied&&!p.shutter_denied;








    }
    if(!psg && interact_read(NULL,(uint64_t)(ULONG_PTR)sp-56,&work,8) &&
       zoom_observe(work,now,&manager,&cam,&om,&imask) && action_observe(&actor,&hatch) && !hatch &&
       interact_read(NULL,g_camera_a.mask,&shutter,4) &&
       interact_read(NULL,g_camera_a.pad+4,&status,4))
        allowed=p.valid&&p.zoom_active&&!p.denied&&!p.shutter_denied&&!p.shutter_down&&!(status&shutter);
    if(InterlockedExchange(&g_action_lost,0))allowed=0;
    direction=psg ? dg_psg_zoom_step(&g_psg_zoom_owner,p.sample,p.source,manager,now,p.age_ms,
                        allowed,(float)p.psg_grip_zoom)
                  : dg_camera_zoom_step(&g_zoom_owner,p.sample,p.source,manager,now,p.age_ms,
                        allowed,p.zoom_y);
    if(!psg)memset(&g_psg_zoom_owner,0,sizeof g_psg_zoom_owner);
    /* Retail out_mask branch ADDS focal length; in_mask SUBTRACTS it.
       Grip-in means greater magnification, regardless of those legacy names. */
    if(psg)direction=-direction;





    if(direction){
        dg_camera_zoom_merge(direction,om,imask,(uint64_t*)(sp-48),(uint64_t*)(sp-112),(uint64_t*)(sp-104));
        InterlockedIncrement(&g_zoom_applied);
    }
    InterlockedExchange(&g_zoom_direction,direction);
    ReleaseSRWLockExclusive(&g_action_lock);
}
/* Render observer: identity + native angle, never a native camera write. */
int dg_bridge_zoom_now(uint64_t *identity,float *angle) {
    uint64_t manager=0,cam=0;unsigned om=0,imask=0;int ok=0;
    if(!TryAcquireSRWLockShared(&g_action_lock))return 0;
    if(zoom_observe(0,GetTickCount64(),&manager,&cam,&om,&imask) &&
       interact_read(NULL,cam+0x20,angle,4)&&*angle>=2 && *angle<=24){*identity=manager;ok=1;}
    ReleaseSRWLockShared(&g_action_lock);return ok;
}
static void zoom_install(void) {
    const char *why="anchors unavailable";
    if(g_zoom_a.valid && g_camera_detour.installed && dg_detour_install_ex(&g_zoom_detour,
       (void*)(ULONG_PTR)g_zoom_a.seam,(void*)zoom_consumer,
       (void*)(ULONG_PTR)g_zoom_a.callback,(void*)(ULONG_PTR)g_zoom_a.end,&why,1)) {
        if(g_b.log)g_b.log("  camera zoom consumer installed; local status/pressure only\r\n");
    }else if(g_b.log)g_b.log("  camera zoom disabled: %s\r\n",why);
}
