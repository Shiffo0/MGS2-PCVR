#include "dg_action_resolve.inl"
static DG_ACTION_ANCHORS g_action_a;
static DG_DETOUR g_action_detour;
static DG_ACTION_OWNER g_action_owner;
static SRWLOCK g_action_lock=SRWLOCK_INIT;
static DG_ACTION_PROVIDER g_action_provider;
static uint64_t g_action_lease;
static volatile LONG g_action_enabled;
static volatile LONG g_action_lost;
#include "dg_camera_bridge.inl"
static int action_observe(uint64_t *actor,int *hatch);
#include "dg_zoom_bridge.inl"
typedef struct {DG_ACTION_INPUT in;unsigned raw,bits,flags;int blocked;} DA_LOG;
static DA_LOG g_action_logs[16];
static unsigned g_action_log_write,g_action_log_read;
static unsigned g_action_log_bits;
static int g_action_log_context=-1,g_action_log_down=-1;

/* Identity and relevant fields are sampled twice. No callback or actor writes. */
static int action_observe(uint64_t *actor,int *hatch) {
    uint64_t id,id2,fn,fn2,pad,pad2,player,late;
    unsigned game,gs,menu,ms;int weapon,weapon2,data,data2;LONG age,samples;
    *actor=0;*hatch=0;
    if(!g_action_a.valid || !g_b.armed || g_b.script_menu_only ||
       !interact_player_now(&id,&weapon) || id>UINT64_MAX-0x10000 ||
       !interact_read(NULL,id+g_action_a.action_off,&fn,8) ||
       !interact_read(NULL,id+g_action_a.data_off,&data,4) ||
       !interact_read(NULL,id+g_action_a.pad_off,&pad,8) ||
       !interact_read(NULL,g_b.a.gm_player_status,&player,8) ||
       !interact_read(NULL,g_b.a.gm_game_status,&game,4) ||
       !interact_read(NULL,g_b.a.gm_game_status_scn,&gs,4) ||
       !interact_read(NULL,g_b.a.gm_menu_status,&menu,4) ||
       !interact_read(NULL,g_b.a.gm_menu_status_scn,&ms,4) ||
       !g_b.a.player_pad || pad!=g_b.a.player_pad || !RD32(pad-4))return 0;
    game|=gs;menu|=ms;
    samples=InterlockedCompareExchange(&g_b.c_late_status,0,0);
    age=(LONG)((DWORD)g_b.c_ticks-(DWORD)g_b.late_tick);
    late=((uint64_t)(DWORD)g_b.s_late_player_hi<<32)|(DWORD)g_b.s_late_player_lo;
    fold_late_status(&game,&menu,&player,(unsigned)g_b.s_late_game,
        (unsigned)g_b.s_late_menu,late,samples,age);
    if((game&DG_GAME_UNSAFE_MASK)||(menu&DG_MENU_UNSAFE_MASK))return 0;
    if(fn==g_action_a.hold && (data==0 || data==1)) {
        if(player&(DG_PLAYER_UNSAFE_MASK&~UINT64_C(0x2000)))return 0;
        *hatch=data?3:2;
    } else if(fn==g_action_a.not_open ||
              (player&(DG_PLAYER_UNSAFE_MASK&~UINT64_C(0x10000)))) return 0;
    /* Plain ladder admits native X to its scenario endpoint trigger.
       FORCE/transition/damage gates above remain closed. */
    if(!interact_player_now(&id2,&weapon2)||id2!=id ||
       !interact_read(NULL,id+g_action_a.action_off,&fn2,8)||fn!=fn2 ||
       !interact_read(NULL,id+g_action_a.data_off,&data2,4)||data!=data2 ||
       !interact_read(NULL,id+g_action_a.pad_off,&pad2,8)||pad!=pad2)return 0;
    *actor=id;return 1;
}
int dg_bridge_action_ladder_now(void) {
    uint64_t actor,player;int hatch;
    return g_action_enabled && action_observe(&actor,&hatch) && !hatch &&
        interact_read(NULL,g_b.a.gm_player_status,&player,8) &&
        (player&UINT64_C(0x10000))!=0;
}
int dg_bridge_hatch_now(void) {
    uint64_t actor;int hatch;
    if(!InterlockedCompareExchange(&g_action_enabled,0,0))return 0;
    return action_observe(&actor,&hatch)?hatch:0;
}
void dg_bridge_action_register(DG_ACTION_PROVIDER provider) {
    AcquireSRWLockExclusive(&g_action_lock);
    InterlockedExchange(&g_action_enabled,0);
    g_action_provider=provider;g_action_lease=0;
    memset(&g_action_owner,0,sizeof g_action_owner);
    memset(&g_camera_owner,0,sizeof g_camera_owner);
    memset(&g_zoom_owner,0,sizeof g_zoom_owner);
    if(provider && g_action_detour.installed)InterlockedExchange(&g_action_enabled,1);
    ReleaseSRWLockExclusive(&g_action_lock);
}
void dg_bridge_action_context(int allowed) {
    AcquireSRWLockExclusive(&g_action_lock);
    g_action_lease=allowed?GetTickCount64():0;
    /* A complete revoked interval may fall between two native pad updates. */
    if(!allowed){g_action_owner.held=0;g_action_owner.blocked=1;g_camera_owner.blocked=1;g_zoom_owner.blocked=1;g_zoom_owner.direction=0;}
    ReleaseSRWLockExclusive(&g_action_lock);
}
/* Common Direct-update return runs even when NORMAL takes its demo branch.
   This is revocation only: never inject into Direct or scenario input. */
static void action_update_boundary(void) {
    unsigned flags;
    if(!InterlockedCompareExchange(&g_action_enabled,0,0))return;
    if(!TryAcquireSRWLockExclusive(&g_action_lock)){InterlockedExchange(&g_action_lost,1);return;}
    if(!interact_read(NULL,g_b.a.gv_pad_data+36,&flags,4) ||
       (flags&(0x103|0x1000|0x4000|0x30))) {
        g_action_owner.held=0;g_action_owner.blocked=1;
        g_camera_owner.blocked=1;g_zoom_owner.blocked=1;g_zoom_owner.direction=0;
    }
    ReleaseSRWLockExclusive(&g_action_lock);
}
static void action_pre_mask(void *original_rsp) {
    DG_ACTION_INPUT in;uint64_t now,actor=0;int hatch=0;unsigned raw,flags,pm,sm,bits=0;
    unsigned char *stack=(unsigned char *)original_rsp;
    if(!TryAcquireSRWLockExclusive(&g_action_lock)){InterlockedExchange(&g_action_lost,1);return;}
    memset(&in,0,sizeof in);now=GetTickCount64();
    if(!g_action_enabled || !g_action_provider)goto done;
    g_action_provider(&in.physical);in.physical.now_ms=now;
    in.physical.denied|=InterlockedExchange(&g_action_lost,0)!=0;
    if(action_observe(&actor,&hatch))in.context=hatch?hatch:1;
    in.physical.actor=actor;
    /* This resolved site belongs to NORMAL port zero; its demo branch skips
       this callback. Reads here still guard against a mid-update flag change. */
    if(!interact_read(NULL,(uint64_t)(ULONG_PTR)(stack+0x24),&raw,4) ||
       !interact_read(NULL,g_b.a.gv_pad_data+36,&flags,4) ||
       !interact_read(NULL,g_action_a.mask_prg,&pm,4) ||
       !interact_read(NULL,g_action_a.mask_scn,&sm,4))goto revoke;
    in.native_down=(raw&0x10)!=0;
    in.pad_allowed=g_action_lease && now>=g_action_lease && now-g_action_lease<=100 &&
        !(flags&(0x103|0x1000|0x4000|0x30)) && (!(flags&4)||(pm&0x10)) && (!(flags&8)||(sm&0x10));
    bits=dg_action_step(&g_action_owner,&in);
    bits|=camera_pre_mask(&in.physical,raw,flags,pm,sm,
        in.context==1 && g_action_lease && now>=g_action_lease && now-g_action_lease<=100);
    if(bits) *(volatile unsigned *)(stack+0x24)=raw|bits;
    if(bits!=g_action_log_bits || in.context!=g_action_log_context || in.physical.down!=g_action_log_down) {
        DA_LOG *v=&g_action_logs[g_action_log_write++%16];
        v->in=in;v->raw=raw;v->bits=bits;v->flags=flags;v->blocked=g_action_owner.blocked;
        if(g_action_log_write-g_action_log_read>16)g_action_log_read=g_action_log_write-16;
        g_action_log_bits=bits;g_action_log_context=in.context;g_action_log_down=in.physical.down;
    }
    goto done;
revoke:
    g_camera_owner.blocked=1;g_zoom_owner.blocked=1;g_zoom_owner.direction=0;
    in.pad_allowed=0;dg_action_step(&g_action_owner,&in);
done:
    ReleaseSRWLockExclusive(&g_action_lock);
}
/* Drain at the existing game tick, never from the pre-mask callback. */
static void action_log_drain(void) {
    DA_LOG rows[16];unsigned n=0,i;
    LONG zooms=InterlockedExchange(&g_zoom_applied,0);
    LONG shots=InterlockedExchange(&g_camera_pulses,0);
    if(zooms && g_b.log)g_b.log("  camera zoom input: direction=%ld consumer_updates=%ld\r\n",g_zoom_direction,zooms);
    LONG accepted=InterlockedExchange(&g_camera_native_shutters,0);
    if(shots && g_b.log)g_b.log("  camera shutter: native status pulses %ld (not photo ACK)\r\n",shots);
    if(accepted && g_b.log)g_b.log("  camera shutter: native state 0->1 observed %ld (physical or VR input; not save ACK)\r\n",accepted);
    if(!TryAcquireSRWLockExclusive(&g_action_lock))return;
    while(g_action_log_read!=g_action_log_write && n<16)
        rows[n++]=g_action_logs[g_action_log_read++%16];
    ReleaseSRWLockExclusive(&g_action_lock);
    if(g_b.log)for(i=0;i<n;i++){
        const DA_LOG *v=&rows[i];
        g_b.log("  action NORMAL: ctx=%d down=%d valid=%d deny=%d blocked=%d raw=%X add=%X flags=%X actor=%p\r\n",
            v->in.context,v->in.physical.down,v->in.physical.valid,v->in.physical.denied,
            v->blocked,v->raw,v->bits,v->flags,(void *)(ULONG_PTR)v->in.physical.actor);
    }
}
static void action_resolve(const LiveImage *im) {
    da_resolve(im,g_b.a.gv_pad_data,&g_action_a);
    if(!(g_interact_player.valid_bits&DG_RINV_ACTOR))memset(&g_action_a,0,sizeof g_action_a);
    if(g_action_a.valid && im->base+g_b.a.pad_direct_seam!=g_action_a.seam-0x4e)
        memset(&g_action_a,0,sizeof g_action_a);
    if(g_b.log)g_b.log("  action NORMAL/hatch anchors: %s\r\n",g_action_a.valid?"resolved":"unavailable");
}
static void action_install(void) {
    const char *why="anchors unavailable";
    if(g_action_a.valid && g_b.pad_detour_live && dg_detour_install_ex(&g_action_detour,
        (void *)(ULONG_PTR)g_action_a.seam,(void *)action_pre_mask,
        (void *)(ULONG_PTR)g_action_a.begin,(void *)(ULONG_PTR)g_action_a.end,&why,1)) {
        if(g_b.log)g_b.log("  action NORMAL pre-mask installed\r\n");
    } else if(g_b.log)g_b.log("  action NORMAL disabled: %s\r\n",why);
}
