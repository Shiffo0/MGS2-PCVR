static unsigned coolant_pad_status(uint64_t pad) {

    if(pad>=0x10000 && pad<UINT64_C(0x00007fffffff0000) && !(pad&3) && region_end(pad)>=pad+8)
        return RD32(pad+4);
    return 0xffffffffu;
}
#include "dg_coolant_input.h"
static DG_COOLANT_LEASE coolant_lease;
static DG_BRIDGE_FIRE coolant_command;
static unsigned coolant_command_tick,coolant_epoch;
/* The registered provider's frame exists only inside controls_begin/end.
 * Carry its already-approved sample over that boundary, without re-entering
 * the provider (which would process menus/gestures twice). Caller holds lock. */
static int coolant_read_command(DG_BRIDGE_FIRE *cmd,LONG *stamp) {
    if(coolant_epoch!=g_controls_epoch || g_controls_fire_retired)return 0;
    if(!g_controls_provider)return fire_read(cmd,stamp);
    *cmd=coolant_command;*stamp=(LONG)coolant_command_tick;return 1;
}
/* Merge only the already-admitted held level after the native pad rebuild.
 * Never change press/release, player.trigger, native actions or weapon state. */
static int coolant_merge_locked(uint64_t player) {
    DG_COOLANT_CONSUMER in;
    DG_BRIDGE_FIRE cmd;
    DG_CAMERA_GATE camera;
    ULONGLONG owner=0,arm=0,pad,end;
    LONG weapon=0,stamp=0,index;
    memset(&in,0,sizeof in);memset(&camera,0,sizeof camera);
    if(!coolant_lease.ready)return 0;
    end=region_end(player);
    if(!end || player+0xD08>end)goto refuse;
    pad=*(volatile ULONGLONG *)(ULONG_PTR)(player+0xD00);
    if(pad!=g_b.a.player_pad || coolant_pad_status(pad)==0xffffffffu ||
       region_end(pad)<pad+DG_PAD_PRESSURE_OFFSET+DG_PAD_PRESSURE_COUNT ||
       !g_b.a.pad_weapon || !g_b.a.pad_press_weapon)goto refuse;
    index=RD32(g_b.a.pad_press_weapon);
    if(index<0 || index>=DG_PAD_PRESSURE_COUNT)goto refuse;
    if(!coolant_read_command(&cmd,&stamp))goto refuse;
    in.player=player;in.pad=pad;in.tick=(unsigned)g_b.c_ticks;
    in.stream=cmd.stream_id;in.press=cmd.press_seq;in.release=cmd.release_seq;
    in.mask=(unsigned)RD32(g_b.a.pad_weapon);in.status=coolant_pad_status(pad);
    in.fresh=cmd.valid && cmd.value>=0.0 && cmd.value<=1.0 &&
        (unsigned)(in.tick-(unsigned)stamp)<=DG_FIRE_FRESH_TICKS;
    in.safe=g_b.armed && g_b.fire_mode==DG_FIRE_MODE_ON &&
        !g_b.script_menu_only && !g_b.s_late_unsafe && !g_mod_capture &&
        g_b.fps.state==DG_FPS_ACTIVE && RD32(pad-4)!=0 &&
        resolve_player(&arm,&owner,&weapon)==DG_RESOLVE_OK && owner==player &&
        dg_bridge_camera_gate_now(&camera) && camera.arm_body==arm;
    in.weapon=weapon;
    return dg_coolant_merge_pad(&coolant_lease,&in,
        (volatile unsigned *)(ULONG_PTR)(pad+DG_PAD_STATUS_OFFSET),
        (volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+index));
refuse:
    coolant_lease.ready=0;return 0;
}
static int coolant_merge(uint64_t player) {
    int result;
    if(!TryAcquireSRWLockShared(&g_controls_lock)) {
        coolant_lease.ready=0;return 0;
    }
    result=coolant_merge_locked(player);
    ReleaseSRWLockShared(&g_controls_lock);return result;
}
/* Stage 1 observes before merge; stage 2 after merge, both before native gates. */
static DG_DETOUR g_coolant_consumer;
static uint64_t g_coolant_base;
static void coolant_consumer(void *rsp) {
    uint64_t player=0;LONG weapon=0;DG_FIRE_IN in;DG_FIRE_OUT out;
    if(!g_b.armed || !interact_player_now(&player,&weapon) || weapon!=14 ||
       player!=((uint64_t *)rsp-16)[8])return; /* saved RDI */
    memset(&in,0,sizeof in);memset(&out,0,sizeof out);

    coolant_merge(player);

}
static void coolant_resolve(const LiveImage *im) {
    g_coolant_base=0;
    if(m9_match(im,0x51f9b1,(const unsigned char *)"\x0f\xb7\x87\xc2\x0c\0\0\x44\x0f\xb7\xbf\xd0\x13\0\0",15) &&
       m9_match(im,0x51f9f9,(const unsigned char *)"\x48\x8b\x87\0\x0d\0\0\x8b\x48\x04\x85\x0d\xe7\x04\x46\0",16))g_coolant_base=im->base;
}
static void coolant_install(void) {
    const char *why="retail witness unavailable";uint64_t b=g_coolant_base;int ok=0;
    if(b)ok=dg_detour_install_ex(&g_coolant_consumer,(void *)(ULONG_PTR)(b+0x51f9b1),
        coolant_consumer,(void *)(ULONG_PTR)(b+0x51f880),(void *)(ULONG_PTR)(b+0x51fd38),&why,1);
    if(g_b.log)g_b.log("  coolant: held-input consumer seam %s (%s)\r\n",ok?"installed":"unavailable",why);
}
static void coolant_stop(void) {coolant_lease.ready=0;dg_detour_remove(&g_coolant_consumer);}
