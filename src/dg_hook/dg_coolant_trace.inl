/* Bounded tick snapshots; formatting runs only in dg_bridge_stats. No game writes.
   Retail JetSpray51F880: CC2 countdown, CF0 arm_motion, D00->+4 native status.
   Those offsets are independently disassembled in the research artifact. */
typedef struct {
    DWORD tick; int stage,stance; unsigned written_status; ULONGLONG written_pad; int safe,fps,late,resolver,can_write,enabled,mode,physical;
    int input,age,press,status,release,wait,arm_motion,unit,trigger;
    unsigned native_status,mask;
    ULONGLONG player,arm,body,action,action2,camera,pad;
    int camera_on;
} COOLANT_TRACE;
static COOLANT_TRACE coolant_rows[256];
static unsigned coolant_n,coolant_drops;
static DWORD coolant_last[2];
static SRWLOCK coolant_lock=SRWLOCK_INIT;
static unsigned coolant_pad_status(uint64_t pad) {
    /* GV_PAD contains 32-bit fields; retail commonly places it at ...844. */
    if(pad>=0x10000 && pad<UINT64_C(0x00007fffffff0000) && !(pad&3) && region_end(pad)>=pad+8)
        return RD32(pad+4);
    return 0xffffffffu;
}
static void coolant_trace_tick(int safe,int mode,const DG_FIRE_IN *in,const DG_FIRE_OUT *out,int stage) {
    ULONGLONG pw=0,end=0,arm=0,pad;LONG weapon=0;COOLANT_TRACE r;DG_CAMERA_GATE cam;
    DWORD now=(DWORD)g_b.c_ticks;
    if(!g_b.log || now-coolant_last[stage]<6 || !interact_player_now(&pw,&weapon) || weapon!=14)return;
    end=region_end(pw);if(!end || pw+0x13D2>end)return;
    memset(&r,0,sizeof r);memset(&cam,0,sizeof cam);
    r.stage=stage;r.stance=*(volatile short *)(ULONG_PTR)(pw+0x13D0);
    r.written_pad=g_b.a.player_pad;r.written_status=coolant_pad_status(r.written_pad);
    r.tick=now;r.player=pw;r.safe=safe;r.fps=g_b.fps.state;r.late=g_b.s_late_unsafe;
    r.resolver=resolve_player(&arm,NULL,NULL);r.arm=arm;r.can_write=in->can_write;
    r.mode=mode;r.physical=in->physical_down;r.input=in->input_ok;r.age=in->sample_age;
    r.press=out->press;r.status=out->status;r.release=out->release;
    r.body=*(volatile ULONGLONG *)(ULONG_PTR)(pw+0xBA8);r.unit=RD32(pw+0xBB0);
    r.action=*(volatile ULONGLONG *)(ULONG_PTR)(pw+0xC60);
    r.action2=*(volatile ULONGLONG *)(ULONG_PTR)(pw+0xC78);
    r.trigger=RD32(pw+0xB94);r.wait=*(volatile short *)(ULONG_PTR)(pw+0xCC2);
    r.arm_motion=*(volatile unsigned char *)(ULONG_PTR)(pw+0xCF0);
    pad=*(volatile ULONGLONG *)(ULONG_PTR)(pw+0xD00);r.pad=pad;
    r.native_status=coolant_pad_status(pad);
    if(g_b.a.player_pad && region_end(g_b.a.player_pad-4)>=g_b.a.player_pad)r.enabled=RD32(g_b.a.player_pad-4);
    if(g_b.a.pad_weapon && region_end(g_b.a.pad_weapon)>=g_b.a.pad_weapon+4)r.mask=RD32(g_b.a.pad_weapon);
    dg_bridge_camera_gate_now(&cam);r.camera=cam.camera;r.camera_on=cam.arm_camera_on;
    if(!TryAcquireSRWLockExclusive(&coolant_lock))return;
    coolant_last[stage]=now;if(coolant_n<256)coolant_rows[coolant_n++]=r;else coolant_drops++;
    ReleaseSRWLockExclusive(&coolant_lock);
}
static void coolant_trace_flush(void) {
    COOLANT_TRACE rows[256];unsigned n,i,dropped;
    if(!g_b.log || !TryAcquireSRWLockExclusive(&coolant_lock))return;
    n=coolant_n;memcpy(rows,coolant_rows,n*sizeof rows[0]);dropped=coolant_drops;
    coolant_n=coolant_drops=0;ReleaseSRWLockExclusive(&coolant_lock);
    for(i=0;i<n;i++){const COOLANT_TRACE *r=&rows[i];
        g_b.log("  coolant trace: tick=%lu stage=%d stance=%d written=%08X writepad=%llX safe=%d fps=%d late=%d resolver=%d enabled=%d write=%d mode=%d input=%d age=%d physical=%d output=%d/%d/%d native=%08X mask=%08X trigger=%d wait=%d motion=%d player=%llX arm=%llX body=%llX unit=%d action=%llX/%llX camera=%llX on=%d pad=%llX\r\n",
            r->tick,r->stage,r->stance,r->written_status,r->written_pad,r->safe,r->fps,r->late,r->resolver,r->enabled,r->can_write,r->mode,r->input,r->age,r->physical,
            r->press,r->status,r->release,r->native_status,r->mask,r->trigger,r->wait,r->arm_motion,
            r->player,r->arm,r->body,r->unit,r->action,r->action2,r->camera,r->camera_on,r->pad);
    }
    if(dropped)g_b.log("  coolant trace: dropped=%u\r\n",dropped);
}

/* Sample the actual JetSpray consumer before its wait/motion/status gates.
 * This callback is observation only; the game's input and trigger stay native. */
static DG_DETOUR g_coolant_consumer;
static uint64_t g_coolant_base;
static void coolant_consumer(void *rsp) {
    uint64_t player=0;LONG weapon=0;DG_FIRE_IN in;DG_FIRE_OUT out;
    if(!g_b.armed || !interact_player_now(&player,&weapon) || weapon!=14 ||
       player!=((uint64_t *)rsp-16)[8])return; /* saved RDI */
    memset(&in,0,sizeof in);memset(&out,0,sizeof out);
    coolant_trace_tick(!g_b.s_late_unsafe,-1,&in,&out,1);
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
    if(g_b.log)g_b.log("  coolant trace: consumer observer %s (%s)\r\n",ok?"installed":"unavailable",why);
}
static void coolant_stop(void) {dg_detour_remove(&g_coolant_consumer);}
