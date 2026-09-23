/* Retail 2.1.0.0 mobile tools. Keep JetSpray/SetMic, their trigger, reload,
 * stance transitions and native attached actors. Only standing VR presentation
 * and locomotion are adapted. No weapon type table or action pointer writes. */
static struct {
    uint64_t base;
    int resolved, live;
    DG_DETOUR stance, anchor;
    unsigned char *relay;
    unsigned char calls[3][5];
    int patched;
    uint64_t pose_arm;
    int pose_weapon;
    volatile LONG turns, steps, poses, anchors;
} g_mobile_tools;
static const unsigned mobile_sites[3]={0x51fc49,0x51fc50,0x51ffa7};
static const unsigned mobile_targets[3]={0x53e180,0x53f240,0x53f240};

/* Pure admission and pose rules are shared by the desk tests. */
static int mobile_action(int weapon,uint64_t action,uint64_t base) {
    return (weapon==14 && action==base+0x51f880) ||
           (weapon==12 && action==base+0x51fd40);
}
static int mobile_standing(int weapon,uint64_t action,uint64_t base,
                           int stance,int transition,int grounded) {
    return mobile_action(weapon,action,base) && stance==0 && transition==0 && grounded==1;
}
static int mobile_pose(int weapon,int motion) {
    /* PAspr_ready/PAspr_fire/PAmic_ready -> PAm9_ready. Empty/reload,
       crouched and shared animation events retain their original motion. */
    if((weapon==14 && (motion==40 || motion==42)) ||
       (weapon==12 && motion==43))return 2;
    return motion;
}
static void mobile_turn_pad(unsigned char out[48],const unsigned char in[48]) {
    unsigned status;
    memcpy(out,in,48);
    memcpy(&status,out+4,4);status&=~0xf000u;memcpy(out+4,&status,4);
    out[0x16]=out[0x17]=128; /* left stick is movement, never tool pitch/yaw */
    memset(out+0x18,0,4); /* directional pressure only; weapon pressure retained */
}
static int mobile_context(uint64_t expected,uint64_t *player_out,int *weapon_out) {
    ULONGLONG player=0,arm=0,pad;LONG weapon=0;DG_CAMERA_GATE camera;
    uint64_t now=GetTickCount64();
    LONG age=(LONG)((DWORD)g_b.c_ticks-(DWORD)g_b.hand_command_tick);
    if(!g_mobile_tools.live || !g_b.armed || g_b.script_menu_only ||
       g_b.s_late_unsafe || g_mod_capture || g_b.fps.state!=DG_FPS_ACTIVE ||
       !g_controls_allowed || !g_controls_lease || now<g_controls_lease ||
       now-g_controls_lease>100 || !g_b.hand_command_valid ||
       !g_b.hand_command_requested || age<0 || age>DG_HAND_COMMAND_TICKS ||
       resolve_motion_player(&arm,&player,&weapon)!=DG_RESOLVE_OK ||
       (expected && player!=expected) || region_end(player)<player+0x14a0 ||
       !mobile_standing(weapon,*(uint64_t *)(uintptr_t)(player+0xc60),g_mobile_tools.base,
           *(short *)(uintptr_t)(player+0x13d0),RD32(player+0x5e0),
           *(unsigned char *)(uintptr_t)(player+0x11a)) ||
       !g_b.a.pl_subject_move || !RD32(g_b.a.pl_subject_move) ||
       !dg_bridge_camera_gate_now(&camera) || camera.arm_body!=arm)return 0;
    pad=*(uint64_t *)(uintptr_t)(player+0xd00);
    if(pad!=g_b.a.player_pad || region_end(pad)<pad+48 || !RD32(pad-4))return 0;
    *player_out=player;*weapon_out=weapon;return 1;
}
/* Entry to ActSubjectStanceControl: changing only its standing animation
 * argument preserves native crouch/stand input, transition state and return. */
static void mobile_stance(void *rsp) {
    uint64_t *regs=(uint64_t *)rsp-16,player,table;int weapon;
    if(!mobile_context(regs[13],&player,&weapon)){g_mobile_tools.pose_arm=0;return;} /* RCX */
    if(region_end(player+0x518)<player+0x520)return;
    table=*(uint64_t *)(uintptr_t)(player+0x518);
    /* This is a short array, not an OBJECT pointer: two-byte alignment is valid. */
    if(table<0x10000 || table>=UINT64_C(0x00007fffffff0000) ||
       (table&1) || region_end(table)<table+8)return;
    regs[12]=(uint32_t)*(short *)(uintptr_t)(table+
        ((RD32(g_b.a.player_pad+4)&0xf000u)?6:0)); /* MS.change[Mwalk/Mstand] */
    /* Ready may have been selected before the camera/hand became eligible.
       Request one native animation refresh, without changing arm_motion or
       restarting a reload. Arm ChangeMotion consumes SET_OVER normally. */
    if((g_mobile_tools.pose_arm!=*(uint64_t *)(uintptr_t)(player+0xba8) ||
        g_mobile_tools.pose_weapon!=weapon) &&
       (*(unsigned char *)(uintptr_t)(player+0xcf0)==1 ||
        (weapon==14 && *(unsigned char *)(uintptr_t)(player+0xcf0)==2)))
        *(unsigned *)(uintptr_t)(player+0xcf4)|=4;
}
static void mobile_arm_pose(void *rsp) {
    uint64_t *regs=(uint64_t *)rsp-16,player,arm;int weapon,motion;
    if(!g_mobile_tools.live || (unsigned)regs[12]!=0 ||
       !g_b.a.gm_player_arm_body ||
       regs[13]!=*(uint64_t *)(uintptr_t)g_b.a.gm_player_arm_body ||
       !mobile_context(0,&player,&weapon))return;
    arm=*(uint64_t *)(uintptr_t)(player+0xba8);
    if(regs[13]!=arm || (unsigned)regs[12]!=0)return; /* exact OBJECT, channel 0 */
    motion=mobile_pose(weapon,(int)regs[7]); /* R8 */
    if(motion!=(int)regs[7]){
        regs[7]=(uint32_t)motion;g_mobile_tools.pose_arm=arm;
        g_mobile_tools.pose_weapon=weapon;InterlockedIncrement(&g_mobile_tools.poses);
    }
}
/* ArmAction has selected a camera-offset table and tool-specific index, but
 * has not read the vector yet. Pistol animation requires the corresponding
 * pistol root offset: leaving CS_SPRAY/CS_MIC raises the shoulders hundreds
 * of units. Change only this invocation's index and wrist rotation selector.
 * No table/global offset writes; native wall proximity and smoothing follow. */
static int mobile_anchor_index(int weapon,int index,uint64_t table,uint64_t base,
                               unsigned trigger,int rotation) {
    int wall=(trigger&0x20u)!=0;
    if(trigger&(0x10u|0x40u|0x100u))return 0; /* combo/reload/blade */
    if(wall) {
        if(table!=base+0x980390 && table!=base+0x980400)return 0;
        return (weapon==14 && index==5 && rotation==0) ||
               (weapon==12 && index==6 && rotation==1);
    }
    if(table!=base+0x9801f0 && table!=base+0x9802c0)return 0;
    return (weapon==14 && index==10 && rotation==0) ||
           (weapon==12 && index==9 && rotation==1);
}
static void mobile_arm_anchor(void *rsp) {
    uint64_t *regs=(uint64_t *)rsp-16,player,arm;int weapon;
    if(!g_mobile_tools.live || !g_b.a.gm_player_arm_body)return;
    arm=*(uint64_t *)(uintptr_t)g_b.a.gm_player_arm_body;
    if(regs[9]+0x60!=arm || g_mobile_tools.pose_arm!=arm ||
       !mobile_context(0,&player,&weapon) || g_mobile_tools.pose_weapon!=weapon ||
       (*(unsigned char *)(uintptr_t)(player+0xcf0)!=1 &&
        !(weapon==14 && *(unsigned char *)(uintptr_t)(player+0xcf0)==2)) ||
       !mobile_anchor_index(weapon,(int)regs[10],regs[11],g_mobile_tools.base,
                            (unsigned)regs[8],(int)regs[14]))return;
    /* RBP=CS_SOCOM; RAX=RTS_NONE. Native code scales RBP by 16 afterward. */
    regs[10]=3;regs[14]=0;InterlockedIncrement(&g_mobile_tools.anchors);
}
typedef void (__fastcall *MOBILE_NATIVE_TURN)(uint64_t);
static void mobile_apply(uint64_t player,uint64_t pad,MOBILE_NATIVE_TURN turn,MOBILE_NATIVE_TURN step) {
    unsigned char turn_pad[48];
    mobile_turn_pad(turn_pad,(const unsigned char *)(uintptr_t)pad);
    __try {
        *(uint64_t *)(uintptr_t)(player+0xd00)=(uint64_t)(uintptr_t)turn_pad;
        turn(player);
    } __finally { *(uint64_t *)(uintptr_t)(player+0xd00)=pad; }
    InterlockedIncrement(&g_mobile_tools.turns);
    if((RD32(pad+4)&0xf000u) && !(RD32(player+0xc90)&0x10u)) {
        step(player);
        *(uint64_t *)(uintptr_t)(player+0x1498)|=UINT64_C(0x1000);
        InterlockedIncrement(&g_mobile_tools.steps);
    }
}
static void mobile_turn(uint64_t player,unsigned fallback) {
    uint64_t owner,pad;int weapon;
    uint64_t b=g_mobile_tools.base;
    if(!mobile_context(player,&owner,&weapon)) {
        ((MOBILE_NATIVE_TURN)(uintptr_t)(b+fallback))(player);return;
    }
    pad=*(uint64_t *)(uintptr_t)(player+0xd00);
    /* The native turn sees a private left-neutral pad; native movement then
       sees the untouched original. CheckStep/hazard processing runs afterward.
       FLAG2_SUBJECT_MOVE is cleared by the native player every frame. */
    mobile_apply(player,pad,(MOBILE_NATIVE_TURN)(uintptr_t)(b+0x53e180),
                           (MOBILE_NATIVE_TURN)(uintptr_t)(b+0x53deb0));
}
static void mobile_turn_move(uint64_t p){mobile_turn(p,0x53e180);}
static void mobile_turn_subject(uint64_t p){mobile_turn(p,0x53f240);}

static void mobile_resolve(const LiveImage *im) {
    unsigned i;int32_t rel;
    memset(&g_mobile_tools,0,sizeof g_mobile_tools);
    for(i=0;i<3;i++) {
        if(!all_valid(im->valid,mobile_sites[i],5,im->size) || im->bytes[mobile_sites[i]]!=0xe8)return;
        memcpy(&rel,im->bytes+mobile_sites[i]+1,4);
        if((int64_t)mobile_sites[i]+5+rel!=mobile_targets[i])return;
        memcpy(g_mobile_tools.calls[i],im->bytes+mobile_sites[i],5);
    }
    if(!m9_match(im,0x521ab0,(const unsigned char *)"\x48\x89\x5c\x24\x08\x48\x89\x6c\x24\x10",10) ||
       !m9_match(im,0x67dd70,(const unsigned char *)"\x40\x57\x48\x83\xec\x30\x48\x8b\xf9",9) ||
       !m9_match(im,0x53deb0,(const unsigned char *)"\x40\x53\x48\x81\xec\x80\0\0\0",9) ||
       !m9_match(im,0x53e180,(const unsigned char *)"\x48\x8b\xc4\x48\x89\x58\x20",7) ||
       !m9_match(im,0x51a94c,(const unsigned char *)"\x48\x8b\x88\x18\x05\0\0\x0f\xbf\x11",10) ||
       !m9_match(im,0x9800c0,(const unsigned char *)"\x00\x00\x28\x00\x2a\x00\x2e\x00\x3a\x00\x34\x00",12) ||
       !m9_match(im,0x9800d8,(const unsigned char *)"\x00\x00\x2b\x00\x3a\x00\x3a\x00",8) ||
       !m9_match(im,0x980048,(const unsigned char *)"\x00\x00\x02\x00\x03\x00\x04\x00",8) ||
       !m9_match(im,0x57bbd7,(const unsigned char *)"\x48\x09\x88\x98\x14\0\0\xc3",8) ||
       !m9_match(im,0x57cec0,(const unsigned char *)"\x40\x56\x48\x83\xec\x50",6) ||
       !m9_match(im,0x57d063,(const unsigned char *)"\x48\x63\xed\x48\x03\xed\xf3\x0f\x10\x04\xeb",11) ||
       !m9_match(im,0x980220,(const unsigned char *)"\0\0\x82\xc2\0\xc0\x04\x44\0\0\xb0\x43\0\0\0\0",16) ||
       !m9_match(im,0x9802f0,(const unsigned char *)"\0\0\x98\xc2\0\x80\0\x44\0\0\x9c\x43\0\0\0\0",16))return;
    if(!m9_match(im,0x9803c0,(const unsigned char *)"\x00\x00\x00\xc1\x00\x00\xca\x43\x00\x00\x0c\x43\x00\x00\x00\x00",16))return;
    if(!m9_match(im,0x980430,(const unsigned char *)"\x00\x00\x00\xc1\x00\x00\xca\x43\x00\x00\x0c\x43\x00\x00\x00\x00",16))return;
    g_mobile_tools.base=im->base;g_mobile_tools.resolved=1;
}
static void mobile_stop(void) {
    int i;g_mobile_tools.live=0;
    for(i=0;i<g_mobile_tools.patched;i++)
        patch_code((unsigned char *)(uintptr_t)(g_mobile_tools.base+mobile_sites[i]),g_mobile_tools.calls[i],5);
    g_mobile_tools.patched=0;
    dg_detour_remove(&g_mobile_tools.stance);motion_release(DG_MOTION_MOBILE);
    dg_detour_remove(&g_mobile_tools.anchor);
    /* Keep the tiny relay allocation until process exit: a suspended native
       caller may already have fetched its CALL destination during shutdown. */
}
static void mobile_install(void) {
    const char *why="retail witnesses unavailable";unsigned i;uint64_t b=g_mobile_tools.base;
    if(!g_mobile_tools.resolved)goto done;
    if(!dg_detour_install_ex(&g_mobile_tools.stance,(void *)(uintptr_t)(b+0x521ab0),mobile_stance,
        (void *)(uintptr_t)(b+0x521ab0),(void *)(uintptr_t)(b+0x521c30),&why,1))goto fail;
    if(!motion_acquire(b,DG_MOTION_MOBILE,&why))goto fail;
    if(!dg_detour_install_ex(&g_mobile_tools.anchor,(void *)(uintptr_t)(b+0x57d063),mobile_arm_anchor,
        (void *)(uintptr_t)(b+0x57cec0),(void *)(uintptr_t)(b+0x57d180),&why,1))goto fail;
    g_mobile_tools.relay=alloc_near((ULONG_PTR)(b+mobile_sites[0]),0x1000);
    if(!g_mobile_tools.relay){why="relay allocation failed";goto fail;}
    emit_jmp_abs(g_mobile_tools.relay,mobile_turn_move);
    emit_jmp_abs(g_mobile_tools.relay+16,mobile_turn_subject);
    FlushInstructionCache(GetCurrentProcess(),g_mobile_tools.relay,32);
    for(i=0;i<3;i++) {
        unsigned char call[5]={0xe8};int64_t delta=(int64_t)(uintptr_t)(g_mobile_tools.relay+(i?16:0))-(int64_t)(b+mobile_sites[i]+5);
        int32_t rel=(int32_t)delta;
        if(delta!=rel){why="relay outside CALL range";goto fail;}
        memcpy(call+1,&rel,4);
        if(!patch_code((unsigned char *)(uintptr_t)(b+mobile_sites[i]),call,5)){why="CALL patch failed";goto fail;}
        g_mobile_tools.patched++;
    }
    g_mobile_tools.live=1;goto done;
fail: mobile_stop();
done: if(g_b.log)g_b.log("  mobile tools: standing locomotion and neutral arms %s (%s)\r\n",g_mobile_tools.live?"installed":"unavailable",why);
}
