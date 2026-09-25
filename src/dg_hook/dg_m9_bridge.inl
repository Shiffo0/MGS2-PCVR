/* Retail 2.1.0.0 adapter, gated by the bridge fingerprint AND independent
 * instruction/table witnesses. No pointer is inferred from an actor list.
 * See m9_retail_evidence.py and M9_HANDMATIGE_SLEDE_IMPLEMENTATIE. */
static SRWLOCK g_m9_lock=SRWLOCK_INIT;
static DG_GRIP_DEBUG g_grip_debug;
int dg_bridge_grip_debug_snapshot(DG_GRIP_DEBUG *out) {
    if(!out || !TryAcquireSRWLockShared(&g_m9_lock))return 0;
    *out=g_grip_debug;ReleaseSRWLockShared(&g_m9_lock);return out->valid;
}
static DG_DETOUR g_m9_slide_detour,g_m9_sea_detour,g_m9_shot_detour;
static volatile LONG g_m9_enabled,g_m9_events;
static struct {
    uint64_t base,subglobal,objs,ctrl,player,stamp,epoch,shots,rebound_arm;
    int resolved,live,active,render,blocked,last_state,last_block,fault,fault_reported,reset_context;
    double anchor[3],scale;
    double pose_bias[3],grab_bias[3],pose_scale;
    uint64_t pose_source,pose_sequence,pose_stamp;
    uint64_t grip_log_ms;
    int grip_log_down,kind;
    DG_M9_STATE state;
    DG_M9_OUTPUT out;
} g_m9;
unsigned dg_bridge_m9_events(void) {return (unsigned)InterlockedExchange(&g_m9_events,0);}
/* Measure the controller-to-rendered-finger-contact difference from ONE successful
 * coherent arm solve. Independent arm calibration/reach limits make a raw
 * controller origin an unreliable proxy for the visible wrist. */
static void m9_observe_hand(const DG_BRIDGE_ARM_TARGET *t,const double wrist[3]) {
    double delta[3],local[3],inv[4],bias[3],scale;int k;
    if(!g_m9_enabled || !t || !t->m9_pose.valid || !t->hands_coherent ||
       !t->aim_write || t->aim_weapon_id!=1 || t->left_weight!=1 ||
       t->m9_pose.source!=t->stream_id ||
       t->m9_pose.sequence!=t->left_sample_seq ||
       t->m9_pose.sequence!=t->aim_sample_seq ||
       t->left_sample_time!=t->aim_sample_time)return;
    scale=t->m9_pose.units_per_metre*(g_b.camera_position.ready?g_b.camera_position.scale:1);
    if(!_finite(scale)||scale<334||scale>10000)return;
    for(k=0;k<3;k++)delta[k]=(wrist[k]-g_b.pred_wrist[k])/scale;
    dg_ik_quat_conj(g_b.hand_desired,inv);dg_position_rotate(inv,delta,local);
    for(k=0;k<3;k++)bias[k]=local[k]-t->m9_pose.local[k];
    if(!dg_m9_vec_ok(bias))return;
    AcquireSRWLockExclusive(&g_m9_lock);
    if(!g_m9.out.attached) {
        memcpy(g_m9.pose_bias,bias,sizeof bias);g_m9.pose_scale=scale;
        g_m9.pose_source=t->m9_pose.source;g_m9.pose_sequence=t->m9_pose.sequence;
        g_m9.pose_stamp=GetTickCount64();
    }
    ReleaseSRWLockExclusive(&g_m9_lock);
}
static int m9_match(const LiveImage *im,size_t rva,const unsigned char *bytes,size_t n) {
    return rva+n<=im->size && all_valid(im->valid,rva,n,im->size) && !memcmp(im->bytes+rva,bytes,n);
}
static void m9_resolve(const LiveImage *im) {
    uint64_t fn=0,sea=0;uint32_t sw=0;int32_t subdisp=0;
    memset(&g_m9,0,sizeof g_m9);g_m9.last_state=-1;
    /* These RVAs are an explicitly version-locked adapter, not a signature
     * scan that claims support for unknown executables. */
    if(im->size<=0x97be80 || !all_valid(im->valid,0x97be20,16,im->size) ||
       !all_valid(im->valid,0x97be70,16,im->size) ||
       !all_valid(im->valid,0x4b05f8,4,im->size))return;
    memcpy(&fn,im->bytes+0x97be20,8);memcpy(&sw,im->bytes+0x4b05f8,4);
    memcpy(&sea,im->bytes+0x97be70,8);
    if(fn!=im->base+0x4d6440 || sea!=im->base+0x4d6cd0 || sw!=0x4aff16 ||
       !m9_match(im,0x97be78,(const unsigned char *)"\x05\0\0\0\x04\0\0\0",8) ||
       !m9_match(im,0x4d6cd0,(const unsigned char *)"\x48\x89\x5c\x24\x18\x48\x89\x74\x24\x20",10) ||
       !m9_match(im,0x4d6e33,(const unsigned char *)"\x8b\x45\x20\x4c\x8d\x43\xd8\x89\x43\x08",10) ||
       !m9_match(im,0x4d6ea0,(const unsigned char *)"\x49\x8b\x56\x18\xff\xc6\x48\x81\xc3\x80\x01\0\0",13) ||
       !m9_match(im,0x97be28,(const unsigned char *)"\x32\0\0\0\x06\0\0\0",8) ||
       !m9_match(im,0x4d6440,(const unsigned char *)"\x48\x89\x5c\x24\x18\x48\x89\x6c\x24\x20",10) ||
       !m9_match(im,0x4d656d,(const unsigned char *)"\xc7\x43\x0c\0\0\x48\x42",7) ||
       !m9_match(im,0x4d662f,(const unsigned char *)"\x41\x8b\x47\x20\x4c\x8d\x43\xd8\x89\x43\x08",11) ||
       !m9_match(im,0x4d669f,(const unsigned char *)"\x49\x8b\x56\x18\xff\xc5\x48\x81\xc3\x80\x01\0\0",13) ||
       !m9_match(im,0x4aff16,(const unsigned char *)"\x89\xbe\x90\x02\0\0\x89\xbe\xc8\x02\0\0\x44\x8b\xc7",15) ||
       !m9_match(im,0x4afd99,(const unsigned char *)"\x44\x8b\xc7\x85\xc0\x75\x29\x48\x8d\x55\xa0\x8d\x48\x04\xe8\x44\x15\x0a\0",19))return;
    /* GM_PlayerWeaponSubObject writer; independently witness its RIP target. */
    if(!m9_match(im,0x4b0cbc,(const unsigned char *)"\x48\x89\x35",3) ||
       !all_valid(im->valid,0x4b0cbf,4,im->size))return;
    memcpy(&subdisp,im->bytes+0x4b0cbf,4);
    if((int64_t)0x4b0cc3+subdisp!=0x17df688)return;
    g_m9.subglobal=im->base+0x17df688;
    g_m9.base=im->base;g_m9.resolved=1;
    if(g_b.log)g_b.log("  M9: retail slide/shot/audio witnesses resolved (opt-in)\r\n");
}





static int m9_current_owner(uint64_t arm,uint64_t ctrl,uint64_t objs) {
    uint64_t sub,actor,trigger,player,armobjs;
    if(!g_m9.subglobal || arm<0x60)return 0;
    sub=*(uint64_t *)(ULONG_PTR)g_m9.subglobal;
    if(sub<0xa0)return 0;
    actor=sub-0xa0;
    if(ctrl!=actor+0x2a8 || *(uint64_t *)(ULONG_PTR)sub!=objs)return 0;
    trigger=*(uint64_t *)(ULONG_PTR)(arm+0x1c8);
    if(trigger<0xcf4)return 0;
    player=trigger-0xcf4;
    armobjs=*(uint64_t *)(ULONG_PTR)arm;
    return armobjs && *(int *)(ULONG_PTR)(player+0xb90)==1 &&
        *(uint64_t *)(ULONG_PTR)(actor+0x248)==player+0xba8 &&
        *(uint64_t *)(ULONG_PTR)(actor+0x2e8)==player+0xbb0 &&
        *(uint64_t *)(ULONG_PTR)(player+0xba8)==arm &&
        *(int *)(ULONG_PTR)(player+0xbb0)==6 &&
        *(uint64_t *)(ULONG_PTR)(objs+0x40)==armobjs+0x110+6*0x180;
}
/* original_rsp[-16] is saved R15; -15 R14; -6 RBP; -5 RBX;
 * -7 RSI. The existing detour saves all GPRs and XMM0-5 before calling us. */
static void m9_slide_variant(void *original_rsp,int kind,int index_reg) {
    uint64_t *regs=(uint64_t *)original_rsp-16;
    uint64_t ctrl=regs[1],unit=regs[11],arm=0,objs=0,model=0;
    float anchor[3];int k;
    if(!g_m9_enabled || !g_m9.live || !g_b.armed || regs[index_reg]!=2)return;
    if(!TryAcquireSRWLockExclusive(&g_m9_lock))return;
    __try {
        if(!g_b.a.gm_player_arm_body) __leave;
        arm=*(uint64_t *)(ULONG_PTR)g_b.a.gm_player_arm_body;
        if(!arm || *(int *)(ULONG_PTR)(ctrl+8)!=kind) __leave;
        objs=*(uint64_t *)(ULONG_PTR)(ctrl+0x18);
        if(!objs || unit!=objs+0x438 || *(short *)(ULONG_PTR)(objs+0x64)<3 ||
           *(short *)(ULONG_PTR)(objs+0x64)>16) __leave;
        if(*(uint64_t *)(ULONG_PTR)(ctrl+0x10)!=arm) {
            if(!m9_current_owner(arm,ctrl,objs)) __leave;
            if(g_m9.rebound_arm!=arm) {
                if(g_b.log)g_b.log("  M9: retained weapon validated against recreated arm %llX (cached root %llX)\r\n",
                    arm,*(uint64_t *)(ULONG_PTR)(ctrl+0x10));
                g_m9.rebound_arm=arm;
            }
        }
        model=*(uint64_t *)(ULONG_PTR)(unit+0xb8);
        if(!model || *(int *)(ULONG_PTR)(model+0x2c)!=0) __leave;
        memcpy(anchor,(void *)(ULONG_PTR)(model+0x20),sizeof anchor);
        for(k=0;k<3;k++)if(!_finite(anchor[k]) || fabs(anchor[k])>1000) __leave;
        g_m9.objs=objs;g_m9.ctrl=ctrl;g_m9.stamp=GetTickCount64();
        if(g_m9.kind!=kind && g_b.log)g_b.log("  M9: validated slide variant kind=%d models=%d\r\n",kind,*(short *)(ULONG_PTR)(objs+0x64));
        g_m9.kind=kind;
        for(k=0;k<3;k++)g_m9.anchor[k]=anchor[k];
        /* BEFORE parent multiplication: replace only slide translation.
         * Hammers, timers, other weapons and enemy actors stay native. */
        if(g_m9.active && g_m9.render && !g_m9.fault && g_b.fps.state==DG_FPS_ACTIVE &&
           !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0) && g_b.fire_mode==DG_FIRE_MODE_ON &&
           GetTickCount64()>=g_m9.state.now_ms && GetTickCount64()-g_m9.state.now_ms<=100)
            *(float *)(ULONG_PTR)(unit+0xc)=(float)(50.0*g_m9.out.progress);
    } __except(EXCEPTION_EXECUTE_HANDLER) {g_m9.fault=1;}
    ReleaseSRWLockExclusive(&g_m9_lock);
}
static void m9_slide_seam(void *rsp) {m9_slide_variant(rsp,4,10);}
/* Suppressed M9 uses ESI as loop index instead of EBP; R14/RBX unchanged. */
static void m9_sea_seam(void *rsp) {m9_slide_variant(rsp,9,9);}
static void m9_shot_seam(void *original_rsp) {
    uint64_t actor=((uint64_t *)original_rsp-16)[9];
    if(!g_m9_enabled || !g_m9.live || !g_b.armed)return;
    AcquireSRWLockExclusive(&g_m9_lock);
    __try {
        if(g_m9.active && g_m9.objs && *(uint64_t *)(ULONG_PTR)(actor+0xa0)==g_m9.objs) {
            g_m9.shots++;g_m9.blocked=1;
            g_m9.state.state=M9_NEEDS_RACK;g_m9.state.neutral=0;
            g_m9.state.travel=0;g_m9.state.full_samples=0;
            g_m9.out.progress=0;g_m9.out.attached=0;
            if(g_b.log)g_b.log("  M9: native shot confirmed #%llu\r\n",g_m9.shots);
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) {g_m9.fault=1;}
    ReleaseSRWLockExclusive(&g_m9_lock);
}
static void m9_install(int requested) {
    const char *why="not requested or unresolved";
    if(!requested)return;
    if(!g_m9.resolved){if(g_b.log)g_b.log("  M9: requested but retail witnesses unavailable; native behavior retained\r\n");return;}
    if(dg_detour_install_ex(&g_m9_slide_detour,(void *)(ULONG_PTR)(g_m9.base+0x4d662f),
       (void *)m9_slide_seam,(void *)(ULONG_PTR)(g_m9.base+0x4d6440),
       (void *)(ULONG_PTR)(g_m9.base+0x4d66f4),&why,1) &&
       dg_detour_install_ex(&g_m9_sea_detour,(void *)(ULONG_PTR)(g_m9.base+0x4d6e33),
       (void *)m9_sea_seam,(void *)(ULONG_PTR)(g_m9.base+0x4d6cd0),
       (void *)(ULONG_PTR)(g_m9.base+0x4d6eea),&why,1) &&
       dg_detour_install_ex(&g_m9_shot_detour,(void *)(ULONG_PTR)(g_m9.base+0x4aff16),
       (void *)m9_shot_seam,(void *)(ULONG_PTR)(g_m9.base+0x4af580),
       (void *)(ULONG_PTR)(g_m9.base+0x4b0620),&why,1))g_m9.live=1;
    if(!g_m9.live) {dg_detour_remove(&g_m9_slide_detour);dg_detour_remove(&g_m9_sea_detour);dg_detour_remove(&g_m9_shot_detour);}
    if(g_b.log)g_b.log("  M9: manual slide %s (%s)\r\n",g_m9.live?"adapter installed":"unavailable",why);
}
static void m9_stop(void) {
    InterlockedExchange(&g_m9_enabled,0);
    AcquireSRWLockExclusive(&g_m9_lock);g_m9.active=0;ReleaseSRWLockExclusive(&g_m9_lock);
    dg_detour_remove(&g_m9_slide_detour);dg_detour_remove(&g_m9_sea_detour);dg_detour_remove(&g_m9_shot_detour);g_m9.live=0;
    InterlockedExchange(&g_m9_events,0);
}
/* Called on the game tick BEFORE dg_fire_step. The physical value is sampled
 * before our pad override. A blocked pull is consumed, never queued. */
static int m9_fire_gate(int safe,int can_write,uint64_t player,int weapon,
                       int physical_down,DG_FIRE_IN *fire) {
    DG_M9_INPUT in;DG_M9_SAMPLE *s=&g_controls_frame.m9;
    uint64_t now=GetTickCount64();int block=0,k;unsigned events=0;
    AcquireSRWLockExclusive(&g_m9_lock);
    memset(&g_grip_debug,0,sizeof g_grip_debug);
    if(!g_m9_enabled || !g_m9.live || g_m9.fault || !g_b.armed) {
        if(g_m9.fault && !g_m9.fault_reported){
            if(g_b.log)g_b.log("  M9: adapter fault; manual mode disabled, native behavior restored\r\n");
            g_m9.fault_reported=1;
        }
        g_m9.active=g_m9.render=0;g_m9.blocked=0;goto done;
    }
    memset(&in,0,sizeof in);
    if(player && player!=g_m9.player){g_m9.player=player;g_m9.epoch++;g_m9.pose_source=0;}
    if(!g_m9.epoch)g_m9.epoch=1;
    in.epoch=g_m9.epoch;in.source=s->source;in.sequence=s->sequence;in.now_ms=now;
    in.shot_sequence=g_m9.shots;in.valid=s->valid;in.grip=s->grip;
    in.trigger=s->trigger;in.physical_trigger=physical_down;
    /* Hand separation is scaled by the common right/left position mapping.
       Convert native model offsets with that same scale, not XR units alone. */
    g_m9.scale=s->units_per_metre*(g_b.camera_position.ready?g_b.camera_position.scale:1);
    in.stroke=(_finite(g_m9.scale) && g_m9.scale>=334 && g_m9.scale<=10000)?50/g_m9.scale:0;
    in.return_seconds=16.0/60.0; /* original linear return, normal 60 Hz timebase */
    for(k=0;k<3;k++){in.left[k]=s->local[k];in.anchor[k]=in.stroke>0?g_m9.anchor[k]/g_m9.scale:0;}
    in.contact_valid=g_m9.pose_source==s->source && g_m9.pose_sequence &&
        g_m9.pose_sequence<=s->sequence && now>=g_m9.pose_stamp &&
        now-g_m9.pose_stamp<=50 && g_m9.pose_scale==g_m9.scale;
    if(g_m9.out.attached)for(k=0;k<3;k++)in.anchor[k]-=g_m9.grab_bias[k];
    else if(in.contact_valid)for(k=0;k<3;k++)in.anchor[k]-=g_m9.pose_bias[k];
    in.allowed=safe && can_write && weapon==1 && s->allowed && g_controls_active &&
        g_m9.objs && now>=g_m9.stamp && now-g_m9.stamp<=100;
    /* Never block a weapon we cannot render. Once the adapter is proven and
     * active, stale hand input cancels the grip but preserves the obligation. */
    if(weapon==1 && in.allowed && in.valid)g_m9.active=1;
    g_m9.out=dg_m9_step(&g_m9.state,&in);events=g_m9.out.events;
    if(events&M9_GRAB)memcpy(g_m9.grab_bias,g_m9.pose_bias,sizeof g_m9.grab_bias);
    if(weapon==1 && in.valid && g_m9.objs && now>=g_m9.stamp && now-g_m9.stamp<=100) {
        g_grip_debug.stamp=now;g_grip_debug.sequence=in.sequence;
        g_grip_debug.valid=in.contact_valid||g_m9.out.attached;g_grip_debug.allowed=in.allowed;
        g_grip_debug.state=g_m9.out.state;g_grip_debug.grip=in.grip;
        memcpy(g_grip_debug.origin,s->origin,sizeof s->origin);
        memcpy(g_grip_debug.aim_q,s->aim_q,sizeof s->aim_q);
        memcpy(g_grip_debug.anchor,in.anchor,sizeof in.anchor);
        memcpy(g_grip_debug.left,in.left,sizeof in.left);
    }
    if(weapon==1 && in.grip && (!g_m9.grip_log_down || now-g_m9.grip_log_ms>=500)) {
        double d=0;
        for(k=0;k<3;k++){double v=in.left[k]-in.anchor[k];d+=v*v;}
        if(g_b.log)g_b.log("  M9 grip: edge=%d distance=%.1fmm left=[%.3f %.3f %.3f] anchor=[%.3f %.3f %.3f] scale=%.1f valid=%d allowed=%d state=%d events=%u\r\n",
            !g_m9.grip_log_down,sqrt(d)*1000,in.left[0],in.left[1],in.left[2],
            in.anchor[0],in.anchor[1],in.anchor[2],g_m9.scale,in.valid,in.allowed,g_m9.out.state,events);
        g_m9.grip_log_ms=now;
    }
    g_m9.grip_log_down=in.grip;
    g_m9.render=in.allowed && in.valid;
    block=g_m9.active && weapon==1 && can_write && g_m9.out.block_fire;
    g_m9.blocked=block;
    if(block){
        /* The slide adapter owns cancellation while blocked: m9_block_pad
         * keeps STATUS held at non-firing pressure with no release edge.
         * WAIT_REARM cannot observe release after input_ok is cleared, so
         * retire the generic fire gesture instead of starving radial fire_idle.
         * Consume this press; chamber obligation stays in g_m9.state. */
        dg_fire_reset(&g_b.fire);
        fire->input_ok=0;g_b.fire.handled_press=fire->press_seq;g_b.fire.started=1;
    }
    if(g_m9.last_state!=g_m9.out.state || g_m9.last_block!=block || events&(M9_GRAB|M9_END|M9_COMPLETE|M9_CLICK)) {
        if(g_b.log)g_b.log("  M9: state=%d blocked=%d progress=%.3f sample=%llu events=%u valid=%d allowed=%d contact=%d neutral=%d cancel=%d\r\n",
            g_m9.out.state,block,g_m9.out.progress,s->sequence,events,in.valid,in.allowed,in.contact_valid,g_m9.state.neutral,g_m9.out.cancel_reason);
        g_m9.last_state=g_m9.out.state;
        g_m9.last_block=block;
    }
    if(events&(M9_GRAB|M9_END|M9_COMPLETE))InterlockedOr(&g_m9_events,(LONG)events);
    if((events&M9_CLICK) && in.allowed && g_m9.base) {
        typedef void (__fastcall *M9_AUDIO)(int,const float *,int);
        float pos[4]={0,0,0,1};
        __try {
            uint64_t root=*(uint64_t *)(ULONG_PTR)(g_m9.objs+0x40);
            memcpy(pos,(void *)(ULONG_PTR)(root+0x30),sizeof pos);
            ((M9_AUDIO)(ULONG_PTR)(g_m9.base+0x5512f0))(4,pos,1);
        } __except(EXCEPTION_EXECUTE_HANDLER) {g_m9.fault=1;}
    }
done:
    ReleaseSRWLockExclusive(&g_m9_lock);return block;
}
/* A level-load/gameover boundary revokes the old chamber epoch even if
 * the allocator later reuses exactly the same player address. */
static void m9_context_boundary(int reset) {
    if(!g_m9_enabled)return;
    AcquireSRWLockExclusive(&g_m9_lock);
    if(reset && !g_m9.reset_context){
        g_m9.epoch++;dg_m9_reset(&g_m9.state);
        g_m9.pose_source=0;
        g_m9.out.attached=0;g_m9.render=0;g_m9.blocked=1;
    }
    g_m9.reset_context=reset;
    ReleaseSRWLockExclusive(&g_m9_lock);
}
static void m9_block_pad(LONG mask,LONG index) {
    uint64_t pad=g_b.a.player_pad;
    if(!mask || index<0 || index>=12 || !pad)return;
    /* Low held pressure is the native non-firing cancel path. In particular,
     * do not clear STATUS: a ready pistol fires on that falling edge. */
    *(volatile LONG *)(ULONG_PTR)(pad+DG_PAD_PRESS_OFFSET)&=~mask;
    *(volatile LONG *)(ULONG_PTR)(pad+DG_PAD_RELEASE_OFFSET)&=~mask;
    *(volatile LONG *)(ULONG_PTR)(pad+DG_PAD_STATUS_OFFSET)|=mask;
    *(volatile unsigned char *)(ULONG_PTR)(pad+DG_PAD_PRESSURE_OFFSET+index)=DG_FIRE_ABORT_PRESSURE;
}
static int m9_hand_contact(double local[3],uint64_t source) {
    int ok=0,k;uint64_t now=GetTickCount64();
    if(!g_m9_enabled || !TryAcquireSRWLockShared(&g_m9_lock))return 0;
    if(g_m9.active && g_m9.render && !g_m9.fault && g_m9.state.source==source &&
       g_m9.out.attached && now>=g_m9.state.now_ms &&
       now-g_m9.state.now_ms<=100) {
        /* Acquisition radius is input tolerance, never a permanent visual
         * gap. The hand solver eases its measured finger contact onto this
         * native slide anchor and subtracts its wrist-to-finger geometry. */
        for(k=0;k<3;k++)local[k]=g_m9.anchor[k];
        local[1]+=50.0*g_m9.out.progress;ok=1;
    }
    ReleaseSRWLockShared(&g_m9_lock);return ok;
}
static void m9_contact_cancel(void) {
    AcquireSRWLockExclusive(&g_m9_lock);
    if(g_m9.out.attached) {
        /* Geometry releases contact, never erases a confirmed rear endpoint. */
        g_m9.state.state=g_m9.state.state==M9_FULL_REAR?M9_RETURN_GOOD:
            g_m9.state.travel>0?M9_RETURN_BAD:M9_NEEDS_RACK;
        g_m9.state.full_samples=0;g_m9.state.grab_deadline=0;
        g_m9.state.grip_prev=1;g_m9.state.neutral=0;
        g_m9.out.attached=0;g_m9.out.state=g_m9.state.state;g_m9.blocked=1;
        InterlockedOr(&g_m9_events,M9_CANCEL);
    }
    ReleaseSRWLockExclusive(&g_m9_lock);
}
