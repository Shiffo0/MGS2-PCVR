/* Retail 2.1.0.0 only. Original cartridge actors own models, lighting,
 * lifetime and hand transforms; we change only their dispatch registers.
 * Ammo remains native; reload start time uses the native motion API. */
#include "dg_pistol_reload.h"
static SRWLOCK g_reload_lock=SRWLOCK_INIT;
static DG_DETOUR g_reload_gate_hook,g_reload_start_hook,g_reload_model_hook,g_reload_second_hook;
static struct {
    uint64_t base,player,arm,actor,actor_ms,tick_ms,anim_ms;
    int body_seek,arm_seek;
    int driving,cancel_ticks,active,resolved,enabled,live,fault,weapon,safe,valid,empty,reserve,waiting,anim,neutral;
    DG_M9_SAMPLE sample; /* same coherent two-hand sample, no second XR fetch */
    DG_RELOAD_GESTURE gesture;
} g_reload;
static int reload_match(const LiveImage *im,size_t r,const char *b,size_t n) {
    return r+n<=im->size && all_valid(im->valid,r,n,im->size) && !memcmp(im->bytes+r,b,n);
}
static void reload_resolve(const LiveImage *im) {
    memset(&g_reload,0,sizeof g_reload);
    if(!reload_match(im,0x67dd70,"\x40\x57\x48\x83\xec\x30\x48\x8b\xf9\x48\x8b\x49\x08",13) ||
       !reload_match(im,0x57b4dd,"\xe8\x8e\x28\x10\x00",5) ||
       !reload_match(im,0x57b7f0,"\xe8\x7b\x25\x10\x00",5) ||
       !reload_match(im,0x57ba5e,"\x44\x88\x81\xf0\x0c\x00\x00\x66\x89\x81\xf2\x0c\x00\x00",14) ||
       !reload_match(im,0x51c6f2,"\xba\x2f\x00\x00\x00\x48\x8b\xcb\xe8\x91\xea\x05\x00",13) ||
       !reload_match(im,0x51b9e8,"\x44\x8b\x71\x04\x44\x89\x74\x24\x48",9) ||
       !reload_match(im,0x51ba00,"\x89\x44\x24\x44",4) ||
       !reload_match(im,0x51c006,"\x8b\x44\x24\x48\x85\x05\xe0\x3e\x46\0",10) ||
       !reload_match(im,0x57bd0c,"\x48\x89\x91\x78\x0c\0\0",7) ||
       !reload_match(im,0x51bda4,"\x39\x3d\xa2\xdb\x1c\x01\x0f\x85\x9e\0\0\0",12) ||
       !reload_match(im,0x51bdb0,"\x8b\x8b\x90\x0b\0\0\xe8\x05\x8d\xb5\xff\x85\xc0\x0f\x8e\x8b\0\0\0",19) ||
       !reload_match(im,0x51bdc3,"\x48\x8b\x05\xb6\x39\x2c\x01",7) ||
       !reload_match(im,0x51be2b,"\x48\xc7\x83\xa8\x0c\0\0\x03\0\0\0",11) ||
       !reload_match(im,0x4d7358,"\x48\x8b\x47\x58\x8b\x08\x83\xe1\x0f\x83\xf9\x06",12) ||
       !reload_match(im,0x4d7558,"\x48\x8b\x47\x58\x8b\x08\xc1\xf9\x04\x83\xe1\x0f",12) ||
       !reload_match(im,0x4d79ab,"\x48\x89\x6b\x68\x48\x89\x73\x60\x89\xbb\x80\0\0\0",14) ||
       !reload_match(im,0x4d7417,"\x48\x8b\x08\x48\x81\xc1\x10\x01\0\0\x48\x03\xca",13) ||
       !reload_match(im,0x74ac0,"\x48\x8b\x05\x59\xc1\x4c\x01\x48\x63\xd1\x0f\xbf\x04\x50\xc3",15) ||
       !reload_match(im,0x550fb0,"\x8b\x05\x96\x89\x19\x01\xff\xc8\x89\x05\x8e\x89\x19\x01\xc3",15))return;
    {
        static const float usp[8]={17.5f,-115.9f,-1.2f,0,-17.5f,-110,-11.2f,0};
        static const float socom[8]={17.5f,-86,14,0,-17,-95,0,0};
        static const unsigned models[2]={3223612,77724};
        if(!reload_match(im,0x97bf40+2*64,(const char *)usp,sizeof usp) ||
           !reload_match(im,0x97bf40+8*64,(const char *)socom,sizeof socom) ||
           !reload_match(im,0x97beb0+2*4,(const char *)&models[0],4) ||
           !reload_match(im,0x97beb0+8*4,(const char *)&models[1],4))return;
    }
    /* Model selection + offsets are witnessed independently in research. */
    if(!all_valid(im->valid,0x97bf40,12*4*16,im->size) ||
       !reload_match(im,0x97bee4,(const char *)"\x0a\0\0\0",4))return;
    g_reload.base=im->base;g_reload.resolved=1;
}
static int reload_fresh(void) {
    uint64_t now=GetTickCount64();
    return g_reload.enabled && g_reload.live && !g_reload.fault && g_b.armed &&
        g_reload.safe && g_b.fire_mode==DG_FIRE_MODE_ON && g_b.fps.state==DG_FPS_ACTIVE &&
        !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0) &&
        now>=g_reload.tick_ms && now-g_reload.tick_ms<=100;
}
static int reload_visual_ready(void) {
    uint64_t now=GetTickCount64();
    return g_reload.actor && now>=g_reload.actor_ms && now-g_reload.actor_ms<=100;
}
/* This callback runs AFTER the original CMP, before its JNE. Only ZF is
 * cleared to take the existing no-reload branch; no register/ammo spoof. */
static void reload_gate(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    AcquireSRWLockExclusive(&g_reload_lock);
    if(reload_fresh() && r[11]==g_reload.player && (r[15]&0x40) &&
       g_reload.active && g_reload.reserve && !g_reload.anim) {
        g_reload.empty=1;g_reload.waiting=1;
        /* Model freshness grants permission; it must never lift an adopted reload obligation. */
        if(!g_reload.valid || !reload_visual_ready() || !g_reload.gesture.permit) {
            r[15]&=~0x40ULL;
            /* The native case continues past reload into its fire test. Its
             * pad snapshot was already copied BEFORE this seam. Hold only
             * that local weapon bit/pressure as well, so a physical release
             * or radial cancellation cannot shoot from an empty magazine.
             * These exact stack slots are witnessed above; native globals
             * and other buttons are unchanged. */
            *(unsigned *)((unsigned char *)rsp+0x48)|=(unsigned)RD32(g_b.a.pad_weapon);
            *(unsigned *)((unsigned char *)rsp+0x44)=128;
        }
    }
    ReleaseSRWLockExclusive(&g_reload_lock);
}
static void reload_start(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    AcquireSRWLockExclusive(&g_reload_lock);
    if(reload_fresh() && r[11]==g_reload.player && g_reload.waiting && g_reload.gesture.permit) {
        g_reload.body_seek=g_reload.arm_seek=0;
        g_reload.anim=1;g_reload.waiting=0;g_reload.anim_ms=GetTickCount64();g_reload.neutral=1;g_reload.cancel_ticks=0;
        dg_reload_gesture_reset(&g_reload.gesture);
        if(g_b.log)g_b.log("  reload: native animation started weapon=%d\r\n",g_reload.weapon);
    }
    ReleaseSRWLockExclusive(&g_reload_lock);
}






#define RELOAD_START_TICKS (46*5)
static void reload_motion(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16,caller;int layer,selected=0;
    AcquireSRWLockExclusive(&g_reload_lock);
    if(!reload_fresh() || !g_reload.anim || GetTickCount64()-g_reload.anim_ms>100)goto done;
    __try {
        caller=*(uint64_t *)rsp;layer=(int)r[12];
        if((int)r[7]<0 || r[6]!=0)__leave;
        if(r[13]==g_reload.player+0x290 && !g_reload.body_seek &&
           ((layer==1 && caller==g_reload.base+0x57b4e2 &&
             *(short *)(ULONG_PTR)(g_reload.player+0x13d0)!=2) ||
            (layer==0 && caller==g_reload.base+0x57b7f5 &&
             *(short *)(ULONG_PTR)(g_reload.player+0x13d0)==2))) {
            g_reload.body_seek=1;selected=1;
        } else if(r[13]==g_reload.arm && layer==0 && g_reload.body_seek && !g_reload.arm_seek &&
                  *(int *)(ULONG_PTR)(g_reload.player+0xca8)==3 &&
                  (*(unsigned char *)(ULONG_PTR)(g_reload.player+0xcf0)==3 ||
                   *(unsigned char *)(ULONG_PTR)(g_reload.player+0xcf0)==9)) {
            g_reload.arm_seek=1;selected=1;
            /* Existing game-to-XR queue: short left-hand contact pulse. */
            InterlockedOr(&g_m9_events,M9_GRAB);
        }
        if(selected) {
            r[6]=RELOAD_START_TICKS;
            /* 3 frames / 50ms blend, retaining a smaller native blend. */
            if(*(int *)((unsigned char *)rsp+0x30)>15)
                *(int *)((unsigned char *)rsp+0x30)=15;
            if(g_b.log)g_b.log("  reload: shortened %s start_frame=46 weapon=%d\r\n",
                r[13]==g_reload.arm?"arms":"body",g_reload.weapon);
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){g_reload.fault=1;}
done:ReleaseSRWLockExclusive(&g_reload_lock);
}
static int reload_point(uint64_t mat,const float *offset,double out[3]) {
    const float *m=(const float *)(ULONG_PTR)mat;int i,j;
    if(!mat)return 0;
    for(i=0;i<16;i++)if(!_finite(m[i]))return 0;
    for(j=0;j<3;j++){out[j]=m[12+j];for(i=0;i<3;i++)out[j]+=offset[i]*m[i*4+j];}
    return 1;
}
static void reload_model(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16,actor=r[8],body,weapon,objs,root,hand;
    int pat,expected,k,permitted;double a[3],b[3],distance=0;
    const float *shifts;
    AcquireSRWLockExclusive(&g_reload_lock);
    if(!reload_fresh())goto done;
    __try {
        body=*(uint64_t *)(ULONG_PTR)(actor+0x68);
        pat=*(int *)(ULONG_PTR)(actor+0x80);expected=g_reload.weapon==2?2:8;
        if(body!=g_reload.arm || pat!=expected) __leave;
        weapon=*(uint64_t *)(ULONG_PTR)(actor+0x60);
        if(!weapon || !*(uint64_t *)(ULONG_PTR)(actor+0x70) || !*(uint64_t *)(ULONG_PTR)(actor+0x78)) __leave;
        objs=*(uint64_t *)(ULONG_PTR)weapon;
        /* This adapter renders the first-person player in channel 0. Native
           constructors hide both channels; making FPS visible clears only
           INVISIBLE0. INVISIBLE1 is not evidence that channel 0 is hidden. */
        if(!objs || (*(unsigned *)(ULONG_PTR)(objs+0x58)&0x1000)) __leave;
        root=*(uint64_t *)(ULONG_PTR)(objs+0x40);if(!root)root=objs;

        hand=*(uint64_t *)(ULONG_PTR)body;
        if(!hand || *(short *)(ULONG_PTR)(hand+0x64)<11) __leave;
        hand+=0x110+10*0x180;
        shifts=(const float *)(ULONG_PTR)(g_reload.base+0x97bf40+pat*64);
        if(!reload_point(root,shifts,a) || !reload_point(hand,shifts+4,b)) __leave;
        for(k=0;k<3;k++)distance+=(a[k]-b[k])*(a[k]-b[k]);
        distance=sqrt(distance)/g_reload.sample.units_per_metre;
        g_reload.actor=actor;g_reload.actor_ms=GetTickCount64();
        if(g_reload.valid)g_reload.active=1;
        if(g_reload.active && g_reload.empty && g_reload.reserve && !g_reload.anim) {
            g_reload.waiting=1;g_reload.neutral=1;
            permitted=g_reload.gesture.permit;
            dg_reload_gesture(&g_reload.gesture,g_reload.sample.source,g_reload.sample.sequence,
                g_reload.tick_ms,g_reload.valid,distance);
            if(!permitted && g_reload.gesture.permit && g_b.log)
                g_b.log("  reload: magazine contact weapon=%d distance_m=%.3f sample=%llu\r\n",g_reload.weapon,distance,g_reload.sample.sequence);
            if(g_reload.valid)r[13]=2; /* ECX: original model, left-hand case */
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){g_reload.fault=1;}
done:ReleaseSRWLockExclusive(&g_reload_lock);
}
static void reload_second(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    AcquireSRWLockShared(&g_reload_lock);
    if(reload_fresh() && g_reload.valid && g_reload.waiting && !g_reload.anim &&
       r[8]==g_reload.actor && reload_visual_ready())r[13]=0; /* no duplicate */
    ReleaseSRWLockShared(&g_reload_lock);
}
/* Move the witnessed RIP-relative CMP ahead of pushfq. Its saved flags then
 * belong to the callback; NOP its old trampoline copy. Rest of detour unchanged.
 * Install occurs at startup with admission disabled. */
static int reload_after_cmp(DG_DETOUR *d,const char **why) {
    unsigned char *stub=d->stub;LONG disp;LONGLONG delta;
    size_t n=sizeof(k_stub_head)+7+8+sizeof(k_stub_tail)+8;
    if(d->stolen!=6 || memcmp(d->original,"\x39\x3d",2)) {*why="CMP contract";return 0;}
    memcpy(&disp,d->original+2,4);
    delta=(LONGLONG)(ULONG_PTR)d->target+6+disp-((LONGLONG)(ULONG_PTR)stub+6);
    if(delta>0x7fffffffLL || delta< -0x80000000LL){*why="CMP relocation";return 0;}
    memmove(stub+6,stub,n);memcpy(stub,d->original,6);disp=(LONG)delta;memcpy(stub+2,&disp,4);
    memset(d->tramp,0x90,6);FlushInstructionCache(GetCurrentProcess(),d->page,DG_DETOUR_PAGE);
    return 1;
}
static void reload_stop(void) {
    g_reload.enabled=0;
    motion_release(DG_MOTION_RELOAD);
    dg_detour_remove(&g_reload_gate_hook);dg_detour_remove(&g_reload_start_hook);
    dg_detour_remove(&g_reload_model_hook);dg_detour_remove(&g_reload_second_hook);
    g_reload.live=g_reload.anim=g_reload.waiting=0;
}
static void reload_install(int enabled) {
    const char *why="retail witnesses unavailable";uint64_t b=g_reload.base;
    if(!enabled)return;
    if(g_reload.resolved &&
       motion_acquire(b,DG_MOTION_RELOAD,&why) &&
       dg_detour_install_ex(&g_reload_gate_hook,(void *)(ULONG_PTR)(b+0x51bda4),(void *)reload_gate,
            (void *)(ULONG_PTR)(b+0x51b980),(void *)(ULONG_PTR)(b+0x51d012),&why,1) &&
       reload_after_cmp(&g_reload_gate_hook,&why) &&
       dg_detour_install_ex(&g_reload_start_hook,(void *)(ULONG_PTR)(b+0x51bdc3),(void *)reload_start,
            (void *)(ULONG_PTR)(b+0x51b980),(void *)(ULONG_PTR)(b+0x51d012),&why,1) &&
       dg_detour_install_ex(&g_reload_model_hook,(void *)(ULONG_PTR)(b+0x4d735e),(void *)reload_model,
            (void *)(ULONG_PTR)(b+0x4d7210),(void *)(ULONG_PTR)(b+0x4d7858),&why,1) &&
       dg_detour_install_ex(&g_reload_second_hook,(void *)(ULONG_PTR)(b+0x4d755e),(void *)reload_second,
            (void *)(ULONG_PTR)(b+0x4d7210),(void *)(ULONG_PTR)(b+0x4d7858),&why,1))g_reload.live=1;
    if(!g_reload.live)reload_stop();else g_reload.enabled=1;
    if(g_b.log)g_b.log("  reload: USP/SOCOM adapter %s (%s)\r\n",g_reload.live?"installed":"unavailable",why);
}
static int reload_native_animation(void) {
    int result;AcquireSRWLockShared(&g_reload_lock);
    result=reload_fresh() && g_reload.anim;
    ReleaseSRWLockShared(&g_reload_lock);return result;
}
static int reload_owns_left(void) {
    int result;AcquireSRWLockShared(&g_reload_lock);
    result=reload_fresh() && g_reload.waiting;
    ReleaseSRWLockShared(&g_reload_lock);return result;
}
static int reload_tick(int safe,int writable,uint64_t player,int weapon,int physical,DG_FIRE_IN *fire) {
    int block=0,was_waiting=g_reload.waiting;uint64_t arm=0,inventory;DG_M9_SAMPLE *s=&g_controls_frame.m9;
    AcquireSRWLockExclusive(&g_reload_lock);
    g_reload.safe=0;
    if(!g_reload.enabled || !g_reload.live || g_reload.fault)goto done;
    __try {
        if(g_b.a.gm_player_arm_body)arm=*(uint64_t *)(ULONG_PTR)g_b.a.gm_player_arm_body;
        if(!safe || !writable || !player || !arm || (weapon!=2 && weapon!=3) || !g_controls_active ||
           g_b.fps.state!=DG_FPS_ACTIVE || InterlockedCompareExchange(&g_b.s_late_unsafe,0,0)) {
            g_reload.driving=g_reload.cancel_ticks=g_reload.active=g_reload.waiting=g_reload.anim=g_reload.neutral=0;g_reload.actor=0;
            dg_reload_gesture_reset(&g_reload.gesture);__leave;
        }
        if(player!=g_reload.player || arm!=g_reload.arm || weapon!=g_reload.weapon) {
            g_reload.actor=0;g_reload.driving=g_reload.cancel_ticks=g_reload.active=g_reload.waiting=g_reload.anim=g_reload.neutral=0;
            dg_reload_gesture_reset(&g_reload.gesture);
        }
        g_reload.player=player;g_reload.arm=arm;g_reload.weapon=weapon;
        g_reload.safe=1;g_reload.tick_ms=GetTickCount64();g_reload.sample=*s;
        g_reload.valid=s->valid && s->allowed && s->units_per_metre>=334 && s->units_per_metre<=10000;
        if(!g_reload.valid || !reload_visual_ready() || (g_reload.gesture.source && s->source!=g_reload.gesture.source) ||
           s->sequence<g_reload.gesture.sequence || g_reload.tick_ms<g_reload.gesture.sample_ms ||
           (g_reload.gesture.sequence && g_reload.tick_ms-g_reload.gesture.sample_ms>100) ||
           (g_reload.gesture.permit && g_reload.tick_ms-g_reload.gesture.permit_ms>500))
            dg_reload_gesture_reset(&g_reload.gesture);
        g_reload.empty=*(int *)(ULONG_PTR)(g_reload.base+0x16e994c)==0;
        inventory=*(uint64_t *)(ULONG_PTR)(g_reload.base+0x1540c20);
        g_reload.reserve=inventory && *(short *)(ULONG_PTR)(inventory+weapon*2)>0;
        if(g_reload.anim && (*(uint64_t *)(ULONG_PTR)(player+0xc78)!=g_reload.base+0x51b980 ||
           *(int *)(ULONG_PTR)(player+0xca8)!=3 ||
           g_reload.tick_ms-g_reload.anim_ms>5000))g_reload.anim=0;
        if(g_reload.active && g_reload.empty && g_reload.reserve && !g_reload.anim)
            g_reload.waiting=g_reload.neutral=1;
        if(!g_reload.empty || !g_reload.reserve){g_reload.waiting=0;dg_reload_gesture_reset(&g_reload.gesture);}
        if(!g_reload.anim && !g_reload.waiting && g_reload.neutral && s->allowed) {
            if(g_reload.cancel_ticks>=DG_FIRE_ABORT_TICKS && !s->trigger && !physical && g_reload.valid)g_reload.neutral=0;
            else g_reload.cancel_ticks++;
        }
        block=(g_reload.waiting || g_reload.anim || g_reload.neutral) && s->allowed;
        if(!block)g_reload.driving=0;
        if(!was_waiting && g_reload.waiting && g_b.log)
            g_b.log("  reload: waiting for left magazine weapon=%d\r\n",weapon);
        if(block){
            fire->input_ok=0;g_b.fire.handled_press=fire->press_seq;g_b.fire.started=1;
            if(g_b.fire.state==DG_FIRE_DRAW || g_b.fire.state==DG_FIRE_HOLD){g_b.fire.state=DG_FIRE_ABORT;g_b.fire.ticks=0;}
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){g_reload.fault=1;g_reload.safe=0;}
done:
    if(g_reload.fault && g_reload.enabled){
        g_reload.enabled=0;
        if(g_b.log)g_b.log("  reload: adapter fault; native behavior restored\r\n");
    }
    ReleaseSRWLockExclusive(&g_reload_lock);return block;
}

/* A pistol fires on RELEASE. Keep native ready at ordinary held pressure
 * while waiting/reloading; low pressure would holster before the gesture.
 * After the animation, soft-cancel for at least three ticks before letting
 * the held status fall. Never synthesize a firing release. */
static void reload_pad(LONG mask,LONG index) {
    AcquireSRWLockExclusive(&g_reload_lock);
    m9_block_pad(mask,index);
    if(mask && index>=0 && index<12 && g_b.a.player_pad &&
       (g_reload.waiting || g_reload.anim)) {
        *(volatile unsigned char *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_PRESSURE_OFFSET+index)=128;
        if(!g_reload.driving)
            *(volatile LONG *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_PRESS_OFFSET)|=mask;
        g_reload.driving=1;
    }
    ReleaseSRWLockExclusive(&g_reload_lock);
}
