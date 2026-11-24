/* Retail 2.1.0.0 only. Original cartridge actors own models, lighting,
 * lifetime and hand transforms; we change only their dispatch registers.
 * Ammo remains native; reload start time uses the native motion API. */
#include "dg_pistol_reload.h"
#include "dg_rifle_hold.h"
static SRWLOCK g_reload_lock=SRWLOCK_INIT;
static DG_DETOUR g_reload_gate_hook,g_reload_start_hook,g_reload_model_hook,g_reload_second_hook;
static DG_DETOUR g_reload_tail_hook;
static struct {
    uint64_t base,player,arm,actor,actor_ms,tick_ms,anim_ms;
    int body_seek,arm_seek,rifles_resolved;
    int driving,cancel_ticks,active,resolved,enabled,live,fault,weapon,safe,valid,empty,reserve,waiting,anim,neutral;
    DG_M9_SAMPLE sample; /* same coherent two-hand sample, no second XR fetch */
    DG_RELOAD_GESTURE gesture;

    int v2_resolved,v2,ejected,eject_mag,grip_armed,trigger_was,fills,ejects;
    void (__fastcall *set_magazine)(int,int);

    int insert_mode,slide_resolved,slide_live,ins_idx;
    DG_RELOAD_INSERT insert;
    double guide_wrist[3], release_offset[3];
    uint64_t release_ms;
    int guide_detached;
    double ins_shift[3];uint64_t ins_actor,ins_ms;volatile LONG ins_tail_hits;unsigned ins_lines;

    double ins_left[3];uint64_t ins_left_ms;int ins_from_ctrl,ins_owed;double ins_hand_off_mm;
    /* Rifle hint: the actor's second magazine object (+0x78, same model) shown seated in the well while a
       magazine is owed; dg_reload_highlight.inl colours it blue. */
    int ghost_resolved,ghost_logged;volatile LONG ghost_hits;
} g_reload;
static int left_support_engaged(void); /* dg_left_arm.inl */
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
    {
        static const float aks[]={20,-247,-13,0,-18,-75,-15,0};
        static const float m4[]={20,-225,0,0,-17,-90,0,0};
        static const unsigned models[]={6368044,10823954};
        g_reload.rifles_resolved=reload_match(im,0x97bf40+7*64,(const char*)aks,sizeof aks) &&
            reload_match(im,0x97bf40+4*64,(const char*)m4,sizeof m4) &&
            reload_match(im,0x97beb0+7*4,(const char*)&models[0],4) &&
            reload_match(im,0x97beb0+4*4,(const char*)&models[1],4);
    }

    if(reload_match(im,0x551420,"\x40\x53\x48\x83\xec\x20\x44\x8b\x05\x1b\x85\x19\x01\x85\xd2\x48\x63\xd9"
           "\x48\x8d\x0d\x97\x62\x3f\x00\x44\x0f\x4f\xc2\x44\x89\x05\x04\x85\x19\x01\xf7\x04\x99\x00\x10\x00\x00"
           "\x74\x15\x83\x3d\xf8\x84\x19\x01\x00\x7e\x0c\x41\x8d\x40\x01\x89\x05\xec\x84\x19\x01\xeb\x07\x44\x89"
           "\x05\xe3\x84\x19\x01\x8b\xcb\xe8\x50\x36\xb2\xff\x39\x05\xd6\x84\x19\x01\x7e\x0d\x8b\xcb\xe8\x41\x36"
           "\xb2\xff\x89\x05\xc7\x84\x19\x01\x48\x83\xc4\x20\x5b\xc3",0x6b) &&
       reload_match(im,0x5512f0,"\x40\x56\x57\x48\x83\xec\x48\x48\x8b\x05\xba\x91\x43\x00\x48\x33",16)) {
        g_reload.set_magazine=(void (__fastcall *)(int,int))(ULONG_PTR)(im->base+0x551420);
        g_reload.v2_resolved=1;
    }

    if(g_reload.v2_resolved &&
       reload_match(im,0x4d733a,"\x4c\x8d\x3d\xff\x4b\x4a\x00\x8b\x87\x80\x00\x00\x00",13) &&
       reload_match(im,0x4d7351,"\x8d\x34\x85\x00\x00\x00\x00",7) &&
       reload_match(im,0x4d73da,"\xe8\x71\x19\xbb\xff\x48\x63\xde\xe9\x49\x01\x00\x00",13) &&
       reload_match(im,0x4d7530,"\x48\x8b\xcb\x48\xc1\xe1\x04\x49\x03\xcf\xe8\xe1\xf6\xba\xff\x49\x8d\x8e\x40\xc2\x97\x00\x48\x8d\x0c\xd9\xe8\x21\x10\xbb\xff",31) &&
       reload_match(im,0x88d50,"\xf3\x0f\x10\x01\xf3\x0f\x11\x05\xb4\x94\x4c\x01",12) &&
       reload_match(im,0x86c20,"\x48\x83\xec\x38\x0f\x28\x05\xe5\xb5\x4c\x01\x48\x8b\xd1\x0f\x29\x05\x1b\xb6\x4c\x01",21) &&
       reload_match(im,0x86c75,"\xf3\x0f\x11\x05\xc3\xb5\x4c\x01",8) &&
       reload_match(im,0x4d7858+4,"\xaf\x73\x4d\x00\xe7\x73\x4d\x00",8) &&
       /* table rotations: USP seated/hand 0, SOCOM seated 0, hand (171,0,0) */
       reload_match(im,0x97c240+8*8,"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0",16) &&
       reload_match(im,0x97c240+32*8,"\0\0\0\0\0\0\0\0\xab\0\0\0\0\0\0\0",16))
        g_reload.slide_resolved=1;

    /* 0x4D7558 itself is witnessed above (rifles_resolved implies resolved); no new read over the own hook
     * at 0x4D755E. */
    g_reload.ghost_resolved=g_reload.rifles_resolved &&
        reload_match(im,0x4d7874,"\x7d\x75\x4d\x00\xb2\x75\x4d\x00",8) &&
        reload_match(im,0x4d75b2,"\x48\x8b\x47\x78\x81\x60\x58\xff\xcf\xff\xff",11) &&
        reload_match(im,0x4d75cb,"\x48\x8b\x47\x60\x48\x8b\x08\x48\x8b\x41\x40\x48\x85\xc0\x48\x0f\x45\xc8\xe8\x6e\x17\xbb\xff\x48\x63\xde",26) &&
        reload_match(im,0x4d770d,"\x48\x8b\xcb\x48\xc1\xe1\x04\x49\x03\xcf\xe8\x04\xf5\xba\xff\x49\x8d\x8e\x40\xc2\x97\x00",22) &&
        reload_match(im,0x4d79c5,"\x45\x33\xc0\x4c\x8d\x0d\x91\x33\x28\x00\x48\x8b\xc8\x48\x8b\xf8\x41\x8d\x50\x30",20) &&
        reload_match(im,0x4d7a17,"\x45\x33\xc0\x4c\x8d\x0d\x3f\x33\x28\x00\x48\x8b\xcf\x41\x8d\x50\x30",17);
}

static volatile int *reload_magazine(void) {return (volatile int *)(ULONG_PTR)(g_reload.base+0x16e994c);}
static void reload_v2_events(LONG events) {InterlockedOr(&g_m9_events,events);}

static int reload_direct_fill(void) {
    int wp,before,after;
    if(!g_reload.v2_resolved || !g_reload.set_magazine)return 0;
    __try {
        wp=*(int *)(ULONG_PTR)(g_reload.player+0xb90);
        if(wp!=g_reload.weapon || dg_reload_pattern(wp)<0)return 0;
        before=*reload_magazine();
        g_reload.set_magazine(wp,0);
        after=*reload_magazine();
    } __except(EXCEPTION_EXECUTE_HANDLER){g_reload.fault=1;return 0;}
    g_reload.fills++;g_reload.ejected=0;g_reload.waiting=0;g_reload.empty=after==0;
    g_reload.neutral=1;g_reload.cancel_ticks=0;dg_reload_gesture_reset(&g_reload.gesture);
    dg_reload_insert_reset(&g_reload.insert);
    reload_v2_events(DG_RELOAD_EV_FILL);
    if(g_b.log)g_b.log("  reload v2: direct fill weapon=%d magazine %d -> %d (fill %d)\r\n",wp,before,after,g_reload.fills);
    return 1;
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

static void reload_hold_snapshot(void *rsp) {
    if(dg_reload_rifle(g_reload.weapon)) {
        *(unsigned *)((unsigned char *)rsp+0x48)&=~(unsigned)RD32(g_b.a.pad_weapon);
        *(unsigned *)((unsigned char *)rsp+0x44)=0;
        return;
    }
    *(unsigned *)((unsigned char *)rsp+0x48)|=(unsigned)RD32(g_b.a.pad_weapon);
    *(unsigned *)((unsigned char *)rsp+0x44)=128;
}

static int reload_v2_contact(void) {
    return g_reload.insert_mode ? g_reload.insert.phase==DG_INS_SEATED : g_reload.gesture.permit;
}

static void reload_v2_gate(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;int zf=(r[15]&0x40)!=0;
    if(!reload_fresh() || r[11]!=g_reload.player || !g_reload.active ||
       !g_reload.reserve || g_reload.anim || (!zf && !g_reload.ejected))return;
    if(zf)g_reload.empty=1;
    g_reload.waiting=1;
    if(g_reload.valid && reload_visual_ready() && reload_v2_contact() && reload_direct_fill()) {
        if(zf){r[15]&=~0x40ULL;reload_hold_snapshot(rsp);} /* loaded now: skip the reload branch */
        return;
    }
    if(zf){r[15]&=~0x40ULL;reload_hold_snapshot(rsp);}
}
static void reload_gate(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    AcquireSRWLockExclusive(&g_reload_lock);
    if(g_reload.v2)reload_v2_gate(rsp);
    else if(reload_fresh() && r[11]==g_reload.player && (r[15]&0x40) &&
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

static struct {
    double magwell[3],dist_m;ULONGLONG world_ms;
    int trk_ok;const char *trk_why;double magwell_trk[3],left_trk[3];ULONGLONG trk_ms;
    ULONGLONG last_ms;unsigned lines;

    double mag[3],well_pt[3],held_pt[3];ULONGLONG geo_ms;
    int geo_trk_ok;double mag_trk[3],well_trk[3],held_trk[3];ULONGLONG geo_trk_ms;
} g_reload_probe;

static void reload_probe_new_magazine(void) {g_reload_probe.lines=0;g_reload_probe.last_ms=0;}
static void reload_probe_log(int weapon) {
    char line[1100];ULONGLONG now=GetTickCount64();
    int fresh=g_reload_probe.world_ms && now-g_reload_probe.world_ms<500;
    int trk=fresh && g_reload_probe.trk_ok && now-g_reload_probe.trk_ms<500;
    uint64_t held=g_reload.gesture.near_ms && now>=g_reload.gesture.near_ms?now-g_reload.gesture.near_ms:0;
    if(!g_b.log || !g_reload.waiting || g_reload_probe.lines>=DG_RELOAD_PROBE_LINES || now-g_reload_probe.last_ms<1000)return;
    g_reload_probe.last_ms=now;
    if(++g_reload_probe.lines==DG_RELOAD_PROBE_LINES) {
        g_b.log("  reload probe: %u lines for this magazine, quiet until the next one\r\n",g_reload_probe.lines);
        return;
    }
    dg_reload_probe_format(line,sizeof line,weapon,g_reload.ejected,trk,
        !fresh?"weapon model not seen in the last 0.5 s":g_reload_probe.trk_why?g_reload_probe.trk_why:"left arm solve not running",
        g_reload_probe.magwell_trk,g_reload_probe.left_trk,fresh?g_reload_probe.dist_m:NAN,g_reload.gesture.outside,held,
        g_reload.valid,g_reload.gesture.permit);
    {
        size_t n=strlen(line);
        if(n>=2 && line[n-2]=='\r')n-=2;
        line[n]=0;
        if(g_reload.v2 && n+1<sizeof line && dg_reload_insert_probe_format(line+n,sizeof line-n,&g_reload.insert,!g_reload.insert_mode)>0)
            n=strlen(line);
        if(g_reload.insert_mode && n+1<sizeof line) {
            int geo=trk && g_reload_probe.geo_trk_ok && g_reload_probe.geo_ms && now-g_reload_probe.geo_ms<500 &&
                    now-g_reload_probe.geo_trk_ms<500;
            if(dg_reload_insert_geo_format(line+n,sizeof line-n,geo,g_reload_probe.magwell_trk,g_reload_probe.well_trk,
                   g_reload_probe.mag_trk,g_reload_probe.held_trk,g_reload.ins_from_ctrl,g_reload.ins_hand_off_mm)>0)
                n=strlen(line);
        }
        if(n+3<=sizeof line)memcpy(line+n,"\r\n",3);
    }
    g_b.log("%s",line);
}

static DG_RIFLE_HOLD_METER g_rifle_hold;
static void reload_rifle_hold_meter(int weapon) {
    uint64_t now=GetTickCount64();int hold,due,ws=-1,bs=-1;unsigned flips,mask=0,st=0,pr=0,rel=0;DG_RIFLE_HOLD_SEAM seam;
    if(!dg_reload_rifle(weapon) || !(g_reload.waiting || g_reload.neutral) || !g_b.a.gm_player_status) {
        if(g_rifle_hold.have)dg_rifle_hold_meter_reset(&g_rifle_hold);
        return;
    }
    hold=(*(volatile uint64_t *)(ULONG_PTR)g_b.a.gm_player_status & 0x800ULL)!=0;

    seam=g_rifle_hold.seam;g_rifle_hold.seam.called=0;
    if(g_b.a.pad_weapon)mask=(unsigned)RD32(g_b.a.pad_weapon);
    if(g_b.a.player_pad) {
        st=(unsigned)RD32(g_b.a.player_pad+DG_PAD_STATUS_OFFSET);pr=(unsigned)RD32(g_b.a.player_pad+DG_PAD_PRESS_OFFSET);
        rel=(unsigned)RD32(g_b.a.player_pad+DG_PAD_RELEASE_OFFSET);
        ws=(int)RD32(g_b.a.player_pad+DG_PAD_WEAPON_STATE_OFFSET);bs=(int)RD32(g_b.a.player_pad+DG_PAD_BUTTON_STATE_OFFSET);
    }
    if(ws==DG_RIFLE_HOLD_WS_DRAW)g_rifle_hold.draw_ticks++;
    if(g_b.log && seam.called && seam.released && g_rifle_hold.release_lines<DG_RIFLE_HOLD_RELEASE_LINES) {
        g_rifle_hold.release_lines++;
        g_b.log("  rifle hold release: weapon %d | at the seam: status 0x%X press 0x%X release 0x%X, weaponState %d buttonState %d"
                " -> wrote status 0x%X release 0x%X | the layer answered weaponState %d buttonState %d (2 = still draw state)\r\n",
                weapon,seam.status_in,seam.press_in,seam.release_in,seam.weapon_state,seam.button_state,
                seam.status_out,seam.release_out,ws,bs);
    }
    flips=g_rifle_hold.total_flips;
    due=dg_rifle_hold_meter_step(&g_rifle_hold,hold,now);
    if(g_rifle_hold.total_flips!=flips && g_b.log && g_rifle_hold.flip_lines<DG_RIFLE_HOLD_FLIP_LINES) {
        int ftime=-1,data3=-1;
        if(g_reload.player){ftime=*(volatile int *)(ULONG_PTR)(g_reload.player+0xc84);data3=*(volatile int *)(ULONG_PTR)(g_reload.player+0xca8);}
        g_rifle_hold.flip_lines++;
        g_b.log("  rifle hold flip: weapon %d HOLD %d->%d (flip %u) | pad now status 0x%X press 0x%X release 0x%X, weaponState %d buttonState %d"
                " | previous seam: %s, status in 0x%X out 0x%X, release out 0x%X, weaponState %d | ftime %d data3 %d | source: %s\r\n",
                weapon,!hold,hold,g_rifle_hold.total_flips,st,pr,rel,ws,bs,
                seam.called?(seam.released?"reload_pad released":"reload_pad wrote no release"):"reload_pad not called",
                seam.status_in,seam.status_out,seam.release_out,seam.weapon_state,ftime,data3,dg_rifle_hold_source(&seam,mask));
    }
    if(!due)return;
    if(g_b.log && g_rifle_hold.lines<DG_RIFLE_HOLD_LINES) {
        g_rifle_hold.lines++;
        g_b.log("  rifle hold meter: weapon %d %s | player hold now %d, %u flips in %llu ms over %u ticks (%u since owed) | "
                "latched weapon button released on %u ticks | layer draw state at the seam on %u ticks (%u releases since owed)\r\n",
                weapon,g_reload.waiting?(g_reload.ejected?"ejected":"empty"):"post-fill",
                hold,g_rifle_hold.flips,(unsigned long long)(now-g_rifle_hold.start_ms),g_rifle_hold.ticks,
                g_rifle_hold.total_flips,g_rifle_hold.latched,g_rifle_hold.draw_ticks,g_rifle_hold.total_released);
    }
    dg_rifle_hold_meter_window(&g_rifle_hold,now);
}
static int reload_point(uint64_t mat,const float *offset,double out[3]) {
    const float *m=(const float *)(ULONG_PTR)mat;int i,j;
    if(!mat)return 0;
    for(i=0;i<16;i++)if(!_finite(m[i]))return 0;
    for(j=0;j<3;j++){out[j]=m[12+j];for(i=0;i<3;i++)out[j]+=offset[i]*m[i*4+j];}
    return 1;
}

static const double dg_reload_well_axis[3]={0.0,-0.33826,0.94105};
/* Rifles (AKS, M4): the magazine goes in square to the bore, root +Z (see DG_RIFLE_* in the header). */
static const double dg_reload_rifle_well_axis[3]={0.0,0.0,1.0};

static void reload_left_target(const double w[3]) {
    if(!g_reload.insert_mode || !w || !_finite(w[0]) || !_finite(w[1]) || !_finite(w[2]))return;
    AcquireSRWLockExclusive(&g_reload_lock);
    memcpy(g_reload.ins_left,w,sizeof g_reload.ins_left);g_reload.ins_left_ms=GetTickCount64();
    ReleaseSRWLockExclusive(&g_reload_lock);
}
/* Raw tracking drives the insertion gesture. Only the rendered IK target follows the magazine. */
static void reload_visual_wrist(uint64_t arm, uint64_t weapon, double target[3]) {
    uint64_t now=GetTickCount64();
    AcquireSRWLockExclusive(&g_reload_lock);
    if (weapon==3 && g_reload.weapon==3 && g_reload.arm==arm && g_reload.valid &&
        g_reload.insert_mode && g_reload.slide_live && g_reload.ins_owed &&
        now-g_reload.ins_ms<=100 && g_reload.insert.phase==DG_INS_GUIDED) {
        double distance=0;
        for(int k=0;k<3;k++) {
            double d=g_reload.guide_wrist[k]-target[k];
            distance+=d*d;
        }
        if(!g_reload.guide_detached) {
            if(sqrt(distance)>0.30*g_reload.sample.units_per_metre) {
                g_reload.guide_detached=1;
                g_reload.release_ms=now;
            }
            for(int k=0;k<3;k++)g_reload.release_offset[k]=g_reload.guide_wrist[k]-target[k];
        }
        double w=g_reload.guide_detached ? fmax(0.0,1.0-(now-g_reload.release_ms)/150.0) : 1.0;
        for(int k=0;k<3;k++)target[k]+=w*g_reload.release_offset[k];
    }
    ReleaseSRWLockExclusive(&g_reload_lock);
}
/* Inside reload_model (lock held, game thread), only while the magazine is owed and in insert mode. a/b are
 * the seated and held magazine origins (world). The held magazine's frame is the hand joint turned by its
 * table rotation about X (SOCOM ~15 degrees, USP 0); the sign convention of that rotation is not witnessed,
 * so both signs are measured and the smaller angle counts. */
static void reload_insert_model(uint64_t *r,uint64_t actor,uint64_t root,uint64_t hand,const double a[3],
                                const double b_drawn[3],int pat) {
    const float *R=(const float *)(ULONG_PTR)root,*H=(const float *)(ULONG_PTR)hand;
    double u[3],hp[3],hm[3],along=NAN,side=NAN,deg=NAN,deg2,al2,si2,ang,c,s,ax[2][3],nu,mm,b[3],off=0;
    short rx=*(const short *)(ULONG_PTR)(g_reload.base+0x97c240+(pat*4+1)*8);
    int k,before=g_reload.insert.phase,phase,ok,minus=0,rifle=dg_reload_rifle(g_reload.weapon);
    const double *wa=rifle ? dg_reload_rifle_well_axis : dg_reload_well_axis;
    uint64_t now=GetTickCount64();

    g_reload.ins_from_ctrl=g_reload.ins_left_ms && now>=g_reload.ins_left_ms && now-g_reload.ins_left_ms<=100;
    for(k=0;k<3;k++) {
        b[k]=b_drawn[k];
        if(g_reload.ins_from_ctrl){double d=g_reload.ins_left[k]-H[12+k];b[k]+=d;off+=d*d;}
    }
    g_reload.ins_hand_off_mm=g_reload.ins_from_ctrl && g_reload.sample.units_per_metre>0 ?
        sqrt(off)/g_reload.sample.units_per_metre*1000.0 : 0;
    ang=rx*6.283185307179586/4096.0;c=cos(ang);s=sin(ang);
    ax[0][0]=ax[1][0]=wa[0];
    ax[0][1]=wa[1]*c-wa[2]*s;ax[0][2]=wa[1]*s+wa[2]*c;
    ax[1][1]=wa[1]*c+wa[2]*s;ax[1][2]=-wa[1]*s+wa[2]*c;
    for(k=0;k<3;k++) {
        u[k]=wa[0]*R[k]+wa[1]*R[4+k]+wa[2]*R[8+k];
        hp[k]=ax[0][0]*H[k]+ax[0][1]*H[4+k]+ax[0][2]*H[8+k];
        hm[k]=ax[1][0]*H[k]+ax[1][1]*H[4+k]+ax[1][2]*H[8+k];
    }
    ok=dg_reload_insert_measure(a,b,u,hp,g_reload.sample.units_per_metre,&along,&side,&deg) &&
       dg_reload_insert_measure(a,b,u,hm,g_reload.sample.units_per_metre,&al2,&si2,&deg2);
    if(ok && deg2<deg){deg=deg2;minus=1;}
    if(ok) {

        double nh,nw,*hs=minus?hm:hp,up=g_reload.sample.units_per_metre/10.0;
        nw=sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);nh=sqrt(hs[0]*hs[0]+hs[1]*hs[1]+hs[2]*hs[2]);
        for(k=0;k<3;k++) {
            g_reload_probe.mag[k]=b[k];
            g_reload_probe.well_pt[k]=a[k]+u[k]/nw*up;g_reload_probe.held_pt[k]=b[k]+hs[k]/nh*up;
        }
        g_reload_probe.geo_ms=now;
    }
    g_reload.insert.socom=rifle ? DG_INS_MODE_RIFLE : g_reload.weapon==3 ? DG_INS_MODE_SOCOM : DG_INS_MODE_USP;
    phase=dg_reload_insert_step(&g_reload.insert,g_reload.sample.source,g_reload.sample.sequence,g_reload.tick_ms,
                                ok && g_reload.valid,along,side,deg);
    if(phase!=DG_INS_HAND && g_reload.slide_live && g_reload.valid) {
        nu=sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);
        if(!(nu>1e-9))return;
        if(before==DG_INS_HAND) {
            /* Store the initial visual offset in weapon space so it follows a turning grip. */
            for(int axis=0;axis<3;axis++) {
                double local=0;
                for(k=0;k<3;k++)local+=(b_drawn[k]-a[k]-u[k]/nu*along*g_reload.sample.units_per_metre/1000.0)*R[4*axis+k];
                g_reload.insert.capture_offset[axis]=local;
            }
        }
        double blend=fmax(0.0,1.0-fmax(0.0,g_reload.insert.last_along-g_reload.insert.capture_along)/25.0);
        mm=dg_reload_insert_draw_mm(phase==DG_INS_SEATED?0:g_reload.insert.last_along,g_reload.insert.socom)/1000.0*g_reload.sample.units_per_metre;
        for(k=0;k<3;k++)g_reload.ins_shift[k]=u[k]/nu*mm + blend*(g_reload.insert.capture_offset[0]*R[k]+g_reload.insert.capture_offset[1]*R[4+k]+g_reload.insert.capture_offset[2]*R[8+k]);
        for(k=0;k<3;k++)g_reload.guide_wrist[k]=a[k]+g_reload.ins_shift[k]-(b_drawn[k]-H[12+k]);
        if(before==DG_INS_HAND)g_reload.guide_detached=0;
        g_reload.ins_actor=actor;g_reload.ins_idx=pat*4;g_reload.ins_ms=GetTickCount64();
        r[13]=1; /* ECX: the seated case, moved down the axis by the tail seam */
    }
    if(before==DG_INS_HAND && phase==DG_INS_GUIDED)reload_v2_events(DG_RELOAD_EV_GUIDE);
    if(phase!=before && g_b.log && g_reload.ins_lines<DG_INSERT_LOG_LINES) {

        if(++g_reload.ins_lines==DG_INSERT_LOG_LINES)
            g_b.log("  reload v3: %u phase changes for this magazine, quiet until the next one\r\n",g_reload.ins_lines);
        else g_b.log("  reload v3: magazine %s -> %s weapon=%d along %.0f mm side %.0f mm axis %.0f deg (%s; %s)\r\n",
            dg_insert_phase_names[before],dg_insert_phase_names[phase],g_reload.weapon,g_reload.insert.along,
            g_reload.insert.side,g_reload.insert.deg,dg_insert_why_names[g_reload.insert.why],
            g_reload.ins_from_ctrl?"measured at the controller":"measured at the drawn hand");
    }
}

static void reload_slide_tail(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    AcquireSRWLockShared(&g_reload_lock);
    if(g_reload.insert_mode && g_reload.slide_live && reload_fresh() && g_reload.ins_actor &&
       r[8]==g_reload.ins_actor && r[11]==(uint64_t)g_reload.ins_idx && g_reload.insert.phase!=DG_INS_HAND &&
       GetTickCount64()-g_reload.ins_ms<=100) {
        __try {
            float *t=(float *)(ULONG_PTR)(g_reload.base+0x1552240);int k;
            for(k=0;k<3;k++)t[k]+=(float)g_reload.ins_shift[k];
            InterlockedIncrement(&g_reload.ins_tail_hits);
        } __except(EXCEPTION_EXECUTE_HANDLER){}
    }
    ReleaseSRWLockShared(&g_reload_lock);
}
static void reload_slide_install(void) {
    const char *why="retail witnesses unavailable";uint64_t b=g_reload.base;
    if(g_reload.slide_resolved && g_reload.live &&
       dg_detour_install_ex(&g_reload_tail_hook,(void *)(ULONG_PTR)(b+0x4d7530),(void *)reload_slide_tail,
            (void *)(ULONG_PTR)(b+0x4d7210),(void *)(ULONG_PTR)(b+0x4d7858),&why,1))g_reload.slide_live=1;
    if(g_b.log)g_b.log("  reload v3: guided magazine on the well axis %s (%s)\r\n",
        g_reload.slide_live?"installed":"unavailable, the magazine stays in the hand until seated",why);
}
static void reload_model(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16,actor=r[8],body,weapon,objs,root,hand;
    int pat,expected,k,permitted;double a[3],b[3],distance=0;
    const float *shifts;
    AcquireSRWLockExclusive(&g_reload_lock);
    if(!reload_fresh())goto done;
    __try {
        body=*(uint64_t *)(ULONG_PTR)(actor+0x68);
        pat=*(int *)(ULONG_PTR)(actor+0x80);expected=dg_reload_pattern(g_reload.weapon);
        if(expected<0)__leave;
        if(body!=g_reload.arm || pat!=expected) __leave;
        weapon=*(uint64_t *)(ULONG_PTR)(actor+0x60);
        if(!weapon || !*(uint64_t *)(ULONG_PTR)(actor+0x70) || !*(uint64_t *)(ULONG_PTR)(actor+0x78)) __leave;
        objs=*(uint64_t *)(ULONG_PTR)weapon;

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
        if(g_reload.active && (g_reload.empty || g_reload.ejected) && g_reload.reserve && !g_reload.anim) {
            g_reload.waiting=1;
            if(g_reload.empty)g_reload.neutral=1;
            permitted=g_reload.gesture.permit;
            dg_reload_gesture(&g_reload.gesture,g_reload.sample.source,g_reload.sample.sequence,
                g_reload.tick_ms,g_reload.valid,distance);
            memcpy(g_reload_probe.magwell,a,sizeof a);g_reload_probe.dist_m=distance;g_reload_probe.world_ms=GetTickCount64();
            if(!permitted && g_reload.gesture.permit && g_b.log && !g_reload.insert_mode)
                g_b.log("  reload: magazine contact weapon=%d distance_m=%.3f sample=%llu\r\n",g_reload.weapon,distance,g_reload.sample.sequence);
            if(g_reload.valid)r[13]=2; /* ECX: original model, left-hand case */
            if(g_reload.insert_mode)reload_insert_model(r,actor,root,hand,a,b,pat);
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){g_reload.fault=1;}
done:ReleaseSRWLockExclusive(&g_reload_lock);
}
/* While a rifle magazine is owed the duplicate is the hint: dispatch case 1 (ECX 0x10 before the native SAR 4)
 * draws it seated in the well with the native seated pose; the held magazine stays in the first object. */
static int reload_ghost_owed(void) {
    return g_reload.ghost_resolved && dg_reload_rifle(g_reload.weapon) &&
        (g_reload.empty || g_reload.ejected) && g_reload.reserve;
}
static void reload_second(void *rsp) {
    uint64_t *r=(uint64_t *)rsp-16;
    AcquireSRWLockShared(&g_reload_lock);
    if(reload_fresh() && g_reload.valid && g_reload.waiting && !g_reload.anim &&
       r[8]==g_reload.actor && reload_visual_ready()) {
        if(reload_ghost_owed()){r[13]=0x10;InterlockedIncrement(&g_reload.ghost_hits);}
        else r[13]=0; /* no duplicate */
    }
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
    dg_detour_remove(&g_reload_tail_hook);g_reload.slide_live=0;
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
/* Empty click: the M9's native empty-gun sound at the weapon root. */
static void reload_v2_click(void) {
    typedef void (__fastcall *RELOAD_AUDIO)(int,const float *,int);
    uint64_t weapon,objs,root;float pos[4]={0,0,0,1};
    reload_v2_events(DG_RELOAD_EV_CLICK);
    if(!g_reload.v2_resolved || !reload_visual_ready())return;
    weapon=*(uint64_t *)(ULONG_PTR)(g_reload.actor+0x60);if(!weapon)return;
    objs=*(uint64_t *)(ULONG_PTR)weapon;if(!objs)return;
    root=*(uint64_t *)(ULONG_PTR)(objs+0x40);if(!root)root=objs;
    memcpy(pos,(void *)(ULONG_PTR)(root+0x30),sizeof pos);
    ((RELOAD_AUDIO)(ULONG_PTR)(g_reload.base+0x5512f0))(4,pos,1);
}

static void reload_v2_tick_fill(const DG_M9_SAMPLE *s,int physical) {
    int neutral=g_reload.neutral,engaged=physical || (s->valid && s->trigger) ||
        g_b.fire.state==DG_FIRE_DRAW || g_b.fire.state==DG_FIRE_HOLD;
    if(!g_reload.v2 || !g_reload.waiting || !g_reload.active || g_reload.anim || !g_reload.valid ||
       !reload_visual_ready() || !reload_v2_contact())return;
    if(reload_direct_fill())g_reload.neutral=neutral || engaged;
}

static void reload_v2_tick(const DG_M9_SAMPLE *s,int weapon) {
    const DG_BRIDGE_CONTROLS_FRAME *f=&g_controls_frame; /* eject input, same frame as s */
    volatile int *mag=reload_magazine();int m=*mag,verdict;
    /* Native refilled behind us (load, continue): the magazine is back in. */
    if(g_reload.ejected && m>g_reload.eject_mag) {
        g_reload.ejected=0;
        if(g_b.log)g_b.log("  reload v2: native refill %d -> %d, magazine back in the gun\r\n",g_reload.eject_mag,m);
    }
    if(g_reload.ejected)g_reload.eject_mag=m;
    if(dg_reload_grip_edge(&g_reload.grip_armed,s->valid && s->allowed && f->reload_eject.valid,f->reload_eject.squeeze)) {
        verdict=dg_reload_eject_verdict(g_reload.v2,weapon,g_b.fps.state==DG_FPS_ACTIVE,s->allowed,
            f->reload_eject.ear,left_support_engaged(),
            !g_reload.active || !reload_visual_ready() || !g_reload.valid || g_reload.anim ||
            g_reload.waiting || g_reload.ejected || g_reload.neutral,m);
        if(verdict==DG_EJECT_OK) {
            *mag=dg_reload_eject_mag(m);
            g_reload.ejected=1;g_reload.eject_mag=*mag;g_reload.ejects++;g_reload.empty=*mag==0;
            dg_reload_gesture_reset(&g_reload.gesture);
            reload_v2_events(DG_RELOAD_EV_EJECT);
            if(g_b.log)g_b.log("  reload v2: eject weapon=%d magazine %d -> %d (eject %d)\r\n",weapon,m,*mag,g_reload.ejects);
        } else if(g_b.log)
            g_b.log("  reload v2: eject refused weapon=%d (%s)\r\n",weapon,dg_eject_names[verdict]);
    }
    if(s->valid && s->allowed && s->trigger && !g_reload.trigger_was && g_reload.empty &&
       g_reload.waiting && g_reload.reserve)reload_v2_click();
    g_reload.trigger_was=s->valid && s->allowed && s->trigger;
}
static int reload_tick(int safe,int writable,uint64_t player,int weapon,int physical,DG_FIRE_IN *fire) {
    int block=0,was_waiting=g_reload.waiting;uint64_t arm=0,inventory;DG_M9_SAMPLE *s=&g_controls_frame.m9;
    AcquireSRWLockExclusive(&g_reload_lock);
    g_reload.safe=0;
    if(!g_reload.enabled || !g_reload.live || g_reload.fault)goto done;
    __try {
        if(g_b.a.gm_player_arm_body)arm=*(uint64_t *)(ULONG_PTR)g_b.a.gm_player_arm_body;
        int supported=dg_reload_pattern(weapon)>=0 && (!dg_reload_rifle(weapon) ||
            (g_reload.rifles_resolved && g_reload.v2 && g_reload.v2_resolved));
        if(!safe || !writable || !player || !arm || !supported || !g_controls_active ||
           g_b.fps.state!=DG_FPS_ACTIVE || InterlockedCompareExchange(&g_b.s_late_unsafe,0,0)) {
            g_reload.driving=g_reload.cancel_ticks=g_reload.active=g_reload.waiting=g_reload.anim=g_reload.neutral=0;g_reload.actor=0;
            dg_reload_gesture_reset(&g_reload.gesture);

            g_reload.grip_armed=g_reload.trigger_was=0;dg_reload_insert_reset(&g_reload.insert);
            if(g_reload.ejected && (!player || !supported || g_b.fps.state!=DG_FPS_ACTIVE)) {
                g_reload.ejected=0;
                if(g_b.log)g_b.log("  reload v2: ejected state dropped (weapon=%d fps=%d)\r\n",weapon,(int)g_b.fps.state);
            }
            __leave;
        }
        if(player!=g_reload.player || arm!=g_reload.arm || weapon!=g_reload.weapon) {
            g_reload.actor=0;g_reload.driving=g_reload.cancel_ticks=g_reload.active=g_reload.waiting=g_reload.anim=g_reload.neutral=0;
            dg_reload_gesture_reset(&g_reload.gesture);
            dg_reload_insert_reset(&g_reload.insert);
            g_reload.grip_armed=g_reload.trigger_was=0;
            g_reload.ejected=0;
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
        if(g_reload.v2)reload_v2_tick(s,weapon);
        if(g_reload.insert_mode) {
            int owed=(g_reload.empty || g_reload.ejected) && g_reload.reserve;
            if(owed && !g_reload.ins_owed) {
                g_reload.ins_lines=0;g_reload.ghost_logged=0;InterlockedExchange(&g_reload.ghost_hits,0);
                reload_probe_new_magazine();
            }
            g_reload.ins_owed=owed;
        }
        if(g_reload.active && (g_reload.empty || g_reload.ejected) && g_reload.reserve && !g_reload.anim) {
            g_reload.waiting=1;
            if(g_reload.empty)g_reload.neutral=1;
        }
        if(!(g_reload.empty || g_reload.ejected) || !g_reload.reserve){g_reload.waiting=0;dg_reload_gesture_reset(&g_reload.gesture);
            dg_reload_insert_reset(&g_reload.insert);}
        reload_v2_tick_fill(s,physical);
        if(!g_reload.anim && !g_reload.waiting && g_reload.neutral && s->allowed) {
            if(g_reload.cancel_ticks>=DG_FIRE_ABORT_TICKS && !s->trigger && !physical && g_reload.valid)g_reload.neutral=0;
            else g_reload.cancel_ticks++;
        }

        block=((g_reload.waiting && g_reload.empty) || g_reload.anim || g_reload.neutral) && s->allowed;
        if(!block)g_reload.driving=0;
        if(!was_waiting && g_reload.waiting && g_b.log)
            g_b.log("  reload: waiting for left magazine weapon=%d\r\n",weapon);
        if(g_reload.waiting && !g_reload.ghost_logged && InterlockedCompareExchange(&g_reload.ghost_hits,0,0)>0) {
            g_reload.ghost_logged=1;
            if(g_b.log)g_b.log("  reload ghost: magazine shown seated in the well weapon=%d\r\n",weapon);
        }
        if(g_reload.waiting)reload_probe_log(weapon);
        reload_rifle_hold_meter(weapon);                 /* rifle_empty_bob: HOLD flips per second */
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
    /* Automatic rifles fire on press. A reload must not create a pad press.
       rifle_empty_bob: nor leave the button the rifle shot dry with latched in the Bluepoint layer; end the pull
       like a let-go trigger (dg_rifle_hold.h), so the attack routine stops raising and dropping the rifle. */
    if(dg_reload_rifle(g_reload.weapon)) {
        if(mask && index>=0 && index<12 && g_b.a.player_pad && !g_reload.anim) {

            LONG ws=RD32(g_b.a.player_pad+DG_PAD_WEAPON_STATE_OFFSET);
            DG_RIFLE_HOLD_SEAM *seam=&g_rifle_hold.seam;
            seam->called=1;seam->weapon_state=(int)ws;seam->button_state=(int)RD32(g_b.a.player_pad+DG_PAD_BUTTON_STATE_OFFSET);
            seam->status_in=(unsigned)RD32(g_b.a.player_pad+DG_PAD_STATUS_OFFSET);
            seam->press_in=(unsigned)RD32(g_b.a.player_pad+DG_PAD_PRESS_OFFSET);
            seam->release_in=(unsigned)RD32(g_b.a.player_pad+DG_PAD_RELEASE_OFFSET);
            if(dg_rifle_hold_release((volatile long *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_STATUS_OFFSET),
                                     (volatile long *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_PRESS_OFFSET),
                                     (volatile long *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_RELEASE_OFFSET),
                                     (volatile unsigned char *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_PRESSURE_OFFSET+index),mask,
                                     ws==DG_RIFLE_HOLD_WS_DRAW)) {
                g_rifle_hold.latched++;g_rifle_hold.total_released++;seam->released=1;
            }
            else seam->released=0;
            seam->status_out=(unsigned)RD32(g_b.a.player_pad+DG_PAD_STATUS_OFFSET);
            seam->release_out=(unsigned)RD32(g_b.a.player_pad+DG_PAD_RELEASE_OFFSET);
        }
        ReleaseSRWLockExclusive(&g_reload_lock);return;
    }
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
