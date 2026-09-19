/* Version-locked BladeCheckAttack wrapper. Only its synchronous call sees the
 * shadow pad; native camera/movement/global PlayerPad never see swing axes.
 * Native animation, damage windows, hit tests and cooldowns remain in charge. */
typedef void (__fastcall *DG_BLADE_ATTACK_FN)(void *);
static DG_DETOUR g_blade_detour;
static DG_BLADE_ATTACK_FN g_blade_original;
static SRWLOCK g_blade_lock=SRWLOCK_INIT;
static volatile LONG g_blade_enabled;
static struct {
    uint64_t base,player,stamp,tick,consumed;
    int resolved,admitted,selected;
    DG_BLADE_STATE state;
    DG_AIM_SELECTION selection;
    DG_BLADE_OUTPUT output;
} g_blade;
int dg_bridge_blade_enabled(void) {return InterlockedCompareExchange(&g_blade_enabled,0,0)!=0;}
static int blade_claim(void) {return dg_bridge_blade_enabled() && g_blade.selected;}
static void blade_resolve(const LiveImage *im) {
    memset(&g_blade,0,sizeof g_blade);
    /* Constructor registration, actor slots, byte consumers and stab bit.
     * The table entry is initially null: the plugin fills it on level load. */
    if(!m9_match(im,0x53b5ea,(const unsigned char *)"\x4c\x8d\x8b\x94\x0b\0\0\xff\xd0",9) ||
       !m9_match(im,0x517ca2,(const unsigned char *)"\xc7\x87\x94\x0b\0\0\x06\x40\0\0",10) ||
       !m9_match(im,0x519d7c,(const unsigned char *)"\x48\x8d\x05\x3d\x9d\xfa\xff",7) ||
       !m9_match(im,0x4c3a7f,(const unsigned char *)"\x48\x89\xb3\x70\x03\0\0\x48\x89\xbb\x78\x03\0\0\x48\x89\xab\x80\x03\0\0",21) ||
       !m9_match(im,0x518c10,(const unsigned char *)"\x48\x89\x5c\x24\x10\x57\x48\x83\xec\x20",10) ||
       !m9_match(im,0x518c4f,(const unsigned char *)"\x48\x8b\x87\0\x0d\0\0",7) ||
       !m9_match(im,0x518c7d,(const unsigned char *)"\x0f\xba\xe1\x0a",4) ||
       !m9_match(im,0x51a1e0,(const unsigned char *)"\x0f\xb6\x42\x14",4) ||
       !m9_match(im,0x51a1f9,(const unsigned char *)"\x0f\xb6\x42\x15",4))return;
    g_blade.base=im->base;g_blade.resolved=1;
}
/* Called while the ordinary controls owner holds the game-tick lease. */
static void blade_tick(int safe) {
    uint64_t player=0,arm=0;LONG weapon=0;DG_BLADE_SAMPLE in=g_controls_frame.blade;
    DG_AIM_SELECTION selection={0};
    int admitted=0;
    AcquireSRWLockExclusive(&g_blade_lock);
    g_blade.output.attack=0;g_blade.admitted=0;g_blade.selected=0;
    if(dg_bridge_blade_enabled() && safe && g_b.armed && g_controls_active &&
       g_b.fps.state==DG_FPS_ACTIVE && !g_b.s_late_unsafe && !g_b.script_menu_only &&
       g_b.a.player_pad && RD32(g_b.a.player_pad-4) &&
       resolve_player(&arm,&player,&weapon)==DG_RESOLVE_OK && weapon==13) {
        g_blade.selected=1;
        admitted=dg_aim_capture_hand_selection((uintptr_t)g_blade.base,arm,&selection) && selection.weapon_id==13;
    }
    if(player!=g_blade.player || memcmp(&selection,&g_blade.selection,sizeof selection))dg_blade_reset(&g_blade.state);
    g_blade.selection=selection;
    g_blade.player=player;in.allowed=in.allowed && admitted;
    g_blade.output=dg_blade_step(&g_blade.state,&in);
    g_blade.admitted=in.allowed && dg_blade_sample_valid(&in);
    g_blade.stamp=GetTickCount64();g_blade.tick=(uint64_t)(DWORD)g_b.c_ticks;
    ReleaseSRWLockExclusive(&g_blade_lock);
}
static void __fastcall blade_attack(void *work) {
    unsigned char shadow[40];uint64_t original_pad=0,player=(uint64_t)(uintptr_t)work;
    int swapped=0;DG_BLADE_ATTACK_FN original=g_blade_detour.tramp ?
        (DG_BLADE_ATTACK_FN)g_blade_detour.tramp : g_blade_original;
    if(!original)return;
    /* Context revocation drains this consumer too, not just the pad producer. */
    AcquireSRWLockShared(&g_controls_lock);
    AcquireSRWLockExclusive(&g_blade_lock);
    __try {
        if(dg_bridge_blade_enabled() && g_b.armed && !g_b.s_late_unsafe &&
           g_b.fps.state==DG_FPS_ACTIVE && g_controls_allowed &&
           GetTickCount64()>=g_controls_lease && GetTickCount64()-g_controls_lease<=100 &&
           player==g_blade.player &&
           g_blade.admitted && GetTickCount64()>=g_blade.stamp &&
           GetTickCount64()-g_blade.stamp<=100 &&
           (uint64_t)(DWORD)g_b.c_ticks==g_blade.tick &&
           RD32(player+0xb90)==13 && g_b.a.player_pad && RD32(g_b.a.player_pad-4)) {
            original_pad=*(uint64_t *)(uintptr_t)(player+0xd00);
            if(original_pad==g_b.a.player_pad) {
                memcpy(shadow,(void *)(uintptr_t)original_pad,sizeof shadow);
                /* Never convert a real native button action into a gesture. */
                if(!RD32(original_pad+DG_PAD_PRESS_OFFSET) &&
                   (RD32(original_pad+DG_PAD_STATUS_OFFSET) &
                    ~(0xf000u | (g_b.a.pad_subject ? RD32(g_b.a.pad_subject):0)))==0) {
                    if(g_blade.output.attack && g_blade.consumed!=g_blade.tick) {
                        if(g_blade.output.attack==DG_BLADE_STAB)
                            *(uint32_t *)(shadow+DG_PAD_PRESS_OFFSET)|=0x400;
                        else {
                            shadow[DG_PAD_RIGHT_DX_OFFSET]=g_blade.output.x;
                            shadow[DG_PAD_RIGHT_DX_OFFSET+1]=g_blade.output.y;
                            *(uint16_t *)(shadow+DG_PAD_ANALOG_OFFSET)|=DG_MOVE_ANALOG_R_USE;
                        }
                        g_blade.consumed=g_blade.tick;
                        if(g_b.log)g_b.log("  blade: native intent=%d axes=%u,%u tick=%llu\r\n",
                            g_blade.output.attack,g_blade.output.x,g_blade.output.y,g_blade.tick);
                    }
                    *(uint64_t *)(uintptr_t)(player+0xd00)=(uint64_t)(uintptr_t)shadow;
                    swapped=1;
                }
            }
        }
        original(work);
    } __finally {
        if(swapped)*(uint64_t *)(uintptr_t)(player+0xd00)=original_pad;
        ReleaseSRWLockExclusive(&g_blade_lock);
        ReleaseSRWLockShared(&g_controls_lock);
    }
}
static void blade_install(int requested) {
    const char *why="retail witnesses unavailable";
    if(!requested)return;
    if(g_blade.resolved && dg_detour_install_ex(&g_blade_detour,
       (void *)(uintptr_t)(g_blade.base+0x518c10),(void *)blade_attack,
       (void *)(uintptr_t)(g_blade.base+0x518c10),
       (void *)(uintptr_t)(g_blade.base+0x518e15),&why,2)) {
        /* Mode 2 publishes the trampoline before installing the entry jump. */
        g_blade_original=(DG_BLADE_ATTACK_FN)g_blade_detour.tramp;
        InterlockedExchange(&g_blade_enabled,1);
    }
    if(g_b.log)g_b.log("  blade: %s (%s)\r\n",dg_bridge_blade_enabled()?"gesture adapter installed":"unavailable",why);
}
static void blade_stop(void) {
    InterlockedExchange(&g_blade_enabled,0);
    AcquireSRWLockExclusive(&g_blade_lock);
    g_blade.admitted=0;g_blade.selected=0;dg_blade_reset(&g_blade.state);
    ReleaseSRWLockExclusive(&g_blade_lock);
    dg_detour_remove(&g_blade_detour);
    /* Retain original: an in-flight wrapper may still need the leaked tramp. */
}
