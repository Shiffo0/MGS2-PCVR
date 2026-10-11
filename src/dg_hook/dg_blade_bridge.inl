#include "dg_blade_model.inl"



typedef void (__fastcall *DG_BLADE_ATTACK_FN)(void *);
static DG_DETOUR g_blade_detour;
static DG_DETOUR g_blade_pose_detour, g_blade_homing_detour;
static DG_BLADE_ATTACK_FN g_blade_pose_original, g_blade_homing_original;
static DG_BLADE_ATTACK_FN g_blade_pose_regular;
static unsigned long g_blade_pose_passes, g_blade_homing_skips;
static unsigned long g_blade_pose_calls, g_blade_homing_calls;
static SRWLOCK g_blade_heading_lock=SRWLOCK_INIT;
static struct {uint64_t player,ms;int valid,yaw;} g_blade_heading;
static DG_DETOUR g_blade_target_detour,g_blade_damage_detour;
typedef void (__fastcall *DG_BLADE_TARGET_FN)(void *,const float (*)[4],unsigned);
typedef int (__fastcall *DG_BLADE_DAMAGE_FN)(void *,void *,void *);
static DG_BLADE_TARGET_FN g_blade_target_original;
static DG_BLADE_DAMAGE_FN g_blade_damage_original;
static unsigned long g_blade_target_updates,g_blade_damage_updates;
static int g_blade_target_yaw=-1;
static uint64_t g_blade_target_ms;
/* Called on the native actor thread. Never use a previous owner or a stale
 * controls lease to replace an animation update. */
static int blade_tracked_owner(uint64_t expected, int is_arm);
static int blade_heading_now(int *yaw);
static DG_BLADE_ATTACK_FN g_blade_original;
static SRWLOCK g_blade_lock=SRWLOCK_INIT;
static volatile LONG g_blade_enabled;
static struct {
    uint64_t base,player,stamp,tick,consumed;
    int resolved,admitted,selected;
    DG_BLADE_STATE state;
    DG_AIM_SELECTION selection;
    DG_BLADE_OUTPUT output;
    /* The attack waiting for the native check (one at a time, newest wins). */
    struct { int attack; unsigned char x,y; uint64_t made_ms; } queued;
    DG_BLADE_BRACE brace;   /* left grip with the left hand on the blade */
    int guard_on;           /* the block wants the native guard this tick */
    int guarding;           /* the lock-on bit was written this tick */











} g_blade;
#define DG_BLADE_QUEUE_MS 500
#define DG_BLADE_DIAG_LINES 800
static void blade_guard_log(int active) {








    (void)active;

}























































































int dg_bridge_blade_enabled(void) {return InterlockedCompareExchange(&g_blade_enabled,0,0)!=0;}
static int blade_claim(void) {return dg_bridge_blade_enabled() && g_blade.selected;}
/* The block is on: the left hand belongs on the blade (dg_hook's left IK). */
int dg_bridge_blade_braced(void) {return dg_bridge_blade_enabled() && g_blade.guard_on;}
/* The left grip that braces the blade is the block, not a capture. */
static int blade_brace_claim(void) {return dg_bridge_blade_enabled() && g_blade.guard_on;}
static int blade_tracked_owner(uint64_t expected, int is_arm) {
    uint64_t arm=0,player=0; LONG weapon=-1;
    uint64_t now=GetTickCount64();
    return dg_bridge_blade_enabled() && g_blade.selected && g_b.armed && !g_b.owner &&
        !g_b.script_menu_only && !g_b.s_late_unsafe && g_b.fps.state==DG_FPS_ACTIVE &&
        /* Actor callbacks occur after controls_end; active marks only the
         * borrowed input-sample window. Permission uses the published lease. */
        g_controls_allowed && now>=g_controls_lease &&
        now-g_controls_lease<=100 && now>=g_blade.stamp && now-g_blade.stamp<=100 &&
        g_b.hand_command_requested && g_b.hand_command_valid &&
        resolve_player(&arm,&player,&weapon)==DG_RESOLVE_OK && weapon==13 &&
        player==g_blade.player && expected==(is_arm?arm:player);
}
/* The native special update copies body joints 0..20 OVER the subjective
 * joints after the ordinary update. That discards both controller IK chains.
 * Keep the ordinary update for a freshly controlled VR blade only. */
static void __fastcall blade_pose(void *body) {
    DG_BLADE_ATTACK_FN original=g_blade_pose_detour.tramp ?
        (DG_BLADE_ATTACK_FN)g_blade_pose_detour.tramp:g_blade_pose_original;
    g_blade_pose_calls++;
    if(g_blade_pose_regular && blade_tracked_owner((uint64_t)(uintptr_t)body,1)) {
        g_blade_pose_passes++;g_blade_pose_regular(body);return;
    }
    if(original)original(body);
}
/* Native auto-facing may turn the body toward an enemy when a slash, thrust
 * or guard begins. The VR hand owns the direction; do not add enemy yaw. */
static void __fastcall blade_homing(void *player) {
    DG_BLADE_ATTACK_FN original=g_blade_homing_detour.tramp ?
        (DG_BLADE_ATTACK_FN)g_blade_homing_detour.tramp:g_blade_homing_original;
    g_blade_homing_calls++;
    if(blade_tracked_owner((uint64_t)(uintptr_t)player,0)) {
        g_blade_homing_skips++;return;
    }
    if(original)original(player);
}
void dg_bridge_blade_camera(const float world[4][4]) {
    double x,z,n,h;int yaw;
    if(!TryAcquireSRWLockExclusive(&g_blade_heading_lock))return;
    g_blade_heading.valid=0;
    if(world && blade_tracked_owner(g_blade.player,0)) {
        x=world[2][0];z=world[2][2];n=x*x+z*z;
        if(isfinite(n)&&n>1e-6&&n<1.01) {
            h=atan2(x,z);yaw=((int)floor(h*2048.0/3.141592653589793+0.5))&4095;
            g_blade_heading.yaw=yaw;g_blade_heading.player=g_blade.player;
            g_blade_heading.ms=GetTickCount64();g_blade_heading.valid=1;
        }
    }
    ReleaseSRWLockExclusive(&g_blade_heading_lock);
}
static int blade_heading_now(int *yaw) {
    uint64_t now=GetTickCount64();int ok=0;
    if(!yaw||!TryAcquireSRWLockShared(&g_blade_heading_lock))return 0;
    if(g_blade_heading.valid&&g_blade_heading.player==g_blade.player&&
       now>=g_blade_heading.ms&&now-g_blade_heading.ms<=100&&
       blade_tracked_owner(g_blade.player,0)) {
        *yaw=g_blade_heading.yaw;ok=1;
    }
    ReleaseSRWLockShared(&g_blade_heading_lock);return ok;
}
static void __fastcall blade_target(void *target,const float world[4][4],unsigned flags) {
    DG_BLADE_TARGET_FN original=g_blade_target_detour.tramp?
        (DG_BLADE_TARGET_FN)g_blade_target_detour.tramp:g_blade_target_original;
    __declspec(align(16)) float adjusted[4][4];int yaw;
    if(!original)return;
    if((uint64_t)(uintptr_t)target==g_blade.base+0x16e8d60&&g_blade.guard_on&&
       blade_heading_now(&yaw)&&world&&
       (uint64_t)(uintptr_t)world==*(uint64_t *)(uintptr_t)(g_blade.player+0x290)) {
        double a=yaw*(3.141592653589793/2048.0),s=sin(a),c=cos(a);
        memcpy(adjusted,world,sizeof adjusted);
        /* Retain body translation/height; only the horizontal defense basis
         * follows the view. No player rotation or movement state is changed. */
        adjusted[0][0]=(float)c;adjusted[0][1]=0;adjusted[0][2]=(float)-s;
        adjusted[1][0]=0;adjusted[1][1]=1;adjusted[1][2]=0;
        adjusted[2][0]=(float)s;adjusted[2][1]=0;adjusted[2][2]=(float)c;
        g_blade_target_updates++;g_blade_target_yaw=yaw;g_blade_target_ms=GetTickCount64();
        original(target,adjusted,flags);return;
    }
    original(target,world,flags);
}
static int __fastcall blade_damage(void *player,void *off,void *def) {
    DG_BLADE_DAMAGE_FN original=g_blade_damage_detour.tramp?
        (DG_BLADE_DAMAGE_FN)g_blade_damage_detour.tramp:g_blade_damage_original;
    int yaw,result=0;short saved;
    if(!original)return 0;
    if((uint64_t)(uintptr_t)player!=g_blade.player||!g_blade.guard_on||!blade_heading_now(&yaw))
        return original(player,off,def);
    /* Native callback owns its damage/counter rules. Substitute only its
     * heading input for this synchronous call, then restore body rotation. */
    saved=*(short *)((unsigned char *)player+0x82);
    __try {
        *(short *)((unsigned char *)player+0x82)=(short)yaw;
        g_blade_damage_updates++;result=original(player,off,def);
    } __finally {*(short *)((unsigned char *)player+0x82)=saved;}
    return result;
}
static void blade_resolve(const LiveImage *im) {
    blade_model_resolve(im);
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

       !m9_match(im,0x518ca5,(const unsigned char *)"\x85\x15\x6d\x72\x46\x00",6) ||

       !m9_match(im,0x519c34,(const unsigned char *)"\xff\x05\xd6\xf0\x1c\x01",6) ||

       !m9_match(im,0x518cd3,(const unsigned char *)"\x48\x8d\x15\xb6\xf0\xff\xff",7) ||
       !m9_match(im,0x57bc66,(const unsigned char *)"\x48\x89\x91\x60\x0c\x00\x00",7) ||
       !m9_match(im,0x51a1e0,(const unsigned char *)"\x0f\xb6\x42\x14",4) ||
       !m9_match(im,0x51a1f9,(const unsigned char *)"\x0f\xb6\x42\x15",4))return;
    /* Retail call graph: SetPos chooses special/regular object update;
     * Guard calls the auto-facing helper, whose only control write is turn.y. */
    if(!m9_match(im,0x57db2f,(const unsigned char *)"\xf6\x00\x80\x74\x07\xe8\x97\xea\xff\xff\xeb\x05\xe8\x70\x00\x10\x00",17) ||
       !m9_match(im,0x57c5d0,(const unsigned char *)"\x48\x8b\xc4\x48\x89\x58\x10\x48\x89\x70\x18",11) ||
       !m9_match(im,0x67dbb0,(const unsigned char *)"\x40\x57\x48\x83\xec\x20\x48\x8b\xf9",9) ||
       !m9_match(im,0x517e18,(const unsigned char *)"\xe8\xf3\x26\x00\x00",5) ||
       !m9_match(im,0x51a510,(const unsigned char *)"\x4c\x8b\xdc",3))return;
    if(!m9_match(im,0x51a63e,(const unsigned char *)"\x66\x89\x86\x8a\x00\x00\x00",7))return;


    if(!m9_match(im,0x518103,(const unsigned char *)"\x48\x8b\x97\x90\x02\x00\x00\xe8\x61\x1f\xb6\xff",12)||
       !m9_match(im,0x5180fc,(const unsigned char *)"\x48\x8d\x0d\x5d\x0c\x1d\x01",7)||
       !m9_match(im,0x7a070,(const unsigned char *)"\x0f\x28\x02\x33\xc0\x0f\x11\x41\x20",9)||
       !m9_match(im,0x5198f0,(const unsigned char *)"\x48\x89\x5c\x24\x08\x48\x89\x6c\x24\x10",10)||
       !m9_match(im,0x519951,(const unsigned char *)"\x41\x0f\xbf\x97\x82\x00\x00\x00",8)||
       !m9_match(im,0x519963,(const unsigned char *)"\x3d\x00\x03\x00\x00",5))return;
    g_blade.base=im->base;g_blade.resolved=1;
}
/* Called while the ordinary controls owner holds the game-tick lease. */
static void blade_tick(int safe) {
    uint64_t player=0,arm=0;LONG weapon=0;DG_BLADE_SAMPLE in=g_controls_frame.blade;
    DG_AIM_SELECTION selection={0};
    int admitted=0;
    AcquireSRWLockExclusive(&g_blade_lock);
    /* A queued attack the native check never took in time is gone. */
    if(g_blade.queued.attack && GetTickCount64()-g_blade.queued.made_ms>DG_BLADE_QUEUE_MS) {








        g_blade.queued.attack=0;
    }
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
    {   /* Block: left grip with the left hand on the blade. Judged on the raw
           sample - the native guard animation may cost the right hand's
           selection, and that must not drop the block. */
        int was=g_blade.guard_on;
        dg_blade_brace_step(&g_blade.brace,&in);










        g_blade.guard_on=g_blade.selected && g_blade.brace.active;
        if(g_blade.guard_on!=was)blade_guard_log(g_blade.guard_on);
    }
    g_blade.player=player;in.allowed=in.allowed && admitted;
    {   /* Strokes as before: none with the left grip down (grab, choke, block). */
        DG_BLADE_SAMPLE stroke_in=in;
        if(stroke_in.left_grip)stroke_in.allowed=0;
        g_blade.output=dg_blade_step(&g_blade.state,&stroke_in);
    }
    /* Blocking: walking stays native, attacking does not happen. */
    if(g_blade.guard_on && (g_blade.output.attack || g_blade.queued.attack)) {



        g_blade.output.attack=0;g_blade.queued.attack=0;
    }
    if(g_blade.output.attack) {



        g_blade.queued.attack=g_blade.output.attack;g_blade.queued.x=g_blade.output.x;
        g_blade.queued.y=g_blade.output.y;g_blade.queued.made_ms=GetTickCount64();
    }
    /* Put the blade away, lose the hand or the player: nothing stays queued. */
    if(!g_blade.selected || !player){g_blade.queued.attack=0;g_blade.guard_on=0;memset(&g_blade.brace,0,sizeof g_blade.brace);}
    g_blade.guarding=0;
    g_blade.admitted=in.allowed && dg_blade_sample_valid(&in);
    g_blade.stamp=GetTickCount64();g_blade.tick=(uint64_t)(DWORD)g_b.c_ticks;



    ReleaseSRWLockExclusive(&g_blade_lock);
}




static void blade_guard_pad(int safe) {
    unsigned mask;
    AcquireSRWLockExclusive(&g_blade_lock);
    if(g_blade.guard_on && !g_blade.queued.attack && safe && g_blade.selected &&
       dg_bridge_blade_enabled() && g_b.a.player_pad && g_blade.base) {
        mask=(unsigned)RD32(g_blade.base+0x97ff18);
        if(mask && mask<=0xffffu && !(mask&0xf000u)) {
            *(volatile LONG *)(ULONG_PTR)(g_b.a.player_pad+DG_PAD_STATUS_OFFSET)|=(LONG)mask;
            g_blade.guarding=1;



        }
    }



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
                    if(g_blade.queued.attack && g_blade.consumed!=g_blade.tick &&
                       GetTickCount64()-g_blade.queued.made_ms<=DG_BLADE_QUEUE_MS) {
                        uint64_t waited=GetTickCount64()-g_blade.queued.made_ms;
                        if(g_blade.queued.attack==DG_BLADE_STAB)
                            *(uint32_t *)(shadow+DG_PAD_PRESS_OFFSET)|=0x400;
                        else {
                            shadow[DG_PAD_RIGHT_DX_OFFSET]=g_blade.queued.x;
                            shadow[DG_PAD_RIGHT_DX_OFFSET+1]=g_blade.queued.y;
                            *(uint16_t *)(shadow+DG_PAD_ANALOG_OFFSET)|=DG_MOVE_ANALOG_R_USE;
                        }
                        g_blade.consumed=g_blade.tick;



                        if(g_b.log)g_b.log("  blade: native intent=%d axes=%u,%u tick=%llu waited %llu ms\r\n",
                            g_blade.queued.attack,g_blade.queued.x,g_blade.queued.y,g_blade.tick,
                            (unsigned long long)waited);
                        g_blade.queued.attack=0;
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
        if(dg_detour_install_ex(&g_blade_pose_detour,
            (void *)(uintptr_t)(g_blade.base+0x57c5d0),(void *)blade_pose,
            (void *)(uintptr_t)(g_blade.base+0x57c5d0),(void *)(uintptr_t)(g_blade.base+0x57cebe),&why,2) &&
           dg_detour_install_ex(&g_blade_homing_detour,
            (void *)(uintptr_t)(g_blade.base+0x51a510),(void *)blade_homing,
            (void *)(uintptr_t)(g_blade.base+0x51a510),(void *)(uintptr_t)(g_blade.base+0x51a66b),&why,2) &&
           dg_detour_install_ex(&g_blade_target_detour,
            (void *)(uintptr_t)(g_blade.base+0x7a070),(void *)blade_target,
            (void *)(uintptr_t)(g_blade.base+0x7a070),(void *)(uintptr_t)(g_blade.base+0x7a0af),&why,2) &&
           dg_detour_install_ex(&g_blade_damage_detour,
            (void *)(uintptr_t)(g_blade.base+0x5198f0),(void *)blade_damage,
            (void *)(uintptr_t)(g_blade.base+0x5198f0),(void *)(uintptr_t)(g_blade.base+0x519a27),&why,2)) {
            g_blade_pose_original=(DG_BLADE_ATTACK_FN)g_blade_pose_detour.tramp;
            g_blade_homing_original=(DG_BLADE_ATTACK_FN)g_blade_homing_detour.tramp;
            g_blade_target_original=(DG_BLADE_TARGET_FN)g_blade_target_detour.tramp;
            g_blade_damage_original=(DG_BLADE_DAMAGE_FN)g_blade_damage_detour.tramp;
            g_blade_pose_regular=(DG_BLADE_ATTACK_FN)(uintptr_t)(g_blade.base+0x67dbb0);
            InterlockedExchange(&g_blade_enabled,1);
            {
                int full=blade_model_install();
                if(g_b.log)g_b.log("  blade model: %s (mask %u)\r\n",
                    full?"full subject geometry installed":"native geometry retained or incomplete restore",g_blade_model_mask);
            }
        } else {
            dg_detour_remove(&g_blade_damage_detour);
            dg_detour_remove(&g_blade_target_detour);
            dg_detour_remove(&g_blade_homing_detour);
            dg_detour_remove(&g_blade_pose_detour);
            dg_detour_remove(&g_blade_detour);
        }
    }
    if(g_b.log)g_b.log("  blade: %s (%s)\r\n",dg_bridge_blade_enabled()?"gesture adapter installed":"unavailable",why);
}
static void blade_stop(void) {
    if(!blade_model_restore() && g_b.log)g_b.log("  blade model: restore refused, mask %u\r\n",g_blade_model_mask);
    InterlockedExchange(&g_blade_enabled,0);
    AcquireSRWLockExclusive(&g_blade_lock);
    g_blade.admitted=0;g_blade.selected=0;g_blade.queued.attack=0;g_blade.guarding=0;
    g_blade.guard_on=0;memset(&g_blade.brace,0,sizeof g_blade.brace);dg_blade_reset(&g_blade.state);
    ReleaseSRWLockExclusive(&g_blade_lock);
    dg_detour_remove(&g_blade_damage_detour);
    dg_detour_remove(&g_blade_target_detour);
    dg_detour_remove(&g_blade_homing_detour);
    dg_detour_remove(&g_blade_pose_detour);
    dg_detour_remove(&g_blade_detour);
    /* Retain original: an in-flight wrapper may still need the leaked tramp. */
}
