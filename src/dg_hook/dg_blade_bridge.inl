#include "dg_blade_model.inl"

typedef void (__fastcall *DG_BLADE_ATTACK_FN)(void *);
static DG_DETOUR g_blade_detour,g_blade_guard_visibility;
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

#define DG_BLADE_RVA_STICKPRESS 0x16e8d0c
#define DG_BLADE_RVA_STICKDIR   0x16e8f8c
#define DG_BLADE_RVA_SLASH_MODE 0x516c10
#define DG_BLADE_RVA_THRUST_MODE 0x517c00
#define DG_BLADE_TRIGGER        0xb94
#define DG_BLADE_MODE           0xc60
#define DG_BLADE_FIRE_BITS      0x4003u
#define DG_BLADE_FIRE_MASK      0x400fu     /* FIRE bit and the trigger kind */
#define DG_BLADE_POSTURE_BITS   0x70ull
static int g_blade_stick_ok, g_blade_live_ok, g_blade_probe_ok;
/* blade_follow_v2: the two ordinary standing modes and the grounded byte that
   the body follow gates on (witnessed in blade_resolve, own group). */
#define DG_BLADE_RVA_STAND_STILL 0x51a790
#define DG_BLADE_RVA_STAND_RUN   0x53d620
#define DG_BLADE_GROUNDED        0x11a
static int g_blade_follow_modes_ok, g_blade_guard_turn_field;
static DG_DETOUR g_blade_act_detour;
static int g_blade_act_ok;
static volatile LONG g_blade_live_mode;

static int g_blade_haptic;
static double g_blade_hit_speed;
#define DG_BLADE_HIT_SPEED_DEFAULT 3.0
/* Marker defaults when the key is absent: haptic on, threshold 3.0 m/s. */
static void blade_options(const DG_BRIDGE_CONFIG *cfg) {
    g_blade_haptic=cfg ? !cfg->blade_haptic_off : 1;
    g_blade_hit_speed=cfg && cfg->blade_hit_speed_set ? cfg->blade_hit_speed : DG_BLADE_HIT_SPEED_DEFAULT;
    if(!(g_blade_hit_speed>=0 && g_blade_hit_speed<=20))g_blade_hit_speed=DG_BLADE_HIT_SPEED_DEFAULT;
}

typedef void (__fastcall *DG_BLADE_HIT_FN)(void *,void *,void *);
static DG_DETOUR g_blade_hit_detour;
static volatile LONG64 g_blade_contact_ms;
static volatile LONG g_blade_haptic_events; /* DG_BLADE_HAPTIC_* bits for the XR producer */
static uint64_t g_blade_haptic_run;
#include "dg_blade_haptic.h"

enum { DG_BLADE_REFUSE_NONE=0, DG_BLADE_REFUSE_GATE, DG_BLADE_REFUSE_PAD,
       DG_BLADE_REFUSE_HOLD, DG_BLADE_REFUSE_COUNT };
static const char *blade_refuse_name(int r) {
    static const char *n[DG_BLADE_REFUSE_COUNT]={"native attack check did not run",
        "gate closed (lease/admitted/tick/weapon/pad)","a real button in the pad",
        "stick press hold did not clear"};
    return r>=0 && r<DG_BLADE_REFUSE_COUNT ? n[r] : "?";
}
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
    int refuse;             /* DG_BLADE_REFUSE_* for the queued intent */
    int posture_bad;        /* SQUAT|GROUND|CAUTION this tick */
    DG_BLADE_LIVE live;
    uint64_t live_tick;     /* the tick the gate was judged on */
    uint64_t live_wrote;    /* the tick the FIRE bits were last raised */
    int live_kind;
    int guard_hit_was;
    volatile uint64_t delivered_ms; /* last delivery, for the probe */
    struct {
        uint64_t holds, live_ticks, live_strokes, live_suppressed, live_skipped;
        uint64_t native_slash, native_thrust, native_refused, hits_live, hits_native;
        uint64_t resets[DG_BLADE_RESET_COUNT], duplicates, framed_ticks, stroke_ticks;
        uint64_t not_held[10];
        int last_not_held;      /* the latest not-held reason (index into not_held) */
        uint64_t quiet[DG_BLADE_R_COUNT], capped, stroke_lines, pair_earlier, pair_later, live_thrust_ticks, hits_followed, intents_dropped, haptic_hits, haptic_deflects, rehit_blocks, threshold_blocks;
        int live_was;
        uint64_t selected, admitted, strokes[DG_BLADE_R_COUNT], windups, fired, delivered, dropped, replaced;
        uint64_t guards, guard_ticks;
        uint64_t native_guard_ticks, native_guards, native_guard_ours, deflects;
        uint64_t presses, press_far, blocked_attacks;
        int native_guard, guard_hit_seen;
        uint64_t report_ms;
        int lines;
    } diag;
} g_blade;
#define DG_BLADE_QUEUE_MS 500
#define DG_BLADE_DIAG_LINES 800

#define DG_BLADE_PROBE_PER_S 20
#define DG_BLADE_PROBE_LINES 6000
static struct {uint64_t second;int in_second;int total;} g_blade_probe_budget;
static SRWLOCK g_blade_probe_budget_lock=SRWLOCK_INIT;
static int blade_probe_line(void) {
    uint64_t s=GetTickCount64()/1000;int ok=0;
    if(!g_b.log)return 0;
    AcquireSRWLockExclusive(&g_blade_probe_budget_lock);
    if(s!=g_blade_probe_budget.second){g_blade_probe_budget.second=s;g_blade_probe_budget.in_second=0;}
    if(g_blade_probe_budget.total<DG_BLADE_PROBE_LINES && g_blade_probe_budget.in_second<DG_BLADE_PROBE_PER_S) {
        g_blade_probe_budget.total++;g_blade_probe_budget.in_second++;ok=1;
    }
    ReleaseSRWLockExclusive(&g_blade_probe_budget_lock);
    return ok;
}
static void blade_guard_log(int active) {
    if(g_b.log && g_blade.diag.lines<DG_BLADE_DIAG_LINES) {
        g_blade.diag.lines++;
        g_b.log("  blade guard: %s (left grip on the blade)  hand %.2f m off the blade, %.2f m along it\r\n",
            active?"ENTER":"EXIT",g_blade.brace.dist,g_blade.brace.along);
    }
    if(active)g_blade.diag.guards++;
}
static const char *blade_end_name(int e) {
    return e==DG_BLADE_END_STOP?"stop":e==DG_BLADE_END_REVERSE?"reverse":e==DG_BLADE_END_TIMEOUT?"timeout":"?";
}
static void blade_diag_stroke(const char *what,const DG_BLADE_STROKE *t) {
    g_b.log("  blade stroke: %-9s %-12s peak %.2f m/s (hand %.2f)  right %+.2f up %+.2f fwd %+.2f m  %llu ms  end %s  pointing %+.2f  axes %u,%u\r\n",
        what,dg_blade_reason_name(t->reason),t->peak,t->hand_peak,t->x,t->y,t->z,
        (unsigned long long)(t->end_ms-t->start_ms),blade_end_name(t->end),t->pointing,t->ax,t->ay);
}

static void blade_diag_native_guard(void) {
    uint64_t mode=0;int now_guard;
    if(!g_blade.player || !g_blade.base ||
       !interact_read(NULL,g_blade.player+0xc60,&mode,8))return;
    now_guard=mode==g_blade.base+0x517d90;
    {

        LONG hit=0;
        if(interact_read(NULL,g_blade.base+0x16e8d10,&hit,4)) {
            if(hit && !g_blade.diag.guard_hit_seen) {
                int yaw=-1;g_blade.diag.deflects++;
                blade_heading_now(&yaw);
                if(g_b.log&&g_blade.diag.lines++<DG_BLADE_DIAG_LINES)
                    g_b.log("  blade deflect heading: view %d target %d target_age_ms %llu body %d\r\n",
                        yaw,g_blade_target_yaw,(unsigned long long)(GetTickCount64()-g_blade_target_ms),
                        (int)*(short *)(uintptr_t)(g_blade.player+0x82));
            }
            g_blade.diag.guard_hit_seen=hit!=0;
        }
    }
    if(now_guard){g_blade.diag.native_guard_ticks++;if(g_blade.guarding)g_blade.diag.native_guard_ours++;}
    if(now_guard!=g_blade.diag.native_guard) {
        g_blade.diag.native_guard=now_guard;
        if(now_guard)g_blade.diag.native_guards++;
        if(g_b.log && g_blade.diag.lines<DG_BLADE_DIAG_LINES) {
            g_blade.diag.lines++;
            g_b.log("  blade native guard: %s (our guard %s, lock-on bit %s)\r\n",
                now_guard?"ENTER":"EXIT",g_blade.guard_on?"on":"off",g_blade.guarding?"written":"not written");
        }
    }
}
/* Under g_blade_lock, after each recognizer step. */
static void blade_diag_step(void) {
    const DG_BLADE_OUTPUT *o=&g_blade.output;uint64_t now=GetTickCount64();
    if(g_blade.selected)g_blade.diag.selected++;
    /* Recogniser health: why it reset, duplicates, and whether it has a frame. */
    if(g_blade.selected) {
        if(o->reset>0 && o->reset<DG_BLADE_RESET_COUNT)g_blade.diag.resets[o->reset]++;
        if(o->duplicate)g_blade.diag.duplicates++;
        if(g_blade.state.framed)g_blade.diag.framed_ticks++;
        if(g_blade.state.active)g_blade.diag.stroke_ticks++;
    }

    if(g_blade.admitted)g_blade.diag.admitted++;
    if(o->report) {
        g_blade.diag.strokes[o->stroke.reason]++;
        if(o->attack)g_blade.diag.fired++;
        /* Hand jitter also makes strokes; only near-swings are worth a line. */

        if(!(o->attack || o->stroke.peak>=1.0))g_blade.diag.quiet[o->stroke.reason]++;
        else if(!g_b.log || g_blade.diag.lines>=DG_BLADE_DIAG_LINES)g_blade.diag.capped++;
        else {
            g_blade.diag.lines++;g_blade.diag.stroke_lines++;
            blade_diag_stroke(o->attack==DG_BLADE_STAB?"STAB":o->attack?"SLASH":"no-attack",&o->stroke);
        }
    }
    if(o->dropped) {
        if(o->windup.reason==DG_BLADE_R_WINDUP)g_blade.diag.windups++;
        else g_blade.diag.strokes[o->windup.reason]++;
        if(!(o->windup.peak>=1.0))g_blade.diag.quiet[o->windup.reason]++;
        else if(!g_b.log || g_blade.diag.lines>=DG_BLADE_DIAG_LINES)g_blade.diag.capped++;
        else {
            g_blade.diag.lines++;g_blade.diag.stroke_lines++;
            blade_diag_stroke(o->windup.reason==DG_BLADE_R_WINDUP?"wind-up":"follow-up",&o->windup);
        }
    }
    if(o->pair) {
        const DG_BLADE_STROKE *won=&o->stroke,*lost=&o->windup;
        if(o->pair==DG_BLADE_PAIR_EARLIER_FASTER)g_blade.diag.pair_earlier++;else g_blade.diag.pair_later++;
        if(blade_probe_line())g_b.log("  blade pair: fired %s stroke (peak %.2f, axes %u,%u) over %s (peak %.2f) - rule %s\r\n",
            o->pair==DG_BLADE_PAIR_EARLIER_FASTER?"earlier":"later",won->peak,won->ax,won->ay,
            o->pair==DG_BLADE_PAIR_EARLIER_FASTER?"the later":"the earlier",lost->peak,
            o->pair==DG_BLADE_PAIR_EARLIER_FASTER?"earlier stroke was faster":"later stroke faster or equal");
    }
    if(g_blade.diag.selected && now-g_blade.diag.report_ms>=10000 && g_b.log) {
        int r;char text[256];int len=0;
        g_blade.diag.report_ms=now;
        if(g_b.log)g_b.log("  blade pose: tracked updates %lu native auto-turns suppressed %lu calls pose %lu homing %lu\r\n",
            g_blade_pose_passes,g_blade_homing_skips,g_blade_pose_calls,g_blade_homing_calls);
        if(g_b.log)g_b.log("  blade gameplay heading: target updates %lu melee checks %lu\r\n",
            g_blade_target_updates,g_blade_damage_updates);
        for(r=1;r<DG_BLADE_R_COUNT;r++)
            len+=sprintf_s(text+len,sizeof text-len," %s %llu",dg_blade_reason_name(r),
                (unsigned long long)g_blade.diag.strokes[r]);
        g_b.log("  blade summary: ticks held %llu admitted %llu | brace presses %llu (away from the blade %llu) guards %llu (ticks %llu) native guards %llu (ticks %llu, with our bit %llu) deflects %llu attacks held back %llu | fired %llu delivered %llu expired %llu replaced %llu wind-ups %llu | no-attack:%s\r\n",
            (unsigned long long)g_blade.diag.selected,(unsigned long long)g_blade.diag.admitted,
            (unsigned long long)g_blade.diag.presses,(unsigned long long)g_blade.diag.press_far,
            (unsigned long long)g_blade.diag.guards,(unsigned long long)g_blade.diag.guard_ticks,
            (unsigned long long)g_blade.diag.native_guards,(unsigned long long)g_blade.diag.native_guard_ticks,
            (unsigned long long)g_blade.diag.native_guard_ours,(unsigned long long)g_blade.diag.deflects,
            (unsigned long long)g_blade.diag.blocked_attacks,
            (unsigned long long)g_blade.diag.fired,(unsigned long long)g_blade.diag.delivered,
            (unsigned long long)g_blade.diag.dropped,(unsigned long long)g_blade.diag.replaced,
            (unsigned long long)g_blade.diag.windups,text);
        {   char rs[320];int rl=0;
            for(r=1;r<DG_BLADE_RESET_COUNT;r++)
                rl+=sprintf_s(rs+rl,sizeof rs-rl," %s %llu",dg_blade_reset_name(r),
                    (unsigned long long)g_blade.diag.resets[r]);
            static const char *nh[10]={"","unsafe","not-armed","no-controls","not-first-person","late-unsafe","script-menu","no-pad","no-player","not-blade"};
            int hl=0;char hs[320];hs[0]=0;
            for(r=1;r<10;r++)hl+=sprintf_s(hs+hl,sizeof hs-hl," %s %llu",nh[r],(unsigned long long)g_blade.diag.not_held[r]);
            g_b.log("  blade not held:%s\r\n",hs);
            {   char qs[256];int ql=0;qs[0]=0;
                for(r=1;r<DG_BLADE_R_COUNT;r++)ql+=sprintf_s(qs+ql,sizeof qs-ql," %s %llu",dg_blade_reason_name(r),(unsigned long long)g_blade.diag.quiet[r]);
                g_b.log("  blade stroke lines: written %llu capped %llu quiet (peak < 1.0 m/s, no line):%s | pairs earlier-faster %llu later-faster %llu | intents dropped %llu\r\n",
                    (unsigned long long)g_blade.diag.stroke_lines,(unsigned long long)g_blade.diag.capped,qs,
                    (unsigned long long)g_blade.diag.pair_earlier,(unsigned long long)g_blade.diag.pair_later,
                    (unsigned long long)g_blade.diag.intents_dropped);
            }
            g_b.log("  blade recogniser: framed %d (ticks framed %llu, in a stroke %llu) duplicates %llu | resets:%s\r\n",
                g_blade.state.framed,(unsigned long long)g_blade.diag.framed_ticks,
                (unsigned long long)g_blade.diag.stroke_ticks,(unsigned long long)g_blade.diag.duplicates,rs);
        }
        g_b.log("  blade live summary: mode %ld witnesses stick %d live %d probe %d | live ticks %llu strokes %llu not-written %llu suppressed-native %llu | holds %llu | native slash %llu thrust %llu refused %llu | hits live %llu native %llu followed up %llu | live thrust ticks %llu\r\n",
            InterlockedCompareExchange(&g_blade_live_mode,0,0),g_blade_stick_ok,g_blade_live_ok,g_blade_probe_ok,
            (unsigned long long)g_blade.diag.live_ticks,(unsigned long long)g_blade.diag.live_strokes,
            (unsigned long long)g_blade.diag.live_skipped,(unsigned long long)g_blade.diag.live_suppressed,
            (unsigned long long)g_blade.diag.holds,
            (unsigned long long)g_blade.diag.native_slash,(unsigned long long)g_blade.diag.native_thrust,
            (unsigned long long)g_blade.diag.native_refused,
            (unsigned long long)g_blade.diag.hits_live,(unsigned long long)g_blade.diag.hits_native,
            (unsigned long long)g_blade.diag.hits_followed,(unsigned long long)g_blade.diag.live_thrust_ticks);
    }
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
    g_blade_stick_ok=g_blade_live_ok=g_blade_probe_ok=g_blade_act_ok=0;
    g_blade_follow_modes_ok=0;g_blade_guard_turn_field=0;
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

    g_blade_stick_ok=
        m9_match(im,0x518d67,(const unsigned char *)"\x8b\x05\x9f\xff\x1c\x01",6) &&
        m9_match(im,0x518d8b,(const unsigned char *)"\x8b\x15\xfb\x01\x1d\x01",6) &&
        m9_match(im,0x519a3c,(const unsigned char *)"\xc7\x05\xc6\xf2\x1c\x01\x01\0\0\0",10);

    g_blade_live_ok=
        m9_match(im,0x521dcd,(const unsigned char *)"\x44\x89\xa7\x94\x0b\0\0",7) &&
        m9_match(im,0x5174e5,(const unsigned char *)"\xc7\x86\x94\x0b\0\0\x03\x40\0\0",10) &&
        m9_match(im,0x4c2eb1,(const unsigned char *)"\x48\x8b\x87\x80\x03\0\0",7);

    g_blade_act_ok=
        m9_match(im,0x4c2ae0,(const unsigned char *)"\x4c\x8b\xdc\x55\x53\x57",6) &&
        m9_match(im,0x4c2eb1,(const unsigned char *)"\x48\x8b\x87\x80\x03\0\0",7);

    g_blade_probe_ok=
        m9_match(im,0x4c2ae0,(const unsigned char *)"\x4c\x8b\xdc\x55\x53\x57",6) &&
        m9_match(im,0x4c310e,(const unsigned char *)"\x48\x8d\xb7\xe0\x01\0\0",7) &&
        m9_match(im,0x4c312f,(const unsigned char *)"\xe8\x1c\x6f\xbb\xff",5) &&
        m9_match(im,0x7a050,(const unsigned char *)"\x0f\x28\x02\x0f\x11\x41\x60\x41\x0f\x28\x00\x0f\x11\x41\x70",15) &&
        m9_match(im,0x4c37dc,(const unsigned char *)"\x48\x8d\x15\xad\x01\0\0",7) &&
        m9_match(im,0x4c3990,(const unsigned char *)"\x41\xf6\x80\x8c\x03\0\0\x02\x74\x1b",10);

    g_blade_guard_turn_field=m9_match(im,0x518004,(const unsigned char *)
        "\x3d\x00\x01\x00\x00\x7d\x07\x66\x89\x9f\x8a\x00\x00\x00",14);
    g_blade_follow_modes_ok=
        m9_match(im,0x529914,(const unsigned char *)"\x48\x8d\x0d\x75\x0e\xff\xff\x48\x3b\xc1\x0f\x84",12) &&
        m9_match(im,0x529924,(const unsigned char *)"\x48\x8d\x0d\xf5\x3c\x01\x00\x48\x3b\xc1\x0f\x84",12) &&
        m9_match(im,0x72a50,(const unsigned char *)"\xc6\x83\xba\x00\x00\x00\x01",7);
}

static int blade_queue_output(void) {
    if(!g_blade.output.attack)return 0;
    if(InterlockedCompareExchange(&g_blade_live_mode,0,0)==2) {
        g_blade.diag.live_suppressed++;
        return 0;
    }
    if(g_blade.queued.attack)g_blade.diag.replaced++;
    g_blade.queued.attack=g_blade.output.attack;g_blade.queued.x=g_blade.output.x;
    g_blade.queued.y=g_blade.output.y;g_blade.queued.made_ms=GetTickCount64();
    g_blade.refuse=DG_BLADE_REFUSE_NONE;
    return 1;
}

static void blade_intent_dropped(const char *why) {
    static const char *nh[10]={"","unsafe","not-armed","no-controls","not-first-person","late-unsafe","script-menu","no-pad","no-player","not-blade"};
    int w=g_blade.diag.last_not_held;
    g_blade.diag.intents_dropped++;
    if(g_b.log && blade_probe_line())g_b.log("  blade: intent %d dropped (%s%s%s) after %llu ms\r\n",g_blade.queued.attack,why,
        w>0 && w<10 && strcmp(why,"block held")?": ":"",w>0 && w<10 && strcmp(why,"block held")?nh[w]:"",
        (unsigned long long)(GetTickCount64()-g_blade.queued.made_ms));
}

static void blade_haptic_note(unsigned ev) {
    if(!g_blade_haptic || !ev)return;
    InterlockedOr(&g_blade_haptic_events,(LONG)ev);
    if(ev&DG_BLADE_HAPTIC_DEFLECT)g_blade.diag.haptic_deflects++;
    if(ev&DG_BLADE_HAPTIC_HIT)g_blade.diag.haptic_hits++;
    if(blade_probe_line())g_b.log("  blade haptic: %s tick %llu\r\n",
        (ev&DG_BLADE_HAPTIC_DEFLECT)?"deflect (both hands)":"hit (right hand)",(unsigned long long)(DWORD)g_b.c_ticks);
}
unsigned dg_bridge_blade_events(void) {return (unsigned)InterlockedExchange(&g_blade_haptic_events,0);}

static void blade_guard_hit_step(void) {
    LONG hit=0;
    if(!g_blade.selected || !g_blade.base || !interact_read(NULL,g_blade.base+0x16e8d10,&hit,4)){g_blade.guard_hit_was=0;return;}
    if(hit && !g_blade.guard_hit_was)blade_haptic_note(DG_BLADE_HAPTIC_DEFLECT);
    g_blade.guard_hit_was=hit!=0;
}
/* Called while the ordinary controls owner holds the game-tick lease. */
static void blade_tick(int safe) {
    uint64_t player=0,arm=0;LONG weapon=0;DG_BLADE_SAMPLE in=g_controls_frame.blade;
    DG_AIM_SELECTION selection={0};
    int admitted=0,resolved=-1;
    AcquireSRWLockExclusive(&g_blade_lock);
    /* A queued attack the native check never took in time is gone. */
    if(g_blade.queued.attack && GetTickCount64()-g_blade.queued.made_ms>DG_BLADE_QUEUE_MS) {
        g_blade.diag.dropped++;
        if(g_b.log && g_blade.diag.lines<DG_BLADE_DIAG_LINES) {

            uint64_t mode=0;
            g_blade.diag.lines++;
            if(g_blade.player)interact_read(NULL,g_blade.player+DG_BLADE_MODE,&mode,8);
            g_b.log("  blade: intent %d EXPIRED after %llu ms (%s) mode exe+0x%llX\r\n",
                g_blade.queued.attack,(unsigned long long)(GetTickCount64()-g_blade.queued.made_ms),
                blade_refuse_name(g_blade.refuse),
                (unsigned long long)(mode && g_blade.base && mode>=g_blade.base ? mode-g_blade.base : mode));
        }
        g_blade.queued.attack=0;
    }
    g_blade.output.attack=0;g_blade.admitted=0;g_blade.selected=0;
    if(dg_bridge_blade_enabled() && safe && g_b.armed && g_controls_active &&
       g_b.fps.state==DG_FPS_ACTIVE && !g_b.s_late_unsafe && !g_b.script_menu_only &&
       g_b.a.player_pad && RD32(g_b.a.player_pad-4) &&
       (resolved=resolve_player(&arm,&player,&weapon))==DG_RESOLVE_OK && weapon==13) {
        g_blade.selected=1;
        admitted=dg_aim_capture_hand_selection((uintptr_t)g_blade.base,arm,&selection) && selection.weapon_id==13;
    }

    if(!g_blade.selected && dg_bridge_blade_enabled()) {
        int why=!safe?1:!g_b.armed?2:!g_controls_active?3:g_b.fps.state!=DG_FPS_ACTIVE?4:
            g_b.s_late_unsafe?5:g_b.script_menu_only?6:!(g_b.a.player_pad && RD32(g_b.a.player_pad-4))?7:
            resolved!=DG_RESOLVE_OK?8:9;
        g_blade.diag.not_held[why]++;g_blade.diag.last_not_held=why;
    }
    if(player!=g_blade.player || memcmp(&selection,&g_blade.selection,sizeof selection)) {
        if(g_blade.state.have)g_blade.diag.resets[DG_BLADE_RESET_SELECTION]++;
        dg_blade_reset(&g_blade.state);
    }
    g_blade.selection=selection;
    {   /* Block: left grip with the left hand on the blade. Judged on the raw
           sample - the native guard animation may cost the right hand's
           selection, and that must not drop the block. */
        int was=g_blade.guard_on;
        dg_blade_brace_step(&g_blade.brace,&in);
        if(g_blade.brace.pressed && g_blade.selected) {
            g_blade.diag.presses++;if(!g_blade.brace.active)g_blade.diag.press_far++;
            if(g_b.log && g_blade.diag.lines<DG_BLADE_DIAG_LINES) {
                g_blade.diag.lines++;
                g_b.log("  blade brace: left grip pressed %.2f m off the blade, %.2f m along it -> %s\r\n",
                    g_blade.brace.dist,g_blade.brace.along,g_blade.brace.active?"BLOCK":"no block (hand not on the blade)");
            }
        }
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
        g_blade.diag.blocked_attacks++;
        if(g_blade.queued.attack)blade_intent_dropped("block held");
        g_blade.output.attack=0;g_blade.queued.attack=0;
    }
    blade_queue_output();
    /* Put the blade away, lose the hand or the player: nothing stays queued. */
    if(!g_blade.selected || !player) {
        if(g_blade.queued.attack)blade_intent_dropped(!player?"player lost":"blade no longer held");
        g_blade.queued.attack=0;g_blade.guard_on=0;memset(&g_blade.brace,0,sizeof g_blade.brace);
    }
    g_blade.guarding=0;
    g_blade.admitted=in.allowed && dg_blade_sample_valid(&in);
    g_blade.stamp=GetTickCount64();g_blade.tick=(uint64_t)(DWORD)g_b.c_ticks;

    g_blade.posture_bad=0;
    if(g_b.a.gm_player_status)
        g_blade.posture_bad=(*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status & DG_BLADE_POSTURE_BITS)!=0;
    g_blade.live.hit_speed=g_blade_hit_speed;
    {
        uint64_t c=(uint64_t)InterlockedCompareExchange64(&g_blade_contact_ms,0,0);
        if(c && c!=g_blade.live.contact_ms)dg_blade_live_contact(&g_blade.live,c);
    }
    dg_blade_live_step(&g_blade.live,
        g_blade_live_ok && g_blade.selected && player ? (int)InterlockedCompareExchange(&g_blade_live_mode,0,0) : 0,
        &g_blade.state,g_blade.admitted,g_blade.guard_on,g_blade.posture_bad,g_blade.stamp);
    g_blade.live_tick=g_blade.tick;
    g_blade.live_kind=dg_blade_live_kind(&g_blade.state);
    blade_guard_hit_step();
    if(g_blade.live.on && !g_blade.diag.live_was) {
        g_blade.diag.live_strokes++;
        if(blade_probe_line())g_b.log("  blade live: ON tip %.2f m/s tick %llu\r\n",
            g_blade.state.tip_speed,(unsigned long long)g_blade.tick);
    } else if(!g_blade.live.on && g_blade.diag.live_was) {
        if(blade_probe_line())g_b.log("  blade live: OFF (%s) after %llu ms tick %llu\r\n",
            dg_blade_live_name(g_blade.live.why),(unsigned long long)(g_blade.stamp-g_blade.live.on_ms),
            (unsigned long long)g_blade.tick);
    }
    g_blade.diag.live_was=g_blade.live.on;
    blade_diag_step();
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
            g_blade.diag.guard_ticks++;
        }
    }
    if(g_blade.selected)blade_diag_native_guard();
    ReleaseSRWLockExclusive(&g_blade_lock);
}
static void __fastcall blade_attack(void *work) {
    unsigned char shadow[40];uint64_t original_pad=0,player=(uint64_t)(uintptr_t)work;
    int swapped=0,gate=0,delivered=0;uint64_t mode_before=0;
    LONG stick_press=-1,stick_dir=-1;ULONGLONG posture=0;
    DG_BLADE_ATTACK_FN original=g_blade_detour.tramp ?
        (DG_BLADE_ATTACK_FN)g_blade_detour.tramp : g_blade_original;
    if(!original)return;
    /* Context revocation drains this consumer too, not just the pad producer. */
    AcquireSRWLockShared(&g_controls_lock);
    AcquireSRWLockExclusive(&g_blade_lock);
    __try {
        gate=dg_bridge_blade_enabled() && g_b.armed && !g_b.s_late_unsafe &&
           g_b.fps.state==DG_FPS_ACTIVE && g_controls_allowed &&
           GetTickCount64()>=g_controls_lease && GetTickCount64()-g_controls_lease<=100 &&
           player==g_blade.player &&
           g_blade.admitted && GetTickCount64()>=g_blade.stamp &&
           GetTickCount64()-g_blade.stamp<=100 &&
           (uint64_t)(DWORD)g_b.c_ticks==g_blade.tick &&
           RD32(player+0xb90)==13 && g_b.a.player_pad && RD32(g_b.a.player_pad-4);
        if(!gate && g_blade.queued.attack)g_blade.refuse=DG_BLADE_REFUSE_GATE;
        if(gate) {
            mode_before=*(volatile uint64_t *)(uintptr_t)(player+DG_BLADE_MODE);
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
                        if(g_blade_stick_ok) {
                            stick_press=RD32(g_blade.base+DG_BLADE_RVA_STICKPRESS);
                            stick_dir=RD32(g_blade.base+DG_BLADE_RVA_STICKDIR);
                        }
                        if(g_b.a.gm_player_status)
                            posture=*(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_status & DG_BLADE_POSTURE_BITS;
                        if(g_blade.queued.attack!=DG_BLADE_STAB && g_blade_stick_ok && stick_press!=0) {

                            *(uint16_t *)(shadow+DG_PAD_ANALOG_OFFSET)&=(uint16_t)~DG_MOVE_ANALOG_R_USE;
                            g_blade.refuse=DG_BLADE_REFUSE_HOLD;
                            g_blade.diag.holds++;
                            if(blade_probe_line())g_b.log("  blade: delivery held one tick (stick press %ld stick direction %ld) intent=%d tick=%llu\r\n",
                                stick_press,stick_dir,g_blade.queued.attack,g_blade.tick);
                        } else {
                            if(g_blade.queued.attack==DG_BLADE_STAB)
                                *(uint32_t *)(shadow+DG_PAD_PRESS_OFFSET)|=0x400;
                            else {
                                shadow[DG_PAD_RIGHT_DX_OFFSET]=g_blade.queued.x;
                                shadow[DG_PAD_RIGHT_DX_OFFSET+1]=g_blade.queued.y;
                                *(uint16_t *)(shadow+DG_PAD_ANALOG_OFFSET)|=DG_MOVE_ANALOG_R_USE;
                            }
                            g_blade.consumed=g_blade.tick;
                            g_blade.diag.delivered++;
                            if(g_b.log)g_b.log("  blade: native intent=%d axes=%u,%u tick=%llu waited %llu ms\r\n",
                                g_blade.queued.attack,g_blade.queued.x,g_blade.queued.y,g_blade.tick,
                                (unsigned long long)waited);
                            delivered=g_blade.queued.attack;
                            g_blade.queued.attack=0;g_blade.refuse=DG_BLADE_REFUSE_NONE;
                            g_blade.delivered_ms=GetTickCount64();
                        }
                    }
                    *(uint64_t *)(uintptr_t)(player+0xd00)=(uint64_t)(uintptr_t)shadow;
                    swapped=1;
                } else if(g_blade.queued.attack)g_blade.refuse=DG_BLADE_REFUSE_PAD;
            } else if(g_blade.queued.attack)g_blade.refuse=DG_BLADE_REFUSE_GATE;
        }
        original(work);
        if(gate) {
            uint64_t mode_after=*(volatile uint64_t *)(uintptr_t)(player+DG_BLADE_MODE);
            if(delivered) {
                /* What the game made of it, read straight back from SetMode. */
                const char *what=mode_after==g_blade.base+DG_BLADE_RVA_SLASH_MODE?"blade slash":
                    mode_after==g_blade.base+DG_BLADE_RVA_THRUST_MODE?"blade thrust":"native refused";
                if(what[0]=='n')g_blade.diag.native_refused++;
                else if(mode_after==g_blade.base+DG_BLADE_RVA_SLASH_MODE)g_blade.diag.native_slash++;
                else g_blade.diag.native_thrust++;
                if(blade_probe_line())g_b.log("  blade: native result intent=%d -> %s (stick press %ld stick direction %ld squat/ground/caution 0x%llx mode %+lld) tick=%llu\r\n",
                    delivered,what,stick_press,stick_dir,posture,
                    (long long)(mode_after-g_blade.base),g_blade.tick);
            }
            (void)mode_before;
        }
    } __finally {
        if(swapped)*(uint64_t *)(uintptr_t)(player+0xd00)=original_pad;
        ReleaseSRWLockExclusive(&g_blade_lock);
        ReleaseSRWLockShared(&g_controls_lock);
    }
}

#define DG_BLADE_THRUST_BITS 0x4006u
static int blade_is_main_actor(uint64_t actor,uint64_t player) {
    return actor && player && *(volatile uint64_t *)(uintptr_t)(actor+0x380)==player+DG_BLADE_TRIGGER;
}
static int blade_live_open(uint64_t player) {
    uint64_t now=GetTickCount64();
    return g_blade_live_ok && InterlockedCompareExchange(&g_blade_live_mode,0,0)!=0 &&
        dg_bridge_blade_enabled() && g_b.armed && !g_b.s_late_unsafe &&
        g_blade.live.on && player==g_blade.player &&
        g_blade.live_tick==(uint64_t)(DWORD)g_b.c_ticks &&
        now>=g_blade.stamp && now-g_blade.stamp<=100;
}

static SRWLOCK g_blade_run_lock=SRWLOCK_INIT;
static struct {
    int in_run,ticks,live_ticks,hits;uint64_t start_ms,since_delivery,body;int body_kind,owner;
    float from0[4],to0[4],from[4],to[4];float ik0[3],ik[3];
} g_blade_run;
static const char *blade_body_name(int k){return k==1?"arm (subjective)":k==2?"player body":"other";}
static double blade_root_ik_mm(const float from[4],const float to[4],const float ik[3]) {
    /* The actor puts `from` 500 units behind the hand bone along the blade:
       root = from + 500 * unit(to - from). */
    double d[3],n=0,r;int k;
    for(k=0;k<3;k++){d[k]=to[k]-from[k];n+=d[k]*d[k];}
    n=sqrt(n);if(!(n>1) || !_finite(n))return -1;
    for(r=0,k=0;k<3;k++){double root=from[k]+500.0*d[k]/n-ik[k];r+=root*root;}
    return sqrt(r);
}
static void blade_ik_now(float ik[3]) {
    int k;for(k=0;k<3;k++){LONG v=InterlockedCompareExchange(&g_b.s_arm_ik_target[k],0,0);memcpy(&ik[k],&v,4);}
}
static void blade_run_end(void) {
    if(!g_blade_run.in_run)return;
    g_blade_run.in_run=0;
    if(blade_probe_line())g_b.log("  blade fire: %s ticks %d (live %d) first %lld ms after delivery | body %s | arm on controller %s | hits %d | first seg from (%.0f %.0f %.0f) to (%.0f %.0f %.0f) root-to-controller %.0f mm | last seg from (%.0f %.0f %.0f) to (%.0f %.0f %.0f) root-to-controller %.0f mm\r\n",
        g_blade_run.live_ticks==g_blade_run.ticks?"live":g_blade_run.live_ticks?"mixed":"native",
        g_blade_run.ticks,g_blade_run.live_ticks,(long long)g_blade_run.since_delivery,
        blade_body_name(g_blade_run.body_kind),g_blade_run.owner?"yes":"no",g_blade_run.hits,
        g_blade_run.from0[0],g_blade_run.from0[1],g_blade_run.from0[2],g_blade_run.to0[0],g_blade_run.to0[1],g_blade_run.to0[2],
        blade_root_ik_mm(g_blade_run.from0,g_blade_run.to0,g_blade_run.ik0),
        g_blade_run.from[0],g_blade_run.from[1],g_blade_run.from[2],g_blade_run.to[0],g_blade_run.to[1],g_blade_run.to[2],
        blade_root_ik_mm(g_blade_run.from,g_blade_run.to,g_blade_run.ik));
    devstage_enemy_targets("blade run",g_blade_run.from0,g_blade_run.to0,g_blade_run.from,g_blade_run.to);
}

typedef struct { uint32_t cls,name; uint64_t wt,damaged; short pv[3]; int power; } DG_BLADE_TGT;
static struct { uint64_t def,tick; DG_BLADE_TGT at; } g_blade_hitw;
static int blade_target_snap(uint64_t def,DG_BLADE_TGT *s) {
    uint64_t power=0;
    memset(s,0,sizeof *s);
    if(!def || !interact_read(NULL,def+0xa8,&s->damaged,8) || !interact_read(NULL,def+0x08,&s->cls,4))return 0;
    interact_read(NULL,def+0xa0,&s->wt,8);interact_read(NULL,def+0xe8,&s->name,4);
    if(interact_read(NULL,def+0xb8,&power,8) && power && interact_read(NULL,power+0x12,s->pv,6))s->power=1;
    return 1;
}
static void blade_hit_witness_step(void) {
    DG_BLADE_TGT now;int r;uint64_t dt;
    if(!g_blade_hitw.def || (dt=(uint64_t)(DWORD)g_b.c_ticks-g_blade_hitw.tick)<2)return;
    r=blade_target_snap(g_blade_hitw.def,&now);
    g_blade.diag.hits_followed++;
    if(blade_probe_line()) {
        const DG_BLADE_TGT *a=&g_blade_hitw.at;
        if(!r)g_b.log("  blade hit after %llu ticks: def %p gone (unreadable)\r\n",(unsigned long long)dt,(void *)(uintptr_t)g_blade_hitw.def);
        else g_b.log("  blade hit after %llu ticks: def %p class 0x%08X -> 0x%08X damaged 0x%llx -> 0x%llx def wt 0x%llx -> 0x%llx vital %d -> %d damage %d -> %d faint %d -> %d%s\r\n",
            (unsigned long long)dt,(void *)(uintptr_t)g_blade_hitw.def,a->cls,now.cls,
            (unsigned long long)a->damaged,(unsigned long long)now.damaged,(unsigned long long)a->wt,(unsigned long long)now.wt,
            a->pv[0],now.pv[0],a->pv[1],now.pv[1],a->pv[2],now.pv[2],now.power?"":" (no power)");
    }
    memset(&g_blade_hitw,0,sizeof g_blade_hitw);
}static void blade_probe_after(uint64_t a,uint64_t player,int fire,int live) {
    if(!TryAcquireSRWLockExclusive(&g_blade_run_lock))return;
    if(fire) {
        float *from=(float *)(uintptr_t)(a+0x1e0+0x60),*to=(float *)(uintptr_t)(a+0x1e0+0x70);
        if(!g_blade_run.in_run) {
            uint64_t wb=*(volatile uint64_t *)(uintptr_t)(player+0xba8),arm=0,now=GetTickCount64();
            if(g_b.a.gm_player_arm_body)arm=*(volatile uint64_t *)(uintptr_t)g_b.a.gm_player_arm_body;
            memset(&g_blade_run,0,sizeof g_blade_run);g_blade_run.in_run=1;g_blade_run.start_ms=now;
            g_blade_run.since_delivery=g_blade.delivered_ms && now-g_blade.delivered_ms<3000 ?
                now-g_blade.delivered_ms : (uint64_t)-1;
            g_blade_run.body=wb;g_blade_run.body_kind=wb && wb==arm?1:wb==player+0x290?2:0;
            g_blade_run.owner=blade_tracked_owner(player,0);
            memcpy(g_blade_run.from0,from,16);memcpy(g_blade_run.to0,to,16);blade_ik_now(g_blade_run.ik0);
        }
        g_blade_run.ticks++;if(live)g_blade_run.live_ticks++;
        memcpy(g_blade_run.from,from,16);memcpy(g_blade_run.to,to,16);blade_ik_now(g_blade_run.ik);
    } else blade_run_end();
    ReleaseSRWLockExclusive(&g_blade_run_lock);
}

static void __fastcall blade_hit_probe(void *off,void *def,void *work) {
    DG_BLADE_HIT_FN original=(DG_BLADE_HIT_FN)g_blade_hit_detour.tramp;
    uint64_t player=g_blade.player;
    if(original)original(off,def,work);
    if(blade_is_main_actor((uint64_t)(uintptr_t)work,player)) {
        int live=g_blade.live_wrote==(uint64_t)(DWORD)g_b.c_ticks;
        if(live) {
            InterlockedExchange64(&g_blade_contact_ms,(LONG64)GetTickCount64());
            if(g_blade.live.on_ms!=g_blade_haptic_run){g_blade_haptic_run=g_blade.live.on_ms;blade_haptic_note(DG_BLADE_HAPTIC_HIT);}
        }

        uint64_t d=(uint64_t)(uintptr_t)def;DG_BLADE_TGT s;int r=blade_target_snap(d,&s);
        if(live)g_blade.diag.hits_live++;else g_blade.diag.hits_native++;
        if(TryAcquireSRWLockExclusive(&g_blade_run_lock)){g_blade_run.hits++;ReleaseSRWLockExclusive(&g_blade_run_lock);}

        if(blade_probe_line())g_b.log("  blade hit: source %s def %p class 0x%08X name %s %u (def wt 0x%llx) damaged 0x%llx vital %d damage %d faint %d tick %llu\r\n",
            live?"live":"native window",def,s.cls,(s.cls&0x800000u)?"hit-se":"id",s.name,(unsigned long long)s.wt,
            (unsigned long long)s.damaged,s.power?s.pv[0]:-1,s.power?s.pv[1]:-1,s.power?s.pv[2]:-1,(unsigned long long)(DWORD)g_b.c_ticks);
        if(r && !g_blade_hitw.def){g_blade_hitw.def=d;g_blade_hitw.tick=(uint64_t)(DWORD)g_b.c_ticks;g_blade_hitw.at=s;}
    }
}
static void __fastcall blade_act(void *actor) {
    DG_BLADE_ATTACK_FN original=(DG_BLADE_ATTACK_FN)g_blade_act_detour.tramp;
    uint64_t a=(uint64_t)(uintptr_t)actor,player=g_blade.player;
    volatile uint32_t *trigger=NULL;uint32_t saved=0,written=0;int ours=0;
    int fire=0,live=0;
    if(blade_is_main_actor(a,player)) {
        ours=1;trigger=(volatile uint32_t *)(uintptr_t)(player+DG_BLADE_TRIGGER);saved=*trigger;
        if(blade_live_open(player)) {
            if(!(saved & DG_BLADE_FIRE_MASK)) {
                written=saved|(g_blade.live_kind==DG_BLADE_STAB?DG_BLADE_THRUST_BITS:DG_BLADE_FIRE_BITS);
                *trigger=written;g_blade.live_wrote=(uint64_t)(DWORD)g_b.c_ticks;
                g_blade.diag.live_ticks++;if(g_blade.live_kind==DG_BLADE_STAB)g_blade.diag.live_thrust_ticks++;
            }
            else g_blade.diag.live_skipped++;
        }
        fire=(*trigger&0x4000u)!=0;live=written!=0;
    }
    if(original)original(actor);
    /* No residue: put back exactly what the player code left, unless the
       trigger no longer holds our value (then it is someone else's). */
    if(written && *trigger==written)*trigger=saved;
    if(ours){blade_probe_after(a,player,fire,live);blade_hit_witness_step();}
}
static int blade_act_install(void) {
    const char *why="act witnesses unavailable";
    if(g_blade_act_detour.installed)return 1;
    if(g_blade_act_ok && dg_detour_install_ex(&g_blade_act_detour,(void *)(uintptr_t)(g_blade.base+0x4c2ae0),(void *)blade_act,
           (void *)(uintptr_t)(g_blade.base+0x4c2ae0),(void *)(uintptr_t)(g_blade.base+0x4c3465),&why,2))return 1;
    dg_detour_remove(&g_blade_act_detour);
    if(g_b.log)g_b.log("  blade act: refused (%s)\r\n",why);
    return 0;
}

static int blade_hit_install(const char **why) {
    if(g_blade_hit_detour.installed)return 1;
    if(g_blade_probe_ok && dg_detour_install_ex(&g_blade_hit_detour,(void *)(uintptr_t)(g_blade.base+0x4c3990),(void *)blade_hit_probe,
           (void *)(uintptr_t)(g_blade.base+0x4c3990),(void *)(uintptr_t)(g_blade.base+0x4c39b6),why,2))return 1;
    dg_detour_remove(&g_blade_hit_detour);
    return 0;
}
static void blade_probe_install(void) {
    const char *why="probe witnesses unavailable";
    if(g_blade_probe_ok && blade_act_install() && blade_hit_install(&why))why="installed";
    if(g_b.log)g_b.log("  blade probe: %s\r\n",why);
}

static void __fastcall blade_guard_visibility(uint64_t *sp) {
    uint64_t mode=0;
    if (g_blade.guard_on && blade_tracked_owner(g_blade.player,0) &&
        interact_read(NULL,g_blade.player+DG_BLADE_MODE,&mode,sizeof mode) &&
        mode==g_blade.base+0x517d90) sp[-2]&=~0x100ULL;
}
static void blade_guard_visibility_install(void) {
    /* movabs rax,mask; test [status],rax; je past the EVM hide */
    static const unsigned char site[]={0x48,0xb8,0,1,0,0,0,0,0,0x80,0x48,0x85,0x05,0xeb,0x3b,0x06,1,0x74,0x12};
    unsigned char *base=(unsigned char *)(uintptr_t)g_blade.base;
    const char *why="unknown manager layout";
    int ok=!memcmp(base+0x4ee4a4,site,sizeof site) &&
           !memcmp(base+0x4ee4c2,"\x81\x49\x58\x00\x03\x00\x00",7) &&
           dg_detour_install_ex(&g_blade_guard_visibility,base+0x4ee4a4,blade_guard_visibility,
                                base+0x4ee360,base+0x4ee4df,&why,4);
    if(g_b.log)g_b.log("  blade guard visibility: %s\r\n",ok?"weapon manager ready":why);
}
static void blade_install(int requested,int live) {
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
            blade_guard_visibility_install();
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

    if(live<0 || live>2)live=0;

    if(live && !(dg_bridge_blade_enabled() && g_blade_live_ok && blade_act_install()))live=0;
    InterlockedExchange(&g_blade_live_mode,dg_bridge_blade_enabled() && g_blade_live_ok ? live : 0);
    if(InterlockedCompareExchange(&g_blade_live_mode,0,0)) {
        const char *hwhy="probe witnesses unavailable";
        if(!blade_hit_install(&hwhy) && g_b.log)
            g_b.log("  blade contact hook: refused (%s) - no hit pulse, no re-hit block\r\n",hwhy);
    }
    if(dg_bridge_blade_enabled())blade_probe_install();
    if(g_b.log)g_b.log("  blade options: haptic %s, hit speed %.1f m/s%s\r\n",g_blade_haptic?"on":"off",
        g_blade_hit_speed,g_blade_hit_speed>0?"":" (off: live3 gate)");
    if(g_b.log)g_b.log("  blade: %s (%s)\r\n",dg_bridge_blade_enabled()?"gesture adapter installed":"unavailable",why);
    if(g_b.log)g_b.log("  blade live: mode %ld (requested %d, witnesses live %d stick-hold %d)\r\n",
        InterlockedCompareExchange(&g_blade_live_mode,0,0),live,g_blade_live_ok,g_blade_stick_ok);
}
static void blade_stop(void) {
    if(!blade_model_restore() && g_b.log)g_b.log("  blade model: restore refused, mask %u\r\n",g_blade_model_mask);
    InterlockedExchange(&g_blade_live_mode,0);
    dg_detour_remove(&g_blade_guard_visibility);
    dg_detour_remove(&g_blade_hit_detour);
    dg_detour_remove(&g_blade_act_detour);
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
