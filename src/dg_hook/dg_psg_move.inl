#define DG_PSG_MOVE_STANCE_RVA 0x51eb80
#define DG_PSG_MOVE_ACT_RVA    0x713480    /* PSG weapon Act: entry wrapper */
#define DG_PSG_MOVE_ACT_END    0x71398d
#define DG_PSG_MOVE_WAIT       30          /* seams without a lease before the stance comes back */
#define DG_PSG_MOVE_LOG_MAX    40          /* log lines per session; the summary at session end is extra */
enum { DG_PSG_MOVE_NONE = 0, DG_PSG_MOVE_TO_STILL = 1, DG_PSG_MOVE_TO_STANCE = 2, DG_PSG_MOVE_DROP = 3 };
typedef struct {
    int key;          /* vr_psg_move */
    int may_write;    /* seam safe, no owner, no script menu: a write is allowed */
    int active;       /* armed and first person: the function itself is on */
    int witness;      /* player == [0x17DF780] and its flag word readable */
    int lease_held;   /* vr_psg_rifle holds the rifle */
    int rifle_hip;    /* ... and it is in HIP */
    int psg;          /* weapon 4 or 19 */
    int stance;       /* player +0x13D0 */
    int return_mode;  /* the mode setter would swap a still action for a pending return mode */
    uint64_t action_rva;
} DG_PSG_MOVE_IN;
/* hold: owned, HIP, still action, all gates: the weapon Act may get a shot this tick. */
typedef struct { int op; uint64_t set_rva; int hold; const char *why; } DG_PSG_MOVE_OUT;
typedef struct { int owned, wait; } DG_PSG_MOVE_STATE;
static const uint64_t dg_psg_move_still[3] = { 0x51a790, 0x51ac20, 0x51afe0 };   /* still table: stand, squat, ground */
static int dg_psg_move_is_still(uint64_t rva)
{
    return rva == dg_psg_move_still[0] || rva == dg_psg_move_still[1] || rva == dg_psg_move_still[2];
}

static int dg_psg_move_locomotion(uint64_t rva)
{
    return dg_psg_move_is_still(rva) || rva == 0x53d620;
}

static void dg_psg_move_step(DG_PSG_MOVE_STATE *s, const DG_PSG_MOVE_IN *in, DG_PSG_MOVE_OUT *o)
{
    memset(o, 0, sizeof *o);
    if (!s || !in) return;
    if (!s->owned) {
        s->wait = 0;
        if (!in->key || !in->active || !in->may_write || !in->psg || !in->lease_held || !in->rifle_hip) return;
        if (!in->witness) { o->why = "player witness failed: stays native"; return; }
        if (in->action_rva != DG_PSG_MOVE_STANCE_RVA) { o->why = "hip, but not in the PSG stance: left alone"; return; }
        if (in->stance < 0 || in->stance > 2) { o->why = "stance is not stand/squat/ground: stays native"; return; }
        if (in->return_mode) { o->why = "a return mode is pending: stays native"; return; }
        o->op = DG_PSG_MOVE_TO_STILL; o->set_rva = dg_psg_move_still[in->stance];
        s->owned = 1; o->why = "hip: still action (walk and turn free)";
        return;
    }
    if (!in->psg) { s->owned = 0; o->op = DG_PSG_MOVE_DROP; o->why = "weapon changed: the game set its own action"; return; }
    if (in->action_rva == DG_PSG_MOVE_STANCE_RVA) { s->owned = 0; o->op = DG_PSG_MOVE_DROP; o->why = "PSG stance set natively again"; return; }
    if (!in->may_write || !in->witness || !dg_psg_move_locomotion(in->action_rva)) return;    /* wait, no write */
    if (in->key && in->active && in->lease_held && in->rifle_hip) { s->wait = 0; o->hold = 1; return; }
    if (in->key && in->active && !in->lease_held && ++s->wait < DG_PSG_MOVE_WAIT) return;
    o->op = DG_PSG_MOVE_TO_STANCE; o->set_rva = DG_PSG_MOVE_STANCE_RVA;
    s->owned = 0; s->wait = 0;
    o->why = !in->key ? "key off: PSG stance back" :
             !in->active ? "not active (disarmed or first person left): PSG stance back" :
             !in->lease_held ? "rifle lease lost: PSG stance back" : "scope: PSG stance back";
}

static volatile LONG g_psg_move_on;
static struct { int ok, act_ok, walk_ok; uint64_t base; } g_psg_move_w;
static struct {
    DG_PSG_MOVE_STATE st;
    const char *last_why;
    unsigned lines;
    int had_owner;                                   /* a summary is due at session end */
    LONG hold_tick; uint64_t hold_ms, hold_player;  /* set by the seam when o.hold */
    unsigned long to_still, to_stance, drops, shots, shots_reached, readback_bad, refused, faults;
    unsigned long pulses_refused;                    /* psg_v7: weapon bit in the pad without our own trigger draw */
    unsigned long fp_refused;                        /* psg_v8: our draw, but with the first-person button (A) */
} g_psg_move;

static struct { LONG tick; int set; unsigned long long seq; unsigned long stream; } g_psg_move_trig, g_psg_move_spent;
static void psg_move_trigger_note(LONG tick, unsigned long long seq, unsigned long stream)
{
    g_psg_move_trig.tick = tick; g_psg_move_trig.seq = seq; g_psg_move_trig.stream = stream; g_psg_move_trig.set = 1;
}

static struct { LONG tick; int set; } g_psg_move_fp;
static void psg_move_trigger_fp(LONG tick)
{
    g_psg_move_fp.tick = tick; g_psg_move_fp.set = 1;
}
static int psg_move_trigger_fp_now(void)
{
    return g_psg_move_fp.set && g_psg_move_fp.tick == (LONG)g_b.c_ticks;
}
/* The note of THIS tick, not yet spent on a hip shot. */
static int psg_move_trigger_fresh(void)
{
    return g_psg_move_trig.set && g_psg_move_trig.tick == (LONG)g_b.c_ticks &&
           !(g_psg_move_spent.set && g_psg_move_spent.seq == g_psg_move_trig.seq &&
             g_psg_move_spent.stream == g_psg_move_trig.stream);
}
typedef void (__fastcall *DG_PSG_MODE_FN)(void *player, void *action);
typedef void (__fastcall *DG_PSG_ACT_FN)(void *actor);
static DG_DETOUR g_psg_move_act_detour;
static SRWLOCK g_psg_move_act_lock = SRWLOCK_INIT;
static volatile LONG g_psg_move_act_allowed;       /* between start and stop */

/* Retail mode setter 0x57BBE0..0x57BCF7, whole: a leaf that only stores (+0xC58/60/68/70 and the
   resets it does on every mode change); the return-mode swap reads [0x17DF780]+0xC90 bit 55
   and compares with the still table (0x97E530/38/40). */
static const unsigned char g_psg_mode_bytes[0x117] =
    "\x48\x8b\x05\x99\x3b\x26\x01\x49\xb8\x00\x00\x00\x00\x00\x00\x80\x00\x45\x33\xc9\x4c\x85\x80\x90"
    "\x0c\x00\x00\x74\x5b\x48\x8b\x05\xfc\x3a\x26\x01\x48\x85\xc0\x74\x12\x48\x3b\x15\x20\x29\x40\x00"
    "\x75\x09\x4c\x89\x0d\xe7\x3a\x26\x01\xeb\x3a\x48\x8b\x05\xe6\x3a\x26\x01\x48\x85\xc0\x74\x12\x48"
    "\x3b\x15\x0a\x29\x40\x00\x75\x09\x4c\x89\x0d\xd1\x3a\x26\x01\xeb\x1c\x48\x8b\x05\xd0\x3a\x26\x01"
    "\x48\x85\xc0\x74\x13\x48\x3b\x15\xf4\x28\x40\x00\x75\x0a\x4c\x89\x0d\xbb\x3a\x26\x01\x48\x8b\xd0"
    "\x44\x89\x89\x70\x0c\x00\x00\x44\x89\x89\x68\x0c\x00\x00\x48\x89\x91\x60\x0c\x00\x00\x44\x89\x89"
    "\x58\x0c\x00\x00\x44\x39\x0d\x55\xde\x16\x01\x75\x19\xf6\x05\x14\x64\xfd\x00\x21\x75\x10\x66\x44"
    "\x89\x89\xd6\x13\x00\x00\x66\x44\x89\x89\x20\x0d\x00\x00\x66\x44\x89\x89\x8c\x00\x00\x00\x44\x89"
    "\x89\x38\x14\x00\x00\x4c\x89\x89\xa0\x0c\x00\x00\x44\x38\x89\xd3\x13\x00\x00\x7f\x07\xc6\x81\xd3"
    "\x13\x00\x00\x04\x4c\x39\x89\x78\x0c\x00\x00\x75\x31\x66\x83\xb9\xd0\x13\x00\x00\x02\xba\x06\x00"
    "\x00\x00\x0f\xb6\x81\xf0\x0c\x00\x00\x41\x0f\x45\xd1\x3b\xc2\x74\x15\x88\x91\xf0\x0c\x00\x00\x66"
    "\x44\x89\x89\xf2\x0c\x00\x00\xc6\x81\xf1\x0c\x00\x00\x06\xc3";

/* Log budget: DG_PSG_MOVE_LOG_MAX lines per session, then one line saying so. */
static int psg_move_log_slot(void)
{
    if (!g_b.log) return 0;
    if (g_psg_move.lines < DG_PSG_MOVE_LOG_MAX) { g_psg_move.lines++; return 1; }
    if (g_psg_move.lines == DG_PSG_MOVE_LOG_MAX) {
        g_psg_move.lines++;
        g_b.log("  psg move: log budget reached (%d lines); the counters follow at session end\r\n", DG_PSG_MOVE_LOG_MAX);
    }
    return 0;
}

static void psg_move_note(const char *why)
{
    if (!why || why == g_psg_move.last_why) return;
    g_psg_move.last_why = why;
    if (psg_move_log_slot()) g_b.log("  psg move: %s\r\n", why);
}

static void psg_move_summary(const char *when, int forced)
{
    if (!g_b.log || (!forced && !psg_move_log_slot())) return;
    g_b.log("  psg move: summary (%s): still %lu stance %lu drops %lu hip shots %lu (shot seam reached %lu)"
            " readback %lu refused %lu faults %lu pulses without trigger %lu draws with A %lu\r\n", when, g_psg_move.to_still,
            g_psg_move.to_stance, g_psg_move.drops, g_psg_move.shots, g_psg_move.shots_reached, g_psg_move.readback_bad,
            g_psg_move.refused, g_psg_move.faults, g_psg_move.pulses_refused, g_psg_move.fp_refused);
}

static const unsigned char psg_walk_still_query[] =
    {0xb9,1,0,0,0,0xe8,0xc2,0x15,6,0,0x48,0x85,0xc0,0x74,0x32};
static const unsigned char psg_walk_run_query[] =
    {0xb9,1,0,0,0,0xe8,0x86,0xe8,3,0,0x48,0x85,0xc0,0x0f,0x85,0x80,0,0,0};

static const unsigned char psg_walk_step_query[] =
    {0xb9,1,0,0,0,0xe8,0xe7,0xac,5,0,0x48,0x85,0xc0,0x74,0x26};
static DG_DETOUR g_psg_walk_query[3], g_psg_look_pad;
static __declspec(thread) unsigned char g_psg_look_copy[24];
static volatile LONG g_psg_walk_ready;
static int psg_move_player_witness(uintptr_t base, uint64_t player, uint64_t *flags);

static const DWORD g_optics_set_scope=0x5204e0;
static uint64_t g_optics_walk_player;
static int g_optics_walk_item,g_optics_walk_resolved;

static int optics_walk_query_owner(uint64_t player)
{
    uint64_t action,base=g_psg_move_w.base;
    int parked;
    return g_psg_walk_ready && g_optics_walk_resolved && player==g_optics_walk_player &&
        g_b.armed && g_b.fps.state==DG_FPS_ACTIVE && !g_b.s_late_unsafe &&
        !g_b.owner && !g_b.script_menu_only && psg_move_player_witness(base,player,NULL) &&
        optics_locomotion(player,&parked) && parked &&
        interact_read(NULL,player+0xc60,&action,8) &&
        (action==base+0x51a790 || action==base+0x53d620);
}

static int psg_walk_owner(uint64_t player)
{
    uint64_t action = 0;
    uintptr_t base = (uintptr_t)g_psg_move_w.base;
    int weapon = 0;
    if(optics_walk_query_owner(player))return 1;
    if (!g_psg_walk_ready || !g_psg_move_on || !g_psg_move.st.owned ||
        !g_b.armed || g_b.fps.state != DG_FPS_ACTIVE || g_b.s_late_unsafe ||
        g_b.owner || g_b.script_menu_only || !dg_bridge_psg_rifle_enabled() ||
        !psg_rifle_hip_held() || !dg_bridge_psg_hip_now()) return 0;
    if (!psg_move_player_witness(base,player,NULL) ||
        !interact_read(NULL,player+0xb90,&weapon,4) || (weapon != 4 && weapon != 19) ||
        weapon != g_psg_rifle.binding.weapon || !psg_rifle_same_owner(&g_psg_rifle.binding) ||
        !interact_read(NULL,player+0xc60,&action,8) ||
        (action != base+0x51a790 && action != base+0x53d620)) return 0;
    return 1;
}

static void __fastcall psg_walk_query(uint64_t *sp)
{
    if (sp[-3] == 1 && psg_walk_owner(sp[-5])) sp[-3] = 0;
}

static void __fastcall psg_look_sample(uint64_t *sp)
{
    if (!psg_walk_owner(sp[-6]) ||
        !interact_read(NULL, sp[-3], g_psg_look_copy, sizeof g_psg_look_copy)) return;
    g_psg_look_copy[0x16] = g_psg_look_copy[0x17] = 128;
    g_psg_look_copy[5] &= 0x0f; /* directional status bits 12..15 */
    sp[-3] = (uintptr_t)g_psg_look_copy;
}

static void psg_walk_install(uintptr_t base)
{
    const char *why = "retail locomotion query changed";
    unsigned char *image = (unsigned char *)base;
    if (g_psg_walk_ready) return;
    if (!memcmp(image+0x51a9d4,psg_walk_still_query,sizeof psg_walk_still_query) &&
        !memcmp(image+0x53d710,psg_walk_run_query,sizeof psg_walk_run_query) &&
        !memcmp(image+0x5212af,psg_walk_step_query,sizeof psg_walk_step_query) &&
        !memcmp(image+0x57bfa0,"\x48\x8b\x05\xf1\x60\xfd\x00\x48\x23\xc1\xc3",11) &&
        dg_detour_install_ex(&g_psg_walk_query[0],image+0x51a9d4,psg_walk_query,
                             image+0x51a790,image+0x51ac12,&why,4) &&
        dg_detour_install_ex(&g_psg_walk_query[1],image+0x53d710,psg_walk_query,
                             image+0x53d620,image+0x53da00,&why,4) &&
        dg_detour_install_ex(&g_psg_walk_query[2],image+0x5212af,psg_walk_query,
                             image+0x520f6b,image+0x5214a7,&why,4)) {
        InterlockedExchange(&g_psg_walk_ready,1);
        if (!memcmp(image+0x53e477,"\x48\x8b\x8d\x00\x0d\x00\x00\xf3\x0f\x10",10) &&
            dg_detour_install_ex(&g_psg_look_pad,image+0x53e477,psg_look_sample,
                                 image+0x53e180,image+0x53e900,&why,4)) {
            if (g_b.log) g_b.log("  psg hip: isolated movement axes from subjective look\r\n");
        } else if (g_b.log) g_b.log("  psg hip: native look sample retained (%s)\r\n",why);
        if (g_b.log) g_b.log("  psg walk: stand still, stand run and step check WATCH queries ready\r\n");
    } else {
        dg_detour_remove(&g_psg_walk_query[1]);
        dg_detour_remove(&g_psg_walk_query[0]);
        if (g_b.log) g_b.log("  psg walk: native WATCH queries (%s)\r\n",why);
    }
}

static void __fastcall psg_move_act(void *actor);
static const unsigned char g_psg_act_prologue[26] =
    "\x48\x89\x5c\x24\x10\x48\x89\x74\x24\x18\x55\x57\x41\x56\x48\x8d\x6c\x24\xb9\x48\x81\xec\xb0\0\0\0";

static int psg_move_act_entry_live(uint64_t base, unsigned char live[26])
{
    return interact_read(NULL, base + DG_PSG_MOVE_ACT_RVA, live, 26);
}
static int psg_move_act_publish(uint64_t base, const DG_DETOUR *d, const char **why)
{
    unsigned char live[26];
    if (!d->installed || d->stolen < 5 || d->stolen > 26 || (uint64_t)(uintptr_t)d->target != base + DG_PSG_MOVE_ACT_RVA ||
        memcmp(d->original, g_psg_act_prologue, d->stolen)) { *why = "stolen bytes are not the retail prologue"; return 0; }
    if (!psg_move_act_entry_live(base, live) || live[0] != 0xe9 ||
        memcmp(live + d->stolen, g_psg_act_prologue + d->stolen, 26 - d->stolen)) { *why = "entry after the install is not our jump over the retail tail"; return 0; }
    dg_aim_capture_psg_act_own(base, live, d->stolen);
    return 1;
}
static void psg_move_act_install(void)
{
    const char *why = "retail witnesses unavailable";
    int ok = 0;
    unsigned char live[26];
    AcquireSRWLockExclusive(&g_psg_move_act_lock);
    if (g_psg_move_act_detour.installed) ok = 1;
    else if (InterlockedCompareExchange(&g_psg_move_act_allowed, 0, 0) && g_psg_move_w.ok && g_psg_move_w.act_ok) {
        if (!psg_move_act_entry_live(g_psg_move_w.base, live) || memcmp(live, g_psg_act_prologue, sizeof live))
            why = "the live Act entry is not the retail prologue (another hook?)";
        else if ((ok = dg_detour_install_ex(&g_psg_move_act_detour, (void *)(uintptr_t)(g_psg_move_w.base + DG_PSG_MOVE_ACT_RVA),
                                  (void *)psg_move_act, (void *)(uintptr_t)(g_psg_move_w.base + DG_PSG_MOVE_ACT_RVA),
                                  (void *)(uintptr_t)(g_psg_move_w.base + DG_PSG_MOVE_ACT_END), &why, 2)) != 0 &&
                 !psg_move_act_publish(g_psg_move_w.base, &g_psg_move_act_detour, &why))
            ok = 0;
    }
    else why = "not started";
    if (!ok) { dg_detour_remove(&g_psg_move_act_detour); dg_aim_capture_psg_act_own(0, NULL, 0); }
    if (ok && g_psg_move_w.walk_ok) psg_walk_install((uintptr_t)g_psg_move_w.base);
    ReleaseSRWLockExclusive(&g_psg_move_act_lock);
    if (ok == 1 && why && !strcmp(why, "ok") && g_b.log) g_b.log("  psg move: hip trigger seam installed (0x713480)\r\n");
    if (!ok && g_b.log) g_b.log("  psg move: hip trigger seam refused (%s): no shot at the hip\r\n", why);
}

static void psg_move_configure(const DG_BRIDGE_CONFIG *cfg)
{
    LONG want = (cfg && cfg->psg_move == 1) ? 1 : 0;
    if (InterlockedExchange(&g_psg_move_on, want) != want && g_b.log)
        g_b.log("  psg move: %s\r\n", !want ? "off" : g_psg_move_w.ok ? "on (PSG at the hip: walk and turn; scope = PSG stance)"
                                                         : "on, but retail witnesses missing: nothing is written");
    if (want && g_psg_move_w.ok && InterlockedCompareExchange(&g_psg_move_act_allowed, 0, 0)) psg_move_act_install();
    if (optics_configured() && g_optics_walk_resolved && g_psg_move_w.walk_ok &&
        InterlockedCompareExchange(&g_psg_move_act_allowed,0,0))
        psg_walk_install((uintptr_t)g_psg_move_w.base);
}

/* Retail witnesses for every read and write below. */
static void psg_move_resolve(const LiveImage *im)
{
    uint64_t a4 = 0, a19 = 0, still[3] = { 0, 0, 0 };
    int k;
    memset(&g_psg_move_w, 0, sizeof g_psg_move_w);
    g_psg_move.lines = 0; g_psg_move.last_why = NULL; g_psg_move.had_owner = 0; g_psg_move.hold_ms = 0;
    g_psg_move.to_still = g_psg_move.to_stance = g_psg_move.drops = g_psg_move.shots = g_psg_move.shots_reached = 0;
    g_psg_move.readback_bad = g_psg_move.refused = g_psg_move.faults = g_psg_move.pulses_refused = 0;
    g_psg_move.fp_refused = 0;
    memset(&g_psg_move_trig, 0, sizeof g_psg_move_trig); memset(&g_psg_move_spent, 0, sizeof g_psg_move_spent);
    memset(&g_psg_move_fp, 0, sizeof g_psg_move_fp);
    if (!all_valid(im->valid, 0x97e630 + 8, 8, im->size) || !all_valid(im->valid, 0x97e888 + 8, 8, im->size) ||
        !all_valid(im->valid, 0x97e530, 24, im->size)) return;
    memcpy(&a4, im->bytes + 0x97e630 + 8, 8); memcpy(&a19, im->bytes + 0x97e888 + 8, 8);
    memcpy(still, im->bytes + 0x97e530, 24);
    if (a4 != im->base + DG_PSG_MOVE_STANCE_RVA || a19 != im->base + DG_PSG_MOVE_STANCE_RVA) return;   /* PSG rows: action = stance */
    for (k = 0; k < 3; k++) if (still[k] != im->base + dg_psg_move_still[k]) return;                  /* still table [0..2] */
    if (!m9_match(im, 0x57bbe0, g_psg_mode_bytes, sizeof g_psg_mode_bytes) ||                     /* mode setter, whole */
        !m9_match(im, 0x51f019, (const unsigned char *)"\x48\x8b\x86\x00\x0d\x00\x00", 7) ||            /* pad ptr +0xD00 */
        !m9_match(im, 0x51f025, (const unsigned char *)"\x8b\x50\x08", 3) ||                            /* pad press +8 */
        !m9_match(im, 0x51f028, (const unsigned char *)"\x85\x15\xc2\x0e\x46\x00", 6) ||                /* weapon mask 0x97FEF0 */
        !m9_match(im, 0x51f02e, (const unsigned char *)"\x40\x0f\x95\xc5\xff\xc5", 6) ||                /* setne; inc: 1 or 2 */
        !m9_match(im, 0x51f034, (const unsigned char *)"\x66\x83\xbe\xd0\x13\x00\x00\x02", 8) ||        /* stance +0x13D0 */
        !m9_match(im, 0x51f03c, (const unsigned char *)"\x89\xae\x94\x0b\x00\x00", 6))                  /* trigger +0xB94 */
        return;
    g_psg_move_w.walk_ok = m9_match(im,0x51a9d4,psg_walk_still_query,sizeof psg_walk_still_query) &&
        m9_match(im,0x53d710,psg_walk_run_query,sizeof psg_walk_run_query) &&
        m9_match(im,0x5212af,psg_walk_step_query,sizeof psg_walk_step_query);
    g_psg_move_w.base = im->base; g_psg_move_w.ok = 1;
    g_optics_walk_player=0;g_optics_walk_item=0;
    g_optics_walk_resolved=m9_match(im,g_optics_set_scope,
        (const unsigned char*)"\x40\x53\x48\x83\xec\x20\x48\x8b\xd9\x85\xd2\x0f\x85\x8a\x00\x00\x00\xe8\x6a\x51\x01\x00",22);
    /* The hip shot: the player's Act clears the trigger (0x521D43 r12d = 0; 0x521DCD +0xB94; 0x521DD4 +0xBE0),
       the PSG Act (prologue 0x713480) reads it through +0xF8 and fires on 2 (0x7137E3). */
    g_psg_move_w.act_ok =
        m9_match(im, 0x521d43, (const unsigned char *)"\x45\x33\xe4", 3) &&
        m9_match(im, 0x521dcd, (const unsigned char *)"\x44\x89\xa7\x94\x0b\x00\x00\x44\x89\xa7\xe0\x0b\x00\x00", 14) &&
        m9_match(im, DG_PSG_MOVE_ACT_RVA, g_psg_act_prologue, sizeof g_psg_act_prologue) &&
        m9_match(im, 0x7137e3, (const unsigned char *)"\x49\x8b\x86\xf8\x00\x00\x00\x83\x38\x02", 10);
}

/* Session start, after resolve: the wrapper may be installed from now on. */
static void psg_move_start(void)
{
    InterlockedExchange(&g_psg_move_act_allowed, 1);
    if (InterlockedCompareExchange(&g_psg_move_on, 0, 0) && g_psg_move_w.ok) psg_move_act_install();
    if(optics_configured() && g_optics_walk_resolved && g_psg_move_w.walk_ok)
        psg_walk_install((uintptr_t)g_psg_move_w.base);
}

/* Session stop, after the disarmed seams had their chance to restore the stance. */
static void psg_move_stop(void)
{
    AcquireSRWLockExclusive(&g_psg_move_act_lock);
    InterlockedExchange(&g_psg_move_act_allowed, 0);
    InterlockedExchange(&g_psg_walk_ready,0);
    dg_detour_remove(&g_psg_look_pad);
    dg_detour_remove(&g_psg_walk_query[2]);
    dg_detour_remove(&g_psg_walk_query[1]);
    dg_detour_remove(&g_psg_walk_query[0]);
    dg_detour_remove(&g_psg_move_act_detour);
    dg_aim_capture_psg_act_own(0, NULL, 0);       /* after the restore: the retail prologue is accepted either way */
    ReleaseSRWLockExclusive(&g_psg_move_act_lock);
    g_psg_move.hold_ms = 0;
    if (g_psg_move.had_owner || g_psg_move.st.owned) {
        psg_move_summary(g_psg_move.st.owned ? "session end, PSG stance NOT restored" : "session end", 1);
        g_psg_move.had_owner = 0;
    }
}

/* Player of this seam: the arm's trigger slot (arm-0x60+0x228 = player+0xCF4). */
static int psg_move_player(uintptr_t base, uint64_t *player)
{
    uint64_t arm = 0, trig = 0;
    if (!interact_read(NULL, base + 0x17df698, &arm, 8) || arm < 0x10060 ||
        !interact_read(NULL, arm - 0x60 + 0x228, &trig, 8) || trig < 0x10cf4 || (trig & 3)) return 0;
    *player = trig - 0xcf4;
    return 1;
}

static int psg_move_player_witness(uintptr_t base, uint64_t player, uint64_t *flags)
{
    uint64_t owner = 0, f = 0;
    if (!interact_read(NULL, base + 0x17df780, &owner, 8) || owner != player ||
        !interact_read(NULL, owner + 0xc90, &f, 8)) return 0;
    if (flags) *flags = f;
    return 1;
}

static void optics_walk_tick(void)
{
    uintptr_t base=(uintptr_t)g_psg_move_w.base;
    uint64_t player,action,back,set=0;
    dg_radial_inventory_snapshot inv;
    short stance;
    int parked=0;
    if(!g_optics_walk_resolved || !g_psg_walk_ready ||
       (!optics_configured() && !g_optics_walk_player))return;
    if(!psg_move_player(base,&player) || !camera_inventory(&inv) ||
       inv.player_identity!=player || !dg_optics_grip_item(inv.actor_item) ||
       inv.desired_item!=inv.actor_item ||
       (g_optics_walk_player && (g_optics_walk_player!=player || g_optics_walk_item!=inv.actor_item))){
        g_optics_walk_player=0;g_optics_walk_item=0;return;
    }
    if(!g_b.armed || g_b.fps.state!=DG_FPS_ACTIVE || g_b.s_late_unsafe ||
       g_b.owner || g_b.script_menu_only || !psg_move_player_witness(base,player,NULL) ||
       !interact_read(NULL,player+0xc60,&action,8) ||
       !interact_read(NULL,player+0x13d0,&stance,2))return;
    if(optics_locomotion(player,&parked) && parked && stance==0){
        if(action==base+g_optics_set_scope)set=base+dg_psg_move_still[0];
    }else if(g_optics_walk_player &&
             (action==base+0x51a790 || action==base+0x53d620)){
        set=base+g_optics_set_scope;
    }
    if(!set)return;
    ((DG_PSG_MODE_FN)(uintptr_t)(base+0x57bbe0))((void*)(uintptr_t)player,(void*)(uintptr_t)set);
    int applied=interact_read(NULL,player+0xc60,&back,8) && back==set;
    g_optics_walk_player=applied && parked ? player:0;
    g_optics_walk_item=g_optics_walk_player ? inv.actor_item:0;
    if(g_b.log)g_b.log("  optics locomotion: item=%d parked=%d action=%llx applied=%d\r\n",
        inv.actor_item,parked,(unsigned long long)(set-base),applied);
}

/* Game thread, top of the arm seam (every path, also disarmed; the HIP state is the rifle lease of the last seam). */
static void psg_move_tick(void)
{
    DG_PSG_MOVE_IN in;
    DG_PSG_MOVE_OUT o;
    uintptr_t base;
    uint64_t player = 0, action = 0, flags = 0;
    int weapon = 0, was_owned = g_psg_move.st.owned;
    short stance = 0;
    int key = InterlockedCompareExchange(&g_psg_move_on, 0, 0) != 0;
    g_psg_move.hold_ms = 0;                              /* the weapon Act gets a shot only from a fresh hold */
    if (!key && !g_psg_move.st.owned) return;           /* key off and nothing to give back: no reads */
    if (!g_psg_move_w.ok || g_b.a.gm_player_arm_body <= 0x17df698) return;
    base = (uintptr_t)(g_b.a.gm_player_arm_body - 0x17df698);
    if (base != g_psg_move_w.base) return;
    memset(&in, 0, sizeof in);
    __try {
        if (!psg_move_player(base, &player)) return;
        if (!interact_read(NULL, player + 0xb90, &weapon, 4) || !interact_read(NULL, player + 0xc60, &action, 8) ||
            !interact_read(NULL, player + 0x13d0, &stance, 2)) return;
        in.key = key;
        in.may_write = !InterlockedCompareExchange(&g_b.s_late_unsafe, 0, 0) && !g_b.owner && !g_b.script_menu_only;
        in.active = InterlockedCompareExchange(&g_b.armed, 0, 0) && g_b.fps.state == DG_FPS_ACTIVE;
        in.witness = psg_move_player_witness(base, player, &flags);
        in.lease_held = psg_rifle_held() && dg_bridge_psg_rifle_enabled();
        in.rifle_hip = psg_rifle_hip_held();
        in.psg = weapon == 4 || weapon == 19;
        in.stance = stance;
        in.return_mode = (flags & 0x80000000000000ull) != 0;
        in.action_rva = (action >= base && action < base + 0x2000000) ? action - base : (action ? 1 : 0);
        dg_psg_move_step(&g_psg_move.st, &in, &o);
        if (o.op == DG_PSG_MOVE_TO_STILL || o.op == DG_PSG_MOVE_TO_STANCE) {
            uint64_t back = 0;
            if (!psg_move_player_witness(base, player, NULL)) {          /* again, right before the call */
                g_psg_move.refused++; g_psg_move.st.owned = o.op == DG_PSG_MOVE_TO_STANCE;
                psg_move_note("player witness failed before the mode setter: nothing written");
                o.why = NULL;
            } else {
                ((DG_PSG_MODE_FN)(uintptr_t)(base + 0x57bbe0))((void *)(uintptr_t)player, (void *)(uintptr_t)(base + o.set_rva));
                if (!interact_read(NULL, player + 0xc60, &back, 8) || back != base + o.set_rva) {
                    g_psg_move.readback_bad++; g_psg_move.st.owned = 0; o.why = NULL;
                    psg_move_note("mode setter read back another action: left to the game");
                } else if (o.op == DG_PSG_MOVE_TO_STILL) g_psg_move.to_still++;
                else g_psg_move.to_stance++;
            }
        } else if (o.op == DG_PSG_MOVE_DROP) g_psg_move.drops++;
        else if (o.why && !in.witness) g_psg_move.refused++;
        if (o.hold) {
            g_psg_move.hold_tick = g_b.c_ticks; g_psg_move.hold_player = player; g_psg_move.hold_ms = GetTickCount64();
        }
        psg_move_note(o.why);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_psg_move.faults++; g_psg_move.st.owned = 0; g_psg_move.hold_ms = 0;
        psg_move_note("exception: dropped, left to the game");
    }
    if (g_psg_move.st.owned) g_psg_move.had_owner = 1;
    if (was_owned && !g_psg_move.st.owned) psg_move_summary("released", 0);
}

static volatile LONG *psg_move_fire_slot(uint64_t actor, LONG *saved)
{
    uintptr_t base = (uintptr_t)g_psg_move_w.base;
    uint64_t player = 0, action = 0, act = 0, slot = 0, pad = 0, now = GetTickCount64();
    int weapon = 0, aw = 0; LONG trig = 0; unsigned press = 0, mask = 0;
    if (!g_psg_move_w.ok || !g_psg_move_w.act_ok || !g_psg_move.st.owned || !g_psg_move.hold_ms ||
        now < g_psg_move.hold_ms || now - g_psg_move.hold_ms > 100 ||
        (DWORD)((DWORD)g_b.c_ticks - (DWORD)g_psg_move.hold_tick) > 1u) return NULL;
    if (!InterlockedCompareExchange(&g_b.armed, 0, 0) || g_b.fps.state != DG_FPS_ACTIVE ||
        InterlockedCompareExchange(&g_b.s_late_unsafe, 0, 0) || g_b.owner || g_b.script_menu_only ||
        !psg_rifle_hip_held() || !dg_bridge_psg_rifle_enabled()) return NULL;
    if (g_b.a.gm_player_arm_body != base + 0x17df698 || !psg_move_player(base, &player) ||
        player != g_psg_move.hold_player || !psg_move_player_witness(base, player, NULL)) return NULL;
    if (!interact_read(NULL, actor + 8, &act, 8) || act != base + DG_PSG_MOVE_ACT_RVA ||
        !interact_read(NULL, actor + 0xf8, &slot, 8) || slot != player + 0xb94 ||
        !interact_read(NULL, actor + 0x120, &aw, 4) || !interact_read(NULL, player + 0xb90, &weapon, 4) ||
        (weapon != 4 && weapon != 19) || aw != weapon ||
        !interact_read(NULL, player + 0xc60, &action, 8) || action < base || !dg_psg_move_locomotion(action - base) ||
        !interact_read(NULL, player + 0xd00, &pad, 8) || !pad || !interact_read(NULL, pad + 8, &press, 4) ||
        !interact_read(NULL, base + 0x97fef0, &mask, 4) || !(press & mask) ||
        !interact_read(NULL, player + 0xb94, &trig, 4) || trig != 0) return NULL;
    if (psg_move_trigger_fp_now()) {                  /* psg_v8: our draw, but it came with the A press */
        g_psg_move.fp_refused++;
        psg_move_note("trigger draw together with the first-person button (A) or a pending view toggle: no hip shot");
        return NULL;
    }
    if (!psg_move_trigger_fresh()) {                  /* a weapon-button pulse, not the player's trigger */
        g_psg_move.pulses_refused++;
        psg_move_note("weapon bit in the pad without a VR trigger draw this tick: no hip shot");
        return NULL;
    }
    *saved = trig;
    return (volatile LONG *)(uintptr_t)(player + 0xb94);
}

/* PSG weapon Act (0x713480), entry wrapper. The stance's own rule, at the one place the weapon
   can see it: 2 before the original Act, the value from before the call back after it. */
static void __fastcall psg_move_act(void *actor)
{
    DG_PSG_ACT_FN original = (DG_PSG_ACT_FN)g_psg_move_act_detour.tramp;
    volatile LONG *trig = NULL;
    LONG saved = 0, seam_before = 0;
    int reached;
    if (InterlockedCompareExchange(&g_psg_move_on, 0, 0) && g_psg_move.st.owned) {
        __try { trig = psg_move_fire_slot((uint64_t)(uintptr_t)actor, &saved); }
        __except (EXCEPTION_EXECUTE_HANDLER) { trig = NULL; g_psg_move.faults++; }
    }
    if (trig) {
        seam_before = g_psg_applied + g_psg_refused;
        *trig = 2;
        g_psg_move.hold_ms = 0;                            /* one shot per hold */
        g_psg_move_spent = g_psg_move_trig;                /* and one per pull */
    }
    if (original) original(actor);
    if (!trig) return;
    if (*trig == 2) *trig = saved;                         /* no residue, unless someone else wrote since */
    reached = (g_psg_applied + g_psg_refused) != seam_before;
    g_psg_move.shots++;
    if (reached) g_psg_move.shots_reached++;
    if (psg_move_log_slot())
        g_b.log("  psg move: hip shot %lu: trigger 2 at the PSG Act (%s)\r\n", g_psg_move.shots,
                reached ? "shot seam reached" : g_psg.live ? "Act did not fire: reload, empty or no zoom" : "shot seam not installed");
}
