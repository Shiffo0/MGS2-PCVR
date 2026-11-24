static volatile LONG g_psg_rifle_on;
static volatile LONG g_psg_gest_open_cm, g_psg_gest_close_cm, g_psg_gest_open_deg, g_psg_gest_close_deg;
static volatile LONG g_psg_hip;              /* published: 1 = HIP while held */
enum { PSG_MODE_SCOPE = 0, PSG_MODE_HIP = 1 };
#define PSG_GESTURE_HOLD_MS 250
static struct {
    DG_PSG_BINDING binding;
    uintptr_t base;
    volatile LONG held;
    int cleared, mode, sight_set, sight_prior;
    uint64_t saved_root;
    unsigned long applied, released, owner_lost, refused, cleared_frames;
    unsigned long model_cleared, model_still_hidden, model_refused;
    double max_model_hand_mm;
    unsigned long hip_frames, scope_frames, warp_hidden, opens, closes;
    ULONGLONG gesture_ms;   /* last valid gesture sample */
    int gesture_last;       /* DG_PSG_GESTURE_HIP/SCOPE */
    int min_mm, min_ddeg;   /* session minima from the samples, for the log */
    const char *last_why;
    /* psg_v7: the first-person dummy rifle ([0x17DF688] = actor+0xA0) is what HIP draws. */
    int dummy_leased, dummy_shown;
    uint64_t dummy_objs, dummy_saved_root;
    unsigned long dummy_frames, dummy_hidden_back, dummy_refused, dummy_lines;
    ULONGLONG dummy_log_ms;
} g_psg_rifle;
/* Retail witnesses for the sight bit and the lens warp (psg_rifle_resolve). */
static struct { int ok; uint64_t base; int count_ok; int dummy_ok; int recoil_zoom_ok; } g_psg_overlay;
/* psg_v7: retail 0x4EE43B..0x4EE499, the weapon manager's dummy block: r8 = [[0x17DF688]] (objs),
   r11 = ([[[0x17DF698]+0x30]+0x58] & 0x300) << 4 (the first-person arm's hide bits), then for every model
   (objs+0x1D0, stride 0x180, count objs+0x64): flag = (flag & ~0x3000) | r11 (or all hidden on r10). */
static const unsigned char g_psg_mgr_dummy_block[95] = {
    0x4c,0x8b,0x05,0x46,0x12,0x2f,0x01, 0x4d,0x85,0xc0, 0x74,0x5d, 0x48,0x8b,0x05,0x4a,0x12,0x2f,0x01,
    0x4d,0x8b,0x00, 0x48,0x8b,0x48,0x30, 0x44,0x8b,0x59,0x58, 0x41,0x81,0xe3,0x00,0x03,0x00,0x00,
    0x41,0xc1,0xe3,0x04, 0x66,0x45,0x3b,0x48,0x64, 0x7d,0x39, 0x49,0x8d,0x90,0xd0,0x01,0x00,0x00,
    0x8b,0x0a, 0x48,0x8d,0x92,0x80,0x01,0x00,0x00, 0x8b,0xc1, 0x81,0xc9,0x00,0x30,0x00,0x00,
    0x25,0xff,0xcf,0xff,0xff, 0x4d,0x85,0xd2, 0x0f,0x44,0xc8, 0x41,0xff,0xc1, 0x41,0x0b,0xcb,
    0x89,0x8a,0x80,0xfe,0xff,0xff };

int dg_bridge_psg_rifle_enabled(void) { return InterlockedCompareExchange(&g_psg_rifle_on, 0, 0) != 0; }
int dg_bridge_psg_hip_now(void) { return InterlockedCompareExchange(&g_psg_hip, 0, 0) != 0; }
void dg_bridge_psg_gesture_cfg(DG_PSG_GESTURE_CFG *out)
{
    dg_psg_gesture_defaults(out);
    if (InterlockedCompareExchange(&g_psg_gest_open_cm, 0, 0))
        dg_psg_gesture_cfg_set(out, (int)g_psg_gest_open_cm, (int)g_psg_gest_close_cm,
                               (int)g_psg_gest_open_deg, (int)g_psg_gest_close_deg);
}
static void psg_rifle_configure(const DG_BRIDGE_CONFIG *cfg)
{
    LONG want = (cfg && cfg->psg_rifle == 1) ? 1 : 0;
    DG_PSG_GESTURE_CFG probe;
    int g = cfg && cfg->psg_gesture_set &&
        dg_psg_gesture_cfg_set(&probe, cfg->psg_open_cm, cfg->psg_close_cm, cfg->psg_open_deg, cfg->psg_close_deg);
    InterlockedExchange(&g_psg_gest_close_cm, g ? cfg->psg_close_cm : 0);
    InterlockedExchange(&g_psg_gest_open_deg, g ? cfg->psg_open_deg : 0);
    InterlockedExchange(&g_psg_gest_close_deg, g ? cfg->psg_close_deg : 0);
    InterlockedExchange(&g_psg_gest_open_cm, g ? cfg->psg_open_cm : 0);
    if (InterlockedExchange(&g_psg_rifle_on, want) != want && g_b.log) {
        DG_PSG_GESTURE_CFG c; dg_bridge_psg_gesture_cfg(&c);
        if (want) g_b.log("  psg rifle: on (PSG in the first-person hand, gesture scope open %.0f cm/%.0f deg, close %.0f cm/%.0f deg)\r\n",
                          c.open_m * 100.0, c.open_deg, c.close_m * 100.0, c.close_deg);
        else g_b.log("  psg rifle: off\r\n");
    }
}
static int psg_rifle_held(void) { return g_psg_rifle.held; }
static int psg_rifle_hip_held(void) { return g_psg_rifle.held && g_psg_rifle.mode == PSG_MODE_HIP; }

/* Retail witnesses (2.1.0.0) for writes 3 and 4. Without them HIP is never
   entered: the rifle stays native (SCOPE). */
static struct { uint64_t arm, objs, evm; LONG obj_owned, evm_owned; unsigned long frames, repaid; } g_psg_arm;   /* psg_v8 */
static void psg_rifle_resolve(const LiveImage *im)
{
    memset(&g_psg_overlay, 0, sizeof g_psg_overlay);
    memset(&g_psg_arm, 0, sizeof g_psg_arm);     /* psg_v8: a new session owes no arm bits */
    if (!m9_match(im, 0x71357d, (const unsigned char *)"\x81\x25\xed\x13\x0a\x01\xff\xfe\xff\xff", 10) ||   /* and [0x17B4974],~0x100 */
        !m9_match(im, 0x4e6f26, (const unsigned char *)"\xf7\x05\x44\xda\x2c\x01\x00\x01\x00\x00", 10) ||   /* test [0x17B4974],0x100 */
        !m9_match(im, 0x70ed57, (const unsigned char *)"\x48\x8b\x05\xa2\xe3\xfd\x00", 7) ||                 /* mov rax,[0x16ED100] */
        !m9_match(im, 0x70ed67, (const unsigned char *)"\x48\x8b\x80\xb0\x00\x00\x00", 7) ||                 /* mov rax,[rax+0xB0] */
        !m9_match(im, 0x70ed6e, (const unsigned char *)"\x83\x08\x10", 3) ||                                 /* or [rax],0x10 */
        !m9_match(im, 0x70edc8, (const unsigned char *)"\x48\x8d\x15\x91\xf6\xff\xff", 7) ||                 /* lea rdx,Act 0x70E460 */
        !m9_match(im, 0x7134b2, (const unsigned char *)"\x8b\xb9\x28\x01\x00\x00", 6) ||                     /* chanl +0x128 */
        !m9_match(im, 0x7134b8, (const unsigned char *)"\x8b\xb1\x2c\x01\x00\x00", 6) ||                     /* vflag +0x12C */
        !m9_match(im, 0x7134ec, (const unsigned char *)"\xba\x00\x10\x00\x00", 5) ||                         /* 0x1000 << chanl */

        !m9_match(im, 0x4ee3be, (const unsigned char *)"\x48\x8b\x15\xcb\x12\x2f\x01", 7) ||                 /* mov rdx,[0x17DF690] */
        !m9_match(im, 0x4ee3da, (const unsigned char *)"\x81\xe3\x00\x30\x00\x00", 6) ||                     /* and ebx,0x3000 */
        !m9_match(im, 0x4ee3e7, (const unsigned char *)"\x4c\x8d\x82\xd0\x01\x00\x00", 7) ||                 /* lea r8,[rdx+0x1D0] */
        !m9_match(im, 0x4ee3f3, (const unsigned char *)"\x4d\x8d\x80\x80\x01\x00\x00", 7) ||                 /* model stride 0x180 */
        !m9_match(im, 0x4ee412, (const unsigned char *)"\x41\x89\x88\x80\xfe\xff\xff", 7))                   /* mov [r8-0x180],ecx */
        return;

    g_psg_overlay.recoil_zoom_ok =
        m9_match(im,0x7136a4,(const unsigned char *)"\x8b\x48\x20\x41\x89\x8e\x34\x01\x00\x00",10) &&
        m9_match(im,0x7136b8,(const unsigned char *)"\x41\xc6\x86\x38\x01\x00\x00\x02",8);
    g_psg_overlay.base = im->base; g_psg_overlay.ok = 1;

    g_psg_overlay.count_ok =
        m9_match(im, 0x4ee3d0, (const unsigned char *)"\x48\x8b\x12", 3) &&                                  /* mov rdx,[rdx] = objs */
        m9_match(im, 0x4ee3e0, (const unsigned char *)"\x66\x44\x3b\x4a\x64", 5) &&                          /* cmp r9w,[rdx+0x64] */
        m9_match(im, 0x4ee3e5, (const unsigned char *)"\x7d", 1) &&                                          /* jge: none when <= 0 */
        m9_match(im, 0x4ee419, (const unsigned char *)"\x0f\xbf\x42\x64\x44\x3b\xd8\x7c", 8);                /* movsx eax,[rdx+0x64]; cmp; jl */

    g_psg_overlay.dummy_ok =
        m9_match(im, 0x713acd, (const unsigned char *)"\x48\x8d\xb3\xa0\x00\x00\x00", 7) &&                  /* lea rsi,[rbx+0xA0] */
        m9_match(im, 0x713b06, (const unsigned char *)"\x48\x8b\x06\x81\x48\x58\x00\x30\x00\x00", 10) &&      /* mov rax,[rsi]; or [rax+0x58],0x3000 */
        m9_match(im, 0x713b21, (const unsigned char *)"\x48\x89\x35\x60\xbb\x0c\x01", 7) &&                  /* mov [0x17DF688],rsi */
        m9_match(im, 0x4ee43b, g_psg_mgr_dummy_block, sizeof g_psg_mgr_dummy_block);                          /* manager, second block */
}

/* One log line per change of reason. */
static void psg_rifle_note(const char *why)
{
    if (why == g_psg_rifle.last_why) return;
    g_psg_rifle.last_why = why;
    if (g_b.log) g_b.log("  psg rifle: %s\r\n", why ? why : "?");
}

static struct { int weapon; const char *what; unsigned lines; } g_psg_rifle_wlog;
static void psg_rifle_wline(int weapon, const char *what, const char *why)
{
    if (!g_b.log || (weapon == g_psg_rifle_wlog.weapon && what == g_psg_rifle_wlog.what) || g_psg_rifle_wlog.lines >= 200) return;
    g_psg_rifle_wlog.weapon = weapon; g_psg_rifle_wlog.what = what; g_psg_rifle_wlog.lines++;
    g_b.log("  psg rifle weapon %d: %s%s%s\r\n", weapon, what, why ? ": " : "", why ? why : "");
}
/* The weapon id of the player behind this arm (arm-0x60+0x228 = player+0xCF4, weapon +0xB90), -1 unknown. */
static int psg_rifle_seam_weapon(ULONGLONG arm)
{
    uint64_t trig = 0; int w = -1;
    if (arm < 0x10060 || !interact_read(NULL, arm - 0x60 + 0x228, &trig, 8) || trig < 0x10cf4 ||
        !interact_read(NULL, trig - 0xcf4 + 0xb90, &w, 4)) return -1;
    return w;
}
#define PSG_WLINE(w, what, why) psg_rifle_wline((w), (what), (why))
#define PSG_WRESET(w) do { g_psg_rifle_wlog.weapon = (w); g_psg_rifle_wlog.what = NULL; } while (0)

/* Gesture -> mode. A stale or invalid gesture holds the last valid answer for
   PSG_GESTURE_HOLD_MS, then falls back to SCOPE (native). */
static int psg_rifle_mode_now(ULONGLONG now)
{
    DG_ACTION_SAMPLE p;
    int g = 0;
    memset(&p, 0, sizeof p);
    if (g_action_provider && TryAcquireSRWLockExclusive(&g_action_lock)) {
        g_action_provider(&p);
        ReleaseSRWLockExclusive(&g_action_lock);
        if (p.psg_gesture && p.age_ms <= 100) g = p.psg_gesture;
    }
    if (g) {
        g_psg_rifle.gesture_ms = now; g_psg_rifle.gesture_last = g;
        if (!g_psg_rifle.min_mm || p.psg_gesture_mm < g_psg_rifle.min_mm) g_psg_rifle.min_mm = p.psg_gesture_mm;
        if (!g_psg_rifle.min_ddeg || p.psg_gesture_ddeg < g_psg_rifle.min_ddeg) g_psg_rifle.min_ddeg = p.psg_gesture_ddeg;
    } else if (g_psg_rifle.gesture_ms && now - g_psg_rifle.gesture_ms <= PSG_GESTURE_HOLD_MS) {
        g = g_psg_rifle.gesture_last;
    }
    if (g == DG_PSG_GESTURE_HIP && g_psg_overlay.ok) return PSG_MODE_HIP;
    return PSG_MODE_SCOPE;
}

/* Write 3: sight status bit 0x100. Interlocked on the one bit only. */
static void psg_sight_hide(uintptr_t base)
{
    volatile LONG *status = (volatile LONG *)(uintptr_t)(base + 0x17b4974);
    LONG before;
    if (g_psg_rifle.sight_set) { InterlockedOr(status, 0x100); return; }
    before = InterlockedOr(status, 0x100);
    g_psg_rifle.sight_set = 1; g_psg_rifle.sight_prior = (before & 0x100) != 0;
}
static void psg_sight_restore(uintptr_t base)
{
    if (!g_psg_rifle.sight_set) return;
    if (!g_psg_rifle.sight_prior)
        InterlockedAnd((volatile LONG *)(uintptr_t)(base + 0x17b4974), ~0x100);
    g_psg_rifle.sight_set = 0;
}

static void psg_warp_hide(uintptr_t base)
{
    uint64_t work = 0, act = 0, pack = 0;
    if (!interact_read(NULL, base + 0x16ed100, &work, 8) || !work ||
        !interact_read(NULL, work + 8, &act, 8) || act != base + 0x70e460 ||
        !interact_read(NULL, work + 0xb0, &pack, 8) || !plausible_ptr(pack)) return;
    InterlockedOr((volatile LONG *)(uintptr_t)pack, 0x10);
    g_psg_rifle.warp_hidden++;
}

static void psg_models_show(const DG_PSG_BINDING *b)
{
    short n = 0;
    int i;
    float model[4][4], hand[4][4];
    /* The count is the manager's own loop bound (witnessed in psg_rifle_resolve), read from the same
       objs; the loop never goes further than the manager's own writes. 16 stays as a hard cap. */
    if (!g_psg_overlay.count_ok) {
        g_psg_rifle.model_refused++; psg_rifle_note("model count (objs+0x64) not witnessed at 0x4EE360: models not shown");
        return;
    }
    if (!interact_read(NULL, b->normal_objs + 0x64, &n, 2) || n < 1 || n > 16) { g_psg_rifle.model_refused++; return; }
    for (i = 0; i < n; i++) {
        uint64_t at = b->normal_objs + 0x1d0 + (uint64_t)i * 0x180;
        unsigned f = (unsigned)RD32(at);
        if (f & b->vflag) {
            WR32(at, (LONG)(f & ~b->vflag)); g_psg_rifle.model_cleared++;
        }
        if ((unsigned)RD32(at) & b->vflag) g_psg_rifle.model_still_hidden++;
    }
    if (interact_read(NULL, b->normal_objs + 0x110, model, sizeof model) &&
        interact_read(NULL, b->hand, hand, sizeof hand)) {
        double dx = model[3][0] - hand[3][0], dy = model[3][1] - hand[3][1], dz = model[3][2] - hand[3][2];
        double mm = sqrt(dx * dx + dy * dy + dz * dz);
        if (mm == mm && mm > g_psg_rifle.max_model_hand_mm) g_psg_rifle.max_model_hand_mm = mm;
    }
}

static int psg_dummy_same(const DG_PSG_BINDING *h)
{
    uint64_t sub = 0, objs = 0;
    return g_psg_rifle.dummy_leased && g_psg_rifle.dummy_objs &&
           interact_read(NULL, g_psg_rifle.base + 0x17df688, &sub, 8) && sub == h->subject_object &&
           sub == h->actor + 0xa0 && interact_read(NULL, sub, &objs, 8) && objs == g_psg_rifle.dummy_objs;
}
/* Lease start: the dummy only with its retail witnesses and as the constructor left it (rooted on the
   body's joint 6 = the normal rifle's native root). Otherwise HIP draws nothing, as before psg_v7. */
static void psg_dummy_lease(const DG_PSG_BINDING *b)
{
    uint64_t objs = 0, root = 0;
    g_psg_rifle.dummy_leased = g_psg_rifle.dummy_shown = 0;
    g_psg_rifle.dummy_objs = g_psg_rifle.dummy_saved_root = 0;
    g_psg_rifle.dummy_lines = 0; g_psg_rifle.dummy_log_ms = 0;
    if (!g_psg_overlay.dummy_ok) {
        g_psg_rifle.dummy_refused++; psg_rifle_note("dummy rifle witnesses missing: nothing drawn at the hip"); return;
    }
    if (b->subject_object != b->actor + 0xa0 || !interact_read(NULL, b->subject_object, &objs, 8) || !plausible_ptr(objs) ||
        objs == b->normal_objs || objs == b->arm_objs || !interact_read(NULL, objs + 0x40, &root, 8)) {
        g_psg_rifle.dummy_refused++; psg_rifle_note("dummy rifle not as its constructor left it: nothing drawn at the hip"); return;
    }

    if (root == b->hand && root != b->native_root) {
        WR32(objs + 0x58, (LONG)((unsigned)RD32(objs + 0x58) | 0x3000u));
        *(volatile uint64_t *)(uintptr_t)(objs + 0x40) = b->native_root;
        root = b->native_root; g_psg_rifle.dummy_hidden_back++;
        psg_rifle_note("dummy rifle left on the hand by an earlier lease: hidden and rooted back first");
    }
    if (root != b->native_root) {
        g_psg_rifle.dummy_refused++; psg_rifle_note("dummy rifle not as its constructor left it: nothing drawn at the hip"); return;
    }
    g_psg_rifle.dummy_objs = objs; g_psg_rifle.dummy_saved_root = root; g_psg_rifle.dummy_leased = 1;
}
/* Back to the constructor's state: hidden on every channel (| 0x3000) and the old root. */
static void psg_dummy_hide(void)
{
    uint64_t objs = g_psg_rifle.dummy_objs;
    if (!g_psg_rifle.dummy_shown) return;
    WR32(objs + 0x58, (LONG)((unsigned)RD32(objs + 0x58) | 0x3000u));
    *(volatile uint64_t *)(uintptr_t)(objs + 0x40) = g_psg_rifle.dummy_saved_root;
    g_psg_rifle.dummy_shown = 0; g_psg_rifle.dummy_hidden_back++;
}
/* HIP: root on the arm hand, our channel's object bit cleared. The manager gives its models the arm's
   visibility (second block) after this seam, so they follow the arm. A dummy that is no longer ours
   is dropped without a write. */
static void psg_dummy_show(const DG_PSG_BINDING *b)
{
    uint64_t objs = g_psg_rifle.dummy_objs;
    unsigned f;
    if (!g_psg_rifle.dummy_leased) return;
    if (!psg_dummy_same(b)) {
        g_psg_rifle.dummy_leased = g_psg_rifle.dummy_shown = 0; g_psg_rifle.dummy_refused++;
        psg_rifle_note("dummy rifle changed under the lease: dropped, not written");
        return;
    }
    *(volatile uint64_t *)(uintptr_t)(objs + 0x40) = b->hand;
    f = (unsigned)RD32(objs + 0x58);
    if (f & b->vflag) WR32(objs + 0x58, (LONG)(f & ~b->vflag));
    g_psg_rifle.dummy_shown = 1; g_psg_rifle.dummy_frames++;
}
/* SCOPE (and every other way out while the owner is still the same). */
static void psg_dummy_unshow(const DG_PSG_BINDING *b)
{
    if (!g_psg_rifle.dummy_shown) return;
    if (psg_dummy_same(b)) psg_dummy_hide();
    else { g_psg_rifle.dummy_leased = g_psg_rifle.dummy_shown = 0; g_psg_rifle.dummy_refused++; }
}

static void psg_dummy_unshow_safe(const DG_PSG_BINDING *b)
{
    __try { if (g_psg_rifle.dummy_shown && psg_dummy_same(b)) psg_dummy_hide(); }
    __except (EXCEPTION_EXECUTE_HANDLER) { g_psg_rifle.dummy_refused++; }
}

static void psg_scope_arm_forget(void)       /* the debt only; the counters are for the release line */
{
    g_psg_arm.arm = g_psg_arm.objs = g_psg_arm.evm = 0; g_psg_arm.obj_owned = g_psg_arm.evm_owned = 0;
}
static int psg_scope_arm_step(uint64_t arm
                              ,int optic_open
                              )
{
    uint64_t objs, evm;
    LONG of, ef;
    int want = psg_rifle_held() && !psg_rifle_hip_held() && dg_bridge_psg_rifle_enabled();
    want=want || optic_open;
    if (!plausible_ptr(arm) || region_end(arm) < arm + 0x38) { psg_scope_arm_forget(); return 0; }
    objs = *(volatile uint64_t *)(uintptr_t)arm;
    evm = *(volatile uint64_t *)(uintptr_t)(arm + 0x30);
    if (!plausible_ptr(objs) || region_end(objs) < objs + 0x5c ||
        (evm && (!plausible_ptr(evm) || region_end(evm) < evm + 0x5c))) { psg_scope_arm_forget(); return 0; }
    if (g_psg_arm.arm && (g_psg_arm.arm != arm || g_psg_arm.objs != objs || g_psg_arm.evm != evm)) psg_scope_arm_forget();
    of = RD32(objs + 0x58); ef = evm ? RD32(evm + 0x58) : 0;
    if (want) {
        if (!g_psg_arm.arm) {
            g_psg_arm.arm = arm; g_psg_arm.objs = objs; g_psg_arm.evm = evm;
            g_psg_arm.obj_owned = (~of) & 0x1000; g_psg_arm.evm_owned = evm ? ((~ef) & 0x100) : 0;
        }
        WR32(objs + 0x58, of | 0x1000);
        if (evm) WR32(evm + 0x58, ef | 0x100);
        g_psg_arm.frames++;
        return 1;
    }
    if (g_psg_arm.arm) {
        WR32(objs + 0x58, of & ~g_psg_arm.obj_owned);
        if (evm) WR32(evm + 0x58, ef & ~g_psg_arm.evm_owned);
        g_psg_arm.repaid++;
        psg_scope_arm_forget();
    }
    return 0;
}
/* Diagnostic line in HIP, read before our writes: the dummy's object flag and first model flag at the
   seam (after the first frame expected without our channel bit), its root, and the arm flag the manager
   copies into the dummy's models. First 3 HIP frames of a lease, then once a second, 40 per lease. */
static void psg_dummy_meter(const DG_PSG_BINDING *b)
{
    uint64_t objs = g_psg_rifle.dummy_objs, root = 0, armobj = 0, evm = 0;
    unsigned of = 0, mf = 0, ef = 0; short n = 0;
    ULONGLONG now = GetTickCount64();
    if (!g_b.log || !g_psg_rifle.dummy_leased || g_psg_rifle.dummy_lines >= 40) return;
    if (g_psg_rifle.dummy_frames >= 3 && now >= g_psg_rifle.dummy_log_ms && now - g_psg_rifle.dummy_log_ms < 1000) return;
    if (!interact_read(NULL, objs + 0x58, &of, 4) || !interact_read(NULL, objs + 0x40, &root, 8) ||
        !interact_read(NULL, objs + 0x64, &n, 2)) return;
    if (n >= 1) interact_read(NULL, objs + 0x1d0, &mf, 4);
    if (interact_read(NULL, g_psg_rifle.base + 0x17df698, &armobj, 8) && armobj &&
        interact_read(NULL, armobj + 0x30, &evm, 8) && evm) interact_read(NULL, evm + 0x58, &ef, 4);
    g_psg_rifle.dummy_lines++; g_psg_rifle.dummy_log_ms = now;
    g_b.log("  psg rifle dummy: hip frame %lu entry flags objs 0x%X model0 0x%X (models %d, vflag 0x%X) arm-evm 0x%X root %s\r\n",
            g_psg_rifle.dummy_frames + 1, of, mf, (int)n, b->vflag, ef,
            root == b->hand ? "=hand" : root == g_psg_rifle.dummy_saved_root ? "=native" : "other");
}

static int psg_rifle_same_owner(const DG_PSG_BINDING *h)
{
    uint64_t normal = 0, objs = 0, act = 0, root = 0;
    if (!interact_read(NULL, g_psg_rifle.base + 0x17df690, &normal, 8) || normal != h->normal_object ||
        !interact_read(NULL, h->normal_object, &objs, 8) || objs != h->normal_objs ||
        !interact_read(NULL, h->actor + 8, &act, 8) || act != g_psg_rifle.base + 0x713480 ||
        !interact_read(NULL, h->normal_objs + 0x40, &root, 8))
        return 0;
    return root == h->hand;
}

int dg_bridge_psg_render_zoom(uint64_t *identity,float *angle)
{
    const DG_PSG_BINDING *b=&g_psg_rifle.binding;
    uint64_t zoom=0,scope=0,linked=0,callback=0;
    int weapon=0;
    /* Recoil can interrupt input acceptance while the rifle remains scoped. */
    if(!g_psg_rifle.held)return dg_bridge_psg_zoom_now(identity,angle);
    if(g_psg_rifle.mode!=PSG_MODE_SCOPE || !g_b.armed || g_b.owner || g_b.s_late_unsafe ||
       g_b.script_menu_only || g_b.fps.state!=DG_FPS_ACTIVE || !psg_rifle_same_owner(b) ||
       !interact_read(NULL,b->player+0xb90,&weapon,4) || weapon!=b->weapon ||
       !interact_read(NULL,b->actor+0x110,&zoom,8) || !zoom ||
       !interact_read(NULL,zoom+8,&callback,8) || !g_zoom_a.valid || callback!=g_zoom_a.callback ||
       !interact_read(NULL,b->actor+0x108,&scope,8) || !scope ||
       !interact_read(NULL,zoom+0x58,&linked,8) || linked!=scope)return 0;
    unsigned char recoil=0;
    if(g_psg_overlay.recoil_zoom_ok && !interact_read(NULL,b->actor+0x138,&recoil,1))return 0;
    uint64_t focal=(recoil & 2) ? b->actor+0x134 : scope+0x20;
    if(!interact_read(NULL,focal,angle,4) || !(*angle>0 && *angle<=60))return 0;
    if(*angle<8) {
        if(!(recoil & 2))return 0;
        *angle=8; /* the current native camera can still be interpolating at the wide stop */
    }
    *identity=zoom;return 1;
}

static int psg_rifle_live_psg_now(void)
{
    uint64_t normal = 0, act = 0;
    return interact_read(NULL, g_psg_rifle.base + 0x17df690, &normal, 8) && normal >= 0x10060 &&
           interact_read(NULL, normal - 0x60 + 8, &act, 8) && act == g_psg_rifle.base + 0x713480;
}
static void psg_rifle_release(void)
{
    const DG_PSG_BINDING *h = &g_psg_rifle.binding;
    InterlockedExchange(&g_psg_hip, 0);

    if (!InterlockedExchange(&g_psg_rifle.held, 0)) return;
    __try {

        if (psg_dummy_same(h)) psg_dummy_hide();
        if (psg_rifle_same_owner(h)) {
            psg_sight_restore(g_psg_rifle.base);
            *(volatile uint64_t *)(uintptr_t)(h->normal_objs + 0x40) = g_psg_rifle.saved_root;
            if (g_psg_rifle.cleared)
                WR32(h->normal_objs + 0x58, (LONG)((unsigned)RD32(h->normal_objs + 0x58) | h->vflag));
            g_psg_rifle.released++;
        } else {                            /* destroyed or re-rooted natively: never write the object */
            g_psg_rifle.owner_lost++;

            if (!psg_rifle_live_psg_now()) psg_sight_restore(g_psg_rifle.base);
            g_psg_rifle.sight_set = 0;
        }
    } __except (EXCEPTION_EXECUTE_HANDLER) { g_psg_rifle.owner_lost++; g_psg_rifle.sight_set = 0; }
    psg_probe_rifle_meter_summary();   /* psg_v4: read-only rifle meter (vr_dev_psgprobe) */
    g_psg_rifle.cleared = 0; g_psg_rifle.mode = PSG_MODE_SCOPE;
    g_psg_rifle.dummy_leased = g_psg_rifle.dummy_shown = 0;   /* psg_v7: never written through an old lease */
    if (g_b.log) g_b.log("  psg rifle: released (applied=%lu released=%lu owner_lost=%lu hip=%lu scope=%lu opens=%lu closes=%lu warp=%lu gesture min %d mm %d.%d deg)"
                         " show: objs %lu models %lu still-hidden %lu refused %lu, model-hand max %.0f mm;"
                         " dummy frames %lu hidden back %lu refused %lu; arm hidden in scope %lu frames, given back %lu\r\n",
                         g_psg_rifle.applied, g_psg_rifle.released, g_psg_rifle.owner_lost,
                         g_psg_rifle.hip_frames, g_psg_rifle.scope_frames, g_psg_rifle.opens, g_psg_rifle.closes,
                         g_psg_rifle.warp_hidden, g_psg_rifle.min_mm, g_psg_rifle.min_ddeg / 10, g_psg_rifle.min_ddeg % 10,
                         g_psg_rifle.cleared_frames, g_psg_rifle.model_cleared, g_psg_rifle.model_still_hidden,
                         g_psg_rifle.model_refused, g_psg_rifle.max_model_hand_mm,
                         g_psg_rifle.dummy_frames, g_psg_rifle.dummy_hidden_back, g_psg_rifle.dummy_refused,
                         g_psg_arm.frames, g_psg_arm.repaid);
}

static void psg_rifle_apply(ULONGLONG arm)
{
    DG_PSG_BINDING b;
    const char *why = NULL;
    uintptr_t base;
    int mode;
    if (!dg_bridge_psg_rifle_enabled()) { psg_rifle_release(); return; }
    if (!g_b.armed || g_b.owner || g_b.script_menu_only || g_b.s_late_unsafe ||
        g_b.fps.state != DG_FPS_ACTIVE || g_b.a.gm_player_arm_body <= 0x17df698 || !arm) {
        psg_rifle_release(); return;
    }
    base = (uintptr_t)(g_b.a.gm_player_arm_body - 0x17df698);
    if (!dg_aim_capture_psg_binding(base, arm, &b, &why)) {
        if (why && strcmp(why, "not the PSG")) {
            g_psg_rifle.refused++; psg_rifle_note(why);
            PSG_WLINE(psg_rifle_seam_weapon(arm), "binding refused", why);
        } else PSG_WRESET(psg_rifle_seam_weapon(arm));
        psg_rifle_release(); return;
    }
    if (g_psg_rifle.held &&
        (b.actor != g_psg_rifle.binding.actor || b.normal_objs != g_psg_rifle.binding.normal_objs ||
         b.arm != g_psg_rifle.binding.arm || b.hand != g_psg_rifle.binding.hand ||
         b.subject_object != g_psg_rifle.binding.subject_object ||
         b.native_root != g_psg_rifle.saved_root)) {
        psg_rifle_note("owner changed: released, lease again next seam");
        PSG_WLINE(b.weapon, "released", "owner changed, lease again next seam");
        psg_rifle_release(); return;
    }

    if (g_psg_rifle.held && (b.weapon != g_psg_rifle.binding.weapon || b.root != b.hand)) {
        psg_rifle_note("PSG actor rebuilt (weapon or root changed under the lease): released, lease again next seam");
        PSG_WLINE(b.weapon, "released", b.weapon != g_psg_rifle.binding.weapon ? "weapon id changed under the lease"
                                                                                : "root no longer on the hand (actor rebuilt)");
        psg_rifle_release(); return;
    }
    mode = psg_rifle_mode_now(GetTickCount64());
    if (g_psg_overlay.ok && g_psg_overlay.base != base) mode = PSG_MODE_SCOPE;
    __try {
        if (!g_psg_rifle.held) {
            if (b.root != b.native_root) {      /* a root we did not write: leave it */
                g_psg_rifle.refused++; psg_rifle_note("root is neither native nor ours at lease start");
                PSG_WLINE(b.weapon, "lease refused", "root is neither native nor ours at lease start");
                return;
            }
            g_psg_rifle.binding = b; g_psg_rifle.base = base;
            g_psg_rifle.saved_root = b.native_root; g_psg_rifle.cleared = 0;
            g_psg_rifle.mode = PSG_MODE_SCOPE; g_psg_rifle.sight_set = 0;
            *(volatile uint64_t *)(uintptr_t)(b.normal_objs + 0x40) = b.hand;
            g_psg_rifle.held = 1; g_psg_rifle.applied++;
            psg_dummy_lease(&b);                /* psg_v7: the rifle HIP draws */
            psg_probe_rifle_meter_reset();
            psg_rifle_note(g_psg_overlay.ok ? "ok" : "ok, but no sight/warp witness: scope only");
            if (g_b.log) g_b.log("  psg rifle: leased weapon=%d actor=%llx objs=%llx native_root=%llx hand=%llx chanl=%d vflag=%x flags=%x\r\n",
                                 b.weapon, (unsigned long long)b.actor, (unsigned long long)b.normal_objs,
                                 (unsigned long long)b.native_root, (unsigned long long)b.hand,
                                 b.chanl, b.vflag, b.flags);
            PSG_WLINE(b.weapon, "leased", g_psg_rifle.dummy_leased ? "dummy rifle drawn at the hip" :
                                          "no dummy rifle (reason in the psg rifle note)");
        } else PSG_WLINE(b.weapon, "lease held", NULL);
        if (mode != g_psg_rifle.mode) {
            if (mode == PSG_MODE_SCOPE) g_psg_rifle.opens++; else g_psg_rifle.closes++;
            if (g_b.log) g_b.log("  psg gesture: scope %s\r\n", mode == PSG_MODE_SCOPE ? "OPEN" : "closed (hip)");
            g_psg_rifle.mode = mode;
        }
        if (mode == PSG_MODE_HIP) {
            psg_probe_rifle_meter(&b);   /* read only, before the clears below */
            /* Native Act hid the rifle this frame; show it for this submission. */
            if ((unsigned)RD32(b.normal_objs + 0x58) & b.vflag) {
                WR32(b.normal_objs + 0x58, (LONG)((unsigned)RD32(b.normal_objs + 0x58) & ~b.vflag));
                g_psg_rifle.cleared = 1; g_psg_rifle.cleared_frames++;
            }
            psg_models_show(&b);
            psg_dummy_meter(&b);         /* read only, before the dummy write */
            psg_dummy_show(&b);
            psg_sight_hide(base);
            psg_warp_hide(base);
            g_psg_rifle.hip_frames++;
        } else {
            /* Native scope: Act keeps the rifle hidden; give the sight back. */
            psg_dummy_unshow(&b);        /* psg_v7: dummy hidden again (| 0x3000), its root back */
            psg_sight_restore(base);
            g_psg_rifle.cleared = 0;
            g_psg_rifle.scope_frames++;
        }
        InterlockedExchange(&g_psg_hip, mode == PSG_MODE_HIP);
    } __except (EXCEPTION_EXECUTE_HANDLER) {
        g_psg_rifle.held = 0; g_psg_rifle.owner_lost++; InterlockedExchange(&g_psg_hip, 0);
        psg_dummy_unshow_safe(&g_psg_rifle.binding);
        g_psg_rifle.dummy_leased = g_psg_rifle.dummy_shown = 0;
        psg_rifle_note("exception while leasing: dropped");
    }
}

static int dg_psg_barrel_shot(const float world[4][4], float shot[4][4])
{
    static const float muzzle[3] = { 17.5f, -954.0f, 89.7f };
    double dot, det;
    int i, j, k;
    if (!world || !shot) return 0;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) if (!_finite(world[i][j])) return 0;
    for (i = 0; i < 3; i++) {
        if (fabs(world[i][3]) > 1e-3) return 0;
        for (j = 0; j < 3; j++) {
            dot = 0; for (k = 0; k < 3; k++) dot += (double)world[i][k] * world[j][k];
            if (fabs(dot - (i == j ? 1.0 : 0.0)) > 2e-3) return 0;
        }
    }
    det = world[0][0] * (world[1][1] * world[2][2] - world[1][2] * world[2][1])
        - world[0][1] * (world[1][0] * world[2][2] - world[1][2] * world[2][0])
        + world[0][2] * (world[1][0] * world[2][1] - world[1][1] * world[2][0]);
    if (fabs(det - 1.0) > 3e-3 || fabs(world[3][3] - 1.0) > 1e-3) return 0;
    for (i = 0; i < 4; i++) for (j = 0; j < 4; j++) shot[i][j] = world[i][j];
    for (k = 0; k < 3; k++)
        shot[3][k] = world[3][k] + muzzle[0] * world[0][k] + muzzle[1] * world[1][k] + muzzle[2] * world[2][k];
    shot[3][3] = 1.0f;
    return 1;
}

static int psg_rifle_hip_shot(uint64_t actor, float shot[4][4])
{
    float world[4][4];
    if (!psg_rifle_hip_held() || !dg_bridge_psg_hip_now() || g_psg_rifle.binding.actor != actor) return 0;
    if (!psg_rifle_same_owner(&g_psg_rifle.binding) ||
        !interact_read(NULL, g_psg_rifle.binding.normal_objs + 0x110, world, sizeof world)) return 0;
    return dg_psg_barrel_shot(world, shot);
}
