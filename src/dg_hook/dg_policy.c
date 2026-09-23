/* dg_policy.c - the resident half of the hot-reload boundary: dispatcher,
 * staged loader, and the 1 Hz poll.
 *
 * Threading model, and the whole safety argument in four sentences: the
 * WORKER thread is the only one that loads, validates and OFFERS tables (an
 * interlocked write of g_pending). The TICK seam is the only place an offer
 * becomes ACTIVE (dg_policy_tick_adopt), so a swap lands between ticks,
 * never inside one. Every pass over policy code - one bridge_tick, one VEH
 * camera-seam visit - snapshots the active node into TLS at pass entry and
 * calls through that snapshot, so a pass on EITHER thread is all-old or
 * all-new even though the VEH thread races the tick thread. And a module
 * whose table was ever offered is never FreeLibrary'd, so a call that is
 * still executing inside superseded code always has its pages.
 *
 * Fail closed: every refusal leaves whatever was active - the statically
 * linked builtin at worst - untouched, and is counted and logged.
 */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <string.h>

#include "dg_policy.h"

/* One node per accepted table. Nodes are allocated once and never freed -
   they are what a TLS snapshot points at, and proving no thread holds one
   would need the rundown machinery this design deliberately avoids. */
typedef struct DG_POLICY_NODE {
    const DG_POLICY_TABLE *tab;
    long generation;                 /* 0 = builtin */
    HMODULE mod;                     /* NULL for builtin and test tables */
} DG_POLICY_NODE;

static DG_POLICY_NODE g_builtin_node = { &dg_policy_builtin, 0, NULL };

static DG_POLICY_NODE *volatile g_active = &g_builtin_node;
static DG_POLICY_NODE *volatile g_pending;       /* NULL = nothing offered */

static volatile LONG g_staged;
static volatile LONG g_adopted;
static volatile LONG g_refused;
static volatile LONG g_tls_misses;
static volatile LONG g_generation;               /* last generation handed out */

static DWORD g_tls = TLS_OUT_OF_INDEXES;
static volatile LONG g_init_done;

/* Worker-only; set once before any poll. */
static char g_forbidden_dir[MAX_PATH];
static void (*g_log)(const char *fmt, ...);
static long g_stage_seq;

static void ensure_init(void)
{
    /* TlsAlloc exactly once, from whichever entry point runs first. The
       0->1 winner allocates; everyone else spins the handful of cycles the
       winner needs. 2 means done. */
    LONG was = InterlockedCompareExchange(&g_init_done, 1, 0);
    if (was == 2) return;
    if (was == 0) {
        g_tls = TlsAlloc();          /* may fail; dg_policy() degrades */
        InterlockedExchange(&g_init_done, 2);
        return;
    }
    while (InterlockedCompareExchange(&g_init_done, 2, 2) != 2) { /* spin */ }
}

static DG_POLICY_NODE *active_node(void)
{
    return (DG_POLICY_NODE *)InterlockedCompareExchangePointer(
        (PVOID volatile *)&g_active, NULL, NULL);
}

void dg_policy_setup(const char *forbidden_dir,
                     void (*log)(const char *fmt, ...))
{
    ensure_init();
    g_forbidden_dir[0] = 0;
    if (forbidden_dir && forbidden_dir[0])
        strcpy_s(g_forbidden_dir, sizeof(g_forbidden_dir), forbidden_dir);
    g_log = log;
}

const DG_POLICY_TABLE *dg_policy(void)
{
    DG_POLICY_NODE *n = NULL;
    ensure_init();
    if (g_tls != TLS_OUT_OF_INDEXES)
        n = (DG_POLICY_NODE *)TlsGetValue(g_tls);
    if (!n) {
        /* No open pass on this thread: a worker one-off, a desk test, or a
           TLS allocation failure. Counted, and read once - the caller gets
           ONE coherent table for this single call. */
        InterlockedIncrement(&g_tls_misses);
        n = active_node();
    }
    return n->tab;
}

void *dg_policy_pass_begin(void)
{
    void *prev;
    ensure_init();
    if (g_tls == TLS_OUT_OF_INDEXES) return NULL;
    prev = TlsGetValue(g_tls);
    TlsSetValue(g_tls, active_node());
    return prev;
}

void dg_policy_pass_end(void *token)
{
    if (g_tls == TLS_OUT_OF_INDEXES) return;
    TlsSetValue(g_tls, token);
}

void dg_policy_tick_adopt(void)
{
    DG_POLICY_NODE *n;
    ensure_init();
    n = (DG_POLICY_NODE *)InterlockedExchangePointer(
        (PVOID volatile *)&g_pending, NULL);
    if (n) {
        InterlockedExchangePointer((PVOID volatile *)&g_active, n);
        InterlockedIncrement(&g_adopted);
    }
}

void dg_policy_status(DG_POLICY_STATUS *out)
{
    DG_POLICY_NODE *n = active_node();
    out->generation = n->generation;
    out->staged = InterlockedCompareExchange(&g_staged, 0, 0);
    out->adopted = InterlockedCompareExchange(&g_adopted, 0, 0);
    out->refused = InterlockedCompareExchange(&g_refused, 0, 0);
    out->tls_misses = InterlockedCompareExchange(&g_tls_misses, 0, 0);
    out->active_is_builtin = (n == &g_builtin_node);
}

/* --------------------------------------------------------- validation --- */

/* The X-list of every slot, so "check all 28 for NULL" cannot silently miss
   the one added last. */
#define DG_POLICY_FN_LIST(X) \
    X(pose_cfg_default) X(pose_init) X(pose_step) X(pose_quat_blend) \
    X(ik_solve) X(ik_swing_quat) X(ik_orient) X(ik_quat_mul) \
    X(ik_quat_conj) X(ik_quat_normalize) X(ik_basis_quat) X(ik_quat_angle) \
    X(ik_hand_adjust) X(ik_hand_stabilize) X(ik_quat_to_ps_angles) \
    X(ik_ps_precompensate) \
    X(arm_map_reset) X(arm_map_step) \
    X(fire_reset) X(fire_pressure) X(fire_step) \
    X(recoil_reset) X(recoil_fire) X(recoil_step) X(recoil_amplitude) \
    X(recoil_climb_quat) X(recoil_pull_back) \
    X(move_step)

int dg_policy_validate(const DG_POLICY_TABLE *t, const char **why)
{
    /* Expected sizes, expanded against the .asi's OWN headers at ITS build
       time - the other half of the stamp the DLL carries. */
    static const DG_POLICY_SIZES expect = DG_POLICY_SIZES_INIT;

    if (!t) { *why = "query returned NULL"; return 0; }
    if (t->magic != DG_POLICY_MAGIC) { *why = "bad magic"; return 0; }
    if (t->abi_version != DG_POLICY_ABI_VERSION) {
        *why = "ABI version mismatch";
        return 0;
    }
    if (t->table_bytes != (unsigned int)sizeof(DG_POLICY_TABLE)) {
        *why = "table size mismatch";
        return 0;
    }
    if (memcmp(&t->sizes, &expect, sizeof(expect)) != 0) {
        *why = "boundary struct size mismatch";
        return 0;
    }
#define DG_POLICY_CHECK_NULL(name) \
    if (!t->name) { *why = "NULL slot: " #name; return 0; }
    DG_POLICY_FN_LIST(DG_POLICY_CHECK_NULL)
#undef DG_POLICY_CHECK_NULL
    *why = "ok";
    return 1;
}

/* ------------------------------------------------------------- loader --- */

static int offer_node(const DG_POLICY_TABLE *tab, long generation,
                      HMODULE mod)
{
    DG_POLICY_NODE *n = (DG_POLICY_NODE *)HeapAlloc(
        GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(*n));
    if (!n) return 0;
    n->tab = tab;
    n->generation = generation;
    n->mod = mod;
    /* If a previous offer was never adopted it is simply superseded; its
       node and module stay allocated, by the same never-free rule. */
    InterlockedExchangePointer((PVOID volatile *)&g_pending, n);
    InterlockedIncrement(&g_staged);
    return 1;
}

static int path_inside(const char *dir, const char *full)
{
    size_t n = strlen(dir);
    if (n == 0) return 0;
    if (_strnicmp(full, dir, n) != 0) return 0;
    return full[n] == '\\' || full[n] == '/' || full[n] == 0;
}

long dg_policy_load_from(const char *path, const char **why)
{
    char full[MAX_PATH], staged[MAX_PATH], tmpdir[MAX_PATH];
    HMODULE mod;
    const DG_POLICY_TABLE *(*query)(void);
    const DG_POLICY_TABLE *tab;
    long gen;
    DWORD n;

    ensure_init();
    *why = "ok";

    if (!path || !path[0]) {
        *why = "empty path";
        InterlockedIncrement(&g_refused);
        return 0;
    }
    if (!GetFullPathNameA(path, sizeof(full), full, NULL)) {
        *why = "path does not resolve";
        InterlockedIncrement(&g_refused);
        return 0;
    }
    /* The code-level backstop of the never-touch-game-files rule: a policy
       DLL inside the game's own directory is refused before the file is so
       much as opened. The DLL belongs in the dev tree. */
    if (path_inside(g_forbidden_dir, full)) {
        *why = "path is inside the game directory";
        InterlockedIncrement(&g_refused);
        return 0;
    }

    n = GetTempPathA(sizeof(tmpdir), tmpdir);
    if (n == 0 || n >= sizeof(tmpdir)) {
        *why = "no temp directory";
        InterlockedIncrement(&g_refused);
        return 0;
    }
    _snprintf_s(staged, sizeof(staged), _TRUNCATE, "%sdg_policy_%lu_%ld.dll",
                tmpdir, (unsigned long)GetCurrentProcessId(), ++g_stage_seq);
    if (!CopyFileA(full, staged, FALSE)) {
        *why = "copy to staging failed (source missing or unreadable)";
        InterlockedIncrement(&g_refused);
        return 0;
    }

    mod = LoadLibraryExA(staged, NULL, 0);
    if (!mod) {
        *why = "LoadLibrary failed on staged copy";
        InterlockedIncrement(&g_refused);
        return 0;
    }
    query = (const DG_POLICY_TABLE *(*)(void))(void *)GetProcAddress(
        mod, DG_POLICY_EXPORT_NAME);
    if (!query) {
        *why = "export " DG_POLICY_EXPORT_NAME " missing";
        /* Refused before its table could escape, so this free is provably
           safe - the one and only place a policy module is ever freed. */
        FreeLibrary(mod);
        InterlockedIncrement(&g_refused);
        return 0;
    }
    tab = query();
    if (!dg_policy_validate(tab, why)) {
        FreeLibrary(mod);                        /* same argument as above */
        InterlockedIncrement(&g_refused);
        return 0;
    }

    gen = InterlockedIncrement(&g_generation);
    if (!offer_node(tab, gen, mod)) {
        *why = "node allocation failed";
        /* The table pointer never left this frame, so the module may still
           be freed - but a heap this exhausted has bigger problems; keep it
           loaded rather than add an untested path. */
        InterlockedIncrement(&g_refused);
        return 0;
    }
    return gen;
}

/* --------------------------------------------------------------- poll --- */

static int read_small_file(const char *path, char *buf, size_t bufsz)
{
    HANDLE h;
    DWORD got = 0;
    buf[0] = 0;
    h = CreateFileA(path, GENERIC_READ,
                    FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                    NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) return 0;
    ReadFile(h, buf, (DWORD)(bufsz - 1), &got, NULL);
    CloseHandle(h);
    buf[got] = 0;
    return 1;
}

void dg_policy_poll(const char *dll_path)
{
    /* Worker-only statics: the poll runs on one thread, once a second, the
       same discipline as the marker poll it rides along with. */
    static char last_path[MAX_PATH];
    static char last_ver[128];
    static long adopts_logged;
    static int said_builtin;
    long adopts_now;

    ensure_init();

    if (!dll_path || !dll_path[0]) {
        /* No override wanted. If one is active or pending, put the builtin
           back - through the same offer/adopt gate as any other swap. */
        if (last_path[0]) {
            last_path[0] = 0;
            last_ver[0] = 0;
        }
        if (!active_node()->generation) {
            /* already builtin; nothing pending to displace matters */
        } else if (!said_builtin) {
            offer_node(&dg_policy_builtin, 0, NULL);
            said_builtin = 1;
            if (g_log)
                g_log("  policy: vr_policy cleared - builtin policy returns"
                      " at next tick\r\n");
        }
    } else {
        char ver_path[MAX_PATH + 8], ver[128];
        int trigger;

        said_builtin = 0;
        _snprintf_s(ver_path, sizeof(ver_path), _TRUNCATE, "%s.ver",
                    dll_path);
        read_small_file(ver_path, ver, sizeof(ver));

        /* A reload fires on a CHANGE - of the configured path, or of the
           version file's content, which build_policy.bat rewrites only
           after the DLL is fully linked. Same content, same path: no I/O
           beyond the one small read, and certainly no LoadLibrary storm. */
        trigger = (strcmp(dll_path, last_path) != 0) ||
                  (strcmp(ver, last_ver) != 0);
        if (trigger) {
            const char *why;
            long gen;
            strcpy_s(last_path, sizeof(last_path), dll_path);
            strcpy_s(last_ver, sizeof(last_ver), ver);
            gen = dg_policy_load_from(dll_path, &why);
            if (gen) {
                if (g_log)
                    g_log("  policy: staged gen %ld from %s (ver '%s') -"
                          " adopts at next tick\r\n",
                          gen, dll_path, ver);
            } else {
                DG_POLICY_STATUS st;
                dg_policy_status(&st);
                if (g_log)
                    g_log("  policy: REFUSED %s - %s; %s stays active"
                          " (refused %ld)\r\n",
                          dll_path, why,
                          st.active_is_builtin ? "builtin"
                                               : "previous DLL policy",
                          st.refused);
            }
        }
    }

    /* Adoption happens on the tick seam, which must not write logs; the
       poll reports it from here, one line per adoption. */
    adopts_now = InterlockedCompareExchange(&g_adopted, 0, 0);
    if (adopts_now != adopts_logged) {
        DG_POLICY_NODE *n = active_node();
        adopts_logged = adopts_now;
        if (g_log) {
            if (n->generation)
                g_log("  policy: gen %ld ACTIVE (adopted on the tick seam;"
                      " state carried over)\r\n", n->generation);
            else
                g_log("  policy: builtin policy ACTIVE again\r\n");
        }
    }
}

/* ---------------------------------------------------------- desk tests -- */




























































































































































































































































































































































































































