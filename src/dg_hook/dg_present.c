/* dg_present.c - D3D11 capture-only hook for S4a.
 *
 * The IAT is used because it gives us the game's actual device and factory;
 * the COM vtables are then patched per returned object. No executable code
 * is rewritten, and every pointer written by this module is tracked for a
 * conditional, exact restoration during shutdown.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <d3d11.h>
#include <dxgi.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "dg_present.h"

typedef HRESULT (WINAPI *CREATE_FACTORY_FN)(REFIID, void **);
typedef HRESULT (WINAPI *CREATE_DEVICE_FN)(
    IDXGIAdapter *, D3D_DRIVER_TYPE, HMODULE, UINT,
    const D3D_FEATURE_LEVEL *, UINT, UINT, ID3D11Device **,
    D3D_FEATURE_LEVEL *, ID3D11DeviceContext **);
typedef HRESULT (WINAPI *CREATE_DEVICE_SC_FN)(
    IDXGIAdapter *, D3D_DRIVER_TYPE, HMODULE, UINT,
    const D3D_FEATURE_LEVEL *, UINT, UINT, const DXGI_SWAP_CHAIN_DESC *,
    IDXGISwapChain **, ID3D11Device **, D3D_FEATURE_LEVEL *,
    ID3D11DeviceContext **);
typedef HRESULT (STDMETHODCALLTYPE *CREATE_SC_FN)(
    IDXGIFactory *, IUnknown *, DXGI_SWAP_CHAIN_DESC *, IDXGISwapChain **);
typedef HRESULT (STDMETHODCALLTYPE *PRESENT_FN)(IDXGISwapChain *, UINT, UINT);

typedef struct {
    void **slot;
    void *old_value;
    void *our_value;
} PATCH;

static SRWLOCK g_lock = SRWLOCK_INIT;
static volatile LONG g_started;
static volatile LONG g_stopping;
static volatile LONG g_active;
static volatile LONG g_present_count;
static volatile LONG g_error_count;
static HANDLE g_active_event;
/* Deliberately volatile-and-lock-free rather than lock-protected. log_msg is
   called from inside patch_present and patch_factory, which already hold the
   exclusive lock, and an SRW lock is NOT recursive - taking it shared there
   deadlocks the game's swapchain creation outright. Written once at start,
   cleared once at stop, so a plain volatile read is the correct tool. */
static void (*volatile g_log)(const char *fmt, ...);
static void (*volatile g_callback)(IDXGISwapChain *sc);

static PATCH g_iat[3];
static LONG g_iat_count;
static PATCH g_factory_patch;
static PATCH g_present_patch;
static ID3D11Device *g_device;
static IDXGISwapChain *g_swap_chain;
static CREATE_FACTORY_FN g_create_factory;
static CREATE_DEVICE_FN g_create_device;
static CREATE_DEVICE_SC_FN g_create_device_sc;
static CREATE_SC_FN g_create_sc;
static PRESENT_FN g_present;

__declspec(thread) static int g_inside_present;

static void log_msg(const char *fmt, ...) {
    char text[512];
    va_list args;
    void (*log)(const char *fmt_, ...);
    log = g_log;                    /* no lock - see g_log's declaration */
    if (!log) return;
    va_start(args, fmt);
    _vsnprintf_s(text, sizeof(text), _TRUNCATE, fmt, args);
    va_end(args);
    log("%s", text);
}

static const char *format_name(DXGI_FORMAT format) {
    switch (format) {
    case DXGI_FORMAT_R8G8B8A8_UNORM: return "R8G8B8A8_UNORM";
    case DXGI_FORMAT_R8G8B8A8_UNORM_SRGB: return "R8G8B8A8_UNORM_SRGB";
    case DXGI_FORMAT_B8G8R8A8_UNORM: return "B8G8R8A8_UNORM";
    case DXGI_FORMAT_B8G8R8A8_UNORM_SRGB: return "B8G8R8A8_UNORM_SRGB";
    case DXGI_FORMAT_R10G10B10A2_UNORM: return "R10G10B10A2_UNORM";
    case DXGI_FORMAT_R16G16B16A16_FLOAT: return "R16G16B16A16_FLOAT";
    case DXGI_FORMAT_R32G32B32A32_FLOAT: return "R32G32B32A32_FLOAT";
    default: return "UNKNOWN";
    }
}

static void count_error(const char *what) {
    InterlockedIncrement(&g_error_count);
    log_msg("dg_present: %s\r\n", what);
}

static int patch_pointer(PATCH *patch, void **slot, void *replacement) {
    DWORD old_protection, ignored;
    void *current;
    if (!patch || !slot || !replacement) return 0;
    current = *slot;
    if (current == replacement) return 1;
    patch->slot = slot;
    patch->old_value = current;
    patch->our_value = replacement;
    if (!VirtualProtect(slot, sizeof(*slot), PAGE_READWRITE, &old_protection))
        return 0;
    InterlockedExchangePointer((void * volatile *)slot, replacement);
    if (!VirtualProtect(slot, sizeof(*slot), old_protection, &ignored)) {
        /* A failed protection restore is still best-effort repaired below. */
        InterlockedExchangePointer((void * volatile *)slot, current);
        VirtualProtect(slot, sizeof(*slot), old_protection, &ignored);
        patch->slot = NULL;
        return 0;
    }
    FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
    return 1;
}

static int restore_pointer(PATCH *patch) {
    DWORD old_protection, ignored;
    if (!patch || !patch->slot) return 1;
    if (*patch->slot != patch->our_value) {
        log_msg("dg_present: restore skipped; slot was changed by another hook\r\n");
        patch->slot = NULL;
        return 0;
    }
    if (!VirtualProtect(patch->slot, sizeof(*patch->slot), PAGE_READWRITE,
                        &old_protection)) return 0;
    InterlockedExchangePointer((void * volatile *)patch->slot, patch->old_value);
    if (!VirtualProtect(patch->slot, sizeof(*patch->slot), old_protection,
                        &ignored)) return 0;
    FlushInstructionCache(GetCurrentProcess(), patch->slot, sizeof(*patch->slot));
    patch->slot = NULL;
    return 1;
}

static int patch_import(const char *dll, const char *name, void *hook,
                        PATCH *patch) {
    HMODULE module = GetModuleHandleW(NULL);
    BYTE *base;
    IMAGE_DOS_HEADER *dos;
    IMAGE_NT_HEADERS64 *nt;
    IMAGE_DATA_DIRECTORY imports;
    IMAGE_IMPORT_DESCRIPTOR *desc;
    if (!module) return 0;
    base = (BYTE *)module;
    dos = (IMAGE_DOS_HEADER *)base;
    if (dos->e_magic != IMAGE_DOS_SIGNATURE || dos->e_lfanew <= 0) return 0;
    nt = (IMAGE_NT_HEADERS64 *)(base + dos->e_lfanew);
    if (nt->Signature != IMAGE_NT_SIGNATURE ||
        nt->OptionalHeader.Magic != IMAGE_NT_OPTIONAL_HDR64_MAGIC) return 0;
    imports = nt->OptionalHeader.DataDirectory[IMAGE_DIRECTORY_ENTRY_IMPORT];
    if (!imports.VirtualAddress || !imports.Size) return 0;
    desc = (IMAGE_IMPORT_DESCRIPTOR *)(base + imports.VirtualAddress);
    for (; desc->Name; ++desc) {
        IMAGE_THUNK_DATA64 *names, *addresses;
        if (_stricmp((const char *)(base + desc->Name), dll) != 0) continue;
        names = (IMAGE_THUNK_DATA64 *)(base +
            (desc->OriginalFirstThunk ? desc->OriginalFirstThunk : desc->FirstThunk));
        addresses = (IMAGE_THUNK_DATA64 *)(base + desc->FirstThunk);
        for (; names->u1.AddressOfData; ++names, ++addresses) {
            IMAGE_IMPORT_BY_NAME *import;
            if (IMAGE_SNAP_BY_ORDINAL64(names->u1.Ordinal)) continue;
            import = (IMAGE_IMPORT_BY_NAME *)(base + names->u1.AddressOfData);
            if (strcmp((const char *)import->Name, name) != 0) continue;
            return patch_pointer(patch, (void **)&addresses->u1.Function, hook);
        }
    }
    return 0;
}

static void remember_device(ID3D11Device *device) {
    if (!device) return;
    AcquireSRWLockExclusive(&g_lock);
    if (!g_device) { device->lpVtbl->AddRef(device); g_device = device; }
    ReleaseSRWLockExclusive(&g_lock);
}

static void remember_swap_chain(IDXGISwapChain *sc) {
    if (!sc) return;
    AcquireSRWLockExclusive(&g_lock);
    if (!g_swap_chain) { sc->lpVtbl->AddRef(sc); g_swap_chain = sc; }
    ReleaseSRWLockExclusive(&g_lock);
}

static int patch_present(IDXGISwapChain *sc);
static void rollback(void);

static HRESULT WINAPI hook_create_device(
    IDXGIAdapter *adapter, D3D_DRIVER_TYPE driver_type, HMODULE software,
    UINT flags, const D3D_FEATURE_LEVEL *levels, UINT level_count,
    UINT sdk, ID3D11Device **device, D3D_FEATURE_LEVEL *feature,
    ID3D11DeviceContext **context) {
    HRESULT hr = g_create_device(adapter, driver_type, software, flags, levels,
                                 level_count, sdk, device, feature, context);
    if (SUCCEEDED(hr) && device && *device) remember_device(*device);
    return hr;
}

static HRESULT WINAPI hook_create_device_sc(
    IDXGIAdapter *adapter, D3D_DRIVER_TYPE driver_type, HMODULE software,
    UINT flags, const D3D_FEATURE_LEVEL *levels, UINT level_count, UINT sdk,
    const DXGI_SWAP_CHAIN_DESC *desc, IDXGISwapChain **sc,
    ID3D11Device **device, D3D_FEATURE_LEVEL *feature,
    ID3D11DeviceContext **context) {
    HRESULT hr = g_create_device_sc(adapter, driver_type, software, flags,
        levels, level_count, sdk, desc, sc, device, feature, context);
    if (SUCCEEDED(hr) && device && *device) remember_device(*device);
    if (SUCCEEDED(hr) && sc && *sc) {
        remember_swap_chain(*sc);
        if (!patch_present(*sc)) rollback();
    }
    return hr;
}

static HRESULT STDMETHODCALLTYPE hook_create_sc(
    IDXGIFactory *factory, IUnknown *device, DXGI_SWAP_CHAIN_DESC *desc,
    IDXGISwapChain **sc) {
    HRESULT hr = g_create_sc(factory, device, desc, sc);
    if (SUCCEEDED(hr) && sc && *sc) {
        remember_swap_chain(*sc);
        if (!patch_present(*sc)) rollback();
    }
    return hr;
}

static HRESULT STDMETHODCALLTYPE hook_present(IDXGISwapChain *sc,
                                               UINT sync, UINT flags) {
    PRESENT_FN next;
    void (*callback)(IDXGISwapChain *);
    LONG count;
    InterlockedIncrement(&g_active);
    if (g_active_event) ResetEvent(g_active_event);
    next = g_present;
    if (!g_inside_present) {
        g_inside_present = 1;
        count = InterlockedIncrement(&g_present_count);
        if (count == 1) {
            DXGI_SWAP_CHAIN_DESC desc;
            ID3D11Texture2D *back = NULL;
            D3D11_TEXTURE2D_DESC tex;
            int metadata_ok = 0;
            HRESULT hr = sc ? sc->lpVtbl->GetDesc(sc, &desc) : E_POINTER;
            if (SUCCEEDED(hr) && sc &&
                SUCCEEDED(sc->lpVtbl->GetBuffer(sc, 0, &IID_ID3D11Texture2D,
                                                (void **)&back)) && back) {
                memset(&tex, 0, sizeof(tex));
                back->lpVtbl->GetDesc(back, &tex);
                metadata_ok = 1;
                back->lpVtbl->Release(back);
            }
            if (SUCCEEDED(hr) && metadata_ok)
                log_msg("dg_present: first_present format=%s (%u) width=%u height=%u samples=%u buffers=%u\r\n",
                    format_name(tex.Format), (unsigned)tex.Format,
                    tex.Width, tex.Height, tex.SampleDesc.Count,
                    desc.BufferCount);
            else count_error("first Present metadata query failed");
        }
        AcquireSRWLockShared(&g_lock);
        callback = g_callback;
        ReleaseSRWLockShared(&g_lock);
        if (callback) callback(sc);
        g_inside_present = 0;
    } else {
        InterlockedIncrement(&g_error_count);
    }
    if (next) {
        HRESULT hr = next(sc, sync, flags);
        if (InterlockedDecrement(&g_active) == 0 && g_active_event)
            SetEvent(g_active_event);
        return hr;
    }
    if (InterlockedDecrement(&g_active) == 0 && g_active_event)
        SetEvent(g_active_event);
    return DXGI_ERROR_INVALID_CALL;
}

static int patch_present(IDXGISwapChain *sc) {
    void **table;
    void **slot;
    PRESENT_FN current;
    if (!sc) return 0;
    table = *(void ***)sc;
    if (!table) { count_error("swapchain vtable is null"); return 0; }
    slot = table + 8;
    current = (PRESENT_FN)*slot;
    AcquireSRWLockExclusive(&g_lock);
    if (g_present_patch.slot) { ReleaseSRWLockExclusive(&g_lock); return 1; }
    if (current == hook_present) {
        count_error("Present slot is already ours; refusing double-install");
        ReleaseSRWLockExclusive(&g_lock);
        return 0;
    }
    if (current != (PRESENT_FN)g_present)
        log_msg("dg_present: Present slot already differs; chaining existing pointer\r\n");
    if (!patch_pointer(&g_present_patch, slot, (void *)hook_present)) {
        ReleaseSRWLockExclusive(&g_lock);
        count_error("VirtualProtect failed for Present vtable slot");
        return 0;
    }
    g_present = current;
    ReleaseSRWLockExclusive(&g_lock);
    log_msg("dg_present: Present vtable hook installed\r\n");
    return 1;
}

static int patch_factory(IDXGIFactory *factory) {
    void **table;
    if (!factory) return 0;
    table = *(void ***)factory;
    if (!table) return 0;
    AcquireSRWLockExclusive(&g_lock);
    if (g_factory_patch.slot) { ReleaseSRWLockExclusive(&g_lock); return 1; }
    if (!patch_pointer(&g_factory_patch, table + 10, (void *)hook_create_sc)) {
        ReleaseSRWLockExclusive(&g_lock); return 0;
    }
    g_create_sc = (CREATE_SC_FN)g_factory_patch.old_value;
    ReleaseSRWLockExclusive(&g_lock);
    return 1;
}

static HRESULT WINAPI hook_create_factory(REFIID iid, void **factory) {
    HRESULT hr = g_create_factory(iid, factory);
    if (SUCCEEDED(hr) && factory && *factory &&
        !patch_factory((IDXGIFactory *)*factory)) {
        count_error("factory vtable hook failed");
        rollback();
    }
    return hr;
}

static void rollback(void) {
    int i;
    AcquireSRWLockExclusive(&g_lock);
    restore_pointer(&g_present_patch);
    restore_pointer(&g_factory_patch);
    for (i = g_iat_count - 1; i >= 0; --i) restore_pointer(&g_iat[i]);
    g_iat_count = 0;
    ReleaseSRWLockExclusive(&g_lock);
}

int dg_present_start(void (*log)(const char *fmt, ...)) {
    int ok = 0, have_device = 0, have_factory = 0;
    AcquireSRWLockExclusive(&g_lock);
    if (g_started) { ReleaseSRWLockExclusive(&g_lock); return 1; }
    g_log = log;
    g_stopping = 0;
    g_active_event = CreateEventA(NULL, TRUE, TRUE, NULL);
    if (!g_active_event) { ReleaseSRWLockExclusive(&g_lock); count_error("CreateEvent failed"); return 0; }
    /* A game reaches D3D11 by ONE of these two entry points, never both, so
       a missing import is the normal case rather than a failure. MGS2 imports
       D3D11CreateDevice alone. Demanding all three is what made the first
       attempt roll back the two hooks that had already taken.
       The factory hook is the one that is genuinely required: the swapchain,
       and therefore Present, is only reachable through it on this path. */
    if (patch_import("d3d11.dll", "D3D11CreateDevice",
                     (void *)hook_create_device, &g_iat[g_iat_count])) {
        g_create_device = (CREATE_DEVICE_FN)g_iat[g_iat_count].old_value;
        g_iat_count++; have_device = 1;
    }
    if (patch_import("d3d11.dll", "D3D11CreateDeviceAndSwapChain",
                     (void *)hook_create_device_sc, &g_iat[g_iat_count])) {
        g_create_device_sc = (CREATE_DEVICE_SC_FN)g_iat[g_iat_count].old_value;
        g_iat_count++; have_device = 1;
    }
    if (patch_import("dxgi.dll", "CreateDXGIFactory",
                     (void *)hook_create_factory, &g_iat[g_iat_count])) {
        g_create_factory = (CREATE_FACTORY_FN)g_iat[g_iat_count].old_value;
        g_iat_count++; have_factory = 1;
    }
    ReleaseSRWLockExclusive(&g_lock);
    /* Logged unconditionally: when this fails on someone else's build, which
       of the two numbers is zero is the entire diagnosis. */
    log_msg("dg_present: imports hooked - device %d factory %d\r\n",
            have_device, have_factory);
    AcquireSRWLockExclusive(&g_lock);
    if (!have_device || !have_factory) goto fail;
    g_started = 1;
    ReleaseSRWLockExclusive(&g_lock);
    log_msg("dg_present: started\r\n");
    return 1;
fail:
    ReleaseSRWLockExclusive(&g_lock);
    rollback();
    CloseHandle(g_active_event); g_active_event = NULL;
    count_error("IAT hook install failed; rolled back all hooks");
    return ok;
}

void dg_present_stop(void) {
    HANDLE event;
    AcquireSRWLockExclusive(&g_lock);
    if (!g_started && !g_iat_count && !g_present_patch.slot && !g_factory_patch.slot) {
        ReleaseSRWLockExclusive(&g_lock); return;
    }
    g_stopping = 1;
    event = g_active_event;
    restore_pointer(&g_present_patch);
    restore_pointer(&g_factory_patch);
    while (g_iat_count > 0) { --g_iat_count; restore_pointer(&g_iat[g_iat_count]); }
    g_started = 0;
    ReleaseSRWLockExclusive(&g_lock);
    if (event) { while (InterlockedCompareExchange(&g_active, 0, 0) != 0) WaitForSingleObject(event, 50); }
    AcquireSRWLockExclusive(&g_lock);
    if (g_device) { g_device->lpVtbl->Release(g_device); g_device = NULL; }
    if (g_swap_chain) { g_swap_chain->lpVtbl->Release(g_swap_chain); g_swap_chain = NULL; }
    g_present = NULL; g_create_sc = NULL; g_log = NULL; g_callback = NULL;
    event = g_active_event; g_active_event = NULL;
    ReleaseSRWLockExclusive(&g_lock);
    if (event) CloseHandle(event);
}

int dg_present_get_device(ID3D11Device **dev, IDXGISwapChain **sc) {
    int result = 0;
    if (dev) *dev = NULL;
    if (sc) *sc = NULL;
    AcquireSRWLockShared(&g_lock);
    if (dev && g_device) { g_device->lpVtbl->AddRef(g_device); *dev = g_device; result = 1; }
    if (sc && g_swap_chain) { g_swap_chain->lpVtbl->AddRef(g_swap_chain); *sc = g_swap_chain; result = 1; }
    ReleaseSRWLockShared(&g_lock);
    return result;
}

void dg_present_set_callback(void (*on_present)(IDXGISwapChain *sc)) {
    AcquireSRWLockExclusive(&g_lock); g_callback = on_present; ReleaseSRWLockExclusive(&g_lock);
}

void dg_present_stats(long *presents, long *errors) {
    if (presents) *presents = InterlockedCompareExchange(&g_present_count, 0, 0);
    if (errors) *errors = InterlockedCompareExchange(&g_error_count, 0, 0);
}
