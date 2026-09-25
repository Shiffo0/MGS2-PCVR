#ifndef DG_VQ_CACHE_H
#define DG_VQ_CACHE_H
/* VirtualQuery cost scales with the size of the region: the kernel walks the
   region's page table entries (MiQueryAddressState/MiGetPageProtection) to find
   where its attributes end. The game heap is made of large regions, and
   region_end/interact_read ask once per read. ETW 25 Sep 2026 (stereo, 1080p):
   82% of the game thread's CPU samples were in those two kernel functions -
   the "waiting in the kernel" that kept VR stereo below 60 fps.

   Contract (vq_cache_test.c exercises each point with real VirtualAlloc):
   - Positive answers (committed, readable/accessible, not guard) are cached per
     thread for at most DG_VQ_TTL_MS. Negative answers are never cached.
   - Inside the TTL a cached answer can be stale in two directions: a region that
     grew still reports its old, shorter end (callers refuse or query again past
     it - interact_read continues with a real query at the cached end), and a
     region that was decommitted or reprotected still reads as accessible.
     Game heap regions are not remapped at that timescale; interact_read keeps
     its SEH copy for the rest.
   - The existing bridge suites (DG_HOOK_TEST) re-map memory at the same
     addresses within microseconds, which no cache can model; they run with the
     cache disabled. Define DG_VQ_CACHE_ENABLED to override. */
#include <windows.h>
#include <string.h>

#ifndef DG_VQ_CACHE_ENABLED



#    define DG_VQ_CACHE_ENABLED 1

#endif
#define DG_VQ_SLOTS 16
#define DG_VQ_TTL_MS 20

static volatile LONG g_vq_hits, g_vq_misses;

#if DG_VQ_CACHE_ENABLED
typedef struct { ULONG_PTR base, end, alloc; DWORD protect, type; ULONGLONG t; } DG_VQ_ENTRY;
static __declspec(thread) DG_VQ_ENTRY t_vq[DG_VQ_SLOTS];
static __declspec(thread) unsigned t_vq_next;
static ULONGLONG (WINAPI *dg_vq_clock)(void) = GetTickCount64; /* tests may replace */

#else

#endif

static SIZE_T dg_vq(const void *p, MEMORY_BASIC_INFORMATION *mbi)
{
#if DG_VQ_CACHE_ENABLED
    ULONG_PTR a = (ULONG_PTR)p;
    ULONGLONG now = dg_vq_clock();
    unsigned i;
    for (i = 0; i < DG_VQ_SLOTS; i++) {
        const DG_VQ_ENTRY *e = &t_vq[i];
        if (e->end && a >= e->base && a < e->end && now >= e->t && now - e->t <= DG_VQ_TTL_MS) {
            memset(mbi, 0, sizeof *mbi);
            mbi->BaseAddress = (PVOID)e->base;
            mbi->AllocationBase = (PVOID)e->alloc;
            mbi->RegionSize = e->end - e->base;
            mbi->State = MEM_COMMIT;
            mbi->Protect = e->protect;
            mbi->Type = e->type;
            InterlockedIncrement(&g_vq_hits);
            return sizeof *mbi;
        }
    }
    InterlockedIncrement(&g_vq_misses);
    if (!VirtualQuery(p, mbi, sizeof *mbi)) return 0;
    if (mbi->State == MEM_COMMIT && !(mbi->Protect & (PAGE_GUARD | PAGE_NOACCESS)) &&
        mbi->RegionSize) {
        DG_VQ_ENTRY *e = &t_vq[t_vq_next++ % DG_VQ_SLOTS];
        e->base = (ULONG_PTR)mbi->BaseAddress;
        e->end = e->base + mbi->RegionSize;
        e->alloc = (ULONG_PTR)mbi->AllocationBase;
        e->protect = mbi->Protect;
        e->type = mbi->Type;
        e->t = now;
    }
    return sizeof *mbi;
#else
    InterlockedIncrement(&g_vq_misses);
    return VirtualQuery(p, mbi, sizeof *mbi);
#endif
}
#endif
