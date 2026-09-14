/* Retail fullscreen previous-frame blur only, not rain or other blur passes.
 * DR1 fires BEFORE shl ebx,24 in Blur::Act. Zero the effective alpha register,
 * never the actor's intensity/timer. The native instruction builds the packet
 * and naturally restores original behavior on the next unsuppressed visit.
 */
#ifndef DG_SCENE_BLUR_H
#define DG_SCENE_BLUR_H

#define DG_SCENE_BLUR_RVA 0x2BB6DCull

static int dg_scene_blur_signature(const unsigned char *load,
                                    const unsigned char *colour,
                                    const unsigned char *page,
                                    const unsigned char *store)
{
    static const unsigned char a[] = {0xf3,0x48,0x0f,0x2c,0x5f,0x78};
    static const unsigned char b[] = {0xc1,0xe3,0x18,0x0f,0x57,0xc9,
                                      0x81,0xcb,0x80,0x80,0x80,0x00};
    static const unsigned char c[] = {0xba,1,0,0,0,0x48,0x8b,0xc8,
                                      0xe8,0xf0,0x9b,0xdd,0xff};
    static const unsigned char d[] = {0x89,0x5c,0x24,0x48};
    return !memcmp(load,a,sizeof a) && !memcmp(colour,b,sizeof b) &&
           !memcmp(page,c,sizeof c) && !memcmp(store,d,sizeof d);
}

static int dg_scene_blur_allowed(int armed, int real_xr, int stereo,
                                 int enabled, int theater, int menu, int flags)
{
    return armed && real_xr && stereo && enabled && !theater && !menu &&
           (flags & 3) == 3;
}

static int dg_scene_blur_owns(const CONTEXT *c, ULONG64 address)
{
    return address && c->Rip == address && (c->Dr6 & 15) == 2;
}

/* Used by the shipping VEH and tests. Does not claim a foreign/debugger trap. */
static int dg_scene_blur_context(CONTEXT *c, ULONG64 address, int suppress)
{
    if (!dg_scene_blur_owns(c,address)) return 0;
    if (suppress) c->Rbx = 0;
    c->Dr6 &= ~2ull;
    c->EFlags |= 0x10000; /* resume once, no repeated execution fault */
    return 1;
}
#endif
