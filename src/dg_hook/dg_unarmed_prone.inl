

#include <intrin.h>
static const DWORD unarmed_prone_standing[2]={0x9801f0,0x9802c0};
static const DWORD unarmed_prone_ground[2]={0x980530,0x9805e0};
static const float unarmed_prone_witness[4][8]={
    {0,500,240,0,3,550,301,0},
    {0,451,208,0,20,533,336,0},
    {0,0,0,0,52,115,558,0},
    {0,0,0,0,85,122,549,0}
};
static struct {
    ULONGLONG base;
    int resolved,owned[2];
    float before[2][4],written[2][4];
} g_unarmed_prone;

static int unarmed_prone_exchange(ULONGLONG address,const float before[4],
                                  const float after[4])
{
    __declspec(align(16)) LONG64 expected[2];
    LONG64 desired[2];MEMORY_BASIC_INFORMATION mbi;
    if((address&15) || !VirtualQuery((void *)(ULONG_PTR)address,&mbi,sizeof mbi) ||
       mbi.State!=MEM_COMMIT || (mbi.Protect&(PAGE_GUARD|PAGE_NOACCESS)) ||
       !(mbi.Protect&(PAGE_READWRITE|PAGE_WRITECOPY|PAGE_EXECUTE_READWRITE|PAGE_EXECUTE_WRITECOPY)) ||
       (ULONGLONG)(ULONG_PTR)mbi.BaseAddress+mbi.RegionSize<address+16)return 0;
    memcpy(expected,before,16);memcpy(desired,after,16);
    /* Native actor reads one aligned vector. Publish all four floats in one
       atomic operation; never expose half a Snake/Raiden placement. */
    return _InterlockedCompareExchange128((volatile LONG64 *)(ULONG_PTR)address,
                                         desired[1],desired[0],expected)!=0;
}
static void unarmed_prone_stop(void)
{
    int i;
    for(i=0;i<2;i++)if(g_unarmed_prone.owned[i]) {
        unarmed_prone_exchange(g_unarmed_prone.base+unarmed_prone_ground[i],
                              g_unarmed_prone.written[i],g_unarmed_prone.before[i]);
        g_unarmed_prone.owned[i]=0; /* foreign values are never overwritten */
    }
}
static void unarmed_prone_resolve(const LiveImage *im)
{
    int i;
    memset(&g_unarmed_prone,0,sizeof g_unarmed_prone);
    if(!im || !im->bytes || !im->valid || !g_b.a.arm_cam_rotate_shift)return;
    for(i=0;i<4;i++) {
        DWORD rva=i<2?unarmed_prone_standing[i]:unarmed_prone_ground[i-2];
        if(!all_valid(im->valid,rva,32,im->size) ||
           memcmp(im->bytes+rva,unarmed_prone_witness[i],32))return;
    }
    g_unarmed_prone.base=im->base;g_unarmed_prone.resolved=1;
    for(i=0;i<2;i++) {
        memcpy(g_unarmed_prone.before[i],im->bytes+unarmed_prone_ground[i],16);
        memcpy(g_unarmed_prone.written[i],im->bytes+unarmed_prone_standing[i],16);
    }
}
static void unarmed_prone_install(int enabled)
{
    int i;
    if(!enabled || !g_unarmed_prone.resolved)return;
    for(i=0;i<2;i++) {
        if(!unarmed_prone_exchange(g_unarmed_prone.base+unarmed_prone_ground[i],
                                  g_unarmed_prone.before[i],g_unarmed_prone.written[i]))break;
        g_unarmed_prone.owned[i]=1;
    }
    if(i!=2)unarmed_prone_stop();
    if(g_b.log)g_b.log("  unarmed prone: per-character native arm placement %s\r\n",
        i==2?"installed":"refused (table ownership changed)");
}
