/* Fission Mailed. The scripted fake game-over screen is its own actor: it
 * shrinks the running game into a corner under a Game Over panel while play
 * continues, and sets no game or menu status bit. In VR that composite would
 * fill the eye, so its three visual layers are never created (the game
 * tests every one for NULL): play simply continues in stereo. Its per-frame
 * Act still marks it live for the log. */
#include <intrin.h>   /* _ReturnAddress */
typedef void (__fastcall *DG_FAKE_GO_FN)(void *);
static DG_DETOUR g_fake_go_detour;
static volatile LONG g_fake_go_ms, g_fake_go_live, g_fake_go_frames;
static uint64_t g_fake_go_base;
/* The three layers, refused only for the fake game-over's resource setup. */
typedef void *(__fastcall *DG_FAKE_GO_CTOR)(uint64_t,uint64_t,uint64_t,uint64_t);
static DG_DETOUR g_fake_go_layer[3];
static volatile LONG g_fake_go_suppressed[3];
static const uint64_t k_fake_go_layer_rva[3]={0x5aea90,0x6e8290,0x77850};
static const uint64_t k_fake_go_layer_end[3]={0x5aeb34,0x6e83b7,0x77939};
static const uint64_t k_fake_go_layer_ret[3]={0x59d226,0x59d254,0x59d27f};
static const char *const k_fake_go_layer_name[3]={"screen reduction","panel","text"};
static void *fake_go_layer(int i,uint64_t a,uint64_t b,uint64_t c,uint64_t d,void *ret) {
    DG_FAKE_GO_CTOR original=(DG_FAKE_GO_CTOR)g_fake_go_layer[i].tramp;
    if(g_fake_go_base && (uint64_t)(uintptr_t)ret==g_fake_go_base+k_fake_go_layer_ret[i]) {
        if(InterlockedIncrement(&g_fake_go_suppressed[i])<=8 && g_b.log)
            g_b.log("  fission mailed: %s not created (play continues in VR)\r\n",k_fake_go_layer_name[i]);
        return NULL;
    }
    return original?original(a,b,c,d):NULL;
}
static void *__fastcall fake_go_reduce(uint64_t a,uint64_t b,uint64_t c,uint64_t d){return fake_go_layer(0,a,b,c,d,_ReturnAddress());}
static void *__fastcall fake_go_prim(uint64_t a,uint64_t b,uint64_t c,uint64_t d){return fake_go_layer(1,a,b,c,d,_ReturnAddress());}
static void *__fastcall fake_go_print(uint64_t a,uint64_t b,uint64_t c,uint64_t d){return fake_go_layer(2,a,b,c,d,_ReturnAddress());}
#define DG_FAKE_GO_FRESH_MS 250
static void __fastcall fake_go_act(void *work) {
    DG_FAKE_GO_FN original=(DG_FAKE_GO_FN)g_fake_go_detour.tramp;
    InterlockedExchange(&g_fake_go_ms,(LONG)GetTickCount());
    InterlockedIncrement(&g_fake_go_frames);
    if(original)original(work);
}
static void fake_go_resolve(const LiveImage *im) {
    g_fake_go_base=0;


    if(!m9_match(im,0x59c8c0,(const unsigned char *)"\x40\x53\x48\x81\xec\x80\0\0\0\x48\x8b\x05",12) ||
       !m9_match(im,0x59d636,(const unsigned char *)"\x4c\x8d\x05\x83\xfa\xff\xff",7) ||
       !m9_match(im,0x59d640,(const unsigned char *)"\x48\x8d\x15\x79\xf2\xff\xff",7) ||
       !m9_match(im,0x59d0e7,(const unsigned char *)"\xbf\x6b\x48\x92\x00",5) ||
       /* the three layer constructors and their calls in its resource setup */
       !m9_match(im,0x59d221,(const unsigned char *)"\xe8\x6a\x18\x01\x00",5) ||
       !m9_match(im,0x59d24f,(const unsigned char *)"\xe8\x3c\xb0\x14\x00",5) ||
       !m9_match(im,0x59d27a,(const unsigned char *)"\xe8\xd1\xa5\xad\xff",5) ||
       !m9_match(im,0x5aea90,(const unsigned char *)"\x48\x89\x5c\x24\x08\x48\x89\x74\x24\x10\x57\x48",12) ||
       !m9_match(im,0x6e8290,(const unsigned char *)"\x48\x89\x5c\x24\x10\x48\x89\x6c\x24\x18\x56\x57",12) ||
       !m9_match(im,0x77850,(const unsigned char *)"\x48\x89\x5c\x24\x08\x48\x89\x6c\x24\x10\x48\x89",12))return;
    g_fake_go_base=im->base;
}
static void fake_go_install(void) {
    const char *why="retail witnesses unavailable";
    if(g_fake_go_base && dg_detour_install_ex(&g_fake_go_detour,
       (void *)(uintptr_t)(g_fake_go_base+0x59c8c0),(void *)fake_go_act,
       (void *)(uintptr_t)(g_fake_go_base+0x59c8c0),
       (void *)(uintptr_t)(g_fake_go_base+0x59cb95),&why,2))
        InterlockedExchange(&g_fake_go_live,1);
    if(g_fake_go_base) {
        static void *const cb[3]={(void *)fake_go_reduce,(void *)fake_go_prim,(void *)fake_go_print};
        int i,layers=0;
        for(i=0;i<3;i++) {
            const char *lwhy="?";
            uint64_t at=g_fake_go_base+k_fake_go_layer_rva[i];
            if(dg_detour_install_ex(&g_fake_go_layer[i],(void *)(uintptr_t)at,cb[i],(void *)(uintptr_t)at,
               (void *)(uintptr_t)(g_fake_go_base+k_fake_go_layer_end[i]),&lwhy,2))layers++;
            else if(g_b.log)g_b.log("  fission mailed: %s hook unavailable (%s)\r\n",k_fake_go_layer_name[i],lwhy);
        }
        /* All or nothing: a partly hidden screen would be worse than none. */
        if(layers!=3)for(i=0;i<3;i++)dg_detour_remove(&g_fake_go_layer[i]);
        if(g_b.log)g_b.log("  fission mailed: overlay %s\r\n",layers==3?"suppressed (stereo play continues)":"left native");
    }
    if(g_b.log)g_b.log("  fission mailed: %s (%s)\r\n",
        InterlockedCompareExchange(&g_fake_go_live,0,0)?"watch installed":"unavailable",why);
}
static void fake_go_stop(void) {
    int i;
    InterlockedExchange(&g_fake_go_live,0);
    for(i=0;i<3;i++)dg_detour_remove(&g_fake_go_layer[i]);
    dg_detour_remove(&g_fake_go_detour);
}
int dg_bridge_fake_gameover_now(void) {
    return InterlockedCompareExchange(&g_fake_go_live,0,0) &&
        (DWORD)(GetTickCount()-(DWORD)InterlockedCompareExchange(&g_fake_go_ms,0,0))<=DG_FAKE_GO_FRESH_MS;
}
long dg_bridge_fake_gameover_frames(void) {
    return InterlockedCompareExchange(&g_fake_go_frames,0,0);
}
