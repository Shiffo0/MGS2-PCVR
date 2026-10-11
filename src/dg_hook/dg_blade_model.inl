/* Startup-only geometry choice for the two subject OBJECTs. Keep their
 * actor slots, ONEPIECE flags, root, visibility, targeting and animations.
 * Full models are already loaded by this same constructor for body/mirror.
 * No asset replacement, shared model mutation or per-frame pointer swap. */
static const unsigned k_blade_model_site[2]={0x4c3599,0x4c3637};
static const unsigned char k_blade_model_old[2][5]={
    {0xba,0xce,0xb2,0xeb,0}, {0xba,0x38,0x24,0x14,0}};
static const unsigned char k_blade_model_new[2][5]={
    {0xba,0xea,0x8a,0x92,0}, {0xba,0x7d,0xa1,0x19,0}};
static uint64_t g_blade_model_base;
static unsigned g_blade_model_mask;
static int (*g_blade_model_write)(unsigned char *,const unsigned char *,unsigned)=patch_code;
static void blade_model_resolve(const LiveImage *im) {
    if(g_blade_model_mask)return;
    g_blade_model_base=0;

    if(!m9_match(im,0x4c3592,(const unsigned char *)
       "\x48\x8d\xbd\xe0\0\0\0\xba\xce\xb2\xeb\0\x48\x8b\xcf\x41\xb8\x32\0\0\0\xe8\xc4\xa9\x1b\0",26) ||
       !m9_match(im,0x4c3630,(const unsigned char *)
       "\x4c\x8d\xa5\x20\x01\0\0\xba\x38\x24\x14\0\x41\xb8\x32\0\0\0\x49\x8b\xcc\x81\x48\x58\0\x30\0\0\xe8\x1f\xa9\x1b\0",33))return;
    g_blade_model_base=im->base;
}
static int blade_model_restore(void) {
    int i;
    for(i=1;i>=0;i--)if(g_blade_model_mask&(1u<<i)) {
        unsigned char *at=(unsigned char *)(uintptr_t)(g_blade_model_base+k_blade_model_site[i]);
        /* Never overwrite another owner's change. Retain failed ownership. */
        if(!memcmp(at,k_blade_model_new[i],5) && g_blade_model_write(at,k_blade_model_old[i],5))
            g_blade_model_mask&=~(1u<<i);
    }
    return g_blade_model_mask==0;
}
static int blade_model_install(void) {
    int i;
    if(!g_blade_model_base || g_blade_model_mask)return 0;
    for(i=0;i<2;i++)if(memcmp((void *)(uintptr_t)(g_blade_model_base+k_blade_model_site[i]),k_blade_model_old[i],5))return 0;
    for(i=0;i<2;i++) {
        unsigned char *at=(unsigned char *)(uintptr_t)(g_blade_model_base+k_blade_model_site[i]);
        if(!g_blade_model_write(at,k_blade_model_new[i],5)) {
            blade_model_restore();
            return 0;
        }
        g_blade_model_mask|=1u<<i;
    }
    return 1;
}
