/* Optional general player observer. Reuses the inventory resolver's proved
 * player slot/field offsets, never its catalog/equip policy. */
static dg_radial_inventory_image g_interact_image;
static dg_radial_inventory_anchors g_interact_player;
static int interact_read(void *ctx,uint64_t address,void *dst,size_t n) {
    MEMORY_BASIC_INFORMATION mbi;
    uint64_t cursor=address,end;
    (void)ctx;
    if (!address || !n || n>UINT64_MAX-address) return 0;
    end=address+n;
    while (cursor<end) {
        uint64_t next;
        if (!dg_vq((const void *)(ULONG_PTR)cursor,&mbi) ||
            mbi.State!=MEM_COMMIT || (mbi.Protect&(PAGE_NOACCESS|PAGE_GUARD)) ||
            !(mbi.Protect&(PAGE_READONLY|PAGE_READWRITE|PAGE_WRITECOPY|
              PAGE_EXECUTE_READ|PAGE_EXECUTE_READWRITE|PAGE_EXECUTE_WRITECOPY))) return 0;
        next=(uint64_t)(ULONG_PTR)mbi.BaseAddress+mbi.RegionSize;
        if (next<=cursor) return 0;
        cursor=next;
    }
    __try { memcpy(dst,(const void *)(ULONG_PTR)address,n); }
    __except(EXCEPTION_EXECUTE_HANDLER) { return 0; }
    return 1;
}
static void interact_resolve_player(const LiveImage *image) {
    memset(&g_interact_image,0,sizeof g_interact_image);
    memset(&g_interact_player,0,sizeof g_interact_player);
    g_interact_image.base=image->base;g_interact_image.size=image->size;
    g_interact_image.approved_identity=1; /* caller passed executable fingerprint */
    g_interact_image.read=interact_read;
    dg_radial_inventory_resolve(&g_interact_image,&g_interact_player);
    g_interact_player.valid_bits&=DG_RINV_ACTOR;
}
static int interact_player_now(uint64_t *identity,int *weapon) {
    dg_radial_inventory_snapshot sample;
    *identity=0;*weapon=-1;
    if (!(dg_radial_inventory_sample(&g_interact_image,&g_interact_player,&sample)&DG_RINV_ACTOR) ||
        sample.actor_weapon<0 || sample.actor_weapon>=DG_WEAPON_COUNT) return 0;
    *identity=sample.player_identity;*weapon=sample.actor_weapon;
    return 1;
}
