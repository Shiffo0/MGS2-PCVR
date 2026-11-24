static uint64_t g_optics_scope_act,g_optics_scope_end,g_optics_scope_die;
static uint64_t g_optics_native_zoom_act,g_optics_sight_status;
static uint64_t g_optics_body_slot,g_optics_change_camera;
static DG_DETOUR g_optics_scope_detour;
static SRWLOCK g_optics_native_lock=SRWLOCK_INIT;
static struct {
    uint64_t player,work,model,parent,zoom,view,logged_ms,seen_ms;
    unsigned flags,sight,lines;
    int item,view_on,angle_bucket;
} g_optics_native_last;

static void optics_native_witness(uint64_t work,const dg_radial_inventory_snapshot *inv)
{
    uint64_t model=0,parent=0,zoom=0,view=0,act=0,manager_view=0,now;
    unsigned flags=0,sight=0;
    float angle=0,world_origin[3]={0};
    int zoom_slot=inv->actor_item==14?0xe0:0x100;
    int bucket=-1,view_on=0,world_valid,sight_valid;
    if(!g_b.log || inv->actor_item!=inv->desired_item ||
       !interact_read(NULL,work+0x60,&model,8) || !model ||
       !interact_read(NULL,model+0x40,&parent,8) ||
       !interact_read(NULL,model+0x58,&flags,4))return;
    sight_valid=g_optics_sight_status && interact_read(NULL,g_optics_sight_status,&sight,4);
    world_valid=interact_read(NULL,model+0x30,world_origin,sizeof world_origin);
    if(!interact_read(NULL,work+0xc8,&manager_view,8))manager_view=0;
    if(interact_read(NULL,work+zoom_slot,&zoom,8) && zoom &&
       interact_read(NULL,zoom+8,&act,8) && act==g_optics_native_zoom_act &&
       interact_read(NULL,zoom+0x58,&view,8) && view &&
       interact_read(NULL,view+0x2c,&view_on,4) &&
       interact_read(NULL,view+0x20,&angle,4) && angle>=2 && angle<=24)
        bucket=(int)(angle*2);
    now=GetTickCount64();
    if(!TryAcquireSRWLockExclusive(&g_optics_native_lock))return;
    if(g_optics_native_last.lines<256 &&
       (g_optics_native_last.work!=work || g_optics_native_last.item!=inv->actor_item ||
        g_optics_native_last.model!=model || g_optics_native_last.parent!=parent ||
        g_optics_native_last.zoom!=zoom || g_optics_native_last.view!=view ||
        g_optics_native_last.flags!=flags || g_optics_native_last.angle_bucket!=bucket ||
        g_optics_native_last.sight!=sight || g_optics_native_last.view_on!=view_on ||
        now-g_optics_native_last.logged_ms>=1000)) {
        g_optics_native_last.lines++;
        g_optics_native_last.logged_ms=now;
        g_b.log("  optics native: item=%d player=%llx work=%llx model=%llx parent=%llx flags=%x zoom=%llx view=%llx manager_view=%llx view_on=%d angle_valid=%d angle=%.3f sight_valid=%d sight=%x world_valid=%d origin=(%.2f,%.2f,%.2f)\r\n",
            inv->actor_item,inv->player_identity,work,model,parent,flags,zoom,view,manager_view,view_on,
            bucket>=0,(double)angle,sight_valid,sight,world_valid,
            (double)world_origin[0],(double)world_origin[1],(double)world_origin[2]);
    }
    g_optics_native_last.item=inv->actor_item;
    g_optics_native_last.player=inv->player_identity;
    g_optics_native_last.seen_ms=now;
    g_optics_native_last.work=work;g_optics_native_last.model=model;g_optics_native_last.parent=parent;
    g_optics_native_last.zoom=zoom;g_optics_native_last.view=view;g_optics_native_last.flags=flags;
    g_optics_native_last.angle_bucket=bucket;
    g_optics_native_last.sight=sight;g_optics_native_last.view_on=view_on;
    ReleaseSRWLockExclusive(&g_optics_native_lock);
}

static void optics_scope_observe(void *sp)
{
    dg_radial_inventory_snapshot inv;
    uint64_t work,act,die;
    if(!g_optics_scope_act || !camera_inventory(&inv) || inv.actor_item!=14 ||
       !interact_read(NULL,(uint64_t)(ULONG_PTR)sp-24,&work,8) ||
       !interact_read(NULL,work+8,&act,8) || act!=g_optics_scope_act ||
       !interact_read(NULL,work+0x20,&die,8) || die!=g_optics_scope_die)return;
    optics_native_witness(work,&inv);
}

static void optics_scope_resolve(const LiveImage *im)
{
    const DWORD ctor=0x7174b0,act=0x7172d0,model=0x717596,zoom=0x717650,sight_clear=0x717309;
    const unsigned char *p;
    DWORD begin,end;
    g_optics_scope_act=g_optics_scope_end=g_optics_scope_die=g_optics_native_zoom_act=0;
    g_optics_sight_status=0;
    g_optics_body_slot=g_optics_change_camera=0;
    if(!da_code(im,ctor,0x110) || !da_code(im,act,14) || !da_code(im,zoom,16) ||
       !find_runtime_function(im,act,&begin,&end) || begin!=act ||
       !da_eq(im->bytes+act,"\x40\x53\x48\x83\xec\x50\x48\x8b\xd9\xe8\x72\x03\0\0",14) ||
       !da_eq(im->bytes+model,"\xba\x16\x72\x31\0\x48\x89\x83\xc0\0\0\0\xe8",13))return;
    p=im->bytes+0x7174f9;
    if(!da_eq(p,"\x4c\x8d\x05",3) || !da_eq(p+10,"\x48\x8d\x15",3) ||
       da_rel(im,0x717503,7,p+13)!=im->base+act)return;
    g_optics_scope_die=da_rel(im,0x7174f9,7,p+3);
    if(!g_optics_scope_die || !da_code(im,(DWORD)(g_optics_scope_die-im->base),1))return;
    g_optics_scope_act=im->base+act;g_optics_scope_end=im->base+end;
    if(da_code(im,0x57a800,5))g_optics_native_zoom_act=im->base+0x57a800;
    if(da_code(im,0x717334,12) &&
       da_eq(im->bytes+0x717334,"\x48\x8b\x05\x55\xad\xe3\x00\x48\x8b\x53\x60\x0f",12))
        g_optics_body_slot=im->base+0x1552090;
    if(da_code(im,0x4eda49,11) && da_code(im,0x80bb0,8) &&
       da_eq(im->bytes+0x4eda49,"\x8b\x8b\xa4\x00\x00\x00\xe8\x5c\x31\xb9\xff",11))
        g_optics_change_camera=im->base+0x80bb0;
    if(da_code(im,sight_clear,10)) {
        p=im->bytes+sight_clear;
        if(da_eq(p,"\x81\x25",2) && da_eq(p+6,"\x7f\xff\xff\xff",4)) {
            uint64_t slot=da_rel(im,sight_clear,10,p+2);
            if(slot==im->base+0x17b4974)g_optics_sight_status=slot;
        }
    }
}

static void optics_scope_install(void)
{
    const char *why="retail scope witness unavailable";
    if(g_optics_scope_act && dg_detour_install_ex(&g_optics_scope_detour,
         (void*)(ULONG_PTR)g_optics_scope_act,(void*)optics_scope_observe,
         (void*)(ULONG_PTR)g_optics_scope_act,(void*)(ULONG_PTR)g_optics_scope_end,&why,1)){
        if(g_b.log)g_b.log("  optics native: scope item observer installed\r\n");
    } else if(g_b.log)g_b.log("  optics native: scope item observer disabled: %s\r\n",why);
}

static void optics_scope_stop(void)
{
    dg_detour_remove(&g_optics_scope_detour);
    AcquireSRWLockExclusive(&g_optics_native_lock);
    memset(&g_optics_native_last,0,sizeof g_optics_native_last);
    ReleaseSRWLockExclusive(&g_optics_native_lock);
}
