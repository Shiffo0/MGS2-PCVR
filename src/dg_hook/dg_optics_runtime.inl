#include "dg_optics_world.h"

typedef struct {
    uint64_t player,work,model,native_root,zoom,view;
    int item,hidden_model,parked;
} DG_OPTICS_LEASE;

static volatile LONG g_optics_enabled;
static SRWLOCK g_optics_runtime_lock=SRWLOCK_INIT;
static DG_OPTICS_LEASE g_optics_lease;
static DG_OPTICS_MODEL_GESTURE g_optics_gesture;
static __declspec(align(16)) float g_optics_world[4][4];
static struct { uint64_t work,view,zoom,ms; int item,open,direction; } g_optics_action;
static DG_DETOUR g_optics_layout_scope,g_optics_layout_dg,g_optics_layout_tanker;
static uint64_t g_optics_layout_base;
static DWORD g_optics_layout_end[3];
static int g_optics_layout_live;

static int optics_configured(void)
{
    return InterlockedCompareExchange(&g_optics_enabled,0,0)!=0;
}

int dg_bridge_optics_grip_owned(int item)
{
    return optics_configured() && dg_optics_grip_item(item);
}

static void optics_model_visibility(uint64_t model,int hidden)
{
    LONG *flags=(LONG*)(uintptr_t)(model+0x58);
    if(hidden)InterlockedOr(flags,0x1000);
    else InterlockedAnd(flags,~0x1000);
}

/* Fail closed for mod input during owner changes, menus or stale tracking. */
static int optics_input_now(int item,uint64_t work,uint64_t zoom,int *direction)
{
    uint64_t now=GetTickCount64();
    int open=0;
    *direction=0;
    if(TryAcquireSRWLockShared(&g_optics_runtime_lock)){
        if(g_optics_action.item==item && (!work || work==g_optics_action.work) &&
           (!zoom || zoom==g_optics_action.zoom) && g_optics_action.ms &&
           now>=g_optics_action.ms && now-g_optics_action.ms<=100 &&
           !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0)){
            open=g_optics_action.open;
            *direction=g_optics_action.direction;
        }
        ReleaseSRWLockShared(&g_optics_runtime_lock);
    }
    return open;
}

static int optics_binding_live(const DG_OPTICS_LEASE *lease,int require_desired)
{
    dg_radial_inventory_snapshot inv;
    uint64_t act,die,body_owner,body,model,zoom,view,check;
    int channel;
    uint64_t expected_act=lease->item==14?g_optics_scope_act:g_camera_a.callback;
    uint64_t expected_die=lease->item==14?g_optics_scope_die:g_camera_a.destroy;
    unsigned zoom_slot=lease->item==14?0xe0:0x100;
    return lease->work && g_optics_body_slot && expected_act && expected_die &&
        camera_inventory(&inv) && inv.player_identity==lease->player &&
        inv.actor_item==lease->item && (!require_desired || inv.desired_item==lease->item) &&
        interact_read(NULL,lease->work+8,&act,8) && act==expected_act &&
        interact_read(NULL,lease->work+0x20,&die,8) && die==expected_die &&
        interact_read(NULL,g_optics_body_slot,&body_owner,8) && body_owner &&
        interact_read(NULL,body_owner,&body,8) && body && body+0xa10==lease->native_root &&
        interact_read(NULL,lease->work+0x60,&model,8) && model==lease->model &&
        interact_read(NULL,lease->work+zoom_slot,&zoom,8) && zoom==lease->zoom &&
        interact_read(NULL,zoom+8,&check,8) && check==g_optics_native_zoom_act &&
        interact_read(NULL,zoom+0x58,&view,8) && view==lease->view &&
        interact_read(NULL,lease->work+0xc8,&check,8) && check==view &&
        interact_read(NULL,view+0xa4,&channel,4) && channel==0;
}

static int optics_scope_zoom_now(uint64_t expected,uint64_t *work,uint64_t *view)
{
    uint64_t now=GetTickCount64();
    int open=0;
    if(TryAcquireSRWLockShared(&g_optics_runtime_lock)){
        if(g_optics_action.item==14 && g_optics_action.open &&
           (!expected || expected==g_optics_action.zoom) && g_optics_action.ms &&
           now>=g_optics_action.ms && now-g_optics_action.ms<=100 &&
           !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0) &&
           optics_binding_live(&g_optics_lease,1)){
            *work=g_optics_action.work;*view=g_optics_action.view;open=1;
        }
        ReleaseSRWLockShared(&g_optics_runtime_lock);
    }
    return open;
}

int dg_bridge_optics_zoom_now(uint64_t *identity,float *angle)
{
    uint64_t now=GetTickCount64();
    int ok=0,on=0;
    if(!optics_configured() || !TryAcquireSRWLockShared(&g_optics_runtime_lock))return 0;
    if(g_optics_action.open && g_optics_action.ms && now>=g_optics_action.ms &&
       now-g_optics_action.ms<=100 && !g_b.s_late_unsafe &&
       optics_binding_live(&g_optics_lease,1) &&
       interact_read(NULL,g_optics_action.view+0x2c,&on,4) && on==1 &&
       interact_read(NULL,g_optics_action.view+0x20,angle,4) && *angle>=2 && *angle<=24){
        *identity=g_optics_action.work;
        ok=1;
    }
    ReleaseSRWLockShared(&g_optics_runtime_lock);
    return ok;
}

static int optics_locomotion(uint64_t player,int *parked)
{
    uint64_t now=GetTickCount64();
    int owned=0;
    if(!TryAcquireSRWLockShared(&g_optics_runtime_lock))return 0;
    if(g_optics_action.ms && now>=g_optics_action.ms && now-g_optics_action.ms<=100 &&
       g_optics_lease.player==player && optics_binding_live(&g_optics_lease,1)){
        *parked=optics_configured() && !g_optics_action.open;
        owned=1;
    }
    ReleaseSRWLockShared(&g_optics_runtime_lock);
    return owned;
}

static int optics_camera_on(uint64_t view,int expected,int wanted)
{
    LONG *flag=(LONG*)(uintptr_t)(view+0x2c);
    if(!g_optics_change_camera || InterlockedCompareExchange(flag,wanted,expected)!=expected)return 0;
    ((void (*)(int))(uintptr_t)g_optics_change_camera)(0);
    return 1;
}

/* A replaced owner uses its freshly constructed native parent and view.
   Restoration is conditional on the original actor/view still owning them. */
static void optics_release_locked(void)
{
    DG_OPTICS_LEASE *lease=&g_optics_lease;
    uint64_t root;
    float native[4][4];
    memset(&g_optics_action,0,sizeof g_optics_action);
    memset(&g_optics_gesture,0,sizeof g_optics_gesture);
    if(optics_binding_live(lease,0)){
        __try {
            if(interact_read(NULL,lease->model+0x40,&root,8) &&
               root==(uint64_t)(uintptr_t)g_optics_world){
                optics_model_visibility(lease->model,dg_optics_model_hidden(0,0,lease->hidden_model));
                InterlockedCompareExchangePointer((void**)(uintptr_t)(lease->model+0x40),
                    (void*)(uintptr_t)lease->native_root,g_optics_world);
                if(interact_read(NULL,lease->native_root,native,sizeof native))
                    memcpy((void*)(uintptr_t)lease->model,native,sizeof native);
            }
            if(lease->parked)optics_camera_on(lease->view,0,1);
        } __except(EXCEPTION_EXECUTE_HANDLER) {}
    }
    memset(lease,0,sizeof *lease);
}

static void optics_release(void)
{
    AcquireSRWLockExclusive(&g_optics_runtime_lock);
    optics_release_locked();
    ReleaseSRWLockExclusive(&g_optics_runtime_lock);
}

static DG_OPTICS_LEASE optics_observed_lease(void)
{
    DG_OPTICS_LEASE lease={0};
    uint64_t now=GetTickCount64(),parent,body_owner,body;
    if(!g_optics_body_slot || !TryAcquireSRWLockShared(&g_optics_native_lock))return lease;
    if(g_optics_native_last.seen_ms && now>=g_optics_native_last.seen_ms &&
       now-g_optics_native_last.seen_ms<=100 && g_optics_native_last.angle_bucket>=0){
        lease=(DG_OPTICS_LEASE){g_optics_native_last.player,g_optics_native_last.work,
            g_optics_native_last.model,0,g_optics_native_last.zoom,g_optics_native_last.view,
            g_optics_native_last.item,0,0};
    }
    ReleaseSRWLockShared(&g_optics_native_lock);
    if(!lease.model ||
       !interact_read(NULL,g_optics_body_slot,&body_owner,8) || !body_owner ||
       !interact_read(NULL,body_owner,&body,8) || !body ||
       !interact_read(NULL,lease.model+0x40,&parent,8) ||
       (parent!=body+0xa10 && parent!=(uint64_t)(uintptr_t)g_optics_world))
        return (DG_OPTICS_LEASE){0};
    lease.native_root=body+0xa10;
    return lease;
}

static int optics_arm_frame(const DG_BRIDGE_ARM_TARGET *target,const double palm[3])
{
    static const DG_PSG_GESTURE_CFG eye_box={.105,.145,38,55};
    DG_OPTICS_MODEL_POSE pose,tracking_pose;
    DG_OPTICS_LEASE next;
    DG_OPTICS_LEASE *lease=&g_optics_lease;
    float matrix[4][4];
    uint64_t root;
    int visible=0,open,grip,was_open,was_leased,view_on;
    unsigned flags;
    if(!optics_configured() && !lease->work)return 0;
    AcquireSRWLockExclusive(&g_optics_runtime_lock);
    if(!optics_configured() || !target || !palm || !target->optics.valid || !g_optics_change_camera ||
       !g_optics_layout_live ||
       !g_b.armed || g_b.owner || g_b.script_menu_only || g_b.s_late_unsafe ||
       g_b.fps.state!=DG_FPS_ACTIVE)goto reset;
    next=optics_observed_lease();
    if(!optics_binding_live(&next,1) ||
       !dg_optics_palm_pose(next.item,palm,target->optics.grip_q,
            1.0/target->left_position.units,&pose) ||
       /* Avatar reach and animation must not set the real player's eye box.
          The draw pose is separately anchored to the measured EVM palm. */
       !dg_optics_palm_pose(next.item,target->optics.grip_m,target->optics.grip_q,
            1.0/target->left_position.units,&tracking_pose) ||
       !dg_optics_world_matrix(&pose,&target->left_position,matrix))goto reset;
    if(lease->work!=next.work || lease->model!=next.model || lease->view!=next.view ||
       lease->player!=next.player || lease->item!=next.item)optics_release_locked();
    if(!interact_read(NULL,next.model+0x40,&root,8) ||
       !interact_read(NULL,next.model+0x58,&flags,4))goto reset;
    was_leased=lease->work!=0;
    if(!was_leased){
        if(root!=next.native_root || !interact_read(NULL,next.view+0x2c,&view_on,4) || view_on!=1)
            goto reset;
        next.hidden_model=(flags&0x1000)!=0;
        *lease=next;
    }
    was_open=g_optics_action.open;
    open=dg_optics_model_step(&g_optics_gesture,lease->work,lease->item,1,&eye_box,
        target->optics.eyes,target->optics.head_q,&tracking_pose);
    grip=dg_optics_grip_zoom(&g_optics_gesture.gesture,target->optics.left,target->optics.right);
    __try {
        memcpy(g_optics_world,matrix,sizeof matrix);
        if(root==(uint64_t)(uintptr_t)g_optics_world ||
           (root==lease->native_root &&
            InterlockedCompareExchangePointer((void**)(uintptr_t)(lease->model+0x40),
                g_optics_world,(void*)(uintptr_t)lease->native_root)==(void*)(uintptr_t)root)){
            memcpy((void*)(uintptr_t)lease->model,matrix,sizeof matrix);
            optics_model_visibility(lease->model,dg_optics_model_hidden(1,open,lease->hidden_model));
            if(open && lease->parked){
                optics_camera_on(lease->view,0,1);
                lease->parked=0;
            } else if(!open && optics_camera_on(lease->view,1,0))lease->parked=1;
            visible=1;
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) {}
    if(!visible)goto reset;
    g_optics_action.work=lease->work;g_optics_action.view=lease->view;
    g_optics_action.zoom=lease->zoom;g_optics_action.item=lease->item;
    g_optics_action.open=open;g_optics_action.direction=grip;
    g_optics_action.ms=GetTickCount64();
    if(g_b.log && (!was_leased || was_open!=open)){
        double eyes[2][3],points[2][3],best=1e6;
        int count=dg_optics_model_oculars(lease->item,&tracking_pose,points);
        memcpy(eyes,target->optics.eyes,sizeof eyes);
        for(int eye=0;eye<2;eye++)
            for(int ocular=0;ocular<count;ocular++){
                double d=0;
                for(int i=0;i<3;i++){double x=eyes[eye][i]-points[ocular][i];d+=x*x;}
                if(d<best)best=d;
            }
        g_b.log("  optics behavior: item=%d model=%llx open=%d eye=%d ocular=%d root=%llx palm=(%.4f,%.4f,%.4f) grip=(%.4f,%.4f,%.4f) aim=(%.4f,%.4f,%.4f,%.4f) nearest_mm=%.1f\r\n",
            lease->item,lease->model,open,open?g_optics_gesture.eye:-1,
            open?g_optics_gesture.ocular:-1,(uint64_t)(uintptr_t)g_optics_world,
            palm[0],palm[1],palm[2],target->optics.grip_m[0],target->optics.grip_m[1],
            target->optics.grip_m[2],pose.optical_q[0],pose.optical_q[1],
            pose.optical_q[2],pose.optical_q[3],1000*sqrt(best));
    }
    ReleaseSRWLockExclusive(&g_optics_runtime_lock);
    return visible;
reset:
    optics_release_locked();
    ReleaseSRWLockExclusive(&g_optics_runtime_lock);
    return 0;
}

static void optics_layout_hide(void *sp,int item)
{
    uint64_t now=GetTickCount64();
    if(!optics_configured() || !TryAcquireSRWLockShared(&g_optics_runtime_lock))return;
    if(g_optics_action.item==item && !g_optics_action.open &&
       g_optics_action.ms && now>=g_optics_action.ms && now-g_optics_action.ms<=100 &&
       !InterlockedCompareExchange(&g_b.s_late_unsafe,0,0) &&
       optics_binding_live(&g_optics_lease,1))
        ((uint64_t*)sp)[-1]&=~0x40ull;
    ReleaseSRWLockShared(&g_optics_runtime_lock);
}
static void optics_layout_scope(void *sp){optics_layout_hide(sp,14);}
static void optics_layout_dg(void *sp){optics_layout_hide(sp,15);}
static void optics_layout_tanker(void *sp){optics_layout_hide(sp,21);}

static void optics_layout_resolve(const LiveImage *im)
{
    static const DWORD act[3]={0x4df7f0,0x505400,0x4a3e20};
    static const unsigned char tail[3]={0x48,0x3d,0x44};
    DWORD begin,end;
    g_optics_layout_base=0;
    for(int i=0;i<3;i++){
        DWORD at=act[i];
        const unsigned char *p=im->bytes+at;
        if(!da_code(im,at,26) || !find_runtime_function(im,at,&begin,&end) || begin!=at ||
           !da_eq(p,"\x40\x53\x48\x83\xec\x20\xf6\x05",8) ||
           da_rel(im,at+6,7,p+8)!=g_optics_sight_status ||
           p[12]!=(i?0x01:0x80) ||
           !da_eq(p+13,"\x48\x8b\xd9\x8b\x49\x58\xba\x34\x46\x2a\0\x74",12) ||
           p[25]!=tail[i])return;
        g_optics_layout_end[i]=end;
    }
    g_optics_layout_base=im->base;
}

static void optics_layout_stop(void)
{
    g_optics_layout_live=0;
    optics_release();
    dg_detour_remove(&g_optics_layout_scope);
    dg_detour_remove(&g_optics_layout_dg);
    dg_detour_remove(&g_optics_layout_tanker);
}

static void optics_layout_install(void)
{
    const char *why="layout witnesses unavailable";
    uint64_t b=g_optics_layout_base;
    if(b &&
       dg_detour_install_ex(&g_optics_layout_scope,(void*)(uintptr_t)(b+0x4df7f6),
         optics_layout_scope,(void*)(uintptr_t)(b+0x4df7f0),(void*)(uintptr_t)(b+g_optics_layout_end[0]),&why,4) &&
       dg_detour_install_ex(&g_optics_layout_dg,(void*)(uintptr_t)(b+0x505406),
         optics_layout_dg,(void*)(uintptr_t)(b+0x505400),(void*)(uintptr_t)(b+g_optics_layout_end[1]),&why,4) &&
       dg_detour_install_ex(&g_optics_layout_tanker,(void*)(uintptr_t)(b+0x4a3e26),
         optics_layout_tanker,(void*)(uintptr_t)(b+0x4a3e20),(void*)(uintptr_t)(b+g_optics_layout_end[2]),&why,4))
        g_optics_layout_live=1;
    else optics_layout_stop();
    if(g_b.log)g_b.log("  optics behavior: native layout routes %s\r\n",g_optics_layout_live?"installed":why);
}
