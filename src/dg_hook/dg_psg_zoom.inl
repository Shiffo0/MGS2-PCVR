/* Reuse the native shared zoom actor, but prove PSG ownership independently
   from the photo-camera manager. No pad/camera writes here. */
/* Zoom parent: every native zoom constructor call (photo camera 0x4ee1ac,
   PSG 0x71355e, 0x7176ac) passes rdx=[0x17df6a0], stored at zoom+0x60
   (0x57adbc) - the same global the photo route checks through
   g_zoom_a.parent_slot. It is not the arm camera of the camera gate; that
   comparison refused every live PSG frame (25 Sep 2026: 440 and 1928). The
   constructor also writes its camera back to *(actor+0x108) (0x57ae52), which
   is why zoom+0x58 == scope holds. */
static int psg_zoom_observe(uint64_t expected,uint64_t *owner,uint64_t *cam,unsigned *om,unsigned *imask) {



    uint64_t actor,player,scope,parent,zoom,fn,pad,zc,zp;int weapon,ok=0,oi,ii;
    unsigned pause,flags,pm,sm;
    if((!g_zoom_a.valid) || (!g_zoom_detour.installed) || (!TryAcquireSRWLockShared(&g_controls_lock)))return 0;
    __try {
        if(psg_context(0,&actor,&player,&scope,&parent,&weapon)) {
            zoom=*(uint64_t *)(uintptr_t)(actor+0x110);
            if(!(!((!expected || zoom==expected))) &&
               !(!(interact_read(NULL,zoom+8,&fn,8)))&&!(!(fn==g_zoom_a.callback)) &&
               !(!(interact_read(NULL,zoom+0x68,&pad,8)))&&!(!(pad==g_camera_a.pad)) &&
               !(!(interact_read(NULL,zoom+0x60,&zp,8)))&&!(!(zp==*(volatile uint64_t *)(uintptr_t)(g_psg.base+0x17df6a0))) &&
               !(!(interact_read(NULL,zoom+0x58,&zc,8)))&&!(!(zc==scope)) &&
               !(!(interact_read(NULL,g_camera_a.pause,&pause,4)))&&!(!(!pause)) &&
               !(!(interact_read(NULL,g_zoom_a.out_mask,om,4)))&&!(!(*om))&&!(!(!(*om&(*om-1)))) &&
               !(!(interact_read(NULL,g_zoom_a.in_mask,imask,4)))&&!(!(*imask))&&!(!(!(*imask&(*imask-1))))&&!(!(*om!=*imask)) &&
               !(!(interact_read(NULL,g_zoom_a.out_index,&oi,4)))&&!(!(oi>=0))&&!(!(oi<12)) &&
               !(!(interact_read(NULL,g_zoom_a.in_index,&ii,4)))&&!(!(ii>=0))&&!(!(ii<12))&&!(!(oi!=ii)) &&
               !(!(interact_read(NULL,pad+36,&flags,4)))&&!(!(!(flags&(0x103|0x1000|0x4000|0x30)))) &&
               !(!(interact_read(NULL,g_action_a.mask_prg,&pm,4)))&&
               !(!(interact_read(NULL,g_action_a.mask_scn,&sm,4)))&&
               !(!((!(flags&4)||((pm&(*om|*imask))==(*om|*imask))))) &&
               !(!((!(flags&8)||((sm&(*om|*imask))==(*om|*imask)))))) {
                *owner=zoom;*cam=scope;ok=1;
            }
        }
    } __except(EXCEPTION_EXECUTE_HANDLER){ok=0;}
    ReleaseSRWLockShared(&g_controls_lock);return ok;
}
int dg_bridge_psg_scope_active(void) {
    uint64_t actor,player,scope,cam;int weapon,ok=0;
    /* Called by the controls provider inside controls_begin's shared lock.
       Never reacquire an SRW lock recursively (a queued writer can block it). */
    __try {ok=psg_context(0,&actor,&player,&scope,&cam,&weapon);}
    __except(EXCEPTION_EXECUTE_HANDLER){ok=0;}
    return ok;
}
int dg_bridge_psg_zoom_now(uint64_t *identity,float *angle) {
    uint64_t owner,cam;unsigned om,im;
    if(psg_zoom_observe(0,&owner,&cam,&om,&im) &&
       interact_read(NULL,cam+0x20,angle,4)&&*angle>=8 && *angle<=60) {
        *identity=owner;return 1;
    }
    return 0;
}
