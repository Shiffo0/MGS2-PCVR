


static struct {uint64_t base;int resolved,live;DG_DETOUR detour;} g_nikita_steer;
static SRWLOCK g_nikita_steer_lock=SRWLOCK_INIT;
static struct {DG_NIKITA_STEER_INPUT input;uint64_t actor,stamp;int valid;} g_nikita_command;
static void nikita_steer_revoke(void) {
    AcquireSRWLockExclusive(&g_nikita_steer_lock);
    memset(&g_nikita_command,0,sizeof g_nikita_command);
    ReleaseSRWLockExclusive(&g_nikita_steer_lock);
}
static int nikita_steer_actor(uint64_t expected,uint64_t *actor_out,uint64_t *pad_out) {



    uint64_t b=g_nikita_steer.base,a,fn,pad,ctrl,player,now=GetTickCount64();
    int alive,channel,state,weapon;unsigned flags;
    if((!g_nikita_steer.live) || (!g_b.armed) || (g_b.owner) || (g_b.script_menu_only) ||
       (g_b.s_late_unsafe) || ((!g_b.fps.desired && g_b.fps.state!=DG_FPS_ACTIVE)) ||
       (!g_controls_allowed) || (g_controls_special) || (now<g_controls_lease) || (now-g_controls_lease>100) ||
       (dg_bridge_radial_paused()) || (!dg_bridge_controller_gameplay_now()) ||
       (!interact_player_now(&player,&weapon)) || (weapon!=6) ||
       (!interact_read(NULL,b+0x16e8950,&a,8)) || (!a) || ((expected && expected!=a)) ||
       (!interact_read(NULL,a+8,&fn,8)) || (fn!=b+0x4ea050) ||
       (!interact_read(NULL,a+0x5a0,&channel,4)) || (channel!=0) ||
       (!interact_read(NULL,b+0x17df7e4,&alive,4)) || (alive!=1) ||
       (!interact_read(NULL,a+0x5a8,&state,4)) || (state!=0) ||
       (!interact_read(NULL,a+0x5ac,&flags,4)) || ((flags&0xd8u)) ||
       (!interact_read(NULL,a+0x590,&ctrl,8)) || (ctrl!=player+0x60) ||
       (!interact_read(NULL,a+0x598,&pad,8)) || (pad!=b+0x167d710))return 0;
    *actor_out=a;*pad_out=pad;return 1;
}
static void nikita_steer_tick(int safe) {
    DG_NIKITA_STEER_INPUT in=g_controls_frame.nikita_steer;
    uint64_t actor=0,pad=0,stamp=0;
    int valid=safe && g_controls_active && in.valid && in.allowed && in.sequence &&
        in.age_ms<=100 && in.now_ms>=in.age_ms && _finite(in.x) && fabs(in.x)<=1.01 &&
        nikita_steer_actor(0,&actor,&pad);
    if(valid)stamp=in.now_ms-in.age_ms;







    AcquireSRWLockExclusive(&g_nikita_steer_lock);
    if(!valid)memset(&g_nikita_command,0,sizeof g_nikita_command);
    else if(g_nikita_command.valid && g_nikita_command.actor==actor &&
        in.source==g_nikita_command.input.source && in.context==g_nikita_command.input.context &&
        in.sequence<=g_nikita_command.input.sequence) {
        if(in.sequence<g_nikita_command.input.sequence || in.x!=g_nikita_command.input.x)
            memset(&g_nikita_command,0,sizeof g_nikita_command);
        /* Duplicate frames cannot renew the timestamp. */
    } else {
        g_nikita_command.input=in;g_nikita_command.actor=actor;
        g_nikita_command.stamp=stamp;g_nikita_command.valid=1;
    }
    ReleaseSRWLockExclusive(&g_nikita_steer_lock);
}
static int nikita_steer_pad(uint64_t expected,unsigned char local[40],uint64_t *original_pad) {
    uint64_t actor,pad,now=GetTickCount64();double x=0,dz,mag;int valid=0,dx;
    unsigned status;unsigned short analog;
    if(!TryAcquireSRWLockShared(&g_controls_lock))return 0;
    if(nikita_steer_actor(expected,&actor,&pad)) {
        AcquireSRWLockShared(&g_nikita_steer_lock);
        valid=g_nikita_command.valid && g_nikita_command.actor==actor &&
            now>=g_nikita_command.stamp && now-g_nikita_command.stamp<=100;
        if(valid)x=g_nikita_command.input.x;
        ReleaseSRWLockShared(&g_nikita_steer_lock);
        if(valid)valid=interact_read(NULL,pad,local,40);
    }
    ReleaseSRWLockShared(&g_controls_lock);
    if(!valid)return 0;
    memcpy(&status,local+4,4);memcpy(&analog,local+0x12,2);
    /* Physical gamepad horizontal input retains ownership. */
    if((status&0xa000u) || ((analog&4) && (local[0x16]<96 || local[0x16]>160)))return 0;
    dz=(double)g_b.move_deadzone_mils/1000.0;if(dz<0)dz=0;if(dz>.9)dz=.9;
    mag=fabs(x);if(mag<=dz)return 0;if(mag>1)mag=1;
    dx=49+(int)floor((mag-dz)/(1-dz)*78+.5);
    local[0x16]=(unsigned char)(128+(x<0?-dx:dx));
    status|=x<0?0x8000u:0x2000u;analog|=4;
    memcpy(local+4,&status,4);memcpy(local+0x12,&analog,2);
    *original_pad=pad;return 1;
}
static void nikita_operation(void *actor) {
    typedef void (*NATIVE)(void *);NATIVE original=(NATIVE)g_nikita_steer.detour.tramp;
    union {uint64_t align;unsigned char bytes[40];} local;
    uint64_t pad,a=(uint64_t)(uintptr_t)actor;int supplied;
    if(!original)return;
    supplied=nikita_steer_pad(a,local.bytes,&pad);



    if(!supplied){original(actor);return;}
    *(uint64_t *)(uintptr_t)(a+0x598)=(uint64_t)(uintptr_t)local.bytes;
    __try {original(actor);}
    __finally {
        /* The witnessed Operation never destroys its actor. Do not clobber
           an unexpected native replacement of the pointer. */
        if(*(uint64_t *)(uintptr_t)(a+0x598)==(uint64_t)(uintptr_t)local.bytes)
            *(uint64_t *)(uintptr_t)(a+0x598)=pad;
    }
}
static void nikita_steer_resolve(const LiveImage *im) {
    memset(&g_nikita_steer,0,sizeof g_nikita_steer);
    if(!m9_match(im,0x4ebd50,(const unsigned char *)"\x48\x89\x5c\x24\x20\x57\x48\x83\xec\x40",10) ||
       !m9_match(im,0x4ebcfd,(const unsigned char *)"\x48\x89\x1d\x4c\xcc\x1f\x01",7) ||
       !m9_match(im,0x4eb5c1,(const unsigned char *)"\x43\xc7\x84\xbe\xe4\xf7\x7d\x01\x01\0\0\0",12) ||
       !m9_match(im,0x4eb533,(const unsigned char *)"\x4b\x8d\x04\xbf\x48\x8d\x04\xc5\x10\xd7\x67\x01\x44\x89\xbe\xa0\x05\0\0\x49\x03\xc6\x44\x89\xa6\x78\x03\0\0\x48\x89\x86\x98\x05\0\0",36) ||
       !m9_match(im,0x4ebe9e,(const unsigned char *)"\x48\x8b\x87\x98\x05\0\0\xb9\x02\0\0\0\x8b\x58\x04",15) ||
       !m9_match(im,0x4ebf11,(const unsigned char *)"\xf6\x40\x12\x04\x74\x3c\x0f\xb6\x40\x16",10) ||
       !m9_match(im,0x4ebfe3,(const unsigned char *)"\xf3\x0f\x2c\xc0\x66\x01\x47\x6a",8) ||
       !m9_match(im,0x4ebc73,(const unsigned char *)"\x48\x8d\x15\xd6\xe3\xff\xff",7))return;
    g_nikita_steer.base=im->base;g_nikita_steer.resolved=1;
}
static void nikita_steer_install(void) {
    uint64_t b=g_nikita_steer.base;const char *why="retail witnesses unavailable";
    if(g_nikita_steer.resolved)g_nikita_steer.live=dg_detour_install_ex(&g_nikita_steer.detour,
        (void *)(uintptr_t)(b+0x4ebd50),nikita_operation,
        (void *)(uintptr_t)(b+0x4ebd50),(void *)(uintptr_t)(b+0x4ec021),&why,2);
    if(g_b.log)g_b.log("  Nikita native steering: %s (%s)\r\n",g_nikita_steer.live?"installed":"unavailable",why);
}
static void nikita_steer_stop(void) {
    g_nikita_steer.live=0;nikita_steer_revoke();dg_detour_remove(&g_nikita_steer.detour);
}
