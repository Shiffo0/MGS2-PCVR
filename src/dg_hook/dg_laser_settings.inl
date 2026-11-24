/* Called only while the front-end input route owns this frame. */
static uintptr_t native_options_work;   /* the OPTIONS actor (= its Work) found by native_options_open this frame */

static int native_options_compatible(void) {
    static int checked, compatible;
    if (checked) return compatible;
    __try {
        compatible=!memcmp((void *)(g_base+0xe5390),"\x48\x8d\x15\x69\xed\xff\xff",7) &&
            !memcmp((void *)(g_base+0xa8df4),"\x48\x8d\x3d\xa5\xbf\x5c\x01",7);
    } __except(EXCEPTION_EXECUTE_HANDLER) { compatible=0; }
    checked=1;
    return compatible;
}
static int native_options_open(void) {
    uintptr_t list=(uintptr_t)g_base+0x1674da0+6*0x78;
    uintptr_t actor;
    int count;
    native_options_work=0;
    if (!native_options_compatible())return 0;
    __try {
        actor=*(uintptr_t *)list;
        for(count=0;actor && actor!=list+0x38 && count<192;count++) {
            if (*(uintptr_t *)(actor+8)==g_base+0xe4100){native_options_work=actor;return 1;}
            actor=*(uintptr_t *)actor;
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) { return 0; }
    return 0;
}
static volatile LONG g_laser_native_pulses, g_laser_native_ok, g_laser_native_vr;

static int native_list_open(DG_NATIVE_LIST *l) {
    uintptr_t list=(uintptr_t)g_base+0x1674da0+6*0x78,actor;
    int count,i,save_open=0;
    memset(l,0,sizeof *l);
    if(!native_options_compatible())return 0;
    __try {
        actor=*(uintptr_t *)list;
        for(count=0;actor && actor!=list+0x38 && count<192;count++,actor=*(uintptr_t *)actor) {
            uintptr_t act=*(uintptr_t *)(actor+8);

            if(act==(uintptr_t)g_base+0xcb0e0){save_open=1;continue;}
            if(l->kind!=NS_NONE)continue;
            if(act==(uintptr_t)g_base+0xfe200) {
                int last=-1;
                l->kind=NS_MAIN;l->wrap=1;l->half=0.028;

                for(i=0;i<NS_MAIN_MAX;i++) {
                    uintptr_t spr=*(uintptr_t *)(actor+0xa0+8*i);
                    float x,y,w;
                    l->v[i]=l->u0[i]=l->u1[i]=-1.0;
                    if(!spr)break;
                    if(*(unsigned *)(spr+0x30)&0x8000)continue;
                    x=*(float *)(spr+0x98);y=*(float *)(spr+0x9c);w=*(float *)(spr+0xa4);
                    if(!(y>0 && y<384 && x>=0 && x<512 && w>0 && w<512))continue;
                    l->v[i]=y/384.0;l->u0[i]=x/512.0-0.02;l->u1[i]=(x+w)/512.0+0.03;last=i;
                }
                l->n=last+1;
                l->cur=*(int *)(actor+0x88);
                l->ready=*(int *)(actor+0x70)==1 && *(int *)(actor+0xdc)<=0;
                continue;
            }
            /* LOAD only as the title menu opens it (constructor 0xd8fb0 writes 0 to +0xaaaa); the second constructor
               0xd9190 (called from the in-game block, 0x4fa7d0) writes 1 there and is left alone. */
            if(act==(uintptr_t)g_base+0xd64e0 && !*(unsigned char *)(actor+0xaaaa)) {
                int n=*(int *)(actor+0xa8d0);
                l->kind=NS_LOAD;l->wrap=0;l->half=10.5/384.0;
                if(n<0)n=0;if(n>10)n=10;
                for(i=0;i<n;i++){l->v[i]=(125.5+21.0*i)/384.0;l->u0[i]=0.10;l->u1[i]=0.94;}
                l->n=n;
                l->cur=*(int *)(actor+0xa8b0);
                l->ready=*(int *)(actor+0xa894)==4 && *(int *)(actor+0xa99c)<=0;
            }
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) { memset(l,0,sizeof *l); return 0; }
    if(save_open || count>=192){memset(l,0,sizeof *l);return 0;}
    if(l->cur<0 || l->cur>=l->n || l->v[l->cur]<0)l->ready=0;
    return l->kind!=NS_NONE && l->n>0;
}

static void native_options_cursor(int *cursor,int *ready) {
    uintptr_t w=native_options_work;
    *cursor=-1;*ready=0;
    if(!w)return;
    __try {
        int c=*(int *)(w+0xac);
        if(c>=0 && c<9)*cursor=c;
        *ready=*cursor>=0 && *(int *)(w+0x84)==0 && *(int *)(w+0x88)==3 && *(int *)(w+0x104)<=0;
    } __except(EXCEPTION_EXECUTE_HANDLER) { *cursor=-1;*ready=0; }
}

static void menu_native_probe(void) {
    static ULONGLONG next;
    uintptr_t list=(uintptr_t)g_base+0x1674da0+6*0x78,actor;
    ULONGLONG now=GetTickCount64();
    double u=-1,v=-1;
    int count,have_uv;
    char line[640],msg[2048];
    int mat=0;
    msg[0]=0;
    if(now<next)return;
    next=now+1000;
    have_uv=dg_xr_laser_uv(&u,&v);
    __try {
        actor=*(uintptr_t *)list;
        for(count=0;actor && actor!=list+0x38 && count<192;count++,actor=*(uintptr_t *)actor) {
            uintptr_t act=*(uintptr_t *)(actor+8);
            if(act==(uintptr_t)g_base+0xe4100) {
                mat=mat<0?0:mat;mat+=_snprintf_s(msg+mat,sizeof msg-mat,_TRUNCATE,"  menu probe: OPTIONS row %d id %d step %d sub %d anim %d | laser %s%.4f %.4f | pulses %ld ok %ld vr %ld\r\n",
                      *(int *)(actor+0xac),*(int *)(actor+0xa8),*(int *)(actor+0x84),*(int *)(actor+0x88),*(int *)(actor+0x104),
                      have_uv?"":"none ",u,v,g_laser_native_pulses,g_laser_native_ok,g_laser_native_vr);
            } else if(act==(uintptr_t)g_base+0xfe200) {
                int i,at=0;
                at+=_snprintf_s(line+at,sizeof line-at,_TRUNCATE,"  menu probe: MAIN cursor %d sub %d anim %d |",
                                *(int *)(actor+0x88),*(int *)(actor+0x70),*(int *)(actor+0xdc));
                for(i=0;i<7 && at>0 && at<(int)sizeof line-80;i++) {
                    uintptr_t spr=*(uintptr_t *)(actor+0xa0+8*i);
                    if(!spr){at+=_snprintf_s(line+at,sizeof line-at,_TRUNCATE," [%d -]",i);continue;}
                    at+=_snprintf_s(line+at,sizeof line-at,_TRUNCATE," [%d x%.1f y%.1f w%.1f h%.1f s%.2f p%p f%X]",i,
                                    *(float *)(spr+0x98),*(float *)(spr+0x9c),*(float *)(spr+0xa4),*(float *)(spr+0xa8),
                                    *(float *)(spr+0x5c),*(void **)spr,*(unsigned *)(spr+0x30));
                }
                mat=mat<0?0:mat;mat+=_snprintf_s(msg+mat,sizeof msg-mat,_TRUNCATE,"%s | laser %s%.4f %.4f\r\n",line,have_uv?"":"none ",u,v);
            } else if(act==(uintptr_t)g_base+0xd64e0) {
                mat=mat<0?0:mat;mat+=_snprintf_s(msg+mat,sizeof msg-mat,_TRUNCATE,"  menu probe: LOAD variant %d sub %d row %d page %d/%d rows %d cursor_y %.1f/%.1f anim %d | laser %s%.4f %.4f\r\n",
                      *(unsigned char *)(actor+0xaaaa),*(int *)(actor+0xa894),*(int *)(actor+0xa8b0),*(int *)(actor+0xa8b4),*(int *)(actor+0xa8b8),
                      *(int *)(actor+0xa8d0),*(float *)(actor+0xa8c0),*(float *)(actor+0xa8c4),*(int *)(actor+0xa99c),
                      have_uv?"":"none ",u,v);
            }
        }
    } __except(EXCEPTION_EXECUTE_HANDLER) {}
    if(msg[0])logf_("%s",msg);
}

static volatile LONG g_vr_row_show, g_vr_row_sel;

static volatile LONG g_vr_row_drawn;
static void laser_vr_row_present(IDXGISwapChain *sc) {
    static unsigned pixels[1600*112];
    ID3D11Texture2D *bb=NULL; ID3D11Device *dev=NULL; ID3D11DeviceContext *ctx=NULL;
    D3D11_TEXTURE2D_DESC d; DG_LASER_RECT r; D3D11_BOX box;
    int x,y,scale,sel,rgba,mark,drawn=0;
    if(!InterlockedExchange(&g_vr_row_show,0) || !sc){InterlockedExchange(&g_vr_row_drawn,0);return;}   /* menu_seam sets it every frame it wants the row */
    sel=InterlockedCompareExchange(&g_vr_row_sel,0,0)!=0;
    if(FAILED(sc->lpVtbl->GetBuffer(sc,0,&IID_ID3D11Texture2D,(void **)&bb)) || !bb){InterlockedExchange(&g_vr_row_drawn,0);return;}
    bb->lpVtbl->GetDesc(bb,&d);
    rgba=d.Format==DXGI_FORMAT_R8G8B8A8_UNORM || d.Format==DXGI_FORMAT_R8G8B8A8_UNORM_SRGB;
    if(d.SampleDesc.Count!=1 || !(rgba || d.Format==DXGI_FORMAT_B8G8R8A8_UNORM || d.Format==DXGI_FORMAT_B8G8R8A8_UNORM_SRGB) ||
       !laser_native_vr_rect((int)d.Width,(int)d.Height,&r) || r.w*r.h>(int)(sizeof pixels/sizeof pixels[0]))goto done;
    scale=r.h/12; if(scale<1)scale=1;
    mark=(int)(0.017*d.Width);   /* the menu's cursor square sits ~0.073..0.086 of the width, our text box starts at 0.109 */
    for(y=0;y<r.h;y++)for(x=0;x<r.w;x++)pixels[y*r.w+x]=0xff0b0f0fu;
    dg_laser_text(pixels,r.w,scale,"VR SETTINGS");
    for(y=0;y<r.h;y++)for(x=0;x<r.w;x++){
        unsigned *p=&pixels[y*r.w+x];
        if(*p==0xffefffffu)*p=sel?0xffe8ece8u:0xff6f7470u;
    }
    bb->lpVtbl->GetDevice(bb,&dev); if(!dev)goto done;
    dev->lpVtbl->GetImmediateContext(dev,&ctx); if(!ctx)goto done;
    if(rgba)for(y=0;y<r.h*r.w;y++){unsigned p=pixels[y];pixels[y]=(p&0xff00ff00u)|((p&0xff)<<16)|((p>>16)&0xff);}
    box.left=r.x;box.right=r.x+r.w;box.top=r.y;box.bottom=r.y+r.h;box.front=0;box.back=1;
    ctx->lpVtbl->UpdateSubresource(ctx,(ID3D11Resource *)bb,0,&box,pixels,4*r.w,0);
    drawn=1;
    if(sel && mark>2 && r.x-2*mark>0) {   /* the square marker left of the text, as on the native rows */
        static unsigned sq[64*64];
        int s=mark<64?mark:64;
        for(y=0;y<s*s;y++)sq[y]=0xffc8d0c8u;
        box.left=r.x-2*s;box.right=box.left+s;box.top=r.y+(r.h-s)/2;box.bottom=box.top+s;
        ctx->lpVtbl->UpdateSubresource(ctx,(ID3D11Resource *)bb,0,&box,sq,4*s,0);
    }
done:
    InterlockedExchange(&g_vr_row_drawn,drawn);
    if(ctx)ctx->lpVtbl->Release(ctx);
    if(dev)dev->lpVtbl->Release(dev);
    bb->lpVtbl->Release(bb);
}
static volatile LONG laser_values[LASER_ROWS];
static char laser_ini[MAX_PATH];
static int laser_page;
static void laser_read_settings(DG_XR_CONFIG *xc,DG_BRIDGE_CONFIG *bc) {
    int i,v[LASER_ROWS]={xc->stereo,bc->pistol_reload,bc->m9_slide,
        xc->observer?xc->observer_mode+1:0,0,bc->turn_mode,xc->radar_mode,g_menu_hand_right};
    v[LASER_SECOND]=xc->second_eye;
    if(!laser_ini[0]){
        char path[MAX_PATH],*slash;
        DWORD len=GetModuleFileNameA(NULL,path,sizeof path);
        if(!len || len>=sizeof path || !(slash=strrchr(path,'\\')))return;
        if(strcpy_s(slash+1,sizeof path-(size_t)(slash+1-path),"mgs2vr_settings.ini"))return;
        strcpy_s(laser_ini,sizeof laser_ini,path);
    }
    for(i=0;i<LASER_ROWS;i++){
        int value=(int)GetPrivateProfileIntA("VR",laser_name[i],-1,laser_ini);
        if(value>=0 && value<laser_count[i]){
            v[i]=value;
            switch(i){
            case LASER_VIEW: xc->stereo=value;break;
            case LASER_RELOAD: bc->pistol_reload=value;break;
            case LASER_SLIDE: bc->m9_slide=value;break;
            case LASER_OBSERVER: xc->observer=value!=0;if(value)xc->observer_mode=value-1;break;
            case LASER_SECOND: xc->second_eye=value;break;
            case LASER_TURN: bc->turn_mode=value;break;
            case LASER_RADAR: xc->radar_mode=value;xc->radar_hud=!value;break;
            case LASER_HAND: InterlockedExchange(&g_menu_hand_right,value);break;
            }
        }
        InterlockedExchange(&laser_values[i],v[i]);
    }
}
static void laser_change(int row) {
    char val[4];int v=(laser_values[row]+1)%laser_count[row];
    _snprintf_s(val,sizeof val,_TRUNCATE,"%d",v);
    if(!laser_ini[0] || !WritePrivateProfileStringA("VR",laser_name[row],val,laser_ini)){
        logf_("  menu: unable to save VR setting %s\r\n",laser_name[row]);return;
    }
    InterlockedExchange(&laser_values[row],v);
    if(row==LASER_VIEW)InterlockedExchange(&g_mod_stereo_override,v);
    if(row==LASER_OBSERVER){
        InterlockedExchange(&g_mod_observer_override,v!=0);
        if(v)InterlockedExchange(&g_mod_observer_mode_override,v-1);
    }
    if(row==LASER_SECOND)InterlockedExchange(&g_mod_second_eye_override,v);
    if(row==LASER_RELOAD || row==LASER_SLIDE)
        dg_bridge_mod_menu_request((laser_values[LASER_RELOAD]?2:0)|(laser_values[LASER_SLIDE]?4:0));
}
