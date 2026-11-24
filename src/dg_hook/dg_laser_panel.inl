/* XR owns rasterization. Publish the exact control under the visible cursor
   with one interlocked word, including the render time and current page. */
static volatile LONG laser_target;
static volatile LONG laser_panel, laser_display;
static volatile LONG laser_uv=-1;   /* last hit (u,v)*10000, -1 = no hit */
static volatile LONG laser_uv_tick; /* GetTickCount of the render that wrote laser_uv */
/* options: 1 = native OPTIONS, 2 = a native list (main menu, LOAD) is open: no bar, the Present side hit-tests
   the published (u,v) itself. */
void dg_xr_laser_panel(int options,int page,unsigned values) {
    InterlockedExchange(&laser_display,(LONG)values);
    InterlockedExchange(&laser_panel,(options==1?1:0)|(page?2:0)|(options==2?8:0));
}
int dg_xr_laser_target(int options,int page,int hand) {
    unsigned s=(unsigned)InterlockedCompareExchange(&laser_target,0,0);
    unsigned stamp=(GetTickCount()&0xffff);
    if(!s || ((stamp-(s>>16))&0xffff)>90 ||
       ((s>>8)&15)!=(unsigned)((options==1?1:0)|(page?2:0)|(hand==2?4:0)|(options==2?8:0)))return -1;
    return (int)(s&255)-1;
}
int dg_xr_laser_uv(double *u,double *v) {
    LONG w=InterlockedCompareExchange(&laser_uv,0,0);
    if(w<0 || !u || !v || ((GetTickCount()-(DWORD)InterlockedCompareExchange(&laser_uv_tick,0,0))&0xffffffffu)>90)return 0;
    *u=(w&0xffff)/10000.0;*v=((unsigned)w>>16)/10000.0;return 1;
}
static void laser_panel_draw(const DG_SUBMIT_CAPTURE *cap,ID3D11Texture2D *texture,int hand,
                             double u,double v,int hit) {
    static unsigned pixels[1600*112]; /* largest button at scale four; the VR SETTINGS line up to 4K */
    static const char *const navigation[]={"UP","DOWN","LEFT","RIGHT","OK","BACK","VR SETTINGS"};
    unsigned values=(unsigned)InterlockedCompareExchange(&laser_display,0,0);
    int mode=InterlockedCompareExchange(&laser_panel,0,0),page=mode&2,options=mode&1,list=mode&8;
    int id,target=-1,scale=cap->width/600;
    if(scale<1)scale=1;
    if(scale>4)scale=4;
    InterlockedExchange(&laser_uv,hit && u>=0 && u<=1 && v>=0 && v<=1 ? (LONG)((unsigned)(u*10000)|((unsigned)(v*10000)<<16)) : -1);
    InterlockedExchange(&laser_uv_tick,(LONG)GetTickCount());
    if(list && !page) {

        InterlockedExchange(&laser_target,(LONG)((GetTickCount()<<16)|((mode|(hand==2?4:0))<<8)));
        return;
    }
    if(options && !page) {
        /* Native OPTIONS: no button bar. The rows are the game's; the VR SETTINGS line is drawn into the game's back
           buffer on the Present side (laser_vr_row_present), so the capture already carries it: hit-test only. */
        target=hit ? laser_native_hit(u,v) : -1;
        InterlockedExchange(&laser_target,(LONG)((GetTickCount()<<16)|((mode|(hand==2?4:0))<<8)|(target+1)));
        return;
    }
    for(id=0;id<=LASER_DONE;id++){
        char label[40];
        DG_LASER_RECT r;
        D3D11_BOX box;
        int x,y,hover,row=id-LASER_FIRST_ROW;
        if(!laser_rect(id,cap->width,cap->height,page,options,&r))continue;
        hover=hit && u*cap->width>=r.x && u*cap->width<r.x+r.w &&
                     v*cap->height>=r.y && v*cap->height<r.y+r.h;
        if(hover)target=id;
        if(id==LASER_DONE)strcpy_s(label,sizeof label,"BACK");
        else if(id<LASER_FIRST_ROW)strcpy_s(label,sizeof label,navigation[id]);
        else{
            int value=(values>>(2*row))&3;
            const char *choice=value<laser_count[row]?laser_choice[row][value]:"-";
            _snprintf_s(label,sizeof label,_TRUNCATE,"%s  %s",laser_name[row],choice);
        }
        for(y=0;y<r.h;y++)for(x=0;x<r.w;x++)
            pixels[y*r.w+x]=hover?0xff146369u:0xff16272du;
        dg_laser_text(pixels,r.w,scale,label);
        box.left=r.x;box.right=r.x+r.w;box.top=r.y;box.bottom=r.y+r.h;box.front=0;box.back=1;
        g_ctx->lpVtbl->UpdateSubresource(g_ctx,(ID3D11Resource *)texture,0,&box,pixels,4*r.w,0);
    }
    InterlockedExchange(&laser_target,(LONG)((GetTickCount()<<16)|((mode|(hand==2?4:0))<<8)|(target+1)));
}
