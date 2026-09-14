typedef struct {
    uint64_t callback,end,destroy,channel,pad,mask,pause;
    int valid;
} DG_CAMERA_ANCHORS;
static int camera_resolve_image(const LiveImage *im,uint64_t pad,uint64_t mask,DG_CAMERA_ANCHORS *out) {
    DWORD r,beg,end;unsigned hits=0,k;DG_CAMERA_ANCHORS a;
    memset(out,0,sizeof *out);memset(&a,0,sizeof a);
    if(!im->nt || !im->bytes || !im->valid || !pad || !mask)return 0;
    for(k=0;k<im->section_count;k++) {
        const IMAGE_SECTION_HEADER *s=&im->sections[k];
        if((s->Characteristics&0xe0000000u)==0x60000000u &&
           !all_valid(im->valid,s->VirtualAddress,s->Misc.VirtualSize,im->size))return 0;
    }
    /* Match the constructor chain, derive the callback and its pad consumer.
     * No fixed RVA is used as a live write destination. */
    for(r=0;r+0x300<im->size;r++) {
        const unsigned char *p=im->bytes+r,*q;uint64_t cb,des,set,timer;DWORD c;
        if(p[0]!=0x48 || !da_code(im,r,0xc0) ||
           !da_eq(p,"\x48\x8d\x15",3) ||
           !da_eq(p+7,"\x48\x8b\xc8\xe8",4) ||
           !da_eq(p+0x3d,"\xe8",1) ||
           !da_eq(p+0x42,"\x48\x63\x05",3) ||
           !da_eq(p+0x5a,"\x48\x8d\x0c\x80\x48\x8d\x82",7) ||
           !da_eq(p+0x65,"\x48\x8d\x04\xc8\x48\x89\x83\xd0\0\0\0",11) ||
           r<12 || !da_code(im,r-12,12) ||
           !da_eq(p-12,"\x4c\x8d\x05",3))continue;
        cb=da_rel(im,r,7,p+3);des=da_rel(im,r-12,7,p-9);
        set=da_rel(im,r,15,p+11);
        if(!cb||!des||!set || !da_code(im,(DWORD)(set-im->base),9) ||
           !da_eq(im->bytes+(DWORD)(set-im->base),"\x48\x89\x51\x08\x4c\x89\x41\x20\xc3",9))continue;
        c=(DWORD)(cb-im->base);
        if(!da_code(im,c,0x1a0) || !find_runtime_function(im,c,&beg,&end)||beg!=c)continue;
        q=im->bytes+c;
        if(!da_eq(q,"\x48\x89\x5c\x24\x10\x57\x48\x83\xec\x20\x48\x8b\xf9",13) ||
           !da_eq(q+0xb7,"\x0f\xb7\x8f\xd8\0\0\0",7) ||
           !da_eq(q+0x130,"\x48\x8b\x87\xd0\0\0\0\xb9\x1e\0\0\0",12) ||
           !da_eq(q+0x141,"\x8b\x70\x08\x66\xff\x87\xda\0\0\0",10) ||
           !da_eq(q+0x15b,"\x85\x35",2) ||
           !da_eq(q+0x163,"\x81\xbf\xe0\0\0\0\x80\0\0\0",10) ||
           !da_eq(q+0x16f,"\x83\xbf\xf8\0\0\0\0",7))continue;
        if(q[0x152]!=0xe8)continue;
        timer=da_rel(im,c+0x152,5,q+0x153);
        if(!timer || !da_code(im,(DWORD)(timer-im->base),0x37))continue;
        /* DIRECT_TICK(30): mode one yields (5*30+1)/6 = 25;
         * otherwise returns 30. The mode query is read by native code. */
        {const unsigned char *t=im->bytes+(DWORD)(timer-im->base);
         if(!da_eq(t,"\x40\x53\x48\x83\xec\x20\x8b\xd9\xe8",9) ||
            !da_eq(t+0xd,"\x83\xf8\x01\x75\x1d\x8d\x0c\x9d\x01\0\0\0\xb8\xab\xaa\xaa\x2a\x03\xcb\xf7\xe9\x8b\xc2\xc1\xe8\x1f\x03\xc2\x48\x83\xc4\x20\x5b\xc3\x8b\xc3\x48\x83\xc4\x20\x5b\xc3",42))continue;}
        a.channel=da_rel(im,r+0x42,7,p+0x45);
        a.pad=im->base+da_u32(p+0x61);
        a.mask=da_rel(im,c+0x15b,6,q+0x15d);
        a.pause=da_rel(im,c+0x123,7,q+0x125);
        if(a.pad!=pad||a.mask!=mask||!target_is_writable_data(im,a.channel)||
           !target_is_writable_data(im,a.pause))continue;
        a.callback=cb;a.end=im->base+end;a.destroy=des;hits++;
    }
    if(hits!=1)return 0;
    a.valid=1;*out=a;return 1;
}
