/* Snapshot resolver: relative relationships, approved fingerprint at caller. */
typedef struct {
    uint64_t seam,begin,end,hold,not_open,mask_prg,mask_scn;
    unsigned action_off,data_off,pad_off;
    int valid;
} DG_ACTION_ANCHORS;
static uint32_t da_u32(const unsigned char *p){uint32_t v;memcpy(&v,p,4);return v;}
static int da_code(const LiveImage *im,DWORD r,size_t n) {
    const IMAGE_SECTION_HEADER *s=section_for_rva(im,r);
    return s && (s->Characteristics&0xe0000000u)==0x60000000u &&
        r>=s->VirtualAddress && n<=s->Misc.VirtualSize &&
        r-s->VirtualAddress<=s->Misc.VirtualSize-n && all_valid(im->valid,r,(DWORD)n,im->size);
}
static uint64_t da_rel(const LiveImage *im,DWORD r,unsigned end,const unsigned char *d) {
    int64_t q=(int64_t)r+end+(int32_t)da_u32(d);
    return q>=0 && q<im->size?im->base+q:0;
}
static int da_eq(const unsigned char *p,const char *s,size_t n){return !memcmp(p,s,n);}
static int da_resolve(const LiveImage *im,uint64_t normal,DG_ACTION_ANCHORS *out) {
    DWORD r,beg,end;unsigned pc=0,hc=0,dc=0,k;DG_ACTION_ANCHORS a;
    memset(out,0,sizeof *out);memset(&a,0,sizeof a);
    if(!normal || !im->nt || !im->valid || im->base>UINT64_MAX-im->size)return 0;
    /* Unknown RX bytes could hide a second matching chain. */
    for(k=0;k<im->section_count;k++) {
        const IMAGE_SECTION_HEADER *sec=&im->sections[k];
        if((sec->Characteristics&0xe0000000u)==0x60000000u &&
           !all_valid(im->valid,sec->VirtualAddress,sec->Misc.VirtualSize,im->size))return 0;
    }
    for(r=0;r+0xb0<im->size;r++) {
        const unsigned char *p=im->bytes+r;
        if(p[0]==0xf2 && da_eq(p,"\xf2\x0f\x10\x05",4) && da_code(im,r,0xb0) &&
           da_eq(p+8,"\x4c\x8d\x44\x24\x40",5) &&
           da_eq(p+0x1d,"\x41\xf7\xc2\x03\x01\0\0\x74\x22\x33\xd2",11) &&
           da_eq(p+0x48,"\x8b\x54\x24\x24\x41\xf6\xc2\x0c\x74\x18",10) &&
           da_eq(p+0x52,"\x41\xf6\xc2\x04\x74\x06\x23\x15",8) &&
           da_eq(p+0x5e,"\x41\xf6\xc2\x08\x74\x06\x23\x15",8) &&
           da_eq(p+0x85,"\x89\x15",2) && da_eq(p+0x98,"\x89\x05",2) &&
           da_eq(p+0xa9,"\x89\x05",2) && r>=0x4e &&
           da_eq(p-0x4e,"\x44\x8b\x15",3) &&
           da_rel(im,r-0x4e,7,p-0x4b)==normal+36 &&
           da_rel(im,r,0x8b,p+0x87)==normal+4 &&
           da_rel(im,r,0x9e,p+0x9a)==normal+8 &&
           da_rel(im,r,0xaf,p+0xab)==normal+12 &&
           find_runtime_function(im,r,&beg,&end)) {
            a.mask_prg=da_rel(im,r,0x5e,p+0x5a);
            a.mask_scn=da_rel(im,r,0x6a,p+0x66);
            if(!target_is_writable_data(im,a.mask_prg) || a.mask_scn!=a.mask_prg+4)continue;
            a.seam=im->base+r;a.begin=im->base+beg;a.end=im->base+end;pc++;
        }
        /* Setter: two distinct callback LEAs select the SetMode argument. */
        if(p[0]==0x48 && da_eq(p,"\x48\x8d\x15",3) &&
           da_eq(p+7,"\xf6\xc3\x01\x75\x07\x48\x8d\x15",8) && p[19]==0xe8 &&
           da_code(im,r,24) && r>=36 && da_code(im,r-36,60) &&
           da_eq(p-36,"\xb9\0\x20\0\0\xe8",6)) {
            uint64_t no=da_rel(im,r,7,p+3),hold=da_rel(im,r,19,p+15),set=da_rel(im,r,24,p+20);
            DWORD h=(DWORD)(hold-im->base),m=(DWORD)(set-im->base),n=(DWORD)(no-im->base);
            const unsigned char *q,*v;
            if(!hold||!set||!no||hold==no||!da_code(im,h,0x655)||!da_code(im,m,0xf0)||
               !da_code(im,n,16)||!find_runtime_function(im,h,&beg,&end)||beg!=h)continue;
            q=im->bytes+h;v=im->bytes+m;
            if(!da_eq(q+0x20b,"\x4c\x8b\x87",3)||!da_eq(q+0x218,"\x41\x85\x50\x0c",4)||
               !da_eq(q+0x238,"\x8b\x8f",2)||!da_eq(q+0x2e7,"\x41\x85\x50\x04",4)||
               !da_eq(q+0x39d,"\x48\x8b\x87",3)||!da_eq(q+0x3a4,"\x8b\x48\x08",3)||
               !da_eq(v+0x86,"\x48\x89\x91",3)||!da_eq(v+0xc5,"\x4c\x89\x89",3))continue;
            a.action_off=da_u32(v+0x89);a.data_off=da_u32(q+0x23a);a.pad_off=da_u32(q+0x20e);
            if(a.data_off!=da_u32(v+0xc8)||a.pad_off!=da_u32(q+0x3a0)||
               !a.action_off||a.action_off>0x10000||a.action_off%8||
               a.data_off>0x10000||a.data_off%4||a.pad_off>0x10000||a.pad_off%8)continue;
            a.hold=hold;a.not_open=no;hc++;
        }
    }
    if(pc!=1 || hc!=1)return 0;
    /* Independent native dispatcher uses the same action offset. */
    for(r=0;r+20<im->size;r++)if(im->bytes[r]==0x8b &&
        da_eq(im->bytes+r,"\x8b\xd3\x48\x8b\xcf",5) && da_code(im,r,40)) {
        const unsigned char *p=im->bytes+r;
        if(da_eq(p+31,"\xff\x97",2)&&da_u32(p+33)==a.action_off)dc++;
    }
    if(dc!=1)return 0;
    a.valid=1;*out=a;return 1;
}
