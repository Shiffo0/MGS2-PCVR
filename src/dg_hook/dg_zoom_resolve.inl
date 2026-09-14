#include "dg_zoom_signatures.h"
typedef struct {uint64_t callback,end,seam,parent_slot,out_mask,in_mask,out_index,in_index;int valid;} DG_ZOOM_ANCHORS;
static int zoom_signature(const LiveImage *im,uint64_t va,const unsigned char *b,const unsigned char *m,unsigned n) {
    unsigned i;DWORD r=(DWORD)(va-im->base);
    if(va<im->base || va-im->base>im->size || !da_code(im,r,n))return 0;
    for(i=0;i<n;i++)if((im->bytes[r+i]&m[i])!=(b[i]&m[i]))return 0;
    return 1;
}
static int zoom_resolve_image(const LiveImage *im,const DG_CAMERA_ANCHORS *camera,DG_ZOOM_ANCHORS *out) {
    DG_ZOOM_ANCHORS a={0};uint64_t start,ctor,cb;DWORD c,s,beg,end;const unsigned char *p;
    memset(out,0,sizeof *out);
    if(!camera->valid || camera->callback<im->base)return 0;
    c=(DWORD)(camera->callback-im->base);
    if(!da_code(im,c,18)||im->bytes[c+13]!=0xe8)return 0;
    start=da_rel(im,c+13,5,im->bytes+c+14);s=(DWORD)(start-im->base);
    if(!zoom_signature(im,start,zoom_start_bytes,zoom_start_mask,sizeof zoom_start_bytes))return 0;
    p=im->bytes+s;ctor=da_rel(im,s+0x6c,5,p+0x6d);
    if(!zoom_signature(im,ctor,zoom_ctor_bytes,zoom_ctor_mask,sizeof zoom_ctor_bytes))return 0;
    c=(DWORD)(ctor-im->base);cb=da_rel(im,c+0x5a,7,im->bytes+c+0x5d);
    if(!zoom_signature(im,cb,zoom_consumer_bytes,zoom_consumer_mask,sizeof zoom_consumer_bytes))return 0;
    c=(DWORD)(cb-im->base);p=im->bytes+c;
    if(!find_runtime_function(im,c,&beg,&end)||beg!=c)return 0;
    a.callback=cb;a.end=im->base+end;a.seam=cb+0x164;
    a.parent_slot=da_rel(im,s+0x46,7,im->bytes+s+0x49);
    a.out_mask=da_rel(im,c+0x187,6,p+0x189);
    a.in_mask=da_rel(im,c+0x200,6,p+0x202);
    a.out_index=da_rel(im,c+0xe4,7,p+0xe7);
    a.in_index=da_rel(im,c+0xf4,7,p+0xf7);
    if(!target_is_writable_data(im,a.parent_slot)||!target_is_writable_data(im,a.out_mask)||
       !target_is_writable_data(im,a.in_mask)||!target_is_writable_data(im,a.out_index)||
       !target_is_writable_data(im,a.in_index)||
       da_rel(im,c+0xb,6,p+0xd)!=camera->pause ||
       da_rel(im,s+0x1a,7,im->bytes+s+0x1d)!=camera->pad)return 0;
    a.valid=1;*out=a;return 1;
}
