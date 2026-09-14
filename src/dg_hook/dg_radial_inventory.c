#include "dg_radial_inventory.h"
#include <stdlib.h>
#include <string.h>
typedef struct section { uint32_t rva,size,flags; } section;
typedef struct scan {
    const dg_radial_inventory_image *im;
    section sec[96]; unsigned nsec;
    uint64_t actor_item,actor_weapon,player;
    uint32_t itemoff,weaponoff;
    unsigned actors, desireds, setters, wg,ig;
    uint64_t desired_slot,set_slot,wp,ip;
    uint32_t offsets[2][4];
} scan;
static uint16_t u16(const unsigned char *p) { return (uint16_t)(p[0]|((unsigned)p[1]<<8)); }
static uint32_t u32(const unsigned char *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static int rd(const dg_radial_inventory_image *im,uint64_t a,void *p,size_t n) {
    return im && im->approved_identity && im->read && a && n &&
        (uint64_t)n<=UINT64_MAX-a && im->read(im->ctx,a,p,n)==1;
}
static int mr(const dg_radial_inventory_image *im,uint32_t r,void *p,size_t n) {
    return im && r<=im->size && n<=im->size-r && rd(im,im->base+r,p,n);
}
static int target(scan *s,uint32_t r,unsigned end,int32_t disp,size_t n,uint64_t *out) {
    int64_t v=(int64_t)r+end+disp; unsigned k;
    if(v<0 || (uint64_t)v>s->im->size || n>s->im->size-(size_t)v) return 0;
    for(k=0;k<s->nsec;++k) {
        section *q=&s->sec[k];
        if((q->flags&0xe0000000u)!=0xc0000000u) continue;
        if((uint64_t)v>=q->rva && (uint64_t)v-q->rva<=q->size && n<=q->size-((size_t)v-q->rva)) {
            *out=s->im->base+(uint64_t)v; return 1;
        }
    }
    return 0;
}
static uint64_t calltarget(scan *s,uint32_t r,unsigned end,int32_t disp) {
    int64_t v=(int64_t)r+end+disp; unsigned k;
    for(k=0;k<s->nsec;++k) if(v>=s->sec[k].rva &&
       (uint64_t)v-s->sec[k].rva<s->sec[k].size && (s->sec[k].flags&0x20000000u))
        return s->im->base+(uint64_t)v;
    return 0;
}
static int actorleaf(const unsigned char *b) {
    return !memcmp(b,"\x48\x8b\x05",3) && !memcmp(b+7,"\x48\x85\xc0\x75\x01\xc3\x8b\x80",8) && b[19]==0xc3;
}
static void inspect(scan *s,const unsigned char *b,uint32_t r,int pass) {
    uint64_t a,c; unsigned k;
    if(pass==0) {
        /* Adjacent null-guarded item/weapon leaves, separated by INT3 padding.
         * Scalar displacements are extracted; lower weapon and item +0x24
         * are cross-checked by the independent desired-reader call pair. */
        if(!actorleaf(b) || !actorleaf(b+32)) return;
        for(k=20;k<32;++k) if(b[k]!=0xcc) return;
        if(u32(b+15)<=u32(b+47) || u32(b+15)-u32(b+47)!=0x24 || u32(b+15)>0x10000) return;
        if(!target(s,r,7,(int32_t)u32(b+3),8,&a) || !target(s,r,39,(int32_t)u32(b+35),8,&c) || a!=c) return;
        ++s->actors; s->player=a; s->actor_item=s->im->base+r;
        s->actor_weapon=s->im->base+r+32; s->itemoff=u32(b+15); s->weaponoff=u32(b+47);
        return;
    }
    if(pass==1) {
        static const unsigned char head[]={0x48,0x89,0x5c,0x24,0x08,0x57,0x48,0x83,0xec,0x20,0x48,0x63,0xd9,0xe8};
        /* Two distinct NoUse readers share these constraints; expected two
         * matches, not first-match. Cross tables/masks themselves unpublished. */
        if(s->actors==1 && !memcmp(b,head,sizeof(head)) &&
           !memcmp(b+18,"\x8b\xf8\xe8",3) && b[51]==0x48 && b[52]==0x8b && b[53]==0x05 &&
           !memcmp(b+58,"\x8b\xd3\x0f\xbf\x88\x04\x01\x00\x00\xe8",10) &&
           !memcmp(b+76,"\x48\x8b\x05",3) &&
           !memcmp(b+83,"\x8b\xd3\x0f\xbf\x88\x06\x01\x00\x00\xe8",10) &&
           calltarget(s,r,18,(int32_t)u32(b+14))==s->actor_item &&
           calltarget(s,r,25,(int32_t)u32(b+21))==s->actor_weapon &&
           target(s,r,58,(int32_t)u32(b+54),8,&a) &&
           target(s,r,83,(int32_t)u32(b+79),8,&c) && a==c) {
            if(!s->desireds) s->desired_slot=a;
            else if(s->desired_slot!=a) s->desired_slot=0;
            ++s->desireds;
        }
        /* Active set selection: two alternate rows, four pointer publications.
         * Byte form proved in retail; offsets extracted, not save assumptions. */
        if(!memcmp(b,"\x48\x8b\x05",3) && !memcmp(b+7,"\x85\xc9\x75\x1d",4) &&
           !memcmp(b+11,"\x48\x8d\x88",3) && !memcmp(b+18,"\x48\x8d\x90",3) &&
           !memcmp(b+25,"\x4c\x8d\x80",3) && !memcmp(b+32,"\x48\x05",2) &&
           !memcmp(b+38,"\xeb\x1b\x48\x8d\x88",5) &&
           !memcmp(b+47,"\x48\x8d\x90",3) && !memcmp(b+54,"\x4c\x8d\x80",3) &&
           !memcmp(b+61,"\x48\x05",2) &&
           !memcmp(b+67,"\x48\x89\x05",3) && !memcmp(b+74,"\x4c\x89\x05",3) &&
           !memcmp(b+81,"\x48\x89\x15",3) && !memcmp(b+88,"\x48\x89\x0d",3) && b[95]==0xc3) {
            uint64_t slots[4]={0,0,0,0},link; uint32_t off[2][4]; int good=1;
            if(!target(s,r,7,(int32_t)u32(b+3),8,&link)) return;
            for(k=0;k<4;++k) if(!target(s,r,74+7*k,(int32_t)u32(b+70+7*k),8,&slots[k])) good=0;
            off[0][0]=u32(b+34); off[0][1]=u32(b+28); off[0][2]=u32(b+21); off[0][3]=u32(b+14);
            off[1][0]=u32(b+63); off[1][1]=u32(b+57); off[1][2]=u32(b+50); off[1][3]=u32(b+43);
            for(k=0;k<2;++k) {
                unsigned j;
                for(j=0;j<4;++j) if(off[k][j]>0x10000 || (off[k][j]&1)) good=0;
                for(j=1;j<4;++j) if(off[k][j]<=off[k][j-1]) good=0;
                if(off[k][1]-off[k][0]>2*DG_RINV_MAX_SLOTS || off[k][3]-off[k][2]>2*DG_RINV_MAX_SLOTS) good=0;
            }
            if(off[1][0]<=off[0][3] || off[1][1]-off[1][0]!=off[0][1]-off[0][0] ||
               off[1][3]-off[1][2]!=off[0][3]-off[0][2]) good=0;
            for(k=0;k<4;++k) { unsigned j; for(j=0;j<k;++j) if(slots[k]==slots[j]) good=0; }
            if(!good) return;
            ++s->setters; s->set_slot=link; s->wp=slots[0]; s->ip=slots[2];
            memcpy(s->offsets,off,sizeof(off));
        }
        return;
    }
    if(s->setters==1 && !memcmp(b,"\x48\x8b\x05",3) &&
       !memcmp(b+7,"\x48\x63\xd1\x0f\xbf\x04\x50\xc3",8) &&
       target(s,r,7,(int32_t)u32(b+3),8,&a)) {
        if(a==s->wp) ++s->wg;
        if(a==s->ip) ++s->ig;
    }
}
static int sections(scan *s) {
    unsigned char dos[64],nt[264],q[40]; uint32_t pe,sz; unsigned k,n,opt;
    if(!mr(s->im,0,dos,sizeof(dos)) || u16(dos)!=0x5a4d) return 0;
    pe=u32(dos+60);
    if(!mr(s->im,pe,nt,sizeof(nt)) || u32(nt)!=0x4550 || u16(nt+4)!=0x8664 || u16(nt+24)!=0x20b) return 0;
    n=u16(nt+6); opt=u16(nt+20);
    if(!n || n>96 || opt<240 || opt>4096 || u32(nt+80)!=s->im->size || pe>UINT32_MAX-24-opt) return 0;
    for(k=0;k<n;++k) {
        uint32_t pos=pe+24+opt+40*k;
        if(pos<pe || !mr(s->im,pos,q,sizeof(q))) return 0;
        sz=u32(q+8); if(!sz) sz=u32(q+16);
        if(u32(q+12)>s->im->size || sz>s->im->size-u32(q+12)) return 0;
        s->sec[k].rva=u32(q+12); s->sec[k].size=sz; s->sec[k].flags=u32(q+36);
    }
    s->nsec=n; return 1;
}
unsigned dg_radial_inventory_resolve(const dg_radial_inventory_image *im,dg_radial_inventory_anchors *out) {
    scan s; unsigned k; int pass;
    if(!out) return 0; memset(out,0,sizeof(*out));
    if(!im || !im->base || im->size<4096 || im->size>64u*1024u*1024u || im->size>UINT64_MAX-im->base) return 0;
    memset(&s,0,sizeof(s));s.im=im;
    if(!sections(&s)) return 0;
    for(pass=0;pass<3;++pass) for(k=0;k<s.nsec;++k) {
        section *q=&s.sec[k]; unsigned char *b; size_t p;
        size_t need=pass==0 ? 52u : pass==1 ? 96u : 15u;
        if(!(q->flags&0x20000000u) || !q->size) continue;
        b=(unsigned char *)calloc(q->size,1);
        if(!b) return 0;
        for(p=0;p<q->size;p+=4096) {
            size_t n=q->size-p; if(n>4096)n=4096;
            /* Unknown executable bytes may hide another complete match. */
            if(!mr(im,q->rva+(uint32_t)p,b+p,n)) { free(b);return 0; }
        }
        for(p=0;p+need<=q->size;++p) {
            if(b[p]==0x48) inspect(&s,b+p,q->rva+(uint32_t)p,pass);
        }
        free(b);
    }
    out->module_base=im->base;out->module_size=im->size;
    if(s.setters==1 && s.wg==1 && s.ig==1) {
        out->valid_bits|=DG_RINV_INVENTORY;out->linkvar_slot=s.set_slot;
        out->weapons_slot=s.wp;out->items_slot=s.ip;
        memcpy(out->set_offset,s.offsets,sizeof(s.offsets));
        out->weapon_slots=(s.offsets[0][1]-s.offsets[0][0])/2;
        out->item_slots=(s.offsets[0][3]-s.offsets[0][2])/2;
    }
    if(s.actors==1 && s.desireds==2 && s.desired_slot) {
        if(out->linkvar_slot && out->linkvar_slot!=s.desired_slot) { memset(out,0,sizeof(*out));return 0; }
        out->valid_bits|=DG_RINV_DESIRED|DG_RINV_ACTOR;out->linkvar_slot=s.desired_slot;
        out->player_slot=s.player;out->actor_weapon_offset=s.weaponoff;out->actor_item_offset=s.itemoff;
    }
    return out->valid_bits;
}
unsigned dg_radial_inventory_sample(const dg_radial_inventory_image *im,const dg_radial_inventory_anchors *a,dg_radial_inventory_snapshot *out) {
    uint64_t link=0,player=0,wp=0,ip=0,again; int16_t desired[2]; int actorw,actori; int row=-1;
    if(!out) return 0;memset(out,0,sizeof(*out));
    if(!im || !a || !im->approved_identity || a->module_base!=im->base || a->module_size!=im->size) return 0;
    if((a->valid_bits&(DG_RINV_INVENTORY|DG_RINV_DESIRED)) && !rd(im,a->linkvar_slot,&link,8)) return 0;
    if(link && (a->valid_bits&DG_RINV_DESIRED) && link<=UINT64_MAX-0x108 && rd(im,link+0x104,desired,sizeof(desired))) {
        out->desired_weapon=desired[0];out->desired_item=desired[1];out->valid_bits|=DG_RINV_DESIRED;
    }
    if(link && (a->valid_bits&DG_RINV_INVENTORY) && a->weapon_slots>0 && a->weapon_slots<=DG_RINV_MAX_SLOTS &&
       a->item_slots>0 && a->item_slots<=DG_RINV_MAX_SLOTS && rd(im,a->weapons_slot,&wp,8) && rd(im,a->items_slot,&ip,8)) {
        int k;for(k=0;k<2;++k) if(link<=UINT64_MAX-a->set_offset[k][3] && wp==link+a->set_offset[k][0] && ip==link+a->set_offset[k][2]) row=k;
        if(row>=0 && rd(im,wp,out->weapons,a->weapon_slots*2) && rd(im,ip,out->items,a->item_slots*2)) {
            out->weapon_slots=a->weapon_slots;out->item_slots=a->item_slots;
            out->inventory_identity=link;out->weapons_identity=wp;out->items_identity=ip;out->valid_bits|=DG_RINV_INVENTORY;
        }
    }
    if((a->valid_bits&DG_RINV_ACTOR) && rd(im,a->player_slot,&player,8) && player &&
       player<=UINT64_MAX-a->actor_item_offset && rd(im,player+a->actor_weapon_offset,&actorw,4) && rd(im,player+a->actor_item_offset,&actori,4)) {
        out->actor_weapon=actorw;out->actor_item=actori;out->player_identity=player;out->valid_bits|=DG_RINV_ACTOR;
    }
    /* Detect simple pointer changes during read; caller still owns sequencing.
     * ABA and concurrent mutation of contents cannot be proven absent here. */
    if((out->valid_bits&(DG_RINV_DESIRED|DG_RINV_INVENTORY)) && (!rd(im,a->linkvar_slot,&again,8) || again!=link)) goto changed;
    if((out->valid_bits&DG_RINV_INVENTORY) && (!rd(im,a->weapons_slot,&again,8) || again!=wp || !rd(im,a->items_slot,&again,8) || again!=ip)) goto changed;
    if((out->valid_bits&DG_RINV_ACTOR) && (!rd(im,a->player_slot,&again,8) || again!=player)) goto changed;
    return out->valid_bits;
changed:
    memset(out,0,sizeof(*out));return 0;
}
