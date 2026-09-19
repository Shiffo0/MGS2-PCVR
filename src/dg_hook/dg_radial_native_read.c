#include "dg_radial_native_read.h"
#include <stdlib.h>
#include <string.h>
typedef struct rn_section { uint32_t start,size,flags; } rn_section;
typedef struct rn_image {
    const dg_radial_inventory_image *im;
    rn_section sections[96]; unsigned count;
    unsigned char *bytes;
} rn_image;
typedef struct rn_slice { int id; uint32_t type; const char *name; } rn_slice;
static const rn_slice weapon_slice[]={
    {0,0x00008001,"None"},
    {1,0x0431408e,"M9"},
    {2,0x0431508e,"USP"},
    {3,0x0431508e,"SOCOM"},
    {4,0x0000406a,"PSG1"},
    {5,0x0834408a,"RGB6"},
    {6,0x08084682,"NIKITA"},
    {7,0x00004060,"STINGER"},
    {8,0x03820401,"CLAYMORE"},
    {9,0x03800401,"C4"},
    {10,0x02122103,"CHAFF.G"},
    {11,0x02122103,"STUN.G"},
    {12,0x00008062,"D.MIC"},
    {13,0x00428000,"HF.BLADE"},
    {14,0x00008062,"COOLANT"},
    {15,0x0834581e,"AKS-74U"},
    {16,0x02122103,"MAGAZINE"},
    {17,0x02122103,"GRENADE"},
    {18,0x0834581e,"M4"},
    {19,0x0000406a,"PSG1-T"},
    {21,0x03820401,"BOOK"}
};
static const rn_slice item_slice[]={
    {0,0x00000000,"None"},
    {1,0x00000000,"RATION"},
    {3,0x00000000,"MEDICINE"},
    {4,0x00000000,"BANDAGE"},
    {5,0x00000000,"PENTAZEMIN"},
    {6,0x00008100,"B.D.U"},
    {7,0x00008000,"B.ARMOR"},
    {8,0x00000000,"STEALTH"},
    {9,0x00000000,"MINE.D"},
    {10,0x00000000,"SENSOR A"},
    {11,0x00008000,"SENSOR B"},
    {12,0x00008000,"N.V.G"},
    {13,0x00008000,"THERM.G"},
    {14,0x00008024,"SCOPE"},
    {15,0x00008024,"DG.CAMERA"},
    {16,0x00000128,"BOX 1"},
    {17,0x00008000,"CIGS"},
    {18,0x00000000,"CARD"},
    {19,0x00000000,"SHAVER"},
    {20,0x00000000,"PHONE"},
    {21,0x00008024,"CAMERA"},
    {22,0x00000128,"BOX 2"},
    {23,0x00000128,"BOX 3"},
    {24,0x00000128,"WET BOX"},
    {25,0x00000000,"AP SENSR"},
    {26,0x00000128,"BOX 4"},
    {27,0x00000128,"BOX 5"},
    {28,0x00000000,"RAZOR"},
    {29,0x00008000,"SCM.SUPR"},
    {30,0x00008000,"AK.SUPR"},
    {32,0x00008200,"BANDANA"},
    {33,0x00000000,"DOG TAGS"},
    {34,0x00000000,"MO DISC"},
    {35,0x00008000,"USP.SUPR"},
    {36,0x00008200,"SP.WIG"},
    {37,0x00008000,"WIG A"},
    {38,0x00008000,"WIG B"}
};
static unsigned profile_count(int kind){return kind?(unsigned)(sizeof(item_slice)/sizeof(item_slice[0])):(unsigned)(sizeof(weapon_slice)/sizeof(weapon_slice[0]));}
static uint16_t rn16(const unsigned char *b){return (uint16_t)((unsigned)b[0]|((unsigned)b[1]<<8));}
static uint32_t rn32(const unsigned char *b){return (uint32_t)b[0]|((uint32_t)b[1]<<8)|((uint32_t)b[2]<<16)|((uint32_t)b[3]<<24);}
static uint64_t rn64(const unsigned char *b){return (uint64_t)rn32(b)|((uint64_t)rn32(b+4)<<32);}
static int read_exact(const dg_radial_inventory_image *im,uint64_t a,void *p,size_t n){
    return im && im->approved_identity && im->read && a && n && n<=UINT64_MAX-a && im->read(im->ctx,a,p,n)==1;
}
static int module_read(const dg_radial_inventory_image *im,uint64_t a,void *p,size_t n){
    return a>=im->base && a-im->base<=im->size && n<=im->size-(size_t)(a-im->base) && read_exact(im,a,p,n);
}
/* flags: 1 executable, 2 readable non-code, 3 writable non-code */
static int region(const rn_image *v,uint64_t a,size_t n,int kind){
    unsigned k;if(a<v->im->base || a-v->im->base>v->im->size)return 0;
    a-=v->im->base;
    for(k=0;k<v->count;++k){const rn_section *s=&v->sections[k];uint32_t f=s->flags;
        if(a<s->start || a-s->start>s->size || n>s->size-(size_t)(a-s->start))continue;
        if(kind==1)return (f&0x20000000u)!=0;
        if((f&0x60000000u)!=0x40000000u)return 0;
        return kind==2 || (f&0x80000000u)!=0;
    }return 0;
}
static uint64_t relative(const rn_image *v,uint32_t r,unsigned end,uint32_t displacement,size_t n,int kind){
    int64_t q=(int64_t)r+end+(int32_t)displacement;uint64_t a;
    if(q<0 || (uint64_t)q>v->im->size)return 0;
    a=v->im->base+(uint64_t)q;return region(v,a,n,kind)?a:0;
}
static int load_image(rn_image *v,const dg_radial_inventory_image *im){
    unsigned char dos[64],nt[264],sh[40];uint32_t pe;unsigned opt,n,k,j;
    memset(v,0,sizeof(*v));v->im=im;
    if(!im || !im->approved_identity || !im->base || im->size<4096 || im->size>64u*1024u*1024u || im->size>UINT64_MAX-im->base)return 0;
    if(!module_read(im,im->base,dos,sizeof(dos)) || rn16(dos)!=0x5a4d)return 0;
    pe=rn32(dos+60);
    if(pe>im->size || !module_read(im,im->base+pe,nt,sizeof(nt)) || rn32(nt)!=0x4550 || rn16(nt+4)!=0x8664 || rn16(nt+24)!=0x20b || rn32(nt+80)!=im->size)return 0;
    n=rn16(nt+6);opt=rn16(nt+20);if(!n || n>96 || opt<240 || opt>4096)return 0;
    for(k=0;k<n;++k){uint64_t at=(uint64_t)pe+24+opt+40*k;rn_section *s=&v->sections[k];
        if(!module_read(im,im->base+at,sh,sizeof(sh)))return 0;
        s->start=rn32(sh+12);s->size=rn32(sh+8);if(!s->size)s->size=rn32(sh+16);s->flags=rn32(sh+36);
        if(s->start>im->size || s->size>im->size-s->start)return 0;
        for(j=0;j<k;++j)if(s->size && v->sections[j].size && s->start<(uint64_t)v->sections[j].start+v->sections[j].size && v->sections[j].start<(uint64_t)s->start+s->size)return 0;
    }
    v->count=n;v->bytes=(unsigned char *)calloc(im->size,1);if(!v->bytes)return 0;
    for(k=0;k<n;++k){rn_section *s=&v->sections[k];size_t p;
        if(!(s->flags&0x60000000u))continue;
        for(p=0;p<s->size;p+=4096){size_t len=s->size-p;if(len>4096)len=4096;
            if(!module_read(im,im->base+s->start+p,v->bytes+s->start+p,len)){free(v->bytes);v->bytes=0;return 0;}
        }
    }return 1;
}
static int getter(const rn_image *v,uint64_t fn,uint64_t slot){
    const unsigned char *b;uint32_t r;if(!region(v,fn,15,1))return 0;r=(uint32_t)(fn-v->im->base);b=v->bytes+r;
    return !memcmp(b,"\x48\x8b\x05",3) && !memcmp(b+7,"\x48\x63\xd1\x0f\xbf\x04\x50\xc3",8) && relative(v,r,7,rn32(b+3),8,3)==slot;
}
static int actor_getter(const rn_image *v,uint64_t fn,const dg_radial_inventory_anchors *a,int item){
    const unsigned char *b;uint32_t r;if(!region(v,fn,20,1))return 0;r=(uint32_t)(fn-v->im->base);b=v->bytes+r;
    return !memcmp(b,"\x48\x8b\x05",3) && !memcmp(b+7,"\x48\x85\xc0\x75\x01\xc3\x8b\x80",8) && b[19]==0xc3 &&
       relative(v,r,7,rn32(b+3),8,3)==a->player_slot && rn32(b+15)==(item?a->actor_item_offset:a->actor_weapon_offset);
}
static int helper(const rn_image *v,uint64_t fn,unsigned source_count,uint64_t *table){
    const unsigned char *b;uint32_t r;if(!region(v,fn,63,1))return 0;r=(uint32_t)(fn-v->im->base);b=v->bytes+r;
    if(memcmp(b,"\x48\x63\xc1\x44\x8b\xc2\x48\x8d\x0d",9) ||
       memcmp(b+13,"\x48\x8b\x0c\xc1\x48\x85\xc9\x74\x20\x8b\x01\xff\xc0\x48\x63\xd0\x48\x83\xfa\x01\x7e\x13\xb8\x01\x00\x00\x00\x44\x39\x04\x81\x74\x0b\x48\xff\xc0\x48\x3b\xc2\x7c\xf2\x33\xc0\xc3\xb8\x01\x00\x00\x00\xc3",50))return 0;
    *table=relative(v,r,13,rn32(b+9),source_count*8,2);return *table!=0;
}
static int find_default(const rn_image *v,const dg_radial_inventory_anchors *inv,uint32_t r,size_t remain,int item,dg_radial_native_anchors *a){
    const unsigned char *b=v->bytes+r;uint64_t link,fn;
    if(item){
        if(remain<190 || b[189]!=0xc3 || memcmp(b,"\x48\x89\x5c\x24\x08\x57\x48\x83\xec\x20\x48\x8b\x05",13) ||
           memcmp(b+21,"\x0f\xbf\x88\x06\x01\x00\x00\x66\x85\xc9",10) || b[33]!=0xe8 ||
           memcmp(b+84,"\x48\x8b\x0d",3) || memcmp(b+91,"\x48\x0b\x0d",3) ||
           memcmp(b+111,"\x48\x8d\x15",3) || memcmp(b+121,"\x85\x0d",2))return 0;
        link=relative(v,r,17,rn32(b+13),8,3);fn=relative(v,r,38,rn32(b+34),15,1);
        if(link!=inv->linkvar_slot || !getter(v,fn,inv->items_slot))return 0;
        a->dynamic_mask[1]=relative(v,r,91,rn32(b+87),8,3);a->scenario_mask[1]=relative(v,r,98,rn32(b+94),8,3);
        a->types[1]=relative(v,r,118,rn32(b+114),DG_RN_ITEMS*4,2);a->type_mask[1]=relative(v,r,127,rn32(b+123),4,3);
    }else{
        if(remain<260 || b[259]!=0xc3 || memcmp(b,"\x48\x89\x5c\x24\x08\x48\x89\x74\x24\x10\x57\x48\x83\xec\x20\x4c\x8b\x05",18) ||
           memcmp(b+22,"\x48\x8d\x35",3) || memcmp(b+33,"\x41\x0f\xbf\x80\x04\x01\x00\x00",8) || b[48]!=0xe8 ||
           memcmp(b+95,"\xf7\x04\x86\x00\x40\x00\x00",7) || memcmp(b+151,"\x48\x8b\x0d",3) ||
           memcmp(b+158,"\x48\x0b\x0d",3) || memcmp(b+181,"\x85\x0d",2))return 0;
        link=relative(v,r,22,rn32(b+18),8,3);fn=relative(v,r,53,rn32(b+49),15,1);
        if(link!=inv->linkvar_slot || !getter(v,fn,inv->weapons_slot))return 0;
        a->dynamic_mask[0]=relative(v,r,158,rn32(b+154),8,3);a->scenario_mask[0]=relative(v,r,165,rn32(b+161),8,3);
        a->types[0]=relative(v,r,29,rn32(b+25),DG_RN_WEAPONS*4,2);a->type_mask[0]=relative(v,r,187,rn32(b+183),4,3);
    }
    a->default_function[item]=v->im->base+r;
    return a->dynamic_mask[item] && a->scenario_mask[item] && a->types[item] && a->type_mask[item];
}
static int find_nouse(const rn_image *v,const dg_radial_inventory_anchors *inv,uint32_t r,dg_radial_native_anchors *a){
    const unsigned char *b=v->bytes+r;uint64_t dyn,scn,types,mask,f0,f1,table0,table1;int kind;
    if(b[175]!=0xc3 || memcmp(b,"\x48\x89\x5c\x24\x08\x57\x48\x83\xec\x20\x48\x63\xd9\xe8",14) ||
       memcmp(b+18,"\x8b\xf8\xe8",3) || memcmp(b+25,"\x8b\xd3\x8b\xc8\xe8",5) ||
       memcmp(b+38,"\x8b\xd3\x8b\xcf\xe8",5) || memcmp(b+51,"\x48\x8b\x05",3) ||
       memcmp(b+58,"\x8b\xd3\x0f\xbf\x88\x04\x01\x00\x00\xe8",10) ||
       memcmp(b+76,"\x48\x8b\x05",3) || memcmp(b+83,"\x8b\xd3\x0f\xbf\x88\x06\x01\x00\x00\xe8",10) ||
       memcmp(b+111,"\x48\x85\x05",3) || memcmp(b+120,"\x48\x85\x05",3) ||
       memcmp(b+129,"\x48\x8d\x0d",3) || memcmp(b+136,"\x8b\x04\x99\x85\x05",5))return -1;
    if(!actor_getter(v,relative(v,r,18,rn32(b+14),20,1),inv,1) || !actor_getter(v,relative(v,r,25,rn32(b+21),20,1),inv,0) ||
       relative(v,r,58,rn32(b+54),8,3)!=inv->linkvar_slot || relative(v,r,83,rn32(b+79),8,3)!=inv->linkvar_slot)return -1;
    dyn=relative(v,r,118,rn32(b+114),8,3);scn=relative(v,r,127,rn32(b+123),8,3);
    types=relative(v,r,136,rn32(b+132),4,2);mask=relative(v,r,145,rn32(b+141),4,3);
    for(kind=0;kind<2;++kind)if(types==a->types[kind] && dyn==a->dynamic_mask[kind] && scn==a->scenario_mask[kind] && mask==a->type_mask[kind])break;
    if(kind==2)return -1;
    f0=relative(v,r,34,rn32(b+30),63,1);f1=relative(v,r,47,rn32(b+43),63,1);
    if(!f0 || !f1 || f0==f1 || relative(v,r,72,rn32(b+68),63,1)!=f0 || relative(v,r,97,rn32(b+93),63,1)!=f1 ||
       !helper(v,f0,DG_RN_WEAPONS,&table0) || !helper(v,f1,DG_RN_ITEMS,&table1))return -1;
    a->no_use_function[kind]=v->im->base+r;a->cross_table[kind][0]=table0;a->cross_table[kind][1]=table1;
    return kind;
}
static int name_at(const rn_image *v,uint64_t table,int id,const char *want){
    uint64_t p;size_t n=strlen(want)+1;
    if(!region(v,table+(unsigned)id*8,8,2))return 0;
    p=rn64(v->bytes+(size_t)(table-v->im->base)+(unsigned)id*8);
    return region(v,p,n,2) && !memcmp(v->bytes+(size_t)(p-v->im->base),want,n);
}
static int names_match(const rn_image *v,uint64_t table,int kind){
    const rn_slice *s=kind?item_slice:weapon_slice;unsigned k;
    if(!region(v,table,(kind?DG_RN_ITEMS:DG_RN_WEAPONS)*8,2) || !name_at(v,table,0,"None"))return 0;
    for(k=0;k<profile_count(kind);++k)if(!name_at(v,table,s[k].id,s[k].name))return 0;
    return 1;
}
int dg_radial_native_resolve(const dg_radial_inventory_image *im,const dg_radial_inventory_anchors *inv,dg_radial_native_anchors *out){
    rn_image v;dg_radial_native_anchors a;unsigned k,dc[2]={0,0},nc[2]={0,0},names[2]={0,0};int pass,kind;
    if(!out)return 0;memset(out,0,sizeof(*out));memset(&a,0,sizeof(a));
    if(!im || !inv || inv->valid_bits!=7 || inv->module_base!=im->base || inv->module_size!=im->size || !load_image(&v,im))return 0;
    for(pass=0;pass<2;++pass)for(k=0;k<v.count;++k){rn_section *s=&v.sections[k];size_t p;
        if(!(s->flags&0x20000000u))continue;
        for(p=0;p<s->size;++p){uint32_t r=s->start+(uint32_t)p;size_t remain=s->size-p;
            if(v.bytes[r]!=0x48)continue;
            if(pass==0){for(kind=0;kind<2;++kind)if(find_default(&v,inv,r,remain,kind,&a))++dc[kind];}
            else if(remain>=176 && (kind=find_nouse(&v,inv,r,&a))>=0)++nc[kind];
        }
    }
    if(dc[0]!=1 || dc[1]!=1 || nc[0]!=1 || nc[1]!=1)goto done;
    for(k=0;k<v.count;++k){rn_section *s=&v.sections[k];size_t p;
        if((s->flags&0x60000000u)!=0x40000000u)continue;
        for(p=0;p+8<=s->size;p+=8){uint64_t at=im->base+s->start+p;
            if(!rn64(v.bytes+s->start+p))continue;
            for(kind=0;kind<2;++kind)if(names_match(&v,at,kind)){++names[kind];a.names[kind]=at;}
        }
    }
    if(names[0]!=1 || names[1]!=1)goto done;
    for(kind=0;kind<2;++kind){const rn_slice *s=kind?item_slice:weapon_slice;unsigned j;
        for(j=0;j<profile_count(kind);++j)if(rn32(v.bytes+(size_t)(a.types[kind]-im->base)+s[j].id*4)!=s[j].type)goto done;
    }
    a.valid=1;a.module_base=im->base;a.module_size=im->size;*out=a;
done:free(v.bytes);return out->valid;
}
const char *dg_radial_native_label(int kind,int id){
    const rn_slice *s;unsigned k;if(kind<0 || kind>1)return 0;s=kind?item_slice:weapon_slice;
    for(k=0;k<profile_count(kind);++k)if(s[k].id==id)return s[k].name;return 0;
}
static int cross_mask(const dg_radial_inventory_image *im,uint64_t table,int id,unsigned target_count,uint64_t *mask){
    uint64_t p,again;int32_t count,values[DG_RN_ITEMS];unsigned k;
    if(!module_read(im,table+(unsigned)id*8,&p,8))return 0;
    if(!p){*mask=0;return 1;}
    if(!module_read(im,p,&count,4) || count<0 || count>(int32_t)target_count)return 0;
    if(count && !module_read(im,p+4,values,(size_t)count*4))return 0;
    *mask=0;for(k=0;k<(unsigned)count;++k){if(values[k]<0 || values[k]>=(int)target_count)return 0;*mask|=UINT64_C(1)<<values[k];}
    if(!module_read(im,table+(unsigned)id*8,&again,8) || again!=p)return 0;
    return 1;
}
int dg_radial_native_sample(const dg_radial_inventory_image *im,const dg_radial_inventory_anchors *inv,const dg_radial_native_anchors *a,int phase_fresh,dg_radial_native_snapshot *out){
    dg_radial_native_snapshot o;dg_radial_inventory_snapshot after;uint64_t forbidden[2]={0,0};int ids[2][2],kind,from,j;unsigned k;
    if(!out)return 0;memset(out,0,sizeof(*out));memset(&o,0,sizeof(o));
    if(!im || !inv || !a || !a->valid || a->module_base!=im->base || a->module_size!=im->size || dg_radial_inventory_sample(im,inv,&o.inventory)!=7)return 0;
    if(o.inventory.weapon_slots<DG_RN_WEAPONS || o.inventory.item_slots<DG_RN_ITEMS)return 0;
    ids[0][0]=o.inventory.actor_weapon;ids[0][1]=o.inventory.desired_weapon;
    ids[1][0]=o.inventory.actor_item;ids[1][1]=o.inventory.desired_item;
    for(kind=0;kind<2;++kind)for(j=0;j<2;++j)if(ids[kind][j]<0 || ids[kind][j]>=(kind?DG_RN_ITEMS:DG_RN_WEAPONS))return 0;
    if(!module_read(im,a->types[0],o.weapon_types,sizeof(o.weapon_types)) || !module_read(im,a->types[1],o.item_types,sizeof(o.item_types)))return 0;
    for(kind=0;kind<2;++kind){const rn_slice *s=kind?item_slice:weapon_slice;uint32_t *types=kind?o.item_types:o.weapon_types;
        if(!module_read(im,a->dynamic_mask[kind],&o.dynamic_mask[kind],8) || !module_read(im,a->scenario_mask[kind],&o.scenario_mask[kind],8) || !module_read(im,a->type_mask[kind],&o.type_mask[kind],4))return 0;
        for(from=0;from<2;++from)for(j=0;j<2;++j){uint64_t m;
            if(!cross_mask(im,a->cross_table[kind][from],ids[from][j],kind?DG_RN_ITEMS:DG_RN_WEAPONS,&m))return 0;forbidden[kind]|=m;
        }
        for(k=0;k<profile_count(kind);++k){int id=s[k].id;int count=kind?o.inventory.items[id]:o.inventory.weapons[id];uint64_t bit=UINT64_C(1)<<id;
            if(types[id]!=s[k].type)return 0;
            if(id!=0 && (count<0 || (count==0 && (kind || !(types[id]&0x4000u)))))continue;
            if((forbidden[kind]|o.dynamic_mask[kind]|o.scenario_mask[kind])&bit || (types[id]&o.type_mask[kind]))continue;
            if(phase_fresh){if(kind)o.eligible_items|=bit;else o.eligible_weapons|=bit;}
        }
    }
    if(dg_radial_inventory_sample(im,inv,&after)!=7 || memcmp(&after,&o.inventory,sizeof(after)))return 0;
    o.valid=1;o.eligibility_valid=phase_fresh!=0;*out=o;return 1;
}
