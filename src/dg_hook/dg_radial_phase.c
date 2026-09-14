#include "dg_radial_phase.h"
#include <stdlib.h>
#include <string.h>
typedef struct {uint32_t r,n,f;} sec;
typedef struct {const dg_radial_inventory_image *im;sec s[96];unsigned ns;uint32_t pd,pn;} pe;
static uint16_t u16(const unsigned char *p){return (uint16_t)(p[0]|p[1]<<8);}
static uint32_t u32(const unsigned char *p){return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24;}
static int readr(pe *p,uint32_t r,void *b,size_t n){return r<=p->im->size && n<=p->im->size-r && p->im->read(p->im->ctx,p->im->base+r,b,n)==1;}
static int region(pe *p,uint32_t r,size_t n,unsigned kind){unsigned k;
 for(k=0;k<p->ns;k++)if(r>=p->s[k].r && r-p->s[k].r<=p->s[k].n && n<=p->s[k].n-(r-p->s[k].r)){
  if(kind==1)return (p->s[k].f&0xe0000000u)==0x60000000u;
  if(kind==2)return (p->s[k].f&0xe0000000u)==0xc0000000u;
  return (p->s[k].f&0x40000000u)!=0;
 }return 0;
}
static uint64_t rel(pe *p,uint32_t r,unsigned end,const unsigned char *d,size_t n,unsigned kind){
 int64_t t=(int64_t)r+end+(int32_t)u32(d);
 return t>=0 && t<=UINT32_MAX && region(p,(uint32_t)t,n,kind)?p->im->base+(uint64_t)t:0;
}
static int headers(pe *p){unsigned char d[64],h[264],s[40];uint32_t r;unsigned k,opt;
 if(!readr(p,0,d,64)||u16(d)!=0x5a4d)return 0;r=u32(d+60);
 if(!readr(p,r,h,264)||u32(h)!=0x4550||u16(h+4)!=0x8664||u16(h+24)!=0x20b)return 0;
 p->ns=u16(h+6);opt=u16(h+20);
 if(!p->ns||p->ns>96||opt<240||opt>4096||r>UINT32_MAX-24-opt-96*40||u32(h+80)!=p->im->size||u32(h+132)<4)return 0;
 p->pd=u32(h+160);p->pn=u32(h+164);
 if(!p->pn||p->pn%12||p->pn>12u*200000u)return 0;
 for(k=0;k<p->ns;k++) {unsigned j;
  if(!readr(p,r+24+opt+40*k,s,40))return 0;
  p->s[k].r=u32(s+12);p->s[k].n=u32(s+8);p->s[k].f=u32(s+36);
  if(!p->s[k].n||p->s[k].r>p->im->size||p->s[k].n>p->im->size-p->s[k].r)return 0;
  for(j=0;j<k;j++)if(p->s[k].r<(uint64_t)p->s[j].r+p->s[j].n && p->s[j].r<(uint64_t)p->s[k].r+p->s[k].n)return 0;
 }return region(p,p->pd,p->pn,0);
}
static int eq(const unsigned char*b,unsigned o,const char*v,size_t n){return !memcmp(b+o,v,n);}
static uint64_t call(pe*p,uint32_t r,const unsigned char*b,unsigned o){return b[o]==0xe8?rel(p,r,o+5,b+o+1,1,1):0;}
static int helpers(pe*p,dg_radial_phase_anchors*a){unsigned char b[18];uint64_t slot;unsigned k;
 for(k=0;k<2;k++){
  uint64_t t=k?a->flags2_helper:a->flags_helper;
  if(t<p->im->base||t-p->im->base>UINT32_MAX||!region(p,(uint32_t)(t-p->im->base),18,1)||!readr(p,(uint32_t)(t-p->im->base),b,18))return 0;
  if(!eq(b,0,"\x48\x8b\x05",3)||!eq(b,7,"\x48\x8b\x80",3)||!eq(b,14,"\x48\x23\xc1\xc3",4))return 0;
  slot=rel(p,(uint32_t)(t-p->im->base),7,b+3,8,2);if(slot!=a->player_slot)return 0;
  if(!u32(b+10)||u32(b+10)>0x10000||u32(b+10)%8)return 0;
  if(k)a->flags2_offset=u32(b+10);else a->flags_offset=u32(b+10);
 }
 if(a->flags_offset==a->flags2_offset)return 0;
 if(!region(p,(uint32_t)(a->status_helper-p->im->base),11,1)||!readr(p,(uint32_t)(a->status_helper-p->im->base),b,11)||!eq(b,0,"\x48\x8b\x05",3)||!eq(b,7,"\x48\x23\xc1\xc3",4))return 0;
 a->status_slot=rel(p,(uint32_t)(a->status_helper-p->im->base),7,b+3,8,2);
 return a->status_slot!=0;
}
static int bodies(pe*p,uint32_t wr,uint32_t ir,const dg_radial_inventory_anchors*i,dg_radial_phase_anchors*a){
 unsigned char w[0x262],b[0x202];uint64_t q;
 if(!readr(p,wr,w,sizeof(w))||!readr(p,ir,b,sizeof(b)))return 0;
 if(!eq(w,0,"\x48\x89\x5c\x24\x08\x57\x48\x83\xec\x20\x48\x8b\xd9\xb9\x01\0\0\0\xe8",19)||
 !eq(b,0,"\x48\x89\x6c\x24\x10\x48\x89\x74\x24\x18\x57\x48\x83\xec\x20\x48\x8b\xf9\xb9\x01\0\0\0\xe8",24))return 0;
 a->status_helper=call(p,wr,w,0x12);
 if(!a->status_helper||call(p,ir,b,0x17)!=a->status_helper)return 0;
 if(!eq(w,0xf7,"\x8b\x0d",2)||!eq(w,0xfd,"\x48\x8b\x15",3)||!eq(w,0x108,"\x0f\xbf\x82\x04\x01\0\0\x44\x8b\x83",10)||u32(w+0x112)!=i->actor_weapon_offset)return 0;
 if(!eq(b,0xfd,"\x83\x3d",2)||b[0x103]!=0||!eq(b,0x104,"\x48\x8b\x0d",3)||!eq(b,0x10d,"\x0f\xbf\x81\x06\x01\0\0\x8b\x97",9)||u32(b+0x116)!=i->actor_item_offset)return 0;
 if(!eq(w,0x116,"\x41\x3b\xc0",3)||!eq(b,0x11a,"\x3b\xc2",2))return 0;
 if(rel(p,wr,0x104,w+0x100,8,2)!=i->linkvar_slot||rel(p,ir,0x10b,b+0x107,8,2)!=i->linkvar_slot)return 0;
 a->changed_weapon=rel(p,wr,0xfd,w+0xf9,4,2);a->changed_item=rel(p,ir,0x104,b+0xff,4,2);
 if(!a->changed_weapon||!a->changed_item||a->changed_item!=a->changed_weapon+4)return 0;
 if(!eq(w,0x12f,"\x89\x0d",2)||rel(p,wr,0x135,w+0x131,4,2)!=a->changed_weapon||
 !eq(b,0x12c,"\xc7\x05",2)||u32(b+0x132)!=1||rel(p,ir,0x136,b+0x12e,4,2)!=a->changed_item)return 0;
 if(!eq(w,0x135,"\x0f\xbe\x05",3)||!eq(w,0x14d,"\xc6\x05",2)||w[0x153]!=255||
 !eq(b,0x136,"\x0f\xbe\x05",3)||!eq(b,0x14f,"\xc6\x05",2)||b[0x155]!=255)return 0;
 a->scenario_weapon=rel(p,wr,0x13c,w+0x138,1,2);a->scenario_item=rel(p,ir,0x13d,b+0x139,1,2);
 if(!a->scenario_weapon||a->scenario_item!=a->scenario_weapon+1||rel(p,wr,0x154,w+0x14f,1,2)!=a->scenario_weapon||rel(p,ir,0x156,b+0x151,1,2)!=a->scenario_item)return 0;
 a->flags_helper=call(p,wr,w,0xb4);a->flags2_helper=call(p,wr,w,0x16a);
 q=call(p,ir,b,0x8d);
 if(!a->flags_helper||!a->flags2_helper||q!=a->flags_helper||call(p,ir,b,0x19b)!=a->flags2_helper)return 0;
 a->player_slot=i->player_slot;
 return helpers(p,a);
}
int dg_radial_phase_resolve(const dg_radial_inventory_image*im,const dg_radial_inventory_anchors*inv,dg_radial_phase_anchors*out){
 pe p;dg_radial_phase_anchors a;unsigned char row[12],head[24];uint32_t wr=0,ir=0,prev=0,k;unsigned wc=0,ic=0,chains=0;
 if(!out)return 0;memset(out,0,sizeof(*out));memset(&a,0,sizeof(a));memset(&p,0,sizeof(p));p.im=im;
 if(!im||!im->approved_identity||!im->read||!im->base||im->size>UINT32_MAX||im->size>UINT64_MAX-im->base||!inv||inv->valid_bits!=7||inv->module_base!=im->base||inv->module_size!=im->size||!headers(&p))return 0;
 /* Only unwind-table function entries are candidates; no mid-instruction
    prologue scan. Sorted nonoverlapping entries and readable code required. */
 for(k=0;k<p.pn;k+=12){uint32_t r,e,u;
  if(!readr(&p,p.pd+k,row,12))return 0;r=u32(row);e=u32(row+4);u=u32(row+8);
  if(r>=e||r<prev||!region(&p,r,e-r,1)||!region(&p,u,4,0))return 0;prev=e;
  if(e-r<24)continue;if(!readr(&p,r,head,24))return 0;
  if(e-r==0x262&&eq(head,0,"\x48\x89\x5c\x24\x08\x57\x48\x83\xec\x20\x48\x8b\xd9",13)){wr=r;wc++;}
  if(e-r==0x202&&eq(head,0,"\x48\x89\x6c\x24\x10\x48\x89\x74\x24\x18\x57\x48\x83\xec\x20\x48\x8b\xf9",18)){ir=r;ic++;}
 }
 if(wc!=1||ic!=1||!bodies(&p,wr,ir,inv,&a))return 0;
 /* Every executable byte is inspected for the structural phase and incoming
    direct calls. Conservative byte scan may refuse coincidental E8 data. */
 {unsigned s;unsigned wcall=0,icall=0;
 for(s=0;s<p.ns;s++)if(p.s[s].f&0x20000000u){unsigned char*buf;uint32_t n=p.s[s].n,j;
  buf=(unsigned char*)malloc(n);if(!buf)return 0;
  if(!readr(&p,p.s[s].r,buf,n)){free(buf);return 0;}
  for(j=0;j+5<=n;j++){
   uint32_t r=p.s[s].r+j;
   if(buf[j]==0xe8){uint64_t t=call(&p,r,buf+j,0);if(t==im->base+wr)wcall++;if(t==im->base+ir)icall++;}
   if(j+39>n||!eq(buf+j,0,"\x48\x89\x0d",3)||!eq(buf+j,7,"\x48\x8b\xcf\xe8",4)||!eq(buf+j,15,"\x48\x8b\xcf\xe8",4)||!eq(buf+j,23,"\xb9\x32\0\0\0\xe8",6))continue;
   if(call(&p,r,buf+j,10)!=im->base+wr||call(&p,r,buf+j,18)!=im->base+ir)continue;
   a.no_use_item_slot=rel(&p,r,7,buf+j+3,8,2);a.clear_flags_helper=call(&p,r,buf+j,28);
   if(!a.no_use_item_slot||!a.clear_flags_helper){free(buf);return 0;}
   a.weapon_call=im->base+r+10;a.item_call=im->base+r+18;a.return_address=a.weapon_call+5;chains++;
  }free(buf);
 }if(chains!=1||wcall!=1||icall!=1)return 0;}
 for(k=0;k<p.pn;k+=12){if(!readr(&p,p.pd+k,row,12))return 0;
  if(im->base+u32(row)<=a.weapon_call-10&&im->base+u32(row+4)>=a.item_call+15){a.caller_begin=im->base+u32(row);a.caller_end=im->base+u32(row+4);}
 }
 if(!a.caller_begin)return 0;
 /* The trailing call must really clear the same second player-flag field,
    not just land at some executable address. */
 {unsigned char c[18];uint32_t r=(uint32_t)(a.clear_flags_helper-im->base);
  if(!region(&p,r,18,1)||!readr(&p,r,c,18)||!eq(c,0,"\x48\x8b\x05",3)||
     !eq(c,7,"\x48\xf7\xd1\x48\x21\x88",6)||c[17]!=0xc3||u32(c+13)!=a.flags2_offset||
     rel(&p,r,7,c+3,8,2)!=a.player_slot)return 0;
 }
 a.module_base=im->base;a.weapon_entry=im->base+wr;a.weapon_end=a.weapon_entry+0x262;
 a.item_entry=im->base+ir;a.item_end=a.item_entry+0x202;a.patch_bytes=5;
 memcpy(a.expected_entry,"\x48\x89\x5c\x24\x08",5);a.valid=1;*out=a;return 1;
}
