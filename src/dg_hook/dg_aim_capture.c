#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "dg_aim_capture.h"
#include "dg_aim_probe.h"
#include "dg_ik.h"
#include "dg_weapon_aim.h"

#define CAPACITY 4096u
#define MODELS 16u
enum { CAP_OK=0, CAP_ANCHOR=100, CAP_READ, CAP_SELECTION,
       CAP_MODEL_COUNT, CAP_CHANGED, CAP_HAND };
typedef struct { unsigned mask; uint64_t value[13]; } COOLANT_DIAG;
typedef struct {
    COOLANT_DIAG coolant; int coolant_stable;
    uint64_t id, qpc, publication_aim_seq;
    int64_t aim_time;
    int view_kind, capture_result, probe_result;
    int raw_hand_valid,raw_model_count;
    MAT raw_hand,raw_models[MODELS];
    DG_XR_RAW_POSE right_grip,left_grip;
    const char *refusal; /* static condition name, no runtime pointers */
    uint64_t subobject, subobjs, arm, hand, actor;
    int weapon_id;
    unsigned int count;
    uint64_t model_ptr[MODELS], bp_mesh[MODELS], model_owner[MODELS];
    int32_t bp_unit[MODELS];
    int32_t parent[MODELS];
    MAT model_world[MODELS], projection;
    MAT camera_full, arm_root_full, hand_full, shoulder_full;
    int transforms_valid;
    double trigger_value;
    DG_AIM_PROBE_INPUT input;
    DG_AIM_PROBE_SAMPLE sample;
} CAP_RECORD;
static CAP_RECORD ring[CAPACITY], dump_rows[CAPACITY];
#define NATIVE_WEAPONS 8
#define NATIVE_ROWS 128
static CAP_RECORD native_ring[NATIVE_WEAPONS][NATIVE_ROWS];
static uint64_t native_counts[NATIVE_WEAPONS],native_stamp[NATIVE_WEAPONS],native_total,native_last_capture;
static SRWLOCK ring_lock=SRWLOCK_INIT;
static SRWLOCK dump_lock=SRWLOCK_INIT;
static volatile LONG enabled, busy_drops;
static uint64_t total;
static volatile LONG native_enabled;

/* RPM copies only; unreadable or partial spans fail instead of raising a game
   exception. All reads have fixed/bounded sizes, even for corrupted objects. */
static int read_at(uint64_t p, void *out, size_t n) {
    SIZE_T got=0;
    if(!out || !n || n>4096 || p<0x10000 || p>0x00007fffffffffffULL-n) return 0;
    return ReadProcessMemory(GetCurrentProcess(),(const void *)(uintptr_t)p,
                             out,n,&got) && got==n;
}
static int ptr_at(uint64_t p, uint64_t *out) {
    return read_at(p,out,8) && *out>=0x10000 && *out<0x00007ffffffff000ULL && !(*out&7);
}
static int same_ptr(uint64_t p, uint64_t old) {
    uint64_t now; return ptr_at(p,&now) && now==old;
}
/* Supplemental observation only: failure never changes pistol selection or
   the production arm solve. Re-reading detects some changes, not global
   atomicity. Layout is the same joint-0/4/6 matrix array the bridge uses. */
static int full_transforms(CAP_RECORD *r) {
    uint64_t objs;
    MAT now;
    MAT *dst[4]={&r->arm_root_full,&r->shoulder_full,&r->hand_full,&r->camera_full};
    const unsigned joint[3]={0,4,6};
    int n,j,k;
    if(r->capture_result!=CAP_OK || !ptr_at(r->arm,&objs)) return 0;
    if(objs+0x110+6*0x180!=r->hand) return 0;
    for(n=0;n<4;n++) {
        double basis[3][3],q[4];
        if(n<3 && !read_at(objs+0x110+joint[n]*0x180,dst[n],sizeof(MAT))) return 0;
        for(j=0;j<4;j++) for(k=0;k<4;k++)
            if(!_finite(dst[n]->m[j][k])) return 0;
        for(j=0;j<3;j++) for(k=0;k<3;k++) basis[j][k]=dst[n]->m[j][k];
        if(!dg_ik_basis_quat(basis,q)) return 0;
        if(fabs(dst[n]->m[3][3]-1.0)>0.002) return 0;
        for(j=0;j<3;j++) if(fabs(dst[n]->m[j][3])>0.002) return 0;
    }
    for(n=0;n<3;n++)
        if(!read_at(objs+0x110+joint[n]*0x180,&now,sizeof now) ||
           memcmp(&now,dst[n],sizeof now)) return 0;
    return same_ptr(r->arm,objs);
}
/* The two independent RIP-relative global writers/readers are checked each
   observation. No unchecked fixed global from an older binary is accepted. */
static int anchors(uint64_t base,uint64_t *subglobal,uint64_t *armglobal) {
    unsigned char a[7], b[7], stride[4], unit[10];
    int32_t d;
    if(!read_at(base+0x4b0cbc,a,7) || memcmp(a,"\x48\x89\x35",3) ||
       !read_at(base+0x52a417,b,7) || memcmp(b,"\x48\x8b\x05",3) ||
       !read_at(base+0x67ded7,stride,4) || memcmp(stride,"\x48\xc1\xe1\x07",4) ||
       !read_at(base+0x52a432,unit,10) || memcmp(unit,"\xc7\x83\xb0\x0b\0\0\x06\0\0\0",10)) return 0;
    memcpy(&d,a+3,4); *subglobal=base+0x4b0cc3+d;
    memcpy(&d,b+3,4); *armglobal=base+0x52a41e+d;
    /* Cross-check this known revision's relative layout as well as opcodes. */
    return *subglobal==base+0x17df688 && *armglobal==base+0x17df698;
}
/* Retail 2.1.0.0 layouts, independently checked against the stored image.
   The spray actor publishes weapon (+0x60), not weapon_sub (+0xa0).
   See weapons_run1_retail.py for constructor/body/unit/global evidence. */
typedef struct { unsigned object, body, unit, effect, trigger; } CAP_LAYOUT;
static int layout(uint64_t base, int weapon, uint64_t subobject, CAP_LAYOUT *out) {
    unsigned char code[14];
    if(!dg_weapon_hand_aim(weapon) && weapon!=12 && weapon!=20) return 0; /* observation only */
    out->object=0xa0; out->body=0x248; out->unit=0x2e8;
    out->effect=0x2c0; out->trigger=0;
    /* Native USP GetResources uses GM_InitObject for AKS/M4 (including
       suppressed AKS), not WeaponEfInitObject. actor+0x2c0 is unused there.
       Keep body/unit/root/owner/model guards; never fall back on read failure. */
    if(weapon==15 || weapon==18) out->effect=0;
    if(weapon==13) {
        unsigned char slots[21]; unsigned offset;
        if(!read_at(base+0x4c3a7f,slots,21) || memcmp(slots,
            "\x48\x89\xb3\x70\x03\0\0\x48\x89\xbb\x78\x03\0\0\x48\x89\xab\x80\x03\0\0",21))return 0;
        /* Normal and reversed subject models; reject combo/body-only roots. */
        for(offset=0xe0;offset<=0x120;offset+=0x40) {
            uint64_t actor=subobject-offset,body,unit,trigger;
            unsigned flags;
            if(read_at(actor+0x38c,&flags,4) && (flags&0x1001)==0x1001 &&
               ((flags&2)!=0)==(offset==0x120) &&
               ptr_at(actor+0x370,&body) && ptr_at(actor+0x378,&unit) &&
               read_at(actor+0x380,&trigger,8) && unit==body+8 && trigger==body-0x14) {
                out->object=offset;out->body=0x370;out->unit=0x378;
                out->trigger=0x380;out->effect=0;return 1;
            }
        }
        return 0;
    } else if(weapon==7) {
        unsigned type;
        /* Retail Stinger type 0x4060 lacks the 0x800000 alternate-trigger bit.
         * Player constructor therefore passes player.trigger (+B94), not
         * the arm animation trigger (+CF4). Keep this owner check strict. */
        if(!read_at(base+0x97e6b8,&type,4) || type!=0x4060 ||
           !read_at(base+0x53b5ea,code,9) ||
           memcmp(code,"\x4c\x8d\x8b\x94\x0b\0\0\xff\xd0",9))return 0;
        if(!read_at(base+0x715c60,code,14) ||
           memcmp(code,"\x48\x89\xb3\xe8\0\0\0\x48\x89\xbb\xf0\0\0\0",14)) return 0;
        out->body=0xe8; out->unit=0xf0; out->trigger=0xf8; out->effect=0;
    } else if(weapon==5) {
        if(!read_at(base+0x53b5ea,code,9) ||
           memcmp(code,"\x4c\x8d\x8b\x94\x0b\0\0\xff\xd0",9))return 0;
        if(!read_at(base+0x4b96c5,code,14) ||
           memcmp(code,"\x48\x89\xab\xf8\0\0\0\x48\x89\xb3\x70\x01\0\0",14)) return 0;
        out->body=0xf8; out->unit=0x170; out->effect=0x150; out->trigger=0x178;
    } else if(weapon==14 || weapon==12 || weapon==20) {
        if(!read_at(base+0x716330,code,14) ||
           memcmp(code,"\x48\x89\xb3\x38\x01\0\0\x48\x89\xbb\x40\x01\0\0",14)) return 0;
        /* Native weapon creation passes player.trigger, not arm_trigger. */
        if(!read_at(base+0x53b5ea,code,9) ||
           memcmp(code,"\x4c\x8d\x8b\x94\x0b\0\0\xff\xd0",9))return 0;
        out->object=0x60; out->body=0x138; out->unit=0x140;
        out->effect=0; out->trigger=0x148;
    }
    return 1;
}
#define CAP_REJECT(test) ((test) ? (r->refusal=#test,1) : 0)
/* Retail rigid-rifle route: model matrices are derived later by PreScreen.
   Never substitute a relaxed angle tolerance for this structural contract. */
static int rifle_rigid(uint64_t base,uint64_t objs) {
    unsigned flags;uint64_t rots,movs;unsigned char code[8];
    if(!read_at(base+0x4b0bd5,code,6) || memcmp(code,"\x41\xb8\x02\0\0\0",6) ||
       !read_at(base+0x9a1d9,code,5) || memcmp(code,"\x48\x83\x7b\x78\0",5) ||
       !read_at(base+0x9a1f6,code,8) || memcmp(code,"\x48\x83\xbb\x80\0\0\0\0",8))return 0;
    return read_at(objs+0x58,&flags,4) && (flags&0x32)==2 &&
        read_at(objs+0x78,&rots,8) && !rots &&
        read_at(objs+0x80,&movs,8) && !movs;
}
/* Spray-family coolant and measured microphone are single finished objects attached directly to hand joint 6.
   Its copied draw matrix is not a same-seam ownership witness. */
static int coolant_attached(uint64_t base,uint64_t objs) {
    unsigned flags; uint64_t rots,movs; unsigned char code[6]; short count;
    if(!read_at(base+0x715da5,code,6) || memcmp(code,"\x41\xb8\x32\0\0\0",6) ||
       !read_at(base+0x7164ba,code,5) || memcmp(code,"\xe8\x01\x7a\xf6\xff",5))return 0;
    return read_at(objs+0x58,&flags,4) && (flags&0x32)==0x32 &&
        read_at(objs+0x64,&count,2) && count==1 &&
        read_at(objs+0x78,&rots,8) && !rots && read_at(objs+0x80,&movs,8) && !movs;
}
static int geometry(uint64_t base,CAP_RECORD *r) {
    uint64_t sg,ag,armobjs,trigger,pwork,selected_body,body_slot,unit_slot;
    uint64_t actor_body,root,expected_root,effect_objs;
    int selected_unit,actor_unit;
    CAP_LAYOUT offsets; uint64_t actor_trigger=0,expected_actor_trigger=0;
    short count;
    MAT hand_world,root_world;
    double basis[3][3];
    unsigned int i; int j,k;
    if(CAP_REJECT(!anchors(base,&sg,&ag))) return CAP_ANCHOR;
    if(CAP_REJECT(!ptr_at(sg,&r->subobject)) || CAP_REJECT(!ptr_at(ag,&r->arm)) || CAP_REJECT(!ptr_at(r->arm,&armobjs)) || CAP_REJECT(r->subobject<0x100a0) || CAP_REJECT(r->arm<0x10060))
        return CAP_READ;
    if(CAP_REJECT(!ptr_at(r->subobject,&r->subobjs)) || CAP_REJECT(!read_at(r->arm-0x60+0x228,&trigger,8)) || CAP_REJECT(trigger<0x10cf4) || CAP_REJECT(trigger>=0x00007ffffffff000ULL) || CAP_REJECT((trigger&3))) return CAP_READ;
    pwork=trigger-0xcf4;
    if(CAP_REJECT(pwork&7)) return CAP_READ;
    if(CAP_REJECT(!ptr_at(pwork+0xba8,&selected_body)) || CAP_REJECT(!read_at(pwork+0xbb0,&selected_unit,4)) || CAP_REJECT(!read_at(pwork+0xb90,&r->weapon_id,4))) return CAP_READ;
    if(CAP_REJECT(!layout(base,r->weapon_id,r->subobject,&offsets))) return CAP_SELECTION;
    expected_actor_trigger=(r->weapon_id==7 || r->weapon_id==13 || r->weapon_id==14 || r->weapon_id==5 || r->weapon_id==12 || r->weapon_id==20)?pwork+0xb94:trigger;
    r->actor=r->subobject-offsets.object;
    effect_objs=r->subobjs;
    if(CAP_REJECT((offsets.effect && !ptr_at(r->actor+offsets.effect,&effect_objs))) || CAP_REJECT((offsets.trigger && (!read_at(r->actor+offsets.trigger,&actor_trigger,8) || actor_trigger!=expected_actor_trigger))) || CAP_REJECT(!ptr_at(r->actor+offsets.body,&body_slot)) || CAP_REJECT(!ptr_at(r->actor+offsets.unit,&unit_slot)) || CAP_REJECT(!ptr_at(body_slot,&actor_body)) || CAP_REJECT(!read_at(unit_slot,&actor_unit,4)) || CAP_REJECT(!read_at(r->subobjs+0x40,&root,8))) return CAP_READ;
    expected_root=armobjs+0x110+6*0x180;
    r->hand=expected_root;
    r->input.selected_body=selected_body; r->input.observed_body=actor_body;
    r->input.selected_unit=(uint64_t)selected_unit; r->input.observed_unit=(uint64_t)actor_unit;
    r->input.selected_root=expected_root; r->input.observed_root=root;
    r->input.expected_arm_body=r->arm; r->input.observed_arm_body=actor_body;

    if(CAP_REJECT(selected_body!=r->arm) || CAP_REJECT(actor_body!=r->arm) || CAP_REJECT(selected_unit!=6) || CAP_REJECT(actor_unit!=6) || CAP_REJECT(body_slot!=pwork+0xba8) || CAP_REJECT(unit_slot!=pwork+0xbb0) || CAP_REJECT(effect_objs!=r->subobjs) || CAP_REJECT(r->weapon_id==7 ? root!=0 : root!=expected_root)) return CAP_SELECTION;
    /* Stinger FPS model has no parent; retain its native raise translation. */
    if(r->weapon_id==7) root=r->subobjs;
    if(CAP_REJECT(!read_at(root,&root_world,sizeof root_world)) || CAP_REJECT(!read_at(expected_root,&hand_world,sizeof hand_world)) || CAP_REJECT(!read_at(r->subobjs+0x64,&count,2))) return CAP_READ;
    if(CAP_REJECT(count<1) || CAP_REJECT(count>MODELS)) return CAP_MODEL_COUNT;
    r->count=(unsigned int)count;
    for(i=0;i<r->count;i++) {
        uint64_t model=r->subobjs+0x110+(uint64_t)i*0x180;
        if(CAP_REJECT(!read_at(model,&r->model_world[i],sizeof(MAT))) || CAP_REJECT(!ptr_at(model+0xe0,&r->model_ptr[i])) || CAP_REJECT(!ptr_at(model+0x160,&r->model_owner[i])) || CAP_REJECT(!read_at(model+0x168,&r->bp_unit[i],4)) || CAP_REJECT(!read_at(model+0x170,&r->bp_mesh[i],8)) || CAP_REJECT(!read_at(r->model_ptr[i]+0x2c,&r->parent[i],4))) return CAP_READ;
        if(CAP_REJECT(r->model_owner[i]!=r->subobjs) || CAP_REJECT(r->bp_unit[i]<0) || CAP_REJECT(r->bp_unit[i]>65535) || CAP_REJECT((r->bp_mesh[i] && (r->bp_mesh[i]<0x10000 || r->bp_mesh[i]>=0x00007ffffffff000ULL ||
                              (r->bp_mesh[i]&7))))) return CAP_SELECTION;
        if(CAP_REJECT(r->parent[i]<-1) || CAP_REJECT(r->parent[i]>=(int32_t)i)) return CAP_SELECTION;
        for(j=0;j<4;j++) for(k=0;k<4;k++) if(CAP_REJECT(!_finite(r->model_world[i].m[j][k]))) return CAP_HAND;
    }
    for(j=0;j<3;j++) for(k=0;k<3;k++) {
        basis[j][k]=hand_world.m[j][k];
        r->input.weapon_root_world[j][k]=root_world.m[j][k];
    }
    if(CAP_REJECT(!dg_ik_basis_quat(basis,r->input.hand_world_q))) return CAP_HAND;
    /* Detect selection changes during the bounded copy. This is a camera-seam
       observation on the actor thread, not a claim of global atomic memory. */
    if(CAP_REJECT(r->weapon_id==7 ? memcmp(&root_world,&hand_world,3*sizeof root_world.m[0]) : memcmp(&root_world,&hand_world,sizeof root_world)) || CAP_REJECT(!same_ptr(sg,r->subobject)) || CAP_REJECT(!same_ptr(ag,r->arm)) || CAP_REJECT(!same_ptr(r->arm,armobjs)) || CAP_REJECT(!same_ptr(r->subobject,r->subobjs)) || CAP_REJECT(r->weapon_id==7 ? (!read_at(r->subobjs+0x40,&actor_trigger,8) || actor_trigger!=0) : !same_ptr(r->subobjs+0x40,root)) || CAP_REJECT(!same_ptr(r->actor+offsets.body,body_slot)) || CAP_REJECT(!same_ptr(r->actor+offsets.unit,unit_slot)) || CAP_REJECT((offsets.effect && !same_ptr(r->actor+offsets.effect,effect_objs))) || CAP_REJECT(!same_ptr(body_slot,actor_body)) || CAP_REJECT(!read_at(unit_slot,&j,4)) || CAP_REJECT(j!=actor_unit) || CAP_REJECT(!read_at(pwork+0xb90,&j,4)) || CAP_REJECT(j!=r->weapon_id) || CAP_REJECT(!read_at(r->subobjs+0x64,&count,2)) || CAP_REJECT(count!=(short)r->count) || CAP_REJECT(!read_at(root,&hand_world,sizeof hand_world)) || CAP_REJECT(memcmp(&root_world,&hand_world,sizeof root_world)))
        return CAP_CHANGED;
    {
        uint64_t trigger2;
        if(CAP_REJECT(!read_at(r->arm-0x60+0x228,&trigger2,8)) || CAP_REJECT(trigger2!=trigger) || CAP_REJECT((offsets.trigger && (!read_at(r->actor+offsets.trigger,&trigger2,8) || trigger2!=expected_actor_trigger)))) return CAP_CHANGED;
    }
    for(i=0;i<r->count;i++) {
        MAT now; int32_t parent;
        uint64_t mesh; int32_t unit;
        uint64_t model=r->subobjs+0x110+(uint64_t)i*0x180;
        if(CAP_REJECT(!same_ptr(model+0xe0,r->model_ptr[i])) || CAP_REJECT(!same_ptr(model+0x160,r->model_owner[i])) || CAP_REJECT(!read_at(model+0x168,&unit,4)) || CAP_REJECT(unit!=r->bp_unit[i]) || CAP_REJECT(!read_at(model+0x170,&mesh,8)) || CAP_REJECT(mesh!=r->bp_mesh[i]) || CAP_REJECT(!read_at(r->model_ptr[i]+0x2c,&parent,4)) || CAP_REJECT(parent!=r->parent[i]) || CAP_REJECT(!read_at(model,&now,sizeof now)) || CAP_REJECT(memcmp(&now,&r->model_world[i],sizeof now))) return CAP_CHANGED;
    }
    r->input.selected_body=selected_body; r->input.observed_body=actor_body;
    r->input.selected_unit=(uint64_t)selected_unit; r->input.observed_unit=(uint64_t)actor_unit;
    r->input.selected_root=r->weapon_id==7?r->subobjs:expected_root; r->input.observed_root=root;
    r->input.expected_arm_body=r->arm; r->input.observed_arm_body=actor_body;
    r->input.weapon_type=(uint32_t)r->weapon_id; /* explicit weapon ID, not WeaponSet.type */
    r->input.flags|=DG_AIM_PROBE_HAVE_HAND|DG_AIM_PROBE_HAVE_ROOT;
    r->refusal="ok";return CAP_OK;
}
#undef CAP_REJECT

/* Readable raw geometry survives a production selection refusal. Never marks
   a rejected selection valid and never writes to game memory. */
/* Observation only. Partial reads carry individual validity bits. These
   fields never relax geometry() or grant controller ownership. */
static void coolant_read(const CAP_RECORD *r,COOLANT_DIAG *d) {
    uint64_t player=0;unsigned flags=0;int unit=0;
    memset(d,0,sizeof *d);
#define CD_READ(n,addr) do {if(read_at((addr),&d->value[n],8))d->mask|=1u<<(n);}while(0)
    CD_READ(0,r->arm-0x60+0x228); /* expected trigger, via arm owner */
    CD_READ(1,r->actor+(r->weapon_id!=5?0x148:0x178)); /* observed spray actor trigger */
    CD_READ(2,r->actor+(r->weapon_id!=5?0x138:0xf8)); /* body slot */
    CD_READ(3,r->actor+(r->weapon_id!=5?0x140:0x170)); /* unit slot */
    if(d->mask&4)CD_READ(4,d->value[2]);
    if((d->mask&8) && read_at(d->value[3],&unit,4)){d->value[5]=(uint64_t)(int64_t)unit;d->mask|=32;}
    if((d->mask&1) && d->value[0]>=0x10cf4 && d->value[0]<0x00007ffffffff000ULL && !(d->value[0]&3)) {
        player=d->value[0]-0xcf4;
        d->value[12]=(r->weapon_id==14 || r->weapon_id==5 || r->weapon_id==12 || r->weapon_id==20)?player+0xb94:d->value[0];d->mask|=4096;
        CD_READ(6,player+0xba8);
        if(read_at(player+0xbb0,&unit,4)){d->value[7]=(uint64_t)(int64_t)unit;d->mask|=128;}
    }
    CD_READ(8,r->subobjs+0x40);
    if(read_at(r->subobjs+0x58,&flags,4)){d->value[9]=flags;d->mask|=512;}
    CD_READ(10,r->subobjs+0x78);CD_READ(11,r->subobjs+0x80);
#undef CD_READ
}
static void raw_geometry(CAP_RECORD *r) {
    uint64_t objs;short count;MAT again;int i,j,k;
    if(r->capture_result==CAP_ANCHOR)return;
    if((r->weapon_id==14 || r->weapon_id==5 || r->weapon_id==12 || r->weapon_id==20) && r->actor) {
        COOLANT_DIAG again;
        coolant_read(r,&r->coolant);coolant_read(r,&again);
        r->coolant_stable=memcmp(&r->coolant,&again,sizeof again)==0;
    }
    if(ptr_at(r->arm,&objs) && read_at(objs+0x110+6*0x180,&r->raw_hand,sizeof(MAT)) &&
       read_at(objs+0x110+6*0x180,&again,sizeof again) && !memcmp(&again,&r->raw_hand,sizeof again) && same_ptr(r->arm,objs)) {
        r->raw_hand_valid=1;
        for(j=0;j<4;j++)for(k=0;k<4;k++)if(!_finite(r->raw_hand.m[j][k]))r->raw_hand_valid=0;
    }
    if(!r->subobjs || !read_at(r->subobjs+0x64,&count,2)||count<1||count>MODELS)return;
    for(i=0;i<count;i++) {
        if(!read_at(r->subobjs+0x110+i*0x180,&r->raw_models[i],sizeof(MAT)))return;
        for(j=0;j<4;j++)for(k=0;k<4;k++)if(!_finite(r->raw_models[i].m[j][k]))return;
        if(!read_at(r->subobjs+0x110+i*0x180,&again,sizeof again)||memcmp(&again,&r->raw_models[i],sizeof again))return;
    }
    if(same_ptr(r->subobject,r->subobjs))r->raw_model_count=count;
}
int dg_aim_capture_hand_selection(uintptr_t base, uint64_t expected_arm,
                                DG_AIM_SELECTION *selection) {
    CAP_RECORD r;
    int i,j;
    if(!selection) return 0;
    memset(selection,0,sizeof *selection);
    memset(&r,0,sizeof r);
    if(!expected_arm || geometry((uint64_t)base,&r)!=CAP_OK ||
       r.arm!=expected_arm || !dg_weapon_hand_aim(r.weapon_id) ||
       r.parent[0]!=-1) return 0;
    if(r.weapon_id==15 || r.weapon_id==18 || r.weapon_id==14 || r.weapon_id==12) {
        double basis[3][3],q[4];
        /* A valid older rigid render matrix need not equal the current hand.
           Keep finite/rotation checks and all geometry ownership checks. */
        if(!((r.weapon_id==14 || r.weapon_id==12)?coolant_attached((uint64_t)base,r.subobjs):rifle_rigid((uint64_t)base,r.subobjs)))return 0;
        for(i=0;i<3;i++) for(j=0;j<3;j++)basis[i][j]=r.model_world[0].m[i][j];
        if(!dg_ik_basis_quat(basis,q))return 0;
        if(!same_ptr(r.subobject,r.subobjs) || !same_ptr(r.subobjs+0x40,r.hand) ||
           !((r.weapon_id==14 || r.weapon_id==12)?coolant_attached((uint64_t)base,r.subobjs):rifle_rigid((uint64_t)base,r.subobjs)))return 0;
    } else {
        for(i=0;i<3;i++) for(j=0;j<3;j++)
            if(fabs(r.model_world[0].m[i][j]-r.input.weapon_root_world[i][j])>0.002)
                return 0;
    }
    selection->arm=r.arm; selection->subobject=r.subobject;
    selection->subobjs=r.subobjs; selection->hand=r.hand;
    selection->model=r.model_ptr[0];
    selection->weapon_id=(uint64_t)r.weapon_id;
    return 1;
}
void dg_aim_capture_configure(int on) { InterlockedExchange(&enabled,on?1:0); }
int dg_aim_capture_enabled(void) { return InterlockedCompareExchange(&enabled,0,0)!=0; }
void dg_aim_capture_observe(uintptr_t base,uint64_t stream,const MAT *camera,
                            const MAT *proj,const DG_XR_FRAME *frame,int frame_flags,
                            const DG_XR_RAW_POSE *view,int kind) {
    CAP_RECORD r; LARGE_INTEGER q; int j,k;
    const DG_XR_HAND_POSE *aim;
    if(!dg_aim_capture_enabled() || !camera || !proj || !frame || !view) return;
    /* No blocking camera lock. All record scratch is stack-local. */
    if(!TryAcquireSRWLockExclusive(&ring_lock)) { InterlockedIncrement(&busy_drops); return; }
    memset(&r,0,sizeof r);
    r.id=++total; QueryPerformanceCounter(&q); r.qpc=(uint64_t)q.QuadPart;
    aim=&frame->right_hand.aim; r.publication_aim_seq=aim->sample_seq; r.aim_time=aim->xr_time;
    r.view_kind=kind; r.projection=*proj; r.camera_full=*camera;
    r.trigger_value=frame->right_hand.trigger_value;
    r.right_grip=frame->right_hand.grip.raw_local;r.left_grip=frame->left_hand.grip.raw_local;
    r.input.observation_id=r.input.hierarchy_observation_id=r.id;
    r.input.stream_id=stream;
    /* These tags name provenance from ONE copied DG_XR_FRAME. Eye itself has
       no sample counter. The outer record preserves its actual kind. */
    r.input.publication_aim_seq=r.input.camera_publication_aim_seq=aim->sample_seq;
    r.input.aim_publication_aim_seq=r.input.eye_publication_aim_seq=aim->sample_seq;
    r.input.aim_age_ms=aim->pose_age_ms; r.input.eye_age_ms=0;
    r.input.flags=DG_AIM_PROBE_HAVE_CAMERA;
    if((frame_flags&1) && (((kind==-1 || kind==2) && view==&frame->head_raw) ||
       ((kind==0 || kind==1) && view==&frame->eye[kind].raw))) {
        r.input.flags|=DG_AIM_PROBE_HAVE_EYE;
        memcpy(r.input.eye_raw_q,&view->qx,sizeof(double)*4);
    }
    if((frame_flags&1) && aim->hand==DG_XR_HAND_RIGHT && aim->kind==DG_XR_POSE_AIM &&
       aim->active && aim->tracked && aim->orientation_valid)
        r.input.flags|=DG_AIM_PROBE_HAVE_AIM;
    memcpy(r.input.aim_raw_q,&aim->raw_local.qx,sizeof(double)*4);
    for(j=0;j<3;j++) for(k=0;k<3;k++) r.input.camera_world[j][k]=camera->m[j][k];
    r.capture_result=geometry((uint64_t)base,&r);
    r.transforms_valid=full_transforms(&r);raw_geometry(&r);
    r.probe_result=(int)dg_aim_probe_build(&r.input,&r.sample);
    ring[(r.id-1)%CAPACITY]=r;
    ReleaseSRWLockExclusive(&ring_lock);
}
void dg_aim_capture_native_configure(int on) {InterlockedExchange(&native_enabled,on?1:0);}
int dg_aim_capture_native_enabled(void) {return InterlockedCompareExchange(&native_enabled,0,0)!=0;}
void dg_aim_capture_native_observe(uintptr_t base,const MAT *camera,const MAT *proj) {
    CAP_RECORD r;LARGE_INTEGER q;uint64_t now=GetTickCount64();int slot=5,j,k;
    if(!dg_aim_capture_native_enabled() || !camera || !proj)return;
    if(!TryAcquireSRWLockExclusive(&ring_lock)){InterlockedIncrement(&busy_drops);return;}
    if(native_last_capture && now-native_last_capture<100){ReleaseSRWLockExclusive(&ring_lock);return;}
    native_last_capture=now;
    memset(&r,0,sizeof r);r.view_kind=3;r.camera_full=*camera;r.projection=*proj;
    r.capture_result=geometry(base,&r);r.transforms_valid=full_transforms(&r);raw_geometry(&r);
    /* No fabricated XR pose or valid aim result for native-only observations. */
    r.probe_result=-1;r.input.flags=DG_AIM_PROBE_HAVE_CAMERA;
    for(j=0;j<3;j++)for(k=0;k<3;k++)r.input.camera_world[j][k]=camera->m[j][k];
    switch(r.weapon_id){case 15:slot=0;break;case 18:slot=1;break;case 14:slot=2;break;
        case 5:slot=3;break;case 3:slot=4;break;case 12:slot=6;break;case 20:slot=7;break;}
    if(!native_stamp[slot] || now-native_stamp[slot]>=100) {
        native_stamp[slot]=now;QueryPerformanceCounter(&q);r.qpc=q.QuadPart;
        r.id=++native_total;native_ring[slot][native_counts[slot]++%NATIVE_ROWS]=r;
    }
    ReleaseSRWLockExclusive(&ring_lock);
}
static void doubles(FILE *f,const double *v,int n) {
    int i; fputc('[',f); for(i=0;i<n;i++) {
        if(i) fputc(',',f); if(_finite(v[i])) fprintf(f,"%.17g",v[i]); else fputs("null",f);
    } fputc(']',f);
}
static void floats(FILE *f,const float *v,int n) {
    int i; fputc('[',f); for(i=0;i<n;i++) {
        if(i) fputc(',',f); if(_finite(v[i])) fprintf(f,"%.9g",v[i]); else fputs("null",f);
    } fputc(']',f);
}
static void json_text(FILE *f,const char *s) {
    fputc('"',f);if(s)for(;*s;s++){if(*s=='"'||*s=='\\')fputc('\\',f);
        if((unsigned char)*s>=32)fputc(*s,f);else fputc(' ',f);}fputc('"',f);
}
static int native_order(const void *a,const void *b) {
    const CAP_RECORD *x=(const CAP_RECORD *)a,*y=(const CAP_RECORD *)b;
    return x->id<y->id?-1:x->id>y->id?1:0;
}
static long dump_snapshot(const char *path) {
    uint64_t n,start,i; unsigned slot; int native_mode; LONG drops; FILE *f; LARGE_INTEGER freq;
    if(!path) return -1;
    if(!TryAcquireSRWLockExclusive(&ring_lock)) return -1;
    native_mode=dg_aim_capture_native_enabled();
    n=total<CAPACITY?total:CAPACITY; start=total-n;
    for(i=0;i<n;i++) dump_rows[i]=ring[(start+i)%CAPACITY];
    if(native_mode) {
        n=start=0;
        for(slot=0;slot<NATIVE_WEAPONS;slot++) {
            uint64_t cnt=native_counts[slot],keep=cnt<NATIVE_ROWS?cnt:NATIVE_ROWS;
            start+=cnt-keep;
            for(i=cnt-keep;i<cnt;i++)dump_rows[n++]=native_ring[slot][i%NATIVE_ROWS];
        }
    }
    drops=InterlockedCompareExchange(&busy_drops,0,0);
    ReleaseSRWLockExclusive(&ring_lock);
    if(!n) return 0;
    if(native_mode)qsort(dump_rows,(size_t)n,sizeof(CAP_RECORD),native_order);
    f=fopen(path,"wb"); if(!f) return -1;
    QueryPerformanceFrequency(&freq);
    fprintf(f,"{\"format\":\"DG_AIM_OBSERVATION1\",\"label\":\"%s\",\"qpf\":%lld,\"rows\":%llu,\"overwritten\":%llu,\"busy_drops\":%ld,\"consumed_command_known\":false,\"eye_age_known\":false,\"final_draw_proven\":false}\n",
            DG_AIM_PROBE_LABEL,freq.QuadPart,n,start,drops);
    for(i=0;i<n;i++) {
        const CAP_RECORD *r=&dump_rows[i]; unsigned int j;
        fprintf(f,"{\"id\":%llu,\"qpc\":%llu,\"arm_stream_at_observation\":%llu,\"publication_aim_seq\":%llu,\"aim_time\":%lld,\"view_kind\":%d,\"capture_result\":%d,\"probe_result\":%d,\"valid\":%u,\"weapon_id\":%d,\"aim_age_ms\":%u,\"flags\":%u,\"actor\":\"%llX\",\"subobject\":\"%llX\",\"subobjs\":\"%llX\",\"arm\":\"%llX\",\"hand\":\"%llX\",\"camera_world\":",
                r->id,r->qpc,r->input.stream_id,r->publication_aim_seq,r->aim_time,r->view_kind,
                r->capture_result,r->probe_result,r->sample.valid,r->weapon_id,r->input.aim_age_ms,r->input.flags,
                r->actor,r->subobject,r->subobjs,r->arm,r->hand);
        floats(f,&r->input.camera_world[0][0],9);
        fputs(",\"refusal\":",f);json_text(f,r->refusal);
        fprintf(f,",\"native_geometry\":%s",r->view_kind==3?"true":"false");
        fprintf(f,",\"selected_body\":\"%llX\",\"observed_body\":\"%llX\",\"selected_unit\":%llu,\"observed_unit\":%llu,\"selected_root\":\"%llX\",\"observed_root\":\"%llX\",\"expected_arm\":\"%llX\",\"observed_arm\":\"%llX\"",
                r->input.selected_body,r->input.observed_body,r->input.selected_unit,r->input.observed_unit,
                r->input.selected_root,r->input.observed_root,r->input.expected_arm_body,r->input.observed_arm_body);
        if(r->weapon_id==14 || r->weapon_id==5 || r->weapon_id==12 || r->weapon_id==20) {
            static const char *names[13]={"expected_trigger","actor_trigger","body_slot","unit_slot",
                "body_value","unit_value","selected_body","selected_unit","root","object_flags","rots","movs","expected_actor_trigger"};
            fprintf(f,",\"%s\":{\"version\":2,\"read_mask\":%u,\"stable\":%d,\"observational_only\":true",
                r->weapon_id==14?"coolant_diagnostic":r->weapon_id==5?"rgb6_diagnostic":"microphone_diagnostic",r->coolant.mask,r->coolant_stable);
            for(j=0;j<13;j++) {
                fprintf(f,",\"%s\":",names[j]);
                if(r->coolant.mask&(1u<<j))fprintf(f,"\"%llX\"",r->coolant.value[j]);else fputs("null",f);
            }
            fputc('}',f);
        }
        fputs(",\"projection\":",f); floats(f,&r->projection.m[0][0],16);
        fprintf(f,",\"transform_observation_version\":1,\"transforms_valid\":%d,\"right_trigger\":%.9g",
                r->transforms_valid,r->trigger_value);
        fputs(",\"camera_world_full\":",f); floats(f,&r->camera_full.m[0][0],16);
        if(r->transforms_valid) {
            fputs(",\"arm_root_world_full\":",f); floats(f,&r->arm_root_full.m[0][0],16);
            fputs(",\"shoulder_world_full\":",f); floats(f,&r->shoulder_full.m[0][0],16);
            fputs(",\"hand_world_full\":",f); floats(f,&r->hand_full.m[0][0],16);
        }
        fputs(",\"root_world\":",f); floats(f,&r->input.weapon_root_world[0][0],9);
        fputs(",\"hand_q\":",f); doubles(f,r->input.hand_world_q,4);
        fputs(",\"aim_raw_q\":",f); doubles(f,r->input.aim_raw_q,4);
        fputs(",\"view_raw_q\":",f); doubles(f,r->input.eye_raw_q,4);
        if(r->sample.valid) {
            fputs(",\"root_minus_y_camera\":",f); doubles(f,r->sample.root_minus_y_camera,3);
            fputs(",\"aim_in_raw_view_xr\":",f); doubles(f,r->sample.raw_eye_relative_aim_xr,3);
        }
        fprintf(f,",\"raw_hand_valid\":%d,\"raw_hand_world\":",r->raw_hand_valid);
        floats(f,&r->raw_hand.m[0][0],16);
        fputs(",\"raw_model_world\":[",f);
        for(j=0;j<(unsigned)r->raw_model_count;j++){if(j)fputc(',',f);floats(f,&r->raw_models[j].m[0][0],16);}fputc(']',f);
        fputs(",\"right_grip_raw_local\":",f);doubles(f,&r->right_grip.qx,7);
        fputs(",\"left_grip_raw_local\":",f);doubles(f,&r->left_grip.qx,7);
        fputs(",\"models\":[",f);
        for(j=0;j<r->count;j++) {
            if(j) fputc(',',f);
            fprintf(f,"{\"index\":%u,\"model\":\"%llX\",\"parent\":%d,\"owner\":\"%llX\",\"bp_mesh\":\"%llX\",\"bp_unit\":%d,\"draw_identity_present\":%s,\"world\":",
                    j,r->model_ptr[j],r->parent[j],r->model_owner[j],r->bp_mesh[j],r->bp_unit[j],r->bp_mesh[j]?"true":"false");
            floats(f,&r->model_world[j].m[0][0],16); fputc('}',f);
        }
        fputs("]}\n",f);
    }
    { int bad=ferror(f); if(fclose(f)) bad=1; return bad?-1:(long)n; }
}
long dg_aim_capture_dump(const char *path) {
    long result;
    /* Worker and shutdown may overlap. Never wait from shutdown or let two
       dumpers overwrite each other's scratch rows during file I/O. */
    if(!TryAcquireSRWLockExclusive(&dump_lock)) return -1;
    result=dump_snapshot(path);
    ReleaseSRWLockExclusive(&dump_lock);
    return result;
}
