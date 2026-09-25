#include "dg_build_profile.h"
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


#define CAPACITY DG_DIAGNOSTIC_CAPACITY(4096u)
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
#define NATIVE_ROWS DG_DIAGNOSTIC_CAPACITY(128)
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
#include "dg_nikita_binding.inl"



typedef struct { unsigned object, body, unit, effect, trigger; } CAP_LAYOUT;
static int layout(uint64_t base, int weapon, uint64_t subobject, CAP_LAYOUT *out) {
    unsigned char code[14];
    if(weapon==6)return 0; /* separate native-body Nikita witness, never pistol fallback */
    if(!dg_weapon_hand_aim(weapon) && weapon!=12 && weapon!=20) return 0; /* observation only */
    out->object=0xa0; out->body=0x248; out->unit=0x2e8;
    out->effect=0x2c0; out->trigger=0;



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
/* Both Blade subject constructors create a finished, one-piece rigid object.
   Its draw matrix can lag the live joint at the camera/input seam. Ownership
   comes from the checked actor slots and root pointer, not matrix equality. */
static int blade_attached(uint64_t base,uint64_t objs) {
    unsigned flags; uint64_t rots,movs; unsigned char code[6]; short count;
    if(!read_at(base+0x4c35a1,code,6) || memcmp(code,"\x41\xb8\x32\0\0\0",6) ||
       !read_at(base+0x4c363c,code,6) || memcmp(code,"\x41\xb8\x32\0\0\0",6))return 0;
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
    r->input.weapon_type=(uint32_t)r->weapon_id;
    r->input.flags|=DG_AIM_PROBE_HAVE_HAND|DG_AIM_PROBE_HAVE_ROOT;
    r->refusal="ok";return CAP_OK;
}
#undef CAP_REJECT

/* Readable raw geometry survives a production selection refusal. Never marks
   a rejected selection valid and never writes to game memory. */
/* Observation only. Partial reads carry individual validity bits. These
   fields never relax geometry() or grant controller ownership. */










































int dg_aim_capture_hand_selection(uintptr_t base, uint64_t expected_arm,
                                DG_AIM_SELECTION *selection) {
    CAP_RECORD r;
    int i,j;
    if(!selection) return 0;
    memset(selection,0,sizeof *selection);
    {
        DG_NIKITA_BINDING n;
        if(dg_aim_capture_nikita_binding(base,expected_arm,&n)) {
            selection->arm=n.arm;selection->hand=n.hand;
            selection->subobject=n.subject_object;selection->subobjs=n.subject_objs;
            selection->model=n.subject_model;selection->weapon_id=6;
            return 1;
        }
    }
    memset(&r,0,sizeof r);
    r.weapon_id=-1;
    if(!expected_arm) return 0;
    if(geometry((uint64_t)base,&r)!=CAP_OK) {                                            return 0; }
    if((r.arm!=expected_arm) || (!dg_weapon_hand_aim(r.weapon_id)) ||
       (r.parent[0]!=-1)) return 0;
    if(r.weapon_id==13) {
        double basis[3][3],q[4];
        if((!blade_attached((uint64_t)base,r.subobjs)))return 0;
        for(i=0;i<3;i++) for(j=0;j<3;j++)basis[i][j]=r.model_world[0].m[i][j];
        if(!dg_ik_basis_quat(basis,q))return 0;
        if(!same_ptr(r.subobject,r.subobjs) || !same_ptr(r.subobjs+0x40,r.hand) ||
           !blade_attached((uint64_t)base,r.subobjs))return 0;
    } else if(r.weapon_id==15 || r.weapon_id==18 || r.weapon_id==14 || r.weapon_id==12) {
        double basis[3][3],q[4];
        /* A valid older rigid render matrix need not equal the current hand.
           Keep finite/rotation checks and all geometry ownership checks. */
        if((!((r.weapon_id==14 || r.weapon_id==12)?coolant_attached((uint64_t)base,r.subobjs):rifle_rigid((uint64_t)base,r.subobjs))))return 0;
        for(i=0;i<3;i++) for(j=0;j<3;j++)basis[i][j]=r.model_world[0].m[i][j];
        if(!dg_ik_basis_quat(basis,q))return 0;
        if(!same_ptr(r.subobject,r.subobjs) || !same_ptr(r.subobjs+0x40,r.hand) ||
           !((r.weapon_id==14 || r.weapon_id==12)?coolant_attached((uint64_t)base,r.subobjs):rifle_rigid((uint64_t)base,r.subobjs)))return 0;
    } else if(r.weapon_id==7) {
        /* Stinger: geometry() already requires the object root to carry the live
           hand rotation exactly. Model 0 is the rendered copy, one frame behind a
           moving VR hand, so exact equality flapped and reset the arm map several
           times a second. Require a real rotation within 20 degrees of the root. */
        double basis[3][3],rootb[3][3],qm[4],qr[4],dot;
        for(i=0;i<3;i++) for(j=0;j<3;j++) {
            basis[i][j]=r.model_world[0].m[i][j];
            rootb[i][j]=r.input.weapon_root_world[i][j]; /* float in the probe record */
        }
        if((!dg_ik_basis_quat(basis,qm)))return 0;
        if((!dg_ik_basis_quat(rootb,qr)))return 0;
        dot=fabs(qm[0]*qr[0]+qm[1]*qr[1]+qm[2]*qr[2]+qm[3]*qr[3]);
        if(dot>1)dot=1;
        if((2.0*acos(dot)>20.0*3.14159265358979323846/180.0))return 0;
        if((!same_ptr(r.subobject,r.subobjs)))return 0;
    } else {
        for(i=0;i<3;i++) for(j=0;j<3;j++)
            if((fabs(r.model_world[0].m[i][j]-r.input.weapon_root_world[i][j])>0.002))
                return 0;
    }
    selection->arm=r.arm; selection->subobject=r.subobject;
    selection->subobjs=r.subobjs; selection->hand=r.hand;
    selection->model=r.model_ptr[0];
    selection->weapon_id=(uint64_t)r.weapon_id;
    return 1;
}
void dg_aim_capture_configure(int on) {





}
int dg_aim_capture_enabled(void) {



return 0;

}
void dg_aim_capture_observe(uintptr_t base,uint64_t stream,const MAT *camera,
                            const MAT *proj,const DG_XR_FRAME *frame,int frame_flags,
                            const DG_XR_RAW_POSE *view,int kind) {








































}
void dg_aim_capture_native_configure(int on) {





}
int dg_aim_capture_native_enabled(void) {



return 0;

}
void dg_aim_capture_native_observe(uintptr_t base,const MAT *camera,const MAT *proj) {























}


































































































long dg_aim_capture_dump(const char *path) {











return 0;

}























































