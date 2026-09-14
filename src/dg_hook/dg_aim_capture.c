#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "dg_aim_capture.h"
#include "dg_aim_probe.h"
#include "dg_ik.h"

#define CAPACITY 4096u
#define MODELS 16u
enum { CAP_OK=0, CAP_ANCHOR=100, CAP_READ, CAP_SELECTION,
       CAP_MODEL_COUNT, CAP_CHANGED, CAP_HAND };
typedef struct {
    uint64_t id, qpc, publication_aim_seq;
    int64_t aim_time;
    int view_kind, capture_result, probe_result;
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
static SRWLOCK ring_lock=SRWLOCK_INIT;
static SRWLOCK dump_lock=SRWLOCK_INIT;
static volatile LONG enabled, busy_drops;
static uint64_t total;

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
static int geometry(uint64_t base,CAP_RECORD *r) {
    uint64_t sg,ag,armobjs,trigger,pwork,selected_body,body_slot,unit_slot;
    uint64_t actor_body,root,expected_root,effect_objs;
    int selected_unit,actor_unit;
    short count;
    MAT hand_world,root_world;
    double basis[3][3];
    unsigned int i; int j,k;
    if(!anchors(base,&sg,&ag)) return CAP_ANCHOR;
    if(!ptr_at(sg,&r->subobject) || !ptr_at(ag,&r->arm) ||
       !ptr_at(r->arm,&armobjs) || r->subobject<0x100a0 || r->arm<0x10060)
        return CAP_READ;
    r->actor=r->subobject-0xa0;
    if(!ptr_at(r->subobject,&r->subobjs) ||
       !ptr_at(r->actor+0x2c0,&effect_objs) ||
       !read_at(r->arm-0x60+0x228,&trigger,8) || trigger<0x10cf4 ||
       trigger>=0x00007ffffffff000ULL || (trigger&3)) return CAP_READ;
    pwork=trigger-0xcf4;
    if(pwork&7) return CAP_READ;
    if(!ptr_at(pwork+0xba8,&selected_body) ||
       !read_at(pwork+0xbb0,&selected_unit,4) || !read_at(pwork+0xb90,&r->weapon_id,4) ||
       !ptr_at(r->actor+0x248,&body_slot) || !ptr_at(r->actor+0x2e8,&unit_slot) ||
       !ptr_at(body_slot,&actor_body) || !read_at(unit_slot,&actor_unit,4) ||
       !ptr_at(r->subobjs+0x40,&root)) return CAP_READ;
    expected_root=armobjs+0x110+6*0x180;
    r->hand=expected_root;
    if(selected_body!=r->arm || actor_body!=r->arm || selected_unit!=6 || actor_unit!=6 ||
       body_slot!=pwork+0xba8 || unit_slot!=pwork+0xbb0 || effect_objs!=r->subobjs ||
       root!=expected_root || r->weapon_id<1 || r->weapon_id>3) return CAP_SELECTION;
    if(!read_at(root,&root_world,sizeof root_world) ||
       !read_at(expected_root,&hand_world,sizeof hand_world) ||
       !read_at(r->subobjs+0x64,&count,2)) return CAP_READ;
    if(count<1 || count>MODELS) return CAP_MODEL_COUNT;
    r->count=(unsigned int)count;
    for(i=0;i<r->count;i++) {
        uint64_t model=r->subobjs+0x110+(uint64_t)i*0x180;
        if(!read_at(model,&r->model_world[i],sizeof(MAT)) ||
           !ptr_at(model+0xe0,&r->model_ptr[i]) ||
           !ptr_at(model+0x160,&r->model_owner[i]) ||
           !read_at(model+0x168,&r->bp_unit[i],4) ||
           !read_at(model+0x170,&r->bp_mesh[i],8) ||
           !read_at(r->model_ptr[i]+0x2c,&r->parent[i],4)) return CAP_READ;
        if(r->model_owner[i]!=r->subobjs || r->bp_unit[i]<0 || r->bp_unit[i]>65535 ||
           (r->bp_mesh[i] && (r->bp_mesh[i]<0x10000 || r->bp_mesh[i]>=0x00007ffffffff000ULL ||
                              (r->bp_mesh[i]&7)))) return CAP_SELECTION;
        if(r->parent[i]<-1 || r->parent[i]>=(int32_t)i) return CAP_SELECTION;
        for(j=0;j<4;j++) for(k=0;k<4;k++) if(!_finite(r->model_world[i].m[j][k])) return CAP_HAND;
    }
    for(j=0;j<3;j++) for(k=0;k<3;k++) {
        basis[j][k]=hand_world.m[j][k];
        r->input.weapon_root_world[j][k]=root_world.m[j][k];
    }
    if(!dg_ik_basis_quat(basis,r->input.hand_world_q)) return CAP_HAND;
    /* Detect selection changes during the bounded copy. This is a camera-seam
       observation on the actor thread, not a claim of global atomic memory. */
    if(memcmp(&root_world,&hand_world,sizeof root_world) ||
       !same_ptr(sg,r->subobject) || !same_ptr(ag,r->arm) || !same_ptr(r->arm,armobjs) ||
       !same_ptr(r->subobject,r->subobjs) || !same_ptr(r->subobjs+0x40,root) ||
       !same_ptr(r->actor+0x248,body_slot) || !same_ptr(r->actor+0x2e8,unit_slot) ||
       !same_ptr(r->actor+0x2c0,effect_objs) ||
       !same_ptr(body_slot,actor_body) || !read_at(unit_slot,&j,4) || j!=actor_unit ||
       !read_at(pwork+0xb90,&j,4) || j!=r->weapon_id ||
       !read_at(r->subobjs+0x64,&count,2) || count!=(short)r->count ||
       !read_at(root,&hand_world,sizeof hand_world) || memcmp(&root_world,&hand_world,sizeof root_world))
        return CAP_CHANGED;
    {
        uint64_t trigger2;
        if(!read_at(r->arm-0x60+0x228,&trigger2,8) || trigger2!=trigger) return CAP_CHANGED;
    }
    for(i=0;i<r->count;i++) {
        MAT now; int32_t parent;
        uint64_t mesh; int32_t unit;
        uint64_t model=r->subobjs+0x110+(uint64_t)i*0x180;
        if(!same_ptr(model+0xe0,r->model_ptr[i]) ||
           !same_ptr(model+0x160,r->model_owner[i]) ||
           !read_at(model+0x168,&unit,4) || unit!=r->bp_unit[i] ||
           !read_at(model+0x170,&mesh,8) || mesh!=r->bp_mesh[i] ||
           !read_at(r->model_ptr[i]+0x2c,&parent,4) || parent!=r->parent[i] ||
           !read_at(model,&now,sizeof now) || memcmp(&now,&r->model_world[i],sizeof now)) return CAP_CHANGED;
    }
    r->input.selected_body=selected_body; r->input.observed_body=actor_body;
    r->input.selected_unit=(uint64_t)selected_unit; r->input.observed_unit=(uint64_t)actor_unit;
    r->input.selected_root=expected_root; r->input.observed_root=root;
    r->input.expected_arm_body=r->arm; r->input.observed_arm_body=actor_body;
    r->input.weapon_type=(uint32_t)r->weapon_id; /* explicit weapon ID, not WeaponSet.type */
    r->input.flags|=DG_AIM_PROBE_HAVE_HAND|DG_AIM_PROBE_HAVE_ROOT;
    return CAP_OK;
}
int dg_aim_capture_pistol_selection(uintptr_t base, uint64_t expected_arm,
                                DG_AIM_SELECTION *selection) {
    CAP_RECORD r;
    int i,j;
    if(!selection) return 0;
    memset(selection,0,sizeof *selection);
    memset(&r,0,sizeof r);
    if(!expected_arm || geometry((uint64_t)base,&r)!=CAP_OK ||
       r.arm!=expected_arm || (r.weapon_id!=1 && r.weapon_id!=2) ||
       r.parent[0]!=-1) return 0;
    for(i=0;i<3;i++) for(j=0;j<3;j++)
        if(fabs(r.model_world[0].m[i][j]-r.input.weapon_root_world[i][j])>0.002)
            return 0;
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
    r.transforms_valid=full_transforms(&r);
    r.probe_result=(int)dg_aim_probe_build(&r.input,&r.sample);
    ring[(r.id-1)%CAPACITY]=r;
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
static long dump_snapshot(const char *path) {
    uint64_t n,start,i; LONG drops; FILE *f; LARGE_INTEGER freq;
    if(!path) return -1;
    if(!TryAcquireSRWLockExclusive(&ring_lock)) return -1;
    n=total<CAPACITY?total:CAPACITY; start=total-n;
    for(i=0;i<n;i++) dump_rows[i]=ring[(start+i)%CAPACITY];
    drops=InterlockedCompareExchange(&busy_drops,0,0);
    ReleaseSRWLockExclusive(&ring_lock);
    if(!n) return 0;
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
        fprintf(f,",\"selected_body\":\"%llX\",\"observed_body\":\"%llX\",\"selected_unit\":%llu,\"observed_unit\":%llu,\"selected_root\":\"%llX\",\"observed_root\":\"%llX\",\"expected_arm\":\"%llX\",\"observed_arm\":\"%llX\"",
                r->input.selected_body,r->input.observed_body,r->input.selected_unit,r->input.observed_unit,
                r->input.selected_root,r->input.observed_root,r->input.expected_arm_body,r->input.observed_arm_body);
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
