/* Measured 2026-09-25: 189 native Nikita samples; subject and normal models
 * share geometry but not skeleton ownership. Normal is visible, subject is
 * hidden. Admit the existing subject-to-arm attachment separately from the
 * selected native body. Never fake player.BA8 or route Nikita through RGB6.
 * Native Act refreshes normal.root before its trigger dispatch; its Nikita
 * launch branch constructs a camera matrix independently of the model root.
 */
typedef struct {
    DG_NIKITA_BINDING binding;
    uint64_t body_slot, unit_slot, trigger, control, action, player_body;
    uint64_t roots[2], owner[2], model[2], rots[2], movs[2];
    unsigned flags[2];
    int type, weapon, unit, parent[2], bp_unit[2];
    short body_count, arm_count, count[2];
    MAT hand, world[2];
} NIKITA_SNAPSHOT;

static int nikita_bytes(uint64_t base,unsigned rva,const char *bytes,size_t n) {
    unsigned char actual[64];
    return n<=sizeof actual && read_at(base+rva,actual,n) && !memcmp(actual,bytes,n);
}
static int nikita_revision(uint64_t base) {
    uint64_t constructor,action; unsigned type;
    return nikita_bytes(base,0x4b976f,"\xc7\x44\x24\x28\0\0\0\0\x89\x44\x24\x20\xe8\xa0\xfe\xff\xff",17) &&
        nikita_bytes(base,0x4b96be,"\x4c\x89\xb3\xf0\0\0\0\x48\x89\xab\xf8\0\0\0\x48\x89\xb3\x70\x01\0\0",21) &&
        nikita_bytes(base,0x4b96a4,"\x48\x89\xbb\x78\x01\0\0",7) &&
        nikita_bytes(base,0x4b96f5,"\x89\xbb\xa8\x01\0\0",6) &&
        nikita_bytes(base,0x4b957d,"\x4c\x89\x35\x0c\x61\x32\x01",7) &&
        nikita_bytes(base,0x4b958e,"\x48\x8d\x9e\xa0\0\0\0\x48\x89\x1d\xec\x60\x32\x01",14) &&
        nikita_bytes(base,0x4b95a3,"\x48\x8b\x0d\xe6\x8a\x09\x01",7) &&
        nikita_bytes(base,0x53b5c8,"\x4c\x8d\x83\xb0\x0b\0\0\x48\x8d\x93\xa8\x0b\0\0",14) &&
        nikita_bytes(base,0x53b5ea,"\x4c\x8d\x8b\x94\x0b\0\0\xff\xd0",9) &&
        nikita_bytes(base,0x4b9508,"\x41\xb8\x12\0\0\0",6) &&
        nikita_bytes(base,0x4b954e,"\x41\xb8\x12\0\0\0",6) &&
        read_at(base+0x97e680,&constructor,8) && constructor==base+0x4b9760 &&
        read_at(base+0x97e688,&action,8) && !action &&
        read_at(base+0x97e690,&type,4) && type==0x08084682;
}
static int nikita_matrix(const MAT *m) {
    int i,j;double basis[3][3],q[4];
    for(i=0;i<4;i++)for(j=0;j<4;j++)if(!_finite(m->m[i][j]))return 0;
    for(i=0;i<3;i++) {
        if(fabs(m->m[i][3])>.002)return 0;
        for(j=0;j<3;j++)basis[i][j]=m->m[i][j];
    }
    return fabs(m->m[3][3]-1)<.002 && dg_ik_basis_quat(basis,q);
}








static int nikita_snapshot(uint64_t base,uint64_t expected,NIKITA_SNAPSHOT *s) {
    DG_NIKITA_BINDING *b=&s->binding;
    uint64_t sg,ag,arm_trigger,objects[2],obj,i;
    memset(s,0,sizeof *s);
    if((!anchors(base,&sg,&ag)) || (!nikita_revision(base)) ||
       (!ptr_at(ag,&b->arm)) || (b->arm!=expected) || (b->arm<0x10060) ||
       (!ptr_at(b->arm,&b->arm_objs)) ||
       (!read_at(b->arm-0x60+0x228,&arm_trigger,8)) || (arm_trigger<0x10cf4) || ((arm_trigger&3)))return 0;
    b->player=arm_trigger-0xcf4;
    if(((b->player&7)) || (!read_at(b->player+0xb90,&s->weapon,4)) || (s->weapon!=6) ||
       (!ptr_at(base+0x17df690,&b->normal_object)) || (b->normal_object<0x10060) ||
       (!ptr_at(sg,&b->subject_object)))return 0;
    b->actor=b->normal_object-0x60;
    if((b->subject_object!=b->actor+0xa0) ||
       (!ptr_at(b->actor+8,&s->action)) || (s->action!=base+0x4b89a0) ||
       (!read_at(b->actor+0x1a8,&s->type,4)) || (s->type!=0) ||
       (!ptr_at(b->actor+0xf8,&s->body_slot)) || (s->body_slot!=b->player+0xba8) ||
       (!ptr_at(b->actor+0x170,&s->unit_slot)) || (s->unit_slot!=b->player+0xbb0) ||
       (!ptr_at(b->actor+0xf0,&s->control)) || (s->control!=b->player+0x60) ||
       (!read_at(b->actor+0x178,&s->trigger,8)) || (s->trigger!=b->player+0xb94) ||
       (!ptr_at(s->body_slot,&b->body)) || (!read_at(s->unit_slot,&s->unit,4)) || (s->unit!=6) ||
       (!ptr_at(base+0x1552090,&s->player_body)) ||
       ((b->body!=s->player_body && b->body!=b->arm)) || (!ptr_at(b->body,&b->body_objs)) ||
       (!read_at(b->body_objs+0x64,&s->body_count,2)) || ((s->body_count!=21 && !(b->body==b->arm && s->body_count==55))) ||
       (!read_at(b->arm_objs+0x64,&s->arm_count,2)) || ((s->arm_count!=21 && s->arm_count!=55)))return 0;






    b->native_root=b->body_objs+0x110+6*0x180;
    b->hand=b->arm_objs+0x110+6*0x180;
    if((!read_at(b->hand,&s->hand,sizeof(MAT))) || (!nikita_matrix(&s->hand)) ||
       (!ptr_at(b->normal_object,&b->normal_objs)) ||
       (!ptr_at(b->subject_object,&b->subject_objs)) || (b->normal_objs==b->subject_objs))return 0;
    objects[0]=b->normal_objs;objects[1]=b->subject_objs;
    for(i=0;i<2;i++) {
        obj=objects[i]+0x110;
        if((!read_at(objects[i]+0x40,&s->roots[i],8)) ||
           (!read_at(objects[i]+0x58,&s->flags[i],4)) || ((s->flags[i]&0x32)!=0x12) ||
           (!read_at(objects[i]+0x64,&s->count[i],2)) || (s->count[i]!=1) ||
           (!read_at(objects[i]+0x78,&s->rots[i],8)) || (s->rots[i]) ||
           (!read_at(objects[i]+0x80,&s->movs[i],8)) || (s->movs[i]) ||
           (!ptr_at(obj+0xe0,&s->model[i])) || (!ptr_at(obj+0x160,&s->owner[i])) || (s->owner[i]!=objects[i]) ||
           (!read_at(s->model[i]+0x2c,&s->parent[i],4)) || (s->parent[i]!=-1) ||
           (!read_at(obj+0x168,&s->bp_unit[i],4)) || (s->bp_unit[i]!=0) ||
           (!read_at(obj,&s->world[i],sizeof(MAT))) || (!nikita_matrix(&s->world[i])))return 0;
    }
    /* Both exact roots are known. Detached Snake offset, unknown attachment,
       other models and RGB6 are refused rather than coerced to this profile. */
    if(((s->roots[0]!=b->native_root && s->roots[0]!=b->hand)) || (s->roots[1]!=b->hand) ||
       (s->model[0]!=s->model[1]))return 0;
    b->normal_model=s->model[0];b->subject_model=s->model[1];b->normal_root=s->roots[0];
    return 1;
}
int dg_aim_capture_nikita_binding(uintptr_t base,uint64_t expected,DG_NIKITA_BINDING *out) {
    NIKITA_SNAPSHOT a,b;uint64_t trigger;int weapon;
    if(!out)return 0;memset(out,0,sizeof *out);
    /* Cheap rejection for all other weapons; their hot path stays unchanged. */
    if(expected<0x10060 || !read_at(expected-0x60+0x228,&trigger,8) ||
       trigger<0x10cf4 || (trigger&3) ||
       !read_at(trigger-0xcf4+0xb90,&weapon,4) || weapon!=6 ||
       !nikita_snapshot(base,expected,&a) ||
       !nikita_snapshot(base,expected,&b) || (memcmp(&a,&b,sizeof a)))return 0;
    *out=a.binding;return 1;
}
