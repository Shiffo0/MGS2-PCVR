typedef struct {
    DG_PSG_BINDING b;
    uint64_t act, ctrl_slot, body_slot, unit_slot, trigger_slot, scope, zoom;
    int weapon_no, weapon_type;
    short count, arm_count;
    int parent;
    uint64_t owner;
    MAT hand, world;
} PSG_SNAPSHOT;

static int psg_bytes(uint64_t base,unsigned rva,const char *bytes,size_t n) {
    unsigned char actual[32];
    return n<=sizeof actual && read_at(base+rva,actual,n) && !memcmp(actual,bytes,n);
}

#define PSG_ACT_ENTRY_RVA 0x713480u
#define PSG_ACT_ENTRY_LEN 26u
static const unsigned char k_psg_act_entry[PSG_ACT_ENTRY_LEN]=
    "\x48\x89\x5c\x24\x10\x48\x89\x74\x24\x18\x55\x57\x41\x56\x48\x8d\x6c\x24\xb9\x48\x81\xec\xb0\0\0\0";
static struct { volatile LONG seq; uint64_t base; unsigned n; unsigned char bytes[PSG_ACT_ENTRY_LEN]; } g_psg_act_own;
void dg_aim_capture_psg_act_own(uint64_t base,const unsigned char *bytes,unsigned n) {
    InterlockedIncrement(&g_psg_act_own.seq);                    /* odd: readers refuse a hooked entry */
    memset(g_psg_act_own.bytes,0,sizeof g_psg_act_own.bytes);g_psg_act_own.base=0;g_psg_act_own.n=0;
    if(bytes && base && n>=5 && n<=PSG_ACT_ENTRY_LEN) {
        memcpy(g_psg_act_own.bytes,bytes,n);g_psg_act_own.base=base;g_psg_act_own.n=n;
    }
    InterlockedIncrement(&g_psg_act_own.seq);
}
static int psg_act_entry(uint64_t base) {
    unsigned char live[PSG_ACT_ENTRY_LEN],own[PSG_ACT_ENTRY_LEN];uint64_t ob;unsigned n;LONG s1,s2;
    if(!read_at(base+PSG_ACT_ENTRY_RVA,live,sizeof live))return 0;
    if(!memcmp(live,k_psg_act_entry,sizeof live))return 1;        /* retail, nothing on it */
    s1=InterlockedCompareExchange(&g_psg_act_own.seq,0,0);
    if(s1&1)return 0;
    ob=g_psg_act_own.base;n=g_psg_act_own.n;memcpy(own,g_psg_act_own.bytes,sizeof own);
    MemoryBarrier();
    s2=InterlockedCompareExchange(&g_psg_act_own.seq,0,0);
    if(s1!=s2 || ob!=base || n<5 || n>PSG_ACT_ENTRY_LEN || own[0]!=0xe9)return 0;
    return !memcmp(live,own,n) && !memcmp(live+n,k_psg_act_entry+n,PSG_ACT_ENTRY_LEN-n);
}
static int psg_revision(uint64_t base) {
    uint64_t c4,c19;
    return read_at(base+0x97e5b8+3*40,&c4,8) && c4==base+0x713bd0 &&
        read_at(base+0x97e5b8+18*40,&c19,8) && c19==base+0x713c00 &&
        psg_act_entry(base) &&
        /* normal rifle object -> 0x17DF690 (rdi), hidden copy -> 0x17DF688 (rsi) */
        psg_bytes(base,0x713b1a,"\x48\x89\x3d\x6f\xbb\x0c\x01\x48\x89\x35\x60\xbb\x0c\x01",14) &&
        psg_bytes(base,0x713b4a,"\x89\x83\x24\x01\0\0\x8b\x44\x24\x60\x89\x83\0\x01\0\0\x89\x8b\x20\x01\0\0",22) &&
        psg_bytes(base,0x713b67,"\x4c\x89\xbb\xe8\0\0\0\x4c\x89\xb3\xf0\0\0\0\x48\x89\xab\xf8\0\0\0",21);
}
static int psg_matrix(const MAT *m) {
    int i,j;double basis[3][3],q[4];
    for(i=0;i<4;i++)for(j=0;j<4;j++)if(!_finite(m->m[i][j]))return 0;
    for(i=0;i<3;i++) {
        if(fabs(m->m[i][3])>.002)return 0;
        for(j=0;j<3;j++)basis[i][j]=m->m[i][j];
    }
    return fabs(m->m[3][3]-1)<.002 && dg_ik_basis_quat(basis,q);
}

#define PSG_REFUSE(test) do { if(test) { *why=#test; return 0; } } while(0)
static int psg_snapshot(uint64_t base,uint64_t expected,PSG_SNAPSHOT *s,const char **why) {
    DG_PSG_BINDING *b=&s->b;
    uint64_t sg,ag,arm_trigger,objs_check,body_objs;
    memset(s,0,sizeof *s);
    PSG_REFUSE(!anchors(base,&sg,&ag));
    PSG_REFUSE(!psg_revision(base));
    PSG_REFUSE(!ptr_at(ag,&b->arm) || b->arm!=expected || b->arm<0x10060);
    PSG_REFUSE(!ptr_at(b->arm,&b->arm_objs));
    PSG_REFUSE(!read_at(b->arm-0x60+0x228,&arm_trigger,8) || arm_trigger<0x10cf4 || (arm_trigger&3));
    b->player=arm_trigger-0xcf4;
    PSG_REFUSE((b->player&7) || !read_at(b->player+0xb90,&b->weapon,4));
    PSG_REFUSE(b->weapon!=4 && b->weapon!=19);
    PSG_REFUSE(!ptr_at(sg,&b->subject_object) || b->subject_object<0x100a0);
    b->actor=b->subject_object-0xa0;
    PSG_REFUSE(!ptr_at(base+0x17df690,&b->normal_object) || b->normal_object!=b->actor+0x60);
    PSG_REFUSE(!ptr_at(b->actor+8,&s->act) || s->act!=base+0x713480);
    PSG_REFUSE(!read_at(b->actor+0xe8,&s->body_slot,8) || s->body_slot!=b->player+0xba8);
    PSG_REFUSE(!read_at(b->actor+0xf0,&s->unit_slot,8) || s->unit_slot!=b->player+0xbb0);
    PSG_REFUSE(!read_at(b->actor+0xf8,&s->trigger_slot,8) ||
               (s->trigger_slot!=b->player+0xb94 && s->trigger_slot!=b->player+0xbe0));
    PSG_REFUSE(!read_at(b->actor+0x120,&s->weapon_no,4) || s->weapon_no!=b->weapon);
    PSG_REFUSE(!read_at(b->actor+0x124,&s->weapon_type,4) || s->weapon_type!=(b->weapon==4?4:1));
    PSG_REFUSE(!ptr_at(b->actor+0x108,&s->scope) || !ptr_at(b->actor+0x110,&s->zoom));
    /* The hide bit native Act toggles: 0x1000 << channel (0x7134EC; the same
       channel-0 bit the arm and Nikita code clear); anything else is refused. */
    PSG_REFUSE(!read_at(b->actor+0x128,&b->chanl,4) || b->chanl<0 || b->chanl>3);
    PSG_REFUSE(!read_at(b->actor+0x12c,&b->vflag,4) || b->vflag!=(0x1000u<<b->chanl));
    PSG_REFUSE(!read_at(b->arm_objs+0x64,&s->arm_count,2) || (s->arm_count!=21 && s->arm_count!=55));
    b->hand=b->arm_objs+0x110+6*0x180;
    PSG_REFUSE(!read_at(b->hand,&s->hand,sizeof(MAT)) || !psg_matrix(&s->hand));
    PSG_REFUSE(!ptr_at(base+0x1552090,&b->player_body) || !ptr_at(b->player_body,&body_objs));
    b->native_root=body_objs+0x110+6*0x180;
    PSG_REFUSE(!ptr_at(b->normal_object,&b->normal_objs) || b->normal_objs==b->arm_objs);
    PSG_REFUSE(!ptr_at(b->subject_object,&objs_check) || objs_check==b->normal_objs);
    PSG_REFUSE(!read_at(b->normal_objs+0x40,&b->root,8));
    PSG_REFUSE(!read_at(b->normal_objs+0x58,&b->flags,4));
    PSG_REFUSE(!read_at(b->normal_objs+0x64,&s->count,2) || s->count!=1);
    PSG_REFUSE(!ptr_at(b->normal_objs+0x110+0xe0,&b->model));
    PSG_REFUSE(!ptr_at(b->normal_objs+0x110+0x160,&s->owner) || s->owner!=b->normal_objs);
    PSG_REFUSE(!read_at(b->model+0x2c,&s->parent,4) || s->parent!=-1);
    PSG_REFUSE(!read_at(b->normal_objs+0x110,&s->world,sizeof(MAT)) || !psg_matrix(&s->world));
    /* Native player body joint 6, or the arm hand the lease set. Anything
       else (a cutscene root, a stale lease on another arm) is not ours. */
    PSG_REFUSE(b->root!=b->native_root && b->root!=b->hand);
    return 1;
}
#undef PSG_REFUSE
int dg_aim_capture_psg_binding(uintptr_t base,uint64_t expected,DG_PSG_BINDING *out,const char **why) {
    PSG_SNAPSHOT a,b;const char *dummy=NULL;uint64_t trigger;int weapon;
    if(!why)why=&dummy;
    *why="no output";
    if(!out)return 0;
    memset(out,0,sizeof *out);
    /* Cheap rejection for every other weapon: their hot path stays unchanged. */
    *why="not the PSG";
    if(expected<0x10060 || !read_at(expected-0x60+0x228,&trigger,8) ||
       trigger<0x10cf4 || (trigger&3) ||
       !read_at(trigger-0xcf4+0xb90,&weapon,4) || (weapon!=4 && weapon!=19))return 0;
    if(!psg_snapshot(base,expected,&a,why) || !psg_snapshot(base,expected,&b,why))return 0;
    /* Flags may legitimately change between reads (native Act vs this
       thread); everything else must be identical. */
    b.b.flags=a.b.flags;
    if(memcmp(&a,&b,sizeof a)) { *why="changed between snapshots"; return 0; }
    *out=a.b;*why="ok";return 1;
}
