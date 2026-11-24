#define BAP_SHIFTSET     0x980830u
#define BAP_SHIFTSET_N   22
#define BAP_BLADE_IDX    13
#define BAP_ROWS_RVA     0x980a10u
#define BAP_ROWS         6
#define BAP_M9_RVA       0x9802d0u
#define BAP_CS_M9        1
static const int bap_shiftset_witness[BAP_SHIFTSET_N]={0,1,2,3,0,7,12,6,0,0,8,8,9,0,10,4,8,8,4,0,11,0};
static const float bap_rows_witness[BAP_ROWS][4]={
    {0,500,0,0},{29,353,94,0},{336,494,15,0},{0,500,0,0},{0,500,240,0},{81,298,94,0}};
static const float bap_m9_row[4]={20,533,336,0};
static const struct { DWORD rva; unsigned n; const char *bytes; } bap_code_witness[]={
    {0x57cf43,27,"\x0f\xba\xe7\x08\x73\x1a\x8b\x05\x35\x3a\x40\x00\x48\x8d\x1d\xba\x3a\x40\x00\x8b\xef\xc1\xfd\x0c\x83\xe5\x0f"},
    {0x57cf8f,8,"\x41\x8b\xac\x86\x30\x08\x98\x00"}};
static struct {
    ULONGLONG base;
    int checked,resolved,want,owned_set,owned_row[BAP_ROWS];
    int refused_m9;
    unsigned long installs,restores,refusals;
} g_bap;
static SRWLOCK g_bap_lock=SRWLOCK_INIT;

static int bap_writable(ULONGLONG address,unsigned n)
{
    MEMORY_BASIC_INFORMATION mbi;
    return VirtualQuery((void *)(ULONG_PTR)address,&mbi,sizeof mbi) && mbi.State==MEM_COMMIT &&
        !(mbi.Protect&(PAGE_GUARD|PAGE_NOACCESS)) &&
        (mbi.Protect&(PAGE_READWRITE|PAGE_WRITECOPY|PAGE_EXECUTE_READWRITE|PAGE_EXECUTE_WRITECOPY)) &&
        (ULONGLONG)(ULONG_PTR)mbi.BaseAddress+mbi.RegionSize>=address+n;
}
static int bap_set_exchange(LONG before,LONG after)
{
    ULONGLONG a=g_bap.base+BAP_SHIFTSET+BAP_BLADE_IDX*4;
    return !(a&3) && bap_writable(a,4) &&
        InterlockedCompareExchange((volatile LONG *)(ULONG_PTR)a,after,before)==before;
}
static void bap_undo_locked(void)
{
    int i;
    for(i=0;i<BAP_ROWS;i++)if(g_bap.owned_row[i]) {
        unarmed_prone_exchange(g_bap.base+BAP_ROWS_RVA+i*16,bap_m9_row,bap_rows_witness[i]);
        g_bap.owned_row[i]=0;              /* a foreign value is never overwritten */
    }
    if(g_bap.owned_set){bap_set_exchange(BAP_CS_M9,0);g_bap.owned_set=0;}
}
static void blade_place_resolve(const LiveImage *im)
{
    int i;
    AcquireSRWLockExclusive(&g_bap_lock);

    {   int own=g_bap.owned_set;
        for(i=0;i<BAP_ROWS;i++)own|=g_bap.owned_row[i];
        if(own){ReleaseSRWLockExclusive(&g_bap_lock);return;} }
    g_bap.base=0;g_bap.resolved=0;g_bap.checked=1;g_bap.refused_m9=0;
    if(im && im->bytes && im->valid &&
       all_valid(im->valid,BAP_SHIFTSET,BAP_SHIFTSET_N*4,im->size) &&
       !memcmp(im->bytes+BAP_SHIFTSET,bap_shiftset_witness,sizeof bap_shiftset_witness) &&
       all_valid(im->valid,BAP_ROWS_RVA,sizeof bap_rows_witness,im->size) &&
       !memcmp(im->bytes+BAP_ROWS_RVA,bap_rows_witness,sizeof bap_rows_witness) &&
       all_valid(im->valid,BAP_M9_RVA,16,im->size) && !memcmp(im->bytes+BAP_M9_RVA,bap_m9_row,16)) {
        for(i=0;i<(int)(sizeof bap_code_witness/sizeof bap_code_witness[0]);i++)
            if(!all_valid(im->valid,bap_code_witness[i].rva,bap_code_witness[i].n,im->size) ||
               memcmp(im->bytes+bap_code_witness[i].rva,bap_code_witness[i].bytes,bap_code_witness[i].n))break;
        if(i==(int)(sizeof bap_code_witness/sizeof bap_code_witness[0])){g_bap.base=im->base;g_bap.resolved=1;}
    }
    ReleaseSRWLockExclusive(&g_bap_lock);
}
/* want 0 = off (native rows; restores what we own), 1 = m9. Marker thread or
   startup; the game thread only reads the tables. */
static void blade_place_apply(int want)
{
    int i,was;const char *line=NULL;
    AcquireSRWLockExclusive(&g_bap_lock);
    g_bap.want=want?1:0;
    was=g_bap.owned_set;
    if(!want)g_bap.refused_m9=0;
    if(!g_bap.resolved) {
        if(want && g_bap.checked && !g_bap.refusals++)line="refused (witnesses do not match this exe: nothing written)";
    } else if(want && !was && !g_bap.refused_m9) {
        int ok=bap_set_exchange(0,BAP_CS_M9);
        if(ok)g_bap.owned_set=1;
        for(i=0;ok && i<BAP_ROWS;i++) {
            if(!unarmed_prone_exchange(g_bap.base+BAP_ROWS_RVA+i*16,bap_rows_witness[i],bap_m9_row))ok=0;
            else g_bap.owned_row[i]=1;
        }
        if(ok){g_bap.installs++;line="m9 (the blade uses the M9 arm placement in every phase)";}
        else {bap_undo_locked();g_bap.refusals++;g_bap.refused_m9=1;line="refused (table ownership changed: nothing left written)";}
    } else if(!want && was) {
        bap_undo_locked();g_bap.restores++;line="off (native blade placement restored)";
    }
    ReleaseSRWLockExclusive(&g_bap_lock);
    if(line && g_b.log)g_b.log("  blade armplace: %s\r\n",line);
}
static void blade_place_stop(void)
{
    AcquireSRWLockExclusive(&g_bap_lock);
    if(g_bap.owned_set)g_bap.restores++;
    bap_undo_locked();
    g_bap.resolved=0;g_bap.checked=0;   /* a stopped bridge writes nothing until the next install resolves */
    ReleaseSRWLockExclusive(&g_bap_lock);
}
static int blade_place_on(void){return g_bap.owned_set;}

#define BAP_ARMCAMSHIFT 0x16e9bb0u
static int g_bap_probe_ok;
static struct {
    int phase,have;ULONGLONG phase_ms;float mn[3],mx[3],last[3];unsigned long n,lines;
    double req[3];int req_ok;double miss_sum[2],miss_max[2];unsigned long miss_n[2];
} g_bap_probe;
static const char *const bap_phase_name[3]={"rest","slash/thrust","block"};
static void blade_place_probe_resolve(const LiveImage *im)
{
    g_bap_probe_ok=im && im->bytes && im->valid && all_valid(im->valid,0x57d100,18,im->size) &&
        !memcmp(im->bytes+0x57d100,"\x41\xb8\x03\x00\x00\x00\x48\x8d\x54\x24\x20\x48\x8d\x0d\x9e\xca\x16\x01",18);
}
static void blade_place_probe_flush(void)
{
    int m=blade_place_on();
    if(g_bap_probe.have && g_bap_probe.n && g_b.log && g_bap_probe.lines<400) {
        g_bap_probe.lines++;
        g_b.log("  blade armplace probe: mode %s phase %s eye->arm root now side %.0f below %.0f behind %.0f mm"
            " (side %.0f..%.0f below %.0f..%.0f behind %.0f..%.0f over %lu ticks) | right hand requested->reached"
            " mean %.1f max %.1f mm over %lu pairs (mode %s)\r\n",
            m?"m9":"off",bap_phase_name[g_bap_probe.phase],g_bap_probe.last[0],g_bap_probe.last[1],g_bap_probe.last[2],
            g_bap_probe.mn[0],g_bap_probe.mx[0],g_bap_probe.mn[1],g_bap_probe.mx[1],g_bap_probe.mn[2],g_bap_probe.mx[2],
            g_bap_probe.n,g_bap_probe.miss_n[m]?g_bap_probe.miss_sum[m]/g_bap_probe.miss_n[m]:0.0,g_bap_probe.miss_max[m],
            g_bap_probe.miss_n[m],m?"m9":"off");
    }
    g_bap_probe.have=0;g_bap_probe.n=0;
}
/* Pad seam, game thread. */
static void blade_place_probe_tick(void)
{
    ULONGLONG now;float s[4];int phase,k;
    if(!g_b.log || !g_bap_probe_ok || !g_bap.base || !g_blade.selected){blade_place_probe_flush();return;}
    if(!interact_read(NULL,g_bap.base+BAP_ARMCAMSHIFT,s,16))return;
    for(k=0;k<3;k++)if(!_finite(s[k]) || fabsf(s[k])>5000.0f)return;
    now=GetTickCount64();
    phase=g_blade.guard_on?2:(g_blade.delivered_ms && now-g_blade.delivered_ms<700)?1:0;
    if(g_bap_probe.have && (phase!=g_bap_probe.phase || now-g_bap_probe.phase_ms>=3000))blade_place_probe_flush();
    if(!g_bap_probe.have){g_bap_probe.have=1;g_bap_probe.phase=phase;g_bap_probe.phase_ms=now;memcpy(g_bap_probe.mn,s,12);memcpy(g_bap_probe.mx,s,12);}
    for(k=0;k<3;k++){if(s[k]<g_bap_probe.mn[k])g_bap_probe.mn[k]=s[k];if(s[k]>g_bap_probe.mx[k])g_bap_probe.mx[k]=s[k];}
    memcpy(g_bap_probe.last,s,12);g_bap_probe.n++;
}
/* Right-arm solve (dg_bridge.c): the wrist read this pair against the target asked last pair. */
static void blade_place_hand_reached(const double wrist[3])
{
    double d;int m=blade_place_on();
    if(!g_bap_probe.req_ok)return;
    g_bap_probe.req_ok=0;d=vdist3(wrist,g_bap_probe.req);
    if(!_finite(d))return;
    g_bap_probe.miss_sum[m]+=d;g_bap_probe.miss_n[m]++;if(d>g_bap_probe.miss_max[m])g_bap_probe.miss_max[m]=d;
}
static void blade_place_hand_requested(const double target[3],int blade)
{
    g_bap_probe.req_ok=blade;if(blade)memcpy(g_bap_probe.req,target,sizeof g_bap_probe.req);
}
