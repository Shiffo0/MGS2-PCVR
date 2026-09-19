/* Included after radial: reuse its independently resolved linkvar/player slots
 * and fault-contained read. LIFE is optional and never gates other features. */
#include "dg_health.h"
static SRWLOCK g_health_lock=SRWLOCK_INIT;
static dg_health_view g_health_mail;
static dg_health_state g_health_history;
static int g_health_resolved;
static void health_resolve(const LiveImage *im) {
    unsigned section,hits=0;
    uint64_t slot=g_radial_game.inventory.linkvar_slot;
    g_health_resolved=0;
    memset(&g_health_history,0,sizeof g_health_history);
    AcquireSRWLockExclusive(&g_health_lock);
    memset(&g_health_mail,0,sizeof g_health_mail);
    ReleaseSRWLockExclusive(&g_health_lock);
    /* Two retail ration paths compare signed LIFE + adjustment to maximum.
     * Match instructions and both offsets; RIP slot MUST agree with inventory.
     * No absolute RVA, no guessed save-layout read on a different executable. */
    for(section=0;slot && section<im->section_count;section++) {
        const IMAGE_SECTION_HEADER *s=&im->sections[section];
        size_t i,end=(size_t)s->VirtualAddress+s->Misc.VirtualSize;
        if(!(s->Characteristics&IMAGE_SCN_MEM_EXECUTE) || end>im->size) continue;
        for(i=s->VirtualAddress;i+29<=end;i++) {
            const unsigned char *b=im->bytes+i;
            int32_t disp;
            if(memcmp(b,"\x48\x8b\x05",3) ||
               memcmp(b+7,"\x0f\xbf\x88\xfa\x00\x00\x00\x03\x0d",9) ||
               memcmp(b+20,"\x0f\xbf\x80\xfc\x00\x00\x00\x3b\xc8",9) ||
               !all_valid(im->valid,i,29,im->size)) continue;
            memcpy(&disp,b+3,4);
            if(im->base+i+7+(int64_t)disp!=slot) {hits=99;break;}
            hits++;
        }
    }
    g_health_resolved=hits==2 && (g_radial_game.inventory.valid_bits&DG_RINV_ACTOR);
    if(g_b.log) g_b.log("  LIFE: %s (retail paired reads %u)\r\n",
        g_health_resolved?"read-only observer ready":"unavailable",hits);
}
static void health_tick(unsigned game,unsigned menu,uint64_t status) {
    dg_health_sample s;
    dg_health_view v;
    int16_t life[2]; uint64_t check=0;
    memset(&s,0,sizeof s);s.stamp=GetTickCount64();
    if(g_health_resolved && !(game&DG_GAME_UNSAFE_MASK) && !(menu&DG_MENU_UNSAFE_MASK) &&
       !InterlockedCompareExchange(&g_b.script_menu_only,0,0) &&
       radial_live_read(NULL,g_radial_game.inventory.linkvar_slot,&s.inventory,8) &&
       s.inventory && s.inventory<UINT64_C(0x00007fffffff0000) &&
       radial_live_read(NULL,g_radial_game.inventory.player_slot,&s.player,8) && s.player &&
       radial_live_read(NULL,s.inventory+250,life,sizeof life) &&
       radial_live_read(NULL,s.inventory+188,&s.area,sizeof s.area) &&
       radial_live_read(NULL,g_radial_game.inventory.linkvar_slot,&check,8) && check==s.inventory) {
        s.value=life[0];s.maximum=life[1];s.bleeding=(status&UINT64_C(0x01000000))!=0;
        s.valid=radial_live_read(NULL,g_radial_game.inventory.player_slot,&check,8) && check==s.player;
    }
    v=dg_health_update(&g_health_history,s);
    if(TryAcquireSRWLockExclusive(&g_health_lock)) {
        g_health_mail=v;ReleaseSRWLockExclusive(&g_health_lock);
    }
}
int dg_bridge_health_snapshot(dg_health_view *out) {
    if(!out) return 0;
    memset(out,0,sizeof *out);
    if(!InterlockedCompareExchange(&g_b.armed,0,0) || !TryAcquireSRWLockShared(&g_health_lock)) return 0;
    *out=g_health_mail;ReleaseSRWLockShared(&g_health_lock);
    return dg_health_valid(&out->sample);
}
