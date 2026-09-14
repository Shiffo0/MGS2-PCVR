/* Serialized with the radial producer/native change seam. UseItem reads only
   current(+94) and pad(+38); a private request frame avoids opening/mutating
   the native menu. Inventory, healing and counters remain native-owned. */
#include "dg_item_use_retail.h"
static struct {
    int pending;
    uint64_t last_seq,last_epoch,player,inventory,ms,tick;
    dg_radial_commit_intent intent;
} g_item_use;
static void item_use_cancel(void) { g_item_use.pending=0; }
static int item_consumable(int id) { return id==1 || id==3 || id==4 || id==5; }
void dg_bridge_item_use_offer(const dg_radial_commit_intent *intent,
    uint64_t player,uint64_t inventory,int ready,uint64_t now)
{
    if(!intent)return;
    if(intent->epoch!=g_item_use.last_epoch) {
        g_item_use.last_epoch=intent->epoch;g_item_use.last_seq=0;g_item_use.pending=0;
    }
    if(!intent->seq || intent->seq<=g_item_use.last_seq)return;
    g_item_use.last_seq=intent->seq;
    if(!ready || intent->epoch!=dg_xr_radial_generation() ||
       intent->kind!=DG_RADIAL_EQUIP_ITEM || !item_consumable(intent->id) || !g_controls_active ||
       !g_radial_game.live || !g_radial_game.catalog.valid || g_item_use.pending ||
       g_radial_game.commit.phase!=DG_RADIAL_COMMIT_IDLE ||
       player!=g_radial_game.catalog.player || inventory!=g_radial_game.catalog.inventory ||
       intent->version!=g_radial_game.catalog.version)return;
    g_item_use.intent=*intent;g_item_use.player=player;g_item_use.inventory=inventory;
    g_item_use.ms=now;g_item_use.tick=(uint64_t)(DWORD)g_b.c_ticks;g_item_use.pending=1;
}
static int item_code_matches(uint64_t base,uint64_t offset,const unsigned char *code,size_t n)
{
    unsigned char bytes[sizeof item_use_code];
    return n<=sizeof bytes && radial_live_read(NULL,base+offset,bytes,n) && !memcmp(bytes,code,n);
}
static void item_use_tick(int safe,const dg_radial_native_snapshot *s,uint64_t status,uint64_t now)
{
    uint64_t base=g_radial_game.image.base,inventory=0,items=0;
    unsigned mask=0;short before=0,after=0,count0=0,count1=0;
    int heal0=0,heal1=0,id;
    union {uint64_t align;unsigned char b[0xa0];} work;
    union {uint64_t align;unsigned char b[0x40];} pad;
    if(!g_item_use.pending)return;
    g_item_use.pending=0; /* Spend before validation/call; never auto-retry. */
    id=g_item_use.intent.id;
    if(!safe || now<g_item_use.ms || now-g_item_use.ms>100 ||
       (uint64_t)(DWORD)g_b.c_ticks!=g_item_use.tick ||
       g_item_use.intent.epoch!=dg_xr_radial_generation() ||
       g_item_use.intent.version!=g_radial_game.catalog.version ||
       s->inventory.player_identity!=g_item_use.player ||
       s->inventory.inventory_identity!=g_item_use.inventory ||
       !item_consumable(id) || !(s->eligible_items&(UINT64_C(1)<<id)) ||
       !radial_engine_ready(s,status,DG_RADIAL_EQUIP_ITEM) ||
       g_b.fire.state!=DG_FIRE_IDLE || g_radial_game.commit.phase!=DG_RADIAL_COMMIT_IDLE ||
       !item_code_matches(base,0x1ef840,item_use_code,sizeof item_use_code) ||
       !item_code_matches(base,0x74a10,item_count_code,sizeof item_count_code) ||
       !item_code_matches(base,0x74890,item_decrement_code,sizeof item_decrement_code) ||
       !radial_live_read(NULL,base+0x949340,&inventory,8) || inventory!=g_item_use.inventory ||
       !radial_live_read(NULL,base+0x1540c30,&items,8) || items!=s->inventory.items_identity ||
       !radial_live_read(NULL,base+0x16e9930,&mask,4) || !mask ||
       !radial_live_read(NULL,s->inventory.items_identity+id*2,&before,2) || before<=0 ||
       !radial_live_read(NULL,inventory+0x1590,&count0,2) ||
       !radial_live_read(NULL,base+0x16e9950,&heal0,4)) {
        if(g_b.log)g_b.log("  item use: refused id=%d (context/readiness/layout)\r\n",id);
        return;
    }
    memset(&work,0,sizeof work);memset(&pad,0,sizeof pad);
    *(uint64_t *)(work.b+0x38)=(uint64_t)(ULONG_PTR)pad.b;
    *(int *)(work.b+0x94)=id;*(unsigned *)(pad.b+8)=mask;
    __try { ((void (__fastcall *)(void *))(ULONG_PTR)(base+0x1ef840))(work.b); }
    __except(EXCEPTION_EXECUTE_HANDLER) {
        if(g_b.log)g_b.log("  item use: native fault id=%d; no retry\r\n",id);return;
    }
    if(radial_live_read(NULL,s->inventory.items_identity+id*2,&after,2) &&
       radial_live_read(NULL,inventory+0x1590,&count1,2) &&
       radial_live_read(NULL,base+0x16e9950,&heal1,4) && g_b.log)
        g_b.log("  item use: native id=%d stock=%d->%d heal_adjust=%d->%d ration_counter=%d->%d\r\n",
            id,before,after,heal0,heal1,count0,count1);
}
