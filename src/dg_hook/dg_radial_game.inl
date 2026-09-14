/* Included by dg_bridge.c after its control lock and bridge state. */
#include "dg_radial_game.h"
#include "dg_radial_native_read.h"
#include "dg_radial_phase.h"
static SRWLOCK g_radial_late_lock=SRWLOCK_INIT;
static volatile LONG64 g_radial_late_generation;
static struct { uint64_t player,ms,generation; uint32_t game,menu; int valid; } g_radial_late;
static uint64_t radial_late_revoke(void) {
    return (uint64_t)InterlockedIncrement64(&g_radial_late_generation);
}
static void radial_late_publish(uint64_t player,uint32_t game,uint32_t menu,uint64_t generation) {
    if (!TryAcquireSRWLockExclusive(&g_radial_late_lock)) return;
    g_radial_late.player=player;g_radial_late.game=game;g_radial_late.menu=menu;
    g_radial_late.ms=GetTickCount64();g_radial_late.valid=1;
    g_radial_late.generation=generation;
    ReleaseSRWLockExclusive(&g_radial_late_lock);
}
static int radial_late_box_safe(void) {
    uint64_t now=GetTickCount64();int safe;
    if (!TryAcquireSRWLockShared(&g_radial_late_lock)) return 0;
    safe=g_radial_late.valid && now>=g_radial_late.ms && now-g_radial_late.ms<=100 &&
        !(g_radial_late.player&(DG_PLAYER_UNSAFE_MASK&~UINT64_C(0x4000))) &&
        !(g_radial_late.game&DG_GAME_UNSAFE_MASK) && !(g_radial_late.menu&DG_MENU_UNSAFE_MASK) &&
        g_radial_late.generation==(uint64_t)InterlockedCompareExchange64(&g_radial_late_generation,0,0);
    ReleaseSRWLockShared(&g_radial_late_lock);return safe;
}
static struct {
    dg_radial_inventory_image image;
    dg_radial_inventory_anchors inventory;
    dg_radial_native_anchors native;
    dg_radial_phase_anchors phase;
    DG_DETOUR detour;
    int live, ready, ack_allows_box;
    volatile LONG64 actor_seen, actor_seen_ms;
    uint64_t offered_player, offered_inventory, offered_tick, offered_ms;
    dg_radial_game_catalog catalog;
    dg_radial_commit_state commit;
    unsigned recent_count[2];
    int recent[2][6];
    uint64_t recent_player;
    int previous[2];
} g_radial_game;
static void radial_recent_push(int kind,int id) {
    unsigned n,count;
    if (kind<1 || kind>2 || id<0 || id>=64) return;
    kind--;count=g_radial_game.recent_count[kind];
    for (n=0;n<count;n++) if (g_radial_game.recent[kind][n]==id) break;
    if (n==count) {
        if (count<6) g_radial_game.recent_count[kind]=++count;
        n=count-1;
    }
    while (n) {g_radial_game.recent[kind][n]=g_radial_game.recent[kind][n-1];n--;}
    g_radial_game.recent[kind][0]=id;
}

static int radial_live_read(void *ctx,uint64_t address,void *dst,size_t size) {
    size_t left=size; uint64_t cursor=address;
    (void)ctx;
    if (!address || !dst || !size || address+size<address) return 0;
    while (left) {
        MEMORY_BASIC_INFORMATION m; size_t span;
        if (!VirtualQuery((void *)(ULONG_PTR)cursor,&m,sizeof m) ||
            m.State!=MEM_COMMIT || !protection_readable(m.Protect)) return 0;
        span=(size_t)((ULONG_PTR)m.BaseAddress+m.RegionSize-(ULONG_PTR)cursor);
        if (!span) return 0;
        if (span>left) span=left;
        cursor+=span; left-=span;
    }
    __try { memcpy(dst,(void *)(ULONG_PTR)address,size); }
    __except(EXCEPTION_EXECUTE_HANDLER) { return 0; }
    return 1;
}
static int radial_image_read(void *ctx,uint64_t address,void *dst,size_t size) {
    const LiveImage *im=(const LiveImage *)ctx;
    size_t offset;
    if (address<im->base || address-im->base>im->size) return 0;
    offset=(size_t)(address-im->base);
    if (!all_valid(im->valid,offset,size,im->size)) return 0;
    memcpy(dst,im->bytes+offset,size); return 1;
}
static int radial_actor_present(void) {
    uint64_t current=0,now=GetTickCount64();
    uint64_t stamp=(uint64_t)InterlockedCompareExchange64(&g_radial_game.actor_seen_ms,0,0);
    uint64_t seen=(uint64_t)InterlockedCompareExchange64(&g_radial_game.actor_seen,0,0);
    return g_radial_game.live && seen && now>=stamp && now-stamp<=100 &&
        radial_live_read(NULL,g_radial_game.inventory.player_slot,&current,sizeof current) && current==seen;
}
static void item_use_cancel(void);
void dg_bridge_radial_cancel(void) {
    item_use_cancel();
    g_radial_game.catalog.valid=0;
    g_radial_game.ready=0;
    /* Never turn an uncertain write into an automatically retryable request. */
    if (g_radial_game.commit.phase==DG_RADIAL_COMMIT_AWAIT_ACK)
        g_radial_game.commit.phase=DG_RADIAL_COMMIT_BLOCKED;
    else if (g_radial_game.commit.phase==DG_RADIAL_COMMIT_PENDING)
        g_radial_game.commit.phase=DG_RADIAL_COMMIT_IDLE;
}
int dg_bridge_radial_catalog(uint64_t now,dg_radial_game_catalog *out) {
    if (!out) return 0;
    memset(out,0,sizeof *out);
    if (!g_controls_active || !g_radial_game.live ||
        !g_radial_game.catalog.valid || now<g_radial_game.catalog.sampled_ms ||
        now-g_radial_game.catalog.sampled_ms>100) return 0;
    *out=g_radial_game.catalog;
    out->busy=g_radial_game.commit.phase!=DG_RADIAL_COMMIT_IDLE;
    return 1;
}
void dg_bridge_radial_offer(const dg_radial_commit_intent *intent,
    uint64_t player,uint64_t inventory,int ready,uint64_t now) {
    if (!intent || !g_controls_active || !g_radial_game.live || !ready ||
        !g_radial_game.catalog.valid || intent->version!=g_radial_game.catalog.version ||
        player!=g_radial_game.catalog.player || inventory!=g_radial_game.catalog.inventory)
        return;
    if (dg_radial_commit_offer(&g_radial_game.commit,intent,now)) {
        g_radial_game.offered_player=player;
        g_radial_game.offered_inventory=inventory;
        g_radial_game.offered_tick=(uint64_t)(DWORD)g_b.c_ticks;
        g_radial_game.offered_ms=now;
        g_radial_game.ready=ready;
    }
}
static int radial_environment_safe(uint64_t *status) {
    uint32_t game,scenario,menu,menu_scenario,pad_enable;
    if (!InterlockedCompareExchange(&g_b.armed,0,0) ||
        InterlockedCompareExchange(&g_b.script_menu_only,0,0) || g_b.owner) return 0;
    if (!radial_live_read(NULL,g_radial_game.phase.status_slot,status,sizeof *status) ||
        !radial_live_read(NULL,g_b.a.gm_game_status,&game,sizeof game) ||
        !radial_live_read(NULL,g_b.a.gm_game_status_scn,&scenario,sizeof scenario) ||
        !radial_live_read(NULL,g_b.a.gm_menu_status,&menu,sizeof menu) ||
        !radial_live_read(NULL,g_b.a.gm_menu_status_scn,&menu_scenario,sizeof menu_scenario) ||
        g_b.a.player_pad<4 || !radial_live_read(NULL,g_b.a.player_pad-4,&pad_enable,sizeof pad_enable) ||
        !pad_enable) return 0;
    if (((*status&0x4000u) || InterlockedCompareExchange(&g_b.s_late_unsafe,0,0)) &&
        !radial_late_box_safe()) return 0;
    return !(*status&(DG_PLAYER_UNSAFE_MASK&~UINT64_C(0x4000))) && !((game|scenario)&DG_GAME_UNSAFE_MASK) &&
        !((menu|menu_scenario)&DG_MENU_UNSAFE_MASK);
}
int dg_bridge_controller_radial_now(void) {
    uint64_t status;
    return radial_actor_present() && radial_environment_safe(&status);
}
static int radial_phase_safe(uint64_t now,uint64_t *status) {
    return g_controls_provider && (g_controls_allowed || g_controls_radial_allowed) &&
        now>=g_controls_lease && now-g_controls_lease<=100 && radial_environment_safe(status);
}
static int radial_engine_ready(const dg_radial_native_snapshot *s,
    uint64_t status,int kind) {
    uint64_t flags,flags2; int32_t changed; int8_t scenario;
    const dg_radial_phase_anchors *p=&g_radial_game.phase;
    const dg_radial_inventory_snapshot *i=&s->inventory;
    if (i->desired_weapon!=i->actor_weapon || i->desired_item!=i->actor_item ||
        i->actor_weapon<0 || i->actor_weapon>=DG_RN_WEAPONS ||
        i->actor_item<0 || i->actor_item>=DG_RN_ITEMS ||
        !radial_live_read(NULL,i->player_identity+p->flags_offset,&flags,sizeof flags) ||
        !radial_live_read(NULL,i->player_identity+p->flags2_offset,&flags2,sizeof flags2)) return 0;
    if ((status&UINT64_C(0x50000000040)) ||
        ((status&1u) && !(s->weapon_types[i->actor_weapon]&0x40u) &&
         !(s->item_types[i->actor_item]&4u))) return 0;
    if (kind==DG_RADIAL_EQUIP_WEAPON) {
        if (status&0x4000u) return 0;
        /* CheckChangeWeapon's watch/type restriction, plus conservative refusal
         * of invincibility and behind/caution quick-change paths. */
        if ((status&UINT64_C(0x50000000040)) || (status&0x02000000u) ||
            (flags&0x100u) || (flags2&0x10u) ||
            (s->item_types[i->actor_item]&0x0cu) ||
            ((status&1u) && !(s->weapon_types[i->actor_weapon]&0x40u) &&
             !(s->item_types[i->actor_item]&4u))) return 0;
        if (!radial_live_read(NULL,p->changed_weapon,&changed,sizeof changed) || changed ||
            !radial_live_read(NULL,p->scenario_weapon,&scenario,sizeof scenario) || scenario>=0) return 0;
    } else {
        if ((status&0x04000000u) || (flags&0x200u) || (flags2&0x20u) ||
            !radial_live_read(NULL,p->scenario_item,&scenario,sizeof scenario) || scenario>=0) return 0;
    }
    return 1;
}
#include "dg_item_use.inl"
static void radial_phase_tick(void *player) {
    dg_radial_native_snapshot sample;
    dg_radial_commit_tick tick;
    dg_radial_commit_result result;
    uint64_t now=GetTickCount64(),status=0; int safe,k,id,previous_phase;
    dg_radial_game_catalog next;
    if (!TryAcquireSRWLockShared(&g_controls_lock)) return;
    safe=g_radial_game.live && dg_radial_native_sample(&g_radial_game.image,&g_radial_game.inventory,
            &g_radial_game.native,1,&sample) && sample.eligibility_valid &&
        sample.inventory.player_identity==(uint64_t)(ULONG_PTR)player;
    if (safe) {
        InterlockedExchange64(&g_radial_game.actor_seen,(LONG64)sample.inventory.player_identity);
        InterlockedExchange64(&g_radial_game.actor_seen_ms,(LONG64)now);
    }
    safe=safe && radial_phase_safe(now,&status);
    item_use_tick(safe,&sample,status,now);
    memset(&tick,0,sizeof tick);
    tick.now_ms=now; tick.tick_seq=(uint64_t)(DWORD)g_b.c_ticks;
    tick.epoch=dg_xr_radial_generation();
    tick.context=g_radial_game.commit.pending.context;
    tick.version=g_radial_game.catalog.version;
    if (safe) {
        const dg_radial_commit_intent *pending=&g_radial_game.commit.pending;
        uint64_t eligible=pending->kind==DG_RADIAL_EQUIP_WEAPON ?
            sample.eligible_weapons:sample.eligible_items;
        tick.observation_safe=sample.inventory.player_identity==g_radial_game.offered_player &&
            sample.inventory.inventory_identity==g_radial_game.offered_inventory;
        if ((status&0x4000u) && g_radial_game.commit.phase==DG_RADIAL_COMMIT_AWAIT_ACK &&
            !g_radial_game.ack_allows_box) tick.observation_safe=0;
        tick.observed_weapon=sample.inventory.actor_weapon;
        tick.observed_item=sample.inventory.actor_item;
        tick.all_validation_ready=tick.observation_safe && g_radial_game.ready &&
            tick.tick_seq==g_radial_game.offered_tick && now>=g_radial_game.offered_ms &&
            now-g_radial_game.offered_ms<=100 && g_b.fire.state==DG_FIRE_IDLE &&
            pending->id>=0 && pending->id<64 && (eligible&(UINT64_C(1)<<pending->id)) &&
            radial_engine_ready(&sample,status,pending->kind);
    }
    /* An uncertain proposal is never retried. Once the same native actor has
     * settled both desired slots and passes fresh engine admission, retire the
     * spent transaction. Preserve last_seq so only a new gesture can offer.
     * Bump the catalog token to force neutral rearming in the input owner. */
    if (g_radial_game.commit.phase==DG_RADIAL_COMMIT_BLOCKED && safe &&
        tick.epoch && tick.observation_safe && g_b.fire.state==DG_FIRE_IDLE &&
        tick.tick_seq>g_radial_game.commit.last_tick_seq &&
        now>=g_radial_game.commit.last_tick_ms &&
        radial_engine_ready(&sample,status,g_radial_game.commit.pending.kind)) {
        g_radial_game.commit.phase=DG_RADIAL_COMMIT_IDLE;
        g_radial_game.commit.last_tick_seq=tick.tick_seq;
        g_radial_game.commit.last_tick_ms=now;
        g_radial_game.ready=0;
        g_radial_game.catalog.version++;
        if (g_b.log) g_b.log("  radial equip: RECOVERED settled weapon=%d item=%d; fresh gesture required\r\n",
            sample.inventory.actor_weapon,sample.inventory.actor_item);
    }
    previous_phase=g_radial_game.commit.phase;
    result=dg_radial_commit_step(&g_radial_game.commit,&tick);
    if (previous_phase==DG_RADIAL_COMMIT_AWAIT_ACK &&
        g_radial_game.commit.phase==DG_RADIAL_COMMIT_BLOCKED && g_b.log)
        g_b.log("  radial equip: BLOCKED safe=%d observation=%d epoch=%llu/%llu version=%llu/%llu elapsed=%llu status=0x%llX\r\n",
            safe,tick.observation_safe,(unsigned long long)tick.epoch,
            (unsigned long long)g_radial_game.commit.pending.epoch,
            (unsigned long long)tick.version,(unsigned long long)g_radial_game.commit.pending.version,
            (unsigned long long)(now-g_radial_game.commit.proposed_ms),(unsigned long long)status);
    if (result.acknowledged) radial_recent_push(result.intent.kind,result.intent.id);
    if (result.acknowledged && g_b.log)
        g_b.log("  radial equip: ACK kind=%d id=%d tick=%llu\r\n",
            result.intent.kind,result.intent.id,(unsigned long long)tick.tick_seq);
    if (result.propose_write) {
        g_radial_game.ack_allows_box=(status&0x4000u)!=0 ||
            (result.intent.kind==DG_RADIAL_EQUIP_ITEM && (sample.item_types[result.intent.id]&8u)!=0);
        uint64_t address=sample.inventory.inventory_identity+
            (result.intent.kind==DG_RADIAL_EQUIP_WEAPON ? 0x104u:0x106u);
        /* Desired short only. Native CheckChange* owns previous/changed/actor.
         * A fault leaves the proposal uncertain and is never retried. */
        __try { *(volatile short *)(ULONG_PTR)address=(short)result.intent.id; }
        __except(EXCEPTION_EXECUTE_HANDLER) { g_radial_game.commit.phase=DG_RADIAL_COMMIT_BLOCKED; }
        if (g_b.log) g_b.log("  radial equip: %s kind=%d id=%d tick=%llu\r\n",
            g_radial_game.commit.phase==DG_RADIAL_COMMIT_BLOCKED ? "WRITE FAULT":"PROPOSED",
            result.intent.kind,result.intent.id,(unsigned long long)tick.tick_seq);
    }
    /* Native desired/actor disagreement is expected during the outstanding
     * transaction. Freeze its display catalog while busy, so temporary engine
     * readiness changes cannot invalidate the transaction's own ACK token. */
    if (safe && g_radial_game.commit.phase==DG_RADIAL_COMMIT_AWAIT_ACK) {
        g_radial_game.catalog.sampled_ms=now;
        ReleaseSRWLockShared(&g_controls_lock);
        return;
    }
    memset(&next,0,sizeof next);
    if (safe) {
        if (g_radial_game.recent_player && g_radial_game.recent_player!=sample.inventory.player_identity) {
            memset(g_radial_game.recent_count,0,sizeof g_radial_game.recent_count);
            memset(g_radial_game.recent,0,sizeof g_radial_game.recent);
            memset(g_radial_game.previous,0,sizeof g_radial_game.previous);
        }
        g_radial_game.recent_player=sample.inventory.player_identity;
        next.valid=1; next.player=sample.inventory.player_identity;
        next.inventory=sample.inventory.inventory_identity;
        next.current[0]=sample.inventory.actor_weapon;
        next.current[1]=sample.inventory.actor_item;
        for (k=0;k<2;k++) {
            if (next.current[k]>0 && next.current[k]<64)
                g_radial_game.previous[k]=next.current[k];
            next.previous[k]=g_radial_game.previous[k];
        }
        next.eligible[0]=sample.eligible_weapons; next.eligible[1]=sample.eligible_items;
        for(id=0;id<(int)sample.inventory.item_slots && id<DG_RINV_MAX_SLOTS;id++)
            next.quantities[id]=sample.inventory.items[id];
        memcpy(next.recent_count,g_radial_game.recent_count,sizeof next.recent_count);
        memcpy(next.recent,g_radial_game.recent,sizeof next.recent);
        if (!radial_engine_ready(&sample,status,DG_RADIAL_EQUIP_WEAPON)) next.eligible[0]=0;
        if (!radial_engine_ready(&sample,status,DG_RADIAL_EQUIP_ITEM)) next.eligible[1]=0;
        for (k=0;k<2;k++) for (id=0;id<64;id++) {
            const char *label=dg_radial_native_label(k,id);
            if (label) {
                unsigned n;
                for (n=0;n<16 && label[n];n++) {
                    char c=label[n];
                    next.labels[k][id][n]=((c>='A' && c<='Z') || (c>='a' && c<='z') ||
                        (c>='0' && c<='9') || c=='-') ? c:' ';
                }
            }
        }
    }
    next.version=g_radial_game.catalog.version;
    next.sampled_ms=g_radial_game.catalog.sampled_ms;
    next.busy=g_radial_game.catalog.busy;
    if (memcmp(&next,&g_radial_game.catalog,sizeof next)) next.version++;
    if (!next.version) next.version=1;
    next.sampled_ms=now;
    g_radial_game.catalog=next;
    ReleaseSRWLockShared(&g_controls_lock);
}
static void radial_game_resolve(const LiveImage *image) {
    memset(&g_radial_game,0,sizeof g_radial_game);
    AcquireSRWLockExclusive(&g_radial_late_lock);
    memset(&g_radial_late,0,sizeof g_radial_late);
    ReleaseSRWLockExclusive(&g_radial_late_lock);
    g_radial_game.image.base=image->base; g_radial_game.image.size=image->size;
    g_radial_game.image.approved_identity=1;
    g_radial_game.image.read=radial_image_read; g_radial_game.image.ctx=(void *)image;
    if (dg_radial_inventory_resolve(&g_radial_game.image,&g_radial_game.inventory)==7 &&
        dg_radial_native_resolve(&g_radial_game.image,&g_radial_game.inventory,&g_radial_game.native))
        dg_radial_phase_resolve(&g_radial_game.image,&g_radial_game.inventory,&g_radial_game.phase);
    g_radial_game.image.read=radial_live_read; g_radial_game.image.ctx=NULL;
}
static void radial_game_install(void) {
    const char *why="unresolved";
    if (g_radial_game.phase.valid)
        g_radial_game.live=dg_detour_install(&g_radial_game.detour,
            (void *)(ULONG_PTR)g_radial_game.phase.weapon_entry,(void *)radial_phase_tick,
            (void *)(ULONG_PTR)g_radial_game.phase.weapon_entry,
            (void *)(ULONG_PTR)g_radial_game.phase.weapon_end,&why);
    if (g_b.log) g_b.log("  radial native phase: %s (%s)\r\n",
        g_radial_game.live ? "installed":"unavailable",g_radial_game.live ? "resolved":why);
}
