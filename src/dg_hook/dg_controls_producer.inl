/* Included by dg_hook.c: one bridge tick owns this state. The bridge lock
 * serializes stop against the complete producer/consumer span. No game writes
 * or synthetic inventory lives here. The bridge supplies native eligibility
 * and owns all native writes; this producer builds views and release intents. */
#include "dg_interact_bindings.inl"
#include "dg_radial_game.h"
#include "dg_radial_ready.h"
typedef struct {
    int available;
    uint64_t version;
    dg_radial_view hand[2];
    int ids[2][8];
    uint64_t player, inventory;
    int busy;
    int quick_id[2];
} DG_CONTROLS_CATALOG;
static SRWLOCK g_controls_fallback_lock=SRWLOCK_INIT;
static int g_controls_turn_fallback;
static struct {
    dg_radial_owner owner;
    DG_CONTROLS_CATALOG catalog;
    dg_radial_owner_input last_input;
    dg_radial_owner_output last_output;
    uint64_t context;
    int group[2], page[2]; /* kind-indexed navigation, -1 is category ring */
    int favorites_edit[2];
    int navigate_hand; /* hand+1, retain trigger-confirmed gesture until click release */
    uint64_t navigate_epoch;
    uint64_t navigate_player,navigate_inventory,navigate_sequence;
    uint64_t favorites[2]; /* explicit user choices, retained for this session */
    unsigned long stream;
    int source, cached, stopped, overlay_open;
    int ladder_move_claim;
    unsigned diagnostic_clicks;
    uint64_t ladder_claim_sample;
} g_controls;

static void controls_init(void) {
    memset(&g_controls,0,sizeof g_controls);
    dg_radial_owner_init(&g_controls.owner);
    g_controls.context=1;
    g_controls.group[0]=g_controls.group[1]=-1;
    g_controls.stream=g_arm_pose_stream_id;
    g_controls.source=g_source;
}
static int controls_catalog_group(int kind,int id,int group) {
    int first,second,third;
    if (!kind) {
        first=id>=1 && id<=3;
        second=id==4 || id==15 || id==18 || id==19;
        third=(id>=5 && id<=11) || id==17;
    } else {
        first=id==1 || (id>=3 && id<=5);
        second=(id>=6 && id<=8) || id==32 || id>=36;
        third=id==16 || (id>=22 && id<=24) || id==26 || id==27;
    }
    if (group==2) return first;
    if (group==3) return second;
    if (group==4) return kind ? !first && !second && !third : third;
    if (group==5) return kind ? third : !first && !second && !third;
    return 0;
}
static void controls_catalog_build(const dg_radial_game_catalog *native,int weapon_hand) {
    static const char *names[2][6]={
        {"RECENT","FAVORITES","HANDGUNS","RIFLES","EXPLOSIVES","OTHER"},
        {"RECENT","FAVORITES","RECOVERY","PROTECTION","EQUIPMENT","BOXES"}};
    int h;
    memset(&g_controls.catalog,0,sizeof g_controls.catalog);
    g_controls.catalog.available=1;g_controls.catalog.version=native->version;
    g_controls.catalog.player=native->player;g_controls.catalog.inventory=native->inventory;
    g_controls.catalog.busy=native->busy;
    for (h=0;h<2;h++) {
        int id=native->current[h]>0 ? 0:native->previous[h];
        g_controls.catalog.quick_id[h]=-1;
        if (id>=0 && id<64 && id!=native->current[h] &&
            (native->eligible[h]&(UINT64_C(1)<<id)))
            g_controls.catalog.quick_id[h]=id;
    }
    for (h=0;h<2;h++) {
        int kind=h!=weapon_hand,group=g_controls.group[kind],id,slot;
        dg_radial_view *v=&g_controls.catalog.hand[h];
        v->kind=(unsigned)kind;v->count=6;
        if (group<0 || group>5) {
            g_controls.group[kind]=-1;g_controls.page[kind]=0;
            for (slot=0;slot<6;slot++) {
                strcpy_s(v->labels[slot],17,names[kind][slot]);
                g_controls.catalog.ids[h][slot]=-1-slot;
                /* Categories remain inspectable when their entries are unavailable. */
                v->eligible|=1u<<slot;
            }
        } else {
            int ids[64],count=0,pages,start,editing=group==1 && g_controls.favorites_edit[kind];
            int capacity=group==1 && !editing ? 5:6;
            if (!group) {
                unsigned n;
                for (n=0;n<native->recent_count[kind] && n<6;n++) {
                    id=native->recent[kind][n];
                    if (id>=0 && id<64 && native->labels[kind][id][0]) ids[count++]=id;
                }
            } else {
                for (id=0;id<64;id++) if (native->labels[kind][id][0] &&
                    (editing || (group==1 ? (g_controls.favorites[kind]&(UINT64_C(1)<<id))!=0 :
                     controls_catalog_group(kind,id,group)))) ids[count++]=id;
            }
            pages=(count+capacity-1)/capacity;if (!pages) pages=1;
            if (g_controls.page[kind]<0 || g_controls.page[kind]>=pages) g_controls.page[kind]=0;
            start=g_controls.page[kind]*capacity;v->count=8;
            for (slot=0;slot<capacity && start+slot<count;slot++) {
                id=ids[start+slot];g_controls.catalog.ids[h][slot]=id;
                if (editing) snprintf(v->labels[slot],17,"%s %.10s",
                    (g_controls.favorites[kind]&(UINT64_C(1)<<id)) ? "DROP":"ADD",native->labels[kind][id]);
                else if(kind && (id==1 || id==3 || id==4 || id==5))
                    snprintf(v->labels[slot],17,"%.10s %d",native->labels[kind][id],native->quantities[id]);
                else memcpy(v->labels[slot],native->labels[kind][id],17);
                if (editing || (native->eligible[kind]&(UINT64_C(1)<<id))) v->eligible|=1u<<slot;
            }
            if (group==1 && !editing) {
                strcpy_s(v->labels[5],17,"EDIT FAVORITES");v->eligible|=1u<<5;
                g_controls.catalog.ids[h][5]=-102;
            }
            strcpy_s(v->labels[6],17,"BACK");v->eligible|=1u<<6;
            g_controls.catalog.ids[h][6]=-100;
            if (pages>1) {
                strcpy_s(v->labels[7],17,"NEXT");v->eligible|=1u<<7;
                g_controls.catalog.ids[h][7]=-101;
            }
        }
    }
}
static int controls_catalog_valid(int weapon_hand) {
    int h,k,n;
    if (!g_controls.catalog.available || !g_controls.catalog.version) return 0;
    for (h=0;h<2;h++) {
        const dg_radial_view *v=&g_controls.catalog.hand[h];
        if (v->count<6 || v->count>8 ||
            (v->eligible>>v->count) || v->kind!=(unsigned)(h!=weapon_hand)) return 0;
        for (k=0;k<8;k++) {
            for (n=0;n<17 && v->labels[k][n];n++) {
                unsigned char c=(unsigned char)v->labels[k][n];
                if (!((c>='A' && c<='Z') || (c>='a' && c<='z') ||
                      (c>='0' && c<='9') || c==' ' || c=='-' || c=='.')) return 0;
            }
            if (n==17 || (!n && (v->eligible&(1u<<k)))) return 0;
        }
    }
    return 1;
}
static void controls_stop(void *user) {
    (void)user;
    dg_bridge_radial_cancel();
    g_controls.navigate_hand=0;
    input_turn_rate(0.0);
    if (!g_controls.stopped) {
        g_controls.context++;
        g_controls.cached=0;
        g_controls.stopped=1;
        g_controls.overlay_open=0;
        dg_xr_radial_invalidate();
    }
}
static void controls_route_frame(const DG_XR_FRAME *frame, int allowed,
    int fire_idle, uint64_t now_ms, uint64_t generation,
    DG_BRIDGE_CONTROLS_FRAME *out) {
    DG_XR_FRAME routed;
    const DG_XR_FRAME *movement=frame;
    dg_radial_owner_input in, comparable;
    dg_radial_owner_output owned;
    unsigned age=0;
    int h, radial;
    int native_allowed=allowed==DG_CONTROLS_GAMEPLAY;
    int special_allowed=allowed==DG_CONTROLS_LADDER || allowed==DG_CONTROLS_BEYOND || allowed==DG_CONTROLS_LOCKER;
    int weapon_hand=(g_arm_track==ARM_TRACK_L_GRIP ||
                     g_arm_track==ARM_TRACK_L_AIM) ? 0 : 1;
    memset(out,0,sizeof *out);
    memset(&in,0,sizeof in);
    memset(&owned,0,sizeof owned);
    if (g_controls.stream!=g_arm_pose_stream_id || g_controls.source!=g_source) {
        dg_bridge_radial_cancel();
        g_controls.navigate_hand=0;
        unsigned axes=g_controls.owner.axes;
        dg_radial_owner_init(&g_controls.owner);
        g_controls.owner.axes=axes; /* rearm never releases a deflected claim */
        g_controls.stream=g_arm_pose_stream_id; g_controls.source=g_source;
        g_controls.ladder_claim_sample=0;
        g_controls.context++; g_controls.cached=0; g_controls.overlay_open=0;
        dg_xr_radial_invalidate(); generation=0;
    }
    radial=g_controls.catalog.available || g_controls.owner.axes ||
        g_controls.owner.select.hand>=0;
    if (allowed) g_controls.stopped=0;
    if (frame && radial) {
        const DG_XR_HAND *hands[2]={&frame->left_hand,&frame->right_hand};
        in.have_sample=1;
        for (h=0;h<2;h++) {
            const DG_XR_HAND *hand=hands[h];
            if (!hand->grip.active || !hand->grip.tracked ||
                !hand->grip.orientation_valid || hand->grip.pose_age_ms>100)
                in.have_sample=0;
            if (hand->grip.pose_age_ms>age) age=hand->grip.pose_age_ms;
            in.select.primary[h]=hand->thumbstick_click!=0;
            in.select.trigger[h]=hand->trigger_click;
            in.select.x[h]=hand->thumbstick_x;
            in.select.y[h]=hand->thumbstick_y;
            in.select.count[h]=g_controls.catalog.hand[h].count;
            in.select.eligible[h]=g_controls.catalog.hand[h].eligible;
        }
        in.sequence=frame->left_hand.grip.sample_seq;
        if (!in.sequence || in.sequence!=frame->right_hand.grip.sample_seq)
            in.have_sample=0;
    }
    if (radial) {
        in.now_ms=now_ms;
        in.select.now_ms=now_ms; in.select.quick_ms=250;
        in.sample_ms=age<=now_ms ? now_ms-age : 0;
        /* Radial equip also works outside native FPS. Its phase writer checks
         * current pad/game admission; fire and walking keep their FPS gates. */
        in.select.safe=(native_allowed || allowed==DG_CONTROLS_TURN_ONLY ||
            allowed==DG_CONTROLS_RADIAL_ONLY) && in.have_sample;
        in.select.epoch=generation;
        in.select.context=g_controls.context;
        in.select.version=g_controls.catalog.version;
        in.selection_allowed=generation!=0 && !g_controls.catalog.busy && controls_catalog_valid(weapon_hand);
        in.fire_idle=fire_idle;
        in.weapon_hand=weapon_hand;
        in.move_deadzone=in.turn_deadzone=(double)InterlockedCompareExchange(
            &g_turn_deadzone_mils_live,0,0)/1000.0;
        if(g_controls.navigate_hand) {
            int nh=g_controls.navigate_hand-1;
            if(!in.select.safe || !controls_catalog_valid(weapon_hand) ||
               g_controls.navigate_player!=g_controls.catalog.player ||
               g_controls.navigate_inventory!=g_controls.catalog.inventory ||
               g_controls.navigate_epoch!=generation || !in.select.primary[nh] || in.select.primary[1-nh] ||
               in.select.trigger[1-nh])
                g_controls.navigate_hand=0;
            else if(in.selection_allowed && in.sequence>g_controls.navigate_sequence &&
                    !in.select.trigger[0] && !in.select.trigger[1] &&
                    (g_controls.owner.select.hand<0 ||
                     g_controls.owner.select.version!=in.select.version ||
                     g_controls.owner.select.context!=in.select.context)) {
                /* Keep the same physical click gesture across our action's
                   ACK/catalog refresh. Fresh trigger release rearms selection;
                   stick release only closes, never submits another action. */
                dg_radial_owner_init(&g_controls.owner);
                g_controls.owner.triggers=0;
                g_controls.owner.select.armed=1;
                g_controls.owner.select.epoch=in.select.epoch;
                g_controls.owner.select.context=in.select.context;
                g_controls.owner.select.version=in.select.version;
                g_controls.owner.select.hand=nh;
                g_controls.owner.select.count=in.select.count[nh];
                g_controls.owner.select.opened_ms=now_ms;
                g_controls.owner.select.quick_ms=in.select.quick_ms;
                g_controls.owner.select.deflected=1;
                g_controls.owner.select.suppress_release=1;
                g_controls.cached=0;
            }
        }
        /* A repeated coherent XR snapshot can route its previous filtered axes
         * but must not count as another neutral observation or release intent.
         * Compare all semantic fields; time is checked separately. */
        comparable=in;
        comparable.now_ms=g_controls.last_input.now_ms;
        comparable.select.now_ms=g_controls.last_input.select.now_ms;
        comparable.sample_ms=g_controls.last_input.sample_ms;
        if (g_controls.cached && in.select.safe &&
            now_ms>=g_controls.last_input.now_ms &&
            now_ms-g_controls.last_input.now_ms<=100 &&
            !memcmp(&comparable,&g_controls.last_input,sizeof comparable)) {
            owned=g_controls.last_output;
            owned.selection.intent=owned.selection.quick=0;
        } else {
            owned=dg_radial_owner_step(&g_controls.owner,&in);
            g_controls.last_input=in;
            g_controls.last_output=owned;
            g_controls.cached=owned.route_valid;
        }
        if(g_controls.navigate_hand) {
            owned.deny_new_fire=3u;
        }
        if (frame && owned.route_valid) {
            routed=*frame;
            routed.left_hand.thumbstick_x=owned.move_x;
            routed.left_hand.thumbstick_y=owned.move_y;
            routed.right_hand.thumbstick_x=owned.turn_x;
            /* Camera owns vertical input; radial already read the raw axes.
             * The zoom provider reads the coherent raw XR snapshot directly. */
            if(dg_bridge_camera_stick_owned())routed.right_hand.thumbstick_y=0;
            movement=&routed;
        } else movement=NULL;
        if (owned.selection.intent && owned.selection.quick &&
            owned.selection.hand>=0 && owned.selection.hand<2) {
            dg_radial_commit_intent intent;
            int kind=owned.selection.hand!=weapon_hand;
            memset(&intent,0,sizeof intent);
            intent.kind=owned.equip_kind; intent.id=g_controls.catalog.quick_id[kind];
            intent.epoch=owned.epoch; intent.context=owned.context;
            intent.version=owned.version; intent.seq=owned.sequence;
            if (intent.id>=0)
                dg_bridge_radial_offer(&intent,g_controls.catalog.player,g_controls.catalog.inventory,
                    dg_radial_controls_equip_ready(&in,&owned),now_ms);
        }
        if (owned.selection.intent && !owned.selection.quick && owned.selection.hand>=0 &&
            owned.selection.hand<2 && owned.selection.slot>=0 && owned.selection.slot<8) {
            dg_radial_commit_intent intent;
            int entry=g_controls.catalog.ids[owned.selection.hand][owned.selection.slot];
            int kind=owned.selection.hand!=weapon_hand;
            memset(&intent,0,sizeof intent);
            intent.kind=owned.equip_kind;
            intent.id=entry;
            intent.epoch=owned.epoch; intent.context=owned.context;
            intent.version=owned.version; intent.seq=owned.sequence;
            if(owned.selection.trigger_confirm) {
                g_controls.navigate_hand=owned.selection.hand+1;
                g_controls.navigate_epoch=generation;
                g_controls.navigate_player=g_controls.catalog.player;
                g_controls.navigate_inventory=g_controls.catalog.inventory;
                g_controls.navigate_sequence=in.sequence;
            }
            if (entry<0) {
                if (entry>=-6) {g_controls.group[kind]=-entry-1;g_controls.page[kind]=0;g_controls.favorites_edit[kind]=0;}
                else if (entry==-100) {
                    if (g_controls.favorites_edit[kind]) g_controls.favorites_edit[kind]=0;
                    else g_controls.group[kind]=-1;
                    g_controls.page[kind]=0;
                }
                else if (entry==-101) g_controls.page[kind]++;
                else if (entry==-102) {g_controls.favorites_edit[kind]=1;g_controls.page[kind]=0;}
                g_controls.context++;g_controls.cached=0;
            } else if (g_controls.group[kind]==1 && g_controls.favorites_edit[kind]) {
                g_controls.favorites[kind]^=UINT64_C(1)<<entry;
                g_controls.context++;g_controls.cached=0;
            } else if(kind && owned.selection.trigger_confirm &&
                      (entry==1 || entry==3 || entry==4 || entry==5)) {
                dg_bridge_item_use_offer(&intent,g_controls.catalog.player,g_controls.catalog.inventory,
                    dg_radial_controls_equip_ready(&in,&owned),now_ms);
            } else {
                dg_bridge_radial_offer(&intent,g_controls.catalog.player,g_controls.catalog.inventory,
                    dg_radial_controls_equip_ready(&in,&owned),now_ms);
            }
        }
    }
    if (frame) {
        unsigned clicks=(frame->left_hand.thumbstick_click?1u:0u) |
            (frame->right_hand.thumbstick_click?2u:0u);
        if (clicks & ~g_controls.diagnostic_clicks)
            logf_("  radial open: clicks=%u allowed=%d catalog=%d busy=%d valid=%d sample=%d generation=%llu fire_idle=%d armed=%d hand=%d axes=%u triggers=%u raw_trigger=%.4f/%.4f stick=%.3f,%.3f/%.3f,%.3f\r\n",
                clicks,allowed,g_controls.catalog.available,g_controls.catalog.busy,
                controls_catalog_valid(weapon_hand),in.have_sample,
                (unsigned long long)generation,fire_idle,g_controls.owner.select.armed,
                g_controls.owner.select.hand,g_controls.owner.axes,g_controls.owner.triggers,
                frame->left_hand.trigger_value,frame->right_hand.trigger_value,
                frame->left_hand.thumbstick_x,frame->left_hand.thumbstick_y,
                frame->right_hand.thumbstick_x,frame->right_hand.thumbstick_y);
        g_controls.diagnostic_clicks=clicks;
    }
    out->fire_available=fire_from_frame(frame,&out->fire);
    if (fire_idle && (!native_allowed || (radial && owned.deny_new_fire)))
        out->fire.valid=0;
    if (!allowed || special_allowed || allowed==DG_CONTROLS_RADIAL_ONLY) movement=NULL;
    if (special_allowed) g_controls.ladder_move_claim=1;
    if (g_controls.ladder_move_claim && frame) {
        const DG_XR_HAND_POSE *lp=&frame->left_hand.grip,*rp=&frame->right_hand.grip;
        double dz=(double)InterlockedCompareExchange(&g_turn_deadzone_mils_live,0,0)/1000.0;
        if (native_allowed && lp->sample_seq>g_controls.ladder_claim_sample &&
            lp->sample_seq==rp->sample_seq && lp->active && rp->active &&
            lp->tracked && rp->tracked && lp->orientation_valid && rp->orientation_valid &&
            lp->pose_age_ms<=100 && rp->pose_age_ms<=100 && dz>=0 && dz<1 &&
            fabs(frame->left_hand.thumbstick_x)<=dz && fabs(frame->left_hand.thumbstick_y)<=dz)
            g_controls.ladder_move_claim=0;
        if (lp->sample_seq>g_controls.ladder_claim_sample)
            g_controls.ladder_claim_sample=lp->sample_seq;
    }
    if (g_controls.ladder_move_claim && movement) {
        routed=*movement;
        routed.left_hand.thumbstick_x=routed.left_hand.thumbstick_y=0;
        movement=&routed;
    }
    out->move_available=move_from_frame(movement,&out->move);
    /* TURN_ONLY is "no first person held"; with the third-person walk on the
       walk command still travels - the bridge's move_tick decides. */
    if (!native_allowed &&
        !(allowed==DG_CONTROLS_TURN_ONLY &&
          InterlockedCompareExchange(&g_move_third_live,0,0)))
        out->move_available=0;
    controls_interact_from_frame(frame,allowed==DG_CONTROLS_RADIAL_ONLY ? DG_CONTROLS_NONE:allowed,
        radial && (special_allowed ?
            (g_controls.owner.axes || g_controls.owner.select.hand>=0) :
            (!owned.route_valid || owned.deny_new_fire)),weapon_hand,
        g_controls.context*16u+(unsigned)weapon_hand*8u+
        (unsigned)(special_allowed?allowed:0),&out->interact);
}
static void controls_provide(void *user, int allowed, int fire_idle,
    uint64_t now_ms, uint64_t tick, DG_BRIDGE_CONTROLS_FRAME *out) {
    DG_XR_FRAME frame;
    dg_radial_view view;
    uint64_t generation=dg_xr_radial_generation();
    int have=input_get_frame(&frame)&1;
    dg_radial_game_catalog native;
    int weapon_hand=(g_arm_track==ARM_TRACK_L_GRIP || g_arm_track==ARM_TRACK_L_AIM) ? 0:1;
    (void)user; (void)tick;
    memset(&g_controls.catalog,0,sizeof g_controls.catalog);
    if ((allowed==DG_CONTROLS_GAMEPLAY || allowed==DG_CONTROLS_TURN_ONLY ||
         allowed==DG_CONTROLS_RADIAL_ONLY) && dg_bridge_radial_catalog(now_ms,&native)) {
        controls_catalog_build(&native,weapon_hand);
    }
    controls_route_frame(have ? &frame : NULL,allowed,fire_idle,now_ms,generation,out);
    memset(&view,0,sizeof view);
    if ((allowed==DG_CONTROLS_GAMEPLAY || allowed==DG_CONTROLS_TURN_ONLY ||
         allowed==DG_CONTROLS_RADIAL_ONLY) && have && generation &&
        controls_catalog_valid(g_controls.owner.weapon_hand) &&
        (g_controls.owner.select.hand>=0 || g_controls.navigate_hand) &&
        (g_controls.navigate_hand || g_controls.owner.select.deflected ||
         now_ms-g_controls.owner.select.opened_ms>=g_controls.owner.select.quick_ms)) {
        view=g_controls.catalog.hand[g_controls.navigate_hand ? g_controls.navigate_hand-1:g_controls.owner.select.hand];
        view.visible=1;
        view.selected=g_controls.owner.select.hand<0 ? -1:g_controls.owner.select.selected;
    }
    InterlockedExchange(&g_action_route_denied,
        g_controls.owner.axes || g_controls.owner.select.hand>=0 ||
        (out->interact.suppressed&DG_IA_ACTION));
    InterlockedExchange(&g_camera_route_denied,
        (g_controls.owner.triggers&2u) || g_controls.owner.select.hand>=0);
    /* Cancellation is a lock-free fence, not a best-effort hidden mailbox
     * publication. A contended backend must never retain the old visible UI. */
    if (g_controls.overlay_open && !view.visible) {
        dg_xr_radial_hide();
        g_controls.overlay_open=0;
        return;
    }
    g_controls.overlay_open=view.visible;
    if (generation)
        dg_xr_radial_publish(&view,generation,g_controls.context,g_controls.catalog.version);
}

/* Only the explicitly bridge-off session uses the historical camera seam.
 * Never publishes native fire/move or competes with a registered provider.
 * This shared span is drained before cleanup withdraws session admission. */
static void controls_fallback_turn(void) {
    DG_XR_FRAME frame; DG_BRIDGE_MOVE ignored;
    if (!TryAcquireSRWLockShared(&g_controls_fallback_lock)) return;
    if (g_controls_turn_fallback) {
        if (InterlockedCompareExchange(&g_armed,0,0) &&
            !InterlockedCompareExchange(&g_script_menu_active,0,0) &&
            !g_controls.owner.axes && g_controls.owner.select.hand<0)
            move_from_frame((input_get_frame(&frame)&1) ? &frame : NULL,&ignored);
        else input_turn_rate(0);
    }
    ReleaseSRWLockShared(&g_controls_fallback_lock);
}
