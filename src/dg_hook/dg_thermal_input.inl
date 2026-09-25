/* Pure input edge detector. Thermal uses the item hand: left by default,
 * mirrored with the existing codec/weapon hand in left-handed mode.
 * No queued gestures and no game writes. A release must be freshly observed
 * after context loss, stale tracking or a refused press. */
typedef struct {
    uint64_t sample, now_ms, epoch, context, player, inventory;
    int seen, armed, down, claim, hand;
} DG_THERMAL_INPUT;
static int thermal_input_step(DG_THERMAL_INPUT *s,const DG_XR_FRAME *f,
    int weapon_hand,int ready,uint64_t now,uint64_t epoch,uint64_t context,
    const DG_CONTROLS_CATALOG *c,dg_radial_commit_intent *intent) {
    int hand=1-weapon_hand,down,ear,fresh,valid,target,changed;
    const DG_XR_HAND *p,*other;
    uint64_t sample;
    memset(intent,0,sizeof *intent);
    if (!f) { s->armed=0; return 0; }
    p=hand ? &f->right_hand:&f->left_hand;
    other=hand ? &f->left_hand:&f->right_hand;
    down=p->squeeze_click!=0; sample=p->grip.sample_seq;
    ear=controls_codec_at_ear(f,hand);
    /* Ear ownership also consumes unavailable-item attempts. Keep the claim
     * through tracking/context loss; only a fresh valid release clears it. */
    if (down && ear) s->claim=1;
    changed=!s->seen || epoch!=s->epoch || context!=s->context ||
        hand!=s->hand || c->player!=s->player || c->inventory!=s->inventory;
    fresh=sample && (!s->seen || sample>s->sample);
    valid=p->grip.active && p->grip.tracked && p->grip.position_valid &&
        p->grip.orientation_valid && p->grip.pose_age_ms<=100 &&
        other->grip.active && other->grip.tracked && other->grip.orientation_valid &&
        other->grip.pose_age_ms<=100 && sample && sample==other->grip.sample_seq &&
        (!s->seen || (now>=s->now_ms && sample>=s->sample)) &&
        !(s->seen && sample==s->sample && down!=s->down);
    if (changed || !valid || !ready) s->armed=0;
    if (valid && fresh && !down) s->claim=0;
    target=c->actor_item==13 ? 0:13;
    if (valid && fresh && ready && !changed && s->armed && down && ear &&
        c->thermal_owned && (c->eligible_items&(UINT64_C(1)<<target)) &&
        !other->squeeze_click &&
        !p->primary_button && !p->secondary_button && !p->menu_button &&
        !other->primary_button && !other->secondary_button && !other->menu_button) {
        intent->kind=DG_RADIAL_EQUIP_ITEM;intent->id=target;
        intent->thermal_toggle=1;intent->expected_item=c->actor_item;
        intent->epoch=epoch;intent->context=context;intent->version=c->version;
        intent->seq=sample;
    }
    if (fresh && valid) s->armed=ready && !down;
    if (!valid || down) s->armed=0;
    if (!s->seen || sample>s->sample) s->sample=sample;
    s->now_ms=now;s->epoch=epoch;s->context=context;s->hand=hand;
    s->player=c->player;s->inventory=c->inventory;s->down=down;s->seen=1;
    return intent->kind!=0;
}
