/* Physical buttons stay visible when denied so held chords never turn into
 * fresh gameplay actions when radial/calibration/context ownership ends. */
/* LOCAL metres, before recenter/game-axis/arm calibration. The published
 * frame already requires a tracked head. Rotate the ear offset with its full
 * orientation, so looking sideways or tilting does not move the gesture away
 * from the ear. Mirrored with the existing left-handed codec binding. */
static int controls_codec_at_ear(const DG_XR_FRAME *frame, int weapon_hand) {
    const DG_XR_RAW_POSE *h=&frame->head_raw;
    const DG_XR_HAND_POSE *p=weapon_hand==0 ?
        &frame->left_hand.grip : &frame->right_hand.grip;
    double n=h->qx*h->qx+h->qy*h->qy+h->qz*h->qz+h->qw*h->qw;
    double side=weapon_hand==0 ? -0.15 : 0.15;
    double dx,dy,dz;
    if (!p->active || !p->tracked || !p->position_valid ||
        !p->orientation_valid || !p->sample_seq || p->pose_age_ms>100 ||
        !(n>=0.99 && n<=1.01)) return 0;
    dx=p->raw_local.px-h->px-side*(1-2*(h->qy*h->qy+h->qz*h->qz)/n);
    dy=p->raw_local.py-h->py-side*2*(h->qx*h->qy+h->qw*h->qz)/n;
    dz=p->raw_local.pz-h->pz-side*2*(h->qx*h->qz-h->qw*h->qy)/n;
    /* 13 cm tolerance around a point 15 cm to the side of the head centre.
     * Positive comparison also refuses NaN/infinite positions. */
    return dx*dx+dy*dy+dz*dz<=0.13*0.13;
}

static void controls_interact_from_frame(const DG_XR_FRAME *frame,
    int allowed, int denied, int weapon_hand, uint64_t epoch,
    DG_INTERACT_SAMPLE *out) {
    const DG_XR_HAND *left,*right,*off;
    memset(out,0,sizeof *out);
    out->epoch=epoch;
    if (!frame) return;
    left=&frame->left_hand; right=&frame->right_hand;
    off=weapon_hand==0 ? right : left;
    out->sample=left->grip.sample_seq;
    out->age_ms=left->grip.pose_age_ms>right->grip.pose_age_ms ?
        left->grip.pose_age_ms:right->grip.pose_age_ms;
    out->valid=allowed && left->grip.active && right->grip.active &&
        left->grip.tracked && right->grip.tracked &&
        left->grip.orientation_valid && right->grip.orientation_valid &&
        out->sample && out->sample==right->grip.sample_seq;
    out->levels=(left->primary_button?DG_IA_ACTION:0) |
        (right->secondary_button?DG_IA_POSTURE:0) |
        (off->trigger_click?DG_IA_MELEE:0) |
        (off->squeeze_click?DG_IA_CAPTURE:0) |
        ((weapon_hand==0 ? left : right)->squeeze_click?DG_IA_CODEC:0);
    out->choke_seq=off->trigger_press_seq;
    out->ladder=allowed==DG_IA_LADDER_CONTEXT;
    out->special=(allowed==DG_IA_LADDER_CONTEXT || allowed==DG_IA_BEYOND_CONTEXT ||
        allowed==DG_IA_LOCKER_CONTEXT) ? allowed : 0;
    if (out->special==DG_IA_BEYOND_CONTEXT || out->special==DG_IA_LOCKER_CONTEXT)
        out->levels|=(left->squeeze_click?DG_IA_PEEP_LEFT:0) |
                     (right->squeeze_click?DG_IA_PEEP_RIGHT:0);
    if (out->ladder) {
        out->levels|=(left->thumbstick_y>0.5f?DG_IA_UP:0) |
                     (left->thumbstick_y< -0.5f?DG_IA_DOWN:0);
        if (!(left->thumbstick_y>=-1.0f && left->thumbstick_y<=1.0f)) out->valid=0;
    }
    if (right->trigger_click && right->secondary_button)
        out->suppressed|=DG_IA_POSTURE;
    if (off->secondary_button) out->suppressed|=DG_IA_MELEE;
    /* Preserve the physical level: the adapter latches denial until release,
     * preventing a held grip from becoming a press on entering the ear zone. */
    if ((out->levels&DG_IA_CODEC) && !controls_codec_at_ear(frame,weapon_hand))
        out->suppressed|=DG_IA_CODEC;
    if (denied) out->suppressed=DG_IA_ALL;
}
