/* Physical buttons stay visible when denied so held chords never turn into
 * fresh gameplay actions when radial/calibration/context ownership ends. */
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
    if (denied) out->suppressed=DG_IA_ALL;
}
