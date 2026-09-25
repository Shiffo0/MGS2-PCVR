/* Presentation-only lease on the NORMAL launcher root. Native Act replaces
 * it before its own firing logic. Restore at the next camera seam before any
 * early return, and at shutdown. During grip sight, lease only the channel-0
 * invisible bit as well. Native trigger and rocket camera remain untouched. */
static struct {
    DG_NIKITA_BINDING binding;
    uintptr_t base;
    int held,hidden;unsigned normal_hidden,subject_hidden;
    unsigned long applied,restored,changed_owner;
} g_nikita_visual;

static void nikita_visual_release(void) {
    DG_NIKITA_BINDING now;
    if(!g_nikita_visual.held)return;
    g_nikita_visual.held=0;
    if(dg_aim_capture_nikita_binding(g_nikita_visual.base,g_nikita_visual.binding.arm,&now) &&
       now.player==g_nikita_visual.binding.player && now.actor==g_nikita_visual.binding.actor &&
       now.body==g_nikita_visual.binding.body && now.normal_object==g_nikita_visual.binding.normal_object &&
       now.normal_objs==g_nikita_visual.binding.normal_objs && now.normal_model==g_nikita_visual.binding.normal_model &&
       now.subject_objs==g_nikita_visual.binding.subject_objs &&
       now.subject_model==g_nikita_visual.binding.subject_model &&
       now.native_root==g_nikita_visual.binding.native_root && now.hand==g_nikita_visual.binding.hand) {
        if(g_nikita_visual.hidden) {
            unsigned *nf=(unsigned *)(uintptr_t)(now.normal_objs+0x58);
            unsigned *sf=(unsigned *)(uintptr_t)(now.subject_objs+0x58);
            *nf=(*nf & ~0x1000u)|g_nikita_visual.normal_hidden;
            *sf=(*sf & ~0x1000u)|g_nikita_visual.subject_hidden;
        }
        if(now.normal_root==now.hand) {
            *(uint64_t *)(uintptr_t)(now.normal_objs+0x40)=now.native_root;
            g_nikita_visual.restored++;
        } /* Native Act may already have restored its root. */
    } else g_nikita_visual.changed_owner++; /* never write through an old owner */
}

static void nikita_visual_apply(uint64_t arm,const DG_BRIDGE_ARM_TARGET *t) {



    DG_NIKITA_BINDING n,ctx;uintptr_t base;int scope=0;
    LONG age=(LONG)((DWORD)g_b.c_ticks-(DWORD)g_b.hand_command_tick);
    if((!t) || (!t->write) || (!t->hand_write) || (!t->absolute_aim) || (!t->aim_write) ||
       (t->aim_weapon_id!=6) || (t->aim_arm!=arm) ||
       (t->aim_pair_id!=t->pair_id) || (t->aim_stream_id!=t->stream_id) ||
       (!t->aim_sample_seq) || (t->aim_sample_time<=0) ||
       (!t->position.enabled) || (!t->position.valid) ||
       (!g_b.armed) || (g_b.owner) || (g_b.script_menu_only) || (g_b.s_late_unsafe) ||
       (g_b.fps.state!=DG_FPS_ACTIVE) || (!g_b.ik_active) || (g_b.ik_owned_arm!=arm) ||
       (!g_b.hand_command_requested) || (!g_b.hand_command_valid) ||
       (age<0) || (age>DG_HAND_COMMAND_TICKS) ||
       (g_b.a.gm_player_arm_body<=0x17df698))return;
    base=(uintptr_t)(g_b.a.gm_player_arm_body-0x17df698);
    if((!dg_aim_capture_nikita_binding(base,arm,&n)) ||
       (n.subject_object!=t->aim_subobject) || (n.subject_objs!=t->aim_subobjs) ||
       (n.subject_model!=t->aim_model) || (n.hand!=t->aim_hand))return;
    g_nikita_visual.binding=n;g_nikita_visual.base=base;g_nikita_visual.held=1;
    g_nikita_visual.hidden=0;
    if(TryAcquireSRWLockShared(&g_controls_lock)) {
        if(nikita_context(&ctx) && ctx.actor==n.actor && nikita_mode_now(n.actor,&scope) && scope) {
            unsigned *nf=(unsigned *)(uintptr_t)(n.normal_objs+0x58);
            unsigned *sf=(unsigned *)(uintptr_t)(n.subject_objs+0x58);
            g_nikita_visual.normal_hidden=*nf & 0x1000u;
            g_nikita_visual.subject_hidden=*sf & 0x1000u;
            *nf|=0x1000u;*sf|=0x1000u;g_nikita_visual.hidden=1;
        }
        ReleaseSRWLockShared(&g_controls_lock);
    }
    *(uint64_t *)(uintptr_t)(n.normal_objs+0x40)=n.hand;
    g_nikita_visual.applied++;
}
