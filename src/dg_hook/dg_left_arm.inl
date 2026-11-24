/* Included after the right solver: independent ownership of joints 8/9.
   Joint 10 is independently owned only while free-wrist tracking is valid. */
#include "dg_twohand.h"
#include "dg_m9_contact.h"
#include "dg_twohand_swing.h"
#include "dg_weapon_aim.h"
#include "dg_reach_centre.h"
#define DG_LEFT_MASK ((1ULL << 8) | (1ULL << 9))
typedef struct {
    ULONGLONG arm, objs, mctrl, adjust;
    unsigned long stream, pair;
    LONG settle;
    int active, initialized, root_valid;
    float cached[8];
    int wrist_active;
    float wrist_before[4],wrist_cached[4];
    double wrist_seen[4];
    DG_FREE_WRIST_STATE wrist;
    DG_ARM_MAP_STATE map;
    DG_POSITION_STATE camera_position;
    double root0[4], stick0;
    int stick_valid;
    DG_TWOHAND grip;
    unsigned long long weapon;
    double anchor[3];
    int anchor_valid;
    DG_M9_CONTACT slide_contact;
    double wrist_world[4];
    int wrist_world_valid;
    unsigned long accepted, refused;
    /* Long-weapon steering (vr_twohand_steer): the last hand-local swing, the
       weight it is applied with and what the right solve actually applied. */
    double swing_L[4], swing_w, swing_applied[4];
    int swing_have;
    unsigned long long swing_weapon;
    unsigned long swing_stream, swing_pair, swing_applied_pair;

    DG_REACH_CENTRE centre;
    int centre_frame;   /* 1 tracking space, 3 root/view; a switch restarts the filter */
    int anchor_model;   /* twohand_pistol: anchor is the USP/SOCOM magazine-table point, not a native sample */

    int clav_active;
    float clav_cached[4], clav_before[4];
    double clav_prev[4], clav_next[4];
} DG_LEFT_STATE;
static DG_LEFT_STATE g_left;
static volatile LONG g_left_accepted, g_left_refused, g_left_support_pairs;
static volatile LONG g_left_status, g_left_target[3], g_left_blend;
static volatile LONG g_left_reason;
static volatile LONG g_left_centre_used, g_left_centre_resets, g_left_centre_clamped;
#define LEFT_NOW_MS() GetTickCount64()
/* m9_slide_hand: what the last M9 pair did with the hand (1 slide attached, 2 contact
   allowed, 4 palm geometry, 8 M9, 16 hand driven onto the slide, 32 IK could not reach). */
static volatile LONG g_left_m9_gate;
static volatile LONG g_left_swing_frames, g_left_swing_refused, g_left_swing_applied;
/* Last computed swing: foregrip lever in mm and swing angle in 0.1 degree (heartbeat). */
static volatile LONG g_left_swing_lever_mm, g_left_swing_deci_deg;

static volatile LONG g_left_support_overreach, g_left_anchor_kept;
/* A free hand keeps sampling the native support pose, which is off the foregrip
   on the first trigger-HOLD frames and while the native animation changes
   (moving, recoil). Once a rifle has been held, its foregrip offset is known;
   a free sample further than this from it is not the foregrip. */
#define DG_TWOHAND_ANCHOR_JUMP_MM 100.0
/* With vr_twohand_reach=clamp a rifle may still be taken this far beyond reach. */
#define DG_TWOHAND_GRAB_SLACK_MM 50.0

#define DG_TWOHAND_PISTOL_GRAB_SLACK_MM 150.0

#define DG_LEFT_CLAV_BIT (1ULL << 7)
#define DG_LEFT_PROTRACT_MAX_DEG 30.0
#define DG_LEFT_PROTRACT_REACH 0.985
static volatile LONG g_left_protract_pairs, g_left_protract_capped, g_left_protract_foreign;
typedef struct { unsigned long long weapon; ULONGLONG arm; double anchor[3]; int valid; } DG_LEFT_ANCHOR_MEM;
static DG_LEFT_ANCHOR_MEM g_left_anchor_mem[4];
static DG_LEFT_ANCHOR_MEM *left_anchor_mem(unsigned long long weapon, ULONGLONG arm, int create)
{
    int i;
    for (i = 0; i < 4; i++)
        if (g_left_anchor_mem[i].valid && g_left_anchor_mem[i].weapon == weapon && g_left_anchor_mem[i].arm == arm)
            return &g_left_anchor_mem[i];
    if (!create) return NULL;
    for (i = 0; i < 4; i++)
        if (!g_left_anchor_mem[i].valid || g_left_anchor_mem[i].arm != arm) return &g_left_anchor_mem[i];
    return &g_left_anchor_mem[(int)(weapon & 3)];
}

static unsigned long long g_left_anchor_weapon;
static void left_anchor_weapon_seen(unsigned long long weapon)
{
    if (!weapon || weapon == g_left_anchor_weapon) return;
    if (g_left_anchor_weapon) {
        int i, any = 0;
        for (i = 0; i < 4; i++) any |= g_left_anchor_mem[i].valid;
        if (any && g_b.log)
            g_b.log("  twohand: anchor memory cleared (weapon %llu -> %llu)\r\n", g_left_anchor_weapon, weapon);
        memset(g_left_anchor_mem, 0, sizeof g_left_anchor_mem);
    }
    g_left_anchor_weapon = weapon;
}
static unsigned left_twohand_enter_mm(void) { unsigned v = (unsigned)dg_bridge_twohand_latch() & 0xFFFF; return v ? v : 120; }
static unsigned left_twohand_exit_mm(void) { unsigned v = ((unsigned)dg_bridge_twohand_latch() >> 16) & 0xFFFF; return v ? v : 250; }

typedef struct {
    unsigned long seq; int weapon, gate, grip_ok, left_ok, persistent;
    double grip_trk[3], left_trk[3], shoulder_mm, reach_max, reach_min, dist_mm, lever_mm, mm_per_m;

    int overreach, from_gate, released, refuse_reason, fresh, enter_mm, exit_mm;
} DG_LEFT_PROBE;
static DG_LEFT_PROBE g_left_probe;

static DG_LEFT_PROBE g_left_probe_tr;
static unsigned long g_left_probe_tr_seen, g_left_probe_tr_count, g_left_probe_tr_lines;
static int g_left_probe_solver_gate = -2;
static void left_probe_transition(int gate, int was_held, int held, int reason, int fresh)
{
    if (gate == g_left_probe_solver_gate && !(was_held && !held)) return;
    g_left_probe_tr = g_left_probe;
    g_left_probe_tr.from_gate = g_left_probe_solver_gate;
    g_left_probe_tr.gate = gate;
    g_left_probe_tr.released = was_held && !held;
    g_left_probe_tr.refuse_reason = reason;
    g_left_probe_tr.fresh = fresh;
    g_left_probe_tr.seq = ++g_left_probe_tr_count;
    g_left_probe_solver_gate = gate;
}
static unsigned long g_left_probe_seen, g_left_probe_lines;
static int g_left_probe_last_gate = -1;
static ULONGLONG g_left_probe_ms;
static unsigned long g_left_blade_snap_pair;
static double g_left_blade_snap_wanted[3],g_left_blade_clamp;

static volatile LONG g_left_support_gate[7];

static const char *const g_left_gate_names[8]={"aim/input","right-commit","anchor","unreachable","distance","dwell","attached","safety"};
static void left_support_probe_line(const DG_LEFT_PROBE *p, const char *head)
{
    char where[200],off[120];
    if (p->grip_ok)
        sprintf_s(where,sizeof where,"foregrip at tracking (%.3f %.3f %.3f) m (%s mapping)",p->grip_trk[0],p->grip_trk[1],p->grip_trk[2],
                  p->persistent?"absolute":"calibrated");
    else sprintf_s(where,sizeof where,"foregrip in tracking space unknown (%s)",
                   !p->left_ok?"left controller position not valid":p->persistent?"absolute left mapping refused the camera/view":
                   "left position mapping not calibrated");
    if (p->left_ok && p->grip_ok)
        sprintf_s(off,sizeof off,", left grip now (%.3f %.3f %.3f) m = %.0f mm off",p->left_trk[0],p->left_trk[1],p->left_trk[2],
                  1000.0*sqrt((p->grip_trk[0]-p->left_trk[0])*(p->grip_trk[0]-p->left_trk[0])+(p->grip_trk[1]-p->left_trk[1])*(p->grip_trk[1]-p->left_trk[1])+
                              (p->grip_trk[2]-p->left_trk[2])*(p->grip_trk[2]-p->left_trk[2])));
    else off[0]=0;
    g_b.log("  support probe: weapon %d %s | %s%s | latch distance %.0f mm (enter %u, exit %u) |"
            " foregrip %.0f mm from the left shoulder, reach %.0f..%.0f mm, lever %.0f mm%s%s\r\n",
            p->weapon,head,where,off,p->dist_mm,left_twohand_enter_mm(),left_twohand_exit_mm(),
            p->shoulder_mm,p->reach_min,p->reach_max,p->lever_mm,
            p->overreach ? " -> held beyond the left arm: the arm IK clamps, the latch stays" : "",
            p->gate==3 && p->shoulder_mm>p->reach_max ? " -> beyond the left arm: the foregrip follows the RIGHT controller, bring it closer to the body" :
            p->gate==3 ? " -> too close to the left shoulder: move the RIGHT controller away from the body" : "");
}
static void left_support_probe_log(void)
{
    const DG_LEFT_PROBE *p=&g_left_probe;
    ULONGLONG now=GetTickCount64();
    char head[200];
    if (!g_b.log) return;
    if (g_left_probe_tr.seq!=g_left_probe_tr_seen && g_left_probe_tr_lines<600) {
        const DG_LEFT_PROBE *q=&g_left_probe_tr;
        int g=q->gate<0||q->gate>7?7:q->gate,f=q->from_gate;
        g_left_probe_tr_seen=q->seq; g_left_probe_tr_lines++;
        sprintf_s(head,sizeof head,"gate change %s -> %s%s%s",f<0||f>7?"(start)":g_left_gate_names[f],g_left_gate_names[g],
                  q->released ? (g==7 ? " RELEASED by a safety refusal" : g==4 ? " RELEASED by the hand-foregrip distance" :
                                 g==3 ? " RELEASED by the reach gate" : " RELEASED (no longer aiming/committed)") : "",
                  q->fresh ? "" : " (values from the last measured pair)");
        if (g==7) {
            char r[32];sprintf_s(r,sizeof r," (refuse reason %d)",q->refuse_reason);
            strcat_s(head,sizeof head,r);
        }
        left_support_probe_line(q,head);
    }
    if (p->seq==g_left_probe_seen || p->gate<2 || p->gate>6) return;
    if (p->gate==g_left_probe_last_gate && now-g_left_probe_ms<1000) return;
    if (g_left_probe_lines>=600) return;
    g_left_probe_seen=p->seq; g_left_probe_last_gate=p->gate; g_left_probe_ms=now; g_left_probe_lines++;
    sprintf_s(head,sizeof head,"gate %s",g_left_gate_names[p->gate]);
    left_support_probe_line(p,head);
}
/* Weapon-only pose streams can retain the physical left-hand calibration.
   Older callers use the main stream; owner and explicit calibration changes
   still invalidate the left epoch. */
static unsigned long left_calibration_stream(const DG_BRIDGE_ARM_TARGET *t)
{
    return t->left_stream_id ? t->left_stream_id : t->stream_id;
}

static int left_owner(ULONGLONG *mc, ULONGLONG *adj)
{
    ULONGLONG arm;
    if (!g_b.a.gm_player_arm_body) return 0;
    arm = *(volatile ULONGLONG *)(ULONG_PTR)g_b.a.gm_player_arm_body;
    if (!plausible_ptr(arm) || arm != g_left.arm ||
        region_end(arm) < arm + 0x10) return 0;
    if (*(volatile ULONGLONG *)(ULONG_PTR)arm != g_left.objs) return 0;
    *mc = *(volatile ULONGLONG *)(ULONG_PTR)(arm + 8);
    if (!plausible_ptr(*mc) || *mc != g_left.mctrl ||
        region_end(*mc) < *mc + 0x50 || RD32(*mc + 0x14) < 21 ||
        RD32(*mc + 0x14) > 255) return 0;
    *adj = *(volatile ULONGLONG *)(ULONG_PTR)(*mc + 0x48);
    return plausible_ptr(*adj) && *adj == g_left.adjust &&
        region_end(*adj) >= *adj + 11 * 16;
}

static int left_arm_show_unarmed(ULONGLONG arm, const DG_BRIDGE_ARM_TARGET *t)
{
    ULONGLONG current = 0, mc, adj;
    LONG weapon = -1;
    if (!t || resolve_player(&current, NULL, &weapon) != DG_RESOLVE_OK || current != arm) return 0;
    if (weapon == 13)
        return dg_bridge_blade_braced() && t->aim_write && t->aim_weapon_id == 13 &&
            t->aim_arm == arm && t->aim_pair_id == t->pair_id && g_b.ik_active;
    return weapon == 0 && t->left_enabled && t->left_valid && g_left.active &&
        g_left.arm == arm && g_left.stream == left_calibration_stream(t) &&
        g_left.pair == t->pair_id && left_owner(&mc, &adj);
}

static void left_wrist_release(void)
{
    g_left.wrist_world_valid=0;
    ULONGLONG mc,adj;
    if(g_left.wrist_active && left_owner(&mc,&adj) &&
       !memcmp((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_cached,16)) {
        memcpy((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_before,16);
        *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&=~(1ULL<<10);
    }
    g_left.wrist_active=0;memset(&g_left.wrist,0,sizeof g_left.wrist);
}

/* socom_gap: a slot-7 adjust (adjust space) as a world rotation; the all-zero state of a cleared g_left is the
   identity (no protraction). */
static int left_clav_world(const DG_ADJ_FRAME *frame,const double a[4],double w[4])
{
    if(a[0]==0 && a[1]==0 && a[2]==0 && (a[3]==0 || a[3]==1)) {w[0]=w[1]=w[2]=0;w[3]=1;return 1;}
    return adjust_quat_to_world(frame,a,w);
}
/* socom_gap: give slot 7 back (only while it still holds this writer's bytes) and clear the protraction. */
static void left_clav_release(ULONGLONG mc,ULONGLONG adj)
{
    if(g_left.clav_active && mc && adj && !memcmp((void *)(ULONG_PTR)(adj+7*16),g_left.clav_cached,16)) {
        memcpy((void *)(ULONG_PTR)(adj+7*16),g_left.clav_before,16);
        *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&=~DG_LEFT_CLAV_BIT;
    }
    g_left.clav_active=0;
}
/* socom_gap: the clavicle turn about joint 7 (C) that brings the shoulder (S) to `want` from the target (T):
   in the plane of S-C and T-C, towards T, at most max_rad. 0 when T is already within `want`. Returns the
   angle; q = the world rotation (identity for 0). Closed form: |T-S'|^2 = D^2+L^2-2LD cos(phi-theta). */
static double left_protract_solve(const double C[3],const double S[3],const double T[3],double want,
                                  double max_rad,double q[4])
{
    double u[3],v[3],k[3],L,D,kl,phi,c,th;int i;
    q[0]=q[1]=q[2]=0;q[3]=1;
    for(i=0;i<3;i++){u[i]=S[i]-C[i];v[i]=T[i]-C[i];}
    L=sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);D=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if(!isfinite(L) || !isfinite(D) || !isfinite(want) || L<1 || D<1 || vdist3(T,S)<=want) return 0;
    k[0]=u[1]*v[2]-u[2]*v[1];k[1]=u[2]*v[0]-u[0]*v[2];k[2]=u[0]*v[1]-u[1]*v[0];
    kl=sqrt(k[0]*k[0]+k[1]*k[1]+k[2]*k[2]);
    if(!(kl>1e-9*L*D)) return 0;
    phi=atan2(kl,u[0]*v[0]+u[1]*v[1]+u[2]*v[2]);
    c=(D*D+L*L-want*want)/(2*L*D);
    th=c>=1 ? phi : phi-acos(c<-1 ? -1 : c);
    if(!(th>0)) return 0;
    if(th>max_rad) th=max_rad;
    for(i=0;i<3;i++)q[i]=k[i]/kl*sin(th*.5);
    q[3]=cos(th*.5);
    return th;
}
static double left_protract_shoulder(const double clean[5][3],const double target[3],double span,
                                     double shoulder[3],double turn[4])
{
    const double max_rad=DG_LEFT_PROTRACT_MAX_DEG*3.14159265358979323846/180.0;
    double radians=left_protract_solve(clean[1],clean[2],target,span*DG_LEFT_PROTRACT_REACH,max_rad,turn);
    double offset[3];
    int k;
    for(k=0;k<3;k++)offset[k]=clean[2][k]-clean[1][k];
    arm_quat_rotate(turn,offset,shoulder);
    for(k=0;k<3;k++)shoulder[k]+=clean[1][k];
    return radians;
}

static void left_wrist_now(ULONGLONG base,int stride,ULONGLONG mc,ULONGLONG adj,
    const DG_ADJ_FRAME *frame,const DG_BRIDGE_ARM_TARGET *t,
    const double qa[4],const double qb[4],int had_active,double support_blend,
    const double *contact_q)
{
    DG_IK_HAND_IN in;double rows[3][3],native[4],chain[4],inv[4],want[4],q[4],a[4];
    const double identity[4]={0,0,0,1};int r,k;
    g_left.wrist_world_valid=0;
    memcpy(g_left.wrist_seen,identity,sizeof identity);
    if(g_left.wrist_active)for(k=0;k<4;k++)g_left.wrist_seen[k]=g_left.wrist_cached[k];
    if(!t->free_left.enabled || !t->free_left.valid || support_blend>0) {
        left_wrist_release();return;
    }
    if(g_left.wrist_active) {
        if(!(*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&(1ULL<<10)) ||
           memcmp((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_cached,16)) {
            left_wrist_release();return;
        }
    } else if(*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&(1ULL<<10))return;
    memset(&in,0,sizeof in);
    for(r=0;r<3;r++)for(k=0;k<3;k++)
        rows[r][k]=((volatile float *)(ULONG_PTR)(base+10*(ULONGLONG)stride))[r*4+k];
    if(!dg_ik_basis_quat(rows,in.live))goto refuse_wrist;
    for(r=0;r<3;r++) {
        for(k=0;k<4;k++)a[k]=r==2 ? g_left.wrist_seen[k] :
            had_active ? g_left.cached[r*4+k] : identity[k];
        if(!adjust_quat_to_world(frame,a,r==0?in.prev_upper:r==1?in.prev_fore:in.prev_hand))goto refuse_wrist;
    }
    /* Slot 7 sits beneath the upper arm in both chains. Slide contact can
       rotate the clavicle while this function still owns the wrist. */
    double cp[4],cn[4];
    if(!left_clav_world(frame,g_left.clav_prev,cp) ||
       !left_clav_world(frame,g_left.clav_next,cn))goto refuse_wrist;
    dg_ik_quat_mul(in.prev_upper,cp,in.prev_upper);
    dg_ik_quat_mul(in.prev_fore,in.prev_upper,chain);
    dg_ik_quat_mul(in.prev_hand,chain,chain);dg_ik_quat_conj(chain,inv);
    dg_ik_quat_mul(inv,in.live,native);
    if(!adjust_quat_to_world(frame,qa,in.new_upper) ||
       !adjust_quat_to_world(frame,qb,in.new_fore))goto refuse_wrist;
    dg_ik_quat_mul(in.new_upper,cn,in.new_upper);
    /* None starts with the native wrist relation on the TRACKED forearm.
       Capturing the original animation's world direction instead leaves the
       palm pointing back into an arm that IK has already moved elsewhere.
       Only the initial reference changes; subsequent controller rotations
       remain independent of the arm's position and native animation. */
    /* Equipment changes the native animation, not the controller neutral.
       Keep the accepted free-hand reference until tracking/owner/stream loss
       or an explicit calibration releases it. */
    if(g_b.arm_map_unarmed) {
        dg_ik_quat_mul(in.new_fore,in.new_upper,chain);
        dg_ik_quat_mul(chain,native,native);
    }
    hand_profile_stage(0,t,&t->free_left,native);
    if(!dg_free_wrist_step(&g_left.wrist,&t->free_left,native,want))goto refuse_wrist;
    if(contact_q)memcpy(want,contact_q,sizeof want);
    memcpy(in.desired,want,sizeof want);
    if(!dg_ik_hand_adjust(&in,q))goto refuse_wrist;
    dg_pose_quat_blend(identity,q,t->left_weight,q);
    if(!world_quat_to_adjust(frame,q,a))goto refuse_wrist;
    if(!g_left.wrist_active)memcpy(g_left.wrist_before,(void *)(ULONG_PTR)(adj+10*16),16);
    for(k=0;k<4;k++)g_left.wrist_cached[k]=(float)a[k];
    memcpy((void *)(ULONG_PTR)(adj+10*16),g_left.wrist_cached,16);
    *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=1ULL<<10;g_left.wrist_active=1;
    memcpy(g_left.wrist_world,want,sizeof want);g_left.wrist_world_valid=1;return;
refuse_wrist:
    left_wrist_release();
}

#include "dg_left_model.inl"
static void left_arm_release(void)
{
    dg_bridge_model_arm_clear();
    hand_pose_release();
    left_wrist_release();
    ULONGLONG mc, adj;
    int j, k;
    if (g_left.active && left_owner(&mc, &adj)) {
        left_clav_release(mc, adj);   /* socom_gap: slot 7, same rule */
        /* Relinquish only slots still containing this writer's bytes. */
        for (j = 8; j <= 9; j++) {
            volatile float *q = (volatile float *)(ULONG_PTR)(adj + j * 16);
            int ours = 1;
            for (k = 0; k < 4; k++)
                if (q[k] != g_left.cached[(j-8)*4+k]) ours = 0;
            if (ours) {
                q[0] = q[1] = q[2] = 0; q[3] = 1;
                *(volatile ULONGLONG *)(ULONG_PTR)(mc + 0x38) &= ~(1ULL << j);
            }
        }
    }
    memset(&g_left, 0, sizeof g_left);
    InterlockedExchange(&g_left_status,0);
    InterlockedExchange(&g_left_blend,0);
}

/* Measured 55-joint hand: palm38/48 and five three-joint finger chains.
 * Their distal-joint centroid is an inspectable grip contact, not a guessed
 * wrist offset or a claim about the skinned mesh surface. */
static int left_m9_finger_contact(ULONGLONG base,int stride,ULONGLONG end,double local[3])
{
    static const int parents[17]={10,38,39,40,38,42,43,38,45,46,38,48,49,50,48,52,53};
    static const int tips[5]={41,44,47,51,54};
    double rows[3][3],q[4],inv[4],delta[3]={0,0,0},len=0;
    int i,k;
    if(stride<64 || g_b.skel_parents_read<55 || end<base+54*(ULONGLONG)stride+64)return 0;
    for(i=0;i<17;i++)if(g_b.skel_parents[38+i]!=parents[i])return 0;
    if(!looks_like_matrix(base+10*(ULONGLONG)stride))return 0;
    for(i=0;i<3;i++)for(k=0;k<3;k++)rows[i][k]=((float *)(ULONG_PTR)(base+10*(ULONGLONG)stride))[i*4+k];
    if(!dg_ik_basis_quat(rows,q))return 0;
    for(i=0;i<5;i++) {
        float *tip=(float *)(ULONG_PTR)(base+tips[i]*(ULONGLONG)stride);
        float *wrist=(float *)(ULONG_PTR)(base+10*(ULONGLONG)stride);
        if(!looks_like_matrix((ULONGLONG)(ULONG_PTR)tip))return 0;
        for(k=0;k<3;k++)delta[k]+=(tip[12+k]-wrist[12+k])/5.0;
    }
    dg_ik_quat_conj(q,inv);arm_quat_rotate(inv,delta,local);
    for(k=0;k<3;k++)len+=local[k]*local[k];
    return _finite(len) && len>1 && len<250.0*250.0;
}
/* m9_slide_hand: the palm that goes on the slide. Knuckles = the first joint of
   each finger chain (39,42,45 under palm 38; 49,52 under 48). Palm centre =
   halfway from the wrist to their centroid; palm normal = fingers x the widest
   knuckle spread (its sign is chosen at acquisition, dg_m9_palm_grip). Same
   validated topology as left_m9_finger_contact, which must have succeeded. */
static const int g_left_m9_knuckles[5]={39,42,45,49,52};
static int left_m9_palm(ULONGLONG base,int stride,const double finger_local[3],double palm[3],double normal[3])
{
    double rows[3][3],q[4],inv[4],kn[5][3],c[3]={0,0,0},across[3]={0,0,0},best=-1,len;
    float *wrist=(float *)(ULONG_PTR)(base+10*(ULONGLONG)stride);
    int i,j,k;
    for(i=0;i<3;i++)for(k=0;k<3;k++)rows[i][k]=wrist[i*4+k];
    if(!dg_ik_basis_quat(rows,q))return 0;
    dg_ik_quat_conj(q,inv);
    for(i=0;i<5;i++) {
        float *m=(float *)(ULONG_PTR)(base+g_left_m9_knuckles[i]*(ULONGLONG)stride);
        double d[3];
        if(!looks_like_matrix((ULONGLONG)(ULONG_PTR)m))return 0;
        for(k=0;k<3;k++)d[k]=m[12+k]-wrist[12+k];
        arm_quat_rotate(inv,d,kn[i]);
        for(k=0;k<3;k++)c[k]+=kn[i][k]/5.0;
    }
    for(i=0;i<5;i++)for(j=i+1;j<5;j++) {
        double d[3],dd=0;
        for(k=0;k<3;k++){d[k]=kn[j][k]-kn[i][k];dd+=d[k]*d[k];}
        if(dd>best){best=dd;memcpy(across,d,sizeof across);}
    }
    normal[0]=finger_local[1]*across[2]-finger_local[2]*across[1];
    normal[1]=finger_local[2]*across[0]-finger_local[0]*across[2];
    normal[2]=finger_local[0]*across[1]-finger_local[1]*across[0];
    len=sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
    if(!_finite(len) || len<1e-3 || best<1)return 0;
    for(k=0;k<3;k++){normal[k]/=len;palm[k]=0.5*c[k];}
    len=sqrt(palm[0]*palm[0]+palm[1]*palm[1]+palm[2]*palm[2]);
    return _finite(len) && len>1 && len<150;
}
/* Palm witness from the same subjective skeleton the renderer consumes.
   Never substitute the actor's third-person hand or the raw controller. */
static int left_optics_palm(ULONGLONG arm,const DG_BRIDGE_ARM_TARGET *t,double local[3])
{
    ULONGLONG objs,base,end;
    double finger[3],palm[3],normal[3],world[3];
    float wrist[4][4];
    int stride=g_b.skel_stride;
    if(!t || !t->optics.valid || !g_left.active || g_left.arm!=arm ||
       !interact_read(NULL,arm,&objs,8) || objs!=g_left.objs || stride<64)return 0;
    base=objs+DG_OBJS_ARRAY;end=region_end(objs);
    if(!left_m9_finger_contact(base,stride,end,finger) ||
       !left_m9_palm(base,stride,finger,palm,normal) ||
       !interact_read(NULL,base+10*(ULONGLONG)stride,wrist,sizeof wrist))return 0;
    for(int i=0;i<3;i++){
        world[i]=wrist[3][i];
        for(int axis=0;axis<3;axis++)world[i]+=palm[axis]*wrist[axis][i];
    }
    return dg_position_absolute_inverse(&t->left_position,world,local);
}

/* The reach centre is for the switched-on free hand only: never with the switch off, on the foregrip (engaged or
   fading) or under the blade brace. */
static int left_reach_free_hand(LONG mode, const DG_TWOHAND *grip, double brace_w)
{
    return mode && grip && !grip->engaged && !(grip->blend>0) && !(brace_w>0);
}
/* The origin of the player's tracking space in the world and its rotation, from the left position input the controller
   target is built from (world = camera + R*(grip - head)*units, dg_hand_profile_position): O = camera - R*head*units.
   O moves with the game camera (breathing, walking, stick turns through R) but not with the player's real head. */
static int left_tracking_origin(const DG_BRIDGE_ARM_TARGET *t, double origin[3], double q[4])
{
    double head[3], w[3];
    int k;
    if (!t || !t->left_position.enabled || !t->left_position.valid || !dg_position_frame(&t->left_position,q)) return 0;
    for (k=0;k<3;k++) head[k]=t->left_position.view[4+k]*t->left_position.units;
    dg_position_rotate(q,head,w);
    for (k=0;k<3;k++) { origin[k]=t->left_position.camera[12+k]-w[k]; if (!isfinite(origin[k])) return 0; }
    return 1;
}

static struct { ULONGLONG t0; double lo[7][3], hi[7][3], dmin, dmax; unsigned n; } g_left_sway;
static void left_sway_reset(void){ int i,k; g_left_sway.n=0; g_left_sway.dmin=1e300; g_left_sway.dmax=-1e300;
    for(i=0;i<7;i++)for(k=0;k<3;k++){g_left_sway.lo[i][k]=1e300;g_left_sway.hi[i][k]=-1e300;} }
static double left_sway_pp(int i){ double d[3]; int k; for(k=0;k<3;k++)d[k]=g_left_sway.hi[i][k]-g_left_sway.lo[i][k];
    return sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]); }
static void left_sway_meter(const double shoulder[3], const double hand[3], const double org[3], const double cam[3],
                            const double target[3], int centre_used)
{
    const double *v[7]; double rs[3], rh[3], rt[3]; int i, k; ULONGLONG now=GetTickCount64();
    if (!g_left_sway.t0) { g_left_sway.t0=now; left_sway_reset(); }
    for(k=0;k<3;k++){ rs[k]=shoulder[k]-org[k]; rh[k]=hand[k]-org[k]; rt[k]=target[k]-org[k]; }
    v[0]=shoulder; v[1]=hand; v[2]=org; v[3]=rs; v[4]=rh; v[5]=rt; v[6]=cam;
    for(i=0;i<7;i++) for(k=0;k<3;k++) {
        if(v[i][k]<g_left_sway.lo[i][k])g_left_sway.lo[i][k]=v[i][k];
        if(v[i][k]>g_left_sway.hi[i][k])g_left_sway.hi[i][k]=v[i][k];
    }
    { double d=vdist3(hand,target); if(d<g_left_sway.dmin)g_left_sway.dmin=d; if(d>g_left_sway.dmax)g_left_sway.dmax=d; }
    g_left_sway.n++;
    if (now-g_left_sway.t0>=2000) {
        if (g_b.log && g_left_sway.n>1)
            g_b.log("  left sway: %u pairs, p2p mm: shoulder world %.1f trk-rel %.1f | hand world %.1f trk-rel %.1f |"
                    " tracking origin %.1f camera %.1f | target trk-rel %.1f | hand->target %.1f..%.1f mm | centre %s %s\r\n",
                    g_left_sway.n, left_sway_pp(0), left_sway_pp(3), left_sway_pp(1), left_sway_pp(4),
                    left_sway_pp(2), left_sway_pp(6), left_sway_pp(5), g_left_sway.dmin, g_left_sway.dmax,
                    InterlockedCompareExchange(&g_left_reach_centre_on,0,0)==3 ? "root" :
                    InterlockedCompareExchange(&g_left_reach_centre_on,0,0) ? "tracking" : "off",
                    centre_used ? "used" : "not used");
        g_left_sway.t0=now; left_sway_reset();
    }
}
static double left_brace_weight(const DG_BRIDGE_ARM_TARGET *t)
{
    if((dg_bridge_grip_packed()&15)==DG_GRIP_MODE_OFF)return dg_bridge_blade_braced() ? 1.0 : 0.0;
    return (t->brace_weight>0 && t->brace_weight<=1) ? t->brace_weight : 0.0;
}
/* Long-weapon steering, computed after this pair's right solve from the
   UNSWUNG hand (the published hand minus what was applied), so it never
   accumulates. Game thread. */

static int left_unswung(const DG_BRIDGE_ARM_TARGET *t,double unswung[4])
{
    double applied[4]={0,0,0,1},inv[4];
    if(t->absolute_aim){memcpy(unswung,t->aim_world,4*sizeof(double));return dg_ik_quat_normalize(unswung);}
    if(g_left.swing_applied_pair==t->pair_id)memcpy(applied,g_left.swing_applied,sizeof applied);
    dg_ik_quat_conj(applied,inv);dg_ik_quat_mul(g_b.hand_desired,inv,unswung);
    return dg_ik_quat_normalize(unswung);
}
static void left_swing_update(const DG_BRIDGE_ARM_TARGET *t,const double target[3])
{
    double unswung[4],to[3],L[4];int k;
    if(!left_unswung(t,unswung)){InterlockedIncrement(&g_left_swing_refused);return;}
    for(k=0;k<3;k++)to[k]=target[k]-g_b.pred_wrist[k];
    if(dg_twohand_swing_local(unswung,g_left.anchor,to,L)) {
        memcpy(g_left.swing_L,L,sizeof L);g_left.swing_have=1;InterlockedIncrement(&g_left_swing_frames);
        InterlockedExchange(&g_left_swing_lever_mm,(LONG)sqrt(g_left.anchor[0]*g_left.anchor[0]+g_left.anchor[1]*g_left.anchor[1]+g_left.anchor[2]*g_left.anchor[2]));
        InterlockedExchange(&g_left_swing_deci_deg,(LONG)(20.0*acos(fmin(1.0,fabs(L[3])))*180.0/3.14159265358979323846));
    } else InterlockedIncrement(&g_left_swing_refused);
}
/* Called by the right solve, before it turns the controller's aim into the
   hand: the steered aim for a long weapon held two-handed. 0 = leave it. */
static int left_swing_apply(const DG_BRIDGE_ARM_TARGET *t,const double aim[4],double out[4])
{
    double lw[4];
    if((dg_bridge_twohand_flags()&3)!=3 || !dg_weapon_rifle_support((int)t->aim_weapon_id) ||
       !g_left.swing_have || !(g_left.swing_w>0) || g_left.swing_weapon!=t->aim_weapon_id ||
       g_left.swing_stream!=t->stream_id || t->pair_id-g_left.swing_pair>2 || reload_owns_left())
        return 0;
    dg_twohand_swing_weighted(g_left.swing_L,g_left.swing_w,lw);
    dg_ik_quat_mul(aim,lw,out);
    if(!dg_ik_quat_normalize(out))return 0;
    memcpy(g_left.swing_applied,lw,sizeof lw);g_left.swing_applied_pair=t->pair_id;
    InterlockedIncrement(&g_left_swing_applied);
    return 1;
}

static int left_reload_blocks_support(int weapon)
{
    int r;
    if(dg_reload_pattern(weapon)<0)return 0;
    AcquireSRWLockShared(&g_reload_lock);
    r=g_reload.enabled && g_reload.live && !g_reload.fault && g_reload.weapon==weapon &&
      (g_reload.waiting || g_reload.ejected || g_reload.ins_owed || g_reload.anim);
    ReleaseSRWLockShared(&g_reload_lock);
    return r;
}
/* twohand_pistol: the USP (2) and SOCOM (3) support anchor comes from the weapon's own magazine table at
   exe+0x97BF40+pattern*64 (patterns 2 and 8, the bytes reload_resolve witnesses, dg_pistol_reload.inl:54-58),
   not from the native animation: the left hand sits where it would hold this weapon's magazine seated in the
   grip. [0..2] = the seated magazine origin in the weapon root frame, [3..5] = the held magazine origin in the
   left hand joint 10 frame (game units = mm). The M9 has no magazine pattern and keeps its native sample. */
static const double *left_pistol_support_model(int weapon)
{
    static const double usp[6]={17.5,-115.9,-1.2,-17.5,-110,-11.2};
    static const double socom[6]={17.5,-86,14,-17,-95,0};
    if(!g_reload.resolved)return NULL;
    return weapon==2 ? usp : weapon==3 ? socom : NULL;
}

static int left_pistol_root_is_hand(int weapon,ULONGLONG base,LONG stride)
{
    ULONGLONG actor,wp,wobjs,root,now=GetTickCount64();int ok;
    AcquireSRWLockShared(&g_reload_lock);
    actor=g_reload.actor;
    ok=actor && g_reload.weapon==weapon && now>=g_reload.actor_ms && now-g_reload.actor_ms<=500;
    ReleaseSRWLockShared(&g_reload_lock);
    if(!ok || !plausible_ptr(actor) || region_end(actor)<actor+0x68)return 0;
    wp=*(volatile ULONGLONG *)(ULONG_PTR)(actor+0x60);
    if(!plausible_ptr(wp) || region_end(wp)<wp+8)return 0;
    wobjs=*(volatile ULONGLONG *)(ULONG_PTR)wp;
    if(!plausible_ptr(wobjs) || region_end(wobjs)<wobjs+0x48)return 0;
    root=*(volatile ULONGLONG *)(ULONG_PTR)(wobjs+0x40);
    return root==base+6*(ULONGLONG)stride;
}
static volatile LONG g_left_pistol_model_pairs, g_left_pistol_model_refused, g_left_reload_blocked;
static int g_left_pistol_log_gate=-1, g_left_pistol_log_weapon=-1, g_left_pistol_log_blocked, g_left_pistol_log_over;
static unsigned g_left_pistol_log_lines;
/* socom_gap meter (diag): per USP/SOCOM hold, how far the left hand is from the grip anchor (the support wrist
   point). "solved" = this pair's IK wrist against this pair's anchor; "drawn" = joint 10 as the hierarchy drew it
   this pass (last pair's adjusts) against last pair's anchor. One line when the grip is fully blended in (the
   attach), one when the hold ends, with the maxima over the hold. */
static struct { int holding, weapon, sup_ok; unsigned long n, lines; double solved_max, drawn_max, deg_max, sup[3]; } g_left_gap;
static void left_gap_meter(int weapon,double blend,const double support[3],const double solved[3],const double drawn[3],
                           double support_mm,double reach_mm,double eff_mm,double deg,double clav_mm)
{
    if (g_left_gap.holding && (blend<1 || weapon!=g_left_gap.weapon)) {
        if (g_b.log && g_left_gap.lines<400) {
            g_left_gap.lines++;
            g_b.log("  twohand pistol: weapon %d hold ended after %lu full pairs: hand->anchor solved max %.1f mm, drawn max %.1f mm | clavicle max %.1f deg | protraction pairs %ld capped %ld foreign %ld\r\n",
                    g_left_gap.weapon,g_left_gap.n,g_left_gap.solved_max,g_left_gap.drawn_max,g_left_gap.deg_max,
                    (long)g_left_protract_pairs,(long)g_left_protract_capped,(long)g_left_protract_foreign);
        }
        g_left_gap.holding=0;g_left_gap.sup_ok=0;
    }
    if (!(blend>=1) || (weapon!=2 && weapon!=3)) return;
    {
        double s=vdist3(solved,support),d=g_left_gap.sup_ok ? vdist3(drawn,g_left_gap.sup) : -1;
        if (!g_left_gap.holding) {
            g_left_gap.holding=1;g_left_gap.weapon=weapon;g_left_gap.n=0;
            g_left_gap.solved_max=g_left_gap.drawn_max=g_left_gap.deg_max=0;
        }
        /* the attach line on the second full pair: the first one the hierarchy has drawn */
        g_left_gap.n++;
        if (g_left_gap.n==2 && d>=0 && g_b.log && g_left_gap.lines<400) {
            g_left_gap.lines++;
            g_b.log("  twohand pistol: weapon %d hand on the grip: hand->anchor solved %.1f mm, drawn %.1f mm | support %.0f mm from the left shoulder (reach %.0f), %.0f mm from the protracted shoulder | clavicle %.1f deg (%.0f mm)\r\n",
                    weapon,s,d,support_mm,reach_mm,eff_mm,deg,clav_mm);
        }
        if (s>g_left_gap.solved_max) g_left_gap.solved_max=s;
        if (d>g_left_gap.drawn_max) g_left_gap.drawn_max=d;
        if (deg>g_left_gap.deg_max) g_left_gap.deg_max=deg;
        memcpy(g_left_gap.sup,support,sizeof g_left_gap.sup);g_left_gap.sup_ok=1;
    }
}
static void left_arm_now(ULONGLONG arm, const DG_BRIDGE_ARM_TARGET *t,
                         int right_committed)
{
    static const int joint_ids[5] = {2,7,8,9,10};
    ULONGLONG objs, mc, adj, base, end, pw = 0, pa = 0, pe;
    LONG stride;
    DG_ADJ_FRAME frame = ADJ_FRAME_LEGACY;
    DG_ARM_MAP_IN mi;
    DG_ARM_MAP_OUT mo;
    DG_IK_IN ii;
    DG_IK_OUT io;
    DG_IK_ORIENT_IN oi;
    DG_IK_ORIENT_OUT oo;
    double live[5][3], clean[5][3], root[4][4], rb[3][3], rq[4];
    double cq[4] = {0,0,0,1}, inv[4], rel[4], norm;
    double target[3], support[3], delta[3], qa[4], qb[4], blend = 0, brace_w;
    double sway_target[3]; int sway_target_ok=0;
    float slots[8];

    if (g_reload.waiting && g_reload_probe.world_ms && GetTickCount64()-g_reload_probe.world_ms<500) {
        int lok = t->left_position.enabled && t->left_position.valid;
        g_reload_probe.trk_why = !lok ? "left controller position not valid" : NULL;
        g_reload_probe.trk_ok = lok && dg_support_grip_inverse(t->persistent_hands,&g_left.camera_position,&t->left_position,
                                                               g_reload_probe.magwell,g_reload_probe.magwell_trk);
        if (lok && !g_reload_probe.trk_ok)
            g_reload_probe.trk_why = t->persistent_hands ? "absolute left mapping refused the camera/view" : "left position mapping not calibrated";
        if (lok) memcpy(g_reload_probe.left_trk,t->left_position.grip,sizeof g_reload_probe.left_trk);
        g_reload_probe.trk_ms = GetTickCount64();

        g_reload_probe.geo_trk_ok = g_reload_probe.trk_ok && g_reload.insert_mode && g_reload_probe.geo_ms &&
            GetTickCount64()-g_reload_probe.geo_ms<500 &&
            dg_support_grip_inverse(t->persistent_hands,&g_left.camera_position,&t->left_position,g_reload_probe.mag,g_reload_probe.mag_trk) &&
            dg_support_grip_inverse(t->persistent_hands,&g_left.camera_position,&t->left_position,g_reload_probe.well_pt,g_reload_probe.well_trk) &&
            dg_support_grip_inverse(t->persistent_hands,&g_left.camera_position,&t->left_position,g_reload_probe.held_pt,g_reload_probe.held_trk);
        g_reload_probe.geo_trk_ms = g_reload_probe.trk_ms;
    }
    int j,k,r, aiming, had_active, reason=1, support_gate=0, reload_blocked=0;
    int rifle=t ? dg_weapon_rifle_support((int)t->aim_weapon_id) : 0, steer=(dg_bridge_twohand_flags()&3)==3;
    int pistol=t ? dg_weapon_pistol_support((int)t->aim_weapon_id) : 0;

    int was_held=g_left.grip.engaged, probed=0, overreach=0;
    long twohand=dg_bridge_twohand_flags();
    double finger_local[3],contact_q[4];int contact_drive=0,finger_valid=0;
    double palm_local[3],palm_normal[3];int palm_valid=0;
    if (t) left_anchor_weapon_seen(t->aim_weapon_id);
    if (!t || !t->left_enabled || !t->left_valid || !isfinite(t->left_weight) ||
        t->left_weight <= 0 || t->left_weight > 1) {
        InterlockedExchange(&g_left_reason,0);
        if (rifle && was_held) left_probe_transition(7,1,0,0,0);
        left_arm_release(); return;
    }
    if (!plausible_ptr(arm) || region_end(arm) < arm + 0x10) goto refuse;
    objs = *(volatile ULONGLONG *)(ULONG_PTR)arm;
    mc = *(volatile ULONGLONG *)(ULONG_PTR)(arm+8);
    if (!plausible_ptr(objs) || !plausible_ptr(mc) ||
        region_end(mc) < mc+0x50 || RD32(mc+0x14) < 21 || RD32(mc+0x14)>255)
        goto refuse;
    adj = *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x48);
    if (!plausible_ptr(adj) || region_end(adj) < adj+11*16) goto refuse;
    if (!g_left.initialized || g_left.arm != arm || g_left.objs != objs ||
        g_left.mctrl != mc || g_left.adjust != adj || g_left.stream != left_calibration_stream(t)) {
        left_arm_release();
        g_left.arm=arm; g_left.objs=objs; g_left.mctrl=mc; g_left.adjust=adj;
        g_left.stream=left_calibration_stream(t); g_left.settle=g_b.c_ticks+2;
        g_left.initialized=1; return;
    }
    /* Validate current owner and heading before duplicate handling. */
    reason=2;
    if (!left_owner(&mc,&adj)) goto refuse;
    reason=3;
    if (g_b.adjust_frame == 1) {
        if (resolve_player(&pa,&pw,NULL) != DG_RESOLVE_OK || pa != arm) goto refuse;
        pe=region_end(pw);
        if (!pe || pe < pw+0x84) goto refuse;
        frame.live=1;
        frame.valid=arm_frame_yaw(*(volatile short *)(ULONG_PTR)(pw+0x82),frame.q);
        if (!frame.valid) goto refuse;
    }
    aiming = t->twohand_enabled && t->hands_coherent && t->aim_write &&
        t->aim_sample_seq == t->left_sample_seq &&
        t->aim_sample_time == t->left_sample_time &&
        g_b.fire_mode == DG_FIRE_MODE_ON && g_b.fire.state == DG_FIRE_HOLD;

    if (aiming && (reload_owns_left() || left_reload_blocks_support((int)t->aim_weapon_id))) {
        aiming=0;reload_blocked=1;InterlockedIncrement(&g_left_reload_blocked);
    }
    if (!aiming) dg_twohand_reset(&g_left.grip);
    /* Revoke invalid position even on the closing eye of a cached pair. */
    if(t->left_position.enabled) {
        double position_frame[4];
        if(!dg_position_frame(&t->left_position,position_frame)) goto refuse;
    }
    if(!t->free_left.enabled || !t->free_left.valid)left_wrist_release();
    if (t->pair_id == g_left.pair) return;
    g_left.pair=t->pair_id;
    if (g_b.c_ticks <= g_left.settle) return;
    reason=4;
    stride=g_b.skel_stride;
    if (stride < 64 || g_b.skel_parents_read <= 10 ||
        g_b.skel_parents[7]!=2 || g_b.skel_parents[8]!=7 ||
        g_b.skel_parents[9]!=8 || g_b.skel_parents[10]!=9) goto refuse;
    base=objs+DG_OBJS_ARRAY; end=region_end(objs);
    reason=5;
    if (!end || base+10*(ULONGLONG)stride+64>end || !looks_like_matrix(base)) goto refuse;
    for (r=0;r<4;r++) for(k=0;k<4;k++)
        root[r][k]=((volatile float *)(ULONG_PTR)base)[r*4+k];
    for(r=0;r<3;r++) for(k=0;k<3;k++) rb[r][k]=root[r][k];
    if (!dg_ik_basis_quat(rb,rq)) goto refuse;
    for(j=0;j<5;j++) {
        ULONGLONG m=base+joint_ids[j]*(ULONGLONG)stride;
        if (!looks_like_matrix(m)) goto refuse;
        for(k=0;k<3;k++) live[j][k]=((volatile float *)(ULONG_PTR)m)[12+k];
    }
    memcpy(clean,live,sizeof clean);
    had_active=g_left.active;
    /* socom_gap: no clavicle turn unless one is read back below / decided this pair */
    memset(g_left.clav_prev,0,sizeof g_left.clav_prev);g_left.clav_prev[3]=1;
    memcpy(g_left.clav_next,g_left.clav_prev,sizeof g_left.clav_next);
    reason=6;
    if (had_active) {
        if ((*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_MASK)!=DG_LEFT_MASK)
            goto refuse;
        for(j=0;j<8;j++) {
            slots[j]=((volatile float *)(ULONG_PTR)(adj+8*16))[j];
            if (slots[j]!=g_left.cached[j]) goto refuse;
        }
        if (!arm_remove_cached_adjust(&frame,live,slots,clean)) goto refuse;
        if (g_left.clav_active) {
            /* socom_gap: last pair's clavicle turn is in joints 8..10 too. The strip above took q8/q9 off
               (upper = q8.q7.native); what is left is the rigid q7 turn about joint 7: undo it. */
            double a7[4],q7[4],v[3],w[3];
            if (!(*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_CLAV_BIT) ||
                memcmp((void *)(ULONG_PTR)(adj+7*16),g_left.clav_cached,16)) goto refuse;
            for(k=0;k<4;k++)a7[k]=g_left.clav_cached[k];
            if (!adjust_quat_to_world(&frame,a7,q7)) goto refuse;
            for(j=2;j<5;j++){
                for(k=0;k<3;k++)v[k]=clean[j][k]-clean[1][k];
                arm_quat_unrotate(q7,v,w);
                for(k=0;k<3;k++)clean[j][k]=clean[1][k]+w[k];
            }
            memcpy(g_left.clav_prev,a7,sizeof a7);
        }
    } else if (*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_MASK) goto refuse;
    reason=7;
    memset(&mi,0,sizeof mi);
    mi.anchor_shoulder=1; mi.player_reach=t->player_reach_view;
    mi.upper=vdist3(clean[2],clean[3]); mi.fore=vdist3(clean[3],clean[4]);
    if (!arm_world_to_view(root,clean[2],mi.shoulder_view) ||
        !arm_world_to_view(root,clean[4],mi.native_wrist_view)) goto refuse;
    if (!g_left.root_valid) {
        memcpy(g_left.root0,rq,sizeof rq); g_left.root_valid=1;
        g_left.stick0=t->stick_yaw_rad; g_left.stick_valid=t->stick_yaw_valid;
    }
    arm_reference(&g_b.left_reference,t,g_left.root0,NULL,&g_left.stick0);
    dg_ik_quat_conj(rq,inv); dg_ik_quat_mul(inv,g_left.root0,rel);
    norm=sqrt(rel[1]*rel[1]+rel[3]*rel[3]);
    if (!(norm>1e-6)) goto refuse;
    cq[1]=rel[1]/norm; cq[3]=rel[3]/norm;
    if (g_b.arm_comp==1 && t->stick_yaw_valid && g_left.stick_valid) {
        double software=(g_b.follow_head_sign<0 ? -1:1)*(t->stick_yaw_rad-g_left.stick0);
        double half=atan2(cq[1],cq[3])+software*0.5;
        cq[1]=sin(half); cq[3]=cos(half);
    }
    arm_quat_rotate(cq,t->left_wrist_view,mi.controller_view);
    arm_quat_rotate(cq,t->left_shoulder_view,mi.player_shoulder);
    /* Both raw grips share one physical tracking space. Reusing the accepted
       right-hand mapping preserves their relative separation; calibrating
       the left from its native support animation bakes in a second offset.
       Seed only an uncalibrated left hand: equipment must not replace a
       previously accepted unarmed mapping or depend on right-arm settling. */
    if(t->left_position.enabled && t->position.enabled && !t->persistent_hands && !g_b.arm_map_unarmed &&
       !g_left.camera_position.ready) {
        if(!right_committed || !g_b.camera_position.ready) goto refuse;
        g_left.camera_position=g_b.camera_position;
    }
    if(t->left_position.enabled && (t->persistent_hands || g_left.camera_position.ready)) {
        double camera_target[3];
        int ok=t->persistent_hands ? dg_hand_profile_position(&t->left_position,camera_target) :
            dg_position_target(&g_left.camera_position,&t->left_position,camera_target);
        if(!ok ||
           !arm_world_to_view(root,camera_target,mi.desired_view)) goto refuse;
        mi.explicit_target=1;
        reload_left_target(camera_target);
        reload_visual_wrist(arm,t->aim_weapon_id,camera_target);
        if(!arm_world_to_view(root,camera_target,mi.desired_view)) goto refuse;
        memcpy(sway_target,camera_target,sizeof sway_target);sway_target_ok=1;   /* diag sway meter */
    }
    {

        LONG mode=InterlockedCompareExchange(&g_left_reach_centre_on,0,0);
        int free_hand=left_reach_free_hand(mode,&g_left.grip,left_brace_weight(t));
        unsigned long resets=g_left.centre.resets;
        int frame=mode==3 ? 3 : 1;   /* the off state keeps filtering in the tracking frame */
        if(g_left.centre_frame!=frame){memset(&g_left.centre,0,sizeof g_left.centre);g_left.centre_frame=frame;}
        if(frame==3) {
            mi.has_reach_centre=dg_reach_centre_step(&g_left.centre,mi.shoulder_view,LEFT_NOW_MS(),
                                                     t->aim_weapon_id,free_hand,mi.reach_centre_view);
        } else {
            double org[3],oq[4],c[3];
            mi.has_reach_centre=left_tracking_origin(t,org,oq) &&
                dg_reach_centre_tracking(&g_left.centre,clean[2],org,oq,LEFT_NOW_MS(),t->aim_weapon_id,free_hand,c) &&
                arm_world_to_view(root,c,mi.reach_centre_view);
        }
        if(g_left.centre.resets!=resets)InterlockedIncrement(&g_left_centre_resets);
    }
    if (!dg_arm_map_step(&g_left.map,&mi,&mo)) {
        if (mo.flags&DG_ARM_MAP_F_CALIBRATED) return;
        goto refuse;
    }
    if(sway_target_ok){double org[3],oq[4];if(left_tracking_origin(t,org,oq))left_sway_meter(clean[2],live[4],org,t->left_position.camera+12,sway_target,mi.has_reach_centre);}
    if(mi.has_reach_centre) {
        InterlockedIncrement(&g_left_centre_used);
        if(mo.flags&(DG_ARM_MAP_F_CLAMP_REACH|DG_ARM_MAP_F_SOFT_REACH))InterlockedIncrement(&g_left_centre_clamped);
    }
    arm_view_to_world(root,mo.target_view,target);
    if(t->left_position.enabled && !t->persistent_hands && !g_left.camera_position.ready &&
       !dg_position_calibrate(&g_left.camera_position,&t->left_position,
                              target,mo.scale)) goto refuse;
    /* Slide attachment owns the hand ahead of ordinary support grip. */
    {double slide[3];if(t->aim_weapon_id==1 && m9_hand_contact(slide,t->stream_id)) {
        aiming=0;dg_twohand_reset(&g_left.grip);
    }}
    /* Capture a native support offset only from the right solve that finished
       on THIS invocation. No previous-pair desired pose is a valid anchor. */
    if (g_left.weapon != t->aim_weapon_id) {
        g_left.weapon=t->aim_weapon_id; g_left.anchor_valid=0;
        dg_twohand_reset(&g_left.grip);
    }
    if (aiming && right_committed && g_b.have_pred_wrist && g_b.hand_have_desired &&
        (g_b.rec_pair.flags&DG_REC_PAIR_F_BASE)) {
        const double *model=left_pistol_support_model((int)t->aim_weapon_id);
        double hand_q[4]={0,0,0,1};
        support_gate=2;
        if (model) {   /* twohand_pistol: the root must be the right hand joint 6, the left hand frame readable */
            double rows[3][3];
            for(r=0;r<3;r++)for(k=0;k<3;k++)rows[r][k]=((volatile float *)(ULONG_PTR)(base+10*(ULONGLONG)stride))[r*4+k];
            if(!left_pistol_root_is_hand((int)t->aim_weapon_id,base,stride) || !dg_ik_basis_quat(rows,hand_q)) {
                model=NULL;InterlockedIncrement(&g_left_pistol_model_refused);
            }
        }
        if (!model && g_left.anchor_model) {   /* never carry a model point into the native path */
            g_left.anchor_model=0;g_left.anchor_valid=0;dg_twohand_reset(&g_left.grip);
        }
        if (model) {

            if (!g_left.anchor_model || !g_left.anchor_valid || (!g_left.grip.engaged && g_left.grip.blend==0)) {
                double inv_hd[4],local_q[4],held[3];
                dg_ik_quat_conj(g_b.hand_desired,inv_hd);dg_ik_quat_mul(inv_hd,hand_q,local_q);
                arm_quat_rotate(local_q,model+3,held);
                for(k=0;k<3;k++)g_left.anchor[k]=model[k]-held[k];
                g_left.anchor_valid=1;g_left.anchor_model=1;
            }
            InterlockedIncrement(&g_left_pistol_model_pairs);
        } else
        /* Keep sampling while free: the native aiming animation may still
           be entering its support pose on the first trigger-HOLD frame.
           Freeze the offset only when proximity actually engages it. */
        if (!g_left.anchor_valid || (!g_left.grip.engaged && g_left.grip.blend==0)) {
            double sample[3];int sample_ok;
            const DG_LEFT_ANCHOR_MEM *mem=(rifle && !(twohand&DG_TWOHAND_F_ANCHOR_LIVE)) ?
                left_anchor_mem(t->aim_weapon_id,arm,0) : NULL;
            for(k=0;k<3;k++) delta[k]=clean[4][k]-g_b.rec_pair.joint_world[3][k];
            arm_quat_unrotate(g_b.rec_pair.base_now,delta,sample);
            norm=sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
            /* A rifle foregrip sits further from the wrist than a pistol's support hand. */
            sample_ok=_finite(norm) && (!rifle || norm>=DG_SWING_LEVER_MIN_MM) &&
                norm<(rifle ? DG_SWING_LEVER_MAX_MM : 300);

            if (mem && !(sample_ok && vdist3(sample,mem->anchor)<=DG_TWOHAND_ANCHOR_JUMP_MM)) {
                memcpy(g_left.anchor,mem->anchor,sizeof g_left.anchor);g_left.anchor_valid=1;
                InterlockedIncrement(&g_left_anchor_kept);
            } else {
                memcpy(g_left.anchor,sample,sizeof g_left.anchor);g_left.anchor_valid=sample_ok;
            }
        }
        if (g_left.anchor_valid) {
            arm_quat_rotate(g_b.hand_desired,g_left.anchor,delta);
            for(k=0;k<3;k++) support[k]=g_b.pred_wrist[k]+delta[k];
            norm=vdist3(clean[2],support);
            support_gate=3;
            {

                double rmax=(mi.upper+mi.fore)*0.99,rmin=fabs(mi.upper-mi.fore)+1;
                double slack=rifle ? DG_TWOHAND_GRAB_SLACK_MM : DG_TWOHAND_PISTOL_GRAB_SLACK_MM;
                int inside=norm<rmax && norm>rmin;
                int clamp=(rifle || pistol) && !(twohand&DG_TWOHAND_F_REACH_RELEASE);
                int held=g_left.grip.engaged || g_left.grip.blend>0;
                overreach=!inside && clamp && (held || (norm<rmax+slack && norm>rmin));
            }
            if (rifle) {
                DG_LEFT_PROBE *p=&g_left_probe;
                probed=1; p->overreach=0;
                p->seq++; p->weapon=(int)t->aim_weapon_id; p->shoulder_mm=norm;
                p->reach_max=(mi.upper+mi.fore)*0.99; p->reach_min=fabs(mi.upper-mi.fore)+1; p->dist_mm=-1;
                p->lever_mm=sqrt(g_left.anchor[0]*g_left.anchor[0]+g_left.anchor[1]*g_left.anchor[1]+g_left.anchor[2]*g_left.anchor[2]);
                p->left_ok=t->left_position.enabled && t->left_position.valid;
                if (p->left_ok) memcpy(p->left_trk,t->left_position.grip,sizeof p->left_trk);

                p->persistent=t->persistent_hands!=0;
                p->grip_ok=p->left_ok && dg_support_grip_inverse(t->persistent_hands,&g_left.camera_position,&t->left_position,support,p->grip_trk);
                p->mm_per_m=t->persistent_hands ? t->left_position.units :
                    g_left.camera_position.ready ? g_left.camera_position.units*g_left.camera_position.scale : 0;
            }
            if ((norm < (mi.upper+mi.fore)*0.99 && norm > fabs(mi.upper-mi.fore)+1) || overreach) {
                DG_GRIP_TIMES grip_times;
                dg_grip_unpack(dg_bridge_grip_packed(),DG_GRIP_SUPPORT,&grip_times);
                /* Pistols: controller to controller, as before. Rifles: the support
                   hand to the predicted foregrip (game units = mm). */
                double distance_m=rifle ? vdist3(target,support)/1000.0 : t->hands_distance_m;

                if (rifle) {
                    double unswung[4],free_grip[3],tracking_grip[3];
                    memcpy(free_grip,support,sizeof free_grip);
                    if (left_unswung(t,unswung)) {
                        arm_quat_rotate(unswung,g_left.anchor,free_grip);
                        for(k=0;k<3;k++) free_grip[k]+=g_b.pred_wrist[k];
                    }
                    if(t->persistent_hands)
                        distance_m=dg_position_absolute_inverse(&t->left_position,free_grip,tracking_grip)
                            ? vdist3(t->left_position.grip,tracking_grip) : HUGE_VAL;
                    else distance_m=vdist3(target,free_grip)/1000.0;
                }
                double enter_m=rifle ? left_twohand_enter_mm()/1000.0 : DG_TWOHAND_ENTER_M;
                if (rifle) g_left_probe.dist_mm=distance_m*1000.0;
                blend=dg_twohand_step_gate(&g_left.grip,1,1,distance_m,
                    t->left_sample_seq,t->left_sample_time,&grip_times,
                    enter_m,rifle ? left_twohand_exit_mm()/1000.0 : DG_TWOHAND_EXIT_M);

                support_gate=(g_left.grip.engaged && blend>0) ? 6 :
                    (distance_m>enter_m ? 4 : 5);
                if (overreach && blend>0) InterlockedIncrement(&g_left_support_overreach);
                if (rifle) g_left_probe.overreach=overreach && blend>0;

                if (rifle && g_left.grip.engaged) {
                    DG_LEFT_ANCHOR_MEM *mem=left_anchor_mem(t->aim_weapon_id,arm,1);
                    mem->weapon=t->aim_weapon_id;mem->arm=arm;mem->valid=1;
                    memcpy(mem->anchor,g_left.anchor,sizeof mem->anchor);
                }
            } else dg_twohand_reset(&g_left.grip);
        }
    } else {
        support_gate=aiming ? 1 : 0;
        dg_twohand_reset(&g_left.grip);
    }
    InterlockedIncrement(&g_left_support_gate[support_gate]);

    if ((t->aim_weapon_id==2 || t->aim_weapon_id==3) && g_b.log && g_left_pistol_log_lines<400 &&
        (support_gate!=g_left_pistol_log_gate || (int)t->aim_weapon_id!=g_left_pistol_log_weapon ||
         reload_blocked!=g_left_pistol_log_blocked || (overreach && blend>0)!=g_left_pistol_log_over)) {
        static const char *const names[7]={"aim/input","right-commit","anchor","unreachable","distance","dwell","attached"};
        g_left_pistol_log_lines++;
        if (support_gate>=3)
            g_b.log("  twohand pistol: weapon %d gate %s -> %s%s | anchor %s [%.1f %.1f %.1f] mm | support %.0f mm from the left shoulder, reach %.0f..%.0f (grab slack %.0f) | hands %.0f mm\r\n",
                (int)t->aim_weapon_id,g_left_pistol_log_gate>=0 && g_left_pistol_log_gate<7 ? names[g_left_pistol_log_gate] : "-",
                names[support_gate],overreach && blend>0 ? " (held beyond reach: the arm IK clamps)" : "",
                g_left.anchor_model ? "model" : "native",g_left.anchor[0],g_left.anchor[1],g_left.anchor[2],
                vdist3(clean[2],support),fabs(mi.upper-mi.fore)+1,(mi.upper+mi.fore)*0.99,DG_TWOHAND_PISTOL_GRAB_SLACK_MM,
                t->hands_distance_m*1000.0);
        else
            g_b.log("  twohand pistol: weapon %d gate %s -> %s%s | model pairs %ld refused %ld reload-blocked %ld\r\n",
                (int)t->aim_weapon_id,g_left_pistol_log_gate>=0 && g_left_pistol_log_gate<7 ? names[g_left_pistol_log_gate] : "-",
                names[support_gate],reload_blocked ? " (reload-blocked now)" : "",
                (long)g_left_pistol_model_pairs,(long)g_left_pistol_model_refused,(long)g_left_reload_blocked);
        g_left_pistol_log_gate=support_gate;g_left_pistol_log_weapon=(int)t->aim_weapon_id;
        g_left_pistol_log_blocked=reload_blocked;g_left_pistol_log_over=overreach && blend>0;
    }
    if (rifle) {
        g_left_probe.gate=support_gate;
        if (!probed) g_left_probe.weapon=(int)t->aim_weapon_id;

        left_probe_transition(support_gate,was_held,g_left.grip.engaged,0,probed);
    }
    /* Long weapons: while the support hand holds the foregrip, work out how the
       barrel should turn towards it; the right solve applies it next pair. On
       release the last swing fades with the blend (never toward the departing
       hand); a refused swing keeps the last one and never touches the blend. */
    if(rifle && steer && blend>0 && g_left.grip.engaged)left_swing_update(t,target);
    g_left.swing_w=(rifle && steer && g_left.swing_have) ? blend : 0;
    if(!(blend>0))g_left.swing_have=0;
    g_left.swing_weapon=t->aim_weapon_id;g_left.swing_stream=t->stream_id;g_left.swing_pair=t->pair_id;
    if(blend>0) for(k=0;k<3;k++) target[k]+=(support[k]-target[k])*blend;
    /* Use the same solved right wrist and orientation that drive the blade,
       not a separately calibrated controller-space line. Native Blade0/1
       positions are in game units; a resting point 300..600 units along the
       blade stays away from the hilt and tip. */
    finger_valid=left_m9_finger_contact(base,stride,end,finger_local);
    palm_valid=finger_valid && left_m9_palm(base,stride,finger_local,palm_local,palm_normal);
    brace_w=left_brace_weight(t);
    if(t->aim_weapon_id==13 && brace_w>0 && right_committed &&
       g_b.have_pred_wrist && g_b.hand_have_desired && t->aim_write &&
       t->aim_arm==arm && t->aim_pair_id==t->pair_id && t->hands_coherent) {
        const double blade0[3]={10,-118,75};
        const double axis[3]={3.454840660095215,-383.86553955078125,759.2001342773438};
        double start[3],direction[3],point[3];
        arm_quat_rotate(g_b.hand_desired,blade0,start);
        arm_quat_rotate(g_b.hand_desired,axis,direction);
        for(k=0;k<3;k++) {
            start[k]=(start[k]+g_b.pred_wrist[k])/1000.0;
            point[k]=target[k]/1000.0;
        }
        if(dg_blade_brace_snap(start,direction,point)) {
            double snapped[3];
            for(k=0;k<3;k++)snapped[k]=point[k]*1000.0;
            /* The contact belongs between palm and fingertips, distal to the wrist. */
            if (finger_valid && g_left.wrist_world_valid && t->free_left.valid) {
                DG_FREE_WRIST_STATE preview = g_left.wrist;
                double rotation[4], offset[3];
                if (dg_free_wrist_step(&preview,&t->free_left,g_left.wrist_world,rotation)) {
                    arm_quat_rotate(rotation,finger_local,offset);
                    for (k=0;k<3;k++) snapped[k] -= 0.5*offset[k];
                }
            }
            dg_grip_mix3(target,snapped,brace_w,target);
            g_left_blade_snap_pair=t->pair_id;
            memcpy(g_left_blade_snap_wanted,target,sizeof target);
        }
    }
    reason=8;
    /* A held USP/SOCOM grip can rotate the clavicle towards the anchor.
       A slot 7 owned by another writer stays untouched. */
    double shoulder_eff[3],q7w[4]={0,0,0,1},protract_rad=0;
    int clav_available=g_left.clav_active || !(*(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)&DG_LEFT_CLAV_BIT);
    memcpy(shoulder_eff,clean[2],sizeof shoulder_eff);
    if ((t->aim_weapon_id==2 || t->aim_weapon_id==3) && blend>0 && t->left_weight==1) {
        if (!clav_available)
            InterlockedIncrement(&g_left_protract_foreign);
        else {
            protract_rad=left_protract_shoulder(clean,target,mi.upper+mi.fore,shoulder_eff,q7w);
            if (protract_rad>0) {
                if (!world_quat_to_adjust(&frame,q7w,g_left.clav_next)) goto refuse;
                InterlockedIncrement(&g_left_protract_pairs);
                if (protract_rad>=DG_LEFT_PROTRACT_MAX_DEG*3.14159265358979323846/180.0-1e-9)
                    InterlockedIncrement(&g_left_protract_capped);
            }
        }
    }
    memset(&ii,0,sizeof ii);
    ii.upper=mi.upper; ii.fore=mi.fore; ii.max_reach_frac=.99;
    ii.min_elbow_deg=15; ii.max_elbow_deg=175;
    for(k=0;k<3;k++) { ii.shoulder[k]=shoulder_eff[k]; ii.target[k]=target[k]; }
    delta[0]=clean[1][0]-clean[0][0]; delta[1]=0; delta[2]=clean[1][2]-clean[0][2];
    if (t->aim_weapon_id==13 && t->aim_write && t->aim_arm==arm)
        blade_elbow_side(&t->left_position,-1,delta);
    norm=sqrt(delta[0]*delta[0]+delta[2]*delta[2]);
    if (!(norm>1e-6)) goto refuse;
    ii.pole[0]=shoulder_eff[0]+(mi.upper+mi.fore)*delta[0]/norm;
    ii.pole[1]=shoulder_eff[1]-(mi.upper+mi.fore);
    ii.pole[2]=shoulder_eff[2]+(mi.upper+mi.fore)*delta[2]/norm;
    if(!dg_ik_solve(&ii,&io)) goto refuse;
    for(k=0;k<3;k++){g_diag_arm[1][0][k]=ii.shoulder[k];g_diag_arm[1][1][k]=io.elbow[k];g_diag_arm[1][2][k]=io.wrist[k];}
    g_diag_elbow_deg[1]=io.elbow_deg;g_diag_arm_ms[1]=GetTickCount64();
    {
        double slide[3],free_q[4],contact_target[3];
        DG_FREE_WRIST_STATE preview=g_left.wrist;
        DG_IK_OUT contacted;
        int attached=right_committed && g_b.have_pred_wrist && g_b.hand_have_desired &&
            t->aim_weapon_id==1 && m9_hand_contact(slide,t->stream_id);
        int can_contact=finger_valid && preview.ready && g_left.wrist_world_valid && t->left_weight==1 &&
            t->free_left.enabled && t->free_left.valid && !reload_owns_left() && blend==0;
        g_left_m9_gate=(attached?1:0)|(can_contact?2:0)|(palm_valid?4:0)|(t->aim_weapon_id==1?8:0);
        int m9p_reload=t->aim_weapon_id==1 ? reload_owns_left() : 0;
        double m9p_ease=g_left.slide_contact.active ? (double)(GetTickCount64()-g_left.slide_contact.start_ms) : -1;
        if(t->aim_weapon_id==1)
            m9_probe_left_pair(g_left_m9_gate,blend,g_left.grip.engaged,aiming,m9p_reload,m9p_ease);
        if(can_contact && dg_free_wrist_step(&preview,&t->free_left,g_left.wrist_world,free_q)) {
            if(attached)contact_drive=dg_m9_contact_step(&g_left.slide_contact,GetTickCount64(),
                g_b.hand_desired,g_b.pred_wrist,slide,io.wrist,free_q,finger_local,
                palm_valid?palm_local:NULL,palm_valid?palm_normal:NULL,contact_target,contact_q);
            else contact_drive=dg_m9_contact_release(&g_left.slide_contact,GetTickCount64(),
                io.wrist,free_q,contact_target,contact_q);
            if(contact_drive) {
                DG_IK_IN contact_ik=ii;
                double contact_turn[4],contact_rad=0;
                memcpy(contact_ik.target,contact_target,sizeof contact_ik.target);
                /* Follow the eased, palm-aligned wrist target with the same
                   bounded clavicle rotation used for the pistol support grip.
                   During release the target returns to the tracked wrist. */
                if(clav_available) {
                    contact_rad=left_protract_shoulder(clean,contact_target,mi.upper+mi.fore,
                                                       contact_ik.shoulder,contact_turn);
                    for(k=0;k<3;k++)contact_ik.pole[k]+=contact_ik.shoulder[k]-ii.shoulder[k];
                }
                int contact_ok=dg_ik_solve(&contact_ik,&contacted);
                double miss=contact_ok?vdist3(contacted.wrist,contact_target):-1;
                if(contact_ok && miss<1.0) {
                    if(contact_rad>0) {
                        if(!world_quat_to_adjust(&frame,contact_turn,g_left.clav_next))goto refuse;
                        memcpy(shoulder_eff,contact_ik.shoulder,sizeof shoulder_eff);
                        memcpy(q7w,contact_turn,sizeof q7w);
                        protract_rad=contact_rad;
                    }
                    io=contacted;
                } else {
                    if(attached) {   /* m9_grab_probe: the slide hand point the arm could not reach */
                        m9_probe_site(DG_M9_ROUTE_CONTACT_UNREACHABLE,__LINE__,0,contact_ok,miss,
                            vdist3(contact_ik.shoulder,contact_target),
                            (ii.upper+ii.fore)*ii.max_reach_frac,
                            g_left.slide_contact.active?(double)(GetTickCount64()-g_left.slide_contact.start_ms):-1);
                    }
                    contact_drive=0;g_left_m9_gate|=32;memset(&g_left.slide_contact,0,sizeof g_left.slide_contact);if(attached)m9_contact_cancel();
                }
            }
        } else {
            if(attached) {   /* m9_grab_probe: which term of can_contact failed */
                unsigned m9p=(finger_valid?0:M9P_C_FINGER)|(preview.ready?0:M9P_C_PREVIEW)|
                    (g_left.wrist_world_valid?0:M9P_C_WORLD)|(t->left_weight==1?0:M9P_C_WEIGHT)|
                    (t->free_left.enabled?0:M9P_C_FREE_EN)|(t->free_left.valid?0:M9P_C_FREE_OK)|
                    (m9p_reload?M9P_C_RELOAD:0)|(blend==0?0:M9P_C_BLEND)|(can_contact?M9P_C_WRIST:0);
                m9_probe_site(DG_M9_ROUTE_CONTACT_LOST,__LINE__,m9p,0,0,0,0,m9p_ease);
            }
            memset(&g_left.slide_contact,0,sizeof g_left.slide_contact);
            if(attached)m9_contact_cancel();
        }
    }
    memset(&oi,0,sizeof oi);
    for(k=0;k<3;k++) {
        oi.shoulder[k]=shoulder_eff[k]; oi.elbow[k]=io.elbow[k]; oi.wrist[k]=io.wrist[k];
        oi.rest_upper[k]=clean[3][k]-clean[2][k];
        oi.rest_fore[k]=clean[4][k]-clean[3][k];
    }
    if (protract_rad>0) {   /* socom_gap: the arm hangs from the turned clavicle: rest directions turned with it */
        double v[3];
        arm_quat_rotate(q7w,oi.rest_upper,v);memcpy(oi.rest_upper,v,sizeof v);
        arm_quat_rotate(q7w,oi.rest_fore,v);memcpy(oi.rest_fore,v,sizeof v);
    }
    reason=9;
    if(!dg_ik_orient(&oi,&oo) || !world_quat_to_adjust(&frame,oo.upper_quat,qa) ||
        !world_quat_to_adjust(&frame,oo.fore_quat,qb)) goto refuse;
    { const double identity[4]={0,0,0,1};
      dg_pose_quat_blend(identity,qa,t->left_weight,qa);
      dg_pose_quat_blend(identity,qb,t->left_weight,qb); }
    reason=10;
    if(!left_owner(&mc,&adj)) goto refuse;
    left_model_publish(base,stride,&frame,t,qa,qb,had_active,io.elbow,io.wrist);
    left_wrist_now(base,stride,mc,adj,&frame,t,qa,qb,had_active,blend,contact_drive?contact_q:NULL);
    if(contact_drive && !g_left.wrist_world_valid) {
        m9_probe_site(DG_M9_ROUTE_CONTACT_WRIST,__LINE__,M9P_C_WORLD,0,0,0,0,-1);
        m9_contact_cancel();goto refuse; /* no positional-only attachment */
    }
    for(j=0;j<8;j++) {
        float v=(float)(j<4 ? qa[j] : qb[j-4]);
        ((volatile float *)(ULONG_PTR)(adj+8*16))[j]=v;
        g_left.cached[j]=v;
    }
    *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=DG_LEFT_MASK;
    if (protract_rad>0) {   /* socom_gap: slot 7 carries this pair's clavicle turn */
        if (!g_left.clav_active) memcpy(g_left.clav_before,(void *)(ULONG_PTR)(adj+7*16),16);
        for(k=0;k<4;k++) g_left.clav_cached[k]=(float)g_left.clav_next[k];
        memcpy((void *)(ULONG_PTR)(adj+7*16),g_left.clav_cached,16);
        *(volatile ULONGLONG *)(ULONG_PTR)(mc+0x38)|=DG_LEFT_CLAV_BIT;
        g_left.clav_active=1;
    } else left_clav_release(mc,adj);
    g_left.active=1; g_left.accepted++;
    {   const double *sp=blend>0 ? support : clean[2];   /* support is set whenever the grip blends */
        left_gap_meter((int)t->aim_weapon_id,blend,sp,io.wrist,live[4],vdist3(clean[2],sp),
                       (mi.upper+mi.fore)*0.99,vdist3(shoulder_eff,sp),protract_rad*180.0/3.14159265358979323846,
                       vdist3(clean[1],clean[2])); }
    if(g_left_blade_snap_pair==t->pair_id)g_left_blade_clamp=vdist3(io.wrist,g_left_blade_snap_wanted);
    if(contact_drive)g_left_m9_gate|=16;
    if(right_committed && g_b.have_pred_wrist && g_b.hand_have_desired &&
       finger_valid && g_left.wrist_world_valid) {
        /* The grab radius measures the same hand point the slide anchor holds. */
        double contact[3];arm_quat_rotate(g_left.wrist_world,palm_valid?palm_local:finger_local,contact);
        for(k=0;k<3;k++)contact[k]+=io.wrist[k];
        m9_observe_hand(t,contact);
    }
    InterlockedIncrement(&g_left_accepted);
    InterlockedExchange(&g_left_reason,0);
    if (blend>0) InterlockedIncrement(&g_left_support_pairs);
    InterlockedExchange(&g_left_status,blend>0 ? 2 : 1);
    InterlockedExchange(&g_left_blend,f2l((float)blend));
    for(k=0;k<3;k++) InterlockedExchange(&g_left_target[k],f2l((float)io.wrist[k]));
    return;
refuse:
    InterlockedIncrement(&g_left_refused);
    InterlockedExchange(&g_left_reason,reason);
    if (rifle && was_held) left_probe_transition(7,1,0,reason,0);
    left_arm_release();
}

/* Bounded read-only witness: accepted IK and object visibility do not prove
 * that the rendered hand reached the blade. Observe the actual hierarchy,
 * including its finger contact, separately from this pair's prediction. */
#include "dg_evm_witness.inl"

#define DG_M9_WITNESS_PASS_MM 15.0
static struct {
    int was_on,lines,grabs;
    uint64_t last_ms,start_ms;
    unsigned long samples,settled,driven,over;
    double max_mm,sum_mm,max_palm_dot_err,max_across;
} g_m9_witness;
static void left_m9_observe(ULONGLONG arm,const DG_BRIDGE_ARM_TARGET *t) {
    ULONGLONG objs,base,end,wobjs,sm;int stride=g_b.skel_stride,i,k,r,on=0,box=0;
    double anchor[3],pivot[3],progress=0,local[3],target[3],palm[3],wr[3],kn[3]={0,0,0};
    double up_l[3],up_w[3]={0,0,0},axis[3],fing[3],acr[3]={0,0,0},n[3],len,dist,best=-1,dot_up,across;
    float *wm,*s,*m;uint64_t now=GetTickCount64();int state=-1;LONG gate=g_left_m9_gate;
    if(!g_b.log || !t || t->aim_weapon_id!=1 || !g_m9_enabled || !TryAcquireSRWLockShared(&g_m9_lock))return;
    wobjs=g_m9.objs;on=g_m9.out.attached;state=g_m9.out.state;progress=g_m9.out.progress;box=g_m9.grasp_box;
    memcpy(anchor,g_m9.anchor,sizeof anchor);memcpy(pivot,g_m9.pivot,sizeof pivot);
    ReleaseSRWLockShared(&g_m9_lock);
    if(!on) {
        if(g_m9_witness.was_on && g_m9_witness.lines<400) {
            g_m9_witness.lines++;
            g_b.log("  M9 hand witness summary: grab %d samples %lu settled %lu driven %lu max_mm %.1f mean_mm %.1f palm_up_err_max %.2f fingers_along_slide_max %.2f over_%.0fmm %lu -> %s\r\n",
                g_m9_witness.grabs,g_m9_witness.samples,g_m9_witness.settled,g_m9_witness.driven,
                g_m9_witness.max_mm,g_m9_witness.settled?g_m9_witness.sum_mm/g_m9_witness.settled:-1.0,
                g_m9_witness.max_palm_dot_err,g_m9_witness.max_across,DG_M9_WITNESS_PASS_MM,g_m9_witness.over,
                !g_m9_witness.settled?"FAIL (no settled sample)":g_m9_witness.over?"FAIL":"PASS");
        }
        g_m9_witness.was_on=0;return;
    }
    if(!g_m9_witness.was_on) {
        int grabs=g_m9_witness.grabs+1,lines=g_m9_witness.lines;
        memset(&g_m9_witness,0,sizeof g_m9_witness);
        g_m9_witness.grabs=grabs;g_m9_witness.lines=lines;g_m9_witness.was_on=1;g_m9_witness.start_ms=now;
    }
    if(!plausible_ptr(arm) || stride<64 || stride>4096 || !plausible_ptr(wobjs) || g_b.skel_parents_read<55)return;
    objs=*(ULONGLONG *)(ULONG_PTR)arm;base=objs+DG_OBJS_ARRAY;end=region_end(objs);
    sm=wobjs+DG_OBJS_ARRAY+2*(ULONGLONG)stride;
    if(!plausible_ptr(objs) || end<base+54*(ULONGLONG)stride+64 || region_end(wobjs)<sm+64 ||
       !looks_like_matrix(sm) || !looks_like_matrix(base+10*(ULONGLONG)stride))return;
    for(i=0;i<5;i++)if(!looks_like_matrix(base+g_left_m9_knuckles[i]*(ULONGLONG)stride))return;
    if(!looks_like_matrix(base+44*(ULONGLONG)stride))return;
    wm=(float *)(ULONG_PTR)(base+10*(ULONGLONG)stride);s=(float *)(ULONG_PTR)sm;
    /* grasp point in slide space: the anchor is in model-0 space without travel */
    for(k=0;k<3;k++){local[k]=anchor[k]-pivot[k];wr[k]=wm[12+k];}
    for(k=0;k<3;k++){target[k]=s[12+k];for(r=0;r<3;r++)target[k]+=local[r]*s[r*4+k];}
    for(i=0;i<5;i++){m=(float *)(ULONG_PTR)(base+g_left_m9_knuckles[i]*(ULONGLONG)stride);for(k=0;k<3;k++)kn[k]+=m[12+k]/5.0;}
    for(k=0;k<3;k++)palm[k]=0.5*(wr[k]+kn[k]);
    dist=vdist3(palm,target);
    /* drawn palm normal against the slide's up, fingers against the slide axis */
    len=hypot(anchor[0],anchor[2]);up_l[0]=len>1?anchor[0]/len:0;up_l[1]=0;up_l[2]=len>1?anchor[2]/len:0;
    for(k=0;k<3;k++){for(r=0;r<3;r++)up_w[k]+=up_l[r]*s[r*4+k];axis[k]=s[4+k];}
    {   double tips[3]={0,0,0};static const int tp[5]={41,44,47,51,54};
        for(i=0;i<5;i++){m=(float *)(ULONG_PTR)(base+tp[i]*(ULONGLONG)stride);
            if(!looks_like_matrix((ULONGLONG)(ULONG_PTR)m))return;for(k=0;k<3;k++)tips[k]+=m[12+k]/5.0;}
        for(k=0;k<3;k++)fing[k]=tips[k]-wr[k];
    }
    for(i=0;i<5;i++)for(r=i+1;r<5;r++) {
        float *a=(float *)(ULONG_PTR)(base+g_left_m9_knuckles[i]*(ULONGLONG)stride);
        float *b=(float *)(ULONG_PTR)(base+g_left_m9_knuckles[r]*(ULONGLONG)stride);
        double d[3],dd=0;for(k=0;k<3;k++){d[k]=b[12+k]-a[12+k];dd+=d[k]*d[k];}
        if(dd>best){best=dd;memcpy(acr,d,sizeof acr);}
    }
    n[0]=fing[1]*acr[2]-fing[2]*acr[1];n[1]=fing[2]*acr[0]-fing[0]*acr[2];n[2]=fing[0]*acr[1]-fing[1]*acr[0];
    len=sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);dot_up=0;
    { double lu=sqrt(up_w[0]*up_w[0]+up_w[1]*up_w[1]+up_w[2]*up_w[2]);
      if(len>1e-6 && lu>1e-6)for(k=0;k<3;k++)dot_up+=n[k]/len*up_w[k]/lu; }
    len=sqrt(fing[0]*fing[0]+fing[1]*fing[1]+fing[2]*fing[2]);
    { double la=sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);across=0;
      if(len>1e-6 && la>1e-6)for(k=0;k<3;k++)across+=fing[k]/len*axis[k]/la;
      across=fabs(across); }
    g_m9_witness.samples++;
    if(gate&16)g_m9_witness.driven++;
    if(now-g_m9_witness.start_ms>=150) {
        g_m9_witness.settled++;g_m9_witness.sum_mm+=dist;
        if(dist>g_m9_witness.max_mm)g_m9_witness.max_mm=dist;
        if(dist>DG_M9_WITNESS_PASS_MM)g_m9_witness.over++;
        if(1-fabs(dot_up)>g_m9_witness.max_palm_dot_err)g_m9_witness.max_palm_dot_err=1-fabs(dot_up);
        if(across>g_m9_witness.max_across)g_m9_witness.max_across=across;
    }
    if(g_m9_witness.lines<400 && (!g_m9_witness.last_ms || now-g_m9_witness.last_ms>=100)) {
        g_m9_witness.lines++;g_m9_witness.last_ms=now;
        g_b.log("  M9 hand witness: grab %d t_ms %llu state %d progress %.2f gate 0x%02lX (attached %d contact %d palm %d driven %d unreachable %d) box %d palm_to_slide_mm %.1f palm_normal_dot_up %.2f fingers_along_slide %.2f palm [%.1f %.1f %.1f] slide_grasp [%.1f %.1f %.1f]\r\n",
            g_m9_witness.grabs,(unsigned long long)(now-g_m9_witness.start_ms),state,progress,(unsigned long)gate,
            (int)(gate&1),(int)((gate>>1)&1),(int)((gate>>2)&1),(int)((gate>>4)&1),(int)((gate>>5)&1),box,
            dist,dot_up,across,palm[0],palm[1],palm[2],target[0],target[1],target[2]);
    }
}
static void left_blade_observe(ULONGLONG arm,const DG_BRIDGE_ARM_TARGET *t) {
    left_evm_observe(arm,t);
    left_m9_observe(arm,t);
    static unsigned long last_pair,lines;static uint64_t last_ms;
    ULONGLONG objs,base,end,evm;int stride=g_b.skel_stride,k,r;
    double grip[3],axis[3]={0,0,0},start[3],left[3],palm[3],local[3];
    double from[3]={10,-118,75},direction[3]={3.454840660095215,-383.86553955078125,759.2001342773438};
    double rw[3],lw[3],seen[3],camera_q[4],inverse[4],v[3],raw[3];
    float *right,*hand,predicted[3];int finger,coordinates=0;LONG eflag=-1;
    uint64_t now=GetTickCount64();
    if(!g_b.log||!t||!dg_bridge_blade_braced()||t->aim_weapon_id!=13||
       lines>=80||t->pair_id==last_pair||now-last_ms<500)return;
    if(!plausible_ptr(arm)||region_end(arm)<arm+0x38||stride<64||stride>4096)return;
    objs=*(ULONGLONG *)(ULONG_PTR)arm;base=objs+DG_OBJS_ARRAY;end=region_end(objs);
    if(!plausible_ptr(objs)||end<base+10*(ULONGLONG)stride+64)return;
    if(!looks_like_matrix(base+6*(ULONGLONG)stride)||!looks_like_matrix(base+10*(ULONGLONG)stride))return;
    right=(float *)(ULONG_PTR)(base+6*(ULONGLONG)stride);
    hand=(float *)(ULONG_PTR)(base+10*(ULONGLONG)stride);
    for(k=0;k<3;k++) {
        rw[k]=right[12+k];lw[k]=hand[12+k];start[k]=rw[k];
        for(r=0;r<3;r++){start[k]+=from[r]*right[r*4+k];axis[k]+=direction[r]*right[r*4+k];}
        grip[k]=start[k]/1000;left[k]=lw[k]/1000;
        raw[k]=lw[k];
        { LONG bits=g_left_target[k];memcpy(&predicted[k],&bits,sizeof bits); }
    }
    finger=left_m9_finger_contact(base,stride,end,local);
    for(k=0;k<3;k++) {
        palm[k]=lw[k];if(finger)for(r=0;r<3;r++)palm[k]+=local[r]*hand[r*4+k];
    }
    memcpy(seen,left,sizeof seen);
    if(!dg_blade_brace_snap(grip,axis,seen))return;
    for(k=0;k<3;k++)seen[k]*=1000;
    evm=*(ULONGLONG *)(ULONG_PTR)(arm+0x30);
    if(plausible_ptr(evm)&&region_end(evm)>=evm+0x5c)eflag=RD32(evm+0x58);
    if(t->left_position.enabled&&dg_position_frame(&t->left_position,camera_q)) {
        dg_ik_quat_conj(camera_q,inverse);
        for(k=0;k<3;k++)v[k]=lw[k]-t->left_position.camera[12+k];
        arm_quat_rotate(inverse,v,raw);coordinates=1;
    }
    last_ms=now;last_pair=t->pair_id;lines++;
    g_b.log("  blade hand witness: pair %lu left_pair %lu active %d reason %ld flags %08X evm %08X coherent %d right %d/%d aim %d snap %d clamp_mm %.2f\r\n",
        t->pair_id,g_left.pair,g_left.active,g_left_reason,(unsigned)RD32(objs+0x58),(unsigned)eflag,
        t->hands_coherent,g_b.have_pred_wrist,g_b.hand_have_desired,t->aim_write,
        g_left_blade_snap_pair==t->pair_id,g_left_blade_clamp);
    g_b.log("  blade hand geometry: right [%.2f %.2f %.2f] left [%.2f %.2f %.2f] palm [%.2f %.2f %.2f] fingers %d live_wrist_to_blade_mm %.2f left_camera %d [%.2f %.2f %.2f]\r\n",
        rw[0],rw[1],rw[2],lw[0],lw[1],lw[2],palm[0],palm[1],palm[2],finger,
        vdist3(lw,seen),coordinates,raw[0],raw[1],raw[2]);
    g_b.log("  blade hand desired: right [%.2f %.2f %.2f] snap [%.2f %.2f %.2f] left_pred [%.2f %.2f %.2f] wrist_rotation %d\r\n",
        g_b.pred_wrist[0],g_b.pred_wrist[1],g_b.pred_wrist[2],
        g_left_blade_snap_wanted[0],g_left_blade_snap_wanted[1],g_left_blade_snap_wanted[2],
        (double)predicted[0],(double)predicted[1],(double)predicted[2],g_left.wrist_world_valid);
}

static int left_support_engaged(void) {
    return g_left.grip.engaged || g_left.grip.blend>0;
}
