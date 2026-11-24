#ifndef DG_POLICY_H
#define DG_POLICY_H

#include "dg_pose.h"
#include "dg_ik.h"
#include "dg_arm_map.h"
#include "dg_fire.h"
#include "dg_recoil.h"
#include "dg_move.h"

/* Bumped BY HAND whenever the boundary changes in a way sizeof cannot see:
   a reordered field of equal size, a changed unit, a new flag meaning. A
   size change is caught mechanically; a meaning change is caught only by
   this number. */
#define DG_POLICY_ABI_VERSION 6u

#define DG_POLICY_MAGIC 0x31504744u              /* "DGP1", little-endian */

typedef struct DG_POLICY_SIZES {
    unsigned int pose_cfg, pose_in, pose_out, pose_state;
    unsigned int ik_in, ik_out, ik_orient_in, ik_orient_out;
    unsigned int ik_hand_in, ik_hand_stab_in, ik_hand_stab_out;
    unsigned int arm_map_in, arm_map_out, arm_map_state;
    unsigned int fire_in, fire_out, fire_state;
    unsigned int recoil_state;
    unsigned int move_in, move_out;
} DG_POLICY_SIZES;

#define DG_POLICY_SIZES_INIT { \
    (unsigned int)sizeof(DG_POSE_CFG), (unsigned int)sizeof(DG_POSE_IN), \
    (unsigned int)sizeof(DG_POSE_OUT), (unsigned int)sizeof(DG_POSE_STATE), \
    (unsigned int)sizeof(DG_IK_IN), (unsigned int)sizeof(DG_IK_OUT), \
    (unsigned int)sizeof(DG_IK_ORIENT_IN), \
    (unsigned int)sizeof(DG_IK_ORIENT_OUT), \
    (unsigned int)sizeof(DG_IK_HAND_IN), \
    (unsigned int)sizeof(DG_IK_HAND_STABILIZE_IN), \
    (unsigned int)sizeof(DG_IK_HAND_STABILIZE_OUT), \
    (unsigned int)sizeof(DG_ARM_MAP_IN), (unsigned int)sizeof(DG_ARM_MAP_OUT), \
    (unsigned int)sizeof(DG_ARM_MAP_STATE), \
    (unsigned int)sizeof(DG_FIRE_IN), (unsigned int)sizeof(DG_FIRE_OUT), \
    (unsigned int)sizeof(DG_FIRE), (unsigned int)sizeof(DG_RECOIL), \
    (unsigned int)sizeof(DG_MOVE_IN), (unsigned int)sizeof(DG_MOVE_OUT) }

/* The table. Plain C function pointers over the exact public signatures of
   the six modules - no varargs, no callbacks, no Windows types, so the same
   header compiles it on both sides of the boundary. */
typedef struct DG_POLICY_TABLE {
    unsigned int magic;                 /* DG_POLICY_MAGIC */
    unsigned int abi_version;           /* DG_POLICY_ABI_VERSION */
    unsigned int table_bytes;           /* sizeof(DG_POLICY_TABLE) */
    unsigned int reserved;              /* keeps the sizes 8-aligned; 0 */
    DG_POLICY_SIZES sizes;

    /* dg_pose - stereo-latched controller pose */
    void (*pose_cfg_default)(DG_POSE_CFG *cfg);
    void (*pose_init)(DG_POSE_STATE *s, const DG_POSE_CFG *cfg);
    void (*pose_step)(DG_POSE_STATE *s, const DG_POSE_IN *in,
                      DG_POSE_OUT *out);
    void (*pose_quat_blend)(const double a[4], const double b[4], double t,
                            double out[4]);

    /* dg_ik - two-bone solve, hand orientation, PS2 angle channel */
    int (*ik_solve)(const DG_IK_IN *in, DG_IK_OUT *out);
    int (*ik_swing_quat)(const double from[3], const double to[3],
                         double q[4], int *flipped);
    int (*ik_orient)(const DG_IK_ORIENT_IN *in, DG_IK_ORIENT_OUT *out);
    void (*ik_quat_mul)(const double a[4], const double b[4], double out[4]);
    void (*ik_quat_conj)(const double q[4], double out[4]);
    int (*ik_quat_normalize)(double q[4]);
    int (*ik_basis_quat)(const double basis[3][3], double q[4]);
    double (*ik_quat_angle)(const double a[4], const double b[4]);
    int (*ik_hand_adjust)(const DG_IK_HAND_IN *in, double out[4]);
    int (*ik_hand_stabilize)(const DG_IK_HAND_STABILIZE_IN *in,
                             DG_IK_HAND_STABILIZE_OUT *out);
    int (*ik_quat_to_ps_angles)(const double q[4], short out[3],
                                unsigned *flags);
    int (*ik_ps_precompensate)(const short want[3], int pull_divisor,
                               short out[3]);

    /* dg_arm_map - controller position to IK target */
    void (*arm_map_reset)(DG_ARM_MAP_STATE *state);
    int (*arm_map_step)(DG_ARM_MAP_STATE *state, const DG_ARM_MAP_IN *in,
                        DG_ARM_MAP_OUT *out);

    /* dg_fire - trigger to weapon-pad contract */
    void (*fire_reset)(DG_FIRE *f);
    int (*fire_pressure)(double value, double click);
    void (*fire_step)(DG_FIRE *f, const DG_FIRE_IN *in, DG_FIRE_OUT *out);

    /* dg_recoil - the kick envelope */
    void (*recoil_reset)(DG_RECOIL *r);
    void (*recoil_fire)(DG_RECOIL *r, double shots);
    void (*recoil_step)(DG_RECOIL *r);
    double (*recoil_amplitude)(const DG_RECOIL *r);
    int (*recoil_climb_quat)(const double fore_axis[3], const double up[3],
                             double climb_rad, double q_out[4]);
    int (*recoil_pull_back)(double target[3], const double root[3],
                            double push_mm);

    /* dg_move - stick to walk pad */
    void (*move_step)(const DG_MOVE_IN *in, DG_MOVE_OUT *out);
} DG_POLICY_TABLE;

extern const DG_POLICY_TABLE dg_policy_builtin;

/* The one export a policy DLL carries:
       const DG_POLICY_TABLE *dg_policy_query(void);
   Built by compiling the six modules plus dg_policy_table.c with
   /DDG_POLICY_DLL - see build_policy.bat. */
#define DG_POLICY_EXPORT_NAME "dg_policy_query"

/* ------------------------------------------------------- dispatcher ----- */

typedef struct DG_POLICY_STATUS {
    long generation;        /* of the ACTIVE table; 0 = builtin */
    long staged;            /* tables validated and offered */
    long adopted;           /* tick-seam adoptions */
    long refused;           /* DLLs refused for any reason */
    long tls_misses;        /* dg_policy() calls with no open pass */
    int active_is_builtin;  /* 1 while the builtin table is active */
} DG_POLICY_STATUS;

/* Worker-side setup. `forbidden_dir` is the game's own directory: a
   vr_policy path inside it is refused outright, which is the code-level
   backstop of the never-touch-game-files rule. `log` may be NULL (tests). */
void dg_policy_setup(const char *forbidden_dir,
                     void (*log)(const char *fmt, ...));

void dg_policy_poll(const char *dll_path);

/* Tick seam ONLY, before any policy call of the tick: publish the pending
   table, if any, as active. The single place a swap becomes visible. */
void dg_policy_tick_adopt(void);

void *dg_policy_pass_begin(void);
void dg_policy_pass_end(void *token);

/* The table to call through. Inside a pass: that pass's snapshot. Outside
   (worker one-off calls like dg_pose_init on a config edit): the active
   table, read once per call. */
const DG_POLICY_TABLE *dg_policy(void);

void dg_policy_status(DG_POLICY_STATUS *out);

int dg_policy_validate(const DG_POLICY_TABLE *t, const char **why);
long dg_policy_load_from(const char *path, const char **why);

/* ------------------------------------------------ call-site redirect ---- */

#ifdef DG_POLICY_REDIRECT
#define dg_pose_cfg_default(cfg) (dg_policy()->pose_cfg_default(cfg))
#define dg_pose_init(s, cfg) (dg_policy()->pose_init((s), (cfg)))
#define dg_pose_step(s, in, out) (dg_policy()->pose_step((s), (in), (out)))
#define dg_pose_quat_blend(a, b, t, out) \
    (dg_policy()->pose_quat_blend((a), (b), (t), (out)))
#define dg_ik_solve(in, out) (dg_policy()->ik_solve((in), (out)))
#define dg_ik_swing_quat(from, to, q, flipped) \
    (dg_policy()->ik_swing_quat((from), (to), (q), (flipped)))
#define dg_ik_orient(in, out) (dg_policy()->ik_orient((in), (out)))
#define dg_ik_quat_mul(a, b, out) (dg_policy()->ik_quat_mul((a), (b), (out)))
#define dg_ik_quat_conj(q, out) (dg_policy()->ik_quat_conj((q), (out)))
#define dg_ik_quat_normalize(q) (dg_policy()->ik_quat_normalize(q))
#define dg_ik_basis_quat(basis, q) (dg_policy()->ik_basis_quat((basis), (q)))
#define dg_ik_quat_angle(a, b) (dg_policy()->ik_quat_angle((a), (b)))
#define dg_ik_hand_adjust(in, out) (dg_policy()->ik_hand_adjust((in), (out)))
#define dg_ik_hand_stabilize(in, out) \
    (dg_policy()->ik_hand_stabilize((in), (out)))
#define dg_ik_quat_to_ps_angles(q, out, flags) \
    (dg_policy()->ik_quat_to_ps_angles((q), (out), (flags)))
#define dg_ik_ps_precompensate(want, pull_divisor, out) \
    (dg_policy()->ik_ps_precompensate((want), (pull_divisor), (out)))
#define dg_arm_map_reset(state) (dg_policy()->arm_map_reset(state))
#define dg_arm_map_step(state, in, out) \
    (dg_policy()->arm_map_step((state), (in), (out)))
#define dg_fire_reset(f) (dg_policy()->fire_reset(f))
#define dg_fire_pressure(value, click) \
    (dg_policy()->fire_pressure((value), (click)))
#define dg_fire_step(f, in, out) (dg_policy()->fire_step((f), (in), (out)))
#define dg_recoil_reset(r) (dg_policy()->recoil_reset(r))
#define dg_recoil_fire(r, shots) (dg_policy()->recoil_fire((r), (shots)))
#define dg_recoil_step(r) (dg_policy()->recoil_step(r))
#define dg_recoil_amplitude(r) (dg_policy()->recoil_amplitude(r))
#define dg_recoil_climb_quat(fore_axis, up, climb_rad, q_out) \
    (dg_policy()->recoil_climb_quat((fore_axis), (up), (climb_rad), (q_out)))
#define dg_recoil_pull_back(target, root, push_mm) \
    (dg_policy()->recoil_pull_back((target), (root), (push_mm)))
#define dg_move_step(in, out) (dg_policy()->move_step((in), (out)))
#endif /* DG_POLICY_REDIRECT */

#endif /* DG_POLICY_H */
