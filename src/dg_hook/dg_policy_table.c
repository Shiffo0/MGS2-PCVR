

#include "dg_policy.h"

#ifdef DG_POLICY_TEST_BAD_VERSION
#define DG_POLICY_STAMP_VERSION (DG_POLICY_ABI_VERSION + 1u)
#else
#define DG_POLICY_STAMP_VERSION DG_POLICY_ABI_VERSION
#endif

const DG_POLICY_TABLE dg_policy_builtin = {
    DG_POLICY_MAGIC,
    DG_POLICY_STAMP_VERSION,
    (unsigned int)sizeof(DG_POLICY_TABLE),
    0u,
    DG_POLICY_SIZES_INIT,

    dg_pose_cfg_default,
    dg_pose_init,
    dg_pose_step,
    dg_pose_quat_blend,

    dg_ik_solve,
    dg_ik_swing_quat,
    dg_ik_orient,
    dg_ik_quat_mul,
    dg_ik_quat_conj,
    dg_ik_quat_normalize,
    dg_ik_basis_quat,
    dg_ik_quat_angle,
    dg_ik_hand_adjust,
    dg_ik_hand_stabilize,
    dg_ik_quat_to_ps_angles,
    dg_ik_ps_precompensate,

    dg_arm_map_reset,
    dg_arm_map_step,

    dg_fire_reset,
    dg_fire_pressure,
    dg_fire_step,

    dg_recoil_reset,
    dg_recoil_fire,
    dg_recoil_step,
    dg_recoil_amplitude,
    dg_recoil_climb_quat,
    dg_recoil_pull_back,

    dg_move_step,
};

#ifdef DG_POLICY_DLL
__declspec(dllexport) const DG_POLICY_TABLE *dg_policy_query(void)
{
    return &dg_policy_builtin;
}
#endif
