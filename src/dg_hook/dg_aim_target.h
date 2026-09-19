#ifndef DG_AIM_TARGET_H
#define DG_AIM_TARGET_H

/* Build an absolute desired hand orientation for the measured pistol basis:
   barrel = hand local -Y, weapon up = hand local +Z. camera_world rows are
   camera axes in world (the engine's camera-to-world storage convention).
   view_raw and aim_raw are xyzw OpenXR orientations from the SAME frame and
   raw view used to construct that camera. Default camera axis signs only.

   No controller/animation calibration offset enters this calculation.
   Unit quaternions (length tolerance 1e-6) and a proper rigid camera basis
   (row dot-product tolerance 1e-4) are required. Return 0 with identity in
   hand_world on invalid input. A NULL output returns 0. Inputs may alias
   the output. No state, allocation, policy, game access or weapon selection. */
int dg_aim_target_build(const double camera_world[3][3],
                        const double view_raw[4], const double aim_raw[4],
                        double hand_world[4]);

/* Post-compose the retail Blade0->Blade1 axis onto the controller AIM ray. */
int dg_aim_target_blade(double hand_world[4]);

#endif
