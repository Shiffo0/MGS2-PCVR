#ifndef DG_DRAW_TRIAL_H
#define DG_DRAW_TRIAL_H
#include <d3d11.h>
#ifdef __cplusplus
extern "C" {
#endif
void dg_draw_trial_attach(ID3D11Device *device,void (*log)(const char*,...));
void dg_draw_trial_poll(const char *session_marker);
void dg_draw_trial_present(void);
void dg_draw_trial_stop(void);
void dg_blit_boundary_control(const char *dir,int active,unsigned mark,unsigned present,int eye);
/* Constant-buffer capture (dg_cb_probe.inl): the camera seam publishes the
   matrices it just wrote so the desk analysis can look for them in the
   vertex-shader constant buffers. VEH-safe: interlocked + memcpy only. */
void dg_cb_probe_camera(int eye,const float *eye_pers,const float *pers,
                        const float *eye_inv,const float *eye_world);
/* 2D sprite placement per eye (dg_ui2d.inl). scale in thousandths, conv in
   1e-5 NDC, vs = vertex-shader bytecode hashes that ARE sprites. */
void dg_ui2d_configure(int on,int scale_mils,int conv_e5,int sign,
                       const unsigned long long *vs,unsigned count);
void dg_ui2d_backbuffer(unsigned width,unsigned height);
/* Sprites in a frame without camera uploads: 0 leave, 1 as the last voted frame, 2 as its opposite eye. */
void dg_ui2d_hold(int mode);
/* Camera/Present publish raw stereo-gameplay eligibility; silence closes it. */
void dg_ui2d_gameplay(int allowed);
void dg_ui2d_feedback(int on);
void dg_ui2d_feedback_stats(long*skipped,long*checked);
void *dg_ui2d_measure_source(void);
void dg_ui2d_trace(int frames,void*a,void*b);
void dg_ui2d_backbuffer_ptr(void*resource);
long dg_ui2d_last_frame_bb(long*indexed,void**srv);
/* Present thread only: native draw calls issued for the frame that just ended. */
long dg_ui2d_last_frame_draws(void);
/* Present thread only: +1/-1 = frustum the frame's object shaders (c20) were drawn with, 2 mixed, 0 none. */
int dg_ui2d_last_frame_obj(long*pos,long*neg);
/* Camera seam (VEH-safe): DG_EYE_LEFT/RIGHT of the valid handoff just written. */
void dg_ui2d_eye(int eye);
/* Present thread only: what the frame that just ended uploaded (draws, votes, refusal reasons). */
long dg_ui2d_last_frame_detail(char*out,size_t n);
/* +1/-1: the frustum the frame that just ended was RENDERED with; 0 = none. */
int dg_ui2d_last_frame_sign(void);
/* The FOV the camera seam submits for the eye it just built (radians). */
void dg_ui2d_display(double angle_left,double angle_right);
void dg_ui2d_stats(char *out,size_t n);
/* Wrist radar, draw-hook half (dg_radar.inl). on = recognise the radar composite draw and hand its
   texture to dg_xr_radar_capture; hud_off = also do not forward it while the wrist layer is up. */
void dg_radar_configure(int on,int hud_off);
/* Native HUD radar is suppressed only in a confirmed first-person view. */
void dg_radar_view(int first_person);
void dg_radar_stats(char *out,size_t n);
#ifdef __cplusplus
}
#endif
#endif
