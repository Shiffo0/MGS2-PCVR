#ifndef DG_STEREO_MEASURE_H
#define DG_STEREO_MEASURE_H
/* Observations only. No game addresses or control writes in this interface. */
typedef struct {
    unsigned present, camera_updates, camera_silent;
    int armed, stereo, screen, flat, handoff_valid, linked_valid, eye;
    int eye_truth_mode, draw_skip_mode, frame_sign;
    void *final_source; /* Borrowed until next Present, render thread only. */
} DG_STEREO_PRESENT;
void dg_xr_measure_present(const DG_STEREO_PRESENT *p);
unsigned dg_xr_measure_trace_request(void);
int dg_xr_measure_control(char *dir,unsigned capacity,unsigned *mark);
#endif
