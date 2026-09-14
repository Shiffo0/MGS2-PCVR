

#ifndef DG_XR_SCRIPT_H
#define DG_XR_SCRIPT_H

#include <stddef.h>
#include <stdint.h>

#include "dg_xr.h"

#define DG_XR_SCRIPT_EVENT_RECENTER 1u
#define DG_XR_SCRIPT_BUTTON_SQUEEZE_CLICK 1u
#define DG_XR_SCRIPT_BUTTON_STICK_CLICK   2u
#define DG_XR_SCRIPT_BUTTON_PRIMARY       4u
#define DG_XR_SCRIPT_BUTTON_SECONDARY     8u
#define DG_XR_SCRIPT_BUTTON_MENU         16u

/* Opaque parsed storage, exposed this far so desk tests can parse and sample
   without starting a thread.  Call program_free after every successful parse. */
typedef struct {
    void *steps;
    unsigned int step_count;
    unsigned int total_ticks;
    unsigned int tick_ms;
} DG_XR_SCRIPT_PROGRAM;

/* Strict, bounded text format:

       DGXR_SCRIPT 1 [tick_ms]
       pose left|right grip|aim active px py pz qx qy qz qw
       head px py pz qx qy qz qw
       input left|right trigger squeeze stick_x stick_y buttons_hex
       recenter
       hold ticks

   Pose positions are metres in OpenXR LOCAL axes and quaternions are xyzw.
   Input button bits are: 1 squeeze click, 2 thumbstick click, 4 primary,
   8 secondary, 16 menu.  A hold snapshots all preceding state changes.
   Lines may end in a # comment. */
int  dg_xr_script_program_parse_text(DG_XR_SCRIPT_PROGRAM *out,
                                     const char *text, size_t text_len,
                                     char *error, size_t error_cap);
void dg_xr_script_program_free(DG_XR_SCRIPT_PROGRAM *program);

/* Pure random-access sample. Returns 1 for a live tick, 0 at/after EOF.
   Edge counters and software-turn transforms belong to the live player and
   are intentionally absent here. */
int dg_xr_script_program_sample(const DG_XR_SCRIPT_PROGRAM *program,
                                uint64_t tick, DG_XR_FRAME *frame,
                                unsigned int *events);

/* start parses the immutable program and starts a neutral publisher. go makes
   tick zero visible; callers use it only after bridge and camera seams are
   armed. There is no fallback to physical OpenXR input. */
int  dg_xr_script_start(const char *path, const DG_XR_CONFIG *cfg,
                        void (*log)(const char *fmt, ...));
void dg_xr_script_go(void);
void dg_xr_script_stop(void);
int  dg_xr_script_get_frame(DG_XR_FRAME *out);
/* 0 running/not started, 1 completed or aborted and safely invalidated,
   -1 if stop timed out while the worker still owns its resources. */
int  dg_xr_script_finished(void);

/* Mailboxes corresponding to the three edge routes that live outside
   DG_XR_FRAME in dg_xr.c. Values are process-monotonic across script runs. */
uint64_t      dg_xr_script_primary_press_seq(unsigned int hand);
uint64_t      dg_xr_script_menu_press_seq(void);
unsigned long dg_xr_script_secondary_presses(unsigned int hand);
unsigned long dg_xr_script_secondary_ignored(unsigned int hand);
long          dg_xr_script_recenter_count(void);

void   dg_xr_script_turn_rate(double deg_per_s);
double dg_xr_script_turn_offset_rad(void);
void   dg_xr_script_recenter(void);

#ifdef DG_HOOK_TEST
/* Manual live-player seam for deterministic unit tests. It runs the same edge,
   recenter, turn, transform and neutral-release code as the worker, without a
   thread, foreground checks or sleeps. */
int dg_xr_script_test_begin(const DG_XR_SCRIPT_PROGRAM *program,
                            const DG_XR_CONFIG *cfg);
int dg_xr_script_test_step(DG_XR_FRAME *frame);
int dg_xr_script_test_abort(DG_XR_FRAME *neutral_frame);
#endif

#endif
