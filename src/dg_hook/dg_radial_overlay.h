#ifndef DG_RADIAL_OVERLAY_H
#define DG_RADIAL_OVERLAY_H
#include "dg_radial_view.h"
/* Backend-independent resource lifecycle. No OpenXR, D3D, Windows or global
 * state is linked here. Production callbacks are intentionally not installed.
 * One session/render thread owns state+ops+scratch for their whole lifetime.
 * Callbacks return EXACT OK, not the OpenXR XR_SUCCEEDED classification:
 * in particular an XR_TIMEOUT_EXPIRED wait must map to IO_TIMEOUT.
 */
enum { DG_RADIAL_IO_ERROR=-1, DG_RADIAL_IO_TIMEOUT=0, DG_RADIAL_IO_OK=1 };
typedef struct dg_radial_overlay_ops {
    void *user;
    /* Transactional on failure: create leaves no owned handle. Must create a
     * 256x256, RGBA8, single-face/single-array-layer swapchain on this session.
     * Callback knows negotiated format; do not substitute BGRA silently. */
    int (*create)(void *, unsigned width, unsigned height, uint64_t *handle, unsigned *images);
    int (*acquire)(void *, uint64_t handle, unsigned *image);
    int (*wait)(void *, uint64_t handle); /* bounded/nonblocking attempt */

    int (*upload)(void *, uint64_t handle, unsigned image, const void *rgba, size_t bytes);
    int (*release)(void *, uint64_t handle);
    int (*destroy)(void *, uint64_t handle);
} dg_radial_overlay_ops;
typedef struct dg_radial_overlay_model {
    /* Contents/timestamp immutable per sequence; sequence increases for every
     * publication, including cancel, across the complete session epoch. */
    dg_radial_view view;
    uint64_t session_epoch, context_epoch, reference_epoch, catalog_version;
    uint64_t sequence, sample_ms;
} dg_radial_overlay_model;
typedef struct dg_radial_overlay_frame {
    uint64_t session_epoch, context_epoch, reference_epoch, catalog_version;
    uint64_t sequence, now_ms;
    unsigned base_layers, max_layers; /* min(runtime limit, actual layer-array capacity) */
    int running, focused, should_render, projection_ready, theater_active;
} dg_radial_overlay_frame;
typedef struct dg_radial_overlay {
    uint64_t session_epoch, handle, last_frame, last_ms, last_model, blocked_model;
    unsigned images, image;
    int seen_frame, acquired, fault;
} dg_radial_overlay;
typedef struct dg_radial_overlay_layer {
    int submit; /* true only after a successful upload AND release this call */
    uint64_t handle;
    unsigned array_index, rect_x, rect_y, rect_w, rect_h;
    unsigned source_alpha, unpremultiplied_alpha, both_eyes, view_space;
    float position_z, orientation_w, width_m, height_m;
} dg_radial_overlay_layer;
void dg_radial_overlay_init(dg_radial_overlay *s, uint64_t session_epoch);
dg_radial_overlay_layer dg_radial_overlay_step(dg_radial_overlay *s,
    const dg_radial_overlay_ops *ops, const dg_radial_overlay_frame *frame,
    const dg_radial_overlay_model *model, uint32_t *scratch, size_t capacity);
/* Explicit owner-confirmed stopped session, before destroying its parent.
 * Never resets a live acquired image. Failure retains the handle for teardown.
 * After success call init only for a genuinely new session epoch. */
int dg_radial_overlay_teardown(dg_radial_overlay *s,
    const dg_radial_overlay_ops *ops, int session_stopped);
#endif
