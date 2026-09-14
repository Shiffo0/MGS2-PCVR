#ifndef DG_RADIAL_VIEW_H
#define DG_RADIAL_VIEW_H
#include <stddef.h>
#include <stdint.h>
#define DG_RADIAL_VIEW_SIZE 256u
#define DG_RADIAL_VIEW_PIXELS (256u * 256u)
/* Caller supplies a coherent, fresh snapshot. Empty label = empty slot.
   kind 0 weapons, 1 items. Only ASCII A-Z/a-z, digits, space and '-' accepted.
   No inventory, lifetime, ownership or engine acknowledgement is inferred. */
typedef struct {
    int visible;
    unsigned count;
    int selected;
    unsigned eligible;
    unsigned kind;
    char labels[8][17];
} dg_radial_view;
/* Exactly 256x256 RGBA8 bytes, row-major, straight alpha (0 or 255).
   uint32_t storage is only alignment/storage: do not interpret native integers.
   Returns 1 rendered, 0 cleared (hidden/invalid), -1 null/short buffer untouched.
   Clears exactly DG_RADIAL_VIEW_PIXELS pixels; surplus capacity untouched. */
int dg_radial_view_raster(const dg_radial_view *view, uint32_t *pixels,
                          size_t pixel_count);
#endif
