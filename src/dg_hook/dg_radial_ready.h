#ifndef DG_RADIAL_READY_H
#define DG_RADIAL_READY_H
#include "dg_radial_owner.h"
/* Control part of equip readiness, NOT inventory/retail validation.
 * The owner output must describe this exact coherent fresh input. Stick
 * neutrality is downstream geometry, or an exclusive claim with zero routed
 * output. All triggers/clicks must be released and existing fire must be idle.
 * Production additionally must revoke queued movement/turn from older samples
 * before treating the claim as effective. This CPU API cannot do that itself.
 */
int dg_radial_controls_equip_ready(const dg_radial_owner_input *in,
                                  const dg_radial_owner_output *out);
#endif
