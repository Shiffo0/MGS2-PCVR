/* dg_proj.h - the four-element per-eye projection override.
 *
 * MGS2 stores its projection as a row-vector matrix.  Per-eye FOV changes
 * therefore do not require rebuilding a projection (and accidentally
 * changing its near/far mapping): only the two scale terms and the two
 * off-centre terms are changed.  dg_proj.c deliberately leaves every other
 * float alone.
 */

#ifndef DG_PROJ_H
#define DG_PROJ_H

/* Supplies both MAT and DG_PROJ_FOV. The types sit there, not here, so that
   this header can depend on that one without the dependency running both
   ways - see the note beside DG_PROJ_FOV. */
#include "dg_xr.h"

/* Replace the existing off-centre terms with the offset implied by fov. */
void dg_proj_replace(MAT *projection, const DG_PROJ_FOV *fov);

/* Keep the existing off-centre terms and add the offset implied by fov.
   This is for projection families whose base matrix already is off-centre. */
void dg_proj_add(MAT *projection, const DG_PROJ_FOV *fov);

/* Retarget a projection while preserving that member's own screen units and
   visible physical extent.  Only the two scale and two centre terms change. */
void dg_proj_retarget(MAT *projection, const DG_PROJ_FOV *old_fov,
                      const DG_PROJ_FOV *new_fov);

#endif
