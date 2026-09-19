/* Independent controller-to-native rest orientation, in camera-derived world
   axes. Finger locals and wrist position are not part of this contract. */
#ifndef DG_FREE_WRIST_H
#define DG_FREE_WRIST_H
#include "dg_position_target.h"
typedef struct { int enabled,valid; double world[4]; } DG_FREE_WRIST_INPUT;
typedef struct { int ready; double controller0[4],rest0[4]; } DG_FREE_WRIST_STATE;
static int dg_free_wrist_step(DG_FREE_WRIST_STATE *s,
    const DG_FREE_WRIST_INPUT *in,const double rest[4],double out[4])
{
    double q[4],r[4],inv[4],delta[4];
    if(!s || !in || !in->enabled || !in->valid) {
        if(s) memset(s,0,sizeof *s);
        return 0;
    }
    memcpy(q,in->world,sizeof q);
    if(!dg_ik_quat_normalize(q)) {memset(s,0,sizeof *s);return 0;}
    if(!s->ready) {
        memcpy(r,rest,sizeof r);
        if(!dg_ik_quat_normalize(r))return 0;
        memcpy(s->controller0,q,sizeof q);memcpy(s->rest0,r,sizeof r);s->ready=1;
    }
    dg_ik_quat_conj(s->controller0,inv);dg_ik_quat_mul(q,inv,delta);
    dg_ik_quat_mul(delta,s->rest0,out);
    return dg_ik_quat_normalize(out);
}
#endif
