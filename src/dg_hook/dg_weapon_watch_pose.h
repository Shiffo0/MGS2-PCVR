#ifndef DG_WEAPON_WATCH_POSE_H
#define DG_WEAPON_WATCH_POSE_H
#include "dg_model_radar.h"
/* Text runs across the forearm, top edge toward the hand. Centered on
 * the bone axis, 55 mm proximal to the wrist, 25 mm above the surface.
 * Reverse the surface quarter-turn: inward inspection, based on the
 * user observing the opposite (outward) roll with build 329BD099.
 * This is a candidate anatomical alignment, pending headset acceptance. */
static int dg_watch_pose(const DG_MODEL_ARM *arm,double width,double height,uint64_t now,DG_RADAR_POSE *out) {
 DG_MODEL_ARM basis;double along[3],normal[3],across[3],dot=0;int k;
 if(!arm)return 0;basis=*arm;
 for(k=0;k<3;k++){along[k]=arm->wrist[k]-arm->elbow[k];normal[k]=arm->normal[k];}
 if(!dg_model_unit(along))return 0;
 for(k=0;k<3;k++)dot+=along[k]*normal[k];
 for(k=0;k<3;k++)normal[k]-=dot*along[k];
 if(!dg_model_unit(normal))return 0;
 dg_model_cross(along,normal,across);
 for(k=0;k<3;k++){normal[k]=-across[k];basis.normal[k]=normal[k];}
 dg_model_cross(along,normal,across);
 for(k=0;k<3;k++)basis.elbow[k]=basis.wrist[k]-across[k];
 if(!dg_model_radar_pose(&basis,width,height,now,out))return 0;
 out->px=arm->wrist[0]-.055*along[0]+.025*normal[0];
 out->py=arm->wrist[1]-.055*along[1]+.025*normal[1];
 out->pz=arm->wrist[2]-.055*along[2]+.025*normal[2];
 return isfinite(out->px)&&isfinite(out->py)&&isfinite(out->pz);
}
/* Use the submitted screen center and an independent head direction. Reject
 * a hand folded into the face; changing left-radar pitch must not move this gate. */
static int dg_watch_angles(const DG_RADAR_POSE *head,const DG_RADAR_POSE *quad,
 double *gaze,double *face) {
 double x=quad->px-head->px,y=quad->py-head->py,z=quad->pz-head->pz;
 double distance2=x*x+y*y+z*z;
 *gaze=*face=180;
 if(!isfinite(distance2)||distance2<.20*.20||distance2>1.0)return 0;
 return dg_radar_gaze_angles(head,quad,0,gaze,face);
}
/* Inner-wrist inspection has its own gate. Radar tuning cannot make this
 * permanently visible. Close immediately when the inner surface turns away. */
static int dg_watch_gate_step(DG_RADAR_GATE *g,int valid,double gaze,double face,uint64_t now) {
 int want;
 if(!valid||!isfinite(gaze)||!isfinite(face)||gaze>55||face>65){dg_radar_gate_reset(g);return 0;}
 if(g->open)return 1;
 want=gaze<45 && face<55;
 if(!want){dg_radar_gate_reset(g);return 0;}
 if(!g->timing || now<g->since){g->timing=1;g->since=now;}
 if(now-g->since>=120){g->open=1;g->timing=0;}
 return g->open;
}
#endif
