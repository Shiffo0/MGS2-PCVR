#ifndef DG_MODEL_RADAR_H
#define DG_MODEL_RADAR_H
#include "dg_model_arm.h"
#include "dg_radar_gaze.h"
static int dg_model_unit(double v[3]) {
    double n=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);int i;
    if(!isfinite(n)||n<1e-6)return 0;
    for(i=0;i<3;i++)v[i]/=n;return 1;
}
static void dg_model_cross(const double a[3],const double b[3],double c[3]) {
    c[0]=a[1]*b[2]-a[2]*b[1];c[1]=a[2]*b[0]-a[0]*b[2];c[2]=a[0]*b[1]-a[1]*b[0];
}
static int dg_model_radar_pose(const DG_MODEL_ARM *arm,double width,double height,uint64_t now,DG_RADAR_POSE *out)
{
    double x[3],y[3],z[3],m[3][3],q[4],dot=0,s,trace;int i;
    if(!arm||!arm->valid||now<arm->ms||now-arm->ms>100||!isfinite(width)||width<=0||width>.5||!isfinite(height)||height<=0||height>.5)return 0;
    for(i=0;i<3;i++){x[i]=arm->wrist[i]-arm->elbow[i];z[i]=arm->normal[i];}
    if(!dg_model_unit(x))return 0;
    for(i=0;i<3;i++)dot+=x[i]*z[i];
    for(i=0;i<3;i++)z[i]-=dot*x[i];
    if(!dg_model_unit(z))return 0;
    dg_model_cross(z,x,y);if(!dg_model_unit(y))return 0;
    /* Columns are the quad axes in LOCAL; +X follows elbow->wrist and
       +Z follows the native forearm surface, including its actual roll. */
    for(i=0;i<3;i++){m[i][0]=x[i];m[i][1]=y[i];m[i][2]=z[i];}
    trace=m[0][0]+m[1][1]+m[2][2];
    if(trace>0){s=sqrt(trace+1)*2;q[3]=s*.25;q[0]=(m[2][1]-m[1][2])/s;q[1]=(m[0][2]-m[2][0])/s;q[2]=(m[1][0]-m[0][1])/s;}
    else {int a=m[1][1]>m[0][0]?1:0,b,c;if(m[2][2]>m[a][a])a=2;b=(a+1)%3;c=(b+1)%3;
        s=sqrt(1+m[a][a]-m[b][b]-m[c][c])*2;q[a]=s*.25;q[b]=(m[b][a]+m[a][b])/s;q[c]=(m[c][a]+m[a][c])/s;q[3]=(m[c][b]-m[b][c])/s;}
    out->qx=q[0];out->qy=q[1];out->qz=q[2];out->qw=q[3];
    /* Lower-right corner terminates at the wrist; 30 mm estimated forearm radius sets
       the panel above the bone axis (skin radius needs headset acceptance). The panel extends towards the elbow, never the hand. */
    out->px=arm->wrist[0]-x[0]*width*.5+y[0]*height*.5+z[0]*.030;
    out->py=arm->wrist[1]-x[1]*width*.5+y[1]*height*.5+z[1]*.030;
    out->pz=arm->wrist[2]-x[2]*width*.5+y[2]*height*.5+z[2]*.030;
    return isfinite(out->px)&&isfinite(out->py)&&isfinite(out->pz);
}
/* Aim at the physical wrist, irrespective of panel size/corner offset. */
static int dg_model_radar_angles(const DG_MODEL_ARM *arm,const DG_RADAR_POSE *head,
    const DG_RADAR_POSE *quad,double pitch,double *theta,double *phi)
{
    DG_RADAR_POSE wrist=*quad;
    wrist.px=arm->wrist[0];wrist.py=arm->wrist[1];wrist.pz=arm->wrist[2];
    return dg_radar_gaze_angles(head,&wrist,pitch,theta,phi);
}
#endif
