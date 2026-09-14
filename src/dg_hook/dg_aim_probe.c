#include "dg_aim_probe.h"
#include <float.h>
#include <math.h>
#include <string.h>

static int finite_d(double x) { return x == x && x <= DBL_MAX && x >= -DBL_MAX; }
static double dot(const double a[3], const double b[3]) {
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}
static void cross(const double a[3], const double b[3], double c[3]) {
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
static void unit(double v[3]) {
    double n=sqrt(dot(v,v)); int i; for(i=0;i<3;i++) v[i]/=n;
}
static double angle(double cosine) {
    if(cosine>1.0) cosine=1.0; if(cosine< -1.0) cosine=-1.0;
    return acos(cosine)*57.2957795130823208768;
}
static void rotate(const double q[4], const double v[3], double out[3]) {
    double u[3], uu[3]; int i; cross(q,v,u); cross(q,u,uu);
    for(i=0;i<3;i++) out[i]=v[i]+2.0*(q[3]*u[i]+uu[i]);
}
static DG_AIM_PROBE_RESULT quat(const double q[4]) {
    int i; double n=0.0;
    for(i=0;i<4;i++) { if(!finite_d(q[i])) return DG_AIM_PROBE_NONFINITE; n+=q[i]*q[i]; }
    if(!finite_d(n) || fabs(n-1.0)>DG_AIM_PROBE_QUAT_NORM2_TOL) return DG_AIM_PROBE_QUATERNION;
    return DG_AIM_PROBE_OK;
}
static DG_AIM_PROBE_RESULT matrix(const float m[3][3]) {
    double r[3][3], c[3]; int i,j;
    for(i=0;i<3;i++) for(j=0;j<3;j++) {
        if(!finite_d(m[i][j])) return DG_AIM_PROBE_NONFINITE;
        r[i][j]=m[i][j];
    }
    for(i=0;i<3;i++) for(j=0;j<3;j++)
        if(fabs(dot(r[i],r[j])-(i==j?1.0:0.0))>DG_AIM_PROBE_MATRIX_TOL) return DG_AIM_PROBE_MATRIX;
    cross(r[0],r[1],c);
    if(fabs(dot(c,r[2])-1.0)>DG_AIM_PROBE_MATRIX_TOL) return DG_AIM_PROBE_MATRIX;
    return DG_AIM_PROBE_OK;
}
static void in_camera(const float camera[3][3], const double world[3], double out[3]) {
    int i; for(i=0;i<3;i++) out[i]=camera[i][0]*world[0]+camera[i][1]*world[1]+camera[i][2]*world[2];
    unit(out);
}

DG_AIM_PROBE_RESULT dg_aim_probe_build(const DG_AIM_PROBE_INPUT *in, DG_AIM_PROBE_SAMPLE *out) {
    DG_AIM_PROBE_SAMPLE s; DG_AIM_PROBE_RESULT rc;
    double minus_y[3]={0,-1,0}, minus_z[3]={0,0,-1};
    double inv_eye[4], aim_local[3], hand_basis[3][3], trace=0.0;
    int i,j;
    if(!out) return DG_AIM_PROBE_NULL;
    memset(out,0,sizeof *out);
    if(!in) return DG_AIM_PROBE_NULL;
    if((in->flags & DG_AIM_PROBE_REQUIRED)!=DG_AIM_PROBE_REQUIRED) return DG_AIM_PROBE_MISSING;
    if(!in->selected_body || !in->selected_unit || !in->selected_root || !in->expected_arm_body ||
       in->selected_body!=in->observed_body || in->selected_unit!=in->observed_unit ||
       in->selected_root!=in->observed_root || in->expected_arm_body!=in->observed_arm_body)
        return DG_AIM_PROBE_IDENTITY;
    if(!in->observation_id || !in->stream_id || !in->publication_aim_seq ||
       in->observation_id!=in->hierarchy_observation_id ||
       in->publication_aim_seq!=in->camera_publication_aim_seq ||
       in->publication_aim_seq!=in->aim_publication_aim_seq || in->publication_aim_seq!=in->eye_publication_aim_seq ||
       ((in->flags & DG_AIM_PROBE_CONSUMED_KNOWN) ? !in->consumed_command_sample_seq : in->consumed_command_sample_seq!=0))
        return DG_AIM_PROBE_SAMPLE_MISMATCH;
    if(in->aim_age_ms>DG_AIM_PROBE_MAX_AGE_MS || in->eye_age_ms>DG_AIM_PROBE_MAX_AGE_MS) return DG_AIM_PROBE_STALE;
    rc=quat(in->aim_raw_q); if(rc) return rc;
    rc=quat(in->eye_raw_q); if(rc) return rc;
    rc=quat(in->hand_world_q); if(rc) return rc;
    rc=matrix(in->camera_world); if(rc) return rc;
    rc=matrix(in->weapon_root_world); if(rc) return rc;
    memset(&s,0,sizeof s); s.observation=*in;
    for(i=0;i<3;i++) s.root_minus_y_world[i]= -in->weapon_root_world[1][i];
    unit(s.root_minus_y_world);
    rotate(in->hand_world_q,minus_y,s.hand_minus_y_world); unit(s.hand_minus_y_world);
    in_camera(in->camera_world,s.root_minus_y_world,s.root_minus_y_camera);
    in_camera(in->camera_world,s.hand_minus_y_world,s.hand_minus_y_camera);
    for(i=0;i<4;i++) inv_eye[i]=(i==3?1.0:-1.0)*in->eye_raw_q[i];
    rotate(in->aim_raw_q,minus_z,aim_local);
    rotate(inv_eye,aim_local,s.raw_eye_relative_aim_xr); unit(s.raw_eye_relative_aim_xr);
    for(i=0;i<3;i++) {
        double axis[3]={0,0,0}; axis[i]=1;
        rotate(in->hand_world_q,axis,hand_basis[i]); unit(hand_basis[i]);
        for(j=0;j<3;j++) trace+=hand_basis[i][j]*in->weapon_root_world[i][j];
    }
    s.root_hand_basis_angle_deg=angle((trace-1.0)*0.5);
    s.root_hand_minus_y_angle_deg=angle(dot(s.root_minus_y_world,s.hand_minus_y_world));
    s.version=DG_AIM_PROBE_VERSION; s.valid=1; *out=s;
    return DG_AIM_PROBE_OK;
}
const char *dg_aim_probe_result_name(DG_AIM_PROBE_RESULT r) {
    switch(r) {
    case DG_AIM_PROBE_OK:return "ok_observation_proxy";
    case DG_AIM_PROBE_NULL:return "null";
    case DG_AIM_PROBE_MISSING:return "missing_validity";
    case DG_AIM_PROBE_IDENTITY:return "selected_observed_identity_mismatch";
    case DG_AIM_PROBE_SAMPLE_MISMATCH:return "observation_sample_mismatch";
    case DG_AIM_PROBE_STALE:return "stale";
    case DG_AIM_PROBE_NONFINITE:return "nonfinite";
    case DG_AIM_PROBE_QUATERNION:return "quaternion_not_unit";
    case DG_AIM_PROBE_MATRIX:return "matrix_not_rigid";
    default:return "unknown_rejection";
    }
}
