#ifndef DG_MENU_POINTER_H
#define DG_MENU_POINTER_H
/* Intersect OpenXR aim (-Z) with MGS2's yaw-only LOCAL theater quad. */
static int dg_menu_hit(const DG_XR_HAND_POSE *aim, const DG_XR_RAW_POSE *quad,
                       double width, double height, double *u, double *v) {
    const DG_XR_RAW_POSE *p = &aim->raw_local;
    double norm, dx, dy, dz, nx, nz, rx, rz, t, x, y, z;
    if (!aim->active || !aim->tracked || !aim->sample_seq || aim->pose_age_ms>100 ||
        !aim->orientation_valid || !aim->position_valid || !(width>0 && height>0))
        return 0;
    norm=p->qx*p->qx+p->qy*p->qy+p->qz*p->qz+p->qw*p->qw;
    if (!(norm>0.99 && norm<1.01)) return 0;
    dx=-2*(p->qx*p->qz+p->qw*p->qy)/norm;
    dy=2*(p->qw*p->qx-p->qy*p->qz)/norm;
    dz=2*(p->qx*p->qx+p->qy*p->qy)/norm-1;
    nx=2*quad->qy*quad->qw; nz=1-2*quad->qy*quad->qy;
    rx=nz; rz=-nx;
    x=p->px-quad->px; y=p->py-quad->py; z=p->pz-quad->pz;
    if (!(dx*nx+dz*nz < -0.0001)) return 0;
    t=-(x*nx+z*nz)/(dx*nx+dz*nz);
    if (!(t>0)) return 0;
    *u=0.5+((x+t*dx)*rx+(z+t*dz)*rz)/width;
    *v=0.5-(y+t*dy)/height;
    return *u>=0 && *u<=1 && *v>=0 && *v<=1;
}
#endif
