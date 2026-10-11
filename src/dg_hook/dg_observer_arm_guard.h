#ifndef DG_OBSERVER_ARM_GUARD_H
#define DG_OBSERVER_ARM_GUARD_H


static int observer_arm_box(float box[4],const DG_MODEL_ARM *arm,
                            const DG_XR_RAW_POSE *eye,uint64_t captured) {
    double n,x,y,z,w,points[3][3],length=0;int i,j,k;
    if(!arm->valid||arm->ms>captured||captured-arm->ms>100)return 0;
    n=sqrt(eye->qx*eye->qx+eye->qy*eye->qy+eye->qz*eye->qz+eye->qw*eye->qw);
    if(!isfinite(n)||n<.9||n>1.1)return 0;
    x=-eye->qx/n;y=-eye->qy/n;z=-eye->qz/n;w=eye->qw/n;
    for(i=0;i<3;i++) {
        double d=arm->wrist[i]-arm->elbow[i];
        if(!isfinite(arm->wrist[i])||!isfinite(arm->elbow[i]))return 0;
        length+=d*d;points[0][i]=arm->elbow[i];points[1][i]=arm->wrist[i];
    }
    length=sqrt(length);if(length<.04||length>1)return 0;
    for(i=0;i<3;i++)points[2][i]=arm->wrist[i]+.08*(arm->wrist[i]-arm->elbow[i])/length;
    for(j=0;j<3;j++) {
        double v[3]={points[j][0]-eye->px,points[j][1]-eye->py,points[j][2]-eye->pz};
        double t[3]={2*(y*v[2]-z*v[1]),2*(z*v[0]-x*v[2]),2*(x*v[1]-y*v[0])};
        double p[3]={v[0]+w*t[0]+y*t[2]-z*t[1],v[1]+w*t[1]+z*t[0]-x*t[2],v[2]+w*t[2]+x*t[1]-y*t[0]};
        double radius=j==0?.10:(j==1?.10:.15);
        /* A box around each sphere is conservative under perspective. When
           it crosses the eye plane, do not invent an unbounded screen mask. */
        if(!isfinite(p[2])||p[2]+radius>=-.02)return 0;
        for(k=0;k<8;k++) {
            float u=(float)((p[0]+((k&1)?radius:-radius))/-(p[2]+((k&4)?radius:-radius)));
            float v2=(float)((p[1]+((k&2)?radius:-radius))/-(p[2]+((k&4)?radius:-radius)));
            if(!isfinite(u)||!isfinite(v2))return 0;
            box[0]=fminf(box[0],u);box[1]=fmaxf(box[1],u);
            box[2]=fminf(box[2],v2);box[3]=fmaxf(box[3],v2);
        }
    }
    return 1;
}
static int observer_arm_source(const DG_OBSERVER_LAYOUT *o,const float b[4],int preferred) {
    int n;float pad=fmaxf(o->options[1],.02f);
    for(n=0;n<2;n++) {
        int e=n?!preferred:preferred;
        /* Only the visible part needs coverage; feather stays outside the
           protected geometry. No sampling outside either captured frustum. */
        if(fmaxf(b[0]-pad,o->crop[0])>=o->eye[e][0]&&
           fminf(b[1]+pad,o->crop[1])<=o->eye[e][1]&&
           fmaxf(b[2]-pad,o->crop[2])>=o->eye[e][2]&&
           fminf(b[3]+pad,o->crop[3])<=o->eye[e][3])return e+1;
    }
    return 0;
}
static void observer_arm_guard(DG_OBSERVER_LAYOUT *o,
    const DG_MODEL_ARM *l0,const DG_MODEL_ARM *l1,const DG_MODEL_ARM *r0,const DG_MODEL_ARM *r1,
    const DG_XR_RAW_POSE *e0,const DG_XR_RAW_POSE *e1,uint64_t t0,uint64_t t1) {
    const DG_MODEL_ARM *a[2][2]={{l0,l1},{r0,r1}};int i,j,valid[2]={0,0};
    for(i=0;i<2;i++) {
        float b[4]={1e6f,-1e6f,1e6f,-1e6f};
        if(!observer_arm_box(b,a[i][0],e0,t0)||!observer_arm_box(b,a[i][1],e1,t1))continue;
        if(b[1]<o->crop[0]||b[0]>o->crop[1]||b[3]<o->crop[2]||b[2]>o->crop[3])continue;
        valid[i]=1;memcpy(o->arm[i],b,sizeof b);
        /* Only intervene where the original eye transition cuts the bounds. */
        if(!(b[0]<o->eye[1][0]+o->options[1]&&b[1]>o->eye[1][0])&&
           !(o->eye[0][1]>o->eye[1][1]&&b[0]<o->eye[1][1]&&b[1]>o->eye[1][1]-o->options[1]))continue;
        o->arm_mode[i]=(float)observer_arm_source(o,b,i);
    }
    /* Intersecting protected arms must not cut one another. Prefer a single
       eye covering their union; otherwise retain the original composition. */
    if(valid[0]&&valid[1]&&(o->arm_mode[0]||o->arm_mode[1])) {
        float *a0=o->arm[0],*a1=o->arm[1],pad=fmaxf(o->options[1],.02f)*2;
        if(a0[0]<a1[1]+pad&&a1[0]<a0[1]+pad&&a0[2]<a1[3]+pad&&a1[2]<a0[3]+pad) {
            float u[4]={fminf(a0[0],a1[0]),fmaxf(a0[1],a1[1]),fminf(a0[2],a1[2]),fmaxf(a0[3],a1[3])};
            int source=observer_arm_source(o,u,0);
            for(j=0;j<4;j++)a0[j]=u[j];o->arm_mode[0]=(float)source;o->arm_mode[1]=0;
        }
    }
}
#endif
