#ifndef DG_GRIP_DEBUG_H
#define DG_GRIP_DEBUG_H
#include "dg_m9_runtime.h"
#include "dg_health.h" /* reuse bounded, tested upload lifecycle */
#define DG_GRIPDBG_SIZE 256u
#define DG_GRIPDBG_PIXELS (256u*256u)
/* Inverse of dg_m9_relative, including the native/XR basis permutation.
 * These are the actual hit-test coordinates, not an anatomical estimate. */
static int dg_grip_world(const DG_GRIP_DEBUG *s,const double local[3],double out[3]) {
    double v[3]={-local[0],local[2],local[1]},t[3],n=0;
    const double *q=s->aim_q;int k;
    if(!dg_m9_vec_ok(local)||!dg_m9_vec_ok(s->origin))return 0;
    for(k=0;k<4;k++){if(!isfinite(q[k]))return 0;n+=q[k]*q[k];}
    if(fabs(n-1)>.001)return 0;
    t[0]=2*(q[1]*v[2]-q[2]*v[1]);t[1]=2*(q[2]*v[0]-q[0]*v[2]);t[2]=2*(q[0]*v[1]-q[1]*v[0]);
    out[0]=s->origin[0]+v[0]+q[3]*t[0]+q[1]*t[2]-q[2]*t[1];
    out[1]=s->origin[1]+v[1]+q[3]*t[1]+q[2]*t[0]-q[0]*t[2];
    out[2]=s->origin[2]+v[2]+q[3]*t[2]+q[0]*t[1]-q[1]*t[0];
    return dg_m9_vec_ok(out);
}
static int dg_grip_debug_points(const DG_GRIP_DEBUG *s,uint64_t now,double a[3],double b[3],double *distance) {
    int k;double d=0;
    if(!s||!s->valid||!s->sequence||now<s->stamp||now-s->stamp>100 ||
       !dg_grip_world(s,s->anchor,a)||!dg_grip_world(s,s->left,b))return 0;
    for(k=0;k<3;k++){double v=s->left[k]-s->anchor[k];d+=v*v;}
    *distance=sqrt(d);return isfinite(*distance);
}
/* Quad X follows the measured segment; its normal faces the viewer. */
static int dg_grip_line_q(const double a[3],const double b[3],const double head[3],double q[4]) {
    double x[3],y[3],z[3],n=0,dot=0,t,m[3][3];int i,j;
    for(i=0;i<3;i++){x[i]=b[i]-a[i];n+=x[i]*x[i];}
    if(n<1e-10)return 0;n=sqrt(n);
    for(i=0;i<3;i++){x[i]/=n;z[i]=head[i]-(a[i]+b[i])*.5;dot+=z[i]*x[i];}
    n=0;for(i=0;i<3;i++){z[i]-=dot*x[i];n+=z[i]*z[i];}
    if(n<1e-10)return 0;n=sqrt(n);for(i=0;i<3;i++)z[i]/=n;
    y[0]=z[1]*x[2]-z[2]*x[1];y[1]=z[2]*x[0]-z[0]*x[2];y[2]=z[0]*x[1]-z[1]*x[0];
    for(i=0;i<3;i++){m[i][0]=x[i];m[i][1]=y[i];m[i][2]=z[i];}
    t=m[0][0]+m[1][1]+m[2][2];
    if(t>0){n=2*sqrt(t+1);q[3]=n/4;q[0]=(m[2][1]-m[1][2])/n;q[1]=(m[0][2]-m[2][0])/n;q[2]=(m[1][0]-m[0][1])/n;}
    else {
        i=m[1][1]>m[0][0]?1:0;if(m[2][2]>m[i][i])i=2;j=(i+1)%3;
        {int k=(i+2)%3;n=2*sqrt(1+m[i][i]-m[j][j]-m[k][k]);
        q[i]=n/4;q[j]=(m[i][j]+m[j][i])/n;q[k]=(m[i][k]+m[k][i])/n;q[3]=(m[k][j]-m[j][k])/n;}
    }
    return 1;
}
/* Atlas: ring TL, controller dot TR, distance-line BL, numeric label BR.
 * Straight RGBA. Green=attached, yellow=in radius, red=outside, gray=blocked. */
static void dg_grip_debug_raster(const DG_GRIP_DEBUG *s,double distance,uint32_t *pixels) {
    unsigned x,y,k,row,bit;unsigned char *p=(unsigned char *)pixels;
    unsigned char r=255,g=70,b=70;int mm=(int)(distance*1000+.5);
    static const unsigned char digits[10][7]={
        {14,17,19,21,25,17,14},{4,12,4,4,4,4,14},{14,17,1,2,4,8,31},
        {30,1,1,14,1,1,30},{2,6,10,18,31,2,2},{31,16,16,30,1,1,30},
        {14,16,16,30,17,17,14},{31,1,2,4,8,8,8},{14,17,17,14,17,17,14},{14,17,17,15,1,1,14}};
    unsigned d[4];if(mm>9999)mm=9999;if(mm<0)mm=0;
    if(!s->allowed){r=g=b=150;}else if((s->state==M9_GRABBED || s->state==M9_FULL_REAR)){r=40;g=255;b=90;}
    else if(distance<=DG_M9_GRAB_RADIUS_M){r=255;g=220;b=30;}
    memset(pixels,0,DG_GRIPDBG_PIXELS*4u);
    for(y=0;y<128;y++)for(x=0;x<128;x++) {
        double dx=x-63.5,dy=y-63.5,rr=dx*dx+dy*dy;unsigned i;
        if(rr>=60*60 && rr<=64*64){i=(y*256+x)*4;p[i]=r;p[i+1]=g;p[i+2]=b;p[i+3]=255;}
        if(rr<=60*60){i=(y*256+x+128)*4;p[i]=40;p[i+1]=220;p[i+2]=255;p[i+3]=255;}
        if(y>=56 && y<72){i=((y+128)*256+x)*4;p[i]=r;p[i+1]=g;p[i+2]=b;p[i+3]=255;}
    }
    d[0]=mm/1000;d[1]=mm/100%10;d[2]=mm/10%10;d[3]=mm%10;
    for(k=0;k<6;k++)for(row=0;row<7;row++)for(bit=0;bit<5;bit++) {
        unsigned bits=k<4?digits[d[k]][row]:(row<2?0:(row==2?26:21)); /* mm */
        if(bits&(1u<<(4-bit)))for(y=0;y<3;y++)for(x=0;x<3;x++) {
            unsigned i=((132+row*3+y)*256+132+k*19+bit*3+x)*4;
            p[i]=r;p[i+1]=g;p[i+2]=b;p[i+3]=255;
        }
    }
}
#endif
