


#ifndef DG_SECOND_CAMERA_H
#define DG_SECOND_CAMERA_H
#include <math.h>
#include <stdint.h>
#include <string.h>

static int dg_second_camera_delta(const float source[16],const float target[16],double delta[16]) {
    double a[4][8],out[16];int i,j,k,p;
    if(!source||!target||!delta)return 0;
    for(i=0;i<4;i++)for(j=0;j<4;j++) {
        if(!isfinite(source[i*4+j])||!isfinite(target[i*4+j]))return 0;
        a[i][j]=source[i*4+j];a[i][j+4]=(i==j);
    }
    for(j=0;j<4;j++) {
        double v;p=j;
        for(i=j+1;i<4;i++)if(fabs(a[i][j])>fabs(a[p][j]))p=i;
        if(fabs(a[p][j])<1e-12)return 0;
        for(k=0;k<8;k++){v=a[j][k];a[j][k]=a[p][k];a[p][k]=v;}
        v=a[j][j];for(k=0;k<8;k++)a[j][k]/=v;
        for(i=0;i<4;i++)if(i!=j){v=a[i][j];for(k=0;k<8;k++)a[i][k]-=v*a[j][k];}
    }
    for(i=0;i<4;i++)for(j=0;j<4;j++) {
        double v=0;for(k=0;k<4;k++)v+=a[i][k+4]*target[k*4+j];
        if(!isfinite(v))return 0;out[i*4+j]=v;
    }
    /* Check solve residual before accepting an ill-conditioned inverse. */
    for(i=0;i<4;i++)for(j=0;j<4;j++) {
        double v=0,scale=fabs(target[i*4+j])+1;
        for(k=0;k<4;k++)v+=source[i*4+k]*out[k*4+j];
        if(fabs(v-target[i*4+j])>1e-6*scale)return 0;
    }
    memcpy(delta,out,sizeof out);return 1;
}

static int dg_second_camera_bank(uint64_t shader,const void *source,size_t bytes,
                                const double delta[16],void *destination) {
    float input[16],output[16];int i,j,k;
    uintptr_t a=(uintptr_t)source,b=(uintptr_t)destination;
    if(shader!=UINT64_C(0x544F1B615F77E0FD)||!source||!destination||!delta||
       bytes<384||bytes>65536||bytes%16)return 0;
    /* Reject overlap, including in-place modification, without overflow. */
    if(a<=b?b-a<bytes:a-b<bytes)return 0;
    memcpy(input,(const unsigned char*)source+320,sizeof input);
    for(i=0;i<16;i++)if(!isfinite(input[i])||!isfinite(delta[i]))return 0;
    for(i=0;i<4;i++)for(j=0;j<4;j++) {
        double v=0;
        for(k=0;k<4;k++)v+=delta[k*4+i]*input[k*4+j];
        output[i*4+j]=(float)v;if(!isfinite(output[i*4+j]))return 0;
    }
    memcpy(destination,source,bytes);
    memcpy((unsigned char*)destination+320,output,sizeof output);return 1;
}
#endif
