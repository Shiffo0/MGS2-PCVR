#ifndef DG_PSG_AIM_H
#define DG_PSG_AIM_H
#include <math.h>
#include <string.h>
/* Row-vector camera +Z is forward (retail projection m23=+1).
   Native bullets travel along local -Y. This is a basis conversion, not an
   extra pitch applied to an already corrected native shot matrix. */
static int dg_psg_matrix(const float camera[4][4], float shot[4][4]) {
    float out[4][4]; double det; int i,j,k;
    if(!camera || !shot)return 0;
    for(i=0;i<4;i++)for(j=0;j<4;j++)if(!isfinite(camera[i][j]))return 0;
    for(i=0;i<3;i++) {
        if(fabs(camera[i][3])>1e-4)return 0;
        for(j=0;j<3;j++) {
            double dot=0;for(k=0;k<3;k++)dot+=(double)camera[i][k]*camera[j][k];
            if(fabs(dot-(i==j?1:0))>1e-3)return 0;
        }
    }
    if(fabs(camera[3][3]-1)>1e-4)return 0;
    det=camera[0][0]*(camera[1][1]*camera[2][2]-camera[1][2]*camera[2][1])
       -camera[0][1]*(camera[1][0]*camera[2][2]-camera[1][2]*camera[2][0])
       +camera[0][2]*(camera[1][0]*camera[2][1]-camera[1][1]*camera[2][0]);
    if(fabs(det-1)>1e-3)return 0;
    memset(out,0,sizeof out);
    for(k=0;k<3;k++) {
        out[0][k]=camera[0][k];out[1][k]=-camera[2][k];out[2][k]=camera[1][k];
        out[3][k]=camera[3][k];
    }
    out[3][3]=1;memcpy(shot,out,sizeof out);return 1;
}
#endif
