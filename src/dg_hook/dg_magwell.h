#ifndef DG_MAGWELL_H
#define DG_MAGWELL_H
#include <stdint.h>
#include <string.h>
#include <math.h>
/* Value-only snapshot. Never retain a game pointer in the GPU path. */
typedef struct { uint64_t ms; int valid; float world[16]; } DG_MAGWELL;
#define DG_MAGWELL_VERTICES 48
/* USP receiver rim in native root-local units, measured from installed
   usp.kms / usp_sub.kms model0. Bottom ~-48; use -51 for 3-unit clearance.
   Reload contact anchor is a DIFFERENT point and remains unchanged. */
static const float dg_usp_magwell_mouth[4]={17.5f,-98.f,-51.f,0.f};
static int dg_magwell_vertices(const DG_MAGWELL *s,uint64_t now,float out[DG_MAGWELL_VERTICES][3]) {
    int corner,axis,i,j,k=0;float result[DG_MAGWELL_VERTICES][3];
    static const int ix[6]={0,1,2,2,1,3};
    if(!s||!s->valid||now<s->ms||now-s->ms>100)return 0;
    for(i=0;i<16;i++)if(!isfinite(s->world[i]))return 0;
    for(corner=0;corner<4;corner++)for(axis=0;axis<2;axis++) {
        float sx=(corner&1)?1.f:-1.f,sy=(corner&2)?1.f:-1.f;
        float x=sx*17.f,y=sy*32.f,quad[4][2];
        /* Two short rectangular strips at each corner, open centre. */
        float dx=sx*(axis?2.f:8.f),dy=sy*(axis?10.f:2.f);
        quad[0][0]=x;quad[0][1]=y;quad[1][0]=x-dx;quad[1][1]=y;
        quad[2][0]=x;quad[2][1]=y-dy;quad[3][0]=x-dx;quad[3][1]=y-dy;
        for(i=0;i<6;i++,k++)for(j=0;j<3;j++) {
            result[k][j]=s->world[12+j]+quad[ix[i]][0]*s->world[j]+quad[ix[i]][1]*s->world[4+j];
            if(!isfinite(result[k][j]))return 0;
        }
    }
    memcpy(out,result,sizeof result);return 1;
}
#endif
