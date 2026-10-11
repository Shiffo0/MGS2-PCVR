/* Native light colours use 0..255 units. Codec-green ambient fill, no pulse.
 * Operates on a private render-copy, never the shared native light matrix. */
#ifndef DG_RELOAD_HIGHLIGHT_H
#define DG_RELOAD_HIGHLIGHT_H
#include <math.h>
#include <string.h>
static int dg_reload_highlight_colour(float colour[16]) {
    float out[16]; int row,k;
    static const float green[3]={32.0f,160.0f,104.0f};
    for(k=0;k<16;k++)if(!isfinite(colour[k]) || fabs(colour[k])>4096.0f)return 0;
    memcpy(out,colour,sizeof out);
    for(row=0;row<4;row++)for(k=0;k<3;k++)out[row*4+k]*=0.35f;
    for(k=0;k<3;k++)out[12+k]+=green[k];
    memcpy(colour,out,sizeof out);return 1;
}
#endif
