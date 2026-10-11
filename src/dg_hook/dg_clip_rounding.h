#pragma once
#include <math.h>



static void dgClipSourceRounding(const float source[16], float out[4]) {
    for (int k=0; k<4; ++k) {
        float z=source[k*4+2], w=source[k*4+3];
        out[k]=(5.960464477539063e-8f*0.5f)*(fabsf(2*z-w)+fabsf(w));
    }
}
