#pragma once
#include "dg_second_camera_gpu.h"
static void dgGameClip(const float in[16],float out[16]){
 for(int r=0;r<4;r++){const float*a=in+r*4;float*o=out+r*4;o[0]=a[0];o[1]=-a[1];o[2]=0.5f*a[2]+0.5f*a[3];o[3]=a[3];}
}
struct FramePass {
 ComPtr<ID3D11Texture2D> color,depth;ComPtr<ID3D11RenderTargetView> rt;ComPtr<ID3D11DepthStencilView> ds;
 ComPtr<ID3D11ShaderResourceView> depthView;
};
