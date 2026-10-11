#pragma once
#include <d3d11_1.h>
#include <d3dcompiler.h>
#include <wrl/client.h>
#include "dg_magwell.h"
#pragma comment(lib,"d3dcompiler.lib")
using Microsoft::WRL::ComPtr;
/* Own pipeline state, shared native depth. No depth writes and no game
   constant-buffer edits. All fallible creation happens before state swap. */
struct DgMagwellGpu {
    ComPtr<ID3D11Device> owner;bool ready=false;
    ComPtr<ID3D11VertexShader> vs;ComPtr<ID3D11PixelShader> ps;
    ComPtr<ID3D11InputLayout> layout;ComPtr<ID3D11Buffer> vb;
    ComPtr<ID3D11RasterizerState> raster;ComPtr<ID3D11DepthStencilState> depth[2];
    ComPtr<ID3DDeviceContextState> scratch;
    bool init(ID3D11Device*d) {
        if(owner.Get()==d)return ready;
        *this=DgMagwellGpu{};owner=d;
        ComPtr<ID3D11Device1>d1;if(FAILED(d->QueryInterface(IID_PPV_ARGS(&d1))))return false;
        D3D_FEATURE_LEVEL fl=d->GetFeatureLevel();
        UINT flags=(d->GetCreationFlags()&D3D11_CREATE_DEVICE_SINGLETHREADED)?D3D11_1_CREATE_DEVICE_CONTEXT_STATE_SINGLETHREADED:0;
        if(FAILED(d1->CreateDeviceContextState(flags,&fl,1,D3D11_SDK_VERSION,__uuidof(ID3D11Device),nullptr,&scratch)))return false;
        const char*code="float4 vs(float4 p:POSITION):SV_Position{return p;} float4 ps():SV_Target{return float4(0.12549,0.62745,0.40784,1);}";
        ComPtr<ID3DBlob>a,b,err;
        if(FAILED(D3DCompile(code,strlen(code),"magwell",nullptr,nullptr,"vs","vs_4_0",0,0,&a,&err))||
           FAILED(D3DCompile(code,strlen(code),"magwell",nullptr,nullptr,"ps","ps_4_0",0,0,&b,&err)))return false;
        if(FAILED(d->CreateVertexShader(a->GetBufferPointer(),a->GetBufferSize(),nullptr,&vs))||
           FAILED(d->CreatePixelShader(b->GetBufferPointer(),b->GetBufferSize(),nullptr,&ps)))return false;
        D3D11_INPUT_ELEMENT_DESC element={"POSITION",0,DXGI_FORMAT_R32G32B32A32_FLOAT,0,0,D3D11_INPUT_PER_VERTEX_DATA,0};
        if(FAILED(d->CreateInputLayout(&element,1,a->GetBufferPointer(),a->GetBufferSize(),&layout)))return false;
        D3D11_BUFFER_DESC bd{};bd.ByteWidth=2*DG_MAGWELL_VERTICES*16;bd.Usage=D3D11_USAGE_DYNAMIC;bd.BindFlags=D3D11_BIND_VERTEX_BUFFER;bd.CPUAccessFlags=D3D11_CPU_ACCESS_WRITE;
        if(FAILED(d->CreateBuffer(&bd,nullptr,&vb)))return false;
        D3D11_RASTERIZER_DESC rd{};rd.FillMode=D3D11_FILL_SOLID;rd.CullMode=D3D11_CULL_NONE;rd.DepthClipEnable=TRUE;rd.MultisampleEnable=TRUE;
        if(FAILED(d->CreateRasterizerState(&rd,&raster)))return false;
        for(int i=0;i<2;i++){D3D11_DEPTH_STENCIL_DESC dd{};dd.DepthEnable=TRUE;dd.DepthWriteMask=D3D11_DEPTH_WRITE_MASK_ZERO;dd.DepthFunc=i?D3D11_COMPARISON_GREATER_EQUAL:D3D11_COMPARISON_LESS_EQUAL;
            if(FAILED(d->CreateDepthStencilState(&dd,&depth[i])))return false;}
        ready=true;return true;
    }
    bool renderPair(ID3D11Device*d,ID3D11DeviceContext*c,ID3D11RenderTargetView*rt0,ID3D11DepthStencilView*ds0,
                ID3D11RenderTargetView*rt1,ID3D11DepthStencilView*ds1,const DG_MAGWELL&s,
                const float camera0[16],const float camera1[16],unsigned width,unsigned height,bool reversed,uint64_t now) {
        float world[DG_MAGWELL_VERTICES][3],clip[2][DG_MAGWELL_VERTICES][4];
        if(!rt0||!ds0||!rt1||!ds1||!width||!height||!dg_magwell_vertices(&s,now,world))return false;
        /* Validate and upload BOTH eyes before either draw. */
        for(int eye=0;eye<2;eye++) {
            const float*camera=eye?camera1:camera0;
            for(int i=0;i<16;i++)if(!isfinite(camera[i]))return false;
            for(int i=0;i<DG_MAGWELL_VERTICES;i++)for(int j=0;j<4;j++) {
                double v=camera[12+j];for(int k=0;k<3;k++)v+=(double)world[i][k]*camera[k*4+j];
                clip[eye][i][j]=(float)v;if(!isfinite(clip[eye][i][j]))return false;
            }
        }
        ComPtr<ID3D11DeviceContext1>c1;if(FAILED(c->QueryInterface(IID_PPV_ARGS(&c1)))||!init(d))return false;

        D3D11_MAPPED_SUBRESOURCE mapped{};
        if(FAILED(c->Map(vb.Get(),0,D3D11_MAP_WRITE_DISCARD,0,&mapped)))return false;
        memcpy(mapped.pData,clip,sizeof clip);c->Unmap(vb.Get(),0);
        ComPtr<ID3DDeviceContextState>original,returned;c1->SwapDeviceContextState(scratch.Get(),&original);
        c->ClearState();c->OMSetDepthStencilState(depth[reversed?1:0].Get(),0);
        D3D11_VIEWPORT viewport={0,0,(float)width,(float)height,0,1};c->RSSetViewports(1,&viewport);c->RSSetState(raster.Get());
        UINT stride=16,offset=0;auto buffer=vb.Get();c->IASetVertexBuffers(0,1,&buffer,&stride,&offset);
        c->IASetInputLayout(layout.Get());c->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
        c->VSSetShader(vs.Get(),nullptr,0);c->PSSetShader(ps.Get(),nullptr,0);
        c->OMSetRenderTargets(1,&rt0,ds0);c->Draw(DG_MAGWELL_VERTICES,0);
        c->OMSetRenderTargets(1,&rt1,ds1);c->Draw(DG_MAGWELL_VERTICES,DG_MAGWELL_VERTICES);

        c1->SwapDeviceContextState(original.Get(),&returned);return true;
    }
};

