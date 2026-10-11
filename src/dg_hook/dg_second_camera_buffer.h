/* One-shot second-camera proof: exact currently bound VS buffer, private copy.
 * Synchronous readback deliberately perturbs timing. Never use as a baseline
 * or production stereo path. Caller provides verified same-frame cameras,
 * shader identity, serialized context and private output/depth targets. */
#pragma once
#include <d3d11_1.h>
#include <wrl/client.h>
#include <vector>
#include <new>
#include "dg_second_camera.h"

class SecondCameraBuffer {
    Microsoft::WRL::ComPtr<ID3D11Buffer> original,alternate;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext1> context1;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> preparedContext;
    ID3D11DeviceContext *bound=nullptr;
    UINT first=0,count=0;
public:
    SecondCameraBuffer()=default;
    SecondCameraBuffer(const SecondCameraBuffer&)=delete;
    SecondCameraBuffer& operator=(const SecondCameraBuffer&)=delete;
    ~SecondCameraBuffer(){restore();}
    const char *prepare(ID3D11Device *device,ID3D11DeviceContext *context,
                        uint64_t shader,const float source[16],const float target[16]) {
        if(bound||original||alternate)return "already_prepared";
        if(!device||!context||context->GetType()!=D3D11_DEVICE_CONTEXT_IMMEDIATE)return "context";
        Microsoft::WRL::ComPtr<ID3D11Device> owner;
        context->GetDevice(&owner);
        if(owner.Get()!=device)return "device_mismatch";
        double delta[16];
        if(shader!=UINT64_C(0x544F1B615F77E0FD))return "world_shader";
        if(!dg_second_camera_delta(source,target,delta))return "camera_matrix";
        context->QueryInterface(IID_PPV_ARGS(&context1));
        if(context1)context1->VSGetConstantBuffers1(0,1,&original,&first,&count);
        else context->VSGetConstantBuffers(0,1,&original);
        if(!original)return "missing_cb0";
        D3D11_BUFFER_DESC desc{};original->GetDesc(&desc);
        if(context1&&(first>4096||count>4096||count<24))return "buffer_range";
        UINT begin=context1?first*16:0;
        if(desc.ByteWidth<384||desc.ByteWidth>65536||desc.ByteWidth%16||
           begin>desc.ByteWidth||desc.ByteWidth-begin<384||
           (context1&&count<24))return "buffer_range";
        Microsoft::WRL::ComPtr<ID3D11Buffer> staging;
        auto readDesc=desc;readDesc.Usage=D3D11_USAGE_STAGING;readDesc.BindFlags=0;
        readDesc.CPUAccessFlags=D3D11_CPU_ACCESS_READ;readDesc.MiscFlags=0;readDesc.StructureByteStride=0;
        if(FAILED(device->CreateBuffer(&readDesc,nullptr,&staging)))return "readback_create";
        context->CopyResource(staging.Get(),original.Get());
        /* All potentially throwing allocations precede Map. */
        std::vector<unsigned char> input,output;
        try { input.resize(desc.ByteWidth);output.resize(desc.ByteWidth); }
        catch(const std::bad_alloc&) { return "allocation"; }
        D3D11_MAPPED_SUBRESOURCE map{};
        if(FAILED(context->Map(staging.Get(),0,D3D11_MAP_READ,0,&map)))return "readback_map";
        memcpy(input.data(),map.pData,desc.ByteWidth);context->Unmap(staging.Get(),0);
        memcpy(output.data(),input.data(),desc.ByteWidth);
        if(!dg_second_camera_bank(shader,input.data()+begin,desc.ByteWidth-begin,delta,output.data()+begin))return "bank_transform";
        auto privateDesc=desc;privateDesc.Usage=D3D11_USAGE_IMMUTABLE;
        privateDesc.BindFlags=D3D11_BIND_CONSTANT_BUFFER;privateDesc.CPUAccessFlags=0;
        privateDesc.MiscFlags=0;privateDesc.StructureByteStride=0;
        D3D11_SUBRESOURCE_DATA data{};data.pSysMem=output.data();
        if(FAILED(device->CreateBuffer(&privateDesc,&data,&alternate)))return "private_cb_create";
        preparedContext=context;
        return nullptr;
    }
    bool bind(ID3D11DeviceContext *context) {
        if(bound||!alternate||!original||context!=preparedContext.Get())return false;
        /* Reject a stale preparation rather than restoring an unrelated bank. */
        Microsoft::WRL::ComPtr<ID3D11Buffer> current;
        UINT currentFirst=0,currentCount=0;
        if(context1)context1->VSGetConstantBuffers1(0,1,&current,&currentFirst,&currentCount);
        else context->VSGetConstantBuffers(0,1,&current);
        if(current.Get()!=original.Get() || (context1&&(first!=currentFirst||count!=currentCount)))return false;
        auto p=alternate.Get();
        if(context1)context1->VSSetConstantBuffers1(0,1,&p,&first,&count);
        else context->VSSetConstantBuffers(0,1,&p);
        bound=context;return true;
    }
    void restore() {
        if(!bound)return;auto p=original.Get();
        if(context1)context1->VSSetConstantBuffers1(0,1,&p,&first,&count);
        else bound->VSSetConstantBuffers(0,1,&p);
        bound=nullptr;
    }
};
