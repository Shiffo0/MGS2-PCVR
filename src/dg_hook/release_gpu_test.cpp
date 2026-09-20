#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <cstdio>
#define DG_DRAW_TRIAL_TEST
#include "dg_draw_trial.cpp"
#if DG_ENABLE_DIAGNOSTICS
#error This test requires the release profile
#endif
extern "C" { uint64_t testSiteBase=0; }
static unsigned checks;
#define CHECK(x) do { ++checks; if(!(x)) {printf("FAIL %d: %s\n",__LINE__,#x);return 1;} } while(0)
static void testLog(const char*,...) {}
int main() {
    ComPtr<ID3D11Device>d;ComPtr<ID3D11DeviceContext>c;
    CHECK(SUCCEEDED(D3D11CreateDevice(nullptr,D3D_DRIVER_TYPE_WARP,nullptr,0,nullptr,0,D3D11_SDK_VERSION,&d,nullptr,&c)));
    ComPtr<ID3D11Multithread>mt;CHECK(SUCCEEDED(c.As(&mt)));
    mt->SetMultithreadProtected(TRUE);
    auto vt=*(void***)d.Get();void*queries[]={vt[24],vt[25],vt[26]};
    auto ct=*(void***)c.Get();void*copies[]={ct[46],ct[47],ct[57]};
    BYTE*fake=(BYTE*)VirtualAlloc(nullptr,0x1400000,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE);
    CHECK(fake!=nullptr);testSiteBase=(uint64_t)fake;
    const unsigned offsets[]={0x53bf4,0x53b75};
    const BYTE bytes[2][10]={{0xff,0x50,0x60,0x48,0x8b,0x05,0x82,0x16,0x38,0x01},{0xff,0x50,0x68,0x48,0x8b,0x05,0x01,0x17,0x38,0x01}};
    for(unsigned i=0;i<2;i++)memcpy(fake+offsets[i],bytes[i],10);
    dg_draw_trial_attach(d.Get(),testLog);
    CHECK(installed && sitesOwned() && hookCount==0);
    CHECK(vt[24]==queries[0] && vt[25]==queries[1] && vt[26]==queries[2]);
    CHECK(ct[46]==copies[0] && ct[47]==copies[1] && ct[57]==copies[2]);
    CHECK(vsHook.slot && *vsHook.slot==vsHook.ours);
    CHECK(forwardStatus()==FORWARD_DIRECT && !forwardHook.slot);
    dg_draw_trial_poll((const char*)1);
    dg_ui2d_trace(99,(void*)1,(void*)2);
    CHECK(!requested && !pending && !ui2dTraceLeft && !ui2dCopySetCount);
    unsigned calls=0;requested=1;
    intercept(c.Get(),[&](){calls++;},nullptr,"test");
    CHECK(calls==1 && !pending);requested=0;
    dg_draw_trial_present();dg_draw_trial_stop();
    CHECK(!installed && vt[24]==queries[0] && ct[47]==copies[1]);
    printf("PASS release GPU: %u checks; no query/copy hooks, no trial draws, gameplay shader hook retained\n",checks);
    return 0;
}
