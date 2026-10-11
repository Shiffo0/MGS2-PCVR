/* Inside SecondEyeLive. Measured 2 Oct: 465E downsample, B356 ping-pong,
   A460 back to full size. Native half colour leaking through A460 is visible
   across live event15. Own only this scene-derived, depthless BGRA8 chain. */
struct PostHalf {ComPtr<ID3D11Resource>game;ComPtr<ID3D11Texture2D>colour;ComPtr<ID3D11RenderTargetView>rt;unsigned long long written=~0ull;} half[2];
bool postHalfFailed=false;unsigned long long halfDraws=0,halfLearned=0,halfRefused=0;
// postSwap returns borrowed views; recordEx AddRefs them immediately on return.
// Keep an uncached view alive until that handoff, even when all 16 slots are full.
ComPtr<ID3D11ShaderResourceView> swapOverflow;
void clearPostSwaps(){swapN=0;for(auto&s:swaps)s.mine.Reset();swapOverflow.Reset();}
int halfFind(void*r){if(!r)return -1;for(int k=0;k<2;k++)if(half[k].game.Get()==r)return k;return -1;}
bool halfParent(ID3D11DeviceContext*c){
 ComPtr<ID3D11ShaderResourceView>v;c->PSGetShaderResources(0,1,&v);if(!v)return false;
 D3D11_SHADER_RESOURCE_VIEW_DESC sd{};v->GetDesc(&sd);if(sd.ViewDimension!=D3D11_SRV_DIMENSION_TEXTURE2D||sd.Texture2D.MostDetailedMip!=0||sd.Texture2D.MipLevels!=1)return false;
 ComPtr<ID3D11Resource>r;v->GetResource(&r);
 int k=ppFind(r.Get());if(k>=0)return postUsed&&pp[k].lastUse==frames;
 k=halfFind(r.Get());return k>=0&&half[k].written==frames;
}
bool halfEligible(ID3D11DeviceContext*c,ID3D11Resource*r,void*depth,bool indexed,UINT n){
 if(depth||indexed||n!=4||!r||!halfParent(c))return false;
 ComPtr<ID3D11Texture2D>t;if(FAILED(r->QueryInterface(IID_PPV_ARGS(&t))))return false;D3D11_TEXTURE2D_DESC x{};t->GetDesc(&x);
 if(colorDesc.Width%2||colorDesc.Height%2||x.Width!=colorDesc.Width/2||x.Height!=colorDesc.Height/2||x.MipLevels!=1||x.ArraySize!=1||x.SampleDesc.Count!=1||x.Format!=DXGI_FORMAT_B8G8R8A8_UNORM||!(x.BindFlags&D3D11_BIND_RENDER_TARGET))return false;
 ComPtr<ID3D11VertexShader>vs;c->VSGetShader(&vs,nullptr,nullptr);UINT64 h=vsHashOf(vs.Get(),nullptr,nullptr);
 return h==UINT64_C(0x465E5F2FE0EFD968)||h==UINT64_C(0xB356EBFAD8F1B18E);
}
int learnHalf(ID3D11Device*d,ID3D11RenderTargetView*rt,ID3D11Resource*r){
 int k=halfFind(r);if(k>=0)return k;
 k=!half[0].game?0:!half[1].game?1:half[0].written!=frames?0:half[1].written!=frames?1:-1;
 if(k<0){postHalfFailed=true;++halfRefused;refusePost("half_target_limit");return -1;}
 D3D11_RENDER_TARGET_VIEW_DESC vd{};rt->GetDesc(&vd);if(vd.ViewDimension!=D3D11_RTV_DIMENSION_TEXTURE2D||vd.Texture2D.MipSlice){postHalfFailed=true;++halfRefused;refusePost("half_view");return -1;}
 ComPtr<ID3D11Texture2D>t;if(FAILED(r->QueryInterface(IID_PPV_ARGS(&t))))return -1;D3D11_TEXTURE2D_DESC x{};t->GetDesc(&x);x.Usage=D3D11_USAGE_DEFAULT;x.CPUAccessFlags=0;x.MiscFlags=0;x.BindFlags=D3D11_BIND_RENDER_TARGET|D3D11_BIND_SHADER_RESOURCE;
 PostHalf next;next.game=r;if(FAILED(d->CreateTexture2D(&x,nullptr,&next.colour))||FAILED(d->CreateRenderTargetView(next.colour.Get(),&vd,&next.rt))){postHalfFailed=true;++halfRefused;refusePost("half_allocation");return -1;}
 // Earlier records retain their resources; flush in order before changing identity/cache.
 flushBatch(context.Get(),FL_TARGETS);half[k]=next;clearPostSwaps();++halfLearned;
 if(logger)logger("second_eye: half post target learned %d %ux%u; own scene colour\r\n",k,x.Width,x.Height);
 return k;
}
