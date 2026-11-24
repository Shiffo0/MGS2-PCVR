struct PostFinal {ComPtr<ID3D11Texture2D>colour;ComPtr<ID3D11RenderTargetView>rt;void*game=nullptr;unsigned long long written=~0ull;} fin;
unsigned long long finRecorded=0,finLearned=0,finPublished=0,finSkipped=0,hdfixFailed=0;
DgHdfixColour hdfix;

ComPtr<ID3D11Texture2D>finSnapT3,finSnapBb;
bool finalEligible(ID3D11DeviceContext*c,void*R,void*depth){
 if(!R||R!=ui2dBackbufferPtr||depth||(InterlockedCompareExchange(&seMode,0,0)&32))return false;
 if(fin.written==frames){++finSkipped;return false;}                 /* only the first: later ones are HUD/overlays */
 if(!postUsed||ppCur<0||pp[ppCur].lastUse!=frames||!pp[ppCur].b3)return false;
 ComPtr<ID3D11ShaderResourceView>v;c->PSGetShaderResources(0,1,&v);if(!v)return false;
 ComPtr<ID3D11Resource>r;v->GetResource(&r);return r.Get()==pp[ppCur].rt;   /* reads this frame's post output */
}
bool learnFinal(ID3D11Device*d,ID3D11RenderTargetView*rt,ID3D11Resource*r){
 if(fin.game==r&&fin.colour)return true;
 ComPtr<ID3D11Texture2D>t;if(FAILED(r->QueryInterface(IID_PPV_ARGS(&t)))){refusePost("final_not_texture");return false;}
 D3D11_TEXTURE2D_DESC x{};t->GetDesc(&x);
 if(x.Width!=colorDesc.Width||x.Height!=colorDesc.Height||x.SampleDesc.Count!=1||x.ArraySize!=1){refusePost("final_shape");return false;}
 x.MipLevels=1;x.Usage=D3D11_USAGE_DEFAULT;x.CPUAccessFlags=0;x.MiscFlags=0;x.BindFlags=D3D11_BIND_RENDER_TARGET|D3D11_BIND_SHADER_RESOURCE;
 D3D11_RENDER_TARGET_VIEW_DESC vd{};rt->GetDesc(&vd);
 PostFinal next;next.game=r;
 if(FAILED(d->CreateTexture2D(&x,nullptr,&next.colour))||FAILED(d->CreateRenderTargetView(next.colour.Get(),&vd,&next.rt))){refusePost("final_allocation");return false;}
 flushBatch(context.Get(),FL_TARGETS);fin=next;++finLearned;
 if(logger)logger("second_eye: final colour pass learned: swap-chain buffer %ux%u fmt %u view fmt %u; the replay eye gets its own final image\r\n",x.Width,x.Height,(unsigned)x.Format,(unsigned)vd.Format);
 return true;
}
