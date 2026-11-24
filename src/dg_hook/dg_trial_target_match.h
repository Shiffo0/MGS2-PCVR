// Selection equivalence only. Actual replay still prepares/copies the CURRENT targets.
#pragma once
bool trialTextureDescriptionEqual(const D3D11_TEXTURE2D_DESC&a,const D3D11_TEXTURE2D_DESC&b){
 return a.Width==b.Width&&a.Height==b.Height&&a.MipLevels==b.MipLevels&&a.ArraySize==b.ArraySize&&a.Format==b.Format&&a.SampleDesc.Count==b.SampleDesc.Count&&a.SampleDesc.Quality==b.SampleDesc.Quality&&a.Usage==b.Usage&&a.BindFlags==b.BindFlags&&a.CPUAccessFlags==b.CPUAccessFlags&&a.MiscFlags==b.MiscFlags;
}
template<class View> bool trialViewTextureEqual(View*a,View*b){
 if(!a||!b)return a==b;
 ComPtr<ID3D11Resource> ar,br;ComPtr<ID3D11Texture2D> at,bt;a->GetResource(&ar);b->GetResource(&br);
 if(FAILED(ar.As(&at))||FAILED(br.As(&bt)))return false;
 D3D11_TEXTURE2D_DESC ad{},bd{};at->GetDesc(&ad);bt->GetDesc(&bd);return trialTextureDescriptionEqual(ad,bd);
}
bool trialColorViewEqual(ID3D11RenderTargetView*a,ID3D11RenderTargetView*b){
 if(!a||!b)return false;
 D3D11_RENDER_TARGET_VIEW_DESC ad{},bd{};a->GetDesc(&ad);b->GetDesc(&bd);
 return ad.ViewDimension==D3D11_RTV_DIMENSION_TEXTURE2D&&bd.ViewDimension==ad.ViewDimension&&ad.Format==bd.Format&&ad.Texture2D.MipSlice==bd.Texture2D.MipSlice&&trialViewTextureEqual(a,b);
}
bool trialDepthViewEqual(ID3D11DepthStencilView*a,ID3D11DepthStencilView*b){
 if(!a||!b)return a==b;
 D3D11_DEPTH_STENCIL_VIEW_DESC ad{},bd{};a->GetDesc(&ad);b->GetDesc(&bd);
 return ad.ViewDimension==D3D11_DSV_DIMENSION_TEXTURE2D&&bd.ViewDimension==ad.ViewDimension&&ad.Format==bd.Format&&ad.Flags==bd.Flags&&ad.Texture2D.MipSlice==bd.Texture2D.MipSlice&&trialViewTextureEqual(a,b);
}
bool trialTargetsEqual(ID3D11DeviceContext*c,ID3D11RenderTargetView*surveyRT,ID3D11DepthStencilView*surveyDS){
 ComPtr<ID3D11RenderTargetView> rt;ComPtr<ID3D11DepthStencilView> ds;c->OMGetRenderTargets(1,&rt,&ds);
 return trialColorViewEqual(surveyRT,rt.Get())&&trialDepthViewEqual(surveyDS,ds.Get());
}
// The complete snapshot equality remains unchanged for restoration checks.
bool trialPipelineSelectionEqual(const TrialPipelineState&a,const TrialPipelineState&b){
 if(a.sections.size()!=b.sections.size()||a.sections.size()>64)return false;
 auto mask=a.differences(b);for(size_t i=0;i<a.sections.size();i++)if(!strcmp(a.sections[i].name,"OM.targets"))mask&=~(1ull<<i);
 return mask==0;
}
